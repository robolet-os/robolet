// Lost-robot mission on the chip: the navsim.py "kidnap" experiment in C++ (firmware command [k]).
//   worlds : 0 room (bench_core room), 1 s_corridor (three 2 m lanes in an S), 2 s_corridor_lm (+ one shelf),
//            3 doors (a 0.7 m door in a middle wall and a 0.55 m gap; robot diameter 0.36 m),
//            4 room_tilted (world 0 turned by 12 deg about (4, 3): no wall is aligned with the map grid; the lidar sees
//              the exact tilted walls (x4sim segments), collisions use the walls drawn into the grid)
//   mission: SLAM mapping tour, then goals on the SLAM map with AMCL + A* + pure pursuit; during one random leg the
//            robot is carried to a random free pose (kidnap).
//   modes  : 0 = AMCL only (Nav2 default: no lost-pose detection, no random particles)
//            1 = spin (navsim.py final method): beam check detects the lost pose (3 bad scans) -> global search over all
//                free poses (0.15 m grid x 36 headings) -> one candidate: use it; several: turn in place only (15 s)
//                and drop candidates by "decisive beams" (6 beams, 3 times in a row); still ambiguous: stop, ask for help
//   options: OPT_KIDNAP (1) one kidnap, OPT_BOX (2) one box that is not in the map is placed after mapping (navsim.py BOXES)
//   obstacles (all modes, like the Nav2 obstacle layer + collision monitor + behavior tree, ported from navsim.py):
//     live layer   : scan points within 1.5 m that are > 0.12 m from the SLAM map are marked while the pose is healthy;
//                    if the marked cells (grown by 4 cells) touch the next 40 path points, the robot replans
//     planning     : A* on the SLAM costmap with planning padding (cells < ROBOT_R + 0.06 m from an occupied cell centre
//                    are lethal, like Nav2 footprint_padding) and cost weight 6 (bench_core default 3) so paths keep to
//                    the middle of doors and gaps
//     controller   : pure pursuit, lookahead 0.4 m, 0.25 m when the map clearance is < 0.35 m (regulated pure pursuit);
//                    the arc up to the carrot is checked against the raw scan (Nav2 RPP collision check); blocked: try
//                    lookahead 0.18 / 0.12 / 0.08 m, then sample slow arcs like Nav2 DWB, then turn in place
//     collision stop: the commanded arc reaches a raw scan point within ROBOT_R + 0.01 m in the next 1.0 s -> v = 0
//                    (Nav2 collision monitor, approach model); slowdown zone 0.60 m wide, 0.5 m ahead -> half speed
//     recoveries   : no path, or less than 0.5 m progress in 10 s -> clear / spin / backup / wait in turn; 4 failures -> abort
// Differences from navsim.py: SLAM without loop closure, lidar at the robot centre, chip room differs from navsim's room. Pure C++: runs on a PC (test_host/test_mission.cpp) and on the chip.
#pragma once
#include "nav_core.h"

namespace mis {
using namespace nav;
using bc::Pose;

static const int CAND_CAP = 120000, MAXHYP = 24;
enum { WORLD_ROOM = 0, WORLD_SCORR = 1, WORLD_SCORR_LM = 2, WORLD_DOORS = 3, WORLD_ROOM_TILT = 4, N_WORLDS = 5 };
enum { MODE_AMCL_ONLY = 0, MODE_SPIN = 1 };
inline const char* world_name(int w) { static const char* n[] = {"room", "s_corridor", "s_corridor_lm", "doors", "room_tilted"}; return n[w]; }
inline const char* mode_name(int m) { return m == 0 ? "amcl_only" : "spin"; }

struct Cand { float sc; int32_t k; };
// qx/qy: MAXPTS, cand: CAND_CAP, ex: MAXHYP * MAXPTS / 2, meas: MAXPTS / 2, errs: ERRS_CAP (all can live in PSRAM)
static const int ERRS_CAP = 8192;
// live, live_dist, occ_plan, dist_plan, cost_plan: CELLS bytes each (live obstacle layer and the costmap built with it)
struct MWork { Work* w; float *qx, *qy; Cand* cand; float *ex, *meas, *errs; uint8_t *live, *live_dist, *occ_plan, *dist_plan, *cost_plan; int world; };
enum { OPT_KIDNAP = 1, OPT_BOX = 2 };
enum { OUT_CORRECT = 0, OUT_STOPPED_SAFELY = 1, OUT_INCOMPLETE = 2, OUT_WRONG_POSE = 3, OUT_CONTACT = 4, OUT_N = 5 };
inline const char* out_name(int o) { static const char* n[] = {"CORRECT", "STOPPED_SAFELY", "INCOMPLETE", "WRONG_POSE_DRIVING", "CONTACT"}; return n[o]; }

struct Event { int result; int n_hyp, n_alive; bool truth_kept; float err; float hyp_err[MAXHYP]; float took_s; double cpu_ms; };
enum { EV_UNIQUE = 0, EV_RESOLVED = 1, EV_AMBIGUOUS_STOPPED = 2, EV_NOT_FOUND = 3 };
inline const char* ev_name(int r) { static const char* n[] = {"unique", "resolved_by_turning", "ambiguous_stopped", "not_found"}; return n[r]; }

struct MResult {
  int world = 0, mode = 0, opts = 0; uint32_t seed = 0;
  float slam_err_max = 0, map_wall_err = 0, map_within5 = 0;
  int obstacle_replans = 0, recoveries = 0, collision_stops = 0, aborts = 0, arc_blocked = 0;
  int legs = 0, legs_done = 0, legs_reachable = 0; bool unreach_ok = true;
  bool kidnapped = false; float detect_s = -1; int false_lost = 0;
  Event ev[4]; int n_ev = 0; bool need_help = false;
  float wrong_drive_before_detect_s = 0, wrong_drive_after_s = 0;   // driving (v > 0) while the pose error is > 0.5 m
  int contacts = 0; float loc_err_median = 0;
  double max_cycle_ms = 0, search_ms_max = 0, multi_update_ms_max = 0;
  double search_slice_ms_max = 0; int search_cycles_max = 0;    // node version: the search in slices (longest slice, cycles)
  char outcome[24] = ""; int outcome_code = 0;
};

// ---------------------------------------------------------------- worlds
// room_tilted: points of the room world turned by TILT about (TCX, TCY)
static const float TILT = 0.20943951f, TCX = 4.0f, TCY = 3.0f;          // 12 deg
inline void tilt(float& x, float& y) { float c = cosf(TILT), s = sinf(TILT), dx = x - TCX, dy = y - TCY; x = TCX + c * dx - s * dy; y = TCY + s * dx + c * dy; }
inline void untilt(float& x, float& y) { float c = cosf(TILT), s = sinf(TILT), dx = x - TCX, dy = y - TCY; x = TCX + c * dx + s * dy; y = TCY - s * dx + c * dy; }
// one solid rectangle (room coordinates) turned by TILT: 4 lidar segments + the cells it covers (sampled every 1 cm)
inline void tilt_rect(uint8_t* occ, float x0, float y0, float x1, float y1) {
  float X[4] = {x0, x1, x1, x0}, Y[4] = {y0, y0, y1, y1};
  for (int k = 0; k < 4; k++) tilt(X[k], Y[k]);
  for (int k = 0; k < 4; k++) x4sim::geo_add(X[k], Y[k], X[(k + 1) % 4], Y[(k + 1) % 4]);
  for (float u = x0; u <= x1 + 1e-4f; u += 0.01f) for (float v = y0; v <= y1 + 1e-4f; v += 0.01f) {
    float x = u, y = v; tilt(x, y); int i = bc::cx(x), j = bc::cy(y);
    if (i >= 0 && i < W && j >= 0 && j < H) occ[j * W + i] = 1;
  }
}
// inside the outer walls (where the robot can be)
inline bool in_world(int world, float x, float y) {
  if (world == WORLD_ROOM_TILT) untilt(x, y);
  return x > 0 && x < 8 && y > 0 && y < 6;
}
inline void build_world(uint8_t* occ, int world) {
  x4sim::geo().n = 0;                                            // grid walls unless the world sets segments
  if (world == WORLD_ROOM) { bc::build_room(occ); return; }
  memset(occ, 0, CELLS);
  if (world == WORLD_ROOM_TILT) {                                // bc::build_room walls (1 cell = 5 cm thick), turned
    tilt_rect(occ, 0, 0, 8.05f, 0.05f); tilt_rect(occ, 0, 6, 8.05f, 6.05f); tilt_rect(occ, 0, 0, 0.05f, 6.05f); tilt_rect(occ, 8, 0, 8.05f, 6.05f);
    tilt_rect(occ, 3, 0, 3.05f, 3.55f); tilt_rect(occ, 5.2f, 3.8f, 5.85f, 4.45f); tilt_rect(occ, 1.0f, 4.5f, 1.55f, 5.05f);
    return;
  }
  if (world == WORLD_DOORS) {
    bc::fill_rect(occ, 0, 0, 8, 0); bc::fill_rect(occ, 0, 6, 8, 6); bc::fill_rect(occ, 0, 0, 0, 6); bc::fill_rect(occ, 8, 0, 8, 6);
    // fill_rect includes the cell that holds the end point, so the ends are set 1 cm inside the opening:
    bc::fill_rect(occ, 4, 0, 4, 2.64f); bc::fill_rect(occ, 4, 3.36f, 4, 6);          // middle wall, 0.70 m door, free cells y 2.65-3.35
    bc::fill_rect(occ, 4, 4, 7.44f, 4);                                             // wall y = 4, 0.55 m gap, free cells x 7.45-8.00
    bc::fill_rect(occ, 1.8f, 1.8f, 2.3f, 2.3f);                                     // one fixed box on the left
    return;
  }
  bc::fill_rect(occ, 0, 0, 8, 0); bc::fill_rect(occ, 0, 6, 8, 6); bc::fill_rect(occ, 0, 0, 0, 6); bc::fill_rect(occ, 8, 0, 8, 6);
  bc::fill_rect(occ, 0, 2, 6, 2); bc::fill_rect(occ, 2, 4, 8, 4);                       // lanes y 0-2, 2-4, 4-6
  if (world == WORLD_SCORR_LM) { bc::fill_rect(occ, 4.5f, 5.5f, 5.5f, 5.5f); bc::fill_rect(occ, 4.5f, 5.5f, 4.5f, 6.0f); bc::fill_rect(occ, 5.5f, 5.5f, 5.5f, 6.0f); }
}
struct Leg { float x, y; bool reachable; const char* name; };
// one box (x0, y0, x1, y1) that is not in the map, chosen by seed % 3 like navsim.py BOXES
inline void world_box(int world, uint32_t seed, float* b) {
  static const float ROOM[3][4] = {{3.2f, 3.6f, 3.6f, 4.4f}, {5.0f, 1.2f, 5.4f, 1.8f}, {4.5f, 4.6f, 5.0f, 5.1f}};
  static const float CORR[3][4] = {{3.6f, 2.6f, 4.2f, 3.2f}, {4.0f, 0.2f, 4.6f, 0.8f}, {3.0f, 4.9f, 3.5f, 5.5f}};   // navsim.py BOXES
  // doors: the third box was 0.28 m from the THROUGH_GAP goal (goal inside the padded obstacle), moved to block the diagonal instead
  static const float DOORS[3][4] = {{5.0f, 2.0f, 5.4f, 2.4f}, {2.6f, 4.6f, 3.0f, 5.0f}, {6.7f, 4.4f, 7.1f, 4.8f}};
  const float* s = (world == WORLD_ROOM || world == WORLD_ROOM_TILT) ? ROOM[seed % 3] : (world == WORLD_DOORS ? DOORS[seed % 3] : CORR[seed % 3]);   // room_tilted: room coordinates
  for (int i = 0; i < 4; i++) b[i] = s[i];
}
inline int world_tour(int world, float (*wp)[2]) {
  static const float ROOM[][2] = {{1.5f, 1.5f}, {1.5f, 4.0f}, {2.5f, 4.0f}, {4.0f, 4.0f}, {4.5f, 2.0f}, {6.5f, 1.5f},
                                  {7.0f, 3.0f}, {6.5f, 5.2f}, {4.0f, 5.2f}, {2.0f, 4.0f}, {1.5f, 1.5f}};
  static const float CORR[][2] = {{1, 1}, {7, 1}, {7, 3}, {1, 3}, {1, 5}, {7, 5}, {1, 5}, {1, 3}, {7, 3}, {7, 1}, {1, 1}};   // navsim.py WAY
  static const float DOORS[][2] = {{1, 1}, {1, 5}, {3, 4}, {3, 3}, {5, 3}, {6, 1.5f}, {7.2f, 2.5f}, {7.72f, 3.6f}, {7.72f, 4.6f},
                                   {6, 5.2f}, {4.6f, 5.0f}, {6, 5.2f}, {7.72f, 4.6f}, {7.72f, 3.6f}, {6, 3}, {3, 3}, {3, 1}, {1, 1}};   // last leg goes under the box
  const float (*src)[2] = (world == WORLD_ROOM || world == WORLD_ROOM_TILT) ? ROOM : (world == WORLD_DOORS ? DOORS : CORR);
  int n = world == WORLD_DOORS ? 18 : 11;
  for (int i = 0; i < n; i++) { wp[i][0] = src[i][0]; wp[i][1] = src[i][1]; if (world == WORLD_ROOM_TILT) tilt(wp[i][0], wp[i][1]); }
  return n;
}
inline int world_legs(int world, Leg* L) {
  if (world == WORLD_DOORS) {
    static const Leg D[] = {{6.0f, 1.5f, true, "THROUGH_DOOR"}, {4.0f, 1.0f, false, "UNREACHABLE"}, {6.0f, 5.2f, true, "THROUGH_GAP"},
                            {2.0f, 5.0f, true, "BACK_LEFT"}, {1.0f, 1.0f, true, "DOCK"}};
    for (int i = 0; i < 5; i++) L[i] = D[i]; return 5;
  }
  if (world == WORLD_ROOM || world == WORLD_ROOM_TILT) {
    static const Leg R[] = {{6.5f, 1.5f, true, "A"}, {3.0f, 1.0f, false, "UNREACHABLE"}, {6.8f, 5.2f, true, "B"}, {2.3f, 5.3f, true, "C"}, {1.5f, 1.5f, true, "DOCK"}};
    for (int i = 0; i < 5; i++) { L[i] = R[i]; if (world == WORLD_ROOM_TILT) tilt(L[i].x, L[i].y); } return 5;
  }
  static const Leg C[] = {{7.2f, 5.0f, true, "END"}, {4.0f, 2.0f, false, "UNREACHABLE"}, {0.8f, 1.0f, true, "START"},
                          {7.2f, 5.0f, true, "END"}, {0.8f, 1.0f, true, "START"}, {7.2f, 5.0f, true, "END"}};   // navsim.py mission
  for (int i = 0; i < 6; i++) L[i] = C[i]; return 6;
}

// ---------------------------------------------------------------- beam model on the SLAM occupancy grid
// expected range along world angle a: first occupied cell (half-cell steps from 0.1 m); leaving the grid counts as a hit
inline float ray_occ(const uint8_t* occ, float x, float y, float a) {
  float c = cosf(a), s = sinf(a);
  float fx = (x - OX) * INV_RES + 0.1f * INV_RES * c, fy = (y - OY) * INV_RES + 0.1f * INV_RES * s;
  for (int k = 0; k < 396; k++, fx += 0.5f * c, fy += 0.5f * s) {
    float r = 0.1f + k * 0.025f;
    if (fx < 0 || fy < 0) return r;
    int i = (int)fx, j = (int)fy;
    if (i >= W || j >= H) return r;
    if (occ[j * W + i]) return r;
  }
  return 10.0f;
}
inline float tol_far(float e) { return e * 0.12f > 0.3f ? e * 0.12f : 0.3f; }
// lf: fraction of beams that "see through" a mapped wall (pose is wrong); mf: fraction that agree with the map
inline void beam_check(const uint8_t* occ, Pose p, const float* px, const float* py, int n, int sub, float& lf, float& mf) {
  int cnt = 0, far = 0, ok = 0;
  for (int k = 0; k < n; k += sub) {
    float meas = hypotf(px[k], py[k]), e = ray_occ(occ, p.x, p.y, p.th + atan2f(py[k], px[k]));
    if (meas > e + tol_far(e)) far++;
    float tn = e * 0.06f > 0.15f ? e * 0.06f : 0.15f;
    if (fabsf(meas - e) < tn) ok++;
    cnt++;
  }
  lf = cnt ? (float)far / cnt : 1; mf = cnt ? (float)ok / cnt : 0;
}
inline int fit_filter(const uint8_t* dist, Pose p, const float* px, const float* py, int n, float* qx, float* qy) {
  float c = cosf(p.th), s = sinf(p.th); int m = 0;
  for (int k = 0; k < n; k++) {
    int i = bc::cx(p.x + c * px[k] - s * py[k]), j = bc::cy(p.y + s * px[k] + c * py[k]);
    if (i < 0 || i >= W || j < 0 || j >= H) continue;
    if (dist[j * W + i] / 10.0f * RES < 0.15f) { qx[m] = px[k]; qy[m] = py[k]; m++; }
  }
  return m;
}
inline Pose refine(MWork& mw, Pose q, const float* px, const float* py, int n) {
  Work& w = *mw.w;
  int m = fit_filter(w.dist, q, px, py, n, mw.qx, mw.qy);
  if (m <= 60) return q;
  bc::MatchResult r = bc::csm_fast(w.lik, mw.qx, mw.qy, m, q, w.idx);
  return match_fine(w.lik, mw.qx, mw.qy, m, r.best);
}

// ---------------------------------------------------------------- global search (navsim.py global_search)
inline int cand_cmp(const void* a, const void* b) { float d = ((const Cand*)b)->sc - ((const Cand*)a)->sc; return d > 0 ? 1 : (d < 0 ? -1 : 0); }
// The search can run in slices (one per cycle) so that no cycle takes long: rows of pass 1 first, then sorting, then the
// representatives. The work of each slice is fixed (not timed), so a run gives the same results on the PC and the chip,
// and the candidates are the same as in one go (global_search()).
struct Search {
  const float* px; const float* py; int n;                       // the scan (kept by the caller until done)
  float sx[128], sy[128], hc[36], hs[36]; int m;
  int phase, j, nc, nk, nr, r, ng; float best; Pose reps[MAXHYP];
  Pose out[MAXHYP]; float key[MAXHYP];
};
inline void search_begin(Search& S, const float* px, const float* py, int n) {
  S.px = px; S.py = py; S.n = n;
  int sub = n / 60; if (sub < 1) sub = 1;
  S.m = 0;
  for (int k = 0; k < n && S.m < 128; k += sub) { S.sx[S.m] = px[k] * INV_RES; S.sy[S.m] = py[k] * INV_RES; S.m++; }
  for (int t = 0; t < 36; t++) { float a = -3.14159265f + t * 6.2831853f / 36; S.hc[t] = cosf(a); S.hs[t] = sinf(a); }
  S.phase = 0; S.j = 0; S.nc = 0; S.nk = 0; S.nr = 0; S.r = 0; S.ng = 0; S.best = 0;
}
// pass 1 rows (every 3rd row of the map) in the search
inline int search_rows() { return (H + 2) / 3; }
// does up to `rows` rows of pass 1, or the sorting, or up to `reps` representatives; true when the search is done
inline bool search_step(MWork& mw, Search& S, int rows, int reps) {
  Work& w = *mw.w; const int m = S.m;
  if (S.phase == 0) {                                            // pass 1: score every candidate pose
    for (int r0 = 0; r0 < rows && S.j < H; r0++, S.j += 3) {
      const int j = S.j;
      for (int i = 0; i < W; i += 3) {
        float x = OX + (i + 0.5f) * RES, y = OY + (j + 0.5f) * RES;
        if (!in_world(mw.world, x, y)) continue;
        if (w.dist[j * W + i] / 10.0f * RES <= 0.25f) continue;
        float fx0 = (x - OX) * INV_RES, fy0 = (y - OY) * INV_RES;
        for (int t = 0; t < 36; t++) {
          uint32_t s = 0;
          for (int k = 0; k < m; k++) {
            float fx = fx0 + S.hc[t] * S.sx[k] - S.hs[t] * S.sy[k], fy = fy0 + S.hs[t] * S.sx[k] + S.hc[t] * S.sy[k];
            int a = (int)fx, b = (int)fy;
            if (fx >= 0 && fy >= 0 && a < W && b < H) s += w.lik[b * W + a];
          }
          float sc = s / (255.0f * m);
          if (S.nc < CAND_CAP) { mw.cand[S.nc].sc = sc; mw.cand[S.nc].k = (j * W + i) * 36 + t; S.nc++; }
          if (sc > S.best) S.best = sc;
        }
      }
    }
    if (S.j >= H) S.phase = 1;
    return false;
  }
  if (S.phase == 1) {                                            // pass 2: within 50 % of the best, best first, 0.6 m apart, at most 24
    int nk = 0; for (int c = 0; c < S.nc; c++) if (mw.cand[c].sc >= 0.5f * S.best) mw.cand[nk++] = mw.cand[c];
    qsort(mw.cand, nk, sizeof(Cand), cand_cmp);
    S.nr = 0;
    for (int c = 0; c < nk && c < 5000 && S.nr < MAXHYP; c++) {
      int cell = mw.cand[c].k / 36, t = mw.cand[c].k % 36;
      Pose q = {OX + (cell % W + 0.5f) * RES, OY + (cell / W + 0.5f) * RES, -3.14159265f + t * 6.2831853f / 36};
      bool far = true; for (int r = 0; r < S.nr; r++) if (hypotf(q.x - S.reps[r].x, q.y - S.reps[r].y) <= 0.6f) { far = false; break; }
      if (far) S.reps[S.nr++] = q;
    }
    S.phase = 2; S.r = 0;
    return false;
  }
  // refine each representative and keep the ones the beam model accepts
  for (int c = 0; c < reps && S.r < S.nr; c++, S.r++) {
    // wide search: +-0.2 m in 0.05 m steps, +-8 deg in 2 deg steps, on the subsampled scan
    float wx[128], wy[128]; for (int k = 0; k < m; k++) { wx[k] = S.sx[k] * RES; wy[k] = S.sy[k] * RES; }
    bc::MatchResult mr = bc::csm_fast(w.lik, wx, wy, m, S.reps[S.r], w.idx, 9, 9, 0.05f, 0.0349f);
    Pose q2 = refine(mw, mr.best, S.px, S.py, S.n);
    float lf, mf; beam_check(w.occ, q2, S.px, S.py, S.n, 2, lf, mf);
    if (lf < 0.15f && mf > 0.4f) {
      bool far = true; for (int g = 0; g < S.ng; g++) if (hypotf(q2.x - S.out[g].x, q2.y - S.out[g].y) <= 0.5f) { far = false; break; }
      if (far && S.ng < MAXHYP) { S.out[S.ng] = q2; S.key[S.ng] = lf - mf; S.ng++; }
    }
  }
  if (S.r < S.nr) return false;
  for (int a = 1; a < S.ng; a++) { Pose p = S.out[a]; float k = S.key[a]; int b = a - 1;          // sort by lf - mf
    while (b >= 0 && S.key[b] > k) { S.out[b + 1] = S.out[b]; S.key[b + 1] = S.key[b]; b--; } S.out[b + 1] = p; S.key[b + 1] = k; }
  return true;
}
inline int global_search(MWork& mw, const float* px, const float* py, int n, Pose* out, float* out_key) {
  static Search S;                                               // (about 3 KB: not on the stack)
  search_begin(S, px, py, n);
  while (!search_step(mw, S, 1 << 20, 1 << 20)) {}
  for (int g = 0; g < S.ng; g++) { out[g] = S.out[g]; out_key[g] = S.key[g]; }
  return S.ng;
}

// ---------------------------------------------------------------- the mission
// Planning padding (Nav2 footprint_padding): cells closer than ROBOT_R + 0.06 m to an occupied cell centre are lethal for
// planning. The distance field is measured between cell centres, so a wall face can be 2.5 cm and a wall corner 3.5 cm closer than it says.
inline void pad_cost(const uint8_t* dist, uint8_t* cost) {
  const int lim = (int)((ROBOT_R + 0.06f) / RES * 10.0f + 0.5f);
  for (int k = 0; k < CELLS; k++) if (dist[k] < lim) cost[k] = 254;
}

// Does the arc driven with (v, w) hit a scan point before it has gone smax metres (up to the carrot, like Nav2 RPP
// max_allowed_time_to_collision_up_to_carrot)? Robot frame, circle of radius rad.
inline bool arc_hits(const float* px, const float* py, int m, float v, float w, float smax, float rad = ROBOT_R + 0.02f) {
  const float rr = rad * rad;
  float x = 0, y = 0, th = 0, s = 0;
  for (int st = 0; st < 40 && s < smax; st++) {
    float dt = 0.05f; th += w * dt * 0.5f; x += v * dt * cosf(th); y += v * dt * sinf(th); th += w * dt * 0.5f; s += fabsf(v) * dt;
    for (int k = 0; k < m; k++) {
      float dx = px[k] - x, dy = py[k] - y, d = dx * dx + dy * dy;
      if (d < rr) {
        float d0 = px[k] * px[k] + py[k] * py[k]; if (d0 < rr && d >= d0) continue;   // already that close: only moving closer counts
        return true;
      }
    }
  }
  return false;
}

inline MResult run_mission(MWork& mw, const Hooks& h, int world, int mode, uint32_t seed, int opts = OPT_KIDNAP) {
  Work& w = *mw.w; mw.world = world; MResult R; R.world = world; R.mode = mode; R.seed = seed; R.opts = opts;
  static ScanBuf scan; scan.n = 0;
  X4Parser parser(collect, &scan);
  build_world(w.occ_truth, world); bc::distance_field(w.occ_truth, w.dist_truth);
  memset(w.lo, 0, CELLS);
  float WP[24][2]; int NW = world_tour(world, WP);
  Robot rb; rb.truth = {WP[0][0], WP[0][1], 0.0f}; rb.odom = rb.truth; rb.rng = {777u + seed};
  x4sim::Rng lrng{4242u + seed}, mrng{555u + seed};
  auto urand = [&]() { return (mrng.next() & 0xFFFFFF) / 16777216.0f; };
  auto sense = [&](void) -> int {
    int nb = scan_rev(w, rb, lrng); parser.feed(w.rev, nb);
    int m2 = scan_points(rb, scan, w.px, w.py); rb.hold();    // a second scan without a step is taken standing still
    return m2;
  };
  // ---------------- mapping (same SLAM as nav_core.h)
  int np = 0;
  for (int i = 0; i + 1 < NW && np < PATH_CAP; i++) {
    float L = hypotf(WP[i + 1][0] - WP[i][0], WP[i + 1][1] - WP[i][1]); int k = (int)(L / 0.05f);
    for (int j = 0; j < k && np < PATH_CAP; j++) { w.path_x[np] = WP[i][0] + (WP[i + 1][0] - WP[i][0]) * j / k; w.path_y[np] = WP[i][1] + (WP[i + 1][1] - WP[i][1]) * j / k; np++; }
  }
  w.path_x[np] = WP[NW - 1][0]; w.path_y[np] = WP[NW - 1][1]; np++;
  Pose est = rb.truth, last_kf = est, last_odom = rb.odom;
  int m = sense(); slam_insert(w.lo, est, w.px, w.py, m); slam_rebuild(w);
  int pidx = 0;
  for (int cyc = 0; cyc < 7 * 400; cyc++) {
    float v, wv;
    pursue(est, w.path_x, w.path_y, np, pidx, 0.3f, clearance_at(w.dist, est.x, est.y), v, wv);
    if (pidx >= np - 2 && hypotf(w.path_x[np - 1] - est.x, w.path_y[np - 1] - est.y) < 0.15f) break;
    rb.step(w.dist_truth, v, wv);
    m = sense();
    Pose pred = oplus(est, ominus(last_odom, rb.odom)); last_odom = rb.odom;
    bc::MatchResult cr = bc::csm_fast(w.lik, w.px, w.py, m, pred, w.idx);
    est = match_fine(w.lik, w.px, w.py, m, cr.best);
    Pose dk = ominus(last_kf, est);
    if (hypotf(dk.x, dk.y) > 0.10f || fabsf(dk.th) > 0.10f) { slam_insert(w.lo, est, w.px, w.py, m); slam_rebuild(w); last_kf = est; }
    float e = hypotf(est.x - rb.truth.x, est.y - rb.truth.y); if (e > R.slam_err_max) R.slam_err_max = e;
  }
  { // map quality: distance from each occupied SLAM cell centre to the nearest true wall surface. room and room_tilted:
    // exact wall rectangles (so the two can be compared); other worlds: distance to the nearest true wall cell centre
    const bool rooms = world == WORLD_ROOM || world == WORLD_ROOM_TILT;
    static const float RR[7][4] = {{0, 0, 8.05f, 0.05f}, {0, 6, 8.05f, 6.05f}, {0, 0, 0.05f, 6.05f}, {8, 0, 8.05f, 6.05f},
                                   {3, 0, 3.05f, 3.55f}, {5.2f, 3.8f, 5.85f, 4.45f}, {1.0f, 4.5f, 1.55f, 5.05f}};
    double sum = 0; int n = 0, near5 = 0;
    for (int k = 0; k < CELLS; k++) if (w.occ[k]) {
      float d;
      if (rooms) {
        float x = OX + (k % W + 0.5f) * RES, y = OY + (k / W + 0.5f) * RES;
        if (world == WORLD_ROOM_TILT) untilt(x, y);
        d = 9;
        for (int r = 0; r < 7; r++) {                              // distance to a solid rectangle (0 inside)
          float dx = fmaxf(fmaxf(RR[r][0] - x, 0.0f), x - RR[r][2]), dy = fmaxf(fmaxf(RR[r][1] - y, 0.0f), y - RR[r][3]);
          d = fminf(d, hypotf(dx, dy));
        }
      } else d = w.dist_truth[k] / 10.0f * RES;
      sum += d; n++; if (d <= 0.05f) near5++;
    }
    R.map_wall_err = n ? (float)(sum / n) : 0; R.map_within5 = n ? 100.0f * near5 / n : 0;
  }
  logf_(h, "[k] %s/%s seed %u: map done, SLAM error max %.3f m, map walls %.3f m from true walls on average (%.1f %% within 5 cm)",
        world_name(world), mode_name(mode), seed, R.slam_err_max, R.map_wall_err, R.map_within5);
  pad_cost(w.dist, w.cost);                                      // navigation plans on the padded costmap
  if (opts & OPT_BOX) {                                          // the world changes after mapping: one unmapped box
    float b[4]; world_box(world, seed, b);
    if (world == WORLD_ROOM_TILT) tilt_rect(w.occ_truth, b[0], b[1], b[2], b[3]);   // turned with the room
    else {
      bc::fill_rect(w.occ_truth, b[0], b[1], b[2], b[1]); bc::fill_rect(w.occ_truth, b[0], b[3], b[2], b[3]);
      bc::fill_rect(w.occ_truth, b[0], b[1], b[0], b[3]); bc::fill_rect(w.occ_truth, b[2], b[1], b[2], b[3]);
    }
    bc::distance_field(w.occ_truth, w.dist_truth);
    logf_(h, "[k]   unmapped box placed at (%.1f-%.1f, %.1f-%.1f)%s", b[0], b[2], b[1], b[3], world == WORLD_ROOM_TILT ? " (before turning)" : "");
  }

  // ---------------- mission with one kidnap
  Leg legs[8]; int NL = world_legs(world, legs); R.legs = NL;
  int reach_idx[8], nreach = 0; for (int i = 1; i < NL; i++) if (legs[i].reachable) reach_idx[nreach++] = i;
  int kid_leg = reach_idx[mrng.next() % nreach]; float kid_after = 1.0f + 5.0f * urand();
  if (!(opts & OPT_KIDNAP)) kid_leg = -1;
  Pose kid_pose;
  for (;;) {                                                     // random free pose, true clearance > 0.45 m
    float x = 0.2f + 7.6f * urand(), y = 0.2f + 5.6f * urand();
    if (world == WORLD_ROOM_TILT) tilt(x, y);                    // inside the turned room
    if (clearance_at(w.dist_truth, x, y) > 0.45f) { kid_pose = {x, y, wrap(-3.14159265f + 6.2831853f * urand())}; break; }
  }
  static Amcl am; am.p = w.parts; am.tmp = w.parts2; am.w = w.pw; am.n = NP; am.rng = {99u + seed};
  am.init(est, 0.05f, 0.03f);
  Pose amcl_pose = est, odom_at_update = rb.odom; bool first = true;
  bool kid = false; int kid_cycle = 0, low = 0, n_loc_total = 0;
  float* errs = mw.errs; int ne = 0; int cycle = 0;
  bool detected = false;
  auto localize_reset = [&](Pose p) { am.init(p, 0.05f, 0.03f); amcl_pose = p; odom_at_update = rb.odom; first = true; };

  // ---- live obstacle layer
  uint8_t* live = mw.live; memset(live, 0, CELLS); bool live_any = false;
  auto live_fields = [&]() {
    bc::distance_field(live, mw.live_dist);                                         // cells x 10 to the nearest live cell
    for (int k = 0; k < CELLS; k++) mw.occ_plan[k] = (w.occ[k] || mw.live_dist[k] <= 14) ? 1 : 0;   // live grown by 1 cell
    bc::distance_field(mw.occ_plan, mw.dist_plan); bc::costmap(mw.dist_plan, mw.cost_plan); pad_cost(mw.dist_plan, mw.cost_plan);
  };
  auto live_clear = [&]() { if (live_any) { memset(live, 0, CELLS); live_any = false; } };

  for (int li = 0; li < NL; li++) {
    Leg& L = legs[li];
    if (R.need_help) { logf_(h, "[k]   leg %d %s: NOT_RUN (waiting for help)", li + 1, L.name); continue; }
    int path_n = 0, pidx2 = 0, fails = 0, n_loc = 0, rec_i = 0, rec_left = 0; float rec_v = 0, rec_w = 0;
    float t_leg = 0, t_anchor = 0; Pose anchor = amcl_pose; const char* state = "TIMEOUT";
    int gcell = bc::cy(L.y) * W + bc::cx(L.x);
    auto start_recovery = [&]() {                                    // navsim.py RECOVER: clear, spin, backup, wait
      int kind = rec_i++ % 4; R.recoveries++; live_clear(); path_n = 0;
      if (kind == 0) { rec_left = 0; }                               // clear: forget the live layer and replan
      else if (kind == 1) { rec_left = 20; rec_v = 0; rec_w = 1.0f; }
      else if (kind == 2) { rec_left = 10; rec_v = -0.1f; rec_w = 0; }
      else { rec_left = 14; rec_v = 0; rec_w = 0; }
    };
    if (w.cost[gcell] >= 253) { state = "ABORT_GOAL_BLOCKED"; }
    else for (int cyc = 0; cyc < 7 * 150; cyc++) {
      int64_t c0 = h.now_us();
      m = sense();
      Pose d = ominus(odom_at_update, rb.odom);
      if (first || hypotf(d.x, d.y) > 0.25f || fabsf(d.th) > 0.2f) {
        if (!first) am.motion(d);
        amcl_pose = am.measure(w.lik, w.px, w.py, m); odom_at_update = rb.odom; first = false;
      }
      Pose e = oplus(amcl_pose, ominus(odom_at_update, rb.odom));
      float le = hypotf(e.x - rb.truth.x, e.y - rb.truth.y);
      if (ne < ERRS_CAP) errs[ne++] = le;
      float lf, mf; beam_check(w.occ, e, w.px, w.py, m, 8, lf, mf);
      bool healthy = lf < 0.1f;
      // ---- lost-pose check (spin mode): 3 bad scans in a row
      if (mode == MODE_SPIN) {
        low = lf > 0.25f ? low + 1 : 0;
        if (low >= 3) {
          low = 0; n_loc++; n_loc_total++;
          if (le < 0.3f) R.false_lost++;
          if (kid && !detected) { detected = true; R.detect_s = (cycle - kid_cycle) * DT; }
          if (n_loc > 3) { state = "ABORT_TOO_MANY_RELOCS"; break; }
          live_clear(); rec_left = 0;
          // ===== LOCALIZE =====
          Event& ev = R.ev[R.n_ev < 4 ? R.n_ev : 3]; if (R.n_ev < 4) R.n_ev++;
          memset(&ev, 0, sizeof(ev));
          float t_loc0 = cycle * DT; int64_t s0 = h.now_us();
          static Pose hyp[MAXHYP]; float key[MAXHYP]; int nh = 0;
          // the search uses a scan taken standing still; after a failed search the robot turns for 5 cycles (each step is
          // followed by one lidar revolution, as on the robot and in mission_nodes.h) and searches the last scan again
          m = sense();
          for (int att = 0; att < 5 && nh == 0; att++) {
            nh = global_search(mw, w.px, w.py, m, hyp, key);
            if (nh == 0) for (int t = 0; t < 5; t++) { rb.step(w.dist_truth, 0.0f, 0.8f); cycle++; t_leg += DT; m = sense(); }
          }
          double sm = (h.now_us() - s0) / 1000.0; if (sm > R.search_ms_max) R.search_ms_max = sm;
          ev.n_hyp = nh;
          if (nh == 0) { ev.result = EV_NOT_FOUND; R.need_help = true; state = "STOPPED"; ev.took_s = cycle * DT - t_loc0; break; }
          if (nh == 1) {
            ev.result = EV_UNIQUE; ev.n_alive = 1; ev.err = hypotf(hyp[0].x - rb.truth.x, hyp[0].y - rb.truth.y);
            localize_reset(hyp[0]); path_n = 0; anchor = hyp[0]; t_anchor = 0; ev.took_s = cycle * DT - t_loc0;
            logf_(h, "[k]   relocalized: unique candidate, error %.2f m (search %.0f ms)", ev.err, sm);
            continue;
          }
          // several candidates: turn in place only, drop candidates with decisive beams
          bool dead[MAXHYP] = {}; int streak[MAXHYP] = {}; float acc = 0; int resolved = -1;
          Pose last_o = rb.odom;
          float* meas = mw.meas; float (*ex)[MAXPTS / 2] = (float (*)[MAXPTS / 2])mw.ex;
          for (float t_m = 0; t_m < 15.0f; t_m += DT) {
            rb.step(w.dist_truth, 0.0f, 0.6f); cycle++; t_leg += DT;
            Pose dl = ominus(last_o, rb.odom); last_o = rb.odom;
            for (int k = 0; k < nh; k++) hyp[k] = oplus(hyp[k], dl);
            acc += fabsf(dl.th) + hypotf(dl.x, dl.y);
            m = sense();
            if (acc <= 0.15f) continue;
            acc = 0; int64_t u0 = h.now_us();
            int nb = 0; for (int k = 0; k < m && nb < MAXPTS / 2; k += 2) { meas[nb++] = hypotf(w.px[k], w.py[k]); }
            int nlong[MAXHYP];
            for (int k = 0; k < nh; k++) if (!dead[k]) {
              hyp[k] = refine(mw, hyp[k], w.px, w.py, m);
              nlong[k] = 0; int b2 = 0;
              for (int q = 0; q < m && b2 < nb; q += 2, b2++) {
                float e2 = ray_occ(w.occ, hyp[k].x, hyp[k].y, hyp[k].th + atan2f(w.py[q], w.px[q]));
                ex[k][b2] = e2; if (meas[b2] > e2 + tol_far(e2)) nlong[k]++;
              }
            }
            int b = -1; for (int k = 0; k < nh; k++) if (!dead[k] && (b < 0 || nlong[k] < nlong[b])) b = k;
            for (int k = 0; k < nh; k++) if (!dead[k] && k != b) {
              int dec = 0;
              for (int q = 0; q < nb; q++) {
                float eb = ex[b][q], ek = ex[k][q];
                if (fabsf(ek - eb) > 0.4f && fabsf(meas[q] - eb) < tol_far(eb) && meas[q] > ek + tol_far(ek)) dec++;
              }
              // navsim.py: 6 decisive beams of all beams. The chip uses every 2nd beam and still needs 6, i.e. about twice as
              // strict: the chip's SLAM map has 1-cell walls whose ends are often one cell too long, which gives the TRUE
              // pose 2-3 "see-through" beams near wall ends (found on s_corridor seed 0, where 3 killed the true pose).
              streak[k] = dec >= 6 ? streak[k] + 1 : 0;
              if (streak[k] >= 3) dead[k] = true;
            }
            double um = (h.now_us() - u0) / 1000.0; if (um > R.multi_update_ms_max) R.multi_update_ms_max = um;
            int alive = 0, last = -1; for (int k = 0; k < nh; k++) if (!dead[k]) { alive++; last = k; }
#ifdef MIS_TRACE
            for (int k = 0; k < nh; k++) printf("    t_m %.1f hyp %d err %.2f nlong %d streak %d dead %d%s\n", t_m, k, hypotf(hyp[k].x - rb.truth.x, hyp[k].y - rb.truth.y), dead[k] ? -1 : nlong[k], streak[k], dead[k], k == b ? " <- best" : "");
#endif
            if (alive == 1) { resolved = last; break; }
          }
          ev.took_s = cycle * DT - t_loc0;
          if (resolved >= 0) {
            ev.result = EV_RESOLVED; ev.n_alive = 1; ev.err = hypotf(hyp[resolved].x - rb.truth.x, hyp[resolved].y - rb.truth.y);
            localize_reset(hyp[resolved]); path_n = 0; anchor = hyp[resolved]; t_anchor = 0;
            logf_(h, "[k]   relocalized: %d candidates, resolved by turning, error %.2f m (search %.0f ms)", nh, ev.err, sm);
            continue;
          }
          ev.result = EV_AMBIGUOUS_STOPPED; int na = 0; float mn = 9;
          for (int k = 0; k < nh; k++) if (!dead[k]) { float er = hypotf(hyp[k].x - rb.truth.x, hyp[k].y - rb.truth.y); ev.hyp_err[na++] = er; if (er < mn) mn = er; }
          ev.n_alive = na; ev.truth_kept = mn < 0.3f;
          logf_(h, "[k]   still %d candidates after turning 15 s (true pose among them: %s): stop and ask for help", na, ev.truth_kept ? "yes" : "no");
          R.need_help = true; state = "STOPPED"; break;
        }
      }
      if (hypotf(L.x - e.x, L.y - e.y) < 0.2f) { state = "DONE"; break; }
      // ---- live obstacle layer: mark unmapped points near the robot while the pose is healthy
      if (healthy) {
        float c = cosf(e.th), sn = sinf(e.th); int idxs[MAXPTS]; int nn = 0;
        for (int k = 0; k < m; k++) {
          if (w.px[k] * w.px[k] + w.py[k] * w.py[k] > 1.5f * 1.5f) continue;
          int i = bc::cx(e.x + c * w.px[k] - sn * w.py[k]), j = bc::cy(e.y + sn * w.px[k] + c * w.py[k]);
          if (i < 0 || i >= W || j < 0 || j >= H) continue;
          if (w.dist[j * W + i] / 10.0f * RES > 0.12f) idxs[nn++] = j * W + i;
        }
        if (nn > 3) {
          int added = 0; for (int k = 0; k < nn; k++) if (!live[idxs[k]]) { live[idxs[k]] = 1; added++; }
          if (added) {
            live_any = true; live_fields();
            for (int q = pidx2; q < path_n && q < pidx2 + 40; q++) {
              int i = bc::cx(w.path_x[q]), j = bc::cy(w.path_y[q]);
              if (i >= 0 && i < W && j >= 0 && j < H && mw.live_dist[j * W + i] <= 40) { path_n = 0; R.obstacle_replans++; break; }
            }
          }
        }
      }
      float v = 0, wv = 0;
      if (rec_left > 0) {                                              // a recovery behaviour is running
        v = rec_v; wv = rec_w;
        if (--rec_left == 0) { path_n = 0; anchor = e; t_anchor = 0; }
      } else {
        if (path_n == 0 || cyc % 7 == 0) {
          int s = bc::cy(e.y) * W + bc::cx(e.x);
          if (s >= 0 && s < CELLS) {
            uint8_t* cost = live_any ? mw.cost_plan : w.cost;
            // let the robot leave a lethal cell it believes it is in (navsim.py: cost around the start capped at 200)
            uint8_t saved[25]; int si = s % W, sj = s / W, q = 0;
            for (int dj = -2; dj <= 2; dj++) for (int di = -2; di <= 2; di++) {
              int c = (sj + dj) * W + (si + di); if (c < 0 || c >= CELLS) { saved[q++] = 0; continue; }
              saved[q++] = cost[c]; if (cost[c] > 200) cost[c] = 200; }
            bc::AStarWork wk = {w.g, w.heap, w.pos, w.state};
            bc::AStarResult ar = cost[gcell] >= 253 ? bc::AStarResult{false, 0, 0} : bc::astar_opt(cost, s, gcell, wk, w.f, 6.0f);
            q = 0; for (int dj = -2; dj <= 2; dj++) for (int di = -2; di <= 2; di++) { int c = (sj + dj) * W + (si + di); if (c >= 0 && c < CELLS) cost[c] = saved[q]; q++; }
            if (ar.found) { path_n = extract_path(w.g, w.state, s, gcell, w.path_x, w.path_y, PATH_CAP); pidx2 = 0; }
            else { path_n = 0; if (++fails >= 4) { state = "ABORT_NO_PATH"; R.aborts++; break; } start_recovery(); }
          }
        }
        float clr = clearance_at(w.dist, e.x, e.y);
        if (rec_left == 0 && path_n > 1) {
          // collision-checked pure pursuit: try shorter lookaheads until the arc (v, w) up to the carrot is clear of the scan;
          // if none is clear, use the fallback below
          const float las[4] = {clr < 0.35f ? 0.25f : 0.4f, 0.18f, 0.12f, 0.08f};
          bool ok = false;
          for (int t = 0; t < 4 && !ok; t++) {
            int id = pidx2; pursue(e, w.path_x, w.path_y, path_n, id, 0.3f, clr, v, wv, las[t]);
            if (v <= 0 || !arc_hits(w.px, w.py, m, v, wv, las[t])) { ok = true; pidx2 = id; }
          }
          if (!ok) {
            // fallback like the Nav2 DWB controller: sample slow arcs, keep the clear ones, take the one that ends closest
            // to the carrot 0.3 m ahead on the path; none clear: turn in place towards the carrot
            R.arc_blocked++;
            int q = pidx2; while (q < path_n - 1 && hypotf(w.path_x[q] - e.x, w.path_y[q] - e.y) < 0.3f) q++;
            float best = 1e9f, bv = 0, bw = 0;
            for (int iv = 1; iv <= 2; iv++) for (int iw = -4; iw <= 4; iw++) {
              float cv = 0.05f * iv, cw = 0.15f * iw;
              if (arc_hits(w.px, w.py, m, cv, cw, cv * 1.0f + 0.02f)) continue;
              float th = e.th + cw * 0.5f, ex2 = e.x + cv * cosf(th), ey2 = e.y + cv * sinf(th);   // pose after 1 s
              float sc = hypotf(w.path_x[q] - ex2, w.path_y[q] - ey2);
              if (sc < best) { best = sc; bv = cv; bw = cw; }
            }
            if (best < 1e8f) { v = bv; wv = bw; }
            else {
              float a = wrap(atan2f(w.path_y[q] - e.y, w.path_x[q] - e.x) - e.th);
              v = 0; wv = fabsf(a) > 0.05f ? fmaxf(-0.6f, fminf(0.6f, 1.5f * a)) : 0;
            }
          }
        }
        // progress checker (Nav2 SimpleProgressChecker: 0.5 m in 10 s)
        if (hypotf(e.x - anchor.x, e.y - anchor.y) > 0.5f) { anchor = e; t_anchor = 0; }
        else if ((t_anchor += DT) > 10.0f) {
          anchor = e; t_anchor = 0;
          if (++fails >= 4) { state = "ABORT_NO_PROGRESS"; R.aborts++; break; }
          start_recovery();
        }
      }
      // ---- collision monitor on the raw scan (does not need the pose), like the Nav2 collision monitor:
      //      approach model (stop if the commanded arc reaches a point within ROBOT_R + 0.01 m in the next 1.0 s)
      //      and a slowdown zone 0.60 m wide, 0.5 m ahead -> half speed
      if (v != 0) {
        if (arc_hits(w.px, w.py, m, v, wv, fmaxf(0.05f, fabsf(v) * 1.0f), ROBOT_R + 0.01f)) { v = 0; R.collision_stops++; }
        else {
          int nslow = 0;
          for (int k = 0; k < m; k++) {
            float ax = v > 0 ? w.px[k] : -w.px[k], ay = fabsf(w.py[k]);
            if (ax > 0 && ay < 0.30f && ax < ROBOT_R + 0.5f) nslow++;
          }
          if (nslow > 2) v *= 0.5f;
        }
      }
      if (v > 0.01f && le > 0.5f) { if (kid && !detected && mode == MODE_SPIN) R.wrong_drive_before_detect_s += DT; else R.wrong_drive_after_s += DT; }
      double cm = (h.now_us() - c0) / 1000.0; if (cm > R.max_cycle_ms) R.max_cycle_ms = cm;
      rb.step(w.dist_truth, v, wv); cycle++; t_leg += DT;
      if (li == kid_leg && !kid && t_leg > kid_after) {             // kidnap: carried to another place, odometry does not see it
        rb.carry(kid_pose); kid = true; kid_cycle = cycle; R.kidnapped = true;
        logf_(h, "[k]   kidnapped during leg %d to (%.2f, %.2f, %.2f)", li + 1, kid_pose.x, kid_pose.y, kid_pose.th);
      }
    }
    if (L.reachable) R.legs_reachable++;
    if (!L.reachable && strncmp(state, "ABORT", 5) != 0) R.unreach_ok = false;
    if (L.reachable && strcmp(state, "DONE") == 0) R.legs_done++;
    logf_(h, "[k]   leg %d %s (%.1f, %.1f): %s | true distance %.2f m", li + 1, L.name, L.x, L.y, state, hypotf(L.x - rb.truth.x, L.y - rb.truth.y));
  }
  for (int i = 1; i < ne; i++) { float v = errs[i]; int j = i - 1; while (j >= 0 && errs[j] > v) { errs[j + 1] = errs[j]; j--; } errs[j + 1] = v; }
  R.loc_err_median = ne ? errs[ne / 2] : 0;
  R.contacts = rb.contacts;
  // outcome: did the robot ever drive with a wrong pose after the lost-pose handling had a chance to act?
  bool wrong_reloc = false; for (int i = 0; i < R.n_ev; i++) if ((R.ev[i].result == EV_UNIQUE || R.ev[i].result == EV_RESOLVED) && R.ev[i].err > 0.5f) wrong_reloc = true;
  if (R.contacts > 0) R.outcome_code = OUT_CONTACT;
  else if (R.wrong_drive_after_s > 0.5f || wrong_reloc) R.outcome_code = OUT_WRONG_POSE;
  else if (R.need_help) R.outcome_code = OUT_STOPPED_SAFELY;
  else if (R.legs_done < R.legs_reachable) R.outcome_code = OUT_INCOMPLETE;
  else R.outcome_code = OUT_CORRECT;
  strcpy(R.outcome, out_name(R.outcome_code));
  (void)n_loc_total;
  x4sim::geo().n = 0;                                            // other commands use the grid walls
  return R;
}

inline const char* slices_note(const MResult& R) {             // (empty for a search in one go)
  static char b[64]; b[0] = 0;
  if (R.search_cycles_max > 1) snprintf(b, sizeof b, " (in %d cycles, longest %.0f ms)", R.search_cycles_max, R.search_slice_ms_max);
  return b;
}
inline void print_result(const Hooks& h, const MResult& R) {
  char evs[200] = ""; int p = 0;
  for (int i = 0; i < R.n_ev; i++) {
    const Event& e = R.ev[i];
    if (e.result == EV_AMBIGUOUS_STOPPED) p += snprintf(evs + p, sizeof(evs) - p, "%s(%d->%d, truth kept %s) ", ev_name(e.result), e.n_hyp, e.n_alive, e.truth_kept ? "yes" : "no");
    else p += snprintf(evs + p, sizeof(evs) - p, "%s(%d cand, err %.2f m) ", ev_name(e.result), e.n_hyp, e.err);
  }
  logf_(h, "MISSION %s %s seed %u%s%s | %s | legs %d/%d, unreachable handled %s | kidnap %s, detected after %.2f s, false alarms %d | %s| wrong-pose driving %.1f s (+%.1f s before detection) | contacts %d (%.1f s) | obstacle replans %d, collision stops %d, arc blocked %d, recoveries %d, aborts %d | loc median %.3f m | max cycle %.0f ms, search %.0f ms, multi update %.0f ms%s",
        world_name(R.world), mode_name(R.mode), R.seed, (R.opts & OPT_KIDNAP) ? " kidnap" : "", (R.opts & OPT_BOX) ? " box" : "",
        R.outcome, R.legs_done, R.legs_reachable, R.unreach_ok ? "yes" : "NO",
        R.kidnapped ? "yes" : "no", R.detect_s, R.false_lost, R.n_ev ? evs : "no relocalization ", R.wrong_drive_after_s, R.wrong_drive_before_detect_s,
        R.contacts, R.contacts * DT, R.obstacle_replans, R.collision_stops, R.arc_blocked, R.recoveries, R.aborts, R.loc_err_median, R.max_cycle_ms, R.search_ms_max, R.multi_update_ms_max, slices_note(R));
}

}  // namespace mis
