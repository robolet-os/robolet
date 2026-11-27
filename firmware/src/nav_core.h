// Virtual driving on the chip: the navsim.py pipeline in C++.
//   phase 1 mapping   : robot follows a fixed tour; SLAM = odometry prediction + two-stage scan-to-map matching
//                       (coarse 5 cm / 0.03 rad, fine 1.25 cm / 0.0075 rad) + log-odds occupancy grid updated at keyframes
//   phase 2 navigation: on the SLAM map -> AMCL (odometry motion model, likelihood field, low-variance resampling,
//                       Nav2 defaults update_min_d 0.25 m / update_min_a 0.2 rad) + A* on the inflated costmap
//                       (replanned at 1 Hz) + pure pursuit
// The sensor is the virtual X4 (x4_sim.h) decoded by the real parser; the robot has wheel slip, odometry scale error
// and gyro bias like the simulator. Pure C++: the same file runs on a PC (test_host/test_nav.cpp) and on the chip.
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "bench_core.h"
#include "x4_parser.h"
#include "x4_sim.h"

namespace nav {
using bc::Pose; using bc::W; using bc::H; using bc::CELLS; using bc::RES; using bc::OX; using bc::OY; using bc::INV_RES;

static const float DT = 1.0f / 7.0f;          // one lidar revolution per control cycle (X4 at about 7 Hz)
static const float ROBOT_R = 0.18f;
static const int MAXPTS = 800, PATH_CAP = 4000, NP = 500;

struct Hooks { int64_t (*now_us)(); void (*log)(const char*); };

// All large buffers; the caller allocates them (on the chip: PSRAM except lik).
struct Work {
  uint8_t *occ_truth, *dist_truth;                    // simulated world
  int8_t  *lo;                                        // SLAM log-odds grid
  uint8_t *occ, *dist, *lik, *cost;                   // derived from the SLAM grid
  float *g, *f; uint16_t *heap, *pos; uint8_t *state; // A* work arrays
  float *path_x, *path_y;                             // PATH_CAP
  Pose *parts, *parts2; float *pw;                    // NP particles
  uint8_t *rev;                                       // one X4 revolution (x4sim::REV_BYTES)
  float *px, *py; int32_t *idx;                       // MAXPTS scan points
};

struct Stat { double sum = 0, mx = 0; int n = 0; void add(double v) { sum += v; if (v > mx) mx = v; n++; } double avg() const { return n ? sum / n : 0; } };

struct Result {
  // mapping
  int map_cycles = 0, keyframes = 0; float map_dist_m = 0, slam_err_max = 0, slam_err_final = 0, slam_th_err_max = 0;
  float map_wall_err_mean = 0, map_within_5cm = 0; int map_occ_cells = 0;
  Stat t_parse, t_slam, t_slam_update;
  // navigation
  int goals = 0, goals_reached = 0, goals_unreachable = 0, nav_cycles = 0, amcl_updates = 0, plans = 0;
  float goal_true_err_max = 0;      // distance from the TRUE robot position to the goal when the robot declared arrival
  uint32_t seed = 0;
  float loc_err_max = 0, loc_err_median = 0, loc_th_err_max = 0;
  Stat t_amcl, t_plan, t_control;
  // both
  int contacts = 0; float min_clearance = 9.0f;
  double cpu_load_map = 0, cpu_load_nav = 0;
  bool pass = false;
};

// ---------------------------------------------------------------- helpers
inline float wrap(float a) { while (a > 3.14159265f) a -= 6.2831853f; while (a < -3.14159265f) a += 6.2831853f; return a; }
inline Pose oplus(Pose a, Pose d) { float c = cosf(a.th), s = sinf(a.th); return {a.x + c * d.x - s * d.y, a.y + s * d.x + c * d.y, wrap(a.th + d.th)}; }
inline Pose ominus(Pose a, Pose b) {   // b expressed in frame a
  float c = cosf(a.th), s = sinf(a.th), dx = b.x - a.x, dy = b.y - a.y;
  return {c * dx + s * dy, -s * dx + c * dy, wrap(b.th - a.th)};
}
inline float gauss(x4sim::Rng& r) {
  float u1 = ((r.next() & 0xFFFFFF) + 1) / 16777217.0f, u2 = (r.next() & 0xFFFFFF) / 16777216.0f;
  return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}
inline float clearance_at(const uint8_t* dist, float x, float y) {
  int i = bc::cx(x), j = bc::cy(y);
  if (i < 0 || i >= W || j < 0 || j >= H) return 0;
  return dist[j * W + i] / 10.0f * RES;
}

// ---------------------------------------------------------------- simulated robot
struct Robot {
  Pose truth, odom; x4sim::Rng rng; int contacts = 0; float min_clear = 9.0f;
  Pose truth_prev, odom_prev; bool has_prev = false;       // pose and odometry at the start of the last step (motion distortion)
  void hold() { truth_prev = truth; odom_prev = odom; has_prev = true; }   // standing still: no motion during the scan
  // carried to p (kidnap): the next scan is taken standing at p; odometry did not see it, so the lidar driver still
  // removes the motion of the last step (it only knows the odometry)
  void carry(Pose p) { truth = p; truth_prev = p; }
  void step(const uint8_t* dist_truth, float v, float w) {
    truth_prev = truth; odom_prev = odom; has_prev = true;
    float vt = v * (1.0f + 0.02f * gauss(rng)), wt = w * (1.0f + 0.02f * gauss(rng));   // wheel slip
    float tm = truth.th + wt * DT / 2;
    Pose nt = {truth.x + vt * DT * cosf(tm), truth.y + vt * DT * sinf(tm), wrap(truth.th + wt * DT)};
    float c = clearance_at(dist_truth, nt.x, nt.y);
    if (c < ROBOT_R) { contacts++; return; }                    // blocked by a wall: do not move
    if (c < min_clear) min_clear = c;
    truth = nt;
    float vo = vt * 1.02f, wo = wt + 0.005f + 0.01f * gauss(rng);                   // odometry: scale error, gyro bias
    float om = odom.th + wo * DT / 2;
    odom = {odom.x + vo * DT * cosf(om), odom.y + vo * DT * sinf(om), wrap(odom.th + wo * DT)};
  }
};

// ---------------------------------------------------------------- lidar: virtual X4 -> parser -> points
struct ScanBuf { X4Point pts[MAXPTS]; int n; };
inline void collect(const X4Point* p, int n, bool start, void* u) {
  ScanBuf* s = (ScanBuf*)u; if (start) s->n = 0;
  for (int i = 0; i < n && s->n < MAXPTS; i++) s->pts[s->n++] = p[i];
}

// one revolution taken while the robot made its last step (x4sim::opts().distort), or standing at the true pose
inline int scan_rev(Work& w, const Robot& rb, x4sim::Rng& lrng) {
  if (x4sim::opts().distort && rb.has_prev) return x4sim::revolution_moving(w.occ_truth, rb.truth_prev, rb.truth, w.rev, lrng);
  return x4sim::revolution(w.occ_truth, rb.truth, w.rev, lrng);
}
// decoded points -> robot frame at the end of the revolution; with deskew the odometry change of the step is removed
inline int scan_points(const Robot& rb, const ScanBuf& scan, float* px, float* py) {
  if (x4sim::opts().deskew && rb.has_prev)
    return x4sim::to_robot_frame_deskew(scan.pts, scan.n, px, py, MAXPTS, ominus(rb.odom_prev, rb.odom), x4sim::opts().deskew == 1);
  return x4sim::to_robot_frame(scan.pts, scan.n, px, py, MAXPTS);
}

// ---------------------------------------------------------------- SLAM
inline void slam_insert(int8_t* lo, Pose p, const float* px, const float* py, int n) {
  float c = cosf(p.th), s = sinf(p.th);
  int ri = bc::cx(p.x), rj = bc::cy(p.y);
  for (int k = 0; k < n; k++) {
    int hi = bc::cx(p.x + c * px[k] - s * py[k]), hj = bc::cy(p.y + s * px[k] + c * py[k]);
    if (hi < 0 || hi >= W || hj < 0 || hj >= H) continue;
    int x = ri, y = rj, dx = abs(hi - ri), dy = -abs(hj - rj), sx = ri < hi ? 1 : -1, sy = rj < hj ? 1 : -1, err = dx + dy;
    while (!(x == hi && y == hj)) {                              // Bresenham: cells passed through are free
      int8_t& v = lo[y * W + x]; if (v > -40) v--;
      int e2 = 2 * err; if (e2 >= dy) { err += dy; x += sx; } if (e2 <= dx) { err += dx; y += sy; }
    }
    int8_t& h = lo[hj * W + hi]; h = h > 36 ? 40 : h + 4;          // end cell is occupied
  }
}
inline void slam_rebuild(Work& w) {
  for (int k = 0; k < CELLS; k++) w.occ[k] = w.lo[k] > 3 ? 1 : 0;
  bc::distance_field(w.occ, w.dist); bc::likelihood_field(w.dist, w.lik); bc::costmap(w.dist, w.cost);
}
// fine search below the cell size (nearest-cell lookup, every 2nd point)
inline Pose match_fine(const uint8_t* lik, const float* px, const float* py, int n, Pose g) {
  Pose best = g; uint32_t bs = 0;
  for (int t = -2; t <= 2; t++) {
    float th = g.th + t * 0.0075f, c = cosf(th), s = sinf(th);
    for (int u = -2; u <= 2; u++) for (int v = -2; v <= 2; v++) {
      float x0 = (g.x + u * 0.0125f - OX) * INV_RES, y0 = (g.y + v * 0.0125f - OY) * INV_RES; uint32_t sc = 0;
      for (int k = 0; k < n; k += 2) {
        float fx = x0 + (c * px[k] - s * py[k]) * INV_RES, fy = y0 + (s * px[k] + c * py[k]) * INV_RES;
        int i = (int)fx, j = (int)fy;
        if (fx >= 0 && fy >= 0 && i < W && j < H) sc += lik[j * W + i];
      }
      if (sc > bs) { bs = sc; best = {g.x + u * 0.0125f, g.y + v * 0.0125f, wrap(th)}; }
    }
  }
  return best;
}

// ---------------------------------------------------------------- AMCL
struct Amcl {
  Pose* p; Pose* tmp; float* w; int n; x4sim::Rng rng; bc::AmclTable tab;
  void init(Pose c, float sxy, float sth) {
    for (int i = 0; i < n; i++) p[i] = {c.x + sxy * gauss(rng), c.y + sxy * gauss(rng), wrap(c.th + sth * gauss(rng))};
    bc::amcl_table(tab);
  }
  void motion(Pose d) {                                          // odometry model, alpha1..4 = 0.2 (Nav2 defaults)
    float trans = hypotf(d.x, d.y), rot1 = trans < 0.01f ? 0.0f : atan2f(d.y, d.x), rot2 = wrap(d.th - rot1);
    const float a1 = 0.2f, a2 = 0.2f, a3 = 0.2f, a4 = 0.2f;
    for (int i = 0; i < n; i++) {
      float r1 = rot1 - sqrtf(a1 * rot1 * rot1 + a2 * trans * trans) * gauss(rng);
      float tr = trans - sqrtf(a3 * trans * trans + a4 * (rot1 * rot1 + rot2 * rot2)) * gauss(rng);
      float r2 = rot2 - sqrtf(a1 * rot2 * rot2 + a2 * trans * trans) * gauss(rng);
      p[i].x += tr * cosf(p[i].th + r1); p[i].y += tr * sinf(p[i].th + r1); p[i].th = wrap(p[i].th + r1 + r2);
    }
  }
  Pose measure(const uint8_t* lik, const float* px, const float* py, int m) {
    bc::amcl_weights_opt(lik, px, py, m, p, n, 60, tab, w);
    float mx = w[0]; for (int i = 1; i < n; i++) if (w[i] > mx) mx = w[i];
    float sum = 0; for (int i = 0; i < n; i++) { w[i] = expf(w[i] - mx); sum += w[i]; }
    float sx = 0, sy = 0, sc = 0, ss = 0;
    for (int i = 0; i < n; i++) { w[i] /= sum; sx += w[i] * p[i].x; sy += w[i] * p[i].y; sc += w[i] * cosf(p[i].th); ss += w[i] * sinf(p[i].th); }
    Pose mean = {sx, sy, atan2f(ss, sc)};
    // low-variance resampling (every update, like Nav2 resample_interval 1)
    float r = (rng.next() & 0xFFFFFF) / 16777216.0f / n, c = w[0]; int i = 0;
    for (int m2 = 0; m2 < n; m2++) { float u = r + (float)m2 / n; while (u > c && i < n - 1) { i++; c += w[i]; } tmp[m2] = p[i]; }
    memcpy(p, tmp, sizeof(Pose) * n);
    return mean;
  }
};

// ---------------------------------------------------------------- planning and control
inline int extract_path(const float* g, const uint8_t* st, int start, int goal, float* X, float* Y, int cap) {
  static const int DI[8] = {1, -1, 0, 0, 1, 1, -1, -1}, DJ[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  int n = 0, k = goal;
  // walk down g from the goal to the start, writing backwards, then reverse
  while (n < cap) {
    X[n] = OX + (k % W + 0.5f) * RES; Y[n] = OY + (k / W + 0.5f) * RES; n++;
    if (k == start) break;
    int i = k % W, j = k / W, best = -1; float bg = g[k];
    for (int d = 0; d < 8; d++) {
      int ni = i + DI[d], nj = j + DJ[d]; if (ni < 0 || ni >= W || nj < 0 || nj >= H) continue;
      int q = nj * W + ni; if (st[q] != 2) continue;
      if (g[q] < bg) { bg = g[q]; best = q; }
    }
    if (best < 0) return 0;
    k = best;
  }
  for (int a = 0, b = n - 1; a < b; a++, b--) { float t = X[a]; X[a] = X[b]; X[b] = t; t = Y[a]; Y[a] = Y[b]; Y[b] = t; }
  return n;
}

// pure pursuit, lookahead 0.4 m (la: lookahead; the mission uses 0.25 m near walls, like Regulated Pure Pursuit)
inline void pursue(Pose e, const float* X, const float* Y, int n, int& idx, float vmax, float clear, float& v, float& w, float la = 0.4f) {
  float best = 1e9f; int bi = idx;
  for (int i = idx; i < n && i < idx + 60; i++) { float d = hypotf(X[i] - e.x, Y[i] - e.y); if (d < best) { best = d; bi = i; } }
  idx = bi;
  int li = idx; while (li < n - 1 && hypotf(X[li] - e.x, Y[li] - e.y) < la) li++;
  float dx = X[li] - e.x, dy = Y[li] - e.y, a = wrap(atan2f(dy, dx) - e.th), L = hypotf(dx, dy);
  if (L < 0.05f) { v = 0; w = 0; return; }
  if (fabsf(a) > 0.8f) { v = 0; w = a > 0 ? 0.8f : -0.8f; return; }   // turn in place first
  v = vmax * (1.0f - fabsf(a) / 1.2f);
  if (clear < 0.35f) v *= 0.6f;                                      // slow down near walls
  float dg = hypotf(X[n - 1] - e.x, Y[n - 1] - e.y); if (v > 0.08f + 0.5f * dg) v = 0.08f + 0.5f * dg;
  w = 2.0f * v * sinf(a) / L; if (w > 1.2f) w = 1.2f; if (w < -1.2f) w = -1.2f;
}

// ---------------------------------------------------------------- the whole run
inline void logf_(const Hooks& h, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
inline void logf_(const Hooks& h, const char* fmt, ...) {
  char b[512]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap); h.log(b);
}

inline Result run(Work& w, const Hooks& h, uint32_t seed = 0) {
  Result R; R.seed = seed;
  static ScanBuf scan; scan.n = 0;
  X4Parser parser(collect, &scan);
  x4sim::geo().n = 0;                                    // grid walls
  bc::build_room(w.occ_truth); bc::distance_field(w.occ_truth, w.dist_truth);
  memset(w.lo, 0, CELLS);
  Robot rb; rb.truth = {1.5f, 1.5f, 0.0f}; rb.odom = rb.truth; rb.rng = {777u + seed};
  x4sim::Rng lrng{4242u + seed};
  double robot_us_map = 0, robot_us_nav = 0;

  auto sense = [&](void) -> int {               // one revolution during the last step -> robot-frame points
    int nb = scan_rev(w, rb, lrng);
    int64_t t0 = h.now_us();
    parser.feed(w.rev, nb);
    int m = scan_points(rb, scan, w.px, w.py); rb.hold();     // a second scan without a step is taken standing still
    R.t_parse.add((h.now_us() - t0) / 1000.0);
    return m;
  };

  // ================= phase 1: mapping =================
  static const float WP[][2] = {{1.5f, 1.5f}, {1.5f, 4.0f}, {2.5f, 4.0f}, {4.0f, 4.0f}, {4.5f, 2.0f}, {6.5f, 1.5f},
                                {7.0f, 3.0f}, {6.5f, 5.2f}, {4.0f, 5.2f}, {2.0f, 4.0f}, {1.5f, 1.5f}};
  const int NW = sizeof(WP) / sizeof(WP[0]);
  int np = 0;
  for (int i = 0; i + 1 < NW && np < PATH_CAP; i++) {
    float L = hypotf(WP[i + 1][0] - WP[i][0], WP[i + 1][1] - WP[i][1]); int k = (int)(L / 0.05f);
    for (int j = 0; j < k && np < PATH_CAP; j++) { w.path_x[np] = WP[i][0] + (WP[i + 1][0] - WP[i][0]) * j / k; w.path_y[np] = WP[i][1] + (WP[i + 1][1] - WP[i][1]) * j / k; np++; }
  }
  w.path_x[np] = WP[NW - 1][0]; w.path_y[np] = WP[NW - 1][1]; np++;
  logf_(h, "[map] tour of %d waypoints, %d path points; SLAM: odometry + coarse/fine scan matching + log-odds map", NW, np);

  Pose est = rb.truth, last_kf = est, last_odom = rb.odom;
  int m = sense();
  slam_insert(w.lo, est, w.px, w.py, m); slam_rebuild(w); R.keyframes = 1;
  int pidx = 0; Pose prev_truth = rb.truth;
  for (int cyc = 0; cyc < 7 * 240; cyc++) {
    // control on the SLAM estimate
    float v, wv;
    int64_t c0 = h.now_us();
    pursue(est, w.path_x, w.path_y, np, pidx, 0.3f, clearance_at(w.dist, est.x, est.y), v, wv);
    double ctrl_us = (double)(h.now_us() - c0);
    if (pidx >= np - 2 && hypotf(w.path_x[np - 1] - est.x, w.path_y[np - 1] - est.y) < 0.15f) break;
    rb.step(w.dist_truth, v, wv);
    R.map_dist_m += hypotf(rb.truth.x - prev_truth.x, rb.truth.y - prev_truth.y); prev_truth = rb.truth;
    m = sense();
    int64_t s0 = h.now_us();
    Pose pred = oplus(est, ominus(last_odom, rb.odom)); last_odom = rb.odom;
    bc::MatchResult cr = bc::csm_fast(w.lik, w.px, w.py, m, pred, w.idx);
    est = match_fine(w.lik, w.px, w.py, m, cr.best);
    R.t_slam.add((h.now_us() - s0) / 1000.0);
    double upd_us = 0;
    Pose dk = ominus(last_kf, est);
    if (hypotf(dk.x, dk.y) > 0.10f || fabsf(dk.th) > 0.10f) {
      int64_t u0 = h.now_us();
      slam_insert(w.lo, est, w.px, w.py, m); slam_rebuild(w); last_kf = est; R.keyframes++;
      upd_us = (double)(h.now_us() - u0); R.t_slam_update.add(upd_us / 1000.0);
    }
    robot_us_map += ctrl_us + (double)(h.now_us() - s0);   // control + scan matching + map update
    float e = hypotf(est.x - rb.truth.x, est.y - rb.truth.y), et = fabsf(wrap(est.th - rb.truth.th));
    if (e > R.slam_err_max) R.slam_err_max = e;
    if (et > R.slam_th_err_max) R.slam_th_err_max = et;
    R.map_cycles++;
    if (R.map_cycles % 140 == 0)
      logf_(h, "[map] t=%5.1f s  driven %.1f m  keyframes %d  SLAM error %.3f m %.3f rad", R.map_cycles * DT, R.map_dist_m, R.keyframes, e, et);
  }
  robot_us_map += R.t_parse.sum * 1000.0;
  R.slam_err_final = hypotf(est.x - rb.truth.x, est.y - rb.truth.y);
  // map quality: every SLAM-occupied cell vs distance to the nearest true wall
  { double s = 0; int n5 = 0, n = 0;
    for (int k = 0; k < CELLS; k++) if (w.occ[k]) { float d = w.dist_truth[k] / 10.0f * RES; s += d; if (d <= 0.05f) n5++; n++; }
    R.map_occ_cells = n; R.map_wall_err_mean = n ? (float)(s / n) : 9; R.map_within_5cm = n ? 100.0f * n5 / n : 0; }
  R.cpu_load_map = R.map_cycles ? robot_us_map / R.map_cycles / (DT * 1e6) * 100.0 : 0;
  logf_(h, "[map] done: %d cycles (%.1f s), %.1f m driven, %d keyframes | SLAM error max %.3f m %.3f rad, final %.3f m",
        R.map_cycles, R.map_cycles * DT, R.map_dist_m, R.keyframes, R.slam_err_max, R.slam_th_err_max, R.slam_err_final);
  logf_(h, "[map] map: %d occupied cells, mean distance to true wall %.3f m, %.1f %% within 5 cm",
        R.map_occ_cells, R.map_wall_err_mean, R.map_within_5cm);
  logf_(h, "[map] time per cycle: parse %.1f ms, scan matching %.1f ms (max %.1f), map update %.1f ms per keyframe (max %.1f) -> %.1f %% of one core",
        R.t_parse.avg(), R.t_slam.avg(), R.t_slam.mx, R.t_slam_update.avg(), R.t_slam_update.mx, R.cpu_load_map);

  // ================= phase 2: navigation on the SLAM map =================
  static Amcl am; am.p = w.parts; am.tmp = w.parts2; am.w = w.pw; am.n = NP; am.rng = {99u + seed};
  am.init(est, 0.10f, 0.05f);
  Pose amcl_pose = est, odom_at_update = rb.odom; bool first = true;
  static const float GOALS[][2] = {{6.5f, 1.5f}, {6.8f, 5.2f}, {2.3f, 5.3f}, {1.5f, 1.5f}};
  R.goals = 4;
  static float errs[4096]; int ne = 0;
  logf_(h, "[nav] AMCL %d particles + A* on the inflated SLAM costmap (1 Hz replanning) + pure pursuit, %d goals", NP, R.goals);
  for (int gi = 0; gi < R.goals; gi++) {
    int path_n = 0, pidx2 = 0, plan_fail = 0; bool reached = false, unreachable = false; int cyc = 0;
    for (cyc = 0; cyc < 7 * 90; cyc++) {
      m = sense();
      int64_t a0 = h.now_us();
      Pose d = ominus(odom_at_update, rb.odom);
      if (first || hypotf(d.x, d.y) > 0.25f || fabsf(d.th) > 0.2f) {        // Nav2 update_min_d / update_min_a
        if (!first) am.motion(d);
        amcl_pose = am.measure(w.lik, w.px, w.py, m);
        odom_at_update = rb.odom; first = false; R.amcl_updates++;
        R.t_amcl.add((h.now_us() - a0) / 1000.0);
      }
      Pose e = oplus(amcl_pose, ominus(odom_at_update, rb.odom));         // map->odom correction applied
      double amcl_us = (double)(h.now_us() - a0);
      float le = hypotf(e.x - rb.truth.x, e.y - rb.truth.y), lt = fabsf(wrap(e.th - rb.truth.th));
      if (le > R.loc_err_max) R.loc_err_max = le;
      if (lt > R.loc_th_err_max) R.loc_th_err_max = lt;
      if (ne < 4096) errs[ne++] = le;
      if (hypotf(GOALS[gi][0] - e.x, GOALS[gi][1] - e.y) < 0.2f) { reached = true; break; }   // judged on the estimate, like a real robot
      double plan_us = 0;
      if (path_n == 0 || cyc % 7 == 0) {                                   // replan at 1 Hz
        int64_t p0 = h.now_us();
        int s = bc::cy(e.y) * W + bc::cx(e.x), gcell = bc::cy(GOALS[gi][1]) * W + bc::cx(GOALS[gi][0]);
        bc::AStarWork wk = {w.g, w.heap, w.pos, w.state};
        bc::AStarResult ar = bc::astar_opt(w.cost, s, gcell, wk, w.f);
        if (ar.found) { path_n = extract_path(w.g, w.state, s, gcell, w.path_x, w.path_y, PATH_CAP); pidx2 = 0; plan_fail = 0; }
        else if (++plan_fail >= 3) { unreachable = true; }
        plan_us = (double)(h.now_us() - p0); R.t_plan.add(plan_us / 1000.0); R.plans++;
      }
      float v = 0, wv = 0;
      int64_t c0 = h.now_us();
      if (path_n > 1) pursue(e, w.path_x, w.path_y, path_n, pidx2, 0.3f, clearance_at(w.dist, e.x, e.y), v, wv);
      double ctrl_us = (double)(h.now_us() - c0); R.t_control.add(ctrl_us / 1000.0);
      if (unreachable) break;                                            // no path 3 times in a row: report, go on
      robot_us_nav += amcl_us + plan_us + ctrl_us;
      rb.step(w.dist_truth, v, wv);
      R.nav_cycles++;
    }
    float tde = hypotf(GOALS[gi][0] - rb.truth.x, GOALS[gi][1] - rb.truth.y);
    if (reached) { R.goals_reached++; if (tde > R.goal_true_err_max) R.goal_true_err_max = tde; }
    if (unreachable) R.goals_unreachable++;
    logf_(h, "[nav] goal %d (%.1f, %.1f): %s after %.1f s | true position (%.2f, %.2f), true distance to goal %.2f m", gi + 1, GOALS[gi][0], GOALS[gi][1],
          reached ? "reached" : (unreachable ? "UNREACHABLE (no path)" : "NOT reached (timeout)"), cyc * DT, rb.truth.x, rb.truth.y, tde);
  }
  // median localization error
  for (int i = 1; i < ne; i++) { float v = errs[i]; int j = i - 1; while (j >= 0 && errs[j] > v) { errs[j + 1] = errs[j]; j--; } errs[j + 1] = v; }
  R.loc_err_median = ne ? errs[ne / 2] : 0;
  double parse_nav = R.t_parse.avg();
  R.cpu_load_nav = R.nav_cycles ? (robot_us_nav / R.nav_cycles / 1000.0 + parse_nav) / (DT * 1000.0) * 100.0 : 0;
  R.contacts = rb.contacts; R.min_clearance = rb.min_clear;
  logf_(h, "[nav] goals reached %d/%d (unreachable %d) in %.1f s, true distance at arrival max %.2f m | AMCL updates %d, localization error median %.3f m, max %.3f m, %.3f rad",
        R.goals_reached, R.goals, R.goals_unreachable, R.nav_cycles * DT, R.goal_true_err_max, R.amcl_updates, R.loc_err_median, R.loc_err_max, R.loc_th_err_max);
  logf_(h, "[nav] time: AMCL update %.1f ms (max %.1f), A* plan %.1f ms (max %.1f), control %.2f ms -> %.1f %% of one core",
        R.t_amcl.avg(), R.t_amcl.mx, R.t_plan.avg(), R.t_plan.mx, R.t_control.avg(), R.cpu_load_nav);
  logf_(h, "[all] contacts %d, minimum clearance %.3f m (robot radius %.2f m)", R.contacts, R.min_clearance, ROBOT_R);
  // goal tolerance 0.25 m on the TRUE position = Nav2 default xy_goal_tolerance
  R.pass = R.slam_err_max < 0.10f && R.map_wall_err_mean < 0.05f && R.goals_reached == R.goals && R.goal_true_err_max <= 0.25f &&
           R.loc_err_max < 0.20f && R.contacts == 0;
  return R;
}

}  // namespace nav
