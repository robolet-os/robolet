// PC test of the virtual X4 -> parser -> scan matching chain used by the firmware's [v] command.
// Build and run:  g++ -O2 -std=c++17 -I../src test_vlidar.cpp -o tv && ./tv
#include <cstdio>
#include <vector>
#include "x4_parser.h"
#include "bench_core.h"
#include "x4_sim.h"

struct Scan { std::vector<X4Point> cur, done; int count = 0; };
static void on_packet(const X4Point* p, int n, bool start, void* u) {
  Scan* s = (Scan*)u;
  if (start) { if (s->cur.size() > 100) { s->done = s->cur; s->count++; } s->cur.clear(); }
  for (int i = 0; i < n; i++) s->cur.push_back(p[i]);
}

int main() {
  using namespace bc;
  std::vector<uint8_t> occ(CELLS), dist(CELLS), lik(CELLS);
  build_room(occ.data()); distance_field(occ.data(), dist.data()); likelihood_field(dist.data(), lik.data());
  Scan sc; X4Parser ps(on_packet, &sc);
  x4sim::Rng rng{12345};
  std::vector<uint8_t> buf(x4sim::REV_BYTES);
  const int N = 30;
  Pose est = {1.5f + 0.04f, 1.5f - 0.03f, 0.02f};
  float px[800], py[800]; int32_t idx[800];
  float max_e = 0, max_eth = 0, max_wall = 0; int matched = 0, fails = 0;
  for (int k = 0; k <= N; k++) {
    Pose truth = {1.5f + 0.03f * k, 1.5f + 0.015f * k, 0.02f * k};
    int nb = x4sim::revolution(occ.data(), truth, buf.data(), rng);
    if (nb != x4sim::REV_BYTES) { printf("FAIL bytes %d\n", nb); fails++; }
    int before = sc.count;
    ps.feed(buf.data(), nb);                       // the zero packet of rev k completes rev k-1
    if (sc.count > before && k > 0) {
      Pose t = {1.5f + 0.03f * (k - 1), 1.5f + 0.015f * (k - 1), 0.02f * (k - 1)};
      int n = x4sim::to_robot_frame(sc.done.data(), (int)sc.done.size(), px, py, 800);
      float wall = x4sim::mean_wall_distance(dist.data(), t, px, py, n);
      MatchResult r = csm_fast(lik.data(), px, py, n, est, idx);
      est = r.best;
      float e = hypotf(est.x - t.x, est.y - t.y), eth = fabsf(est.th - t.th);
      if (e > max_e) max_e = e; if (eth > max_eth) max_eth = eth; if (wall > max_wall) max_wall = wall;
      matched++;
      if (k <= 3 || k == N) printf("rev %2d: %zu pts (%d valid) wall dist %.3f m | est (%.3f %.3f %.3f) truth (%.3f %.3f %.3f) err %.3f m %.3f rad\n",
                                   k - 1, sc.done.size(), n, wall, est.x, est.y, est.th, t.x, t.y, t.th, e, eth);
    }
  }
  printf("revolutions matched %d/%d | packets ok %u bad %u | max wall dist %.3f m | max error %.3f m, %.3f rad\n",
         matched, N, ps.packets_ok, ps.packets_bad_cs, max_wall, max_e, max_eth);
  bool ok = fails == 0 && matched == N && ps.packets_bad_cs == 0 && max_wall < 0.03f && max_e < 0.06f && max_eth < 0.035f;

  // ---- motion distortion: one revolution while the robot drives 0.3 m/s and turns 1.0 rad/s for 1/7 s.
  // Points placed with the end pose: without correction the walls are smeared; per-point deskew with the exact motion
  // must bring them back to about the standing-still value.
  {
    const float dt = 1.0f / 7.0f, v = 0.3f, wz = 1.0f;
    Pose p0 = {2.0f, 2.0f, 0.3f}, p1 = {p0.x + v * dt * cosf(p0.th + wz * dt / 2), p0.y + v * dt * sinf(p0.th + wz * dt / 2), p0.th + wz * dt};
    float wall_still = 0, wall_raw = 0, wall_fix = 0;
    for (int pass = 0; pass < 3; pass++) {
      Scan s2; X4Parser p2(on_packet, &s2);
      int nb = pass == 0 ? x4sim::revolution(occ.data(), p1, buf.data(), rng) : x4sim::revolution_moving(occ.data(), p0, p1, buf.data(), rng);
      p2.feed(buf.data(), nb); nb = x4sim::revolution(occ.data(), p1, buf.data(), rng); p2.feed(buf.data(), nb);   // next zero packet ends it
      Pose d = {cosf(p0.th) * (p1.x - p0.x) + sinf(p0.th) * (p1.y - p0.y), -sinf(p0.th) * (p1.x - p0.x) + cosf(p0.th) * (p1.y - p0.y), p1.th - p0.th};
      int n = pass == 2 ? x4sim::to_robot_frame_deskew(s2.done.data(), (int)s2.done.size(), px, py, 800, d)
                        : x4sim::to_robot_frame(s2.done.data(), (int)s2.done.size(), px, py, 800);
      float wd = x4sim::mean_wall_distance(dist.data(), p1, px, py, n);
      (pass == 0 ? wall_still : pass == 1 ? wall_raw : wall_fix) = wd;
    }
    printf("motion distortion (0.3 m/s, 1.0 rad/s): mean wall distance standing %.4f m | moving, no correction %.4f m | moving, deskew %.4f m\n",
           wall_still, wall_raw, wall_fix);
    bool ok2 = wall_raw > 3 * wall_still && wall_fix < 1.5f * wall_still + 0.002f;
    if (!ok2) printf("FAIL motion distortion check\n");
    ok = ok && ok2;
  }
  printf(ok ? "ALL PASSED\n" : "FAILED\n");
  return ok ? 0 : 1;
}
