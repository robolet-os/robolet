// PC test of the parts of the firmware that do not need the chip.
// Build and run:  g++ -O2 -std=c++17 -I../src test_main.cpp -o t && ./t
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <chrono>
#include "x4_parser.h"
#include "bench_core.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

// ---- X4 encoder (inverse of the parser) ----
static void put16(std::vector<uint8_t>& v, uint16_t x) { v.push_back(x & 0xFF); v.push_back(x >> 8); }
static std::vector<uint8_t> make_packet(uint8_t ct, float a0, float a1, const std::vector<uint16_t>& s, bool corrupt = false) {
  uint16_t fsa = ((uint16_t)lroundf(a0 * 64) << 1) | 1, lsa = ((uint16_t)lroundf(a1 * 64) << 1) | 1;
  uint8_t lsn = (uint8_t)s.size();
  uint16_t cs = 0x55AA ^ (uint16_t)(ct | (lsn << 8)) ^ fsa ^ lsa;
  for (auto x : s) cs ^= x;
  if (corrupt) cs ^= 0x0100;
  std::vector<uint8_t> v = {0xAA, 0x55, ct, lsn};
  put16(v, fsa); put16(v, lsa); put16(v, cs);
  for (auto x : s) put16(v, x);
  return v;
}

struct Got { std::vector<X4Point> pts; int scans = 0; };
static void on_packet(const X4Point* p, int n, bool start, void* u) {
  Got* g = (Got*)u; if (start) g->scans++;
  for (int i = 0; i < n; i++) g->pts.push_back(p[i]);
}

static void test_parser() {
  std::vector<uint8_t> stream = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};   // start-scan reply header (ignored)
  std::vector<float> want_a, want_d;
  srand(1);
  for (int rev = 0; rev < 3; rev++) {
    auto z = make_packet(0x01, 0.0f, 0.0f, {(uint16_t)(1000 * 4)});            // zero packet, 1 m
    stream.insert(stream.end(), z.begin(), z.end());
    if (rev == 0) { want_a.push_back(0); want_d.push_back(1000); }
    for (int k = 0; k < 18; k++) {                                              // 18 packets x 40 samples = 720 points
      float a0 = k * 20.0f, a1 = a0 + 19.5f;
      std::vector<uint16_t> s;
      for (int i = 0; i < 40; i++) {
        float d = (i % 7 == 0) ? 0.0f : 500.0f + 100.0f * ((k * 40 + i) % 50);
        s.push_back((uint16_t)(d * 4));
        if (rev == 0) { want_a.push_back(a0 + (a1 - a0) * i / 39.0f); want_d.push_back(d); }
      }
      bool bad = (rev == 1 && k == 5);
      auto p = make_packet(0x00, a0, a1, s, bad);
      stream.insert(stream.end(), p.begin(), p.end());
      if (k == 9) { for (int j = 0; j < 5; j++) stream.push_back(rand() & 0xFF); }  // junk between packets
    }
  }
  Got g; X4Parser ps(on_packet, &g);
  size_t i = 0;                                       // feed in random chunk sizes like a UART would
  while (i < stream.size()) { size_t n = 1 + rand() % 64; if (i + n > stream.size()) n = stream.size() - i; ps.feed(&stream[i], (int)n); i += n; }
  CHECK(ps.packets_ok == 3 * 19 - 1, "packets_ok=%u", ps.packets_ok);
  CHECK(ps.packets_bad_cs == 1, "bad_cs=%u", ps.packets_bad_cs);
  CHECK(g.scans == 3, "scans=%d", g.scans);
  int n = (int)want_a.size(); float max_da = 0, max_dd = 0;
  for (int k = 0; k < n; k++) {
    float d = want_d[k], a = want_a[k];
    if (d > 0) a += atanf(21.8f * (155.3f - d) / (155.3f * d)) * 57.29578f;
    while (a < 0) a += 360; while (a >= 360) a -= 360;
    float da = fabsf(g.pts[k].angle_deg - a); if (da > 180) da = 360 - da;
    if (da > max_da) max_da = da;
    float dd = fabsf(g.pts[k].dist_mm - d); if (dd > max_dd) max_dd = dd;
  }
  CHECK(max_da < 0.02f, "max angle error %.4f deg", max_da);
  CHECK(max_dd < 0.3f, "max dist error %.3f mm", max_dd);
  printf("parser: ok=%u bad_cs=%u skipped=%u scans=%d points=%zu max_angle_err=%.4f deg max_dist_err=%.2f mm\n",
         ps.packets_ok, ps.packets_bad_cs, ps.bytes_skipped, g.scans, g.pts.size(), max_da, max_dd);
}

// A lost zero packet (checksum error) must still split the revolutions (angle wrap rule)
static void test_lost_zero() {
  std::vector<uint8_t> stream;
  for (int rev = 0; rev < 3; rev++) {
    auto z = make_packet(0x01, 0.0f, 0.0f, {(uint16_t)(1000 * 4)}, rev == 1);   // zero packet of revolution 1 is corrupted
    stream.insert(stream.end(), z.begin(), z.end());
    for (int k = 0; k < 18; k++) {
      std::vector<uint16_t> s(40, (uint16_t)(2000 * 4));
      auto p = make_packet(0x00, k * 20.0f, k * 20.0f + 19.5f, s);
      stream.insert(stream.end(), p.begin(), p.end());
    }
  }
  Got g; X4Parser ps(on_packet, &g);
  ps.feed(stream.data(), (int)stream.size());
  CHECK(g.scans == 3 && ps.wraps_without_zero == 1 && ps.packets_bad_cs == 1, "lost zero: scans %d wraps %u bad %u", g.scans, ps.wraps_without_zero, ps.packets_bad_cs);
  printf("lost zero packet: revolutions started %d (expected 3), by angle wrap %u, checksum errors %u\n", g.scans, ps.wraps_without_zero, ps.packets_bad_cs);
}

static double ms_since(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

static void test_bench() {
  using namespace bc;
  std::vector<uint8_t> occ(CELLS), dist(CELLS), lik(CELLS), cost(CELLS);
  build_room(occ.data()); distance_field(occ.data(), dist.data());
  likelihood_field(dist.data(), lik.data()); costmap(dist.data(), cost.data());
  Pose truth = {2.0f, 2.0f, 0.3f};
  std::vector<float> rg(720), px(720), py(720);
  raycast(occ.data(), truth.x, truth.y, truth.th, 720, rg.data());
  int n = scan_points(rg.data(), 720, px.data(), py.data());
  CHECK(n > 700, "scan points %d", n);
  Pose guess = {2.05f, 1.95f, 0.33f};
  auto t0 = std::chrono::steady_clock::now();
  MatchResult a = csm_naive(lik.data(), px.data(), py.data(), n, guess);
  double ta = ms_since(t0);
  std::vector<int32_t> idx(720);
  t0 = std::chrono::steady_clock::now();
  MatchResult b = csm_fast(lik.data(), px.data(), py.data(), n, guess, idx.data());
  double tb = ms_since(t0);
  CHECK(fabsf(a.best.x - truth.x) < 0.03f && fabsf(a.best.y - truth.y) < 0.03f && fabsf(a.best.th - truth.th) < 0.016f,
        "naive best %.3f %.3f %.3f", a.best.x, a.best.y, a.best.th);
  CHECK(fabsf(b.best.x - truth.x) < 0.03f && fabsf(b.best.y - truth.y) < 0.03f && fabsf(b.best.th - truth.th) < 0.016f,
        "fast best %.3f %.3f %.3f", b.best.x, b.best.y, b.best.th);
  printf("csm: naive best (%.3f, %.3f, %.3f) %u lookups %.2f ms | fast best (%.3f, %.3f, %.3f) %u lookups %.2f ms\n",
         a.best.x, a.best.y, a.best.th, a.lookups, ta, b.best.x, b.best.y, b.best.th, b.lookups, tb);
  std::vector<Pose> parts(600); std::vector<float> w(600);
  srand(2);
  for (int p = 0; p < 600; p++) parts[p] = {truth.x + ((rand() % 41) - 20) * 0.01f, truth.y + ((rand() % 41) - 20) * 0.01f, truth.th + ((rand() % 21) - 10) * 0.01f};
  parts[123] = truth;
  amcl_weights(lik.data(), px.data(), py.data(), n, parts.data(), 600, 60, w.data());
  int best = 0; for (int p = 1; p < 600; p++) if (w[p] > w[best]) best = p;
  float e = hypotf(parts[best].x - truth.x, parts[best].y - truth.y);
  CHECK(e < 0.06f, "amcl best particle %.3f m from truth", e);
  printf("amcl: best particle %d, %.3f m from truth\n", best, e);
  std::vector<float> g(CELLS); std::vector<uint16_t> heap(CELLS), pos(CELLS); std::vector<uint8_t> st(CELLS);
  t0 = std::chrono::steady_clock::now();
  AStarResult r = astar(cost.data(), cy(1.0f) * W + cx(1.0f), cy(5.0f) * W + cx(7.0f), {g.data(), heap.data(), pos.data(), st.data()});
  double tr = ms_since(t0);
  CHECK(r.found && r.length_m > 7.2f && r.length_m < 14.0f, "astar found=%d len=%.2f", r.found, r.length_m);
  printf("astar: found=%d weighted length %.2f m, expanded %d cells, %.2f ms\n", r.found, r.length_m, r.expanded, tr);

  // ---- optimized versions (v2) must give the same results ----
  std::vector<float> rg2(720);
  raycast_opt(occ.data(), truth.x, truth.y, truth.th, 720, rg2.data());
  int diff_beams = 0; float max_dr = 0;
  for (int b = 0; b < 720; b++) { float d = fabsf(rg[b] - rg2[b]); if (d > 1e-4f) diff_beams++; if (d > max_dr) max_dr = d; }
  CHECK(diff_beams <= 7 && max_dr <= 0.051f, "raycast_opt differs on %d beams, max %.3f m", diff_beams, max_dr);
  printf("raycast_opt: %d of 720 beams differ, max difference %.3f m\n", diff_beams, max_dr);
  AmclTable tab; amcl_table(tab); std::vector<float> w2(600);
  amcl_weights_opt(lik.data(), px.data(), py.data(), n, parts.data(), 600, 60, tab, w2.data());
  float max_dw = 0; int best2 = 0;
  for (int p = 0; p < 600; p++) { float d = fabsf(w[p] - w2[p]); if (d > max_dw) max_dw = d; if (w2[p] > w2[best2]) best2 = p; }
  CHECK(best2 == best && max_dw < 0.05f, "amcl_opt best %d vs %d, max weight diff %.4f", best2, best, max_dw);
  printf("amcl_opt: best particle %d (same as before: %s), max log-weight difference %.5f\n", best2, best2 == best ? "yes" : "NO", max_dw);
  std::vector<float> fb(CELLS);
  t0 = std::chrono::steady_clock::now();
  AStarResult r2 = astar_opt(cost.data(), cy(1.0f) * W + cx(1.0f), cy(5.0f) * W + cx(7.0f), {g.data(), heap.data(), pos.data(), st.data()}, fb.data());
  double tr2 = ms_since(t0);
  CHECK(r2.found && fabsf(r2.length_m - r.length_m) < 1e-3f, "astar_opt len %.3f vs %.3f", r2.length_m, r.length_m);
  printf("astar_opt: found=%d weighted length %.2f m, expanded %d cells, %.2f ms\n", r2.found, r2.length_m, r2.expanded, tr2);
}

int main() {
  test_parser();
  test_lost_zero();
  test_bench();
  printf(fails ? "\n%d FAILED\n" : "\nALL PASSED\n", fails);
  return fails ? 1 : 0;
}
