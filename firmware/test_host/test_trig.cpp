// robolet PC test: accuracy of the table sine/cosine (src/fast_trig.h) against the C library, and their speed on
// this PC (the chip's speed is measured by the firmware command [c]).
//   g++ -O2 -std=c++17 -Isrc test_host/test_trig.cpp -o /tmp/ttrig && /tmp/ttrig
#include <chrono>
#include <cmath>
#include <cstdio>
#include "fast_trig.h"

static_assert(ftrig::TAB.v[0] == 0.0f && ftrig::TAB.v[ftrig::QN] == 1.0f, "table is built by the compiler");

int main() {
  int fails = 0;
  // 1. table entries against the double-precision library
  double e_tab = 0;
  for (int i = 0; i <= ftrig::QN; i++) e_tab = fmax(e_tab, fabs(ftrig::TAB.v[i] - sin(ftrig::QUARTER_RAD * i / ftrig::QN)));
  printf("table: %d entries, %zu bytes, largest entry error %.2e\n", ftrig::QN + 1, sizeof(ftrig::TAB), e_tab);
  if (e_tab > 6.0e-8) fails++;
  // 2. dense sweep -20..20 rad (negative angles, several turns), step 1e-5 rad
  double e_s = 0, e_c = 0, e_n = 0; float worst = 0;
  for (int i = -2000000; i <= 2000000; i++) {
    float x = i * 1.0e-5f, s, c; fsincos(x, s, c);
    double ds = fabs(s - sin((double)x)), dc = fabs(c - cos((double)x));
    if (ds > e_s) { e_s = ds; worst = x; }
    e_c = fmax(e_c, dc); e_n = fmax(e_n, fabs(sqrt((double)s * s + (double)c * c) - 1.0));
  }
  printf("sweep -20..20 rad: largest error sin %.2e (at %.5f rad), cos %.2e, |(sin,cos)| - 1: %.2e\n", e_s, worst, e_c, e_n);
  if (e_s > 2.0e-6 || e_c > 2.0e-6) fails++;
  // 3. special angles and far angles
  const float PI_F = 3.14159265f;
  struct { float x; const char* name; } sp[] = {{0, "0"}, {PI_F / 2, "pi/2"}, {PI_F, "pi"}, {-PI_F, "-pi"}, {-PI_F / 2, "-pi/2"},
                                                {2 * PI_F, "2pi"}, {1000.0f, "1000"}, {-1000.0f, "-1000"}, {1.0e12f, "1e12"}};
  double e_sp = 0;
  for (auto& p : sp) { float s, c; fsincos(p.x, s, c); e_sp = fmax(e_sp, fmax(fabs(s - sin((double)p.x)), fabs(c - cos((double)p.x)))); }
  printf("special and far angles (0, +-pi/2, +-pi, 2pi, +-1000, 1e12): largest error %.2e\n", e_sp);
  if (e_sp > 1.0e-4) fails++;                               // at 1000 rad a float angle itself is only good to 6e-5
  if (fsin(0.0f) != 0.0f || fcos(0.0f) != 1.0f) { printf("sin(0), cos(0) not exact\n"); fails++; }
  float sn, cn; fsincos(NAN, sn, cn); if (!std::isnan(sn)) { printf("NaN not passed on\n"); fails++; }
  // 4. speed on this PC
  const int N = 20000000; volatile float sink = 0; float acc = 0;
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < N; i++) { float x = (i & 0xFFFF) * 1.0e-4f - 3.2f; acc += sinf(x) + cosf(x); }
  auto t1 = std::chrono::steady_clock::now(); sink = acc; acc = 0;
  for (int i = 0; i < N; i++) { float x = (i & 0xFFFF) * 1.0e-4f - 3.2f, s, c; fsincos(x, s, c); acc += s + c; }
  auto t2 = std::chrono::steady_clock::now(); sink = acc; (void)sink;
  double lib = std::chrono::duration<double, std::nano>(t1 - t0).count() / N, tab = std::chrono::duration<double, std::nano>(t2 - t1).count() / N;
  printf("this PC, one sin + cos pair: library %.1f ns, table %.1f ns\n", lib, tab);
  printf(fails ? "RESULT trig: FAIL (%d)\n" : "RESULT trig: PASS\n", fails);
  return fails ? 1 : 0;
}
