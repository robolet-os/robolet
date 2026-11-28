// robolet: sine and cosine from a table that the compiler builds (constexpr) and the linker puts in flash (.rodata).
// Table: sin over a quarter wave, 513 floats (2052 bytes); values between two entries by linear interpolation.
// Largest error against sinf/cosf: below 2e-6 (test_host/test_trig.cpp measures it). No table is filled at start-up
// and no RAM is used.
// The idea of a constexpr quarter-wave table in flash was suggested by u/Plastic_Fig9225 (Reddit); this file is an
// independent implementation (float entries, interpolation, C++11 constexpr).
#pragma once
#include <math.h>
#include <stdint.h>

namespace ftrig {
constexpr int QN = 512;                                    // table steps in a quarter wave (a power of two)
constexpr double QUARTER_RAD = 1.57079632679489661923;

// sin(x) for 0 <= x <= pi/2 at compile time: Taylor series up to x^21 (the next term is below 3e-16)
constexpr double poly(double x2) {
  return 1 - x2 / 6 * (1 - x2 / 20 * (1 - x2 / 42 * (1 - x2 / 72 * (1 - x2 / 110 * (1 - x2 / 156 * (1 - x2 / 210 *
         (1 - x2 / 272 * (1 - x2 / 342 * (1 - x2 / 420)))))))));
}
constexpr double sin_q(double x) { return x * poly(x * x); }
constexpr float entry(int i) { return i == QN ? 1.0f : (float)sin_q(QUARTER_RAD * i / QN); }

template <int... I> struct Seq {};
template <int N, int... I> struct MakeSeq : MakeSeq<N - 1, N - 1, I...> {};
template <int... I> struct MakeSeq<0, I...> { typedef Seq<I...> type; };

struct Table { float v[QN + 1]; };
template <int... I> constexpr Table make(Seq<I...>) { return Table{{entry(I)...}}; }
static constexpr Table TAB = make(MakeSeq<QN + 1>::type());

inline float quarter(uint32_t k, float f) {                // k: table steps from 0 (any number of turns), f: 0..1
  uint32_t i = k & (QN - 1), q = (k / QN) & 3;
  const float* t = TAB.v;
  float v = (q & 1) ? t[QN - i] + f * (t[QN - i - 1] - t[QN - i]) : t[i] + f * (t[i + 1] - t[i]);
  return (q & 2) ? -v : v;
}
}  // namespace ftrig

// sin and cos of x (rad) together: one index computation for both
inline void fsincos(float x, float& s, float& c) {
  float t = x * (float)(ftrig::QN / ftrig::QUARTER_RAD);
  if (!(t > -1.0e9f && t < 1.0e9f)) { s = sinf(x); c = cosf(x); return; }   // huge angle or NaN: the library
  int32_t k = (int32_t)t; if ((float)k > t) k--;                             // floor
  float f = t - (float)k;
  s = ftrig::quarter((uint32_t)k, f); c = ftrig::quarter((uint32_t)k + ftrig::QN, f);
}
inline float fsin(float x) { float s, c; fsincos(x, s, c); return s; }
inline float fcos(float x) { float s, c; fsincos(x, s, c); return c; }
