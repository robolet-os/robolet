// Compute kernels of the simulator stack (navsim.py) written in C++ so their cost can be measured on the chip.
// Pure C++ (no Arduino): the same file is built on a PC by test_host/ to check the results are correct.
// Map: 8 x 6 m room with a 1 m margin, 5 cm cells -> 200 x 160 cells (same resolution as the simulator).
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "fast_trig.h"

namespace bc {

static const float RES = 0.05f;          // m per cell
static const int   W = 200, H = 160;     // 10 x 8 m
static const float OX = -1.0f, OY = -1.0f;  // world coordinate of cell (0,0)
static const int   CELLS = W * H;

inline int cx(float x) { return (int)floorf((x - OX) / RES); }
inline int cy(float y) { return (int)floorf((y - OY) / RES); }

// ---- map -------------------------------------------------------------------------------------
inline void fill_rect(uint8_t* occ, float x0, float y0, float x1, float y1) {
  for (int j = cy(y0); j <= cy(y1); j++)
    for (int i = cx(x0); i <= cx(x1); i++)
      if (i >= 0 && i < W && j >= 0 && j < H) occ[j * W + i] = 1;
}

// Room 8 x 6 m (walls 5 cm thick), one inner wall and two boxes. occ: 1 = occupied.
inline void build_room(uint8_t* occ) {
  memset(occ, 0, CELLS);
  fill_rect(occ, 0.0f, 0.0f, 8.0f, 0.0f); fill_rect(occ, 0.0f, 6.0f, 8.0f, 6.0f);
  fill_rect(occ, 0.0f, 0.0f, 0.0f, 6.0f); fill_rect(occ, 8.0f, 0.0f, 8.0f, 6.0f);
  fill_rect(occ, 3.0f, 0.0f, 3.0f, 3.5f);                    // inner wall
  fill_rect(occ, 5.2f, 3.8f, 5.8f, 4.4f);                    // box 1
  fill_rect(occ, 1.0f, 4.5f, 1.5f, 5.0f);                    // box 2
}

// Two-pass chamfer distance (in cells x 10, capped at 255) from occupied cells. dist: uint8 per cell.
inline void distance_field(const uint8_t* occ, uint8_t* dist) {
  const int A = 10, D = 14, CAP = 255;
  for (int k = 0; k < CELLS; k++) dist[k] = occ[k] ? 0 : CAP;
  for (int j = 0; j < H; j++)
    for (int i = 0; i < W; i++) {
      int k = j * W + i, v = dist[k];
      if (i > 0) { int t = dist[k - 1] + A; if (t < v) v = t; }
      if (j > 0) {
        int t = dist[k - W] + A; if (t < v) v = t;
        if (i > 0)     { t = dist[k - W - 1] + D; if (t < v) v = t; }
        if (i < W - 1) { t = dist[k - W + 1] + D; if (t < v) v = t; }
      }
      dist[k] = (uint8_t)(v > CAP ? CAP : v);
    }
  for (int j = H - 1; j >= 0; j--)
    for (int i = W - 1; i >= 0; i--) {
      int k = j * W + i, v = dist[k];
      if (i < W - 1) { int t = dist[k + 1] + A; if (t < v) v = t; }
      if (j < H - 1) {
        int t = dist[k + W] + A; if (t < v) v = t;
        if (i < W - 1) { t = dist[k + W + 1] + D; if (t < v) v = t; }
        if (i > 0)     { t = dist[k + W - 1] + D; if (t < v) v = t; }
      }
      dist[k] = (uint8_t)(v > CAP ? CAP : v);
    }
}

// Likelihood field (0..255) = 255 * exp(-d^2 / (2 sigma^2)), sigma = 0.1 m (like AMCL z_hit with a Gaussian).
inline void likelihood_field(const uint8_t* dist, uint8_t* lik) {
  uint8_t tab[256];
  for (int d = 0; d < 256; d++) { float m = d / 10.0f * RES; tab[d] = (uint8_t)(255.0f * expf(-m * m / (2 * 0.1f * 0.1f)) + 0.5f); }
  for (int k = 0; k < CELLS; k++) lik[k] = tab[dist[k]];
}

// Nav2-style inflation cost: 254 lethal inside robot radius 0.18 m, 253*exp(-6 (d-0.18)) up to 0.55 m, else 0.
inline void costmap(const uint8_t* dist, uint8_t* cost) {
  uint8_t tab[256];
  for (int d = 0; d < 256; d++) {
    float m = d / 10.0f * RES;
    tab[d] = m < 0.18f ? 254 : (m < 0.55f ? (uint8_t)(253.0f * expf(-6.0f * (m - 0.18f))) : 0);
  }
  for (int k = 0; k < CELLS; k++) cost[k] = tab[dist[k]];
}

// ---- lidar model -------------------------------------------------------------------------------
// Ray cast n beams (360 deg) from pose, step half a cell, max range 10 m. ranges in m (0 = no return).
inline int raycast(const uint8_t* occ, float x, float y, float th, int n, float* ranges) {
  int lookups = 0;
  for (int b = 0; b < n; b++) {
    float a = th + 6.2831853f * b / n, c = cosf(a), s = sinf(a), r = 0.0f;
    ranges[b] = 0.0f;
    for (r = 0.12f; r < 10.0f; r += RES * 0.5f) {
      int i = cx(x + r * c), j = cy(y + r * s); lookups++;
      if (i < 0 || i >= W || j < 0 || j >= H) break;
      if (occ[j * W + i]) { ranges[b] = r; break; }
    }
  }
  return lookups;
}

// Scan in robot frame (x, y in m) from ranges; returns point count.
inline int scan_points(const float* ranges, int n, float* px, float* py) {
  int m = 0;
  for (int b = 0; b < n; b++) if (ranges[b] > 0) {
    float a = 6.2831853f * b / n; px[m] = ranges[b] * cosf(a); py[m] = ranges[b] * sinf(a); m++;
  }
  return m;
}

// ---- scan matching (correlative, brute force) ----------------------------------------------------
struct Pose { float x, y, th; };
struct MatchResult { Pose best; uint32_t score; uint32_t lookups; };

// Naive: every candidate pose transforms every point (what tools/chip_budget.py counted).
inline MatchResult csm_naive(const uint8_t* lik, const float* px, const float* py, int n, Pose g,
                             int nxy = 5, int nth = 5, float dxy = 0.05f, float dth = 0.03f) {
  MatchResult r = {g, 0, 0};
  for (int t = 0; t < nth; t++) {
    float th = g.th + (t - nth / 2) * dth;
    for (int u = 0; u < nxy; u++) for (int v = 0; v < nxy; v++) {
      float x = g.x + (u - nxy / 2) * dxy, y = g.y + (v - nxy / 2) * dxy;
      float c = cosf(th), s = sinf(th); uint32_t sc = 0;
      for (int k = 0; k < n; k++) {
        int i = cx(x + c * px[k] - s * py[k]), j = cy(y + s * px[k] + c * py[k]);
        if (i >= 0 && i < W && j >= 0 && j < H) sc += lik[j * W + i];
      }
      r.lookups += n;
      if (sc > r.score) { r.score = sc; r.best = {x, y, th}; }
    }
  }
  return r;
}

// Faster: rotate the scan once per heading into cell offsets, then translations are integer adds.
// idx: scratch int32[n].  Same search window as csm_naive (dxy must be a multiple of RES).
inline MatchResult csm_fast(const uint8_t* lik, const float* px, const float* py, int n, Pose g, int32_t* idx,
                            int nxy = 5, int nth = 5, float dxy = 0.05f, float dth = 0.03f) {
  MatchResult r = {g, 0, 0};
  int step = (int)lroundf(dxy / RES), half = nxy / 2, gi = cx(g.x), gj = cy(g.y);
  for (int t = 0; t < nth; t++) {
    float th = g.th + (t - nth / 2) * dth, c = fcos(th), s = fsin(th);
    int m = 0;
    for (int k = 0; k < n; k++) {
      int i = cx(g.x + c * px[k] - s * py[k]) - gi, j = cy(g.y + s * px[k] + c * py[k]) - gj;
      idx[m++] = j * W + i;
    }
    for (int u = 0; u < nxy; u++) for (int v = 0; v < nxy; v++) {
      int oi = gi + (u - half) * step, oj = gj + (v - half) * step;
      // Points come from inside the mapped room and the 1 m margin is wider than the search window,
      // so a cell offset cannot wrap to another row; q is still range-checked.
      int base = oj * W + oi; uint32_t sc = 0;
      for (int k = 0; k < m; k++) { int q = base + idx[k]; if ((unsigned)q < (unsigned)CELLS) sc += lik[q]; }
      r.lookups += m;
      if (sc > r.score) {
        r.score = sc;
        r.best = {g.x + (u - half) * step * RES, g.y + (v - half) * step * RES, th};
      }
    }
  }
  return r;
}

// ---- AMCL-style particle weighting ----------------------------------------------------------------
// np particles around pose, nb beams each (every (n/nb)-th scan point). Returns lookup count; w: weights.
inline uint32_t amcl_weights(const uint8_t* lik, const float* px, const float* py, int n, const Pose* parts, int np,
                             int nb, float* w) {
  int stride = n / nb; if (stride < 1) stride = 1;
  uint32_t lookups = 0;
  for (int p = 0; p < np; p++) {
    float c = fcos(parts[p].th), s = fsin(parts[p].th), lw = 0.0f;
    for (int k = 0; k < n; k += stride) {
      int i = cx(parts[p].x + c * px[k] - s * py[k]), j = cy(parts[p].y + s * px[k] + c * py[k]);
      float z = (i >= 0 && i < W && j >= 0 && j < H) ? lik[j * W + i] / 255.0f : 0.0f;
      lw += logf(0.95f * z + 0.05f);                     // z_hit 0.95, z_rand 0.05
      lookups++;
    }
    w[p] = lw;
  }
  return lookups;
}

// ---- A* on the inflated costmap (8-connected, indexed binary heap, uint16 indices since CELLS < 65536) --
struct AStarWork { float* g; uint16_t* heap; uint16_t* pos; uint8_t* state; };  // state 0 new, 1 open, 2 closed
struct AStarResult { bool found; float length_m; int expanded; };

inline AStarResult astar(const uint8_t* cost, int s, int goal, AStarWork wk) {
  float* g = wk.g; uint16_t* heap = wk.heap; uint16_t* pos = wk.pos; uint8_t* st = wk.state;
  memset(st, 0, CELLS);
  int hn = 0; int gx = goal % W, gy = goal / W;
  auto hcost = [&](int k) { float dx = (float)(k % W - gx), dy = (float)(k / W - gy); return sqrtf(dx * dx + dy * dy); };
  auto f = [&](int k) { return g[k] + hcost(k); };
  auto up = [&](int i) { while (i > 0) { int p = (i - 1) / 2; if (f(heap[p]) <= f(heap[i])) break;
      uint16_t t = heap[p]; heap[p] = heap[i]; heap[i] = t; pos[heap[p]] = p; pos[heap[i]] = i; i = p; } };
  auto down = [&](int i) { for (;;) { int l = 2 * i + 1, r = l + 1, m = i;
      if (l < hn && f(heap[l]) < f(heap[m])) m = l;
      if (r < hn && f(heap[r]) < f(heap[m])) m = r;
      if (m == i) break;
      uint16_t t = heap[m]; heap[m] = heap[i]; heap[i] = t; pos[heap[m]] = m; pos[heap[i]] = i; i = m; } };
  g[s] = 0; st[s] = 1; heap[0] = s; pos[s] = 0; hn = 1;
  static const int DI[8] = {1, -1, 0, 0, 1, 1, -1, -1}, DJ[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static const float DC[8] = {1, 1, 1, 1, 1.41421f, 1.41421f, 1.41421f, 1.41421f};
  int expanded = 0;
  while (hn > 0) {
    int k = heap[0]; heap[0] = heap[--hn]; pos[heap[0]] = 0; down(0);
    st[k] = 2; expanded++;
    if (k == goal) return {true, g[k] * RES, expanded};
    int i = k % W, j = k / W;
    for (int d = 0; d < 8; d++) {
      int ni = i + DI[d], nj = j + DJ[d]; if (ni < 0 || ni >= W || nj < 0 || nj >= H) continue;
      int q = nj * W + ni; if (cost[q] >= 254 || st[q] == 2) continue;
      float ng = g[k] + DC[d] * (1.0f + 3.0f * cost[q] / 253.0f);   // cost weighting like the simulator
      if (st[q] == 0) { g[q] = ng; st[q] = 1; heap[hn] = q; pos[q] = hn; up(hn++); }
      else if (ng < g[q]) { g[q] = ng; up(pos[q]); }
    }
  }
  return {false, 0.0f, expanded};
}

// =============================== optimized versions (v2) ===============================================
// First chip run (2026-11-25): AMCL ~570 cycles and ray cast ~290 cycles per lookup, against ~30 for csm_fast.
// The versions below remove the per-point float division and floorf call, replace logf by a 256-entry table
// and store the A* priority instead of recomputing sqrt in every heap comparison. Results must stay the same.
static const float INV_RES = 1.0f / RES;   // multiply instead of divide

// Ray cast with incremental cell coordinates (no division or floorf inside the loop).
inline int raycast_opt(const uint8_t* occ, float x, float y, float th, int n, float* ranges) {
  int lookups = 0;
  const float fx0 = (x - OX) * INV_RES, fy0 = (y - OY) * INV_RES;
  for (int b = 0; b < n; b++) {
    float a = th + 6.2831853f * b / n, c = cosf(a), s = sinf(a);
    float fx = fx0 + 0.12f * INV_RES * c, fy = fy0 + 0.12f * INV_RES * s, dx = 0.5f * c, dy = 0.5f * s;
    ranges[b] = 0.0f;
    for (int k = 0; 0.12f + k * RES * 0.5f < 10.0f; k++, fx += dx, fy += dy) {
      lookups++;
      if (fx < 0.0f || fy < 0.0f) break;
      int i = (int)fx, j = (int)fy;              // truncation == floor for non-negative values
      if (i >= W || j >= H) break;
      if (occ[j * W + i]) { ranges[b] = 0.12f + k * RES * 0.5f; break; }
    }
  }
  return lookups;
}

struct AmclTable { float logw[256]; };   // log(0.95 * lik/255 + 0.05) for every likelihood byte
inline void amcl_table(AmclTable& t) { for (int v = 0; v < 256; v++) t.logw[v] = logf(0.95f * v / 255.0f + 0.05f); }

// Same weights as amcl_weights: beams pre-scaled to cell units once, table lookup instead of logf.
inline uint32_t amcl_weights_opt(const uint8_t* lik, const float* px, const float* py, int n, const Pose* parts, int np,
                                 int nb, const AmclTable& t, float* w) {
  int stride = n / nb; if (stride < 1) stride = 1;
  float sx[256], sy[256]; int m = 0;
  for (int k = 0; k < n && m < 256; k += stride) { sx[m] = px[k] * INV_RES; sy[m] = py[k] * INV_RES; m++; }
  uint32_t lookups = 0;
  for (int p = 0; p < np; p++) {
    float c = fcos(parts[p].th), s = fsin(parts[p].th), lw = 0.0f;
    float fx0 = (parts[p].x - OX) * INV_RES, fy0 = (parts[p].y - OY) * INV_RES;
    for (int k = 0; k < m; k++) {
      float fx = fx0 + c * sx[k] - s * sy[k], fy = fy0 + s * sx[k] + c * sy[k];
      int i = (int)fx, j = (int)fy;
      bool in = fx >= 0.0f && fy >= 0.0f && i < W && j < H;
      lw += t.logw[in ? lik[j * W + i] : 0];
    }
    lookups += m;
    w[p] = lw;
  }
  return lookups;
}

// A* with the priority f = g + h stored per cell (h computed once when a cell is first reached).
// fbuf: float[CELLS] extra work array.
inline AStarResult astar_opt(const uint8_t* cost, int s, int goal, AStarWork wk, float* fbuf, float cost_weight = 3.0f) {
  float* g = wk.g; float* f = fbuf; uint16_t* heap = wk.heap; uint16_t* pos = wk.pos; uint8_t* st = wk.state;
  memset(st, 0, CELLS);
  int hn = 0; const int gx = goal % W, gy = goal / W;
  const float CW = cost_weight / 253.0f;
  auto up = [&](int i) { uint16_t v = heap[i]; float fv = f[v];
      while (i > 0) { int p = (i - 1) / 2; if (f[heap[p]] <= fv) break; heap[i] = heap[p]; pos[heap[i]] = i; i = p; }
      heap[i] = v; pos[v] = i; };
  auto down = [&](int i) { uint16_t v = heap[i]; float fv = f[v];
      for (;;) { int l = 2 * i + 1, r = l + 1, m = -1; float fm = fv;
        if (l < hn && f[heap[l]] < fm) { m = l; fm = f[heap[l]]; }
        if (r < hn && f[heap[r]] < fm) { m = r; }
        if (m < 0) break;
        heap[i] = heap[m]; pos[heap[i]] = i; i = m; }
      heap[i] = v; pos[v] = i; };
  { float dx = (float)(s % W - gx), dy = (float)(s / W - gy); g[s] = 0; f[s] = sqrtf(dx * dx + dy * dy); }
  st[s] = 1; heap[0] = s; pos[s] = 0; hn = 1;
  static const int DI[8] = {1, -1, 0, 0, 1, 1, -1, -1}, DJ[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static const float DC[8] = {1, 1, 1, 1, 1.41421f, 1.41421f, 1.41421f, 1.41421f};
  int expanded = 0;
  while (hn > 0) {
    int k = heap[0]; heap[0] = heap[--hn]; if (hn > 0) { pos[heap[0]] = 0; down(0); }
    st[k] = 2; expanded++;
    if (k == goal) return {true, g[k] * RES, expanded};
    int i = k % W, j = k / W;
    for (int d = 0; d < 8; d++) {
      int ni = i + DI[d], nj = j + DJ[d]; if (ni < 0 || ni >= W || nj < 0 || nj >= H) continue;
      int q = nj * W + ni; if (cost[q] >= 254 || st[q] == 2) continue;
      float ng = g[k] + DC[d] * (1.0f + CW * cost[q]);
      if (st[q] == 0) {
        float dx = (float)(ni - gx), dy = (float)(nj - gy);
        g[q] = ng; f[q] = ng + sqrtf(dx * dx + dy * dy); st[q] = 1; heap[hn] = q; pos[q] = hn; up(hn++);
      } else if (ng < g[q]) { f[q] += ng - g[q]; g[q] = ng; up(pos[q]); }
    }
  }
  return {false, 0.0f, expanded};
}

}  // namespace bc
