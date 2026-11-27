// Virtual YDLIDAR X4: produces the same byte stream as the real sensor (packet format of the X4 Development
// Manual) from the simulated room in bench_core.h. Pure C++, also built on a PC by test_host/.
//   lidar frame: X4 angles are clockwise, 0 deg = robot forward. Robot-frame angle (counter-clockwise) = -X4 angle.
//   The X4 reports a "raw" angle and the receiver adds a range-dependent correction
//   corr(d) = atan(21.8 (155.3 - d) / (155.3 d)). So the true beam direction is raw + corr(d); the generator
//   finds d along that direction (fixed-point iteration, 3 steps).
//   Motion distortion: the X4 turns at about 7 Hz, so one revolution takes about 0.14 s. A sample at raw angle a is
//   taken a/360 of a revolution after the 0 deg sample, from the pose the lidar has at that moment
//   (revolution_moving). A receiver that places all points with one pose sees the walls bent or doubled;
//   to_robot_frame_deskew moves every point to the pose at the end of the revolution with the odometry change.
#pragma once
#include <stdint.h>
#include <math.h>
#include "bench_core.h"

namespace x4sim {

static const int SAMPLES_PER_PACKET = 40, PACKETS_PER_REV = 18;          // 720 samples per revolution, 0.5 deg apart
// Simulation switches (PC tests change them; the firmware uses the defaults):
//   distort: the robot moves during the revolution (real lidar behaviour)
//   deskew : 0 = points used as if taken at the end of the revolution (no correction)
//            2 = whole scan moved from the start pose to the end pose (scan stamped at its first sample, as the ROS
//                LaserScan header does, and placed with that pose; no per-point correction)
//            1 = every point moved with its own time (per-point deskew, default)
struct SimOpts { bool distort = true; int deskew = 1; };
inline SimOpts& opts() { static SimOpts o; return o; }

static const int REV_BYTES = (10 + 2) + PACKETS_PER_REV * (10 + 2 * SAMPLES_PER_PACKET);   // 1632 bytes

inline float corr_deg(float d_mm) { return d_mm > 0 ? atanf(21.8f * (155.3f - d_mm) / (155.3f * d_mm)) * 57.29578f : 0.0f; }

// Optional exact geometry: when segments are set (a world whose walls are not aligned with the grid), the lidar hits
// these line segments instead of grid cells, so it sees smooth tilted walls rather than 5 cm steps.
struct Seg { float x0, y0, x1, y1; };
struct Geo { Seg s[64]; int n = 0; };
inline Geo& geo() { static Geo g; return g; }
inline void geo_add(float x0, float y0, float x1, float y1) { Geo& g = geo(); if (g.n < 64) g.s[g.n++] = {x0, y0, x1, y1}; }
// nearest hit along (c, s) from (x, y) between 0.12 m and 10 m, 0 = none
inline float ray_segments(float x, float y, float c, float s) {
  const Geo& g = geo(); float best = 0;
  for (int k = 0; k < g.n; k++) {
    float ex = g.s[k].x1 - g.s[k].x0, ey = g.s[k].y1 - g.s[k].y0, den = c * ey - s * ex;
    if (fabsf(den) < 1e-9f) continue;
    float px = g.s[k].x0 - x, py = g.s[k].y0 - y;
    float t = (px * ey - py * ex) / den, u = (px * s - py * c) / den;
    if (u < 0 || u > 1 || t < 0.12f || t >= 10.0f) continue;
    if (best == 0 || t < best) best = t;
  }
  return best;
}

// Range in m along world direction a from (x, y); 0 = nothing within 0.12 .. 10 m.
inline float ray_one(const uint8_t* occ, float x, float y, float a) {
  using namespace bc;
  float c = cosf(a), s = sinf(a);
  if (geo().n) return ray_segments(x, y, c, s);
  float fx = (x - OX) * INV_RES + 0.12f * INV_RES * c, fy = (y - OY) * INV_RES + 0.12f * INV_RES * s;
  for (int k = 0; 0.12f + k * RES * 0.5f < 10.0f; k++, fx += 0.5f * c, fy += 0.5f * s) {
    if (fx < 0.0f || fy < 0.0f) return 0.0f;
    int i = (int)fx, j = (int)fy;
    if (i >= W || j >= H) return 0.0f;
    if (occ[j * W + i]) return 0.12f + k * RES * 0.5f;
  }
  return 0.0f;
}

struct Rng { uint32_t s; uint32_t next() { s = s * 1664525u + 1013904223u; return s >> 8; } };

// One X4 sample (S = distance_mm * 4) for raw clockwise angle raw_deg, robot at pose p.
inline uint16_t sample(const uint8_t* occ, bc::Pose p, float raw_deg, Rng& rng) {
  float d_mm = 3000.0f;
  for (int it = 0; it < 3; it++) {
    float actual_cw = raw_deg + corr_deg(d_mm);
    float r = ray_one(occ, p.x, p.y, p.th - actual_cw * 0.01745329f);
    if (r <= 0) return 0;
    d_mm = r * 1000.0f;
  }
  if (rng.next() % 100 < 2) return 0;                          // 2 % dropped samples
  d_mm += (float)((int)(rng.next() % 21) - 10);                // +-10 mm noise
  if (d_mm < 120.0f || d_mm > 16000.0f) return 0;
  return (uint16_t)(d_mm * 4.0f + 0.5f);
}

inline int put_packet(uint8_t* b, uint8_t ct, float a0, float a1, const uint16_t* s, int n) {
  uint16_t fsa = (uint16_t)(((uint16_t)lroundf(a0 * 64) << 1) | 1), lsa = (uint16_t)(((uint16_t)lroundf(a1 * 64) << 1) | 1);
  uint16_t cs = 0x55AA ^ (uint16_t)(ct | (n << 8)) ^ fsa ^ lsa;
  for (int i = 0; i < n; i++) cs ^= s[i];
  int k = 0;
  b[k++] = 0xAA; b[k++] = 0x55; b[k++] = ct; b[k++] = (uint8_t)n;
  b[k++] = fsa & 0xFF; b[k++] = fsa >> 8; b[k++] = lsa & 0xFF; b[k++] = lsa >> 8; b[k++] = cs & 0xFF; b[k++] = cs >> 8;
  for (int i = 0; i < n; i++) { b[k++] = s[i] & 0xFF; b[k++] = s[i] >> 8; }
  return k;
}

// Pose at fraction f (0..1) of the way from a to b (linear in x, y and in the wrapped heading change).
inline bc::Pose lerp_pose(bc::Pose a, bc::Pose b, float f) {
  float d = b.th - a.th; while (d > 3.14159265f) d -= 6.2831853f; while (d < -3.14159265f) d += 6.2831853f;
  return {a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f, a.th + d * f};
}

// One revolution while the lidar moves from pose p0 (0 deg sample) to p1 (last sample): zero packet (CT=1, 1 sample
// at 0 deg) + 18 packets of 40 samples. buf needs REV_BYTES. p0 == p1: a revolution taken standing still.
inline int revolution_moving(const uint8_t* occ, bc::Pose p0, bc::Pose p1, uint8_t* buf, Rng& rng) {
  uint16_t s[SAMPLES_PER_PACKET];
  int k = 0;
  s[0] = sample(occ, p0, 0.0f, rng);
  k += put_packet(buf + k, 0x01, 0.0f, 0.0f, s, 1);
  for (int pk = 0; pk < PACKETS_PER_REV; pk++) {
    float a0 = pk * 20.0f;
    for (int i = 0; i < SAMPLES_PER_PACKET; i++) {
      float a = a0 + 0.5f * i;
      s[i] = sample(occ, lerp_pose(p0, p1, a / 360.0f), a, rng);
    }
    k += put_packet(buf + k, 0x00, a0, a0 + 0.5f * (SAMPLES_PER_PACKET - 1), s, SAMPLES_PER_PACKET);
  }
  return k;
}
inline int revolution(const uint8_t* occ, bc::Pose p, uint8_t* buf, Rng& rng) { return revolution_moving(occ, p, p, buf, rng); }

// Decoded X4 points (clockwise degrees, mm) -> robot-frame x, y in m. Returns count.
template <typename PointT>
inline int to_robot_frame(const PointT* pts, int n, float* px, float* py, int max_out) {
  int m = 0;
  for (int i = 0; i < n && m < max_out; i++) {
    if (pts[i].dist_mm <= 0) continue;
    float a = -pts[i].angle_deg * 0.01745329f, r = pts[i].dist_mm * 0.001f;
    px[m] = r * cosf(a); py[m] = r * sinf(a); m++;
  }
  return m;
}

// Like to_robot_frame, but every point is moved to the robot pose at the END of the revolution. d is the odometry
// change during the revolution, in the frame of the pose at its start. A point taken at raw angle a was measured from
// the pose q = d * (a / 360); in the end frame it is (d^-1 * q) * point. With d = 0 this is to_robot_frame.
// per_point = false: every point is treated as taken at the start pose (q = 0).
template <typename PointT>
inline int to_robot_frame_deskew(const PointT* pts, int n, float* px, float* py, int max_out, bc::Pose d, bool per_point = true) {
  int m = 0;
  float c1 = cosf(d.th), s1 = sinf(d.th);
  for (int i = 0; i < n && m < max_out; i++) {
    if (pts[i].dist_mm <= 0) continue;
    float a = -pts[i].angle_deg * 0.01745329f, r = pts[i].dist_mm * 0.001f;
    float f = per_point ? pts[i].angle_deg / 360.0f : 0.0f; if (f < 0) f = 0; if (f > 1) f = 1;
    float qx = d.x * f, qy = d.y * f, qth = d.th * f;                  // lidar pose at the sample, start frame
    float sx = qx + r * cosf(qth + a), sy = qy + r * sinf(qth + a);    // the point in the start frame
    float ex = sx - d.x, ey = sy - d.y;                                // into the end frame
    px[m] = c1 * ex + s1 * ey; py[m] = -s1 * ex + c1 * ey; m++;
  }
  return m;
}

// Mean distance (m) from the points, placed with the true pose, to the nearest mapped wall (uses dist field).
inline float mean_wall_distance(const uint8_t* dist, bc::Pose p, const float* px, const float* py, int n) {
  using namespace bc;
  float c = cosf(p.th), s = sinf(p.th), sum = 0; int m = 0;
  for (int k = 0; k < n; k++) {
    int i = cx(p.x + c * px[k] - s * py[k]), j = cy(p.y + s * px[k] + c * py[k]);
    if (i < 0 || i >= W || j < 0 || j >= H) continue;
    sum += dist[j * W + i] / 10.0f * RES; m++;
  }
  return m ? sum / m : 1e9f;
}

}  // namespace x4sim
