// Conversions between the planar types used by the algorithms (bc::Pose: x, y, heading as float) and ROS 2 messages.
// Position float -> double -> float is exact. Heading -> quaternion (z = sin(th/2), w = cos(th/2), computed in double)
// -> heading (2 atan2(z, w), rounded to float) gives back the same float, so passing a pose through a message does not
// change the result of the algorithms (checked for 1e6 random headings in test_host/test_bus.cpp).
#pragma once
#include <math.h>
#include "bench_core.h"
#include "ros2_msgs.h"
#include "bus.h"

namespace conv {
using bc::Pose;

inline void stamp(builtin_interfaces::msg::Time& t, int64_t ns) {
  t.sec = (int32_t)(ns / 1000000000LL); t.nanosec = (uint32_t)(ns % 1000000000LL);
}
inline void header(std_msgs::msg::Header& h, const char* frame) { stamp(h.stamp, bus::now_ns()); h.frame_id.set(frame); }

inline void to_msg(Pose p, geometry_msgs::msg::Pose& m) {
  m.position.x = p.x; m.position.y = p.y; m.position.z = 0;
  double h = 0.5 * (double)p.th;
  m.orientation.x = 0; m.orientation.y = 0; m.orientation.z = sin(h); m.orientation.w = cos(h);
}
inline Pose from_msg(const geometry_msgs::msg::Pose& m) {
  return {(float)m.position.x, (float)m.position.y, (float)(2.0 * atan2(m.orientation.z, m.orientation.w))};
}
inline void to_msg(Pose p, geometry_msgs::msg::Transform& m) {
  m.translation.x = p.x; m.translation.y = p.y; m.translation.z = 0;
  double h = 0.5 * (double)p.th;
  m.rotation.x = 0; m.rotation.y = 0; m.rotation.z = sin(h); m.rotation.w = cos(h);
}

// robot-frame points (x, y) <-> sensor_msgs/PointCloud2 with float32 fields x, y, z
inline void cloud_layout(sensor_msgs::msg::PointCloud2& c) {
  static const char* N[3] = {"x", "y", "z"};
  c.fields.size = 3;
  for (int i = 0; i < 3; i++) {
    c.fields[i].name.set(N[i]); c.fields[i].offset = 4 * i;
    c.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32; c.fields[i].count = 1;
  }
  c.height = 1; c.is_bigendian = false; c.point_step = 12; c.is_dense = true;
}
inline void to_cloud(const float* px, const float* py, int n, sensor_msgs::msg::PointCloud2& c) {
  cloud_layout(c);
  uint32_t cap = sensor_msgs::msg::PointCloud2().data.capacity / 12;
  if ((uint32_t)n > cap) n = (int)cap;
  float z = 0;
  for (int i = 0; i < n; i++) {
    memcpy(&c.data.data[12 * i], &px[i], 4); memcpy(&c.data.data[12 * i + 4], &py[i], 4); memcpy(&c.data.data[12 * i + 8], &z, 4);
  }
  c.width = n; c.row_step = 12 * n; c.data.size = 12 * n;
}
inline int from_cloud(const sensor_msgs::msg::PointCloud2& c, float* px, float* py, int max_out) {
  int n = (int)c.width; if (n > max_out) n = max_out;
  for (int i = 0; i < n; i++) { memcpy(&px[i], &c.data.data[c.point_step * i], 4); memcpy(&py[i], &c.data.data[c.point_step * i + 4], 4); }
  return n;
}

// robot-frame points -> sensor_msgs/LaserScan with 720 bins of 0.5 deg (nearest point per bin; for viewers and tools)
inline void to_scan(const float* px, const float* py, int n, sensor_msgs::msg::LaserScan& s) {
  const int B = 720; const float inc = 6.2831853f / B;
  s.angle_min = -3.14159265f; s.angle_max = -3.14159265f + inc * (B - 1); s.angle_increment = inc;
  s.time_increment = (1.0f / 7.0f) / B; s.scan_time = 1.0f / 7.0f; s.range_min = 0.12f; s.range_max = 10.0f;
  s.ranges.size = B; s.intensities.size = 0;
  for (int i = 0; i < B; i++) s.ranges[i] = INFINITY;
  for (int k = 0; k < n; k++) {
    int b = (int)floorf((atan2f(py[k], px[k]) + 3.14159265f) / inc + 0.5f); if (b < 0) b += B; if (b >= B) b -= B;
    float r = hypotf(px[k], py[k]); if (r < s.ranges[b]) s.ranges[b] = r;
  }
}

}  // namespace conv
