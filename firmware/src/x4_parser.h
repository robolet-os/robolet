// YDLIDAR X4 scan-data packet parser. Pure C++ (no Arduino), so it also builds on a PC for testing.
// Packet layout (X4 Development Manual): PH 0x55AA (bytes AA 55) | CT u8 | LSN u8 | FSA u16 | LSA u16 | CS u16 | LSN x S u16
//   distance_mm = S / 4
//   angle_first = (FSA >> 1) / 64 deg, angle_last = (LSA >> 1) / 64 deg, linear in between (clockwise)
//   correction  = 0 if distance == 0 else atan(21.8 * (155.3 - d) / (155.3 * d)) deg
//   CS          = XOR of all 16-bit little-endian words of the packet except CS itself
//   CT bit0 = 1 marks the zero packet (start of a new revolution). If a zero packet is lost (checksum error), a new
//   revolution is also started when the start angle jumps back by more than 180 deg.
#pragma once
#include <stdint.h>
#include <math.h>

struct X4Point { float angle_deg; float dist_mm; };

class X4Parser {
public:
  static const int MAX_LSN = 128;
  // Callback per decoded packet: points, count, start_of_scan flag.
  typedef void (*PacketFn)(const X4Point* pts, int n, bool start_of_scan, void* user);

  X4Parser(PacketFn fn, void* user) : fn_(fn), user_(user) { reset(); }
  void reset() { st_ = 0; pos_ = 0; last_a0_ = -1.0f; }

  uint32_t packets_ok = 0, packets_bad_cs = 0, bytes_skipped = 0, wraps_without_zero = 0;

  void feed(const uint8_t* data, int len) { for (int i = 0; i < len; i++) push(data[i]); }

  void push(uint8_t b) {
    switch (st_) {
      case 0: if (b == 0xAA) { st_ = 1; } else { bytes_skipped++; } return;
      case 1:
        if (b == 0x55) { st_ = 2; pos_ = 0; }
        else if (b == 0xAA) { bytes_skipped++; }
        else { st_ = 0; bytes_skipped += 2; }
        return;
      case 2:  // fixed part after PH: CT LSN FSA(2) LSA(2) CS(2) = 8 bytes
        hdr_[pos_++] = b;
        if (pos_ == 8) {
          lsn_ = hdr_[1];
          if (lsn_ == 0 || lsn_ > MAX_LSN) { st_ = 0; bytes_skipped += 10; return; }
          need_ = lsn_ * 2; pos_ = 0; st_ = 3;
        }
        return;
      case 3:
        body_[pos_++] = b;
        if (pos_ == need_) { finish(); st_ = 0; }
        return;
    }
  }

private:
  static uint16_t le16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

  void finish() {
    uint16_t ct_lsn = le16(&hdr_[0]), fsa = le16(&hdr_[2]), lsa = le16(&hdr_[4]), cs = le16(&hdr_[6]);
    uint16_t x = 0x55AA ^ ct_lsn ^ fsa ^ lsa;
    for (int i = 0; i < lsn_; i++) x ^= le16(&body_[2 * i]);
    if (x != cs) { packets_bad_cs++; return; }
    packets_ok++;
    float a0 = (fsa >> 1) / 64.0f, a1 = (lsa >> 1) / 64.0f;
    float diff = a1 - a0; if (diff < 0) diff += 360.0f;
    for (int i = 0; i < lsn_; i++) {
      float d = le16(&body_[2 * i]) / 4.0f;
      float a = (lsn_ > 1) ? a0 + diff * i / (lsn_ - 1) : a0;
      if (d > 0) a += atanf(21.8f * (155.3f - d) / (155.3f * d)) * 57.29578f;
      while (a >= 360.0f) a -= 360.0f;
      while (a < 0.0f) a += 360.0f;
      pts_[i].angle_deg = a; pts_[i].dist_mm = d;
    }
    bool start = (hdr_[0] & 0x01) != 0;
    if (!start && last_a0_ >= 0.0f && a0 + 180.0f < last_a0_) { start = true; wraps_without_zero++; }
    last_a0_ = a0;
    if (fn_) fn_(pts_, lsn_, start, user_);
  }

  PacketFn fn_; void* user_;
  int st_, pos_, need_, lsn_;
  float last_a0_;
  uint8_t hdr_[8];
  uint8_t body_[MAX_LSN * 2];
  X4Point pts_[MAX_LSN];
};
