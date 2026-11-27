// CDR serialization as used by ROS 2 (rmw_fastrtps / rmw_cyclonedds: plain CDR, little endian, XCDR version 1):
//   4-byte encapsulation header 00 01 00 00, then the fields in .msg order; every number is aligned to its own size
//   (at most 8) counted from the end of the header; string = uint32 length including the final 0, the characters, 0;
//   T[] = uint32 count, then the elements; T[N] = the N elements.
// The generated cdr_write / cdr_read functions in ros2_msgs.h use this; test_host/test_cdr.cpp + tools/check_cdr.py
// check the bytes against an independent ROS 2 implementation (rosbags).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace cdr {

struct Writer {
  uint8_t* b; size_t cap; size_t pos = 0; bool ok = true;
  Writer(uint8_t* buf, size_t n) : b(buf), cap(n) {}
  void header() { const uint8_t h[4] = {0, 1, 0, 0}; raw(h, 4); }
  void align(size_t a) { size_t o = (pos - 4) % a; if (o) pad(a - o); }
  void pad(size_t n) { if (pos + n > cap) { ok = false; return; } memset(b + pos, 0, n); pos += n; }
  void raw(const void* p, size_t n) { if (pos + n > cap) { ok = false; return; } memcpy(b + pos, p, n); pos += n; }
  template <typename T> void put(T v) { align(sizeof(T)); raw(&v, sizeof(T)); }
  void put(bool v) { uint8_t u = v ? 1 : 0; raw(&u, 1); }
  template <typename T> void array(const T* p, uint32_t n) { if (n) { align(sizeof(T)); raw(p, (size_t)n * sizeof(T)); } }
  void array(const bool* p, uint32_t n) { for (uint32_t i = 0; i < n; i++) put(p[i]); }
  void str(const char* s) { uint32_t n = (uint32_t)strlen(s) + 1; put(n); raw(s, n); }
};

struct Reader {
  const uint8_t* b; size_t n; size_t pos = 0; bool ok = true;
  Reader(const uint8_t* buf, size_t len) : b(buf), n(len) {}
  bool header() { if (n < 4 || b[0] != 0 || b[1] != 1) { ok = false; return false; } pos = 4; return true; }
  void align(size_t a) { size_t o = (pos - 4) % a; if (o) pos += a - o; if (pos > n) ok = false; }
  void raw(void* p, size_t k) { if (!ok || pos + k > n) { ok = false; memset(p, 0, k); return; } memcpy(p, b + pos, k); pos += k; }
  template <typename T> void get(T& v) { align(sizeof(T)); raw(&v, sizeof(T)); }
  void get(bool& v) { uint8_t u = 0; raw(&u, 1); v = u != 0; }
  template <typename T> void array(T* p, uint32_t k) { if (k) { align(sizeof(T)); raw(p, (size_t)k * sizeof(T)); } }
  void array(bool* p, uint32_t k) { for (uint32_t i = 0; i < k; i++) get(p[i]); }
  // sequence count; more than the capacity is an error (the message would not fit)
  uint32_t count(uint32_t capacity) { uint32_t c = 0; get(c); if (c > capacity) { ok = false; c = 0; } return c; }
  void str(char* s, size_t cap) {
    uint32_t k = 0; get(k);
    if (!ok || k == 0 || pos + k > n || k > cap) { ok = false; s[0] = 0; return; }
    memcpy(s, b + pos, k); s[k - 1] = 0; pos += k;
  }
};

}  // namespace cdr
