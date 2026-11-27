// Chip <-> PC serial link: frames that carry topics (ROS 2 CDR) between plain text log lines on the same USB port.
//   frame: A5 5A | kind u8 | flags u8 | id u16 | len u16 | payload[len] | crc16 u16      (little endian)
//          crc16 = CRC-16/CCITT-FALSE (poly 0x1021, start 0xFFFF) over kind..payload
//   Text is ASCII and never contains A5, so the PC splits the stream into text lines and frames; a frame with a bad
//   CRC (bytes lost on the USB port) is skipped and the reader looks for the next A5 5A. While the link is on, log
//   lines also travel in frames (LOG), so the PC can treat any other byte as noise.
//   kinds and payloads: see tools/ros2wire.py (the PC side of this file).
// Pure C++: the same file runs on a PC (test_host/) and on the chip.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <atomic>

namespace wire {

enum Kind : uint8_t { HELLO = 1, ADVERTISE = 2, DATA = 3, SUBSCRIBE = 4, UNSUBSCRIBE = 5, PUBLISH = 6, LIST = 7, STATS = 8, LOG = 9,
                     CALL = 10, REPLY = 11, SERVICE = 12, NODE = 13 };
enum Flags : uint8_t { F_RLE = 1 };
static const size_t HEAD = 8, TAIL = 2;     // bytes around the payload

inline uint16_t crc16(const uint8_t* p, size_t n, uint16_t crc = 0xFFFF) {
  static uint16_t T[256]; static bool init = false;
  if (!init) {
    for (int i = 0; i < 256; i++) { uint16_t c = (uint16_t)(i << 8); for (int k = 0; k < 8; k++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1); T[i] = c; }
    init = true;
  }
  for (size_t i = 0; i < n; i++) crc = (uint16_t)((crc << 8) ^ T[((crc >> 8) ^ p[i]) & 0xFF]);
  return crc;
}

// run-length code: pairs (count 1..255, byte). Returns the coded length, or 0 if it would not be shorter than n.
inline size_t rle_encode(const uint8_t* in, size_t n, uint8_t* out, size_t cap) {
  size_t o = 0;
  for (size_t i = 0; i < n;) {
    size_t j = i; while (j < n && j - i < 255 && in[j] == in[i]) j++;
    if (o + 2 > cap || o + 2 >= n) return 0;
    out[o++] = (uint8_t)(j - i); out[o++] = in[i]; i = j;
  }
  return o;
}

// Single-producer / single-consumer byte ring (the node task writes frames, the I/O task sends them).
struct Ring {
  uint8_t* b = nullptr; size_t cap = 0;
  std::atomic<size_t> head{0}, tail{0};       // head: next write, tail: next read (both grow; index = value % cap)
  size_t used() const { return head.load() - tail.load(); }
  size_t space() const { return cap - used(); }
  void put(const uint8_t* p, size_t n) {      // caller checked space()
    size_t h = head.load();
    for (size_t i = 0; i < n;) { size_t k = h % cap, c = cap - k < n - i ? cap - k : n - i; memcpy(b + k, p + i, c); i += c; h += c; }
    head.store(h);
  }
  size_t get(uint8_t* p, size_t n) {          // up to n bytes
    size_t t = tail.load(), u = head.load() - t; if (n > u) n = u;
    for (size_t i = 0; i < n;) { size_t k = t % cap, c = cap - k < n - i ? cap - k : n - i; memcpy(p + i, b + k, c); i += c; t += c; }
    tail.store(t); return n;
  }
};

// Writes whole frames into a ring, or nothing (a frame that does not fit is dropped and counted).
struct Sender {
  Ring* ring = nullptr; uint32_t sent = 0, dropped = 0; uint64_t bytes = 0;
  bool frame(uint8_t kind, uint16_t id, const uint8_t* payload, size_t n, uint8_t flags = 0) {
    if (n > 0xFFFF || ring->space() < HEAD + n + TAIL) { dropped++; return false; }
    uint8_t h[HEAD] = {0xA5, 0x5A, kind, flags, (uint8_t)id, (uint8_t)(id >> 8), (uint8_t)n, (uint8_t)(n >> 8)};
    uint16_t crc = crc16(h + 2, 6); crc = crc16(payload, n, crc);
    uint8_t t[TAIL] = {(uint8_t)crc, (uint8_t)(crc >> 8)};
    ring->put(h, HEAD); ring->put(payload, n); ring->put(t, TAIL);
    sent++; bytes += HEAD + n + TAIL; return true;
  }
  bool text(const char* s) { return frame(LOG, 0, (const uint8_t*)s, strlen(s)); }   // one log line (the PC prints it)
};

// Finds frames in received bytes (anything else is skipped). put() adds bytes, next() returns the frames one by one.
// A frame with a bad CRC or an impossible length is skipped one byte at a time, so a damaged frame (bytes lost on the
// way) cannot swallow the frames after it. payload stays valid until the next call of put() or next().
struct Parser {
  uint8_t* buf; size_t cap, n = 0, drop = 0, max_len; uint32_t bad = 0;
  uint8_t kind = 0, flags = 0; uint16_t id = 0; const uint8_t* payload = nullptr; size_t len = 0;
  Parser(uint8_t* b, size_t c, size_t max = 0) : buf(b), cap(c), max_len(max && max < c - HEAD - TAIL ? max : c - HEAD - TAIL) {}
  void compact() { if (drop) { memmove(buf, buf + drop, n - drop); n -= drop; drop = 0; } }
  size_t put(const uint8_t* p, size_t k) {     // takes at least one byte (k > 0)
    compact();
    if (n == cap) { memmove(buf, buf + 1, n - 1); n--; bad++; }   // full: its start cannot begin a valid frame
    if (k > cap - n) k = cap - n;
    memcpy(buf + n, p, k); n += k; return k;
  }
  bool next() {
    compact();
    size_t i = 0;
    for (;;) {
      while (i < n && buf[i] != 0xA5) i++;
      if (i + 1 >= n) break;                     // nothing, or a lone A5 at the end: wait for more bytes
      if (buf[i + 1] != 0x5A) { i++; continue; }
      if (n - i < HEAD) break;
      size_t L = buf[i + 6] | (buf[i + 7] << 8);
      if (L > max_len) { bad++; i++; continue; }
      if (n - i < HEAD + L + TAIL) break;        // frame not complete yet
      const uint8_t* f = buf + i;
      uint16_t crc = (uint16_t)(f[HEAD + L] | (f[HEAD + L + 1] << 8));
      if (crc != crc16(f + 2, 6 + L)) { bad++; i++; continue; }
      kind = f[2]; flags = f[3]; id = (uint16_t)(f[4] | (f[5] << 8)); payload = f + HEAD; len = L;
      drop = i + HEAD + L + TAIL;
      return true;
    }
    drop = i;
    return false;
  }
};

}  // namespace wire
