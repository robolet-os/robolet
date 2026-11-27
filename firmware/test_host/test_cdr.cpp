// CDR round trip for tools/check_cdr.py: reads records (u16 type-name length, name, u32 byte count, CDR bytes) from
// stdin, decodes each into the firmware struct with cdr_read, encodes it again with cdr_write and writes the same record
// format to stdout. check_cdr.py compares the bytes with those made by rosbags (an independent ROS 2 implementation).
// Build: g++ -O2 -std=c++17 -I../src test_cdr.cpp -o tcdr
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "ros2_msgs.h"

template <typename T> static bool roundtrip(const std::vector<uint8_t>& in, std::vector<uint8_t>& out) {
  static T m; m = T();
  cdr::Reader r(in.data(), in.size()); r.header(); cdr_read(r, m);
  if (!r.ok || r.pos != in.size()) return false;
  out.resize(in.size() + 64);
  cdr::Writer w(out.data(), out.size()); w.header(); cdr_write(w, m);
  out.resize(w.pos);
  return w.ok;
}

int main() {
  for (;;) {
    uint16_t tl; if (fread(&tl, 2, 1, stdin) != 1) break;
    std::string tn(tl, 0); if (fread(&tn[0], 1, tl, stdin) != tl) break;
    uint32_t n; if (fread(&n, 4, 1, stdin) != 1) break;
    std::vector<uint8_t> in(n), out; if (fread(in.data(), 1, n, stdin) != n) break;
    bool ok = false, known = false;
#define X(T) if (tn == T::TYPE) { known = true; ok = roundtrip<T>(in, out); }
    ROS2_MSG_TYPES(X)
#undef X
    if (!known || !ok) out.clear();
    uint32_t m = (uint32_t)out.size();
    fwrite(&tl, 2, 1, stdout); fwrite(tn.data(), 1, tl, stdout); fwrite(&m, 4, 1, stdout); fwrite(out.data(), 1, m, stdout);
  }
  return 0;
}
