// PC run of a mission with the bridge node (bridge.h): the bytes the chip would send to the PC go to a file, and
// the requests the PC would send (SUBSCRIBE, one PUBLISH) are fed in at the start. Checks:
//   - the MISSION result is the same with and without the bridge (the bridge only reads),
//   - tools/check_link.py then decodes the file (frames, CDR) and checks the messages.
// Build: g++ -O2 -std=c++17 -I../src test_bridge.cpp -o tbr
// Run:   ./tbr <world> <mode> <seed> <out.bin> [rate10 for /scan_cloud ...]
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <string>
#include "mission_nodes.h"
#include "bridge.h"

static int64_t now_us() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static std::string last_mission;
static void logline(const char* s) { std::string l(s); if (l.rfind("MISSION", 0) == 0) last_mission = l.substr(0, l.find(" | max cycle")); }

int main(int argc, char** argv) {
  using namespace nav;
  static Work w;
  w.occ_truth = (uint8_t*)malloc(CELLS); w.dist_truth = (uint8_t*)malloc(CELLS); w.lo = (int8_t*)malloc(CELLS);
  w.occ = (uint8_t*)malloc(CELLS); w.dist = (uint8_t*)malloc(CELLS); w.lik = (uint8_t*)malloc(CELLS); w.cost = (uint8_t*)malloc(CELLS);
  w.g = (float*)malloc(CELLS * 4); w.f = (float*)malloc(CELLS * 4); w.heap = (uint16_t*)malloc(CELLS * 2); w.pos = (uint16_t*)malloc(CELLS * 2);
  w.state = (uint8_t*)malloc(CELLS); w.path_x = (float*)malloc(PATH_CAP * 4); w.path_y = (float*)malloc(PATH_CAP * 4);
  w.parts = (Pose*)malloc(NP * sizeof(Pose)); w.parts2 = (Pose*)malloc(NP * sizeof(Pose)); w.pw = (float*)malloc(NP * 4);
  w.rev = (uint8_t*)malloc(x4sim::REV_BYTES); w.px = (float*)malloc(MAXPTS * 4); w.py = (float*)malloc(MAXPTS * 4); w.idx = (int32_t*)malloc(MAXPTS * 4);
  mis::MWork mw = {&w, (float*)malloc(MAXPTS * 4), (float*)malloc(MAXPTS * 4), (mis::Cand*)malloc(mis::CAND_CAP * sizeof(mis::Cand)),
                   (float*)malloc(mis::MAXHYP * (MAXPTS / 2) * 4), (float*)malloc((MAXPTS / 2) * 4), (float*)malloc(mis::ERRS_CAP * 4),
                   (uint8_t*)malloc(CELLS), (uint8_t*)malloc(CELLS), (uint8_t*)malloc(CELLS), (uint8_t*)malloc(CELLS), (uint8_t*)malloc(CELLS)};
  int world = argc > 1 ? atoi(argv[1]) : 0, mode = argc > 2 ? atoi(argv[2]) : 1; uint32_t seed = argc > 3 ? atoi(argv[3]) : 0;
  const char* out = argc > 4 ? argv[4] : "bridge_out.bin";
  Hooks h = {now_us, logline};

  // 1. without the bridge
  mis::print_result(h, misn::run_mission_nodes(mw, h, world, mode, seed, 3));
  std::string plain = last_mission;

  // 2. with the bridge
  static uint8_t txb[1 << 20], rxb[1 << 16], work[3 * 65536];
  static wire::Ring tx, rx; tx.b = txb; tx.cap = sizeof txb; rx.b = rxb; rx.cap = sizeof rxb;
  wire::Sender req; req.ring = &rx;                                 // what the PC tool sends after HELLO
  auto sub = [&](const char* name, uint16_t rate10) {
    uint8_t p[64]; size_t a = strlen(name) + 1; memcpy(p, name, a); p[a] = (uint8_t)rate10; p[a + 1] = (uint8_t)(rate10 >> 8);
    req.frame(wire::SUBSCRIBE, 0, p, a + 2); };
  sub("/map", 10); sub("/scan", 0); sub("/amcl/robot_pose", 0); sub("/pose", 0); sub("/ground_truth", 0); sub("/plan", 0);
  sub("/localization/status", 0); sub("/localization/candidates", 0); sub("/cmd_vel", 0); sub("/no_such_topic", 0);
  { // a PUBLISH on a new topic (the chip makes it): std_msgs/Int32 42
    std_msgs::msg::Int32 m; m.data = 42; uint8_t c[16]; cdr::Writer cw(c, sizeof c); cw.header(); cdr_write(cw, m);
    uint8_t p[96]; const char* nm = "/pc/hello"; const char* ty = std_msgs::msg::Int32::TYPE;
    size_t a = strlen(nm) + 1, b = strlen(ty) + 1; memcpy(p, nm, a); memcpy(p + a, ty, b); memcpy(p + a + b, c, cw.pos);
    req.frame(wire::PUBLISH, 0, p, a + b + cw.pos);
  }
  FILE* f = fopen(out, "wb");
  bridge::Bridge* br = nullptr;
  navn::run_hooks().extra_node = [&]() -> bus::NodeBase* { br = new bridge::Bridge(&tx, &rx, work, 65536, "robolet PC test of the bridge"); return br; };
  uint64_t bytes = 0; int64_t ticks = 0;
  navn::run_hooks().after_tick = [&](int64_t t) { ticks = t; uint8_t tmp[4096]; size_t k; while ((k = tx.get(tmp, sizeof tmp)) > 0) { fwrite(tmp, 1, k, f); bytes += k; } };
  mis::MResult r = misn::run_mission_nodes(mw, h, world, mode, seed, 3);
  std::string bridged = (mis::print_result(h, r), last_mission);
  navn::run_hooks() = navn::RunHooks();
  fclose(f);
  printf("%s\n", bridged.c_str());
  printf("bridge: %llu bytes to the PC in %lld cycles (%.1f KB per second at real time)\n", (unsigned long long)bytes, (long long)ticks,
         bytes / 1024.0 / (ticks * nav::DT));
  bool same = plain == bridged;
  printf("RESULT bridge: %s\n", same ? "same mission result with and without the bridge" : "DIFFERENT result with the bridge");
  return same ? 0 : 1;
}
