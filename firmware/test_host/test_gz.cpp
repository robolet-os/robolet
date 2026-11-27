// PC stand-in for the chip: the same mission nodes and bridge as the firmware command "k w m s o [gz] [rt] [hold] bridge", on a
// pseudo terminal instead of the USB port. Lets the PC tools (tools/gzbridge.py with Gazebo, tools/check_ros.py,
// tools/robolet.py) be tested without the board. "gz": Gazebo plays the robot; "rt": one cycle per 1/7 s.
// Build: g++ -O2 -std=c++17 -pthread -I../src test_gz.cpp -o tgz -lutil
// Run:   ./tgz            prints the terminal name, e.g. /dev/pts/3; then (another shell)
//        /usr/bin/python3 tools/gzbridge.py --port /dev/pts/3 --chip "k 0 1 0 3" --view
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <string>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pty.h>
#include "mission_nodes.h"
#include "bridge.h"

static int64_t now_us() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static wire::Ring tx, rx;
static wire::Sender text;
static void logline(const char* s) {                             // like the firmware: log lines go out as LOG frames
  printf("%s\n", s); fflush(stdout);
  for (int i = 0; i < 500 && tx.space() < strlen(s) + 16; i++) usleep(1000);
  text.text(s);
}

int main() {
  using namespace nav;
  int master, slave; char name[64];
  if (openpty(&master, &slave, name, nullptr, nullptr) != 0) { perror("openpty"); return 1; }
  termios t; tcgetattr(slave, &t); cfmakeraw(&t); tcsetattr(slave, TCSANOW, &t);
  printf("%s\n", name); fflush(stdout);
  // wait for the command line ("k 0 1 0 3 gz bridge")
  std::string cmd; char c;
  while (read(master, &c, 1) == 1) { if (c == '\n' || c == '\r') { if (cmd.find("bridge") != std::string::npos) break; cmd.clear(); } else cmd += c; }
  int world = 0, mode = 1, seed = 0, opts = 3;
  sscanf(cmd.c_str() + 1, "%d %d %d %d", &world, &mode, &seed, &opts);
  printf("command: %s\n", cmd.c_str());

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

  static uint8_t txb[1 << 18], rxb[1 << 14], work[3 * 65536];
  tx.b = txb; tx.cap = sizeof txb; rx.b = rxb; rx.cap = sizeof rxb; text.ring = &tx;
  std::atomic<bool> on{true};
  std::thread io([&]() {                                         // the firmware's link task: ring -> port, port -> ring
    uint8_t buf[4096];
    fcntl(master, F_SETFL, fcntl(master, F_GETFL) | O_NONBLOCK);
    while (on || tx.used()) {
      size_t k = tx.get(buf, sizeof buf); bool moved = k > 0;
      for (size_t off = 0; off < k;) { ssize_t n = write(master, buf + off, k - off); if (n > 0) off += n; else usleep(200); }
      while (rx.space() > sizeof buf) { ssize_t n = read(master, buf, sizeof buf); if (n <= 0) break; rx.put(buf, n); moved = true; }
      if (!moved) usleep(500);
    }
  });
  char hello[160]; snprintf(hello, sizeof hello, "robolet PC stand-in (test_gz) | %s", cmd.c_str());
  bridge::Bridge* br = nullptr;
  const bool gz = cmd.find(" gz") != std::string::npos, rt = cmd.find(" rt") != std::string::npos;
  navn::run_hooks().gz = gz; navn::run_hooks().hold = cmd.find(" hold") != std::string::npos;
  navn::run_hooks().extra_node = [&]() -> bus::NodeBase* { br = new bridge::Bridge(&tx, &rx, work, 65536, hello); return br; };
  int64_t t_start = now_us();
  if (rt) navn::run_hooks().after_tick = [&](int64_t tick) { int64_t due = t_start + (tick + 1) * 142857; while (now_us() < due) usleep(1000); };
  if (gz) navn::run_hooks().before_tick = [&](int64_t tick) -> bool {   // wait for the PC's data of this cycle (/gz/step >= tick)
    int64_t t0 = now_us(), limit = tick == 0 ? 120000000 : 5000000;
    for (;;) {
      br->pump();
      bus::Topic* st = bus::find_topic("/gz/step");
      if (st && st->seq > 0 && ((const std_msgs::msg::Int32*)st->slot(st->seq - 1))->data >= tick) return true;
      if (now_us() - t0 > limit) return false;
      usleep(200);
    }
  };
  Hooks h = {now_us, logline};
  int64_t t0 = now_us();
  mis::MResult r = misn::run_mission_nodes(mw, h, world, mode, (uint32_t)seed, opts, logline);
  mis::print_result(h, r);
  char line[96]; snprintf(line, sizeof line, "  (PC time %.0f s)", (now_us() - t0) / 1e6); logline(line);
  snprintf(line, sizeof line, "RESULT mission: %s", r.outcome_code == mis::OUT_CORRECT ? "PASS" : "FAIL"); logline(line);
  navn::run_hooks() = navn::RunHooks();
  on = false; io.join();
  usleep(500000);
  return 0;
}
