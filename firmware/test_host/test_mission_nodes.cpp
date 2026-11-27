// PC check of the node version of the mission (mission_nodes.h, what the firmware runs for [k]) against the single-loop
// version (mission_core.h mis::run_mission): every log line and the MISSION line must be the same, except the timings.
// Build:  g++ -O2 -std=c++17 -I../src test_mission_nodes.cpp -o tmn
// Run:    [OPTS=0..3] [DISTORT=0|1] [DESKEW=0|1|2] [GRAPH=1] ./tmn <world 0..4> <mode 0|1> <seed...>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <string>
#include <vector>
#include "mission_nodes.h"

static int64_t now_us() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static std::vector<std::string>* sink = nullptr;
static bool echo = false;
// timings removed: "(search N ms)", the MISSION tail "| max cycle ...", the node load line
static void logline(const char* s) {
  std::string l(s);
  if (echo) { printf("%s\n", s); fflush(stdout); }
  if (l.find("robot nodes:") != std::string::npos) return;
  size_t p = l.find(" (search "); if (p != std::string::npos) l = l.substr(0, p);
  p = l.find(" | max cycle"); if (p != std::string::npos) l = l.substr(0, p);
  if (sink) sink->push_back(l);
}
static void graphline(const char* s) { printf("[graph] %s\n", s); }

int main(int argc, char** argv) {
  using namespace nav;
  setvbuf(stdout, nullptr, _IOLBF, 0);
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
  if (getenv("DISTORT")) x4sim::opts().distort = atoi(getenv("DISTORT")) != 0;
  if (getenv("DESKEW")) x4sim::opts().deskew = atoi(getenv("DESKEW"));
  int opts = getenv("OPTS") ? atoi(getenv("OPTS")) : mis::OPT_KIDNAP;
  // the single loop searches and checks candidates in one go: the node version too (SLICES=n / TURN=k: spread over
  // cycles, as on the chip; then the results may differ)
  navn::run_hooks().search_slices = getenv("SLICES") ? atoi(getenv("SLICES")) : 1;
  navn::run_hooks().turn_hyps = getenv("TURN") ? atoi(getenv("TURN")) : 0;   // (0: all candidates in one cycle, as the single loop)
  int world = argc > 1 ? atoi(argv[1]) : 1, mode = argc > 2 ? atoi(argv[2]) : 1;
  echo = getenv("ECHO") != nullptr;
  Hooks h = {now_us, logline};
  bool all_same = true;
  for (int i = 3; i < (argc > 3 ? argc : 4); i++) {
    uint32_t seed = argc > 3 ? (uint32_t)atoi(argv[i]) : 0;
    std::vector<std::string> a, b;
    int64_t t0 = now_us();
    sink = &b; mis::MResult rb = misn::run_mission_nodes(mw, h, world, mode, seed, opts, (getenv("GRAPH") && i == 3) ? graphline : nullptr);
    mis::print_result(h, rb);
    double tb = (now_us() - t0) / 1e6; t0 = now_us();
    sink = &a; mis::MResult ra = mis::run_mission(mw, h, world, mode, seed, opts);
    mis::print_result(h, ra);
    double ta = (now_us() - t0) / 1e6; sink = nullptr;
    int diff = 0; size_t n = a.size() > b.size() ? a.size() : b.size();
    for (size_t k = 0; k < n; k++) {
      const char* sa = k < a.size() ? a[k].c_str() : "(none)"; const char* sb = k < b.size() ? b[k].c_str() : "(none)";
      if (strcmp(sa, sb) != 0) { if (diff < 6) printf("  line %zu differs\n    single loop: %s\n    nodes:       %s\n", k, sa, sb); diff++; }
    }
    printf("%s\n", b.back().c_str());
    printf("world %d mode %d opts %d seed %u: %s (%zu lines, %d differ) | PC run nodes %.1f s, single loop %.1f s\n", world, mode, opts, seed,
           diff ? "NOT the same" : "same", b.size(), diff, tb, ta);
    all_same = all_same && diff == 0;
  }
  printf("RESULT mission nodes: %s\n", all_same ? "same as the single loop" : "DIFFERS from the single loop");
  return all_same ? 0 : 1;
}
