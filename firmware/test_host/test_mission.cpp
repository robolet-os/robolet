// PC run of the lost-robot mission (firmware command [k]).
// Build:  g++ -O2 -std=c++17 -I../src test_mission.cpp -o tm
// Run:    [OPTS=0|1|2|3] [DISTORT=0|1] [DESKEW=0|1|2] ./tm <world 0..4> <mode 0|1> <seed...>
//         world 0 room, 1 s_corridor, 2 s_corridor_lm, 3 doors, 4 room_tilted; mode 0 AMCL only, 1 spin; OPTS 0 none, 1 kidnap (default), 2 unmapped box, 3 both
//         DISTORT 1 (default) lidar moves during a revolution; DESKEW 0 none, 2 whole scan from start pose, 1 per point (default)
// All worlds x modes x options, 8 seeds each:  bash regress.sh ./tm   (about 8 min on a PC; WORLDS="0 4" picks worlds)
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include "mission_core.h"

static int64_t now_us() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static bool verbose = true;
static void logline(const char* s) { if (verbose || strncmp(s, "MISSION", 7) == 0) { printf("%s\n", s); fflush(stdout); } }

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
  if (getenv("DISTORT")) x4sim::opts().distort = atoi(getenv("DISTORT")) != 0;   // default 1: lidar moves during a revolution
  if (getenv("DESKEW")) x4sim::opts().deskew = atoi(getenv("DESKEW"));      // default 1: receiver corrects it
  int opts = getenv("OPTS") ? atoi(getenv("OPTS")) : mis::OPT_KIDNAP;     // 1 kidnap, 2 box, 3 both
  int world = argc > 1 ? atoi(argv[1]) : 1, mode = argc > 2 ? atoi(argv[2]) : 1;
  if (argc > 4) verbose = false;
  Hooks h = {now_us, logline};
  for (int i = 3; i < (argc > 3 ? argc : 4); i++) {
    uint32_t seed = argc > 3 ? (uint32_t)atoi(argv[i]) : 0;
    int64_t t0 = now_us();
    mis::MResult r = mis::run_mission(mw, h, world, mode, seed, opts);
    mis::print_result(h, r);
    if (verbose) printf("(PC run %.1f s)\n", (now_us() - t0) / 1e6);
  }
  return 0;
}
