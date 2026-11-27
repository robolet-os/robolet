// PC run of the on-chip virtual driving (firmware command [n]).
// Build and run:  g++ -O2 -std=c++17 -I../src test_nav.cpp -o tn && ./tn [seed ...]
// Runs the node version (nav_nodes.h, what the firmware uses) and the single-loop version (nav_core.h nav::run) and
// checks that every result number except the timings is exactly the same.
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include "nav_nodes.h"

static int64_t now_us() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static bool quiet = false;
static void logline(const char* s) { if (!quiet) { printf("%s\n", s); fflush(stdout); } }
static void graphline(const char* s) { printf("[graph] %s\n", s); }

int main(int argc, char** argv) {
  using namespace nav;
  if (getenv("DISTORT")) x4sim::opts().distort = atoi(getenv("DISTORT")) != 0;
  if (getenv("DESKEW")) x4sim::opts().deskew = atoi(getenv("DESKEW"));
  Work w;
  w.occ_truth = (uint8_t*)malloc(CELLS); w.dist_truth = (uint8_t*)malloc(CELLS); w.lo = (int8_t*)malloc(CELLS);
  w.occ = (uint8_t*)malloc(CELLS); w.dist = (uint8_t*)malloc(CELLS); w.lik = (uint8_t*)malloc(CELLS); w.cost = (uint8_t*)malloc(CELLS);
  w.g = (float*)malloc(CELLS * 4); w.f = (float*)malloc(CELLS * 4); w.heap = (uint16_t*)malloc(CELLS * 2); w.pos = (uint16_t*)malloc(CELLS * 2);
  w.state = (uint8_t*)malloc(CELLS); w.path_x = (float*)malloc(PATH_CAP * 4); w.path_y = (float*)malloc(PATH_CAP * 4);
  w.parts = (Pose*)malloc(NP * sizeof(Pose)); w.parts2 = (Pose*)malloc(NP * sizeof(Pose)); w.pw = (float*)malloc(NP * 4);
  w.rev = (uint8_t*)malloc(x4sim::REV_BYTES); w.px = (float*)malloc(MAXPTS * 4); w.py = (float*)malloc(MAXPTS * 4); w.idx = (int32_t*)malloc(MAXPTS * 4);
  Hooks h = {now_us, logline};
  int nseeds = argc > 1 ? argc - 1 : 1; bool all_pass = true, all_same = true;
  for (int si = 0; si < nseeds; si++) {
    uint32_t seed = argc > 1 ? (uint32_t)atoi(argv[1 + si]) : 0;
    quiet = nseeds > 1;
    int64_t t0 = now_us();
    Result b = navn::run_nodes(w, h, seed, si == 0 && !quiet ? graphline : nullptr);
    double tb = (now_us() - t0) / 1e6;
    quiet = true; t0 = now_us();
    Result a = run(w, h, seed);
    double ta = (now_us() - t0) / 1e6;
    int diff = 0;
#define SAME(f) if (a.f != b.f) { diff++; printf("  differs: %-20s single loop %g, nodes %g\n", #f, (double)a.f, (double)b.f); }
    SAME(map_cycles) SAME(keyframes) SAME(map_dist_m) SAME(slam_err_max) SAME(slam_err_final) SAME(slam_th_err_max)
    SAME(map_wall_err_mean) SAME(map_within_5cm) SAME(map_occ_cells) SAME(goals) SAME(goals_reached) SAME(goals_unreachable)
    SAME(nav_cycles) SAME(amcl_updates) SAME(plans) SAME(goal_true_err_max) SAME(loc_err_max) SAME(loc_err_median)
    SAME(loc_th_err_max) SAME(contacts) SAME(min_clearance) SAME(pass)
    printf("seed %u: nodes %s, single loop %s | %s (%d differences) | PC run nodes %.2f s, single loop %.2f s\n", seed, b.pass ? "PASS" : "FAIL",
           a.pass ? "PASS" : "FAIL", diff ? "NOT the same" : "same results", diff, tb, ta);
    all_pass = all_pass && b.pass; all_same = all_same && diff == 0;
  }
  printf("RESULT nav: %s, node version %s the single-loop version\n", all_pass ? "PASS" : "FAIL", all_same ? "gives the same results as" : "DIFFERS from");
  return all_pass && all_same ? 0 : 1;
}
