// PC check of the real-robot mode (real_nodes.h, firmware command [r]) with a simulated Jarabot: a simulated X4 lidar
// whose bytes go through the revolution framer, and a simulated Arduino that turns the "pwm,<turn>,<fwd>" lines back into
// wheel speeds (inverse of the Jarabot mapping) and counts encoder pulses. An operator node plays the user: it turns
// and drives the robot with /teleop/cmd_vel while slam maps, calls /mission/finish_mapping, sends two navigate_to_pose
// goals and calls /mission/stop. PASS: both goals SUCCEEDED, the robot ends within 0.3 m of the second goal, no contact.
// Build: g++ -O2 -std=c++17 -I../src test_real.cpp -o treal      Run: ./treal [VERBOSE=1]
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include "real_nodes.h"

static int64_t now_us() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static bool verbose = false;
static void logline(const char* s) { if (verbose || strncmp(s, "[r]", 3) == 0) { printf("%s\n", s); fflush(stdout); } }

// ---- the simulated robot
static uint8_t *occ_world, *dist_world;
static nav::Robot rb;
static x4sim::Rng lrng{4242u};
static uint8_t revbytes[x4sim::REV_BYTES];
static realn::RevFramer framer;
static int pwm_ang = 1500, pwm_lin = 1500, nlines = 0;
static double cl = 0, cr = 0;
static int goals_ok = 0;                                     // encoder counts (fractional)
static const double WR = 0.035, WB = 0.22, ENC = 1200;

static void arduino_line(const char* s) {                         // motor.ino: "pwm,<roll>,<pitch>"
  int a, l; if (sscanf(s, "pwm,%d,%d", &a, &l) == 2) { pwm_ang = a; pwm_lin = l; nlines++; }
}
static void step_robot() {                                        // one revolution of time with the last command
  const double db = 170, gl = (500.0 - 170.0) / 0.26, ga = (500.0 - 170.0) / 1.0;
  float v = pwm_lin >= 1500 + db ? (float)((pwm_lin - 1500 - db) / gl) : pwm_lin <= 1500 - db ? (float)((pwm_lin - 1500 + db) / gl) : 0.0f;
  float w = pwm_ang <= 1500 - db ? (float)((1500 - db - pwm_ang) / ga) : pwm_ang >= 1500 + db ? (float)(-(pwm_ang - 1500 - db) / ga) : 0.0f;
  bc::Pose p0 = rb.truth;
  rb.step(dist_world, v, w);
  bc::Pose d = nav::ominus(p0, rb.truth);                          // motion in the frame of the start pose
  double dth = d.th, dist = d.x;
  double dl = dist - dth * WB / 2, dr = dist + dth * WB / 2;
  cl += dl / (2 * 3.14159265358979 * WR) * ENC; cr += dr / (2 * 3.14159265358979 * WR) * ENC;
}
static int lidar_rev(uint8_t* out, int cap) {
  step_robot();
  int n = x4sim::revolution_moving(occ_world, rb.truth_prev, rb.truth, revbytes, lrng);
  framer.feed(revbytes, n);
  return framer.take(out, cap);
}

// ---- the operator
struct Operator : ros::Node {
  ros::Client<std_srvs::srv::Trigger> finish, stop; ros::ActionClient<nav2_msgs::action::NavigateToPose> nav;
  bus::Publisher<geometry_msgs::msg::Twist> tel;
  int t = 0, goals_sent = 0, step = 0; bool waiting = false;
  float GX[2] = {6.5f, 2.0f}, GY[2] = {1.5f, 5.0f};
  Operator() : ros::Node("operator") {
    snprintf(finish.name, sizeof finish.name, "/mission/finish_mapping"); snprintf(stop.name, sizeof stop.name, "/mission/stop");
    snprintf(nav.name, sizeof nav.name, "/navigate_to_pose");
    tel = create_publisher<geometry_msgs::msg::Twist>("/teleop/cmd_vel");
    create_timer(navn::TICK_NS, [this]() { tick(); });
  }
  void twist(float v, float w) { auto& m = tel.loan(); m.linear.x = v; m.angular.z = w; tel.publish(); }
  void call(ros::Client<std_srvs::srv::Trigger>& c) {
    std_srvs::srv::Trigger_Request q; std_srvs::srv::Trigger_Response r;
    bool ok = c.call(q, r); printf("[op] %s: %s %s\n", c.name, ok && r.success ? "ok" : "FAILED", r.message.c_str());
  }
  void tick() {
    t++;
    // mapping by hand: 2 s still, turn 7 rad, 0.6 m ahead, turn back, 0.6 m back
    const int T1 = 14, T2 = T1 + 100, T3 = T2 + 28, T4 = T3 + 44, T5 = T4 + 28;
    if (t > T1 && t <= T2) twist(0.0f, 0.5f);
    else if (t > T2 && t <= T3) twist(0.15f, 0.0f);
    else if (t > T3 && t <= T4) twist(0.0f, 0.5f);
    else if (t > T4 && t <= T5) twist(0.15f, 0.0f);
    else if (t == T5 + 7) call(finish);
    else if (t > T5 + 17 && !waiting && goals_sent < 2) {
      nav2_msgs::action::NavigateToPose_Goal g; conv::header(g.pose.header, "map"); conv::to_msg({GX[goals_sent], GY[goals_sent], 0.0f}, g.pose.pose);
      int k = goals_sent++; waiting = true;
      printf("[op] goal %d (%.1f, %.1f)\n", k + 1, GX[k], GY[k]);
      nav.send_goal(g, [this, k](int8_t st, const nav2_msgs::action::NavigateToPose_Result& r) {
        printf("[op] goal %d: %s (robot at %.2f, %.2f)\n", k + 1, st == ros::GoalStatus::STATUS_SUCCEEDED ? "SUCCEEDED" : r.error_msg.c_str(), rb.truth.x, rb.truth.y);
        if (st == ros::GoalStatus::STATUS_SUCCEEDED) goals_ok++;
        waiting = false; });
    } else if (goals_sent == 2 && !waiting && step == 0) { step = 1; call(stop); }
  }
};

int main() {
  using namespace nav;
  setvbuf(stdout, nullptr, _IOLBF, 0);
  verbose = getenv("VERBOSE") && atoi(getenv("VERBOSE"));
  bus::hooks().alloc = [](size_t n) { return calloc(1, n); }; bus::hooks().alloc_fast = bus::hooks().alloc; bus::hooks().release = free;
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
  occ_world = (uint8_t*)malloc(CELLS); dist_world = (uint8_t*)malloc(CELLS);
  mis::build_world(occ_world, mis::WORLD_ROOM); bc::distance_field(occ_world, dist_world);
  rb.truth = realn::start_pose(); rb.odom = {0, 0, 0}; rb.rng = {777u}; rb.hold();
  realn::io().lidar_rev = lidar_rev;
  realn::io().base_write = arduino_line;
  realn::io().base_counts = [](long& l, long& r) { l = lround(cl); r = lround(cr); return true; };
  navn::run_hooks().extra_node = []() -> bus::NodeBase* { return new Operator(); };
  static uint8_t rev[realn::RevFramer::CAP];
  Hooks h = {now_us, logline};
  int64_t t0 = now_us();
  realn::run_real(mw, h, rev, sizeof rev, 7LL * 600);
  bool near = hypotf(rb.truth.x - 2.0f, rb.truth.y - 5.0f) < 0.3f;
  printf("framer: %u revolutions, %u bad packets | arduino: %d lines | contacts %d | robot at (%.2f, %.2f) | PC run %.1f s\n",
         framer.revs, framer.bad, nlines, rb.contacts, rb.truth.x, rb.truth.y, (now_us() - t0) / 1e6);
  bool ok = goals_ok == 2 && near && rb.contacts == 0;
  printf("RESULT real: %s (goals reached %d of 2, ends near goal 2: %s, contacts %d)\n", ok ? "PASS" : "FAIL", goals_ok, near ? "yes" : "no", rb.contacts);
  return ok ? 0 : 1;
}
