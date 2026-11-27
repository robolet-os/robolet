// Real robot (firmware command [r]): the robot-side nodes of the mission (mission_nodes.h: bt_navigator, x4_driver,
// slam, amcl, localization_monitor, planner, controller, behaviors, collision monitor, goal checker) with a real lidar
// and a real motor base instead of sim_base. Pure C++: the hardware comes in through RealIO, so the PC test
// (test_host/test_real.cpp) runs the same code with a simulated lidar and a simulated motor controller.
//
//   node            publishes                          subscribes
//   base_driver     /odom, /tf (odom -> base_link)     /cmd_vel (navigation), /teleop/cmd_vel (keyboard; wins for cmd_timeout)
//   mission         -                                  services ~/finish_mapping, ~/stop
//
// How a run goes: the robot starts at the middle of the map (the map is 10 x 8 m, so the room may reach 5 m to either
// side and 4 m ahead and behind). Mapping: drive the robot by hand (teleop_twist_keyboard remapped to /teleop/cmd_vel,
// through tools/ros2_bridge.py) while slam builds the map. /mission/finish_mapping ends mapping: amcl starts at the slam
// pose and navigate_to_pose goals (rviz2 "2D Goal Pose" through the bridge) drive the robot. /mission/stop ends the run.
//
// Motor base: the serial protocol of the Jarabot Arduino sketch (motor.ino, github.com/firstbot1/jarabot):
//   to the Arduino   "pwm,<turn>,<fwd>\n"   RC-style values 1001..1999, 1500 = stop; turn below 1500 = counter-clockwise
//   from the Arduino "<left>,<right>\n"     encoder counts (encoder_resolution per wheel turn)
// The speed -> value mapping is the one of Jarabot's jara_controller (1500 +- (deadband + speed * gain)); the sketch
// keeps the last command, so this node sends one every cycle and a stop when no command came for cmd_timeout.
#pragma once
#include "mission_nodes.h"
#include <functional>

namespace realn {
using namespace misn;

// ---- the hardware (main.cpp: the lidar UART and the motor-base UART)
struct RealIO {
  std::function<int(uint8_t* out, int cap)> lidar_rev;     // bytes of the next complete revolution (waits for it); 0: none
  std::function<void(const char* line)> base_write;        // one line to the motor controller
  std::function<bool(long& left, long& right)> base_counts;   // latest encoder counts; false: none since the last call
};
inline RealIO& io() { static RealIO x; return x; }

// ---- lidar bytes -> whole revolutions. Keeps complete packets (header, length, checksum) and hands out the packets
// from one zero packet (CT bit 0) to the next, so x4_driver always gets exactly one revolution per cycle.
struct RevFramer {
  static const int CAP = 4096;
  uint8_t pkt[10 + 2 * 128]; int pn = 0, need = 0;
  uint8_t cur[CAP]; int ncur = 0; bool have_start = false;
  uint8_t done[CAP]; int ndone = 0; bool ready = false;
  uint32_t revs = 0, bad = 0;
  static uint16_t le16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
  void push(uint8_t b) {
    if (pn == 0) { if (b == 0xAA) pkt[pn++] = b; return; }
    if (pn == 1) { if (b == 0x55) pkt[pn++] = b; else pn = (b == 0xAA) ? 1 : 0; return; }
    pkt[pn++] = b;
    if (pn == 4) { int lsn = pkt[3]; if (lsn == 0 || lsn > 128) { pn = 0; return; } need = 10 + 2 * lsn; }
    if (pn >= 10 && pn == need) { packet(); pn = 0; }
  }
  void packet() {
    uint16_t x = 0;
    for (int i = 0; i < need; i += 2) if (i != 8) x ^= le16(pkt + i);
    if (x != le16(pkt + 8)) { bad++; return; }
    bool start = pkt[2] & 1;
    if (start) {
      if (have_start && ncur > 0) { memcpy(done, cur, ncur); ndone = ncur; ready = true; revs++; }
      ncur = 0; have_start = true;
    }
    if (!have_start) return;
    if (ncur + need <= CAP) { memcpy(cur + ncur, pkt, need); ncur += need; }
  }
  void feed(const uint8_t* d, int n) { for (int i = 0; i < n; i++) push(d[i]); }
  void reset() { pn = need = ncur = ndone = 0; have_start = ready = false; revs = bad = 0; }
  int take(uint8_t* out, int cap) { if (!ready) return 0; ready = false; int n = ndone < cap ? ndone : cap; memcpy(out, done, n); return n; }
};

// ---- "<left>,<right>\n" encoder lines from the motor controller (a byte stream): keeps the last complete line
struct CountLines {
  char line[32]; int n = 0; bool synced = false, got = false; long l = 0, r = 0;
  void feed(const uint8_t* d, int k) {
    for (int i = 0; i < k; i++) {
      char c = (char)d[i];
      if (c == '\n') {
        line[n] = 0;
        if (synced) { char* p = strchr(line, ','); if (p && p != line) { l = atol(line); r = atol(p + 1); got = true; } }
        synced = true; n = 0;                                    // (the first line may be cut)
      } else if (c != '\r' && n < (int)sizeof(line) - 1) line[n++] = c;
    }
  }
  bool take(long& left, long& right) { if (!got) return false; got = false; left = l; right = r; return true; }
};

// ---- base_driver
struct BaseDriver : ros::Node {
  double &wheel_radius, &wheel_base, &enc_res, &max_v, &max_w, &deadband, &lin_gain, &ang_gain, &cmd_timeout, &ramp;
  double &left_sign, &right_sign;
  bus::Publisher<Odometry> odom_pub; bus::Publisher<tf2_msgs::msg::TFMessage> tf_pub;
  Pose odom{0, 0, 0}; float v_meas = 0, w_meas = 0;
  bool have_counts = false; long l0 = 0, r0 = 0;
  float nav_v = 0, nav_w = 0, tel_v = 0, tel_w = 0; int nav_age = 1 << 20, tel_age = 1 << 20;
  double cur_lin = 1500, cur_ang = 1500; bool teleop_used = false;
  BaseDriver() : ros::Node("base_driver"),
      wheel_radius(declare_parameter("wheel_radius", 0.035, "m", 0.005, 0.5)),
      wheel_base(declare_parameter("wheel_base", 0.220, "m between the wheels", 0.05, 1.0)),
      enc_res(declare_parameter("encoder_resolution", 1200.0, "encoder counts per wheel turn", 1.0, 100000.0)),
      max_v(declare_parameter("max_vel_x", 0.26, "m/s sent to the motors at most", 0.0, 1.0)),
      max_w(declare_parameter("max_vel_theta", 1.0, "rad/s sent to the motors at most", 0.0, 4.0)),
      deadband(declare_parameter("pwm_deadband", 170.0, "added to 1500 so that the wheels start to turn", 0.0, 499.0)),
      lin_gain(declare_parameter("pwm_per_mps", (500.0 - 170.0) / 0.26, "pwm units per m/s above the deadband", 0.0, 10000.0)),
      ang_gain(declare_parameter("pwm_per_radps", (500.0 - 170.0) / 1.0, "pwm units per rad/s above the deadband", 0.0, 10000.0)),
      cmd_timeout(declare_parameter("cmd_timeout", 0.5, "s without a command before the motors stop", 0.1, 5.0)),
      ramp(declare_parameter("pwm_step", 100.0, "largest change of a pwm value per cycle", 1.0, 1000.0)),
      left_sign(declare_parameter("left_encoder_sign", 1.0, "1 or -1", -1.0, 1.0)),
      right_sign(declare_parameter("right_encoder_sign", 1.0, "1 or -1", -1.0, 1.0)) {
    odom_pub = create_publisher<Odometry>("/odom"); tf_pub = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 4);
    create_subscription<Twist>("/cmd_vel", [this](const Twist& t) { nav_v = (float)t.linear.x; nav_w = (float)t.angular.z; nav_age = 0; });
    create_subscription<Twist>("/teleop/cmd_vel", [this](const Twist& t) { tel_v = (float)t.linear.x; tel_w = (float)t.angular.z; tel_age = 0; });
    create_timer(TICK_NS, [this]() { tick(); });
    publish_state();                                             // slam and x4_driver have odometry from the first cycle
  }
  ~BaseDriver() { for (int i = 0; i < 3; i++) send_pwm(1500, 1500); }
  void send_pwm(int ang, int lin) {
    char s[32]; snprintf(s, sizeof s, "pwm,%d,%d\n", ang, lin);
    if (io().base_write) io().base_write(s);
  }
  static double toward(double cur, double target, double step) { return cur < target ? fmin(cur + step, target) : fmax(cur - step, target); }
  // as Jarabot's jara_controller: 1500 +- (deadband + speed * gain); turning counter-clockwise lowers <turn>
  int lin_pwm(float v) const { return v > 0.001f ? (int)lround(1500 + deadband + v * lin_gain) : v < -0.001f ? (int)lround(1500 - deadband + v * lin_gain) : 1500; }
  int ang_pwm(float w) const { return w > 0.001f ? (int)lround(1500 - deadband - w * ang_gain) : w < -0.001f ? (int)lround(1500 + deadband - w * ang_gain) : 1500; }
  void publish_state() {
    Odometry& o = odom_pub.loan(); conv::header(o.header, "odom"); o.child_frame_id.set("base_link");
    conv::to_msg(odom, o.pose.pose); o.twist.twist.linear.x = v_meas; o.twist.twist.angular.z = w_meas; odom_pub.publish();
    navn::publish_tf(tf_pub, "odom", "base_link", odom);
  }
  void tick() {
    long l, r;
    if (io().base_counts && io().base_counts(l, r)) {
      if (have_counts) {
        double k = 2 * 3.14159265358979 * wheel_radius / enc_res;
        double dl = (l - l0) * k * left_sign, dr = (r - r0) * k * right_sign;
        double d = 0.5 * (dl + dr), dth = (dr - dl) / wheel_base;
        float mid = odom.th + (float)(dth / 2);
        odom.x += (float)(d * cos(mid)); odom.y += (float)(d * sin(mid)); odom.th = wrap(odom.th + (float)dth);
        v_meas = (float)(d / DT); w_meas = (float)(dth / DT);
      }
      l0 = l; r0 = r; have_counts = true;
    } else { v_meas = 0; w_meas = 0; }
    publish_state();
    int limit = (int)lround(cmd_timeout / DT);
    float v = 0, w = 0;
    if (tel_age <= limit) { v = tel_v; w = tel_w; teleop_used = true; }
    else if (nav_age <= limit) { v = nav_v; w = nav_w; }
    tel_age++; nav_age++;
    v = fmaxf(-(float)max_v, fminf((float)max_v, v)); w = fmaxf(-(float)max_w, fminf((float)max_w, w));
    int tl = lin_pwm(v), ta = ang_pwm(w);
    if (tl == 1500) cur_lin = 1500; else cur_lin = toward(cur_lin == 1500 ? 1500 + (tl > 1500 ? deadband : -deadband) : cur_lin, tl, ramp);
    if (ta == 1500) cur_ang = 1500; else cur_ang = toward(cur_ang == 1500 ? 1500 + (ta > 1500 ? deadband : -deadband) : cur_ang, ta, ramp);
    send_pwm((int)fmax(1001, fmin(1999, cur_ang)), (int)fmax(1001, fmin(1999, cur_lin)));
  }
};

// ---- mission (the operator's side of a real run)
struct RealMission : ros::Node {
  const Hooks& h; MNavigator* nav; bool stop = false;
  RealMission(const Hooks& h_, MNavigator* n) : ros::Node("mission"), h(h_), nav(n) {
    create_service<std_srvs::srv::Trigger>("~/finish_mapping", [this](const std_srvs::srv::Trigger_Request&, std_srvs::srv::Trigger_Response& r, uint32_t) {
      if (nav->phase != PHASE_MAPPING) { r.success = false; r.message.set("mapping is already over"); return true; }
      if (!nav->map->last || !nav->slam_pose->last) { r.success = false; r.message.set("no map yet"); return true; }
      nav->manual_finish = true; r.success = true; r.message.set("mapping ends; amcl starts at the slam pose"); return true; });
    create_service<std_srvs::srv::Trigger>("~/stop", [this](const std_srvs::srv::Trigger_Request&, std_srvs::srv::Trigger_Response& r, uint32_t) {
      stop = true; r.success = true; r.message.set("the run ends"); return true; });
  }
};

// start pose: the middle of the map
inline Pose start_pose() { return {OX + W * RES / 2, OY + H * RES / 2, 0.0f}; }

// One real run. Each cycle waits for one lidar revolution. Returns when /mission/stop is called, after max_cycles,
// or when the lidar sends nothing for 10 cycles. mount_x, mount_yaw: the lidar's place on the robot.
inline void run_real(MWork& mw, const Hooks& h_in, uint8_t* revbuf, int revcap, int64_t max_cycles = 7LL * 3600,
                     void (*graph_out)(const char*) = nullptr) {
  static void (*log_out)(const char*); log_out = h_in.log;
  const Hooks h = {h_in.now_us, [](const char* s) { ros::rosout(rcl_interfaces::msg::Log::INFO, s); log_out(s); }};
  ros::sim_time() = true; ros::rosout_topic();
  Work& w = *mw.w; mw.world = 0;
  MResult R; R.world = 0; R.mode = MODE_SPIN; R.seed = 0; R.opts = 0;
  nav::Result stats;
  bus::hooks().now_ns = navn::sim_clock; navn::sim_ns() = 0;
  memset(w.occ_truth, 0, CELLS); bc::distance_field(w.occ_truth, w.dist_truth);   // (no simulated world)
  static navn::Uart uart;
  Pose start = start_pose();
  MNavigator* nav = new MNavigator(w, R, h, 0, MODE_SPIN, 0);
  nav->manual_mapping = true;
  RealMission* mission = new RealMission(h, nav);
  navn::X4Driver* drv = new navn::X4Driver(stats, h, uart);
  navn::Slam* slam = new navn::Slam(w, stats, h, start, true);
  navn::AmclNode* amcl = new navn::AmclNode(w, stats, h, 0, "/localization/status", LOC_LOST, LOC_BUSY, true);
  MMonitor* mon = new MMonitor(mw, R, h, MODE_SPIN);
  MGoalChecker* gc = new MGoalChecker();
  MPlanner* plan = new MPlanner(w, mw, R, h);
  MController* ctl = new MController(w, R, h);
  MBehavior* beh = new MBehavior(R);
  MCollisionMonitor* cm = new MCollisionMonitor(R);
  BaseDriver* base = new BaseDriver();
  ros::LifecycleManager* lm = new ros::LifecycleManager("lifecycle_manager_navigation", {"slam", "amcl", "localization_monitor",
      "controller_server", "planner_server", "behavior_server", "collision_monitor", "goal_checker", "bt_navigator"});
  if (!lm->startup()) logf_(h, "[r] lifecycle: not all nodes could be started");
  bus::NodeBase* extra = navn::run_hooks().extra_node ? navn::run_hooks().extra_node() : nullptr;   // bridge to the PC
  bus::Publisher<rosgraph_msgs::msg::Clock> clock_pub; clock_pub.t = bus::get_topic<rosgraph_msgs::msg::Clock>("/clock", 1, false); clock_pub.t->pubs++;
  logf_(h, "[r] real robot: map %.0f x %.0f m, start at its middle (%.1f, %.1f); mapping: drive with /teleop/cmd_vel, then call "
           "/mission/finish_mapping; goals: navigate_to_pose; end: /mission/stop", W * RES, H * RES, start.x, start.y);
  int missing = 0; int last_phase = PHASE_MAPPING; bool got_any = false;
  for (int64_t tick = 0; !mission->stop && tick < max_cycles; tick++) {
    int n = io().lidar_rev ? io().lidar_rev(revbuf, revcap) : 0;
    if (n <= 0) { if (got_any && ++missing >= 10) { logf_(h, "[r] no lidar revolution for 10 cycles: run stopped"); break; } }
    else { if (!got_any) logf_(h, "[r] first lidar revolution: %d bytes", n); missing = 0; got_any = true; }
    uart.bytes = revbuf; uart.n = n > 0 ? n : 0;
    navn::sim_ns() = tick * TICK_NS;
    { rosgraph_msgs::msg::Clock& c = clock_pub.loan(); c.clock.sec = (int32_t)(navn::sim_ns() / 1000000000); c.clock.nanosec = (uint32_t)(navn::sim_ns() % 1000000000); clock_pub.publish(); }
    bus::spin_once();
    if (nav->phase != last_phase) { last_phase = nav->phase; logf_(h, "[r] %s", last_phase == PHASE_NAVIGATING ? "mapping over: navigating (goals: navigate_to_pose)" : "done"); }
    if (navn::run_hooks().after_tick) navn::run_hooks().after_tick(tick);
  }
  logf_(h, "[r] run over: %d keyframes in the map, %d amcl updates, %d relocalizations, %d recoveries", stats.keyframes, stats.amcl_updates, R.n_ev, R.recoveries);
  if (graph_out) { bus::graph_print(graph_out); ros::graph_print(graph_out); }
  delete extra; delete lm; delete base; delete cm; delete beh; delete ctl; delete plan; delete gc; delete mon; delete amcl; delete slam; delete drv; delete mission; delete nav;
  bus::reset();
  ros::sim_time() = false;
}

}  // namespace realn
