// Lost-robot mission (firmware command [k]) split into nodes that talk only through topics (bus.h), with ROS 2 / Nav2
// names and message types. Same algorithms and same results as mis::run_mission in mission_core.h (checked by
// test_host/test_mission_nodes.cpp: every log line and every MISSION line except the timings).
//
//   node                   publishes                                         subscribes (main ones)
//   bt_navigator           /navigator/phase, /plan (mapping tour),           /controller_server/result, /goal_checker/reached,
//                          /initialpose, /goal_pose, /navigator/leg,         /localization/status, /behavior_server/result,
//                          /navigator/cycle (one behavior-tree tick)         /pose, /map, /ground_truth (leg log only)
//   x4_driver              /scan_cloud (/scan only with a subscriber)        /odom                    (+ X4 bytes from the "UART")
//   slam                   /map (latched), /pose, /tf                        /navigator/phase, /odom, /scan_cloud
//   amcl                   /amcl_pose, /amcl/robot_pose, /tf                 /map, /odom, /initialpose, /scan_cloud, /localization/status
//   localization_monitor   /localization/status, /initialpose, /cmd_vel      /map, /goal_pose, /odom, /amcl/robot_pose, /scan_cloud
//                          (turning in place), /localization/candidates
//   goal_checker           /goal_checker/reached                             /goal_pose, /localization/status, /amcl/robot_pose
//   planner_server         /plan, /planner_server/result                     /map, /goal_pose, /scan_cloud, /amcl/robot_pose, /navigator/cycle,
//                          (global costmap = SLAM map + live obstacle layer) /behavior_server/*, /controller_server/feedback, ...
//   controller_server      /cmd_vel_nav, /controller_server/result,          /map, /plan, /scan_cloud, /pose, /amcl/robot_pose, /amcl_pose, ...
//                          /controller_server/feedback (index on the path)
//   behavior_server        /cmd_vel_nav, /behavior_server/event,             /goal_pose, /localization/status, /planner_server/result,
//                          /behavior_server/active, /behavior_server/result  /controller_server/result, /amcl/robot_pose
//   collision_monitor      /cmd_vel                                          /cmd_vel_nav, /scan_cloud, /behavior_server/result
//   judge (simulation)     -                                                 /ground_truth, /pose, /amcl/robot_pose, /cmd_vel, ...
//   sim_base (simulation)  /odom, /ground_truth, /tf, /sim/kidnapped         /cmd_vel, /navigator/leg, /localization/status, ...
//
// Gazebo (gz, RunHooks::gz): the robot, the lidar and the world are in Gazebo on the PC (tools/gzbridge.py, through the
// bridge node). The PC publishes /odom, /ground_truth, /gz/scan (Gazebo's lidar), /gz/contacts and /gz/step each cycle;
//   gz_lidar (first node)  turns /gz/scan into X4 bytes for x4_driver (so the robot side parses them as on the robot)
//   sim_base               only decides the world events (unmapped box, kidnap) and asks the PC to make them
//                          (/sim/box, /sim/teleport), and publishes /gz/done when the cycle is over.
//
// The status and result topics are std_msgs/Int32 (codes below); in Nav2 most of them are action results and feedback
// (step 10 turns them into actions). One executor runs the nodes once per lidar revolution in the order above: a node
// gets in the same cycle what an earlier node published, and in the next cycle what a later node published.
// Cycle types (as in mis::run_mission): a normal cycle (one controller command, one robot step), a cycle that ends a leg
// (no step), and the relocalization cycles (only the localization monitor commands the robot; AMCL waits).
#pragma once
#include "mission_core.h"
#include "nav_nodes.h"
#include "ros.h"
#include <initializer_list>

namespace misn {
using namespace mis;
using bc::Pose;
using navn::TICK_NS; using navn::PHASE_MAPPING; using navn::PHASE_NAVIGATING; using navn::PHASE_DONE;
using navn::STATUS_SUCCEEDED; using navn::STATUS_ABORTED;
using navn::publish_int; using navn::publish_pose; using navn::occ_from_map; using navn::falloc; using navn::balloc;
using std_msgs::msg::Int32;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Path;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::LaserScan;

// /localization/status, one message per navigation cycle
enum { LOC_HEALTHY = 0,      // pose agrees with the map (beam check: < 10 % of beams see through a mapped wall)
       LOC_UNHEALTHY = 1,    // pose usable, but the live obstacle layer is not updated
       LOC_LOST = 2,         // 3 bad scans in a row: relocalization starts (AMCL waits, recoveries are cancelled)
       LOC_LOST_ABORT = 3,   // lost a 4th time in this leg: the leg is aborted
       LOC_BUSY = 4,         // relocalization running (global search, turning in place)
       LOC_RELOCALIZED = 5,  // new pose sent on /initialpose
       LOC_STOPPED = 6 };    // no unique pose found: stop and ask for help
inline bool loc_normal(int s) { return s == LOC_HEALTHY || s == LOC_UNHEALTHY; }
// /behavior_server/event
enum { BEH_STARTED = 1, BEH_FINISHED = 2 };
// /behavior_server/result
enum { BEH_ABORT_NO_PATH = 1, BEH_ABORT_NO_PROGRESS = 2 };

inline void release_all(std::initializer_list<void*> l) { for (void* p : l) bus::hooks().release(p); }
inline void send_twist(bus::Publisher<Twist>& p, float v, float wv) { Twist& t = p.loan(); t.linear.x = v; t.angular.z = wv; p.publish(); }
inline void publish_initialpose(bus::Publisher<PoseWithCovarianceStamped>& p, Pose x, float sxy, float sth) {
  PoseWithCovarianceStamped& ip = p.loan(); conv::header(ip.header, "map"); conv::to_msg(x, ip.pose.pose);
  for (double& c : ip.pose.covariance) c = 0;
  ip.pose.covariance[0] = ip.pose.covariance[7] = (double)sxy * (double)sxy; ip.pose.covariance[35] = (double)sth * (double)sth;   // sqrt gives sxy back exactly
  p.publish();
}
inline Pose truth_of(bus::Subscription<Odometry>* gt) { return conv::from_msg(gt->last->pose.pose); }

// --------------------------------------------------------------------------------------------- mission (the test)
// The mission of the test, as a ROS 2 user would run it: first the mapping tour as a NavigateThroughPoses goal, then
// one NavigateToPose goal per leg; logs each leg with its true distance (simulation). Not part of the robot software.
// The results come through the action client callback while bt_navigator ends the goal, so the next goal starts in the
// same cycle (as in mis::run_mission).
struct MMission : ros::Node {
  MResult& R; const Hooks& h; int world; Leg legs[8]; int NL = 0, li = -1; bool done = false, started = false;
  bool in_send = false, got = false; char got_msg[48] = {0};
  bool hold, stop = false;                                       // hold: after the legs, wait for goals from the PC
  bus::Subscription<Odometry>* gt; bus::Publisher<Int32> leg_pub, phase_pub;
  ros::ActionClient<nav2_msgs::action::NavigateThroughPoses> tour;
  ros::ActionClient<nav2_msgs::action::NavigateToPose> nav;
  MMission(MResult& R_, const Hooks& h_, int world_) : ros::Node("mission"), R(R_), h(h_), world(world_), hold(navn::run_hooks().hold) {
    NL = world_legs(world, legs); R.legs = NL;
    gt = create_subscription<Odometry>("/ground_truth", nullptr);
    leg_pub = create_publisher<Int32>("/navigator/leg", 1, true); phase_pub = create_publisher<Int32>("/navigator/phase", 1, true);
    snprintf(tour.name, sizeof tour.name, "/navigate_through_poses"); snprintf(nav.name, sizeof nav.name, "/navigate_to_pose");
    create_timer(TICK_NS, [this]() { if (!started) { started = true; send_tour(); } });
    create_service<std_srvs::srv::Trigger>("~/stop", [this](const std_srvs::srv::Trigger_Request&, std_srvs::srv::Trigger_Response& r, uint32_t) {
      stop = true; r.success = true; r.message.set(done ? "the run ends" : "the run ends after this cycle (legs not over)"); return true; });
  }
  bool over() const { return stop || (done && !hold); }
  void send_tour() {
    float WP[24][2]; int NW = world_tour(world, WP);
    auto* g = ros::alloc_msg<nav2_msgs::action::NavigateThroughPoses_Goal>();
    g->poses.size = NW;
    for (int i = 0; i < NW; i++) { conv::header(g->poses[i].header, "map"); conv::to_msg({WP[i][0], WP[i][1], 0.0f}, g->poses[i].pose); }
    tour.send_goal(*g, [this](int8_t st, const nav2_msgs::action::NavigateThroughPoses_Result&) { if (st == ros::GoalStatus::STATUS_SUCCEEDED) next_leg(0); });
    ros::free_msg(g);
  }
  void log_leg(int i, const char* state) {
    Leg& L = legs[i];
    if (L.reachable) R.legs_reachable++;
    if (!L.reachable && strncmp(state, "ABORT", 5) != 0) R.unreach_ok = false;
    if (L.reachable && strcmp(state, "DONE") == 0) R.legs_done++;
    Pose t = truth_of(gt);
    logf_(h, "[k]   leg %d %s (%.1f, %.1f): %s | true distance %.2f m", i + 1, L.name, L.x, L.y, state, hypotf(L.x - t.x, L.y - t.y));
  }
  // the next leg that runs; legs before it end at once (waiting for help, goal inside an obstacle)
  void next_leg(int i) {
    for (; i < NL; i++) {
      Leg& L = legs[i];
      if (R.need_help) { logf_(h, "[k]   leg %d %s: NOT_RUN (waiting for help)", i + 1, L.name); continue; }
      li = i; publish_int(leg_pub, li);
      nav2_msgs::action::NavigateToPose_Goal g; conv::header(g.pose.header, "map"); conv::to_msg({L.x, L.y, 0.0f}, g.pose.pose);
      got = false; in_send = true;
      nav.send_goal(g, [this](int8_t st, const nav2_msgs::action::NavigateToPose_Result& r) { on_result(st, r); });
      in_send = false;
      if (got) { log_leg(i, got_msg); continue; }               // ended at once
      return;
    }
    if (hold) { logf_(h, "[k]   legs over; holding: goals from the PC (navigate_to_pose) until /mission/stop"); done = true; return; }
    publish_int(phase_pub, PHASE_DONE); done = true;
  }
  void on_result(int8_t st, const nav2_msgs::action::NavigateToPose_Result& r) {
    const char* state = st == ros::GoalStatus::STATUS_SUCCEEDED ? "DONE" : r.error_msg.c_str();
    if (in_send) { got = true; snprintf(got_msg, sizeof got_msg, "%s", state); return; }
    log_leg(li, state); next_leg(li + 1);
  }
};

// --------------------------------------------------------------------------------------------- bt_navigator
// Action servers navigate_through_poses (the mapping tour: drive through the poses while SLAM maps) and
// navigate_to_pose (one leg). A goal ends SUCCEEDED, or ABORTED with the reason in error_msg (error_code stays 0):
// ABORT_GOAL_BLOCKED, ABORT_NO_PATH, ABORT_NO_PROGRESS, ABORT_TOO_MANY_RELOCS, STOPPED (waiting for help), TIMEOUT;
// a cancel request ends the leg CANCELED at the next cycle (the mapping tour cannot be canceled).
// Feedback every cycle while a goal runs (as Nav2's bt_navigator): current pose (AMCL; SLAM while mapping), time since
// the goal started, distance left along /plan (from the path point nearest to the robot), estimated time left (distance
// left at the average speed so far), recoveries in this goal; the tour also gives the number of poses left.
struct MNavigator : ros::Node {
  Work& w; MResult& R; const Hooks& h; int world, mode; uint32_t seed;
  int phase = PHASE_MAPPING;
  bool ev_tour = false, ev_reached = false; int ev_beh = 0, last_status = LOC_HEALTHY;
  int leg_cyc = 0; bool leg_active = false;
  bool manual_mapping = false, manual_finish = false;            // real robot (real_nodes.h): mapping ends on request
  double& leg_timeout;
  uint8_t *occ, *dist, *cost; bool map_dirty = false;
  bus::Subscription<PoseWithCovarianceStamped>* slam_pose; bus::Subscription<OccupancyGrid>* map;
  bus::Publisher<Int32> phase_pub, cycle_pub; bus::Publisher<Path> tour_pub; bus::Publisher<PoseStamped> goal_pub;
  bus::Publisher<PoseWithCovarianceStamped> init_pub;
  ros::ActionServer<nav2_msgs::action::NavigateThroughPoses> tour;
  ros::ActionServer<nav2_msgs::action::NavigateToPose> nav;
  // feedback (read only: the results are the same with and without it)
  bus::Subscription<PoseWithCovarianceStamped>* amcl_pose; bus::Subscription<Path>* plan;
  uint32_t plan_seq = 0, plan_old = 0; int plan_i = 0, fb_cyc = 0, rec0 = 0, tour_wp[24], tour_nw = 0; float fb_d0 = 0;
  nav2_msgs::action::NavigateToPose_Feedback* fb; nav2_msgs::action::NavigateThroughPoses_Feedback* fbt;
  MNavigator(Work& w_, MResult& R_, const Hooks& h_, int world_, int mode_, uint32_t seed_) : ros::Node("bt_navigator", true), w(w_), R(R_), h(h_),
      world(world_), mode(mode_), seed(seed_),
      leg_timeout(declare_parameter("leg_timeout", 150.0, "s; a goal not reached by then ends with TIMEOUT", 1.0, 3600.0)),
      occ(balloc(CELLS)), dist(balloc(CELLS)), cost(balloc(CELLS)), tour(this, "/navigate_through_poses"), nav(this, "/navigate_to_pose") {
    phase_pub = create_publisher<Int32>("/navigator/phase", 1, true); tour_pub = create_publisher<Path>("/plan");
    init_pub = create_publisher<PoseWithCovarianceStamped>("/initialpose"); goal_pub = create_publisher<PoseStamped>("/goal_pose");
    cycle_pub = create_publisher<Int32>("/navigator/cycle");
    create_subscription<Int32>("/controller_server/result", [this](const Int32& m) { if (m.data == STATUS_SUCCEEDED && phase == PHASE_MAPPING) ev_tour = true; });
    create_subscription<Int32>("/goal_checker/reached", [this](const Int32&) { ev_reached = true; });
    create_subscription<Int32>("/localization/status", [this](const Int32& m) { last_status = m.data; });
    create_subscription<Int32>("/behavior_server/result", [this](const Int32& m) { ev_beh = m.data; });
    map = create_subscription<OccupancyGrid>("/map", [this](const OccupancyGrid&) { map_dirty = true; });
    slam_pose = create_subscription<PoseWithCovarianceStamped>("/pose", nullptr);
    amcl_pose = create_subscription<PoseWithCovarianceStamped>("/amcl_pose", nullptr);
    plan = create_subscription<Path>("/plan", nullptr);
    fb = ros::alloc_msg<nav2_msgs::action::NavigateToPose_Feedback>(); fbt = ros::alloc_msg<nav2_msgs::action::NavigateThroughPoses_Feedback>();
    create_timer(TICK_NS, [this]() { tick(); feedback(); });
    tour.handle_accepted = [this](const nav2_msgs::action::NavigateThroughPoses_Goal& g) { start_mapping(g); };
    nav.handle_accepted = [this](const nav2_msgs::action::NavigateToPose_Goal& g) { start_leg(g); };
    nav.handle_cancel = [this]() { return leg_active; };          // the leg ends CANCELED at the next cycle
    tour.handle_cancel = []() { return false; };                  // the mapping tour runs to its end
  }
  ~MNavigator() { release_all({occ, dist, cost}); ros::free_msg(fb); ros::free_msg(fbt); }
  // distance left along /plan from the point nearest to p (searched forward from the last one); -1 without a plan
  // for this goal yet (plans published before the goal started are older ones)
  float left_on_plan(Pose p) {
    if (!plan->last || plan->t->seq <= plan_old) return -1;
    const Path& pa = *plan->last; int n = (int)pa.poses.size; if (n == 0) return -1;
    int from = plan_i, to = plan_i + 200;
    if (plan->t->seq != plan_seq) { plan_seq = plan->t->seq; from = 0; to = n; }  // a new path: search all of it
    if (to > n) to = n;
    if (from >= n) from = n - 1;
    float best = 1e9f; int bi = from;
    for (int i = from; i < to; i++) {
      float d = hypotf((float)pa.poses[i].pose.position.x - p.x, (float)pa.poses[i].pose.position.y - p.y);
      if (d < best) { best = d; bi = i; }
    }
    plan_i = bi; float left = best;
    for (int i = bi; i + 1 < n; i++)
      left += hypotf((float)(pa.poses[i + 1].pose.position.x - pa.poses[i].pose.position.x), (float)(pa.poses[i + 1].pose.position.y - pa.poses[i].pose.position.y));
    return left;
  }
  template <typename F> void fill(F& f, const PoseWithCovarianceStamped* src, float left, int cyc) {
    conv::header(f.current_pose.header, "map"); f.current_pose.pose = src->pose.pose;
    double t = cyc * DT; f.navigation_time.sec = (int32_t)t; f.navigation_time.nanosec = (uint32_t)((t - (int32_t)t) * 1e9);
    if (fb_d0 <= 0 && left > 0) fb_d0 = left;
    float done_m = fb_d0 - left; double eta = (done_m > 0.1f && t > 1.0) ? left * t / done_m : 0;
    f.estimated_time_remaining.sec = (int32_t)eta; f.estimated_time_remaining.nanosec = (uint32_t)((eta - (int32_t)eta) * 1e9);
    f.number_of_recoveries = (int16_t)(R.recoveries - rec0); f.distance_remaining = left > 0 ? left : 0;
  }
  void feedback() {
    if (phase == PHASE_MAPPING && tour.running() && slam_pose->last) {
      Pose p = conv::from_msg(slam_pose->last->pose.pose); float left = left_on_plan(p);
      fill(*fbt, slam_pose->last, left, fb_cyc++);
      int k = 0; for (int i = 0; i < tour_nw; i++) if (tour_wp[i] > plan_i) k++;
      fbt->number_of_poses_remaining = (int16_t)k;
      tour.publish_feedback(*fbt);
    } else if (phase == PHASE_NAVIGATING && leg_active && nav.running() && amcl_pose->last) {
      Pose p = conv::from_msg(amcl_pose->last->pose.pose); float left = left_on_plan(p);
      if (left < 0) { Pose g = conv::from_msg(nav.goal->pose.pose); left = hypotf(g.x - p.x, g.y - p.y); }
      fill(*fb, amcl_pose->last, left, fb_cyc++);
      nav.publish_feedback(*fb);
    }
  }
  void start_mapping(const nav2_msgs::action::NavigateThroughPoses_Goal& goal) {
    int NW = (int)goal.poses.size; float WP[24][2];
    for (int i = 0; i < NW; i++) { Pose q = conv::from_msg(goal.poses[i].pose); WP[i][0] = q.x; WP[i][1] = q.y; }
    Path& p = tour_pub.loan(); conv::header(p.header, "map"); int np = 0;
    auto add = [&](float x, float y) { if (np < (int)p.poses.capacity) { conv::header(p.poses[np].header, "map"); conv::to_msg({x, y, 0.0f}, p.poses[np].pose); np++; } };
    for (int i = 0; i + 1 < NW; i++) {
      float L = hypotf(WP[i + 1][0] - WP[i][0], WP[i + 1][1] - WP[i][1]); int k = (int)(L / 0.05f);
      for (int j = 0; j < k; j++) add(WP[i][0] + (WP[i + 1][0] - WP[i][0]) * j / k, WP[i][1] + (WP[i + 1][1] - WP[i][1]) * j / k);
    }
    add(WP[NW - 1][0], WP[NW - 1][1]);
    p.poses.size = np; tour_pub.publish();
    tour_nw = NW; for (int i = 0, acc = 0; i < NW; i++) { tour_wp[i] = acc; if (i + 1 < NW) acc += (int)(hypotf(WP[i + 1][0] - WP[i][0], WP[i + 1][1] - WP[i][1]) / 0.05f); }
    fb_cyc = 0; fb_d0 = 0; rec0 = R.recoveries; plan_seq = 0; plan_old = plan->t->seq - 1;   // (the tour is the plan)
    phase = PHASE_MAPPING; publish_int(phase_pub, PHASE_MAPPING);
  }
  void finish_mapping() {
    // costmap of the finished map with planning padding (only to refuse goals inside an obstacle without driving)
    occ_from_map(*map->last, occ); bc::distance_field(occ, dist); bc::costmap(dist, cost); pad_cost(dist, cost); map_dirty = false;
    phase = PHASE_NAVIGATING; publish_int(phase_pub, PHASE_NAVIGATING);
    publish_initialpose(init_pub, conv::from_msg(slam_pose->last->pose.pose), 0.05f, 0.03f);
    nav2_msgs::action::NavigateThroughPoses_Result r; if (tour.running()) tour.succeed(r);   // the mission sends its first leg now
  }
  void start_leg(const nav2_msgs::action::NavigateToPose_Goal& g) {
    Pose q = conv::from_msg(g.pose.pose);
    if (cost[bc::cy(q.y) * W + bc::cx(q.x)] >= 253) { end_leg("ABORT_GOAL_BLOCKED"); return; }
    leg_active = true; leg_cyc = 0; last_status = LOC_HEALTHY;
    fb_cyc = 0; fb_d0 = 0; rec0 = R.recoveries; plan_seq = 0; plan_old = plan->t->seq;
    PoseStamped& gp = goal_pub.loan(); conv::header(gp.header, "map"); conv::to_msg({q.x, q.y, 0.0f}, gp.pose); goal_pub.publish();
    publish_int(cycle_pub, leg_cyc++);
  }
  void end_leg(const char* why) {                                // the mission may send the next goal during this call
    leg_active = false;
    nav2_msgs::action::NavigateToPose_Result r;
    if (strcmp(why, "DONE") == 0) nav.succeed(r); else { r.error_msg.set(why); nav.abort(r); }
  }
  void tick() {
    if (phase == PHASE_MAPPING) { if ((ev_tour && !manual_mapping) || manual_finish) finish_mapping(); ev_tour = manual_finish = false; return; }
    if (phase != PHASE_NAVIGATING || !leg_active) return;
    if (nav.is_canceling()) { leg_active = false; nav2_msgs::action::NavigateToPose_Result r; r.error_msg.set("CANCELED"); nav.canceled(r); return; }
    const char* end = nullptr;
    if (ev_reached) end = "DONE";
    else if (last_status == LOC_LOST_ABORT) end = "ABORT_TOO_MANY_RELOCS";
    else if (last_status == LOC_STOPPED) end = "STOPPED";
    else if (ev_beh == BEH_ABORT_NO_PATH) end = "ABORT_NO_PATH";
    else if (ev_beh == BEH_ABORT_NO_PROGRESS) end = "ABORT_NO_PROGRESS";
    ev_reached = false; ev_beh = 0;
    if (end) { end_leg(end); return; }
    if (last_status == LOC_LOST || last_status == LOC_BUSY) return;      // the relocalization goes on in this cycle
    if (leg_cyc == (int)lround(leg_timeout * 7)) { end_leg("TIMEOUT"); return; }
    publish_int(cycle_pub, leg_cyc++);
  }
};

// --------------------------------------------------------------------------------------------- localization_monitor
// Beam check of the AMCL pose against the map; spin mode: 3 bad scans in a row -> global search over all free poses,
// one candidate: use it; several: turn in place and drop candidates by decisive beams; none or still several: stop.
// The search is spread over cycles so that no cycle takes long (the robot stands still meanwhile): pass 1 in
// search_slices cycles, sorting in one, then search_reps_per_cycle representatives per cycle. The work per cycle is fixed
// (not timed), so the PC and the chip get the same results. search_slices 1: all in one cycle (as the single loop).
// On the chip (1.3 s for the whole search) the defaults 12 and 2 give cycles of about 0.1 s.
// While turning, each check of the candidates (every 0.15 rad or m) uses the scan and odometry of the cycle it starts in
// and does relocalization_hyps_per_cycle candidates per cycle (0: all at once); a candidate checked later is moved by the
// odometry since then. On the chip one candidate takes about 27 ms; the default 4 keeps a cycle near 0.1 s.
struct MMonitor : ros::Node {
  MWork& mw; MResult& R; const Hooks& h; int mode;
  Work lw; MWork lmw;                                            // the search works on this node's own map fields
  uint8_t *occ, *dist, *lik; float *px, *py; bool map_dirty = false, active = false, has_e = false;
  enum { IDLE, SEARCH, TURN } state = IDLE;
  int low = 0, n_loc = 0, att = 0, spins = 0, nh = 0, evi = 0, steps = 0; Pose e, last_o;
  Pose hyp[MAXHYP]; float key[MAXHYP]; bool dead[MAXHYP]; int streak[MAXHYP]; float acc = 0, t_m = 0, t_loc0 = 0; double sm = 0;
  Search* srch; float *spx, *spy; int sn = 0, scyc = 0; bool searching = false; int64_t &slices, &reps_per, &hyps_per;
  bool upd = false; int upd_k = 0, upd_m = 0, upd_nb = 0, nlong[MAXHYP]; Pose upd_od, snap[MAXHYP];   // a check of the candidates in slices
  bus::Subscription<OccupancyGrid>* map; bus::Subscription<Odometry> *odom, *gt;
  bus::Publisher<Int32> status_pub; bus::Publisher<PoseWithCovarianceStamped> init_pub; bus::Publisher<Twist> cmd_pub; bus::Publisher<PoseArray> cand_pub;
  MMonitor(MWork& mw_, MResult& R_, const Hooks& h_, int mode_) : ros::Node("localization_monitor", true), mw(mw_), R(R_), h(h_), mode(mode_),
      occ(balloc(CELLS)), dist(balloc(CELLS)), lik((uint8_t*)bus::hooks().alloc_fast(CELLS)), px(falloc(MAXPTS)), py(falloc(MAXPTS)),
      srch((Search*)bus::hooks().alloc(sizeof(Search))), spx(falloc(MAXPTS)), spy(falloc(MAXPTS)),
      slices(declare_parameter("search_slices", (int64_t)navn::run_hooks().search_slices, "cycles for pass 1 of a global search (1: the whole search in one cycle)", 1, 64)),
      reps_per(declare_parameter("search_reps_per_cycle", (int64_t)2, "global search: candidates refined per cycle", 1, 24)),
      hyps_per(declare_parameter("relocalization_hyps_per_cycle", (int64_t)navn::run_hooks().turn_hyps, "candidates checked per cycle while turning (0: all)", 0, 24)) {
    lw = *mw.w; lw.occ = occ; lw.dist = dist; lw.lik = lik;     // idx (scratch) is shared with slam, which is idle by then
    lmw = mw; lmw.w = &lw;
    status_pub = create_publisher<Int32>("/localization/status"); init_pub = create_publisher<PoseWithCovarianceStamped>("/initialpose");
    cmd_pub = create_publisher<Twist>("/cmd_vel"); cand_pub = create_publisher<PoseArray>("/localization/candidates");
    create_subscription<Twist>("/cmd_vel", [this](const Twist&) { steps++; });            // robot steps (relocalization time)
    map = create_subscription<OccupancyGrid>("/map", [this](const OccupancyGrid&) { map_dirty = true; });
    create_subscription<Int32>("/navigator/phase", [this](const Int32& m) { active = false; if (m.data == PHASE_NAVIGATING) steps = 0; });
    create_subscription<PoseStamped>("/goal_pose", [this](const PoseStamped&) { active = true; n_loc = 0; });
    odom = create_subscription<Odometry>("/odom", nullptr);
    gt = create_subscription<Odometry>("/ground_truth", nullptr);  // simulation only: error of the chosen pose in the result
    create_subscription<PoseStamped>("/amcl/robot_pose", [this](const PoseStamped& p) { e = conv::from_msg(p.pose); has_e = true; });
    create_subscription<PointCloud2>("/scan_cloud", [this](const PointCloud2& c) { on_scan(c); });
  }
  ~MMonitor() { release_all({occ, dist, lik, px, py, spx, spy}); bus::hooks().release(srch); }
  void status(int s) { publish_int(status_pub, s); }
  void candidates(const Pose* p, const bool* alive, int n) {
    PoseArray& a = cand_pub.loan(); conv::header(a.header, "map"); a.poses.size = 0;
    for (int k = 0; k < n && a.poses.size < a.poses.capacity; k++) if (!alive || alive[k]) conv::to_msg(p[k], a.poses[a.poses.size++]);
    cand_pub.publish();
  }
  Event& ev() { return R.ev[evi]; }
  void relocalized(Pose p) { publish_initialpose(init_pub, p, 0.05f, 0.03f); status(LOC_RELOCALIZED); state = IDLE; }
  void stopped() { R.need_help = true; ev().took_s = steps * DT - t_loc0; status(LOC_STOPPED); state = IDLE; }
  void on_scan(const PointCloud2& c) {
    if (!active) return;
    if (map_dirty && !searching) { occ_from_map(*map->last, occ); bc::distance_field(occ, dist); bc::likelihood_field(dist, lik); map_dirty = false; }
    int m = conv::from_cloud(c, px, py, MAXPTS);
    if (state == IDLE) {
      if (!has_e) return;
      has_e = false;
      float lf, mf; beam_check(occ, e, px, py, m, 8, lf, mf);
      if (mode == MODE_SPIN) {
        low = lf > 0.25f ? low + 1 : 0;
        if (low >= 3) {
          low = 0; n_loc++;
          if (n_loc > 3) { status(LOC_LOST_ABORT); return; }
          evi = R.n_ev < 4 ? R.n_ev : 3; if (R.n_ev < 4) R.n_ev++;
          memset(&ev(), 0, sizeof(Event));
          t_loc0 = steps * DT; sm = 0; att = 0; spins = 0; nh = 0; searching = false; upd = false; state = SEARCH;
          status(LOC_LOST);                                      // this cycle ends here: the next scan is taken standing still
          return;
        }
      }
      status(lf < 0.1f ? LOC_HEALTHY : LOC_UNHEALTHY);
      return;
    }
    Pose truth = gt->last ? truth_of(gt) : e;                    // (real robot: no ground truth)
    if (state == SEARCH) {
      if (spins > 0) { spins--; send_twist(cmd_pub, 0.0f, 0.8f); status(LOC_BUSY); return; }
      if (att == 5) {                                            // 5 searches, 5 x 5 cycles of turning: nothing found
        if (sm > R.search_ms_max) R.search_ms_max = sm;
        ev().n_hyp = 0; ev().result = EV_NOT_FOUND; stopped(); return;
      }
      if (!searching) {                                          // a new search on this scan (kept until the search ends)
        memcpy(spx, px, m * sizeof(float)); memcpy(spy, py, m * sizeof(float)); sn = m;
        search_begin(*srch, spx, spy, sn); searching = true; scyc = 0;
      }
      int64_t s0 = h.now_us(); bool done;
      if (slices <= 1) { while (!(done = search_step(lmw, *srch, 1 << 20, 1 << 20))) {} }
      else done = search_step(lmw, *srch, (search_rows() + (int)slices - 1) / (int)slices, (int)reps_per);
      double dms = (h.now_us() - s0) / 1000.0; sm += dms; scyc++;
      if (dms > R.search_slice_ms_max) R.search_slice_ms_max = dms;
      if (scyc > R.search_cycles_max) R.search_cycles_max = scyc;
      if (!done) { status(LOC_BUSY); return; }                   // standing still while the search goes on
      searching = false; att++;
      nh = srch->ng; for (int g = 0; g < nh; g++) { hyp[g] = srch->out[g]; key[g] = srch->key[g]; }
      m = sn; memcpy(px, spx, m * sizeof(float)); memcpy(py, spy, m * sizeof(float));   // (the scan of the search)
      if (nh == 0) { spins = 4; send_twist(cmd_pub, 0.0f, 0.8f); status(LOC_BUSY); return; }   // turn 5 cycles, search again
      if (sm > R.search_ms_max) R.search_ms_max = sm;
      ev().n_hyp = nh; candidates(hyp, nullptr, nh);
      if (nh == 1) {
        ev().result = EV_UNIQUE; ev().n_alive = 1; ev().err = hypotf(hyp[0].x - truth.x, hyp[0].y - truth.y); ev().took_s = steps * DT - t_loc0;
        logf_(h, "[k]   relocalized: unique candidate, error %.2f m (search %.0f ms)", ev().err, sm);
        relocalized(hyp[0]); return;
      }
      for (int k = 0; k < nh; k++) { dead[k] = false; streak[k] = 0; }
      acc = 0; t_m = 0; upd = false; last_o = conv::from_msg(odom->last->pose.pose); state = TURN;
      send_twist(cmd_pub, 0.0f, 0.6f); status(LOC_BUSY); return;
    }
    // TURN: one step done since the last scan
    Pose od = conv::from_msg(odom->last->pose.pose);
    Pose dl = ominus(last_o, od); last_o = od;
    for (int k = 0; k < nh; k++) hyp[k] = oplus(hyp[k], dl);
    acc += fabsf(dl.th) + hypotf(dl.x, dl.y);
    float* meas = mw.meas; float (*ex)[MAXPTS / 2] = (float (*)[MAXPTS / 2])mw.ex;
    if (!upd && acc > 0.15f) {                                   // a new check: this cycle's scan and odometry
      acc = 0; upd = true; upd_k = 0; upd_od = od; upd_m = m;
      memcpy(spx, px, m * sizeof(float)); memcpy(spy, py, m * sizeof(float));
      for (int k = 0; k < nh; k++) snap[k] = hyp[k];
    }
    if (upd) {
      int64_t u0 = h.now_us();
      const int mm = upd_m; int nb = 0; for (int k = 0; k < mm && nb < MAXPTS / 2; k += 2) { meas[nb++] = hypotf(spx[k], spy[k]); }
      upd_nb = nb;
      Pose d = ominus(upd_od, od);                               // odometry since the check started (none: all in one cycle)
      int per = hyps_per > 0 ? (int)hyps_per : MAXHYP, did = 0;
      for (; upd_k < nh && did < per; upd_k++) {
        const int k = upd_k; if (dead[k]) continue;
        did++;
        Pose r = refine(lmw, snap[k], spx, spy, mm);
        nlong[k] = 0; int b2 = 0;
        for (int q = 0; q < mm && b2 < nb; q += 2, b2++) {
          float e2 = ray_occ(occ, r.x, r.y, r.th + atan2f(spy[q], spx[q]));
          ex[k][b2] = e2; if (meas[b2] > e2 + tol_far(e2)) nlong[k]++;
        }
        hyp[k] = per >= MAXHYP ? r : oplus(r, d);
      }
      double um = (h.now_us() - u0) / 1000.0; if (um > R.multi_update_ms_max) R.multi_update_ms_max = um;
    }
    if (upd && upd_k >= nh) {                                    // all candidates checked: drop the ones decisive beams rule out
      upd = false; const int nb = upd_nb;
      int b = -1; for (int k = 0; k < nh; k++) if (!dead[k] && (b < 0 || nlong[k] < nlong[b])) b = k;
      for (int k = 0; k < nh; k++) if (!dead[k] && k != b) {
        int dec = 0;
        for (int q = 0; q < nb; q++) {
          float eb = ex[b][q], ek = ex[k][q];
          if (fabsf(ek - eb) > 0.4f && fabsf(meas[q] - eb) < tol_far(eb) && meas[q] > ek + tol_far(ek)) dec++;
        }
        streak[k] = dec >= 6 ? streak[k] + 1 : 0;
        if (streak[k] >= 3) dead[k] = true;
      }
      int alive = 0, last = -1; for (int k = 0; k < nh; k++) if (!dead[k]) { alive++; last = k; }
      if (alive == 1) {
        ev().took_s = steps * DT - t_loc0;
        ev().result = EV_RESOLVED; ev().n_alive = 1; ev().err = hypotf(hyp[last].x - truth.x, hyp[last].y - truth.y);
        candidates(&hyp[last], nullptr, 1);
        logf_(h, "[k]   relocalized: %d candidates, resolved by turning, error %.2f m (search %.0f ms)", nh, ev().err, sm);
        relocalized(hyp[last]); return;
      }
    }
    t_m += DT;
    if (t_m < 15.0f) { send_twist(cmd_pub, 0.0f, 0.6f); status(LOC_BUSY); return; }
    Event& v = ev(); v.result = EV_AMBIGUOUS_STOPPED; int na = 0; float mn = 9;
    for (int k = 0; k < nh; k++) if (!dead[k]) { float er = hypotf(hyp[k].x - truth.x, hyp[k].y - truth.y); v.hyp_err[na++] = er; if (er < mn) mn = er; }
    v.n_alive = na; v.truth_kept = mn < 0.3f;
    bool alive[MAXHYP]; for (int k = 0; k < nh; k++) alive[k] = !dead[k];
    candidates(hyp, alive, nh);
    logf_(h, "[k]   still %d candidates after turning 15 s (true pose among them: %s): stop and ask for help", na, v.truth_kept ? "yes" : "no");
    stopped();
  }
};

// --------------------------------------------------------------------------------------------- goal_checker
struct MGoalChecker : ros::Node {
  bool active = false; float gx = 0, gy = 0; int st = LOC_HEALTHY; double& xy_tol;
  bus::Publisher<Int32> reached_pub;
  MGoalChecker() : ros::Node("goal_checker", true),
      xy_tol(declare_parameter("xy_goal_tolerance", 0.2, "m; goal reached within this distance", 0.01, 2.0)) {
    reached_pub = create_publisher<Int32>("/goal_checker/reached");
    create_subscription<PoseStamped>("/goal_pose", [this](const PoseStamped& g) { gx = (float)g.pose.position.x; gy = (float)g.pose.position.y; active = true; });
    create_subscription<Int32>("/localization/status", [this](const Int32& m) { st = m.data; });
    create_subscription<PoseStamped>("/amcl/robot_pose", [this](const PoseStamped& p) {
      Pose e = conv::from_msg(p.pose);
      if (active && loc_normal(st) && hypotf(gx - e.x, gy - e.y) < (float)xy_tol) { active = false; publish_int(reached_pub, STATUS_SUCCEEDED); }
    });
  }
};

// --------------------------------------------------------------------------------------------- planner_server
// Global costmap = SLAM map with planning padding + live obstacle layer (scan points not in the map, marked while the
// pose is healthy). Plans at once when there is no path, otherwise once a second (behavior-tree cycle % 7); a live
// obstacle on the next 40 path points drops the path (Nav2 IsPathValid).
struct MPlanner : ros::Node {
  Work& w; MWork& mw; MResult& R; const Hooks& h;
  uint8_t *occ, *dist, *cost; float *X, *Y, *px, *py; int m = 0;
  bool active = false, map_dirty = false, live_any = false, reached = false, rec_active = false, path_changed = false;
  int gcell = 0, path_n = 0, pidx = 0, cyc = 0, st = LOC_HEALTHY;
  bus::Publisher<Path> plan_pub; bus::Publisher<Int32> result_pub; bus::Subscription<OccupancyGrid>* map;
  MPlanner(Work& w_, MWork& mw_, MResult& R_, const Hooks& h_) : ros::Node("planner_server", true), w(w_), mw(mw_), R(R_), h(h_),
      occ(balloc(CELLS)), dist(balloc(CELLS)), cost(balloc(CELLS)), X(falloc(PATH_CAP)), Y(falloc(PATH_CAP)), px(falloc(MAXPTS)), py(falloc(MAXPTS)) {
    memset(mw.live, 0, CELLS);
    plan_pub = create_publisher<Path>("/plan"); result_pub = create_publisher<Int32>("/planner_server/result");
    map = create_subscription<OccupancyGrid>("/map", [this](const OccupancyGrid&) { map_dirty = true; });
    create_subscription<PoseStamped>("/goal_pose", [this](const PoseStamped& g) {
      gcell = bc::cy((float)g.pose.position.y) * W + bc::cx((float)g.pose.position.x); active = true; drop(); rec_active = false; });
    create_subscription<Int32>("/behavior_server/event", [this](const Int32& m) { if (m.data == BEH_STARTED) live_clear(); drop(); }, 4);
    create_subscription<PoseWithCovarianceStamped>("/initialpose", [this](const PoseWithCovarianceStamped&) { drop(); });
    create_subscription<Int32>("/behavior_server/active", [this](const Int32& m) { rec_active = m.data > 0; });
    create_subscription<Int32>("/controller_server/feedback", [this](const Int32& m) { pidx = m.data; });
    create_subscription<Int32>("/navigator/cycle", [this](const Int32& m) { cyc = m.data; });
    create_subscription<Int32>("/localization/status", [this](const Int32& m) { st = m.data; if (st == LOC_LOST) { live_clear(); rec_active = false; } });
    create_subscription<Int32>("/goal_checker/reached", [this](const Int32&) { reached = true; });
    create_subscription<PointCloud2>("/scan_cloud", [this](const PointCloud2& c) { m = conv::from_cloud(c, px, py, MAXPTS); });
    create_subscription<PoseStamped>("/amcl/robot_pose", [this](const PoseStamped& p) { on_pose(conv::from_msg(p.pose)); });
  }
  ~MPlanner() { release_all({occ, dist, cost, X, Y, px, py}); }
  void drop() { if (path_n) path_changed = true; path_n = 0; pidx = 0; }
  void live_clear() { if (live_any) { memset(mw.live, 0, CELLS); live_any = false; } }
  void live_fields() {
    bc::distance_field(mw.live, mw.live_dist);                                      // cells x 10 to the nearest live cell
    for (int k = 0; k < CELLS; k++) mw.occ_plan[k] = (occ[k] || mw.live_dist[k] <= 14) ? 1 : 0;   // live grown by 1 cell
    bc::distance_field(mw.occ_plan, mw.dist_plan); bc::costmap(mw.dist_plan, mw.cost_plan); pad_cost(mw.dist_plan, mw.cost_plan);
  }
  void publish_path() {
    Path& pm = plan_pub.loan(); conv::header(pm.header, "map");
    int n = path_n; if (n > (int)pm.poses.capacity) { n = (int)pm.poses.capacity; logf_(h, "planner_server: path cut to %d points", n); }
    pm.poses.size = n;
    for (int i = 0; i < n; i++) { conv::header(pm.poses[i].header, "map"); conv::to_msg({X[i], Y[i], 0.0f}, pm.poses[i].pose); }
    plan_pub.publish(); path_changed = false;
  }
  void on_pose(Pose e) {
    bool skip = !active || reached || !loc_normal(st); reached = false;
    if (skip) return;
    if (map_dirty) { occ_from_map(*map->last, occ); bc::distance_field(occ, dist); bc::costmap(dist, cost); pad_cost(dist, cost); map_dirty = false; }
    if (st == LOC_HEALTHY) {                                       // live obstacle layer
      float c = fcos(e.th), sn = fsin(e.th); static int idxs[MAXPTS]; int nn = 0;
      for (int k = 0; k < m; k++) {
        if (px[k] * px[k] + py[k] * py[k] > 1.5f * 1.5f) continue;
        int i = bc::cx(e.x + c * px[k] - sn * py[k]), j = bc::cy(e.y + sn * px[k] + c * py[k]);
        if (i < 0 || i >= W || j < 0 || j >= H) continue;
        if (dist[j * W + i] / 10.0f * RES > 0.12f) idxs[nn++] = j * W + i;
      }
      if (nn > 3) {
        int added = 0; for (int k = 0; k < nn; k++) if (!mw.live[idxs[k]]) { mw.live[idxs[k]] = 1; added++; }
        if (added) {
          live_any = true; live_fields();
          for (int q = pidx; q < path_n && q < pidx + 40; q++) {
            int i = bc::cx(X[q]), j = bc::cy(Y[q]);
            if (i >= 0 && i < W && j >= 0 && j < H && mw.live_dist[j * W + i] <= 40) { drop(); R.obstacle_replans++; break; }
          }
        }
      }
    }
    if (!rec_active && (path_n == 0 || cyc % 7 == 0)) {
      int s = bc::cy(e.y) * W + bc::cx(e.x);
      if (s >= 0 && s < CELLS) {
        uint8_t* cs = live_any ? mw.cost_plan : cost;
        // let the robot leave a lethal cell it believes it is in (navsim.py: cost around the start capped at 200)
        uint8_t saved[25]; int si = s % W, sj = s / W, q = 0;
        for (int dj = -2; dj <= 2; dj++) for (int di = -2; di <= 2; di++) {
          int c = (sj + dj) * W + (si + di); if (c < 0 || c >= CELLS) { saved[q++] = 0; continue; }
          saved[q++] = cs[c]; if (cs[c] > 200) cs[c] = 200; }
        bc::AStarWork wk = {w.g, w.heap, w.pos, w.state};
        bc::AStarResult ar = cs[gcell] >= 253 ? bc::AStarResult{false, 0, 0} : bc::astar_opt(cs, s, gcell, wk, w.f, 6.0f);
        q = 0; for (int dj = -2; dj <= 2; dj++) for (int di = -2; di <= 2; di++) { int c = (sj + dj) * W + (si + di); if (c >= 0 && c < CELLS) cs[c] = saved[q]; q++; }
        if (ar.found) { path_n = extract_path(w.g, w.state, s, gcell, X, Y, PATH_CAP); pidx = 0; path_changed = true; }
        else { drop(); publish_int(result_pub, STATUS_ABORTED); }
      }
    }
    if (path_changed) publish_path();
  }
};

// --------------------------------------------------------------------------------------------- controller_server
// Mapping: drives the fixed tour (pure pursuit) and reports its end. Navigation: collision-checked pure pursuit with a
// DWB-like fallback, and the progress checker (0.5 m in 10 s).
struct MController : ros::Node {
  Work& w; MResult& R; const Hooks& h; double& max_vel_x; uint8_t *occ, *dist; float *px, *py; int m = 0;
  int phase = PHASE_MAPPING, n = 0, idx = 0, tour_calls = 0, st = LOC_HEALTHY; bool active = false, reached = false, rec_active = false, map_dirty = false;
  Pose anchor, amcl_last, e_last; float t_anchor = 0;
  bus::Publisher<Twist> cmd_pub; bus::Publisher<Int32> result_pub, fb_pub; bus::Subscription<OccupancyGrid>* map;
  MController(Work& w_, MResult& R_, const Hooks& h_) : ros::Node("controller_server", true), w(w_), R(R_), h(h_),
      max_vel_x(declare_parameter("max_vel_x", 0.3, "m/s, forward speed limit", 0.0, 1.0)),
      occ(balloc(CELLS)), dist(balloc(CELLS)), px(falloc(MAXPTS)), py(falloc(MAXPTS)) {
    cmd_pub = create_publisher<Twist>("/cmd_vel_nav"); result_pub = create_publisher<Int32>("/controller_server/result");
    fb_pub = create_publisher<Int32>("/controller_server/feedback");
    map = create_subscription<OccupancyGrid>("/map", [this](const OccupancyGrid&) { map_dirty = true; });
    create_subscription<Int32>("/navigator/phase", [this](const Int32& m) { phase = m.data; });
    // path drops come before /plan: the planner may already have sent the new path in this cycle
    create_subscription<PoseWithCovarianceStamped>("/initialpose", [this](const PoseWithCovarianceStamped& p) {
      Pose q = conv::from_msg(p.pose.pose); amcl_last = q; anchor = q; t_anchor = 0; n = 0; });
    create_subscription<PoseStamped>("/goal_pose", [this](const PoseStamped&) { n = 0; idx = 0; anchor = amcl_last; t_anchor = 0; active = true; rec_active = false; });
    create_subscription<Int32>("/behavior_server/event", [this](const Int32& m) {
      n = 0; if (m.data == BEH_FINISHED) { anchor = e_last; t_anchor = 0; } }, 4);
    create_subscription<Path>("/plan", [this](const Path& p) {
      n = (int)p.poses.size; idx = 0;
      for (int i = 0; i < n; i++) { Pose q = conv::from_msg(p.poses[i].pose); w.path_x[i] = q.x; w.path_y[i] = q.y; } });
    create_subscription<Int32>("/behavior_server/active", [this](const Int32& m) { rec_active = m.data > 0; });
    create_subscription<PoseWithCovarianceStamped>("/amcl_pose", [this](const PoseWithCovarianceStamped& p) { amcl_last = conv::from_msg(p.pose.pose); });
    create_subscription<Int32>("/localization/status", [this](const Int32& m) { st = m.data; if (st == LOC_LOST) rec_active = false; });
    create_subscription<Int32>("/goal_checker/reached", [this](const Int32&) { reached = true; });
    create_subscription<PointCloud2>("/scan_cloud", [this](const PointCloud2& c) { m = conv::from_cloud(c, px, py, MAXPTS); });
    create_subscription<PoseWithCovarianceStamped>("/pose", [this](const PoseWithCovarianceStamped& p) {
      if (phase == PHASE_MAPPING) follow_tour(conv::from_msg(p.pose.pose)); });
    create_subscription<PoseStamped>("/amcl/robot_pose", [this](const PoseStamped& p) { on_pose(conv::from_msg(p.pose)); });
  }
  ~MController() { release_all({occ, dist, px, py}); }
  void derive() { if (map_dirty) { occ_from_map(*map->last, occ); bc::distance_field(occ, dist); map_dirty = false; } }
  void follow_tour(Pose est) {                                   // the tour ends at its last point, or after 400 s
    derive();
    if (tour_calls++ >= 7 * 400) { publish_int(result_pub, STATUS_SUCCEEDED); return; }
    float v, wv;
    pursue(est, w.path_x, w.path_y, n, idx, (float)max_vel_x, clearance_at(dist, est.x, est.y), v, wv);
    if (idx >= n - 2 && hypotf(w.path_x[n - 1] - est.x, w.path_y[n - 1] - est.y) < 0.15f) { publish_int(result_pub, STATUS_SUCCEEDED); return; }
    send_twist(cmd_pub, v, wv);
  }
  void on_pose(Pose e) {
    e_last = e;
    bool skip = phase != PHASE_NAVIGATING || !active || reached || !loc_normal(st) || rec_active; reached = false;
    if (skip) return;                                            // (a recovery behaviour drives the robot)
    derive();
    float v = 0, wv = 0, clr = clearance_at(dist, e.x, e.y);
    if (n > 1) {
      // try shorter lookaheads until the arc (v, w) up to the carrot is clear of the scan (Nav2 RPP collision check)
      const float las[4] = {clr < 0.35f ? 0.25f : 0.4f, 0.18f, 0.12f, 0.08f};
      bool ok = false;
      for (int t = 0; t < 4 && !ok; t++) {
        int id = idx; pursue(e, w.path_x, w.path_y, n, id, (float)max_vel_x, clr, v, wv, las[t]);
        if (v <= 0 || !arc_hits(px, py, m, v, wv, las[t])) { ok = true; idx = id; }
      }
      if (!ok) {
        // fallback like the Nav2 DWB controller: slow arcs, keep the clear ones, take the one ending closest to the carrot
        // 0.3 m ahead on the path; none clear: turn in place towards the carrot
        R.arc_blocked++;
        int q = idx; while (q < n - 1 && hypotf(w.path_x[q] - e.x, w.path_y[q] - e.y) < 0.3f) q++;
        float best = 1e9f, bv = 0, bw = 0;
        for (int iv = 1; iv <= 2; iv++) for (int iw = -4; iw <= 4; iw++) {
          float cv = 0.05f * iv, cw = 0.15f * iw;
          if (arc_hits(px, py, m, cv, cw, cv * 1.0f + 0.02f)) continue;
          float th = e.th + cw * 0.5f, ex2 = e.x + cv * fcos(th), ey2 = e.y + cv * fsin(th);
          float sc = hypotf(w.path_x[q] - ex2, w.path_y[q] - ey2);
          if (sc < best) { best = sc; bv = cv; bw = cw; }
        }
        if (best < 1e8f) { v = bv; wv = bw; }
        else {
          float a = wrap(atan2f(w.path_y[q] - e.y, w.path_x[q] - e.x) - e.th);
          v = 0; wv = fabsf(a) > 0.05f ? fmaxf(-0.6f, fminf(0.6f, 1.5f * a)) : 0;
        }
      }
    }
    // progress checker (Nav2 SimpleProgressChecker: 0.5 m in 10 s); a failure goes to the behavior server
    if (hypotf(e.x - anchor.x, e.y - anchor.y) > 0.5f) { anchor = e; t_anchor = 0; }
    else if ((t_anchor += DT) > 10.0f) { anchor = e; t_anchor = 0; publish_int(result_pub, STATUS_ABORTED); }
    send_twist(cmd_pub, v, wv);
    publish_int(fb_pub, idx);
  }
};

// --------------------------------------------------------------------------------------------- behavior_server
// Counts failures (no path, no progress) of one leg: 4 -> abort the leg; otherwise run the next recovery in turn:
// clear the live layer / spin 20 cycles / back up 10 cycles / wait 14 cycles (navsim.py RECOVER).
struct MBehavior : ros::Node {
  MResult& R; int64_t& max_failures; int fails = 0, rec_i = 0, rec_left = 0, st = LOC_HEALTHY; float rec_v = 0, rec_w = 0; bool active = false, reached = false, aborted = false;
  bus::Publisher<Twist> cmd_pub; bus::Publisher<Int32> event_pub, active_pub, result_pub;
  explicit MBehavior(MResult& R_) : ros::Node("behavior_server", true), R(R_),
      max_failures(declare_parameter("max_failures", (int64_t)4, "failures in one goal before it is aborted", 1, 100)) {
    cmd_pub = create_publisher<Twist>("/cmd_vel_nav"); event_pub = create_publisher<Int32>("/behavior_server/event", 4);
    active_pub = create_publisher<Int32>("/behavior_server/active"); result_pub = create_publisher<Int32>("/behavior_server/result");
    create_subscription<PoseStamped>("/goal_pose", [this](const PoseStamped&) { fails = 0; rec_i = 0; set_left(0); active = true; });
    create_subscription<Int32>("/localization/status", [this](const Int32& m) { st = m.data; if (st == LOC_LOST) set_left(0); });
    create_subscription<Int32>("/goal_checker/reached", [this](const Int32&) { reached = true; });
    create_subscription<PoseStamped>("/amcl/robot_pose", [this](const PoseStamped&) { on_cycle(); });
    create_subscription<Int32>("/planner_server/result", [this](const Int32&) { on_fail(BEH_ABORT_NO_PATH); });
    create_subscription<Int32>("/controller_server/result", [this](const Int32& m) { if (m.data == STATUS_ABORTED) on_fail(BEH_ABORT_NO_PROGRESS); });
  }
  void set_left(int n) { if (n != rec_left) publish_int(active_pub, n); rec_left = n; }
  void on_cycle() {                                              // before this cycle's failures (a running recovery has none)
    aborted = false;
    bool skip = !active || reached || !loc_normal(st); reached = false;
    if (skip || rec_left == 0) return;
    send_twist(cmd_pub, rec_v, rec_w);
    if (rec_left == 1) publish_int(event_pub, BEH_FINISHED);     // then plan again, progress measured from here
    set_left(rec_left - 1);
  }
  void on_fail(int why) {
    if (aborted) return;                                         // the leg already ended in this cycle
    if (++fails >= (int)max_failures) { R.aborts++; aborted = true; active = false; publish_int(result_pub, why); return; }
    int kind = rec_i++ % 4; R.recoveries++;
    publish_int(event_pub, BEH_STARTED);                         // the planner clears the live layer, both drop the path
    if (kind == 0) { set_left(0); }
    else if (kind == 1) { rec_v = 0; rec_w = 1.0f; set_left(20); }
    else if (kind == 2) { rec_v = -0.1f; rec_w = 0; set_left(10); }
    else { rec_v = 0; rec_w = 0; set_left(14); }
  }
};

// --------------------------------------------------------------------------------------------- collision_monitor
// Navigation only. Works on the raw scan only (no pose): approach model (stop if the commanded arc reaches a point within ROBOT_R + 0.01 m
// in the next 1.0 s) and a slowdown zone 0.60 m wide, 0.5 m ahead (half speed).
struct MCollisionMonitor : ros::Node {
  MResult& R; double& slowdown; float *px, *py; int m = 0, phase = PHASE_MAPPING; bool aborted = false;
  bus::Publisher<Twist> cmd_pub;
  explicit MCollisionMonitor(MResult& R_) : ros::Node("collision_monitor", true), R(R_),
      slowdown(declare_parameter("slowdown_ratio", 0.5, "speed factor with points close ahead", 0.0, 1.0)), px(falloc(MAXPTS)), py(falloc(MAXPTS)) {
    cmd_pub = create_publisher<Twist>("/cmd_vel");
    create_subscription<Int32>("/navigator/phase", [this](const Int32& m) { phase = m.data; });
    create_subscription<Int32>("/behavior_server/result", [this](const Int32&) { aborted = true; });
    create_subscription<PointCloud2>("/scan_cloud", [this](const PointCloud2& c) { m = conv::from_cloud(c, px, py, MAXPTS); });
    create_subscription<Twist>("/cmd_vel_nav", [this](const Twist& t) { on_cmd((float)t.linear.x, (float)t.angular.z); });
  }
  ~MCollisionMonitor() { release_all({px, py}); }
  void on_cmd(float v, float wv) {
    if (aborted) { aborted = false; return; }                    // the leg ended in this cycle: the robot does not move
    if (v != 0 && phase == PHASE_NAVIGATING) {                   // (the mapping tour is driven as planned, as in mis::run_mission)
      if (arc_hits(px, py, m, v, wv, fmaxf(0.05f, fabsf(v) * 1.0f), ROBOT_R + 0.01f)) { v = 0; R.collision_stops++; }
      else {
        int nslow = 0;
        for (int k = 0; k < m; k++) { float ax = v > 0 ? px[k] : -px[k], ay = fabsf(py[k]); if (ax > 0 && ay < 0.30f && ax < ROBOT_R + 0.5f) nslow++; }
        if (nslow > 2) v *= (float)slowdown;
      }
    }
    send_twist(cmd_pub, v, wv);
  }
};

// --------------------------------------------------------------------------------------------- judge (simulation only)
struct MJudge : bus::NodeBase {
  Work& w; MWork& mw; MResult& R; const Hooks& h; int world, mode; uint32_t seed;
  bus::Subscription<Odometry>* gt; bus::Subscription<OccupancyGrid>* map; uint8_t* occ;
  bool first_pose = true, kid = false, detected = false; int cycle = 0, kid_cycle = 0, ne = 0, phase = PHASE_MAPPING; float le = 0;
  MJudge(Work& w_, MWork& mw_, MResult& R_, const Hooks& h_, int world_, int mode_, uint32_t seed_) : NodeBase("judge"), w(w_), mw(mw_), R(R_), h(h_),
      world(world_), mode(mode_), seed(seed_), occ(balloc(CELLS)) {
    gt = create_subscription<Odometry>("/ground_truth", nullptr);
    map = create_subscription<OccupancyGrid>("/map", nullptr);
    create_subscription<Int32>("/sim/kidnapped", [this](const Int32&) { kid = true; kid_cycle = cycle; });
    create_subscription<Int32>("/navigator/phase", [this](const Int32& m) { phase = m.data; if (phase == PHASE_NAVIGATING) cycle = 0; });
    create_subscription<PoseWithCovarianceStamped>("/pose", [this](const PoseWithCovarianceStamped& p) {
      if (first_pose) { first_pose = false; return; }            // the first map insert is not a mapping cycle
      Pose est = conv::from_msg(p.pose.pose), t = truth_of(gt);
      float e = hypotf(est.x - t.x, est.y - t.y); if (e > R.slam_err_max) R.slam_err_max = e; });
    create_subscription<PoseStamped>("/amcl/robot_pose", [this](const PoseStamped& p) {
      Pose e = conv::from_msg(p.pose), t = truth_of(gt);
      le = hypotf(e.x - t.x, e.y - t.y); if (ne < ERRS_CAP) mw.errs[ne++] = le; });
    create_subscription<Int32>("/localization/status", [this](const Int32& m) {
      if (m.data != LOC_LOST && m.data != LOC_LOST_ABORT) return;
      if (le < 0.3f) R.false_lost++;
      if (kid && !detected) { detected = true; R.detect_s = (cycle - kid_cycle) * DT; } });
    create_subscription<Twist>("/cmd_vel", [this](const Twist& t) {
      float v = (float)t.linear.x;
      if (phase == PHASE_NAVIGATING && v > 0.01f && le > 0.5f) {
        if (kid && !detected && mode == MODE_SPIN) R.wrong_drive_before_detect_s += DT; else R.wrong_drive_after_s += DT; }
      cycle++; });
    create_subscription<Int32>("/controller_server/result", [this](const Int32& m) {
      if (m.data == STATUS_SUCCEEDED && phase == PHASE_MAPPING) map_quality(); });
  }
  ~MJudge() { release_all({occ}); }
  void map_quality() {   // distance from each occupied SLAM cell centre to the nearest true wall surface (as mis::run_mission)
    occ_from_map(*map->last, occ);
    const bool rooms = world == WORLD_ROOM || world == WORLD_ROOM_TILT;
    static const float RR[7][4] = {{0, 0, 8.05f, 0.05f}, {0, 6, 8.05f, 6.05f}, {0, 0, 0.05f, 6.05f}, {8, 0, 8.05f, 6.05f},
                                   {3, 0, 3.05f, 3.55f}, {5.2f, 3.8f, 5.85f, 4.45f}, {1.0f, 4.5f, 1.55f, 5.05f}};
    double sum = 0; int n = 0, near5 = 0;
    for (int k = 0; k < CELLS; k++) if (occ[k]) {
      float d;
      if (rooms) {
        float x = OX + (k % W + 0.5f) * RES, y = OY + (k / W + 0.5f) * RES;
        if (world == WORLD_ROOM_TILT) untilt(x, y);
        d = 9;
        for (int r = 0; r < 7; r++) {
          float dx = fmaxf(fmaxf(RR[r][0] - x, 0.0f), x - RR[r][2]), dy = fmaxf(fmaxf(RR[r][1] - y, 0.0f), y - RR[r][3]);
          d = fminf(d, hypotf(dx, dy));
        }
      } else d = w.dist_truth[k] / 10.0f * RES;
      sum += d; n++; if (d <= 0.05f) near5++;
    }
    R.map_wall_err = n ? (float)(sum / n) : 0; R.map_within5 = n ? 100.0f * near5 / n : 0;
    logf_(h, "[k] %s/%s seed %u: map done, SLAM error max %.3f m, map walls %.3f m from true walls on average (%.1f %% within 5 cm)",
          world_name(world), mode_name(mode), seed, R.slam_err_max, R.map_wall_err, R.map_within5);
  }
};

// --------------------------------------------------------------------------------------------- gz_lidar (Gazebo only)
// Gazebo's scan (/gz/scan: 720 ranges counter-clockwise from -pi in the robot frame) -> the bytes the X4 sends for one
// revolution (zero packet + 18 packets of 40 samples, raw clockwise angles with the X4 angle correction, as x4_sim.h),
// so x4_driver turns them back into points exactly as it does with the real lidar. Gazebo's scan is taken at one
// instant, so the run uses no motion correction (deskew off).
struct MGzLidar : bus::NodeBase {
  Work& w; navn::Uart& uart;
  MGzLidar(Work& w_, navn::Uart& u) : NodeBase("gz_lidar"), w(w_), uart(u) {
    create_subscription<LaserScan>("/gz/scan", [this](const LaserScan& s) { encode(s); });
  }
  static float range_at(const LaserScan& s, float th) {         // range at robot angle th (rad); 0 = no echo
    int n = (int)s.ranges.size; if (n == 0 || !(s.angle_increment > 0)) return 0;
    int i = (int)lroundf((th - s.angle_min) / s.angle_increment) % n; if (i < 0) i += n;
    float r = s.ranges[i];
    return (r == r && r >= s.range_min && r <= s.range_max) ? r : 0.0f;
  }
  static uint16_t sample(const LaserScan& s, float raw_deg) {   // as x4sim::sample, with Gazebo's range instead of a ray
    float d_mm = 3000.0f;
    for (int it = 0; it < 3; it++) {
      float r = range_at(s, -(raw_deg + x4sim::corr_deg(d_mm)) * 0.01745329f);
      if (r <= 0) return 0;
      d_mm = r * 1000.0f;
    }
    if (d_mm < 120.0f || d_mm > 16000.0f) return 0;
    return (uint16_t)(d_mm * 4.0f + 0.5f);
  }
  void encode(const LaserScan& s) {
    uint16_t smp[x4sim::SAMPLES_PER_PACKET]; int k = 0;
    smp[0] = sample(s, 0.0f);
    k += x4sim::put_packet(w.rev + k, 0x01, 0.0f, 0.0f, smp, 1);
    for (int pk = 0; pk < x4sim::PACKETS_PER_REV; pk++) {
      float a0 = pk * 20.0f;
      for (int i = 0; i < x4sim::SAMPLES_PER_PACKET; i++) smp[i] = sample(s, a0 + 0.5f * i);
      k += x4sim::put_packet(w.rev + k, 0x00, a0, a0 + 0.5f * (x4sim::SAMPLES_PER_PACKET - 1), smp, x4sim::SAMPLES_PER_PACKET);
    }
    uart.bytes = w.rev; uart.n = k;
  }
};

// --------------------------------------------------------------------------------------------- sim_base (simulation only)
// Robot, world and virtual X4. Places the unmapped box when the mapping tour ends and carries the robot away once
// (kidnap) during one random leg; odometry does not see it.
struct MSim : bus::NodeBase {
  Work& w; MResult& R; const Hooks& h; int world, opts; uint32_t seed;
  Robot rb; x4sim::Rng lrng, mrng; navn::Uart& uart; bool gz;
  bus::Publisher<Odometry> odom_pub, gt_pub; bus::Publisher<tf2_msgs::msg::TFMessage> tf_pub; bus::Publisher<Int32> kid_pub;
  bus::Publisher<PoseStamped> teleport_pub; bus::Publisher<PoseArray> box_pub; bus::Publisher<Int32> done_pub;   // gz
  float v = 0, wv = 0, t_leg = 0, kid_after = 0; bool cmd = false, kid = false; int phase = PHASE_MAPPING, li = -1, kid_leg = -1, st = LOC_HEALTHY;
  Pose kid_pose;
  MSim(Work& w_, MResult& R_, const Hooks& h_, navn::Uart& u, int world_, int opts_, uint32_t seed_, Pose start, bool gz_ = false)
      : NodeBase("sim_base"), w(w_), R(R_), h(h_), world(world_), opts(opts_), seed(seed_), lrng{4242u + seed_}, mrng{555u + seed_}, uart(u), gz(gz_) {
    rb.truth = start; rb.odom = rb.truth; rb.rng = {777u + seed};
    kid_pub = create_publisher<Int32>("/sim/kidnapped");
    if (gz) {                                                    // Gazebo moves the robot and makes the scans
      teleport_pub = create_publisher<PoseStamped>("/sim/teleport"); box_pub = create_publisher<PoseArray>("/sim/box");
      done_pub = create_publisher<Int32>("/gz/done");
      create_subscription<Int32>("/gz/contacts", [this](const Int32& m) { rb.contacts = m.data; });
    } else {
      odom_pub = create_publisher<Odometry>("/odom"); gt_pub = create_publisher<Odometry>("/ground_truth");
      tf_pub = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 4);
    }
    create_subscription<Int32>("/navigator/phase", [this](const Int32& m) { phase = m.data; });
    create_subscription<Int32>("/navigator/leg", [this](const Int32& m) { li = m.data; t_leg = 0; });
    create_subscription<Int32>("/controller_server/result", [this](const Int32& m) {
      if (m.data == STATUS_SUCCEEDED && phase == PHASE_MAPPING) world_after_mapping(); });
    create_subscription<Int32>("/localization/status", [this](const Int32& m) { st = m.data; });
    create_subscription<Twist>("/cmd_vel", [this](const Twist& m) { v = (float)m.linear.x; wv = (float)m.angular.z; cmd = true; });
    create_timer(TICK_NS, [this]() { tick(); });
    if (!gz) { revolution(); publish_state(); }                  // the lidar already turns before the first cycle
  }
  float urand() { return (mrng.next() & 0xFFFFFF) / 16777216.0f; }
  void world_after_mapping() {
    if (opts & OPT_BOX) {                                        // the world changes after mapping: one unmapped box
      float b[4]; world_box(world, seed, b);
      if (world == WORLD_ROOM_TILT) tilt_rect(w.occ_truth, b[0], b[1], b[2], b[3]);
      else {
        bc::fill_rect(w.occ_truth, b[0], b[1], b[2], b[1]); bc::fill_rect(w.occ_truth, b[0], b[3], b[2], b[3]);
        bc::fill_rect(w.occ_truth, b[0], b[1], b[0], b[3]); bc::fill_rect(w.occ_truth, b[2], b[1], b[2], b[3]);
      }
      bc::distance_field(w.occ_truth, w.dist_truth);
      logf_(h, "[k]   unmapped box placed at (%.1f-%.1f, %.1f-%.1f)%s", b[0], b[2], b[1], b[3], world == WORLD_ROOM_TILT ? " (before turning)" : "");
      if (gz) {                                                  // the same box in Gazebo: centre + yaw, then size
        float x = (b[0] + b[2] + RES) / 2, y = (b[1] + b[3] + RES) / 2, yaw = 0;   // the cells reach RES past b[2], b[3]
        if (world == WORLD_ROOM_TILT) { tilt(x, y); yaw = TILT; }
        PoseArray& a = box_pub.loan(); conv::header(a.header, "map"); a.poses.size = 2;
        conv::to_msg({x, y, yaw}, a.poses[0]); conv::to_msg({b[2] - b[0] + RES, b[3] - b[1] + RES, 0.0f}, a.poses[1]);
        box_pub.publish();
      }
    }
    Leg legs[8]; int NL = world_legs(world, legs);
    int reach_idx[8], nreach = 0; for (int i = 1; i < NL; i++) if (legs[i].reachable) reach_idx[nreach++] = i;
    kid_leg = reach_idx[mrng.next() % nreach]; kid_after = 1.0f + 5.0f * urand();
    if (!(opts & OPT_KIDNAP)) kid_leg = -1;
    for (;;) {                                                   // random free pose, true clearance > 0.45 m
      float x = 0.2f + 7.6f * urand(), y = 0.2f + 5.6f * urand();
      if (world == WORLD_ROOM_TILT) tilt(x, y);
      if (clearance_at(w.dist_truth, x, y) > 0.45f) { kid_pose = {x, y, wrap(-3.14159265f + 6.2831853f * urand())}; break; }
    }
  }
  void revolution() { uart.bytes = w.rev; uart.n = scan_rev(w, rb, lrng); rb.hold(); }
  void publish_state() {
    Odometry& o = odom_pub.loan(); conv::header(o.header, "odom"); o.child_frame_id.set("base_link");
    conv::to_msg(rb.odom, o.pose.pose); o.twist.twist.linear.x = v; o.twist.twist.angular.z = wv; odom_pub.publish();
    Odometry& g = gt_pub.loan(); conv::header(g.header, "map"); g.child_frame_id.set("base_link");
    conv::to_msg(rb.truth, g.pose.pose); gt_pub.publish();
    navn::publish_tf(tf_pub, "odom", "base_link", rb.odom);
  }
  void tick() {
    if (gz) { tick_gz(); return; }
    if (cmd) {
      rb.step(w.dist_truth, v, wv);
      if (phase == PHASE_NAVIGATING) {
        t_leg += DT;
        if (st != LOC_BUSY && li == kid_leg && !kid && t_leg > kid_after) {   // not during a relocalization turn
          rb.carry(kid_pose); kid = true; R.kidnapped = true; publish_int(kid_pub, 1);
          logf_(h, "[k]   kidnapped during leg %d to (%.2f, %.2f, %.2f)", li + 1, kid_pose.x, kid_pose.y, kid_pose.th);
        }
      }
    }
    cmd = false;
    revolution(); publish_state();
  }
  // Gazebo: the same kidnap decision; the PC carries the robot before it steps the world (the next scan is taken there)
  void tick_gz() {
    if (cmd && phase == PHASE_NAVIGATING) {
      t_leg += DT;
      if (st != LOC_BUSY && li == kid_leg && !kid && t_leg > kid_after) {
        PoseStamped& p = teleport_pub.loan(); conv::header(p.header, "map"); conv::to_msg(kid_pose, p.pose); teleport_pub.publish();
        kid = true; R.kidnapped = true; publish_int(kid_pub, 1);
        logf_(h, "[k]   kidnapped during leg %d to (%.2f, %.2f, %.2f)", li + 1, kid_pose.x, kid_pose.y, kid_pose.th);
      }
    }
    cmd = false;
    publish_int(done_pub, (int)(bus::now_ns() / TICK_NS));       // this cycle is over: the PC applies /cmd_vel and steps
  }
};

// --------------------------------------------------------------------------------------------- run
// Same result as mis::run_mission (same log lines; the timings differ). graph_out: node and topic list + time per node.
inline MResult run_mission_nodes(MWork& mw, const Hooks& h_in, int world, int mode, uint32_t seed, int opts = OPT_KIDNAP,
                                 void (*graph_out)(const char*) = nullptr) {
  // every log line also goes to /rosout (rcl_interfaces/Log, with the name of the node that logs it)
  static void (*log_out)(const char*); log_out = h_in.log;
  const Hooks h = {h_in.now_us, [](const char* s) { ros::rosout(rcl_interfaces::msg::Log::INFO, s); log_out(s); }};
  ros::sim_time() = true;                                        // the nodes run on the simulation's clock (/clock)
  ros::rosout_topic();                                           // (so the PC can subscribe before the first line)
  Work& w = *mw.w; mw.world = world;
  MResult R; R.world = world; R.mode = mode; R.seed = seed; R.opts = opts;
  nav::Result stats;                                             // timings of the shared nodes (x4_driver, slam, amcl)
  bus::hooks().now_ns = navn::sim_clock; navn::sim_ns() = 0;
  build_world(w.occ_truth, world); bc::distance_field(w.occ_truth, w.dist_truth);
  float WP[24][2]; world_tour(world, WP);
  Pose start = {WP[0][0], WP[0][1], 0.0f};
  static navn::Uart uart;
  const bool gz = navn::run_hooks().gz;
  const int deskew = x4sim::opts().deskew;
  if (gz) x4sim::opts().deskew = 0;                              // Gazebo's scans are taken at one instant
  // creation order = execution order in each cycle
  if (navn::run_hooks().stage) navn::run_hooks().stage("mission: creating the nodes");
  MGzLidar* gzl = gz ? new MGzLidar(w, uart) : nullptr;         // first: the cycle's scan is in the "UART" before x4_driver runs
  MMission* mission = new MMission(R, h, world);                // the test's goals (sent before bt_navigator runs)
  MNavigator* nav = new MNavigator(w, R, h, world, mode, seed);
  navn::X4Driver* drv = new navn::X4Driver(stats, h, uart);
  navn::Slam* slam = new navn::Slam(w, stats, h, start, true);
  navn::AmclNode* amcl = new navn::AmclNode(w, stats, h, seed, "/localization/status", LOC_LOST, LOC_BUSY, true);
  MMonitor* mon = new MMonitor(mw, R, h, mode);
  MGoalChecker* gc = new MGoalChecker();
  MPlanner* plan = new MPlanner(w, mw, R, h);
  MController* ctl = new MController(w, R, h);
  MBehavior* beh = new MBehavior(R);
  MCollisionMonitor* cm = new MCollisionMonitor(R);
  MJudge* judge = new MJudge(w, mw, R, h, world, mode, seed);
  MSim* sim = new MSim(w, R, h, uart, world, opts, seed, start, gz);
  // as nav2_lifecycle_manager with autostart: configure all managed nodes, then activate them, before the first cycle
  ros::LifecycleManager* lm = new ros::LifecycleManager("lifecycle_manager_navigation", {"slam", "amcl", "localization_monitor",
      "controller_server", "planner_server", "behavior_server", "collision_monitor", "goal_checker", "bt_navigator"});
  char stg[96]; auto stage = [&](const char* what, long long n = -1) {
    if (!navn::run_hooks().stage) return;
    if (n >= 0) snprintf(stg, sizeof stg, "mission world %d seed %u: %s %lld", world, (unsigned)seed, what, n);
    else snprintf(stg, sizeof stg, "mission world %d seed %u: %s", world, (unsigned)seed, what);
    navn::run_hooks().stage(stg);
  };
  stage("lifecycle startup");
  if (!lm->startup()) logf_(h, "[k]   lifecycle: not all nodes could be started");
  bus::NodeBase* extra = navn::run_hooks().extra_node ? navn::run_hooks().extra_node() : nullptr;   // bridge to the PC
  bus::NodeBase* robot[] = {nav, drv, slam, amcl, mon, gc, plan, ctl, beh, cm};   // robot side: all but judge and sim_base
  auto robot_us = [&]() { double t = 0; for (bus::NodeBase* n : robot) t += n->busy_us; return t; };
  bus::hooks().cpu_us = h.now_us;
  bus::Publisher<rosgraph_msgs::msg::Clock> clock_pub; clock_pub.t = bus::get_topic<rosgraph_msgs::msg::Clock>("/clock", 1, false); clock_pub.t->pubs++;
  double us_map = 0, us_nav = 0; int ticks_map = 0, ticks_nav = 0; bool watchdog = false;
  for (int64_t tick = 0; !mission->over() && tick < 7 * 4000; tick++) {
    navn::sim_ns() = tick * TICK_NS;
    stage("cycle", tick);
    { rosgraph_msgs::msg::Clock& c = clock_pub.loan(); c.clock.sec = (int32_t)(navn::sim_ns() / 1000000000); c.clock.nanosec = (uint32_t)(navn::sim_ns() % 1000000000); clock_pub.publish(); }
    if (navn::run_hooks().before_tick && !navn::run_hooks().before_tick(tick)) {
      logf_(h, "[k]   no scan / odometry from the PC for too long: run stopped (watchdog)"); watchdog = true; break;
    }
    int ph = nav->phase; double u0 = robot_us();
    bus::spin_once();
    double du = robot_us() - u0;
    if (ph == PHASE_MAPPING) { us_map += du; ticks_map++; }
    else { us_nav += du; ticks_nav++; if (du / 1000.0 > R.max_cycle_ms) R.max_cycle_ms = du / 1000.0; }
    if (navn::run_hooks().after_tick) navn::run_hooks().after_tick(tick);
  }
  stage("results");
  // ---- results (as mis::run_mission)
  float* errs = mw.errs; int ne = judge->ne;
  for (int i = 1; i < ne; i++) { float v = errs[i]; int j = i - 1; while (j >= 0 && errs[j] > v) { errs[j + 1] = errs[j]; j--; } errs[j + 1] = v; }
  R.loc_err_median = ne ? errs[ne / 2] : 0;
  R.contacts = sim->rb.contacts;
  bool wrong_reloc = false; for (int i = 0; i < R.n_ev; i++) if ((R.ev[i].result == EV_UNIQUE || R.ev[i].result == EV_RESOLVED) && R.ev[i].err > 0.5f) wrong_reloc = true;
  if (R.contacts > 0) R.outcome_code = OUT_CONTACT;
  else if (R.wrong_drive_after_s > 0.5f || wrong_reloc) R.outcome_code = OUT_WRONG_POSE;
  else if (R.need_help) R.outcome_code = OUT_STOPPED_SAFELY;
  else if (R.legs_done < R.legs_reachable || watchdog || !mission->done) R.outcome_code = OUT_INCOMPLETE;
  else R.outcome_code = OUT_CORRECT;
  strcpy(R.outcome, out_name(R.outcome_code));
  double load_map = ticks_map ? us_map / ticks_map / (DT * 1e6) * 100.0 : 0, load_nav = ticks_nav ? us_nav / ticks_nav / (DT * 1e6) * 100.0 : 0;
  logf_(h, "[k]   robot nodes: %.1f %% of one core while mapping, %.1f %% while navigating (%d + %d cycles)", load_map, load_nav, ticks_map, ticks_nav);
  if (graph_out) {
    stage("printing the graph");
    bus::graph_print(graph_out);
    ros::graph_print(graph_out);
    graph_out("time per node (mapping + navigation):");
    bus::load_print(graph_out, (ticks_map + ticks_nav) * DT);
  }
  bus::hooks().cpu_us = nullptr;
  stage("deleting the nodes");
  delete extra; delete lm; delete gzl; delete sim; delete judge; delete cm; delete beh; delete ctl; delete plan; delete gc; delete mon; delete amcl; delete slam; delete drv; delete nav; delete mission;
  stage("bus reset");
  bus::reset();
  stage("done");
  ros::sim_time() = false;
  x4sim::geo().n = 0;                                            // other commands use the grid walls
  x4sim::opts().deskew = deskew;
  return R;
}

}  // namespace misn
