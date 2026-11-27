// Virtual driving (firmware command [n]) split into nodes that talk only through topics (bus.h), with ROS 2 names
// and message types. Same algorithms and same results as nav::run in nav_core.h (checked by test_host/test_nav.cpp).
//
//   node              publishes                                            subscribes
//   bt_navigator      /navigator/phase, /plan (mapping tour), /goal_pose,  /controller_server/result, /goal_checker/reached,
//                     /initialpose                                         /planner_server/result, /pose, /ground_truth
//   x4_driver         /scan, /scan_cloud                                   /odom              (+ X4 bytes from the "UART")
//   slam              /map (latched), /pose, /tf                           /navigator/phase, /odom, /scan_cloud
//   amcl              /amcl_pose, /amcl/robot_pose, /tf                    /navigator/phase, /map, /odom, /initialpose, /scan_cloud
//   goal_checker      /goal_checker/reached                                /goal_pose, /amcl/robot_pose
//   planner_server    /plan, /planner_server/result                        /map, /goal_pose, /goal_checker/reached, /amcl/robot_pose
//   controller_server /cmd_vel, /controller_server/result                  /navigator/phase, /map, /goal_pose, /plan, /goal_checker/reached,
//                                                                          /planner_server/result, /pose, /amcl/robot_pose
//   judge             -                                                    /pose, /amcl/robot_pose, /ground_truth   (simulation only)
//   sim_base          /odom, /ground_truth, /tf                            /cmd_vel           (+ X4 bytes to the "UART")
//
// Standard ROS 2 / Nav2 names: /scan, /odom, /map, /tf, /cmd_vel, /plan, /goal_pose, /initialpose, /amcl_pose, /pose
// (slam_toolbox). Not standard: /scan_cloud (deskewed points, sensor_msgs/PointCloud2, used by slam and amcl),
// /amcl/robot_pose (pose in the map frame every cycle; Nav2 nodes get it from tf), /ground_truth (simulator), and the
// result topics (std_msgs/Int32 with action_msgs/GoalStatus codes; in Nav2 these are action results, see step 10).
// One executor runs the nodes once per lidar revolution in the order above, so a run is repeatable.
#pragma once
#include "nav_core.h"
#include "bus.h"
#include "ros2_conv.h"
#include "ros.h"

namespace navn {
using namespace nav;
using bc::Pose;
using std_msgs::msg::Int32;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Path;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud2;
using tf2_msgs::msg::TFMessage;

enum { PHASE_MAPPING = 0, PHASE_NAVIGATING = 1, PHASE_DONE = 2 };
enum { STATUS_SUCCEEDED = 4, STATUS_ABORTED = 6 };            // action_msgs/msg/GoalStatus values
static const int64_t TICK_NS = 142857143;                      // one X4 revolution, DT = 1/7 s

inline int64_t& sim_ns() { static int64_t t = 0; return t; }
inline int64_t sim_clock() { return sim_ns(); }
inline float* falloc(int n) { return (float*)bus::hooks().alloc(sizeof(float) * n); }
inline uint8_t* balloc(int n) { return (uint8_t*)bus::hooks().alloc(n); }

inline void publish_tf(bus::Publisher<TFMessage>& p, const char* parent, const char* child, Pose t) {
  TFMessage& m = p.loan(); m.transforms.size = 1;
  conv::header(m.transforms[0].header, parent); m.transforms[0].child_frame_id.set(child); conv::to_msg(t, m.transforms[0].transform);
  p.publish();
}
inline void publish_pose(bus::Publisher<PoseStamped>& p, const char* frame, Pose x) {
  PoseStamped& m = p.loan(); conv::header(m.header, frame); conv::to_msg(x, m.pose); p.publish();
}
inline void publish_int(bus::Publisher<Int32>& p, int v) { p.loan().data = v; p.publish(); }
// map grid derived the same way as nav::slam_rebuild: occupied = 100
inline void occ_from_map(const OccupancyGrid& m, uint8_t* occ) { for (int k = 0; k < CELLS; k++) occ[k] = m.data[k] == 100 ? 1 : 0; }

struct Uart { uint8_t* bytes = nullptr; int n = 0; };           // stands in for the lidar serial port
// where the lidar sits on a real robot (real_nodes.h): x ahead of the robot centre (m), yaw of its 0 deg mark (deg, counter-
// clockwise); the simulation uses 0, 0
struct LidarMount { float x = 0, yaw_deg = 0; };
inline LidarMount& lidar_mount() { static LidarMount m; return m; }
// optional additions to a run: a node created after all others (the bridge to the PC) and a call after every cycle
// (the firmware sends the link bytes there or keeps real time). Neither changes the results of the robot nodes.
// gz (missions only): the robot, its lidar and the world are in Gazebo on the PC (tools/gzbridge.py) instead of the
// chip's simulator; before_tick(tick) then waits until the PC has sent that cycle's lidar scan and odometry
// (false: nothing came in time, the run stops -- the watchdog).
struct RunHooks {
  std::function<bus::NodeBase*()> extra_node; std::function<void(int64_t tick)> after_tick;
  bool gz = false; std::function<bool(int64_t tick)> before_tick;
  bool hold = false;                    // missions: after the legs, go on (goals from the PC) until /mission/stop
  int search_slices = 12;               // missions: global search spread over this many cycles (1: in one cycle)
  int turn_hyps = 4;                    // missions: candidates checked per cycle while turning (0: all in one cycle)
  void (*stage)(const char*) = nullptr; // missions: where the run is (the chip keeps it over a restart)
};
inline RunHooks& run_hooks() { static RunHooks r; return r; }

// --------------------------------------------------------------------------------------------- sim_base
struct SimBase : bus::NodeBase {
  Work& w; Result& R; Robot rb; x4sim::Rng lrng; Uart& uart;
  bus::Publisher<Odometry> odom_pub, gt_pub; bus::Publisher<TFMessage> tf_pub;
  float v = 0, wv = 0; bool cmd = false; int phase = PHASE_MAPPING;
  SimBase(Work& w_, Result& R_, Uart& u, uint32_t seed) : NodeBase("sim_base"), w(w_), R(R_), lrng{4242u + seed}, uart(u) {
    rb.truth = {1.5f, 1.5f, 0.0f}; rb.odom = rb.truth; rb.rng = {777u + seed};
    odom_pub = create_publisher<Odometry>("/odom"); gt_pub = create_publisher<Odometry>("/ground_truth");
    tf_pub = create_publisher<TFMessage>("/tf", 4);
    create_subscription<Int32>("/navigator/phase", [this](const Int32& m) { phase = m.data; });
    create_subscription<Twist>("/cmd_vel", [this](const Twist& m) { v = (float)m.linear.x; wv = (float)m.angular.z; cmd = true; });
    create_timer(TICK_NS, [this]() { tick(); });
    revolution(); publish_state();                               // the lidar already turns before the first cycle
  }
  void revolution() { uart.bytes = w.rev; uart.n = scan_rev(w, rb, lrng); rb.hold(); }
  void publish_state() {
    Odometry& o = odom_pub.loan(); conv::header(o.header, "odom"); o.child_frame_id.set("base_link");
    conv::to_msg(rb.odom, o.pose.pose); o.twist.twist.linear.x = v; o.twist.twist.angular.z = wv; odom_pub.publish();
    Odometry& g = gt_pub.loan(); conv::header(g.header, "map"); g.child_frame_id.set("base_link");
    conv::to_msg(rb.truth, g.pose.pose); gt_pub.publish();
    publish_tf(tf_pub, "odom", "base_link", rb.odom);
  }
  void tick() {
    if (cmd) { rb.step(w.dist_truth, v, wv); if (phase == PHASE_NAVIGATING) R.nav_cycles++; }
    cmd = false;
    revolution(); publish_state();
  }
};

// --------------------------------------------------------------------------------------------- x4_driver
struct X4Driver : bus::NodeBase {
  Result& R; const Hooks& h; Uart& uart; ScanBuf* scan; X4Parser parser; float *px, *py;
  bus::Subscription<Odometry>* odom; Pose last_odom; bool has_last = false;
  bus::Publisher<LaserScan> scan_pub; bus::Publisher<PointCloud2> cloud_pub;
  X4Driver(Result& R_, const Hooks& h_, Uart& u) : NodeBase("x4_driver"), R(R_), h(h_), uart(u),
      scan((ScanBuf*)bus::hooks().alloc(sizeof(ScanBuf))), parser(collect, scan), px(falloc(MAXPTS)), py(falloc(MAXPTS)) {
    scan->n = 0;
    odom = create_subscription<Odometry>("/odom", nullptr);
    scan_pub = create_publisher<LaserScan>("/scan"); cloud_pub = create_publisher<PointCloud2>("/scan_cloud");
    create_timer(TICK_NS, [this]() { tick(); });
  }
  ~X4Driver() { bus::hooks().release(scan); bus::hooks().release(px); bus::hooks().release(py); }
  void tick() {
    int64_t t0 = h.now_us();
    parser.feed(uart.bytes, uart.n); uart.n = 0;
    if (!odom->last) has_last = false;                           // no odometry yet (Gazebo / robot just started)
    Pose od = odom->last ? conv::from_msg(odom->last->pose.pose) : last_odom;
    int m;
    if (x4sim::opts().deskew && has_last)                        // remove the motion during the revolution (odometry)
      m = x4sim::to_robot_frame_deskew(scan->pts, scan->n, px, py, MAXPTS, ominus(last_odom, od), x4sim::opts().deskew == 1);
    else m = x4sim::to_robot_frame(scan->pts, scan->n, px, py, MAXPTS);
    const LidarMount& mt = lidar_mount();
    if (mt.x != 0 || mt.yaw_deg != 0) {                          // lidar frame -> robot frame
      float c = cosf(mt.yaw_deg * 0.01745329f), s = sinf(mt.yaw_deg * 0.01745329f);
      for (int i = 0; i < m; i++) { float x = px[i], y = py[i]; px[i] = mt.x + c * x - s * y; py[i] = s * x + c * y; }
    }
    last_odom = od; has_last = true;
    PointCloud2& c = cloud_pub.loan(); conv::header(c.header, "base_link"); conv::to_cloud(px, py, m, c); cloud_pub.publish();
    R.t_parse.add((h.now_us() - t0) / 1000.0);
    if (scan_pub.t->subs > 0) {                                   // /scan only when someone (a viewer, a bridge) listens
      LaserScan& s = scan_pub.loan(); conv::header(s.header, "laser"); conv::to_scan(px, py, m, s); scan_pub.publish();
    }
  }
};

// --------------------------------------------------------------------------------------------- slam
struct Slam : ros::Node {
  Work& w; Result& R; const Hooks& h; bool active = true, first = true;
  double &min_dist, &min_head;              // keyframe: a new scan goes into the map after this much motion
  Pose est, last_kf, last_odom;
  bus::Subscription<Odometry>* odom;
  bus::Publisher<OccupancyGrid> map_pub; bus::Publisher<PoseWithCovarianceStamped> pose_pub; bus::Publisher<TFMessage> tf_pub;
  Slam(Work& w_, Result& R_, const Hooks& h_, Pose start, bool managed = false) : ros::Node("slam", managed), w(w_), R(R_), h(h_),
      min_dist(declare_parameter("minimum_travel_distance", 0.10, "m between scans added to the map", 0.0, 2.0)),
      min_head(declare_parameter("minimum_travel_heading", 0.10, "rad between scans added to the map", 0.0, 3.2)), est(start), last_kf(start) {
    map_pub = create_publisher<OccupancyGrid>("/map", 1, true);
    pose_pub = create_publisher<PoseWithCovarianceStamped>("/pose"); tf_pub = create_publisher<TFMessage>("/tf", 4);
    create_subscription<Int32>("/navigator/phase", [this](const Int32& m) { active = m.data == PHASE_MAPPING; });
    odom = create_subscription<Odometry>("/odom", nullptr);
    create_subscription<PointCloud2>("/scan_cloud", [this](const PointCloud2& c) { on_scan(c); });
    memset(w.lo, 0, CELLS);
  }
  void publish_map() {
    OccupancyGrid& g = map_pub.loan(); conv::header(g.header, "map");
    g.info.resolution = RES; g.info.width = W; g.info.height = H; conv::to_msg({OX, OY, 0.0f}, g.info.origin);
    g.data.size = CELLS;
    for (int k = 0; k < CELLS; k++) g.data[k] = w.lo[k] > 3 ? 100 : (w.lo[k] < 0 ? 0 : -1);
    map_pub.publish();
  }
  void on_scan(const PointCloud2& c) {
    if (!active) return;
    int m = conv::from_cloud(c, w.px, w.py, MAXPTS);
    Pose od = conv::from_msg(odom->last->pose.pose);
    int64_t s0 = h.now_us();
    if (first) {                                                 // first scan: start the map at the start pose
      slam_insert(w.lo, est, w.px, w.py, m); slam_rebuild(w); R.keyframes = 1; last_odom = od; first = false; publish_map();
    } else {
      Pose pred = oplus(est, ominus(last_odom, od)); last_odom = od;
      bc::MatchResult cr = bc::csm_fast(w.lik, w.px, w.py, m, pred, w.idx);
      est = match_fine(w.lik, w.px, w.py, m, cr.best);
      R.t_slam.add((h.now_us() - s0) / 1000.0);
      Pose dk = ominus(last_kf, est);
      if (hypotf(dk.x, dk.y) > (float)min_dist || fabsf(dk.th) > (float)min_head) {
        int64_t u0 = h.now_us();
        slam_insert(w.lo, est, w.px, w.py, m); slam_rebuild(w); last_kf = est; R.keyframes++;
        R.t_slam_update.add((h.now_us() - u0) / 1000.0);
        publish_map();
      }
    }
    PoseWithCovarianceStamped& p = pose_pub.loan(); conv::header(p.header, "map"); conv::to_msg(est, p.pose.pose); pose_pub.publish();
    publish_tf(tf_pub, "map", "odom", oplus(est, ominus(od, {0, 0, 0})));   // map->odom = est * odom^-1 (for viewers)
  }
};

// --------------------------------------------------------------------------------------------- amcl
struct AmclNode : ros::Node {
  Work& w; Result& R; const Hooks& h; Amcl am; bool active = false, first = true, map_dirty = false;
  Pose amcl_pose, odom_at_update; uint8_t *occ, *dist, *lik; float *px, *py;
  bus::Subscription<Odometry>* odom; bus::Subscription<OccupancyGrid>* map;
  bus::Publisher<PoseWithCovarianceStamped> amcl_pub; bus::Publisher<PoseStamped> robot_pub; bus::Publisher<TFMessage> tf_pub;
  // status_topic: pause while that topic reports a relocalization (mission_nodes.h), resume on /initialpose
  AmclNode(Work& w_, Result& R_, const Hooks& h_, uint32_t seed, const char* status_topic = nullptr, int lost_code = -1, int busy_code = -1,
           bool managed = false)
      : ros::Node("amcl", managed), w(w_), R(R_), h(h_),
      occ(balloc(CELLS)), dist(balloc(CELLS)), lik((uint8_t*)bus::hooks().alloc_fast(CELLS)), px(falloc(MAXPTS)), py(falloc(MAXPTS)) {
    am.p = w.parts; am.tmp = w.parts2; am.w = w.pw; am.n = NP; am.rng = {99u + seed};
    declare_parameter("max_particles", (int64_t)NP, "particles (fixed in this build)", 0, 1e9, true);
    amcl_pub = create_publisher<PoseWithCovarianceStamped>("/amcl_pose"); robot_pub = create_publisher<PoseStamped>("/amcl/robot_pose");
    tf_pub = create_publisher<TFMessage>("/tf", 4);
    create_subscription<Int32>("/navigator/phase", [this](const Int32& m) { if (m.data == PHASE_DONE) active = false; });
    map = create_subscription<OccupancyGrid>("/map", [this](const OccupancyGrid&) { map_dirty = true; });   // derived when used
    odom = create_subscription<Odometry>("/odom", nullptr);
    create_subscription<PoseWithCovarianceStamped>("/initialpose", [this](const PoseWithCovarianceStamped& m) {
      Pose p = conv::from_msg(m.pose.pose);
      am.init(p, (float)sqrt(m.pose.covariance[0]), (float)sqrt(m.pose.covariance[35]));
      amcl_pose = p; odom_at_update = conv::from_msg(odom->last->pose.pose); first = true; active = true; });
    if (status_topic) create_subscription<Int32>(status_topic, [this, lost_code, busy_code](const Int32& m) {
      if (m.data == lost_code || m.data == busy_code) active = false; });
    create_subscription<PointCloud2>("/scan_cloud", [this](const PointCloud2& c) { on_scan(c); });
  }
  ~AmclNode() { for (void* p : {(void*)occ, (void*)dist, (void*)lik, (void*)px, (void*)py}) bus::hooks().release(p); }
  void on_scan(const PointCloud2& c) {
    if (!active) return;
    if (map_dirty) { occ_from_map(*map->last, occ); bc::distance_field(occ, dist); bc::likelihood_field(dist, lik); map_dirty = false; }
    int m = conv::from_cloud(c, px, py, MAXPTS);
    Pose od = conv::from_msg(odom->last->pose.pose);
    int64_t a0 = h.now_us();
    Pose d = ominus(odom_at_update, od);
    if (first || hypotf(d.x, d.y) > 0.25f || fabsf(d.th) > 0.2f) {          // Nav2 update_min_d / update_min_a
      if (!first) am.motion(d);
      amcl_pose = am.measure(lik, px, py, m);
      odom_at_update = od; first = false; R.amcl_updates++;
      R.t_amcl.add((h.now_us() - a0) / 1000.0);
      PoseWithCovarianceStamped& p = amcl_pub.loan(); conv::header(p.header, "map"); conv::to_msg(amcl_pose, p.pose.pose); amcl_pub.publish();
    }
    Pose e = oplus(amcl_pose, ominus(odom_at_update, od));                     // map->odom correction applied
    publish_pose(robot_pub, "map", e);
    publish_tf(tf_pub, "map", "odom", oplus(amcl_pose, ominus(odom_at_update, {0, 0, 0})));
  }
};

// --------------------------------------------------------------------------------------------- goal_checker
struct GoalChecker : bus::NodeBase {
  bool active = false; float gx = 0, gy = 0;
  bus::Publisher<Int32> reached_pub;
  GoalChecker() : NodeBase("goal_checker") {
    reached_pub = create_publisher<Int32>("/goal_checker/reached");
    create_subscription<PoseStamped>("/goal_pose", [this](const PoseStamped& g) {
      gx = (float)g.pose.position.x; gy = (float)g.pose.position.y; active = true; });
    create_subscription<PoseStamped>("/amcl/robot_pose", [this](const PoseStamped& p) {
      Pose e = conv::from_msg(p.pose);
      if (active && hypotf(gx - e.x, gy - e.y) < 0.2f) { active = false; publish_int(reached_pub, STATUS_SUCCEEDED); }  // judged on the estimate
    });
  }
};

// --------------------------------------------------------------------------------------------- planner_server
struct Planner : bus::NodeBase {
  Work& w; Result& R; const Hooks& h; uint8_t *occ, *dist, *cost; float *X, *Y;
  bool active = false, skip = false, has_path = false, map_dirty = false; float gx = 0, gy = 0; int cyc = 0, fails = 0;
  bus::Publisher<Path> plan_pub; bus::Publisher<Int32> result_pub; bus::Subscription<OccupancyGrid>* map;
  Planner(Work& w_, Result& R_, const Hooks& h_) : NodeBase("planner_server"), w(w_), R(R_), h(h_),
      occ(balloc(CELLS)), dist(balloc(CELLS)), cost(balloc(CELLS)), X(falloc(PATH_CAP)), Y(falloc(PATH_CAP)) {
    plan_pub = create_publisher<Path>("/plan"); result_pub = create_publisher<Int32>("/planner_server/result");
    map = create_subscription<OccupancyGrid>("/map", [this](const OccupancyGrid&) { map_dirty = true; });   // derived when used
    create_subscription<PoseStamped>("/goal_pose", [this](const PoseStamped& g) {
      gx = (float)g.pose.position.x; gy = (float)g.pose.position.y; active = true; has_path = false; cyc = 0; fails = 0; });
    create_subscription<Int32>("/goal_checker/reached", [this](const Int32&) { active = false; skip = true; });
    create_subscription<PoseStamped>("/amcl/robot_pose", [this](const PoseStamped& p) { on_pose(conv::from_msg(p.pose)); });
  }
  ~Planner() { for (void* p : {(void*)occ, (void*)dist, (void*)cost, (void*)X, (void*)Y}) bus::hooks().release(p); }
  void on_pose(Pose e) {
    if (skip || !active) { skip = false; return; }
    if (!has_path || cyc % 7 == 0) {                                  // at once for a new goal, then 1 Hz
      if (map_dirty) { occ_from_map(*map->last, occ); bc::distance_field(occ, dist); bc::costmap(dist, cost); map_dirty = false; }
      int64_t p0 = h.now_us();
      int s = bc::cy(e.y) * W + bc::cx(e.x), gcell = bc::cy(gy) * W + bc::cx(gx);
      bc::AStarWork wk = {w.g, w.heap, w.pos, w.state};
      bc::AStarResult ar = bc::astar_opt(cost, s, gcell, wk, w.f);
      if (ar.found) {
        int n = extract_path(w.g, w.state, s, gcell, X, Y, PATH_CAP); fails = 0; has_path = true;
        Path& pm = plan_pub.loan(); conv::header(pm.header, "map");
        if (n > (int)pm.poses.capacity) n = (int)pm.poses.capacity;
        pm.poses.size = n;
        for (int i = 0; i < n; i++) { conv::header(pm.poses[i].header, "map"); conv::to_msg({X[i], Y[i], 0.0f}, pm.poses[i].pose); }
        plan_pub.publish();
      } else if (++fails >= 3) { active = false; publish_int(result_pub, STATUS_ABORTED); }   // no path 3 times in a row
      R.t_plan.add((h.now_us() - p0) / 1000.0); R.plans++;
    }
    cyc++;
  }
};

// --------------------------------------------------------------------------------------------- controller_server
struct Controller : bus::NodeBase {
  Work& w; Result& R; const Hooks& h; uint8_t *occ, *dist; int phase = PHASE_MAPPING, n = 0, idx = 0; bool skip = false;
  bus::Publisher<Twist> cmd_pub; bus::Publisher<Int32> result_pub;
  Controller(Work& w_, Result& R_, const Hooks& h_) : NodeBase("controller_server"), w(w_), R(R_), h(h_), occ(balloc(CELLS)), dist(balloc(CELLS)) {
    cmd_pub = create_publisher<Twist>("/cmd_vel"); result_pub = create_publisher<Int32>("/controller_server/result");
    create_subscription<Int32>("/navigator/phase", [this](const Int32& m) { phase = m.data; });
    create_subscription<OccupancyGrid>("/map", [this](const OccupancyGrid& m) { occ_from_map(m, occ); bc::distance_field(occ, dist); });
    create_subscription<PoseStamped>("/goal_pose", [this](const PoseStamped&) { n = 0; idx = 0; });   // new goal: no path yet
    create_subscription<Path>("/plan", [this](const Path& p) {
      n = (int)p.poses.size; idx = 0;
      for (int i = 0; i < n; i++) { Pose q = conv::from_msg(p.poses[i].pose); w.path_x[i] = q.x; w.path_y[i] = q.y; } });
    create_subscription<Int32>("/goal_checker/reached", [this](const Int32&) { skip = true; });
    create_subscription<Int32>("/planner_server/result", [this](const Int32&) { skip = true; });
    create_subscription<PoseWithCovarianceStamped>("/pose", [this](const PoseWithCovarianceStamped& p) {
      if (phase == PHASE_MAPPING) follow_tour(conv::from_msg(p.pose.pose)); });
    create_subscription<PoseStamped>("/amcl/robot_pose", [this](const PoseStamped& p) {
      if (phase == PHASE_NAVIGATING) follow_plan(conv::from_msg(p.pose)); });
  }
  ~Controller() { bus::hooks().release(occ); bus::hooks().release(dist); }
  void send(float v, float wv) { Twist& t = cmd_pub.loan(); t.linear.x = v; t.angular.z = wv; cmd_pub.publish(); }
  void follow_tour(Pose est) {                                   // mapping: drive the fixed tour, report its end
    float v, wv;
    pursue(est, w.path_x, w.path_y, n, idx, 0.3f, clearance_at(dist, est.x, est.y), v, wv);
    if (idx >= n - 2 && hypotf(w.path_x[n - 1] - est.x, w.path_y[n - 1] - est.y) < 0.15f) { publish_int(result_pub, STATUS_SUCCEEDED); return; }
    send(v, wv);
  }
  void follow_plan(Pose e) {
    if (skip) { skip = false; return; }                          // goal reached or no path: stop, the navigator decides
    float v = 0, wv = 0; int64_t c0 = h.now_us();
    if (n > 1) pursue(e, w.path_x, w.path_y, n, idx, 0.3f, clearance_at(dist, e.x, e.y), v, wv);
    R.t_control.add((h.now_us() - c0) / 1000.0);
    send(v, wv);
  }
};

// --------------------------------------------------------------------------------------------- judge (simulation only)
struct Judge : bus::NodeBase {
  Result& R; const Hooks& h; bus::Subscription<Odometry>* gt; bool first_pose = true; Pose prev_truth;
  float* errs; int ne = 0;
  Judge(Result& R_, const Hooks& h_) : NodeBase("judge"), R(R_), h(h_), errs(falloc(4096)) {
    gt = create_subscription<Odometry>("/ground_truth", nullptr);
    create_subscription<PoseWithCovarianceStamped>("/pose", [this](const PoseWithCovarianceStamped& p) { on_slam(conv::from_msg(p.pose.pose)); });
    create_subscription<PoseStamped>("/amcl/robot_pose", [this](const PoseStamped& p) { on_loc(conv::from_msg(p.pose)); });
  }
  ~Judge() { bus::hooks().release(errs); }
  Pose truth() const { return conv::from_msg(gt->last->pose.pose); }
  void on_slam(Pose est) {
    Pose t = truth();
    if (first_pose) { first_pose = false; prev_truth = t; return; }   // the first map insert is not a mapping cycle
    R.map_dist_m += hypotf(t.x - prev_truth.x, t.y - prev_truth.y); prev_truth = t;
    float e = hypotf(est.x - t.x, est.y - t.y), et = fabsf(wrap(est.th - t.th));
    if (e > R.slam_err_max) R.slam_err_max = e;
    if (et > R.slam_th_err_max) R.slam_th_err_max = et;
    R.map_cycles++;
    if (R.map_cycles % 140 == 0)
      logf_(h, "[map] t=%5.1f s  driven %.1f m  keyframes %d  SLAM error %.3f m %.3f rad", R.map_cycles * DT, R.map_dist_m, R.keyframes, e, et);
  }
  void on_loc(Pose e) {
    Pose t = truth();
    float le = hypotf(e.x - t.x, e.y - t.y), lt = fabsf(wrap(e.th - t.th));
    if (le > R.loc_err_max) R.loc_err_max = le;
    if (lt > R.loc_th_err_max) R.loc_th_err_max = lt;
    if (ne < 4096) errs[ne++] = le;
  }
};

// --------------------------------------------------------------------------------------------- bt_navigator
struct Navigator : bus::NodeBase {
  Work& w; Result& R; const Hooks& h; int phase = PHASE_MAPPING; bool started = false, done = false;
  bool ev_tour = false, ev_reached = false, ev_noplan = false; int gi = -1, ticks = 0, map_ticks = 0;
  bus::Subscription<PoseWithCovarianceStamped>* slam_pose; bus::Subscription<Odometry>* gt;
  bus::Publisher<Int32> phase_pub; bus::Publisher<Path> tour_pub; bus::Publisher<PoseStamped> goal_pub;
  bus::Publisher<PoseWithCovarianceStamped> init_pub;
  static const int NG = 4;
  float GOALS[NG][2] = {{6.5f, 1.5f}, {6.8f, 5.2f}, {2.3f, 5.3f}, {1.5f, 1.5f}};
  Navigator(Work& w_, Result& R_, const Hooks& h_) : NodeBase("bt_navigator"), w(w_), R(R_), h(h_) {
    phase_pub = create_publisher<Int32>("/navigator/phase", 1, true); tour_pub = create_publisher<Path>("/plan");
    goal_pub = create_publisher<PoseStamped>("/goal_pose"); init_pub = create_publisher<PoseWithCovarianceStamped>("/initialpose");
    create_subscription<Int32>("/controller_server/result", [this](const Int32&) { ev_tour = true; });
    create_subscription<Int32>("/goal_checker/reached", [this](const Int32&) { ev_reached = true; });
    create_subscription<Int32>("/planner_server/result", [this](const Int32&) { ev_noplan = true; });
    slam_pose = create_subscription<PoseWithCovarianceStamped>("/pose", nullptr);
    gt = create_subscription<Odometry>("/ground_truth", nullptr);
    create_timer(TICK_NS, [this]() { tick(); });
  }
  void start_mapping() {
    static const float WP[][2] = {{1.5f, 1.5f}, {1.5f, 4.0f}, {2.5f, 4.0f}, {4.0f, 4.0f}, {4.5f, 2.0f}, {6.5f, 1.5f},
                                  {7.0f, 3.0f}, {6.5f, 5.2f}, {4.0f, 5.2f}, {2.0f, 4.0f}, {1.5f, 1.5f}};
    const int NW = sizeof(WP) / sizeof(WP[0]);
    Path& p = tour_pub.loan(); conv::header(p.header, "map"); int np = 0;
    auto add = [&](float x, float y) { if (np < (int)p.poses.capacity) { conv::header(p.poses[np].header, "map"); conv::to_msg({x, y, 0.0f}, p.poses[np].pose); np++; } };
    for (int i = 0; i + 1 < NW; i++) {
      float L = hypotf(WP[i + 1][0] - WP[i][0], WP[i + 1][1] - WP[i][1]); int k = (int)(L / 0.05f);
      for (int j = 0; j < k; j++) add(WP[i][0] + (WP[i + 1][0] - WP[i][0]) * j / k, WP[i][1] + (WP[i + 1][1] - WP[i][1]) * j / k);
    }
    add(WP[NW - 1][0], WP[NW - 1][1]);
    p.poses.size = np; tour_pub.publish();
    publish_int(phase_pub, PHASE_MAPPING);
    logf_(h, "[map] tour of %d waypoints, %d path points; SLAM: odometry + coarse/fine scan matching + log-odds map", NW, np);
  }
  Pose truth() const { return conv::from_msg(gt->last->pose.pose); }
  void finish_mapping() {
    Pose est = conv::from_msg(slam_pose->last->pose.pose), t = truth();
    R.slam_err_final = hypotf(est.x - t.x, est.y - t.y);
    { double s = 0; int n5 = 0, n = 0;
      for (int k = 0; k < CELLS; k++) if (w.occ[k]) { float d = w.dist_truth[k] / 10.0f * RES; s += d; if (d <= 0.05f) n5++; n++; }
      R.map_occ_cells = n; R.map_wall_err_mean = n ? (float)(s / n) : 9; R.map_within_5cm = n ? 100.0f * n5 / n : 0; }
    logf_(h, "[map] done: %d cycles (%.1f s), %.1f m driven, %d keyframes | SLAM error max %.3f m %.3f rad, final %.3f m",
          R.map_cycles, R.map_cycles * DT, R.map_dist_m, R.keyframes, R.slam_err_max, R.slam_th_err_max, R.slam_err_final);
    logf_(h, "[map] map: %d occupied cells, mean distance to true wall %.3f m, %.1f %% within 5 cm", R.map_occ_cells, R.map_wall_err_mean, R.map_within_5cm);
    phase = PHASE_NAVIGATING; publish_int(phase_pub, PHASE_NAVIGATING);
    PoseWithCovarianceStamped& ip = init_pub.loan(); conv::header(ip.header, "map"); conv::to_msg(est, ip.pose.pose);
    for (double& c : ip.pose.covariance) c = 0;
    ip.pose.covariance[0] = ip.pose.covariance[7] = (double)0.10f * (double)0.10f; ip.pose.covariance[35] = (double)0.05f * (double)0.05f;
    init_pub.publish();
    R.goals = NG;
    logf_(h, "[nav] AMCL %d particles + A* on the inflated SLAM costmap (1 Hz replanning) + pure pursuit, %d goals", NP, R.goals);
    next_goal();
  }
  void next_goal() {
    gi++; ticks = 0;
    if (gi >= NG) { phase = PHASE_DONE; publish_int(phase_pub, PHASE_DONE); done = true; return; }
    PoseStamped& g = goal_pub.loan(); conv::header(g.header, "map"); conv::to_msg({GOALS[gi][0], GOALS[gi][1], 0.0f}, g.pose); goal_pub.publish();
  }
  void end_goal(const char* how, int cyc, bool reached, bool unreachable) {
    Pose t = truth(); float tde = hypotf(GOALS[gi][0] - t.x, GOALS[gi][1] - t.y);
    if (reached) { R.goals_reached++; if (tde > R.goal_true_err_max) R.goal_true_err_max = tde; }
    if (unreachable) R.goals_unreachable++;
    logf_(h, "[nav] goal %d (%.1f, %.1f): %s after %.1f s | true position (%.2f, %.2f), true distance to goal %.2f m", gi + 1, GOALS[gi][0], GOALS[gi][1],
          how, cyc * DT, t.x, t.y, tde);
  }
  void tick() {
    if (!started) { started = true; start_mapping(); return; }
    if (phase == PHASE_MAPPING) {
      if (ev_tour || ++map_ticks > 7 * 240) finish_mapping();   // tour driven (or the mapping time limit, about 1 cycle later than nav::run)
      ev_tour = false;
    } else if (phase == PHASE_NAVIGATING) {
      if (ev_reached) end_goal("reached", ticks - 1, true, false), next_goal();
      else if (ev_noplan) end_goal("UNREACHABLE (no path)", ticks - 1, false, true), next_goal();
      else if (ticks == 7 * 90) {                                 // tell the robot to stop: the next goal takes over
        end_goal("NOT reached (timeout)", ticks, false, false); next_goal();
      }
      ev_reached = ev_noplan = false;
    }
    if (phase == PHASE_NAVIGATING) ticks++;
  }
};

// --------------------------------------------------------------------------------------------- run
// Same output and PASS rule as nav::run.
inline Result run_nodes(Work& w, const Hooks& h, uint32_t seed = 0, void (*graph_out)(const char*) = nullptr) {
  Result R; R.seed = seed;
  bus::hooks().now_ns = sim_clock; sim_ns() = 0;
  x4sim::geo().n = 0;                                            // grid walls
  bc::build_room(w.occ_truth); bc::distance_field(w.occ_truth, w.dist_truth);
  static Uart uart;
  Pose start = {1.5f, 1.5f, 0.0f};
  // creation order = execution order in each cycle
  Navigator* nav = new Navigator(w, R, h);
  X4Driver* drv = new X4Driver(R, h, uart);
  Slam* slam = new Slam(w, R, h, start);
  AmclNode* amcl = new AmclNode(w, R, h, seed);
  GoalChecker* gc = new GoalChecker();
  Planner* plan = new Planner(w, R, h);
  Controller* ctl = new Controller(w, R, h);
  Judge* judge = new Judge(R, h);
  SimBase* base = new SimBase(w, R, uart, seed);
  bus::NodeBase* extra = run_hooks().extra_node ? run_hooks().extra_node() : nullptr;
  // robot side = every node except the simulator (sim_base, which also makes the virtual lidar) and the judge
  bus::NodeBase* robot[] = {nav, drv, slam, amcl, gc, plan, ctl};
  auto robot_us = [&]() { double t = 0; for (bus::NodeBase* n : robot) t += n->busy_us; return t; };
  bus::hooks().cpu_us = h.now_us;
  double us_map = 0, us_nav = 0; int ticks_map = 0, ticks_nav = 0;
  int64_t tick = 0;
  while (!nav->done && tick < 7 * 1200) {
    sim_ns() = tick * TICK_NS;
    int ph = nav->phase; double u0 = robot_us();
    bus::spin_once(); tick++;
    double du = robot_us() - u0;
    if (run_hooks().after_tick) run_hooks().after_tick(tick);
    if (ph == PHASE_MAPPING) { us_map += du; ticks_map++; } else if (ph == PHASE_NAVIGATING) { us_nav += du; ticks_nav++; }
  }
  // ---- results (same lines as nav::run)
  float* errs = judge->errs; int ne = judge->ne;
  for (int i = 1; i < ne; i++) { float v = errs[i]; int j = i - 1; while (j >= 0 && errs[j] > v) { errs[j + 1] = errs[j]; j--; } errs[j + 1] = v; }
  R.loc_err_median = ne ? errs[ne / 2] : 0;
  // load = all robot-side node time (algorithms, message packing, each node's own map derivations) per cycle
  R.cpu_load_map = ticks_map ? us_map / ticks_map / (DT * 1e6) * 100.0 : 0;
  R.cpu_load_nav = ticks_nav ? us_nav / ticks_nav / (DT * 1e6) * 100.0 : 0;
  logf_(h, "[map] time per cycle: parse %.1f ms, scan matching %.1f ms (max %.1f), map update %.1f ms per keyframe (max %.1f) -> %.1f %% of one core",
        R.t_parse.avg(), R.t_slam.avg(), R.t_slam.mx, R.t_slam_update.avg(), R.t_slam_update.mx, R.cpu_load_map);
  R.contacts = base->rb.contacts; R.min_clearance = base->rb.min_clear;
  logf_(h, "[nav] goals reached %d/%d (unreachable %d) in %.1f s, true distance at arrival max %.2f m | AMCL updates %d, localization error median %.3f m, max %.3f m, %.3f rad",
        R.goals_reached, R.goals, R.goals_unreachable, R.nav_cycles * DT, R.goal_true_err_max, R.amcl_updates, R.loc_err_median, R.loc_err_max, R.loc_th_err_max);
  logf_(h, "[nav] time: AMCL update %.1f ms (max %.1f), A* plan %.1f ms (max %.1f), control %.2f ms -> %.1f %% of one core",
        R.t_amcl.avg(), R.t_amcl.mx, R.t_plan.avg(), R.t_plan.mx, R.t_control.avg(), R.cpu_load_nav);
  logf_(h, "[all] contacts %d, minimum clearance %.3f m (robot radius %.2f m)", R.contacts, R.min_clearance, ROBOT_R);
  R.pass = R.slam_err_max < 0.10f && R.map_wall_err_mean < 0.05f && R.goals_reached == R.goals && R.goal_true_err_max <= 0.25f &&
           R.loc_err_max < 0.20f && R.contacts == 0;
  if (graph_out) {
    bus::graph_print(graph_out);
    graph_out("time per node (mapping + navigation):");
    bus::load_print(graph_out, (ticks_map + ticks_nav) * DT);
  }
  bus::hooks().cpu_us = nullptr;
  delete extra; delete base; delete judge; delete ctl; delete plan; delete gc; delete amcl; delete slam; delete drv; delete nav;
  bus::reset();
  return R;
}

}  // namespace navn
