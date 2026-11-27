// PC test of the topic layer (src/bus.h) and the message conversions (src/ros2_conv.h).
// Build and run:  g++ -O2 -std=c++17 -I../src test_bus.cpp -o tb && ./tb
#include <cstdio>
#include <cstdlib>
#include <random>
#include "bus.h"
#include "ros2_conv.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
static int64_t sim_ns = 0;
static int64_t clock_ns() { return sim_ns; }
static void out(const char* s) { printf("%s\n", s); }

int main() {
  bus::hooks().now_ns = clock_ns;
  using std_msgs::msg::Int32;

  // ---- order, depth, lost messages
  {
    std::vector<int> got_a, got_b;
    bus::NodeBase talker("talker");
    auto pub = talker.create_publisher<Int32>("/chatter", 3);
    bus::NodeBase la("listener_a"), lb("listener_b");
    la.create_subscription<Int32>("/chatter", [&](const Int32& m) { got_a.push_back(m.data); });
    lb.create_subscription<Int32>("/chatter", [&](const Int32& m) { got_b.push_back(m.data); });
    for (int i = 0; i < 2; i++) { pub.loan().data = i; pub.publish(); }
    bus::spin_once();
    for (int i = 2; i < 7; i++) { Int32 m; m.data = i; pub.publish(m); }   // 5 messages, depth 3: 2 are lost
    bus::spin_once();
    CHECK(got_a.size() == 5 && got_a[0] == 0 && got_a[1] == 1 && got_a[2] == 4 && got_a[4] == 6, "listener a got %zu", got_a.size());
    CHECK(got_a == got_b, "listeners differ");
    CHECK(la.subs[0]->lost == 2, "lost %u", la.subs[0]->lost);
  }
  bus::reset();
  // ---- latched topic: a late subscriber gets the last message; type check; node order inside one spin
  {
    bus::NodeBase mapper("map_server");
    auto pub = mapper.create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1, true);
    nav_msgs::msg::OccupancyGrid& g = pub.loan(); g.info.width = 7; g.data.size = 3; g.data[2] = 100; pub.publish();
    int seen = 0, w = 0;
    bus::NodeBase late("amcl");
    late.create_subscription<nav_msgs::msg::OccupancyGrid>("/map", [&](const nav_msgs::msg::OccupancyGrid& m) { seen++; w = m.info.width; });
    bus::spin_once(); bus::spin_once();
    CHECK(seen == 1 && w == 7, "latched: seen %d width %d", seen, w);
    // a node created later still receives in the same spin what an earlier node published
    bus::NodeBase a("a"), b("b"); int order = 0, at_b = -1;
    auto pa = a.create_publisher<Int32>("/step");
    a.create_timer(0, [&]() { order = 1; pa.loan().data = 42; pa.publish(); });
    b.create_subscription<Int32>("/step", [&](const Int32& m) { at_b = order * 100 + m.data; });
    sim_ns = 1; bus::spin_once();
    CHECK(at_b == 142, "same-spin delivery %d", at_b);
    bus::graph_print(out);
  }
  bus::reset();
  // ---- heading through a quaternion and back gives the same float
  {
    std::mt19937 rng(1); std::uniform_real_distribution<float> U(-3.14159265f, 3.14159265f);
    int bad = 0; float worst = 0;
    for (int i = 0; i < 1000000; i++) {
      bc::Pose p = {U(rng) * 3, U(rng) * 2, U(rng)};
      if (i == 0) p.th = 3.14159265f;
      if (i == 1) p.th = -3.14159265f;
      if (i == 2) p.th = 0.0f;
      if (i == 3) p.th = 1e-30f;
      geometry_msgs::msg::Pose m; conv::to_msg(p, m); bc::Pose q = conv::from_msg(m);
      if (q.x != p.x || q.y != p.y || q.th != p.th) { bad++; float d = fabsf(q.th - p.th); if (d > worst) worst = d; }
    }
    CHECK(bad == 0, "%d of 1e6 poses changed (worst heading difference %g)", bad, worst);
    printf("pose -> geometry_msgs/Pose -> pose: %d of 1000000 changed\n", bad);
  }
  // ---- points through PointCloud2
  {
    float px[800], py[800], qx[800], qy[800];
    for (int i = 0; i < 800; i++) { px[i] = 0.001f * i - 0.3f; py[i] = 1.0f / (i + 1); }
    static sensor_msgs::msg::PointCloud2 c; conv::to_cloud(px, py, 800, c);
    int n = conv::from_cloud(c, qx, qy, 800);
    bool same = n == 800; for (int i = 0; i < n; i++) same = same && qx[i] == px[i] && qy[i] == py[i];
    CHECK(same, "cloud round trip n=%d", n);
    printf("points -> sensor_msgs/PointCloud2 -> points: %s (%d points, %u bytes)\n", same ? "same" : "DIFFERENT", n, (unsigned)c.data.size);
  }
  printf(fails ? "\n%d FAILED\n" : "\nALL PASSED\n", fails);
  return fails ? 1 : 0;
}
