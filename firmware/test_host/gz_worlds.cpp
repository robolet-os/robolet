// Writes the Gazebo worlds of the five mission worlds (gazebo/worlds/<name>.sdf) from the same geometry the chip uses
// (mission_core.h build_world), plus the robot: a two-wheel base of radius ROBOT_R with a 720-sample lidar at 7 Hz
// (like the YDLIDAR X4), a contact sensor on the body, and the Gazebo systems the PC bridge (tools/gzbridge.py) uses.
// Build and run (from test_host/):  g++ -O2 -std=c++17 -I../src gz_worlds.cpp -o gzw && ./gzw ../gazebo/worlds
//
// Walls: the occupied cells of the chip's world grid (5 cm), merged into rectangles, 0.5 m high; so Gazebo and the chip
// have exactly the same walls. room_tilted: the turned rectangles themselves (the chip's lidar also uses them).
// Time: physics steps of 1/700 s, so 100 steps are one lidar revolution (1/7 s); the bridge steps the world itself.
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "mission_core.h"

using namespace mis;

struct Box { float cx, cy, sx, sy, yaw; };

static std::vector<Box> merge_cells(const uint8_t* occ) {
  std::vector<uint8_t> used(CELLS, 0);
  std::vector<Box> out;
  for (int j = 0; j < H; j++)
    for (int i = 0; i < W; i++) {
      if (!occ[j * W + i] || used[j * W + i]) continue;
      int i1 = i; while (i1 + 1 < W && occ[j * W + i1 + 1] && !used[j * W + i1 + 1]) i1++;
      int j1 = j;
      for (;;) {                                                 // grow down while the whole run is free and occupied
        if (j1 + 1 >= H) break;
        bool ok = true; for (int k = i; k <= i1; k++) if (!occ[(j1 + 1) * W + k] || used[(j1 + 1) * W + k]) { ok = false; break; }
        if (!ok) break; j1++;
      }
      for (int jj = j; jj <= j1; jj++) for (int k = i; k <= i1; k++) used[jj * W + k] = 1;
      float x0 = OX + i * RES, x1 = OX + (i1 + 1) * RES, y0 = OY + j * RES, y1 = OY + (j1 + 1) * RES;
      out.push_back({(x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, 0});
    }
  return out;
}

static std::vector<Box> tilted_room() {
  static const float R[7][4] = {{0, 0, 8.05f, 0.05f}, {0, 6, 8.05f, 6.05f}, {0, 0, 0.05f, 6.05f}, {8, 0, 8.05f, 6.05f},
                                {3, 0, 3.05f, 3.55f}, {5.2f, 3.8f, 5.85f, 4.45f}, {1.0f, 4.5f, 1.55f, 5.05f}};
  std::vector<Box> out;
  for (auto& r : R) { float x = (r[0] + r[2]) / 2, y = (r[1] + r[3]) / 2; tilt(x, y); out.push_back({x, y, r[2] - r[0], r[3] - r[1], TILT}); }
  return out;
}

static const char* ROBOT = R"(
    <model name="robot">
      <pose>%.3f %.3f 0 0 0 %.4f</pose>
      <link name="base_link">
        <pose>0 0 0.06 0 0 0</pose>
        <inertial><mass>3.0</mass><inertia><ixx>0.03</ixx><iyy>0.03</iyy><izz>0.05</izz></inertia></inertial>
        <collision name="body"><geometry><cylinder><radius>%.3f</radius><length>0.10</length></cylinder></geometry></collision>
        <visual name="body"><geometry><cylinder><radius>%.3f</radius><length>0.10</length></cylinder></geometry>
          <material><ambient>0.2 0.5 0.9 1</ambient><diffuse>0.2 0.5 0.9 1</diffuse></material></visual>
        <collision name="caster_front"><pose>0.12 0 -0.045 0 0 0</pose><geometry><sphere><radius>0.015</radius></sphere></geometry>
          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
        <collision name="caster_back"><pose>-0.12 0 -0.045 0 0 0</pose><geometry><sphere><radius>0.015</radius></sphere></geometry>
          <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
        <sensor name="bumper" type="contact">
          <contact><collision>body</collision></contact>
          <update_rate>100</update_rate>
          <topic>/robot/contact</topic>
        </sensor>
        <sensor name="lidar" type="gpu_lidar">
          <pose>0 0 0.08 0 0 0</pose>
          <topic>/robot/scan</topic>
          <update_rate>7</update_rate>
          <lidar>
            <scan><horizontal><samples>720</samples><resolution>1</resolution>
              <min_angle>-3.14159265</min_angle><max_angle>3.13286601</max_angle></horizontal></scan>
            <range><min>0.12</min><max>12.0</max><resolution>0.01</resolution></range>
            <noise><type>gaussian</type><mean>0</mean><stddev>0.005</stddev></noise>
          </lidar>
          <always_on>1</always_on>
          <visualize>0</visualize>
        </sensor>
      </link>
      <link name="left_wheel">
        <pose>0 0.12 0.035 -1.5707963 0 0</pose>
        <inertial><mass>0.2</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
        <collision name="c"><geometry><cylinder><radius>0.035</radius><length>0.02</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>0.035</radius><length>0.02</length></cylinder></geometry></visual>
      </link>
      <link name="right_wheel">
        <pose>0 -0.12 0.035 -1.5707963 0 0</pose>
        <inertial><mass>0.2</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
        <collision name="c"><geometry><cylinder><radius>0.035</radius><length>0.02</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>0.035</radius><length>0.02</length></cylinder></geometry></visual>
      </link>
      <joint name="left_wheel_joint" type="revolute"><parent>base_link</parent><child>left_wheel</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>
      <joint name="right_wheel_joint" type="revolute"><parent>base_link</parent><child>right_wheel</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.24</wheel_separation><wheel_radius>0.035</wheel_radius>
        <topic>/robot/cmd_vel</topic><odom_topic>/robot/odom</odom_topic><odom_publish_frequency>700</odom_publish_frequency>
        <frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id>
      </plugin>
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose><publish_model_pose>true</publish_model_pose>
        <publish_nested_model_pose>true</publish_nested_model_pose><use_pose_vector_msg>true</use_pose_vector_msg>
        <update_frequency>700</update_frequency>
      </plugin>
    </model>
)";

static void write_world(const char* dir, int world) {
  static uint8_t occ[CELLS];
  build_world(occ, world);
  std::vector<Box> boxes = world == WORLD_ROOM_TILT ? tilted_room() : merge_cells(occ);
  float WP[24][2]; world_tour(world, WP);
  std::string path = std::string(dir) + "/" + world_name(world) + ".sdf";
  FILE* f = fopen(path.c_str(), "w"); if (!f) { perror(path.c_str()); exit(1); }
  fprintf(f, R"(<?xml version="1.0"?>
<!-- Made by test_host/gz_worlds.cpp from the chip's world "%s" (mission_core.h); do not edit by hand. -->
<sdf version="1.9">
  <world name="robolet">
    <physics name="1_700" type="ode"><max_step_size>0.00142857143</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.3 0.2 -0.9</direction></light>
    <model name="ground"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual></link></model>
    <model name="walls"><static>true</static><link name="l">
)", world_name(world));
  int k = 0;
  for (const Box& b : boxes) {
    fprintf(f, "      <collision name=\"c%d\"><pose>%.3f %.3f 0.25 0 0 %.6f</pose><geometry><box><size>%.3f %.3f 0.5</size></box></geometry></collision>\n",
            k, b.cx, b.cy, b.yaw, b.sx, b.sy);
    fprintf(f, "      <visual name=\"v%d\"><pose>%.3f %.3f 0.25 0 0 %.6f</pose><geometry><box><size>%.3f %.3f 0.5</size></box></geometry>"
               "<material><ambient>0.85 0.85 0.8 1</ambient><diffuse>0.85 0.85 0.8 1</diffuse></material></visual>\n", k, b.cx, b.cy, b.yaw, b.sx, b.sy);
    k++;
  }
  fprintf(f, "    </link></model>\n");
  fprintf(f, ROBOT, WP[0][0], WP[0][1], 0.0, ROBOT_R, ROBOT_R);
  fprintf(f, "  </world>\n</sdf>\n");
  fclose(f);
  printf("%s: %zu wall boxes, robot at (%.2f, %.2f)\n", path.c_str(), boxes.size(), WP[0][0], WP[0][1]);
}

int main(int argc, char** argv) {
  const char* dir = argc > 1 ? argv[1] : "../gazebo/worlds";
  for (int w = 0; w < N_WORLDS; w++) write_world(dir, w);
  return 0;
}
