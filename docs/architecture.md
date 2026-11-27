# Architecture

## One executor, nodes and topics (`firmware/src/bus.h`)

All nodes run in one FreeRTOS task on one core. A cycle is one lidar revolution (1/7 s). In each cycle the executor runs the nodes in the order they were created; for each node it first delivers the messages that arrived on its subscriptions, then runs its timers. There are no threads between nodes, so a run is repeatable: the same input gives the same output, on the PC and on the chip.

A topic has a name, a ROS 2 message type and a small ring of message slots (depth). Publishers write into a slot ("loan") and publish; subscribers get a pointer to the slot. Latched topics keep their last message for late subscribers (like `transient_local`). Message types are plain C++ structs generated from the `.msg` files in `firmware/msg/` by `tools/gen_msgs.py`; unbounded arrays get fixed capacities (`firmware/msg/capacities.txt`), so no memory is allocated while the stack runs.

Memory: maps, cost maps, particle sets, path buffers and topic slots are allocated once per run, in PSRAM, or in internal RAM where speed matters (likelihood fields). Everything is freed at the end of a run; the board check after five runs in a row shows the same free memory as before.

## ROS 2 layer (`firmware/src/ros.h`)

| Feature | What exists on the chip | Interface types |
|---|---|---|
| Services | registry of named services; calls inside the chip run at once; calls from the PC arrive as CDR and are answered as CDR (an answer may come later, e.g. an action result) | any `.srv` in `firmware/msg/` |
| Parameters | `declare_parameter` (double, integer, bool, string, string array) with description, range and read-only flag; services `get_parameters`, `set_parameters`, `set_parameters_atomically`, `list_parameters`, `describe_parameters`, `get_parameter_types`; `/parameter_events` on every change | `rcl_interfaces` |
| Every node | read-only `use_sim_time` (true while the clock is the simulation's) | |
| Lifecycle | states unconfigured / inactive / active / finalized; only active nodes run; services `change_state`, `get_state`, `get_available_transitions`; `~/transition_event`, two events per transition through the transition state, as rclcpp | `lifecycle_msgs` |
| Lifecycle manager | as nav2_lifecycle_manager with autostart: configure all, then activate all; `manage_nodes` (startup, pause, resume, reset, shutdown, configure, cleanup), `is_active` | `nav2_msgs/srv/ManageLifecycleNodes`, `std_srvs/srv/Trigger` |
| Actions | as rclcpp_action: `<name>/_action/send_goal`, `cancel_goal`, `get_result`, `feedback`, `status`; one goal at a time, a new goal ends the running one | `action_msgs`, `unique_identifier_msgs`, `nav2_msgs/action/*` |
| Logs and time | every log line on `/rosout` with the name of the node that wrote it; `/clock` | `rcl_interfaces/msg/Log`, `rosgraph_msgs/msg/Clock` |

Counts in a mission run: 15 nodes, 43 topics, 102 services, 2 actions.

## Nodes of a mission (`firmware/src/mission_nodes.h`)

| Node | Main topics in → out | Parameters |
|---|---|---|
| `bt_navigator` (managed) | `/controller_server/result`, `/goal_checker/reached`, `/localization/status` → `/navigator/phase`, `/plan` (mapping tour); actions `navigate_to_pose`, `navigate_through_poses` | `leg_timeout` 150 s |
| `x4_driver` | lidar bytes, `/odom` → `/scan`, `/scan_cloud` | |
| `slam` (managed) | `/scan_cloud`, `/odom` → `/map`, `/pose`, `/tf` | `minimum_travel_distance` 0.10 m, `minimum_travel_heading` 0.10 rad |
| `amcl` (managed) | `/scan_cloud`, `/map`, `/initialpose` → `/amcl_pose`, `/amcl/robot_pose`, `/tf` | `max_particles` 500 (read only) |
| `localization_monitor` (managed) | `/scan_cloud`, `/amcl/robot_pose`, `/map` → `/localization/status`, `/initialpose`, `/localization/candidates`, `/cmd_vel` while relocalizing | `search_slices` 12, `search_reps_per_cycle` 2, `relocalization_hyps_per_cycle` 4 |
| `planner_server` (managed) | `/map`, `/goal_pose` → `/plan` | |
| `controller_server` (managed) | `/plan`, `/amcl/robot_pose` → `/cmd_vel_nav` | `max_vel_x` 0.3 m/s (0–1) |
| `behavior_server` (managed) | results of planner and controller → recoveries on `/cmd_vel_nav` | `max_failures` 4 |
| `collision_monitor` (managed) | `/scan_cloud`, `/cmd_vel_nav` → `/cmd_vel` | `slowdown_ratio` 0.5 |
| `goal_checker` (managed) | `/goal_pose`, `/amcl/robot_pose` → `/goal_checker/reached` | `xy_goal_tolerance` 0.2 m |
| `lifecycle_manager_navigation` | | `autostart`, `node_names` |
| `mission` | the test's goals (through the actions), `/mission/stop` | |
| `sim_base`, `judge` | simulation of the robot and of the lidar on the chip; the truth for the results | |

`sim_base` and `judge` exist only for simulated runs. With Gazebo, the robot and the lidar come from the PC instead (`gz_lidar` node, see [testing.md](testing.md)).

## Relocalization after a kidnapping

1. `localization_monitor` compares each scan with the map at the AMCL pose (beam check). Three bad scans in a row: the robot is lost; it stops and AMCL waits.
2. Global search over all free poses of the map (every 3rd cell, 36 headings), best candidates refined by scan matching and kept if the beam model accepts them. The search is spread over about 20 cycles with a fixed amount of work per cycle (pass 1 in `search_slices` cycles, sorting in one, `search_reps_per_cycle` candidates per cycle), so no cycle takes long and the PC and the chip give the same results.
3. One candidate: use it. Several: turn in place; at every 0.15 rad of turning, check the candidates against the new scan (`relocalization_hyps_per_cycle` per cycle) and drop those that decisive beams rule out. One left: use it. After 15 s, or with no candidate after 5 searches: stop and ask for help.

## Chip ↔ PC link (`firmware/src/link.h`, `bridge.h`, `tools/ros2wire.py`)

Frames on the USB serial port, between plain text log lines: `A5 5A | kind | flags | id | len | payload | CRC-16`. Kinds: HELLO, ADVERTISE (topic name + type), DATA (topic id + CDR, run-length encoded when large), SUBSCRIBE (name + rate), UNSUBSCRIBE, PUBLISH (PC → chip), LIST, STATS, LOG, CALL / REPLY (services), SERVICE (service name + type), NODE (node name + topics). A topic without a rate limit is sent message by message; with a rate limit, the newest message when the period is over. While waiting for the PC, the chip sends a zero-length USB packet and a small STATS frame every 0.5 s, because the PC's USB serial driver can hold a full 64-byte packet until a shorter one arrives.

## PC tools (`firmware/tools/`)

| Tool | Use |
|---|---|
| `ros2.py` | `ros2`-style command line: `node`, `topic` (list, info, echo, hz, pub), `service`, `param`, `lifecycle`, `action` (send_goal with feedback, Ctrl+C cancels), `interface`; a daemon keeps the port, other terminals connect to 127.0.0.1:11811; `shell` gives a prompt |
| `viewer.py` | browser viewer (map, scan, poses, path, candidates, log), live or from a recording |
| `gzbridge.py` | runs the chip against a robot in Gazebo Harmonic, cycle by cycle |
| `robolet.py` | link library (session, port search) and a simple recorder |
| `ros2wire.py` | frame parser and CDR encoder / decoder from the `.msg` files |
| `check_*.py` | checks used by the tests |
