# Changelog

The project was called esp32ros up to 0.15.

## 0.16 (2026-11-27)
- Renamed to Robolet (firmware `robolet-fw`, PC library `tools/robolet.py`, environment variable `ROBOLET_DAEMON_PORT`).
- Relocalization: the check of the candidates while turning is spread over cycles (`relocalization_hyps_per_cycle`, default 4), so it no longer takes 239 ms in one cycle with 9 candidates. Chip (`lostquick`, 5 worlds): longest cycle 244 → 133 ms in the doors world, 124–136 ms in all five.
- `tools/ros2_bridge.py`: the chip's topics on a ROS 2 network (Humble or later) for rviz2 and `ros2 topic`; rviz2 "2D Goal Pose" and "2D Pose Estimate" and keyboard teleop back to the chip. `tools/robolet.rviz`.
- `/tf` keeps up to 4 messages per cycle, so both map -> odom and odom -> base_link reach the PC.
- Real-robot mode, firmware command `r` (`src/real_nodes.h`): real lidar and the Jarabot motor controller instead of the simulator; keyboard mapping, `/mission/finish_mapping`, then goals. `r relay`: lidar and motor controller on the USB ports of a Raspberry Pi or PC (`tools/relay.py`). PlatformIO environment `jarabot`. PC test with a simulated Jarabot (`test_host/test_real.cpp`); not yet tested on the robot. See docs/jarabot.md.
- Message types std_msgs/UInt8MultiArray, MultiArrayLayout, MultiArrayDimension (86 types).
- `test_host/run_pc_tests.sh` and GitHub Actions (both firmware environments).

## 0.15 (2026-11-27)
- Global search after a kidnapping spread over about 20 cycles with fixed work per cycle (`search_slices`, `search_reps_per_cycle`): longest cycle on the chip 1269 ms → 124–136 ms.
- Restart diagnostics: the last stage of a mission is kept in RTC memory; the board check prints the restart reason and that stage. `run_test.py` waits for the port after a restart and writes its log line by line.

## 0.14 (2026-11-26)
- `tools/ros2.py`: `ros2`-style command line with a daemon and a shell.
- Mission option `hold`: after its goals the robot waits for goals from the PC until `/mission/stop`.

## 0.13 (2026-11-26)
- Node list over the link; `describe_parameters`, `get_parameter_types`, `set_parameters_atomically`, `/parameter_events`; `get_available_transitions` and `~/transition_event`; action feedback from `bt_navigator`; `/rosout`; `/clock` and `use_sim_time`.

## 0.12 (2026-11-26)
- ROS 2 layer on the chip (`src/ros.h`): services (also called from the PC), parameters, lifecycle nodes, lifecycle manager, actions `navigate_to_pose` and `navigate_through_poses`.

## 0.9 – 0.11 (2026-11-26)
- Serial link to the PC: frames with CDR messages, recorder and checks; browser viewer; hardware in the loop with Gazebo Harmonic.

## 0.8 (2026-11-25)
- ROS 2-shaped structure: topics, nodes and an executor on the chip; message types generated from ROS 2 `.msg` files; the virtual driving split into nodes with the same results as the single loop.

## 0.4 – 0.7 (2026-11-25)
- Missions with a kidnapped robot and an unmapped box; worlds with a narrow door, an S-shaped corridor and a tilted room; lidar motion distortion and per-point correction; map accuracy against the true walls.

## Before 0.4
- Board check, compute benchmark, virtual YDLIDAR X4 on the chip, virtual driving.
