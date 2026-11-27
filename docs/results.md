# Results

All chip measurements: ESP32-S3-N16R8 (ESP32-S3 rev 0, 2 cores, 240 MHz, 16 MB flash, 8 MB PSRAM), PlatformIO espressif32 6.13.0, Arduino core 2.0.17, ESP-IDF 4.4.7. The robot and the lidar are simulated on the chip unless stated otherwise (Gazebo). "Share of one core" counts all callbacks of the robot-side nodes (not `sim_base` and `judge`) divided by the simulated time. The log files are in [results/chip/](../results/chip/). The project was called esp32ros before 0.16; older logs show that name.

## Missions with kidnapping and an unmapped box (`bash test.sh lostquick`, 5 worlds × seed 0)

Firmware 0.16, log `chip_20261127_070423.txt` (total chip time 684 s):

| World | Result | Legs | Relocalization | Map walls vs truth | Robot nodes: mapping / navigating | Longest cycle | Global search | Candidate check while turning |
|---|---|---|---|---|---|---|---|---|
| room | correct | 4/4 | 4 candidates, resolved by turning, error 0.02 m | 1.6 cm | 28.9 % / 14.7 % | 124 ms | 1169 ms in 19 cycles, longest 119 ms | 102 ms |
| s_corridor | stopped safely ¹ | 2/3 | 2 candidates left after 15 s, true pose among them | 1.9 cm | 28.9 % / 15.5 % | 136 ms | 1509 ms in 24 cycles, longest 130 ms | 63 ms |
| s_corridor_lm | correct | 5/5 | 2 candidates, resolved, error 0.01 m | 1.8 cm | 29.6 % / 13.9 % | 136 ms | 1483 ms in 24 cycles, longest 130 ms | 62 ms |
| doors | stopped safely ¹ | 1/2 | 9 → 2 candidates, true pose among them | 2.8 cm | 29.1 % / 19.7 % | 133 ms ² | 1509 ms in 24 cycles, longest 128 ms | 113 ms ² |
| room_tilted | correct | 4/4 | 5 candidates, resolved, error 0.04 m | 1.5 cm | 28.9 % / 14.8 % | 124 ms | 1218 ms in 19 cycles, longest 120 ms | 103 ms |

In all five: kidnapping detected after 0.29 s, no driving with a wrong pose after detection, no contact with walls. Outcome classes and completed legs are the same as in the PC build. Memory after the run: PSRAM 8141 KB and internal 292 KB free, the same as before.

¹ Several places look the same to the lidar; the robot stops instead of driving with a wrong pose. Intended behavior.
² Firmware 0.15 (`chip_20261127_001217.txt`): the check of 9 candidates while turning ran in one cycle, 239 ms; longest cycle 244 ms. Firmware 0.16 spreads it over cycles (4 candidates per cycle, `relocalization_hyps_per_cycle`). The outcome of each world is the same in 0.15 and 0.16.

Firmware 0.14, same missions, the global search in one cycle: longest cycle 1269 ms (room).

## ROS 2 interfaces on the chip

| Test | Firmware | Log | Result |
|---|---|---|---|
| `bash test.sh ros`: service list, parameters (get, set, refused values, describe, types, atomic, events), lifecycle (get, list, pause / resume, transition events), actions (status, feedback, cancel, result), `/clock`, `/rosout`, node list | 0.13 | `ros_20261126_175757.txt` | 33 of 33 checks |
| `bash test.sh cli`: `ros2.py` commands as separate processes, two goals after the mission (one reached, one canceled with Ctrl+C), `mission stop` | 0.14 | `cli_20261126_201134.txt` | 29 of 29 checks |

In the mission run: 15 nodes, 43 topics, 102 services, 2 actions.

## ROS 2 bridge and rviz2

`tools/ros2_bridge.py` with ROS 2 Humble and rviz2 on the PC, room mission on the chip (firmware 0.16 before the `/tf`
change, 2026-11-27, screenshot by the author): the map, the plan and the estimated and true poses were shown in rviz2;
the lidar points and the TF frames were not, because only odom -> base_link reached the PC. After the `/tf` change
(4 messages per cycle), the PC stand-in of the chip sends both map -> odom and odom -> base_link (30 of each in
60 messages); not yet checked again with the chip and rviz2.

## Gazebo (hardware in the loop)

Firmware 0.11, room world: 1151 cycles in lockstep with Gazebo Harmonic 8.15, 0 damaged frames on the USB link. Mean map wall distance in Gazebo: 5.5–7.7 cm (PC replay of the Gazebo data: mean SLAM error 3.8 cm with near-perfect odometry). Interpretation: the chip's own simulated lidar sees grid-aligned walls in four of the five worlds, which favors scan matching.

## PC checks

| Check | Result |
|---|---|
| Node version = single-loop version, every log line (5 worlds × 2 modes × options × seeds 0–7) | 240 / 240 cases the same (firmware 0.16) |
| CDR bytes = `rosbags` | 86 / 86 types (PC build of firmware 0.16; 83 types in 0.13–0.15) |
| Global search and candidate checks spread over cycles vs. in one cycle (80 kidnapping cases) | 78 same outcome; 2 (doors, seeds 1 and 5) correct instead of stopped safely |

## Build

Firmware 0.15: RAM 25.0 % (81796 of 327680 bytes), flash 8.1 % (529545 of 6553600 bytes).
