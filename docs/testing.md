# Testing

## PC (no board)

```bash
cd firmware
bash test_host/run_pc_tests.sh          # quick set, about 5 minutes (also run by GitHub Actions)
bash test_host/run_pc_tests.sh full     # about 30 minutes
```

| Test | Checks |
|---|---|
| `test_main`, `test_bus`, `test_vlidar` | firmware parts, topics and executor, virtual X4 → parser → scan matching |
| `test_trig` | table sine/cosine (`fast_trig.h`) against the C library: largest error below 2e-6 over -20..20 rad |
| `test_nav` | node version of the virtual driving = single-loop version, every result number |
| `check_cdr` | the firmware's CDR bytes = `rosbags` (an independent ROS 2 implementation) for all 86 message, service and action types |
| `test_bridge` + `check_link` | the mission result is the same with the bridge; the bytes for the PC decode correctly |
| `test_mission_nodes`, `cmpall.sh` | node version of the mission = single-loop version, every log line (5 worlds × 2 modes × options × 8 seeds = 240 cases in `full`) |
| `check_ros`, `check_cli` (`full`) | services, parameters, lifecycle, actions; the `ros2.py` command line, against `test_gz` (the PC stand-in of the chip, same nodes, on a pseudo terminal) |

The single-loop version (`mission_core.h`) does the global search and the candidate checks in one cycle. The node version spreads them over cycles on the chip; for the comparison, `test_mission_nodes` runs it with `SLICES=1 TURN=0` (one cycle). With the chip's settings (`SLICES=12 TURN=4`) the outcome class of all 80 kidnapping cases was the same, except two cases in the narrow-door world that went from a safe stop to a correct relocalization.

## Chip (`firmware/test.sh`)

| Command | Runs | About |
|---|---|---|
| `bash test.sh` | build, upload, board check, compute benchmark, virtual lidar, virtual driving | 3 min |
| `bash test.sh lidar` | the same + a real YDLIDAR X4 for 10 s | 3 min |
| `bash test.sh doors` | one mission in the narrow-door world, kidnapping + unmapped box | 4 min |
| `bash test.sh lostquick` | missions in 5 worlds × 1 seed, kidnapping + unmapped box | 13 min |
| `bash test.sh lost` / `lostbase` | 5 worlds × 4 seeds with relocalization / with AMCL only | 50 min |
| `bash test.sh bridge` | one mission with the topics sent to the PC, recorded and checked | 2 min |
| `bash test.sh view [rt]` | one mission shown live in the browser (http://localhost:8765) | |
| `bash test.sh gz [rt]` | one mission with the robot and the lidar in Gazebo | |
| `bash test.sh ros` | services, parameters, lifecycle, actions from the PC during a mission | 3 min |
| `bash test.sh cli` | the `ros2.py` command line, end to end | 4 min |
| `bash test.sh shell` | a mission in real time and a `ros2`-style prompt | |

Logs go to `../results/` (`chip_*.txt`, `ros_*.txt`, `cli_*.txt`, ...). `nobuild` skips the upload. Missions run the node version; add `old` for the single loop.

If the chip restarts during a test, `run_test.py` waits for the port, runs the board check, and the log shows the restart reason and the last stage of the mission (kept in RTC memory).

## Gazebo (hardware in the loop)

Needs Gazebo Harmonic and its Python modules for the system Python (`sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-serial`). Each cycle: the chip runs its nodes and sends `/cmd_vel`; `tools/gzbridge.py` applies it, steps the world by 1/7 s (100 physics steps), and sends the new scan, odometry, true pose and contacts; the chip waits for them. The run is the same however slow the PC is. See [firmware/gazebo/README.md](../firmware/gazebo/README.md) (Korean).

Known: in the room world, seed 0, the first goal ends with no path in Gazebo (on the chip and on the PC stand-in alike); the Gazebo map is less exact than the chip's own simulation (mean wall distance 5.5–7.7 cm vs 1.5–2.8 cm).
