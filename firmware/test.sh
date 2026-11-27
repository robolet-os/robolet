#!/usr/bin/env bash
# One command: build, upload, run the tests, save the log to ../results/
#   bash test.sh            board + compute + virtual lidar + virtual driving tests
#   bash test.sh lidar      the same + real lidar test
#   bash test.sh nobuild    run the tests again without uploading
#   bash test.sh doors      one mission in the narrow-door room, kidnap + unmapped box (about 3-4 min)
#   bash test.sh lostquick  missions with one kidnap and one unmapped box, 5 worlds x 1 seed, spin mode (about 12-14 min)
#   bash test.sh lost       the same with 5 worlds x 4 seeds (about 50 min; 4 worlds x 4 seeds took 42 min on the chip)
#   bash test.sh lostbase   5 worlds x 4 seeds with AMCL only (Nav2 default) for comparison
#   bash test.sh bridge     one room mission with the bridge: topics to the PC tool (tools/robolet.py), recorded and checked
#   bash test.sh view       one room mission shown live in the browser (tools/viewer.py), recorded; add "rt" for real time
#                           (bash test.sh view nobuild: without uploading again)
#   bash test.sh gz         one room mission with the robot, lidar and room in Gazebo (tools/gzbridge.py): Gazebo window
#                           + browser viewer; add "rt" for real time; another mission: GZ_CMD="k 3 1 0 3" bash test.sh gz
#   bash test.sh ros        services, parameters, lifecycle and actions from the PC during a room mission (tools/check_ros.py)
#   bash test.sh cli        the ros2-style command line (tools/ros2.py) checked end to end (tools/check_cli.py, about 4 min)
#   bash test.sh shell      a room mission in real time and a ros2-style prompt (robolet> node list, topic echo /rosout,
#                           action send_goal ...; other terminals: python3 tools/ros2.py ...); "mission stop" ends it
#   (all mission options run the node version; add "old" for the single loop, e.g. bash test.sh lostquick old)
#   bash test.sh jarabot lidar   the Jarabot firmware (PlatformIO env "jarabot", docs/jarabot.md) + board + X4 Pro check
#   bash test.sh jarabot base    the same + the Arduino motor base: encoder counts, then the wheels forward for 1 s
#                                (lift the wheels off the ground first)
cd "$(dirname "$0")" || exit 1

# Pick a Python that has pyserial (PlatformIO's own Python has it)
PY=""
PIO_PY="$(head -1 "$(command -v pio)" 2>/dev/null | sed 's/^#!//')"
for cand in "$HOME/.platformio/penv/bin/python" "$PIO_PY" python3; do
  if [ -n "$cand" ] && "$cand" -c "import serial" 2>/dev/null; then PY="$cand"; break; fi
done
if [ -z "$PY" ]; then
  echo "pyserial not found: run  pip install --user pyserial  and try again"; exit 1
fi

CMDS="b c v n"
UPLOAD=1
PIO_ENV=s3
for a in "$@"; do
  case "$a" in
    lidar) CMDS="b c v n l" ;;
    jarabot) PIO_ENV=jarabot ;;
    base) CMDS="b l m_go" ;;
    nobuild) UPLOAD=0 ;;
    doors) CMDS="b k_3_1_0_3" ;;
    lostquick) CMDS="b K_1_1_3" ;;
    lost) CMDS="b K_4_1_3" ;;
    lostbase) CMDS="b K_4_0_3" ;;
    bridge) CMDS="b"; BRIDGE=1 ;;
    view) CMDS="b"; VIEW=1 ;;
    gz) CMDS="b"; GZ=1 ;;
    ros) CMDS="b"; ROS=1 ;;
    cli) CMDS="b"; CLI=1 ;;
    shell) CMDS="b"; SHELL_RUN=1 ;;
    rt) RT=" rt" ;;
  esac
done
[ "$PIO_ENV" = jarabot ] && [ "$CMDS" = "b c v n l" ] && CMDS="b l"
for a in "$@"; do [ "$a" = old ] && CMDS=$(echo "$CMDS" | sed -E 's/((k|K)_[0-9_]+)/\1_old/g'); done

if [ "$UPLOAD" = 1 ]; then
  "$PY" tools/gen_msgs.py || exit 1                 # src/ros2_msgs.h from msg/ (ROS 2 message, service and action types)
  echo "=== 1/2 build and upload (first time takes a few minutes) ==="
  pio run -e "$PIO_ENV" -t upload || { echo "upload failed: check the cable, or hold BOOT, press RST, release BOOT and run again"; exit 1; }
  sleep 3
fi
echo "=== 2/2 tests: $CMDS ==="
"$PY" run_test.py $CMDS || { echo "=== the board did not answer: check the cable / port (stopped here)"; exit 1; }
if [ "$BRIDGE" = 1 ]; then
  STAMP=$(date +%Y%m%d_%H%M%S); mkdir -p ../results
  echo "=== bridge: room mission, topics to the PC (about 2 min) ==="
  "$PY" -u tools/robolet.py run "k 0 1 0 3" --record ../results/bridge_$STAMP.bin 2>&1 | tee ../results/bridge_$STAMP.txt
  "$PY" -u tools/check_link.py ../results/bridge_$STAMP.bin --chip 2>&1 | tee -a ../results/bridge_$STAMP.txt
  echo "=== saved: ../results/bridge_$STAMP.txt and .bin"
fi
if [ "$VIEW" = 1 ]; then
  STAMP=$(date +%Y%m%d_%H%M%S); mkdir -p ../results
  echo "=== view: room mission in the browser (http://localhost:8765); Ctrl+C when done looking ==="
  "$PY" -u tools/viewer.py --chip "k 0 1 0 3$RT" --record ../results/view_$STAMP.bin 2>&1 | tee ../results/view_$STAMP.txt
  echo "=== saved: ../results/view_$STAMP.bin  (again: $PY tools/viewer.py --play ../results/view_$STAMP.bin)"
fi
if [ "$GZ" = 1 ]; then
  STAMP=$(date +%Y%m%d_%H%M%S); mkdir -p ../results
  GZPY=/usr/bin/python3                              # Gazebo's Python modules are for the system Python
  "$GZPY" -c "import gz.transport13, gz.msgs10, serial" 2>/dev/null || {
    echo "needs: sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-serial"; exit 1; }
  [ -f gazebo/worlds/room.sdf ] || { echo "=== making the Gazebo worlds (gazebo/worlds/*.sdf) from the chip's worlds"
    (cd test_host && g++ -O2 -std=c++17 -I../src gz_worlds.cpp -o gzw && mkdir -p ../gazebo/worlds && ./gzw ../gazebo/worlds) || exit 1; }
  echo "=== gz: ${GZ_CMD:-k 0 1 0 3} with the robot in Gazebo (Gazebo window + http://localhost:8765); Ctrl+C when done looking ==="
  "$GZPY" -u tools/gzbridge.py --chip "${GZ_CMD:-k 0 1 0 3}" --gui --view ${RT:+--rt} --record ../results/gz_$STAMP.bin \
    --csv ../results/gz_$STAMP.csv 2>&1 | tee ../results/gz_$STAMP.txt
  echo "=== saved: ../results/gz_$STAMP.txt, .bin (tools/viewer.py --play), .csv (true pose, odometry, estimate per cycle)"
fi
if [ "$CLI" = 1 ]; then
  STAMP=$(date +%Y%m%d_%H%M%S); mkdir -p ../results
  echo "=== cli: tools/ros2.py commands against a room mission in real time (about 4 min) ==="
  "$PY" -u tools/check_cli.py --record ../results/cli_$STAMP.bin 2>&1 | tee ../results/cli_$STAMP.txt
  echo "=== saved: ../results/cli_$STAMP.txt"
fi
if [ "$SHELL_RUN" = 1 ]; then
  echo "=== shell: room mission in real time; type ros2 commands (help), 'mission stop' ends the run, Ctrl+D leaves ==="
  "$PY" tools/ros2.py shell
fi
if [ "$ROS" = 1 ]; then
  STAMP=$(date +%Y%m%d_%H%M%S); mkdir -p ../results
  echo "=== ros: services, parameters, lifecycle, actions during a room mission (about 3 min) ==="
  "$PY" -u tools/check_ros.py --record ../results/ros_$STAMP.bin 2>&1 | tee ../results/ros_$STAMP.txt
  echo "=== saved: ../results/ros_$STAMP.txt"
fi
