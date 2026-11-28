#!/usr/bin/env bash
# PC tests (no board needed). Builds the test programs and runs them; the last line is "PC tests: PASS" or "... FAIL".
#   bash test_host/run_pc_tests.sh          quick set (about 5 min): topics, CDR vs. ROS 2, bridge, node version = single
#                                           loop (2 worlds x 2 seeds), virtual lidar, virtual driving
#   bash test_host/run_pc_tests.sh full     + node version = single loop for all worlds/modes/options x 8 seeds (about
#                                           20 min) + ROS 2 services/actions and the ros2-style command line against the
#                                           PC stand-in of the chip, in real time (about 8 min)
# Needs: g++ (C++17), python3 with numpy, pyserial and rosbags (pip install numpy pyserial rosbags).
cd "$(dirname "$0")/.." || exit 1
B=$(mktemp -d)
FULL=0; [ "$1" = full ] && FULL=1
fails=0
step() { echo; echo "=== $1"; }
check() { if [ "$1" = 0 ]; then echo "--- PASS $2"; else echo "--- FAIL $2"; fails=$((fails + 1)); fi; }

python3 tools/gen_msgs.py >/dev/null || exit 1
build() { g++ -O2 -std=c++17 -Isrc "test_host/$1.cpp" -o "$B/$2" ${3:-} || { echo "build of $1 failed"; exit 1; }; }
step "build"
build test_main t; build test_bus tb; build test_vlidar tv; build test_nav tn; build test_cdr tcdr
build test_trig ttr; build test_bridge tbr -pthread; build test_mission_nodes tmn; build test_gz tgz "-pthread -lutil"; build test_real treal
echo "built in $B"

step "test_main: parts of the firmware that do not need the chip";     "$B/t" | tail -3; check ${PIPESTATUS[0]} test_main
step "test_trig: table sine/cosine against the C library";               "$B/ttr" | tail -5; check ${PIPESTATUS[0]} test_trig
step "test_bus: topics, executor, message conversions";                "$B/tb" | tail -3; check ${PIPESTATUS[0]} test_bus
step "test_vlidar: virtual X4 -> parser -> scan matching";             "$B/tv" | tail -3; check ${PIPESTATUS[0]} test_vlidar
step "test_nav: node version = single loop (virtual driving)";        "$B/tn" 0 1 | tail -3; check ${PIPESTATUS[0]} test_nav
step "check_cdr: the firmware's CDR = rosbags (an independent ROS 2 implementation)"
TCDR="$B/tcdr" python3 tools/check_cdr.py 2>&1 | tail -3; check ${PIPESTATUS[0]} check_cdr
step "test_bridge + check_link: the bytes the chip sends to the PC"
"$B/tbr" 0 1 0 "$B/bridge.bin" | tail -2; r1=${PIPESTATUS[0]}
python3 tools/check_link.py "$B/bridge.bin" | tail -3; r2=${PIPESTATUS[0]}; check $((r1 | r2)) test_bridge
step "test_real: real-robot mode ([r]) with a simulated Jarabot: keyboard mapping, two goals"
"$B/treal" | tail -2; check ${PIPESTATUS[0]} test_real
if [ $FULL = 1 ]; then
  step "node version = single loop: all worlds, modes and options x 8 seeds"
  last=$(cd test_host && bash cmpall.sh "$B/tmn" | tail -1); echo "$last"
  echo "$last" | grep -q " 0 differ"; check $? cmpall
else
  step "node version = single loop: worlds 0 and 4, kidnap + unmapped box, seeds 0 1"
  ok=0; for w in 0 4; do OPTS=3 "$B/tmn" $w 1 0 1 | tail -1 | tee /dev/stderr | grep -q "same as the single loop" || ok=1; done; check $ok test_mission_nodes
fi
if [ $FULL = 1 ]; then
  for t in check_ros check_cli; do
    step "$t against the PC stand-in (real time)"
    "$B/tgz" > "$B/tgz.log" 2>&1 & sleep 1; PTS=$(head -1 "$B/tgz.log")
    if [ $t = check_ros ]; then python3 -u tools/check_ros.py --port "$PTS" --command "k 0 1 0 3 rt" > "$B/$t.txt" 2>&1
    else python3 -u tools/check_cli.py --port "$PTS" > "$B/$t.txt" 2>&1; fi
    r=$?; grep -E "^FAIL" "$B/$t.txt"; tail -1 "$B/$t.txt"
    check $r $t; kill %1 2>/dev/null; wait 2>/dev/null; sleep 2
  done
fi
echo
[ $fails = 0 ] && echo "PC tests: PASS" || echo "PC tests: FAIL ($fails)"
rm -rf "$B"
[ $fails = 0 ]
