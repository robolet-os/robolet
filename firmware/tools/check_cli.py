#!/usr/bin/env python3
"""Checks tools/ros2.py (the ros2-style command line) end to end against a run on the chip.

  python3 tools/check_cli.py                    the chip on the first ESP32 port (firmware 0.14 or later)
  python3 tools/check_cli.py --port /dev/pts/3  the PC stand-in (test_host/test_gz.cpp)

Starts `ros2.py daemon` (room mission in real time, "hold"), then runs the ros2.py commands as a user would, each as
its own process: node / topic / service / param / lifecycle / action / interface. After the mission's legs it sends
two goals with `action send_goal` (one reached, one canceled with Ctrl+C) and ends the run with `mission stop`.
Each check prints PASS or FAIL; the last line is RESULT cli: ...
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
results = []


def check(what, ok, detail=""):
    results.append(bool(ok))
    print("%s %s%s" % ("PASS" if ok else "FAIL", what, (" | " + detail) if detail else ""), flush=True)


def ros2(*args, timeout=30):
    try:
        r = subprocess.run([PY, os.path.join(HERE, "ros2.py")] + list(args), capture_output=True, text=True, timeout=timeout)
        return r.stdout + r.stderr
    except subprocess.TimeoutExpired as e:
        return (e.stdout or b"").decode(errors="replace") if isinstance(e.stdout, bytes) else (e.stdout or "")


def first(out, n=1):
    return " / ".join(l.strip() for l in out.strip().splitlines()[:n])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--record")
    args = ap.parse_args()
    cmd = [PY, "-u", os.path.join(HERE, "ros2.py"), "daemon", "--chip", "k 0 1 0 3 rt hold"]
    if args.port:
        cmd += ["--port", args.port]
    if args.record:
        cmd += ["--record", args.record]
    d = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    log = []
    ev_serving, ev_hold, ev_over = threading.Event(), threading.Event(), threading.Event()

    def pump():
        for line in d.stdout:
            line = line.rstrip("\n")
            log.append(line)
            if "chip: [k]" in line or "MISSION" in line or "RESULT" in line or "[daemon]" in line:
                print("   " + line.strip(), flush=True)
            if "serving the ros2 commands" in line:
                ev_serving.set()
            if "holding" in line:
                ev_hold.set()
            if "the run is over" in line:
                ev_over.set()
        ev_over.set()
    threading.Thread(target=pump, daemon=True).start()
    if not ev_serving.wait(30):
        d.kill()
        sys.exit("the daemon did not start:\n" + "\n".join(log[-10:]))

    out = ros2("node", "list")
    check("node list", "/bt_navigator" in out and "/controller_server" in out, "%d nodes" % len(out.split()))
    out = ros2("node", "info", "/bt_navigator")
    check("node info /bt_navigator", "Action Servers:" in out and "/navigate_to_pose: nav2_msgs/action/NavigateToPose" in out)
    out = ros2("topic", "list")
    check("topic list (hidden topics left out)", "/scan" in out.split() and "/_action/" not in out, "%d topics" % len(out.split()))
    out = ros2("topic", "list", "-t")
    check("topic list -t", "/scan [sensor_msgs/msg/LaserScan]" in out)
    out = ros2("topic", "info", "/scan")
    check("topic info /scan", "Type: sensor_msgs/msg/LaserScan" in out and "Publisher count: 1" in out, first(out, 3))
    out = ros2("topic", "echo", "/pose", "--once")
    check("topic echo --once", "frame_id: map" in out and out.strip().endswith("---"), first(out, 4))
    out = ros2("topic", "hz", "/scan", timeout=7)
    m = re.findall(r"average rate: ([\d.]+)", out)
    check("topic hz /scan (7 Hz)", m and 6.5 < float(m[-1]) < 7.5, first(out, 2))
    out = ros2("service", "list", "-t")
    check("service list -t", "/mission/stop [std_srvs/srv/Trigger]" in out and "/_action/" not in out, "%d services" % len(out.split("\n")))
    out = ros2("service", "type", "/amcl/get_parameters")
    check("service type", out.strip() == "rcl_interfaces/srv/GetParameters", out.strip())
    out = ros2("service", "call", "/controller_server/get_parameters", "{names: [max_vel_x]}")
    check("service call", "double_value: 0.3" in out)
    out = ros2("param", "list", "/controller_server")
    check("param list", "max_vel_x" in out and "use_sim_time" in out, first(out, 3))
    out = ros2("param", "get", "/controller_server", "max_vel_x")
    check("param get", "Double value is: 0.3" in out, out.strip())
    out = ros2("param", "set", "/controller_server", "max_vel_x", "0.25")
    out2 = ros2("param", "get", "/controller_server", "max_vel_x")
    check("param set", "Set parameter successful" in out and "0.25" in out2, out2.strip())
    out = ros2("param", "set", "/controller_server", "max_vel_x", "5.0")
    check("param set refused (out of range)", "Setting parameter failed: value out of range" in out, out.strip())
    out = ros2("param", "set", "/controller_server", "max_vel_x", "0.3")
    check("param set back", "successful" in out)
    out = ros2("param", "describe", "/controller_server", "max_vel_x")
    check("param describe", "Type: double" in out and "Max value: 1.0" in out)
    out = ros2("param", "dump", "/bt_navigator")
    check("param dump", "ros__parameters:" in out and "leg_timeout: 150.0" in out)
    out = ros2("lifecycle", "nodes")
    check("lifecycle nodes", len(out.split()) == 9 and "/bt_navigator" in out, "%d nodes" % len(out.split()))
    out = ros2("lifecycle", "get", "/controller_server")
    check("lifecycle get", out.strip() == "active [3]", out.strip())
    out = ros2("lifecycle", "list", "/controller_server")
    check("lifecycle list", "- deactivate [4]" in out and "Goal: deactivating" in out)
    out = ros2("lifecycle", "set", "/controller_server", "deactivate")
    out2 = ros2("lifecycle", "get", "/controller_server")
    out3 = ros2("lifecycle", "set", "/controller_server", "activate")
    check("lifecycle set deactivate / activate", "successful" in out and out2.strip() == "inactive [2]" and "successful" in out3, out2.strip())
    out = ros2("action", "list", "-t")
    check("action list -t", "/navigate_to_pose [nav2_msgs/action/NavigateToPose]" in out and "/navigate_through_poses" in out)
    out = ros2("action", "info", "/navigate_to_pose")
    check("action info", "Action servers: 1" in out and "/bt_navigator" in out)
    out = ros2("interface", "show", "nav2_msgs/action/NavigateToPose")
    check("interface show", "string behavior_tree" in out and "float32 distance_remaining" in out)
    # topic pub: a topic the chip makes when the first message comes; an echo started first gets the messages
    ros2("topic", "pub", "/cli_test", "std_msgs/msg/Int32", "{data: 1}")
    time.sleep(0.5)
    e = subprocess.Popen([PY, os.path.join(HERE, "ros2.py"), "topic", "echo", "/cli_test", "--once"], stdout=subprocess.PIPE, text=True)
    time.sleep(1.5)
    out = ros2("topic", "pub", "/cli_test", "std_msgs/msg/Int32", "{data: 42}", "--times", "3", "--rate", "2")
    try:
        eo = e.communicate(timeout=10)[0]
    except subprocess.TimeoutExpired:
        e.kill(); eo = ""
    check("topic pub + echo", "publishing #3" in out and "data: 42" in eo, first(eo))

    print("[cli] waiting for the mission's legs (then the robot waits for goals) ...", flush=True)
    if not ev_hold.wait(600):
        check("mission reaches hold", False)
    else:
        out = ros2("action", "send_goal", "/navigate_to_pose", "nav2_msgs/action/NavigateToPose",
                   "{pose: {pose: {position: {x: 6.5, y: 1.5}}}}", "--feedback", timeout=240)
        dl = [float(x) for x in re.findall(r"distance_remaining: ([\d.e+-]+)", out)]
        check("action send_goal (reached)", "Goal finished with status: SUCCEEDED" in out and dl and max(dl) > 3 and min(dl) < 0.5,
              "%d feedback messages, distance left %.2f..%.2f m" % (len(dl), min(dl or [0]), max(dl or [0])))
        p = subprocess.Popen([PY, os.path.join(HERE, "ros2.py"), "action", "send_goal", "/navigate_to_pose",
                              "{pose: {pose: {position: {x: 2.3, y: 5.3}}}}"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        buf = []
        t0 = time.time()
        for line in p.stdout:
            buf.append(line)
            if line.startswith("Goal accepted") or time.time() - t0 > 20:
                break
        time.sleep(4)
        p.send_signal(signal.SIGINT)
        try:
            rest = p.communicate(timeout=30)[0]
        except subprocess.TimeoutExpired:
            p.kill(); rest = ""
        out = "".join(buf) + rest
        check("action send_goal canceled with Ctrl+C", "Goal finished with status: CANCELED" in out, first(out[out.find("Canceling"):] if "Canceling" in out else out[-200:]))
        out = ros2("mission", "stop")
        check("mission stop", "the run ends" in out, out.strip())
    ev_over.wait(60)
    try:
        d.wait(10)
    except subprocess.TimeoutExpired:
        d.kill()
    mis = [l for l in log if "MISSION" in l]
    check("the run ended", bool(mis) and ev_over.is_set(), mis[-1].strip()[:110] if mis else "")
    ok = all(results)
    print("RESULT cli: %s (%d of %d checks)" % ("PASS" if ok else "FAIL", sum(results), len(results)), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
