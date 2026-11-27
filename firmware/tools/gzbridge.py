#!/usr/bin/python3
"""Chip <-> Gazebo (hardware in the loop): the chip runs the whole robot stack, Gazebo plays the robot, its lidar and
the room. Needs Gazebo Harmonic (gz sim 8) with its Python modules and pyserial, all for the system Python:
    sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-serial

  /usr/bin/python3 tools/gzbridge.py --chip "k 0 1 0 3"          room mission: starts Gazebo (no window), runs the chip
       --gui            also open the Gazebo window (watch the robot in 3D)
       --view           also the browser viewer (map, poses, lidar, path: http://localhost:8765)
       --rt             real time (one cycle per 1/7 s); default: as fast as the chip and Gazebo can
       --record f.bin   save what the chip sent (tools/viewer.py --play f.bin shows it again)
       --port /dev/ttyACM0, --no-gazebo (Gazebo already running with gazebo/worlds/<world>.sdf)

One cycle (lock step, so a run does not depend on how fast the PC is):
  chip  : runs its nodes for cycle t, sends /cmd_vel (and /sim/teleport, /sim/box when the mission asks), then /gz/done t
  here  : gives Gazebo the velocity (and moves the robot / places the box), steps the world by 1/7 s (100 physics steps),
          waits for the new lidar scan, then sends /ground_truth, /odom, /gz/scan, /gz/contacts and /gz/step t+1
  chip  : waits for /gz/step t+1 (watchdog: 5 s without data stops the run), turns /gz/scan into X4 bytes, next cycle
Odometry is Gazebo's wheel odometry (the robot does not feel the kidnap); /ground_truth is the model pose in Gazebo.
"""
import argparse
import math
import os
import struct
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ros2wire as rw  # noqa: E402
import robolet  # noqa: E402

WORLDS = ["room", "s_corridor", "s_corridor_lm", "doors", "room_tilted"]   # mission_core.h world numbers
GZ_WORLD = "robolet"                                    # <world name> inside the .sdf files
STEPS_PER_TICK = 100                                     # physics step 1/700 s, lidar 7 Hz
DT = 1.0 / 7.0
CONTROL_SUBS = ["/cmd_vel", "/sim/teleport", "/sim/box", "/gz/done"]   # /gz/done last: it closes the cycle


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat(yaw):
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}


def header(t, frame):
    sec = int(t)
    return {"stamp": {"sec": sec, "nanosec": int(round((t - sec) * 1e9))}, "frame_id": frame}


class Gazebo:
    """Gazebo through gz-transport: lidar, pose, wheel odometry, contacts in; velocity, stepping, set_pose, create out"""
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.contacts_pb2 import Contacts
        from gz.msgs10.twist_pb2 import Twist
        self.Twist = Twist
        self.node, self.lock, self.cv = Node(), threading.Lock(), threading.Condition()
        self.scan, self.scan_n, self.pose, self.odom, self.touch = None, 0, None, None, False
        self.node.subscribe(LaserScan, "/robot/scan", self._on_scan)
        self.node.subscribe(Pose_V, "/model/robot/pose", self._on_pose)
        self.node.subscribe(Odometry, "/robot/odom", self._on_odom)
        self.node.subscribe(Contacts, "/world/%s/model/robot/link/base_link/sensor/bumper/contact" % GZ_WORLD, self._on_contact)
        self.cmd = self.node.advertise("/robot/cmd_vel", Twist)

    def _on_scan(self, m):
        with self.cv:
            self.scan, self.scan_n = m, self.scan_n + 1
            self.cv.notify_all()

    def _on_pose(self, m):
        for p in m.pose:
            if p.name == "robot":
                with self.lock:
                    self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_odom(self, m):
        with self.lock:
            self.odom = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation), m.twist.linear.x, m.twist.angular.z)

    def _on_contact(self, m):
        if len(m.contact):
            with self.lock:
                self.touch = True

    def ready(self, timeout=60):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if "/world/%s/control" % GZ_WORLD in self.node.service_list():
                return True
            time.sleep(0.2)
        return False

    def request(self, service, req, rep_type, timeout_ms=5000):
        ok, rep = self.node.request(service, req, type(req), rep_type, timeout_ms)
        return ok and getattr(rep, "data", True)

    def step(self, n, timeout=10.0):
        """steps the (paused) world n physics steps and waits for the lidar scan this produces"""
        from gz.msgs10.world_control_pb2 import WorldControl
        from gz.msgs10.boolean_pb2 import Boolean
        with self.cv:
            n0 = self.scan_n
        req = WorldControl(); req.pause = True; req.multi_step = n
        if not self.request("/world/%s/control" % GZ_WORLD, req, Boolean):
            raise RuntimeError("Gazebo did not accept the step request")
        with self.cv:
            if not self.cv.wait_for(lambda: self.scan_n > n0, timeout):
                return False
        time.sleep(0.002)                                # the pose / odometry of the same step (published every step)
        return True

    def velocity(self, v, w):
        t = self.Twist(); t.linear.x = v; t.angular.z = w
        self.cmd.publish(t)

    def set_pose(self, x, y, yaw):
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.boolean_pb2 import Boolean
        p = Pose(); p.name = "robot"; p.position.x, p.position.y, p.position.z = x, y, 0.0
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return self.request("/world/%s/set_pose" % GZ_WORLD, p, Boolean)

    def add_box(self, x, y, yaw, sx, sy):
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.boolean_pb2 import Boolean
        sdf = ('<sdf version="1.9"><model name="unmapped_box"><static>true</static><pose>%.3f %.3f 0.25 0 0 %.5f</pose><link name="l">'
               '<collision name="c"><geometry><box><size>%.3f %.3f 0.5</size></box></geometry></collision>'
               '<visual name="v"><geometry><box><size>%.3f %.3f 0.5</size></box></geometry>'
               '<material><ambient>0.9 0.5 0.1 1</ambient><diffuse>0.9 0.5 0.1 1</diffuse></material></visual>'
               '</link></model></sdf>') % (x, y, yaw, sx, sy, sx, sy)
        req = EntityFactory(); req.sdf = sdf
        return self.request("/world/%s/create" % GZ_WORLD, req, Boolean)

    def take(self):
        with self.lock:
            touch, self.touch = self.touch, False
            return self.scan, self.pose, self.odom, touch


def publish_frame(name, typ, msg):
    return rw.frame(rw.KIND["PUBLISH"], 0, name.encode() + b"\0" + typ.encode() + b"\0" + rw.encode(typ, msg))


class Link:
    """the serial port to the chip: frames in (with the robolet Session), frames out"""
    def __init__(self, port, record=None):
        import serial
        self.s = serial.Serial(port, 115200, timeout=0.01)
        self.parser, self.rec = rw.FrameParser(), (open(record, "wb", buffering=0) if record else None)
        self.port, self.error = port, None

    def send(self, b):
        self.s.write(b)

    def read(self):
        try:
            data = self.s.read(65536)
        except Exception as e:                           # port gone (cable, reset, or the PC stand-in has ended)
            self.error = str(e)
            return None
        if data and self.rec:
            self.rec.write(data)
        return self.parser.feed(data) if data else []


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--chip", default="k 0 1 0 3", help='mission command: k <world> <mode> <seed> <options> (default "k 0 1 0 3")')
    ap.add_argument("--port", help="serial port of the chip (default: the first ESP32 port found)")
    ap.add_argument("--gui", action="store_true", help="also open the Gazebo window")
    ap.add_argument("--view", action="store_true", help="also show the browser viewer (http://localhost:8765)")
    ap.add_argument("--rt", action="store_true", help="real time: at most one cycle per 1/7 s")
    ap.add_argument("--record", help="save the bytes from the chip (play: tools/viewer.py --play FILE)")
    ap.add_argument("--csv", help="one line per cycle: true pose, odometry, velocity command, SLAM / AMCL estimate")
    ap.add_argument("--no-gazebo", action="store_true", help="do not start Gazebo (it already runs the right world, paused)")
    args = ap.parse_args()

    parts = args.chip.split()
    if not parts or parts[0] != "k":
        sys.exit("--chip must be a single mission: k <world> <mode> <seed> <options>")
    world = int(parts[1]) if len(parts) > 1 else 0
    world_file = os.path.join(os.path.dirname(HERE), "gazebo", "worlds", WORLDS[world] + ".sdf")
    if not os.path.exists(world_file):
        sys.exit("[gz] %s is missing: cd test_host && g++ -O2 -std=c++17 -I../src gz_worlds.cpp -o gzw && ./gzw ../gazebo/worlds" % world_file)
    procs = []

    def start_gazebo(headless):                          # the server (paused); the lidar needs rendering (EGL if headless)
        cmd = ["gz", "sim", "-s"] + (["--headless-rendering"] if headless else []) + [world_file]
        log = open(os.path.join(os.path.dirname(HERE), "gazebo", "server.log"), "w")
        procs.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT))
        print("[gz] Gazebo: %s (log: gazebo/server.log)" % " ".join(cmd), flush=True)
    args.start_gazebo = None if args.no_gazebo else start_gazebo
    args.procs = procs
    if args.gui:
        procs.append(subprocess.Popen(["gz", "sim", "-g"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    state = None
    if args.view:                                        # the browser viewer, fed from the same link
        import viewer
        from http.server import ThreadingHTTPServer
        state = viewer.State()
        srv = ThreadingHTTPServer(("127.0.0.1", 8765), viewer.make_handler(state))
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        import webbrowser
        threading.Timer(1.0, lambda: webbrowser.open("http://localhost:8765")).start()
        print("[gz] viewer: http://localhost:8765", flush=True)
    try:
        run(args, state)
    except KeyboardInterrupt:
        print("\n[gz] stopped (the chip stops by itself 5 s after the last data: watchdog)")
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()


def run(args, state):
    gz = Gazebo()
    for headless in (True, False):                       # headless rendering first; if the lidar stays silent, without
        if args.start_gazebo:
            args.start_gazebo(headless)
        if not gz.ready():
            sys.exit("[gz] Gazebo did not start: see gazebo/server.log")
        gz.velocity(0, 0)
        if gz.step(STEPS_PER_TICK + 1, timeout=20):      # the lidar makes its first scan one physics step after 1/7 s
            break
        if not args.start_gazebo or not headless:
            sys.exit("[gz] no lidar scan from Gazebo (the lidar needs rendering: see gazebo/server.log and gazebo/README.md)")
        print("[gz] no lidar scan with headless rendering: starting Gazebo again without it", flush=True)
        p = args.procs.pop()                             # the server (started last)
        p.terminate(); p.wait(10); time.sleep(2)
    scan, pose, odom, _ = gz.take()
    anchor = pose                                        # wheel odometry starts at 0: put it at the start pose
    print("[gz] Gazebo ready: robot at (%.2f, %.2f), lidar %d ranges" % (pose[0], pose[1], len(scan.ranges)), flush=True)

    link = Link(args.port or robolet.find_port(), args.record)
    cmd = args.chip.strip()
    for opt in ("gz", "bridge"):
        if opt not in cmd.split():
            cmd += " " + opt
    link.s.reset_input_buffer()
    link.send(b"\n" + cmd.encode() + b"\n")
    print("[gz] %s: sent '%s'" % (link.port, cmd), flush=True)

    ctl = {"cmd": None, "teleport": None, "box": None, "done": None}
    est, last_cmd = [None], [(0.0, 0.0)]                 # latest SLAM / AMCL pose from the chip, last velocity (for --csv)
    contacts = [0]

    def on_message(name, typ, m):
        if name == "/cmd_vel":
            ctl["cmd"] = (float(m["linear"]["x"]), float(m["angular"]["z"]))
        elif name == "/sim/teleport":
            p = m["pose"]; ctl["teleport"] = (float(p["position"]["x"]), float(p["position"]["y"]), 2 * math.atan2(float(p["orientation"]["z"]), float(p["orientation"]["w"])))
        elif name == "/sim/box":
            c, s = m["poses"][0], m["poses"][1]
            ctl["box"] = (float(c["position"]["x"]), float(c["position"]["y"]), 2 * math.atan2(float(c["orientation"]["z"]), float(c["orientation"]["w"])),
                          float(s["position"]["x"]), float(s["position"]["y"]))
        elif name in ("/pose", "/amcl/robot_pose"):
            p = m["pose"]["pose"] if name == "/pose" else m["pose"]
            est[0] = ("%.4f" % float(p["position"]["x"]), "%.4f" % float(p["position"]["y"]),
                      "%.4f" % (2 * math.atan2(float(p["orientation"]["z"]), float(p["orientation"]["w"]))), "slam" if name == "/pose" else "amcl")
        elif name == "/gz/done":
            ctl["done"] = int(m["data"])
        if state is not None:
            state.message(name, typ, m)

    sess = robolet.Session(quiet=True, on_message=on_message)

    def chip_stats():                                    # the chip's STATS frame (src/bridge.h), once a second
        if not sess.stats:
            return ""
        s = sess.stats
        return "; chip: %d frames sent, %d dropped, %d frames from the PC, %d damaged" % (s[0], s[1], s[3], s[2])

    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write("cycle,true_x,true_y,true_th,odom_x,odom_y,odom_th,cmd_v,cmd_w,est_x,est_y,est_th,est_src\n")

    def send_state(tick, scan, pose, odom):
        t = tick * DT
        ax, ay, ath = anchor
        ox, oy, oth, ov, ow = odom
        odom_xy = (ax + math.cos(ath) * ox - math.sin(ath) * oy, ay + math.sin(ath) * ox + math.cos(ath) * oy)
        if csv:
            e = est[0] or ("", "", "", "")
            csv.write("%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%s,%s,%s,%s\n" % ((tick, pose[0], pose[1], pose[2], odom_xy[0], odom_xy[1],
                      math.atan2(math.sin(ath + oth), math.cos(ath + oth))) + tuple(last_cmd[0]) + tuple(e)))
        out = publish_frame("/ground_truth", "nav_msgs/msg/Odometry", {
            "header": header(t, "map"), "child_frame_id": "base_link",
            "pose": {"pose": {"position": {"x": pose[0], "y": pose[1], "z": 0.0}, "orientation": quat(pose[2])}}})
        out += publish_frame("/odom", "nav_msgs/msg/Odometry", {
            "header": header(t, "odom"), "child_frame_id": "base_link",
            "pose": {"pose": {"position": {"x": odom_xy[0], "y": odom_xy[1], "z": 0.0}, "orientation": quat(ath + oth)}},
            "twist": {"twist": {"linear": {"x": ov}, "angular": {"z": ow}}}})
        out += publish_frame("/gz/scan", "sensor_msgs/msg/LaserScan", {
            "header": header(t, "laser"), "angle_min": scan.angle_min, "angle_max": scan.angle_max,
            "angle_increment": scan.angle_step, "scan_time": DT, "range_min": scan.range_min, "range_max": scan.range_max,
            "ranges": list(scan.ranges), "intensities": []})
        out += publish_frame("/gz/contacts", "std_msgs/msg/Int32", {"data": contacts[0]})
        out += publish_frame("/gz/step", "std_msgs/msg/Int32", {"data": tick})   # last: the chip may start the cycle
        link.send(out)
        if state is not None:
            state.message("/ground_truth", "", {"pose": {"pose": {"position": {"x": pose[0], "y": pose[1]}, "orientation": quat(pose[2])}}})
        return out

    tick, subscribed, sent, t_sent, t_wall, t_report, t_end = 0, False, None, 0.0, time.time(), time.time(), None
    stats = {"resends": 0, "late_scans": 0}
    while True:
        items = link.read()
        if items is None:
            if not sess.done:
                print("[gz] serial port: %s" % link.error, flush=True)
            break
        for it in items:
            if it[0] == "text":
                if state is not None:
                    state.text(it[1])                    # (prints it too)
                else:
                    print(it[1], flush=True)
        sess.handle(items)
        now = time.time()
        if sess.hello and not subscribed:
            subs = (viewer_subs() if state is not None or args.record or args.csv else []) + CONTROL_SUBS
            link.send(robolet.subscribe_frames(subs)); subscribed = True
            print("[gz] chip: " + sess.hello, flush=True)
            sent = send_state(0, scan, pose, odom); t_sent = now
        if sess.done and t_end is None:
            t_end = now + 1.0
        if t_end and now > t_end:
            break
        if not subscribed:
            if now - t_wall > 15:
                sys.exit("[gz] no answer from the chip on %s (firmware 0.11 or later? press RST and try again)" % link.port)
            continue
        if sess.done:                                    # the run is over: send nothing more (it would reach the console)
            continue
        if ctl["done"] is not None and ctl["done"] >= tick:   # the chip finished cycle `tick`: move Gazebo on
            ctl["done"] = None
            v, w = ctl["cmd"] or (0.0, 0.0); ctl["cmd"] = None; last_cmd[0] = (v, w)
            if ctl["teleport"]:
                gz.set_pose(*ctl["teleport"]); print("[gz] robot carried to (%.2f, %.2f) (kidnap)" % ctl["teleport"][:2], flush=True)
                ctl["teleport"] = None
            if ctl["box"]:
                gz.add_box(*ctl["box"]); print("[gz] box placed at (%.2f, %.2f), %.2f x %.2f m" % (ctl["box"][0], ctl["box"][1], ctl["box"][3], ctl["box"][4]), flush=True)
                ctl["box"] = None
            gz.velocity(v, w)
            if args.rt:                                  # real time: one cycle per 1/7 s of wall clock
                wait = t_wall + DT - time.time()
                if wait > 0:
                    time.sleep(wait)
            t_wall = time.time()
            if not gz.step(STEPS_PER_TICK):
                stats["late_scans"] += 1
                print("[gz] warning: no new lidar scan after the step (cycle %d)" % tick, flush=True)
            scan, pose, odom, touch = gz.take()
            if touch:
                contacts[0] += 1
            tick += 1
            sent = send_state(tick, scan, pose, odom); t_sent = time.time()
        elif sent and now - t_sent > 1.0:               # frames lost on the way: send the cycle's data again
            link.send(sent); t_sent = now; stats["resends"] += 1
            print("[gz] no /gz/done for cycle %d after 1 s: its data sent again (%d so far)%s" % (tick, stats["resends"], chip_stats()), flush=True)
            if now - t_wall > 30:
                print("[gz] no answer from the chip for 30 s: giving up", flush=True); break
        if now - t_report > 10:
            t_report = now
            print("[gz] cycle %d (%.0f s simulated), robot at (%.2f, %.2f), contacts %d, sent again %d, %d frames from the chip (%d damaged)%s"
                  % (tick, tick * DT, pose[0], pose[1], contacts[0], stats["resends"], link.parser.frames, link.parser.bad_crc, chip_stats()), flush=True)
    print("[gz] run over: %d cycles, %d frames from the chip (%d damaged), sent again %d, scans late %d%s"
          % (tick, link.parser.frames, link.parser.bad_crc, stats["resends"], stats["late_scans"], chip_stats()), flush=True)
    if link.rec:
        link.rec.close(); print("[gz] recording: " + args.record, flush=True)
    if state is not None:
        print("[gz] the viewer page stays open until Ctrl+C", flush=True)
        while True:
            time.sleep(1)


def viewer_subs():
    import viewer
    return [s for s in viewer.SUBS if s.split(":")[0] not in ("/ground_truth",)]   # the truth comes from here


if __name__ == "__main__":
    main()
