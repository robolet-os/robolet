#!/usr/bin/env python3
"""ROS 2 bridge: the chip's topics on a ROS 2 network (rviz2, ros2 topic, ros2 bag), and the rviz2 tools back to the
chip. Runs on a PC with ROS 2 (Humble or later) and talks to the chip through the daemon of tools/ros2.py, so the
ros2.py commands keep working at the same time. The chip's CDR bytes are published unchanged (no conversion).

  python3 tools/ros2.py daemon --chip "k 0 1 0 3 rt hold"        terminal 1: room mission in real time, waits for goals
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 tools/ros2_bridge.py                           terminal 2 (the Python that ROS 2 was built for:
                                                                  3.10 on Ubuntu 22.04, not a pyenv/conda Python)
  rviz2 -d tools/robolet.rviz --ros-args -p use_sim_time:=true    terminal 3

  --topics /map:1,/tf,...   chip -> ROS 2 (name[:max Hz]); default: DEFAULT_TOPICS below
  --no-goals                do not forward the rviz2 tools to the chip

From ROS 2 to the chip:
  /goal_pose    (rviz2 "2D Goal Pose")     -> a navigate_to_pose goal on the chip (as Nav2's bt_navigator does)
  /initialpose  (rviz2 "2D Pose Estimate") -> /initialpose on the chip
  /teleop/cmd_vel (keyboard)               -> the real robot's motors (firmware [r]):
      ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/teleop/cmd_vel
Services and actions of the chip are not on the ROS 2 network yet: use tools/ros2.py for them.
"""
import argparse
import base64
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ros2wire as rw  # noqa: E402
import ros2 as r2  # noqa: E402

DEFAULT_TOPICS = ["/clock", "/tf", "/odom", "/map:1", "/scan_cloud", "/amcl/robot_pose", "/amcl_pose", "/pose",
                  "/plan:2", "/localization/candidates", "/ground_truth", "/cmd_vel", "/rosout"]
LATCHED = {"/map", "/navigator/phase", "/navigator/leg"}         # latched on the chip: transient local in ROS 2
TO_CHIP = ["/goal_pose", "/initialpose", "/teleop/cmd_vel"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topics", help="comma-separated name[:max Hz]")
    ap.add_argument("--no-goals", action="store_true")
    a = ap.parse_args()

    try:
        import rclpy
    except ImportError as e:
        sys.exit("cannot import rclpy with Python %d.%d (%s).\nRun the bridge with the Python ROS 2 was built for, after "
                 "sourcing ROS 2:  source /opt/ros/humble/setup.bash && /usr/bin/python3 tools/ros2_bridge.py"
                 % (sys.version_info[0], sys.version_info[1], e))
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    cl = r2.Client()
    g = cl.graph()
    if g.get("done"):
        sys.exit("the run on the chip is over: start a new one (tools/ros2.py daemon --chip \"k 0 1 0 3 rt hold\")")
    types = g["topics"]                                         # name -> type, as the chip advertises them

    rclpy.init()
    node = Node("robolet_bridge")
    log = node.get_logger()
    wanted = [t.strip() for t in a.topics.split(",")] if a.topics else DEFAULT_TOPICS
    to_chip = [] if a.no_goals else TO_CHIP
    pubs, groups = {}, {}
    for w in wanted:
        name, _, hz = w.partition(":")
        if name in to_chip:
            log.warn("%s goes to the chip; not also bridged back (it would come back to itself)" % name)
            continue
        if name not in types:
            log.warn("the chip has no topic %s (skipped)" % name)
            continue
        cls = get_message(types[name])
        if name in LATCHED:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        else:
            qos = QoSProfile(depth=100 if name == "/tf" else 10, reliability=ReliabilityPolicy.RELIABLE)
        pubs[name] = (node.create_publisher(cls, name, qos), cls)
        groups.setdefault(float(hz) if hz else 0.0, []).append(name)

    raw_ok = [True]                                             # rclpy publishes bytes as they are (Galactic and later)

    def publish(name, cdr):
        pub, cls = pubs[name]
        if raw_ok[0]:
            try:
                pub.publish(cdr)
                return
            except TypeError:
                raw_ok[0] = False
                log.warn("this rclpy cannot publish CDR bytes directly: converting each message")
        pub.publish(deserialize_message(cdr, cls))

    stop = threading.Event()
    count = {}

    def forward(names, hz):
        try:
            for r in r2.Client().stream(names, hz, raw=True):
                if stop.is_set():
                    return
                publish(r["topic"], base64.b64decode(r["cdr"]))
                count[r["topic"]] = count.get(r["topic"], 0) + 1
        except Exception as e:                                  # noqa: BLE001
            log.error("stream %s: %s" % (",".join(names), e))
        stop.set()

    for hz, names in groups.items():
        threading.Thread(target=forward, args=(names, hz), daemon=True).start()

    # ---- ROS 2 -> chip
    if "/goal_pose" in to_chip:
        def on_goal(cdr):
            goal = rw.decode("geometry_msgs/msg/PoseStamped", bytes(cdr))
            req = {"goal_id": {"uuid": list(os.urandom(16))}, "goal": {"pose": r2.jsonable(goal)}}
            p = goal["pose"]["position"]

            def send():
                try:
                    r = cl.call("/navigate_to_pose/_action/send_goal", req, "nav2_msgs/action/NavigateToPose_SendGoal")
                    log.info("goal (%.2f, %.2f): %s" % (p["x"], p["y"], "accepted" if r["accepted"] else "rejected"))
                except Exception as e:                          # noqa: BLE001
                    log.error("goal: %s" % e)
            threading.Thread(target=send, daemon=True).start()
        node.create_subscription(get_message("geometry_msgs/msg/PoseStamped"), "/goal_pose", on_goal, 10, raw=True)
    if "/initialpose" in to_chip:
        typ = "geometry_msgs/msg/PoseWithCovarianceStamped"

        def on_initialpose(cdr):
            cl.request(op="pubraw", topic="/initialpose", type=typ, cdr=base64.b64encode(bytes(cdr)).decode())
            log.info("initial pose sent to the chip")
        node.create_subscription(get_message(typ), "/initialpose", on_initialpose, 10, raw=True)

    if "/teleop/cmd_vel" in to_chip:
        def on_teleop(cdr):
            cl.request(op="pubraw", topic="/teleop/cmd_vel", type="geometry_msgs/msg/Twist", cdr=base64.b64encode(bytes(cdr)).decode())
        node.create_subscription(get_message("geometry_msgs/msg/Twist"), "/teleop/cmd_vel", on_teleop, 10, raw=True)

    log.info("chip: %s" % (g.get("hello") or "(attached)"))
    log.info("chip -> ROS 2: %s" % " ".join(sorted(pubs)))
    log.info("ROS 2 -> chip: %s" % (" ".join(to_chip) or "(none)"))
    t_sum = time.time()
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.time() - t_sum > 10:
                log.info("forwarded: " + ", ".join("%s %d" % (k, v) for k, v in sorted(count.items())))
                t_sum = time.time()
    except KeyboardInterrupt:
        pass
    stop.set()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
