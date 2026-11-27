#!/usr/bin/env python3
"""Checks the chip's services, parameters, lifecycle states and actions (src/ros.h) from the PC, during a mission.

  python3 tools/check_ros.py                     the chip on the first ESP32 port (firmware 0.12 or later)
  python3 tools/check_ros.py --port /dev/pts/3   the PC stand-in (test_host/test_gz.cpp; add --rt there: "k ... rt")

Starts "k 0 1 0 3 bridge" and, while the mission runs, does what these ROS 2 commands do:
  ros2 service list -t                     the services the chip offers
  ros2 node list                           the chip's nodes with their topics (NODE frames)
  ros2 param list / get / set              /controller_server: max_vel_x (also refused values: out of range, wrong
                                           type, read only, unknown name); use_sim_time
  ros2 param describe / get types          descriptor with range and read-only flag; types of several parameters
  set_parameters_atomically                all or nothing; /parameter_events after each change
  ros2 lifecycle get / list                /controller_server, /bt_navigator; available transitions; transition events
  ros2 topic echo /clock, /rosout          simulated time goes on; log lines with node names
  nav2 lifecycle manager                   manage_nodes PAUSE (all managed nodes inactive), is_active, RESUME
  ros2 action (status, feedback, cancel)   /navigate_to_pose/_action/status and feedback (also the mapping tour's);
                                           cancel_goal of the running leg
Then lets the mission end and prints the result. Each check prints PASS or FAIL; the last line is RESULT ros: ...
"""
import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ros2wire as rw  # noqa: E402
import robolet  # noqa: E402

PT = {"NOT_SET": 0, "BOOL": 1, "INTEGER": 2, "DOUBLE": 3, "STRING": 4, "STRING_ARRAY": 9}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--command", default="k 0 1 0 3", help="mission to run (default: k 0 1 0 3)")
    ap.add_argument("--record", help="save the bytes from the chip")
    args = ap.parse_args()
    import serial
    port = args.port or robolet.find_port()
    s = serial.Serial(port, 115200, timeout=0.02)
    rec = open(args.record, "wb") if args.record else None
    parser, sess = rw.FrameParser(), robolet.Session(quiet=True)
    got = {}                                                       # topic -> messages received
    sess.on_message = lambda name, typ, m: got.setdefault(name, []).append(m)
    status_log = got.setdefault("/navigate_to_pose/_action/status", [])
    cmd = args.command + " bridge"
    s.reset_input_buffer(); s.write(b"\n" + cmd.encode() + b"\n")
    print("[ros] %s: sent '%s'" % (port, cmd), flush=True)
    results = []

    def pump(t=0.05):
        t_end = time.time() + t
        while time.time() < t_end:
            try:
                data = s.read(65536)
            except Exception:                            # port closed (the PC stand-in ends after the run)
                sess.done = True; return
            if data:
                if rec:
                    rec.write(data)
                items = parser.feed(data)
                for it in items:
                    if it[0] == "text" and (it[1].startswith("[k]") or it[1].startswith("RESULT") or it[1].startswith("MISSION")):
                        print("   chip: " + it[1], flush=True)
                sess.handle(items)

    def call(service, request, timeout=5.0):
        cid, fr = sess.call_frame(service, request)
        s.write(fr)
        t0 = time.time()
        while time.time() - t0 < timeout:
            pump(0.02)
            r = sess.reply(cid, service)
            if r is not None:
                return r
        raise RuntimeError("%s: no reply in %.0f s" % (service, timeout))

    def check(what, ok, detail=""):
        results.append(ok)
        print("%s %s%s" % ("PASS" if ok else "FAIL", what, (" | " + detail) if detail else ""), flush=True)

    t0 = time.time()
    while not sess.hello and time.time() - t0 < 20:
        pump(0.1)
    if not sess.hello:
        sys.exit("no HELLO from %s (firmware 0.12 or later? press RST)" % port)
    print("[ros] chip: " + sess.hello, flush=True)
    s.write(robolet.subscribe_frames(["/navigate_to_pose/_action/status", "/parameter_events", "/controller_server/transition_event",
                                       "/clock:1", "/rosout", "/navigate_to_pose/_action/feedback:2",
                                       "/navigate_through_poses/_action/feedback:2"]))
    pump(0.5)

    # ros2 service list -t
    srv = sess.services
    need = ["/controller_server/get_parameters", "/controller_server/set_parameters", "/controller_server/list_parameters",
            "/controller_server/change_state", "/controller_server/get_state", "/lifecycle_manager_navigation/manage_nodes",
            "/lifecycle_manager_navigation/is_active", "/navigate_to_pose/_action/send_goal", "/navigate_to_pose/_action/cancel_goal",
            "/navigate_to_pose/_action/get_result", "/navigate_through_poses/_action/send_goal"]
    missing = [n for n in need if n not in srv]
    check("service list: %d services" % len(srv), not missing, "missing " + ", ".join(missing) if missing else
          "e.g. %s [%s]" % (need[0], srv.get(need[0])))

    # ros2 node list
    nd = sess.nodes
    nav_pubs = nd.get("bt_navigator", ([], []))[0]
    check("node list: %d nodes" % len(nd), all(n in nd for n in ("bt_navigator", "controller_server", "amcl", "slam", "lifecycle_manager_navigation"))
          and "/navigate_to_pose/_action/feedback" in nav_pubs, ", ".join(sorted(nd)))

    # ros2 param list / get / set
    names = [x for x in call("/controller_server/list_parameters", {})["result"]["names"]]
    check("param list /controller_server", "max_vel_x" in names, ", ".join(names))
    v = call("/controller_server/get_parameters", {"names": ["max_vel_x", "no_such_parameter"]})["values"]
    check("param get max_vel_x", v[0]["type"] == PT["DOUBLE"] and abs(v[0]["double_value"] - 0.3) < 1e-9 and v[1]["type"] == PT["NOT_SET"],
          "%s, unknown name -> type %d" % (v[0]["double_value"], v[1]["type"]))

    def setp(node, name, value):
        if isinstance(value, bool):
            pv = {"type": PT["BOOL"], "bool_value": value}
        elif isinstance(value, int):
            pv = {"type": PT["INTEGER"], "integer_value": value}
        elif isinstance(value, float):
            pv = {"type": PT["DOUBLE"], "double_value": value}
        else:
            pv = {"type": PT["STRING"], "string_value": value}
        r = call("/%s/set_parameters" % node, {"parameters": [{"name": name, "value": pv}]})["results"][0]
        return r["successful"], r["reason"]

    ok, why = setp("controller_server", "max_vel_x", 0.25)
    v = call("/controller_server/get_parameters", {"names": ["max_vel_x"]})["values"][0]["double_value"]
    check("param set max_vel_x 0.25", ok and abs(v - 0.25) < 1e-9, "now %s" % v)
    for value, label in ((5.0, "out of range"), ("fast", "wrong type")):
        ok, why = setp("controller_server", "max_vel_x", value)
        check("param set max_vel_x %r refused (%s)" % (value, label), not ok, why)
    ok, why = setp("amcl", "max_particles", 100)
    check("param set amcl max_particles refused (read only)", not ok, why)
    ok, why = setp("controller_server", "no_such_parameter", 1.0)
    check("param set unknown name refused", not ok, why)
    ok, why = setp("controller_server", "max_vel_x", 0.3)
    check("param set max_vel_x back to 0.3", ok)
    ok, why = setp("behavior_server", "max_failures", 4)
    check("param set behavior_server max_failures 4 (integer)", ok, why)
    ev = [e for e in got.get("/parameter_events", []) if e["node"] == "/controller_server"]
    ch = [(p["name"], p["value"]["double_value"]) for e in ev for p in e["changed_parameters"]]
    check("/parameter_events: max_vel_x changes", ("max_vel_x", 0.25) in ch and ("max_vel_x", 0.3) in ch, "%d events, %s" % (len(ev), ch[:4]))
    v = call("/controller_server/get_parameters", {"names": ["use_sim_time"]})["values"][0]
    check("param get use_sim_time (simulated clock)", v["type"] == PT["BOOL"] and v["bool_value"], "%s" % v["bool_value"])
    d = call("/controller_server/describe_parameters", {"names": ["max_vel_x"]})["descriptors"][0]
    fr = d["floating_point_range"]
    check("param describe max_vel_x", d["type"] == PT["DOUBLE"] and len(fr) == 1 and fr[0]["from_value"] == 0.0 and fr[0]["to_value"] == 1.0
          and not d["read_only"], "'%s', range %s" % (d["description"], [(r["from_value"], r["to_value"]) for r in fr]))
    d = call("/amcl/describe_parameters", {"names": ["max_particles"]})["descriptors"][0]
    check("param describe amcl max_particles: read only", d["read_only"], "type %d" % d["type"])
    t = call("/controller_server/get_parameter_types", {"names": ["max_vel_x", "use_sim_time", "no_such_parameter"]})["types"]
    t = [int(x) for x in t]
    check("param types", t == [PT["DOUBLE"], PT["BOOL"], PT["NOT_SET"]], "%s" % t)
    dv = lambda x: {"type": PT["DOUBLE"], "double_value": x}
    r = call("/controller_server/set_parameters_atomically", {"parameters": [{"name": "max_vel_x", "value": dv(0.2)},
                                                                              {"name": "no_such_parameter", "value": dv(1.0)}]})["result"]
    v = call("/controller_server/get_parameters", {"names": ["max_vel_x"]})["values"][0]["double_value"]
    check("set_parameters_atomically with one bad value: nothing set", not r["successful"] and abs(v - 0.3) < 1e-9, "%s, max_vel_x %s" % (r["reason"], v))
    r = call("/controller_server/set_parameters_atomically", {"parameters": [{"name": "max_vel_x", "value": dv(0.3)}]})["result"]
    check("set_parameters_atomically", r["successful"])
    nn = call("/lifecycle_manager_navigation/get_parameters", {"names": ["node_names"]})["values"][0]
    check("param get lifecycle manager node_names", nn["type"] == PT["STRING_ARRAY"] and "bt_navigator" in list(nn["string_array_value"]),
          ", ".join(nn["string_array_value"]))

    # lifecycle
    def state(node):
        r = call("/%s/get_state" % node, {})["current_state"]
        return r["id"], r["label"]
    st = [state(n) for n in ("controller_server", "bt_navigator")]
    check("lifecycle get: controller_server, bt_navigator active", all(x[0] == 3 for x in st), ", ".join(x[1] for x in st))
    av = call("/controller_server/get_available_transitions", {})["available_transitions"]
    check("lifecycle list: active -> deactivate, shutdown", sorted(a["transition"]["id"] for a in av) == [4, 7],
          ", ".join("%s -> %s" % (a["transition"]["label"], a["goal_state"]["label"]) for a in av))
    act = call("/lifecycle_manager_navigation/is_active", {})
    check("lifecycle manager is_active", act["success"], act["message"])
    r = call("/lifecycle_manager_navigation/manage_nodes", {"command": 1})        # PAUSE
    st = state("controller_server"); act = call("/lifecycle_manager_navigation/is_active", {})
    check("manage_nodes PAUSE: nodes inactive", r["success"] and st[0] == 2 and not act["success"], "controller_server %s" % st[1])
    pump(1.0)                                                                       # the robot stands still meanwhile
    r = call("/lifecycle_manager_navigation/manage_nodes", {"command": 2})        # RESUME
    st = state("controller_server")
    check("manage_nodes RESUME: nodes active", r["success"] and st[0] == 3, "controller_server %s" % st[1])
    pump(0.3)
    te = [(e["transition"]["label"], e["start_state"]["label"], e["goal_state"]["label"]) for e in got.get("/controller_server/transition_event", [])]
    want = [("deactivate", "active", "deactivating"), ("transition_success", "deactivating", "inactive"),
            ("activate", "inactive", "activating"), ("transition_success", "activating", "active")]
    check("transition events (pause, resume)", te[-4:] == want, "; ".join("%s %s->%s" % x for x in te[-4:]))
    r = call("/controller_server/change_state", {"transition": {"id": 1}})        # CONFIGURE while active: not allowed
    check("change_state configure while active refused", not r["success"])

    # actions: wait for the first leg (after the mapping tour), then cancel it
    print("[ros] waiting for the first navigate_to_pose goal (after the mapping tour) ...", flush=True)
    t0 = time.time()
    while time.time() - t0 < 400 and not any(m["status_list"] and m["status_list"][0]["status"] == 2 for m in status_log):
        pump(0.2)
        if sess.done:
            break
    running = [m for m in status_log if m["status_list"] and m["status_list"][0]["status"] == 2]
    check("action status: a goal executing", bool(running))
    if running:
        gid = list(running[-1]["status_list"][0]["goal_info"]["goal_id"]["uuid"])
        r = call("/navigate_to_pose/_action/cancel_goal", {"goal_info": {"goal_id": {"uuid": gid}}})
        check("action cancel_goal accepted", r["return_code"] == 0 and len(r["goals_canceling"]) == 1, "return_code %d" % r["return_code"])
        g = call("/navigate_to_pose/_action/get_result", {"goal_id": {"uuid": gid}}, timeout=10)
        check("action get_result: CANCELED", g["status"] == 5, "status %d, error_msg '%s'" % (g["status"], g["result"]["error_msg"]))
    print("[ros] letting the mission end ...", flush=True)
    t0 = time.time()
    while not sess.done and time.time() - t0 < 900:
        pump(0.5)
    fbt = got.get("/navigate_through_poses/_action/feedback", [])
    check("action feedback: mapping tour", len(fbt) > 5 and fbt[0]["feedback"]["number_of_poses_remaining"] > fbt[-1]["feedback"]["number_of_poses_remaining"],
          "%d messages, poses left %d -> %d, distance left %.1f -> %.1f m" % (len(fbt), fbt[0]["feedback"]["number_of_poses_remaining"] if fbt else -1,
          fbt[-1]["feedback"]["number_of_poses_remaining"] if fbt else -1, fbt[0]["feedback"]["distance_remaining"] if fbt else -1,
          fbt[-1]["feedback"]["distance_remaining"] if fbt else -1))
    fb = got.get("/navigate_to_pose/_action/feedback", [])
    dl = [m["feedback"]["distance_remaining"] for m in fb]
    nt = [m["feedback"]["navigation_time"]["sec"] for m in fb]
    check("action feedback: navigate_to_pose", len(fb) > 5 and max(dl) > 1.0 and min(dl) < 0.5 and max(nt) > 3,
          "%d messages, distance left %.2f..%.2f m, navigation time up to %d s" % (len(fb), min(dl or [0]), max(dl or [0]), max(nt or [0])))
    ck = [m["clock"]["sec"] + m["clock"]["nanosec"] * 1e-9 for m in got.get("/clock", [])]
    check("/clock: simulated time goes on", len(ck) > 10 and all(b > a for a, b in zip(ck, ck[1:])), "%d messages, %.1f .. %.1f s" % (len(ck), ck[0] if ck else -1, ck[-1] if ck else -1))
    lg = got.get("/rosout", [])
    names = sorted(set(m["name"] for m in lg))
    check("/rosout: log lines with node names", any("map done" in m["msg"] for m in lg) and any("leg 3" in m["msg"] for m in lg) and len(names) >= 2,
          "%d lines from %s" % (len(lg), ", ".join(names)))
    ok = all(results)
    print("RESULT ros: %s (%d of %d checks)" % ("PASS" if ok else "FAIL", sum(results), len(results)), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
