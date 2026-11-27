#!/usr/bin/env python3
"""ros2-style command line for the chip: nodes, topics, services, parameters, lifecycle states and actions of a run on
the chip (src/ros.h, src/bridge.h), with the names and output of the ROS 2 `ros2` commands. Needs pyserial.

One process keeps the serial port (the daemon, as `ros2 daemon`); the commands talk to it on 127.0.0.1:11811
(ROBOLET_DAEMON_PORT changes the port), so several terminals can use the chip at once:

  python3 tools/ros2.py daemon [--chip "k 0 1 0 3 rt hold"] [--port /dev/ttyACM0] [--record run.bin]
        starts the run on the chip (default: room mission in real time; "hold": after its legs the robot waits for
        goals until `mission stop`) and serves the commands; --chip "" attaches to a run that is already going
  python3 tools/ros2.py shell [same options]
        the same daemon (if none is running) and a prompt for the commands below ("ros2" in front is optional)

  node list | node info /bt_navigator
  topic list [-t] | topic info /scan | topic echo /amcl_pose [--once] [--rate 2] [--full] | topic hz /scan
  topic pub /goal_pose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 2.0}}}" [--times 1]
  service list [-t] | service type /amcl/get_parameters | service call /mission/stop [type] ["{...}"]
  param list [/node] | param get /controller_server max_vel_x | param set /controller_server max_vel_x 0.25
  param describe /controller_server max_vel_x | param dump /controller_server
  lifecycle nodes | lifecycle get [/node] | lifecycle set /controller_server deactivate | lifecycle list /node
  action list [-t] | action info /navigate_to_pose
  action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 6.5, y: 1.5}}}}" [--feedback]
  interface list | interface show nav2_msgs/action/NavigateToPose
  mission stop                                   ends a "hold" run (/mission/stop)

Values are YAML as in ros2 ({a: 1, b: {c: x}}; fields left out are 0 / empty). Ctrl+C ends echo, hz and send_goal.
"""
import argparse
import base64
import collections
import json
import math
import os
import queue
import re
import shlex
import socket
import socketserver
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ros2wire as rw  # noqa: E402
import robolet  # noqa: E402

ADDR = ("127.0.0.1", int(os.environ.get("ROBOLET_DAEMON_PORT", "11811")))
PT_NAME = {0: "not set", 1: "boolean", 2: "integer", 3: "double", 4: "string", 5: "byte array", 6: "boolean array",
           7: "integer array", 8: "double array", 9: "string array"}
TRANSITIONS = {"configure": 1, "cleanup": 2, "activate": 3, "deactivate": 4, "shutdown": None}
STATUS = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


def jsonable(v):
    if isinstance(v, dict):
        return {k: jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [jsonable(x) for x in v]
    if hasattr(v, "tolist"):
        return v.tolist()
    if isinstance(v, (bytes, bytearray, memoryview)):
        return list(bytes(v))
    return v


# =========================================================================================================== daemon
class Daemon:
    """owns the serial port: one reader thread, the requests of the clients (call, subscribe, publish, graph)"""

    def __init__(self, port, chip, record=None, quiet=False):
        import serial
        self.quiet = quiet                                        # shell: only the mission's lines from the chip
        self.port = port or robolet.find_port()
        self.s = serial.Serial(self.port, 115200, timeout=0.02)
        self.parser, self.sess = rw.FrameParser(), robolet.Session(quiet=True)
        self.sess.on_message, self.sess.on_raw = self.on_message, self.on_raw
        self.raw_qs = set()                                       # client queues that get CDR bytes (ros2_bridge.py)
        self.sess.decoded = lambda n: any(q not in self.raw_qs for q in self.subs.get(n, {}))
        self.cv, self.wlock = threading.Condition(), threading.Lock()
        self.subs = {}                                            # topic -> {client queue: rate in Hz (0 = all)}
        self.rec = open(record, "wb") if record else None
        self.done = False
        if chip:
            self.s.reset_input_buffer()
            self.write(b"\n" + (chip + " bridge").encode() + b"\n")
            print("[daemon] %s: sent '%s bridge'" % (self.port, chip), flush=True)
        threading.Thread(target=self.reader, daemon=True).start()
        if not chip:
            self.write(rw.frame(rw.KIND["LIST"], 0, b""))
        t0 = time.time()
        with self.cv:
            while not (self.sess.hello or self.sess.services) and time.time() - t0 < 20:
                self.cv.wait(0.2)
        if not (self.sess.hello or self.sess.services):
            sys.exit("[daemon] no answer from %s (a run with the bridge, firmware 0.13 or later? press RST)" % self.port)
        print("[daemon] chip: %s" % (self.sess.hello or "(attached)"), flush=True)

    def write(self, b):
        with self.wlock:
            self.s.write(b)

    def reader(self):
        while True:
            try:
                data = self.s.read(65536)
            except Exception as e:                                # port gone
                print("[daemon] port closed: %s" % e, flush=True)
                break
            if not data:
                continue
            if self.rec:
                self.rec.write(data)
            items = self.parser.feed(data)
            for it in items:
                if it[0] == "text" and (not self.quiet or it[1].startswith(("[k]", "MISSION", "RESULT"))):
                    print("   chip: " + it[1], flush=True)
            with self.cv:
                self.sess.handle(items)
                if self.sess.done:
                    self.done = True
                self.cv.notify_all()
            if self.done:
                break
        with self.cv:
            self.done = True
            self.cv.notify_all()

    def on_message(self, name, typ, msg):
        for q in list(self.subs.get(name, {})):
            if q not in self.raw_qs:
                q.put((name, typ, msg))

    def on_raw(self, name, typ, cdr):
        for q in list(self.subs.get(name, {})):
            if q in self.raw_qs:
                q.put((name, typ, cdr))

    def _subscribe_frame(self, topic):
        rates = list(self.subs.get(topic, {}).values())
        if not rates:
            return rw.frame(rw.KIND["UNSUBSCRIBE"], 0, topic.encode() + b"\0")
        rate10 = 0 if 0 in rates else int(round(max(rates) * 10))
        return rw.frame(rw.KIND["SUBSCRIBE"], 0, topic.encode() + b"\0" + struct.pack("<H", rate10))

    def subscribe(self, topic, rate, q):
        self.subs.setdefault(topic, {})[q] = rate
        self.write(self._subscribe_frame(topic))

    def unsubscribe(self, topic, q):
        self.subs.get(topic, {}).pop(q, None)
        self.write(self._subscribe_frame(topic))

    def call(self, service, request, typ=None, timeout=5.0):
        with self.cv:
            typ = typ or self.sess.services.get(service)
            if not typ:
                raise RuntimeError("no service %s" % service)
            cid, fr = self.sess.call_frame(service, request, typ)
        self.write(fr)
        t0 = time.time()
        with self.cv:
            while cid not in self.sess.replies:
                if self.done:
                    raise RuntimeError("the run is over")
                if time.time() - t0 > timeout:
                    raise RuntimeError("%s: no answer in %.0f s" % (service, timeout))
                self.cv.wait(0.1)
            return self.sess.reply(cid, service, typ)

    def publish(self, topic, typ, msg=None, cdr=None):
        cdr = rw.encode(typ, msg) if cdr is None else cdr
        self.write(rw.frame(rw.KIND["PUBLISH"], 0, topic.encode() + b"\0" + typ.encode() + b"\0" + cdr))

    def graph(self, refresh):
        if refresh:
            self.write(rw.frame(rw.KIND["LIST"], 0, b""))
            time.sleep(0.4)
        with self.cv:
            return {"hello": self.sess.hello, "nodes": self.sess.nodes, "topics": dict(self.sess.topics.values()),
                    "services": dict(self.sess.services), "done": self.done}

    # ---- clients: one JSON line in, JSON lines out
    def handle(self, rf, wf):
        req = json.loads(rf.readline())
        op = req["op"]
        send = lambda o: (wf.write((json.dumps(jsonable(o)) + "\n").encode()), wf.flush())
        try:
            if op == "graph":
                send(self.graph(req.get("refresh", True)))
            elif op == "call":
                send({"ok": self.call(req["service"], req.get("request", {}), req.get("type"), req.get("timeout", 5.0))})
            elif op == "pub":
                self.publish(req["topic"], req["type"], req.get("msg", {}))
                send({"ok": True})
            elif op == "pubraw":                                 # CDR bytes (base64) as they are
                self.publish(req["topic"], req["type"], cdr=base64.b64decode(req["cdr"]))
                send({"ok": True})
            elif op == "pubstream":                              # many pubraw lines on one connection (tools/relay.py)
                send({"ok": True})
                for line in rf:
                    m = json.loads(line)
                    self.publish(m["topic"], m["type"], cdr=base64.b64decode(m["cdr"]))
            elif op == "sub":
                q, raw = queue.Queue(), bool(req.get("raw"))
                if raw:
                    self.raw_qs.add(q)
                for t in req["topics"]:
                    self.subscribe(t, req.get("rate", 0), q)
                try:
                    send({"ok": True})
                    while not self.done:
                        try:
                            name, typ, msg = q.get(timeout=0.5)
                        except queue.Empty:
                            send({"tick": True})              # (finds a client that went away)
                            continue
                        if raw:
                            send({"topic": name, "type": typ, "cdr": base64.b64encode(msg).decode(), "t": time.time()})
                        else:
                            send({"topic": name, "type": typ, "msg": msg, "t": time.time()})
                    send({"end": "the run is over"})
                finally:
                    for t in req["topics"]:
                        self.unsubscribe(t, q)
                    self.raw_qs.discard(q)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:                                    # noqa: BLE001  (the client shows it)
            try:
                send({"error": str(e)})
            except OSError:
                pass

    def serve(self):
        d = self

        class H(socketserver.StreamRequestHandler):
            def handle(self):
                d.handle(self.rfile, self.wfile)

        class S(socketserver.ThreadingTCPServer):
            daemon_threads = True
            allow_reuse_address = True
        srv = S(ADDR, H)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print("[daemon] serving the ros2 commands on %s:%d" % ADDR, flush=True)
        return srv


def daemon_running():
    try:
        socket.create_connection(ADDR, timeout=0.3).close()
        return True
    except OSError:
        return False


# =========================================================================================================== client
class Client:
    def _open(self, req):
        try:
            c = socket.create_connection(ADDR, timeout=None)
        except OSError:
            raise SystemExit("no daemon on %s:%d: start one with  python3 tools/ros2.py daemon  (or use: ros2.py shell)" % ADDR)
        f = c.makefile("rw")
        f.write(json.dumps(req) + "\n"); f.flush()
        return c, f

    def request(self, **req):
        c, f = self._open(req)
        try:
            r = json.loads(f.readline())
        finally:
            c.close()
        if "error" in r:
            raise RuntimeError(r["error"])
        return r

    def graph(self, refresh=True):
        return self.request(op="graph", refresh=refresh)

    def call(self, service, request=None, typ=None, timeout=5.0):
        return self.request(op="call", service=service, request=request or {}, type=typ, timeout=timeout)["ok"]

    def stream(self, topics, rate=0, raw=False):
        c, f = self._open({"op": "sub", "topics": topics, "rate": rate, "raw": raw})
        try:
            for line in f:
                r = json.loads(line)
                if "error" in r:
                    raise RuntimeError(r["error"])
                if "end" in r:
                    print("(%s)" % r["end"])
                    return
                if "topic" in r:
                    yield r
        finally:
            c.close()


# ---------------------------------------------------------------------------------------------------- formatting
def yaml_lines(v, ind=0, full=False):
    pad = "  " * ind
    out = []
    if isinstance(v, dict):
        for k, x in v.items():
            if isinstance(x, dict):
                out.append("%s%s:" % (pad, k)); out += yaml_lines(x, ind + 1, full)
            elif isinstance(x, list):
                if not x:
                    out.append("%s%s: []" % (pad, k))
                elif not full and len(x) > 16:
                    out.append("%s%s: '<sequence, length: %d>'" % (pad, k, len(x)))
                elif isinstance(x[0], dict):
                    out.append("%s%s:" % (pad, k))
                    for e in x:
                        sub = yaml_lines(e, ind + 1, full)
                        out.append(pad + "- " + sub[0].lstrip()); out += sub[1:]
                else:
                    out.append("%s%s:" % (pad, k)); out += ["%s- %s" % (pad, scalar(e)) for e in x]
            else:
                out.append("%s%s: %s" % (pad, k, scalar(x)))
    else:
        out.append(pad + scalar(v))
    return out


def scalar(x):
    if isinstance(x, bool):
        return "true" if x else "false"
    if isinstance(x, float):
        return repr(x)
    if isinstance(x, str):
        return "'%s'" % x if (x == "" or re.search(r"[:{}\[\],#]|^\s|\s$", x)) else x
    return str(x)


def parse_yaml(s):
    """ros2-style values: JSON, YAML (if PyYAML is there) or YAML flow style like {a: 1, b: {c: x}}"""
    if s is None or s.strip() == "":
        return {}
    try:
        return json.loads(s)
    except ValueError:
        pass
    try:
        import yaml
        return yaml.safe_load(s)
    except ImportError:
        pass
    t = re.sub(r"([{,]\s*)([A-Za-z_]\w*)\s*:", r'\1"\2":', s.strip())
    t = re.sub(r'([\[,:]\s*)([A-Za-z_/][\w/.\-]*)(\s*)(?=[,\]}])', lambda m: m.group(1) + (
        m.group(2) if m.group(2) in ("true", "false", "null") else '"%s"' % m.group(2)) + m.group(3), t)
    return json.loads(t)


def param_value(s):
    """a value given on the command line -> rcl_interfaces/ParameterValue"""
    low = s.lower()
    if low in ("true", "false"):
        return {"type": 1, "bool_value": low == "true"}
    if re.fullmatch(r"[-+]?\d+", s):
        return {"type": 2, "integer_value": int(s)}
    try:
        return {"type": 3, "double_value": float(s)}
    except ValueError:
        return {"type": 4, "string_value": s}


def show_value(v):
    t = v["type"]
    val = {1: lambda: "True" if v["bool_value"] else "False", 2: lambda: v["integer_value"], 3: lambda: v["double_value"],
           4: lambda: v["string_value"], 9: lambda: list(v["string_array_value"])}.get(t, lambda: "")()
    return "%s value is: %s" % (PT_NAME.get(t, str(t)).capitalize(), val) if t else "Parameter not set."


def stamp_of(m):
    for path in (("header", "stamp"), ("stamp",), ("clock",)):
        x = m
        for p in path:
            x = x.get(p) if isinstance(x, dict) else None
        if isinstance(x, dict) and "sec" in x:
            return x["sec"] + x["nanosec"] * 1e-9
    return None


# ------------------------------------------------------------------------------------------------------ commands
HIDDEN = "/_action/"


def node_of_service(name):
    return "/" + name.split("/")[1]


def cmd_node(cl, a):
    g = cl.graph()
    if a.verb == "list":
        for n in sorted(g["nodes"]):
            print("/" + n)
        return
    n = a.name.lstrip("/")
    if n not in g["nodes"]:
        raise SystemExit("Unable to find node '/%s'" % n)
    pubs, subs = g["nodes"][n]
    T = g["topics"]
    print("/" + n)
    print("  Subscribers:"); [print("    %s: %s" % (t, T.get(t, "?"))) for t in subs if HIDDEN not in t]
    print("  Publishers:"); [print("    %s: %s" % (t, T.get(t, "?"))) for t in pubs if HIDDEN not in t]
    print("  Service Servers:")
    for s, t in sorted(g["services"].items()):
        if s.startswith("/%s/" % n) and HIDDEN not in s:
            print("    %s: %s" % (s, t))
    print("  Action Servers:")
    for t in pubs:
        if t.endswith("/_action/status"):
            act = t[:-len("/_action/status")]
            print("    %s: %s" % (act, action_type(g, act)))


def action_type(g, act):
    t = g["services"].get(act + "/_action/send_goal", "")
    return t[:-len("_SendGoal")] if t.endswith("_SendGoal") else t


def cmd_topic(cl, a):
    if a.verb == "list":
        g = cl.graph()
        for t, ty in sorted(g["topics"].items()):
            if HIDDEN not in t or a.include_hidden_topics:
                print("%s [%s]" % (t, ty) if a.show_types else t)
        return
    if a.verb == "info":
        g = cl.graph()
        if a.name not in g["topics"]:
            raise SystemExit("Unknown topic '%s'" % a.name)
        pubs = [n for n, (p, s) in g["nodes"].items() if a.name in p]
        subs = [n for n, (p, s) in g["nodes"].items() if a.name in s]
        print("Type: %s\nPublisher count: %d\nSubscription count: %d" % (g["topics"][a.name], len(pubs), len(subs)))
        if a.verbose:
            for n in pubs:
                print("  publisher: /%s" % n)
            for n in subs:
                print("  subscription: /%s" % n)
        return
    if a.verb == "echo":
        for r in cl.stream([a.name], a.rate):
            m = r["msg"]
            if a.field:
                for p in a.field.split("."):
                    m = m[p]
            print("\n".join(yaml_lines(m, 0, a.full) if isinstance(m, dict) else [scalar(m)]))
            print("---", flush=True)
            if a.once:
                return
        return
    if a.verb == "hz":
        win = collections.deque(maxlen=a.window)
        t_print = time.time() + 2
        for r in cl.stream([a.name], 0):
            st = stamp_of(r["msg"])
            win.append((r["t"], st))
            if time.time() >= t_print and len(win) > 2:
                t_print = time.time() + 2
                stamped = all(s is not None for _, s in win)
                use = [s for _, s in win] if stamped else [w for w, _ in win]
                d = [b - x for x, b in zip(use, use[1:]) if b > x]
                if d:
                    mean = sum(d) / len(d)
                    sd = math.sqrt(sum((x - mean) ** 2 for x in d) / len(d))
                    print("average rate: %.3f (%s)\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %d" %
                          (1 / mean, "message stamps" if stamped else "wall clock",
                           min(d), max(d), sd, len(d) + 1), flush=True)
        return
    if a.verb == "pub":
        g = cl.graph(refresh=False)
        typ = a.type or g["topics"].get(a.name)
        if not typ:
            raise SystemExit("give the message type: topic pub %s <type> <values>" % a.name)
        typ = full_type(typ, "msg")
        msg = parse_yaml(a.values)
        for i in range(a.times):
            cl.request(op="pub", topic=a.name, type=typ, msg=msg)
            print("publishing #%d: %s" % (i + 1, msg), flush=True)
            if i + 1 < a.times:
                time.sleep(1.0 / a.rate)


def full_type(t, kind):
    p = t.split("/")
    return "%s/%s/%s" % (p[0], kind, p[1]) if len(p) == 2 else t


def cmd_service(cl, a):
    g = cl.graph()
    S = g["services"]
    if a.verb == "list":
        for s in sorted(S):
            if HIDDEN not in s or a.include_hidden_services:
                print("%s [%s]" % (s, S[s]) if a.show_types else s)
    elif a.verb == "type":
        print(S.get(a.name) or "Unknown service '%s'" % a.name)
    elif a.verb == "call":
        typ, vals = a.type, a.values
        if typ and typ.lstrip().startswith("{"):
            typ, vals = None, typ
        typ = full_type(typ, "srv") if typ else S.get(a.name)
        if not typ:
            raise SystemExit("Unknown service '%s'" % a.name)
        req = parse_yaml(vals)
        print("requester: making request: %s_Request(%s)\n" % (typ, req))
        r = cl.call(a.name, req, typ, timeout=a.timeout)
        print("response:\n" + "\n".join(yaml_lines(r)))


def param_nodes(g, node=None):
    ns = sorted(s[:-len("/list_parameters")] for s in g["services"] if s.endswith("/list_parameters"))
    if node:
        node = "/" + node.lstrip("/")
        if node not in ns:
            raise SystemExit("Node not found: %s" % node)
        return [node]
    return ns


def cmd_param(cl, a):
    g = cl.graph(refresh=False)
    if a.verb == "list":
        for n in param_nodes(g, a.node):
            names = cl.call(n + "/list_parameters", {})["result"]["names"]
            print(n + ":")
            for x in sorted(names):
                print("  " + x)
        return
    n = param_nodes(g, a.node)[0]
    if a.verb == "get":
        print(show_value(cl.call(n + "/get_parameters", {"names": [a.name]})["values"][0]))
    elif a.verb == "set":
        r = cl.call(n + "/set_parameters", {"parameters": [{"name": a.name, "value": param_value(a.value)}]})["results"][0]
        print("Set parameter successful" if r["successful"] else "Setting parameter failed: %s" % r["reason"])
    elif a.verb == "describe":
        d = cl.call(n + "/describe_parameters", {"names": [a.name]})["descriptors"][0]
        print("Parameter name: %s\n  Type: %s" % (d["name"], PT_NAME.get(d["type"], d["type"])))
        if d["description"]:
            print("  Description: %s" % d["description"])
        print("  Constraints:")
        if d["read_only"]:
            print("    Read only: true")
        for r in d["floating_point_range"]:
            print("    Min value: %s\n    Max value: %s" % (r["from_value"], r["to_value"]))
        for r in d["integer_range"]:
            print("    Min value: %s\n    Max value: %s" % (r["from_value"], r["to_value"]))
    elif a.verb == "dump":
        names = sorted(cl.call(n + "/list_parameters", {})["result"]["names"])
        vals = cl.call(n + "/get_parameters", {"names": names})["values"]
        print("%s:\n  ros__parameters:" % n)
        for k, v in zip(names, vals):
            x = {1: v["bool_value"], 2: v["integer_value"], 3: v["double_value"], 4: v["string_value"],
                 9: list(v["string_array_value"])}.get(v["type"])
            print("    %s: %s" % (k, "[%s]" % ", ".join(x) if isinstance(x, list) else scalar(x)))


def lifecycle_nodes(g):
    return sorted(s[:-len("/get_state")] for s in g["services"] if s.endswith("/get_state"))


def cmd_lifecycle(cl, a):
    g = cl.graph(refresh=False)
    if a.verb == "nodes":
        print("\n".join(lifecycle_nodes(g)))
        return
    nodes = ["/" + a.node.lstrip("/")] if a.node else lifecycle_nodes(g)
    if a.verb == "get":
        for n in nodes:
            st = cl.call(n + "/get_state", {})["current_state"]
            print(("%s: " % n if not a.node else "") + "%s [%d]" % (st["label"], st["id"]))
    elif a.verb == "list":
        for t in cl.call(nodes[0] + "/get_available_transitions", {})["available_transitions"]:
            print("- %s [%d]\n\tStart: %s\n\tGoal: %s" % (t["transition"]["label"], t["transition"]["id"], t["start_state"]["label"],
                                                         t["goal_state"]["label"]))
    elif a.verb == "set":
        tid = TRANSITIONS.get(a.transition)
        if a.transition == "shutdown" or tid is None:
            av = cl.call(nodes[0] + "/get_available_transitions", {})["available_transitions"]
            ids = [t["transition"]["id"] for t in av if t["transition"]["label"] == a.transition]
            if not ids:
                raise SystemExit("Unknown transition requested, available ones are:\n" +
                                 "\n".join("- %s [%d]" % (t["transition"]["label"], t["transition"]["id"]) for t in av))
            tid = ids[0]
        ok = cl.call(nodes[0] + "/change_state", {"transition": {"id": tid}})["success"]
        print("Transitioning successful" if ok else "Transitioning failed")


def cmd_action(cl, a):
    g = cl.graph()
    acts = sorted(s[:-len("/_action/send_goal")] for s in g["services"] if s.endswith("/_action/send_goal"))
    if a.verb == "list":
        for x in acts:
            print("%s [%s]" % (x, action_type(g, x)) if a.show_types else x)
        return
    if a.name not in acts:
        raise SystemExit("Unknown action '%s'" % a.name)
    typ = action_type(g, a.name)
    if a.verb == "info":
        servers = [n for n, (p, s) in g["nodes"].items() if a.name + "/_action/status" in p]
        print("Action: %s\nAction clients: 0\nAction servers: %d" % (a.name, len(servers)))
        for n in servers:
            print("    /" + n)
        return
    # send_goal
    vals = a.values
    if a.type and a.type.lstrip().startswith("{"):
        vals = a.type
    elif a.type:
        typ = full_type(a.type, "action")
    goal = parse_yaml(vals)
    _fill_stamped(goal)
    uuid = list(os.urandom(16))
    print("Waiting for an action server to become available...\nSending goal:\n" + "\n".join(yaml_lines(goal, 2)))
    stop = threading.Event()
    if a.feedback:
        def fb():
            for r in cl.stream([a.name + "/_action/feedback"], 2):
                if stop.is_set():
                    return
                if list(r["msg"]["goal_id"]["uuid"]) == uuid:
                    print("Feedback:\n" + "\n".join(yaml_lines(r["msg"]["feedback"], 2)), flush=True)
        threading.Thread(target=fb, daemon=True).start()
        time.sleep(0.3)
    r = cl.call(a.name + "/_action/send_goal", {"goal_id": {"uuid": uuid}, "goal": goal}, typ + "_SendGoal")
    if not r["accepted"]:
        print("Goal was rejected.")
        return
    print("Goal accepted with ID: %s\n" % "".join("%02x" % b for b in uuid), flush=True)
    try:
        res = cl.call(a.name + "/_action/get_result", {"goal_id": {"uuid": uuid}}, typ + "_GetResult", timeout=3600)
    except KeyboardInterrupt:
        c = cl.call(a.name + "/_action/cancel_goal", {"goal_info": {"goal_id": {"uuid": uuid}}})
        print("\nCanceling goal... return code %d" % c["return_code"])
        res = cl.call(a.name + "/_action/get_result", {"goal_id": {"uuid": uuid}}, typ + "_GetResult", timeout=60)
    stop.set()
    print("Result:\n" + "\n".join(yaml_lines(res["result"], 2)) + "\n\nGoal finished with status: %s" % STATUS.get(res["status"], res["status"]))


def _fill_stamped(m):
    """PoseStamped without frame_id: "map" (as rviz2 sends goals); a quaternion left out: no rotation"""
    if isinstance(m, dict):
        if "pose" in m and isinstance(m["pose"], dict) and "header" not in m and "position" in m["pose"]:
            m["header"] = {"frame_id": "map"}
        if "position" in m and "orientation" not in m:
            m["orientation"] = {"w": 1.0}
        for v in m.values():
            _fill_stamped(v)
            if isinstance(v, list):
                for e in v:
                    _fill_stamped(e)


def cmd_interface(cl, a):
    if a.verb == "list":
        kinds = collections.defaultdict(set)
        for t in rw.TYPES:
            p, k, n = t.split("/")
            if k == "msg":
                kinds["Messages"].add(t)
            else:
                kinds["Services" if k == "srv" else "Actions"].add("%s/%s/%s" % (p, k, re.sub(r"_(Request|Response|Goal|Result|Feedback|"
                                                                                           r"SendGoal_Request|SendGoal_Response|GetResult_Request|"
                                                                                           r"GetResult_Response|FeedbackMessage)$", "", n)))
        for k in ("Messages", "Services", "Actions"):
            print(k + ":"); [print("    " + t) for t in sorted(kinds[k])]
        return
    p, k, n = full_type(a.type, "msg").split("/") if a.type.count("/") >= 1 else (None, None, None)
    ext = {"msg": ".msg", "srv": ".srv", "action": ".action"}.get(k, ".msg")
    path = os.path.join(rw.MSG_DIR, p or "", (n or "") + ext)
    if not os.path.exists(path):
        raise SystemExit("Unknown type '%s' (types: python3 tools/ros2.py interface list)" % a.type)
    print(open(path).read().rstrip())


def cmd_mission(cl, a):
    r = cl.call("/mission/stop", {})
    print(r["message"])


# ------------------------------------------------------------------------------------------------------- parser
def parser():
    ap = argparse.ArgumentParser(prog="ros2.py", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    for name in ("daemon", "shell"):
        p = sp.add_parser(name)
        p.add_argument("--chip", default="k 0 1 0 3 rt hold", help='run to start ("" = attach to a run that is going)')
        p.add_argument("--port")
        p.add_argument("--record", help="save all bytes from the chip (tools/viewer.py --play)")
    p = sp.add_parser("node"); s = p.add_subparsers(dest="verb", required=True)
    s.add_parser("list"); s.add_parser("info").add_argument("name")
    p = sp.add_parser("topic"); s = p.add_subparsers(dest="verb", required=True)
    q = s.add_parser("list"); q.add_argument("-t", "--show-types", action="store_true"); q.add_argument("--include-hidden-topics", action="store_true")
    q = s.add_parser("info"); q.add_argument("name"); q.add_argument("-v", "--verbose", action="store_true")
    q = s.add_parser("echo"); q.add_argument("name"); q.add_argument("type", nargs="?")
    q.add_argument("--once", action="store_true"); q.add_argument("--rate", type=float, default=0, help="at most this many Hz")
    q.add_argument("--full", action="store_true", help="print long arrays"); q.add_argument("--field")
    q = s.add_parser("hz"); q.add_argument("name"); q.add_argument("--window", type=int, default=100)
    q = s.add_parser("pub"); q.add_argument("name"); q.add_argument("type", nargs="?"); q.add_argument("values", nargs="?", default="{}")
    q.add_argument("--times", "-t", type=int, default=1); q.add_argument("--rate", "-r", type=float, default=1.0)
    p = sp.add_parser("service"); s = p.add_subparsers(dest="verb", required=True)
    q = s.add_parser("list"); q.add_argument("-t", "--show-types", action="store_true"); q.add_argument("--include-hidden-services", action="store_true")
    s.add_parser("type").add_argument("name")
    q = s.add_parser("call"); q.add_argument("name"); q.add_argument("type", nargs="?"); q.add_argument("values", nargs="?", default="{}")
    q.add_argument("--timeout", type=float, default=5.0)
    p = sp.add_parser("param"); s = p.add_subparsers(dest="verb", required=True)
    s.add_parser("list").add_argument("node", nargs="?")
    for v in ("get", "describe"):
        q = s.add_parser(v); q.add_argument("node"); q.add_argument("name")
    q = s.add_parser("set"); q.add_argument("node"); q.add_argument("name"); q.add_argument("value")
    s.add_parser("dump").add_argument("node")
    p = sp.add_parser("lifecycle"); s = p.add_subparsers(dest="verb", required=True)
    s.add_parser("nodes"); s.add_parser("get").add_argument("node", nargs="?"); s.add_parser("list").add_argument("node")
    q = s.add_parser("set"); q.add_argument("node"); q.add_argument("transition")
    p = sp.add_parser("action"); s = p.add_subparsers(dest="verb", required=True)
    s.add_parser("list").add_argument("-t", "--show-types", action="store_true")
    s.add_parser("info").add_argument("name")
    q = s.add_parser("send_goal"); q.add_argument("name"); q.add_argument("type", nargs="?"); q.add_argument("values", nargs="?", default="{}")
    q.add_argument("-f", "--feedback", action="store_true")
    p = sp.add_parser("interface"); s = p.add_subparsers(dest="verb", required=True)
    s.add_parser("list"); s.add_parser("show").add_argument("type")
    p = sp.add_parser("mission"); s = p.add_subparsers(dest="verb", required=True); s.add_parser("stop")
    return ap


COMMANDS = {"node": cmd_node, "topic": cmd_topic, "service": cmd_service, "param": cmd_param, "lifecycle": cmd_lifecycle,
            "action": cmd_action, "interface": cmd_interface, "mission": cmd_mission}


def run_line(argv, cl):
    if argv and argv[0] == "ros2":
        argv = argv[1:]
    if not argv:
        return
    try:
        a = parser().parse_args(argv)
    except SystemExit:
        return
    if a.cmd in ("daemon", "shell"):
        print("(already running)")
        return
    try:
        COMMANDS[a.cmd](cl, a)
    except KeyboardInterrupt:
        print()
    except (RuntimeError, SystemExit) as e:
        print(e)


def main():
    a = parser().parse_args()
    cl = Client()
    if a.cmd in ("daemon", "shell"):
        if a.cmd == "daemon" or not daemon_running():
            d = Daemon(a.port, a.chip, a.record, quiet=a.cmd == "shell")
            d.serve()
        else:
            d = None
            print("(using the daemon that is running on %s:%d)" % ADDR)
        if a.cmd == "daemon":
            try:
                while not d.done:
                    time.sleep(0.3)
                time.sleep(1.0)
                print("[daemon] the run is over", flush=True)
            except KeyboardInterrupt:
                pass
            return 0
        print("ros2-style commands for the chip; 'help' lists them, Ctrl+D ends", flush=True)
        while True:
            try:
                line = input("robolet> ")
            except (EOFError, KeyboardInterrupt):
                print()
                return 0
            if line.strip() in ("help", "?"):
                print(__doc__.split("\n\n", 2)[2].split("Values are")[0])
                continue
            if line.strip() in ("exit", "quit"):
                return 0
            run_line(shlex.split(line), cl)
            if d is not None and d.done:
                print("(the run on the chip is over)")
    COMMANDS[a.cmd](cl, a)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except RuntimeError as e:
        sys.exit(str(e))
