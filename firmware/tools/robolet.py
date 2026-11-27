#!/usr/bin/env python3
"""PC tool for the chip's topics (the bridge node, src/bridge.h). Needs pyserial; numpy is optional.

  python3 tools/robolet.py run "k 0 1 0 3"            run a mission on the chip with the bridge, show the topics
  python3 tools/robolet.py run "k 0 1 0 3 rt"         the same in real time (one cycle per 1/7 s)
       --sub /scan --sub /map:1                        topics to forward (name[:max Hz]); default: the viewer set
       --echo /amcl/robot_pose                         print every message of that topic
       --record run.bin                                save all received bytes (play them back later)
       --port /dev/ttyACM0                             default: the first ESP32 port found
  python3 tools/robolet.py play run.bin [--echo T]    the same from a recording, without the chip

Every 2 s it prints one line per topic: messages, rate (Hz), bandwidth -- what `ros2 topic hz` / `bw` show.
"""
import argparse
import collections
import glob
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ros2wire as rw  # noqa: E402

DEFAULT_SUBS = ["/map:1", "/scan", "/amcl/robot_pose", "/pose", "/plan", "/ground_truth", "/localization/status",
                "/localization/candidates", "/cmd_vel"]


def find_port():
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usbmodem*", "/dev/cu.usbserial*"):
        ports = sorted(glob.glob(pat))
        if ports:
            return ports[0]
    sys.exit("no serial port found: use --port")


def short(v, depth=0):
    """one-line view of a decoded message"""
    if isinstance(v, dict):
        return "{" + ", ".join("%s: %s" % (k, short(x, depth + 1)) for k, x in v.items()) + "}"
    if isinstance(v, (list, tuple)) or hasattr(v, "dtype"):
        n = len(v)
        if n > 6:
            return "[%d items]" % n
        return "[" + ", ".join(short(x, depth + 1) for x in v) + "]"
    if isinstance(v, float):
        return "%.4g" % v
    try:
        f = float(v)
        return "%.4g" % f if f != int(f) else str(int(f))
    except (TypeError, ValueError):
        return repr(v)


class Session:
    """keeps the topic table and statistics; handle() takes the parser output"""
    def __init__(self, echo=(), on_message=None, quiet=False):
        self.topics, self.echo, self.on_message, self.quiet = {}, set(echo), on_message, quiet
        self.count, self.bytes, self.window = collections.Counter(), collections.Counter(), collections.defaultdict(collections.deque)
        self.hello, self.stats, self.done, self.t_last = None, None, False, time.time()
        self.services, self.replies, self.next_call = {}, {}, 1     # name -> srv type; call id -> (code, CDR)
        self.nodes = {}                                               # node name -> (published topics, subscribed topics)
        self.on_raw = None                                            # (name, type, CDR bytes) for every DATA frame
        self.decoded = None                                           # name -> bool: decode for on_message (None: all)

    def handle(self, items, now=None):
        now = time.time() if now is None else now
        for it in items:
            if it[0] == "text":
                if not self.quiet:
                    print(it[1])
                if it[1].startswith("RESULT"):
                    self.done = True
                continue
            _, kind, flags, fid, payload = it
            k = rw.KIND_NAME.get(kind)
            if k == "HELLO":
                self.hello = bytes(payload).decode(errors="replace")
                print("[bridge] HELLO", self.hello)
            elif k == "ADVERTISE":
                (name, typ), _ = rw.cstrings(payload, 2)
                self.topics[fid] = (name, typ)
            elif k == "SERVICE":
                (name, typ), _ = rw.cstrings(payload, 2)
                self.services[name] = typ
            elif k == "NODE":
                (name, pubs, subs), _ = rw.cstrings(payload, 3)
                self.nodes[name] = (pubs.split(), subs.split())
            elif k == "REPLY":
                cid, code = struct.unpack_from("<Ii", payload)
                self.replies[cid] = (code, bytes(payload[8:]))
            elif k == "STATS":
                self.stats = struct.unpack("<6I", payload)
            elif k == "DATA" and fid in self.topics:
                name, typ = self.topics[fid]
                self.count[name] += 1; self.bytes[name] += len(payload)
                w = self.window[name]; w.append((now, len(payload)))
                while w and now - w[0][0] > 5:
                    w.popleft()
                if self.on_raw:
                    self.on_raw(name, typ, bytes(payload))
                if name in self.echo or (self.on_message and (self.decoded is None or self.decoded(name))):
                    msg = rw.decode(typ, payload)
                    if name in self.echo:
                        print("[%s] %s" % (name, short(msg)))
                    if self.on_message:
                        self.on_message(name, typ, msg)

    def call_frame(self, service, request, srv_type=None):
        """(call id, frame bytes) for a service call; request is a dict (missing fields = 0 / empty)"""
        typ = srv_type or self.services[service]
        cid = self.next_call; self.next_call += 1
        payload = struct.pack("<I", cid) + service.encode() + b"\0" + rw.encode(typ + "_Request", request)
        return cid, rw.frame(rw.KIND["CALL"], 0, payload)

    def reply(self, cid, service, srv_type=None):
        """decoded response (dict), or None if not here yet; raises on an error code"""
        if cid not in self.replies:
            return None
        code, data = self.replies.pop(cid)
        if code != 0:
            raise RuntimeError("%s: %s" % (service, {-1: "no such service", -2: "bad request"}.get(code, code)))
        return rw.decode((srv_type or self.services[service]) + "_Response", data)

    def summary(self):
        lines = []
        for name in sorted(self.count):
            w = self.window[name]
            span = (w[-1][0] - w[0][0]) if len(w) > 1 else 0
            hz = (len(w) - 1) / span if span > 0 else 0
            bw = sum(b for _, b in w) / span / 1024 if span > 0 else 0
            lines.append("  %-28s %6d msgs %6.2f Hz %7.1f KB/s" % (name, self.count[name], hz, bw))
        if self.stats:
            s = self.stats
            lines.append("  link: %d frames sent, %d dropped (link full), %d bad frames received, ring %d bytes" % (s[0], s[1], s[2], s[5]))
        return "\n".join(lines)


def subscribe_frames(subs):
    out = b""
    for s in subs:
        name, _, hz = s.partition(":")
        rate10 = int(round(float(hz) * 10)) if hz else 0
        out += rw.frame(rw.KIND["SUBSCRIBE"], 0, name.encode() + b"\x00" + struct.pack("<H", rate10))
    return out


def run(args):
    import serial  # pyserial
    port = args.port or find_port()
    s = serial.Serial(port, 115200, timeout=0.05)
    rec = open(args.record, "wb") if args.record else None
    cmd = args.command.strip()
    if "bridge" not in cmd:
        cmd += " bridge"
    s.reset_input_buffer()
    s.write(b"\n" + cmd.encode() + b"\n")
    print("[robolet] %s: sent '%s'" % (port, cmd))
    parser, sess = rw.FrameParser(), Session(args.echo, quiet=args.quiet)
    subscribed, t_sum, t_end = False, time.time(), None
    try:
        while True:
            data = s.read(65536)
            if data:
                if rec:
                    rec.write(data)
                items = parser.feed(data)
                sess.handle(items)
                if sess.hello and not subscribed:
                    s.write(subscribe_frames(args.sub or DEFAULT_SUBS)); subscribed = True
            if time.time() - t_sum > 2 and sess.count:
                print(sess.summary()); t_sum = time.time()
            if sess.done and t_end is None:
                t_end = time.time() + 1.0
            if t_end and time.time() > t_end:
                break
    except KeyboardInterrupt:
        print("\n[robolet] stopped (the chip goes on until the run ends; press RST to stop it)")
    print(sess.summary())
    print("[robolet] %d frames, %d damaged frames skipped, %d bytes of noise" % (parser.frames, parser.bad_crc, parser.noise_bytes))
    if rec:
        rec.close(); print("[robolet] recording:", args.record)


def play(args):
    data = open(args.file, "rb").read()
    parser, sess = rw.FrameParser(), Session(args.echo, quiet=args.quiet)
    t = [0.0]

    def stamp(name, typ, msg):                   # replay time = the newest message stamp (simulated time)
        h = msg.get("header") if isinstance(msg, dict) else None
        if h:
            t[0] = max(t[0], h["stamp"]["sec"] + h["stamp"]["nanosec"] * 1e-9)
    sess.on_message = stamp
    for i in range(0, len(data), 4096):
        sess.handle(parser.feed(data[i:i + 4096]), now=t[0])
    print(sess.summary())
    print("[robolet] %d frames, %d damaged frames skipped, %d bytes of noise" % (parser.frames, parser.bad_crc, parser.noise_bytes))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="what", required=True)
    r = sp.add_parser("run"); r.add_argument("command"); r.add_argument("--port"); r.add_argument("--sub", action="append")
    r.add_argument("--echo", action="append", default=[]); r.add_argument("--record"); r.add_argument("--quiet", action="store_true")
    p = sp.add_parser("play"); p.add_argument("file"); p.add_argument("--echo", action="append", default=[]); p.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    run(args) if args.what == "run" else play(args)


if __name__ == "__main__":
    main()
