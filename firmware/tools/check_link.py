#!/usr/bin/env python3
"""Decode a recording of the chip -> PC link (test_host/tbr writes one) and check every frame.

    python3 tools/check_link.py bridge_out.bin [--fuzz] [--chip]   (--chip: a recording from the chip, see test.sh bridge)

Checks: every frame has a good CRC, every DATA frame decodes as its advertised ROS 2 type and uses all its bytes,
maps have width x height cells, scans have 720 ranges. --fuzz: drop / corrupt random bytes and check that the parser
never returns a frame that was not sent (it skips damaged ones and finds the next).
"""
import collections
import random
import struct
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ros2wire as rw  # noqa: E402


def parse(data, chunk=4096):
    p = rw.FrameParser()
    items = []
    for i in range(0, len(data), chunk):
        items += p.feed(data[i:i + chunk])
    return items, p


def main():
    path = sys.argv[1]
    data = open(path, "rb").read()
    items, p = parse(data)
    topics, counts, sizes, texts, bad = {}, collections.Counter(), collections.Counter(), [], 0
    hello, stats, last = None, None, {}
    for it in items:
        if it[0] == "text":
            texts.append(it[1]); continue
        _, kind, flags, fid, payload = it
        k = rw.KIND_NAME.get(kind)
        if k == "HELLO":
            hello = payload.decode()
        elif k == "ADVERTISE":
            (name, typ), _ = rw.cstrings(payload, 2); topics[fid] = (name, typ)
        elif k == "STATS":
            stats = struct.unpack("<6I", payload)
        elif k == "DATA":
            name, typ = topics[fid]
            try:
                msg = rw.decode(typ, payload)
                if rw.encode(typ, msg) != bytes(payload):
                    raise ValueError("does not encode back to the same bytes")
            except Exception as e:  # noqa: BLE001
                bad += 1; print("bad DATA on %s: %s" % (name, e)); continue
            counts[name] += 1; sizes[name] += len(payload); last[name] = msg
            if typ == "nav_msgs/msg/OccupancyGrid":
                assert len(msg["data"]) == msg["info"]["width"] * msg["info"]["height"]
            if typ == "sensor_msgs/msg/LaserScan":
                assert len(msg["ranges"]) == 720
    print("HELLO:", hello)
    print("%d frames, %d bad CRC, %d bytes of noise, %d text lines %s" % (p.frames, p.bad_crc, p.noise_bytes, len(texts), texts[:3]))
    print("advertised %d topics" % len(topics))
    for name in sorted(counts):
        print("  %-28s %6d messages  %8.1f KB" % (name, counts[name], sizes[name] / 1024))
    print("last STATS (sent, dropped, bad rx, rx frames, bad publish, ring used):", stats)
    if "/map" in last:
        m = last["/map"]; d = list(m["data"])
        print("last /map: %d x %d, %d occupied, %d free, %d unknown" % (m["info"]["width"], m["info"]["height"], d.count(100), d.count(0), d.count(-1)))
    if "/amcl/robot_pose" in last:
        print("last /amcl/robot_pose:", {k: round(float(v), 3) for k, v in last["/amcl/robot_pose"]["pose"]["position"].items()})
    ok = bad == 0 and (p.bad_crc == 0 or "--chip" in sys.argv) and bool(counts)   # on a real port a few frames may be damaged
    if "--fuzz" in sys.argv:
        rng = random.Random(3)
        good = set(bytes(it[4]) for it in items if it[0] == "frame")
        dam = bytearray(data)
        for _ in range(2000):                    # drop runs of bytes and flip bits
            i = rng.randrange(len(dam))
            if rng.random() < 0.5:
                del dam[i:i + rng.randint(1, 64)]
            else:
                dam[i] ^= 1 << rng.randrange(8)
        items2, p2 = parse(bytes(dam), chunk=777)
        frames2 = [bytes(it[4]) for it in items2 if it[0] == "frame"]
        false = sum(1 for f in frames2 if f not in good)
        junk = sum(1 for it in items2 if it[0] == "text") - len(texts)
        print("fuzz: 2000 damages -> %d frames kept of %d, %d bad CRC skipped, %d frames that were never sent, %d extra text lines"
              % (len(frames2), len(good), p2.bad_crc, false, junk))
        ok = ok and false == 0
    print("RESULT link:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
