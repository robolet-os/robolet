#!/usr/bin/env python3
"""Check that the chip's CDR bytes are ROS 2 CDR bytes.

    pip install rosbags numpy
    (cd test_host && g++ -O2 -std=c++17 -I../src test_cdr.cpp -o tcdr)
    python3 tools/check_cdr.py

For every message type in msg/: random messages (empty, short and full-capacity sequences) are serialized by rosbags,
an independent ROS 2 implementation. Then
  1. tools/ros2wire.py decodes them to the same values and encodes them back to the same bytes, and
  2. the firmware code (cdr_read + cdr_write in src/ros2_msgs.h, run by test_host/tcdr) gives back the same bytes.
"""
import os
import random
import struct
import subprocess
import sys

import numpy as np
from rosbags.typesys import Stores, get_typestore

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ros2wire as rw  # noqa: E402

ts = get_typestore(Stores.ROS2_HUMBLE)
caps, sdef = {}, 24
for line in open(os.path.join(rw.MSG_DIR, "capacities.txt")):
    line = line.split("#")[0].strip()
    if line:
        k, v = line.split()
        if k == "string":
            sdef = int(v)
        else:
            caps[k] = int(v)
rng = random.Random(1)


def rb(tname):
    """rosbags names only <pkg>/msg/<Type>: services and actions go in as <pkg>/msg/<Kind><Type> (same bytes)"""
    pkg, kind, name = tname.split("/")
    return tname if kind == "msg" else "%s/msg/%s%s" % (pkg, kind.capitalize(), name)


def register_missing():
    """types rosbags does not ship (services, actions, nav2_msgs): give it their definitions from msg/, so it still
    serializes them with its own CDR code (an independent check of alignment, sequences and strings)"""
    from rosbags.typesys import get_types_from_msg
    add = {}
    for tname, fields in rw.TYPES.items():
        if rb(tname) in ts.types:
            continue
        lines = []
        for fname, ftype, arr in fields:
            t = (rb(ftype) if "/" in ftype else ftype) + ("" if arr is None else ("[%d]" % arr[1] if arr[0] == "fixed" else "[]"))
            lines.append("%s %s" % (t, fname))
        add.update(get_types_from_msg("\n".join(lines), rb(tname)))
    ts.register(add)
    return len(add)


def rand_prim(t):
    if t == "bool":
        return rng.random() < 0.5
    if t.startswith("float"):
        return rng.choice([0.0, -0.0, 1.5, rng.uniform(-1e6, 1e6), 3.14159265])
    code, size, dt = rw.PRIM[t]
    info = np.iinfo(dt)
    return int(rng.choice([info.min, info.max, 0, rng.randint(int(info.min), int(info.max))]))


def make(tname, mode):
    """rosbags message object; mode 0 = empty sequences, 1 = short, 2 = full capacity"""
    pkg, kind, name = tname.split("/")
    capkey = "%s/%s" % (pkg, name) if kind == "msg" else tname
    vals = {}
    for fname, ftype, arr in rw.TYPES[tname]:
        if arr is None:
            if ftype in rw.PRIM:
                vals[fname] = rand_prim(ftype)
            elif ftype == "string":
                vals[fname] = "".join(rng.choice("abc_/xyz019") for _ in range(rng.randint(0, sdef - 1)))
            else:
                vals[fname] = make(ftype, mode)
            continue
        if arr[0] == "fixed":
            n = arr[1]
        else:
            cap = arr[1] or caps["%s.%s" % (capkey, fname)]       # T[<=N]: N
            n = [0, rng.randint(1, min(cap, 5)), cap][mode]
        if ftype in rw.PRIM:
            vals[fname] = np.array([rand_prim(ftype) for _ in range(n)], dtype=rw.PRIM[ftype][2])
        elif ftype == "string":
            vals[fname] = ["".join(rng.choice("abc_/xyz019") for _ in range(rng.randint(0, sdef - 1))) for _ in range(n)]
        else:
            vals[fname] = [make(ftype, mode) for _ in range(n)]
    return ts.types[rb(tname)](**vals)


def same(a, b):
    """rosbags object a == ros2wire dict b"""
    if isinstance(b, dict):
        return all(same(getattr(a, k), v) for k, v in b.items())
    if isinstance(b, list):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    if isinstance(b, np.ndarray):
        return np.array_equal(np.asarray(a), b, equal_nan=True)
    if isinstance(b, float):
        return struct.pack("<d", a) == struct.pack("<d", b) or (np.float32(a) == np.float32(b))
    return a == b


def main():
    print("types given to rosbags from msg/ (it does not ship them): %d" % register_missing())
    tcdr = os.environ.get("TCDR") or os.path.join(os.path.dirname(HERE), "test_host", "tcdr")   # TCDR: another build
    records, fails = [], 0
    for tname in sorted(rw.TYPES):
        for mode in (0, 1, 2):
            for _ in range(3 if mode == 1 else 1):
                msg = make(tname, mode)
                data = bytes(ts.serialize_cdr(msg, rb(tname)))
                d = rw.decode(tname, data)
                if not same(msg, d):
                    print("FAIL decode", tname, mode); fails += 1
                if rw.encode(tname, d) != data:
                    print("FAIL encode", tname, mode); fails += 1
                records.append((tname, data))
    blob = b"".join(struct.pack("<H", len(t)) + t.encode() + struct.pack("<I", len(d)) + d for t, d in records)
    out = subprocess.run([tcdr], input=blob, capture_output=True, check=True).stdout
    p = 0
    for tname, data in records:
        tl = struct.unpack_from("<H", out, p)[0]; p += 2 + tl
        n = struct.unpack_from("<I", out, p)[0]; p += 4
        got = out[p:p + n]; p += n
        if got != data:
            print("FAIL firmware round trip %s (%d bytes from rosbags, %d back)" % (tname, len(data), len(got))); fails += 1
    print("%d messages of %d types: %s" % (len(records), len(rw.TYPES), "ALL SAME BYTES AS ROS 2 (rosbags)" if not fails else "%d FAILED" % fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
