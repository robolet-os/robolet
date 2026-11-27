#!/usr/bin/env python3
"""ROS 2 messages and the chip <-> PC link, on the PC side (no ROS 2 needed).

  - message types: read from ../msg/*.msg (the same files the firmware structs are generated from)
  - CDR: ROS 2 serialization (plain CDR, little endian), the same bytes as cdr_write / cdr_read on the chip
  - frames: the serial link format of src/link.h (see FRAME below)

Decoded messages are plain Python: dict per message, numpy array for number sequences / arrays (a list when numpy is
not installed), str for strings. Only the Python standard library is needed (numpy is optional).
"""
import os
import re
import struct
import sys

try:
    import numpy as np
except ImportError:  # the link and the decoder work without numpy (lists instead of arrays)
    np = None

HERE = os.path.dirname(os.path.abspath(__file__))
MSG_DIR = os.path.join(os.path.dirname(HERE), "msg")

_N = (lambda name: getattr(np, name)) if np is not None else (lambda name: name)
PRIM = {  # ROS type: (struct code, size, numpy dtype)
    "bool": ("?", 1, _N("bool_")), "byte": ("B", 1, _N("uint8")), "char": ("B", 1, _N("uint8")),
    "int8": ("b", 1, _N("int8")), "uint8": ("B", 1, _N("uint8")), "int16": ("h", 2, _N("int16")), "uint16": ("H", 2, _N("uint16")),
    "int32": ("i", 4, _N("int32")), "uint32": ("I", 4, _N("uint32")), "int64": ("q", 8, _N("int64")), "uint64": ("Q", 8, _N("uint64")),
    "float32": ("f", 4, _N("float32")), "float64": ("d", 8, _N("float64")),
}


# ------------------------------------------------------------------------------------------------ message types
def _parse_fields(lines, pkg):
    fields = []
    for raw in lines:
        line = raw.split("#")[0].strip()
        if not line or re.match(r"^\S+\s+[A-Z][A-Z0-9_]*\s*=", line):
            continue
        ftype, fname = line.split()[:2]
        arr = None
        m = re.match(r"^(.+)\[(<=)?(\d*)\]$", ftype)
        if m:
            ftype = m.group(1)
            arr = ("fixed", int(m.group(3))) if m.group(3) and not m.group(2) else ("seq", int(m.group(3)) if m.group(3) else None)
        if ftype not in PRIM and ftype != "string":
            if "/" not in ftype:
                ftype = "std_msgs/Header" if ftype == "Header" else pkg + "/" + ftype
            if ftype.count("/") == 1:
                p, t = ftype.split("/")
                ftype = "%s/msg/%s" % (p, t)
        fields.append((fname, ftype, arr))
    if not fields:                                       # rosidl: an empty structure still has one member
        fields.append(("structure_needs_at_least_one_member", "uint8", None))
    return fields


def _sections(path):
    secs, cur = [], []
    for raw in open(path):
        if raw.strip() == "---":
            secs.append(cur); cur = []
        else:
            cur.append(raw)
    return secs + [cur]


def load_types(msg_dir=MSG_DIR):
    """{'sensor_msgs/msg/LaserScan': [(name, type, arr), ...]}; type is a ROS primitive, 'string' or a full type name;
    arr is None, ('fixed', n) or ('seq', None). Services and actions give the types rosidl makes
    (pkg/srv/Name_Request, pkg/action/Name_SendGoal_Request, ...), as tools/gen_msgs.py does for the chip."""
    types = {}
    for pkg in sorted(os.listdir(msg_dir)):
        d = os.path.join(msg_dir, pkg)
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            path = os.path.join(d, f)
            base, ext = os.path.splitext(f)
            if ext == ".msg":
                types["%s/msg/%s" % (pkg, base)] = _parse_fields(open(path), pkg)
            elif ext == ".srv":
                rq, rs = _sections(path)
                types["%s/srv/%s_Request" % (pkg, base)] = _parse_fields(rq, pkg)
                types["%s/srv/%s_Response" % (pkg, base)] = _parse_fields(rs, pkg)
            elif ext == ".action":
                g, r, fb = _sections(path)
                A = "%s/action/%s" % (pkg, base)
                types[A + "_Goal"], types[A + "_Result"], types[A + "_Feedback"] = (_parse_fields(x, pkg) for x in (g, r, fb))
                uuid = "unique_identifier_msgs/msg/UUID"
                types[A + "_SendGoal_Request"] = [("goal_id", uuid, None), ("goal", A + "_Goal", None)]
                types[A + "_SendGoal_Response"] = [("accepted", "bool", None), ("stamp", "builtin_interfaces/msg/Time", None)]
                types[A + "_GetResult_Request"] = [("goal_id", uuid, None)]
                types[A + "_GetResult_Response"] = [("status", "int8", None), ("result", A + "_Result", None)]
                types[A + "_FeedbackMessage"] = [("goal_id", uuid, None), ("feedback", A + "_Feedback", None)]
    return types


TYPES = load_types()


# ------------------------------------------------------------------------------------------------ CDR
class _Reader:
    def __init__(self, b):
        if len(b) < 4 or b[0] != 0 or b[1] != 1:
            raise ValueError("not little-endian CDR")
        self.b, self.p = b, 4

    def align(self, a):
        o = (self.p - 4) % a
        if o:
            self.p += a - o

    def prim(self, t):
        code, size, _ = PRIM[t]
        self.align(size)
        v = struct.unpack_from("<" + code, self.b, self.p)[0]
        self.p += size
        return v

    def array(self, t, n):
        code, size, dt = PRIM[t]
        if n == 0:
            return np.zeros(0, dt) if np is not None else []
        self.align(size)
        if np is not None:
            a = np.frombuffer(self.b, dtype=np.dtype(dt).newbyteorder("<"), count=n, offset=self.p).copy()
        else:
            a = list(struct.unpack_from("<%d%s" % (n, code), self.b, self.p))
        self.p += n * size
        return a

    def string(self):
        n = self.prim("uint32")
        s = bytes(self.b[self.p:self.p + n - 1]).decode("utf-8", "replace")
        self.p += n
        return s


def _read(r, tname):
    out = {}
    for fname, ftype, arr in TYPES[tname]:
        if arr is None:
            out[fname] = r.prim(ftype) if ftype in PRIM else r.string() if ftype == "string" else _read(r, ftype)
        else:
            n = arr[1] if arr[0] == "fixed" else r.prim("uint32")
            if ftype in PRIM:
                out[fname] = r.array(ftype, n)
            elif ftype == "string":
                out[fname] = [r.string() for _ in range(n)]
            else:
                out[fname] = [_read(r, ftype) for _ in range(n)]
    return out


def decode(tname, data):
    """CDR bytes (with the 4-byte header) -> dict"""
    return _read(_Reader(memoryview(bytes(data))), tname)


class _Writer:
    def __init__(self):
        self.b = bytearray(b"\x00\x01\x00\x00")

    def align(self, a):
        o = (len(self.b) - 4) % a
        if o:
            self.b += bytes(a - o)

    def prim(self, t, v):
        code, size, _ = PRIM[t]
        self.align(size)
        self.b += struct.pack("<" + code, v)

    def array(self, t, a):
        code, size, dt = PRIM[t]
        if len(a):
            self.align(size)
            if np is not None:
                self.b += np.asarray(a, dtype=dt).astype(np.dtype(dt).newbyteorder("<")).tobytes()
            else:
                self.b += struct.pack("<%d%s" % (len(a), code), *a)

    def string(self, s):
        e = s.encode("utf-8")
        self.prim("uint32", len(e) + 1)
        self.b += e + b"\x00"


def _write(w, tname, msg):
    for fname, ftype, arr in TYPES[tname]:
        v = msg.get(fname) if isinstance(msg, dict) else getattr(msg, fname)
        if arr is None:
            if ftype in PRIM:
                w.prim(ftype, v if v is not None else 0)
            elif ftype == "string":
                w.string(v or "")
            else:
                _write(w, ftype, v if v is not None else default(ftype))
        else:
            if v is None:                                # missing: empty sequence, or a fixed array of zeros
                v = [0] * arr[1] if arr[0] == "fixed" and ftype in PRIM else ([default(ftype)] * arr[1] if arr[0] == "fixed" else [])
            if arr[0] == "seq":
                w.prim("uint32", len(v))
            if ftype in PRIM:
                w.array(ftype, v)
            elif ftype == "string":
                for s in v:
                    w.string(s)
            else:
                for e in v:
                    _write(w, ftype, e)


def encode(tname, msg):
    """dict (missing fields = 0 / empty) -> CDR bytes"""
    w = _Writer()
    _write(w, tname, msg)
    return bytes(w.b)


def default(tname):
    out = {}
    for fname, ftype, arr in TYPES[tname]:
        if arr is not None:
            fixed = arr[0] == "fixed" and ftype in PRIM
            out[fname] = (np.zeros(arr[1], PRIM[ftype][2]) if np is not None else [0] * arr[1]) if fixed else []
        elif ftype in PRIM:
            out[fname] = 0
        elif ftype == "string":
            out[fname] = ""
        else:
            out[fname] = default(ftype)
    if tname == "geometry_msgs/msg/Quaternion":
        out["w"] = 1.0
    return out


# ------------------------------------------------------------------------------------------------ frames
# FRAME (src/link.h): A5 5A | kind u8 | flags u8 | id u16 | len u16 | payload[len] | crc16 u16  (little endian)
#   crc16 = CRC-16/CCITT-FALSE over kind..payload. Text (log lines) can sit between frames: it is ASCII, never A5.
#   kinds: chip -> PC  HELLO (firmware text), ADVERTISE (id: topic id, payload "name\0type\0"), DATA (CDR payload),
#                      LOG (one log line), STATS (6 x u32: frames sent, dropped, bad received, received, bad publish, ring)
#          PC -> chip  SUBSCRIBE (payload "name\0" + u16 max rate in 1/10 Hz, 0 = all), UNSUBSCRIBE ("name\0"),
#                      PUBLISH (payload "name\0type\0" + CDR), LIST (ask for all ADVERTISE and SERVICE frames again)
#                      CALL (u32 call id + "service\0" + CDR request)
#          chip -> PC  SERVICE (payload "name\0type\0"), REPLY (u32 call id, i32 code: 0 answered, -1 no such service,
#                      -2 bad request; then the CDR response). An action's get_result is answered when the goal ends.
#                      NODE (id: node index, payload "name\0publishers\0subscriptions\0", topic names separated by spaces)
#   flags bit 0: payload is run-length coded (RLE below), used for maps.
SYNC = b"\xA5\x5A"
KIND = {"HELLO": 1, "ADVERTISE": 2, "DATA": 3, "SUBSCRIBE": 4, "UNSUBSCRIBE": 5, "PUBLISH": 6, "LIST": 7, "STATS": 8, "LOG": 9,
        "CALL": 10, "REPLY": 11, "SERVICE": 12, "NODE": 13}
KIND_NAME = {v: k for k, v in KIND.items()}
F_RLE = 1


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


try:  # 30x faster CRC when available
    import binascii
    def crc16(data, crc=0xFFFF):  # noqa: F811  (binascii.crc_hqx is CRC-16/CCITT with a given start value)
        return binascii.crc_hqx(bytes(data), crc)
except ImportError:
    pass


def rle_decode(b):
    """pairs (count 1..255, byte)"""
    out = bytearray()
    for i in range(0, len(b) - 1, 2):
        out += bytes([b[i + 1]]) * b[i]
    return bytes(out)


def rle_encode(b):
    out = bytearray()
    i = 0
    while i < len(b):
        j = i
        while j < len(b) and j - i < 255 and b[j] == b[i]:
            j += 1
        out += bytes([j - i, b[i]])
        i = j
    return bytes(out)


def frame(kind, fid, payload, flags=0):
    body = struct.pack("<BBHH", kind, flags, fid, len(payload)) + bytes(payload)
    return SYNC + body + struct.pack("<H", crc16(body))


_PRINTABLE = set(range(0x20, 0x7F)) | {9}
_WORDY = set(b"abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 .,:;%()=/-_[]|")


def _clean(line):
    """a real log line: printable ASCII, 1..300 characters, mostly letters / digits / spaces (pieces of a damaged
    frame are short runs of random characters)"""
    if not line or len(line) > 300 or any(c not in _PRINTABLE for c in line):
        return False
    return len(line.strip()) == 0 or (len(line) >= 6 and sum(c in _WORDY for c in line) >= 0.8 * len(line))


class FrameParser:
    """feed(bytes) -> list of ('text', line) and ('frame', kind, flags, id, payload).
    Frames with a bad CRC are skipped (counted in bad_crc). Text outside frames is kept only as clean lines (printable
    ASCII, at most 300 characters): the pieces of a damaged frame are counted in noise_bytes instead of printed."""
    def __init__(self):
        self.buf = bytearray()
        self.text = bytearray()
        self.bad_crc = 0
        self.frames = 0
        self.noise_bytes = 0

    def _text(self, seg, out):
        while seg:
            k = seg.find(b"\n")
            part = seg if k < 0 else seg[:k]
            self.text += part
            if k < 0:
                if len(self.text) > 300:
                    self.noise_bytes += len(self.text); self.text = bytearray()
                return
            line = bytes(self.text).rstrip(b"\r")
            self.text = bytearray()
            if _clean(line):
                out.append(("text", line.decode("ascii")))
            else:
                self.noise_bytes += len(line) + 1
            seg = seg[k + 1:]

    def feed(self, data):
        self.buf += data
        out = []
        b = self.buf
        i = 0
        while i < len(b):
            j = b.find(SYNC, i)
            if j < 0:                                # no frame start: text up to the end (keep a lone A5 at the end)
                end = len(b) - 1 if b[-1] == 0xA5 else len(b)
                self._text(bytes(b[i:end]), out); i = end
                break
            if j > i:
                self._text(bytes(b[i:j]), out)
            i = j
            if i + 8 > len(b):
                break
            kind, flags, fid, n = struct.unpack_from("<BBHH", b, i + 2)
            if i + 10 + n > len(b):
                break
            body = bytes(b[i + 2:i + 8 + n])
            crc = struct.unpack_from("<H", b, i + 8 + n)[0]
            if crc == crc16(body):
                payload = body[6:]
                if flags & F_RLE:
                    payload = rle_decode(payload)
                if kind == KIND["LOG"]:
                    out.append(("text", payload.decode("utf-8", "replace")))
                else:
                    out.append(("frame", kind, flags, fid, payload))
                self.frames += 1
                i += 10 + n
            else:
                self.bad_crc += 1
                self.noise_bytes += 2
                i += 2                               # look for the next A5 5A after this one
        del b[:i]
        return out


def cstrings(payload, n):
    """first n zero-terminated strings and the rest"""
    parts, p = [], 0
    for _ in range(n):
        q = payload.index(0, p)
        parts.append(bytes(payload[p:q]).decode())
        p = q + 1
    return parts, payload[p:]


if __name__ == "__main__":
    print("%d message types from %s" % (len(TYPES), MSG_DIR))
    for t in sorted(TYPES):
        print(" ", t)
    sys.exit(0)
