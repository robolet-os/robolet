#!/usr/bin/env python3
"""Relay for the real robot without rewiring (firmware command "r relay"): the lidar and the motor controller stay on
the USB ports of the robot's Raspberry Pi (or a PC); this program passes their bytes to and from the chip through the
daemon of tools/ros2.py. Mapping, localization, planning and the motor commands are computed on the chip; this
program only copies bytes. Needs pyserial.

  python3 tools/ros2.py daemon --chip "r relay"        terminal 1 (on the Raspberry Pi: the chip is on its USB)
  python3 tools/relay.py                               terminal 2

  --lidar /dev/mylidar   YDLIDAR USB adapter (CP2102), 128000 bps; default: /dev/mylidar (Jarabot udev rule), else
                         the first port with USB id 10c4
  --base /dev/mydriver   Arduino Nano with motor.ino (CH340), 57600 bps; default: /dev/mydriver, else USB id 1a86

  lidar bytes   -> /relay/lidar_rx  (std_msgs/UInt8MultiArray)  -> the chip's lidar driver
  Arduino bytes -> /relay/base_rx                               -> the chip's base_driver (encoder counts)
  /relay/base_tx (the chip's "pwm,<turn>,<fwd>" lines)          -> Arduino
Safety: when no motor command comes from the chip for 0.5 s, the relay sends "pwm,1500,1500" (stop) itself.
"""
import argparse
import base64
import json
import os
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ros2wire as rw  # noqa: E402
import ros2 as r2  # noqa: E402

TYPE = "std_msgs/msg/UInt8MultiArray"
STOP = b"pwm,1500,1500\n"


def cdr_bytes(d):
    """std_msgs/UInt8MultiArray CDR: header, no dimensions, data_offset 0, the bytes"""
    return b"\x00\x01\x00\x00" + struct.pack("<III", 0, 0, len(d)) + d


def find(default, vid):
    if os.path.exists(default):
        return default
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        if p.vid == vid:
            return p.device
    sys.exit("no port for %s (USB id %04x): use --lidar / --base" % (default, vid))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lidar"); ap.add_argument("--base")
    a = ap.parse_args()
    import serial
    lp, bp = a.lidar or find("/dev/mylidar", 0x10C4), a.base or find("/dev/mydriver", 0x1A86)
    lid = serial.Serial(lp, 128000, timeout=0)
    lid.dtr = True                                           # the adapter turns the lidar motor on with DTR (X4 family)
    lid.write(b"\xA5\x60")                                   # start scanning (models with an Rx line)
    base = serial.Serial(bp, 57600, timeout=0)
    print("[relay] lidar %s, motor controller %s (the Arduino restarts when its port opens: 2 s)" % (lp, bp), flush=True)
    time.sleep(2.0)
    base.reset_input_buffer(); base.write(STOP)

    g = r2.Client().graph(refresh=True)
    if g.get("done"):
        sys.exit("the run on the chip is over: start it again (tools/ros2.py daemon --chip \"r relay\")")
    c = r2.Client()._open({"op": "pubstream"})[1]           # one connection for all the bytes to the chip
    if "error" in json.loads(c.readline()):
        sys.exit("the daemon does not take pubstream: update tools/ros2.py")
    wlock = threading.Lock()

    def send(topic, d):
        line = json.dumps({"topic": topic, "type": TYPE, "cdr": base64.b64encode(cdr_bytes(d)).decode()}) + "\n"
        with wlock:
            c.write(line); c.flush()

    st = {"cmd_t": 0.0, "cmds": 0, "lid": 0, "base": 0, "last": ""}

    def from_chip():                                         # the chip's motor commands -> Arduino
        try:
            for r in r2.Client().stream(["/relay/base_tx"], 0, raw=True):
                d = bytes(rw.decode(TYPE, base64.b64decode(r["cdr"]))["data"])
                base.write(d); st["cmd_t"] = time.time(); st["cmds"] += 1; st["last"] = d.decode(errors="replace").strip()
        except Exception as e:                               # noqa: BLE001
            print("[relay] commands from the chip: %s" % e, flush=True)
    threading.Thread(target=from_chip, daemon=True).start()

    t_stat = t_stop = time.time()
    try:
        while True:
            d = lid.read(4096)
            for i in range(0, len(d), 2000):
                send("/relay/lidar_rx", d[i:i + 2000])
            st["lid"] += len(d)
            e = base.read(4096)
            for i in range(0, len(e), 2000):
                send("/relay/base_rx", e[i:i + 2000])
            st["base"] += len(e)
            now = time.time()
            if now - st["cmd_t"] > 0.5 and now - t_stop > 0.2:   # no command from the chip: stop the wheels
                base.write(STOP); t_stop = now
            if now - t_stat > 5:
                print("[relay] 5 s: lidar %.1f KB/s, Arduino %.1f KB/s to the chip | %d motor commands from the chip, last %s"
                      % (st["lid"] / 5120, st["base"] / 5120, st["cmds"], st["last"] or "-"), flush=True)
                st["lid"] = st["base"] = st["cmds"] = 0; t_stat = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            base.write(STOP); time.sleep(0.02)
        lid.write(b"\xA5\x65"); lid.dtr = False
        print("[relay] stopped: wheels stopped, lidar off", flush=True)


if __name__ == "__main__":
    main()
