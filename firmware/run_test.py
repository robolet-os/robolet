#!/usr/bin/env python3
"""Talks to the test firmware: finds the board's port, sends test commands, prints and saves the output.
Usage: python run_test.py [b c v n k_1_1_0 K_4 ...] [--port /dev/ttyACM0]   ("_" stands for a space)
The log is saved to ../results/chip_<date>_<time>.txt
"""
import sys, time, os, datetime
import serial, serial.tools.list_ports

# Seconds to wait for each test to print its RESULT line
TIMEOUT = {"b": 20, "c": 90, "v": 90, "n": 400, "k": 1200, "K": 14400, "l": 30, "p": 15, "m": 40}
KNOWN_VID = {0x303A: "ESP32-S3 USB", 0x1A86: "CH34x USB-UART", 0x10C4: "CP210x USB-UART", 0x0403: "FTDI USB-UART"}


def find_port():
    ports = [p for p in serial.tools.list_ports.comports() if p.vid in KNOWN_VID]
    if not ports:
        ports = [p for p in serial.tools.list_ports.comports() if "ttyACM" in p.device or "ttyUSB" in p.device]
    ports.sort(key=lambda p: (p.vid != 0x303A, p.device))  # prefer the chip's own USB port
    return [p.device for p in ports]


def reopen_and_ask(dev, log):
    """after a restart: wait for the port, run the board check (it prints the restart reason and the last stage)"""
    for _ in range(40):
        time.sleep(0.5)
        try:
            s = serial.Serial(); s.port, s.baudrate, s.timeout = dev, 115200, 0.2; s.dtr = False; s.rts = False; s.open()
            break
        except Exception:
            continue
    else:
        print("(the port did not come back)"); return None
    time.sleep(2.5); s.reset_input_buffer(); s.write(b"b\n")
    t0, buf = time.time(), b""
    while time.time() - t0 < 20:
        buf += s.read(512)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("utf-8", "replace").rstrip("\r")
            print(text, flush=True); log.write(text + "\n")
            if text.startswith("RESULT"):
                return s
    return s


def main():
    args = sys.argv[1:]
    port = None
    if "--port" in args:
        k = args.index("--port"); port = args[k + 1]; del args[k:k + 2]
    cmds = args or ["b", "c"]
    candidates = [port] if port else find_port()
    if not candidates:
        print("No board found. Check the USB cable, then run again.")
        return 1
    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(here, "..", "results")
    os.makedirs(out_dir, exist_ok=True)
    log_path = os.path.join(out_dir, "chip_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".txt")

    for dev in candidates:
        print(f"--- trying {dev}")
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = dev, 115200, 0.2
        s.dtr = False; s.rts = False             # do not reset the board when opening
        try:
            s.open()
        except Exception as e:
            print(f"    cannot open: {e}"); continue
        time.sleep(2.5)                          # board may still be booting after an upload
        s.reset_input_buffer()
        log = open(log_path, "w", encoding="utf-8", buffering=1)   # line by line: the file is up to date while it runs
        got_any = False
        for c in cmds:
            s.write((c.replace("_", " ") + "\n").encode())   # "k_1_1_0" -> "k 1 1 0"
            t0, buf, done, lost = time.time(), b"", False, False
            while time.time() - t0 < TIMEOUT.get(c[0], 30):
                try:
                    chunk = s.read(512)
                except (serial.SerialException, OSError):  # the port went away: the chip restarted (crash?)
                    print("\n(the port went away: the chip restarted; asking it why)"); log.write("\n(the port went away: the chip restarted)\n")
                    s = reopen_and_ask(dev, log); lost = True
                    break
                if not chunk:
                    continue
                got_any = True
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", "replace").rstrip("\r")
                    print(text, flush=True); log.write(text + "\n")
                    if text.startswith("RESULT") or text.startswith("disabled:"):
                        done = True
                if done:
                    time.sleep(0.3)
                    rest = s.read(4096).decode("utf-8", "replace")
                    if rest:
                        print(rest, end=""); log.write(rest)
                    break
            if lost:
                print("=== stopped here: the chip restarted during '%s' (the board check above says why)" % c)
                log.write("=== stopped here: the chip restarted during '%s'\n" % c)
                break
            if not got_any:
                break                            # nothing at all on this port: try the next one
            if not done:
                print(f"(no RESULT line for '{c}' within {TIMEOUT.get(c[0], 30)} s)")
        log.close()
        if s:
            s.close()
        if got_any:
            print(f"\n=== saved: {os.path.normpath(log_path)}")
            return 0
        os.remove(log_path)
        print("    no answer on this port")
    print("The board did not answer on any port. Press the board's RST button and run again.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
