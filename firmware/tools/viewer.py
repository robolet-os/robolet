#!/usr/bin/env python3
"""Viewer for the chip's topics, in the browser (like a small rviz2). Needs pyserial for a live chip; nothing else.

  python3 tools/viewer.py --chip "k 0 1 0 3"          run a mission on the chip (bridge) and show it live
  python3 tools/viewer.py --chip "k 0 1 0 3 rt"       the same in real time (one cycle per 1/7 s)
  python3 tools/viewer.py --play ../results/bridge_20261126_115054.bin --speed 2     a recording, without the chip
       --record run.bin   (with --chip) also save what arrived;  --port /dev/ttyACM0   serial port (default: first found)
  The page opens in the default browser: http://localhost:8765 (--port-http to change, --no-browser not to open).

Shown: the SLAM map as it grows (/map), the estimated pose (/pose while mapping, /amcl/robot_pose after), the true
pose of the simulation (/ground_truth), the lidar points (/scan) drawn from the estimated pose, the planned path
(/plan), the candidate poses of a relocalization (/localization/candidates), the localization status and the log.
"""
import argparse
import base64
import json
import math
import os
import queue
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ros2wire as rw  # noqa: E402
import robolet  # noqa: E402

SUBS = ["/map:2", "/scan", "/amcl/robot_pose", "/pose", "/plan", "/ground_truth", "/localization/status",
        "/localization/candidates"]
STATUS = {0: "healthy", 1: "unhealthy", 2: "LOST", 3: "lost (leg aborted)", 4: "relocalizing", 5: "relocalized", 6: "STOPPED"}


def r3(v):
    return round(float(v), 3)                       # numpy values (ros2wire with numpy) -> plain numbers for JSON


def yaw(q):
    return 2.0 * math.atan2(float(q["z"]), float(q["w"]))


def pose2(p):
    return [r3(p["position"]["x"]), r3(p["position"]["y"]), round(yaw(p["orientation"]), 4)]


class State:
    """latest message of each kind, turned into small JSON events for the page"""
    def __init__(self):
        self.clients, self.lock = [], threading.Lock()
        self.last = {}                                   # event kind -> last event (sent to a page that opens later)
        self.hist = []                                   # poses and log lines so far
        self.counts = {}

    def emit(self, kind, data):
        ev = json.dumps({"k": kind, "d": data}, separators=(",", ":"))
        with self.lock:
            if kind in ("est", "truth", "log"):          # kept in full, so a page opened later still draws the trails
                self.hist.append(ev)
                if len(self.hist) > 20000:
                    del self.hist[:5000]
            else:
                self.last[kind] = ev
            for q in self.clients:
                q.put(ev)

    def message(self, name, typ, m):
        try:
            self._message(name, typ, m)
        except Exception as e:                           # one odd message should not stop the viewer
            self.text("[viewer] %s: %s" % (name, e))

    def _message(self, name, typ, m):
        self.counts[name] = self.counts.get(name, 0) + 1
        if name == "/map":
            info = m["info"]
            d = m["data"]
            raw = d.astype("int8").tobytes() if hasattr(d, "tobytes") else bytes((int(v) & 0xFF) for v in d)
            self.emit("map", {"w": int(info["width"]), "h": int(info["height"]), "res": float(info["resolution"]),
                              "ox": float(info["origin"]["position"]["x"]), "oy": float(info["origin"]["position"]["y"]),
                              "data": base64.b64encode(raw).decode()})
        elif name == "/pose":
            self.emit("est", {"p": pose2(m["pose"]["pose"]), "src": "slam"})
        elif name == "/amcl/robot_pose":
            self.emit("est", {"p": pose2(m["pose"]), "src": "amcl"})
        elif name == "/ground_truth":
            self.emit("truth", {"p": pose2(m["pose"]["pose"])})
        elif name == "/scan":
            pts, a, inc = [], float(m["angle_min"]), float(m["angle_increment"])
            for i, r in enumerate(m["ranges"]):
                r = float(r)
                if math.isfinite(r) and r > 0:
                    th = a + i * inc
                    pts.append([r3(r * math.cos(th)), r3(r * math.sin(th))])
            self.emit("scan", {"pts": pts})
        elif name == "/plan":
            self.emit("plan", {"pts": [[r3(p["pose"]["position"]["x"]), r3(p["pose"]["position"]["y"])] for p in m["poses"]]})
        elif name == "/localization/candidates":
            self.emit("cand", {"poses": [pose2(p) for p in m["poses"]]})
        elif name == "/localization/status":
            self.emit("status", {"s": STATUS.get(int(m["data"]), str(m["data"]))})

    def text(self, line):
        print(line, flush=True)                          # also in the terminal (and in test.sh's .txt)
        self.emit("log", {"t": line})
        if line.startswith("RESULT") or line.startswith("MISSION"):
            self.emit("done", {"t": line})


# ------------------------------------------------------------------------------------------------ sources
def run_play(state, path, speed):
    data = open(path, "rb").read()
    parser, sess = rw.FrameParser(), robolet.Session(quiet=True)
    clock = {"t0": None, "w0": None}

    def on_msg(name, typ, m):
        h = m.get("header") if isinstance(m, dict) else None
        if h and speed > 0:                              # keep the recording's own pace (message stamps)
            t = h["stamp"]["sec"] + h["stamp"]["nanosec"] * 1e-9
            if clock["t0"] is None:
                clock["t0"], clock["w0"] = t, time.time()
            wait = clock["w0"] + (t - clock["t0"]) / speed - time.time()
            if wait > 0:
                time.sleep(min(wait, 1.0))
        state.message(name, typ, m)
    sess.on_message = on_msg
    time.sleep(1.0)                                      # give the page a moment to connect
    for i in range(0, len(data), 4096):
        items = parser.feed(data[i:i + 4096])
        for it in items:
            if it[0] == "text":
                state.text(it[1])
        sess.handle(items)
    state.text("[viewer] end of recording (%d frames, %d damaged)" % (parser.frames, parser.bad_crc))


def find_port():
    """the chip's own USB port first (vendor 303A), then USB-UART chips -- the same order as run_test.py"""
    from serial.tools import list_ports
    known = (0x303A, 0x1A86, 0x10C4, 0x0403)
    ports = [p for p in list_ports.comports() if p.vid in known]
    ports.sort(key=lambda p: (p.vid != 0x303A, p.device))
    if ports:
        return ports[0].device
    return robolet.find_port()


def run_chip(state, cmd, port, record=None):
    import serial
    port = port or find_port()
    try:
        s = serial.Serial(port, 115200, timeout=0.05)
    except Exception as e:
        state.text("[viewer] cannot open %s: %s (another program using it? close pio device monitor)" % (port, e))
        return
    if "bridge" not in cmd:
        cmd += " bridge"
    s.reset_input_buffer()
    s.write(b"\n" + cmd.encode() + b"\n")
    state.text("[viewer] %s: sent '%s'" % (port, cmd))
    rec = open(record, "wb", buffering=0) if record else None
    parser, sess = rw.FrameParser(), robolet.Session(quiet=True, on_message=state.message)
    subscribed, t_end, t0, nbytes, t_sum = False, None, time.time(), 0, time.time()
    while True:
        data = s.read(65536)
        if not data and nbytes == 0 and time.time() - t0 > 5:
            state.text("[viewer] nothing from %s in 5 s: is it the ESP32 with firmware 0.10? (press RST, or use --port)" % port)
            t0 = time.time() + 25                        # say it again after 30 s
        if time.time() - t_sum > 5 and nbytes:
            t_sum = time.time()
            print("[viewer] %d KB received, %d frames, %d damaged; %s" % (nbytes // 1024, parser.frames, parser.bad_crc,
                  ", ".join("%s %d" % (k, v) for k, v in sorted(state.counts.items()))), flush=True)
        if data:
            nbytes += len(data)
            if rec:
                rec.write(data)
            items = parser.feed(data)
            for it in items:
                if it[0] == "text":
                    state.text(it[1])
            sess.handle(items)
            if sess.hello and not subscribed:
                s.write(robolet.subscribe_frames(SUBS)); subscribed = True
                state.text("[viewer] chip: " + sess.hello)
        if sess.done and t_end is None:
            t_end = time.time() + 1.0
        if t_end and time.time() > t_end:
            break
    state.text("[viewer] run over: %d frames, %d damaged" % (parser.frames, parser.bad_crc))
    if rec:
        rec.close(); state.text("[viewer] recording: " + record)


# ------------------------------------------------------------------------------------------------ web page
PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8"><title>robolet viewer</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#101418;--panel:#182028;--fg:#d8e0e8;--dim:#7c8a98;--free:#1e2a33;--occ:#e8eef4;--unk:#101418;
--est:#36c2ff;--truth:#ffb000;--scan:#ff4d6d;--plan:#5ce08a;--cand:#c77dff}
body{margin:0;background:var(--bg);color:var(--fg);font:13px system-ui,sans-serif;display:flex;height:100vh}
#v{flex:1;position:relative}canvas{position:absolute;inset:0;width:100%;height:100%}
#side{width:300px;background:var(--panel);padding:10px;box-sizing:border-box;display:flex;flex-direction:column;gap:8px}
h1{font-size:15px;margin:0}.row{display:flex;justify-content:space-between}.dim{color:var(--dim)}
label{display:block}#log{flex:1;overflow:auto;font:11px ui-monospace,monospace;white-space:pre-wrap;color:var(--dim)}
.k{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px;vertical-align:-1px}
#status{font-weight:600}
</style></head><body>
<div id="v"><canvas id="c"></canvas></div>
<div id="side">
<h1>robolet viewer</h1>
<div class="row"><span>localization</span><span id="status">-</span></div>
<div class="row"><span>pose (estimate)</span><span id="pe" class="dim">-</span></div>
<div class="row"><span>error vs. truth</span><span id="err" class="dim">-</span></div>
<div>
<label><input type="checkbox" id="sMap" checked> map</label>
<label><input type="checkbox" id="sScan" checked><span class="k" style="background:var(--scan)"></span>lidar (/scan)</label>
<label><input type="checkbox" id="sPlan" checked><span class="k" style="background:var(--plan)"></span>path (/plan)</label>
<label><input type="checkbox" id="sEst" checked><span class="k" style="background:var(--est)"></span>estimated pose</label>
<label><input type="checkbox" id="sTruth" checked><span class="k" style="background:var(--truth)"></span>true pose (simulation)</label>
<label><input type="checkbox" id="sCand" checked><span class="k" style="background:var(--cand)"></span>relocalization candidates</label>
<label><input type="checkbox" id="sTrail" checked> trail</label>
</div>
<div id="log"></div>
</div>
<script>
const cv=document.getElementById('c'),cx=cv.getContext('2d');
const S={map:null,img:null,est:null,src:'',truth:null,scan:[],plan:[],cand:[],trail:[],ttrail:[]};
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
function fit(){cv.width=cv.clientWidth*devicePixelRatio;cv.height=cv.clientHeight*devicePixelRatio;draw();}
addEventListener('resize',fit);
function mapImage(m){
  const raw=atob(m.data),img=new ImageData(m.w,m.h);
  const f=[0x1e,0x2a,0x33],o=[0xe8,0xee,0xf4],u=[0x10,0x14,0x18];
  for(let j=0;j<m.h;j++)for(let i=0;i<m.w;i++){
    const v=raw.charCodeAt(j*m.w+i),c=v===100?o:(v===0?f:u),k=((m.h-1-j)*m.w+i)*4;   // row 0 = bottom
    img.data[k]=c[0];img.data[k+1]=c[1];img.data[k+2]=c[2];img.data[k+3]=255;}
  const oc=document.createElement('canvas');oc.width=m.w;oc.height=m.h;oc.getContext('2d').putImageData(img,0,0);return oc;}
function view(){ // world (m) -> canvas pixels; fit the map extent
  const m=S.map||{w:200,h:160,res:0.05,ox:-1,oy:-1};
  const W=m.w*m.res,H=m.h*m.res,s=Math.min(cv.width/W,cv.height/H)*0.96;
  const x0=(cv.width-W*s)/2,y0=(cv.height+H*s)/2;
  return {s,tx:x=>x0+(x-m.ox)*s,ty:y=>y0-(y-m.oy)*s,W,H,x0,y0};}
function robot(p,color,r,V){const x=V.tx(p[0]),y=V.ty(p[1]),R=r*V.s;
  cx.strokeStyle=color;cx.lineWidth=2*devicePixelRatio;cx.beginPath();cx.arc(x,y,R,0,7);cx.stroke();
  cx.beginPath();cx.moveTo(x,y);cx.lineTo(x+Math.cos(p[2])*R*1.6,y-Math.sin(p[2])*R*1.6);cx.stroke();}
const on=id=>document.getElementById(id).checked;
function draw(){
  const V=view();cx.fillStyle=css('--bg');cx.fillRect(0,0,cv.width,cv.height);
  if(S.img&&on('sMap')){cx.imageSmoothingEnabled=false;cx.drawImage(S.img,V.x0,V.y0-V.H*V.s,V.W*V.s,V.H*V.s);}
  const line=(pts,color,w)=>{if(pts.length<2)return;cx.strokeStyle=color;cx.lineWidth=w*devicePixelRatio;cx.beginPath();
    let up=true;for(const p of pts){if(!p){up=true;continue;}if(up)cx.moveTo(V.tx(p[0]),V.ty(p[1]));else cx.lineTo(V.tx(p[0]),V.ty(p[1]));up=false;}cx.stroke();};
  if(on('sTrail')){line(S.ttrail,css('--truth')+'66',1.5);line(S.trail,css('--est')+'99',1.5);}
  if(on('sPlan'))line(S.plan,css('--plan'),2);
  if(on('sScan')&&S.est){const p=S.est,c=Math.cos(p[2]),s=Math.sin(p[2]),r=1.6*devicePixelRatio;cx.fillStyle=css('--scan');
    for(const q of S.scan){const x=p[0]+c*q[0]-s*q[1],y=p[1]+s*q[0]+c*q[1];cx.fillRect(V.tx(x)-r/2,V.ty(y)-r/2,r,r);}}
  if(on('sCand'))for(const q of S.cand)robot(q,css('--cand'),0.12,V);
  if(on('sTruth')&&S.truth)robot(S.truth,css('--truth'),0.15,V);
  if(on('sEst')&&S.est)robot(S.est,css('--est'),0.15,V);}
document.querySelectorAll('input').forEach(e=>e.onchange=draw);
let pending=false;const later=()=>{if(!pending){pending=true;requestAnimationFrame(()=>{pending=false;draw();});}};
function trail(T,p){const q=T[T.length-1];   // a jump (kidnap, relocalization) breaks the line
  if(q&&Math.hypot(p[0]-q[0],p[1]-q[1])>0.5)T.push(null);T.push(p);if(T.length>6000)T.shift();}
const logEl=document.getElementById('log');
function log(t){logEl.textContent+=t+'\n';if(logEl.textContent.length>40000)logEl.textContent=logEl.textContent.slice(-30000);logEl.scrollTop=1e9;}
function upd(){if(S.est){document.getElementById('pe').textContent=S.src+' '+S.est[0].toFixed(2)+', '+S.est[1].toFixed(2)+', '+(S.est[2]*57.3).toFixed(0)+'°';
  if(S.truth)document.getElementById('err').textContent=Math.hypot(S.est[0]-S.truth[0],S.est[1]-S.truth[1]).toFixed(3)+' m';}}
const es=new EventSource('/events');
es.onmessage=e=>{const m=JSON.parse(e.data),d=m.d;
  if(m.k==='map'){S.map=d;S.img=mapImage(d);}
  else if(m.k==='est'){S.est=d.p;S.src=d.src;trail(S.trail,d.p);upd();}
  else if(m.k==='truth'){S.truth=d.p;trail(S.ttrail,d.p);upd();}
  else if(m.k==='scan')S.scan=d.pts;
  else if(m.k==='plan')S.plan=d.pts;
  else if(m.k==='cand')S.cand=d.poses;
  else if(m.k==='status'){const el=document.getElementById('status');el.textContent=d.s;
    el.style.color=/LOST|STOP/.test(d.s)?css('--scan'):(/reloc/.test(d.s)?css('--cand'):css('--plan'));
    if(d.s==='healthy'||d.s==='unhealthy')S.cand=[];}
  else if(m.k==='log')log(d.t);
  later();};
fit();
</script></body></html>
"""


def make_handler(state):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                q = queue.Queue()
                with state.lock:
                    last = dict(state.last)
                    first, final = last.pop("map", None), last.pop("status", None)   # status after candidates
                    backlog = [e for e in [first] + state.hist + list(last.values()) + [final] if e]
                    state.clients.append(q)
                try:
                    for ev in backlog:
                        self.wfile.write(b"data: " + ev.encode() + b"\n\n")
                    self.wfile.flush()
                    while True:
                        try:
                            ev = q.get(timeout=10)
                            self.wfile.write(b"data: " + ev.encode() + b"\n\n")
                        except queue.Empty:
                            self.wfile.write(b": keep-alive\n\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with state.lock:
                        state.clients.remove(q)
                return
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--chip", help='command for the chip, e.g. "k 0 1 0 3"')
    g.add_argument("--play", help="recording (.bin from robolet.py --record or test.sh bridge)")
    ap.add_argument("--speed", type=float, default=1.0, help="play speed (0 = as fast as possible)")
    ap.add_argument("--port", help="serial port (default: the first ESP32 port found)")
    ap.add_argument("--record", help="with --chip: also save the received bytes (play them later with --play)")
    ap.add_argument("--port-http", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true", help="do not open the page in the default browser")
    args = ap.parse_args()
    state = State()
    srv = ThreadingHTTPServer(("127.0.0.1", args.port_http), make_handler(state))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = "http://localhost:%d" % args.port_http
    print("[viewer] page: %s  (Ctrl+C to quit)" % url)
    if not args.no_browser:
        import webbrowser
        threading.Timer(0.3, lambda: webbrowser.open(url)).start()
    try:
        if args.play:
            run_play(state, args.play, args.speed)
        else:
            run_chip(state, args.chip, args.port, args.record)
        print("[viewer] done; the page stays open until Ctrl+C")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
