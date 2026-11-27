import numpy as np, heapq, time, json, sys, signal
from scipy import ndimage
from scipy.optimize import least_squares
from collections import Counter

res = 0.05; off = 20; dt = 1 / 7; LASER = np.array([0.06, 0.0, 0.0])
PROGRESS = []
HOOK = {}

def wrap(a): return (a + np.pi) % (2 * np.pi) - np.pi
def compose(a, b):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])])
def inv(a):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]])
def to_g(px, py): return (np.asarray(py) / res).astype(int) + off, (np.asarray(px) / res).astype(int) + off
def to_gc(px, py):
    gi, gj = to_g(px, py); return np.clip(gi, 0, SH[0] - 1), np.clip(gj, 0, SH[1] - 1)
def inb(gi, gj): return (gi >= 0) & (gi < SH[0]) & (gj >= 0) & (gj < SH[1])
def tdist(sg):
    D = np.full(SH, 9.0)
    for (x1, y1), (x2, y2) in sg:
        ax_, ay_ = x2 - x1, y2 - y1
        t = np.clip(((X - x1) * ax_ + (Y - y1) * ay_) / (ax_ * ax_ + ay_ * ay_), 0, 1)
        D = np.minimum(D, np.hypot(X - (x1 + t * ax_), Y - (y1 + t * ay_)))
    return D

def setup(name):
    global W, H, SH, SEGS, WAY, BOXES, X, Y, inside, TD, NAME
    NAME = name
    W, H = 8.0, 6.0
    outer = [((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0))]
    if name == 'room':
        SEGS = outer + [((2, 0), (2, 3)), ((5, 6), (5, 3.5)), ((3.5, 1.5), (4.5, 1.5)), ((4.5, 1.5), (4.5, 2.5)), ((6, 1), (7, 2))]
        WAY = [(1, 1), (1, 5), (3.5, 5), (3.5, 3), (6.5, 3), (7, 5), (7, 0.5), (5, 0.6), (3, 0.6), (3, 2.5), (1, 2.5), (1, 1), (1, 4)]
        BOXES = [((5.0, 2.55), (6.1, 2.55), (6.1, 3.45), (5.0, 3.45)), ((2.6, 3.8), (3.2, 3.8), (3.2, 4.4), (2.6, 4.4)), ((5.6, 0.3), (6.0, 0.3), (6.0, 0.8), (5.6, 0.8))]
    elif name in ('s_corridor', 's_corridor_lm'):
        # three 2 m wide lanes joined in an S: lane1 y0-2, lane2 y2-4, lane3 y4-6
        SEGS = outer + [((0, 2), (6, 2)), ((2, 4), (8, 4))]
        if name == 's_corridor_lm':   # one fixed shelf (1.0 m x 0.5 m) against the top wall of lane3 breaks the symmetry
            SEGS += [((4.5, 5.5), (5.5, 5.5)), ((4.5, 5.5), (4.5, 6.0)), ((5.5, 5.5), (5.5, 6.0))]
        WAY = [(1, 1), (7, 1), (7, 3), (1, 3), (1, 5), (7, 5), (1, 5), (1, 3), (7, 3), (7, 1), (1, 1)]
        BOXES = [((3.6, 2.6), (4.2, 2.6), (4.2, 3.2), (3.6, 3.2)), ((4.0, 0.2), (4.6, 0.2), (4.6, 0.8), (4.0, 0.8)), ((3.0, 4.9), (3.5, 4.9), (3.5, 5.5), (3.0, 5.5))]
    SH = (int(H / res) + 40, int(W / res) + 40)
    gI, gJ = np.mgrid[0:SH[0], 0:SH[1]]
    X = (gJ - off) * res + res / 2; Y = (gI - off) * res + res / 2
    inside = (X > 0) & (X < W) & (Y > 0) & (Y < H)
    TD = tdist(SEGS)

def make_mission(r):
    if NAME == 'room':
        goals = [("A", (7.2, 5.0)), ("B", (3.0, 0.7)), ("C", (6.6, 1.0)), ("D", (3.0, 5.3)), ("E", (4.0, 3.2))]
        order = [goals[i] for i in r.permutation(5)]
        m = order[:2] + [("UNREACHABLE", (2.0, 1.5))] + order[2:] + [("DOCK", (1.0, 1.0))]
    else:
        m = [("END", (7.2, 5.0)), ("UNREACHABLE", (4.0, 2.0)), ("START", (0.8, 1.0)), ("END", (7.2, 5.0)), ("START", (0.8, 1.0)), ("END", (7.2, 5.0))]
    legs = [i for i, (n, _) in enumerate(m) if n != "UNREACHABLE" and i > 0]
    return m, int(r.choice(legs))

class Sim:
    def __init__(s, seed):
        s.rng = np.random.default_rng(seed); r = s.rng
        s.gb = r.uniform(0.002, 0.01) * r.choice([-1, 1]); s.slip = r.uniform(0.04, 0.08)
        s.drop = r.uniform(0.05, 0.10); s.scale = r.uniform(1.0, 1.04); s.world = list(SEGS)
    def scan(s, truth):
        lx_, ly_, lth = compose(truth, LASER); n = 720; r = s.rng
        a = lth + np.linspace(0, 2 * np.pi, n, endpoint=False); dx, dy = np.cos(a), np.sin(a); R = np.full(n, 10.0)
        for (x1, y1), (x2, y2) in s.world:
            ex, ey = x2 - x1, y2 - y1; den = dx * ey - dy * ex
            with np.errstate(divide='ignore', invalid='ignore'):
                t = ((x1 - lx_) * ey - (y1 - ly_) * ex) / den; u = ((x1 - lx_) * dy - (y1 - ly_) * dx) / den
            ok = (t > 0) & (u >= 0) & (u <= 1); R = np.where(ok & (t < R), t, R)
        R = R * (1 + r.standard_normal(n) * 0.015) + r.standard_normal(n) * 0.005
        R[R < 0.12] = 10.0; R[r.random(n) < s.drop] = 10.0
        ang = np.linspace(0, 2 * np.pi, n, endpoint=False); m = R < 9.9
        return LASER[0] + R[m] * np.cos(ang[m]), LASER[1] + R[m] * np.sin(ang[m])
    def drive(s, truth, v, w, base=0.2):
        r = s.rng; truth[2] = wrap(truth[2] + w * dt); truth[0] += v * dt * np.cos(truth[2]); truth[1] += v * dt * np.sin(truth[2])
        dl = (v - w * base / 2) * dt * (1 + r.standard_normal() * s.slip) * s.scale
        dr = (v + w * base / 2) * dt * (1 + r.standard_normal() * s.slip)
        return dl, dr, w + s.gb + r.standard_normal() * 0.01

class Fusion:
    def __init__(s): s.p = np.zeros(3)
    def step(s, dl, dr, gyro, base=0.2):
        d = (dl + dr) / 2; s.p[2] = wrap(s.p[2] + 0.9 * gyro * dt + 0.1 * (dr - dl) / base)
        s.p[0] += d * np.cos(s.p[2]); s.p[1] += d * np.sin(s.p[2]); return s.p.copy()

def score(M, lx, ly, p):
    c, s = np.cos(p[2]), np.sin(p[2]); gi, gj = to_g(p[0] + c * lx - s * ly, p[1] + s * lx + c * ly); ok = inb(gi, gj)
    return M[gi[ok], gj[ok]].sum()
def csm(M, lx, ly, p, xy=0.1, nxy=5, ang=3, nang=5):
    best = (score(M, lx, ly, p), p.copy())
    for dx in np.linspace(-xy, xy, nxy):
        for dy in np.linspace(-xy, xy, nxy):
            for dth in np.radians(np.linspace(-ang, ang, nang)):
                q = p + np.array([dx, dy, dth]); sc = score(M, lx, ly, q)
                if sc > best[0]: best = (sc, q)
    return best[1], best[0]
def add_scan(M, lx, ly, p, w=5):
    c, s = np.cos(p[2]), np.sin(p[2]); gi, gj = to_g(p[0] + c * lx - s * ly, p[1] + s * lx + c * ly); ok = inb(gi, gj)
    np.add.at(M, (gi[ok], gj[ok]), w); np.clip(M, 0, 100, out=M)
def blur(M): return ndimage.gaussian_filter(M.astype(float), 1.0)
def local_map(idxs, scans, poses):
    L = np.zeros(SH, np.int16)
    for k in idxs: add_scan(L, *scans[k], poses[k], w=10)
    return L

def run_slam(sim):
    truth = np.array([WAY[0][0], WAY[0][1], np.pi / 2]); fus = Fusion(); op = fus.p.copy(); est = truth.copy()
    kfs = []; kfp = []; kft = []; edges = []; loops = []; BUF = 10
    lx, ly = sim.scan(truth); kfs.append((lx, ly)); kfp.append(est.copy()); kft.append(truth.copy()); buf = blur(local_map([0], kfs, kfp))
    for wi in range(len(WAY) - 1):
        tgt = np.array(WAY[wi + 1])
        while True:
            dv = tgt - truth[:2]
            if np.linalg.norm(dv) < 0.05: break
            ae = wrap(np.arctan2(dv[1], dv[0]) - truth[2]); v = 0.3 if abs(ae) < 0.3 else 0.0; w = np.clip(2.5 * ae, -1.2, 1.2)
            dl, dr, gy = sim.drive(truth, v, w); o = fus.step(dl, dr, gy); est = compose(est, compose(inv(op), o)); op = o.copy()
            lx, ly = sim.scan(truth); est, _ = csm(buf, lx, ly, est)
            if np.hypot(*(est[:2] - kfp[-1][:2])) > 0.3 or abs(wrap(est[2] - kfp[-1][2])) > 0.5:
                i = len(kfp); rel = compose(inv(kfp[-1]), est); edges.append((i - 1, i, rel, np.array([50, 50, 100.])))
                kfs.append((lx, ly)); kfp.append(est.copy()); kft.append(truth.copy()); buf = blur(local_map(range(max(0, i - BUF + 1), i + 1), kfs, kfp))
                cands = [j for j in range(0, i - 30) if np.hypot(*(kfp[j][:2] - est[:2])) < 1.5]
                if cands and not (loops and i - loops[-1][1] < 5):
                    jc = min(cands, key=lambda j: np.hypot(*(kfp[j][:2] - est[:2])))
                    L = local_map(range(max(0, jc - 5), min(jc + 6, i - 30)), kfs, kfp); Lb = blur(L); D = ndimage.distance_transform_edt(L < 10) * res
                    p1, _ = csm(Lb, lx[::3], ly[::3], est, xy=0.5, nxy=11, ang=10, nang=11); p2, _ = csm(Lb, lx, ly, p1, xy=0.05, nxy=5, ang=1.5, nang=5)
                    c_, s_ = np.cos(p2[2]), np.sin(p2[2]); gi, gj = to_g(p2[0] + c_ * lx - s_ * ly, p2[1] + s_ * lx + c_ * ly); ok = inb(gi, gj)
                    d = np.full(len(lx), 9.0); d[ok] = D[gi[ok], gj[ok]]
                    if (d < 0.1).mean() > 0.55:
                        edges.append((jc, i, compose(inv(kfp[jc]), p2), np.array([50, 50, 100.]))); loops.append((jc, i))
                        def resid(x):
                            P = x.reshape(-1, 3); out = [(P[0] - kfp[0]) * 1e3]
                            for a, b, r_, wv in edges:
                                e = compose(inv(P[a]), P[b]) - r_; e[2] = wrap(e[2]); out.append(e * wv)
                            return np.concatenate(out)
                        newP = least_squares(resid, np.array(kfp).ravel(), loss='huber', f_scale=1.0).x.reshape(-1, 3)
                        for k in range(len(kfp)): kfp[k] = newP[k].copy()
                        est = kfp[-1].copy(); buf = blur(local_map(range(max(0, i - BUF + 1), i + 1), kfs, kfp))
    ch = [kft[0].copy()]
    for a, b, r_, wv in edges:
        if b == a + 1 and len(ch) == b: ch.append(compose(ch[-1], r_))
    e_fe = np.mean([np.hypot(*(ch[k][:2] - kft[k][:2])) for k in range(len(ch))])
    e_lc = np.mean([np.hypot(*(kfp[k][:2] - kft[k][:2])) for k in range(len(kfp))])
    MAP = local_map(range(len(kfp)), kfs, kfp); occ = MAP >= 40
    return MAP, occ, dict(loops=len(loops), err_no_loop=round(float(e_fe), 3), err_loop=round(float(e_lc), 3),
                          wall_err=round(float(np.median(TD[occ])), 3), bad_pct=round(100 * float((TD[occ] > 0.15).mean()), 1))

Z_HIT, Z_RAND, SIG = 0.95, 0.05, 0.2; A_SLOW, A_FAST = 0.001, 0.1; UPD_D, UPD_A = 0.25, 0.2
ROBOT_R = 0.18; INFL = 0.55; CSF = 6.0; REQ_R, REQ_T = 0.5, 10.0
def astar(cost, s, g):
    if cost[g] >= 253: return None
    h = lambda a: np.hypot(a[0] - g[0], a[1] - g[1]); openl = [(h(s), 0, s)]; came = {s: None}; gs = {s: 0}
    nb = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    while openl:
        _, c, u = heapq.heappop(openl)
        if u == g:
            p = []
            while u: p.append(u); u = came[u]
            return p[::-1]
        if c > gs[u]: continue
        for di, dj, wc in nb:
            v = (u[0] + di, u[1] + dj)
            if not (0 <= v[0] < SH[0] and 0 <= v[1] < SH[1]) or cost[v] >= 253: continue
            nc = c + wc * (1 + cost[v] / 50.0)
            if nc < gs.get(v, 1e9): gs[v] = nc; came[v] = u; heapq.heappush(openl, (nc + h(v), nc, v))
    return None

def run_mission(sim, MAP, occ, seed, lost_mode='fit'):
    r = np.random.default_rng(seed + 100)
    dist_static = ndimage.distance_transform_edt(~occ) * res; MAPb = blur(MAP); occ_r = ndimage.binary_dilation(occ)
    free_cells = np.argwhere((dist_static > 0.3) & inside)
    def rparts(n):
        sel = free_cells[r.integers(len(free_cells), size=n)]
        return np.c_[(sel[:, 1] - off) * res + r.uniform(0, res, n), (sel[:, 0] - off) * res + r.uniform(0, res, n), r.uniform(-np.pi, np.pi, n)]
    def lik(parts, lx, ly):
        c, s = np.cos(parts[:, 2])[:, None], np.sin(parts[:, 2])[:, None]
        gi, gj = to_g(parts[:, 0:1] + c * lx - s * ly, parts[:, 1:2] + s * lx + c * ly); ok = inb(gi, gj)
        d = np.where(ok, dist_static[np.clip(gi, 0, SH[0] - 1), np.clip(gj, 0, SH[1] - 1)], 2.0)
        p = Z_HIT * np.exp(-d ** 2 / (2 * SIG ** 2)) + Z_RAND / 10.0
        return np.sum(np.log(p), axis=1), p.mean(axis=1)
    class AMCL:
        def __init__(s, init, n=600):
            s.n = n; s.parts = init + r.standard_normal((n, 3)) * [0.05, 0.05, 0.03]; s.w = np.ones(n) / n; s.ws = s.wf = None; s.acc = np.zeros(3)
        def reinit(s): s.parts = rparts(4000); s.w = np.ones(4000) / 4000; s.ws = s.wf = None
        def predict(s, delta):
            k = len(s.parts); d = np.hypot(delta[0], delta[1])
            nd = delta + r.standard_normal((k, 3)) * np.array([0.005 + 0.1 * d, 0.005 + 0.1 * d, 0.003 + 0.2 * abs(delta[2])])
            c, sn = np.cos(s.parts[:, 2]), np.sin(s.parts[:, 2])
            s.parts[:, 0] += c * nd[:, 0] - sn * nd[:, 1]; s.parts[:, 1] += sn * nd[:, 0] + c * nd[:, 1]; s.parts[:, 2] = wrap(s.parts[:, 2] + nd[:, 2])
            s.acc += np.abs([d, 0, delta[2]])
        def update(s, lx, ly, force=False):
            if not force and s.acc[0] < UPD_D and s.acc[2] < UPD_A: return False
            s.acc[:] = 0; ll, pm = lik(s.parts, lx, ly); wavg = float(np.sum(s.w * pm))
            s.w = s.w * np.exp((ll - ll.max()) / len(lx) * 8); s.w /= s.w.sum()
            if s.ws is None: s.ws = s.wf = wavg
            s.ws += A_SLOW * (wavg - s.ws); s.wf += A_FAST * (wavg - s.wf)
            if 1 / np.sum(s.w ** 2) < len(s.parts) / 2 or len(s.parts) > s.n:
                big = len(s.parts) > s.n; prand = 0.0 if big else max(0.0, 1 - s.wf / s.ws); nr = int(prand * s.n)
                m = max(s.n, len(s.parts) // 2) if big else s.n; idx = r.choice(len(s.parts), size=m - nr, p=s.w)
                s.parts = np.vstack([s.parts[idx], rparts(nr)]) if nr > 0 else s.parts[idx]; s.w = np.ones(len(s.parts)) / len(s.parts)
            return True
        def pose(s):
            b = s.parts[np.argmax(s.w)]; m = np.hypot(s.parts[:, 0] - b[0], s.parts[:, 1] - b[1]) < 0.3; ww = s.w[m]; P = s.parts[m]
            return np.array([np.sum(ww * P[:, 0]) / ww.sum(), np.sum(ww * P[:, 1]) / ww.sum(),
                             np.arctan2(np.sum(ww * np.sin(P[:, 2])), np.sum(ww * np.cos(P[:, 2])))]), np.std(s.parts[:, :2], axis=0).max()
    def fit_of(p, lx, ly):
        c_, s_ = np.cos(p[2]), np.sin(p[2]); gi_, gj_ = to_g(p[0] + c_ * lx - s_ * ly, p[1] + s_ * lx + c_ * ly); okk = inb(gi_, gj_)
        dd = np.full(len(lx), 9.0); dd[okk] = dist_static[gi_[okk], gj_[okk]]; return dd, gi_, gj_, okk
    steps_ray = np.arange(0.1, 10.0, 0.025)
    def beam_check(p, lx, ly, sub=8):
        # beam model: unmapped obstacles only make ranges SHORTER; seeing "through" mapped walls means the pose is wrong
        rx = lx[::sub] - LASER[0]; ry = ly[::sub] - LASER[1]; meas = np.hypot(rx, ry); a = np.arctan2(ry, rx)
        lp = compose(p, LASER); wa = lp[2] + a
        xs = lp[0] + np.cos(wa)[:, None] * steps_ray[None, :]; ys = lp[1] + np.sin(wa)[:, None] * steps_ray[None, :]
        gi, gj = to_g(xs, ys); ok = inb(gi, gj); hit = ~ok
        hit[ok] = occ[gi[ok], gj[ok]]
        exp = np.where(hit.any(axis=1), steps_ray[np.argmax(hit, axis=1)], 10.0)
        # tolerance grows with range: grazing rays on a gridded wall hit its edge early
        return float((meas > exp + np.maximum(0.3, 0.12 * exp)).mean()), float((np.abs(meas - exp) < np.maximum(0.15, 0.06 * exp)).mean())
    def beam_pred(p, lx, ly):
        rx = lx - LASER[0]; ry = ly - LASER[1]; meas = np.hypot(rx, ry); a = np.arctan2(ry, rx)
        lp = compose(p, LASER); wa = lp[2] + a
        xs = lp[0] + np.cos(wa)[:, None] * steps_ray[None, :]; ys = lp[1] + np.sin(wa)[:, None] * steps_ray[None, :]
        gi, gj = to_g(xs, ys); ok = inb(gi, gj); hit = ~ok; hit[ok] = occ[gi[ok], gj[ok]]
        exp = np.where(hit.any(axis=1), steps_ray[np.argmax(hit, axis=1)], 10.0)
        return meas, exp, np.maximum(0.3, 0.12 * exp)
    def costmap(extra):
        d = ndimage.distance_transform_edt(~(occ | extra)) * res
        return np.where(d <= ROBOT_R, 254, np.where(d < INFL, (252 * np.exp(-CSF * (d - ROBOT_R))).astype(int), 0))
    # ---- global search over all free poses (sparse hypothesis sampling), returns every pose that explains the scan ----
    fc = np.argwhere((dist_static > 0.25) & inside); fc = fc[(fc[:, 0] % 3 == 0) & (fc[:, 1] % 3 == 0)]
    cand_xy = np.c_[(fc[:, 1] - off) * res + res / 2, (fc[:, 0] - off) * res + res / 2]
    HEADS = np.linspace(-np.pi, np.pi, 36, endpoint=False)
    PALL = np.c_[np.repeat(cand_xy, len(HEADS), axis=0), np.tile(HEADS, len(cand_xy))]
    def lf_many(P, lx, ly):
        c, s = np.cos(P[:, 2])[:, None], np.sin(P[:, 2])[:, None]
        gi, gj = to_g(P[:, 0:1] + c * lx - s * ly, P[:, 1:2] + s * lx + c * ly)
        d = dist_static[np.clip(gi, 0, SH[0] - 1), np.clip(gj, 0, SH[1] - 1)]
        return np.exp(-d ** 2 / (2 * 0.1 ** 2)).mean(axis=1)
    def global_search(lx, ly):
        sub = max(1, len(lx) // 60); slx, sly = lx[::sub], ly[::sub]
        sc = np.concatenate([lf_many(PALL[i:i + 20000], slx, sly) for i in range(0, len(PALL), 20000)])
        order = np.argsort(-sc); reps = []
        for k in order[:5000]:
            if sc[k] < 0.5 * sc[order[0]]: break
            if all(np.hypot(*(PALL[k, :2] - q[:2])) > 0.6 for q in reps): reps.append(PALL[k].copy())
            if len(reps) >= 24: break
        good = []
        for q in reps:
            q1, _ = csm(MAPb, slx, sly, q, xy=0.2, nxy=9, ang=8, nang=9)
            dd, _, _, _ = fit_of(q1, lx, ly); f = dd < 0.15
            q2 = csm(MAPb, lx[f], ly[f], q1)[0] if f.sum() > 60 else q1
            lf, mf = beam_check(q2, lx, ly, sub=2)
            if lf < 0.15 and mf > 0.4 and all(np.hypot(*(q2[:2] - g[0][:2])) > 0.5 for g in good): good.append((q2, lf, mf))
        return sorted(good, key=lambda g: g[1] - g[2])
    def pursuit(p, wp, idx):
        dists = [np.hypot(a - p[0], b - p[1]) for a, b in wp[idx:idx + 60]]; idx += int(np.argmin(dists)); la = idx
        while la < len(wp) - 1 and np.hypot(wp[la][0] - p[0], wp[la][1] - p[1]) < 0.3: la += 1
        ae = wrap(np.arctan2(wp[la][1] - p[1], wp[la][0] - p[0]) - p[2]); dn = dist_static[to_gc(p[0], p[1])]
        vmax = 0.3 if dn > 0.45 else max(0.1, 0.3 * (dn - 0.18) / 0.27)
        return (vmax if abs(ae) < 0.5 else 0.0), float(np.clip(2.0 * ae, -1.2, 1.2)), idx
    def plan_from(p, goal, extra):
        cost = costmap(extra)
        s = tuple(int(v) for v in to_gc(p[0], p[1])); g = tuple(int(v) for v in to_gc(*goal))
        if cost[g] >= 253: return None, 'goal_blocked'
        cost = cost.copy(); sl = (slice(max(0, s[0] - 2), s[0] + 3), slice(max(0, s[1] - 2), s[1] + 3)); cost[sl] = np.minimum(cost[sl], 200)
        path = astar(cost, s, g)
        return (None, 'no_path') if path is None else ([((j - off) * res, (i - off) * res) for i, j in path], 'ok')
    if HOOK.get('stop'):
        HOOK.update(locals()); return None

    box = BOXES[seed % 3]; sim.world = list(SEGS) + [(box[i], box[(i + 1) % 4]) for i in range(4)]; TDn = tdist(sim.world)
    mission, kid_leg = make_mission(r); kid_after = r.uniform(1.0, 6.0)
    while True:
        c = free_cells[r.integers(len(free_cells))]
        if TDn[c[0], c[1]] > 0.45: break
    kid_pose = np.array([(c[1] - off) * res, (c[0] - off) * res, r.uniform(-np.pi, np.pi)])
    truth = np.array([1.0, 1.0, 0.0]); fus = Fusion(); op = fus.p.copy(); amcl = AMCL(truth.copy()); est = truth.copy()
    live = np.zeros(SH, bool); tsim = 0; lerr = []; clear = []; contacts = Counter(); low = 0; results = []
    kid = False; kid_t = 0; det = None; reloc_events = []; false_lost = 0
    def rec(ph):
        cl = TDn[to_gc(truth[0], truth[1])]; clear.append(cl)
        if cl < ROBOT_R: contacts[ph] += 1
    def health(p, lx, ly, dd):
        if lost_mode == 'fit':
            fr = (dd < 0.1).mean(); return fr < 0.45, fr > 0.6
        lf, mf = beam_check(p, lx, ly); return lf > 0.25, lf < 0.1
    need_help = False; lx, ly = sim.scan(truth)
    for gidx, (name, goal) in enumerate(mission):
        if need_help:
            results.append((name, "NOT_RUN", round(float(np.hypot(*(truth[:2] - np.array(goal)))), 2))); continue
        state = "PLAN"; fails = 0; t_goal = 0; rec_i = 0; n_loc = 0
        while state not in ("DONE", "ABORT"):
            if state == "LOCALIZE" and lost_mode in ('safe', 'spin'):
                n_loc += 1
                if n_loc > 3: state = "ABORT"; break
                t0 = tsim; hyps = []
                for attempt in range(5):
                    lx, ly = sim.scan(truth); hyps = global_search(lx, ly)
                    if hyps: break
                    for _ in range(5):   # turn a little and look again
                        dl, dr, gy = sim.drive(truth, 0.0, 0.8); tsim += dt; t_goal += dt
                        o = fus.step(dl, dr, gy); op = o.copy(); rec('localize')
                live[:] = False; low = 0
                if not hyps:
                    reloc_events.append(dict(t=round(tsim, 1), result='not_found', took=round(tsim - t0, 1))); need_help = True; state = "ABORT"; continue
                if len(hyps) == 1:
                    est = hyps[0][0].copy(); amcl = AMCL(est.copy())
                    e = float(np.hypot(*(est[:2] - truth[:2])))
                    reloc_events.append(dict(t=round(tsim, 1), result='unique', n_hyp=1, err=round(e, 2), took=round(tsim - t0, 1))); state = "PLAN"; continue
                # several poses explain the scan equally well: track all of them while moving (multi-hypothesis)
                mh = [h[0].copy() for h in hyps]; dead = [False] * len(mh); streak = [0] * len(mh); t_m = 0.0; cur = -1; accd = 0.0; acca = 0.0
                wp = None; idx = 0; resolved = None; n_hyp0 = len(mh)
                while t_m < (15.0 if lost_mode == 'spin' else 45.0):
                    bi = next(k for k in range(len(mh)) if not dead[k])   # any surviving hypothesis; in a symmetric map their plans are physically the same
                    if bi != cur:
                        wp, _ = plan_from(mh[bi], goal, np.zeros(SH, bool)); idx = 0; cur = bi
                    if lost_mode == 'spin': v, w = 0.0, 0.6       # do not drive while the pose is ambiguous: turn in place only
                    elif wp: v, w, idx = pursuit(mh[bi], wp, idx)
                    else: v, w = 0.0, 0.6
                    front = (lx > 0) & (np.abs(ly) < 0.22) & (lx < ROBOT_R + 0.3)   # raw-scan collision monitor
                    if front.sum() > 2: v = 0.0; w = 0.8
                    dl, dr, gy = sim.drive(truth, v, w); tsim += dt; t_goal += dt; t_m += dt
                    o = fus.step(dl, dr, gy); delta = compose(inv(op), o); op = o.copy()
                    mh = [compose(h, delta) for h in mh]; accd += np.hypot(delta[0], delta[1]); acca += abs(delta[2])
                    lx, ly = sim.scan(truth); rec('multi')
                    if accd > 0.15 or acca > 0.15:
                        accd = acca = 0.0
                        alive = [k for k in range(len(mh)) if not dead[k]]
                        pred = {}
                        for k in alive:
                            dd, _, _, _ = fit_of(mh[k], lx, ly); f = dd < 0.15
                            if f.sum() > 60:
                                q = csm(MAPb, lx[f], ly[f], mh[k])[0]
                                mh[k] = csm(MAPb, lx[f], ly[f], q, xy=0.03, nxy=7, ang=1.0, nang=5)[0]
                            pred[k] = beam_pred(mh[k], lx, ly)
                        nlong = {k: int((pred[k][0] > pred[k][1] + pred[k][2]).sum()) for k in alive}
                        b = min(alive, key=lambda k: nlong[k]); mb, eb, tb = pred[b]
                        for k in alive:
                            if k == b: continue
                            mk, ek, tk = pred[k]
                            # decisive beam: the two hypotheses predict clearly different ranges, b explains it, k sees through a wall
                            dec = int(((np.abs(ek - eb) > 0.4) & (np.abs(mb - eb) < tb) & (mk > ek + tk)).sum())
                            streak[k] = streak[k] + 1 if dec >= 6 else 0
                            if streak[k] >= 3: dead[k] = True
                        if HOOK.get('trace'):
                            print('  t_m', round(t_m, 1), [(k, round(float(np.hypot(*(mh[k][:2] - truth[:2]))), 2), nlong[k], streak[k]) for k in alive], flush=True)
                        alive = [k for k in range(len(mh)) if not dead[k]]
                        if len(alive) == 1: resolved = alive[0]; break
                if resolved is not None:
                    est = mh[resolved].copy(); amcl = AMCL(est.copy()); live[:] = False
                    e = float(np.hypot(*(est[:2] - truth[:2])))
                    reloc_events.append(dict(t=round(tsim, 1), result='resolved_by_moving', n_hyp=n_hyp0, err=round(e, 2), took=round(tsim - t0, 1)))
                    state = "PLAN"
                else:
                    e_all = [round(float(np.hypot(*(mh[k][:2] - truth[:2]))), 2) for k in range(len(mh)) if not dead[k]]
                    reloc_events.append(dict(t=round(tsim, 1), result='ambiguous_stopped', n_hyp=n_hyp0, n_alive=len(e_all), truth_kept=bool(min(e_all) < 0.3), hyp_errs=e_all, took=round(tsim - t0, 1)))
                    need_help = True; state = "ABORT"
                continue
            if state == "LOCALIZE":
                n_loc += 1
                if n_loc > 3: state = "ABORT"; break
                amcl.reinit(); ok_loc = False; t0 = tsim
                for k in range(140):
                    dl, dr, gy = sim.drive(truth, 0.0, 0.8); tsim += dt; t_goal += dt
                    o = fus.step(dl, dr, gy); amcl.predict(compose(inv(op), o)); op = o.copy()
                    lx, ly = sim.scan(truth); amcl.update(lx[::12], ly[::12], force=True); rec('localize')
                    p, spr = amcl.pose()
                    if spr < 0.15 and k > 8:
                        dd, _, _, _ = fit_of(p, lx, ly); f = dd < 0.15; q = csm(MAPb, lx[f], ly[f], p)[0] if f.sum() > 60 else p
                        dd, _, _, _ = fit_of(q, lx, ly)
                        if lost_mode == 'fit': acc = (dd < 0.1).mean() > 0.6
                        else:
                            lf, mf = beam_check(q, lx, ly); acc = lf < 0.1 and mf > 0.5
                        if acc: est = q; ok_loc = True; break
                live[:] = False; low = 0
                e = float(np.hypot(*(est[:2] - truth[:2]))); em = float(np.hypot(est[0] - (W - truth[0]), est[1] - (H - truth[1])))
                reloc_events.append(dict(t=round(tsim, 1), ok=ok_loc, err=round(e, 2), mirrored=bool(e > 0.5 and em < 0.4), took=round(tsim - t0, 1)))
                state = "PLAN" if ok_loc else "ABORT"; continue
            if state == "PLAN":
                cost = costmap(ndimage.binary_dilation(live, iterations=1))
                s = tuple(int(v) for v in to_gc(est[0], est[1])); g = tuple(int(v) for v in to_gc(*goal))
                if cost[g] >= 253: state = "ABORT"; break
                cost = cost.copy(); sl = (slice(max(0, s[0] - 2), s[0] + 3), slice(max(0, s[1] - 2), s[1] + 3)); cost[sl] = np.minimum(cost[sl], 200)
                path = astar(cost, s, g)
                if path is None: fails += 1; state = "ABORT" if fails >= 4 else "RECOVER"; continue
                wp = [((j - off) * res, (i - off) * res) for i, j in path]; idx = 0; state = "FOLLOW"; anchor = est[:2].copy(); ta = 0.0
            if state == "RECOVER":
                kind = ["clear", "spin", "backup", "wait"][rec_i % 4]; rec_i += 1; live[:] = False
                for v, w in {"clear": [], "spin": [(0, 1.0)] * 20, "backup": [(-0.1, 0)] * 10, "wait": [(0, 0)] * 14}[kind]:
                    dl, dr, gy = sim.drive(truth, v, w); tsim += dt; t_goal += dt
                    o = fus.step(dl, dr, gy); amcl.predict(compose(inv(op), o)); op = o.copy()
                    lx, ly = sim.scan(truth); amcl.update(lx[::12], ly[::12]); rec('recover')
                est, _ = amcl.pose(); state = "PLAN"; continue
            dists = [np.hypot(a - est[0], b - est[1]) for a, b in wp[idx:idx + 60]]; idx += int(np.argmin(dists)); la = idx
            while la < len(wp) - 1 and np.hypot(wp[la][0] - est[0], wp[la][1] - est[1]) < 0.3: la += 1
            ae = wrap(np.arctan2(wp[la][1] - est[1], wp[la][0] - est[0]) - est[2]); dn = dist_static[to_gc(est[0], est[1])]
            vmax = 0.3 if dn > 0.45 else max(0.1, 0.3 * (dn - 0.18) / 0.27); v = vmax if abs(ae) < 0.5 else 0.0; w = np.clip(2.0 * ae, -1.2, 1.2)
            dl, dr, gy = sim.drive(truth, v, w); tsim += dt; t_goal += dt; ta += dt
            if gidx == kid_leg and not kid and t_goal > kid_after: truth[:] = kid_pose; kid = True; kid_t = tsim
            o = fus.step(dl, dr, gy); delta = compose(inv(op), o); op = o.copy(); amcl.predict(delta)
            lx, ly = sim.scan(truth)
            if amcl.update(lx[::12], ly[::12]):
                p, _ = amcl.pose(); dd, _, _, _ = fit_of(p, lx, ly); f = dd < 0.15; est = csm(MAPb, lx[f], ly[f], p)[0] if f.sum() > 60 else p
            else: est = compose(est, delta)
            rec('follow')
            dd, gi_, gj_, okk = fit_of(est, lx, ly); bad, healthy = health(est, lx, ly, dd); low = low + 1 if bad else 0
            if low >= 3:
                e_now = np.hypot(*(est[:2] - truth[:2]))
                if e_now < 0.3: false_lost += 1
                if kid and det is None: det = round(tsim - kid_t, 2)
                state = "LOCALIZE"; continue
            lerr.append(np.hypot(*(est[:2] - truth[:2])))
            near = (np.hypot(lx, ly) < 1.5) & (dd > 0.12) & okk
            if near.sum() > 3 and healthy:
                live[gi_[near], gj_[near]] = True; blk = ndimage.binary_dilation(live, iterations=4)
                if any(blk[i, j] for i, j in path[idx:idx + 40]): state = "PLAN"; continue
            if np.hypot(est[0] - goal[0], est[1] - goal[1]) < 0.15: state = "DONE"; break
            if np.hypot(*(est[:2] - anchor)) > REQ_R: anchor = est[:2].copy(); ta = 0.0
            elif ta > REQ_T: fails += 1; state = "ABORT" if fails >= 4 else "RECOVER"
            if t_goal > 150: state = "ABORT"
        leg = (name, state, round(float(np.hypot(*(truth[:2] - np.array(goal)))), 2))
        results.append(leg); PROGRESS.append(dict(leg=leg, t=round(tsim, 1), relocs=n_loc))
    reach = [x for x in results if x[0] != "UNREACHABLE"]
    return dict(done=sum(x[1] == "DONE" for x in reach), total=len(reach),
                unreach_ok=[x for x in results if x[0] == "UNREACHABLE"][0][1] == "ABORT",
                err_max=max([x[2] for x in reach if x[1] == "DONE"] or [None]),
                kidnap=kid, detect_s=det, relocs=reloc_events, false_lost=false_lost, need_help=need_help,
                contacts=dict(contacts), contact_s=round(sum(contacts.values()) * dt, 1), min_clear=round(float(min(clear)), 3),
                loc_med=round(float(np.median(lerr)), 3), loc_p99=round(float(np.percentile(lerr, 99)), 3), sim_time=round(tsim, 1),
                failed=[x for x in reach if x[1] != "DONE"])

def run_seed(world, seed, lost_mode):
    setup(world); t0 = time.time(); sim = Sim(seed)
    MAP, occ, sres = run_slam(sim)
    m = run_mission(sim, MAP, occ, seed, lost_mode)
    return dict(world=world, mode=lost_mode, seed=seed, gyro_bias=round(sim.gb, 4), slip=round(sim.slip, 3), drop=round(sim.drop, 3),
                scale=round(sim.scale, 3), slam=sres, mission=m, sec=round(time.time() - t0))

if __name__ == "__main__":
    world, mode = sys.argv[1], sys.argv[2]
    def handler(sig, frm): raise TimeoutError()
    signal.signal(signal.SIGALRM, handler)
    for sd in [int(x) for x in sys.argv[3:]]:
        PROGRESS.clear(); signal.alarm(300)
        try:
            print(json.dumps(run_seed(world, sd, mode), default=str), flush=True)
        except TimeoutError:
            print(json.dumps(dict(world=world, mode=mode, seed=sd, timeout=True, progress=PROGRESS), default=str), flush=True)
        signal.alarm(0)
