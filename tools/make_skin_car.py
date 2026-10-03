#!/usr/bin/env python3
"""Writes assets/vehicles/shell_car/fem_<name>.truck: a Rigs of Rods mod's car rebuilt as a FEM car under its own
graphics, as the Shell Car M3 - the physics a shell of FEM triangles on frames of FEM members with the Shell Car's
suspension, the look the mod's meshes as flexbodies skinned to the shell's nodes.

The shell is wrapped round the mod's meshes: at stations along the car a ring of nodes where rays from the section's
middle leave the body (the outermost hit), skin triangles between the rings, a cap at each end; a ring frame of members
at every other station and at the axles, six longitudinal members (sills, waist, roof rails), and a sheet body over the
triangles (not drawn: the cab material's `hidden`). The suspension is the Shell Car's (double wishbones of tube on a
subframe at each axle, coil-overs, a rack, toe links) scaled to the mod's wheels and tied into the ring frames; ring
tyres with the mod's rims. The mod's engine, brakes and mass are kept.

    BL_EXPORT_FLEX=<dir> ./build/beamlab --scene proving --vehicle <folder>/<truck> --frames 2 --hidden --screenshot x.png
    python3 tools/make_skin_car.py <name> <folder>/<truck> <dir> ["Title"]

Coordinates are Rigs of Rods': -x forward, y up, +z left.
"""
import math
import os
import re
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAME, SRC, DIR = sys.argv[1], sys.argv[2], sys.argv[3]
SRC_DIR, SRC_FILE = os.path.join(ROOT, "assets", "vehicles", os.path.dirname(SRC)), os.path.join(ROOT, "assets", "vehicles", SRC + ".truck")
OUT = os.environ.get("SC_OUT") or os.path.join(ROOT, "assets", "vehicles", "shell_car", "fem_%s.truck" % NAME)
TITLE = sys.argv[4] if len(sys.argv) > 4 else "FEM " + NAME

# ------------------------------------------------------------------------------------------------ the mod as exported
flex, props, wheels_src, MASS = [], [], [], 1500.0
for line in open(os.path.join(DIR, "vehicle.txt")):
    t = line.split()
    if t[0] in ("flex", "prop"):
        rec = {"i": int(t[1]), "line": int(t[2]), "mesh": t[3], "pos": np.array(t[4:7], float), "M": np.array(t[7:16], float).reshape(3, 3).T}
        (flex if t[0] == "flex" else props).append(rec)
    elif t[0] == "wheel":
        a, b = np.array(t[1:4], float), np.array(t[4:7], float)
        wheels_src.append({"c": (a + b) / 2, "half": abs(b[2] - a[2]) / 2, "R": float(t[7]), "rim": float(t[8]), "w": float(t[9]), "drive": int(t[10]), "brake": int(t[11]),
                           "side": t[12], "mesh": t[13]})
    elif t[0] == "mass":
        MASS = float(t[1])
SRC_LINES = open(SRC_FILE, encoding="latin-1").read().split("\n")


def load_obj(path):
    V, F = [], []
    for l in open(path):
        if l.startswith("v "):
            V.append([float(x) for x in l.split()[1:4]])
        elif l.startswith("f "):
            F.append([int(x.split("/")[0]) - 1 for x in l.split()[1:4]])
    return np.array(V), np.array(F, dtype=int)


# (what is not of the body: the wheels' and the suspension's meshes - the ring tyres and the FEM suspension take their place)
SKIP = ("tire", "tyre", "wheel", "rim_", "shock", "spring", "leaf", "damper", "arm", "axle", "knuckle", "link", "hydro", "cv_", "ca_", "disc", "disk", "brake", "caliper",
        "pitman", "actuator", "extsteering", "steeringrod", "driveshaft", "diff", "hub", "_r17", "hitch", "goose")
for f in flex:
    f["V"], f["F"] = load_obj(os.path.join(DIR, "%d_%s.obj" % (f["i"], f["mesh"])))
    lo, hi = f["V"].min(axis=0), f["V"].max(axis=0)
    f["skip"] = any(k in f["mesh"].lower() for k in SKIP)
    for w in wheels_src:   # (a mesh within a wheel: its tyre, its rim)
        if np.linalg.norm((lo + hi) / 2 - w["c"]) < 0.6 * w["R"] and (hi - lo).max() < 2.6 * w["R"]:
            f["skip"] = True
body = [f for f in flex if not f["skip"]]
assert body, "no body meshes"
TRI = np.concatenate([f["V"][f["F"]] for f in body])     # (n, 3, 3)
T0, E1, E2 = TRI[:, 0], TRI[:, 1] - TRI[:, 0], TRI[:, 2] - TRI[:, 0]
ALLV = np.concatenate([f["V"] for f in body])


def hits(o, d):
    """the distances along the ray o + t d (t > 0) at which it crosses the body's triangles"""
    o, d = np.asarray(o, float), np.asarray(d, float)
    p = np.cross(d, E2)
    det = (E1 * p).sum(axis=1)
    ok = np.abs(det) > 1e-12
    inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
    s = o - T0
    u = (s * p).sum(axis=1) * inv
    q = np.cross(s, E1)
    v = (q * d).sum(axis=1) * inv
    t = (E2 * q).sum(axis=1) * inv
    return np.sort(t[ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6)])


# the ride height: a mod's wheels are drawn at full droop (it settles when spawned); a car's wheel goes up into its arch
# to a hand's gap over the tyre (a truck's stay: its travel is long)
if wheels_src[0]["R"] <= 0.42:
    for w in wheels_src:
        gaps = []
        for zf in (0.8, 0.9, 1.0):
            h = hits((w["c"][0], w["c"][1], w["c"][2] * zf), (0, 1, 0))
            if len(h):
                gaps.append(float(h[0]) - w["R"])
        w["gap"] = min(gaps) if gaps else 0.0
    for ax in {round(float(w["c"][0]), 3) for w in wheels_src}:
        ws_ = [w for w in wheels_src if abs(w["c"][0] - ax) < 0.05]
        gap = float(np.mean([w["gap"] for w in ws_]))
        print("axle at %.2f: %.3f m over the tyre" % (ax, gap))
        if gap > 0.10:
            for w in ws_:
                w["c"] = w["c"] + np.array([0.0, gap - 0.07, 0.0])


# ------------------------------------------------------------------------------------------------ the model
nodes, node_part, load = [], {}, {}
members, fem_tris, shocks, stops, hydros, slides, wheels = [], [], [], [], [], [], []


def node(p, part="body"):
    nodes.append(tuple(float(c) for c in p))
    node_part[len(nodes) - 1] = part
    return len(nodes) - 1


_at = {}


def node_at(p, part="body"):
    key = (round(p[0], 3), round(p[1], 3), round(p[2], 3))
    if key not in _at:
        _at[key] = node(p, part)
    return _at[key]


def member(a, b, sec, ja=None, jb=None):
    if a != b:
        members.append((a, b, sec, ja, jb))


def dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


# ---- the shell: rings of nodes round the body at stations along it
lo, hi = np.percentile(ALLV, 0.2, axis=0), np.percentile(ALLV, 99.8, axis=0)
x0, x1 = lo[0], hi[0]
LEN = x1 - x0
axles = sorted({round(float(w["c"][0]), 3) for w in wheels_src})
AX_F, AX_R = axles[0], axles[-1]
n_st = int(round((LEN - 0.36) / 0.44)) + 1
xs = list(np.linspace(x0 + 0.18, x1 - 0.18, n_st))
for ax in (AX_F, AX_R):   # (a station at each axle)
    k = min(range(len(xs)), key=lambda i: abs(xs[i] - ax))
    if 0 < k < len(xs) - 1:
        xs[k] = ax
NA = 14
ANG = [2 * math.pi * (k + 0.5) / NA for k in range(NA)]   # (from +z round over the top: symmetric side to side)
rings, centres = [], []
for x in xs:
    # the section's middle: between the body's top and bottom on the centre line
    ys = []
    for z in (0.0, 0.25, -0.25):
        up, dn = hits((x, lo[1] - 1.0, z), (0, 1, 0)), hits((x, hi[1] + 1.0, z), (0, -1, 0))
        if len(up) and len(dn):
            ys += [lo[1] - 1.0 + up[0], hi[1] + 1.0 - dn[0]]
    yb, yt = (min(ys), max(ys)) if ys else (lo[1], hi[1])
    yc = 0.5 * (yb + yt)
    for again in range(3):   # (the rays from the middle of what they find: a nose's section is not where its centre line's is)
        r = []
        for a in ANG:
            h = hits((x, yc, 0.0), (0.0, math.sin(a), math.cos(a)))
            r.append(float(h[-1]) if len(h) else float("nan"))
        ys = [yc + math.sin(a) * v for a, v in zip(ANG, r) if not math.isnan(v)]
        if again == 2 or len(ys) < 4 or abs(0.5 * (min(ys) + max(ys)) - yc) < 0.02:
            break
        yc = 0.5 * (min(ys) + max(ys))
    rings.append(r), centres.append((x, yc))
R = np.array(rings)
R[R < 0.08] = float("nan")   # (a ray that stops at its own start, in an opening: its neighbours' instead; no triangle of no area)
for i in range(len(xs)):   # (side to side alike; a ray through an opening takes its neighbours')
    for k in range(NA):
        mk = (NA // 2 - 1 - k) % NA   # (the angle mirrored in the vertical plane: pi - a)
        vals = [v for v in (R[i, k], R[i, mk]) if not math.isnan(v)]
        if vals:
            R[i, k] = R[i, mk] = max(vals)
for _ in range(3):
    for i in range(len(xs)):
        for k in range(NA):
            if math.isnan(R[i, k]):
                nb = [R[i, (k - 1) % NA], R[i, (k + 1) % NA]] + ([R[i - 1, k]] if i > 0 else []) + ([R[i + 1, k]] if i + 1 < len(xs) else [])
                nb = [v for v in nb if not math.isnan(v)]
                if nb:
                    R[i, k] = float(np.mean(nb))
R = np.nan_to_num(R, nan=0.5)
for i in range(len(xs)):   # (a mirror, an aerial: no spike in the shell)
    for k in range(NA):
        nb = [R[i, (k - 1) % NA], R[i, (k + 1) % NA]] + ([R[i - 1, k]] if i > 0 else []) + ([R[i + 1, k]] if i + 1 < len(xs) else [])
        R[i, k] = min(R[i, k], 1.3 * float(np.median(nb)) + 0.05)
RING = []
for i, (x, yc) in enumerate(centres):
    RING.append([node((x, yc + math.sin(a) * (R[i, k] - 0.012), math.cos(a) * (R[i, k] - 0.012))) for k, a in enumerate(ANG)])
for i in range(len(xs) - 1):
    for k in range(NA):
        a, b, c, d = RING[i][k], RING[i][(k + 1) % NA], RING[i + 1][(k + 1) % NA], RING[i + 1][k]
        fem_tris.extend([(a, b, c), (a, c, d)] if (i + k) % 2 else [(a, b, d), (b, c, d)])
for end, i in ((-1, 0), (1, len(xs) - 1)):   # the caps: a node on the nose and on the tail
    x, yc = centres[i]
    h = hits((x, yc, 0.0), (end, 0, 0))
    tip = node((x + end * (float(h[-1]) - 0.012 if len(h) else 0.2), yc, 0.0))
    for k in range(NA):
        a, b = RING[i][k], RING[i][(k + 1) % NA]
        fem_tris.append((tip, b, a) if end < 0 else (tip, a, b))

# ---- its frames: a ring of members at every other station and at the axles' stations, longitudinals along six lines
BIG = wheels_src[0]["R"] > 0.42          # (a truck: heavier sections, thicker skin)
SECTIONS = {  # material, shape, outer (m), wall (m), joints, [break force N, joint damping N m s/rad]
    "ring": ("MildSteel", "box", 0.110 if BIG else 0.080, 0.0030 if BIG else 0.0018, "rigid"),
    "rail": ("MildSteel", "box", 0.130 if BIG else 0.100, 0.0035 if BIG else 0.0020, "rigid"),
    "subframe": ("Steel", "box", 0.110 if BIG else 0.080, 0.0035 if BIG else 0.0025, "rigid"),
    "tower": ("Steel", "box", 0.080 if BIG else 0.060, 0.0030 if BIG else 0.0020, "rigid"),
    "arm": ("Maraging", "tube", 0.060 if BIG else 0.045, 0.0070 if BIG else 0.0050, "rigid", 0.0, 3.0),
    "tierod": ("Maraging", "tube", 0.042 if BIG else 0.034, 0.0070 if BIG else 0.0060, "ball"),
    "rack": ("Maraging", "tube", 0.044 if BIG else 0.036, 0.0070 if BIG else 0.0060, "rigid"),
    "upright": ("Maraging", "tube", 0.090 if BIG else 0.070, 0.0180 if BIG else 0.0140, "rigid", 0.0, 3.0),
    "hub": ("Maraging", "tube", 0.075 if BIG else 0.055, 0.0140 if BIG else 0.0100, "rigid", 0.0, 3.0),
}
SKIN_T = 0.0015 if BIG else 0.0012
frame_st = sorted(set(range(0, len(xs), 2)) | {len(xs) - 1} | {i for i, x in enumerate(xs) if x in (AX_F, AX_R)})
for i in frame_st:
    for k in range(NA):
        member(RING[i][k], RING[i][(k + 1) % NA], "ring")
LONG = sorted({min(range(NA), key=lambda k: abs(((ANG[k] - a + math.pi) % (2 * math.pi)) - math.pi)) for a in
               (math.radians(-55), math.radians(235), math.radians(-12), math.radians(192), math.radians(50), math.radians(130))})
for k in LONG:
    for i in range(len(xs) - 1):
        member(RING[i][k], RING[i + 1][k], "rail")
FRAME_NODES = sorted({n for a, b, s_, ja, jb in members for n in (a, b)})


def tie(n, count=3, sec="tower"):
    """a member from n to each of the frames' nodes nearest it"""
    for m in sorted(FRAME_NODES, key=lambda q: dist(nodes[q], nodes[n]))[:count]:
        member(n, m, sec)


# ---- the suspension: the Shell Car's, its hard points off the wheel's centre scaled to the wheel (k: its radius over
# the Shell Car's 0.32 m; across the car by the track, kz)
rack_ends, HOUSING = {}, None
for wx in (AX_F, AX_R):
    front = wx == AX_F
    ws = [w for w in wheels_src if abs(w["c"][0] - wx) < 0.05]
    yw, zc, Rw = float(ws[0]["c"][1]), float(np.mean([abs(w["c"][2]) for w in ws])), ws[0]["R"]
    k, kz = Rw / 0.32, min(1.3, max(0.8, zc / 0.77))
    zp, zu = zc - 0.55 * kz, zc - 0.37 * kz              # the lower and the upper pivots
    xs_piv = (wx + 0.12 * k, wx - 0.14 * k)
    sub = {}
    for x in xs_piv:   # the subframe: a cross member through the lower pivots, a tower each side up to the upper pivot and the coil-over's top
        row = [node_at((x, yw + 0.04 * k, z), "sub") for z in (-zp, 0.0, zp)]
        member(row[0], row[1], "subframe"), member(row[1], row[2], "subframe")
        for s in (1, -1):
            lowp, upp = row[2 if s > 0 else 0], node_at((x, yw + 0.26 * k, s * zu), "sub")
            member(lowp, upp, "tower")
            sub[(x, s)] = (lowp, upp)
    for s in (1, -1):
        top = node_at((wx, yw + 0.48 * k, s * (zc - 0.19 * kz)), "sub")
        (l0, u0), (l1, u1) = sub[(xs_piv[0], s)], sub[(xs_piv[1], s)]
        member(l0, l1, "subframe"), member(u0, u1, "tower"), member(u0, top, "tower"), member(u1, top, "tower"), member(l0, u1, "tower")
        for n in (l0, l1, u0, u1, top):
            tie(n)
    member(node_at((wx, yw + 0.48 * k, zc - 0.19 * kz), "sub"), node_at((wx, yw + 0.48 * k, -(zc - 0.19 * kz)), "sub"), "tower")   # (a brace across the tops)
    RACK_X, RACK_Y = wx + 0.18 * k, yw + 0.033 * k
    if front:
        HOUSING = [node_at((RACK_X, RACK_Y, z), "sub") for z in (-zp - 0.16 * kz, -zp, 0.0, zp, zp + 0.16 * kz)]
        for a, b in zip(HOUSING, HOUSING[1:]):
            member(a, b, "subframe")
        for n in HOUSING:
            tie(n, 2)
        member(HOUSING[1], sub[(xs_piv[0], -1)][0], "subframe"), member(HOUSING[3], sub[(xs_piv[0], 1)][0], "subframe")
    for s in (1, -1):
        w = [q for q in ws if q["c"][2] * s > 0][0]
        arm_x = wx + 0.18 * k if front else wx - 0.18 * k
        bl, bu = node((wx, yw - 0.14 * k, s * (zc - 0.10 * kz)), "susp"), node((wx, yw + 0.24 * k, s * (zc - 0.10 * kz)), "susp")
        n1, n2 = node((wx, yw, s * (zc - 0.15 * kz)), "susp"), node((wx, yw, s * (zc + 0.15 * kz)), "susp")
        sa = node((arm_x, yw - 0.14 * k, s * (zc - 0.08 * kz)), "susp")
        lower = [sub[(x, s)][0] for x in xs_piv]
        upper = [sub[(x, s)][1] for x in xs_piv]
        top = node_at((wx, yw + 0.48 * k, s * (zc - 0.19 * kz)), "sub")
        for p in lower:
            member(p, bl, "arm", ja="ball")
        for p in upper:
            member(p, bu, "arm", ja="ball")
        member(bl, n1, "upright", ja="ball"), member(n1, bu, "upright", jb="ball")
        member(n1, n2, "hub"), member(n1, sa, "hub"), member(bl, sa, "hub", ja="ball")
        shocks.append((bl, top, front, k))
        wheels.append((n1, n2, top, w))
        kp = np.array(nodes[bl])
        r = np.array(nodes[sa]) - kp
        if front:
            re_ = node((RACK_X, RACK_Y, s * 0.12 * kz), "susp")
            ti = node((RACK_X, RACK_Y, s * (zc - 0.561 * kz)), "susp")
            load[re_] = load[ti] = 3.0
            rack_ends[s] = re_
            member(re_, ti, "rack"), member(ti, sa, "tierod")
            slides.append(re_)
            end, lim = HOUSING[-1 if s > 0 else 0], 34.0
        else:
            br = node_at((xs_piv[0], yw + 0.067 * k, s * (zc - 0.565 * kz)), "sub")   # the toe link's bracket
            member(br, lower[0], "tower"), member(br, sub[(xs_piv[0], s)][1], "tower")
            member(sa, br, "arm", ja="ball", jb="ball")
            end, lim = lower[1], 6.0
        L0 = dist(nodes[sa], nodes[end])
        ds = []
        for deg in (-lim, lim):
            c, sn = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            p = kp + np.array((r[0] * c - r[2] * sn, r[1], r[0] * sn + r[2] * c))
            ds.append(float(np.linalg.norm(p - np.array(nodes[end]))))
        stops.append((sa, end, (L0 - min(ds)) / L0, (max(ds) - L0) / L0))
    if front:
        member(rack_ends[1], rack_ends[-1], "rack")
        kf = k
        travel = 0.18 * kf * math.sin(math.radians(30.0))
        hydros.append((rack_ends[1], HOUSING[0], -travel / dist(nodes[rack_ends[1]], nodes[HOUSING[0]])))

# ---- the mass: the mod's, less what the elements and the wheels weigh, on the shell's lower nodes (55% ahead of the middle)
DENS = {"MildSteel": 7850.0, "Steel": 7850.0, "Maraging": 8000.0}


def sec_kg_m(sec):
    mat, shape, outer, wall = SECTIONS[sec][:4]
    area = (outer ** 2 - (outer - 2 * wall) ** 2) if shape == "box" else math.pi / 4 * (outer ** 2 - (outer - 2 * wall) ** 2)
    return area * DENS[mat]


def tri_area(t):
    a, b, c = (np.array(nodes[k]) for k in t)
    return 0.5 * float(np.linalg.norm(np.cross(b - a, c - a)))


# ---- the camera's triple: it sets the car level when it is put down, so along the line under the wheels (a mod's axles
# are not drawn at one height), three nodes of their own tied to the frames
def _under(ax):
    ws_ = [w for w in wheels_src if abs(w["c"][0] - ax) < 0.05]
    return float(np.mean([w["c"][1] - w["R"] for w in ws_])), float(np.mean([w["c"][1] for w in ws_]))
(_bf, _cf), (_br, _cr) = _under(AX_F), _under(AX_R)
_u = np.array([AX_R - AX_F, _br - _bf, 0.0])
_u /= np.linalg.norm(_u)
_c = np.array([0.5 * (AX_F + AX_R), 0.5 * (_cf + _cr), 0.0])
CAMERA = (node(_c, "cam"), node(_c + 0.9 * _u, "cam"), node(_c + np.array([0.0, 0.0, 0.4]), "cam"))
for n in CAMERA:
    tie(n)
member(CAMERA[0], CAMERA[1], "tower"), member(CAMERA[0], CAMERA[2], "tower")

SHEET_KG_M2 = 4.7
ON_SUB = float(os.environ.get("ON_SUB", "0.6"))
WHEEL_KG = 20.0 * (wheels_src[0]["R"] / 0.32) ** 2
fem_kg = sum(tri_area(t) for t in fem_tris) * (SKIN_T * 7850.0 + SHEET_KG_M2) + sum(dist(nodes[a], nodes[b]) * sec_kg_m(s_) for a, b, s_, ja, jb in members)
rest = max(100.0, MASS - fem_kg - 4 * WHEEL_KG)
low = [n for i in range(len(xs)) for k_, n in enumerate(RING[i]) if math.sin(ANG[k_]) < -0.3]
xm = 0.5 * (AX_F + AX_R)
subs = [n for n in range(len(nodes)) if node_part.get(n) == "sub"]
# (the engine, the gearbox, the axles and the tank: on the subframes, straight over the suspension; the rest on the floor)
for part, share in ((True, 0.55), (False, 0.45)):
    for group, frac in ((low, 1.0 - ON_SUB), (subs, ON_SUB)):
        ns = [n for n in group if (nodes[n][0] < xm) == part]
        for n in ns:
            load[n] = load.get(n, 0.0) + rest * share * frac / len(ns)

# ------------------------------------------------------------------------------------------------ the mod's own lines
def section_lines(name, count=1):
    """the first data lines of a section of the mod's truck file"""
    out, on = [], False
    for l in SRC_LINES:
        t = l.strip()
        if t == name:
            on = True
            continue
        if on:
            if not t or t.startswith(";"):
                continue
            if re.fullmatch(r"[a-z_0-9]+", t) and not t[0].isdigit():
                break
            out.append(t)
            if len(out) >= count:
                break
    return out


def placement(pref, px, py):
    pref, X, Y = np.array(pref), np.array(px) - np.array(pref), np.array(py) - np.array(pref)
    n = np.cross(Y, X)
    n /= np.linalg.norm(n)
    rx = X / np.linalg.norm(X)
    return pref, np.column_stack([X, Y, n]), np.column_stack([rx, n, np.cross(rx, n)])


def frame_of(ns):
    """three of the nodes ns for a frame: two far apart, the third far off their line"""
    P = {k: np.array(nodes[k]) for k in ns}
    a = min(ns, key=lambda k: tuple(P[k]))
    b = max(ns, key=lambda k: np.linalg.norm(P[k] - P[a]))
    e = (P[b] - P[a]) / np.linalg.norm(P[b] - P[a])
    c = max(ns, key=lambda k: np.linalg.norm((P[k] - P[a]) - e * np.dot(P[k] - P[a], e)))
    return a, b, c


def replace(pos, M, own):
    """the offset and the rotation (the truck format's: Rz Ry Rx, degrees) that put a mesh at pos, M on the nodes own"""
    po, Bo, Ro = placement(*(nodes[k] for k in own))
    Rm = Ro.T @ M
    ry = -math.asin(max(-1.0, min(1.0, Rm[2, 0])))
    return np.linalg.solve(Bo, pos - po), [math.degrees(a) for a in (math.atan2(Rm[2, 1], Rm[2, 2]), ry, math.atan2(Rm[1, 0], Rm[0, 0]))]


def ranges(ns):
    ns, out = sorted(set(ns)), []
    for k in ns:
        if out and out[-1][1] == k - 1:
            out[-1][1] = k
        else:
            out.append([k, k])
    return ",".join("%d-%d" % (a, b) if b > a else "%d" % a for a, b in out)


BODY_NODES = [n for n, g in node_part.items() if g == "body"]
body_frame = frame_of(BODY_NODES)
flex_out = []
for f in body:
    o, r = replace(f["pos"], f["M"], body_frame)
    flex_out.append("%d, %d, %d, %.5f, %.5f, %.5f, %.4f, %.4f, %.4f, %s\nforset %s" % (body_frame + tuple(o) + tuple(r) + (f["mesh"], ranges(BODY_NODES))))
prop_out = []
for p in props:
    name = p["mesh"].lower()
    if any(k in name for k in SKIP) or any(np.linalg.norm(p["pos"] - w["c"]) < 0.9 * w["R"] for w in wheels_src):
        continue
    src = SRC_LINES[p["line"] - 1].strip() if 0 < p["line"] <= len(SRC_LINES) else ""
    tok = [t.strip() for t in src.split(",")]
    if len(tok) < 10:
        continue
    near = sorted(BODY_NODES, key=lambda n: dist(nodes[n], p["pos"]))[:6]
    own = frame_of(near)
    o, r = replace(p["pos"], p["M"], own)
    prop_out.append("%d, %d, %d, %.5f, %.5f, %.5f, %.4f, %.4f, %.4f, %s" % (own + tuple(o) + tuple(r) + (",".join(tok[9:]),)))
    j = p["line"]   # (its animations: the lines after it)
    while j < len(SRC_LINES) and SRC_LINES[j].strip().startswith("add_animation"):
        prop_out.append(SRC_LINES[j].strip())
        j += 1
# a wheel's rim: the mod's wheel mesh, or a prop of a wheel standing at it (its own x along the axle)
for n1, n2, top, w in wheels:
    if w["mesh"] != "-":
        continue
    for p in props:
        if any(k in p["mesh"].lower() for k in ("wheel", "rim")) and "steer" not in p["mesh"].lower() and np.linalg.norm(p["pos"] - w["c"]) < 0.5 * w["R"] and abs(p["M"][2, 0]) > 0.9:
            w["mesh"], w["side"] = p["mesh"], "l" if p["M"][2, 0] > 0 else "r"
            break

# ------------------------------------------------------------------------------------------------ the file
mats, on = [], False
for l in SRC_LINES:
    t = l.strip()
    if t == "managedmaterials":
        on = True
    elif on and t and re.fullmatch(r"[a-z_0-9]+", t) and not t[0].isdigit():
        on = False
    elif on and t and not t.startswith(";") and not t.startswith("set_") and t.split()[0] not in {m.split()[0] for m in mats}:
        mats.append(t)
with open(OUT, "w") as f:
    f.write("%s\n;generated by tools/make_skin_car.py from %s: a shell of FEM triangles wrapped round the mod's meshes on ring frames and\n" % (TITLE, SRC))
    f.write(";longitudinals of FEM members, the Shell Car's suspension scaled to its wheels, ring tyres; its look the mod's meshes as\n")
    f.write(";flexbodies skinned to the shell's nodes (the sheet body and the plates are there and not drawn)\n")
    f.write(";resources: ../%s\n" % os.path.dirname(SRC))
    if mats:
        f.write("managedmaterials\n%s\n" % "\n".join(mats))
    f.write("globals\n10.0, 0.0, sheet/Steel/%.1f/0.004/1/hidden\nminimass\n0.05\n" % SHEET_KG_M2)
    f.write("nodes\n;id, x, y, z, options[, load kg]\n")
    for i, (x, y, z) in enumerate(nodes):
        f.write("%d, %.4f, %.4f, %.4f, nl, %.2f\n" % (i, x, y, z, load[i]) if i in load else "%d, %.4f, %.4f, %.4f, n\n" % (i, x, y, z))
    f.write("beams\n")
    for sec, spec in SECTIONS.items():
        lst = [(a, b, ja, jb) for a, b, s_, ja, jb in members if s_ == sec]
        if not lst:
            continue
        extra = ", 0, %.0f, %.1f" % spec[5:7] if len(spec) > 5 else ""
        f.write(";%s\nset_beam_defaults 3000000, 400, 80000, 700000, 0.05, tracks/beam, 0\nset_frame_section %s, %s, %.4f, %.4f, %s%s\n" % (sec, spec[0], spec[1], spec[2], spec[3], spec[4], extra))
        opt = "F" if sec in ("arm", "tierod", "rack", "upright") else "Fi"   # (the body's and the subframes' members not drawn)
        for a, b, ja, jb in lst:
            f.write("%d, %d, %s, %s, %s\n" % (a, b, opt, ja or spec[4], jb or spec[4]) if ja or jb else "%d, %d, %s\n" % (a, b, opt))
    # the coil-overs: the corner's load on a spring of 1.6 Hz (a truck's 1.4) damped at 0.4 of critical, preloaded to
    # stand at the design length; a bump stop and a droop strap beside each; the steering's and the rear toe's stops
    sprung = MASS - 4 * WHEEL_KG
    f.write("shocks\nset_beam_defaults 9000000, 12000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, front, k in shocks:
        L0 = dist(nodes[a], nodes[b])
        corner = sprung * 9.81 * (0.55 if front else 0.45) / 2
        m = corner / 9.81
        ks = m * (2 * math.pi * (1.4 if BIG else 1.6)) ** 2
        f.write("%d, %d, %.0f, %.0f, 0.5, 0.5, %.3f, n\n" % (a, b, ks, 2 * 0.40 * math.sqrt(ks * m), 1 + corner / (ks * L0)))
    f.write("set_beam_defaults 20000000, 200000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, front, k in shocks:
        L0 = dist(nodes[a], nodes[b])
        f.write("%d, %d, 0, 0, %.3f, %.3f, 1.0, i\n" % (a, b, 0.14 * k / L0, 0.09 * k / L0))
    f.write("set_beam_defaults 3000000, 20000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, lo_, hi_ in stops:
        f.write("%d, %d, 0, 0, %.3f, %.3f, 1.0, i\n" % (a, b, lo_, hi_))
    f.write("hydros\nset_beam_defaults 4000000, 2000, 99999999999999999999999999999999999999999, 99999999999999999999999999999999999999999, 0.02, tracks/beam, 0\n")
    for a, b, fac in hydros:
        f.write("%d, %d, %.4f, i\n" % (a, b, fac))
    f.write("slidenodes\n")
    for n in slides:
        f.write("%d, %s, S2000000, T0\n" % (n, ", ".join(str(h) for h in HOUSING)))
    f.write("ringwheels\n;(BeamLab) radius, rim radius, width, node1, node2, braking, propulsion, arm, mass, tyre stiffness, damping, grip[, side, rim mesh]\n")
    for n1, n2, top, w in wheels:
        rim = w["rim"] if 0.4 * w["R"] < w["rim"] < 0.9 * w["R"] else 0.66 * w["R"]
        corner = MASS * 9.81 / 4
        f.write("%.3f, %.3f, %.3f, %d, %d, 1, %d, %d, %.1f, %.0f, %.0f, 1.0%s\n" % (
            w["R"], rim, min(max(w["w"], 0.18), 0.45), n1, n2, 1 if w["drive"] else 0, top, WHEEL_KG, corner / 0.02, 1000.0 * math.sqrt(MASS / 1250.0),
            ", %s, %s" % (w["side"], w["mesh"]) if w["mesh"] != "-" else ""))
    eng = section_lines("engine") or ["900.0, 6500.0, 280.0, 3.15, 3.7, 1.0, 4.2, 2.5, 1.66, 1.22, 1.0, -1.0"]
    f.write("engine\n%s\n" % eng[0])
    f.write("engoption\n%s\n" % (section_lines("engoption") or ["0.02, c, 1000.0, 0.3, 0.4, 0.3"])[0])
    f.write("brakes\n%s\n" % (section_lines("brakes") or ["4000"])[0])
    f.write("cameras\n%d, %d, %d\n" % CAMERA)
    cine = section_lines("cinecam")
    cp = [float(x) for x in cine[0].split(",")[:3]] if cine else [xm - 0.2, hi[1] - 0.35, 0.35]
    f.write("cinecam\n%.3f, %.3f, %.3f, %s\n" % (cp[0], cp[1], cp[2], ", ".join(str(n) for n in sorted(BODY_NODES, key=lambda n: dist(nodes[n], cp))[:8])))
    f.write("contacters\n" + "".join("%d\n" % i for i in range(len(nodes))))
    f.write("shells\n" + "".join("%d, %d, %d\n" % t for t in fem_tris))
    f.write("fem_tris\nset_fem_shell MildSteel, %.4f\n" % SKIN_T + "".join("%d, %d, %d\n" % t for t in fem_tris))
    f.write("flexbodies\n" + "\n".join(flex_out) + "\n")
    if prop_out:
        f.write("props\n" + "\n".join(prop_out) + "\n")
    f.write("end\n")
print("wrote %s: %d nodes (%d of the shell: %d rings of %d), %d triangles, %d members; %.0f kg (elements %.0f, on the floor %.0f, wheels %.0f); %d meshes, %d props; skipped: %s" % (
    OUT, len(nodes), len(BODY_NODES), len(xs), NA, len(fem_tris), len(members), MASS, fem_kg, rest, 4 * WHEEL_KG, len(body), len([p for p in prop_out if not p.startswith("add_")]),
    ", ".join(f["mesh"].replace(".mesh", "") for f in flex if f["skip"])))
