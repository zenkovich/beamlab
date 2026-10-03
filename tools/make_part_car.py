#!/usr/bin/env python3
"""Writes assets/vehicles/shell_car/fem_<car>.truck: a Rigs of Rods mod's car rebuilt as the Shell Car is - a
body-in-white of FEM triangle sheets and members, the hang-on parts on mounts that let go, collision volumes, the
suspension of FEM tubes on ring tyres - under the mod's own meshes as flexbodies.

The Shell Car's body (tools/make_shell_car.py: the floor, the firewall, the roof, the sills', rails' and roof rails'
sections, the aprons, the wheelhouses, the quarters, the rear panel, the pillars, the radiator support, the bumpers'
beams) is laid out on the BMW E36's lines. Here the same layout is put on another car by a map between the two: the
landmarks of the mod's meshes (the nose, the axles, the door's front and rear edges, the roof's front and rear edges,
the trunk lid's front edge, the tail; the floor, the belt line, the roof; the half width) against the E36's, each axis
piecewise linear between them. The mod's meshes are taken into the E36's space by the map's inverse, the car is built
there as the Shell Car is - what follows the E36's skin there (the quarters, the roof, the body side's line) follows
the mod's - and its nodes go back by the map. Lengths, areas and masses are the mapped ones (dist, area).

The hang-on parts are the mod's own, each mesh scanned into one sheet of FEM triangles (rays on structured grids over
the mesh's extent, the bumpers' round their outlines): the hood (hinges on the cowl, a latch on the radiator support,
buffers, a stay), the front fenders (bolts along the upper rail), the doors (two hinges on the pillar ahead, latches on
the pillar behind, a check strap, hidden bars and, where the mod's door has a window frame, its sash), the trunk lid or
the hatch (hinges, a latch, buffers, a stay), the bumpers (brackets on the beams, clips at the sides that twist off),
the headlights, the tail lights and the grille (bolts). A two-door body has a fixed panel behind its door, a body
without a lid a deck over its trunk.

The suspension is the Shell Car's double wishbones of maraging tube with a rack and tie rods in front and toe links at
the rear, as a unit scaled to the mod's wheel (its radius over 0.32 m, its track) about each wheel's centre, on
subframes of its own tied to the body's nodes near them; coil-overs for the corner's load at 1.6 Hz; ring tyres under
the mod's rims. A mod draws its wheels at full droop: the wheel goes up into its arch to a hand's gap over the tyre.
The mod's engine, gearbox and brakes; its mass, less what the elements weigh, on the front rails, the cabin's and the
trunk's floor.

The mod's meshes placed in its definition space as .obj files with vehicle.txt (their placements, the wheels, the mass):

    BL_EXPORT_FLEX=<dir> ./build/beamlab --scene proving --vehicle <folder>/<truck> --frames 2 --hidden --screenshot x.png
    python3 tools/make_part_car.py <car> <dir>          (the cars: CARS below)

Coordinates are Rigs of Rods': -x forward, y up, +z left.
"""
import itertools
import math
import os
import re
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ------------------------------------------------------------------------------------------------ the cars
# meshes: the mod's meshes of each role (BODY: the fixed body the quarters and the roof follow; DWIN: the door's glass,
# its foot the belt line); forset: (a part of a mesh's name, the groups of nodes it is skinned to), the first that fits,
# the body-in-white's otherwise; skip: meshes left out (the mod's wheels and suspension: the ring tyres and the FEM
# suspension are there instead)
CARS = {
    "mercedes_clk": dict(
        title="FEM Mercedes CLK 55", src="mercedes_clk/CLK_55",
        meshes=dict(BODY=["CLK_208_Chassis2"], HOOD=["CLK_208_Hood2"], FENDER=["CLK_208_Fenders2_CLK55"], FDOOR=["CLK_208_Doors2"], DWIN=["CLK_208_Doorwindows"],
                    TRUNK=["CLK_208_Trunk2"], FBUMP=["CLK_208_FbumperAMG2"], RBUMP=["CLK_208_RbumperAMG2"], GRILLE=["CLK_208_Grill"],
                    HEAD=["CLK_208_LProjectorHeadlight", "CLK_208_RProjectorHeadlight"]),
        forset=[("Headlight", ["headlight 1", "headlight -1"]), ("lens", ["headlight 1", "headlight -1"]), ("Door", ["door 1 front", "door -1 front"]),
                ("Mirror", ["door 1 front", "door -1 front"]), ("Hood", ["hood"]), ("Grill", ["grille", "hood"]), ("MBLogo", ["hood"]), ("Fbumper", ["fascia"]),
                ("Rbumper", ["rear bumper"]), ("Fenders", ["fender 1", "fender -1"]), ("Trunk", ["trunk"]), ("Spoilerlip", ["trunk"])],
        skip=("FrontSubframe",)),
    "audi_quattro": dict(
        title="FEM Audi Quattro", src="audi_quattro/1988_audi_quattro_edited",
        meshes=dict(BODY=["Quattro_Body"], HOOD=["Quattro_Hood"], FENDER=["Quattro_Fenders"], FDOOR=["Quattro_Doors"], DWIN=["Quattro_DoorWind"],
                    TRUNK=["Quattro_Trunk"], FBUMP=["Quattro_FBumper"], RBUMP=["Quattro_RBumper"], GRILLE=["Quattro_Grille"], HEAD=["Quattro_Lights"],
                    TAIL=["Quattro_RLights"]),
        forset=[("RLights", ["lamp 1", "lamp -1"]), ("Lights", ["headlight 1", "headlight -1"]), ("Lens", ["headlight 1", "headlight -1"]),
                ("Door", ["door 1 front", "door -1 front"]), ("Mirror", ["door 1 front", "door -1 front"]), ("Hood", ["hood"]), ("Grille", ["grille"]),
                ("FBumper", ["fascia"]), ("RBumper", ["rear bumper"]), ("Fenders", ["fender 1", "fender -1"]), ("Trunk", ["trunk"]), ("Wing", ["trunk"])],
        skip=("SteeringRod", "FShocks", "RShocks", "CV_", "CA_", "Discs", "RearDiff", "Driveshaft", "Subframe")),
    "viper": dict(
        title="FEM Dodge Viper", src="dodge_viper/1996_dodge_viper_gts_coupe_edited",
        meshes=dict(BODY=["DodgeViperShell", "DodgeViperUnibody"], HOOD=["DodgeViperHood"], FENDER=["DodgeViperHood"], FDOOR=["DodgeViperDoors"],
                    DWIN=["DodgeViperDoorWindows"], FBUMP=["DodgeViperFrontBumper"], RBUMP=["DodgeViperRearBumperE", "DodgeViperShell"],
                    HEAD=["DodgeViperFrontLights"], TAIL=["DodgeViperRearLights"]),
        forset=[("RearLights", ["lamp 1", "lamp -1"]), ("RearLens", ["lamp 1", "lamp -1"]), ("FrontLights", ["headlight 1", "headlight -1"]),
                ("FrontLens", ["headlight 1", "headlight -1"]), ("Door", ["door 1 front", "door -1 front"]), ("Mirror", ["door 1 front", "door -1 front"]),
                ("Hood", ["hood", "fender 1", "fender -1"]), ("FrontBumper", ["fascia"]), ("RearBumper", ["rear bumper"])],
        clamshell=True, fender_x0=0.45, rear_y=(0.24, 0.62), rear_span=65, skip=("Brakes",)),
    # a pickup (style): the cab the saloon's body to its rear door's post (post: the rear door's rear edge; cback: the
    # cab's back at the roof), a ladder frame under it (FRAME: the mod's chassis, rail_z its rails' line), the bed (BED) and
    # its tailgate (GATE)
    "ford_f250": dict(
        title="FEM Ford F-250", src="ford_f250_2014/2014superduty", style="pickup", rail_z=0.55,
        landmarks=dict(post=0.18, cback=0.29), drop=("hdr1", "axr", "tail"),
        meshes=dict(BODY=["F250ExtCab"], HOOD=["F250ExtHood"], FENDER=["F250ExtFender"], FDOOR=["F250ExtDoor"], RDOOR=["F250ExtDoorExt"], DWIN=["F250ExtFDoorGlass"],
                    FBUMP=["F250ExtBumperF"], RBUMP=["F250ExtBumperR"], GRILLE=["F250ExtFascia"], HEAD=["F250ExtHeadlight"], TAIL=["BaseTailLight"],
                    FRAME=["FordExt14Chassis"], BED=["F250ExtBedExt"], GATE=["F250ExtTailgateExt"]),
        forset=[("Tailgate", ["tailgate"]), ("TailLight", ["lamp 1", "lamp -1"]), ("Headlight", ["headlight 1", "headlight -1"]), ("LightGlass", ["headlight 1", "headlight -1"]),
                ("DoorExt", ["door 1 rear", "door -1 rear"]), ("RDoorGlass", ["door 1 rear", "door -1 rear"]), ("rldoorpanel", ["door 1 rear"]), ("rrdoorpanel", ["door -1 rear"]),
                ("ExtDoor", ["door 1 front", "door -1 front"]), ("FDoorGlass", ["door 1 front", "door -1 front"]), ("fldoorpanel", ["door 1 front"]),
                ("frdoorpanel", ["door -1 front"]), ("Mirror", ["door 1 front", "door -1 front"]), ("Hood", ["hood"]), ("Fender", ["fender 1", "fender -1"]),
                ("Fascia", ["grille"]), ("BumperF", ["fascia"]), ("BumperR", ["rear bumper"])],
        skip=("axle", "arms", "steering", "draglink", "actuator", "pitman")),
    # a tube-frame truck (style): a cage of tube on the mod's chassis mesh (FRAME), its body the mod's panels on quick-
    # release fasteners - the front clip (FCLIP), the cab (CAB), the bed's sides (BEDSIDE), the light bar (LIGHTS); keep:
    # words of the skipped meshes' names that are this mod's own (its spare wheels)
    "trophy_truck": dict(
        title="FEM Trophy Truck", src="trophy_truck_v2/UnlimitedTrophyTruck", style="tube",
        meshes=dict(FRAME=["UnlimitedFrame"], FCLIP=["UnlimitedFrontFenders"], CAB=["UnlimitedBody"], BEDSIDE=["UnlimitedRearFenders"], LIGHTS=["UnlimitedLights"]),
        forset=[("FrontFenders", ["hood", "fender 1", "fender -1"]), ("RearFenders", ["bedside 1", "bedside -1"]), ("Body", ["side 1", "side -1", "roof"]),
                ("Lights", ["lightbar 1", "lightbar -1"]), ("Lens", ["lightbar 1", "lightbar -1"])],
        skip=("upperlink", "axle", "trailingarm"), keep=("tyre", "wheel")),
}
NAME = sys.argv[1] if len(sys.argv) > 1 else ""
assert NAME in CARS and len(sys.argv) > 2, "usage: make_part_car.py <%s> <the mod's export dir>" % "|".join(CARS)
CFG = CARS[NAME]
MESH_DIR = sys.argv[2]
SRC = CFG["src"]
SRC_FILE = os.path.join(ROOT, "assets", "vehicles", SRC + ".truck")
OUT = os.environ.get("SC_OUT") or os.path.join(ROOT, "assets", "vehicles", "shell_car", "fem_%s.truck" % NAME)

# ------------------------------------------------------------------------------------------------ the mod as exported
flex_src, props_src, wheels_src, MASS = [], [], [], 1500.0
for line in open(os.path.join(MESH_DIR, "vehicle.txt")):
    t = line.split()
    if t[0] in ("flex", "prop"):
        rec = {"i": int(t[1]), "line": int(t[2]), "mesh": t[3], "pos": np.array(t[4:7], float), "M": np.array(t[7:16], float).reshape(3, 3).T}
        (flex_src if t[0] == "flex" else props_src).append(rec)
    elif t[0] == "wheel":
        a, b = np.array(t[1:4], float), np.array(t[4:7], float)
        wheels_src.append({"c": (a + b) / 2, "R": float(t[7]), "rim": float(t[8]), "w": float(t[9]), "drive": int(t[10]), "brake": int(t[11]), "side": t[12], "mesh": t[13]})
    elif t[0] == "mass":
        MASS = float(t[1])
SRC_LINES = open(SRC_FILE, encoding="latin-1").read().split("\n")
AXLES = sorted({round(float(np.mean([q["c"][0] for q in wheels_src if abs(q["c"][0] - w["c"][0]) < 0.05])), 4) for w in wheels_src})
AXF_T, AXR_T = AXLES[0], AXLES[-1]


def load_obj(path):
    V, F = [], []
    for line in open(path):
        if line.startswith("v "):
            V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            F.append([int(x.split("/")[0]) - 1 for x in line.split()[1:4]])
    return np.array(V), np.array(F, dtype=int)


def role_mesh(role):
    """the role's meshes as exported (the mod's space): vertices, faces"""
    Vs, Fs, off = [], [], 0
    for n in CFG["meshes"][role]:
        fn = [f for f in os.listdir(MESH_DIR) if f.endswith(".mesh.obj") and f.split("_", 1)[1] == n + ".mesh.obj"]
        assert fn, "no %s in %s" % (n, MESH_DIR)
        for f in fn:
            V, F = load_obj(os.path.join(MESH_DIR, f))
            Vs.append(V), Fs.append(F + off)
            off += len(V)
    return np.concatenate(Vs), np.concatenate(Fs)


class Mesh:
    """the meshes of some roles, their triangles for rays (in the space their vertices are given in)"""
    def __init__(self, roles, to=None):
        Vs, Fs, off = [], [], 0
        for r in roles:
            V, F = role_mesh(r)
            Vs.append(V), Fs.append(F + off)
            off += len(V)
        V, F = np.concatenate(Vs), np.concatenate(Fs)
        if to is None:
            to = TI_ARRAY   # (the E36's space: where the car is built)
        self.V = V = to(V)
        self.A = V[F[:, 0]]
        self.E1 = V[F[:, 1]] - self.A
        self.E2 = V[F[:, 2]] - self.A
        self.lo, self.hi = V.min(axis=0), V.max(axis=0)

    def side(self, s):
        """the extent of its half on side s"""
        V = self.V[self.V[:, 2] * s > 0.02]
        return V.min(axis=0), V.max(axis=0)

    def hits(self, o, d):
        o, d = np.asarray(o, float), np.asarray(d, float)
        P = np.cross(d, self.E2)
        det = (self.E1 * P).sum(1)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        T_ = o - self.A
        u = (T_ * P).sum(1) * inv
        Q = np.cross(T_, self.E1)
        v = (Q * d).sum(1) * inv
        t = (Q * self.E2).sum(1) * inv
        m = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0)
        return np.sort(t[m])

    def first(self, o, d):
        h = self.hits(o, d)
        return float(h[0]) if len(h) else None


# ------------------------------------------------------------------------------------------------ the map
# the ride height: a mod's wheels are drawn at full droop (it settles when spawned); the wheel goes up into its arch
# to a hand's gap over the tyre
_roles = [r for r in ("BODY", "FENDER", "HOOD") if r in CFG["meshes"]]
_arch = Mesh(_roles, lambda V: V) if _roles else None
for w in (wheels_src if _arch is not None else []):
    gaps_ = [float(h[0]) - w["R"] for zf in (0.8, 0.9, 1.0) for h in [_arch.hits((w["c"][0], w["c"][1], w["c"][2] * zf), (0, 1, 0))] if len(h)]
    w["gap"] = min(gaps_) if gaps_ else 0.0
for ax in (AXLES if wheels_src[0]["R"] <= 0.42 and _arch is not None else []):   # (a truck's stay: its travel is long)
    ws_ = [w for w in wheels_src if abs(w["c"][0] - ax) < 0.05]
    gap = float(np.mean([w["gap"] for w in ws_]))
    print("axle at %.2f: %.3f m over the tyre" % (ax, gap))
    if gap > 0.10:
        for w in ws_:
            w["c"] = w["c"] + np.array([0.0, gap - 0.07, 0.0])


# the E36's landmarks (its meshes measured by the rules of measure below; its belt line the Shell Car's doors' top)
E36 = dict(ground=-0.06, nose=-2.32, axf=-1.44, door0=-0.787, hdr0=-0.05, door1=0.398, post=1.18, hdr1=1.35, cback=1.45, axr=1.57, lid0=1.923, tail=2.646, floor=0.255, belt=0.92, roof=1.443, zhalf=0.908)


def measure():
    """the mod's landmarks, in its space"""
    ident = lambda V: V
    body, door = Mesh(["BODY"], ident), Mesh(["FDOOR"], ident)
    L = dict(nose=float(Mesh(["FBUMP"], ident).lo[0]), tail=float(Mesh(["RBUMP"], ident).hi[0]), axf=AXF_T, axr=AXR_T)
    L["belt"] = CFG.get("belt") or float(Mesh(["DWIN"], ident).lo[1]) + 0.02
    lo, hi = door.side(1)
    ymid = 0.5 * (lo[1] + L["belt"])
    xs = [x for x in np.arange(lo[0] - 0.02, hi[0] + 0.02, 0.005) if door.first((x, ymid, 3.0), (0, 0, -1)) is not None]
    L["door0"], L["door1"] = float(xs[0]), float(xs[-1])
    zs = [3.0 - t for x in np.linspace(xs[0] + 0.1, xs[-1] - 0.1, 5) for y in np.linspace(lo[1] + 0.05, L["belt"] - 0.05, 5)
          for t in [door.first((x, y, 3.0), (0, 0, -1))] if t is not None]
    L["zhalf"] = max(zs)
    top = [(x, 5.0 - t) for x in np.arange(AXF_T, L["tail"], 0.01) for t in [body.first((x, 5.0, 0.0), (0, -1, 0))] if t is not None]
    L["roof"] = max(y for x, y in top)
    rx = [x for x, y in top if y > L["roof"] - 0.05]
    L["hdr0"], L["hdr1"] = float(rx[0]), float(rx[-1])
    bx = body.V[(body.V[:, 0] > L["door0"]) & (body.V[:, 0] < L["door1"])]
    L["floor"] = float(np.percentile(bx[:, 1], 1.0))
    if "TRUNK" in CFG["meshes"]:
        L["lid0"] = float(Mesh(["TRUNK"], ident).lo[0])
    L["ground"] = min(float(np.mean([w["c"][1] - w["R"] for w in wheels_src])), L["floor"] - 0.05)   # (under the wheels: what is under the floor keeps off it)
    L.update(CFG.get("landmarks", {}))
    for k in CFG.get("drop", ()):
        L.pop(k, None)
    return L


def _axis(keys, far):
    """a piecewise linear map of one axis through the landmarks both cars have, in the E36's order; beyond them as it is
    at the ends"""
    src = [E36[k] for k in keys if k in TGT]
    dst = [TGT[k] for k in keys if k in TGT]
    assert all(b > a + 0.02 for a, b in zip(src, src[1:])) and all(b > a + 0.02 for a, b in zip(dst, dst[1:])), "landmarks out of order: %s -> %s" % (src, dst)
    src = [src[0] - far] + src + [src[-1] + far]
    dst = [dst[0] - far] + dst + [dst[-1] + far]
    return np.array(src), np.array(dst)


TI_ARRAY = None
TGT = measure() if CFG.get("style") != "tube" else dict(E36)   # (a tube-frame truck is built in its own space: the map the identity)
_X, _Y = _axis(["nose", "axf", "door0", "hdr0", "door1", "post", "hdr1", "cback", "axr", "lid0", "tail"], 5.0), _axis(["ground", "floor", "belt", "roof"], 5.0)
_KZ = TGT["zhalf"] / E36["zhalf"]


def T(p):
    """a point of the E36's space on the mod's car"""
    return (float(np.interp(p[0], *_X)), float(np.interp(p[1], *_Y)), float(p[2]) * _KZ)


def TI(p):
    """a point of the mod's car in the E36's space"""
    return (float(np.interp(p[0], _X[1], _X[0])), float(np.interp(p[1], _Y[1], _Y[0])), float(p[2]) / _KZ)


def TI_ARRAY(V):
    return np.column_stack([np.interp(V[:, 0], _X[1], _X[0]), np.interp(V[:, 1], _Y[1], _Y[0]), V[:, 2] / _KZ])


if CFG.get("style") != "tube":
    print("the landmarks: " + ", ".join("%s %.3f" % (k, v) for k, v in TGT.items()))


def lerp(a, b, t):
    return a + (b - a) * t


def scan_grid(mesh, origin, d, us, nv, vlo, vhi, accept=None, min_span=0.15):
    """a height field of the mesh seen along d: per u a row of nv points across the span of v it covers (rays from
    origin(u, v)); a row the mesh hardly covers is left out, a ray that misses in a row's middle takes its neighbours'
    depth (a hole in the mesh: a lamp's, the grille's)"""
    d = np.asarray(d, float)
    hit = lambda u, v: (lambda t: None if t is None or (accept and not accept(np.asarray(origin(u, v)) + d * t)) else t)(mesh.first(origin(u, v), d))
    rows = []
    for u in us:
        vs = [v for v in np.arange(vlo, vhi + 1e-9, 0.01) if hit(u, v) is not None]
        if not vs or vs[-1] - vs[0] < min_span:
            continue
        row_v = [lerp(vs[0] + 0.01, vs[-1] - 0.01, j / (nv - 1)) for j in range(nv)]
        ts = [hit(u, v) for v in row_v]
        known = [j for j in range(nv) if ts[j] is not None]
        if not known:
            continue
        for j in range(nv):
            if ts[j] is None:
                lo = max((k for k in known if k < j), default=None)
                hi = min((k for k in known if k > j), default=None)
                ts[j] = ts[lo] if hi is None else ts[hi] if lo is None else lerp(ts[lo], ts[hi], (j - lo) / (hi - lo))
        rows.append([tuple(np.asarray(origin(u, v)) + d * t) for v, t in zip(row_v, ts)])
    return rows


def fender_rows(mesh, s, xs, nside, inner=True):
    """a front fender's rows: per x its side seen from the side (nside points from the foot of the span over the arch
    up to its shoulder) and, with inner, its top's inner edge seen from above (where it meets the hood's edge), 4 mm
    under it; a station the mesh is not at is left out"""
    rows = []
    for x in xs:
        vs = [v for v in np.arange(0.10, 1.10, 0.01) if mesh.first((x, v, s * 2.0), (0, 0, -s)) is not None]
        if len(vs) < 3:
            continue
        run = [vs[-1]]
        for v in reversed(vs[:-1]):   # (the run down from its top: not across the wheel's arch)
            if run[-1] - v > 0.015:
                break
            run.append(v)
        lo, hi = run[-1] + 0.005, run[0] - 0.005
        side = []
        if hi - lo < 0.12:   # (a strip over the wheel's arch: its top, and down from it 5 cm a point - no slivers)
            t = mesh.first((x, hi, s * 2.0), (0, 0, -s))
            side = [(x, hi - 0.05 * (nside - 1 - j), s * (2.0 - t)) for j in range(nside)]
        for j in range(nside if not side else 0):
            y = lerp(lo, hi, j / (nside - 1))
            t = mesh.first((x, y, s * 2.0), (0, 0, -s))
            if t is None:   # (a pinhole of the mesh: the nearest row it was hit at)
                y = min(run, key=lambda v: abs(v - y))
                t = mesh.first((x, y, s * 2.0), (0, 0, -s))
            side.append((x, y, s * (2.0 - t)))
        if inner:
            zs = [z for z in np.arange(0.40, 1.02, 0.005) if mesh.first((x, 3.0, s * z), (0, -1, 0)) is not None]
            if not zs:
                continue
            zi = min(zs[0], abs(side[-1][2]) - 0.08)   # (at least 8 cm in from its side's top)
            ti = mesh.first((x, 3.0, s * zi), (0, -1, 0))
            side.append((x, 3.0 - ti - 0.004 if ti is not None else side[-1][1], s * zi))
        rows.append(side)
    return rows


def scan_round(mesh, cx, ys, n, back, amax=90):
    """a bumper's rows of n points round its outline in plan: per y rays from (cx, y, 0) out to the tail (back 1) or the
    nose (back -1) and round to the sides, the last hit each (its outer face), evenly in angle over the span it covers -
    the corners' wrap resolved as a scan along x does not"""
    rows = []
    for y in ys:
        d = lambda a: (back * math.cos(math.radians(a)), 0.0, math.sin(math.radians(a)))
        hit = lambda a: (lambda h: float(h[-1]) if len(h) else None)(mesh.hits((cx, y, 0.0), d(a)))
        span = [a for a in range(-amax, amax + 1) if hit(a) is not None]
        row = []
        for j in range(n):
            a = lerp(float(span[0]), float(span[-1]), j / (n - 1))
            a, t = next((a + e, hit(a + e)) for e in [0] + [q * g for g in range(1, 181) for q in (1, -1)] if hit(a + e) is not None)   # (a hole: the nearest hit)
            row.append((cx + d(a)[0] * t, y, d(a)[2] * t))
        rows.append(row)
    return rows


def pick(n, k):
    """k indices spread evenly over range(n), the ends included (k a list: those)"""
    if isinstance(k, (list, tuple)):
        return list(k)
    return sorted({int(round(i * (n - 1) / (k - 1))) for i in range(k)})


# ------------------------------------------------------------------------------------------------------ the nodes
nodes = []                # (x, y, z)
node_key = {}             # (group, rounded point) -> node
node_part = {}            # node -> "biw", "susp", a part's name, "skin <part>", "plain"
load = {}                 # node -> kg
fem = {}                  # FEM shell group -> [(a, b, c)]
fem_seen = set()
members = []              # (a, b, section, joint at a, joint at b): the FEM frame's members
member_seen = set()
shocks, stops, hydros, wheels, mounts, slides = [], [], [], [], [], []


def node(p, group="biw"):
    key = (group, round(p[0], 3), round(p[1], 3), round(p[2], 3))
    if key not in node_key:
        node_key[key] = len(nodes)
        nodes.append(tuple(float(c) for c in p))
        node_part[len(nodes) - 1] = group
    return node_key[key]


def ftri(a, b, c, shell):
    if len({a, b, c}) < 3:
        return
    k = tuple(sorted((a, b, c)))
    if k in fem_seen:
        return
    fem_seen.add(k)
    fem.setdefault(shell, []).append((a, b, c))


def fquad(a, b, c, d, shell, flip=False):
    if flip:
        ftri(a, b, c, shell), ftri(a, c, d, shell)
    else:
        ftri(a, b, d, shell), ftri(b, c, d, shell)


def grid(pts, shell, parity=0):
    """quads over a grid of points (rows of equal length), of the body-in-white"""
    for i in range(len(pts) - 1):
        for k in range(len(pts[i]) - 1):
            fquad(node(pts[i][k]), node(pts[i + 1][k]), node(pts[i + 1][k + 1]), node(pts[i][k + 1]), shell, (i + k + parity) % 2 == 0)


def strip(A, B, key, shell):
    """triangles between two rows of points (each in order along key(p)), zipped"""
    i = j = 0
    while i < len(A) - 1 or j < len(B) - 1:
        if j == len(B) - 1 or (i < len(A) - 1 and key(A[i + 1]) <= key(B[j + 1])):
            ftri(node(A[i]), node(B[j]), node(A[i + 1]), shell)
            i += 1
        else:
            ftri(node(A[i]), node(B[j]), node(B[j + 1]), shell)
            j += 1


def member_n(a, b, sec, ja=None, jb=None):
    if a == b or (min(a, b), max(a, b)) in member_seen:
        return
    member_seen.add((min(a, b), max(a, b)))
    members.append((a, b, sec, ja, jb))


def member(p, q, sec, ja=None, jb=None):
    member_n(node(p), node(q), sec, ja, jb)


def chain(pts, sec):
    for p, q in zip(pts, pts[1:]):
        member(p, q, sec)


def dist(a, b):
    """the distance of two points of the E36's space on the mod's car"""
    a, b = T(a), T(b)
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def nearest(group, p, maxd=1e9):
    best, bd = None, maxd
    for key, n in node_key.items():
        if key[0] != group:
            continue
        d = dist(nodes[n], p)
        if d < bd:
            best, bd = n, d
    return best


# ------------------------------------------------------------------------------------------------ the dimensions
# (the E36's: the car is built in its space, see the map)
WHEEL_R, WHEEL_W = 0.32, 0.22
RIM_R = 0.203                   # (a 16 inch rim: the ring tyres')
AX_F, AX_R = -1.44, 1.57        # the E36's axles in its definition space (its wheels' meshes)
WHEEL_Y = 0.26                  # (the E36's 0.22 and its sag)
Y_FLOOR, Y_RFLOOR = 0.22, 0.34  # the cabin's floor, the rear floor over the axle
X_FW = -0.96                    # the firewall
Y_COWL = 0.86                   # the firewall's top, the A pillars' feet (under the hood's rear edge)
Z_RAIL = 0.38                   # the front and rear rails
X_REAR = 2.47                   # the rear panel's foot (13 cm inside the bumper's middle)
Z_PIVOT = 0.22                  # the lower wishbones' inner pivots, on the subframes' cross members
Z_BALL = 0.67                   # the ball joints, at the wheel's inner face (the rim's offset)
RACK_X, RACK_Y = AX_F + 0.18, 0.293   # the steering rack (the steering arms behind the front axle), at its tie rods' inner ends' height
SIDE_Y = [0.20, 0.27, 0.48, 0.62, 0.77, 0.92, 1.09, 1.26, 1.37]    # the E36's body side in section (its chassis mesh)
SIDE_Z = [0.80, 0.82, 0.86, 0.86, 0.84, 0.80, 0.72, 0.63, 0.55]


X_E = TI((T((1.18, 0, 0))[0] + 0.04, 0, 0))[0]


def z_side(x, y):
    """the body side's members and panels: 5 cm inside the E36's skin, narrowing at the tail"""
    z = float(np.interp(y, SIDE_Y, SIDE_Z)) - 0.05
    if x > 2.05:
        z *= 1.0 - 0.10 * min(1.0, (x - 2.05) / (X_REAR - 2.05)) ** 2
    return z


def S(x, y, s):
    """a point of the body side s at x, y"""
    return (x, y, s * z_side(x, y))


def z_low(x):
    """the rear quarters' foot behind the rear wheel: tucked in under the bumper's wrap (6 cm inside it at the sides,
    12 at the corners: the bumper's corners outside the body's)"""
    return float(np.interp(x, [1.97, 2.25, X_REAR], [z_side(1.97, Y_RFLOOR), 0.72, 0.60]))


RP_TOP = [(2.52, 0.62, -Z_RAIL), (2.53, 0.62, 0.0), (2.52, 0.62, Z_RAIL)]   # the rear panel's top between the corners: the E36's plan, 3 cm inside it


chassis = Mesh(["BODY"]) if CFG.get("style") != "tube" else None
# the body side in section: the mod's (its body and its door seen from the side at the door's middle; a height it is
# not at - a window, over the roof - keeps the E36's)
_side = Mesh(["BODY", "FDOOR"]) if chassis is not None else None
for _i, _y in enumerate(SIDE_Y if _side is not None else []):
    _t = _side.first((0.0, _y, 2.0), (0, 0, -1))
    if _t is not None and 0.45 < 2.0 - _t < 1.1:
        SIDE_Z[_i] = 2.0 - _t


def roof_y(x, z):
    """the E36's roof over (x, z), 5 mm under its skin"""
    if chassis is not None:
        t = chassis.first((x, 3.0, z), (0, -1, 0))
        if t is not None and 3.0 - t > 1.25:
            return 3.0 - t - 0.005
    return 1.37 + 0.05 * (1.0 - (z / 0.55) ** 2)


def interp_y(x, xs, ys):
    return float(np.interp(x, xs, ys))


TUBE = CFG.get("style") == "tube"
if not TUBE:   # (a tube-frame truck has none: its cage below)
    # ---------------------------------------------------------------------------------- the body-in-white: surfaces
    # (the load-carrying lines are sheets too, closed sections of FEM triangles along the lines where the sheets meet - a
    # sheet one wall of each, one new line of nodes: the sills under the floor's edges, the front rails inside the aprons,
    # the radiator support, the bumpers' beams; flanges on the other edges - the roof's (its rails and bows), the cowl, the
    # floor's cross members, the quarters' edges round the doors and the trunk's opening, the rear rails behind the
    # wheelhouses. Only the pillars (hinge, A, B, C) stay members)
    def off(pts, d):
        """the points moved by d (or by d(p))"""
        return [tuple(p[k] + (d(p) if callable(d) else d)[k] for k in range(3)) for p in pts]


    def walls(*lines, shell, closed=False):
        """strips of triangles between each two lines of points in turn (each the same stations along it; the last to the
        first too if closed): a flange (two lines), a closed section (three)"""
        ls = list(lines) + ([lines[0]] if closed else [])
        for P, Q_ in zip(ls, ls[1:]):
            grid([P, Q_], shell)


    inward = lambda s, d: (lambda p: (0.0, 0.0, -s * d))
    RAIL_IN = Z_PIVOT              # the front rails' sections' inner feet: the lower wishbones' pivots' line
    FX = [X_FW, -0.40, 0.46, 1.18]                       # the cabin floor's stations: the firewall, the seats, the B pillars, the heel kick
    FZ = [-z_side(0, Y_FLOOR), -Z_RAIL, 0.0, Z_RAIL, z_side(0, Y_FLOOR)]
    grid([[(x, Y_FLOOR, z) for z in FZ] for x in FX], "floor")
    # the firewall: the floor to the cowl, out to the hinge pillars (and the front rails' sections' inner feet)
    FW_Y = [Y_FLOOR, 0.56, Y_COWL]
    FW_Z = lambda y: [-z_side(X_FW, y), -Z_RAIL, -RAIL_IN, 0.0, RAIL_IN, Z_RAIL, z_side(X_FW, y)]
    grid([[(X_FW, y, z) for z in FW_Z(y)] for y in FW_Y], "firewall")
    # the roof between the rails and the bows (the E36's crown)
    RX = [-0.10, 0.18, 0.46, 0.96, 1.45]                 # the windscreen's header, B pillars' tops, the rear window's header
    RZ = [-z_side(0, 1.37), -0.17, 0.17, z_side(0, 1.37)]
    ROOF = [[(x, roof_y(x, z), z) for z in RZ] for x in RX]
    grid(ROOF, "roof")
    # the rear floor between the rails (up over the axle), the trunk's floor out to the quarters
    RFX = [1.18, AX_R, 1.97, 2.25, X_REAR]
    RFY = [Y_FLOOR, Y_RFLOOR, Y_RFLOOR, Y_RFLOOR, Y_RFLOOR]
    rail_y = lambda x: interp_y(x, RFX, RFY)
    grid([[(x, y, z) for z in (-Z_RAIL, 0.0, Z_RAIL)] for x, y in zip(RFX, RFY)], "floor")
    for s in (1, -1):
        grid([[(x, Y_RFLOOR, s * z) for z in (Z_RAIL, z_low(x))] for x in (1.97, 2.25, X_REAR)], "floor", 1)


    # the body side behind the rear door: the quarter panel round the wheel's arch (the arch's lip, the rear door's post),
    # its foot behind the arch tucked in under the bumper
    def skin(x, y, s):
        """a point of the body side s at x, y on the E36's skin, 8 mm under it (its depth taken where the skin is nearest:
        in the wheel's arch a ray finds the wheelhouse behind it; without the mesh 3 cm out of the members' line)"""
        if chassis is not None:
            for r in (0.0, 0.02, 0.04, 0.06, 0.08):
                for dx, dy in ((0, 0), (r, 0), (-r, 0), (0, r), (0, -r), (r, r), (-r, r)):
                    if r == 0 and (dx, dy) != (0, 0):
                        continue
                    t = chassis.first((x + dx, y + dy, s * 2.0), (0, 0, -s))
                    if t is not None and 2.0 - t > 0.6:
                        return (x, y, s * (2.0 - t - 0.008))
        return (x, y, s * (z_side(x, y) + 0.03))


    def quarter(s):
        """the quarter panel on the E36's skin (the members' line 5 cm inside it left it sunken behind the door) round its
        arch, a shoulder line across it (S1 - S3), its top along the trunk's opening, its rear edge round the tail light"""
        P, C, T1, T2 = skin(1.55, 0.92, s), skin(1.80, 0.92, s), skin(2.05, 0.92, s), skin(X_REAR, 0.92, s)
        A1, A2, A3, A4 = skin(1.25, 0.52, s), skin(AX_R, 0.68, s), skin(1.89, 0.52, s), skin(1.97, Y_RFLOOR, s)
        S1, S2, S3 = skin(1.68, 0.80, s), skin(1.95, 0.79, s), skin(2.22, 0.78, s)
        M1, M2, B1, B2 = skin(2.25, 0.62, s), skin(X_REAR, 0.62, s), (2.25, Y_RFLOOR, s * z_low(2.25)), (X_REAR, Y_RFLOOR, s * z_low(X_REAR))
        L = (X_REAR, 0.77, skin(2.40, 0.77, s)[2])   # (the tail light's middle on the rear edge: the skin ahead of its opening, through which a ray found the body 10 cm in)
        for t in ((A1, P, A2), (A2, P, S1), (S1, P, C), (A2, S1, A3), (S1, C, S2), (C, T1, S2), (A3, S1, S2), (A3, S2, A4), (A4, S2, M1),
                  (S2, S3, M1), (S2, T1, S3), (T1, T2, S3), (S3, T2, L), (S3, L, M1), (M1, L, M2), (A4, M1, B1), (B1, M1, M2), (B1, M2, B2)):
            ftri(*(node(p) for p in t), "quarter")
        return dict(P=P, C=C, T1=T1, T2=T2, A1=A1, A2=A2, A3=A3, A4=A4, S1=S1, S2=S2, S3=S3, M1=M1, M2=M2, B1=B1, B2=B2, L=L)


    Q = {s: quarter(s) for s in (1, -1)}
    # the rear panel: between the quarters, the floor (behind the bumper) to the trunk's opening
    RP_BOT = [(X_REAR, Y_RFLOOR, z) for z in (-Z_RAIL, 0.0, Z_RAIL)]
    grid([[Q[-1]["B2"]] + RP_BOT + [Q[1]["B2"]], [Q[-1]["M2"]] + RP_TOP + [Q[1]["M2"]]], "panel")
    # the parcel shelf: between the C pillars' feet and the trunk's opening
    SHELF = {x: [Q[-1][k]] + [(x, 0.92, z) for z in (-Z_RAIL, 0.0, Z_RAIL)] + [Q[1][k]] for x, k in ((1.80, "C"), (2.05, "T1"))}
    grid([SHELF[1.80], SHELF[2.05]], "panel")
    # the rear wheelhouses: from the rail up over the wheel (the upper wishbone's pivots on its middle row, the shock's top
    # on its top row) and out to the arch's lip
    WHX = [1.18, 1.45, AX_R, 1.72, 1.97]
    WH_MID = {s: [(x, y, s * 0.42) for x, y in zip(WHX, (0.36, 0.54, 0.54, 0.54, 0.40))] for s in (1, -1)}
    WH_TOP = {s: [(x, y, s * 0.58) for x, y in zip(WHX, (0.50, 0.70, 0.74, 0.70, 0.46))] for s in (1, -1)}
    for s in (1, -1):
        grid([[(x, rail_y(x), s * Z_RAIL) for x in WHX], WH_MID[s], WH_TOP[s]], "wheelhouse", 0 if s > 0 else 1)
        strip(WH_TOP[s], [Q[s]["A1"], Q[s]["A2"], Q[s]["A3"], Q[s]["A4"]], lambda p: p[0], "wheelhouse")
        ftri(node(WH_TOP[s][0]), node(Q[s]["A1"]), node(S(1.18, Y_FLOOR, s)), "wheelhouse")
    # the front: the lower rails, the upper rails along the fenders' line (under the hood's edges), the aprons between them
    # (their middle row carries the upper wishbones' pivots, the upper rail the shocks' tops); the lower rail's line through
    # the steering rack's housing's end
    AP_X = [X_FW, RACK_X, -1.32, AX_F, -1.58, -1.80, -2.10]
    LOW_Y = lambda x: interp_y(-x, [0.96, 1.44, 1.80, 2.10], [Y_FLOOR, 0.30, 0.32, 0.32])
    UP_YZ = lambda x: (interp_y(-x, [0.96, 1.44, 1.80, 2.08], [Y_COWL, 0.74, 0.68, 0.62]), interp_y(-x, [0.96, 1.44, 1.80, 2.08], [z_side(X_FW, Y_COWL), 0.68, 0.66, 0.62]))
    LOW = {s: [(x, RACK_Y if x == RACK_X else LOW_Y(x), s * Z_RAIL) for x in AP_X] for s in (1, -1)}
    MID = {s: [(X_FW, 0.56, s * Z_RAIL)] + [(x, 0.52, s * 0.40) for x in AP_X[1:]] for s in (1, -1)}
    UP = {s: [S(X_FW, Y_COWL, s)] + [(max(x, -2.08), UP_YZ(x)[0], s * UP_YZ(x)[1]) for x in AP_X[1:]] for s in (1, -1)}
    for s in (1, -1):
        grid([LOW[s], MID[s], UP[s]], "apron", 0 if s > 0 else 1)

    # ------------------------------------------------------------------------ the body-in-white: sections and flanges
    SEC = {}   # (name, side) -> its lines: the volumes' zones wrap some
    OPEN = {}  # side -> the trunk opening's inner corners (the flanges' inner edges): its members' line
    for s in (1, -1):
        # the sills: a triangle over the floor's edge - the floor, the rocker up 10 cm (the doors' sill: they shut on it),
        # back down to the rail's line (under the floor it scraped the rough field's bumps: 6 cm lower than the members were)
        edge, rail = [(x, Y_FLOOR, s * FZ[-1]) for x in FX], [(x, Y_FLOOR, s * Z_RAIL) for x in FX]
        SEC["sill", s] = (edge, off(edge, (0, 0.10, 0)), rail)
        walls(*SEC["sill", s][:2], shell="sill"), walls(*SEC["sill", s][1:], shell="sill")
        # the rocker's cover on the skin under the doors (their foot 9 cm out of the sill's face: a dark slot under them),
        # down to 17 cm off the ground, a box with the sill's face; at its rear end a piece of the quarter ahead of the arch
        kt, kb = [skin(x, 0.29, s) for x in FX], [skin(x, 0.17, s) for x in FX]
        walls(SEC["sill", s][1], kt, kb, edge, shell="rib")
        e_hi, e_lo = skin(X_E, 0.32, s), skin(X_E, 0.20, s)   # (4 cm behind the sill's end on the mod's car)
        for t in ((kt[-1], e_hi, Q[s]["A1"]), (kb[-1], e_lo, kt[-1]), (kt[-1], e_lo, e_hi)):
            ftri(*(node(p) for p in t), "quarter")
        # the front rails: a triangle inside the apron's lower half - the apron, its foot in to the lower wishbones' pivots'
        # line (their pivots on it), up to its middle row
        SEC["frail", s] = (LOW[s], [(x, y, s * RAIL_IN) for x, y, z in LOW[s]], MID[s])
        walls(*SEC["frail", s][:2], shell="rail"), walls(*SEC["frail", s][1:], shell="rail")
        # the upper rails: a triangle inside the apron's upper half - the apron, its top edge 6 cm in, down to its middle
        # row (the fenders' bolts, the hood's buffers, the shocks' tops on it)
        SEC["urail", s] = (UP[s], off(UP[s], inward(s, 0.06)), MID[s])
        walls(*SEC["urail", s][:2], shell="rail"), walls(*SEC["urail", s][1:], shell="rail")
        # the roof's rails: a flange down 7 cm along its edge
        rr = [r[-1 if s > 0 else 0] for r in ROOF]
        walls(rr, off(rr, (0, -0.07, 0)), shell="roofrail")
        # the rear rails behind the wheelhouses (in front of them the floor and the wheelhouse make the corner): a flange 8 cm up
        rl = [(x, Y_RFLOOR, s * Z_RAIL) for x in (1.97, 2.25, X_REAR)]
        walls(rl, off(rl, (0, 0.08, 0)), shell="rib")
        # the rear door's post and the quarter's top edges (the side window's foot, the trunk opening's side): flanges 5 cm in
        for line in ([S(1.18, Y_FLOOR, s), Q[s]["A1"], Q[s]["P"]], [Q[s]["P"], Q[s]["C"]]):
            walls(line, off(line, inward(s, 0.05)), shell="rib")
        # (the trunk opening's side: in 5 cm under the lid's edge - 0.62 at its front, 0.55 at its rear - no gap beside it,
        # the opening's members along its inner edge out of sight)
        line = [Q[s]["T1"], Q[s]["T2"]]
        walls(line, [(2.05, 0.92, s * 0.56), (X_REAR, 0.92, s * 0.49)], shell="rib")
        OPEN[s] = dict(C=off([Q[s]["C"]], inward(s, 0.05))[0], T1=(2.05, 0.92, s * 0.56), T2=(X_REAR, 0.92, s * 0.49))
    # the roof's headers over the windscreen and the rear window: triangles under its edges - its face 7 cm down, back up
    # to its next row (a free rib folded flat and tore under the slab)
    for i, j in ((0, 1), (len(RX) - 1, len(RX) - 2)):
        walls(ROOF[i], off(ROOF[i], (0, -0.07, 0)), ROOF[j], shell="rib")
    # (no cross members on the floor: its sheet takes the side's push between the sills' sections in its plane; ribs under
    # it scraped the rough field's bumps, their free edges over it were squeezed flat by the slab onto the floor)
    # the cowl: a triangle in front of the firewall's top - a shelf 8 cm forwards (the windscreen's foot, the hood's
    # hinges), down and back to the firewall's middle row
    row = [(X_FW, Y_COWL, z) for z in FW_Z(Y_COWL)]
    walls(row, off(row, (-0.08, -0.02, 0)), [(X_FW, 0.56, z) for z in FW_Z(0.56)], shell="rib")
    # (the trunk's opening is framed by members: see the members)
    # the radiator support: a panel across the nose between the rails' tips and the upper rails' ends (the headlights'
    # bolts and the hood's latch on its top; narrow strips of a box there tore and flew off struck by a pole); the front
    # bumper's beam a box across the rails' tips, 9 cm deep, the rear one's behind the rear panel's foot (a triangle, 5 cm)
    FOOT = [(-2.10, 0.32, z) for z in (-Z_RAIL, -Z_RAIL / 2, 0.0, Z_RAIL / 2, Z_RAIL)]
    RAD_TOP = [(-2.08, 0.62, z) for z in (-0.62, -0.31, 0.0, 0.31, 0.62)]
    RAD_MID = [tuple(round(a + (b - a) * 0.73, 3) for a, b in zip(p, q)) for p, q in zip(FOOT, RAD_TOP)]   # (y 0.54: the headlights' and the grille's lower bolts)
    grid([FOOT, RAD_MID, RAD_TOP], "rib")
    FOOT_T = off(FOOT, (0, 0.09, 0))
    walls(FOOT, off(FOOT, (-0.09, 0, 0)), off(FOOT_T, (-0.09, 0, 0)), FOOT_T, shell="beam", closed=True)
    walls(RP_BOT, off(RP_BOT, (0.05, 0, 0)), off(RP_BOT, (0.05, 0.08, 0)), shell="beam", closed=True)

    # ------------------------------------------------------------------------------------ the body-in-white: members
    SECTIONS = {  # material, shape, outer (m), wall (m), joints, [break force N, joint damping N m s/rad]
        "hinge": ("MildSteel", "box", 0.100, 0.0020, "rigid"),     # the hinge pillars (the front doors' hinges, the fenders' bolts)
        "bpillar": ("MildSteel", "box", 0.080, 0.0018, "rigid"),   # the B pillars
        "pillar": ("MildSteel", "box", 0.070, 0.0016, "rigid"),    # the A and C pillars
        "sash": ("MildSteel", "box", 0.030, 0.0012, "rigid"),      # the doors' window frames and belt rails
        "doorbar": ("MildSteel", "tube", 0.038, 0.0020, "rigid"),  # (hidden) the doors' diagonals and the frames round their feet
        "glass": ("MildSteel", "tube", 0.025, 0.0015, "rigid"),    # (hidden) crosses in the windscreen's and the rear window's openings: the glass's stiffness
        "dglass": ("MildSteel", "tube", 0.025, 0.0015, "rigid"),   # (hidden) the doors' windows' crosses
        "opening": ("MildSteel", "box", 0.050, 0.0018, "rigid"),   # (hidden) round the trunk's opening (its hinges, its latch, its buffers)
        "subframe": ("Steel", "box", 0.080, 0.0025, "rigid"),  # the subframes' cross members (the lower wishbones' pivots)
        "tower": ("Steel", "box", 0.060, 0.0020, "rigid"),     # the frames round the upper wishbones' pivots
        # the suspension: maraging steel (1.9 GPa), sized to stay elastic through every test (35 x 3 chromoly bent on a
        # rough field and folded in the crashes; 50 x 8 spring steel uprights bent a little in the head-on, the wheel
        # struck by the other car: ~300 kN through it)
        "arm": ("Maraging", "tube", 0.045, 0.0050, "rigid", 0.0, 3.0),     # wishbones, toe links
        "tierod": ("Maraging", "tube", 0.034, 0.0060, "ball"),              # tie rods
        "rack": ("Maraging", "tube", 0.036, 0.0060, "rigid"),               # the rack bar and its arms
        "upright": ("Maraging", "tube", 0.070, 0.0140, "rigid", 0.0, 3.0),  # the uprights between the ball joints and the hub
        "hub": ("Maraging", "tube", 0.055, 0.0100, "rigid", 0.0, 3.0),      # the stub axles and the steering arms
    }
    ROOF_S = {}
    for s in (1, -1):
        ROOF_S[s] = [ROOF[i][-1 if s > 0 else 0] for i in range(len(RX))]
        chain([(X_FW, y, s * z_side(X_FW, y)) for y in FW_Y], "hinge")
        chain([S(X_FW, Y_COWL, s), S(-0.63, 0.95, s), S(-0.34, 1.15, s), ROOF_S[s][0]], "pillar")          # A (the E36's line: the door frame's)
        chain([S(0.46, y, s) for y in (Y_FLOOR, 0.56, 0.92, 1.16)] + [ROOF_S[s][RX.index(0.46)]], "bpillar")   # B
        chain([ROOF_S[s][-1], S(1.62, 1.16, s), Q[s]["C"]], "pillar")                                      # C
        chain([OPEN[s]["C"], OPEN[s]["T1"], OPEN[s]["T2"], RP_TOP[2 if s > 0 else 0]], "opening")   # the trunk opening's side, down its rear corner
    # the trunk's opening framed by members inside the quarters' skin, under the lid's edges (the parcel shelf sagged
    # under the lid's hinges and the lid shook): the rear window's base, the hinges' line, the rear panel's top (the latch)
    chain([OPEN[-1]["C"]] + SHELF[1.80][1:-1] + [OPEN[1]["C"]], "opening"), chain([OPEN[-1]["T1"]] + SHELF[2.05][1:-1] + [OPEN[1]["T1"]], "opening")
    chain(RP_TOP, "opening")
    # the windscreen's and the rear window's openings: hidden crosses for the glass's stiffness (corner to corner)
    HIDDEN_SECS = ("glass", "doorbar", "dglass", "opening")   # (the trunk opening's frame too: seen beside the lid's edges it looked a hole)
    member(S(X_FW, Y_COWL, 1), ROOF_S[-1][0], "glass"), member(S(X_FW, Y_COWL, -1), ROOF_S[1][0], "glass")
    member(Q[1]["C"], ROOF_S[-1][-1], "glass"), member(Q[-1]["C"], ROOF_S[1][-1], "glass")
    BIW_NODES = {n for n, g in node_part.items() if g == "biw"}

# ------------------------------------------------------------------------------------------- a tube-frame truck
# (CARS: style "tube") No body-in-white: a cage of steel tube on the lines of the mod's chassis mesh - a ring of nodes
# at each station along it (rays from the section's middle out to the mesh's outer tubes; a ray between the tubes takes
# its neighbours'), every ring a frame, a tube along each line of nodes, a diagonal in every bay, the end rings braced
# across; a floor of sheet under it. The body is panels on it (below: the parts)
TUBE = CFG.get("style") == "tube"
HIDDEN_SECS = ("glass", "doorbar", "dglass", "opening")
if TUBE:
    SECTIONS = {
        # (quenched and tempered 4130, as the Buggy's cage: of mild steel at 45 x 3 it split in 190 places against the wall
        # at 80 km/h and the truck came apart)
        "cage": ("ChromolyHT", "tube", 0.051, 0.0035, "rigid"), "cagerail": ("ChromolyHT", "tube", 0.064, 0.0045, "rigid"),
        "subframe": ("ChromolyHT", "tube", 0.064, 0.0045, "rigid"), "tower": ("ChromolyHT", "tube", 0.051, 0.0040, "rigid"),
        "arm": ("Maraging", "tube", 0.060, 0.0070, "rigid", 0.0, 3.0), "tierod": ("Maraging", "tube", 0.042, 0.0070, "ball"),
        "rack": ("Maraging", "tube", 0.044, 0.0070, "rigid"), "upright": ("Maraging", "tube", 0.090, 0.0180, "rigid", 0.0, 3.0),
        "hub": ("Maraging", "tube", 0.075, 0.0140, "rigid", 0.0, 3.0)}
    fr = Mesh(["FRAME"])
    x0, x1 = float(fr.lo[0]) + 0.10, float(fr.hi[0]) - 0.10
    CXS = list(np.linspace(x0, x1, int(round((x1 - x0) / 0.45)) + 1))
    for ax in (AXF_T, AXR_T):   # (a station at each axle)
        k_ = min(range(len(CXS)), key=lambda i: abs(CXS[i] - ax))
        if 0 < k_ < len(CXS) - 1:
            CXS[k_] = ax
    NA = 10
    ANG = [2 * math.pi * (k + 0.5) / NA for k in range(NA)]   # (from +z round over the top: symmetric side to side)
    rings, centres = [], []
    for x in CXS:
        ys = []
        for z in (0.0, 0.25, -0.25):
            up, dn = fr.hits((x, fr.lo[1] - 1.0, z), (0, 1, 0)), fr.hits((x, fr.hi[1] + 1.0, z), (0, -1, 0))
            if len(up) and len(dn):
                ys += [fr.lo[1] - 1.0 + up[0], fr.hi[1] + 1.0 - dn[0]]
        yc = 0.5 * (min(ys) + max(ys)) if ys else 0.5 * float(fr.lo[1] + fr.hi[1])
        for again in range(3):   # (the rays from the middle of what they find)
            r = []
            for a in ANG:
                h = fr.hits((x, yc, 0.0), (0.0, math.sin(a), math.cos(a)))
                r.append(float(h[-1]) if len(h) else float("nan"))
            ys = [yc + math.sin(a) * v for a, v in zip(ANG, r) if not math.isnan(v)]
            if again == 2 or len(ys) < 4 or abs(0.5 * (min(ys) + max(ys)) - yc) < 0.02:
                break
            yc = 0.5 * (min(ys) + max(ys))
        rings.append(r), centres.append((x, yc))
    R = np.array(rings)
    R[R < 0.08] = float("nan")
    for i in range(len(CXS)):   # (side to side alike)
        for k in range(NA):
            mk = (NA // 2 - 1 - k) % NA
            vals = [v for v in (R[i, k], R[i, mk]) if not math.isnan(v)]
            if vals:
                R[i, k] = R[i, mk] = max(vals)
    for _ in range(4):
        for i in range(len(CXS)):
            for k in range(NA):
                if math.isnan(R[i, k]):
                    nb = [R[i, (k - 1) % NA], R[i, (k + 1) % NA]] + ([R[i - 1, k]] if i > 0 else []) + ([R[i + 1, k]] if i + 1 < len(CXS) else [])
                    nb = [v for v in nb if not math.isnan(v)]
                    if nb:
                        R[i, k] = float(np.mean(nb))
    R = np.nan_to_num(R, nan=0.5)
    for i in range(len(CXS)):   # (no spike: a lamp's bracket, an aerial)
        for k in range(NA):
            nb = [R[i, (k - 1) % NA], R[i, (k + 1) % NA]] + ([R[i - 1, k]] if i > 0 else []) + ([R[i + 1, k]] if i + 1 < len(CXS) else [])
            R[i, k] = min(R[i, k], 1.3 * float(np.median(nb)) + 0.05)
    RING = [[node((x, yc + math.sin(a) * R[i, k], math.cos(a) * R[i, k])) for k, a in enumerate(ANG)] for i, (x, yc) in enumerate(centres)]
    low = lambda k: math.sin(ANG[k]) < -0.5
    for i in range(len(CXS)):
        for k in range(NA):
            member_n(RING[i][k], RING[i][(k + 1) % NA], "cage")
            if i + 1 < len(CXS):
                member_n(RING[i][k], RING[i + 1][k], "cagerail" if low(k) else "cage")
                member_n(RING[i][k], RING[i + 1][(k + 1) % NA], "cage") if (i + k) % 2 else member_n(RING[i][(k + 1) % NA], RING[i + 1][k], "cage")
    for i in (0, len(CXS) - 1):
        for k in range(NA // 2):
            member_n(RING[i][k], RING[i][k + NA // 2], "cage")
    FLOOR_K = [k for k in range(NA) if low(k)]   # (the three lowest lines: the floor between them)
    for i in range(len(CXS) - 1):
        for ka, kb in zip(FLOOR_K, FLOOR_K[1:]):
            fquad(RING[i][ka], RING[i][kb], RING[i + 1][kb], RING[i + 1][ka], "floor", (i + ka) % 2 == 0)
    BIW_NODES = {n for n, g in node_part.items() if g == "biw"}
    print("the cage: %d rings of %d nodes, %d tubes" % (len(CXS), NA, len(members)))

# ------------------------------------------------------------------------------------------- a pickup's rear
# (CARS: style "pickup") The body-in-white ends behind the cab: what the saloon has behind its rear door's post - the
# rear seat's floor, the wheelhouses, the quarters, the parcel shelf, the trunk - is cut off, the cab closed by a back
# wall from the floor's rear edge up to the roof's (corner pillars beside it); under it a ladder frame (two rails of
# box section with cross members, where the mod's chassis mesh has them) from bumper to bumper, the cab and the bed on
# it by mounts (members to the body's nodes nearest each rail node); the bed a box of sheets on the mod's: its floor,
# its sides on the mod's skin, its front wall, rails round its top
PICKUP = CFG.get("style") == "pickup"
FRAME_RAILS = {}


def tnode(p, group):
    """a node at p of the mod's space"""
    return node(TI(p), group)


def NT(n):
    return np.array(T(nodes[n]))


def cut_nodes(dead):
    """the nodes dead taken out with their triangles and members, the rest renumbered"""
    remap, keep = {}, []
    for i in range(len(nodes)):
        if i not in dead:
            remap[i] = len(keep)
            keep.append(i)
    nodes[:] = [nodes[i] for i in keep]
    for d_ in (node_key, ):
        for k_ in [k_ for k_, v in d_.items() if v not in remap]:
            del d_[k_]
        for k_ in d_:
            d_[k_] = remap[d_[k_]]
    for d_ in (node_part, load):
        items = [(remap[k_], v) for k_, v in d_.items() if k_ in remap]
        d_.clear()
        d_.update(items)
    fem_seen.clear()
    for sh in fem:
        fem[sh] = [tuple(remap[k_] for k_ in t) for t in fem[sh] if all(k_ in remap for k_ in t)]
        fem_seen.update(tuple(sorted(t)) for t in fem[sh])
    members[:] = [(remap[a], remap[b], sec, ja, jb) for a, b, sec, ja, jb in members if a in remap and b in remap]
    member_seen.clear()
    member_seen.update((min(a, b), max(a, b)) for a, b, sec, ja, jb in members)


if PICKUP:
    cut_nodes({i for i, p in enumerate(nodes) if p[0] > 1.20 and not (p[1] > 1.25 and p[0] < 1.46)})
    # the cab's back wall and its corner pillars
    for s in (1, -1):
        chain([S(1.18, Y_FLOOR, s), S(1.33, 0.92, s), ROOF_S[s][-1]], "pillar")
    BACK_MID = [S(1.33, 0.92, -1), (1.33, 0.92, -Z_RAIL), (1.33, 0.92, 0.0), (1.33, 0.92, Z_RAIL), S(1.33, 0.92, 1)]
    grid([[(1.18, Y_FLOOR, z) for z in FZ], BACK_MID], "panel")
    strip(BACK_MID, ROOF[-1], lambda p: p[2], "panel")
    chain(BACK_MID, "opening")
    # the ladder frame, in the mod's space: the rails' line the chassis mesh's (its top seen from above along the
    # rails' z, 8 cm under it; a station it is not at: its neighbours')
    SECTIONS["frame"] = ("Steel", "box", 0.150, 0.0050, "rigid")
    SECTIONS["cross"] = ("Steel", "box", 0.100, 0.0040, "rigid")
    SECTIONS["bedrail"] = ("MildSteel", "box", 0.060, 0.0020, "rigid")
    fr = Mesh(["FRAME"], lambda V: V)
    zr = CFG["rail_z"]
    x0, x1 = float(fr.lo[0]) + 0.10, float(fr.hi[0]) - 0.10
    n_st = int(round((x1 - x0) / 0.55)) + 1
    FXS = list(np.linspace(x0, x1, n_st))
    for ax in (AXF_T, AXR_T):   # (a station at each axle)
        k_ = min(range(len(FXS)), key=lambda i: abs(FXS[i] - ax))
        if 0 < k_ < len(FXS) - 1:
            FXS[k_] = ax
    tops = []
    for x in FXS:
        hs = [5.0 - t for dz in (0.0, 0.03, -0.03, 0.06, -0.06) for t in [fr.first((x, 5.0, zr + dz), (0, -1, 0))] if t is not None]
        tops.append(max(hs) if hs else None)
    known = [i for i, v in enumerate(tops) if v is not None]
    assert known, "the chassis mesh is not at the rails' line z %.2f" % zr
    for i in range(len(tops)):
        if tops[i] is None:
            tops[i] = tops[min(known, key=lambda k_: abs(k_ - i))]
    for s in (1, -1):
        FRAME_RAILS[s] = [tnode((x, y - 0.08, s * zr), "biw") for x, y in zip(FXS, tops)]
        for a, b in zip(FRAME_RAILS[s], FRAME_RAILS[s][1:]):
            member_n(a, b, "frame")
    for a, b in zip(FRAME_RAILS[1], FRAME_RAILS[-1]):
        mid = tnode(tuple(0.5 * (np.array(T(nodes[a])) + np.array(T(nodes[b])))), "biw")
        member_n(a, mid, "cross"), member_n(mid, b, "cross")
    # the bed: its floor (the mod's, seen from above on the middle line), its sides on the mod's skin (three rows: the
    # floor's edge, the middle, the top), its front wall; rails of box section round its top
    bed = Mesh(["BED"], lambda V: V)
    bx0, bx1 = float(bed.lo[0]) + 0.03, float(bed.hi[0]) - 0.04
    BXS = [bx0] + [x for x in FXS if bx0 + 0.25 < x < bx1 - 0.25] + [bx1]
    t_ = bed.first((0.5 * (bx0 + bx1), 5.0, 0.0), (0, -1, 0))
    y_floor = 5.0 - t_ if t_ is not None and 5.0 - t_ < float(bed.hi[1]) - 0.25 else float(np.mean(tops)) + 0.10
    y_top = float(bed.hi[1]) - 0.03
    zmax = float(max(abs(bed.lo[2]), abs(bed.hi[2])))

    def bed_side(x, y, s):
        for dx, dy in ((0, 0), (0.06, 0), (-0.06, 0), (0, 0.06), (0.12, 0), (-0.12, 0), (0, 0.12)):
            t = bed.first((x + dx, y + dy, s * 3.0), (0, 0, -s))
            if t is not None and 3.0 - t > 0.7 * zmax:
                return (x, y, s * (3.0 - t - 0.008))
        return (x, y, s * (zmax - 0.03))

    BED_ROWS = {}
    for s in (1, -1):
        rows = [[TI(bed_side(x, y, s)) for x in BXS] for y in (y_floor, 0.5 * (y_floor + y_top), y_top)]
        grid(rows, "quarter", 0 if s > 0 else 1)
        BED_ROWS[s] = rows
        chain(rows[2], "bedrail")
    floor_rows = [[BED_ROWS[-1][0][i], TI((x, y_floor, -zr)), TI((x, y_floor, 0.0)), TI((x, y_floor, zr)), BED_ROWS[1][0][i]] for i, x in enumerate(BXS)]
    grid(floor_rows, "floor")
    front = [floor_rows[0], [BED_ROWS[-1][1][0], TI((bx0, 0.5 * (y_floor + y_top), -zr)), TI((bx0, 0.5 * (y_floor + y_top), 0.0)), TI((bx0, 0.5 * (y_floor + y_top), zr)), BED_ROWS[1][1][0]],
             [BED_ROWS[-1][2][0], TI((bx0, y_top, -zr)), TI((bx0, y_top, 0.0)), TI((bx0, y_top, zr)), BED_ROWS[1][2][0]]]
    grid(front, "panel")
    chain(front[2], "bedrail")
    BED_REAR = dict(floor=floor_rows[-1], top={s: BED_ROWS[s][2][-1] for s in (1, -1)}, mid={s: BED_ROWS[s][1][-1] for s in (1, -1)})
    # the mounts: each rail node on the two nodes of the cab or the bed nearest it (not the frame's own)
    own = {n for s in (1, -1) for n in FRAME_RAILS[s]} | {i for i in range(len(nodes)) if any(m_[2] == "cross" and i in m_[:2] for m_ in members)}
    others = [i for i, g in node_part.items() if g == "biw" and i not in own]
    for s in (1, -1):
        for n in FRAME_RAILS[s]:
            for m_ in sorted(others, key=lambda q: dist(nodes[q], nodes[n]))[:2]:
                if dist(nodes[m_], nodes[n]) < 0.9:
                    member_n(n, m_, "tower")
    BIW_NODES = {n for n, g in node_part.items() if g == "biw"}
if wheels_src[0]["R"] > 0.42:   # (a truck's wheels: the suspension's and the subframes' sections heavier)
    SECTIONS.update({"subframe": ("Steel", "box", 0.110, 0.0035, "rigid"), "tower": ("Steel", "box", 0.080, 0.0030, "rigid"),
                     "arm": ("Maraging", "tube", 0.060, 0.0070, "rigid", 0.0, 3.0), "tierod": ("Maraging", "tube", 0.042, 0.0070, "ball"),
                     "rack": ("Maraging", "tube", 0.044, 0.0070, "rigid"), "upright": ("Maraging", "tube", 0.090, 0.0180, "rigid", 0.0, 3.0),
                     "hub": ("Maraging", "tube", 0.075, 0.0140, "rigid", 0.0, 3.0)})

# ------------------------------------------------------------------------------------------------ the suspension
# The Shell Car's, as a unit about each wheel's centre scaled to the mod's wheel (k: its radius over the Shell Car's
# 0.32 m; across the car by the track, kz), in the mod's space: per wheel the lower and upper ball joints (bl, bu:
# inside the wheel), the hub's inner and outer nodes (n1, n2), the steering or toe arm (sa); the wishbones' pivots, the
# coil-overs' tops, the rack's housing and the toe links' brackets on subframes of their own (members across the car
# and towers up each side), each of their nodes tied to the three nodes of the body nearest it
FRAME_NODES = sorted(BIW_NODES)
# (the coil-overs' tops under the mod's skin: a low hood, a bed's floor - 6 cm under what is over them, the coil-over no
# shorter than 0.44 of the wheel's radius-scaled unit)
_skin_roles = [r for r in ("HOOD", "FENDER", "BODY", "TRUNK", "BED", "FCLIP", "CAB", "BEDSIDE") if r in CFG["meshes"]]
_skin = Mesh(_skin_roles, lambda V: V) if _skin_roles else None


def top_height(x, z, y_def, y_min):
    h = _skin.hits((x, 9.0, z), (0, -1, 0)) if _skin is not None else []
    return max(y_min, min(y_def, 9.0 - float(h[0]) - 0.06)) if len(h) else y_def



def tie(n, count=3, sec="tower"):
    """a member from n to each of the body's nodes nearest it"""
    for m in sorted(FRAME_NODES, key=lambda q: dist(nodes[q], nodes[n]))[:count]:
        member_n(n, m, sec)


rack_ends, HOUSING = {}, None
for wx in (AXF_T, AXR_T):
    front = wx == AXF_T
    ws = [w for w in wheels_src if abs(w["c"][0] - wx) < 0.05]
    yw, zc, Rw = float(ws[0]["c"][1]), float(np.mean([abs(w["c"][2]) for w in ws])), ws[0]["R"]
    k, kz = Rw / 0.32, min(1.3, max(0.8, zc / 0.77))
    zp, zu = zc - 0.55 * kz, zc - 0.37 * kz              # the lower and the upper pivots
    xs_piv = (wx + 0.12 * k, wx - 0.14 * k)
    ytop = top_height(wx, zc - 0.19 * kz, yw + 0.48 * k, yw + 0.30 * k)
    if ytop < yw + 0.48 * k - 1e-6:
        print("axle at %.2f: the coil-overs' tops %.2f m lower, under the skin" % (wx, yw + 0.48 * k - ytop))
    sub = {}
    for x in xs_piv:   # the subframe: a cross member through the lower pivots, a tower each side up to the upper pivot
        row = [tnode((x, yw + 0.04 * k, z), "sub") for z in (-zp, 0.0, zp)]
        member_n(row[0], row[1], "subframe"), member_n(row[1], row[2], "subframe")
        for s in (1, -1):
            lowp, upp = row[2 if s > 0 else 0], tnode((x, yw + 0.26 * k, s * zu), "sub")
            member_n(lowp, upp, "tower")
            sub[(x, s)] = (lowp, upp)
    for s in (1, -1):
        top = tnode((wx, ytop, s * (zc - 0.19 * kz)), "sub")
        (l0, u0), (l1, u1) = sub[(xs_piv[0], s)], sub[(xs_piv[1], s)]
        member_n(l0, l1, "subframe"), member_n(u0, u1, "tower"), member_n(u0, top, "tower"), member_n(u1, top, "tower"), member_n(l0, u1, "tower")
        for n in (l0, l1, u0, u1, top):
            tie(n)
    member_n(tnode((wx, ytop, zc - 0.19 * kz), "sub"), tnode((wx, ytop, -(zc - 0.19 * kz)), "sub"), "tower")   # (a brace across the tops)
    RACK_XT, RACK_YT = wx + 0.18 * k, yw + 0.033 * k
    if front:
        HOUSING = [tnode((RACK_XT, RACK_YT, z), "sub") for z in (-zp - 0.16 * kz, -zp, 0.0, zp, zp + 0.16 * kz)]
        for a, b in zip(HOUSING, HOUSING[1:]):
            member_n(a, b, "subframe")
        for n in HOUSING:
            tie(n, 2)
        member_n(HOUSING[1], sub[(xs_piv[0], -1)][0], "subframe"), member_n(HOUSING[3], sub[(xs_piv[0], 1)][0], "subframe")
    for s in (1, -1):
        w = [q for q in ws if q["c"][2] * s > 0][0]
        arm_x = wx + 0.18 * k if front else wx - 0.18 * k
        bl, bu = tnode((wx, yw - 0.14 * k, s * (zc - 0.10 * kz)), "susp"), tnode((wx, yw + 0.24 * k, s * (zc - 0.10 * kz)), "susp")
        n1, n2 = tnode((wx, yw, s * (zc - 0.15 * kz)), "susp"), tnode((wx, yw, s * (zc + 0.15 * kz)), "susp")
        sa = tnode((arm_x, yw - 0.14 * k, s * (zc - 0.08 * kz)), "susp")
        lower = [sub[(x, s)][0] for x in xs_piv]
        upper = [sub[(x, s)][1] for x in xs_piv]
        top = tnode((wx, ytop, s * (zc - 0.19 * kz)), "sub")
        for p in lower:
            member_n(p, bl, "arm", ja="ball")
        for p in upper:
            member_n(p, bu, "arm", ja="ball")
        member_n(bl, n1, "upright", ja="ball"), member_n(n1, bu, "upright", jb="ball")
        member_n(n1, n2, "hub"), member_n(n1, sa, "hub"), member_n(bl, sa, "hub", ja="ball")
        shocks.append((bl, top, front, k))
        wheels.append((n1, n2, top, w))
        kp = NT(bl)
        r = NT(sa) - kp
        if front:
            # the rack's end (it slides along the housing), its arm out to the tie rod's inner end
            re_ = tnode((RACK_XT, RACK_YT, s * 0.12 * kz), "susp")
            ti = tnode((RACK_XT, RACK_YT, s * (zc - 0.561 * kz)), "susp")
            load[re_] = load[ti] = 3.0
            rack_ends[s] = re_
            member_n(re_, ti, "rack"), member_n(ti, sa, "tierod")
            slides.append(re_)
            end, lim = HOUSING[-1 if s > 0 else 0], 34.0    # the steering stop: free through the lock and 4 degrees past it
        else:
            br = tnode((xs_piv[0], yw + 0.067 * k, s * (zc - 0.565 * kz)), "sub")   # the toe link's bracket
            member_n(br, lower[0], "tower"), member_n(br, sub[(xs_piv[0], s)][1], "tower")
            member_n(sa, br, "arm", ja="ball", jb="ball")
            end, lim = lower[1], 6.0                         # the toe stop: 6 degrees each way
        L0 = dist(nodes[sa], nodes[end])
        ds = []
        for deg in (-lim, lim):
            c, sn = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            p = kp + np.array((r[0] * c - r[2] * sn, r[1], r[0] * sn + r[2] * c))
            ds.append(float(np.linalg.norm(p - NT(end))))
        stops.append((sa, end, (L0 - min(ds)) / L0, (max(ds) - L0) / L0))
    if front:
        member_n(rack_ends[1], rack_ends[-1], "rack")   # the rack bar
        # (negative: the hydro shortens for a right turn - the rack goes right and the arms behind the axle turn the
        # wheels' fronts right)
        hydros.append((rack_ends[1], HOUSING[0], -0.18 * k * math.sin(math.radians(30.0)) / dist(nodes[rack_ends[1]], nodes[HOUSING[0]])))

# the camera's triple: it sets the car level when it is put down, so along the line under the wheels (a mod's axles
# are not drawn at one height), three nodes of their own tied to the body
def _under(ax):
    ws_ = [w for w in wheels_src if abs(w["c"][0] - ax) < 0.05]
    return float(np.mean([w["c"][1] - w["R"] for w in ws_])), float(np.mean([w["c"][1] for w in ws_]))


(_bf, _cf), (_br, _cr) = _under(AXF_T), _under(AXR_T)
_u = np.array([AXR_T - AXF_T, _br - _bf, 0.0])
_u /= np.linalg.norm(_u)
_c = np.array([0.5 * (AXF_T + AXR_T), 0.5 * (_cf + _cr), 0.0])
CAMERA = (tnode(_c, "sub"), tnode(_c + 0.9 * _u, "sub"), tnode(_c + np.array([0.0, 0.0, 0.4]), "sub"))
for n in CAMERA:
    tie(n)
member_n(CAMERA[0], CAMERA[1], "tower"), member_n(CAMERA[0], CAMERA[2], "tower")


# ---- the FEM shells: material, thickness, colour (None: the car's paint) - the body-in-white's sheets, the parts' (each
# part one sheet for its skin and its inner frame)
BLACK, RED, CLEAR, GRILLE = (0.13, 0.13, 0.14), (0.75, 0.05, 0.04), (0.84, 0.86, 0.88), (0.05, 0.05, 0.055)   # (the bumpers: black polypropylene, as the base E36s; the tail lights; the headlights' lenses)
# (the body's and the parts' sheet steel a deep-drawing one, MildSteel: it yields at half the load of the structural
# steel the car was of and stretches twice as far before it tears - the body and its parts dent and crumple, they do
# not shrug a blow off; the fenders 1 mm, the hood, doors and lid 1.2)
SHELLS = {"floor": ("MildSteel", 0.0010, None), "firewall": ("MildSteel", 0.0010, None), "roof": ("MildSteel", 0.0010, None), "quarter": ("MildSteel", 0.0010, None),
          "panel": ("MildSteel", 0.0010, None), "wheelhouse": ("MildSteel", 0.0012, None), "apron": ("MildSteel", 0.0012, None),
          "hood": ("MildSteel", 0.0012, None), "fender": ("MildSteel", 0.0010, None), "door": ("MildSteel", 0.0012, None), "lid": ("MildSteel", 0.0012, None),
          "bumper": ("Plastic", 0.0040, BLACK), "lamp": ("Plastic", 0.0030, RED), "headlamp": ("Plastic", 0.0030, CLEAR), "grille": ("Plastic", 0.0030, GRILLE),
          # the sections' and the flanges' walls: the sills' and the front and upper rails' 1.8 mm (the members' they stand
          # for), the radiator support, the ribs and flanges 1.5, the roof's rails and the bumpers' beams 2.0
          "sill": ("MildSteel", 0.0018, None), "rail": ("MildSteel", 0.0018, None), "roofrail": ("MildSteel", 0.0020, None), "rib": ("MildSteel", 0.0015, None),
          "beam": ("MildSteel", 0.0020, None)}
BOX_SHELLS = ("sill", "rail", "roofrail", "rib", "beam")   # (the body-in-white's sections and flanges: sheets of their own)
BIW_SHELLS = ("floor", "firewall", "roof", "quarter", "panel", "wheelhouse", "apron") + BOX_SHELLS
DENSITY = {"Steel": 7850.0, "MildSteel": 7850.0, "Plastic": 950.0}
SHEET_KG_M2 = 4.7   # (the sheet body over the plates: 0.6 mm of steel - 0.8 made the body stiff)


def area(t):
    a, b, c = (np.array(T(nodes[k])) for k in t)   # (on the mod's car)
    return 0.5 * float(np.linalg.norm(np.cross(b - a, c - a)))


# ---------------------------------------------------------------------------------- the hang-on parts (the mod's)
# (the bumpers' clamps break past this share of their part's weight's moment about them, hung there alone: a bumper
# left on one bracket or clip swung on it about under the car; the kClampGive turn, 0.2 rad, first)
TWIST = 0.35


class Part:
    def __init__(self, name, shell, grid_pts, rows, cols, kg):
        """a part of FEM triangles (the FEM shell `shell`) over rows x columns of the grid's points (counts spread evenly
        over the grid, or lists of its indices); kg: its hinges', latches', glass's, trim's mass, on its nodes"""
        self.name, self.shell = name, shell
        ri, ci = pick(len(grid_pts), rows), pick(len(grid_pts[0]), cols)
        self.fem = [[node(grid_pts[i][j], name) for j in ci] for i in ri]
        for i in range(len(ri) - 1):
            for j in range(len(ci) - 1):
                fquad(self.fem[i][j], self.fem[i][j + 1], self.fem[i + 1][j + 1], self.fem[i + 1][j], shell, (i + j) % 2 == 0)
        ns = [k for row in self.fem for k in row]
        for k in ns:
            load[k] = load.get(k, 0.0) + kg / len(ns)

    def frame(self, mesh, s):
        """a door's window frame (the mod's: its door mesh's outline over the belt, 1 cm inside its face; a frameless
        door has none) from its top row's front and rear nodes, a belt rail along that row, and its hidden
        reinforcements"""
        ytop = float(mesh.side(s)[1][1])
        edge = []
        for y in ([0.92 + (ytop - 0.92) * k for k in (0.30, 0.62, 0.93)] if ytop > 1.12 else []):
            xs = [x for x in np.arange(-1.0, 2.0, 0.01) if len(mesh.hits((x, y, s * 2.0), (0, 0, -s)))]
            xs = [x for x in xs if abs(x - np.mean(xs)) < 0.8]
            if len(xs) < 4:
                edge = []
                break
            edge.append([(x, y, s * (2.0 - mesh.first((x, y, s * 2.0), (0, 0, -s)) - 0.01)) for x in (xs[0] + 0.015, xs[-1] - 0.015)])
        top = self.fem[-1]
        chain_n = None
        if edge:
            pts = [nodes[top[0]]] + [e[0] for e in edge] + [e[1] for e in edge[::-1]] + [nodes[top[-1]]]
            chain_n = [top[0]] + [node(p, self.name) for p in pts[1:-1]] + [top[-1]]
            for a, b in zip(chain_n, chain_n[1:]):
                member_n(a, b, "sash")
        for a, b in zip(top, top[1:]):
            member_n(a, b, "sash")
        # its reinforcements, hidden: the two diagonals of its panel (an intrusion beam's), round its foot (the bottom
        # edge, up its ends to the first row), and the window's cross (the glass's stiffness)
        f = self.fem
        member_n(f[-1][0], f[0][-1], "doorbar"), member_n(f[0][0], f[-1][-1], "doorbar")
        for a, b in list(zip(f[0], f[0][1:])) + [(f[0][0], f[1][0]), (f[0][-1], f[1][-1])]:
            member_n(a, b, "doorbar")
        if chain_n:
            member_n(top[0], chain_n[4], "dglass"), member_n(top[-1], chain_n[3], "dglass")
        # the door shut: its edges on the opening's flanges (stops, stiff) - the bottom on the sill, the hinge and lock
        # sides on the pillars, the frame's top corners on the roof rail
        rim = {(0, j) for j in range(len(self.fem[0]))} | {(i, j) for i in range(len(self.fem)) for j in (0, len(self.fem[0]) - 1)}
        for i, j in sorted(rim):
            self.stop(i, j, nodes[self.fem[i][j]], 0.0)
        for n in ((chain_n[3], chain_n[4]) if chain_n else ()):
            self.stop(None, n, nodes[n], 0.0)

    def near(self, p):
        """the (i, j) of the part's node nearest p"""
        return min(((i, j) for i in range(len(self.fem)) for j in range(len(self.fem[0]))), key=lambda ij: dist(nodes[self.fem[ij[0]][ij[1]]], p))

    def P(self, i, j):
        return nodes[self.fem[i][j]]

    def mass_centre(self):
        """its mass (its triangles' and its load, kg) and its centre (on the mod's car)"""
        ns = {k for row in self.fem for k in row}
        mat, th, _ = SHELLS[self.shell]
        m, mc = 0.0, np.zeros(3)
        for t in fem.get(self.shell, []):
            if all(k in ns for k in t):
                kg = area(t) * th * DENSITY[mat]
                m, mc = m + kg, mc + kg * np.mean([T(nodes[k]) for k in t], axis=0)
        for k in ns:
            m, mc = m + load.get(k, 0.0), mc + load.get(k, 0.0) * np.array(T(nodes[k]))
        return m, mc / m

    def twist(self, i, j, share=TWIST):
        """a clamp's break moment at its node (i, j) (N m): a share (TWIST) of its weight's moment about it, hung there
        alone - a part left on that bolt twists it off"""
        m, c = self.mass_centre()
        p = T(self.P(i, j))
        return share * m * 9.81 * math.hypot(c[0] - p[0], c[2] - p[2])

    def straighten_hinges(self):
        """its hinges' nodes put on one line, the one that fits them best (an edge scanned off the mesh is curved: a
        door held at four points off one line could not turn - its latch gone, it stood 2 cm open)"""
        own = {k for row in self.fem for k in row}
        hn = sorted({n for m in mounts if m[3] == "h" and m[1] in own for n in (m[1], m[4])})
        if len(hn) < 3:
            return
        P = np.array([T(nodes[n]) for n in hn])   # (on the mod's car: straight there)
        c = P.mean(axis=0)
        e = np.linalg.svd(P - c)[2][0]   # (the principal axis)
        for n, p in zip(hn, P):
            nodes[n] = TI(tuple(float(v) for v in c + e * np.dot(p - c, e)))

    def mount(self, kind, i, j, target, brk, param=0.0, i2=None, j2=None, k=0.0):
        """the shell's node (i, j) (negative: from the end) on the body's node nearest target: p a latch, c a clamp
        (param: its break moment), h a hinge (its second node (i2, j2) on the line); k its stiffness (0: the step's)"""
        a = nearest("biw", target)
        mounts.append((a, self.fem[i][j], brk, kind, self.fem[i2][j2] if kind == "h" else param, k))

    def stop(self, i, j, target, k=100000.0):
        """a rubber buffer (100 kN/m) under the part's node (i, j) (or node j, i None) on the body's node nearest target"""
        mounts.append((nearest("biw", target), self.fem[i][j] if i is not None else j, 0.0, "s", 0.0, k))

    def strap(self, i, j, target, axis0, axis1, deg):
        """an opening stay from (i, j) to the body's node nearest target: taut when the part has turned deg about the
        hinge's line axis0 - axis1 (the way that takes it away from that node)"""
        a = nearest("biw", target)
        b = self.fem[i][j]
        p, o = np.array(T(nodes[b])), np.array(T(axis0))   # (on the mod's car)
        e = np.array(T(axis1)) - o
        e /= np.linalg.norm(e)
        best = 0.0
        for sign in (1, -1):
            th = math.radians(deg) * sign
            v = p - o
            rr = o + v * math.cos(th) + np.cross(e, v) * math.sin(th) + e * np.dot(e, v) * (1 - math.cos(th))
            best = max(best, float(np.linalg.norm(rr - np.array(T(nodes[a])))))
        mounts.append((a, b, 20000.0, "r", best / dist(nodes[a], nodes[b]), 0.0))


parts = []
if wheels_src[0]["R"] > 0.42:   # (a truck's hood: two metres wide, of 1.2 mm it yielded at its corners as it lay)
    SHELLS["hood"] = ("MildSteel", 0.0016, None)
HAS = lambda role: role in CFG["meshes"]
from_above = lambda x, z: (x, 3.0, z)


def lamp_parts(role, name, shell, x_from, dx, cols, kg):
    """a pair of lamps, each side's half of the role's meshes seen along x: bolted (clamps) at its corners to the body's
    nodes nearest them (four bolts, a real lamp's: on two it shook)"""
    m = Mesh([role])
    x_from = float(m.hi[0]) + 1.0 if dx < 0 else float(m.lo[0]) - 1.0   # (from outside it)
    for s in (1, -1):
        lo, hi = m.side(s)
        ys = list(np.linspace(lo[1] + 0.02, hi[1] - 0.02, 3))
        pts = scan_grid(m, lambda y, z: (x_from, y, z), (dx, 0, 0), ys, cols, lo[2] - 0.005, hi[2] + 0.005, min_span=0.03)
        if len(pts) < 2:
            print("  no %s on side %d" % (name, s))
            continue
        lamp = Part("%s %d" % (name, s), shell, pts, len(pts), cols, kg)
        for i, j in ((0, 0), (0, -1), (-1, 0), (-1, -1)):
            lamp.mount("c", i, j, lamp.P(i, j), 3000.0, 150.0)
        parts.append(lamp)


def bumper_part(role, name, back, kg, span):
    """a bumper's cover scanned round its outline (rays from a point inside its wrap), on brackets (clamps) to the beam
    and clips at its sides' ends (clamps too, soft) - each twisted off past the moment of the bumper's weight hung on it"""
    m = Mesh(role)
    yr = CFG.get(name.split()[0] + "_y")
    y0, y1 = (TI((0, yr[0], 0))[1], TI((0, yr[1], 0))[1]) if yr else (m.lo[1], m.hi[1])
    ys = list(np.linspace(y0 + 0.03, y1 - 0.03, 4))
    cx = min(float(m.lo[0]) + 0.35, float(m.hi[0]) - 0.08) if back < 0 else max(float(m.hi[0]) - 0.60, float(m.lo[0]) + 0.08)
    part = Part(name, "bumper", scan_round(m, cx, ys, 11, back, span), 4, 11, kg)
    ym = 0.5 * (ys[1] + ys[2])
    for s in (1, -1):
        if back < 0:
            sets = (((float(m.lo[0]) + 0.06, ym, s * Z_RAIL), (-2.19, 0.32, s * Z_RAIL), 6000.0, 0.0), ((cx, ys[2], s * 1.5), UP[s][-2], 2500.0, 30000.0))
        else:
            at0, at1 = (float(m.hi[0]) - 0.06, ym, s * Z_RAIL), (cx, ys[2], s * 1.5)
            # (a pickup's: on the frame's and the bed's nodes nearest its own)
            sets = ((at0, part.P(*part.near(at0)) if PICKUP else (2.52, 0.38, s * Z_RAIL), 6000.0, 0.0), (at1, part.P(*part.near(at1)) if PICKUP else Q[s]["A4"], 2500.0, 30000.0))
        for at, target, brk, k in sets:
            part.mount("c", *part.near(at), target, brk, part.twist(*part.near(at), TWIST if k == 0.0 else 0.5), k=k)
    parts.append(part)
    return part


if not TUBE and not os.environ.get("BIW_ONLY"):   # (BIW_ONLY=1: the body-in-white alone, for a look at it)
    # the hood: its hinges on the cowl (its rear edge's line), the latch on the radiator support's top, buffers on it
    # and on the upper rails, an opening stay to the firewall at 70 degrees; its rows follow its front's outline
    # (scanned per column across it, front to rear)
    hm = Mesh(["HOOD"])
    zw = (float(max(abs(hm.lo[2]), abs(hm.hi[2]))) - 0.03) * CFG.get("hood_width", 1.0)
    hcols = scan_grid(hm, lambda z, x: (x, 3.0, z), (0, -1, 0), [f * zw for f in (-1.0, -0.86, -0.52, 0.0, 0.52, 0.86, 1.0)], 8, hm.lo[0] - 0.05, hm.hi[0] + 0.05, min_span=0.3)
    hood = Part("hood", "hood", [list(r) for r in zip(*hcols)], 5, len(hcols), 2.0)
    nc = len(hood.fem[0])
    for s in (1, -1):
        hood.mount("h", -1, nc - 2 if s > 0 else 1, (X_FW, Y_COWL, s * Z_RAIL), 15000.0, i2=-1, j2=nc - 1 if s > 0 else 0)
    hood.mount("p", 0, nc // 2, (-2.08, 0.62, 0.0), 6000.0)
    for j in (0, -1):
        hood.stop(0, j, (-2.08, 0.62, math.copysign(0.62, hood.P(0, j)[2])))
        hood.stop(2, j, (AX_F, 0.74, math.copysign(0.68, hood.P(2, j)[2])))
    hood.strap(2, nc // 2, (X_FW, 0.56, 0.0), hood.P(-1, 0), hood.P(-1, -1), 70.0)
    parts.append(hood)
    REAR_TOP = [Q[-1]["M2"]] + RP_TOP + [Q[1]["M2"]]
    if HAS("TRUNK"):
        # the trunk lid (or the hatch), an L in section: its top and its rear face; hinges on the trunk opening's front,
        # the latch on the rear panel's top, buffers there and on the quarters' rear corners, a stay to the parcel shelf
        tm = Mesh(["TRUNK"])
        zr = float(max(abs(tm.lo[2]), abs(tm.hi[2]))) + 0.02
        top = scan_grid(tm, from_above, (0, -1, 0), list(np.linspace(tm.lo[0] + 0.05, tm.hi[0] - 0.08, 5)), 7, -zr, zr)
        y_edge = min(p[1] for p in top[-1])
        ys = list(np.linspace(y_edge - 0.08, tm.lo[1] + 0.03, 4)) if y_edge - 0.08 > tm.lo[1] + 0.12 else []
        lid = top + (scan_grid(tm, lambda y, z: (3.2, y, z), (-1, 0, 0), ys, 7, -zr, zr) if ys else [])
        assert len(lid) >= 3, "the trunk lid's scan: %d rows" % len(lid)
        trunk = Part("trunk", "lid", lid, min(6, len(lid)), 5, 6.0)
        for s in (1, -1):
            trunk.mount("h", 0, 3 if s > 0 else 1, (2.05, 0.92, s * Z_RAIL), 12000.0, i2=0, j2=4 if s > 0 else 0)
        trunk.mount("p", -1, 2, (X_REAR, 0.62, 0.0), 6000.0)
        for j in (0, -1):
            trunk.stop(-1, j, (X_REAR, 0.62, math.copysign(Z_RAIL, trunk.P(-1, j)[2])))
            trunk.stop(2, j, Q[1 if trunk.P(2, j)[2] > 0 else -1]["T2"])
        trunk.strap(1, 2, (1.80, 0.92, 0.0), trunk.P(0, 0), trunk.P(0, -1), 75.0)
        parts.append(trunk)
    elif not PICKUP:
        # no lid: a deck over the trunk, the body's skin between the parcel shelf's rear edge and the rear panel's top
        mid = []
        for p, q in zip(SHELF[2.05], REAR_TOP):
            x, z = 0.5 * (p[0] + q[0]), 0.5 * (p[2] + q[2])
            t = chassis.first((x, 3.0, z), (0, -1, 0))
            mid.append((x, 3.0 - t - 0.008 if t is not None else 0.5 * (p[1] + q[1]), z))
        grid([SHELF[2.05], mid, REAR_TOP], "panel")
    if HAS("TAIL"):
        lamp_parts("TAIL", "lamp", "lamp", 4.0, -1, 4, 0.8)
    if HAS("HEAD"):
        lamp_parts("HEAD", "headlight", "headlamp", -4.0, 1, 5, 1.2)
    if HAS("GRILLE"):
        # the grille: black plastic, bolted (clamps) to the radiator support
        gm = Mesh(["GRILLE"])
        pts = scan_grid(gm, lambda y, z: (-4.0, y, z), (1, 0, 0), list(np.linspace(gm.lo[1] + 0.02, gm.hi[1] - 0.02, 3)), 5, gm.lo[2] - 0.005, gm.hi[2] + 0.005, min_span=0.2)
        if len(pts) >= 2:
            grille = Part("grille", "grille", pts, len(pts), 5, 0.6)
            for i, j in ((0, 0), (0, -1), (-1, 0), (-1, 2), (-1, -1)):
                grille.mount("c", i, j, grille.P(i, j), 2000.0, 60.0)
            parts.append(grille)
    fd = Mesh(["FDOOR"])
    ff = Mesh(["FENDER"]) if HAS("FENDER") else None
    for s in (1, -1):
        if ff is not None:
            # a fender: its side and its top in to the hood's edge, from the headlight to the door; bolted (clamps) along
            # its top to the upper rail, at the front to its end, at the rear to the hinge pillar; resting on the apron
            lo, hi = ff.side(s)
            x1 = min(float(hi[0]), fd.side(s)[0][0] + 0.02)   # (to the door's front edge: a hood that is the fenders too runs on over the cowl)
            rows = fender_rows(ff, s, list(np.linspace(lo[0] + CFG.get("fender_x0", 0.08), x1 - 0.03, 6)), 3, inner=not CFG.get("clamshell"))
            if len(rows) >= 3:
                fender = Part("fender %d" % s, "fender", rows, len(rows), len(rows[0]), 1.5)
                nr = len(fender.fem)
                for i in range(1, nr - 1):
                    fender.mount("c", i, -1, fender.P(i, -1), 10000.0, 1200.0)
                fender.mount("c", 0, -1, UP[s][-1], 10000.0, 1200.0)
                fender.mount("c", -1, 0, S(X_FW, Y_FLOOR, s), 10000.0, 1200.0)
                for i, j in [(i, 1) for i in range(1, nr - 1)] + [(-1, j) for j in range(len(fender.fem[0]))]:
                    fender.stop(i, j, fender.P(i, j), 0.0)
                parts.append(fender)
            else:
                print("  no fender on side %d: %d rows" % (s, len(rows)))
        # the door: two hinges on the pillar ahead, the latch and its foot's catch on the pillar behind, a check strap
        lo, hi = fd.side(s)
        door_side = lambda u, v, s=s: (v, u, s * 2.0)
        pts = scan_grid(fd, door_side, (0, 0, -s), np.linspace(float(lo[1]) + 0.04, 0.91, 4), 6, float(lo[0]) - 0.03, float(hi[0]) + 0.03)
        assert len(pts) == 4, "the door's scan: %d rows" % len(pts)
        door = Part("door %d front" % s, "door", pts, 4, 5, 9.0)
        # (both hinges hold the same two nodes of its front edge: their line is the axis)
        door.mount("h", 2, 0, S(X_FW, Y_COWL, s), 40000.0, i2=1, j2=0)
        door.mount("h", 1, 0, S(X_FW, 0.56, s), 40000.0, i2=2, j2=0)
        door.mount("p", 1, -1, S(0.46, 0.56, s), 16000.0)
        door.mount("p", 0, -1, S(0.46, Y_FLOOR + 0.10, s), 12000.0)
        door.frame(fd, s)
        door.strap(1, 1, (-0.40, Y_FLOOR + 0.10, s * FZ[-1]), door.P(1, 0), door.P(2, 0), 70.0)
        parts.append(door)
        if HAS("RDOOR"):
            # the rear door: hinges on the B pillar, the latch and its foot's catch on the post behind
            rd = Mesh(["RDOOR"])
            lo, hi = rd.side(s)
            pts = scan_grid(rd, door_side, (0, 0, -s), np.linspace(float(lo[1]) + 0.04, 0.91, 4), 6, float(lo[0]) - 0.03, float(hi[0]) + 0.03)
            assert len(pts) == 4, "the rear door's scan: %d rows" % len(pts)
            door_r = Part("door %d rear" % s, "door", pts, 4, 5, 8.0)
            door_r.mount("h", 2, 0, S(0.46, 0.92, s), 40000.0, i2=1, j2=0)
            door_r.mount("h", 1, 0, S(0.46, 0.56, s), 40000.0, i2=2, j2=0)
            door_r.mount("p", 1, -1, door_r.P(1, -1), 16000.0)
            door_r.mount("p", 0, -1, door_r.P(0, -1), 12000.0)
            door_r.frame(rd, s)
            door_r.strap(1, 1, S(0.46, 0.56, s), door_r.P(1, 0), door_r.P(2, 0), 70.0)
            parts.append(door_r)
            continue
        if PICKUP:
            continue
        # a two-door body: the fixed panel behind the door, on the mod's skin between the B pillar and the rear arch's post
        sill_top = lambda x, s=s: (x, Y_FLOOR + 0.10, s * FZ[-1])
        grid([[sill_top(0.46), S(0.46, 0.56, s), S(0.46, 0.92, s)], [skin(0.82, 0.32, s), skin(0.82, 0.56, s), skin(0.82, 0.92, s)],
              [sill_top(1.18), Q[s]["A1"], Q[s]["P"]]], "quarter", 0 if s > 0 else 1)
    if HAS("GATE"):
        # the tailgate: hinged along its foot on the bed's floor's rear edge, latched at its top corners to the bed's sides,
        # held by stays when open
        gm = Mesh(["GATE"])
        pts = scan_grid(gm, lambda y, z: (9.0, y, z), (-1, 0, 0), list(np.linspace(gm.lo[1] + 0.03, gm.hi[1] - 0.03, 4)), 5, gm.lo[2] - 0.005, gm.hi[2] + 0.005)
        assert len(pts) >= 3, "the tailgate's scan: %d rows" % len(pts)
        gate = Part("tailgate", "lid", pts, len(pts), 5, 6.0)
        gate.mount("h", 0, 1, BED_REAR["floor"][1], 20000.0, i2=0, j2=0)
        gate.mount("h", 0, 3, BED_REAR["floor"][3], 20000.0, i2=0, j2=4)
        for j, s in ((0, -1), (-1, 1)):
            gate.mount("p", -1, j, BED_REAR["top"][s], 8000.0)
            gate.stop(-2, j, BED_REAR["mid"][s], 0.0)
            gate.strap(-1, j, BED_REAR["top"][s], gate.P(0, 0), gate.P(0, -1), 90.0)
        parts.append(gate)
    fascia = bumper_part(["FBUMP"], "fascia", -1, 4.0, CFG.get("fascia_span", 90))
    rbump = bumper_part(["RBUMP"], "rear bumper", 1, 3.0, CFG.get("rear_span", 90))
    for part in parts:
        part.straighten_hinges()
    for part in parts:
        if part.shell == "bumper":
            m_, c_ = part.mass_centre()
            print("  %s: %.1f kg, its clamps twist off at %s N m" % (part.name, m_, ", ".join("%.0f" % mm[4] for mm in mounts if mm[3] == "c" and node_part[mm[1]] == part.name)))
if not TUBE:
    BIW_NODES = {n for n, g in node_part.items() if g == "biw"}

def panel(name, pts, kg, brk=4000.0, moment=250.0):
    """a body panel of a tube-frame truck: one sheet of FEM triangles on quick-release fasteners (clamps) at its corners
    and its edges' middles, to the cage's nodes nearest them"""
    part = Part(name, "bumper", pts, len(pts), len(pts[0]), kg)
    nr, nc = len(part.fem), len(part.fem[0])
    for i, j in sorted({(0, 0), (0, nc - 1), (nr - 1, 0), (nr - 1, nc - 1), (0, nc // 2), (nr - 1, nc // 2), (nr // 2, 0), (nr // 2, nc - 1)}):
        part.mount("c", i, j, part.P(i, j), brk, moment)
    parts.append(part)
    return part


if TUBE and not os.environ.get("BIW_ONLY"):
    # the front clip: its top (the hood and the fenders' tops, seen from above) and each fender's side; the cab: each
    # side and the roof; the bed's sides; the light bar
    fc = Mesh(["FCLIP"])
    zw = float(max(abs(fc.lo[2]), abs(fc.hi[2]))) - 0.06
    cols = scan_grid(fc, lambda z, x: (x, 5.0, z), (0, -1, 0), [f * zw for f in (-1.0, -0.7, -0.38, 0.0, 0.38, 0.7, 1.0)], 6, fc.lo[0] - 0.05, fc.hi[0] + 0.05, min_span=0.3)
    panel("hood", [list(r) for r in zip(*cols)], 4.0)
    cab = Mesh(["CAB"])
    cz = 0.6 * float(max(abs(cab.lo[2]), abs(cab.hi[2])))
    rcols = scan_grid(cab, lambda z, x: (x, 5.0, z), (0, -1, 0), [-cz, -0.5 * cz, 0.0, 0.5 * cz, cz], 5, cab.lo[0] - 0.05, cab.hi[0] + 0.05, min_span=0.3,
                      accept=lambda p: p[1] > float(cab.hi[1]) - 0.35)
    if len(rcols) >= 3:
        panel("roof", [list(r) for r in zip(*rcols)], 3.0)
    for s in (1, -1):
        side = lambda u, v, s=s: (v, u, s * 4.0)
        lo, hi = fc.side(s)
        rows = fender_rows(fc, s, list(np.linspace(lo[0] + 0.10, hi[0] - 0.06, 5)), 3, inner=False)
        if len(rows) >= 3:
            panel("fender %d" % s, rows, 2.0)
        for role, name, ny, nx in (("CAB", "side", 4, 5), ("BEDSIDE", "bedside", 4, 6)):
            m = Mesh([role])
            lo, hi = m.side(s)
            pts = scan_grid(m, side, (0, 0, -s), list(np.linspace(float(lo[1]) + 0.06, float(hi[1]) - 0.10, ny)), nx, float(lo[0]) - 0.03, float(hi[0]) + 0.03,
                            accept=lambda p, s=s, m=m: p[2] * s > 0.45 * float(max(abs(m.lo[2]), abs(m.hi[2]))), min_span=0.4)
            if len(pts) >= 2:
                panel("%s %d" % (name, s), pts, 4.0)
            else:
                print("  no %s on side %d: %d rows" % (name, s, len(pts)))
    if HAS("LIGHTS"):
        lamp_parts("LIGHTS", "lightbar", "lamp", -6.0, 1, 4, 1.5)
    for part in parts:
        m_, c_ = part.mass_centre()
        print("  %s: %d nodes, %.1f kg" % (part.name, sum(len(r) for r in part.fem), m_))
    BIW_NODES = {n for n, g in node_part.items() if g == "biw"}


# ---- the collision volumes (phys::CollisionVolume): what fills the car, as convex hulls riding on the body-in-white
# round them (their anchors: its nodes near each), in zones that each follow their own part of the body as it deforms -
# the engine (a straight six: its sump, block and head; drawn) and its gearbox in the tunnel (drawn); the others fitted
# close to the geometry round them (fit_zone): the left and the right tower with its rails (out past the apron and the
# upper rail to under the fender and the hood, over the front wheel's room), the four seats' spaces and the two behind
# the rear seats (to the doors, the floor, the roof, the window openings), the trunk's halves (to the lid, the lamps,
# the rear panel, the bumper), and the bumpers' reinforcements (their beams' bars, to the covers). Other bodies and the
# car's own parts (the hood, the doors, the fenders, the lid, the bumpers, the lamps) are kept out of them
VOLUMES = []   # (name, anchors, points, break rms, colour or None)
CRUSH = {"bumper_front": 150000.0, "bumper_rear": 150000.0}   # (a volume's crush force, N: collision_volumes' third value)


def biw_near(x0, x1, y0, y1, z0, z1):
    """the body-in-white's nodes in the box (a zone's anchors)"""
    return sorted(n for n in BIW_NODES if x0 <= nodes[n][0] <= x1 and y0 <= nodes[n][1] <= y1 and z0 <= nodes[n][2] <= z1)


def mirror_z(pts, s):
    return [(x, y, s * z) for x, y, z in pts]


def hull_np(P):
    """a convex hull's planes of the points (n outwards, d: inside n . x <= d; over all their triples)"""
    P = np.asarray(P, float)
    idx = np.array(list(itertools.combinations(range(len(P)), 3)))
    a, b, c = P[idx[:, 0]], P[idx[:, 1]], P[idx[:, 2]]
    N = np.cross(b - a, c - a)
    L = np.linalg.norm(N, axis=1)
    ok = L > 1e-9
    N, a = N[ok] / L[ok, None], a[ok]
    D = (N * a).sum(1)
    S = P @ N.T - D
    pos, neg = (S > 1e-6).any(0), (S < -1e-6).any(0)
    keep = ~(pos & neg)
    N, D, flip = N[keep], D[keep], pos[keep]
    N[flip] *= -1
    D[flip] *= -1
    _, u = np.unique(np.round(np.hstack([N, D[:, None]]), 4), axis=0, return_index=True)
    return N[u], D[u]


def fit_surfaces():
    """what the zones are fitted to: (a, b, c, what) - the FEM triangles (what: a part's name, or "biw <its shell>"),
    the window openings ("window": the side windows on the body side's line over the belt, the windscreen and the rear
    window between their pillars), the body side's line under the doors ("side": the sills') and the front wheels' room ("wheel": a drum round the axle, the tyre 9 cm up, from
    its inner face out: the apron inside it)"""
    out = []
    for sh, ts in fem.items():
        for a, b, c in ts:
            grp = node_part[a]
            out.append((nodes[a], nodes[b], nodes[c], "biw " + sh if grp == "biw" else grp))

    def quad(p, q, r, t, what):
        out.append((p, q, r, what))
        out.append((p, r, t, what))
    for s in (1, -1):
        xs, ys = np.linspace(X_FW - 0.10, 1.95, 14), [0.90, 1.05, 1.20, 1.40]
        for x0, x1 in zip(xs, xs[1:]):
            for y0, y1 in zip(ys, ys[1:]):
                quad(S(x0, y0, s), S(x1, y0, s), S(x1, y1, s), S(x0, y1, s), "window")
            quad(S(x0, 0.16, s), S(x1, 0.16, s), S(x1, 0.32, s), S(x0, 0.32, s), "side")   # (the sills' line under the doors)
    pillars = (lambda s: [S(X_FW, Y_COWL, s), S(-0.63, 0.95, s), S(-0.34, 1.15, s), ROOF[0][-1 if s > 0 else 0]],
               lambda s: [ROOF[-1][-1 if s > 0 else 0], S(1.62, 1.16, s), Q[s]["C"]])
    for pil in pillars:
        L, R = pil(1), pil(-1)
        for k in range(len(L) - 1):
            quad(L[k], L[k + 1], R[k + 1], R[k], "window")
    for s in (1, -1):
        ring = lambda a, z: (AX_F + (WHEEL_R + 0.09) * math.cos(a), WHEEL_Y + (WHEEL_R + 0.09) * math.sin(a), s * z)
        for k in range(16):
            a0, a1 = 2 * math.pi * k / 16, 2 * math.pi * (k + 1) / 16
            quad(ring(a0, WHEEL_ZI), ring(a1, WHEEL_ZI), ring(a1, 1.0), ring(a0, 1.0), "wheel")
            out.append(((AX_F, WHEEL_Y, s * WHEEL_ZI), ring(a0, WHEEL_ZI), ring(a1, WHEEL_ZI), "wheel"))   # (its inner face)
    return out


def tri_dist(p, A, B, C):
    """the distances of the points p (m x 3, or one) to the triangles (m x n, or n)"""
    one = np.ndim(p) == 1
    p = np.atleast_2d(p)[:, None, :]
    E0, E1 = B - A, C - A
    n = np.cross(E0, E1)
    n = n / (np.linalg.norm(n, axis=1) + 1e-12)[:, None]
    V = p - A
    h = (V * n).sum(2)
    W = V - n * h[..., None]
    d00, d01, d11 = (E0 * E0).sum(1), (E0 * E1).sum(1), (E1 * E1).sum(1)
    d20, d21 = (W * E0).sum(2), (W * E1).sum(2)
    den = d00 * d11 - d01 * d01 + 1e-18
    v, w = (d11 * d20 - d01 * d21) / den, (d00 * d21 - d01 * d20) / den
    inside = (v >= 0) & (w >= 0) & (v + w <= 1)

    def seg(P0, P1):
        D = P1 - P0
        t = np.clip(((p - P0) * D).sum(2) / ((D * D).sum(1) + 1e-18), 0, 1)
        return np.linalg.norm(p - (P0 + D * t[..., None]), axis=2)
    d = np.where(inside, np.abs(h), np.minimum(np.minimum(seg(A, B), seg(B, C)), seg(C, A)))
    return d[0] if one else d


def first_hit(o, d, A, E1, E2):
    """the ray's first hit of the triangles (inf: none)"""
    P = np.cross(d, E2)
    det = (E1 * P).sum(1)
    ok = np.abs(det) > 1e-12
    inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
    T = o - A
    u = (T * P).sum(1) * inv
    Qv = np.cross(T, E1)
    v = (Qv * d).sum(1) * inv
    t = (Qv * E2).sum(1) * inv
    m = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6)
    return float(t[m].min()) if m.any() else math.inf


FIT_DIRS = np.array([(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1) if (i, j, k) != (0, 0, 0)], float)
FIT_DIRS /= np.linalg.norm(FIT_DIRS, axis=1)[:, None]


def fit_zone_raw(cs, lo, hi, gap_of, seeds=()):
    """a zone's hull fitted from inside to the geometry round it: from each of its middles cs (one, or a few for a
    zone of an awkward shape) a ray each way (the 26 of a cube's corners, edges and faces) out to where it first comes
    nearer a surface than the gap gap_of(what) gives it (None: through it; stepped at 5 mm - a ray between two parts
    slipped through their seam), through none (the openings, the wheels' room) and within the zone's box (lo, hi); of
    them and of seeds (points it must take in: round the members it wraps) the hull's; then, of each face a surface's
    sample (its corners, edges, middles) came behind or nearer than its gap, the corner nearest it pulled in towards
    its middle"""
    surf = [(np.array(a), np.array(b), np.array(cc), gap_of(w)) for a, b, cc, w in FIT_SURF]
    surf = [t for t in surf if t[3] is not None]
    A, B, C = (np.array([t[k] for t in surf]) for k in range(3))
    G = np.array([t[3] for t in surf])
    E1, E2 = B - A, C - A
    cs = [np.array(c, float) for c in (cs if isinstance(cs[0], (tuple, list)) else [cs])]
    lo, hi = np.array(lo, float), np.array(hi, float)
    clear = lambda p: float((tri_dist(p, A, B, C) - G).min())
    pts, own = [], []
    for c in cs:
        assert clear(c) > 0 and np.all(c > lo) and np.all(c < hi), "the zone's middle %s is not clear" % (c,)
        for d in FIT_DIRS:
            with np.errstate(divide="ignore", invalid="ignore"):
                tb = np.where(d > 1e-9, (hi - c) / d, np.where(d < -1e-9, (lo - c) / d, np.inf)).min()
            t = min(tb, first_hit(c, d, A, E1, E2))
            ts = np.arange(0.0, t + 1e-9, 0.005)
            ok = ((tri_dist(c + d * ts[:, None], A, B, C) - G).min(1) >= 0)
            k = int(np.argmin(ok)) if not ok.all() else len(ts)
            pts.append(c + d * (ts[k - 1] if k > 0 else 0.0))
            own.append(c)
    for q in seeds:
        q = np.array(q, float)
        pts.append(q)
        own.append(min(cs, key=lambda c: float(np.linalg.norm(q - c))))
    pts, own = np.array(pts), np.array(own)
    N, D = hull_np(pts)
    on = (np.abs(pts @ N.T - D) < 1e-5).sum(1) >= 3
    pts, own = pts[on], own[on]
    # (the surfaces' samples near the zone: a triangle's 10 points, barycentric thirds)
    bary = np.array([(i / 3, j / 3) for i in range(4) for j in range(4 - i)])
    Sm = (A[:, None, :] + E1[:, None, :] * bary[None, :, 0:1] + E2[:, None, :] * bary[None, :, 1:2]).reshape(-1, 3)
    Sg = np.repeat(G, len(bary))
    near = np.all((Sm > lo - 0.1) & (Sm < hi + 0.1), axis=1)
    Sm, Sg = Sm[near], Sg[near]
    for _ in range(3000):
        N, D = hull_np(pts)
        S_ = Sm @ N.T - D
        sd = S_.max(1)
        bad = sd < 0.9 * Sg - 1e-4
        if not bad.any():
            break
        i = np.flatnonzero(bad)[np.argmin((sd - Sg)[bad])]
        k = int(np.argmax(S_[i]))   # (the face it is nearest behind, or outside nearest to: its corner nearest it in)
        on = np.flatnonzero(np.abs(pts @ N[k] - D[k]) < 1e-5)
        j = on[np.argmin(((pts[on] - Sm[i]) ** 2).sum(1))]
        pts[j] = own[j] + (pts[j] - own[j]) * 0.98
    N, D = hull_np(pts)
    on = (np.abs(pts @ N.T - D) < 1e-5).sum(1) >= 3
    return [tuple(round(float(x), 3) for x in p) for p in pts[on]]


def fit_zone(cs, lo, hi, gap_of, seeds=()):
    """fit_zone_raw, or nothing where this body leaves the zone no room (its middle not clear, no hull)"""
    try:
        return fit_zone_raw(cs, lo, hi, gap_of, seeds)
    except Exception as e:   # noqa
        print("  a zone left out: %s" % (str(e) or type(e).__name__)[:110])
        return None


# (the front wheels' room in the E36's space: the mod's wheel there)
_fw = [w for w in wheels_src if abs(w["c"][0] - AXF_T) < 0.05][0]
WHEEL_R, WHEEL_Y = _fw["R"], TI(tuple(_fw["c"]))[1]
WHEEL_ZI = min(0.62, (abs(_fw["c"][2]) - 0.5 * _fw["w"]) / _KZ - 0.03)   # (the tyre's inner face: a wide wheel's nearer the rail)
if not TUBE and not os.environ.get("BIW_ONLY"):
    FIT_SURF = fit_surfaces()
    is_part = lambda w: not w.startswith("biw ") and w not in ("window", "wheel")
    ENGINE_COL, BOX_COL = (0.16, 0.17, 0.18), (0.42, 0.43, 0.45)
    eng_an = sorted({nearest("biw", p) for s in (1, -1) for p in [(x, LOW_Y(x), s * Z_RAIL) for x in (-1.32, AX_F, -1.58)] + [(X_FW, Y_FLOOR, s * Z_RAIL)]})
    eng_pts = [(x, y, s * z) for x, y, z in ((-1.85, 0.30, 0.13), (-1.20, 0.30, 0.13), (-1.95, 0.50, 0.26), (-1.08, 0.50, 0.26),
                                             (-1.93, 0.78, 0.19), (-1.08, 0.83, 0.19)) for s in (1, -1)]
    VOLUMES.append(("engine", eng_an, eng_pts, 0.12, ENGINE_COL))
    box_an = biw_near(X_FW - 0.05, -0.30, Y_FLOOR - 0.01, 0.60, -Z_RAIL - 0.01, Z_RAIL + 0.01)
    box_pts = [(x, y, s * z) for x, y, z in ((-1.05, 0.26, 0.20), (-1.05, 0.60, 0.20), (-0.22, 0.26, 0.10), (-0.22, 0.42, 0.10)) for s in (1, -1)]
    VOLUMES.append(("gearbox", box_an, box_pts, 0.12, BOX_COL))
    # (the gaps: the parts 2 cm, the body-in-white's sheets 1 cm, the openings and the wheels' room none)
    def gaps(biw=(), wheel=False, part=0.02, side=False):
        return lambda w: part if is_part(w) else 0.01 if w[4:] in biw and w.startswith("biw ") else \
            0.0 if w == "window" or (wheel and w == "wheel") or (side and w == "side") else None
    for s, side in ((1, "left"), (-1, "right")):
        zb = lambda z0, z1: (min(s * z0, s * z1), max(s * z0, s * z1))   # (the side's z range)
        zbox = lambda x0, x1, y0, y1, z0, z1: ((x0, y0, zb(z0, z1)[0]), (x1, y1, zb(z0, z1)[1]))
        cabin = ("floor", "firewall", "roof", "wheelhouse", "quarter", "panel") + BOX_SHELLS
        zones = [  # name, middle, box, gaps, anchors' box, break rms
            ("rail_" + side, [(-1.62, 0.55, s * 0.34), (-1.20, 0.60, s * 0.34)], zbox(-2.14, X_FW - 0.005, 0.22, 1.0, 0.20, 0.62), gaps(("firewall",), wheel=True),
             (-2.20, X_FW + 0.02, 0.15, 0.60, 0.20, 0.50), 0.12),
            ("tower_" + side, [(-1.50, 0.745, s * 0.68), (-1.10, 0.80, s * 0.72), (-1.90, 0.66, s * 0.62)], zbox(-2.14, X_FW - 0.005, 0.50, 1.0, 0.45, 1.0), gaps(("firewall",), wheel=True),
             (-2.20, X_FW + 0.02, 0.50, 0.95, 0.50, 0.85), 0.12),
            ("seat_front_" + side, (-0.25, 0.70, s * 0.42), zbox(X_FW, 0.46, Y_FLOOR, 1.5, 0.0, 1.0), gaps(cabin, side=True), (X_FW - 0.05, 0.50, Y_FLOOR - 0.01, 1.45, 0.0, 0.85), 0.15),
            ("seat_rear_" + side, (0.85, 0.70, s * 0.42), zbox(0.46, 1.25, Y_FLOOR, 1.5, 0.0, 1.0), gaps(cabin, side=True), (0.40, 1.30, Y_FLOOR - 0.01, 1.45, 0.0, 0.85), 0.15),
            ("seat_back_" + side, (1.55, 0.75, s * 0.25), zbox(1.25, 1.95, Y_FLOOR, 1.5, 0.0, 1.0), gaps(cabin, side=True), (1.20, 2.00, Y_FLOOR - 0.01, 1.45, 0.0, 0.85), 0.15),
            ("trunk_" + side, (2.20, 0.62, s * 0.30), zbox(1.82, 2.62, Y_RFLOOR, 1.2, 0.0, 1.0), gaps(cabin), (1.90, 2.60, Y_RFLOOR - 0.01, 0.95, 0.0, 0.85), 0.12),
        ]
        # (the rail zone wraps the front rail's section and the apron up from it, 1 cm beyond its corners, the apron's
        # middle row 1 cm out; the tower the upper rail's flange)
        def box_round(corners):
            out = []
            for k in range(len(corners[0])):
                ps = [np.array(line[k]) for line in corners]
                c = np.mean(ps, axis=0)
                if c[0] > X_FW + 0.01:
                    out += [tuple(float(v) for v in p + (p - c) / np.linalg.norm(p - c) * 0.014) for p in ps]
            return out
        seeds = {"rail": box_round(SEC["frail", s]) + [(x, y, s * (z * s + 0.01)) for x, y, z in MID[s] if x > X_FW + 0.01],
                 "tower": box_round(SEC["urail", s][:2]) + [(x, y + 0.04, s * (z * s + 0.01)) for x, y, z in UP[s] if x > X_FW + 0.01]}
        for name, mid, (lo, hi), gap_of, ab, rms in zones:
            an = biw_near(ab[0], ab[1], ab[2], ab[3], *zb(ab[4], ab[5]))
            if len(an) < 4:   # (no body there: a pickup's behind its cab)
                continue
            if name.startswith("trunk"):
                an = sorted(set(an) | {nearest("biw", p) for p in RP_TOP if p[2] * s >= 0})
            VOLUMES.append((name, an, fit_zone(mid, lo, hi, gap_of, seeds.get(name.split("_")[0], ())), rms, None))
    # the bumpers' reinforcements: a bar over each bumper's beam, from the radiator support's foot (the rear panel) out
    # to 1.5 cm inside the cover, on the beam and its crash boxes' nodes; crushed at 150 kN (a beam with its crash
    # boxes'): then the beam's members take the blow (rigid, two cars' bars met at 28 m/s head-on - 22 MN at a point
    # on six nodes, thousands of strain clamps, the cars sprang apart at 22 km/h)
    VOLUMES.append(("bumper_front", biw_near(-2.25, -2.05, 0.25, 0.40, -0.5, 0.5),
                    fit_zone((-2.20, 0.34, 0.0), (-2.40, 0.25, -0.80), (-2.105, 0.47, 0.80), gaps(part=0.015)), 0.10, None))
    VOLUMES.append(("bumper_rear", biw_near(2.44, 2.56, 0.30, 0.66, -0.5, 0.5),
                    fit_zone((2.56, 0.41, 0.0), (2.50, 0.30, -0.78), (2.75, 0.52, 0.78), gaps(("panel",), part=0.015)), 0.10, None))
    # (each one's clearance to the parts: their nearest nodes, cm; below 0 inside it - not held off)
    VOLUMES[:] = [v for v in VOLUMES if v[2] and len(v[1]) >= 3]
    # (on another body's lines a volume laid out for the E36 may reach into a part - a low hood over the engine: it pushed
    # on it for good, 17 kN as the car drove; each is drawn in about its middle till the parts' nodes, their triangles'
    # middles and edges' are 3 cm clear of it, or left out)
    _pp = [np.array(nodes[n]) for n, g in node_part.items() if g not in ("biw", "susp", "sub")]
    for sh, ts in fem.items():
        for t in ts:
            if node_part[t[0]] not in ("biw", "susp", "sub"):
                q_ = [np.array(nodes[k]) for k in t]
                _pp += [(q_[0] + q_[1] + q_[2]) / 3, (q_[0] + q_[1]) / 2, (q_[1] + q_[2]) / 2, (q_[2] + q_[0]) / 2]
    _pp = np.array(_pp)
    for vi, (name, an, pts, rms, col) in enumerate(VOLUMES):
        P_, ok_ = np.array(pts, float), False
        c_ = P_.mean(axis=0)
        for it in range(22):
            N, D = hull_np(P_)
            if float((_pp @ N.T - D).max(axis=1).min()) >= 0.03:
                ok_ = True
                break
            P_ = c_ + (P_ - c_) * 0.95
        if it:
            print("  %s drawn in to %.0f%%%s" % (name, 100 * 0.95 ** it, "" if ok_ else ": left out"))
        VOLUMES[vi] = (name, an, [tuple(float(x) for x in p) for p in P_] if ok_ else None, rms, col)
    VOLUMES[:] = [v for v in VOLUMES if v[2]]
    for name, an, pts, rms, col in VOLUMES:
        N, D = hull_np(pts)
        gap = {}
        for n, grp in node_part.items():
            if grp in ("biw", "susp"):
                continue
            g = float((N @ np.array(nodes[n]) - D).max())
            gap[grp] = min(gap.get(grp, 1e9), g)
        near = sorted((g, p) for p, g in gap.items() if g < 0.12)
        print("  %s (%d anchors, %d points, %d faces): %s" % (name, len(an), len(pts), len(N), ", ".join("%s %.1f" % (p, g * 100) for g, p in near) or "no part within 12 cm"))

# ---- the loads: the mod's mass, less what the elements, the parts' fittings and the wheels weigh, on the front rails
# (the engine and its gearbox: 40%), the cabin's floor (the crew and the trim: 48%) and the trunk's floor (12%)
DRY_KG = 10.0
WHEEL_KG = 20.0 * (wheels_src[0]["R"] / 0.32) ** 2


def section_kg_m(sec):
    m_, shape, D, t = SECTIONS[sec][:4]
    if shape == "bar":
        return 7850.0 * math.pi / 4 * D * D
    return 7850.0 * ((D * D - (D - 2 * t) ** 2) if shape == "box" else math.pi / 4 * (D * D - (D - 2 * t) ** 2))


SHEET_SHELLS = [name for name in SHELLS if name not in ("lamp", "headlamp", "grille") + BOX_SHELLS]   # (the sheet body lies on these)
_kg = sum(area(t) * (SHELLS[name][1] * DENSITY[SHELLS[name][0]] + (SHEET_KG_M2 if name in SHEET_SHELLS else 0.0)) for name, lst in fem.items() for t in lst)
_kg += sum(dist(nodes[a], nodes[b]) * section_kg_m(sec) for a, b, sec, ja, jb in members)
_kg += sum(load.values()) + 4 * WHEEL_KG + DRY_KG
REST_KG = max(60.0, MASS - _kg)
if TUBE:   # (on the cage's floor lines: the front half, the rear half)
    low_n = [RING[i][k] for i in range(len(CXS)) for k in FLOOR_K]
    xm_ = 0.5 * (AXF_T + AXR_T)
    eng, cabin, trunk_floor = [n for n in low_n if nodes[n][0] < xm_], [n for n in low_n if nodes[n][0] >= xm_], []
else:
    eng = sorted({node(p) for s in (1, -1) for p in LOW[s]})
    cabin = [n for n in BIW_NODES if X_FW < nodes[n][0] < 1.2 and abs(nodes[n][1] - Y_FLOOR) < 1e-6]
    trunk_floor = [n for n in BIW_NODES if nodes[n][0] > 1.9 and abs(nodes[n][1] - Y_RFLOOR) < 1e-6] if not PICKUP else sorted({node(p) for row in floor_rows for p in row})
for group, share in ((eng, 0.50), (cabin, 0.50), (trunk_floor, 0.0)) if TUBE else ((eng, 0.40), (cabin, 0.48), (trunk_floor, 0.12)):
    for v in group:
        load[v] = load.get(v, 0.0) + REST_KG * share / len(group)
print("the mass: the mod's %.0f kg; the elements, the fittings and the wheels %.0f, the rest %.0f on the rails and the floors" % (MASS, _kg, REST_KG))


# ---- the editor's layers (its `;editor-layers:` and `;layer:` comments): the body, its pillars, the suspension, each
# kind of part
LAYERS = ["Body", "Members", "Suspension", "Hood", "Fenders", "Doors", "Trunk", "Bumpers", "Lamps"]
PART_LAYER = {"hood": "Hood", "trunk": "Trunk", "lid": "Trunk", "fender": "Fenders", "door": "Doors", "sash": "Doors", "fascia": "Bumpers", "rear": "Bumpers",
              "bumper": "Bumpers", "lamp": "Lamps", "headlight": "Lamps", "headlamp": "Lamps", "susp": "Suspension", "hinge": "Members", "pillar": "Members",
              "bpillar": "Members", "opening": "Members", "glass": "Members", "doorbar": "Doors", "dglass": "Doors", "grille": "Bumpers"}
layer_of = lambda name: PART_LAYER.get(name.split()[0], "Suspension" if name in SECTIONS else "Body")


class Marks:
    """writes `;layer:` before a line of another layer than the last (from Body at each section's start)"""
    def __init__(self, f):
        self.f, self.cur = f, None

    def section(self):
        self.cur = "Body"

    def __call__(self, layer):
        if layer != self.cur:
            self.f.write(";layer:%s\n" % layer)
            self.cur = layer



# ------------------------------------------------------------------------------------- the mod's graphical model
# each of its meshes a flexbody on the nodes of what it is of (RoR's way: each vertex on the three nodes of its forset
# nearest it): the body-in-white's, or a part's (CARS: forset); its props on the body's nodes near them; the wheels' and
# the suspension's meshes left out
SHOW = bool(os.environ.get("SC_SHOW"))   # (SC_SHOW=1: the physical model drawn - the sheet body, the plates and the members - and no meshes)
NT_ALL = [T(p) for p in nodes]   # (the nodes on the mod's car)
SKIP = tuple(k for k in ("tire", "tyre", "wheel", "rim_", "shock", "spring", "leaf", "damper", "knuckle", "hydro", "caliper") if k not in CFG.get("keep", ())) + tuple(k.lower() for k in CFG.get("skip", ()))


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


def managed_materials():
    out, on = [], False
    for l in SRC_LINES:
        t = l.strip()
        if t == "managedmaterials":
            on = True
            continue
        if on:
            if re.fullmatch(r"[a-z_0-9]+", t) and not t[0].isdigit():
                on = False
                continue
            if t and not t.startswith(";") and t not in out:
                out.append(t)
    return out


def placement(pref, px, py):
    """a flexbody's or a prop's frame on its ref, x and y nodes (VehicleVisual's place, RoR's): the offset's basis
    (X, Y, the normal) and the orientation's (X's direction, the normal, their cross)"""
    pref, X, Y = np.array(pref), np.array(px) - np.array(pref), np.array(py) - np.array(pref)
    n = np.cross(Y, X)
    n /= np.linalg.norm(n)
    rx = X / np.linalg.norm(X)
    return pref, np.column_stack([X, Y, n]), np.column_stack([rx, n, np.cross(rx, n)])


def frame_of(ns):
    """three of the nodes ns for a frame: two far apart, the third far off their line"""
    P = {k: np.array(NT_ALL[k]) for k in ns}
    a = min(ns, key=lambda k: tuple(P[k]))
    b = max(ns, key=lambda k: np.linalg.norm(P[k] - P[a]))
    e = (P[b] - P[a]) / np.linalg.norm(P[b] - P[a])
    c = max(ns, key=lambda k: np.linalg.norm((P[k] - P[a]) - e * np.dot(P[k] - P[a], e)))
    return a, b, c


def replace(pos, M, own):
    """the offset and the rotation (the truck format's: Rz Ry Rx, degrees) that put a mesh at pos, M on the nodes own"""
    po, Bo, Ro = placement(*(NT_ALL[k] for k in own))
    Rm = Ro.T @ M
    ry = -math.asin(max(-1.0, min(1.0, Rm[2, 0])))
    return np.linalg.solve(Bo, pos - po), [math.degrees(a) for a in (math.atan2(Rm[2, 1], Rm[2, 2]), ry, math.atan2(Rm[1, 0], Rm[0, 0]))]


def ranges(ns):
    """a forset's node list as ranges"""
    ns, out = sorted(set(ns)), []
    for k in ns:
        if out and out[-1][1] == k - 1:
            out[-1][1] = k
        else:
            out.append([k, k])
    return ",".join("%d-%d" % (a, b) if b > a else "%d" % a for a, b in out)


by_part = {}
for k_, g_ in node_part.items():
    by_part.setdefault(g_, []).append(k_)
flex_out, flex_skipped = [], []
for f_ in flex_src:
    name = f_["mesh"].lower()
    V = load_obj(os.path.join(MESH_DIR, "%d_%s.obj" % (f_["i"], f_["mesh"])))[0]
    lo_, hi_ = V.min(axis=0), V.max(axis=0)
    in_wheel = any(np.linalg.norm((lo_ + hi_) / 2 - w["c"]) < 0.6 * w["R"] and (hi_ - lo_).max() < 2.6 * w["R"] for w in wheels_src)
    if any(k in name for k in SKIP) or in_wheel:
        flex_skipped.append(f_["mesh"].replace(".mesh", ""))
        continue
    groups = next((g for key, g in CFG["forset"] if key.lower() in name), ["biw"])
    ns = [k for g in groups for k in by_part.get(g, [])]
    if len(ns) < 3:
        ns = by_part["biw"]
    own = frame_of(ns)
    o, r = replace(f_["pos"], f_["M"], own)
    flex_out.append("%d, %d, %d, %.5f, %.5f, %.5f, %.4f, %.4f, %.4f, %s\nforset %s" % (own + tuple(o) + tuple(r) + (f_["mesh"], ranges(ns))))
    P = np.array([NT_ALL[k] for k in ns])
    d = np.array([np.min(np.linalg.norm(P - v, axis=1)) for v in V[::11]])
    print("  %s on %s (%d nodes): its vertices %.2f m from their nearest node at most, %.2f on average" % (f_["mesh"], "+".join(groups), len(ns), d.max(), d.mean()))
prop_out = []
for p_ in props_src:
    name = p_["mesh"].lower()
    if any(k in name for k in SKIP) or any(np.linalg.norm(p_["pos"] - w["c"]) < 0.9 * w["R"] for w in wheels_src):
        continue
    src = SRC_LINES[p_["line"] - 1].strip() if 0 < p_["line"] <= len(SRC_LINES) else ""
    tok = [t.strip() for t in src.split(",")]
    if len(tok) < 10:
        continue
    own = frame_of(sorted(by_part["biw"], key=lambda n: float(np.linalg.norm(np.array(NT_ALL[n]) - p_["pos"])))[:6])
    o, r = replace(p_["pos"], p_["M"], own)
    prop_out.append("%d, %d, %d, %.5f, %.5f, %.5f, %.4f, %.4f, %.4f, %s" % (own + tuple(o) + tuple(r) + (",".join(tok[9:]),)))
    j = p_["line"]   # (its animations: the lines after it)
    while j < len(SRC_LINES) and SRC_LINES[j].strip().startswith("add_animation"):
        prop_out.append(SRC_LINES[j].strip())
        j += 1

# ------------------------------------------------------------------------------------------------------ the file
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as f:
    f.write("%s\n" % CFG["title"])
    f.write(";generated by tools/make_part_car.py from %s: the Shell Car's body-in-white (FEM triangle sheets and members) laid on the\n" % SRC)
    f.write(";mod's lines, the mod's hood, fenders, doors, lid, bumpers and lamps each a sheet of FEM triangles on hinges, latches, clamped\n")
    f.write(";bolts, buffers and stays that let go; collision volumes; the Shell Car's suspension scaled to its wheels, ring tyres;\n")
    f.write(";its look the mod's meshes, flexbodies skinned to the nodes of the parts they are (the sheet body and the plates are\n;there and not drawn)\n")
    f.write(";resources: ../%s\n" % os.path.dirname(SRC))
    mats = managed_materials()
    if mats:
        f.write("managedmaterials\n%s\n" % "\n".join(mats))
    f.write("globals\n;dry mass, cargo mass, cab material (the sheet body over the shells below)\n%.1f, 0.0, sheet/Steel/%.1f/0.004/1%s\n" % (DRY_KG, SHEET_KG_M2, "" if SHOW else "/hidden"))
    f.write("minimass\n0.05\n")
    f.write(";editor-layers:%s\n" % "".join(" %s|1|0" % l for l in LAYERS))
    mark = Marks(f)
    f.write("nodes\n;id, x, y, z, options[, load kg]\n")
    mark.section()
    for i, (x, y, z) in enumerate(NT_ALL):
        mark(layer_of(node_part[i]))
        if i in load:
            f.write("%d, %.4f, %.4f, %.4f, nl, %.2f\n" % (i, x, y, z, load[i]))
        else:
            f.write("%d, %.4f, %.4f, %.4f, n\n" % (i, x, y, z))
    f.write("beams\n;the body-in-white's members and the suspension's (FEM frame elements: a, b, F[, joint at a, joint at b])\n")
    mark.section()
    for sec in SECTIONS:
        lst = [(a, b, ja, jb) for a, b, s_, ja, jb in members if s_ == sec]
        if not lst:
            continue
        spec = SECTIONS[sec]
        extra = ", 0, %.0f, %.1f" % spec[5:7] if len(spec) > 5 else ""
        f.write(";%s\nset_beam_defaults 3000000, 400, 80000, 700000, 0.05, tracks/beam, 0\nset_frame_section %s, %s, %.4f, %.4f, %s%s\n" % (
            sec, spec[0], spec[1], spec[2], spec[3], spec[4], extra))
        mark(layer_of(sec))
        opt = "F" if sec in ("arm", "tierod", "rack", "upright") or (SHOW and sec not in HIDDEN_SECS) else "Fi"   # (i: not drawn - the body's all, the hubs' stubs through the rims)
        for a, b, ja, jb in lst:
            if ja or jb:
                f.write("%d, %d, %s, %s, %s\n" % (a, b, opt, ja or spec[4], jb or spec[4]))
            else:
                f.write("%d, %d, %s\n" % (a, b, opt))
    # the coil-overs: the corner's load on a spring of 1.6 Hz at the wheel (damped at 0.4 of critical), preloaded to
    # stand at the design length; a bump stop and a droop strap beside each; the steering's and the rear toe's stops
    sprung = MASS - 4 * WHEEL_KG
    f.write("shocks\n;n1, n2, spring, damp, short bound, long bound, precompression, options\n")
    mark.section(), mark("Suspension")
    f.write("set_beam_defaults 9000000, 12000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, front, k in shocks:
        L0 = dist(nodes[a], nodes[b])
        corner = sprung * 9.81 * (0.55 if front else 0.45) / 2
        m = corner / 9.81
        ks = m * (2 * math.pi * (1.4 if wheels_src[0]["R"] > 0.42 else 1.6)) ** 2
        f.write("%d, %d, %.0f, %.0f, 0.5, 0.5, %.3f, n\n" % (a, b, ks, 2 * 0.40 * math.sqrt(ks * m), 1 + corner / (ks * L0)))
    f.write("set_beam_defaults 20000000, 200000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, front, k in shocks:
        L0 = dist(nodes[a], nodes[b])
        f.write("%d, %d, 0, 0, %.3f, %.3f, 1.0, i\n" % (a, b, 0.14 * k / L0, 0.09 * k / L0))
    f.write("set_beam_defaults 3000000, 20000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, lo_, hi_ in stops:
        f.write("%d, %d, 0, 0, %.3f, %.3f, 1.0, i\n" % (a, b, lo_, hi_))
    f.write("hydros\n;the steering rack: node1, node2, factor, options\n")
    mark.section(), mark("Suspension")
    f.write("set_beam_defaults 4000000, 2000, 99999999999999999999999999999999999999999, 99999999999999999999999999999999999999999, 0.02, tracks/beam, 0\n")
    for a, b, fac in hydros:
        f.write("%d, %d, %.4f, i\n" % (a, b, fac))
    f.write("slidenodes\n;the rack's ends slide along its housing\n")
    for n in slides:
        f.write("%d, %s, S2000000, T0\n" % (n, ", ".join(str(h) for h in HOUSING)))
    f.write("ringwheels\n;(BeamLab) radius, rim radius, width, node1, node2, braking, propulsion, arm, mass, tyre stiffness (N/m, 2 cm in), damping (N s/m), grip[, side, rim mesh]\n")
    mark.section(), mark("Suspension")
    for n1, n2, top, w in wheels:
        rim = w["rim"] if 0.4 * w["R"] < w["rim"] < 0.9 * w["R"] else 0.66 * w["R"]
        f.write("%.3f, %.3f, %.3f, %d, %d, 1, %d, %d, %.1f, %.0f, %.0f, 1.0%s\n" % (
            w["R"], rim, min(max(w["w"], 0.18), 0.45), n1, n2, 1 if w["drive"] else 0, top, WHEEL_KG, MASS * 9.81 / 4 / 0.02, 1000.0 * math.sqrt(MASS / 1250.0),
            ", %s, %s" % (w["side"], w["mesh"]) if w["mesh"] != "-" else ""))
    f.write("engine\n%s\n" % (section_lines("engine") or ["900.0, 6500.0, 280.0, 3.15, 3.7, 1.0, 4.2, 2.5, 1.66, 1.22, 1.0, -1.0"])[0])
    f.write("engoption\n%s\n" % (section_lines("engoption") or ["0.02, c, 1000.0, 0.3, 0.4, 0.3"])[0])
    f.write("brakes\n%s\n" % (section_lines("brakes") or ["4000"])[0])
    f.write("cameras\n%d, %d, %d\n" % CAMERA)
    cine = section_lines("cinecam")
    cp = [float(x) for x in cine[0].split(",")[:3]] if cine else list(T((0.15, 1.05, 0.35)))
    f.write("cinecam\n%.3f, %.3f, %.3f, %s\n" % (cp[0], cp[1], cp[2], ", ".join(str(n) for n in sorted(BIW_NODES, key=lambda n: float(np.linalg.norm(np.array(NT_ALL[n]) - np.array(cp))))[:8])))
    f.write("contacters\n")
    for i in range(len(nodes)):
        f.write("%d\n" % i)
    if VOLUMES:
        f.write("collision_volumes\n;(BeamLab) volume name, break rms (m) - its anchors (frame nodes) - its hull's points (x, y, z)\n")
        for name, an, pts, rms, col in VOLUMES:
            crush = CRUSH.get(name, 0.0)
            f.write("volume %s, %.2f%s\nanchors %s\n" % (name, rms, ", %.0f" % crush if crush else "", ", ".join(str(a) for a in an)))
            for p in pts:
                f.write("vertex %.3f, %.3f, %.3f\n" % T(p))
            if col and SHOW:
                f.write("color %.2f, %.2f, %.2f\n" % col)
    if mounts:
        f.write("mounts\n;node a (the body's), node b (the part's), break force N, stiffness N/m (0: the step's), turning damping N m s/rad, kind:\n")
        f.write(";h a hinge (its second node on the line), p a latch, c a clamped bolt (its break moment N m), s a buffer, r a stay (its length)\n")
        for a, b, brk, kind, param, k in mounts:
            tail = {"h": ", h, %d" % (param if kind == "h" else 0), "p": ", p", "c": ", c, %.0f" % param, "s": ", s", "r": ", r, %.3f" % param}[kind]
            f.write("%d, %d, %.0f, %.0f, %.1f%s\n" % (a, b, brk, k, 2.0 if kind == "h" else 0.0, tail))
    f.write("shells\n;the sheet body's triangles (their FEM triangles' collision triangles): all but the lamps' and the grille's\n")
    for name in SHELLS:
        if name not in ("bumper", "lamp", "headlamp", "grille") + BOX_SHELLS:
            for a, b, c in fem.get(name, []):
                f.write("%d, %d, %d\n" % (a, b, c))
    f.write("set_shell_material bumpers, Steel, %.1f, 0.004, %.2f, %.2f, %.2f, 1\n" % ((SHEET_KG_M2,) + BLACK))
    for a, b, c in fem.get("bumper", []):
        f.write("%d, %d, %d\n" % (a, b, c))
    f.write("set_shell_material default\n")
    f.write("fem_tris\n;the body-in-white's sheets and the parts: triangle elements of the FEM frame (n1, n2, n3) of the shell set before them\n")
    mark.section()
    for name, (mat, th, col) in SHELLS.items():
        if not fem.get(name):
            continue
        f.write(";%s\nset_fem_shell %s, %.4f%s\n" % (name, mat, th, ", %.2f, %.2f, %.2f" % col if col else ""))
        mark(layer_of(name))
        for a, b, c in fem[name]:
            f.write("%d, %d, %d\n" % (a, b, c))
    if SHOW:
        flex_out, prop_out = [], []
    f.write("flexbodies\n;the mod's meshes, each on its part's nodes: ref, x, y, offset, rotation, mesh - forset: the nodes it is skinned to\n")
    f.write("\n".join(flex_out) + "\n" if flex_out else "")
    if prop_out:
        f.write("props\n" + "\n".join(prop_out) + "\n")
    f.write("end\n")

kg_tri = {name: sum(area(t) for t in lst) * SHELLS[name][1] * DENSITY[SHELLS[name][0]] for name, lst in fem.items()}
biw_tris = sum(len(v) for k, v in fem.items() if k in BIW_SHELLS)
susp_secs = ("arm", "tierod", "rack", "hub", "upright")
part_secs = ("sash", "doorbar", "dglass")
print("FEM sheets " + ", ".join("%s %.0f" % (k, v) for k, v in sorted(kg_tri.items(), key=lambda x: -x[1])) + " kg")
print("wrote %s: %d nodes (%d of the body), the body-in-white %d members and %d triangles, the suspension %d members; %d FEM triangles of "
      "the parts and %d members, %d mounts (%s), %d parts (%s), %d collision volumes; %d meshes, %d props; skipped: %s" % (
          OUT, len(nodes), len(BIW_NODES), sum(1 for m in members if m[2] not in susp_secs + part_secs), biw_tris, sum(1 for m in members if m[2] in susp_secs),
          sum(len(v) for v in fem.values()) - biw_tris, sum(1 for m in members if m[2] in part_secs), len(mounts),
          ", ".join("%s %d" % (k, sum(1 for m in mounts if m[3] == k)) for k in "hpcsr"), len(parts), ", ".join(p.name for p in parts), len(VOLUMES),
          len(flex_out), len([p for p in prop_out if not p.startswith("add_")]), ", ".join(flex_skipped)))
