#!/usr/bin/env python3
"""Writes assets/vehicles/shell_car/shell_car.truck: a saloon on the BMW E36's lines, all of FEM frame members and FEM
triangle elements (no sheets).

The body-in-white is a thin spatial structure: the load-carrying lines are members (phys::FemFrame's elements, box
sections of steel) - the sills, the hinge, A, B and C pillars, the roof rails and three roof bows, the cowl, the floor's
cross members, the front rails with the upper rails along the fenders' line and the radiator support, the rear rails,
the bumpers' beams on crash boxes, the rear doors' posts, the arches' lips, the quarters' tops, the rear panel's top -
and the surfaces between them single sheets of triangle elements (phys::FrameTri): the floor, the firewall, the roof,
the aprons, the rear wheelhouses, the quarter panels, the parcel shelf, the rear floor and the rear panel. Members and
triangles share their nodes.

The hood, the front fenders, the doors, the trunk lid (an L: its top and its rear face down to the tail lights' foot),
the bumpers and the tail lights are the E36's (its flexbody meshes remeshed: rays cast on structured grids, the
bumpers' round their outlines), each one sheet of FEM triangles - steel, the bumpers and the lamps polypropylene. Each
is a part on the body on mounts (phys::FrameMount) that let go: the hood's, the trunk lid's and the doors' hinges (two
of the part's nodes on the hinge's line held: the part turns about it only), the latches (a point), the fenders',
bumpers' and lamps' bolts (clamps: the part's nodes round the bolt held, so a part left on one bolt does not swing about
it - past a break moment the bolt lets go), the hood's and the lid's buffers (stops: with the latch gone the lid rests
on them, it does not fall in) and opening stays (straps). No glass. The quarters' feet behind the rear wheels and the
rear panel's corners are tucked in under the rear bumper's wrap.

The suspension is of FEM members in the Frame Car's layout, stiff: double wishbones of tube on ball joints at the body
(the lower ones 0.45 m long on the subframes' cross members between the rails, the upper ones on the aprons in front
and on the wheelhouses at the rear, in frames of members round their pivots), the upright (the stub and a steering arm
level with the lower ball joint, welded together) on ball joints inside the wheel, a coil-over from the lower ball
joint up to the upper rail or the wheelhouse's top with a bump stop and a droop strap (9 cm); in front a rack (a bar of
tube whose ends slide along its housing, moved by one hydro) and tie rods of tube on ball joints, at the rear toe links
on brackets - their inner ends placed for no bump steer; stops at the steering's lock and the rear toe. Maraging steel,
sized to stay elastic through every test. Rear-wheel drive, 280 N m (a 328i's), the engine's inertia a real one's
(0.2 kg m2 with its flywheel: RoR's engoption figure is taken in rpm, 0.10 there was ten times that - in first gear
as much again as the car's own mass to spin up); tyres of 24 rays, stiffer and less damped than the Frame Car's (their
rolling resistance a third less).

Three collision volumes (phys::CollisionVolume: convex hulls riding on the body-in-white's nodes) stand for what fills
the car - the engine in its bay, the seats and occupants in the cabin, the trunk's load - and keep other bodies out
once the panels give.

The E36's meshes placed in its definition space as .obj files, from the game:

    BL_EXPORT_FLEX=<dir> ./build/beamlab --scene proving --vehicle bmw_e36/E36Sedan --frames 2 --hidden --screenshot x.png
    python3 tools/make_shell_car.py <dir>          (BIW_ONLY=1: the body-in-white alone; NO_WHEELS=1: no wheels)

Coordinates are Rigs of Rods' (and the E36's): -x forward, y up, +z left.
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "assets", "vehicles", "shell_car", "shell_car.truck")
MESH_DIR = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("E36_OBJ", "")


# ------------------------------------------------------------------------------------------------ the E36's meshes
def load_obj(path):
    V, F = [], []
    for line in open(path):
        if line.startswith("v "):
            V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            F.append([int(x.split("/")[0]) - 1 for x in line.split()[1:4]])
    return np.array(V), np.array(F)


class Mesh:
    def __init__(self, names):
        Vs, Fs, off = [], [], 0
        for n in names:
            fn = [f for f in os.listdir(MESH_DIR) if f.endswith("_" + n + ".mesh.obj")]
            assert fn, "no %s in %s" % (n, MESH_DIR)
            V, F = load_obj(os.path.join(MESH_DIR, fn[0]))
            Vs.append(V), Fs.append(F + off)
            off += len(V)
        V, F = np.concatenate(Vs), np.concatenate(Fs)
        self.A = V[F[:, 0]]
        self.E1 = V[F[:, 1]] - self.A
        self.E2 = V[F[:, 2]] - self.A

    def hits(self, o, d):
        o, d = np.asarray(o, float), np.asarray(d, float)
        P = np.cross(d, self.E2)
        det = (self.E1 * P).sum(1)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        T = o - self.A
        u = (T * P).sum(1) * inv
        Q = np.cross(T, self.E1)
        v = (Q * d).sum(1) * inv
        t = (Q * self.E2).sum(1) * inv
        m = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0)
        return np.sort(t[m])

    def first(self, o, d):
        h = self.hits(o, d)
        return float(h[0]) if len(h) else None


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
        for j in range(nv):
            if ts[j] is None:
                lo = max((k for k in known if k < j), default=None)
                hi = min((k for k in known if k > j), default=None)
                ts[j] = ts[lo] if hi is None else ts[hi] if lo is None else lerp(ts[lo], ts[hi], (j - lo) / (hi - lo))
        rows.append([tuple(np.asarray(origin(u, v)) + d * t) for v, t in zip(row_v, ts)])
    return rows


def scan_round(mesh, cx, ys, n, back):
    """a bumper's rows of n points round its outline in plan: per y rays from (cx, y, 0) out to the tail (back 1) or the
    nose (back -1) and round to the sides, the last hit each (its outer face), evenly in angle over the span it covers -
    the corners' wrap resolved as a scan along x does not"""
    rows = []
    for y in ys:
        d = lambda a: (back * math.cos(math.radians(a)), 0.0, math.sin(math.radians(a)))
        hit = lambda a: (lambda h: float(h[-1]) if len(h) else None)(mesh.hits((cx, y, 0.0), d(a)))
        span = [a for a in range(-90, 91) if hit(a) is not None]
        row = []
        for j in range(n):
            a = lerp(float(span[0]), float(span[-1]), j / (n - 1))
            a, t = next((a + e, hit(a + e)) for e in (0, 1, -1, 2, -2, 3, -3, 4, -4) if hit(a + e) is not None)   # (a hole: the nearest hit)
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
WHEEL_R, WHEEL_W = 0.32, 0.22
AX_F, AX_R = -1.44, 1.57        # the E36's axles in its definition space (its wheels' meshes)
WHEEL_Y = 0.26                  # (the E36's 0.22 and its sag)
Y_FLOOR, Y_RFLOOR = 0.22, 0.34  # the cabin's floor, the rear floor over the axle
X_FW = -0.96                    # the firewall
Y_COWL = 0.86                   # the firewall's top, the A pillars' feet (under the hood's rear edge)
Z_RAIL = 0.38                   # the front and rear rails
X_REAR = 2.47                   # the rear panel's foot (13 cm inside the bumper's middle)
Z_PIVOT = 0.22                  # the lower wishbones' inner pivots, on the subframes' cross members
Z_BALL = 0.67                   # the ball joints, 5 cm inside the wheel's inner face (the rim's offset)
RACK_X, RACK_Y = AX_F + 0.18, 0.293   # the steering rack (the steering arms behind the front axle), at its tie rods' inner ends' height
SIDE_Y = [0.20, 0.27, 0.48, 0.62, 0.77, 0.92, 1.09, 1.26, 1.37]    # the E36's body side in section (its chassis mesh)
SIDE_Z = [0.80, 0.82, 0.86, 0.86, 0.84, 0.80, 0.72, 0.63, 0.55]


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


chassis = Mesh(["E36_CHASSIS"]) if MESH_DIR else None


def roof_y(x, z):
    """the E36's roof over (x, z), 5 mm under its skin"""
    if chassis is not None:
        t = chassis.first((x, 3.0, z), (0, -1, 0))
        if t is not None and 3.0 - t > 1.25:
            return 3.0 - t - 0.005
    return 1.37 + 0.05 * (1.0 - (z / 0.55) ** 2)


def interp_y(x, xs, ys):
    return float(np.interp(x, xs, ys))


# ---------------------------------------------------------------------------------- the body-in-white: surfaces
FX = [X_FW, -0.40, 0.46, 1.18]                       # the cabin floor's stations: the firewall, the seats, the B pillars, the heel kick
FZ = [-z_side(0, Y_FLOOR), -Z_RAIL, 0.0, Z_RAIL, z_side(0, Y_FLOOR)]
grid([[(x, Y_FLOOR, z) for z in FZ] for x in FX], "floor")
# the firewall: the floor to the cowl, out to the hinge pillars
FW_Y = [Y_FLOOR, 0.56, Y_COWL]
grid([[(X_FW, y, z) for z in [-z_side(X_FW, y), -Z_RAIL, 0.0, Z_RAIL, z_side(X_FW, y)]] for y in FW_Y], "firewall")
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
def quarter(s):
    P, C, T1, T2 = S(1.55, 0.92, s), S(1.80, 0.92, s), S(2.05, 0.92, s), S(X_REAR, 0.92, s)
    A1, A2, A3, A4 = S(1.25, 0.52, s), S(AX_R, 0.68, s), S(1.89, 0.52, s), S(1.97, Y_RFLOOR, s)
    M1, M2, B1, B2 = S(2.25, 0.62, s), S(X_REAR, 0.62, s), (2.25, Y_RFLOOR, s * z_low(2.25)), (X_REAR, Y_RFLOOR, s * z_low(X_REAR))
    L = S(X_REAR, 0.77, s)   # (the tail light's middle on the rear edge)
    for t in ((A1, P, A2), (A2, P, C), (A2, C, A3), (A3, C, T1), (A3, T1, A4), (A4, T1, M1), (A4, M1, B1), (T1, T2, M1), (M1, T2, L), (M1, L, M2),
              (B1, M1, M2), (B1, M2, B2)):
        ftri(*(node(p) for p in t), "quarter")
    return dict(P=P, C=C, T1=T1, T2=T2, A1=A1, A2=A2, A3=A3, A4=A4, M1=M1, M2=M2, B1=B1, B2=B2, L=L)


Q = {s: quarter(s) for s in (1, -1)}
# the rear panel: between the quarters, the floor (behind the bumper) to the trunk's opening
grid([[Q[-1]["B2"], (X_REAR, Y_RFLOOR, -Z_RAIL), (X_REAR, Y_RFLOOR, 0.0), (X_REAR, Y_RFLOOR, Z_RAIL), Q[1]["B2"]],
      [Q[-1]["M2"]] + RP_TOP + [Q[1]["M2"]]], "panel")
# the parcel shelf: between the C pillars' feet and the trunk's opening
PZ = [-z_side(1.9, 0.92), -Z_RAIL, 0.0, Z_RAIL, z_side(1.9, 0.92)]
grid([[(x, 0.92, z) for z in PZ] for x in (1.80, 2.05)], "panel")
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
# (their middle row carries the upper wishbones' pivots, the upper rail the shocks' tops)
AP_X = [X_FW, -1.32, AX_F, -1.58, -1.80, -2.10]
LOW_Y = lambda x: interp_y(-x, [0.96, 1.44, 1.80, 2.10], [Y_FLOOR, 0.30, 0.32, 0.32])
UP_YZ = lambda x: (interp_y(-x, [0.96, 1.44, 1.80, 2.08], [Y_COWL, 0.74, 0.68, 0.62]), interp_y(-x, [0.96, 1.44, 1.80, 2.08], [z_side(X_FW, Y_COWL), 0.68, 0.66, 0.62]))
LOW = {s: [(x, LOW_Y(x), s * Z_RAIL) for x in AP_X] for s in (1, -1)}
MID = {s: [(X_FW, 0.56, s * Z_RAIL)] + [(x, 0.52, s * 0.40) for x in AP_X[1:]] for s in (1, -1)}
UP = {s: [S(X_FW, Y_COWL, s)] + [(max(x, -2.08), UP_YZ(x)[0], s * UP_YZ(x)[1]) for x in AP_X[1:]] for s in (1, -1)}
for s in (1, -1):
    grid([LOW[s], MID[s], UP[s]], "apron", 0 if s > 0 else 1)

# ------------------------------------------------------------------------------------ the body-in-white: members
SECTIONS = {  # material, shape, outer (m), wall (m), joints, [break force N, joint damping N m s/rad]
    "sill": ("Steel", "box", 0.110, 0.0018, "rigid"),      # the sills
    "hinge": ("Steel", "box", 0.100, 0.0020, "rigid"),     # the hinge pillars (the front doors' hinges, the fenders' bolts)
    "bpillar": ("Steel", "box", 0.080, 0.0018, "rigid"),   # the B pillars
    "pillar": ("Steel", "box", 0.070, 0.0016, "rigid"),    # the A and C pillars
    "rail": ("Steel", "box", 0.060, 0.0015, "rigid"),      # the roof rails, the front upper rails, the radiator support
    "bow": ("Steel", "box", 0.050, 0.0012, "rigid"),       # the roof bows
    "cross": ("Steel", "box", 0.060, 0.0015, "rigid"),     # the cowl, the floor's cross members, the parcel shelf's, the rear panel's top
    "frail": ("Steel", "box", 0.090, 0.0018, "rigid"),     # the front rails and the crash beam between their tips
    "rrail": ("Steel", "box", 0.080, 0.0016, "rigid"),     # the rear rails
    "edge": ("Steel", "box", 0.040, 0.0012, "rigid"),      # the arches' lips, the rear doors' posts, the quarters' tops
    "sash": ("Steel", "box", 0.030, 0.0012, "rigid"),      # the doors' window frames and belt rails
    "subframe": ("Steel", "box", 0.080, 0.0025, "rigid"),  # the subframes' cross members (the lower wishbones' pivots)
    "tower": ("Steel", "box", 0.060, 0.0020, "rigid"),     # the frames round the upper wishbones' pivots
    "bbeam": ("Steel", "box", 0.070, 0.0020, "rigid"),     # the bumpers' beams and their crash boxes
    # the suspension: maraging steel (1.9 GPa), sized to stay elastic through every test (35 x 3 chromoly bent on a
    # rough field and folded in the crashes; 50 x 8 spring steel uprights bent a little in the head-on, the wheel
    # struck by the other car: ~300 kN through it)
    "arm": ("Maraging", "tube", 0.045, 0.0050, "rigid", 0.0, 3.0),     # wishbones, toe links
    "tierod": ("Maraging", "tube", 0.034, 0.0060, "ball"),              # tie rods
    "rack": ("Maraging", "tube", 0.036, 0.0060, "rigid"),               # the rack bar and its arms
    "upright": ("Maraging", "tube", 0.070, 0.0140, "rigid", 0.0, 3.0),  # the uprights between the ball joints and the hub
    "hub": ("Maraging", "tube", 0.055, 0.0100, "rigid", 0.0, 3.0),      # the stub axles and the steering arms
}
for s in (1, -1):
    chain([(x, Y_FLOOR, s * z_side(0, Y_FLOOR)) for x in FX], "sill")
    chain([(X_FW, y, s * z_side(X_FW, y)) for y in FW_Y], "hinge")
    ROOF_S = [ROOF[i][-1 if s > 0 else 0] for i in range(len(RX))]
    chain(ROOF_S, "rail")
    chain([S(X_FW, Y_COWL, s), S(-0.63, 0.95, s), S(-0.34, 1.15, s), ROOF_S[0]], "pillar")          # A (the E36's line: the door frame's)
    chain([S(0.46, y, s) for y in (Y_FLOOR, 0.56, 0.92, 1.16)] + [ROOF_S[2]], "bpillar")               # B
    chain([ROOF_S[4], S(1.62, 1.16, s), Q[s]["C"]], "pillar")                                          # C
    chain([S(1.18, Y_FLOOR, s), Q[s]["A1"], Q[s]["P"]], "edge")                                        # the rear door's post
    chain([Q[s]["A1"], Q[s]["A2"], Q[s]["A3"], Q[s]["A4"]], "edge")                                    # the arch's lip
    chain([Q[s]["P"], Q[s]["C"], Q[s]["T1"], Q[s]["T2"]], "edge")                                      # the quarter's top
    chain([Q[s]["T2"], Q[s]["L"], Q[s]["M2"], Q[s]["B2"]], "edge")                                     # its rear edge
    chain(sorted(LOW[s] + [(RACK_X, RACK_Y, s * Z_RAIL)], key=lambda p: -p[0]), "frail")             # (the rack's housing on it)
    chain(UP[s], "rail")
    member(LOW[s][-1], UP[s][-1], "rail")                                                              # the radiator support's post
    chain(sorted([(x, rail_y(x), s * Z_RAIL) for x in RFX + [1.35, 1.45, 1.72]], key=lambda p: p[0]), "rrail")
for i in range(len(RX)):
    if RX[i] in (RX[0], 0.46, 1.45):
        chain(ROOF[i], "bow")
chain([(X_FW, Y_COWL, z) for z in [-z_side(X_FW, Y_COWL), -Z_RAIL, 0.0, Z_RAIL, z_side(X_FW, Y_COWL)]], "cross")   # the cowl
for x in (X_FW, 0.46, 1.18):
    chain([(x, Y_FLOOR, z) for z in FZ], "cross")                                                      # the floor's cross members
chain([(1.80, 0.92, z) for z in PZ], "cross")                                                          # the rear window's base
chain([(2.05, 0.92, z) for z in PZ], "cross")                                                          # the trunk opening's front
chain([Q[-1]["M2"]] + RP_TOP + [Q[1]["M2"]], "cross")                                                 # the rear panel's top
chain([(-2.08, 0.62, -0.62), (-2.08, 0.62, 0.0), (-2.08, 0.62, 0.62)], "rail")                          # the radiator support's top
chain([(-2.10, 0.32, -Z_RAIL), (-2.10, 0.32, 0.0), (-2.10, 0.32, Z_RAIL)], "frail")                     # the radiator support's foot
# the bumpers' beams on short crash boxes from the rails' ends, 2-3 cm inside the covers
for x0, y0, x1, y1 in ((-2.10, 0.32, -2.19, 0.32), (X_REAR, Y_RFLOOR, 2.52, 0.38)):
    for s in (1, -1):
        member((x0, y0, s * Z_RAIL), (x1, y1, s * Z_RAIL), "bbeam")
    chain([(x1, y1, -Z_RAIL), (x1, y1, 0.0), (x1, y1, Z_RAIL)], "bbeam")
BIW_NODES = {n for n, g in node_part.items() if g == "biw"}

# ------------------------------------------------------------------------------------------------ the suspension
# per wheel: the lower and upper ball joints (bl, bu: inside the wheel), the hub's inner and outer nodes (n1, n2), the
# steering or toe arm (sa: level with the lower ball joint, braced to it); the lower wishbone's pivots on a subframe's
# cross members between the rails (long arms: 0.45 m), the upper one's on the apron or the wheelhouse (0.28 m), in a
# frame of members round them (at the rear triangulated down to the lower pivots); the coil-over's top (the wheel's arm
# node). The tie rods' and the toe links' inner ends where their arcs follow the steering arms' through the travel (a
# kinematic sweep of the linkage): no bump steer (under 0.05 degrees over +-10 cm; the E36's own rear: a lateral link),
# camber -3 degrees at full droop, +5 (the top in) 9 cm up
SUB_X = {True: (-1.32, -1.58), False: (1.45, 1.72)}
TOE_IN = {True: (RACK_Y, 0.209), False: (0.327, 0.205)}   # (y, z): the tie rods' inner ends on the rack, the toe links' on brackets
for front, xs in SUB_X.items():
    yl = (lambda x: LOW_Y(x)) if front else rail_y
    for x in xs:
        chain([(x, yl(x), z) for z in (-Z_RAIL, -Z_PIVOT, 0.0, Z_PIVOT, Z_RAIL)], "subframe")
    for s in (1, -1):
        mids = [p for p in (MID[s] if front else WH_MID[s]) if p[0] in xs]
        tops = [p for p in (UP[s] if front else WH_TOP[s]) if p[0] in xs]
        member(mids[0], mids[1], "tower")
        for x, m_, t_ in zip(xs, mids, tops):
            member((x, yl(x), s * Z_RAIL), m_, "tower"), member(m_, t_, "tower")
            if not front:
                member((x, yl(x), s * Z_PIVOT), m_, "tower")
        if not front:   # the toe link's bracket on the front cross member
            x = xs[0]
            for z in (0.0, Z_RAIL):
                member((x, yl(x), s * z), (x, TOE_IN[False][0], s * TOE_IN[False][1]), "tower")
HOUSING = [(RACK_X, RACK_Y, z) for z in (-Z_RAIL, 0.0, Z_RAIL)]
chain(HOUSING, "subframe")                                                 # the rack's housing across the rails
rack_ends = {}
for wx in (AX_F, AX_R):
    front = wx < 0
    for s in (1, -1):
        arm_x = wx + 0.18 if front else wx - 0.18
        bl, bu = node((wx, 0.12, s * Z_BALL), "susp"), node((wx, 0.50, s * Z_BALL), "susp")
        n1, n2 = node((wx, WHEEL_Y, s * 0.62), "susp"), node((wx, WHEEL_Y, s * 0.84), "susp")
        sa = node((arm_x, 0.12, s * (Z_BALL + 0.02)), "susp")
        yl = (lambda x: LOW_Y(x)) if front else rail_y
        lower = [node((x, yl(x), s * Z_PIVOT)) for x in SUB_X[front]]
        upper = [node(p) for p in (MID[s] if front else WH_MID[s]) if p[0] in SUB_X[front]]
        top = node(UP[s][2] if front else WH_TOP[s][2])
        assert len(lower) == 2 and len(upper) == 2
        for p in lower:
            member_n(p, bl, "arm", ja="ball")
        for p in upper:
            member_n(p, bu, "arm", ja="ball")
        member_n(bl, n1, "upright", ja="ball")
        member_n(n1, bu, "upright", jb="ball")
        member_n(n1, n2, "hub")
        member_n(n1, sa, "hub")
        member_n(bl, sa, "hub", ja="ball")
        shocks.append((bl, top, front))
        wheels.append((n1, n2, top, front))
        kp = np.array(nodes[bl])   # (the steering axis: upright through the ball joints)
        r = np.array(nodes[sa]) - kp
        if front:
            # the rack's end (it slides along the housing), its arm out to the tie rod's inner end
            re = node((RACK_X, RACK_Y, s * 0.12), "susp")   # (the rack's arm along its axis: turning the bar about it moves no tie rod)
            ti = node((RACK_X, TOE_IN[True][0], s * TOE_IN[True][1]), "susp")
            load[re] = load[ti] = 3.0
            rack_ends[s] = re
            member_n(re, ti, "rack")
            member_n(ti, sa, "tierod")
            slides.append(re)
            end, lim = node(HOUSING[2 if s > 0 else 0]), 34.0    # the steering stop: free through the lock and 4 degrees past it
        else:
            member_n(sa, node((SUB_X[False][0], TOE_IN[False][0], s * TOE_IN[False][1])), "arm", ja="ball", jb="ball")   # the toe link
            end, lim = lower[1], 6.0                                 # the toe stop: 6 degrees each way
        L0 = dist(nodes[sa], nodes[end])
        ds = []
        for deg in (-lim, lim):
            c, sn = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            p = kp + np.array((r[0] * c - r[2] * sn, r[1], r[0] * sn + r[2] * c))
            ds.append(float(np.linalg.norm(p - np.array(nodes[end]))))
        stops.append((sa, end, (L0 - min(ds)) / L0, (max(ds) - L0) / L0))
member_n(rack_ends[1], rack_ends[-1], "rack")   # the rack bar
RACK_TRAVEL = 0.18 * math.sin(math.radians(30.0))
# (negative: the hydro shortens for a right turn - the rack goes right and the arms behind the axle turn the wheels'
# fronts right)
hydros.append((rack_ends[1], node(HOUSING[0]), -RACK_TRAVEL / (Z_RAIL + 0.12)))


# ---------------------------------------------------------------------------------- the hang-on parts (the E36's)
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
        """a door's window frame (the E36's: its door mesh's outline over the belt, 1 cm inside its face) from its top
        row's front and rear nodes, and a belt rail along that row"""
        edge = []
        for y in (1.05, 1.20, 1.35):
            xs = [x for x in np.arange(-1.0, 2.0, 0.01) if len(mesh.hits((x, y, s * 2.0), (0, 0, -s)))]
            xs = [x for x in xs if abs(x - np.mean(xs)) < 0.8]
            edge.append([(x, y, s * (2.0 - mesh.first((x, y, s * 2.0), (0, 0, -s)) - 0.01)) for x in (xs[0] + 0.015, xs[-1] - 0.015)])
        top = self.fem[-1]
        pts = [nodes[top[0]]] + [e[0] for e in edge] + [e[1] for e in edge[::-1]] + [nodes[top[-1]]]
        chain_n = [top[0]] + [node(p, self.name) for p in pts[1:-1]] + [top[-1]]
        for a, b in zip(chain_n, chain_n[1:]):
            member_n(a, b, "sash")
        for a, b in zip(top, top[1:]):
            member_n(a, b, "sash")
        # the door shut: its edges on the opening's flanges (stops, stiff) - the bottom on the sill, the hinge and lock
        # sides on the pillars, the frame's top corners on the roof rail - so a car on its side or struck there does not
        # hang on the hinges and the latch
        rim = {(0, j) for j in range(len(self.fem[0]))} | {(i, j) for i in range(len(self.fem)) for j in (0, len(self.fem[0]) - 1)}
        for i, j in sorted(rim):
            self.stop(i, j, nodes[self.fem[i][j]], 0.0)
        for n in (chain_n[3], chain_n[4]):
            self.stop(None, n, nodes[n], 0.0)

    def near(self, p):
        """the (i, j) of the part's node nearest p"""
        return min(((i, j) for i in range(len(self.fem)) for j in range(len(self.fem[0]))), key=lambda ij: dist(nodes[self.fem[ij[0]][ij[1]]], p))

    def P(self, i, j):
        return nodes[self.fem[i][j]]

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
        p, o = np.array(nodes[b]), np.array(axis0)
        e = np.array(axis1) - o
        e /= np.linalg.norm(e)
        best = 0.0
        for sign in (1, -1):
            th = math.radians(deg) * sign
            v = p - o
            rr = o + v * math.cos(th) + np.cross(e, v) * math.sin(th) + e * np.dot(e, v) * (1 - math.cos(th))
            best = max(best, float(np.linalg.norm(rr - np.array(nodes[a]))))
        mounts.append((a, b, 20000.0, "r", best / dist(nodes[a], nodes[b]), 0.0))


parts = []
if MESH_DIR and not os.environ.get("BIW_ONLY"):   # (BIW_ONLY=1: the body-in-white alone, for a look at it)
    from_above = lambda x, z: (x, 3.0, z)
    # the hood: its hinges on the cowl (its rear edge's line), the latch on the radiator support's top, buffers on it
    # and on the upper rails, an opening stay to the firewall at 70 degrees
    hood = Part("hood", "hood", scan_grid(Mesh(["E36_HOOD"]), from_above, (0, -1, 0), np.linspace(-2.14, -0.80, 8), 9, -0.9, 0.9), 5, 5, 2.0)
    for s in (1, -1):
        hood.mount("h", -1, 3 if s > 0 else 1, (X_FW, Y_COWL, s * Z_RAIL), 15000.0, i2=-1, j2=4 if s > 0 else 0)
    hood.mount("p", 0, 2, (-2.08, 0.62, 0.0), 6000.0)
    for j in (0, -1):
        hood.stop(0, j, (-2.08, 0.62, math.copysign(0.62, hood.P(0, j)[2])))
        hood.stop(2, j, (AX_F, 0.74, math.copysign(0.68, hood.P(2, j)[2])))
    hood.strap(2, 2, (X_FW, 0.56, 0.0), hood.P(-1, 0), hood.P(-1, -1), 70.0)
    # the trunk lid, an L in section: its top and its rear face down to the tail lights' foot (the corner kept: rows 4
    # and 5); hinges on the trunk opening's front, the latch on the rear panel's top, buffers there and on the quarters'
    # rear corners, a stay to the parcel shelf
    tm = Mesh(["E36_TRUNK"])
    lid = scan_grid(tm, from_above, (0, -1, 0), [1.97, 2.12, 2.27, 2.42, 2.54], 7, -0.7, 0.7) + \
        scan_grid(tm, lambda y, z: (3.2, y, z), (-1, 0, 0), [0.93, 0.83, 0.73, 0.66], 7, -0.7, 0.7)
    trunk = Part("trunk", "lid", lid, [0, 2, 4, 5, 6, 8], 5, 6.0)
    for s in (1, -1):
        trunk.mount("h", 0, 3 if s > 0 else 1, (2.05, 0.92, s * Z_RAIL), 12000.0, i2=0, j2=4 if s > 0 else 0)
    trunk.mount("p", -1, 2, (X_REAR, 0.62, 0.0), 6000.0)
    for j in (0, -1):
        trunk.stop(-1, j, (X_REAR, 0.62, math.copysign(Z_RAIL, trunk.P(-1, j)[2])))
        trunk.stop(2, j, Q[1 if trunk.P(2, j)[2] > 0 else -1]["T2"])
    trunk.strap(1, 2, (1.80, 0.92, 0.0), trunk.P(0, 0), trunk.P(0, -1), 75.0)
    parts += [hood, trunk]
    # the tail lights beside the lid's rear face: red plastic, bolted (clamps) to the quarter's rear edge and the rear
    # panel's top
    tl = Mesh(["E36_TAILLIGHT"])
    for s in (1, -1):
        lamp = Part("lamp %d" % s, "lamp", scan_grid(tl, lambda y, z: (3.2, y, z), (-1, 0, 0), [0.69, 0.77, 0.85], 4, 0.44 if s > 0 else -0.80,
                                                     0.80 if s > 0 else -0.44), 3, 4, 0.8)
        lamp.mount("c", *lamp.near(Q[s]["L"]), Q[s]["L"], 3000.0, 150.0)
        lamp.mount("c", *lamp.near((2.52, 0.69, s * 0.45)), RP_TOP[2 if s > 0 else 0], 3000.0, 150.0)
        parts.append(lamp)
    ff = Mesh(["E36_FFENDER"])
    fd, rd = Mesh(["E36_FDOOR"]), Mesh(["E36_RDOOR"])
    for s in (1, -1):
        side = lambda u, v, s=s: (u, v, s * 2.0)
        # a fender: bolted (clamps) along its top to the upper rail, at the front to the radiator support, at the rear
        # to the hinge pillar
        fender = Part("fender %d" % s, "fender", scan_grid(ff, side, (0, 0, -s), np.linspace(-1.98, -0.80, 8), 6, 0.2, 0.95), 5, 4, 1.5)
        for i in (1, 3):
            fender.mount("c", i, -1, (fender.P(i, -1)[0], 0.72, s * 0.66), 10000.0, 1200.0)
        fender.mount("c", 0, -1, UP[s][-1], 10000.0, 1200.0)
        fender.mount("c", -1, 0, S(X_FW, Y_FLOOR, s), 10000.0, 1200.0)
        for i, j in [(1, 1), (3, 1)] + [(-1, j) for j in range(len(fender.fem[0]))]:   # (resting on the apron, its rear edge on the hinge pillar)
            fender.stop(i, j, fender.P(i, j), 0.0)
        # the doors: two hinges on the pillar ahead (each: two nodes of the door's front edge), the latch on the pillar
        # behind
        door_side = lambda u, v, s=s: (v, u, s * 2.0)
        door_f = Part("door %d front" % s, "door", scan_grid(fd, door_side, (0, 0, -s), np.linspace(0.30, 0.91, 4), 6, -0.80, 0.55), 4, 5, 9.0)
        door_f.mount("h", -1, 0, S(X_FW, Y_COWL, s), 40000.0, i2=2, j2=0)
        door_f.mount("h", 1, 0, S(X_FW, 0.56, s), 40000.0, i2=0, j2=0)
        door_f.mount("p", 1, -1, S(0.46, 0.56, s), 16000.0)
        door_r = Part("door %d rear" % s, "door", scan_grid(rd, door_side, (0, 0, -s), np.linspace(0.30, 0.91, 4), 6, 0.42, 1.60), 4, 5, 8.0)
        door_r.mount("h", -1, 0, S(0.46, 0.92, s), 40000.0, i2=2, j2=0)
        door_r.mount("h", 1, 0, S(0.46, 0.56, s), 40000.0, i2=0, j2=0)
        door_r.mount("p", 1, -1, Q[s]["A1"], 16000.0)
        for door, mesh in ((door_f, fd), (door_r, rd)):
            door.frame(mesh, s)
        parts += [fender, door_f, door_r]
    # the bumpers: polypropylene covers round the corners (their outlines scanned round), on brackets (clamps) to the
    # beams, plastic clips at the sides (soft: 30 kN/m)
    fascia = Part("fascia", "bumper", scan_round(Mesh(["E36_FBUMP", "E36_NOSE", "E36_HEADLIGHTS"]), -1.9, [0.22, 0.35, 0.48, 0.60], 11, -1), 4, 11, 4.0)
    rbump = Part("rear bumper", "bumper", scan_round(Mesh(["E36_RBUMP"]), 2.0, [0.27, 0.36, 0.45, 0.55], 11, 1), 4, 11, 3.0)
    for s in (1, -1):
        fascia.mount("c", *fascia.near((-2.26, 0.32, s * Z_RAIL)), (-2.19, 0.32, s * Z_RAIL), 6000.0, 900.0)
        fascia.mount("p", *fascia.near((-2.0, 0.48, s * 0.87)), UP[s][-2], 2500.0, k=30000.0)
        rbump.mount("c", *rbump.near((2.58, 0.38, s * Z_RAIL)), (2.52, 0.38, s * Z_RAIL), 6000.0, 900.0)
        rbump.mount("p", *rbump.near((2.05, 0.45, s * 0.83)), Q[s]["A4"], 2500.0, k=30000.0)
    parts += [fascia, rbump]

# ---- the collision volumes (phys::CollisionVolume): what fills the car, as convex hulls riding on the body-in-white -
# the engine and gearbox in the engine bay (on the firewall, the rails' middle and the towers: the crush zone ahead of
# it folds onto it), the seats and the occupants in the cabin (on the floor, the sills, the B pillars and the cowl; 30 cm
# under the roof, 15 inside the doors), the load in the trunk (on its floor, the rear panel and the parcel shelf). Other
# bodies and the static world are kept out of them, their push onto the anchors
VOLUMES = []   # (name, anchors, points, break rms)
if not os.environ.get("BIW_ONLY"):
    eng_pts = [(x, y, s * 0.34) for x, y in ((-2.00, 0.30), (-1.02, 0.30), (-2.00, 0.66), (-1.02, 0.80)) for s in (1, -1)]
    eng_an = sorted({nearest("biw", p) for s in (1, -1) for p in [(X_FW, Y_FLOOR, s * Z_RAIL), (X_FW, 0.56, s * Z_RAIL), (X_FW, Y_COWL, s * Z_RAIL)] +
                     [(x, LOW_Y(x), s * Z_RAIL) for x in (-1.32, AX_F, -1.58)] + [p for p in MID[s] if p[0] in (-1.32, -1.58)]})
    cab_pts = [(x, y, s * z) for x, y, z in ((-0.88, 0.27, 0.66), (1.12, 0.27, 0.66), (-0.40, 1.08, 0.55), (1.12, 1.08, 0.55)) for s in (1, -1)]
    cab_an = sorted({n for n in BIW_NODES if abs(nodes[n][1] - Y_FLOOR) < 1e-6 and X_FW - 1e-6 <= nodes[n][0] <= 1.18 + 1e-6} |
                    {nearest("biw", p) for s in (1, -1) for p in (S(0.46, 0.56, s), S(0.46, 0.92, s), S(X_FW, Y_COWL, s))})
    trk_pts = [(x, y, s * 0.58) for x, y in ((1.98, 0.37), (2.42, 0.37), (1.98, 0.84), (2.42, 0.84)) for s in (1, -1)]
    trk_an = sorted({n for n in BIW_NODES if nodes[n][0] >= 1.97 - 1e-6 and abs(nodes[n][1] - Y_RFLOOR) < 1e-6} |
                    {nearest("biw", p) for p in [(2.05, 0.92, 0.0), (1.80, 0.92, 0.0)] + RP_TOP})
    VOLUMES = [("engine", eng_an, eng_pts, 0.12), ("cabin", cab_an, cab_pts, 0.15), ("trunk", trk_an, trk_pts, 0.12)]

# ---- the loads: the engine and gearbox on the front rails, the crew and the trim on the cabin's floor, the fuel and
# the spare in the trunk's floor
ENGINE_KG, CREW_KG, TRIM_KG, TRUNK_KG, DRY_KG = 190.0, 160.0, 180.0, 60.0, 10.0
eng = sorted({node(p) for s in (1, -1) for p in LOW[s]})
cabin = [n for n in BIW_NODES if X_FW < nodes[n][0] < 1.2 and abs(nodes[n][1] - Y_FLOOR) < 1e-6]
trunk_floor = [n for n in BIW_NODES if nodes[n][0] > 1.9 and abs(nodes[n][1] - Y_RFLOOR) < 1e-6]
for group, kg in ((eng, ENGINE_KG), (cabin, CREW_KG + TRIM_KG), (trunk_floor, TRUNK_KG)):
    for v in group:
        load[v] = load.get(v, 0.0) + kg / len(group)

# ---- the FEM shells: material, thickness, colour (None: the car's paint) - the body-in-white's sheets, the parts' (each
# part one sheet for its skin and its inner frame)
BLACK, RED = (0.13, 0.13, 0.14), (0.75, 0.05, 0.04)   # (the bumpers: black polypropylene, as the base E36s; the tail lights)
SHELLS = {"floor": ("Steel", 0.0010, None), "firewall": ("Steel", 0.0010, None), "roof": ("Steel", 0.0010, None), "quarter": ("Steel", 0.0010, None),
          "panel": ("Steel", 0.0010, None), "wheelhouse": ("Steel", 0.0012, None), "apron": ("Steel", 0.0012, None),
          "hood": ("Steel", 0.0015, None), "fender": ("Steel", 0.0015, None), "door": ("Steel", 0.0015, None), "lid": ("Steel", 0.0015, None),
          "bumper": ("Plastic", 0.0040, BLACK), "lamp": ("Plastic", 0.0030, RED)}
BIW_SHELLS = list(SHELLS)[:7]
DENSITY = {"Steel": 7850.0, "Plastic": 950.0}


def area(t):
    a, b, c = (np.array(nodes[k]) for k in t)
    return 0.5 * float(np.linalg.norm(np.cross(b - a, c - a)))


def section_kg_m(sec):
    m_, shape, D, t = SECTIONS[sec][:4]
    if shape == "bar":
        return 7850.0 * math.pi / 4 * D * D
    return 7850.0 * ((D * D - (D - 2 * t) ** 2) if shape == "box" else math.pi / 4 * (D * D - (D - 2 * t) ** 2))


# ------------------------------------------------------------------------------------------------------ the file
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as f:
    f.write("Shell Car\n")
    f.write(";generated by tools/make_shell_car.py: a saloon on the BMW E36's lines, its body-in-white FEM members (sills, pillars,\n")
    f.write(";rails, bows, cross members) with single sheets of FEM triangles between them (floor, firewall, roof, aprons, wheelhouses,\n")
    f.write(";quarters, rear panel), the hood, fenders, doors, trunk lid, bumpers and tail lights FEM triangles (no sheets);\n")
    f.write(";the parts on hinges, latches, clamped bolts, buffers and stays that let go; the Frame Car's suspension of FEM tubes; no glass\n")
    f.write("globals\n;dry mass, cargo mass, cab material (no sheets: unused)\n%.1f, 0.0, sheet/Steel/6.3/0.004/1\n" % DRY_KG)
    f.write("minimass\n0.05\n")
    f.write("nodes\n;id, x, y, z, options[, load kg]\n")
    for i, (x, y, z) in enumerate(nodes):
        if i in load:
            f.write("%d, %.4f, %.4f, %.4f, nl, %.2f\n" % (i, x, y, z, load[i]))
        else:
            f.write("%d, %.4f, %.4f, %.4f, n\n" % (i, x, y, z))
    f.write("beams\n;the body-in-white's members and the suspension's (FEM frame elements: a, b, F[, joint at a, joint at b])\n")
    for sec in SECTIONS:
        lst = [(a, b, ja, jb) for a, b, s_, ja, jb in members if s_ == sec]
        if not lst:
            continue
        spec = SECTIONS[sec]
        extra = ", 0, %.0f, %.1f" % spec[5:7] if len(spec) > 5 else ""
        f.write(";%s\nset_beam_defaults 3000000, 400, 80000, 700000, 0.05, tracks/beam, 0\nset_frame_section %s, %s, %.4f, %.4f, %s%s\n" % (
            sec, spec[0], spec[1], spec[2], spec[3], spec[4], extra))
        for a, b, ja, jb in lst:
            if ja or jb:
                f.write("%d, %d, F, %s, %s\n" % (a, b, ja or spec[4], jb or spec[4]))
            else:
                f.write("%d, %d, F\n" % (a, b))
    # the coil-overs: the corner's load on a spring of 1.6 Hz at the wheel (damped at 0.4 of critical), preloaded to stand at the design length;
    # the stroke 14 cm up, 9 down; a bump stop and a droop strap beside each; the steering's and the rear toe's stops
    corner = {True: 3300.0, False: 3000.0}   # (N at the coil-over standing)
    f.write("shocks\n;n1, n2, spring, damp, short bound, long bound, precompression, options\n")
    f.write("set_beam_defaults 9000000, 12000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, front in shocks:
        L0 = dist(nodes[a], nodes[b])
        m = corner[front] / 9.81
        k = m * (2 * math.pi * 1.6) ** 2
        c = 2 * 0.40 * math.sqrt(k * m)
        pre = 1 + corner[front] / (k * L0)
        f.write("%d, %d, %.0f, %.0f, 0.5, 0.5, %.3f, n\n" % (a, b, k, c, pre))
    f.write("set_beam_defaults 20000000, 200000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, front in shocks:
        L0 = dist(nodes[a], nodes[b])
        f.write("%d, %d, 0, 0, %.3f, %.3f, 1.0, i\n" % (a, b, 0.14 / L0, 0.09 / L0))
    f.write("set_beam_defaults 3000000, 20000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, lo, hi in stops:
        f.write("%d, %d, 0, 0, %.3f, %.3f, 1.0, i\n" % (a, b, lo, hi))
    f.write("hydros\n;the steering rack: node1, node2, factor, options\n")
    f.write("set_beam_defaults 4000000, 2000, 99999999999999999999999999999999999999999, 99999999999999999999999999999999999999999, 0.02, tracks/beam, 0\n")
    for a, b, fac in hydros:
        f.write("%d, %d, %.4f, i\n" % (a, b, fac))
    f.write("slidenodes\n;the rack's ends slide along its housing\n")
    for n in slides:
        f.write("%d, %s, S2000000, T0\n" % (n, ", ".join(str(node(p)) for p in HOUSING)))
    f.write("wheels\n;radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, face, band\n")
    f.write("set_beam_defaults 3000000, 400, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for n1, n2, arm, front in wheels if not os.environ.get("NO_WHEELS") else []:   # (NO_WHEELS=1: for a look at the suspension)
        f.write("%.2f, %.2f, 24, %d, %d, 9999, 1, %d, %d, 40.0, 200000.0, 300.0, tracks/wheelface tracks/wheelband\n" % (WHEEL_R, WHEEL_W, n1, n2, 0 if front else 1, arm))
    f.write("engine\n;min rpm, max rpm, torque, differential, reverse, neutral, gears...\n900.0, 6500.0, 280.0, 3.15, 3.7, 1.0, 4.2, 2.5, 1.66, 1.22, 1.0, -1.0\n")
    f.write("engoption\n;inertia (rpm-based: 0.02 ~ 0.2 kg m2), type, clutch force, shift time, clutch time, post-shift time\n0.02, c, 1000.0, 0.3, 0.4, 0.3\n")
    f.write("brakes\n4000\n")
    centre = nearest("biw", (0.46, Y_FLOOR, 0.0))
    back = nearest("biw", (1.18, Y_FLOOR, 0.0))
    left = nearest("biw", (0.46, Y_FLOOR, 0.75))
    f.write("cameras\n%d, %d, %d\n" % (centre, back, left))
    near = sorted(BIW_NODES, key=lambda n: dist(nodes[n], (0.15, 1.0, 0.35)))[:8]
    f.write("cinecam\n0.15, 1.05, 0.35, %s\n" % ", ".join(str(n) for n in near))
    f.write("contacters\n")
    for i in range(len(nodes)):
        f.write("%d\n" % i)
    if VOLUMES:
        f.write("collision_volumes\n;(BeamLab) volume name, break rms (m) - its anchors (frame nodes) - its hull's points (x, y, z)\n")
        for name, an, pts, rms in VOLUMES:
            f.write("volume %s, %.2f\nanchors %s\n" % (name, rms, ", ".join(str(a) for a in an)))
            for p in pts:
                f.write("vertex %.3f, %.3f, %.3f\n" % p)
    if mounts:
        f.write("mounts\n;node a (the body's), node b (the part's), break force N, stiffness N/m (0: the step's), turning damping N m s/rad, kind:\n")
        f.write(";h a hinge (its second node on the line), p a latch, c a clamped bolt (its break moment N m), s a buffer, r a stay (its length)\n")
        for a, b, brk, kind, param, k in mounts:
            tail = {"h": ", h, %d" % (param if kind == "h" else 0), "p": ", p", "c": ", c, %.0f" % param, "s": ", s", "r": ", r, %.3f" % param}[kind]
            f.write("%d, %d, %.0f, %.0f, %.1f%s\n" % (a, b, brk, k, 2.0 if kind == "h" else 0.0, tail))
    f.write("fem_tris\n;the body-in-white's sheets and the parts: triangle elements of the FEM frame (n1, n2, n3) of the shell set before them\n")
    for name, (mat, th, col) in SHELLS.items():
        if not fem.get(name):
            continue
        f.write(";%s\nset_fem_shell %s, %.4f%s\n" % (name, mat, th, ", %.2f, %.2f, %.2f" % col if col else ""))   # (no colour: the car's paint)
        for a, b, c in fem[name]:
            f.write("%d, %d, %d\n" % (a, b, c))
    f.write("end\n")

kg_tri = {name: sum(area(t) for t in lst) * SHELLS[name][1] * DENSITY[SHELLS[name][0]] for name, lst in fem.items()}
kg_mem = {}
for a, b, sec, ja, jb in members:
    kg_mem[sec] = kg_mem.get(sec, 0.0) + dist(nodes[a], nodes[b]) * section_kg_m(sec)
biw_tris = sum(len(v) for k, v in fem.items() if k in BIW_SHELLS)
susp_secs = ("arm", "tierod", "rack", "hub", "upright")
part_secs = ("sash",)   # (the doors' window frames)
print("FEM sheets " + ", ".join("%s %.0f" % (k, v) for k, v in sorted(kg_tri.items(), key=lambda x: -x[1])) + "; members " +
      ", ".join("%s %.0f" % (k, v) for k, v in sorted(kg_mem.items(), key=lambda x: -x[1])) + " kg; the body-in-white %.0f kg" %
      (sum(v for k, v in kg_tri.items() if k in BIW_SHELLS) + sum(v for k, v in kg_mem.items() if k not in susp_secs + part_secs)))
print("wrote %s: %d nodes (%d of the body), the body-in-white %d members and %d triangles, the suspension %d members; %d FEM triangles of "
      "the parts and %d members (the window frames), %d mounts (%s), %d parts, %d collision volumes (%s anchors)" % (
          OUT, len(nodes), len(BIW_NODES), sum(1 for m in members if m[2] not in susp_secs + part_secs), biw_tris, sum(1 for m in members if m[2] in susp_secs),
          sum(len(v) for v in fem.values()) - biw_tris, sum(1 for m in members if m[2] in part_secs), len(mounts),
          ", ".join("%s %d" % (k, sum(1 for m in mounts if m[3] == k)) for k in "hpcsr"), len(parts), len(VOLUMES), "/".join(str(len(v[1])) for v in VOLUMES)))
