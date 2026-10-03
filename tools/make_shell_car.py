#!/usr/bin/env python3
"""Writes assets/vehicles/shell_car/shell_car.truck: a saloon on the BMW E36's lines, of FEM frame members and FEM
triangle elements, with a sheet body (0.6 mm of steel: the cab material, `shells`) over the triangles of the body and
of its parts but the lamps'.

The body-in-white is a thin spatial structure: the load-carrying lines are members (phys::FemFrame's elements, box
sections of steel) - the sills, the hinge, A, B and C pillars, the roof rails and three roof bows, the cowl, the floor's
cross members, the front rails with the upper rails along the fenders' line and the radiator support, the rear rails,
the bumpers' beams on crash boxes, the rear doors' posts, the arches' lips, the quarters' tops, the rear panel's top -
and the surfaces between them single sheets of triangle elements (phys::FrameTri): the floor, the firewall, the roof,
the aprons, the rear wheelhouses, the quarter panels, the parcel shelf, the rear floor and the rear panel. Members and
triangles share their nodes.

The hood, the front fenders, the doors, the trunk lid (an L: its top and its rear face down to the tail lights' foot),
the bumpers, the tail lights and the headlights are the E36's (its flexbody meshes remeshed: rays cast on structured
grids, the bumpers' round their outlines), each one sheet of FEM triangles - deep-drawing steel (MildSteel: it
yields early and stretches far; the body's members of it too), the bumpers and the lamps polypropylene. Each
is a part on the body on mounts (phys::FrameMount) that let go: the hood's, the trunk lid's and the doors' hinges (two
of the part's nodes on the hinge's line held: the part turns about it only), the latches (a point), the fenders',
bumpers' and lamps' bolts (clamps: the part's nodes round the bolt held, so a part left on one bolt does not swing about
it - past a break moment the bolt lets go; the bumpers' brackets and clips twist off under the bumper hung on one of
them: TWIST), the hood's and the lid's buffers (stops: with the latch gone the lid rests
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
as much again as the car's own mass to spin up); ring tyres (phys::Wheel::ring: a rigid 16" rim, a flexible tyre over
it; NODE_WHEELS=1: RoR's wheels of 24 rays as before).

Collision volumes (phys::CollisionVolume: convex hulls riding on the body-in-white's nodes round each) stand for what
fills the car, in zones that each follow their own part of the body as it deforms: the engine (a straight six) and its
gearbox (drawn), and fitted close to the geometry round them (fit_zone: 2 cm off the parts, 1 cm off the body's
sheets, to the window openings) - the left and the right front rail with its apron and upper rail (1 cm beyond their
box sections), the four seats' spaces and the two behind the rear seats up to the rear window, the trunk's halves, the
bumpers' reinforcements over their beams (crushed at 150 kN); they keep other bodies and the car's own parts out of
them.

The E36's meshes placed in its definition space as .obj files, from the game:

    BL_EXPORT_FLEX=<dir> ./build/beamlab --scene proving --vehicle bmw_e36/E36Sedan --frames 2 --hidden --screenshot x.png
    python3 tools/make_shell_car.py <dir>          (BIW_ONLY=1: the body-in-white alone; NO_WHEELS=1: no wheels)
    SC_M3=1 python3 tools/make_shell_car.py <dir>  (the Shell Car M3: shell_car_m3.truck, the same car under the E36
                                                    Lightweight's meshes as flexbodies on its parts' nodes)

Coordinates are Rigs of Rods' (and the E36's): -x forward, y up, +z left.
"""
import itertools
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# SC_M3=1: the Shell Car M3 - the same car with the E36 Lightweight's (the M3's) graphical model on it instead of the
# sheet body's and the plates' look: its meshes as flexbodies skinned to the nodes of the parts they are (RoR's way:
# each vertex on the three nodes of its forset nearest it), its rims on the ring tyres; its meshes stay in the E36's folder (`;resources:`)
M3 = bool(os.environ.get("SC_M3"))
E36_DIR = os.path.join(ROOT, "assets", "vehicles", "bmw_e36")
OUT = os.environ.get("SC_OUT") or os.path.join(ROOT, "assets", "vehicles", "shell_car", "shell_car_m3.truck" if M3 else "shell_car.truck")
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


def fender_rows(mesh, s, xs, nside):
    """a front fender's rows: per x its side seen from the side (nside points from the foot of the span over the arch
    up to its shoulder) and its top's inner edge seen from above (where it meets the hood's edge), 4 mm under it"""
    rows = []
    for x in xs:
        vs = [v for v in np.arange(0.20, 0.96, 0.01) if mesh.first((x, v, s * 2.0), (0, 0, -s)) is not None]
        run = [vs[-1]]
        for v in reversed(vs[:-1]):   # (the run down from its top: not across the wheel's arch)
            if run[-1] - v > 0.015:
                break
            run.append(v)
        lo, hi = run[-1] + 0.005, run[0] - 0.005
        side = []
        for j in range(nside):
            y = lerp(lo, hi, j / (nside - 1))
            t = mesh.first((x, y, s * 2.0), (0, 0, -s))
            if t is None:   # (a pinhole of the mesh: the nearest row it was hit at)
                y = min(run, key=lambda v: abs(v - y))
                t = mesh.first((x, y, s * 2.0), (0, 0, -s))
            side.append((x, y, s * (2.0 - t)))
        zs = [z for z in np.arange(0.40, 0.95, 0.005) if mesh.first((x, 3.0, s * z), (0, -1, 0)) is not None]
        zi = zs[0]
        rows.append(side + [(x, 3.0 - mesh.first((x, 3.0, s * zi), (0, -1, 0)) - 0.004, s * zi)])
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
    e_hi, e_lo = skin(1.22, 0.32, s), skin(1.22, 0.20, s)
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
        # (in front between the rails' sections' inner feet, the pivots: at the rear out to the rails)
        chain([(x, yl(x), z) for z in ((-Z_PIVOT, 0.0, Z_PIVOT) if front else (-Z_RAIL, -Z_PIVOT, 0.0, Z_PIVOT, Z_RAIL))], "subframe")
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
HOUSING = [(RACK_X, RACK_Y, z) for z in (-Z_RAIL, -RAIL_IN, 0.0, RAIL_IN, Z_RAIL)]   # (across the rails' sections' feet)
chain(HOUSING, "subframe")                                                 # the rack's housing across the rails
rack_ends = {}
for wx in (AX_F, AX_R):
    front = wx < 0
    for s in (1, -1):
        arm_x = wx + 0.18 if front else wx - 0.18
        bl, bu = node((wx, 0.12, s * Z_BALL), "susp"), node((wx, 0.50, s * Z_BALL), "susp")
        n1, n2 = node((wx, WHEEL_Y, s * 0.62), "susp"), node((wx, WHEEL_Y, s * 0.92), "susp")   # (the wheel between them: 0.66 - 0.88, the E36's M3 tyres' 0.60 - 0.87)
        sa = node((arm_x, 0.12, s * (Z_BALL + 0.02)), "susp")
        yl = (lambda x: LOW_Y(x)) if front else rail_y
        lower = [node((x, yl(x), s * Z_PIVOT)) for x in SUB_X[front]]
        upper = [node(p) for p in (MID[s] if front else WH_MID[s]) if p[0] in SUB_X[front]]
        top = node(UP[s][AP_X.index(AX_F)] if front else WH_TOP[s][2])
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
            end, lim = node(HOUSING[-1 if s > 0 else 0]), 34.0    # the steering stop: free through the lock and 4 degrees past it
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
    a, b, c = (np.array(nodes[k]) for k in t)
    return 0.5 * float(np.linalg.norm(np.cross(b - a, c - a)))


# ---------------------------------------------------------------------------------- the hang-on parts (the E36's)
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
        # its reinforcements, hidden: the two diagonals of its panel (an intrusion beam's), round its foot (the bottom
        # edge, up its ends to the first row), and the window's cross (the glass's stiffness: it shook and swung)
        f = self.fem
        member_n(f[-1][0], f[0][-1], "doorbar"), member_n(f[0][0], f[-1][-1], "doorbar")
        for a, b in list(zip(f[0], f[0][1:])) + [(f[0][0], f[1][0]), (f[0][-1], f[1][-1])]:
            member_n(a, b, "doorbar")
        member_n(top[0], chain_n[4], "dglass"), member_n(top[-1], chain_n[3], "dglass")
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

    def mass_centre(self):
        """its mass (its triangles' and its load, kg) and its centre"""
        ns = {k for row in self.fem for k in row}
        mat, th, _ = SHELLS[self.shell]
        m, mc = 0.0, np.zeros(3)
        for t in fem.get(self.shell, []):
            if all(k in ns for k in t):
                kg = area(t) * th * DENSITY[mat]
                m, mc = m + kg, mc + kg * np.mean([nodes[k] for k in t], axis=0)
        for k in ns:
            m, mc = m + load.get(k, 0.0), mc + load.get(k, 0.0) * np.array(nodes[k])
        return m, mc / m

    def twist(self, i, j, share=TWIST):
        """a clamp's break moment at its node (i, j) (N m): a share (TWIST) of its weight's moment about it, hung there
        alone - a part left on that bolt twists it off"""
        m, c = self.mass_centre()
        p = self.P(i, j)
        return share * m * 9.81 * math.hypot(c[0] - p[0], c[2] - p[2])

    def straighten_hinges(self):
        """its hinges' nodes put on one line, the one that fits them best (an edge scanned off the mesh is curved: a
        door held at four points off one line could not turn - its latch gone, it stood 2 cm open)"""
        own = {k for row in self.fem for k in row}
        hn = sorted({n for m in mounts if m[3] == "h" and m[1] in own for n in (m[1], m[4])})
        if len(hn) < 3:
            return
        P = np.array([nodes[n] for n in hn])
        c = P.mean(axis=0)
        e = np.linalg.svd(P - c)[2][0]   # (the principal axis)
        for n, p in zip(hn, P):
            nodes[n] = tuple(float(v) for v in c + e * np.dot(p - c, e))

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
    # (its rows follow its front's outline: scanned per column across it, front to rear - rows across at one x cut its
    # front corners off over the headlights, the wheels seen through them)
    hcols = scan_grid(Mesh(["E36_HOOD"]), lambda z, x: (x, 3.0, z), (0, -1, 0), [-0.77, -0.66, -0.40, 0.0, 0.40, 0.66, 0.77], 8, -2.30, -0.70, min_span=0.3)
    hood = Part("hood", "hood", [list(r) for r in zip(*hcols)], 5, 7, 2.0)   # (a column at 0.66: its corner over the headlight's back)
    for s in (1, -1):
        hood.mount("h", -1, -2 if s > 0 else 1, (X_FW, Y_COWL, s * Z_RAIL), 15000.0, i2=-1, j2=-1 if s > 0 else 0)
    hood.mount("p", 0, 3, (-2.08, 0.62, 0.0), 6000.0)
    for j in (0, -1):
        hood.stop(0, j, (-2.08, 0.62, math.copysign(0.62, hood.P(0, j)[2])))
        hood.stop(2, j, (AX_F, 0.74, math.copysign(0.68, hood.P(2, j)[2])))
    hood.strap(2, 3, (X_FW, 0.56, 0.0), hood.P(-1, 0), hood.P(-1, -1), 70.0)
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
        # (four bolts, a real lamp's: its outer edge's top, middle and foot on the quarter's rear edge, its inner foot on
        # the rear panel's top; on two it shook)
        for at, to in ((Q[s]["L"], Q[s]["L"]), (Q[s]["T2"], Q[s]["T2"]), (Q[s]["M2"], Q[s]["M2"]), ((2.52, 0.69, s * 0.45), RP_TOP[2 if s > 0 else 0])):
            lamp.mount("c", *lamp.near(at), to, 3000.0, 150.0)
        parts.append(lamp)
    # the headlights in the nose over the bumper: clear plastic lenses, bolted (clamps) to the radiator support's top at
    # its end and its middle
    hl = Mesh(["E36_HEADLIGHTS"])
    HEADLIGHT = {}
    for s in (1, -1):
        head = Part("headlight %d" % s, "headlamp", scan_grid(hl, lambda y, z: (-3.0, y, z), (1, 0, 0), [0.555, 0.615, 0.675], 5, 0.38 if s > 0 else -0.82,
                                                              0.82 if s > 0 else -0.38), 3, 5, 1.2)
        # (four bolts, a real lamp's: its top on the radiator support's top at its inner end and at the upper rail's,
        # its foot on the support's middle row and at its outer end on the upper rail; on two it shook)
        for p_, t_ in (((-2.10, 0.66, s * 0.62), UP[s][-1]), ((-2.10, 0.66, s * 0.42), RAD_TOP[3 if s > 0 else 1]),
                       ((-2.12, 0.56, s * 0.55), RAD_MID[4 if s > 0 else 0]), ((-2.02, 0.56, s * 0.80), UP[s][-2])):
            head.mount("c", *head.near(p_), t_, 3000.0, 150.0)
        parts.append(head)
        HEADLIGHT[s] = head
    # the grille between the headlights over the bumper (the E36 nose's kidneys): black plastic, bolted (clamps) to the
    # radiator support's top and middle row
    grille = Part("grille", "grille", scan_grid(Mesh(["E36_NOSE"]), lambda y, z: (-3.0, y, z), (1, 0, 0), [0.53, 0.61, 0.685], 5, -0.40, 0.40, min_span=0.3), 3, 5, 0.6)
    for at, to in (((-2.2, 0.685, -0.3), RAD_TOP[1]), ((-2.2, 0.685, 0.0), RAD_TOP[2]), ((-2.2, 0.685, 0.3), RAD_TOP[3]),
                   ((-2.2, 0.53, -0.3), RAD_MID[1]), ((-2.2, 0.53, 0.3), RAD_MID[3])):
        grille.mount("c", *grille.near(at), to, 2000.0, 60.0)
    parts.append(grille)
    ff, hm = Mesh(["E36_FFENDER"]), Mesh(["E36_HOOD"])
    fd, rd = Mesh(["E36_FDOOR"]), Mesh(["E36_RDOOR"])
    for s in (1, -1):
        side = lambda u, v, s=s: (u, v, s * 2.0)
        # a fender: its side and its top in to the hood's edge (the side alone left a slot along the hood), from the
        # headlight to the door; bolted (clamps) along its top to the upper rail, at the front to the radiator support,
        # at the rear to the hinge pillar; 6 x 4 of its points (30 triangles; two stations behind the arch: one left a
        # hole down to the sill)
        # (its front row on the headlight's outer end - at x -2.04 its side was a point; its rear one at -0.80: at -0.78
        # the mesh's edge is ragged, the side there 10 cm tall)
        hl_out = [nodes[r[-1 if s > 0 else 0]] for r in HEADLIGHT[s].fem]
        front = [(x + 0.012, y, z + s * 0.012) for x, y, z in hl_out] + [(-2.04, 0.694, s * 0.73)]
        fender = Part("fender %d" % s, "fender", [front] + fender_rows(ff, s, [-1.72, -1.41, -1.12, -0.95, -0.80], 3), 6, 4, 1.5)
        for i in (1, 2, 3):
            fender.mount("c", i, -1, fender.P(i, -1), 10000.0, 1200.0)
        fender.mount("c", 0, -1, UP[s][-1], 10000.0, 1200.0)
        fender.mount("c", -1, 0, S(X_FW, Y_FLOOR, s), 10000.0, 1200.0)
        for i, j in [(1, 1), (2, 1), (3, 1)] + [(-1, j) for j in range(len(fender.fem[0]))]:   # (resting on the apron, its rear edge on the hinge pillar)
            fender.stop(i, j, fender.P(i, j), 0.0)
        # the doors: two hinges on the pillar ahead, the latch on the pillar behind
        door_side = lambda u, v, s=s: (v, u, s * 2.0)
        door_f = Part("door %d front" % s, "door", scan_grid(fd, door_side, (0, 0, -s), np.linspace(0.30, 0.91, 4), 6, -0.80, 0.55), 4, 5, 9.0)
        # (both hinges hold the same two nodes of its front edge, at the hinges' heights: their line is the axis, the
        # edge's curve off it kept - held at four points off one line the door could not turn)
        door_f.mount("h", 2, 0, S(X_FW, Y_COWL, s), 40000.0, i2=1, j2=0)
        door_f.mount("h", 1, 0, S(X_FW, 0.56, s), 40000.0, i2=2, j2=0)
        door_f.mount("p", 1, -1, S(0.46, 0.56, s), 16000.0)
        door_f.mount("p", 0, -1, S(0.46, Y_FLOOR + 0.10, s), 12000.0)   # (and its foot's catch on the B pillar's foot: the corner flapped 3-6 cm on the rough field)
        door_r = Part("door %d rear" % s, "door", scan_grid(rd, door_side, (0, 0, -s), np.linspace(0.30, 0.91, 4), 6, 0.42, 1.60), 4, 5, 8.0)
        door_r.mount("h", 2, 0, S(0.46, 0.92, s), 40000.0, i2=1, j2=0)
        door_r.mount("h", 1, 0, S(0.46, 0.56, s), 40000.0, i2=2, j2=0)
        door_r.mount("p", 1, -1, Q[s]["A1"], 16000.0)
        door_r.mount("p", 0, -1, (1.18, Y_FLOOR + 0.10, s * FZ[-1]), 12000.0)
        # (the check straps' anchors behind the hinges' line, inboard: opening takes the door's node away from them - the
        # hinge pillar ahead of it came nearer, and the strap never pulled)
        for door, mesh, pillar in ((door_f, fd, (-0.40, Y_FLOOR + 0.10, s * FZ[-1])), (door_r, rd, S(0.46, 0.56, s))):
            door.frame(mesh, s)
            door.strap(1, 1, pillar, door.P(1, 0), door.P(2, 0), 70.0)   # (a check strap: it opens 70 degrees about the hinges' line)
        parts += [fender, door_f, door_r]
    # the bumpers: polypropylene covers round the corners (their outlines scanned round; the front one under the
    # headlights, its sides ending 20 cm ahead of the wheels: the steered tyres' volumes pushed on its wrap, 9 kN
    # circling, and twisted its clips off), on brackets (clamps) to the beams, plastic clips at the sides (clamps too, soft: 30 kN/m) - each one
    # twisted off past the moment of the bumper's weight hung on it alone (TWIST)
    fascia = Part("fascia", "bumper", scan_round(Mesh(["E36_FBUMP", "E36_NOSE"]), -1.97, [0.22, 0.33, 0.43, 0.545], 11, -1), 4, 11, 4.0)
    rbump = Part("rear bumper", "bumper", scan_round(Mesh(["E36_RBUMP"]), 2.0, [0.27, 0.36, 0.45, 0.55], 11, 1), 4, 11, 3.0)
    for s in (1, -1):
        for part, at, target, brk, k in ((fascia, (-2.26, 0.32, s * Z_RAIL), (-2.19, 0.32, s * Z_RAIL), 6000.0, 0.0),
                                         (fascia, (-2.0, 0.48, s * 0.87), UP[s][-2], 2500.0, 30000.0),
                                         (rbump, (2.58, 0.38, s * Z_RAIL), (2.52, 0.38, s * Z_RAIL), 6000.0, 0.0),
                                         (rbump, (2.05, 0.45, s * 0.83), Q[s]["A4"], 2500.0, 30000.0)):
            # (the side clips 0.5 of it: at 0.35 the front's twisted off on the rough field with the wheels out at the
            # E36's track)
            part.mount("c", *part.near(at), target, brk, part.twist(*part.near(at), TWIST if k == 0.0 else 0.5), k=k)
    parts += [fascia, rbump]
    for part in parts:
        part.straighten_hinges()
    for part in parts:
        if part.shell == "bumper":
            m_, c_ = part.mass_centre()
            print("  %s: %.1f kg, its clamps twist off at %s N m" % (part.name, m_, ", ".join("%.0f" % mm[4] for mm in mounts if mm[3] == "c" and node_part[mm[1]] == part.name)))

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
            quad(ring(a0, 0.62), ring(a1, 0.62), ring(a1, 1.0), ring(a0, 1.0), "wheel")
            out.append(((AX_F, WHEEL_Y, s * 0.62), ring(a0, 0.62), ring(a1, 0.62), "wheel"))   # (its inner face)
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


def fit_zone(cs, lo, hi, gap_of, seeds=()):
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


if not os.environ.get("BIW_ONLY"):
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

# ---- the loads: the engine and gearbox on the front rails, the crew and the trim on the cabin's floor, the fuel and
# the spare in the trunk's floor
ENGINE_KG, CREW_KG, TRIM_KG, TRUNK_KG, DRY_KG = 190.0, 160.0, 143.0, 60.0, 10.0   # (the trim: the car weighs ~1245 kg, as with the members' body)
eng = sorted({node(p) for s in (1, -1) for p in LOW[s]})
cabin = [n for n in BIW_NODES if X_FW < nodes[n][0] < 1.2 and abs(nodes[n][1] - Y_FLOOR) < 1e-6]
trunk_floor = [n for n in BIW_NODES if nodes[n][0] > 1.9 and abs(nodes[n][1] - Y_RFLOOR) < 1e-6]
for group, kg in ((eng, ENGINE_KG), (cabin, CREW_KG + TRIM_KG), (trunk_floor, TRUNK_KG)):
    for v in group:
        load[v] = load.get(v, 0.0) + kg / len(group)

def section_kg_m(sec):
    m_, shape, D, t = SECTIONS[sec][:4]
    if shape == "bar":
        return 7850.0 * math.pi / 4 * D * D
    return 7850.0 * ((D * D - (D - 2 * t) ** 2) if shape == "box" else math.pi / 4 * (D * D - (D - 2 * t) ** 2))


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


# ------------------------------------------------------------------------- the M3's graphical model (SC_M3=1)
M3_TRUCK = os.path.join(E36_DIR, "E36Lightweight.truck")
M3_WHEEL = (0.34, 0.262, 0.24, "E36LW_Frim.mesh")   # (its tyres' radius, its 17" rims' with their flanges, the tyre's width; the rim's mesh)
# each mesh on the nodes of what it is of: the body-in-white's, or its parts'
M3_FORSET = {"CHASSIS": ["biw"], "SKIRT": ["biw"], "WIND": ["biw"], "WINDINT": ["biw"], "DASH": ["biw"], "SEAT": ["biw"], "NOSE": ["grille", "biw"],
             "TAILLIGHT": ["lamp 1", "lamp -1"], "TRUNK": ["trunk"], "FFENDER": ["fender 1", "fender -1"], "HOOD": ["hood"],
             "FDOOR": ["door 1 front", "door -1 front"], "MIRRORS": ["door 1 front", "door -1 front"], "RDOOR": ["door 1 rear", "door -1 rear"],
             "HEADLIGHTS": ["headlight 1", "headlight -1"], "FBUMP": ["fascia"], "LIP": ["fascia"], "RBUMP": ["rear bumper"]}


def rot_xyz(deg):
    """the truck format's rotation: Rz Ry Rx (degrees)"""
    rx, ry, rz = (math.radians(a) for a in deg)
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def placement(pref, px, py):
    """a flexbody's or a prop's frame on its ref, x and y nodes (VehicleVisual's place, RoR's): the offset's basis
    (X, Y, the normal) and the orientation's (X's direction, the normal, their cross)"""
    pref, X, Y = np.array(pref), np.array(px) - np.array(pref), np.array(py) - np.array(pref)
    n = np.cross(Y, X)
    n /= np.linalg.norm(n)
    rx = X / np.linalg.norm(X)
    return pref, np.column_stack([X, Y, n]), np.column_stack([rx, n, np.cross(rx, n)])


def replace_frame(e36, line_nodes, off, rot, own):
    """the offset and the rotation that place a mesh on the nodes `own` (ref, x, y of this car) where the E36 has it on
    its nodes `line_nodes` with `off` and `rot`"""
    pe, Be, Re = placement(*(e36[k] for k in line_nodes))
    pos, M = pe + Be @ np.array(off), Re @ rot_xyz(rot)
    po, Bo, Ro = placement(*(nodes[k] for k in own))
    R = Ro.T @ M
    ry = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    return np.linalg.solve(Bo, pos - po), [math.degrees(a) for a in (math.atan2(R[2, 1], R[2, 2]), ry, math.atan2(R[1, 0], R[0, 0]))]


def frame_of(ns):
    """three of the nodes ns for a frame: two far apart, the third far off their line"""
    P = {k: np.array(nodes[k]) for k in ns}
    a = min(ns, key=lambda k: tuple(P[k]))
    b = max(ns, key=lambda k: np.linalg.norm(P[k] - P[a]))
    e = (P[b] - P[a]) / np.linalg.norm(P[b] - P[a])
    c = max(ns, key=lambda k: np.linalg.norm((P[k] - P[a]) - e * np.dot(P[k] - P[a], e)))
    return a, b, c


def ranges(ns):
    """a forset's node list as ranges"""
    ns, out = sorted(set(ns)), []
    for k in ns:
        if out and out[-1][1] == k - 1:
            out[-1][1] = k
        else:
            out.append([k, k])
    return ",".join("%d-%d" % (a, b) if b > a else "%d" % a for a, b in out)


def m3_model():
    """the E36 Lightweight's managed materials, its flexbodies and its steering wheel as this car's lines"""
    lines = [l.rstrip("\n") for l in open(M3_TRUCK, encoding="latin-1")]
    e36, mats, flex, props, sec = {}, [], [], [], None
    for l in lines:
        t = l.strip()
        if not t or t.startswith(";"):
            continue
        word = t.split()[0].split(",")[0]
        if word == t and not t[0].isdigit() and t == t.lower():   # (a section's keyword: a line of its own)
            sec = t
            continue
        if word in ("forset", "add_animation") or word.startswith("set_"):
            continue
        if sec == "nodes":
            f = [x.strip() for x in t.split(",")]
            if len(f) >= 4 and f[0].isdigit():
                e36[int(f[0])] = tuple(float(x) for x in f[1:4])
        elif sec == "managedmaterials":
            mats.append(t)
        elif sec == "flexbodies" and t.count(",") >= 9:
            f = [x.strip() for x in t.split(",")]
            flex.append(([int(x) for x in f[0:3]], [float(x) for x in f[3:6]], [float(x) for x in f[6:9]], f[9]))
        elif sec == "props" and "E36_STEER" in t:
            f = [x.strip() for x in t.split(",")]
            props.append(([int(x) for x in f[0:3]], [float(x) for x in f[3:6]], [float(x) for x in f[6:9]], ",".join(f[9:])))
    by_part = {}
    for k, g in node_part.items():
        by_part.setdefault(g, []).append(k)
    out_flex = []
    for ln, off, rot, mesh in flex:
        key = "LIP" if "lip" in mesh else mesh.split(".")[0].replace("E36_", "").replace("_B", "")
        if key not in M3_FORSET:    # (the tyres' flexbodies: the ring tyres draw their own)
            continue
        ns = [k for g in M3_FORSET[key] for k in by_part.get(g, [])]
        if len(ns) < 3:
            continue
        own = frame_of(ns)
        o, r = replace_frame(e36, ln, off, rot, own)
        out_flex.append("%d, %d, %d, %.5f, %.5f, %.5f, %.4f, %.4f, %.4f, %s\nforset %s" % (own + tuple(o) + tuple(r) + (mesh, ranges(ns))))
        # (how far its vertices are from the nodes they hang on: the sedan's mesh, the same geometry)
        fn = [f_ for f_ in os.listdir(MESH_DIR) if f_.endswith("_E36_" + key + ".mesh.obj")]
        if fn:
            V = load_obj(os.path.join(MESH_DIR, fn[0]))[0]
            P = np.array([nodes[k] for k in ns])
            d = np.array([np.min(np.linalg.norm(P - v, axis=1)) for v in V[::7]])
            print("  M3 %s on %d nodes: its vertices %.2f m from their nearest node at most, %.2f on average" % (mesh, len(ns), d.max(), d.mean()))
    out_props = []
    floor = [k for k in by_part["biw"] if abs(nodes[k][1] - Y_FLOOR) < 0.02 and -0.9 < nodes[k][0] < 0.6]
    for ln, off, rot, tail in props:   # (the steering wheel: on the floor's nodes under it)
        own = frame_of(floor)
        o, r = replace_frame(e36, ln, off, rot, own)
        out_props.append("%d, %d, %d, %.5f, %.5f, %.5f, %.4f, %.4f, %.4f, %s" % (own + tuple(o) + tuple(r) + (tail,)))
    return mats, out_flex, out_props


M3_MODEL = m3_model() if M3 and MESH_DIR else None

# ------------------------------------------------------------------------------------------------------ the file
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as f:
    f.write("Shell Car M3\n" if M3 else "Shell Car\n")
    f.write(";generated by tools/make_shell_car.py: a saloon on the BMW E36's lines, its body-in-white FEM members (sills, pillars,\n")
    f.write(";rails, bows, cross members) with single sheets of FEM triangles between them (floor, firewall, roof, aprons, wheelhouses,\n")
    f.write(";quarters, rear panel), the hood, fenders, doors, trunk lid, bumpers, tail lights and headlights FEM triangles, a sheet\n;body over them but the lamps;\n")
    f.write(";the parts on hinges, latches, clamped bolts, buffers and stays that let go; the Frame Car's suspension of FEM tubes; no glass\n")
    # (the cab material: a sheet body on the collision triangles - `shells`: the body's and the parts', 0.6 mm of steel
    # over their FEM plates, its dents and cracks; the car's crashes were tuned with it, without it its plates took the
    # blows alone - thousands of strain clamps in a head-on, the bumpers' plastic most. The bumpers' is black; the lamps
    # and the headlights keep out of it, their own colours seen, their FEM triangles collide)
    if M3_MODEL:
        f.write(";its look: the BMW E36 Lightweight's (the M3's) meshes, flexbodies skinned to the nodes of the parts they are (SC_M3=1);\n")
        f.write(";the sheet body and the FEM plates are there and not drawn (the cab material's `hidden`), the body's members hidden\n")
        f.write(";resources: ../bmw_e36\n")
        f.write("managedmaterials\n%s\n" % "\n".join(M3_MODEL[0]))
    f.write("globals\n;dry mass, cargo mass, cab material (the sheet body over the shells below)\n%.1f, 0.0, sheet/Steel/%.1f/0.004/1%s\n" % (DRY_KG, SHEET_KG_M2, "/hidden" if M3_MODEL else ""))
    f.write("minimass\n0.05\n")
    f.write(";editor-layers:%s\n" % "".join(" %s|1|0" % l for l in LAYERS))
    mark = Marks(f)
    f.write("nodes\n;id, x, y, z, options[, load kg]\n")
    mark.section()
    for i, (x, y, z) in enumerate(nodes):
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
        opt = "Fi" if sec in HIDDEN_SECS or (M3_MODEL and sec not in ("arm", "tierod", "rack", "upright")) else "F"   # (i: not drawn; under the M3's meshes: the body's all, the hubs' stubs through the rims)
        for a, b, ja, jb in lst:
            if ja or jb:
                f.write("%d, %d, %s, %s, %s\n" % (a, b, opt, ja or spec[4], jb or spec[4]))
            else:
                f.write("%d, %d, %s\n" % (a, b, opt))
    # the coil-overs: the corner's load on a spring of 1.6 Hz at the wheel (damped at 0.4 of critical), preloaded to stand at the design length;
    # the stroke 14 cm up, 9 down; a bump stop and a droop strap beside each; the steering's and the rear toe's stops
    corner = {True: 3300.0, False: 3000.0}   # (N at the coil-over standing)
    f.write("shocks\n;n1, n2, spring, damp, short bound, long bound, precompression, options\n")
    mark.section(), mark("Suspension")
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
    mark.section(), mark("Suspension")
    f.write("set_beam_defaults 4000000, 2000, 99999999999999999999999999999999999999999, 99999999999999999999999999999999999999999, 0.02, tracks/beam, 0\n")
    for a, b, fac in hydros:
        f.write("%d, %d, %.4f, i\n" % (a, b, fac))
    f.write("slidenodes\n;the rack's ends slide along its housing\n")
    for n in slides:
        f.write("%d, %s, S2000000, T0\n" % (n, ", ".join(str(node(p)) for p in HOUSING)))
    if os.environ.get("NODE_WHEELS"):   # (NODE_WHEELS=1: RoR's wheels of nodes and spokes, as before the ring tyres)
        f.write("wheels\n;radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, face, band\n")
        mark.section(), mark("Suspension")
        f.write("set_beam_defaults 3000000, 400, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
        for n1, n2, arm, front in wheels if not os.environ.get("NO_WHEELS") else []:   # (NO_WHEELS=1: for a look at the suspension)
            f.write("%.2f, %.2f, 24, %d, %d, 9999, 1, %d, %d, 40.0, 200000.0, 300.0, tracks/wheelface tracks/wheelband\n" % (WHEEL_R, WHEEL_W, n1, n2, 0 if front else 1, arm))
    else:   # ring tyres (phys::Wheel::ring): a rigid 16" rim, a flexible 205/55-like tyre over it, 20 kg
        f.write("ringwheels\n;(BeamLab) radius, rim radius, width, node1, node2, braking, propulsion, arm, mass, tyre stiffness (N/m, 2 cm in), damping (N s/m), grip\n")
        mark.section(), mark("Suspension")
        for n1, n2, arm, front in wheels if not os.environ.get("NO_WHEELS") else []:
            if M3_MODEL:   # (the M3's 17" wheels: its rims' mesh on them - the side, the mesh)
                f.write("%.2f, %.3f, %.2f, %d, %d, 1, %d, %d, 20.0, 200000.0, 1000.0, 1.0, %s, %s\n" % (
                    M3_WHEEL[:3] + (n1, n2, 0 if front else 1, arm, "r" if nodes[n1][2] > 0 else "l", M3_WHEEL[3])))
                continue
            f.write("%.2f, %.3f, %.2f, %d, %d, 1, %d, %d, 20.0, 200000.0, 1000.0, 1.0\n" % (WHEEL_R, RIM_R, WHEEL_W, n1, n2, 0 if front else 1, arm))
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
        for name, an, pts, rms, col in VOLUMES:
            crush = CRUSH.get(name, 0.0)
            f.write("volume %s, %.2f%s\nanchors %s\n" % (name, rms, ", %.0f" % crush if crush else "", ", ".join(str(a) for a in an)))
            for p in pts:
                f.write("vertex %.3f, %.3f, %.3f\n" % p)
            if col:
                f.write("color %.2f, %.2f, %.2f\n" % col)
    if mounts:
        f.write("mounts\n;node a (the body's), node b (the part's), break force N, stiffness N/m (0: the step's), turning damping N m s/rad, kind:\n")
        f.write(";h a hinge (its second node on the line), p a latch, c a clamped bolt (its break moment N m), s a buffer, r a stay (its length)\n")
        for a, b, brk, kind, param, k in mounts:
            tail = {"h": ", h, %d" % (param if kind == "h" else 0), "p": ", p", "c": ", c, %.0f" % param, "s": ", s", "r": ", r, %.3f" % param}[kind]
            f.write("%d, %d, %.0f, %.0f, %.1f%s\n" % (a, b, brk, k, 2.0 if kind == "h" else 0.0, tail))
    f.write("shells\n;the sheet body's triangles (their FEM triangles' collision triangles): all but the lamps' and the headlights'\n")
    for name in SHELLS:
        if name not in ("bumper", "lamp", "headlamp", "grille") + BOX_SHELLS:
            for a, b, c in fem.get(name, []):
                f.write("%d, %d, %d\n" % (a, b, c))
    f.write("set_shell_material bumpers, Steel, %.1f, 0.004, %.2f, %.2f, %.2f, 1\n" % ((SHEET_KG_M2,) + BLACK))   # (the bumpers': the sheet's steel, their black)
    for a, b, c in fem.get("bumper", []):
        f.write("%d, %d, %d\n" % (a, b, c))
    f.write("set_shell_material default\n")
    f.write("fem_tris\n;the body-in-white's sheets and the parts: triangle elements of the FEM frame (n1, n2, n3) of the shell set before them\n")
    mark.section()
    for name, (mat, th, col) in SHELLS.items():
        if not fem.get(name):
            continue
        f.write(";%s\nset_fem_shell %s, %.4f%s\n" % (name, mat, th, ", %.2f, %.2f, %.2f" % col if col else ""))   # (no colour: the car's paint)
        mark(layer_of(name))
        for a, b, c in fem[name]:
            f.write("%d, %d, %d\n" % (a, b, c))
    if M3_MODEL:
        f.write("flexbodies\n;the E36 Lightweight's meshes, each on its part's nodes: ref, x, y, offset, rotation, mesh - forset: the nodes it is skinned to\n")
        f.write("\n".join(M3_MODEL[1]) + "\n")
        if M3_MODEL[2]:
            f.write("props\n;the steering wheel\n" + "\n".join(M3_MODEL[2]) + "\nadd_animation -460, 0, 0, source: steeringwheel, mode: x-rotation\n")
    f.write("end\n")

kg_tri = {name: sum(area(t) for t in lst) * SHELLS[name][1] * DENSITY[SHELLS[name][0]] for name, lst in fem.items()}
kg_mem = {}
for a, b, sec, ja, jb in members:
    kg_mem[sec] = kg_mem.get(sec, 0.0) + dist(nodes[a], nodes[b]) * section_kg_m(sec)
biw_tris = sum(len(v) for k, v in fem.items() if k in BIW_SHELLS)
susp_secs = ("arm", "tierod", "rack", "hub", "upright")
part_secs = ("sash", "doorbar", "dglass")   # (the doors' window frames, their hidden braces)
print("FEM sheets " + ", ".join("%s %.0f" % (k, v) for k, v in sorted(kg_tri.items(), key=lambda x: -x[1])) + "; members " +
      ", ".join("%s %.0f" % (k, v) for k, v in sorted(kg_mem.items(), key=lambda x: -x[1])) + " kg; the body-in-white %.0f kg" %
      (sum(v for k, v in kg_tri.items() if k in BIW_SHELLS) + sum(v for k, v in kg_mem.items() if k not in susp_secs + part_secs)))
print("wrote %s: %d nodes (%d of the body), the body-in-white %d members and %d triangles, the suspension %d members; %d FEM triangles of "
      "the parts and %d members (the window frames), %d mounts (%s), %d parts, %d collision volumes (%s anchors)" % (
          OUT, len(nodes), len(BIW_NODES), sum(1 for m in members if m[2] not in susp_secs + part_secs), biw_tris, sum(1 for m in members if m[2] in susp_secs),
          sum(len(v) for v in fem.values()) - biw_tris, sum(1 for m in members if m[2] in part_secs), len(mounts),
          ", ".join("%s %d" % (k, sum(1 for m in mounts if m[3] == k)) for k in "hpcsr"), len(parts), len(VOLUMES), "/".join(str(len(v[1])) for v in VOLUMES)))
