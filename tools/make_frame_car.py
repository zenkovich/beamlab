#!/usr/bin/env python3
"""Writes assets/vehicles/frame_car/frame_car.truck: a five-door hatchback on a welded space frame of FEM frame elements.

The frame is chromoly tube, welded at its joints (phys/frame_fem.h), as a car's body in white is laid out: floor rails
from the front crash beam to the rear one, sills, cross members under the firewall, the B and the C pillars, the
A, B, C and D pillars, roof rails and bows, the cowl, the aprons over the front wheels, the towers, the rear belt
rails and posts. The panels are sheet metal and glass (triangle elements) on it, each with nodes of its own: where a
panel's edge (or a row of it) runs along a tube, the tube is divided at the panel's nodes, a frame node under every
one of them, and each such pair is a weld (`welds`, phys::SoftBody::Weld) that holds the panel there by the weighted
centre of its nodes round the weld (the pull spread over them, falling off with the distance), and lets go past its
strength. The doors are frames of their own on two hinges (ball joints on a vertical line) with a latch that breaks
(then they swing open, a check strap ending their travel both ways), skinned in sheet and glass. The bumpers hang on
the crash beams by welds a few centimetres off; the head and tail lights and the mirrors are little glass and plastic
boxes of their own on beams that tear.

Suspension: double wishbones of tube on ball joints at the frame, the upright (the stub and a steering arm welded
together) on ball joints at their ends. A coil-over from the lower ball joint to the tower carries the car, a travel
stop beside it ends the wheel's travel. At the front a steering rack: a bar of tube whose ends slide along the rack
housing (slide nodes), moved by one hydro, with tie rods of tube on ball joints to the steering arms; a stop beside
each steering arm ends its turn a few degrees past the lock. At the rear a toe link of tube holds the wheel straight.
Rear-wheel drive.

Coordinates follow Rigs of Rods: -x forward, y up, +z left. The design is written with the front at +x and mirrored
when the nodes are written.

    python3 tools/make_frame_car.py
"""
import math
import os

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "vehicles", "frame_car", "frame_car.truck")

# ---------------------------------------------------------------------------------------------------- the body's lines
Y0 = 0.30                    # floor (rails, sills, cross members)
YB = 0.92                    # belt line
ZS = 0.80                    # the sides (sills, belt), ZR the floor rails, ZT the roof rails
ZR = 0.48
ZT = 0.66
XA, XC = 0.85, -0.92         # A pillar (firewall), C pillar (behind the rear doors)
X_WS = 0.28                  # the windscreen's top (the roof's front)
X_RR = -0.90                 # the roof's rear
YR_F, YR_R = 1.40, 1.39      # the roof rails' height at its front and rear
XF, XR = 1.92, -1.90         # the crash beams
AXLES = (1.30, -1.30)
WHEEL_R = 0.30
ARCH_R = 0.38                # wheel arches (their centre: the axle at design height)
GRID = 0.13                  # the panels' cell (about)
ROOF_NU, ROOF_B = 9, 3
XB = X_WS - (X_WS - X_RR) * ROOF_B / ROOF_NU   # the B pillar: under a row of the roof
D_T = 0.7                    # where the hatch's glass ends down the D pillar
PRELOAD = {True: 1.09, False: 1.075}

SECTIONS = {
    "rail": ("Chromoly", "tube", 0.050, 0.0025, "rigid"),     # floor rails, sills, cross members, crash beams
    "pillar": ("Chromoly", "tube", 0.045, 0.0025, "rigid"),   # pillars, roof rails and bows, cowl
    "light": ("Chromoly", "tube", 0.030, 0.0020, "rigid"),    # aprons, rear rails and posts, towers, the front's crosses
    "door": ("Chromoly", "tube", 0.028, 0.0020, "rigid"),     # the doors' frames
    "hinge": ("Steel", "tube", 0.030, 0.0060, "rigid"),       # door hinges (a ball joint at the door)
    "arm": ("Chromoly", "tube", 0.035, 0.0030, "rigid"),      # wishbones, toe links
    "tierod": ("Chromoly", "tube", 0.028, 0.0040, "ball"),    # tie rods
    "rack": ("Steel", "tube", 0.030, 0.0050, "rigid"),        # the rack bar and its housing
    "hub": ("Steel", "tube", 0.050, 0.0060, "rigid"),         # the uprights
}
MAT = {  # shell materials: name -> (material, kg/m2, drawn thickness, colour or None: the body's)
    "body": None,
    "glass": ("Glass", 10.0, 0.004, (0.30, 0.38, 0.44)),
    "trim": ("Steel", 6.0, 0.006, (0.10, 0.10, 0.11)),
    "floor": ("Steel", 7.9, 0.004, (0.22, 0.22, 0.23)),
    "headlight": ("Glass", 6.0, 0.006, (0.95, 0.93, 0.80)),
    "taillight": ("Glass", 6.0, 0.006, (0.75, 0.05, 0.04)),
}
WELD_R, WELD_BRK = 0.17, 3500.0   # weld radius (m), strength (N)
# the panels' refinement depth (a dent's triangles halved once: at 3 levels a crash made 7000 triangles of 2200, and
# the membrane's projection, one body's own work, took most of the step)
REFINE = int(os.environ.get("FC_REFINE", "1"))

# -------------------------------------------------------------------------------------------------------- the nodes
nodes = []          # design space (+x forward)
node_kind = []      # "fem", "panel", "plain"
node_mm = []        # the node's minimum mass group: None (the global), or kg
fem_at = {}


def v_add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def v_sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def v_mul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def v_dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def v_cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def v_len(a): return math.sqrt(v_dot(a, a))
def v_norm(a): l = v_len(a); return v_mul(a, 1.0 / l) if l > 0 else a
def lerp(p, q, t): return tuple(p[i] + (q[i] - p[i]) * t for i in range(3))
def mir(p, s): return (p[0], p[1], p[2] * s)


def new_node(p, kind, mm=None):
    nodes.append(tuple(p))
    node_kind.append(kind)
    node_mm.append(mm)
    return len(nodes) - 1


def fem_node(p):
    key = (round(p[0], 3), round(p[1], 3), round(p[2], 3))
    if key not in fem_at:
        fem_at[key] = new_node(key, "fem")
    return fem_at[key]


def outward(p, n):
    """n turned to face away from the car's middle"""
    o = (p[0] * 0.3, p[1] - 0.65, p[2])
    return n if v_dot(n, o) >= 0 else v_mul(n, -1)


# ------------------------------------------------------------------------------------------------------- the panels
class Panel:
    def __init__(self, name, mat):
        self.name, self.mat = name, mat
        self.tris = []     # node ids
        self.nodes = []    # its node ids


panels = []


def quad_panel(name, mat, p00, p10, p01, p11, nu, nv, dome=0.0, bow=0.0, hole=None, sides=False):
    """A panel over the bilinear quad (u from p00 to p10, v from p00 to p01), nu x nv cells, pushed out along its normal
    by `dome` sin(pi u) sin(pi v) (its edges stay on the frame) and `bow` sin(pi v) (a door's skin: its frame follows).
    `hole(point)`: cells whose centre it takes are left out (the wheel arches). Returns the panel and its grid of
    points (None where no triangle uses one)."""
    P = [[None] * (nv + 1) for _ in range(nu + 1)]
    for i in range(nu + 1):
        for j in range(nv + 1):
            u, v = i / nu, j / nv
            b = lerp(lerp(p00, p10, u), lerp(p01, p11, u), v)
            du = v_sub(lerp(p10, p11, v), lerp(p00, p01, v))
            dv = v_sub(lerp(p01, p11, u), lerp(p00, p10, u))
            n = outward(b, v_norm(v_cross(du, dv)))
            P[i][j] = v_add(b, v_mul(n, dome * math.sin(math.pi * u) * math.sin(math.pi * v) + bow * math.sin(math.pi * v)))
    keep = [[False] * nv for _ in range(nu)]
    for i in range(nu):
        for j in range(nv):
            c = v_mul(v_add(v_add(P[i][j], P[i + 1][j]), v_add(P[i + 1][j + 1], P[i][j + 1])), 0.25)
            keep[i][j] = hole is None or not hole(c)
    pan = Panel(name, mat)
    ids = [[None] * (nv + 1) for _ in range(nu + 1)]

    def nid(i, j):
        if ids[i][j] is None:
            ids[i][j] = new_node(P[i][j], "panel")
            pan.nodes.append(ids[i][j])
        return ids[i][j]
    for i in range(nu):
        for j in range(nv):
            if not keep[i][j]:
                continue
            a, b, c, d = nid(i, j), nid(i + 1, j), nid(i + 1, j + 1), nid(i, j + 1)
            if (i + j) % 2 == 0:
                pan.tris += [(a, b, c), (a, c, d)]
            else:
                pan.tris += [(a, b, d), (b, c, d)]
    if hole is not None and hasattr(hole, "centre"):
        # the arch's edge round: its nodes within a third of a cell of the circle (or inside it) put on it
        cx, cy = hole.centre
        cellw = v_len(v_sub(p10, p00)) / nu
        for i in range(nu + 1):
            for j in range(nv + 1):
                k = ids[i][j]
                if k is None:
                    continue
                x, y, z = nodes[k]
                d = math.hypot(x - cx, y - cy)
                if d < ARCH_R + 0.35 * cellw and d > 1e-6 and y > cy - 0.01:
                    nodes[k] = (cx + (x - cx) * ARCH_R / d, cy + (y - cy) * ARCH_R / d, z)
                    P[i][j] = nodes[k]
    panels.append(pan)
    return pan, [[P[i][j] if ids[i][j] is not None else None for j in range(nv + 1)] for i in range(nu + 1)]


def tri_panel(name, mat, A, B, C, n, dome=0.0):
    """A triangular panel A B C cut into n x n triangles (a rear quarter window)"""
    pan = Panel(name, mat)
    ids = {}
    nrm = outward(A, v_norm(v_cross(v_sub(B, A), v_sub(C, A))))

    def nid(i, j):
        if (i, j) not in ids:
            l1, l2 = i / n, j / n
            l0 = 1 - l1 - l2
            p = v_add(v_add(v_add(v_mul(A, l0), v_mul(B, l1)), v_mul(C, l2)), v_mul(nrm, dome * 27 * l0 * l1 * l2))
            ids[(i, j)] = new_node(p, "panel")
            pan.nodes.append(ids[(i, j)])
        return ids[(i, j)]
    for i in range(n):
        for j in range(n - i):
            pan.tris.append((nid(i, j), nid(i + 1, j), nid(i, j + 1)))
            if i + j < n - 1:
                pan.tris.append((nid(i + 1, j), nid(i + 1, j + 1), nid(i, j + 1)))
    panels.append(pan)
    return pan


def row(grid, i):
    return [p for p in grid[i] if p is not None]


def col(grid, j):
    return [grid[i][j] for i in range(len(grid)) if grid[i][j] is not None]


# ------------------------------------------------------------------------------------------ the frame (tubes, members)
members = []        # (a, b, section, joint at a or None, joint at b or None)
member_set = set()
tubes = []          # (polyline, section)
extra_splits = []   # points the tubes are divided at besides the panels' (hinges, latches, suspension mounts)
late = []           # members joined to the nearest frame nodes once the tubes are divided: (p, q, section, joint a, joint b)


def tube(points, sec):
    tubes.append(([tuple(p) for p in points], sec))


def member(a, b, sec, ja=None, jb=None):
    if a == b or (min(a, b), max(a, b)) in member_set:
        return
    member_set.add((min(a, b), max(a, b)))
    members.append((a, b, sec, ja, jb))


def arch(ax):
    f = lambda c: (c[0] - ax) ** 2 + (c[1] - Y0) ** 2 < ARCH_R ** 2
    f.centre = (ax, Y0)
    return f


def n_cells(a, b):
    return max(1, int(round(v_len(v_sub(a, b)) / GRID)))


# ---- the greenhouse and the front: windscreen, roof, hood (their shared edges divided alike)
NV_TOP = 11   # across the car (cowl, windscreen, roof, hatch)
cowl_l, cowl_r = (XA, YB, -ZS), (XA, YB, ZS)
ws_l, ws_r = (X_WS, YR_F, -ZT), (X_WS, YR_F, ZT)
rr_l, rr_r = (X_RR, YR_R, -ZT), (X_RR, YR_R, ZT)
apron_f = lambda s: (XF, 0.74, s * 0.66)
quad_panel("windscreen", "glass", cowl_l, ws_l, cowl_r, ws_r, 5, NV_TOP, dome=0.03)
_, roof = quad_panel("roof", "body", ws_l, rr_l, ws_r, rr_r, ROOF_NU, NV_TOP, dome=0.045)
quad_panel("hood", "body", cowl_l, apron_f(-1), cowl_r, apron_f(1), 8, NV_TOP, dome=0.05)
d_pt = lambda s, t: lerp((X_RR, YR_R, s * ZT), (-1.82, 0.93, s * 0.74), t)   # the D pillar
quad_panel("hatch glass", "glass", rr_l, d_pt(-1, D_T), rr_r, d_pt(1, D_T), 7, NV_TOP, dome=0.03)
quad_panel("hatch", "body", d_pt(-1, D_T), d_pt(-1, 1.0), d_pt(1, D_T), d_pt(1, 1.0), 3, NV_TOP, dome=0.01)
rpost = lambda s, y: lerp((-1.82, 0.93, s * 0.74), (XR, Y0, s * 0.70), (0.93 - y) / (0.93 - Y0))  # the rear posts
NV_SIDE = 5   # rows up the sides (fenders, quarters)
Y_TAIL = Y0 + (0.93 - Y0) * 3 / NV_SIDE   # (on the quarters' third row)
quad_panel("rear panel", "body", rpost(-1, 0.93), rpost(-1, Y_TAIL), rpost(1, 0.93), rpost(1, Y_TAIL), 2, NV_TOP, dome=0.02)
quad_panel("grille", "trim", (XF, 0.52, -0.66), (XF, 0.74, -0.66), (XF, 0.52, 0.66), (XF, 0.74, 0.66), 2, NV_TOP)
# the floor: under the cabin (between the sills), behind it between the rails
NV_FLOOR = 10   # (0.16 m: rows on the rails at +-0.48)
quad_panel("floor front", "floor", (XA, Y0, -ZS), (XB, Y0, -ZS), (XA, Y0, ZS), (XB, Y0, ZS), n_cells((XA, 0, 0), (XB, 0, 0)), NV_FLOOR)
quad_panel("floor rear", "floor", (XB, Y0, -ZS), (XC, Y0, -ZS), (XB, Y0, ZS), (XC, Y0, ZS), n_cells((XB, 0, 0), (XC, 0, 0)), NV_FLOOR)
quad_panel("boot floor", "floor", (XC, Y0, -ZR), (XR, Y0, -ZR), (XC, Y0, ZR), (XR, Y0, ZR), n_cells((XC, 0, 0), (XR, 0, 0)), 6)
# the bumpers: wrapped round the corners, off the crash beams (welds a few centimetres long)
fb, _ = quad_panel("front bumper", "trim", (2.00, 0.24, -0.78), (2.00, 0.24, 0.78), (2.00, 0.52, -0.78), (2.00, 0.52, 0.78), 12, 3,
                   bow=0.0)
rb, _ = quad_panel("rear bumper", "trim", (-1.96, 0.24, 0.78), (-1.96, 0.24, -0.78), (-1.96, 0.60, 0.78), (-1.96, 0.60, -0.78), 12, 3)
for pan, sgn in ((fb, 1), (rb, -1)):   # (pushed forward / back in the middle, round at the corners)
    for k in pan.nodes:
        x, y, z = nodes[k]
        nodes[k] = (x + sgn * 0.08 * math.cos(0.5 * math.pi * z / 0.78), y, z)

# ---- the sides: fenders, quarters, quarter windows; the doors further down
DOORS = {}
DOOR_NV = 5
for s in (1, -1):
    quad_panel("fender", "body", (XA, Y0, s * ZS), (XF, Y0, s * 0.66), (XA, YB, s * ZS), apron_f(s), 8, NV_SIDE, dome=0.035, hole=arch(AXLES[0]))
    quad_panel("quarter", "body", (XC, Y0, s * ZS), (XR, Y0, s * 0.70), (XC, YB, s * ZS), (-1.82, 0.93, s * 0.74), 10, NV_SIDE, dome=0.035,
               hole=arch(AXLES[1]))
    tri_panel("quarter glass", "glass", (XC, YB, s * ZS), (-1.82, 0.93, s * 0.74), (X_RR, YR_R, s * ZT), 10, dome=0.01)
    # the doors (their own frames: the skin's edges; a 2 cm gap to the pillars)
    for front in (True, False):
        x0, x1 = (XA - 0.02, XB + 0.02) if front else (XB - 0.02, XC + 0.02)
        y0 = 0.33
        skin, g = quad_panel("door", "body", (x0, y0, s * ZS), (x1, y0, s * ZS), (x0, YB, s * ZS), (x1, YB, s * ZS), n_cells((x0, 0, 0), (x1, 0, 0)), DOOR_NV,
                             bow=0.045)
        top0 = lerp((XA, YB, s * ZS), (X_WS, YR_F, s * ZT), 0.94) if front else (x0, 1.37, s * 0.67)
        top0 = (top0[0] - 0.02, top0[1] - 0.02, top0[2]) if front else top0
        top1 = (x1, 1.37, s * 0.67)
        win, _ = quad_panel("door glass", "glass", (x0, YB, s * ZS), (x1, YB, s * ZS), top0, top1, n_cells((x0, 0, 0), (x1, 0, 0)), 3, dome=0.01)
        DOORS[(s, front)] = dict(grid=g, top0=top0, top1=top1, skin=skin, win=win, x0=x0, x1=x1)

# ---- the lights and the mirrors: little closed boxes (a rectangle for a face, a point behind it) on beams that tear
lamps = []   # (nodes, tris, material, attach [(node index, anchor point)])


def lamp(mat, face, back, attach):
    """face: its 4 corners round; back: the point behind it"""
    ids = [new_node(p, "plain", 0.4) for p in face + [back]]
    a, b, c, d, e = ids
    lamps.append((ids, [(a, b, c), (a, c, d), (a, e, b), (b, e, c), (c, e, d), (d, e, a)], mat, attach))


for s in (1, -1):
    hx = 1.945
    lamp("headlight", [(hx, 0.57, s * 0.38), (hx, 0.57, s * 0.62), (hx, 0.70, s * 0.62), (hx, 0.70, s * 0.38)], (1.86, 0.635, s * 0.50),
         [(0, (XF, 0.52, s * 0.38)), (1, (XF, 0.52, s * 0.62)), (2, (XF, 0.74, s * 0.62)), (3, (XF, 0.74, s * 0.38))])
    t0, t1 = rpost(s, Y_TAIL), rpost(s, 0.93)
    tl = [lerp(t0, t1, 0.12), lerp(t0, t1, 0.88)]
    lamp("taillight", [(tl[0][0] - 0.03, tl[0][1], s * 0.56), (tl[0][0] - 0.03, tl[0][1], tl[0][2] - s * 0.01), (tl[1][0] - 0.03, tl[1][1], tl[1][2] - s * 0.01),
                       (tl[1][0] - 0.03, tl[1][1], s * 0.56)], (tl[0][0] + 0.06, 0.5 * (tl[0][1] + tl[1][1]), s * 0.64),
         [(0, (rpost(0, Y_TAIL)[0], Y_TAIL, s * 0.56)), (1, lerp(t0, t1, 0.0)), (2, lerp(t0, t1, 1.0)), (3, (-1.82, 0.93, s * 0.56))])
    a0 = lerp((XA, YB, s * ZS), (X_WS, YR_F, s * ZT), 0.05)
    a1 = lerp((XA, YB, s * ZS), (X_WS, YR_F, s * ZT), 0.16)
    lamp("trim", [(0.74, 0.96, s * 0.88), (0.74, 0.96, s * 1.02), (0.74, 1.07, s * 1.02), (0.74, 1.07, s * 0.88)], (0.82, 1.015, s * 0.93),
         [(0, a0), (3, a1), (1, a0)])
for ids, tris, mat, attach in lamps:
    extra_splits += [p for _, p in attach]

# ------------------------------------------------------------------------------------------------ the frame's tubes
for s in (1, -1):
    tube([(XF, Y0, s * ZR), (XR, Y0, s * ZR)], "rail")                                        # floor rail
    tube([(XA, Y0, s * ZS), (XC, Y0, s * ZS)], "rail")                                        # sill
    tube([(XA, Y0, s * ZS), (XA, YB, s * ZS), (X_WS, YR_F, s * ZT)], "pillar")               # A pillar
    tube([(XB, Y0, s * ZS), (XB, YB, s * ZS)] + [roof[ROOF_B][0 if s < 0 else NV_TOP]], "pillar")  # B pillar
    tube([(XC, Y0, s * ZS), (XC, YB, s * ZS), (X_RR, YR_R, s * ZT)], "pillar")               # C pillar
    tube([(X_WS, YR_F, s * ZT), (X_RR, YR_R, s * ZT)], "pillar")                             # roof rail
    tube([(X_RR, YR_R, s * ZT), (-1.82, 0.93, s * 0.74)], "pillar")                          # D pillar
    tube([(XC, YB, s * ZS), (-1.82, 0.93, s * 0.74)], "light")                               # rear belt rail
    tube([(-1.82, 0.93, s * 0.74), (XR, Y0, s * 0.70)], "light")                             # rear post
    tube([(XA, YB, s * ZS), apron_f(s)], "light")                                            # apron
    tube([(XF, Y0, s * 0.66), apron_f(s)], "light")                                          # front post
    for ax, xs in ((AXLES[0], (1.45, 1.15)), (AXLES[1], (-1.15, -1.45))):
        for x in xs:
            tube([(x, Y0, s * ZR), (x, 0.66, s * ZR)], "rail")                               # tower posts
            extra_splits.append((x, 0.50, s * ZR))                                           # (the upper wishbone's pivot)
            late.append(((x, Y0, s * ZR), (x, 0.17, s * ZR), "rail", None, None))           # lower wishbone brackets
        tube([(xs[0], 0.66, s * ZR), (xs[1], 0.66, s * ZR)], "rail")                         # tower top
        tube([(xs[0], 0.17, s * ZR), (xs[1], 0.17, s * ZR)], "rail")                         # lower pivot rail
        belt = lerp((XA, YB, s * ZS), apron_f(s), (ax - XA) / (XF - XA)) if ax > 0 else lerp((XC, YB, s * ZS), (-1.82, 0.93, s * 0.74), (XC - ax) / (XC + 1.82))
        tube([(ax, 0.66, s * ZR), belt], "pillar")                                           # tower brace
for x in (XA, XB, XC):
    tube([(x, Y0, -ZS), (x, Y0, ZS)], "rail")                                                # floor cross members
tube([(XF, Y0, -0.66), (XF, Y0, 0.66)], "rail")                                              # front crash beam
tube([(XR, Y0, -0.70), (XR, Y0, 0.70)], "rail")                                              # rear crash beam
tube([cowl_l, cowl_r], "pillar")                                                             # cowl
tube([ws_l, ws_r], "pillar")                                                                 # roof: front header
tube(row(roof, ROOF_B), "pillar")                                                                 # roof: the B bow (follows the roof's crown)
tube([rr_l, rr_r], "pillar")                                                                 # roof: rear header
tube([d_pt(-1, D_T), d_pt(1, D_T)], "light")                                                 # hatch: the glass's lower frame
tube([d_pt(-1, 1.0), d_pt(1, 1.0)], "light")                                                 # hatch: its bottom
tube([rpost(-1, Y_TAIL), rpost(1, Y_TAIL)], "light")                                         # the tail's lower cross
tube([(XF, 0.52, -0.66), (XF, 0.52, 0.66)], "light")                                         # the front's middle cross
tube([apron_f(-1), apron_f(1)], "light")                                                     # the front's upper cross
RACK_X = AXLES[0] - 0.18
tube([(RACK_X, Y0, -ZR), (RACK_X, Y0, 0.0), (RACK_X, Y0, ZR)], "rack")                       # the rack housing

# the doors' frames: round their skins and windows (the tubes along their edges, bowed with the skin)
HINGE_ROWS, LATCH_ROW = (1, 4), 2
for (s, front), d in DOORS.items():
    g = d["grid"]
    nu = len(g) - 1
    tube(col(g, 0), "door")                              # bottom
    tube(col(g, DOOR_NV), "door")                        # belt
    tube(row(g, 0), "door")                              # front edge
    tube(row(g, nu), "door")                             # rear edge
    tube([g[0][DOOR_NV], d["top0"], d["top1"], g[nu][DOOR_NV]], "door")   # the window's frame
    xp = XA if front else XB                             # the hinges' pillar and the latch's
    xl = XB if front else XC
    for j in HINGE_ROWS + (LATCH_ROW,):   # (the pillars' nodes for the hinges and the latch, the door's own)
        extra_splits.append((xp if j in HINGE_ROWS else xl, g[0][j][1], s * ZS))
        extra_splits.append(g[0][j] if j in HINGE_ROWS else g[nu][j])
    extra_splits.append((xl, Y0, 0.0))    # (the check strap's anchor on the floor's cross member)
extra_splits += [(XB, Y0, 0.0), (XC, Y0, 0.0), (XB, Y0, ZS)]   # (the cameras' nodes)

# the suspension's points on the frame (the tubes are divided there too)
SUSP = []
for ax in AXLES:
    front = ax > 0
    for s in (1, -1):
        arm_x = ax - 0.18 if front else ax + 0.18
        SUSP.append(dict(ax=ax, s=s, front=front, bl=(ax, 0.17, s * 0.62), bu=(ax, 0.50, s * 0.59), n1=(ax, WHEEL_R, s * 0.62),
                         n2=(ax, WHEEL_R, s * 0.82), sa=(arm_x, WHEEL_R, s * 0.64), top=(ax, 0.66, s * ZR),
                         lower=[(x, 0.17, s * ZR) for x in ((1.45, 1.15) if front else (-1.15, -1.45))],
                         upper=[(x, 0.50, s * ZR) for x in ((1.45, 1.15) if front else (-1.15, -1.45))],
                         link=(arm_x, Y0, s * ZR)))
        extra_splits += [(ax, 0.66, s * ZR), (arm_x, Y0, s * ZR)]


def on_segment(p, a, b, tol=0.002):
    ab = v_sub(b, a)
    l2 = v_dot(ab, ab)
    t = v_dot(v_sub(p, a), ab) / l2
    if t <= 1e-4 or t >= 1 - 1e-4:
        return None
    q = v_add(a, v_mul(ab, t))
    return t if v_len(v_sub(p, q)) < tol else None


# every tube divided at the other tubes' ends and the extra points, and at the panels' nodes on it no closer than
# SEAM apart (about every other one: a frame node under it, the same point); the panel's nodes between are welded to
# the tube's point under them
SEAM = 0.20
panel_pts = [nodes[k] for pan in panels for k in pan.nodes]
key_pts = [p for pts, _ in tubes for p in pts] + extra_splits
seg_members = []   # (a, b, point a, point b): the tubes' members, for the welds between their nodes
for pts, sec in tubes:
    for a, b in zip(pts, pts[1:]):
        L = v_len(v_sub(b, a))
        keys = sorted({round(t, 6) for p in key_pts for t in [on_segment(p, a, b)] if t is not None} | {0.0, 1.0})
        pans = sorted({round(t, 6) for p in panel_pts for t in [on_segment(p, a, b)] if t is not None} - set(keys))
        ts = list(keys)
        for t in pans:   # (the panel's nodes kept as far as they are from the ones kept and the fixed ones)
            if all(abs(t - u) * L >= SEAM * 0.999 for u in ts):
                ts.append(t)
        ts = sorted(ts)
        chain = [fem_node(lerp(a, b, t)) for t in ts]
        for m, n in zip(chain, chain[1:]):
            member(m, n, sec)
            seg_members.append((m, n))

fem_ids = [k for k, kind in enumerate(node_kind) if kind == "fem"]
cell = {}
for k in fem_ids:
    cell.setdefault(tuple(int(math.floor(c / 0.05)) for c in nodes[k]), []).append(k)


def fem_near(p, r):
    """the frame node nearest p within r (None: none)"""
    c = tuple(int(math.floor(x / 0.05)) for x in p)
    best, bd = None, r
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                for k in cell.get((c[0] + dx, c[1] + dy, c[2] + dz), []):
                    d = v_len(v_sub(nodes[k], p))
                    if d < bd:
                        best, bd = k, d
    return best


def fem_get(p, r=0.07):
    k = fem_near(p, r)
    assert k is not None, "no frame node near %s" % (p,)
    return k


for p, q, sec, ja, jb in late:
    member(fem_get(p), fem_get(q), sec, ja, jb)

# ---- the suspension: wishbones on ball joints at the frame, welded at their outer ends; the upright on ball joints
shocks, stops, hydros, wheels, slides = [], [], [], [], []
rack_ends = {}
for sp in SUSP:
    s, front = sp["s"], sp["front"]
    bl, bu, n1, n2, sa = (fem_node(sp[k]) for k in ("bl", "bu", "n1", "n2", "sa"))
    for p in sp["lower"]:
        member(fem_get(p, 0.002), bl, "arm", ja="ball")
    for p in sp["upper"]:
        member(fem_get(p, 0.002), bu, "arm", ja="ball")
    member(bl, n1, "hub", ja="ball")
    member(n1, bu, "hub", jb="ball")
    member(n1, n2, "hub")
    member(n1, sa, "hub")
    shocks.append((bl, fem_get(sp["top"], 0.002), PRELOAD[front]))
    wheels.append((n1, n2, fem_get(sp["top"], 0.002), front))
    if front:
        re = new_node((RACK_X, Y0, s * 0.30), "plain", 3.0)
        rack_ends[s] = re
        member(re, sa, "tierod")
        slides.append(re)
        # the steering stop: a limit beside the arm, to the housing's end, free through the lock and 4 degrees past it
        kp = lerp(sp["bl"], sp["bu"], (WHEEL_R - 0.17) / (0.50 - 0.17))
        r = v_sub(sp["sa"], kp)
        end = sp["link"]
        L0 = v_len(v_sub(sp["sa"], end))
        ds = []
        for deg in (-34.0, 34.0):
            c, sn = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            p = v_add(kp, (r[0] * c - r[2] * sn, r[1], r[0] * sn + r[2] * c))
            ds.append(v_len(v_sub(p, end)))
        stops.append((sa, fem_get(end, 0.002), (L0 - min(ds)) / L0, (max(ds) - L0) / L0))
    else:
        member(sa, fem_get(sp["link"]), "arm", ja="ball", jb="ball")   # the toe link (to the rail's nearest node)
        # the toe stop: a limit to the lower wishbone's rear pivot, free 6 degrees each way (a frontal crash lifted
        # the tail, the wheel hung far down and its short toe link turned it through 90 degrees)
        kp = lerp(sp["bl"], sp["bu"], (WHEEL_R - 0.17) / (0.50 - 0.17))
        r = v_sub(sp["sa"], kp)
        end = sp["lower"][0]
        L0 = v_len(v_sub(sp["sa"], end))
        ds = []
        for deg in (-6.0, 6.0):
            c, sn = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            ds.append(v_len(v_sub(v_add(kp, (r[0] * c - r[2] * sn, r[1], r[0] * sn + r[2] * c)), end)))
        stops.append((sa, fem_get(end, 0.002), (L0 - min(ds)) / L0, (max(ds) - L0) / L0))
member(rack_ends[1], rack_ends[-1], "rack")   # the rack bar
RACK_TRAVEL = 0.18 * math.sin(math.radians(30.0))
hydros.append((rack_ends[1], fem_get((RACK_X, Y0, -ZR), 0.002), RACK_TRAVEL / (ZR + 0.30)))

# ---- the doors: two hinges (a short member from the pillar, a ball joint at the door), a latch that breaks, a strap
latches, door_straps = [], []
for (s, front), d in DOORS.items():
    g = d["grid"]
    nu = len(g) - 1
    xp, xl = (XA, XB) if front else (XB, XC)
    for j in HINGE_ROWS:
        member(fem_get((xp, g[0][j][1], s * ZS)), fem_get(g[0][j], 0.002), "hinge", jb="ball")
    dl = fem_get(g[nu][LATCH_ROW], 0.002)
    latches.append((dl, fem_get((xl, g[nu][LATCH_ROW][1], s * ZS))))
    door_straps.append((dl, fem_get((xl, Y0, 0.0), 0.002)))

# ---- the welds: every panel node on a frame node (the same point), and the bumpers a few centimetres off their beams
welds = []
on_member = 0
for pan in panels:
    if pan.name in ("front bumper", "rear bumper"):
        continue
    for k in pan.nodes:
        f = fem_near(nodes[k], 0.002)
        if f is not None:
            welds.append((f, k))
            continue
        for m, n in seg_members:   # (on a tube between its nodes: held at that point of it)
            t = on_segment(nodes[k], nodes[m], nodes[n])
            if t is not None:
                welds.append((m, k, n, t))
                on_member += 1
                break
for pan, beams in ((fb, [(XF, Y0), (XF, 0.52)]), (rb, [(XR, Y0), rpost(0, Y_TAIL)[:2]])):
    for k in fem_ids:
        x, y, z = nodes[k]
        if any(abs(x - bx) < 0.02 and abs(y - by) < 0.02 for bx, by in beams):
            near = min(pan.nodes, key=lambda n: v_len(v_sub(nodes[n], nodes[k])))
            if v_len(v_sub(nodes[near], nodes[k])) < 0.16:
                welds.append((k, near))

# ---- the lamps' beams: their boxes, and to the frame
lamp_beams, lamp_mounts = [], []
for ids, tris, mat, attach in lamps:
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            lamp_beams.append((ids[i], ids[j]))
    for i, p in attach:
        lamp_mounts.append((ids[i], fem_get(p, 0.002)))
    lamp_mounts.append((ids[-1], fem_get(attach[0][1], 0.002)))   # (the back point too: it does not turn about the face)
assert all(m is not None for _, m in lamp_mounts), "a lamp's mount is not on the frame"

# ---- cab triangles facing out
all_tris = []   # (a, b, c, material)
for pan in panels:
    for t in pan.tris:
        all_tris.append(t + (pan.mat,))
for ids, tris, mat, attach in lamps:
    for t in tris:
        all_tris.append(t + (mat,))
lamp_of = {x: ids for ids, _, _, _ in lamps for x in ids}
for i, (a, b, c, m) in enumerate(all_tris):
    pa, pb, pc = nodes[a], nodes[b], nodes[c]
    n = v_cross(v_sub(pb, pa), v_sub(pc, pa))
    mid = v_mul(v_add(v_add(pa, pb), pc), 1.0 / 3.0)
    if a in lamp_of:   # (a lamp's faces: out of its own box)
        ls = lamp_of[a]
        ctr = v_mul(v_add(v_mul(v_add(v_add(nodes[ls[0]], nodes[ls[1]]), v_add(nodes[ls[2]], nodes[ls[3]])), 0.25), nodes[ls[4]]), 0.5)
        out = v_sub(mid, ctr)
    else:
        out = (mid[0] * 0.3, mid[1] - 0.65, mid[2])
    if v_dot(n, out) < 0:
        all_tris[i] = (a, c, b, m)

# ---------------------------------------------------------------------------------------------------------- write
os.makedirs(os.path.dirname(OUT), exist_ok=True)


def spec(sec):
    m, sh, o, w, j = SECTIONS[sec]
    return "%s, %s, %.4f, %.4f, %s" % (m, sh, o, w, j)


def P(k):  # RoR space
    x, y, z = nodes[k]
    return (-x, y, z)


centre, back, left = fem_node((XB, Y0, 0.0)), fem_node((XC, Y0, 0.0)), fem_node((XB, Y0, ZS))
head = (0.05, 1.02, 0.36)
near = sorted(fem_ids, key=lambda k: v_len(v_sub(nodes[k], head)))[:8]
with open(OUT, "w") as f:
    f.write("Frame Car\n")
    f.write(";generated by tools/make_frame_car.py: a five-door hatchback on a welded chromoly space frame (FEM frame elements),\n")
    f.write(";its panels sheet metal and glass (triangle elements) welded on, four doors on hinges with latches that break\n")
    f.write("globals\n;dry mass, cargo mass, cab material (sheet/<material>/<kg per m2>/<drawn thickness>: the panels are a sheet body)\n")
    f.write("520.0, 0.0, sheet/Steel/7.9/0.004/%d\n" % REFINE)
    f.write("minimass\n0.05\n")
    f.write("nodes\n;id, x, y, z, options (the frame's nodes, the panels', the lamps', the rack's)\n")
    mm = None
    for i in range(len(nodes)):
        if node_mm[i] != mm:
            mm = node_mm[i]
            f.write("set_default_minimass %s\n" % ("-1" if mm is None else "%.2f" % mm))
        x, y, z = P(i)
        f.write("%d, %.4f, %.4f, %.4f, n\n" % (i, x, y, z))
    f.write("beams\n")
    by_sec = {}
    for a, b, sec, ja, jb in members:
        by_sec.setdefault(sec, []).append((a, b, ja, jb))
    for sec, lst in by_sec.items():
        f.write(";%s\nset_beam_defaults 3000000, 400, 80000, 700000, 0.05, tracks/beam, 0\nset_frame_section %s\n" % (sec, spec(sec)))
        default = SECTIONS[sec][4]
        for a, b, ja, jb in lst:
            if ja or jb:
                f.write("%d, %d, F, %s, %s\n" % (a, b, ja or default, jb or default))
            else:
                f.write("%d, %d, F\n" % (a, b))
    f.write(";door latches: they hold till 12 kN, then break (the door swings open)\n")
    f.write("set_beam_defaults 1000000, 300, 12000, 12000, 0.02, tracks/beam, 0\n")
    for a, b in latches:
        f.write("%d, %d, i\n" % (a, b))
    f.write(";the lamps' and the mirrors' boxes, and their mounts (they tear at 1.5 kN)\n")
    f.write("set_beam_defaults 200000, 60, 1000000000, 1000000000, 0.01, tracks/beam, 0\n")
    for a, b in lamp_beams:
        f.write("%d, %d, i\n" % (a, b))
    f.write("set_beam_defaults 200000, 60, 1500, 1500, 0.01, tracks/beam, 0\n")
    for a, b in lamp_mounts:
        f.write("%d, %d, i\n" % (a, b))
    f.write("shocks\n;n1, n2, spring, damp, short bound, long bound, precompression, options\n")
    f.write("set_beam_defaults 9000000, 12000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, pre in shocks:
        f.write("%d, %d, 60000, 4000, 0.25, 0.25, %.3f, n\n" % (a, b, pre))
    # travel stops beside each coil-over (a bump stop at 20% compression, a droop strap at 4% extension)
    f.write("set_beam_defaults 20000000, 200000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, _ in shocks:
        f.write("%d, %d, 0, 0, 0.20, 0.04, 1.0, i\n" % (a, b))
    # steering stops (free through the lock and 4 degrees past it) and the rear toe stops (6 degrees)
    f.write("set_beam_defaults 3000000, 20000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, lo, hi in stops:
        f.write("%d, %d, 0, 0, %.3f, %.3f, 1.0, i\n" % (a, b, lo, hi))
    # the doors' check straps: the latch's node to the floor's middle; a door does not swing in, and opens to ~75 deg
    f.write("set_beam_defaults 500000, 3000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b in door_straps:
        f.write("%d, %d, 0, 0, 0.01, 1.20, 1.0, i\n" % (a, b))
    f.write("hydros\n;the steering rack: node1, node2, factor, options\n")
    f.write("set_beam_defaults 4000000, 2000, 99999999999999999999999999999999999999999, 99999999999999999999999999999999999999999, 0.02, tracks/beam, 0\n")
    for a, b, fac in hydros:
        f.write("%d, %d, %.4f, i\n" % (a, b, fac))
    f.write("slidenodes\n;the rack's ends slide along its housing\n")
    housing = [fem_node((RACK_X, Y0, -ZR)), fem_node((RACK_X, Y0, 0.0)), fem_node((RACK_X, Y0, ZR))]
    for n in slides:
        f.write("%d, %s, S2000000, T0\n" % (n, ", ".join(str(h) for h in housing)))
    f.write("wheels\n;radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, face, band\n")
    f.write("set_beam_defaults 3000000, 400, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for n1, n2, arm, front in wheels:
        f.write("%.2f, 0.20, 12, %d, %d, 9999, 1, %d, %d, 45.0, 120000.0, 900.0, tracks/wheelface tracks/wheelband\n" % (WHEEL_R, n1, n2, 0 if front else 1, arm))
    f.write("engine\n;min rpm, max rpm, torque, differential, reverse, neutral, gears...\n1000.0, 6800.0, 260.0, 4.1, 3.2, 1.0, 3.3, 2.1, 1.5, 1.15, 0.9, -1.0\n")
    f.write("engoption\n0.35, c, 200.0, 0.3, 0.4, 0.3\n")
    f.write("brakes\n4000\n")
    f.write("cameras\n%d, %d, %d\n" % (centre, back, left))
    x, y, z = head
    f.write("cinecam\n%.3f, %.3f, %.3f, %s\n" % (-x, y, z, ", ".join(str(k) for k in near)))
    f.write("contacters\n")
    for i in range(len(nodes)):
        f.write("%d\n" % i)
    f.write("welds\n;anchor (a frame node), sheet node, radius m, strength N\n")
    for w in welds:
        if len(w) == 2:
            f.write("%d, %d, %.3f, %.0f\n" % (w[0], w[1], WELD_R, WELD_BRK))
        else:
            f.write("%d, %d, %.3f, %.0f, 0, %d, %.4f\n" % (w[0], w[1], WELD_R, WELD_BRK, w[2], w[3]))
    f.write("submesh\ntexcoords\n")
    used = sorted({k for t in all_tris for k in t[:3]})
    for k in used:
        x, y, z = nodes[k]
        f.write("%d, %.3f, %.3f\n" % (k, (x + 2.1) / 4.2, (z + 1.0 + (y - Y0)) / 3.0))
    f.write("cab\n")
    for a, b, c, m in all_tris:
        f.write("%d, %d, %d, c\n" % (a, b, c))
    f.write("shells\n")
    cur = None
    for a, b, c, m in sorted(all_tris, key=lambda t: list(MAT).index(t[3])):
        if m != cur:
            cur = m
            if MAT[m] is None:
                f.write("set_shell_material default\n")
            else:
                mt, kg, th, col = MAT[m]
                f.write("set_shell_material %s, %s, %.1f, %.3f, %.2f, %.2f, %.2f, %d\n" % (m, mt, kg, th, col[0], col[1], col[2], REFINE))
        f.write("%d, %d, %d\n" % (a, b, c))
    f.write("end\n")
n_fem = len(fem_ids)
print("wrote %s: %d nodes (%d frame, %d on panels), %d frame elements, %d triangles in %d panels, %d welds (%d on a member), %d lamps, %d doors" % (
    OUT, len(nodes), n_fem, sum(len(p.nodes) for p in panels), len(members), len(all_tris), len(panels), len(welds), on_member, len(lamps), len(DOORS)))
