#!/usr/bin/env python3
"""Writes assets/vehicles/frame_car/frame_car.truck: a five-door hatchback on a welded space frame of FEM frame elements.

The frame is chromoly tube, welded at its joints (phys/frame_fem.h), as a car's body in white is laid out: floor rails
from the front crash beam to the rear one, sills, cross members under the firewall, the B and the C pillars, the
A, B, C and D pillars, roof rails and bows, the cowl, the aprons over the front wheels, the towers, the rear belt
rails and posts. The panels are sheet metal and glass (triangle elements) on it, each with nodes of its own: where a
panel's edge (or a row of it) runs along a tube, the tube is divided at the panel's nodes, a frame node under every
one of them, and each such pair is a weld (`welds`, phys::SoftBody::Weld) that holds the panel there by the weighted
centre of its nodes round the weld (the pull spread over them, falling off with the distance), and lets go past its
strength. The panels stand GAP off their tubes (out along their normals), the welds holding them there.

The parts that come off are frames of their own, their skins welded to them, held on the body at a distance by mounts
(`mounts`, phys::FrameMount: the part's node held at a point carried by the body's nodes round it, no member between
them) that let go past their break force (weaker than anything of the body's frame); each part is a component of the
frame solved on its own, beside the body's:
- the doors: a frame round the skin and the window, one intrusion beam across it (the skin welded to it too), two
  hinges (mounts on a vertical line, their turning damped) and a latch that breaks (then they swing open, a check strap
  ending their travel both ways);
- the hood: a frame round its skin and ribs under it (two along, one across), on two damped hinges at the cowl, a latch
  at the front, stays that end its opening; the tailgate alike (hinges at the roof, the latch at the bottom);
- the front fenders: a frame round each (and round the wheel arch) on four bolts to the apron, the A pillar and the
  front post;
- the bumpers: plastic covers, a profile swept round the corners to the wheel arches, on a frame of plastic tube (two
  rails, ribs) on plastic clips a few centimetres off the crash beam and the cross above it;
- the mirrors: plastic boxes on an arm of plastic tube on three feet on the front door, that bends and tears off.
The head and tail lights are little glass boxes, every node on beams to four frame nodes round the box, that tear.

Suspension: double wishbones of tube on ball joints at the frame, the upright (the stub and a steering arm welded
together) on ball joints at their ends (the ball joints damped: a wishbone torn loose does not swing freely). A
coil-over from the lower ball joint to the tower carries the car, a travel stop beside it ends the wheel's travel. At
the front a steering rack: a bar of tube whose ends slide along the rack housing (slide nodes), moved by one hydro,
arms of it out to the wishbones' pivot line, and from them tie rods of tube on ball joints to the steering arms (as
long as the wishbones: no bump steer); a stop beside each steering arm ends its turn a few degrees past the lock. At
the rear a toe link of tube holds the wheel straight.
All-wheel drive (driven at the rear alone its tyres slipped at a third of a g and the traction assist held it back).

Coordinates follow Rigs of Rods: -x forward, y up, +z left. The design is written with the front at +x and mirrored
when the nodes are written.

    python3 tools/make_frame_car.py
"""
import math
import os

OUT = os.environ.get("FC_OUT") or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "vehicles", "frame_car", "frame_car.truck")

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
# the heavy parts low down, as a car's: the engine and gearbox on the floor rails' nodes in the engine bay, the fuel tank
# and the battery on the rear floor's (kg); the dry mass spread over every node is less by as much. Spread over all the
# nodes (the roof's, the glass's) the car's centre was high: it rolled over in a tight turn at 40 km/h
# The dry mass is spread over the members by their length (the builder, as RoR over its beams): the rest of the car's
# mass (its interior, the seats, the trim) on the body's floor-level frame nodes, each alike - spread over every member
# the bolted-on parts' frames took their share (a hood of 35 kg, a bumper of 25: their hinges and bolts let go landing
# from 5 m); on all the body's nodes alike the roof's took theirs, and the car rolled over in a turn at 40 km/h
ENGINE_KG, TANK_KG, DRY_KG, BODY_KG = 140.0, 40.0, 60.0, 280.0

# name: (material, shape, outer, wall, joints[, break force N (the member tears off its first node)[, the released
# joint's damping N m s/rad]]). The parts' mounts break far below the body's own tubes (a 50 mm rail yields at 180 kN)
SECTIONS = {
    "rail": ("Chromoly", "tube", 0.050, 0.0025, "rigid"),     # floor rails, sills, cross members, crash beams
    "pillar": ("Chromoly", "tube", 0.045, 0.0025, "rigid"),   # pillars, roof rails and bows, cowl
    "light": ("Chromoly", "tube", 0.030, 0.0020, "rigid"),    # aprons, rear rails and posts, towers, the front's crosses
    "door": ("Chromoly", "tube", 0.028, 0.0020, "rigid"),     # the doors' frames
    "doorbeam": ("Chromoly", "tube", 0.032, 0.0025, "rigid"), # the doors' intrusion beams (one across each door)
    "panel": ("Steel", "tube", 0.024, 0.0015, "rigid"),       # the hood's, the fenders' and the tailgate's frames and ribs
    "bumper": ("Plastic", "tube", 0.026, 0.0040, "rigid"),               # the bumpers' frames
    "mirror": ("Plastic", "tube", 0.018, 0.0030, "rigid"),               # the mirrors' arms: two stalks, a triangle at the tip
    "arm": ("Chromoly", "tube", 0.035, 0.0030, "rigid", 0.0, 3.0),       # wishbones, toe links
    "tierod": ("Chromoly", "tube", 0.028, 0.0040, "ball"),    # tie rods
    "rack": ("Steel", "tube", 0.030, 0.0050, "rigid"),        # the rack bar and its housing
    "hub": ("Steel", "tube", 0.050, 0.0060, "rigid", 0.0, 3.0),          # the uprights
}
# The parts on the body (`mounts`, phys::FrameMount): the part's frame node held at the point of the body's where it
# stands, a few centimetres off (no member between them: every part a component of the frame solved on its own, beside
# the body's), letting go when the force stands past the break force for 5 ms; far below anything of the body's frame
# (a 50 mm rail yields at 180 kN). The spring is explicit: a steel part's stiff frame takes a stiff one, a plastic
# part's frame is soft across and its node alone holds the spring (5e5 N/m on a bumper's 100 g node blew it off at
# 50 kN as the car stood): there the most the node's mass takes at the step (0), the node given the mass of the part's
# hardware there. name: (break force N, stiffness N/m, turning damping N m s/rad, the part node's hardware kg)
MOUNTS = {
    "door hinge": (20000.0, 2.0e6, 4.0, 1.0),     # (at 12 kN a side impact tore the hit doors off)
    "lid hinge": (15000.0, 1.5e6, 2.0, 0.5),      # the hood's and the tailgate's (at 10 kN a hood lost one landing from 5 m)
    "fender bolt": (9000.0, 1.5e6, 0.0, 0.2),     # (a ring on four points: the body's front bending as it lands from 5 m pulls them)
    "bumper bracket": (2500.0, 0.0, 0.0, 0.4),    # plastic clips
    "mirror": (500.0, 0.0, 0.0, 0.05),            # the mirror's stalks on the door
}
MAT = {  # shell materials: name -> (material, kg/m2, drawn thickness, colour or None: the body's)
    "body": None,
    "glass": ("Glass", 10.0, 0.004, (0.30, 0.38, 0.44)),
    "trim": ("Steel", 6.0, 0.006, (0.10, 0.10, 0.11)),
    "floor": ("Steel", 7.9, 0.004, (0.22, 0.22, 0.23)),
    "headlight": ("Glass", 6.0, 0.006, (0.95, 0.93, 0.80)),
    "taillight": ("Glass", 6.0, 0.006, (0.75, 0.05, 0.04)),
    "plastic": ("Plastic", 3.0, 0.006, (0.13, 0.13, 0.14)),   # the bumpers' covers, the mirrors (3 mm polypropylene)
}
# weld radius (m), strength (N; FC_WELD_BRK to try others: at 3500 the panels hardly came off, at 1500 a head-on shed
# two thirds of them)
WELD_R, WELD_BRK = 0.17, float(os.environ.get("FC_WELD_BRK", "2500"))
# the panels stand off their tubes (the tube's radius and a centimetre: a panel on the tube's axis had the tube through
# it); their welds hold them there. The glass stays in its frame's plane
GAP = float(os.environ.get("FC_GAP", "0.035"))
# the glass's welds: its seal (a door window in rubber channels, a windscreen on its bead), N/m. Welded as stiff as the
# metal and off the frame, the front door window cracked at its sharp top corner as the car settled on its wheels: a
# 5 cm edge there, 1% of it half a millimetre, a frame node's turn of a hundredth of a radian on the weld's offset
GLASS_WELD_K = float(os.environ.get("FC_GLASS_K", "10000"))
# the lamps: every node of a box on beams to four frame nodes round it, not on one plane (a mirror's beams went to two
# points on the pillar and it swung about it; a headlight's five mounts left it one free motion), each beam letting go
# at LAMP_BRK (N)
LAMP_BRK = 2500.0
# latches (N): a door's, the hood's and the tailgate's
DOOR_LATCH, LID_LATCH = 12000.0, 6000.0
# the parts' frames off the body's tubes (m): the hood's over the aprons and the cowl, the fenders' out of the aprons
# and the posts, the tailgate's out of the roof's rear and the D pillars (a part's frame on the body's tube would be
# one node with it)
HOOD_UP, FENDER_OUT, TG_OUT = 0.03, 0.03, 0.03
# the panels' refinement depth (a dent's triangles halved once: at 3 levels a crash made 7000 triangles of 2200, and
# the membrane's projection, one body's own work, took most of the step)
REFINE = int(os.environ.get("FC_REFINE", "1"))

# -------------------------------------------------------------------------------------------------------- the machinery
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fem_car import *   # noqa: E402,F401,F403 (nodes, panels, tubes, members, welds, mounts, lamps, the hull, the writer)
configure(centre_y=0.65, grid=GRID, seam=0.20, arch_r=ARCH_R, arch_y=Y0, gap=GAP, weld_r=WELD_R, weld_brk=WELD_BRK, glass_weld_k=GLASS_WELD_K,
          lamp_brk=LAMP_BRK, refine=REFINE)


# ---- the greenhouse: windscreen, roof (their shared edges divided alike)
NV_TOP = 11   # across the car (cowl, windscreen, roof, hood, hatch)
cowl_l, cowl_r = (XA, YB, -ZS), (XA, YB, ZS)
ws_l, ws_r = (X_WS, YR_F, -ZT), (X_WS, YR_F, ZT)
rr_l, rr_r = (X_RR, YR_R, -ZT), (X_RR, YR_R, ZT)
apron_f = lambda s: (XF, 0.74, s * 0.66)
quad_panel("windscreen", "glass", cowl_l, ws_l, cowl_r, ws_r, 5, NV_TOP, dome=0.03)
_, roof = quad_panel("roof", "body", ws_l, rr_l, ws_r, rr_r, ROOF_NU, NV_TOP, dome=0.045)
d_pt = lambda s, t: lerp((X_RR, YR_R, s * ZT), (-1.82, 0.93, s * 0.74), t)   # the D pillar
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

# ---- the hood: its own frame HOOD_UP over the aprons, the cowl and the front's upper cross
hood_r = lambda s: (XA + 0.02, YB + HOOD_UP, s * ZS)
hood_f = lambda s: (XF + 0.01, 0.74 + HOOD_UP, s * 0.66)
HOOD_NU = 8
_, hood = quad_panel("hood", "body", hood_r(-1), hood_f(-1), hood_r(1), hood_f(1), HOOD_NU, NV_TOP, dome=0.05, part="hood")

# ---- the tailgate: its own frame TG_OUT out of the opening (the roof's rear, the D pillars, the rear panel's top)
tg_o = v_mul(v_norm((-(YR_R - 0.93), X_RR + 1.82, 0.0)), TG_OUT)   # (back and up, square to the hatch)
TG_GLASS_NU, TG_SKIN_NU = 7, 3
_, tg_glass = quad_panel("hatch glass", "glass", v_add(rr_l, tg_o), v_add(d_pt(-1, D_T), tg_o), v_add(rr_r, tg_o), v_add(d_pt(1, D_T), tg_o), TG_GLASS_NU,
                         NV_TOP, dome=0.03, part="tailgate")
_, tg_skin = quad_panel("hatch", "body", v_add(d_pt(-1, D_T), tg_o), v_add(d_pt(-1, 1.0), tg_o), v_add(d_pt(1, D_T), tg_o), v_add(d_pt(1, 1.0), tg_o),
                        TG_SKIN_NU, NV_TOP, dome=0.01, part="tailgate")

# ---- the sides: fenders, quarters, quarter windows; the doors further down
DOORS, FENDERS = {}, {}
DOOR_NV = 5
FENDER_NU = 8
for s in (1, -1):
    # the fender: its own frame FENDER_OUT out of the apron, the A pillar and the front post, round the wheel arch too
    o = (0.0, 0.0, s * FENDER_OUT)
    fen, fg = quad_panel("fender", "body", v_add((XA, Y0, s * ZS), o), v_add((XF, Y0, s * 0.66), o), v_add((XA, YB, s * ZS), o), v_add(apron_f(s), o),
                         FENDER_NU, NV_SIDE, dome=0.035, hole=arch(AXLES[0]), part="fender %d" % s)
    cx, cy = AXLES[0], Y0
    arc = sorted({nodes[k] for k in fen.nodes if abs(math.hypot(nodes[k][0] - cx, nodes[k][1] - cy) - ARCH_R) < 1e-6},
                 key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    FENDERS[s] = dict(grid=fg, arc=arc, o=o, part="fender %d" % s)
    quad_panel("quarter", "body", (XC, Y0, s * ZS), (XR, Y0, s * 0.70), (XC, YB, s * ZS), (-1.82, 0.93, s * 0.74), 10, NV_SIDE, dome=0.035,
               hole=arch(AXLES[1]))
    tri_panel("quarter glass", "glass", (XC, YB, s * ZS), (-1.82, 0.93, s * 0.74), (X_RR, YR_R, s * ZT), 10, dome=0.01)
    # the doors (their own frames: the skin's edges; a 2 cm gap to the pillars)
    for front in (True, False):
        part = "door %d %s" % (s, "front" if front else "rear")
        x0, x1 = (XA - 0.02, XB + 0.02) if front else (XB - 0.02, XC + 0.02)
        y0 = 0.33
        skin, g = quad_panel("door", "body", (x0, y0, s * ZS), (x1, y0, s * ZS), (x0, YB, s * ZS), (x1, YB, s * ZS), n_cells((x0, 0, 0), (x1, 0, 0)), DOOR_NV,
                             bow=0.045, part=part)
        top0 = lerp((XA, YB, s * ZS), (X_WS, YR_F, s * ZT), 0.94) if front else (x0, 1.37, s * 0.67)
        top0 = (top0[0] - 0.02, top0[1] - 0.02, top0[2]) if front else top0
        top1 = (x1, 1.37, s * 0.67)
        win, _ = quad_panel("door glass", "glass", (x0, YB, s * ZS), (x1, YB, s * ZS), top0, top1, n_cells((x0, 0, 0), (x1, 0, 0)), 3, dome=0.01, part=part)
        DOORS[(s, front)] = dict(grid=g, top0=top0, top1=top1, skin=skin, win=win, x0=x0, x1=x1, part=part)

# ---- the bumpers: a profile swept along a plan line round the corners (a quarter superellipse from the face's middle
# to the wing's end on each side), a plastic cover on a frame of plastic tube
# (the bottom's 29 cm is 25 in the gap off the frame, and 21 as the car sits: at 22 the front one scraped the road in a
# turn at 35 km/h)
BUMPER_PROFILE = {  # [(depth in from the plan line, y)]: the bottom's back edge, round the face, the top's back edge
    1: [(0.13, 0.29), (0.03, 0.29), (0.0, 0.34), (0.0, 0.45), (0.03, 0.52), (0.12, 0.535)],
    -1: [(0.13, 0.29), (0.03, 0.29), (0.0, 0.35), (0.0, 0.52), (0.03, 0.595), (0.12, 0.605)],
}
BUMPER_RAILS, BUMPER_RIB_EVERY = (1, 4), 4   # (the rails along the profile's points 1 and 4; a rib every fourth column)
BUMPERS = {}


def bumper_grid(sgn, xc, a, b, nhalf):
    """the cover's points [column][profile point]: the plan line x = xc + a sqrt(cos f), z = b sqrt(sin f) (x mirrored
    for the rear), cut into equal lengths, nhalf a side"""
    fine = [(xc + a * math.sqrt(math.cos(0.5 * math.pi * k / 800)), b * math.sqrt(math.sin(0.5 * math.pi * k / 800))) for k in range(801)]
    fine[-1] = (xc, b)
    acc = [0.0]
    for p, q in zip(fine, fine[1:]):
        acc.append(acc[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))
    half = []
    for k in range(nhalf + 1):
        L = acc[-1] * k / nhalf
        i = max(1, next((i for i in range(1, len(acc)) if acc[i] >= L), len(acc) - 1))
        t = (L - acc[i - 1]) / max(1e-9, acc[i] - acc[i - 1])
        half.append((fine[i - 1][0] + (fine[i][0] - fine[i - 1][0]) * t, fine[i - 1][1] + (fine[i][1] - fine[i - 1][1]) * t))
    line = [(x, -z) for x, z in reversed(half[1:])] + half
    prof = BUMPER_PROFILE[sgn]
    P, C = [], []
    for i, (x, z) in enumerate(line):
        p0, p1 = line[max(0, i - 1)], line[min(len(line) - 1, i + 1)]
        tx, tz = p1[0] - p0[0], p1[1] - p0[1]
        l = math.hypot(tx, tz)
        nx, nz = tz / l, -tx / l
        if nx * (xc - a - x) + nz * (0 - z) < 0:
            nx, nz = -nx, -nz
        P.append([(sgn * (x + nx * d), y, z + nz * d) for d, y in prof])
        dm = sum(d for d, _ in prof) / len(prof) + 0.04
        C.append((sgn * (x + nx * dm), sum(y for _, y in prof) / len(prof), z + nz * dm))
    return P, C


for sgn, (xc, a, b) in ((1, (1.72, 0.34, 0.72)), (-1, (1.70, 0.32, 0.72))):
    name = "front bumper" if sgn > 0 else "rear bumper"
    part = "bumper %d" % sgn
    P, C = bumper_grid(sgn, xc, a, b, 9)
    pan, ids = grid_panel(name, "plastic", P, part)
    for i, r in enumerate(ids):
        for k in r:
            node_centre[k] = C[i]
    BUMPERS[sgn] = dict(grid=P, part=part, panel=pan)

# ---- the lights and the mirrors: little closed boxes (a rectangle for a face, a point behind it) on beams that tear
MIRRORS = []
for s in (1, -1):
    hx = 1.945
    lamp("headlight", [(hx, 0.57, s * 0.38), (hx, 0.57, s * 0.62), (hx, 0.70, s * 0.62), (hx, 0.70, s * 0.38)], (1.86, 0.635, s * 0.50),
         [(0, (XF, 0.52, s * 0.38)), (1, (XF, 0.52, s * 0.62)), (2, (XF, 0.74, s * 0.62)), (3, (XF, 0.74, s * 0.38))])
    t0, t1 = rpost(s, Y_TAIL), rpost(s, 0.93)
    tl = [lerp(t0, t1, 0.12), lerp(t0, t1, 0.88)]
    lamp("taillight", [(tl[0][0] - 0.03, tl[0][1], s * 0.56), (tl[0][0] - 0.03, tl[0][1], tl[0][2] - s * 0.01), (tl[1][0] - 0.03, tl[1][1], tl[1][2] - s * 0.01),
                       (tl[1][0] - 0.03, tl[1][1], s * 0.56)], (tl[0][0] + 0.06, 0.5 * (tl[0][1] + tl[1][1]), s * 0.64),
         [(0, (rpost(0, Y_TAIL)[0], Y_TAIL, s * 0.56)), (1, lerp(t0, t1, 0.0)), (2, lerp(t0, t1, 1.0)), (3, (-1.82, 0.93, s * 0.56))])
    # the mirror: its box on the tip of a plastic arm (a triangle of tube) on two stalks from the front door's corner
    box = lamp("plastic", [(0.74, 0.96, s * 0.88), (0.74, 0.96, s * 1.02), (0.74, 1.07, s * 1.02), (0.74, 1.07, s * 0.88)], (0.82, 1.015, s * 0.93), None, 0.12)
    d = DOORS[(s, True)]
    # (three feet off one line: on two its mounts' ball joints let the arm turn about their line)
    base = [d["grid"][0][DOOR_NV], lerp(d["grid"][0][DOOR_NV], d["top0"], 0.15), d["grid"][1][DOOR_NV]]
    split_at(base[1], d["part"])
    split_at(base[2], d["part"])
    MIRRORS.append(dict(s=s, box=box, part=d["part"], base=base, tip=[(0.79, 0.985, s * 0.89), (0.79, 1.045, s * 0.89), (0.77, 1.015, s * 0.97)]))
for lp in lamps:
    if lp["attach"]:
        for _, p in lp["attach"]:
            split_at(p)

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
    tube([(XA, YB, s * ZS), apron_f(s)], "pillar")                                           # apron (as stout as a pillar: the fender on it is a part now, bolted on, no web)
    tube([(XF, Y0, s * 0.66), apron_f(s)], "light")                                          # front post
    for ax, xs in ((AXLES[0], (1.45, 1.15)), (AXLES[1], (-1.15, -1.45))):
        for x in xs:
            tube([(x, Y0, s * ZR), (x, 0.66, s * ZR)], "rail")                               # tower posts
            split_at((x, 0.50, s * ZR))                                                      # (the upper wishbone's pivot)
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
tube(row(roof, ROOF_B), "pillar")                                                            # roof: the B bow (follows the roof's crown)
tube([rr_l, rr_r], "pillar")                                                                 # roof: rear header
tube([d_pt(-1, 1.0), d_pt(1, 1.0)], "light")                                                 # the tailgate's opening: its bottom
tube([rpost(-1, Y_TAIL), rpost(1, Y_TAIL)], "light")                                         # the tail's lower cross
tube([(XF, 0.52, -0.66), (XF, 0.52, 0.66)], "light")                                         # the front's middle cross
tube([apron_f(-1), apron_f(1)], "light")                                                     # the front's upper cross
RACK_X = AXLES[0] - 0.18
TIE_Z = 0.46                 # the tie rods' inner joints (on the rack's arms): on the wishbones' pivot line
tube([(RACK_X, Y0, -ZR), (RACK_X, Y0, 0.0), (RACK_X, Y0, ZR)], "rack")                       # the rack housing

# the hood's frame: round its skin, two ribs along under it and one across, all welded to it
HOOD_RIBS = (3, NV_TOP - 3)
for t in (ends(row(hood, 0)), ends(row(hood, HOOD_NU)), ends(col(hood, 0)), ends(col(hood, NV_TOP))):
    tube(t, "panel", "hood")
for j in HOOD_RIBS:
    tube(thin(col(hood, j), (HOOD_NU // 2,)), "panel", "hood")
tube(thin(row(hood, HOOD_NU // 2), HOOD_RIBS), "panel", "hood")
HOOD_HINGES, HOOD_LATCH = (1, NV_TOP - 1), NV_TOP // 2
for j in HOOD_HINGES:
    split_at((XA, YB, hood[0][j][2]))                     # (the hinges' nodes on the cowl, the hood's)
    split_at(hood[0][j], "hood")
split_at((XF, 0.74, hood[HOOD_NU][HOOD_LATCH][2]))        # (the latch's on the front's upper cross, the hood's)
split_at(hood[HOOD_NU][HOOD_LATCH], "hood")

# the tailgate's frame: round the glass and the skin, two ribs down the skin
for j in (0, NV_TOP):   # (its sides down the D pillars, the glass's and the skin's)
    tube([tg_glass[0][j], tg_skin[TG_SKIN_NU][j]], "panel", "tailgate")
for t in (ends(row(tg_glass, 0)), ends(row(tg_glass, TG_GLASS_NU)), ends(row(tg_skin, TG_SKIN_NU)), col(tg_skin, 3), col(tg_skin, NV_TOP - 3)):
    tube(t, "panel", "tailgate")
TG_HINGES, TG_LATCH = (2, NV_TOP - 2), NV_TOP // 2
for j in TG_HINGES:
    split_at(v_sub(tg_glass[0][j], tg_o))                  # (on the roof's rear header, and the tailgate's)
    split_at(tg_glass[0][j], "tailgate")
split_at(v_sub(tg_skin[TG_SKIN_NU][TG_LATCH], tg_o))       # (on the opening's bottom, and the tailgate's)
split_at(tg_skin[TG_SKIN_NU][TG_LATCH], "tailgate")
for j in (0, NV_TOP):
    split_at(tg_glass[TG_GLASS_NU // 2][j], "tailgate")    # (the stays')

# the fenders' frames: round the skin and the arch, bolted (members that tear) to the apron, the A pillar, the front post
FENDER_MOUNTS = [("top", 2), ("top", 6), ("rear", 1), ("front", 2)]   # (each bolt ties the fender's frame into the body's: fewer, a cheaper solve)
for s, fd in FENDERS.items():
    g, part = fd["grid"], fd["part"]
    tube(ends(row(g, 0)), "panel", part)
    tube(ends(row(g, FENDER_NU)), "panel", part)
    tube(ends(col(g, NV_SIDE)), "panel", part)
    for r in runs([g[i][0] for i in range(len(g))]):
        tube(ends(r), "panel", part)
    tube(fd["arc"], "panel", part)
    fd["mounts"] = []
    for edge, k in FENDER_MOUNTS:
        p = g[k][NV_SIDE] if edge == "top" else g[0][k] if edge == "rear" else g[FENDER_NU][k]
        fd["mounts"].append(p)
        split_at(v_sub(p, fd["o"]))
        split_at(p, part)

# the bumpers' frames: rails along the cover, ribs round its profile; brackets to the crash beam and the cross above it
for sgn, bd in BUMPERS.items():
    P, part = bd["grid"], bd["part"]
    mid = (len(P) - 1) // 2
    ribs = sorted({0, len(P) - 1} | {i for i in range(2, len(P) - 2) if (i - mid) % BUMPER_RIB_EVERY == 0})
    cols = [min(range(len(P)), key=lambda i: abs(P[i][1][2] - z)) for z in (-0.40, 0.40)]
    for j in BUMPER_RAILS:
        tube(thin([c[j] for c in P], set(ribs) | set(cols)), "bumper", part)
    for i in ribs:   # (round the profile: the face's middle points welded off the chord)
        tube([P[i][0], P[i][1], P[i][4], P[i][5]], "bumper", part)
    bd["mounts"] = []   # (the bumper's point, the body's or the fender's)
    for i in cols:   # (the lower rail on the crash beam at two points, the upper one on the cross above it in the middle)
        lo = P[i][BUMPER_RAILS[0]]
        bd["mounts"].append((lo, (XF, Y0, lo[2]) if sgn > 0 else (XR, Y0, lo[2]), "body"))
    hi = P[mid][BUMPER_RAILS[1]]
    bd["mounts"].append((hi, (XF, 0.52, hi[2]) if sgn > 0 else (rpost(0, Y_TAIL)[0], Y_TAIL, hi[2]), "body"))
    for p, q, qp in bd["mounts"]:
        split_at(q, qp)
    bd["wings"] = [P[0][BUMPER_RAILS[1]], P[-1][BUMPER_RAILS[1]]]

# the doors' frames: round their skins and windows (the tubes along their edges, bowed with the skin)
HINGE_ROWS, LATCH_ROW = (1, 4), 2
for (s, front), d in DOORS.items():
    g = d["grid"]
    nu = len(g) - 1
    part = d["part"]
    tube(ends(col(g, 0)), "door", part)                        # bottom
    tube(ends(col(g, DOOR_NV)), "door", part)                  # belt
    tube(row(g, 0), "door", part)                              # front edge
    tube(row(g, nu), "door", part)                             # rear edge
    tube([g[0][DOOR_NV], d["top0"], d["top1"], g[nu][DOOR_NV]], "door", part)   # the window's frame
    # the intrusion beam: across the door from the upper hinge's row down to the rear edge's lower row (the two
    # points bowed alike), the skin welded to it where it runs under
    d["beam"] = tube([g[0][HINGE_ROWS[1]], g[nu][HINGE_ROWS[0]]], "doorbeam", part)
    xp = XA if front else XB                             # the hinges' pillar and the latch's
    xl = XB if front else XC
    for j in HINGE_ROWS + (LATCH_ROW,):   # (the pillars' nodes for the hinges and the latch, the door's own)
        split_at((xp if j in HINGE_ROWS else xl, g[0][j][1], s * ZS))
        split_at(g[0][j] if j in HINGE_ROWS else g[nu][j], part)
    split_at((xl, Y0, 0.0))    # (the check strap's anchor on the floor's cross member)
for p in ((XB, Y0, 0.0), (XC, Y0, 0.0), (XB, Y0, ZS)):   # (the cameras' nodes)
    split_at(p)

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
        split_at((ax, 0.66, s * ZR))
        split_at((arm_x, Y0, s * ZR))


# every tube divided at its part's other tubes' ends and extra points, and at its part's panels' nodes on it no closer
# than SEAM apart (fem_car.divide_tubes); the late members
divide_tubes()

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
        # the tie rod's inner joint on an arm of the rack out to the wishbones' pivot line (the rails): a tie rod from
        # the rack's end itself was twice as long as the arms, and a wheel in full bump (a tight turn) was pulled in
        # at the kingpin, not at the steering arm - it steered itself 9 degrees further and the car tripped over it
        ti = new_node((RACK_X, Y0, s * TIE_Z), "plain", 3.0)
        member(re, ti, "rack")
        member(ti, sa, "tierod")
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
# (negative: the hydro shortens for a right turn - the rack goes right and the arms behind the axle turn the wheels'
# fronts right; with a positive factor the wheels turned against the input)
hydros.append((rack_ends[1], fem_get((RACK_X, Y0, -ZR), 0.002), -RACK_TRAVEL / (ZR + 0.30)))

# ---- the parts on the body: hinges and bolts (mounts: the part's node held at a distance, letting go past its break
# force), latches that break, straps that end a door's or a lid's travel
for (s, front), d in DOORS.items():
    g = d["grid"]
    nu = len(g) - 1
    xp, xl = (XA, XB) if front else (XB, XC)
    for j in HINGE_ROWS:
        mounts.append((fem_get((xp, g[0][j][1], s * ZS)), fem_get(g[0][j], 0.002, d["part"]), "door hinge"))
    dl = fem_get(g[nu][LATCH_ROW], 0.002, d["part"])
    latches.append((dl, fem_get((xl, g[nu][LATCH_ROW][1], s * ZS)), DOOR_LATCH))
    # the check strap: the latch's node to the floor's middle; a door does not swing in, and opens to ~75 deg
    straps.append((dl, fem_get((xl, Y0, 0.0), 0.002), 0.01, 1.20))
# the hood: hinged at the cowl, latched to the front's upper cross, stays to the towers' tops (opens 55 degrees)
for j in HOOD_HINGES:
    mounts.append((fem_get((XA, YB, hood[0][j][2]), 0.002), fem_get(hood[0][j], 0.002, "hood"), "lid hinge"))
latches.append((fem_get(hood[HOOD_NU][HOOD_LATCH], 0.002, "hood"), fem_get((XF, 0.74, hood[HOOD_NU][HOOD_LATCH][2]), 0.002), LID_LATCH))
for j, s in ((0, -1), (NV_TOP, 1)):
    stay(hood[HOOD_NU // 2][j], "hood", (1.45, 0.66, s * ZR), hood_r(0), (0.0, 0.0, 1.0), 55.0)
# the tailgate: hinged at the roof's rear header, latched at the bottom, stays to the D pillars' feet (opens 75 degrees)
for j in TG_HINGES:
    mounts.append((fem_get(v_sub(tg_glass[0][j], tg_o), 0.002), fem_get(tg_glass[0][j], 0.002, "tailgate"), "lid hinge"))
latches.append((fem_get(tg_skin[TG_SKIN_NU][TG_LATCH], 0.002, "tailgate"), fem_get(v_sub(tg_skin[TG_SKIN_NU][TG_LATCH], tg_o), 0.002), LID_LATCH))
for j, s in ((0, -1), (NV_TOP, 1)):
    stay(tg_glass[TG_GLASS_NU // 2][j], "tailgate", (-1.82, 0.93, s * 0.74), v_add(rr_l, tg_o), (0.0, 0.0, 1.0), -75.0)
# the fenders' bolts, the bumpers' brackets (the wings' to the body's nearest node: a fender's light arc node carried little)
for s, fd in FENDERS.items():
    for p in fd["mounts"]:
        mounts.append((fem_get(v_sub(p, fd["o"]), 0.002), fem_get(p, 0.002, fd["part"]), "fender bolt"))
for sgn, bd in BUMPERS.items():
    for p, q, qp in bd["mounts"]:
        mounts.append((fem_get(q, 0.002, qp), fem_get(p, 0.002, bd["part"]), "bumper bracket"))
    for p in bd["wings"]:
        mounts.append((nearest_of("body", p, 0.35), fem_get(p, 0.002, bd["part"]), "bumper bracket"))
# the mirrors' arms: stalks from three feet on the door (mounted there: they bend, and tear off it), a triangle at the
# tip in the box
for mr in MIRRORS:
    mp = "mirror %d" % mr["s"]
    t = [fem_node(p, mp) for p in mr["tip"]]
    a, b, c = (fem_node(p, mp) for p in mr["base"])
    for p, f in zip(mr["base"], (a, b, c)):
        mounts.append((fem_get(p, 0.002, mr["part"]), f, "mirror"))
    member(a, t[0], "mirror")
    member(b, t[1], "mirror")
    member(a, t[1], "mirror")
    member(c, t[0], "mirror")
    member(t[0], t[1], "mirror")
    member(t[1], t[2], "mirror")
    member(t[2], t[0], "mirror")
    mr["box"]["mounts"] = t

# ---- the welds: every panel node on a frame node of its part (the same point) or on a tube of it between its nodes
weld_panels()
# the bumpers' covers where they run off their frames' chords (the rails' every other column, the face between the
# ribs' corners): held at the nearest point of a member within a few centimetres
for sgn, bd in BUMPERS.items():
    weld_near(bd["panel"], 0.045)
# the doors' skins on their beams: the nodes over it (within half a cell of its line, seen square to the door) held at
# the beam's point under them, the skin's bow and its gap off it
for (s, front), d in DOORS.items():
    weld_to_chain(d["skin"], tube_chain[d["beam"]], 0.45 * GRID, lambda p, q: math.hypot(p[0] - q[0], p[1] - q[1]))

# ---- the panels off their tubes: each panel node moved GAP out along its panel's normal there (the glass stays in its
# frame's plane); the lamps out with the panel they sit on, the mirrors on their arms
offset_panels()

# ---- the lamps: their boxes' beams, and every node's beams to four body nodes round the box; the mirrors' to their
# arm's tip
mount_lamps()

# ---- the frame's collision hull: a coarse closed shell of big triangles on the frame's own nodes (the tubes' ends and
# corners, the doors' frames), a few centimetres inside the panels, colliding and not drawn. The panels are thin and
# tear, the tubes are points to another body's triangles: without it two cars' frames went through each other.
Fl = lambda s: (XF, Y0, s * 0.66)                  # the front crash beam's ends
Al, Bl, Cl = ((x, Y0, 0) for x in (XA, XB, XC))
Rl = lambda s: (XR, Y0, s * 0.70)                  # the rear crash beam's ends
side = lambda p, s: (p[0], p[1], s * ZS)
Ab = lambda s: (XA, YB, s * ZS)                    # the cowl's ends
Cb = lambda s: (XC, YB, s * ZS)
Db = lambda s: (-1.82, 0.93, s * 0.74)             # the D pillars' feet
Wr = lambda s: (X_WS, YR_F, s * ZT)                # the roof's corners and the B bow's ends
Br = lambda s: roof[ROOF_B][0 if s < 0 else NV_TOP]
Rr = lambda s: (X_RR, YR_R, s * ZT)
hull_quad(Fl(-1), Fl(1), side(Al, 1), side(Al, -1))           # floor
hull_quad(side(Al, -1), side(Al, 1), side(Bl, 1), side(Bl, -1))
hull_quad(side(Bl, -1), side(Bl, 1), side(Cl, 1), side(Cl, -1))
hull_quad(side(Cl, -1), side(Cl, 1), Rl(1), Rl(-1))
hull_quad(Fl(-1), Fl(1), apron_f(1), apron_f(-1))             # the front
hull_quad(apron_f(-1), apron_f(1), Ab(1), Ab(-1))             # under the hood
hull_quad(Ab(-1), Ab(1), Wr(1), Wr(-1))                       # the windscreen
hull_quad(Wr(-1), Wr(1), Br(1), Br(-1))                       # the roof
hull_quad(Br(-1), Br(1), Rr(1), Rr(-1))
hull_quad(Rr(-1), Rr(1), Db(1), Db(-1))                       # the tailgate's opening
hull_quad(Db(-1), Db(1), Rl(1), Rl(-1))                       # the tail
for s in (1, -1):
    hull_quad(Fl(s), side(Al, s), Ab(s), apron_f(s))          # under the fender
    hull_quad(side(Cl, s), Rl(s), Db(s), Cb(s))               # the quarter
    hull_tri(Cb(s), Db(s), Rr(s))                             # the quarter window
    hull_tri(Ab(s), Wr(s), DOORS[(s, True)]["top0"], DOORS[(s, True)]["part"])   # the A pillar over the front door's window
    for front in (True, False):                               # the doors, on their own frames (they open with them)
        d = DOORS[(s, front)]
        g = d["grid"]
        nu = len(g) - 1
        hull_quad(g[0][0], g[nu][0], g[nu][DOOR_NV], g[0][DOOR_NV], d["part"])
        hull_quad(g[0][DOOR_NV], g[nu][DOOR_NV], d["top1"], d["top0"], d["part"])
orient_hull()

# ---------------------------------------------------------------------------------------------------------- write
centre, back, left = fem_node((XB, Y0, 0.0)), fem_node((XC, Y0, 0.0)), fem_node((XB, Y0, ZS))
# the engine's and the tank's nodes: the floor rails' in the engine bay, the rear floor's (the load weight of each)
bay = [k for k in body_ids if abs(nodes[k][1] - Y0) < 0.01 and abs(abs(nodes[k][2]) - ZR) < 0.01 and 0.9 < nodes[k][0] < 1.7]
tank = [k for k in body_ids if abs(nodes[k][1] - Y0) < 0.01 and abs(nodes[k][2]) < ZR + 0.01 and -1.25 < nodes[k][0] < -0.85]
assert len(bay) >= 4 and len(tank) >= 4, (len(bay), len(tank))
heavy.update({k: ENGINE_KG / len(bay) for k in bay})
heavy.update({k: TANK_KG / len(tank) for k in tank})
shell_ids = sorted({k for (pts, sec, part), chain in zip(tubes, tube_chain) if part == "body" for k in chain if nodes[k][1] < Y0 + 0.05})
for k in shell_ids:
    heavy[k] = heavy.get(k, 0.0) + BODY_KG / len(shell_ids)
for a, b, kind in mounts:   # (the parts' hardware at their mounts)
    heavy[b] = heavy.get(b, 0.0) + MOUNTS[kind][3]
head = (0.05, 1.02, 0.36)
near = sorted(body_ids, key=lambda k: v_len(v_sub(nodes[k], head)))[:8]
shocks_text = "shocks\n;n1, n2, spring, damp, short bound, long bound, precompression, options\n"
shocks_text += "set_beam_defaults 9000000, 12000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n"
for a, b, pre in shocks:
    shocks_text += "%d, %d, 60000, 4000, 0.25, 0.25, %.3f, n\n" % (a, b, pre)
# travel stops beside each coil-over (a bump stop at 20% compression, a droop strap at 4% extension)
shocks_text += "set_beam_defaults 20000000, 200000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n"
for a, b, _ in shocks:
    shocks_text += "%d, %d, 0, 0, 0.20, 0.04, 1.0, i\n" % (a, b)
# steering stops (free through the lock and 4 degrees past it) and the rear toe stops (6 degrees)
shocks_text += "set_beam_defaults 3000000, 20000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n"
for a, b, lo, hi in stops:
    shocks_text += "%d, %d, 0, 0, %.3f, %.3f, 1.0, i\n" % (a, b, lo, hi)
# the doors' check straps (the latch's node to the floor's middle: a door does not swing in, and opens to ~75 deg), the
# hood's and the tailgate's stays
shocks_text += "set_beam_defaults 500000, 3000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n"
for a, b, lo, hi in straps:
    shocks_text += "%d, %d, 0, 0, %.3f, %.3f, 1.0, i\n" % (a, b, lo, hi)
wheels_text = "wheels\n;radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, face, band\n"
wheels_text += "set_beam_defaults 3000000, 400, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n"
for n1, n2, arm, front in wheels:
    wheels_text += "%.2f, 0.20, 12, %d, %d, 9999, 1, 1, %d, 45.0, 120000.0, 900.0, tracks/wheelface tracks/wheelband\n" % (WHEEL_R, n1, n2, arm)
engine_text = ("engine\n;min rpm, max rpm, torque, differential, reverse, neutral, gears...\n1000.0, 6800.0, 420.0, 4.1, 3.2, 1.0, 3.3, 2.1, 1.5, 1.15, 0.9, -1.0\n"
               "engoption\n0.08, c, 1000.0, 0.3, 0.4, 0.3\nbrakes\n4000\n")
housing = [fem_node((RACK_X, Y0, -ZR)), fem_node((RACK_X, Y0, 0.0)), fem_node((RACK_X, Y0, ZR))]
all_tris = write(OUT, dict(
    title="Frame Car",
    head=["generated by tools/make_frame_car.py: a five-door hatchback on a welded chromoly space frame (FEM frame elements),",
          "its panels sheet metal, glass and plastic (triangle elements) welded on; doors, hood, tailgate, fenders, bumpers and",
          "mirrors frames of their own on hinges, bolts and latches that break"],
    globals=(DRY_KG, "sheet/Steel/7.9/0.004/%d" % REFINE), sections=SECTIONS, mat=MAT, mount_kinds=MOUNTS,
    latch_note=";latches: the doors' hold till %.0f kN, the hood's and the tailgate's till %.0f kN, then break (it swings open)" % (DOOR_LATCH / 1000, LID_LATCH / 1000),
    shocks_text=shocks_text, hydros=hydros, slides=[(n, housing) for n in slides], wheels_text=wheels_text, engine_text=engine_text,
    cameras=(centre, back, left), cinecam=(head, near), uv_span=(2.1, 4.2, 1.0, 3.0, Y0)))
summary(OUT, all_tris, len(DOORS))
print("%d mounts: %s" % (len(mounts), ", ".join("%s %d" % (k, sum(1 for m in mounts if m[2] == k)) for k in MOUNTS)))
