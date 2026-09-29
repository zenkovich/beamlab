#!/usr/bin/env python3
"""Writes assets/vehicles/buggy/buggy.truck: an unlimited-class desert racer (a Class 1 buggy, the trophy cars of the
Baja 1000) on the Frame Car's scheme (tools/fem_car.py).

The chassis is a chromoly tube cage of FEM frame elements, welded at its joints: floor rails and sills, the cockpit's
cage (firewall posts and A pillars, the main hoop with its diagonal and harness bar, roof rails and bows, door bars),
the front clip (lower and upper rails, the shock towers, the upper arms' pivot rails, the nose and its bumper tubes) and
the rear clip (the engine cradle, the trailing arms' pivot beam, the rear shock hoop braced to the main hoop, the rear
bumper). The body is aluminium sheet (triangle elements) welded on: the roof and the floor pan on the cage; the nose, the
side panels and the engine cover are frames of their own, their skins welded to them, bolted on at a distance (mounts)
and letting go past their bolts' strength - each a component of the frame solved on its own. A light bar on the roof,
headlights on the nose, tail lights on the rear bumper.

Long-travel suspension: at the front double wishbones on ball joints, 0.6 m of travel, a coil-over from the lower ball
joint to the shock tower, a steering rack behind the axle (the frame car's: a bar sliding along its housing, a hydro,
tie rods on the arms' pivot line); at the rear trailing arms (a truss on two ball joints at the pivot beam: the wheel
swings in an arc), 0.75 m of travel, the coil-over to the shock hoop. Travel stops beside each. 39 inch tyres, a mid
engine (850 N m) driving the rear wheels through a four-speed box. About 1500 kg.

Coordinates follow Rigs of Rods: -x forward, y up, +z left. The design is written with the front at +x and mirrored
when the nodes are written.

    python3 tools/make_buggy.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fem_car import *   # noqa: E402,F401,F403

OUT = os.environ.get("BG_OUT") or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "vehicles", "buggy", "buggy.truck")

# ---------------------------------------------------------------------------------------------------- the chassis' lines
Y0 = 0.56                    # the floor rails (a skid plate's clearance under them)
WHEEL_R, WHEEL_W = 0.49, 0.34    # 39 x 13.5 tyres
AX_F, AX_R = 1.50, -1.60     # the axles: a 3.1 m wheelbase
ZW = 0.88                    # the wheels' inner faces (a 2.1 m track)
ZC = 0.72                    # the cockpit's sides (sills, firewall posts, main hoop)
ZF = 0.42                    # the front and rear clips' rails
ZT = 0.62                    # the roof rails
X_FW, X_MH = 0.75, -0.55     # the firewall, the main hoop
X_RF = 0.15                  # the roof's front
X_NOSE, X_TAIL = 2.15, -2.20
Y_DASH, Y_ROOF = 1.05, 1.80
Y_DOOR = 0.85                # the door bars
Y_TF, Y_TR = 1.20, 1.55      # the front shock towers' tops, the rear shock hoop's
X_TR = -0.95                 # the rear shock hoop (on the floor's rear cross: the shocks lean forward, a rising rate)
SM = (-1.45, 0.60)           # the rear shocks' lower mounts on the trailing arms (x, y; z at the arm's outer side)
X_PIV = X_MH                 # the trailing arms' pivots: at the main hoop's foot and on its floor cross
RACK_X = 1.20                # the steering rack, behind the front axle
Y_UP = 0.96                  # the upper arms' pivots
BL, BU = (0.42, 0.88), (0.82, 0.84)   # the lower and upper ball joints (y, z): the arms 16 and 22 degrees down at ride height
# (the rack on the line between the arms' inner pivots at the steering arm's height along the upright: no bump steer)
Y_RACK = Y0 + (Y_UP - Y0) * (WHEEL_R - BL[0]) / (BU[0] - BL[0])
GRID = 0.14
# the masses: the engine and transaxle on the rear rails in the cradle, the fuel cell (60 gal) behind the seats, the
# crew, seats and gear on the cockpit's floor, two spare tyres on the rear upper rails; the dry mass spread over the
# members by their length. About 1450 kg ready to race, 40 / 60 front to rear
ENGINE_KG, FUEL_KG, CREW_KG, SPARE_KG, DRY_KG = 360.0, 160.0, 220.0, 60.0, 80.0
# the suspension: wheel travel (m) up from the design height (where it stands) and down, the ride frequency (Hz), the
# damping ratio; the shocks' preload carries the static load at their design length (STATIC: N per shock, measured
# with BL_SHOCKDBG at rest - the spring's k (L - len) - and the rates it was measured at, iterated twice)
TRAVEL = {True: (0.46, 0.21), False: (0.52, 0.26)}        # front 26", rear 30" (Class 1)
BUMP_STOP, BUMP_STOP_G = 0.10, 8.0   # the last 10 cm up in the bump stops (their spring and damping growing), 8 x the static load at the end
RIDE_HZ, ZETA = {True: 1.10, False: 0.95}, 0.45   # (soft: the springs still push at full droop, no tender spring here)
STATIC = {True: 2700.0, False: 5050.0}

SECTIONS = {  # name: (material, shape, outer, wall, joints[, break force N[, the released joint's damping N m s/rad]])
    "main": ("Chromoly", "tube", 0.045, 0.0030, "rigid"),     # floor rails and sills, the main hoop, A pillars, roof rails
    "cage": ("Chromoly", "tube", 0.038, 0.0025, "rigid"),     # the clips' rails, door bars, towers, bows, bumpers
    "light": ("Chromoly", "tube", 0.032, 0.0020, "rigid"),    # braces, the pivot rails' posts
    "panel": ("Aluminium", "tube", 0.022, 0.0020, "rigid"),   # the nose's, the side panels' and the engine cover's frames
    "arm": ("Chromoly", "tube", 0.045, 0.0035, "rigid", 0.0, 3.0),   # the front wishbones
    "tarm": ("Chromoly", "tube", 0.055, 0.0040, "rigid", 0.0, 3.0),  # the trailing arms
    "brace": ("Chromoly", "tube", 0.038, 0.0030, "rigid"),     # the arms' and the hub carriers' triangulating braces (in tension
                                                               # and compression: a light tube)
    "pivot": ("Chromoly", "tube", 0.045, 0.0035, "rigid"),     # the front arms' pivot rails and posts, the anti-roll bars' bearings
    "pickup": ("Chromoly", "tube", 0.050, 0.0040, "rigid"),    # the front lower rail between the lower wishbone's pivots
    "hub": ("Steel", "tube", 0.060, 0.0080, "rigid", 0.0, 3.0),      # uprights and hubs
    "tierod": ("Chromoly", "tube", 0.032, 0.0050, "ball"),
    "rack": ("Steel", "tube", 0.035, 0.0050, "rigid"),
    "arbf": ("SpringSteel", "tube", 0.030, 0.0040, "rigid"),  # the anti-roll bars' torsion tubes, front and rear (spring steel:
    "arbr": ("SpringSteel", "tube", 0.030, 0.0040, "rigid"),  # chromoly ones took a set in the ruts; 26 mm at the rear: with the hubs triangulated it rolled on the circle instead of spinning)
}
# the panels on the cage at a distance (`mounts`): (break force N, stiffness N/m (0: the step's), turning damping, the
# part node's hardware kg)
MOUNTS = {   # (on rubber-isolated tabs: the cage flexing a few mm between a panel's bolts at a 5 m drop tore stiffer ones)
    "nose bolt": (5000.0, 4.0e5, 0.0, 0.3),
    "side bolt": (4000.0, 2.5e5, 0.0, 0.2),
    "cover bolt": (4000.0, 3.0e5, 0.0, 0.2),
}
MAT = {  # shell materials: name -> (material, kg/m2, drawn thickness, colour or None: the body's)
    "body": None,
    "floor": ("Aluminium", 5.4, 0.004, (0.25, 0.25, 0.26)),   # (2 mm)
    "headlight": ("Glass", 6.0, 0.006, (0.95, 0.93, 0.80)),
    "taillight": ("Glass", 6.0, 0.006, (0.75, 0.05, 0.04)),
}
PART_OFF = 0.03             # the parts' frames off the cage's tubes (m)
REFINE = int(os.environ.get("BG_REFINE", "1"))
configure(centre_y=0.80, grid=GRID, seam=0.22, arch_r=0.62, arch_y=WHEEL_R, gap=0.03, weld_r=0.17, weld_brk=2500.0, lamp_brk=2500.0, lamp_reach=1.0, refine=REFINE)

def ax_up(p, d=PART_OFF): return v_add(p, (0.0, d, 0.0))


# ---------------------------------------------------------------------------------------------------- the key points
D = lambda s: (X_FW, Y_DASH, s * 0.70)            # the dash's corners (the A pillars' feet)
T = lambda s: (AX_F, Y_TF, s * ZF)                 # the front shock towers' tops
N = lambda s: (X_NOSE, 0.95, s * 0.40)             # the nose's top corners
NB = lambda s: (X_NOSE - 0.07, 0.66, s * 0.40)     # its bottom corners
FB = lambda s: (X_NOSE + 0.20, 0.72, s * 0.34)     # the front bumper's corners
RF = lambda s: (X_RF, Y_ROOF, s * ZT)              # the roof's corners
RR = lambda s: (X_MH, Y_ROOF, s * ZT)
MH = lambda s, y: (X_MH, y, s * (ZC + (ZT - ZC) * (y - Y0) / (Y_ROOF - Y0)))   # on the main hoop
FW = lambda s, y: (X_FW, y, s * (ZC + (0.70 - ZC) * (y - Y0) / (Y_DASH - Y0)))  # on the firewall post
Y_TL, Y_TT = 0.80, 1.05      # the tail's bottom and top: kicked up behind the transaxle (the rear bottomed out on a
                              # kicker's face scraped a tail at 0.64 m)
TL = lambda s: (X_TAIL, Y_TL, s * ZF)
TT = lambda s: (X_TAIL, Y_TT, s * 0.45)
RU = lambda s, t: lerp(MH(s, 1.30), TT(s), t)                                       # the rear upper rails
TRT = lambda s: (X_TR, Y_TR, s * 0.50)             # the rear shock hoop's tops
TRB = lambda s: (X_TR, Y0, s * ZF)

# ------------------------------------------------------------------------------------------------------- the panels
_, roof = quad_panel("roof", "body", RF(-1), RR(-1), RF(1), RR(1), 5, 9, dome=0.03)
quad_panel("floor pan", "floor", (X_FW, Y0, -ZC), (X_MH, Y0, -ZC), (X_FW, Y0, ZC), (X_MH, Y0, ZC), n_cells((X_FW, 0, 0), (X_MH, 0, 0)), 10)
# the nose: its own frame over the front clip's upper rails, peaked at the shock towers, and down the front
NOSE_NV = 6   # (across: cells of about the grid's size at the towers and the front)
_, nose_a = quad_panel("nose", "body", ax_up(D(-1)), ax_up(T(-1)), ax_up(D(1)), ax_up(T(1)), 5, NOSE_NV, dome=0.04, part="nose")
_, nose_b = quad_panel("nose", "body", ax_up(T(-1)), ax_up(N(-1)), ax_up(T(1)), ax_up(N(1)), 5, NOSE_NV, dome=0.03, part="nose")
fwd = lambda p: v_add(p, (PART_OFF, 0.0, 0.0))
_, nose_c = quad_panel("nose front", "body", ax_up(N(-1)), fwd(NB(-1)), ax_up(N(1)), fwd(NB(1)), 2, NOSE_NV, dome=0.0, part="nose")
# the side panels: the cockpit's sides under the door bars, out of the sill and the posts
SIDES, SIDE_NV = {}, 2
for s in (1, -1):
    o = (0.0, 0.0, s * PART_OFF)
    nu = n_cells((X_FW, 0, 0), (X_MH, 0, 0))
    _, g = quad_panel("side", "body", v_add(FW(s, Y0 + 0.02), o), v_add(MH(s, Y0 + 0.02), o), v_add(FW(s, Y_DOOR - 0.02), o), v_add(MH(s, Y_DOOR - 0.02), o), nu, SIDE_NV,
                      dome=0.02, part="side %d" % s)
    SIDES[s] = dict(grid=g, o=o, part="side %d" % s)
# the engine cover: over the rear upper rails from the main hoop to the tail (the shock hoop's tops stand through it)
COVER_NU, COVER_NV = 10, 8
_, cover = quad_panel("engine cover", "body", ax_up(RU(-1, 0.0)), ax_up(RU(-1, 1.0)), ax_up(RU(1, 0.0)), ax_up(RU(1, 1.0)), COVER_NU, COVER_NV, dome=0.07,
                      part="cover")

# ---- the lights: a bar on the roof's front, two on the nose, two on the rear bumper
lamp("headlight", [(X_RF + 0.02, Y_ROOF + 0.05, -0.50), (X_RF + 0.02, Y_ROOF + 0.05, 0.50), (X_RF + 0.02, Y_ROOF + 0.14, 0.50), (X_RF + 0.02, Y_ROOF + 0.14, -0.50)],
     (X_RF - 0.08, Y_ROOF + 0.09, 0.0), [(0, RF(-1)), (1, RF(1))])
for s in (1, -1):
    lamp("headlight", [(X_NOSE + 0.09, 0.72, s * 0.10), (X_NOSE + 0.09, 0.72, s * 0.32), (X_NOSE + 0.07, 0.88, s * 0.32), (X_NOSE + 0.07, 0.88, s * 0.10)],
         (X_NOSE - 0.02, 0.80, s * 0.21), [(0, N(s)), (1, NB(s))])
    lamp("taillight", [(X_TAIL - 0.04, 0.87, s * 0.20), (X_TAIL - 0.04, 0.87, s * 0.36), (X_TAIL - 0.04, 0.97, s * 0.36), (X_TAIL - 0.04, 0.97, s * 0.20)],
         (X_TAIL + 0.04, 0.92, s * 0.28), None)

# ------------------------------------------------------------------------------------------------ the cage's tubes
for s in (1, -1):
    tube([(X_FW, Y0, s * ZC), (1.20, Y0, s * ZF)], "main")                                    # front lower rail
    tube([(1.20, Y0, s * ZF), (1.80, Y0, s * ZF)], "pickup")                                 # (the lower wishbone's pivots on it)
    tube([(1.80, Y0, s * ZF), NB(s)], "main")
    tube([(1.36, Y0, s * ZF), (1.20, Y_UP, s * ZF)], "pivot")                                # the pivot box braced across: the
    tube([(1.64, Y0, s * ZF), (1.80, Y_UP, s * ZF)], "pivot")                                # rail bent between the pivots
    tube([(X_FW, Y0, s * ZC), (X_MH, Y0, s * ZC)], "main")                                    # sill
    tube([(X_MH, Y0, s * ZC), TRB(s), TL(s)], "main")                                                           # rear lower rail
    tube([(X_FW, Y0, s * ZC), D(s), RF(s)], "main")                                           # firewall post, A pillar
    tube([MH(s, Y0), RR(s)], "main")                                                          # main hoop
    tube([RF(s), RR(s)], "main")                                                              # roof rail
    tube([D(s), T(s), N(s)], "main")                                                          # front upper rail
    tube([(AX_F, Y0, s * ZF), T(s)], "pickup")                                                # front shock tower (the shock's top)
    tube([N(s), NB(s)], "cage")                                                               # the nose's post
    # the upper arms' pivot rail and its posts (the rear one carries the rack): the arms' loads in bending between the
    # pivots (a 38 x 2.5 rail took a set on a rough field)
    tube([(1.20, Y_UP, s * ZF), (1.80, Y_UP, s * ZF)], "pivot")
    tube([(1.20, Y0, s * ZF), (1.20, Y_RACK, s * ZF), (1.20, Y_UP, s * ZF)], "pivot")
    tube([(1.80, Y0, s * ZF), (1.80, Y_UP, s * ZF)], "pivot")
    tube([(1.80, Y_UP, s * ZF), N(s)], "light")                                               # (the nose braced to it)
    # the front clip triangulated (a box of posts and rails folded up as a mechanism once its joints yielded: two cars
    # head-on went into each other): its sides, and the bumper ahead of the nose
    tube([(1.20, Y0, s * ZF), D(s)], "cage")
    tube([(1.80, Y0, s * ZF), T(s)], "cage")
    tube([NB(s), FB(s), N(s)], "cage")                                                        # the front bumper's ends
    tube([FW(s, Y_DOOR), MH(s, Y_DOOR)], "cage")                                              # door bar
    tube([FW(s, Y_DOOR), MH(s, Y0)], "cage")                                                  # and its diagonal
    tube([MH(s, 1.30), TT(s)], "cage")                                                        # rear upper rail
    tube([TRB(s), TRT(s)], "main")                                                            # the rear shock hoop's post
    tube([RR(s), TRT(s)], "main")                                                             # braced to the main hoop's top
    # the trailing arms' inner pivot: a post behind the seat up to the harness bar, a seat rail forward along the floor
    tube([(X_MH, Y0, s * 0.36), (X_MH, 1.30, s * 0.36)], "cage")
    tube([(X_MH, Y0, s * 0.36), (0.10, Y0, s * 0.36)], "cage")
    tube([TRT(s), TT(s)], "light")                                                            # and to the tail
    tube([TL(s), TT(s)], "cage")                                                              # rear bumper post
    split_at((AX_F, Y_UP, s * ZF))           # (the tower crosses the pivot rail)
    split_at((AX_F, Y_RACK, s * ZF))
    for x in (1.36, 1.64):                   # (the wishbones' pivots)
        split_at((x, Y0, s * ZF))
        split_at((x, Y_UP, s * ZF))
    split_at((X_PIV, Y0, s * 0.36))          # (the trailing arms' inner pivots)
    split_at((X_MH, 1.30, s * 0.36))         # (the seat posts' tops on the harness bar, the seat rails' fronts)
    split_at((0.10, Y0, s * 0.36))
    split_at(MH(s, 1.30))
    for t in (0.36, 0.72):                   # (the engine's and the transaxle's mounts on the rear lower rail)
        split_at(lerp(TRB(s), TL(s), t))
    for t in (0.35, 0.7):                    # (the engine cover's bolts)
        split_at(RU(s, t))
for x in (X_FW, 0.10, X_MH, -0.95):
    zz = ZC if x >= X_MH else ZF
    tube([(x, Y0, -zz), (x, Y0, zz)], "main")                                                 # floor cross members
tube([TL(-1), TL(1)], "cage")                                                                  # rear bumper
tube([TT(-1), TT(1)], "cage")
tube([TRT(-1), TRT(1)], "main")                                                                # the rear shock hoop's top
tube([N(-1), N(1)], "cage")                                                                    # the nose's bumper tubes
tube([NB(-1), NB(1)], "cage")
tube([T(-1), T(1)], "cage")                                                                    # the front towers' cross
tube([T(1), N(-1)], "light")                                                                   # the clip's top and bottom diagonals
tube([(1.20, Y0, ZF), (1.80, Y0, -ZF)], "light")
tube([FB(-1), FB(1)], "cage")                                                                  # the front bumper
tube([(1.80, Y0, -ZF), (1.80, Y0, ZF)], "main")
tube([(1.20, Y0, -ZF), (1.20, Y0, ZF)], "main")
tube([D(-1), D(1)], "cage")                                                                    # dash bar
tube([RF(-1), RF(1)], "main")                                                                  # the roof's front and rear
tube([RR(-1), RR(1)], "main")
tube(row(roof, 2), "light")                                                                    # a roof bow (follows the crown)
tube([MH(-1, 1.30), MH(1, 1.30)], "cage")                                                      # harness bar
tube([MH(-1, Y0), MH(1, 1.30)], "cage")                                                        # the main hoop's diagonal
tube([(RACK_X, Y_RACK, -ZF), (RACK_X, Y_RACK, 0.0), (RACK_X, Y_RACK, ZF)], "rack")           # the rack's housing
for p in ((0.10, Y0, 0.0), (X_MH, Y0, 0.0), (0.10, Y0, ZC)):   # (the cameras' nodes)
    split_at(p)

# ---- the parts' frames: round the nose (and its middle along), the side panels, the engine cover (and its middle)
for t in (ends(row(nose_a, 0)), ends(col(nose_a, 0)), ends(col(nose_a, NOSE_NV)), ends(row(nose_b, 0)), ends(col(nose_b, 0)), ends(col(nose_b, NOSE_NV)),
          ends(row(nose_b, 5)), ends(row(nose_c, 2)), ends(col(nose_c, 0)), ends(col(nose_c, NOSE_NV)), thin(col(nose_a, NOSE_NV // 2)), thin(col(nose_b, NOSE_NV // 2))):
    tube(t, "panel", "nose")
for s, sd in SIDES.items():
    g = sd["grid"]
    for t in (ends(row(g, 0)), ends(row(g, len(g) - 1)), ends(col(g, 0)), ends(col(g, SIDE_NV))):
        tube(t, "panel", sd["part"])
for t in (ends(row(cover, 0)), ends(row(cover, COVER_NU)), ends(col(cover, 0)), ends(col(cover, COVER_NV)), thin(col(cover, COVER_NV // 2))):
    tube(t, "panel", "cover")
NOSE_BOLTS = [(ax_up(D(s)), D(s)) for s in (-1, 1)] + [(ax_up(T(s)), T(s)) for s in (-1, 1)] + [(fwd(NB(s)), NB(s)) for s in (-1, 1)]
COVER_BOLTS = [ax_up(RU(s, t)) for s in (-1, 1) for t in (0.0, 0.35, 0.7, 1.0)]
for p in COVER_BOLTS:
    split_at(p, "cover")

# ---- the anti-roll bars (Class 1 cars run them: soft springs alone rolled the car over at 0.8 g on asphalt): a
# torsion tube across the car turning in two bearings (a member from the frame along its axis, a swivel at the bar's
# end), a lever back from each end and a drop link down to the arm (on the front lower wishbone's front leg, on the
# trailing arm's outer member). front: 30 x 4 mm, rear: 26 x 4 mm; the roll stiffness they add ~1.3 x the springs'
ARB = {}
for front in (True, False):
    if front:
        P = lambda s: lerp((1.64, Y0, s * ZF), (AX_F, BL[0], s * BL[1]), 0.4)        # (on the lower arm's front leg)
        xb, yb, frame = 1.80, 0.66, lambda s: (1.80, 0.66, s * ZF)                  # (on the pivot rail's front post)
    else:
        P = lambda s: lerp((X_PIV, Y0, s * ZC), (AX_R, WHEEL_R, s * ZW), 0.619)      # (on the trailing arm, x = -1.20)
        yb = 0.75
        xb, frame = X_TR, lambda s, yb=yb: lerp(TRB(s), TRT(s), (yb - Y0) / (Y_TR - Y0))   # (on the rear shock hoop's post)
    ARB[front] = dict(P=P, B=lambda s, P=P, xb=xb, yb=yb: (xb, yb, P(s)[2]), A=lambda s, P=P, yb=yb: (P(s)[0], yb, P(s)[2]), F=frame,
                      sec="arbf" if front else "arbr")
    for s in (1, -1):
        split_at(frame(s))

# ---- the suspension's points
SUSP = []
for s in (1, -1):
    SUSP.append(dict(front=True, s=s, bl=(AX_F, BL[0], s * BL[1]), bu=(AX_F, BU[0], s * BU[1]), n1=(AX_F, WHEEL_R, s * ZW), n2=(AX_F, WHEEL_R, s * (ZW + WHEEL_W)),
                     sa=(RACK_X, WHEEL_R, s * ZW), top=T(s), lower=[(1.36, Y0, s * ZF), (1.64, Y0, s * ZF)], upper=[(1.36, Y_UP, s * ZF), (1.64, Y_UP, s * ZF)],
                     link=(RACK_X, Y0, s * ZF)))
    SUSP.append(dict(front=False, s=s, piv=[(X_PIV, Y0, s * 0.36), (X_PIV, Y0, s * ZC)], n1=(AX_R, WHEEL_R, s * ZW), n2=(AX_R, WHEEL_R, s * (ZW + WHEEL_W)),
                     sm=(SM[0], SM[1], s * 0.84), top=TRT(s)))

divide_tubes()

# ---- the suspension: at the front wishbones on ball joints at the frame, the upright on ball joints at their ends; at
# the rear a trailing arm (a truss from two ball joints on the pivot beam to the hub, and up to the shock's mount)
shocks, stops, hydros, wheels, slides = [], [], [], [], []
rack_ends = {}
TIE_Z = ZF
for sp in SUSP:
    s = sp["s"]
    n1, n2 = fem_node(sp["n1"]), fem_node(sp["n2"])
    member(n1, n2, "hub")
    if sp["front"]:
        bl, bu, sa = (fem_node(sp[k]) for k in ("bl", "bu", "sa"))
        for p in sp["lower"]:
            if p[0] > AX_F:   # (the front leg: through the anti-roll bar's drop link's point)
                pa = ARB[True].setdefault("pn", {})[s] = fem_node(ARB[True]["P"](s))
                member(fem_get(p, 0.002), pa, "arm", ja="ball")
                member(pa, bl, "arm")
                # (the drop link's point in the leg's middle braced to the other pivot: the leg bent there)
                member(fem_get([q for q in sp["lower"] if q[0] <= AX_F][0], 0.002), pa, "brace", ja="ball")
            else:
                member(fem_get(p, 0.002), bl, "arm", ja="ball")
        for p in sp["upper"]:
            member(fem_get(p, 0.002), bu, "arm", ja="ball")
        member(bl, n1, "hub", ja="ball")
        member(n1, bu, "hub", jb="ball")
        member(n1, sa, "hub")
        # (the upright to the axle's outer node too: its camber held by a triangle, not by the upright's bending at
        # one node - on the axle's line, the kingpin free)
        member(bl, n2, "brace", ja="ball")
        member(n2, bu, "brace", jb="ball")
        member(n2, sa, "brace")   # (the steering arm triangulated: on n1 alone the tie rod bent it)
        shocks.append((bl, fem_get(sp["top"], 0.002), True))
        wheels.append((n1, n2, fem_get(sp["top"], 0.002), True))
        re = new_node((RACK_X, Y_RACK, s * 0.28), "plain", 3.0)
        rack_ends[s] = re
        ti = new_node((RACK_X, Y_RACK, s * TIE_Z), "plain", 3.0)
        member(re, ti, "rack")
        member(ti, sa, "tierod")
        slides.append(re)
        # the steering stop: free through the lock and 4 degrees past it
        kp = lerp(sp["bl"], sp["bu"], (WHEEL_R - BL[0]) / (BU[0] - BL[0]))
        r = v_sub(sp["sa"], kp)
        end = sp["link"]
        L0 = v_len(v_sub(sp["sa"], end))
        ds = []
        for deg in (-34.0, 34.0):
            c, sn = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            ds.append(v_len(v_sub(v_add(kp, (r[0] * c - r[2] * sn, r[1], r[0] * sn + r[2] * c)), end)))
        stops.append((sa, fem_get(end, 0.002), (L0 - min(ds)) / L0, (max(ds) - L0) / L0))
    else:
        sm = fem_node(sp["sm"])
        pvs = []
        for p in sp["piv"]:
            pv = fem_get(p, 0.002)
            pvs.append(pv)
            if abs(p[2]) > 0.5:   # (the outer member: through the anti-roll bar's drop link's point)
                pa = ARB[False].setdefault("pn", {})[s] = fem_node(ARB[False]["P"](s))
                member(pv, pa, "tarm", ja="ball")
                member(pa, n1, "tarm")
            else:
                member(pv, n1, "tarm", ja="ball")
            member(pv, sm, "tarm", ja="ball")
        member(sm, n1, "tarm")
        # (the hub carrier to the axle's outer node too: the wheel's camber held by triangles through the arm - on one
        # node, the arm's three tubes took the side force's moment in bending and twist and turned the wheel flat)
        member(sm, n2, "brace")
        member(ARB[False]["pn"][s], n2, "brace")
        # (and the arm a rigid truss on its two pivots: pin-jointed it had two ways to fold besides its swing, held by
        # its joints' bending alone - on a rough field it folded in 30 cm and laid the wheel flat)
        member(pvs[1], n1, "brace", ja="ball")
        member(pvs[0], ARB[False]["pn"][s], "brace", ja="ball")
        shocks.append((sm, fem_get(sp["top"], 0.002), False))
        # (the wheel's torque reacted on the frame over its axle - the cover's bolt node on the rear upper rail, 10 cm
        # behind it: the pair of forces the reaction is made of then runs along the car, through the arm's pivots; from
        # the shock's top, 31 degrees off the vertical, it pushed the wheel down and lifted the car 0.1-0.3 m on the throttle)
        wheels.append((n1, n2, fem_get(RU(s, 0.7), 0.002), False))
member(rack_ends[1], rack_ends[-1], "rack")   # the rack bar
for front, ab in ARB.items():   # the anti-roll bars: bearings, the torsion tube, levers, drop links
    ends = {}
    for s in (1, -1):
        b, a = fem_node(ab["B"](s)), fem_node(ab["A"](s))
        member(fem_get(ab["F"](s), 0.002), b, "pivot", jb="swivel")
        if not front:   # (the rear bearing 38 cm out from the shock hoop's post: braced down to the rail's foot - on a ball at
            member(fem_get(TRB(s), 0.002), b, "pivot", jb="ball")   # the bearing: welded to it the brace held the bar from turning)
        member(b, a, "arm")
        member(a, ab["pn"][s], "tierod", ja="ball", jb="ball")
        ends[s] = b
    member(ends[1], ends[-1], ab["sec"])
RACK_TRAVEL = (AX_F - RACK_X) * math.sin(math.radians(30.0))
hydros.append((rack_ends[1], fem_get((RACK_X, Y_RACK, -ZF), 0.002), -RACK_TRAVEL / (ZF + 0.28)))

# ---- the panels on the cage: bolts at a distance
for p, q in NOSE_BOLTS:
    mounts.append((fem_get(q, 0.002), fem_get(p, 0.002, "nose"), "nose bolt"))
for s, sd in SIDES.items():
    g = sd["grid"]
    for p in (g[0][0], g[0][SIDE_NV], g[len(g) - 1][0], g[len(g) - 1][SIDE_NV]):
        q = v_sub(p, sd["o"])
        mounts.append((nearest_of("body", q, 0.06), fem_get(p, 0.002, sd["part"]), "side bolt"))
for p in COVER_BOLTS:
    mounts.append((fem_get(v_sub(p, (0, PART_OFF, 0)), 0.002), fem_get(p, 0.002, "cover"), "cover bolt"))

# ---- welds, the panels off their tubes, the lamps
weld_panels()
offset_panels(no_gap=lambda pan: False)
mount_lamps()

# ---- the collision hull: a coarse shell on the cage's corners
FLf = lambda s: (1.20, Y0, s * ZF)
hull_quad(FLf(-1), FLf(1), (X_FW, Y0, ZC), (X_FW, Y0, -ZC))                        # the front clip's floor
hull_quad((X_FW, Y0, -ZC), (X_FW, Y0, ZC), (X_MH, Y0, ZC), (X_MH, Y0, -ZC))        # the cockpit's floor
hull_quad((X_MH, Y0, -ZC), (X_MH, Y0, ZC), (-0.95, Y0, ZF), (-0.95, Y0, -ZF))      # the rear's floor
hull_quad(TRB(-1), TRB(1), TL(1), TL(-1))
hull_quad(NB(-1), NB(1), FLf(1), FLf(-1))                                            # under the nose
hull_quad(N(-1), N(1), NB(1), NB(-1))                                                # the nose's face
hull_quad(T(-1), T(1), N(1), N(-1))                                                  # the front's top
hull_quad(D(-1), D(1), T(1), T(-1))
hull_quad(D(-1), D(1), RF(1), RF(-1))                                                # the windscreen's opening
hull_quad(RF(-1), RF(1), RR(1), RR(-1))                                              # the roof
hull_quad(RR(-1), RR(1), MH(1, 1.30), MH(-1, 1.30))                                  # the main hoop, above the harness bar
hull_quad(MH(-1, 1.30), MH(1, 1.30), TT(1), TT(-1))                                  # the engine's top
hull_quad(TT(-1), TT(1), TL(1), TL(-1))                                              # the tail
for s in (1, -1):
    hull_quad(FLf(s), NB(s), N(s), T(s))                                             # the front clip's sides
    hull_quad(FLf(s), T(s), D(s), (X_FW, Y0, s * ZC))
    hull_quad((X_FW, Y0, s * ZC), D(s), MH(s, 1.30), MH(s, Y0))                      # the cockpit's sides
    hull_tri(D(s), RF(s), MH(s, 1.30))
    hull_tri(RF(s), RR(s), MH(s, 1.30))
    hull_quad(MH(s, Y0), MH(s, 1.30), TT(s), TL(s))                                  # the rear's sides
orient_hull()

# ---------------------------------------------------------------------------------------------------------- write
centre, back, left = fem_get((0.10, Y0, 0.0), 0.002), fem_get((X_MH, Y0, 0.0), 0.002), fem_get((0.10, Y0, ZC), 0.002)
floor = lambda k: -0.01 < nodes[k][1] - Y0 < 0.09
engine = [k for k in body_ids if nodes[k][1] < 0.75 and abs(abs(nodes[k][2]) - ZF) < 0.01 and -1.95 < nodes[k][0] < -0.9]   # (the cradle: the rails behind the hoop)
fuel = [k for k in body_ids if floor(k) and -0.96 < nodes[k][0] < X_MH + 0.01]   # (between the main hoop and the cradle)
crew = [k for k in body_ids if floor(k) and X_MH - 0.01 < nodes[k][0] < X_FW + 0.01]
assert len(engine) >= 4 and len(fuel) >= 4 and len(crew) >= 8, (len(engine), len(fuel), len(crew))
spares = [fem_get(RU(s, t), 0.002) for s in (-1, 1) for t in (0.35, 0.7)]
for group, kg in ((engine, ENGINE_KG), (fuel, FUEL_KG), (crew, CREW_KG), (spares, SPARE_KG)):
    for k in group:
        heavy[k] = heavy.get(k, 0.0) + kg / len(group)
for a, b, kind in mounts:   # (the parts' hardware at their bolts)
    heavy[b] = heavy.get(b, 0.0) + MOUNTS[kind][3]
head = (0.0, 1.30, 0.36)
near = sorted(body_ids, key=lambda k: v_len(v_sub(nodes[k], head)))[:8]

# the coil-overs: the travel's ends found from the linkages' kinematics (the front's wishbones in the front view, the
# rear's trailing arm in the side view), the spring from the ride frequency at the wheel (k = F (2 pi f)^2 / (g MR): the
# corner's sprung mass F MR / g, the wheel rate k MR^2), preloaded so the static load holds it at its design length
def rot2(p, c, a):
    ca, sa = math.cos(a), math.sin(a)
    return (c[0] + (p[0] - c[0]) * ca - (p[1] - c[1]) * sa, c[1] + (p[0] - c[0]) * sa + (p[1] - c[1]) * ca)


def circles(c0, r0, c1, r1, near):
    d = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    m = (c0[0] + a * (c1[0] - c0[0]) / d, c0[1] + a * (c1[1] - c0[1]) / d)
    ps = [(m[0] + h * (c1[1] - c0[1]) / d * sg, m[1] - h * (c1[0] - c0[0]) / d * sg) for sg in (1, -1)]
    return min(ps, key=lambda q: math.hypot(q[0] - near[0], q[1] - near[1]))


def front_pose(a):
    """the front corner with the lower arm turned by a (rad; y, z in the front view): (wheel centre's rise, shock length)"""
    pl, pu, bl0, bu0 = (Y0, ZF), (Y_UP, ZF), BL, BU
    hub0 = (WHEEL_R, ZW + 0.5 * WHEEL_W)
    bu = bu0
    for i in range(1, 21):   # (the upper ball joint followed from the design pose: the nearer of the circles' crossings)
        bl = rot2(bl0, pl, a * i / 20)
        bu = circles(pu, math.hypot(bu0[0] - pu[0], bu0[1] - pu[1]), bl, math.hypot(bu0[0] - bl0[0], bu0[1] - bl0[1]), bu)
    t = math.atan2(bu[1] - bl[1], bu[0] - bl[0]) - math.atan2(bu0[1] - bl0[1], bu0[0] - bl0[0])
    hub = rot2((hub0[0] - bl0[0] + bl[0], hub0[1] - bl0[1] + bl[1]), bl, t)
    return hub[0] - hub0[0], math.hypot(Y_TF - bl[0], ZF - bl[1])


def rear_pose(a):
    """the rear corner with the trailing arm turned by a (x, y in the side view): (the hub's rise, shock length)"""
    pv, sm0 = (X_PIV, Y0), SM
    hub = rot2((AX_R, WHEEL_R), pv, a)
    sm = rot2(sm0, pv, a)
    return hub[1] - WHEEL_R, math.sqrt((X_TR - sm[0]) ** 2 + (Y_TR - sm[1]) ** 2 + (0.50 - 0.84) ** 2)


def at_rise(pose, w):
    lo, hi = -0.95, 0.95
    up = pose(0.01)[0] > 0
    for _ in range(80):
        m = 0.5 * (lo + hi)
        if (pose(m)[0] < w) == up: lo = m
        else: hi = m
    return pose(0.5 * (lo + hi))[1]


SHOCK = {}
for front, pose in ((True, front_pose), (False, rear_pose)):
    L0 = pose(0.0)[1]
    bump, droop = TRAVEL[front]
    mr = (at_rise(pose, 0.01) - at_rise(pose, -0.01)) / -0.02      # (the shock shortens as the wheel rises)
    F = STATIC[front]
    k = F * (2 * math.pi * RIDE_HZ[front]) ** 2 / (9.81 * mr)
    m_corner = F * mr / 9.81
    c = 2 * ZETA * math.sqrt(k * mr * mr * m_corner) / (mr * mr)
    pre = 1 + F / (k * L0)
    L = L0 * pre
    s_stop, s_hard, s_droop = at_rise(pose, bump - BUMP_STOP), at_rise(pose, bump), at_rise(pose, -droop)
    lo, hi = 1 - s_stop / L, max(0.0, s_droop / L - 1)
    # the bump stop (the shock past its short bound: its spring and damping blend towards the bound's by the metres in,
    # BeamLab's shocks): BUMP_STOP_G x the static load at the end of it, three times the damping
    zone = s_stop - s_hard
    kb = (BUMP_STOP_G * F / (L - s_hard) - k) / zone
    cb = c + 2 * c / zone
    # the travel's hard ends (a second shock of no spring, at the design length: its bounds are of that): the bump
    # stop's end and the droop strap
    SHOCK[front] = dict(k=k, c=c, lo=lo, hi=hi, pre=pre, kb=kb, cb=cb, hard_lo=1 - s_hard / L0, hard_hi=s_droop / L0 - 1 + 0.005)
    print("%s: shock %.3f m, motion ratio %.2f, %.0f kg sprung, spring %.0f N/m, damping %.0f N s/m, preload %.3f, stroke %.3f m up "
          "(%.3f in the bump stop: %.0f N/m, %.0f N s/m), %.3f m down" % ("front" if front else "rear", L0, mr, m_corner, k, c, pre, L0 - s_hard, zone, kb, cb, s_droop - L0))
    if L < s_droop:
        print("  (the spring is free %.0f mm before full droop: the wheel hangs short of it)" % (1000 * (s_droop - L)))
shocks_text = "shocks\n;n1, n2, spring, damp, short bound, long bound, precompression, options\n"
for front in (True, False):
    sh = SHOCK[front]
    shocks_text += "set_beam_defaults %.0f, %.0f, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n" % (sh["kb"], sh["cb"])
    for a, b, f in shocks:
        if f == front:
            shocks_text += "%d, %d, %.0f, %.0f, %.3f, %.3f, %.3f, n\n" % (a, b, sh["k"], sh["c"], sh["lo"], sh["hi"], sh["pre"])
shocks_text += "set_beam_defaults 20000000, 200000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n"
for a, b, front in shocks:
    shocks_text += "%d, %d, 0, 0, %.3f, %.3f, 1.0, i\n" % (a, b, SHOCK[front]["hard_lo"], SHOCK[front]["hard_hi"])
shocks_text += "set_beam_defaults 3000000, 20000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n"
for a, b, lo, hi in stops:
    shocks_text += "%d, %d, 0, 0, %.3f, %.3f, 1.0, i\n" % (a, b, lo, hi)
wheels_text = "wheels\n;radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, face, band\n"
wheels_text += "set_beam_defaults 3000000, 400, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n"
for n1, n2, arm, front in wheels:   # (the rear wheels driven)
    # (20 rays: 14 on a 39" tyre were 22 cm facets that caught the ground sliding sideways; the tread's springs the same in all)
    wheels_text += "%.2f, %.2f, 20, %d, %d, 9999, 1, %d, %d, 60.0, 105000.0, 840.0, tracks/wheelface tracks/wheelband\n" % (WHEEL_R, WHEEL_W, n1, n2, 0 if front else 1, arm)
engine_text = ("engine\n;min rpm, max rpm, torque, differential, reverse, neutral, gears...\n900.0, 6500.0, 850.0, 4.9, 3.0, 1.0, 2.9, 1.8, 1.3, 1.0, -1.0\n"
               "engoption\n0.12, c, 1000.0, 0.3, 0.4, 0.3\nbrakes\n6000\n")
housing = [fem_get((RACK_X, Y_RACK, -ZF), 0.002), fem_get((RACK_X, Y_RACK, 0.0), 0.002), fem_get((RACK_X, Y_RACK, ZF), 0.002)]
all_tris = write(OUT, dict(
    title="Desert Buggy",
    head=["generated by tools/make_buggy.py: an unlimited-class desert racer (a Class 1 buggy) on a welded chromoly cage (FEM frame",
          "elements), long-travel wishbones and trailing arms, its aluminium panels welded on; the nose, side panels and engine cover",
          "frames of their own on bolts that let go"],
    globals=(DRY_KG, "sheet/Aluminium/3.3/0.004/%d" % REFINE), sections=SECTIONS, mat=MAT, mount_kinds=MOUNTS, latch_note="",
    shocks_text=shocks_text, hydros=hydros, slides=[(n, housing) for n in slides], wheels_text=wheels_text, engine_text=engine_text,
    cameras=(centre, back, left), cinecam=(head, near), uv_span=(2.3, 4.6, 1.1, 3.2, Y0)))
summary(OUT, all_tris)
print("%d mounts: %s" % (len(mounts), ", ".join("%s %d" % (k, sum(1 for m in mounts if m[2] == k)) for k in MOUNTS)))
