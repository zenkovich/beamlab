#!/usr/bin/env python3
"""Writes assets/vehicles/frame_car/frame_car.truck: a two-seater on a tubular space frame of FEM frame elements.

The frame is chromoly tube welded at its joints (phys/frame_fem.h): floor rails with cross members and diagonals, sills
and belt rails, a cowl, a roll cage (A pillars, the main hoop with a diagonal, roof rails and bows, rear stays),
suspension towers and bumper bulkheads. The body panels are sheet metal (cab triangles the game turns into triangle
elements, as the Sheet Car's) hung on it: the roof, the bonnet, the doors and the rear deck, their edges on the frame's
tubes (divided where the panels' grid meets them).

Suspension: double wishbones of tube. Each wishbone is an A of two tubes welded at its outer end and on ball joints at
the frame, so it swings about the line through its two mounts; the upright (the axle stub and a steering arm welded
together) sits on ball joints at the wishbones' ends and turns about them. A shock from the lower ball joint to the
tower carries the car, preloaded so it stands at the design height where both wishbones are level (the lower one on
brackets under the floor rail); beside it a travel stop (a shock without spring: a bump stop at 20% compression, a droop
strap at 4% extension) ends the wheel's travel ~10 cm up and ~2.5 cm down from there. At the front a tie rod (a hydro) from the steering arm to a rack point on the frame steers; at
the rear a toe link of tube on ball joints holds the wheel straight. Rear-wheel drive.

Coordinates follow Rigs of Rods: -x forward, y up, +z left. The design is written with the front at +x and mirrored
when the nodes are made.

    python3 tools/make_frame_car.py
"""
import math
import os

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "vehicles", "frame_car", "frame_car.truck")

Y0 = 0.30            # floor rails
ZR = 0.40            # floor rails' half width (and the towers)
ZS = 0.62            # the body's sides: sills, belt rails
YB = 0.80            # belt line
YR = 1.28            # roof
ZT = 0.55            # roof rails
XF = [1.75, 1.45, 1.05, 0.75, 0.30, -0.15, -0.65, -1.05, -1.45, -1.75]   # floor stations
DOOR_X = [0.75, 0.52, 0.30, 0.075, -0.15, -0.40, -0.65]                    # sill and belt rail nodes (the doors' grid)
ROOF_X = [0.35, 0.10, -0.15, -0.40, -0.65]
AXLES = (1.25, -1.25)
WHEEL_R = 0.30
YT = 0.62            # tower tops
YL = 0.17            # the lower wishbones' pivots (brackets under the floor rails), level with the lower ball joints
YM = (Y0 + YT) / 2   # the upper wishbones' pivots (the towers' posts halfway up), about level with the upper ball joints
# the coil-overs' preload (RoR precompression: the spring's free length over its fitted length): the car stands at its
# design height, where the wishbones are level, and hangs little lower when lifted (the droop strap: 4% of the fitted
# length); measured sag without it 9.0% at the front, 7.5% at the rear
PRELOAD = {True: 1.090, False: 1.075}

# sections: material, shape, outer (m), wall (m), joints at the ends by default
SECTIONS = {
    "main": ("Chromoly", "tube", 0.040, 0.0020, "rigid"),   # rails, cross members, the body's frame
    "cage": ("Chromoly", "tube", 0.045, 0.0025, "rigid"),   # the roll cage: A pillars, main hoop, roof, rear stays
    "light": ("Chromoly", "tube", 0.030, 0.0020, "rigid"),  # diagonals, panel frames
    "arm": ("Chromoly", "tube", 0.035, 0.0030, "rigid"),    # wishbones, the toe links (a leg folding on a curb strike: 30 x 2.5 was too light)
    "hub": ("Steel", "tube", 0.050, 0.0060, "rigid"),       # the uprights (a casting in a real car: the wheel's load bends it)
}

nodes = []          # (x, y, z) in RoR space
node_of = {}
members = []        # (a, b, section, joint a or None, joint b or None)
member_set = set()
cabs = []
shocks = []
hydros = []
wheels = []
plain_nodes = set()


def node(x, y, z):
    key = (round(x, 3), round(y, 3), round(z, 3))
    if key not in node_of:
        node_of[key] = len(nodes)
        nodes.append((-key[0], key[1], key[2]))
    return node_of[key]


def lerp(p, q, t):
    return tuple(p[i] + (q[i] - p[i]) * t for i in range(3))


def member(p, q, sec, div=1, ja=None, jb=None):
    """A chain of `div` frame elements from p to q (design space): the nodes along it."""
    ids = [node(*lerp(p, q, k / div)) for k in range(div + 1)]
    for k in range(div):
        a, b = ids[k], ids[k + 1]
        if a == b or (min(a, b), max(a, b)) in member_set:
            continue
        member_set.add((min(a, b), max(a, b)))
        members.append((a, b, sec, ja if k == 0 else None, jb if k == div - 1 else None))
    return ids


def both(f):
    for s in (1, -1):
        f(s)


def panel(p00, p10, p01, p11, nu, nv):
    """A sheet panel over the quad (bilinear), nu x nv cells: its nodes (the frame's where they meet) and triangles."""
    grid = [[node(*lerp(lerp(p00, p10, i / nu), lerp(p01, p11, i / nu), j / nv)) for j in range(nv + 1)] for i in range(nu + 1)]
    for i in range(nu):
        for j in range(nv):
            a, b, c, d = grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]
            if (i + j) % 2 == 0:
                cabs.append((a, b, c))
                cabs.append((a, c, d))
            else:
                cabs.append((a, b, d))
                cabs.append((b, c, d))


def side_rail(front, s, k):
    """node k (of 4) of the bonnet's (front) or the rear deck's side rail"""
    return lerp((0.75, YB, s * ZS), (1.75, 0.62, s * ZS), k / 4) if front else lerp((-0.65, YB, s * ZS), (-1.75, 0.65, s * ZS), k / 4)


# ---- floor: rails, cross members (split at the rack points and the middle), V diagonals between them
def floor_side(s):
    for x0, x1 in zip(XF, XF[1:]):
        member((x0, Y0, s * ZR), (x1, Y0, s * ZR), "main")


both(floor_side)
for x in XF:
    member((x, Y0, -ZR), (x, Y0, ZR), "main", div=4)
for x0, x1 in zip(XF, XF[1:]):
    member((x0, Y0, ZR), (x1, Y0, 0.0), "light")
    member((x0, Y0, -ZR), (x1, Y0, 0.0), "light")


# ---- sides: sills and belt rails at the body's width, door posts, outriggers to the floor rails
def sides(s):
    for x0, x1 in zip(DOOR_X, DOOR_X[1:]):
        member((x0, Y0, s * ZS), (x1, Y0, s * ZS), "main")
        member((x0, YB, s * ZS), (x1, YB, s * ZS), "main")
    for x in (0.75, -0.15, -0.65):                      # A post (door front), B post, C post: split at the door's middle row
        member((x, Y0, s * ZS), (x, YB, s * ZS), "main", div=2)
    for x in (0.75, 0.30, -0.15, -0.65):
        member((x, Y0, s * ZR), (x, Y0, s * ZS), "main")
    member((0.75, Y0, s * ZS), (0.30, YB, s * ZS), "light")   # door frame diagonals (under the skin)
    member((-0.65, Y0, s * ZS), (-0.15, YB, s * ZS), "light")
    panel((0.75, Y0, s * ZS), (-0.65, Y0, s * ZS), (0.75, YB, s * ZS), (-0.65, YB, s * ZS), 6, 2)   # the door skins


both(sides)

# ---- the cage: A pillars, roof rails and bows, the main hoop (B) with its diagonal, rear stays
member((0.75, YB, -ZS), (0.75, YB, ZS), "main", div=4)          # cowl


def cage(s):
    member((0.75, YB, s * ZS), (ROOF_X[0], YR, s * ZT), "cage", div=2)       # A pillar
    for x0, x1 in zip(ROOF_X, ROOF_X[1:]):
        member((x0, YR, s * ZT), (x1, YR, s * ZT), "cage")                    # roof rail
    member((-0.15, YB, s * ZS), (-0.15, YR, s * ZT), "cage", div=2)          # B pillar (main hoop)
    member((-0.65, YR, s * ZT), side_rail(False, s, 2), "cage", div=2)       # rear stay


both(cage)
for x in (ROOF_X[0], -0.15, ROOF_X[-1]):
    member((x, YR, -ZT), (x, YR, ZT), "cage", div=4)                         # roof bows
member((-0.15, YR, ZT), (-0.15, Y0, -ZR), "cage", div=3)                     # main hoop diagonal
panel((ROOF_X[0], YR, -ZT), (ROOF_X[-1], YR, -ZT), (ROOF_X[0], YR, ZT), (ROOF_X[-1], YR, ZT), 4, 4)   # the roof


# ---- front: bonnet frame on towers, bumper bulkhead; rear: deck frame, towers, bulkhead
def ends(s):
    member((0.75, YB, s * ZS), (1.75, 0.62, s * ZS), "main", div=4)          # bonnet side rails (fender tops)
    member((1.75, Y0, s * ZR), (1.75, 0.62, s * ZS), "main")                 # front bulkhead posts
    member((-0.65, YB, s * ZS), (-1.75, 0.65, s * ZS), "main", div=4)        # rear deck side rails
    member((-1.75, Y0, s * ZR), (-1.75, 0.65, s * ZS), "main")               # rear bulkhead posts
    for ax in AXLES:
        # the towers: two posts from the floor rail, their top rail (the shock mount in its middle), braced
        xa, xb = ax + 0.20, ax - 0.20
        member((xa, Y0, s * ZR), (xa, YT, s * ZR), "main", div=2)   # (their middle: the steering / toe link's inner joint)
        member((xb, Y0, s * ZR), (xb, YT, s * ZR), "main", div=2)
        member((xa, YT, s * ZR), (xb, YT, s * ZR), "main", div=2)
        member((xa, Y0, s * ZR), (ax, YT, s * ZR), "light")
        member((xb, Y0, s * ZR), (ax, YT, s * ZR), "light")
        # braces from the tower tops to the bonnet / deck side rail (to its division nodes)
        outer = (xa, YT, s * ZR) if ax > 0 else (xb, YT, s * ZR)
        member(outer, side_rail(ax > 0, s, 3), "light")
        member((ax, YT, s * ZR), side_rail(ax > 0, s, 2), "light")


both(ends)
member((1.75, 0.62, -ZS), (1.75, 0.62, ZS), "main", div=4)                    # front upper cross
member((-1.75, 0.65, -ZS), (-1.75, 0.65, ZS), "main", div=4)                  # rear upper cross
member((-0.65, YB, -ZS), (-0.65, YB, ZS), "main", div=4)                      # rear bulkhead top (parcel shelf)
for ax in AXLES:
    member((ax, YT, -ZR), (ax, YT, ZR), "main", div=2)                        # strut brace across the towers
panel((0.75, YB, -ZS), (1.75, 0.62, -ZS), (0.75, YB, ZS), (1.75, 0.62, ZS), 4, 4)        # the bonnet
panel((-0.65, YB, -ZS), (-1.75, 0.65, -ZS), (-0.65, YB, ZS), (-1.75, 0.65, ZS), 4, 4)    # the rear deck

# ---- suspension: wishbones on ball joints at the frame, welded at their outer ends; the upright on ball joints there.
# Both wishbones are level at the design height (the lower one on brackets under the floor rail, the upper one on the
# towers' posts halfway up, shorter): the wheel keeps its track through its travel and gains negative camber in bump;
# the steering (front) and toe (rear) links run level from the steering arm at the axle's height to the floor rail, as
# long as the wishbones, so the wheel does not steer itself as it moves
def brackets(s):
    for ax in AXLES:
        for dx in (0.20, -0.20):
            member((ax + dx, Y0, s * ZR), (ax + dx, YL, s * ZR), "main")          # drop bracket
            member((ax + dx, YL, s * ZR), (ax + dx, Y0, s * 0.2), "light")        # its brace across to the cross member
        member((ax + 0.20, YL, s * ZR), (ax - 0.20, YL, s * ZR), "main")          # the pivot rail
        member((ax + 0.20, Y0, s * ZR), (ax - 0.20, YL, s * ZR), "light")         # its diagonal


both(brackets)
for ax in AXLES:
    front = ax > 0
    for s in (1, -1):
        bl = (ax, 0.17, s * 0.64)          # lower ball joint
        bu = (ax, 0.50, s * 0.60)          # upper ball joint
        n1 = (ax, WHEEL_R, s * 0.66)       # axle: inner and outer node
        n2 = (ax, WHEEL_R, s * 0.84)
        arm_x = 1.05 if front else -1.05   # steering arm / toe link arm on the upright, behind (front) or ahead (rear)
        sa = (arm_x, WHEEL_R, s * 0.64)
        for dx in (0.20, -0.20):
            member((ax + dx, YL, s * ZR), bl, "arm", ja="ball")     # lower wishbone: its legs welded at the ball joint's end
            member((ax + dx, YM, s * ZR), bu, "arm", ja="ball")     # upper wishbone
        member(bl, n1, "hub", ja="ball")    # the upright: on the ball joints, the stub and the arm welded to it
        member(n1, bu, "hub", jb="ball")
        member(n1, n2, "hub")
        member(n1, sa, "hub")
        shocks.append((node(*bl), node(ax, YT, s * ZR), PRELOAD[front]))
        plain_nodes.update({node(*bl), node(ax, YT, s * ZR)})
        rack = (arm_x, Y0, s * ZR)       # (the link's inner joint: on the floor rail, level with the steering arm)
        if front:
            hydros.append((node(*sa), node(*rack), -0.15 * s))  # the tie rod (the arm behind the axle: the other sign than an arm ahead)
            plain_nodes.update({node(*sa), node(*rack)})
        else:
            member(sa, rack, "arm", ja="ball", jb="ball")        # the toe link
        wheels.append((node(*n1), node(*n2), node(ax + 0.20, Y0, s * ZR), front))
        plain_nodes.update({node(*n1), node(*n2)})

# ---- cab triangles facing out (away from the car's middle)
cx0 = 0.0
for i, (a, b, c) in enumerate(cabs):
    pa, pb, pc = (nodes[k] for k in (a, b, c))
    u = [pb[k] - pa[k] for k in range(3)]
    v = [pc[k] - pa[k] for k in range(3)]
    n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    m = [(pa[k] + pb[k] + pc[k]) / 3 for k in range(3)]
    out = (m[0] * 0.3, m[1] - 0.7, m[2])
    if n[0] * out[0] + n[1] * out[1] + n[2] * out[2] < 0:
        cabs[i] = (a, c, b)
skin = {k for t in cabs for k in t}

# ---- write
os.makedirs(os.path.dirname(OUT), exist_ok=True)
by_sec = {}
for a, b, sec, ja, jb in members:
    by_sec.setdefault(sec, []).append((a, b, ja, jb))


def spec(sec):
    m, sh, o, w, j = SECTIONS[sec]
    return "%s, %s, %.4f, %.4f, %s" % (m, sh, o, w, j)


centre, back, left = node(-0.15, Y0, 0.0), node(-1.75, Y0, 0.0), node(-0.15, Y0, ZR)
head = (0.05, 0.98, 0.25)
near = sorted(range(len(nodes)), key=lambda k: sum((nodes[k][i] - (-head[0] if i == 0 else head[i])) ** 2 for i in range(3)))
near = [k for k in near if k not in skin][:8]
with open(OUT, "w") as f:
    f.write("Frame Car\n")
    f.write(";generated by tools/make_frame_car.py: a two-seater on a welded chromoly space frame (FEM frame elements), double\n")
    f.write(";wishbones on ball joints, sheet metal panels (triangle elements) on the frame\n")
    f.write("globals\n;dry mass, cargo mass, cab material (sheet/<material>/<kg per m2>/<drawn thickness>: the panels are a sheet body)\n")
    f.write("420.0, 0.0, sheet/Steel/7.9/0.004\n")
    f.write("minimass\n4.0\n")
    f.write("nodes\n;id, x, y, z, options\n")
    for i, (x, y, z) in enumerate(nodes):
        f.write("%d, %.3f, %.3f, %.3f, %s\n" % (i, x, y, z, "l" if i not in skin or i in plain_nodes else "n"))
    f.write("beams\n")
    for sec, lst in by_sec.items():
        f.write(";%s\nset_beam_defaults 3000000, 400, 80000, 700000, 0.05, tracks/beam, 0\nset_frame_section %s\n" % (sec, spec(sec)))
        default = SECTIONS[sec][4]
        for a, b, ja, jb in lst:
            if ja or jb:
                f.write("%d, %d, F, %s, %s\n" % (a, b, ja or default, jb or default))
            else:
                f.write("%d, %d, F\n" % (a, b))
    f.write("shocks\n;n1, n2, spring, damp, short bound, long bound, precompression, options\n")
    f.write("set_beam_defaults 9000000, 12000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, pre in shocks:
        f.write("%d, %d, 60000, 4000, 0.25, 0.25, %.3f, n\n" % (a, b, pre))
    # the travel stops beside each coil-over: no spring or damper inside the travel, beyond it a stop that stiffens by
    # 2e7 N/m per metre (a rubber bump stop: 15 kN at 1 cm, 55 kN at 3 cm) at 20% compression (the wheel up ~10 cm
    # from the design height, where the preloaded car stands) and a droop strap at 4% extension (a lifted car's wheels
    # hang ~2.5 cm lower, the strap holding the spring's preload); the shock's own bounds (25% of its free length,
    # RoR's soft ramp) lie beyond them
    f.write(";travel stops: bump stop and droop strap on the coil-over's mounts\n")
    f.write("set_beam_defaults 20000000, 200000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, _ in shocks:
        f.write("%d, %d, 0, 0, 0.20, 0.04, 1.0, i\n" % (a, b))
    f.write("hydros\n;tie rods: node1, node2, factor, options\n")
    f.write("set_beam_defaults 4000000, 500, 99999999999999999999999999999999999999999, 99999999999999999999999999999999999999999, 0.02, tracks/beam, 0\n")
    for a, b, fac in hydros:
        f.write("%d, %d, %.2f, i\n" % (a, b, fac))
    f.write("wheels\n;radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, face, band\n")
    # (the rim's beams take these defaults: 3e6 N/m on 1.9 kg tread nodes is within the explicit step's budget)
    f.write("set_beam_defaults 3000000, 400, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for n1, n2, arm, front in wheels:
        f.write("%.2f, 0.20, 12, %d, %d, 9999, 1, %d, %d, 45.0, 120000.0, 900.0, tracks/wheelface tracks/wheelband\n" % (WHEEL_R, n1, n2, 0 if front else 1, arm))
    f.write("engine\n;min rpm, max rpm, torque, differential, reverse, neutral, gears...\n1000.0, 6800.0, 240.0, 4.1, 3.2, 1.0, 3.3, 2.1, 1.5, 1.15, 0.9, -1.0\n")
    f.write("engoption\n0.35, c, 200.0, 0.3, 0.4, 0.3\n")
    f.write("brakes\n3500\n")
    f.write("cameras\n%d, %d, %d\n" % (centre, back, left))
    f.write("cinecam\n%.3f, %.3f, %.3f, %s\n" % (-head[0], head[1], head[2], ", ".join(str(k) for k in near)))
    f.write("contacters\n")
    for i in range(len(nodes)):
        f.write("%d\n" % i)
    f.write("submesh\ntexcoords\n")
    for i in sorted(skin):
        x, y, z = nodes[i]
        f.write("%d, %.3f, %.3f\n" % (i, (x + 1.8) / 3.6, (z + 0.7 + (y - Y0)) / 2.4))
    f.write("cab\n")
    for a, b, c in cabs:
        f.write("%d, %d, %d, c\n" % (a, b, c))
    f.write("end\n")
print("wrote %s: %d nodes (%d on panels), %d frame elements, %d panel triangles, %d shocks, %d tie rods, %d wheels" % (
    OUT, len(nodes), len(skin), len(members), len(cabs), len(shocks), len(hydros), len(wheels)))
