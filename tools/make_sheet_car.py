#!/usr/bin/env python3
"""Writes assets/vehicles/sheet_car/sheet_car.truck: a hatchback whose body is a sheet on a deformable space frame.

The frame (floor rails, sills, cross members, pillars, roof bows, bulkheads, bonnet and boot supports, axle mounts)
is nodes and beams as in any Rigs of Rods vehicle, set to yield plastically and never break: a crash bends the cage,
it does not fall apart. The body panels are a grid of nodes over the car's outline (a profile swept across seven rows
with a rounded shoulder and a tapered plan, side walls with belt and sill rows and wheel arches) with `cab` triangles
between them and no beams: the game turns those triangles into triangle elements (phys::Shell) when the car is spawned
as a sheet car (Vehicle::make_sheet_body), so the body dents, tears and cracks like the sheets of the Materials Lab.
The pillars and bows run through skin nodes: the skin hangs on the cage.

Steering: the outer axle node of each front wheel swings about a kingpin axis (the inner axle node and a node above
it) and a hydro tie rod to a rack node on the frame turns it.

Coordinates follow Rigs of Rods: -x forward, y up, +z left. The design below is written with the front at +x and
mirrored when the nodes are made.

    python3 tools/make_sheet_car.py
"""
import math
import os

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "vehicles", "sheet_car", "sheet_car.truck")

# body outline in the x-y plane, front to back (design space: +x forward)
# (consecutive points share an x, a vertical face, or are at least 0.12 m apart: a narrow strip of the top rows
# is a sliver whose stiff hinges would cap the bending stiffness of the whole body)
DESIGN = [(2.12, 0.30), (2.12, 0.50), (1.98, 0.68), (1.80, 0.74), (1.45, 0.82), (1.05, 0.88), (0.80, 0.92),   # bumper, bonnet
          (0.62, 1.10), (0.42, 1.30), (0.25, 1.42),                                                            # windscreen
          (-0.15, 1.46), (-0.60, 1.45), (-0.95, 1.36),                                                         # roof
          (-1.25, 1.14), (-1.55, 0.95), (-1.85, 0.80),                                                         # rear window, hatch
          (-2.05, 0.60), (-2.05, 0.32)]                                                                        # rear bumper
Z_ROWS = [-0.78, -0.56, -0.29, 0.0, 0.29, 0.56, 0.78]    # rows of the top strip across the car
FLOOR_Y = 0.36
RAIL_Z = 0.52            # floor rails (inner), the sills are the side walls' bottom row
WHEEL_X = (1.32, -1.30)
WHEEL_R = 0.32
ARCH_R = 0.46            # wheel arch radius about the axle
PILLAR_X = [0.80, -0.15, -0.95]     # A, B, C pillars (cowl, behind the front seats, rear)
BULKHEAD_X = [1.80, 0.80, -0.95, -1.85]


# the profile: the design polyline with the wheel arches' x positions put in (the side walls need columns there)
ARCH_XS = sorted({wx + d for wx in WHEEL_X for d in (-ARCH_R, -ARCH_R * 0.55, 0.0, ARCH_R * 0.55, ARCH_R)}, reverse=True)
PROFILE = []
for (x0, y0), (x1, y1) in zip(DESIGN, DESIGN[1:]):
    PROFILE.append((x0, y0))
    for ax in ARCH_XS:
        if x1 + 0.10 < ax < x0 - 0.10:
            PROFILE.append((ax, y0 + (y1 - y0) * (x0 - ax) / (x0 - x1)))
PROFILE.append(DESIGN[-1])


def width_scale(x):
    """plan taper towards the bumpers"""
    t = max(0.0, (abs(x) - 1.35) / 0.8)
    return 1.0 - 0.14 * min(1.0, t) ** 2


def shoulder_y(y, z):
    """rounded shoulder: the outer rows sit lower"""
    return FLOOR_Y + (y - FLOOR_Y) * (1.0 - 0.10 * (abs(z) / Z_ROWS[-1]) ** 4)


def sill_y(x):
    """the bottom of the side wall: the floor, lifted over the wheel arches"""
    y = FLOOR_Y
    for wx in WHEEL_X:
        d = abs(x - wx)
        if d < ARCH_R - 1e-6:
            y = max(y, WHEEL_R + math.sqrt(ARCH_R * ARCH_R - d * d))
    return y


nodes = []            # (x, y, z) in RoR space
node_of = {}
frame_nodes = set()
beams = []
beam_set = set()
cabs = []


def node(x, y, z):
    key = (round(x, 3), round(y, 3), round(z, 3))
    if key not in node_of:
        node_of[key] = len(nodes)
        nodes.append((-x, y, z)) # (mirrored: RoR drives towards -x)
    return node_of[key]


def beam(a, b):
    if a != b and (a, b) not in beam_set and (b, a) not in beam_set:
        beam_set.add((a, b))
        beams.append((a, b))
        frame_nodes.add(a)
        frame_nodes.add(b)


def strut(a, b, c):
    """a triangle of beams"""
    beam(a, b)
    beam(b, c)
    beam(a, c)


# ---- skin: the top strip (front face, bonnet, windscreen, roof, rear window, tailgate)
strip = [[node(x, shoulder_y(y, z), z * width_scale(x)) for (x, y) in PROFILE] for z in Z_ROWS]
for r in range(len(Z_ROWS) - 1):
    for i in range(len(PROFILE) - 1):
        a, b = strip[r][i], strip[r][i + 1]
        c, d = strip[r + 1][i + 1], strip[r + 1][i]
        if (r + i) % 2 == 0:
            cabs += [(a, b, c), (a, c, d)]
        else:
            cabs += [(a, b, d), (b, c, d)]

# ---- skin: the side walls, columns of edge / belt / mid / sill nodes zipped into triangles
side_cols = {}
for s in (1, -1):
    edge = strip[-1] if s > 0 else strip[0]
    cols = []
    for j, (x, y) in enumerate(PROFILE):
        zz = s * Z_ROWS[-1] * width_scale(x)
        top = nodes[edge[j]][1]
        bot = sill_y(x)
        h = top - bot
        fr = [0.72, 0.45, 0.2] if h > 0.75 else ([0.5] if h > 0.3 else [])
        col = [edge[j]] + [node(x, bot + h * f, zz) for f in fr] + ([node(x, bot, zz)] if h > 0.12 else [])
        cols.append(col)
    side_cols[s] = cols
    for i in range(len(cols) - 1):
        ca, cb = cols[i], cols[i + 1]
        # zipper: advance on the column whose next node is lower in normalized height
        ya = [nodes[k][1] for k in ca]
        yb = [nodes[k][1] for k in cb]
        ta = [(ya[0] - y) / max(1e-6, ya[0] - ya[-1]) for y in ya]
        tb = [(yb[0] - y) / max(1e-6, yb[0] - yb[-1]) for y in yb]
        p = q = 0
        while p < len(ca) - 1 or q < len(cb) - 1:
            if q == len(cb) - 1 or (p < len(ca) - 1 and ta[p + 1] <= tb[q + 1]):
                cabs.append((ca[p], cb[q], ca[p + 1]))
                p += 1
            else:
                cabs.append((ca[p], cb[q], cb[q + 1]))
                q += 1

# ---- frame: floor rails, spine, cross members, sills tied in
rail = {s: [node(x, FLOOR_Y, s * RAIL_Z * width_scale(x)) for (x, y) in PROFILE] for s in (1, -1)}
spine = [node(x, FLOOR_Y, 0.0) for (x, y) in PROFILE]
for s in (1, -1):
    sills = [c[-1] for c in side_cols[s]]
    for i in range(len(PROFILE) - 1):
        beam(rail[s][i], rail[s][i + 1])
        beam(spine[i], spine[i + 1])
        beam(rail[s][i], spine[i + 1])
        beam(spine[i], rail[s][i + 1])
        beam(sills[i], sills[i + 1])
        beam(sills[i], rail[s][i + 1])
        beam(rail[s][i], sills[i + 1])
    for i in range(len(PROFILE)):
        beam(rail[s][i], spine[i])
        beam(rail[s][i], sills[i])

# ---- frame: pillars, belt rails, roof bows (through the skin nodes: the skin hangs on the cage)
j_of = {}
for j, (x, y) in enumerate(PROFILE):
    j_of.setdefault(round(x, 3), j)
col_at = {s: {round(PROFILE[j][0], 3): side_cols[s][j_of[round(PROFILE[j][0], 3)]] for j in range(len(PROFILE))} for s in (1, -1)}
for px in PILLAR_X:
    j = j_of[round(px, 3)]
    for s in (1, -1):
        col = col_at[s][round(px, 3)]
        for a, b in zip(col, col[1:]):
            beam(a, b)                       # the pillar
        beam(col[0], rail[s][j])             # to the floor
        if len(col) > 2:
            beam(col[1], rail[s][j])
            beam(col[1], spine[j])
    # roof bow across the car
    for k in range(len(Z_ROWS) - 1):
        beam(strip[k][j], strip[k + 1][j])
    beam(strip[0][j], spine[j])
    beam(strip[-1][j], spine[j])
# belt rails and roof rails along the car between the pillars, with shear diagonals
for s in (1, -1):
    for j in range(len(PROFILE) - 1):
        x0, x1 = PROFILE[j][0], PROFILE[j + 1][0]
        if x0 > PILLAR_X[0] + 1e-6 or x1 < PILLAR_X[-1] - 1e-6:
            continue
        c0, c1 = side_cols[s][j], side_cols[s][j + 1]
        beam(c0[0], c1[0])                   # roof rail
        if len(c0) > 2 and len(c1) > 2:
            beam(c0[1], c1[1])               # belt rail
            beam(c0[1], c1[0])               # shear diagonal
            beam(c0[1], c1[-1])              # sill diagonal
# ---- frame: bulkheads (cowl, rear seat, bumpers): the cross section braced to the floor
for bx in BULKHEAD_X:
    j = j_of[round(bx, 3)]
    for s in (1, -1):
        col = side_cols[s][j]
        for a, b in zip(col, col[1:]):
            beam(a, b)
        beam(col[0], rail[s][j])
        beam(col[0], spine[j])
        beam(col[0], rail[-s][j])
    for k in range(len(Z_ROWS) - 1):
        beam(strip[k][j], strip[k + 1][j])
    beam(strip[len(Z_ROWS) // 2][j], spine[j])
# ---- frame: bonnet and boot supports (the panels crumple with the frame), fenders to the rails
for j, (x, y) in enumerate(PROFILE):
    if x > PILLAR_X[0] + 1e-6 or x < PILLAR_X[-1] - 1e-6:
        for s in (1, -1):
            col = side_cols[s][j]
            beam(col[0], rail[s][j])         # fender edge to the rail
            beam(col[-1], rail[s][j])
            if j + 1 < len(PROFILE):
                beam(col[0], rail[s][j + 1])
        if j % 2 == 0:
            beam(strip[len(Z_ROWS) // 2][j], spine[j])
            beam(strip[2][j], rail[1][j])
            beam(strip[4][j], rail[-1][j])

# ---- axles: inner node on the rail line, outer node in the arch, a strut tower above each wheel (the rails and
# the axle are nearly in one plane: without the tower the wheel load folded the mounts and the wheels splayed);
# on the front axle the outer node swings about the kingpin (inner node - tower) and a tie rod to a rack turns it
wheels = []
hydros = []
for wx in WHEEL_X:
    front = wx > 0
    near = sorted(range(len(PROFILE)), key=lambda i: abs(PROFILE[i][0] - wx))[:3]
    arch = sorted(range(len(PROFILE)), key=lambda i: abs(PROFILE[i][0] - wx))[:5]   # the arch's columns
    for s in (1, -1):
        n1 = node(wx, WHEEL_R, s * 0.56)
        n2 = node(wx, WHEEL_R, s * 0.86)
        tower = node(wx, WHEEL_R + 0.34, s * 0.56)
        for i in near:
            beam(n1, rail[s][i])
            beam(n1, spine[i])
            beam(tower, rail[s][i])
            beam(tower, spine[i])
        beam(n1, n2)
        beam(n1, tower)
        beam(n2, tower)
        for i in arch:                        # the tower to the arch and belt nodes of the side wall
            col = side_cols[s][i]
            beam(tower, col[-1])
            if len(col) > 2:
                beam(tower, col[1])
        beam(tower, node(wx, WHEEL_R + 0.34, -s * 0.56)) # across the car
        if front:
            rack = node(wx + 0.42, WHEEL_R, s * 0.34)
            for i in near:
                beam(rack, rail[s][i])
                beam(rack, spine[i])
            beam(rack, tower)
            hydros.append((n2, rack, 0.15 * s))
        else:
            for i in near:
                beam(n2, rail[s][i])
            beam(n2, spine[near[0]])
        arm = spine[near[0]]
        wheels.append((n1, n2, arm, front))

# ---- winding and cleanup
for i, (a, b, c) in enumerate(cabs):
    # outward winding: the normal of every triangle points away from the car's centre line
    pa, pb, pc = (nodes[k] for k in (a, b, c))
    n = ((pb[1] - pa[1]) * (pc[2] - pa[2]) - (pb[2] - pa[2]) * (pc[1] - pa[1]),
         (pb[2] - pa[2]) * (pc[0] - pa[0]) - (pb[0] - pa[0]) * (pc[2] - pa[2]),
         (pb[0] - pa[0]) * (pc[1] - pa[1]) - (pb[1] - pa[1]) * (pc[0] - pa[0]))
    cx = (pa[0] + pb[0] + pc[0]) / 3
    cy = (pa[1] + pb[1] + pc[1]) / 3 - 0.75
    cz = (pa[2] + pb[2] + pc[2]) / 3
    if n[0] * cx * 0.3 + n[1] * cy + n[2] * cz < 0:
        cabs[i] = (a, c, b)


def area2(t):
    pa, pb, pc = (nodes[k] for k in t)
    u = tuple(pb[i] - pa[i] for i in range(3))
    v = tuple(pc[i] - pa[i] for i in range(3))
    n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    return (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5


cabs = [t for t in cabs if len(set(t)) == 3 and area2(t) > 1e-4]
skin_nodes = {k for t in cabs for k in t}

# ---- write: the frame nodes first (minimass 5 kg: the cage's beams need it), then the skin-only nodes (0.3 kg:
# the sheet gives them their own; 1.5 kg stands for the glass, trim and doors the sheet does not carry)
order = sorted(frame_nodes) + sorted(set(range(len(nodes))) - frame_nodes)
remap = {old: new for new, old in enumerate(order)}
beams = [(remap[a], remap[b]) for a, b in beams]
cabs = [(remap[a], remap[b], remap[c]) for a, b, c in cabs]
hydros = [(remap[a], remap[b], fac) for a, b, fac in hydros]
wheels = [(remap[n1], remap[n2], remap[arm], front) for n1, n2, arm, front in wheels]
rail = {s: [remap[k] for k in rail[s]] for s in rail}
spine = [remap[k] for k in spine]
skin_nodes = {remap[k] for k in skin_nodes}
frame_nodes = {remap[k] for k in frame_nodes}
nodes = [nodes[old] for old in order]
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as f:
    f.write("Sheet Car\n")
    f.write(";generated by tools/make_sheet_car.py: a hatchback whose body panels are a sheet (triangle elements) on a deformable cage\n")
    f.write("globals\n;dry mass, cargo mass, cab material: sheet/<Material>[/kg per m2[/thickness]] makes the cab triangles a sheet body (BeamLab)\n800.0, 0.0, sheet/Steel/15.7/0.006\n")
    f.write("minimass\n5.0\n")
    f.write("nodes\n;id, x, y, z, options\n")
    for i, (x, y, z) in enumerate(nodes):
        if i == len(frame_nodes):
            f.write("set_default_minimass 1.5\n")
        opt = "l" if i in frame_nodes else "n"
        f.write("%d, %.3f, %.3f, %.3f, %s\n" % (i, x, y, z, opt))
    # the cage yields above 30 kN (plastic, keeps the bent shape) and never breaks
    f.write("beams\nset_beam_defaults 3000000, 300, 30000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0.0\n")
    for a, b in beams:
        f.write("%d, %d\n" % (a, b))
    f.write("hydros\n;tie rods: node1, node2, factor (steering lengthens or shortens the rod by this fraction), options\n")
    f.write("set_beam_defaults 15000000, 500, 99999999999999999999999999999999999999999, 99999999999999999999999999999999999999999, 0.02, tracks/beam, 0.0\n")
    for a, b, fac in hydros:
        f.write("%d, %d, %.2f, i\n" % (a, b, fac))
    f.write("wheels\n;radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, face, band\n")
    for n1, n2, arm, front in wheels:
        f.write("%.2f, 0.20, 12, %d, %d, 9999, %d, %d, %d, 45.0, 120000.0, 900.0, tracks/wheelface tracks/wheelband\n" % (WHEEL_R, n1, n2, 1, 1 if front else 0, arm))
    f.write("engine\n;min rpm, max rpm, torque, differential, reverse, neutral, gears...\n1000.0, 6000.0, 260.0, 4.1, 3.2, 1.0, 3.2, 2.0, 1.4, 1.0, 0.8, -1.0\n")
    f.write("engoption\n0.4, c, 200.0, 0.3, 0.4, 0.3\n")
    f.write("brakes\n3000\n")
    mid = j_of[round(-0.15, 3)]
    centre, back, left = spine[mid], spine[-1], rail[1][mid]
    f.write("cameras\n%d, %d, %d\n" % (centre, back, left))
    f.write("cinecam\n%.2f, 1.12, 0.35, %d, %d, %d, %d, %d, %d, %d, %d\n" % (0.25, spine[mid - 2], spine[mid + 2], rail[1][mid - 2], rail[1][mid + 2], rail[-1][mid - 2],
                                                                             rail[-1][mid + 2], spine[mid - 4], spine[mid + 4]))
    f.write("contacters\n")
    for i in sorted(skin_nodes | set(rail[1]) | set(rail[-1]) | set(spine)):
        f.write("%d\n" % i)
    f.write("submesh\ntexcoords\n")
    for i in sorted(skin_nodes):
        x, y, z = nodes[i]
        f.write("%d, %.3f, %.3f\n" % (i, (x + 2.1) / 4.2, (z + 0.8 + (y - FLOOR_Y)) / 3.0))
    f.write("cab\n")
    for a, b, c in cabs:
        f.write("%d, %d, %d, c\n" % (a, b, c))
    f.write("end\n")
print("wrote %s: %d nodes (%d frame, %d skin), %d beams, %d hydros, %d cab triangles, %d wheels" % (OUT, len(nodes), len(frame_nodes), len(skin_nodes), len(beams), len(hydros), len(cabs), len(wheels)))
