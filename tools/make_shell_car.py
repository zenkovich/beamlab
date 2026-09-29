#!/usr/bin/env python3
"""Writes assets/vehicles/shell_car/shell_car.truck: a hatchback whose body is a shell of FEM triangle elements.

No frame: the body is the structure (a monocoque), triangles of the FEM frame (phys::FrameTri: membrane and bending,
co-rotational, in the frame's implicit step; the truck's `fem_tris`) over the Sheet Car's outline - the top strip
(front face, bonnet, windscreen, roof, rear window, tailgate), the side walls with their wheel arches - and a floor pan
between the arches. Steel: 0.9 mm skin, a 1.5 mm floor, 2.5 mm round the suspension's mounts.

The suspension is the only thing that is not the shell: per wheel a trailing arm (beams from the hub to two pivots on
the floor), a coil-over up to a strut tower braced into the side wall and the floor; at the front the hub's outer node
turns about a kingpin on the arm and a tie rod (a hydro) to the floor steers it. Front-wheel drive, 260 N m.

Coordinates follow Rigs of Rods: -x forward, y up, +z left. The design is written with the front at +x and mirrored
when the nodes are made.

    python3 tools/make_shell_car.py
"""
import math
import os

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "vehicles", "shell_car", "shell_car.truck")

# the Sheet Car's outline (design space: +x forward): the profile in x-y, the rows across the top strip
DESIGN = [(2.12, 0.36), (2.12, 0.50), (1.98, 0.68), (1.80, 0.74), (1.45, 0.82), (1.05, 0.88), (0.80, 0.92),
          (0.62, 1.10), (0.42, 1.30), (0.25, 1.42),
          (-0.15, 1.46), (-0.60, 1.45), (-0.95, 1.36),
          (-1.25, 1.14), (-1.55, 0.95), (-1.85, 0.80),
          (-2.05, 0.60), (-2.05, 0.36)]            # (the bumpers' bottom edges at the floor: the faces join it)
Z_ROWS = [-0.78, -0.56, -0.29, 0.0, 0.29, 0.56, 0.78]
FLOOR_Y = 0.36
RAIL_Z = 0.56
WHEEL_X = (1.32, -1.30)
WHEEL_R = 0.32
ARCH_R = 0.46
T_SKIN, T_FLOOR, T_MOUNT = 0.0009, 0.0015, 0.0025
MOUNT_R = 0.18                  # triangles within this of a suspension mount: the thick section
ENGINE_KG, CREW_KG, TRIM_KG, DRY_KG = 150.0, 160.0, 120.0, 10.0
HUB_KG = 12.0                   # (the arms' nodes: beams of 3e6 N/m need their mass at the explicit step)

ARCH_XS = sorted({wx + d for wx in WHEEL_X for d in (-ARCH_R, -ARCH_R * 0.55, 0.0, ARCH_R * 0.55, ARCH_R)}, reverse=True)
PROFILE = []
for (x0, y0), (x1, y1) in zip(DESIGN, DESIGN[1:]):
    PROFILE.append((x0, y0))
    for ax in ARCH_XS:
        if x1 + 0.10 < ax < x0 - 0.10:
            PROFILE.append((ax, y0 + (y1 - y0) * (x0 - ax) / (x0 - x1)))
PROFILE.append(DESIGN[-1])


def width_scale(x):
    t = max(0.0, (abs(x) - 1.35) / 0.8)
    return 1.0 - 0.14 * min(1.0, t) ** 2


def shoulder_y(y, z):
    return FLOOR_Y + (y - FLOOR_Y) * (1.0 - 0.10 * (abs(z) / Z_ROWS[-1]) ** 4)


def sill_y(x):
    y = FLOOR_Y
    for wx in WHEEL_X:
        d = abs(x - wx)
        if d < ARCH_R - 1e-6:
            y = max(y, WHEEL_R + math.sqrt(ARCH_R * ARCH_R - d * d))
    return y


nodes = []           # (x, y, z) in RoR space
node_of = {}
load = {}            # node -> kg
tris = []            # (a, b, c, kind) kind: floor / skin
beams, beam_set = [], set()
shocks, hydros, wheels = [], [], []


def node(x, y, z):
    key = (round(x, 3), round(y, 3), round(z, 3))
    if key not in node_of:
        node_of[key] = len(nodes)
        nodes.append((-x, y, z))
    return node_of[key]


def beam(a, b):
    if a != b and (a, b) not in beam_set and (b, a) not in beam_set:
        beam_set.add((a, b))
        beams.append((a, b))


def quad(a, b, c, d, kind, flip):
    tris.extend([(a, b, c, kind), (a, c, d, kind)] if flip else [(a, b, d, kind), (b, c, d, kind)])


# ---- the top strip and the side walls (the Sheet Car's)
strip = [[node(x, shoulder_y(y, z), z * width_scale(x)) for (x, y) in PROFILE] for z in Z_ROWS]
for r in range(len(Z_ROWS) - 1):
    for i in range(len(PROFILE) - 1):
        quad(strip[r][i], strip[r][i + 1], strip[r + 1][i + 1], strip[r + 1][i], "skin", (r + i) % 2 == 0)
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
        cols.append([edge[j]] + [node(x, bot + h * f, zz) for f in fr] + ([node(x, bot, zz)] if h > 0.12 else []))
    side_cols[s] = cols
    for i in range(len(cols) - 1):
        ca, cb = cols[i], cols[i + 1]
        ya = [nodes[k][1] for k in ca]
        yb = [nodes[k][1] for k in cb]
        ta = [(ya[0] - y) / max(1e-6, ya[0] - ya[-1]) for y in ya]
        tb = [(yb[0] - y) / max(1e-6, yb[0] - yb[-1]) for y in yb]
        p = q = 0
        while p < len(ca) - 1 or q < len(cb) - 1:
            if q == len(cb) - 1 or (p < len(ca) - 1 and ta[p + 1] <= tb[q + 1]):
                tris.append((ca[p], cb[q], ca[p + 1], "skin"))
                p += 1
            else:
                tris.append((ca[p], cb[q], cb[q + 1], "skin"))
                q += 1

# ---- the floor pan: from bumper to bumper between the rails' lines, out to the sills where there is no wheel arch
FLOOR_Z = [-0.56, -0.29, 0.0, 0.29, 0.56]         # (the top strip's inner rows: its ends' bottom nodes are the floor's)
floor = [[node(x, FLOOR_Y, z * width_scale(x)) for (x, y) in PROFILE] for z in FLOOR_Z]
for r in range(len(FLOOR_Z) - 1):
    for i in range(len(PROFILE) - 1):
        quad(floor[r][i], floor[r][i + 1], floor[r + 1][i + 1], floor[r + 1][i], "floor", (r + i) % 2 == 1)
for s in (1, -1):
    rail = floor[-1] if s > 0 else floor[0]
    for i in range(len(PROFILE) - 1):
        a, b = side_cols[s][i][-1], side_cols[s][i + 1][-1]
        if abs(nodes[a][1] - FLOOR_Y) < 1e-3 and abs(nodes[b][1] - FLOOR_Y) < 1e-3:
            quad(rail[i], rail[i + 1], b, a, "floor", i % 2 == 0)

# ---- the suspension: per wheel a trailing arm on two floor pivots ahead of it (the rail line and the floor's middle
# row), the hub's inner node n1 and a knuckle k above it on the arm; a coil-over from n1 up to a strut tower braced
# into the side wall and the floor; the outer node n2 on n1 and k (the kingpin), a tie rod to the floor (front: a
# hydro, it steers; rear: a beam)
def nearest_floor(x, z):
    j = min(range(len(PROFILE)), key=lambda i: abs(PROFILE[i][0] - x))
    r = min(range(len(FLOOR_Z)), key=lambda i: abs(FLOOR_Z[i] - z))
    return floor[r][j]


mounts = set()
for wx in WHEEL_X:
    front = wx > 0
    for s in (1, -1):
        n1, n2 = node(wx, WHEEL_R, s * 0.62), node(wx, WHEEL_R, s * 0.86)
        k = node(wx, WHEEL_R + 0.22, s * 0.64)
        px = wx + (0.55 if not front else -0.55)        # (front: a leading arm from behind the wheel)
        p1, p2 = nearest_floor(px, s * RAIL_Z), nearest_floor(px, s * 0.29)
        for a in (n1, k):
            beam(a, p1), beam(a, p2)
        beam(n1, k)
        beam(n2, n1), beam(n2, k)
        # the strut tower: above the hub, braced to the side wall's belt and sill nodes and the floor round it
        tower = node(wx, WHEEL_R + 0.42, s * 0.62)
        arch = sorted(range(len(PROFILE)), key=lambda i: abs(PROFILE[i][0] - wx))[:5]
        for i in arch:
            col = side_cols[s][i]
            beam(tower, col[-1])
            if len(col) > 2:
                beam(tower, col[1])
        for dx in (-0.45, 0.45):
            beam(tower, nearest_floor(wx + dx, s * RAIL_Z))
            beam(tower, nearest_floor(wx + dx, s * 0.29))
        shocks.append((n1, tower, front))
        # the tie rod: from n2's side of the kingpin to a rack node on the floor ahead (front) or behind the axle
        tr = nearest_floor(wx + (0.45 if front else -0.45), s * 0.29)
        if front:
            hydros.append((n2, tr, 0.12 * s))
        else:
            beam(n2, tr)
        wheels.append((n1, n2, k, front))
        for v in (n1, n2, k):
            load[v] = HUB_KG
        load[tower] = HUB_KG      # (the tower hangs on nine beams of 3e6 N/m: 3 kg rang at the step)
        mounts.update((p1, p2, tr, tower))
        mounts.update(side_cols[s][i][-1] for i in arch)

# ---- the loads: the engine on the front floor, the crew and the trim over the floor between the axles
front_floor = {v for row in floor for v in row if nodes[v][0] < -0.9 and nodes[v][0] > -2.0}      # (RoR x = -design x)
cabin_floor = {v for row in floor for v in row if -0.9 <= nodes[v][0] <= 1.1}
for group, kg in ((front_floor, ENGINE_KG), (cabin_floor, CREW_KG + TRIM_KG)):
    for v in group:
        load[v] = load.get(v, 0.0) + kg / len(group)

# ---- the sections: the thick one round the mounts
def near_mount(t):
    for v in t[:3]:
        p = nodes[v]
        for m in mounts:
            q = nodes[m]
            if (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2 < MOUNT_R ** 2:
                return True
    return False


def area2(t):
    pa, pb, pc = (nodes[k] for k in t[:3])
    u = tuple(pb[i] - pa[i] for i in range(3))
    v = tuple(pc[i] - pa[i] for i in range(3))
    n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    return (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5


tris = [t for t in tris if len(set(t[:3])) == 3 and area2(t) > 1e-4]
groups = {"mount": [], "floor": [], "skin": []}
for t in tris:
    groups["mount" if near_mount(t) else t[3]].append(t[:3])
shell_nodes = {v for t in tris for v in t[:3]}

# the coil-overs: the corner's load on a spring of 1.4 Hz at the wheel, preloaded to stand at the design length; the
# stroke 0.12 m up, 0.10 down
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as f:
    f.write("Shell Car\n")
    f.write(";generated by tools/make_shell_car.py: a hatchback whose body is a shell of FEM triangle elements (no frame: the body is the\n")
    f.write(";structure), trailing arms and coil-overs on its floor and strut towers\n")
    f.write("globals\n;dry mass, cargo mass\n%.1f, 0.0\n" % DRY_KG)
    f.write("minimass\n2.0\n")
    f.write("nodes\n;id, x, y, z, options[, load kg]\n")
    for i, (x, y, z) in enumerate(nodes):
        if i in load:
            f.write("%d, %.3f, %.3f, %.3f, nl, %.2f\n" % (i, x, y, z, load[i]))
        else:
            f.write("%d, %.3f, %.3f, %.3f, n\n" % (i, x, y, z))
    f.write("beams\n;the suspension: arms, knuckles, towers (stiff, they yield past 60 kN)\n")
    f.write("set_beam_defaults 3000000, 1500, 60000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0.0\n")
    for a, b in beams:
        f.write("%d, %d\n" % (a, b))
    corner = {True: 2170.0, False: 1470.0}   # (N at the coil-over, measured standing: BL_SHOCKDBG, the spring's k (L - len))
    f.write("shocks\n;n1, n2, spring, damp, short bound, long bound, precompression, options\n")
    f.write("set_beam_defaults 600000, 8000, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for a, b, front in shocks:
        L0 = 0.42
        m = corner[front] / 9.81
        k = m * (2 * math.pi * 1.4) ** 2
        c = 2 * 0.35 * math.sqrt(k * m)
        pre = 1 + corner[front] / (k * L0)
        L = L0 * pre
        f.write("%d, %d, %.0f, %.0f, %.3f, %.3f, %.3f, n\n" % (a, b, k, c, 1 - (L0 - 0.12) / L, max(0.0, (L0 + 0.10) / L - 1), pre))
    f.write("hydros\n;tie rods: node1, node2, factor, options\n")
    f.write("set_beam_defaults 6000000, 800, 99999999999999999999999999999999999999999, 99999999999999999999999999999999999999999, 0.02, tracks/beam, 0.0\n")
    for a, b, fac in hydros:
        f.write("%d, %d, %.2f, i\n" % (a, b, fac))
    f.write("wheels\n;radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, face, band\n")
    f.write("set_beam_defaults 3000000, 400, 400000, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0\n")
    for n1, n2, arm, front in wheels:
        f.write("%.2f, 0.20, 16, %d, %d, 9999, 1, %d, %d, 40.0, 110000.0, 900.0, tracks/wheelface tracks/wheelband\n" % (WHEEL_R, n1, n2, 1 if front else 0, arm))
    f.write("engine\n;min rpm, max rpm, torque, differential, reverse, neutral, gears...\n1000.0, 6000.0, 260.0, 4.1, 3.2, 1.0, 3.2, 2.0, 1.4, 1.0, 0.8, -1.0\n")
    f.write("engoption\n0.10, c, 1000.0, 0.3, 0.4, 0.3\n")
    f.write("brakes\n3000\n")
    mid = min(range(len(PROFILE)), key=lambda i: abs(PROFILE[i][0] + 0.15))
    centre, back, left = floor[2][mid], floor[2][-1], floor[-1][mid]
    f.write("cameras\n%d, %d, %d\n" % (centre, back, left))
    f.write("cinecam\n%.2f, 1.12, 0.35, %d, %d, %d, %d, %d, %d, %d, %d\n" % (0.25, floor[2][mid - 2], floor[2][mid + 2], floor[-1][mid - 2], floor[-1][mid + 2],
                                                                            floor[0][mid - 2], floor[0][mid + 2], floor[1][mid], floor[3][mid]))
    f.write("fem_tris\n;the body: triangle elements of the FEM frame (n1, n2, n3) of the shell set before them\n")
    for name, thick, col in (("skin", T_SKIN, (0.80, 0.14, 0.10)), ("floor", T_FLOOR, (0.80, 0.14, 0.10)), ("mount", T_MOUNT, (0.80, 0.14, 0.10))):
        if not groups[name]:
            continue
        f.write("set_fem_shell Steel, %.4f, %.2f, %.2f, %.2f\n" % (thick, *col))
        for a, b, c in groups[name]:
            f.write("%d, %d, %d\n" % (a, b, c))
    f.write("end\n")
print("wrote %s: %d nodes (%d of the shell), %d triangles (%d skin, %d floor, %d round the mounts), %d beams, %d shocks, %d wheels" % (
    OUT, len(nodes), len(shell_nodes), len(tris), len(groups["skin"]), len(groups["floor"]), len(groups["mount"]), len(beams), len(shocks), len(wheels)))
