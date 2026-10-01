"""The machinery the FEM vehicles' generators share (tools/make_frame_car.py, tools/make_buggy.py): a vehicle's nodes in
design space (+x forward, y up, z left; written mirrored in x, as Rigs of Rods has -x forward), its panels (triangle
elements, each with nodes of its own), its frame (tubes divided into FEM members, parts that are components of their
own), the welds that hold the panels on the tubes, the mounts that hold the parts on the body, lamps, the collision hull,
and the .truck writer.

A generator imports it (`from fem_car import *`), sets its constants (configure), lays out its panels, tubes and extra
split points, then calls divide_tubes, adds its members (suspension, parts' attachments), weld_panels, offset_panels,
mount_lamps, its hull, and write. The containers are the module's own and change in place: a name imported from here
stays the same list or dict.
"""
import math
import os

# ---- the settings (configure): the panels' cell, the seam between a tube's frame nodes, the wheel arches, the gap of
# the panels off their tubes, the welds, the lamps' mounts, the panels' refinement; `centre_y`: the height the panels'
# and the hull's outward faces point away from (with x scaled by 0.3: a long body)
CFG = dict(centre_y=0.65, grid=0.13, seam=0.20, arch_r=0.38, arch_y=0.30, gap=0.035, weld_r=0.17, weld_brk=2500.0, glass_weld_k=10000.0,
           lamp_brk=2500.0, lamp_reach=0.6, refine=1)


def configure(**kw):
    for k, v in kw.items():
        assert k in CFG, k
        CFG[k] = v


# -------------------------------------------------------------------------------------------------------- the nodes
nodes = []          # design space (+x forward)
node_kind = []      # "fem", "panel", "plain"
node_mm = []        # the node's minimum mass group: None (the global), or kg
fem_at = {}         # (part, rounded point) -> frame node
node_part = {}      # frame node -> its part: "body", "hood", "fender 1", "door 1 front", "bumper front", ...
node_centre = {}    # a node -> the middle its faces point away from (a bumper's profile), else the body's
heavy = {}          # node -> load weight (kg, the truck's `nl`)


def v_add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def v_sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def v_mul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def v_dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def v_cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def v_len(a): return math.sqrt(v_dot(a, a))
def v_norm(a): l = v_len(a); return v_mul(a, 1.0 / l) if l > 0 else a
def lerp(p, q, t): return tuple(p[i] + (q[i] - p[i]) * t for i in range(3))
def mir(p, s): return (p[0], p[1], p[2] * s)


def rot(p, c, axis, deg):
    """p turned about the line through c along the unit `axis` by deg"""
    a = math.radians(deg)
    v = v_sub(p, c)
    r = v_add(v_add(v_mul(v, math.cos(a)), v_mul(v_cross(axis, v), math.sin(a))), v_mul(axis, v_dot(axis, v) * (1 - math.cos(a))))
    return v_add(c, r)


def new_node(p, kind, mm=None):
    nodes.append(tuple(p))
    node_kind.append(kind)
    node_mm.append(mm)
    return len(nodes) - 1


def fem_node(p, part="body"):
    key = (round(p[0], 3), round(p[1], 3), round(p[2], 3))
    if (part, key) not in fem_at:
        k = new_node(key, "fem")
        fem_at[(part, key)] = k
        node_part[k] = part
    return fem_at[(part, key)]


def away(p):
    """the way out of the body at p (from its long axis at centre_y)"""
    return (p[0] * 0.3, p[1] - CFG["centre_y"], p[2])


def outward(p, n):
    """n turned to face away from the car's middle"""
    return n if v_dot(n, away(p)) >= 0 else v_mul(n, -1)


# ------------------------------------------------------------------------------------------------------- the panels
class Panel:
    def __init__(self, name, mat, part):
        self.name, self.mat, self.part = name, mat, part
        self.tris = []     # node ids
        self.nodes = []    # its node ids


panels = []


def quad_panel(name, mat, p00, p10, p01, p11, nu, nv, dome=0.0, bow=0.0, hole=None, part="body"):
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
    pan = Panel(name, mat, part)
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
        R = hole.radius
        cellw = v_len(v_sub(p10, p00)) / nu
        for i in range(nu + 1):
            for j in range(nv + 1):
                k = ids[i][j]
                if k is None:
                    continue
                x, y, z = nodes[k]
                d = math.hypot(x - cx, y - cy)
                if d < R + 0.35 * cellw and d > 1e-6 and y > cy - 0.01:
                    nodes[k] = (cx + (x - cx) * R / d, cy + (y - cy) * R / d, z)
                    P[i][j] = nodes[k]
    panels.append(pan)
    return pan, [[P[i][j] if ids[i][j] is not None else None for j in range(nv + 1)] for i in range(nu + 1)]


def grid_panel(name, mat, P, part):
    """A panel on the grid of points P[i][j] (every cell kept)"""
    nu, nv = len(P) - 1, len(P[0]) - 1
    pan = Panel(name, mat, part)
    ids = [[new_node(P[i][j], "panel") for j in range(nv + 1)] for i in range(nu + 1)]
    pan.nodes = [k for r in ids for k in r]
    for i in range(nu):
        for j in range(nv):
            a, b, c, d = ids[i][j], ids[i + 1][j], ids[i + 1][j + 1], ids[i][j + 1]
            pan.tris += [(a, b, c), (a, c, d)] if (i + j) % 2 == 0 else [(a, b, d), (b, c, d)]
    panels.append(pan)
    return pan, ids


def tri_panel(name, mat, A, B, C, n, dome=0.0, part="body"):
    """A triangular panel A B C cut into n x n triangles (a rear quarter window)"""
    pan = Panel(name, mat, part)
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


def thin(pts, keep=()):
    """a curved line's points every other one (and `keep`, the ends): the ones between welded to its members"""
    return [p for i, p in enumerate(pts) if i % 2 == 0 or i == len(pts) - 1 or i in keep]


def ends(pts):
    """a straight line's ends: divided at its panel's nodes SEAM apart, the ones between welded on its members"""
    return [pts[0], pts[-1]]


def runs(pts):
    """a grid line's stretches without a gap (a hole's cells cut it)"""
    out, cur = [], []
    for p in pts + [None]:
        if p is None:
            if len(cur) > 1:
                out.append(cur)
            cur = []
        else:
            cur.append(p)
    return out


def arch(ax, r=None, y=None):
    """a wheel arch round (ax, y) of radius r: a hole for quad_panel"""
    r = CFG["arch_r"] if r is None else r
    y = CFG["arch_y"] if y is None else y
    f = lambda c: (c[0] - ax) ** 2 + (c[1] - y) ** 2 < r ** 2
    f.centre = (ax, y)
    f.radius = r
    return f


def n_cells(a, b):
    return max(1, int(round(v_len(v_sub(a, b)) / CFG["grid"])))


# ------------------------------------------------------------------------------------------ the frame (tubes, members)
members = []        # (a, b, section, joint at a or None, joint at b or None)
member_set = set()
tubes = []          # (polyline, section, part)
extra_splits = []   # (point, part): points the tubes are divided at besides the panels' (hinges, latches, mounts)
late = []           # members joined to the nearest frame nodes once the tubes are divided: (p, q, section, joint a, joint b)
seg_members = {}    # part -> [(a, b)]: the tubes' members, for the welds between their nodes
tube_chain = []     # each tube's frame nodes in order
fem_ids = []        # the frame nodes (divide_tubes)
body_ids = []       # the body's
cell = {}           # (a grid of the frame nodes, 5 cm)
mounts = []         # (body node, part node, kind): the parts on the body (see MOUNTS in a generator)
volumes = []        # (name, anchors, hull points (design space), break rms): the collision volumes (`collision_volumes`)
latches = []        # (node, anchor, strength)
straps = []         # (node, anchor, short bound, long bound)
stats = {"on_member": 0}


def tube(points, sec, part="body"):
    tubes.append(([tuple(p) for p in points], sec, part))
    return len(tubes) - 1


def split_at(p, part="body"):
    extra_splits.append((tuple(p), part))


def member(a, b, sec, ja=None, jb=None):
    if a == b or (min(a, b), max(a, b)) in member_set:
        return
    member_set.add((min(a, b), max(a, b)))
    members.append((a, b, sec, ja, jb))


def on_segment(p, a, b, tol=0.002):
    ab = v_sub(b, a)
    l2 = v_dot(ab, ab)
    t = v_dot(v_sub(p, a), ab) / l2
    if t <= 1e-4 or t >= 1 - 1e-4:
        return None
    q = v_add(a, v_mul(ab, t))
    return t if v_len(v_sub(p, q)) < tol else None


def divide_tubes():
    """every tube divided at its part's other tubes' ends and extra points, and at its part's panels' nodes on it no
    closer than SEAM apart (about every other one: a frame node under it, the same point); the panel's nodes between
    are welded to the tube's point under them. Then the frame nodes' grid, and the `late` members"""
    seam = CFG["seam"]
    panel_pts, key_pts = {}, {}
    for pan in panels:
        panel_pts.setdefault(pan.part, []).extend(nodes[k] for k in pan.nodes)
    for pts, _, part in tubes:
        key_pts.setdefault(part, []).extend(pts)
    for p, part in extra_splits:
        key_pts.setdefault(part, []).append(p)
    for pts, sec, part in tubes:
        whole = []
        for a, b in zip(pts, pts[1:]):
            L = v_len(v_sub(b, a))
            keys = sorted({round(t, 6) for p in key_pts[part] for t in [on_segment(p, a, b)] if t is not None} | {0.0, 1.0})
            pans = sorted({round(t, 6) for p in panel_pts.get(part, []) for t in [on_segment(p, a, b)] if t is not None} - set(keys))
            ts = list(keys)
            for t in pans:   # (the panel's nodes kept as far as they are from the ones kept and the fixed ones)
                if all(abs(t - u) * L >= seam * 0.999 for u in ts):
                    ts.append(t)
            ts = sorted(ts)
            chain = [fem_node(lerp(a, b, t), part) for t in ts]
            for m, n in zip(chain, chain[1:]):
                member(m, n, sec)
                seg_members.setdefault(part, []).append((m, n))
            whole += chain if not whole else chain[1:]
        tube_chain.append(whole)
    fem_ids[:] = [k for k, kind in enumerate(node_kind) if kind == "fem"]
    body_ids[:] = [k for k in fem_ids if node_part[k] == "body"]
    cell.clear()
    for k in fem_ids:
        cell.setdefault(tuple(int(math.floor(c / 0.05)) for c in nodes[k]), []).append(k)
    for p, q, sec, ja, jb in late:
        member(fem_get(p), fem_get(q), sec, ja, jb)


def fem_near(p, r, part="body"):
    """the part's frame node nearest p within r (None: none)"""
    c = tuple(int(math.floor(x / 0.05)) for x in p)
    best, bd = None, r
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                for k in cell.get((c[0] + dx, c[1] + dy, c[2] + dz), []):
                    d = v_len(v_sub(nodes[k], p))
                    if d < bd and node_part[k] == part:
                        best, bd = k, d
    return best


def body_nodes_on(polylines, box=None):
    """the body's frame nodes on these polylines (their ends too), within box ((x0, x1), (y0, y1)) if given: a
    collision volume's anchors"""
    out = set()
    for k in body_ids:
        p = nodes[k]
        if box and not (box[0][0] - 1e-6 <= p[0] <= box[0][1] + 1e-6 and box[1][0] - 1e-6 <= p[1] <= box[1][1] + 1e-6):
            continue
        for pl in polylines:
            if any(v_len(v_sub(p, q)) < 0.002 for q in pl) or any(on_segment(p, a, b) is not None for a, b in zip(pl, pl[1:])):
                out.add(k)
                break
    return sorted(out)


def volume(name, anchors, points, rms=0.12, color=None):
    """a collision volume (phys::CollisionVolume): the convex hull of the points (design space, both sides given)
    riding on the anchors (the body's frame nodes); color: drawn as a solid of it (an engine block)"""
    assert len(anchors) >= 3, (name, len(anchors))
    volumes.append((name, sorted(set(anchors)), points, rms, color))


def body_near(box):
    """the body's frame nodes in the box ((x0, x1), (y0, y1), (z0, z1)): a zone's anchors"""
    return sorted(k for k in body_ids if all(box[a][0] - 1e-6 <= nodes[k][a] <= box[a][1] + 1e-6 for a in range(3)))


def hull_planes(pts):
    """the convex hull's faces of these points as planes (n, d: inside n . x <= d), brute force as phys::convex_hull"""
    out, n = [], len(pts)
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                nn = v_cross(v_sub(pts[j], pts[i]), v_sub(pts[k], pts[i]))
                if v_len(nn) < 1e-9:
                    continue
                nn = v_norm(nn)
                d = v_dot(nn, pts[i])
                s = [v_dot(nn, q) - d for q in pts]
                if max(s) > 1e-5 and min(s) < -1e-5:
                    continue
                if max(s) > 1e-5:
                    nn, d = v_mul(nn, -1.0), -d
                if not any(v_dot(nn, m) > 0.9999 and abs(d - e) < 1e-4 for m, e in out):
                    out.append((nn, d))
    return out


def volume_clearances():
    """each collision volume's clearance to the parts (their frames' and their skins' nodes): [(name, {part: the
    least distance of its nodes outside the hull, negative: inside})]"""
    part_of = {k: p for k, p in node_part.items() if p != "body"}
    for pan in panels:
        if pan.part != "body":
            for k in pan.nodes:
                part_of[k] = pan.part
    res = []
    for name, an, pts, rms, col in volumes:
        planes = hull_planes(pts)
        gap = {}
        for k, part in part_of.items():
            g = max(v_dot(nn, nodes[k]) - d for nn, d in planes)
            gap[part] = min(gap.get(part, 1e9), g)
        res.append((name, gap))
    return res


def fem_get(p, r=0.07, part="body"):
    k = fem_near(p, r, part)
    assert k is not None, "no frame node of the %s near %s" % (part, p)
    return k


def nearest_of(part, p, r):
    """the part's frame node nearest p within r (farther than the grid's reach)"""
    k = min((k for k in fem_ids if node_part[k] == part), key=lambda k: v_len(v_sub(nodes[k], p)))
    assert v_len(v_sub(nodes[k], p)) < r, "no frame node of the %s within %.2f of %s" % (part, r, p)
    return k


def stay(p, part, q, c, axis, deg, short=0.9):
    """a strap from the part's p to the body's q that lets the part open by deg about the line (c, axis)"""
    L0, L1 = v_len(v_sub(p, q)), v_len(v_sub(rot(p, c, axis, deg), q))
    straps.append((fem_get(p, 0.002, part), fem_get(q, 0.002), short, L1 / L0 - 1.0))


# ------------------------------------------------------------------------------------------------------- the welds
welds = []
welded = set()


def weld_panels():
    """every panel node on a frame node of its part (the same point) or on a tube of it between its nodes"""
    for pan in panels:
        for k in pan.nodes:
            f = fem_near(nodes[k], 0.002, pan.part)
            if f is not None:
                welds.append((f, k))
                welded.add(k)
                continue
            for m, n in seg_members.get(pan.part, []):   # (on a tube between its nodes: held at that point of it)
                t = on_segment(nodes[k], nodes[m], nodes[n])
                if t is not None:
                    welds.append((m, k, n, t))
                    welded.add(k)
                    stats["on_member"] += 1
                    break


def weld_near(pan, maxd, segs=None):
    """the panel's nodes not welded yet held at the nearest point of a member of its part (or of `segs`) within maxd"""
    segs = seg_members.get(pan.part, []) if segs is None else segs
    for k in pan.nodes:
        if k in welded:
            continue
        p, best = nodes[k], None
        for m, n in segs:
            a, ab = nodes[m], v_sub(nodes[n], nodes[m])
            t = min(1.0, max(0.0, v_dot(v_sub(p, a), ab) / v_dot(ab, ab)))
            dd = v_len(v_sub(p, v_add(a, v_mul(ab, t))))
            if best is None or dd < best[0]:
                best = (dd, m, n, t)
        if best is not None and best[0] < maxd:
            welds.append((best[1], k, best[2], best[3]))
            welded.add(k)
            stats["on_member"] += 1


def weld_to_chain(pan, chain, maxd, dist):
    """the panel's nodes not welded yet within maxd of the tube's chain (by `dist(p, q)`, p the node, q its foot on the
    tube) held at the tube's point under them"""
    for k in pan.nodes:
        if k in welded:
            continue
        p, best = nodes[k], None
        for m, n in zip(chain, chain[1:]):
            a, ab = nodes[m], v_sub(nodes[n], nodes[m])
            t = v_dot(v_sub(p, a), ab) / v_dot(ab, ab)
            if 0.0 <= t <= 1.0:
                d = dist(p, v_add(a, v_mul(ab, t)))
                if best is None or d < best[0]:
                    best = (d, m, n, t)
        if best is not None and best[0] < maxd:
            welds.append((best[1], k, best[2], best[3]))
            welded.add(k)
            stats["on_member"] += 1


# ------------------------------------------------------------------------------------------------ the lights and boxes
lamps = []   # dict(ids, tris, mat, attach [(node index, anchor point)] or None, mounts: frame nodes of its own)


def lamp(mat, face, back, attach, mm=0.6):
    """a little closed box: face its 4 corners round, back the point behind it"""
    ids = [new_node(p, "plain", mm) for p in face + [back]]
    a, b, c, d, e = ids
    lamps.append(dict(ids=ids, tris=[(a, b, c), (a, c, d), (a, e, b), (b, e, c), (c, e, d), (d, e, a)], mat=mat, attach=attach, mounts=None))
    return lamps[-1]


def out_of(t):
    """the way out of the car at triangle t (out of its own profile where its nodes have a centre)"""
    mid = v_mul(v_add(v_add(nodes[t[0]], nodes[t[1]]), nodes[t[2]]), 1.0 / 3.0)
    if all(k in node_centre for k in t):
        return v_sub(mid, v_mul(v_add(v_add(node_centre[t[0]], node_centre[t[1]]), node_centre[t[2]]), 1.0 / 3.0))
    return away(mid)


def offset_panels(no_gap=lambda pan: pan.mat == "glass"):
    """each panel node moved `gap` out along its panel's normal there (not the panels no_gap names); the welds, found
    where the panels met the tubes, hold them off by that much. The lamps with attach points go out with the panel
    under them"""
    gap = CFG["gap"]
    normal_at = {}
    skip = set(filter(None, os.environ.get("FC_GAP_SKIP", "").split(",")))   # (experiments: panels left on their tubes)
    for pan in panels:
        if no_gap(pan) or pan.name in skip:
            continue
        for t in pan.tris:
            pa, pb, pc = (nodes[k] for k in t)
            n = v_cross(v_sub(pb, pa), v_sub(pc, pa))
            n = n if v_dot(n, out_of(t)) >= 0 else v_mul(n, -1)
            for k in t:
                normal_at[k] = v_add(normal_at.get(k, (0.0, 0.0, 0.0)), n)
    normal_at = {k: v_norm(n) for k, n in normal_at.items()}
    for k, n in normal_at.items():
        nodes[k] = v_add(nodes[k], v_mul(n, gap))
    for lp in lamps:
        if lp["attach"] is None:
            continue
        ids = lp["ids"]
        face = v_mul(v_add(v_add(nodes[ids[0]], nodes[ids[1]]), v_add(nodes[ids[2]], nodes[ids[3]])), 0.25)
        under = min(normal_at, key=lambda k: v_len(v_sub(nodes[k], face)))
        for k in ids:
            nodes[k] = v_add(nodes[k], v_mul(normal_at[under], gap))


def dist_line(p, a, b):
    ab = v_norm(v_sub(b, a))
    ap = v_sub(p, a)
    return v_len(v_sub(ap, v_mul(ab, v_dot(ap, ab))))


def dist_plane(p, a, b, c):
    return abs(v_dot(v_sub(p, a), v_norm(v_cross(v_sub(b, a), v_sub(c, a)))))


lamp_beams, lamp_mounts = [], []   # (a, b); (a, b, strength)


def mount_lamps():
    """the lamps' boxes' beams, and every node's beams to four body nodes round the box (the nearest, then the nearest
    off the first two's line, then the nearest off their plane); those with frame nodes of their own (a mirror's arm)
    to those"""
    for lp in lamps:
        ids = lp["ids"]
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                lamp_beams.append((ids[i], ids[j]))
        if lp["mounts"] is not None:
            lamp_mounts.extend((k, f, 1e9) for k in ids for f in lp["mounts"])
            continue
        ctr = v_mul(v_add(v_add(v_add(nodes[ids[0]], nodes[ids[1]]), v_add(nodes[ids[2]], nodes[ids[3]])), nodes[ids[4]]), 0.2)
        near = sorted(body_ids, key=lambda k: v_len(v_sub(nodes[k], ctr)))
        m = [near[0]]
        m.append(next(k for k in near if v_len(v_sub(nodes[k], nodes[m[0]])) > 0.06))
        m.append(next(k for k in near if dist_line(nodes[k], nodes[m[0]], nodes[m[1]]) > 0.05))
        m.append(next(k for k in near if dist_plane(nodes[k], nodes[m[0]], nodes[m[1]], nodes[m[2]]) > 0.04))
        assert max(v_len(v_sub(nodes[k], ctr)) for k in m) < CFG["lamp_reach"], "a lamp's mounts are far off"
        lamp_mounts.extend((k, f, CFG["lamp_brk"]) for k in ids for f in m)


# ---------------------------------------------------------------------------------------------- the collision hull
hull = []   # (a, b, c) frame nodes, facing out


def hull_quad(p00, p10, p11, p01, part="body"):
    a, b, c, d = (fem_get(p, 0.07, part) for p in (p00, p10, p11, p01))
    hull.extend([(a, b, c), (a, c, d)])


def hull_tri(p0, p1, p2, part2="body"):
    hull.append((fem_get(p0), fem_get(p1), fem_get(p2, 0.07, part2)))


def orient_hull():
    for i, (a, b, c) in enumerate(hull):   # (facing out of the car)
        pa, pb, pc = nodes[a], nodes[b], nodes[c]
        mid = v_mul(v_add(v_add(pa, pb), pc), 1.0 / 3.0)
        if v_dot(v_cross(v_sub(pb, pa), v_sub(pc, pa)), away(mid)) < 0:
            hull[i] = (a, c, b)
    assert len(set(hull)) == len(hull) and all(len(set(t)) == 3 for t in hull), "a degenerate hull triangle"


def cab_tris():
    """the panels' and the lamps' triangles, facing out: [(a, b, c, material)]"""
    all_tris = []
    for pan in panels:
        for t in pan.tris:
            all_tris.append(t + (pan.mat,))
    for lp in lamps:
        for t in lp["tris"]:
            all_tris.append(t + (lp["mat"],))
    lamp_of = {x: lp["ids"] for lp in lamps for x in lp["ids"]}
    for i, (a, b, c, m) in enumerate(all_tris):
        pa, pb, pc = nodes[a], nodes[b], nodes[c]
        n = v_cross(v_sub(pb, pa), v_sub(pc, pa))
        mid = v_mul(v_add(v_add(pa, pb), pc), 1.0 / 3.0)
        if a in lamp_of:   # (a lamp's faces: out of its own box)
            ls = lamp_of[a]
            ctr = v_mul(v_add(v_mul(v_add(v_add(nodes[ls[0]], nodes[ls[1]]), v_add(nodes[ls[2]], nodes[ls[3]])), 0.25), nodes[ls[4]]), 0.5)
            out = v_sub(mid, ctr)
        else:
            out = out_of((a, b, c))
        if v_dot(n, out) < 0:
            all_tris[i] = (a, c, b, m)
    return all_tris


# ---------------------------------------------------------------------------------------------------------- write
def spec(sections, sec):
    m, sh, o, w, j = sections[sec][:5]
    extra = sections[sec][5:]
    s = "%s, %s, %.4f, %.4f, %s" % (m, sh, o, w, j)
    if extra:   # (joint stiffness 0: the default)
        s += ", 0, %.0f, %.1f" % (extra[0], extra[1] if len(extra) > 1 else 0.0)
    return s


def P(k):  # RoR space
    x, y, z = nodes[k]
    return (-x, y, z)


def write(path, v):
    """the .truck. `v`: title, head (comment lines after the title), globals (dry kg, cab material), sections, mat,
    mount_kinds, latch_note; shocks_text (the shocks section's lines), hydros, slides (node, rail nodes), wheels_text,
    engine_text (engine, engoption, brakes...), cameras (3 nodes), cinecam (point, nodes)"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    sections, MAT = v["sections"], v["mat"]
    all_tris = cab_tris()
    with open(path, "w") as f:
        f.write(v["title"] + "\n")
        for line in v["head"]:
            f.write(";" + line + "\n")
        f.write("globals\n;dry mass, cargo mass, cab material (sheet/<material>/<kg per m2>/<drawn thickness>: the panels are a sheet body)\n")
        f.write("%.1f, 0.0, %s\n" % v["globals"])
        f.write("minimass\n0.05\n")
        f.write("nodes\n;id, x, y, z, options (the frame's nodes, the panels', the lamps', the rack's)\n")
        mm = None
        for i in range(len(nodes)):
            if node_mm[i] != mm:
                mm = node_mm[i]
                f.write("set_default_minimass %s\n" % ("-1" if mm is None else "%.2f" % mm))
            x, y, z = P(i)
            if i in heavy:
                f.write("%d, %.4f, %.4f, %.4f, nl, %.2f\n" % (i, x, y, z, heavy[i]))
            else:
                f.write("%d, %.4f, %.4f, %.4f, n\n" % (i, x, y, z))
        f.write("beams\n")
        by_sec = {}
        for a, b, sec, ja, jb in members:
            by_sec.setdefault(sec, []).append((a, b, ja, jb))
        for sec, lst in by_sec.items():
            f.write(";%s\nset_beam_defaults 3000000, 400, 80000, 700000, 0.05, tracks/beam, 0\nset_frame_section %s\n" % (sec, spec(sections, sec)))
            default = sections[sec][4]
            for a, b, ja, jb in lst:
                if ja or jb:
                    f.write("%d, %d, F, %s, %s\n" % (a, b, ja or default, jb or default))
                else:
                    f.write("%d, %d, F\n" % (a, b))
        if latches:
            f.write(v["latch_note"] + "\n")
            # (a spring between two components of the frame, explicit: 1e6 N/m on a lid's light latch node shook it loose as the
            # car stood)
            for brk in sorted({l[2] for l in latches}):
                f.write("set_beam_defaults 300000, 300, %.0f, %.0f, 0.02, tracks/beam, 0\n" % (brk, brk))
                for a, b, lb in latches:
                    if lb == brk:
                        f.write("%d, %d, i\n" % (a, b))
        f.write(";the lamps' and the mirrors' boxes, and their mounts (a lamp's to four frame nodes, tearing at %.0f N; a mirror's to its arm)\n" % CFG["lamp_brk"])
        f.write("set_beam_defaults 150000, 60, 1000000000, 1000000000, 0.01, tracks/beam, 0\n")
        for a, b in lamp_beams:
            f.write("%d, %d, i\n" % (a, b))
        for brk in sorted({l[2] for l in lamp_mounts}):
            f.write("set_beam_defaults 150000, 60, %.0f, %.0f, 0.01, tracks/beam, 0\n" % (brk, brk))
            for a, b, lb in lamp_mounts:
                if lb == brk:
                    f.write("%d, %d, i\n" % (a, b))
        f.write(v["shocks_text"])
        f.write("hydros\n;the steering rack: node1, node2, factor, options\n")
        f.write("set_beam_defaults 4000000, 2000, 99999999999999999999999999999999999999999, 99999999999999999999999999999999999999999, 0.02, tracks/beam, 0\n")
        for a, b, fac in v["hydros"]:
            f.write("%d, %d, %.4f, i\n" % (a, b, fac))
        f.write("slidenodes\n;the rack's ends slide along its housing\n")
        for n, housing in v["slides"]:
            f.write("%d, %s, S2000000, T0\n" % (n, ", ".join(str(h) for h in housing)))
        f.write(v["wheels_text"])
        f.write(v["engine_text"])
        f.write("cameras\n%d, %d, %d\n" % v["cameras"])
        (x, y, z), near = v["cinecam"]
        f.write("cinecam\n%.3f, %.3f, %.3f, %s\n" % (-x, y, z, ", ".join(str(k) for k in near)))
        f.write("contacters\n")
        for i in range(len(nodes)):
            f.write("%d\n" % i)
        if volumes:
            f.write("collision_volumes\n;(BeamLab) volume name, break rms (m) - its anchors (frame nodes) - its hull's points (x, y, z)\n")
            for name, an, pts, rms, col in volumes:
                f.write("volume %s, %.2f\nanchors %s\n" % (name, rms, ", ".join(str(a) for a in an)))
                for x, y, z in pts:
                    f.write("vertex %.3f, %.3f, %.3f\n" % (-x, y, z))
                if col:
                    f.write("color %.2f, %.2f, %.2f\n" % col)
        f.write("welds\n;anchor (a frame node), sheet node, radius m, strength N, stiffness N/m (0: the step's), anchor2, t (a point on a member)\n")
        mat_of = {k: pan.mat for pan in panels for k in pan.nodes}
        for w in welds:
            k = CFG["glass_weld_k"] if mat_of.get(w[1]) == "glass" else 0.0
            if len(w) == 2:
                f.write("%d, %d, %.3f, %.0f, %.0f\n" % (w[0], w[1], CFG["weld_r"], CFG["weld_brk"], k))
            else:
                f.write("%d, %d, %.3f, %.0f, %.0f, %d, %.4f\n" % (w[0], w[1], CFG["weld_r"], CFG["weld_brk"], k, w[2], w[3]))
        f.write("mounts\n;node a (the body's), node b (the part's), break force N, stiffness N/m, turning damping N m s/rad: b held where it stands off a\n")
        for a, b, kind in mounts:
            brk, k, damp, _ = v["mount_kinds"][kind]
            f.write("%d, %d, %.0f, %.0f, %.1f\n" % (a, b, brk, k, damp))
        f.write("submesh\ntexcoords\n")
        used = sorted({k for t in all_tris for k in t[:3]})
        span = v.get("uv_span", (2.1, 4.2, 1.0, 3.0, CFG["arch_y"]))
        for k in used:
            x, y, z = nodes[k]
            f.write("%d, %.3f, %.3f\n" % (k, (x + span[0]) / span[1], (z + span[2] + (y - span[4])) / span[3]))
        f.write("cab\n")
        for a, b, c, m in all_tris:
            f.write("%d, %d, %d, c\n" % (a, b, c))
        f.write(";the frame's collision hull (option h: one-sided and solid, facing out; not in shells, not drawn)\n")
        for a, b, c in hull:   # (written mirrored in x: the winding turns, so a, c, b faces out in RoR space)
            f.write("%d, %d, %d, ch\n" % (a, c, b))
        f.write("shells\n")
        cur = None
        for a, b, c, m in sorted(all_tris, key=lambda t: list(MAT).index(t[3])):
            if m != cur:
                cur = m
                if MAT[m] is None:
                    f.write("set_shell_material default\n")
                else:
                    mt, kg, th, col_ = MAT[m]
                    f.write("set_shell_material %s, %s, %.1f, %.3f, %.2f, %.2f, %.2f, %d\n" % (m, mt, kg, th, col_[0], col_[1], col_[2], CFG["refine"]))
            f.write("%d, %d, %d\n" % (a, b, c))
        f.write("end\n")
    return all_tris


def summary(path, all_tris, doors=0):
    n_fem = len([k for k, kind in enumerate(node_kind) if kind == "fem"])
    parts = {}
    for k, p in node_part.items():
        parts[p] = parts.get(p, 0) + 1
    print("wrote %s: %d nodes (%d frame, %d on panels), %d frame elements, %d triangles in %d panels, %d welds (%d on a member, %.0f N), %d lamps (%d mount beams), %d doors" % (
        path, len(nodes), n_fem, sum(len(p.nodes) for p in panels), len(members), len(all_tris), len(panels), len(welds), stats["on_member"], CFG["weld_brk"], len(lamps),
        len(lamp_mounts), doors))
    print("frame nodes by part: " + ", ".join("%s %d" % kv for kv in sorted(parts.items())))
    if volumes:
        print("collision volumes: " + ", ".join("%s (%d anchors)" % (v[0], len(v[1])) for v in volumes))
        for name, gap in volume_clearances():   # (the parts near them: how far off, cm; below 0 inside - not held off)
            near = sorted((g, p) for p, g in gap.items() if g < 0.15)
            print("  %s: %s" % (name, ", ".join("%s %.1f" % (p, g * 100) for g, p in near) or "no part within 15 cm"))
