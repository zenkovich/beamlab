#!/usr/bin/env python3
"""A BeamLab vehicle from a crash-test finite element model (LS-DYNA keyword file), such as the public NHTSA / CCSA
vehicle models (https://www.ccsa.gmu.edu/models/): the body-in-white parts (rails, floor, tunnel, firewall, rockers,
pillars, roof rails, cross members, wheel wells, bumpers...) become a stiff node-beam frame, the outer body panels
(doors, hood, fenders, body sides, roof, boot lid, bumper covers) become sheets of triangle elements hung on it, and
four wheels with steering are added at the model's tyres.

  python3 tools/import_lsdyna_car.py <model.key> <vehicle id> [--title "..."] [--cell 0.25] [--panel-cell 0.14]

The frame: every structural part's nodes are clustered on a grid (cell) and a cluster is a frame node; every element
that spans two clusters gives a beam, and pairs of clusters two beams apart but within 1.7 cells get a bracing beam
(a lattice of plate elements has no diagonals of its own). The panels: vertex clustering at a finer grid per part,
quads split into triangles, degenerate and thin triangles dropped; a panel node within a frame node's reach takes
the frame node instead (the panel hangs there). Parts are classified by their names (see IGNORE / PANEL / STRUCTURE).
Coordinates: the FE models have x along the car, y left, z up, in mm; the truck gets -x forward, y up, +z left, m.
The editor's groups (;grp:) name the parts and its layers separate the frame, the panels and the wheels.
"""
import argparse
import collections
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

IGNORE = ["seat", "trim", "dash", "ipbeam", "windshield", "window", "glass", "engine", "exhaust", "gastank", "suspension", "tire",
          "drum", "aarm", "upright", "steering", "battery", "headlight", "taillig", "fusebox", "washing", "airfilter", "brakebooster",
          "intake", "consol", "emergency", "crowbar", "hinge", "lock", "motor", "airbag", "curt", "null", "tailhook", "antiroll",
          "beltwheel", "oilpan", "driveshield", "driverod", "manifold", "hub", "disk", "rim", "tread", "radiatortop", "radiatorbottom",
          "radiatorfan", "radiatorblade", "radiatorside", "shockfront", "shockrear", "shockleft", "doorrearpatch", "doorrearwindow",
          "spotweld", "plasticbumperbrkt", "sprocket"]
# (the doors' inner panels, frames and impact bars, the hood and boot inner panels, the hinges and locks are structure:
# the outer skins hang on them, and they tie the doors, hood and boot lid to the body)
PANEL = [r"doorfrontouter", r"doorrearouter", r"^\d+_hood$", r"fender(left|right)$", r"sidepanel", r"^\d+_roof$", r"trunkouter", r"bumperplastic(rear)?$"]
STRUCTURE = ["rail", "frame", "floor", "tunnel", "firewall", "rocker", "pillar", "roof", "xmember", "trunk", "wheelwell",
             "shockhousing", "housingsupport", "subframe", "bumper", "rearpanel", "rearsection", "support", "sprt", "brkt", "plate",
             "connector", "mount", "apron", "header", "cowl", "member", "reinf", "cnt", "panel", "cap", "extension", "tray", "strip",
             "internal", "housing", "fill", "doorfrontinner", "doorrearinner", "doorframe", "doorfrontframe", "doorrearsupport",
             "doorfrontsupport", "doorfrontupper", "doorfrontspacing", "impactbar", "hoodinner", "trunkinner", "hinge", "lock"]

BEAM_KEEP = 26      # beams a frame node keeps (the longest extras are dropped: the stability budget)
BRACE = 8           # long bracing beams per node (a single layer of clusters is a plate lattice: stiff in its plane,
                    # free to bend; the braces across it make a space frame)


def classify(name):
    n = name.lower()
    for k in IGNORE:
        if k in n:
            return "ignore"
    for k in PANEL:
        if re.search(k, n):
            return "panel"
    for k in STRUCTURE:
        if k in n:
            return "structure"
    return "unknown"


def parse_key(path):
    parts, nodes, shells = {}, {}, []
    cur, pend, title = None, None, ""
    with open(path, errors="ignore") as f:
        for line in f:
            if line.startswith("$"):
                continue
            if line.startswith("*"):
                cur = line.strip().upper()
                pend = "title" if cur.startswith("*PART") else None
                continue
            if pend == "title":
                title, pend = line.strip(), "pid"
                continue
            if pend == "pid":
                t = line.split()
                if t:
                    parts[int(t[0])] = title
                pend = None
                continue
            if cur == "*NODE":
                try:
                    nodes[int(line[:8])] = (float(line[8:24]), float(line[24:40]), float(line[40:56]))
                except ValueError:
                    pass
            elif cur is not None and cur.startswith("*ELEMENT_SHELL"):
                t = line.split()
                if len(t) >= 6:
                    ns = [int(v) for v in t[2:6]]
                    ns = [n for n in ns if n > 0]
                    shells.append((int(t[1]), ns))
    return parts, nodes, shells


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("key")
    ap.add_argument("vid", help="vehicle id: assets/vehicles/<vid>/<vid>.truck")
    ap.add_argument("--title", default=None)
    ap.add_argument("--cell", type=float, default=0.3, help="frame cluster cell (m)")
    ap.add_argument("--brace", type=float, default=0.9, help="reach of the bracing beams (m)")
    ap.add_argument("--panel-cell", type=float, default=0.14, help="panel decimation cell (m)")
    ap.add_argument("--attach", type=float, default=0.15, help="a panel node this close to a frame node hangs on it (m)")
    ap.add_argument("--mass", type=float, default=1100.0, help="dry mass of the frame (kg)")
    ap.add_argument("--min-quality", type=float, default=0.22)
    ap.add_argument("--source", default="", help="text for SOURCE.txt")
    ap.add_argument("--panel-mass", type=float, default=0.8, help="minimum mass of a panel node (kg)")
    ap.add_argument("--no-wheels", action="store_true", help="(diagnostics) no wheels: the frame rests on the ground")
    ap.add_argument("--k-factor", type=float, default=1.0, help="frame stiffness: k = factor x m_min / (n_beams dt^2)")
    ap.add_argument("--deform", type=float, default=8000.0, help="force (N) above which a frame beam yields; it breaks at 20 x that")
    ap.add_argument("--max-fold", type=float, default=125.0, help="a sharper fold between two panel triangles drops the smaller (deg)")
    ap.add_argument("--damp-factor", type=float, default=2.5e-4, help="frame damping d = k x factor")
    args = ap.parse_args()

    parts, nodes, shells = parse_key(args.key)
    print("parsed: %d parts, %d nodes, %d shell elements" % (len(parts), len(nodes), len(shells)))
    kinds = {pid: classify(name) for pid, name in parts.items()}
    used = collections.Counter(pid for pid, _ in shells)
    unknown = sorted((used[p], parts[p]) for p in used if kinds.get(p) == "unknown")
    if unknown:
        print("unclassified parts (ignored):")
        for n, name in unknown[-25:]:
            print("   %5d %s" % (n, name))

    # ---- coordinates: the FE model's x along the car (front at max x), y left, z up, mm -> RoR
    xs = [p[0] for p in nodes.values()]
    cx = (min(xs) + max(xs)) * 0.5
    def conv(p):
        return (-(p[0] - cx) / 1000.0, p[2] / 1000.0, p[1] / 1000.0)

    # ---- the frame: clusters of the structural parts' nodes
    cell = args.cell
    cluster_of = {}          # FE node -> cluster key
    clusters = collections.OrderedDict()  # key -> [sum x, y, z, count, part counter]
    struct_elems = []
    for pid, ns in shells:
        if kinds.get(pid) != "structure":
            continue
        struct_elems.append((pid, ns))
        for n in ns:
            if n not in nodes:
                continue
            p = conv(nodes[n])
            key = (int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell)), int(math.floor(p[2] / cell)))
            c = clusters.setdefault(key, [0.0, 0.0, 0.0, 0, collections.Counter()])
            if n not in cluster_of:
                c[0] += p[0]
                c[1] += p[1]
                c[2] += p[2]
                c[3] += 1
                c[4][pid] += 1
                cluster_of[n] = key
    keys = list(clusters.keys())
    cid = {k: i for i, k in enumerate(keys)}
    fnodes = [(c[0] / c[3], c[1] / c[3], c[2] / c[3]) for c in clusters.values()]
    fpart = [c[4].most_common(1)[0][0] for c in clusters.values()]
    print("frame: %d clusters from %d structural elements" % (len(fnodes), len(struct_elems)))
    # beams: every element spanning clusters
    adj = collections.defaultdict(set)
    for pid, ns in struct_elems:
        cs = sorted({cid[cluster_of[n]] for n in ns if n in cluster_of})
        for i in range(len(cs)):
            for j in range(i + 1, len(cs)):
                adj[cs[i]].add(cs[j])
                adj[cs[j]].add(cs[i])
    # bracing: two beams apart and near
    def dist(a, b):
        return math.dist(fnodes[a], fnodes[b])
    extra = collections.defaultdict(set)
    for a in list(adj.keys()):
        for b in adj[a]:
            for c in adj[b]:
                if c != a and c not in adj[a] and dist(a, c) < 1.7 * cell:
                    extra[a].add(c), extra[c].add(a)
    for a, s in extra.items():
        adj[a] |= s
    # bracing: every node to up to BRACE nodes within reach that are not its neighbours (the longest first, spread
    # over directions: each new brace must be at least 35 degrees from the ones already taken)
    keys_all = list(adj.keys())
    for a in keys_all:
        cands = [b for b in keys_all if b != a and b not in adj[a] and dist(a, b) < args.brace and dist(a, b) > 1.2 * cell]
        cands.sort(key=lambda b: -dist(a, b))
        taken = []
        for b in cands:
            if len(taken) >= BRACE:
                break
            d = [fnodes[b][i] - fnodes[a][i] for i in range(3)]
            l = math.sqrt(sum(x * x for x in d))
            ok = True
            for t in taken:
                e = [fnodes[t][i] - fnodes[a][i] for i in range(3)]
                le = math.sqrt(sum(x * x for x in e))
                if sum(d[i] * e[i] for i in range(3)) / (l * le) > math.cos(math.radians(35)):
                    ok = False
                    break
            if ok:
                taken.append(b)
        for b in taken:
            extra[a].add(b), extra[b].add(a)
    for a, s in extra.items():
        adj[a] |= s
    # a node keeps its shortest BEAM_KEEP beams; nodes with fewer than two beams are dropped
    for a in list(adj.keys()):
        if len(adj[a]) > BEAM_KEEP:
            keep = sorted(adj[a], key=lambda b: dist(a, b))[:BEAM_KEEP]
            for b in adj[a] - set(keep):
                adj[b].discard(a)
            adj[a] = set(keep)
    changed = True
    while changed:
        changed = False
        for a in list(adj.keys()):
            if len(adj[a]) < 2:
                for b in adj[a]:
                    adj[b].discard(a)
                del adj[a]
                changed = True
    # islands: parts the clustering left unconnected (a door on its hinges, the hatch, a bracket) are welded to the
    # main body by beams between their nearest cluster pairs
    comp = {}
    for a in adj.keys():
        if a in comp:
            continue
        stack = [a]
        comp[a] = a
        while stack:
            x = stack.pop()
            for y in adj[x]:
                if y not in comp:
                    comp[y] = a
                    stack.append(y)
    groups = collections.defaultdict(list)
    for n, c in comp.items():
        groups[c].append(n)
    main = max(groups.values(), key=len)
    welds = 0
    for c, members in groups.items():
        if members is main:
            continue
        pairs = sorted(((dist(a, b), a, b) for a in members for b in main), key=lambda t: t[0])[:4]
        for d, a, b in pairs:
            if d < 0.6:
                adj[a].add(b), adj[b].add(a)
                welds += 1
    print("frame: %d islands welded to the body with %d beams" % (len(groups) - 1, welds))
    live = sorted(adj.keys())
    fmap = {old: new for new, old in enumerate(live)}
    frame_pos = [fnodes[i] for i in live]
    frame_part = [fpart[i] for i in live]
    beams = set()
    for a in live:
        for b in adj[a]:
            if b in fmap and a < b:
                beams.add((fmap[a], fmap[b]))
    beams = sorted(beams)
    print("frame: %d nodes, %d beams" % (len(frame_pos), len(beams)))

    # ---- the panels: vertex clustering per part
    pcell = args.panel_cell
    out_nodes = list(frame_pos)          # RoR positions
    node_part = list(frame_part)
    node_kind = ["frame"] * len(frame_pos)
    tris = []                             # (a, b, c, part)
    seen_tri = set()
    def frame_near(p, reach):
        best, bd = -1, reach * reach
        for i, q in enumerate(frame_pos):
            d = (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2
            if d < bd:
                bd, best = d, i
        return best
    panel_parts = sorted({pid for pid, _ in shells if kinds.get(pid) == "panel"}, key=lambda p: parts[p])
    for pid in panel_parts:
        # a bumper cover stands well off the bumper beam and its brackets, a body side off the pillars: they reach
        # further for the frame nodes they hang on
        name = parts[pid].lower()
        reach = args.attach * (1.8 if "bumperplastic" in name else 1.4 if "sidepanel" in name else 1.0)
        pc = collections.OrderedDict()   # cell -> [sum, count]
        vmap = {}
        elems = [ns for p, ns in shells if p == pid]
        for ns in elems:
            for n in ns:
                if n not in nodes or n in vmap:
                    continue
                p = conv(nodes[n])
                key = (int(math.floor(p[0] / pcell)), int(math.floor(p[1] / pcell)), int(math.floor(p[2] / pcell)))
                c = pc.setdefault(key, [0.0, 0.0, 0.0, 0])
                c[0] += p[0]
                c[1] += p[1]
                c[2] += p[2]
                c[3] += 1
                vmap[n] = key
        node_id = {}
        for key, c in pc.items():
            p = (c[0] / c[3], c[1] / c[3], c[2] / c[3])
            f = frame_near(p, reach)
            if f >= 0:
                node_id[key] = f
            else:
                node_id[key] = len(out_nodes)
                out_nodes.append(p)
                node_part.append(pid)
                node_kind.append("panel")
        made = 0
        for ns in elems:
            ids = [node_id[vmap[n]] for n in ns if n in vmap]
            polys = [(ids[0], ids[1], ids[2])] if len(ids) == 3 else ([(ids[0], ids[1], ids[2]), (ids[0], ids[2], ids[3])] if len(ids) == 4 else [])
            for t in polys:
                if len(set(t)) < 3:
                    continue
                k = tuple(sorted(t))
                if k in seen_tri:
                    continue
                a, b, c = (out_nodes[i] for i in t)
                u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
                v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
                n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
                area2 = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
                l2 = sum((q[0] - r[0]) ** 2 + (q[1] - r[1]) ** 2 + (q[2] - r[2]) ** 2 for q, r in ((a, b), (b, c), (c, a)))
                quality = 2.0 * math.sqrt(3) * area2 / max(1e-12, l2)   # 1 equilateral
                if area2 < 2e-4 or quality < args.min_quality:
                    continue
                seen_tri.add(k)
                tris.append((t[0], t[1], t[2], pid))
                made += 1
        print("panel %-28s %5d elements -> %4d triangles" % (parts[pid][:28], len(elems), made))
    # winding: the sheet's hinges need the two triangles of an edge wound the same way round (the edge runs one way in
    # one and the other way in the other), so the orientation is spread from triangle to triangle over shared edges,
    # then every connected patch is turned to face away from the model's centre by its majority normal
    edge_tris = collections.defaultdict(list)
    for i, t in enumerate(tris):
        for e in range(3):
            edge_tris[tuple(sorted((t[e], t[(e + 1) % 3])))].append(i)
    done = [False] * len(tris)
    flips = 0
    for seed in range(len(tris)):
        if done[seed]:
            continue
        comp = [seed]
        done[seed] = True
        stack = [seed]
        while stack:
            i = stack.pop()
            ti = tris[i]
            for e in range(3):
                a, b = ti[e], ti[(e + 1) % 3]
                for j in edge_tris[tuple(sorted((a, b)))]:
                    if j == i or done[j]:
                        continue
                    tj = tris[j]
                    same = any(tj[f] == a and tj[(f + 1) % 3] == b for f in range(3)) # the edge runs the same way: flip
                    if same:
                        tris[j] = (tj[0], tj[2], tj[1], tj[3])
                        flips += 1
                    done[j] = True
                    comp.append(j)
                    stack.append(j)
        # face the patch outwards
        score = 0.0
        for i in comp:
            t = tris[i]
            a, b, c = (out_nodes[k] for k in t[:3])
            u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            cen = ((a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3 - 0.7, (a[2] + b[2] + c[2]) / 3)
            score += n[0] * cen[0] * 0.3 + n[1] * cen[1] + n[2] * cen[2]
        if score < 0:
            for i in comp:
                t = tris[i]
                tris[i] = (t[0], t[2], t[1], t[3])
    # folds: the FE skins are hemmed round their edges and the covers have deep grille folds; two triangles folded
    # back to back over an edge (a dihedral past --max-fold) are a singular hinge, so the smaller one is dropped,
    # until no fold is left
    def tri_normal_area(t):
        a, b, c = (out_nodes[k] for k in t[:3])
        u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        l = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
        return ((n[0] / l, n[1] / l, n[2] / l) if l > 0 else (0, 0, 1)), l * 0.5
    dropped_folds = 0
    while True:
        edge_tris = collections.defaultdict(list)
        for i, t in enumerate(tris):
            for e in range(3):
                edge_tris[tuple(sorted((t[e], t[(e + 1) % 3])))].append(i)
        kill = set()
        cos_max = math.cos(math.radians(args.max_fold))
        for e, lst in edge_tris.items():
            if len(lst) < 2:
                continue
            if len(lst) > 2:      # a non-manifold edge: keep the two biggest
                lst = sorted(lst, key=lambda i: -tri_normal_area(tris[i])[1])
                for i in lst[2:]:
                    kill.add(i)
                lst = lst[:2]
            (n1, a1), (n2, a2) = tri_normal_area(tris[lst[0]]), tri_normal_area(tris[lst[1]])
            if n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2] < cos_max:
                kill.add(lst[0] if a1 < a2 else lst[1])
        if not kill:
            break
        tris = [t for i, t in enumerate(tris) if i not in kill]
        dropped_folds += len(kill)
    print("panels: %d nodes, %d triangles, winding made consistent (%d flipped), %d folded triangles dropped" % (len(out_nodes) - len(frame_pos), len(tris), flips, dropped_folds))

    # ---- wheels at the tyres
    tyres = collections.defaultdict(list)
    for pid, ns in shells:
        name = parts.get(pid, "").lower()
        if "tire" not in name or "hub" in name or "disk" in name:
            continue
        for n in ns:
            if n in nodes:
                tyres[(conv(nodes[n])[0] < 0, conv(nodes[n])[2] > 0)].append(conv(nodes[n]))
    wheels = []
    hydros = []
    frame_count = len(frame_pos)
    n_frame_beams = len(beams)       # (the wheel mounts and camera braces after these get a stiffer, tougher preset)
    def nearest_frame(p, k, exclude=()):
        order = sorted(range(frame_count), key=lambda i: math.dist(frame_pos[i], p))
        return [i for i in order if i not in exclude][:k]
    beams = list(beams)
    if args.no_wheels:
        tyres = {}
    for (front, left), pts in sorted(tyres.items(), key=lambda kv: (not kv[0][0], not kv[0][1])):
        cxw = sum(p[0] for p in pts) / len(pts)
        cyw = sum(p[1] for p in pts) / len(pts)
        czw = sum(p[2] for p in pts) / len(pts)
        radius = max(0.25, (max(p[1] for p in pts) - min(p[1] for p in pts)) * 0.5)
        s = 1.0 if left else -1.0
        n1 = (cxw, radius, s * (abs(czw) - 0.10))
        n2 = (cxw, radius, s * (abs(czw) + 0.12))
        i1 = len(out_nodes); out_nodes.append(n1); node_part.append(-1); node_kind.append("wheel")
        i2 = len(out_nodes); out_nodes.append(n2); node_part.append(-1); node_kind.append("wheel")
        tower = (cxw, radius + 0.34, n1[2])
        it = len(out_nodes); out_nodes.append(tower); node_part.append(-1); node_kind.append("wheel")
        for f in nearest_frame(n1, 6):
            beams.append((i1, f))
        for f in nearest_frame(tower, 6):
            beams.append((it, f))
        beams += [(i1, i2), (i1, it), (i2, it)]
        if front:
            rack = (cxw + 0.42, radius, s * (abs(czw) - 0.42))
            ir = len(out_nodes); out_nodes.append(rack); node_part.append(-1); node_kind.append("wheel")
            for f in nearest_frame(rack, 4):
                beams.append((ir, f))
            beams.append((ir, it))
            hydros.append((i2, ir, 0.15 * s))
        else:
            for f in nearest_frame(n2, 3):
                beams.append((i2, f))
        arm = nearest_frame(n1, 1)[0]
        wheels.append((radius, i1, i2, arm, front))
        print("wheel %s %s at x %.2f z %.2f r %.2f" % ("front" if front else "rear", "left" if left else "right", cxw, s * abs(czw), radius))

    # ---- camera nodes: the game orients the model by three nodes (forward = centre - back, left = left - centre),
    # so three level, square nodes are added and braced to the frame
    mid = min(range(frame_count), key=lambda i: math.dist(frame_pos[i], (0, 0.5, 0)))
    mp = frame_pos[mid]
    cam_ids = []
    for off in ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.8)):
        p = (mp[0] + off[0], mp[1] + off[1], mp[2] + off[2])
        cid_ = len(out_nodes)
        out_nodes.append(p)
        node_part.append(-1)
        node_kind.append("wheel")
        for f in nearest_frame(p, 6):
            beams.append((cid_, f))
        cam_ids.append(cid_)
    beams += [(cam_ids[0], cam_ids[1]), (cam_ids[1], cam_ids[2]), (cam_ids[0], cam_ids[2])]

    # ---- masses and stiffness: the frame's beams take a stability budget of the lightest, busiest node
    dt = 0.0005
    minimass = 5.0
    deg = collections.Counter()
    for a, b in beams:
        deg[a] += 1
        deg[b] += 1
    worst = max(deg.values())
    # (the beams of a node point in many directions: the stiffness it sees is a fraction of their sum. RoR's cars run
    # 9e6 N/m beams on 3 kg nodes with ~13 beams each at this step; 3 x m / (n dt^2) stays well inside that)
    k = args.k_factor * minimass / (worst * dt * dt)
    k = min(k, 4.0e6)
    print("frame beams: %d, busiest node %d beams -> k %.3g N/m" % (len(beams), worst, k))

    # ---- write the truck
    vid = args.vid
    title = args.title or vid
    dir_ = os.path.join(ROOT, "assets", "vehicles", vid)
    os.makedirs(dir_, exist_ok=True)
    path = os.path.join(dir_, vid + ".truck")
    nid = list(range(len(out_nodes)))
    with open(path, "w") as f:
        f.write("%s\n" % title)
        f.write(";written by tools/import_lsdyna_car.py from %s\n" % os.path.basename(args.key))
        f.write(";editor-layers: Frame|1|0 Panels|1|0 Wheels|1|0\n")
        f.write("globals\n;dry mass, cargo mass, cab material (the panels: a sheet of 1.5 mm steel, refined one level under load)\n%.0f, 0.0, sheet/Steel/12/0.004/1\n" % args.mass)
        f.write("minimass\n%.1f\n" % minimass)
        f.write("nodes\n;id, x, y, z, options\n")
        cur_layer, cur_grp = None, None
        for i, p in enumerate(out_nodes):
            layer = {"frame": "Frame", "panel": "Panels", "wheel": "Wheels"}[node_kind[i]]
            grp = parts.get(node_part[i], "wheels") if node_part[i] >= 0 else "wheels"
            grp = re.sub(r"^\d+[_\- ]*", "", grp).strip() or grp
            if node_kind[i] == "panel" and i > 0 and node_kind[i - 1] == "frame":
                f.write("set_default_minimass %.2f\n" % args.panel_mass)
            if node_kind[i] == "wheel" and i > 0 and node_kind[i - 1] == "panel":
                f.write("set_default_minimass 5.0\n")
            if layer != cur_layer:
                f.write(";layer:%s\n" % layer)
                cur_layer = layer
            if grp != cur_grp:
                f.write(";grp:%s\n" % grp)
                cur_grp = grp
            opt = "l" if node_kind[i] != "panel" else "n"
            f.write("%d, %.4f, %.4f, %.4f, %s\n" % (i, p[0], p[1], p[2], opt))
        f.write("enable_advanced_deformation\n") # (else the format raises a deform threshold under 400 kN to 400 kN)
        f.write("beams\n;the frame: a stiff structure that yields early and crumples (the crash energy goes into plastic work, not a bounce)\nset_beam_defaults %.0f, %.0f, %.0f, %.0f, 0.05, tracks/beam, 0.3\n" % (k, k * args.damp_factor, args.deform, args.deform * 20))
        for i, (a, b) in enumerate(beams):
            if i == n_frame_beams:
                f.write(";the wheel mounts, steering rack and camera braces: no yielding under the drive and steering loads\n")
                f.write("set_beam_defaults %.0f, %.0f, %.0f, 99999999999999999999999999999999999999999, 0.05, tracks/beam, 0.0\n" % (k, k * args.damp_factor, 80000))
            f.write("%d, %d\n" % (a, b))
        f.write("hydros\nset_beam_defaults 15000000, 500, 99999999999999999999999999999999999999999, 99999999999999999999999999999999999999999, 0.02, tracks/beam, 0.0\n")
        for a, b, fac in hydros:
            f.write("%d, %d, %.2f, i\n" % (a, b, fac))
        f.write("wheels\n;radius, width, rays, node1, node2, rigidity, braking, propulsion, arm, mass, spring, damping, face, band\n")
        for radius, i1, i2, arm, front in wheels:
            f.write("%.3f, 0.20, 12, %d, %d, 9999, 1, %d, %d, 30.0, 120000.0, 900.0, tracks/wheelface tracks/wheelband\n" % (radius, i1, i2, 1 if front else 0, arm))
        if wheels:
            f.write("engine\n1000.0, 6000.0, 400.0, 4.3, 3.4, 1.0, 3.6, 2.2, 1.5, 1.0, 0.8, -1.0\nengoption\n0.4, c, 300.0\nbrakes\n4000\n")
        f.write("contacters\n")
        for i in nid:
            f.write("%d\n" % i)
        f.write("cameras\n%d, %d, %d\n" % (cam_ids[0], cam_ids[1], cam_ids[2]))
        near = sorted(range(frame_count), key=lambda i: math.dist(frame_pos[i], (0.3, 0.9, 0.35)))[:8]
        f.write("cinecam\n0.3, 1.05, 0.35, %s\n" % ", ".join(str(i) for i in near))
        used_nodes = sorted({i for t in tris for i in t[:3]})
        f.write("submesh\ntexcoords\n")
        for i in used_nodes:
            p = out_nodes[i]
            f.write("%d, %.4f, %.4f\n" % (i, (p[0] + 2.2) / 4.4, (p[2] + 0.9 + p[1]) / 3.2))
        f.write("cab\n")
        cur_grp = None
        for a, b, c, pid in tris:
            grp = re.sub(r"^\d+[_\- ]*", "", parts.get(pid, "panel")).strip()
            if grp != cur_grp:
                f.write(";grp:%s\n" % grp)
                cur_grp = grp
            f.write("%d, %d, %d, c\n" % (a, b, c))
        f.write("end\n")
    src = os.path.join(dir_, "SOURCE.txt")
    if not os.path.exists(src):
        with open(src, "w") as f:
            f.write("Crash-test FE models\n" + (args.source or "Converted by tools/import_lsdyna_car.py from an LS-DYNA vehicle model.") + "\n")
    print("wrote %s: %d nodes (%d frame, %d panel), %d beams, %d triangles, %d wheels, %d hydros" %
          (path, len(out_nodes), frame_count, len(out_nodes) - frame_count - 4 * 3 - 2, len(beams), len(tris), len(wheels), len(hydros)))


if __name__ == "__main__":
    main()
