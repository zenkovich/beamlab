#!/usr/bin/env python3
"""Scenario checks through the application (headless): what the physics has to keep doing.

    python3 tools/test_scenes.py [--quick]

sheet_run: the car drives through all three lead sheets, each cracks, pieces fall off, nothing is lost (area);
materials: glass shatters, rubber holds, the metals punch through, the ball lays each material's fracture pattern;
physics lab: cannonballs through the steel sheet;
sheet shapes: the car through the 6 x 4 m gate and the dome, cannonballs through each panel, a laser cut across the gate;
frame car: hung from a crane tilted about both axes, its wheels hang in their travel;
steel barrels (and with FEM rings): dropped, rolled, thrown and stacked they dent but keep their shape, a 40 kg ball dents
them without sticking, nothing tears; tipped over or dropped they come to rest where they land; left alone they stay put;
the pile of 15 with three thrown in within a 30 FPS frame; the editor's drum from a circle lands and rests;
rally: the autopilot finishes in the usual time; all scenes: no numerical instability.
"""
import os, re, subprocess, sys, csv, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = os.path.join(ROOT, "build", "beamlab")
TMP = tempfile.mkdtemp(prefix="bl_scenes_")
quick = "--quick" in sys.argv
results = []


def run(args, env=None, timeout=900):
    e = dict(os.environ)
    e.update(env or {})
    p = subprocess.run([EXE] + args, env=e, capture_output=True, text=True, timeout=timeout)
    return p.stdout + p.stderr


def check(name, ok, detail):
    results.append((name, ok, detail))
    print(("PASS " if ok else "FAIL ") + name + ": " + detail, flush=True)


def shell_blocks(out):
    """Last BL_SHELLDBG block: {sheet name: (cracks, pieces)} and the area lines."""
    sheets, areas = {}, []
    for line in out.splitlines():
        m = re.match(r"\s+(\S.*?)\s+shells\s+\d+ \(levels.*?refined\s+\d+ cracks\s+(\d+) pieces (\d+)", line)
        if m:
            sheets[m.group(1)] = (int(m.group(2)), int(m.group(3)))
        m = re.match(r"\s+area ([\d.]+) m2", line)
        if m:
            areas.append(float(m.group(1)))
    return sheets, areas


def sheet_areas(out):
    """Every BL_SHELLDBG block: [{sheet name: area of the sheet and its pieces}]."""
    blocks, area = [], None
    for line in out.splitlines():
        if re.match(r"t=\s*[\d.]+s physics", line):
            blocks.append({})
        m = re.match(r"\s+area ([\d.]+) m2", line)
        if m:
            area = float(m.group(1))
        m = re.match(r"\s+(\S.*?)\s+shells\s+\d+ \(levels", line)
        if m and blocks and area is not None:
            blocks[-1][m.group(1)] = area
    return blocks


def areas_kept(out):
    b = sheet_areas(out)
    return len(b) >= 2 and len(b[0]) > 0 and all(abs(b[-1].get(k, 0) - a) < 2e-4 * a for k, a in b[0].items())


def unstable(out):
    return len(re.findall(r"unstable|explod", out, re.I))


# ---- sheet run
csvp = os.path.join(TMP, "sheet_run.csv")
out = run(["--scene", "sheet_run", "--size", "640x360", "--frames", "600", "--hidden", "--novsync", "--launch", "70", "--drive", "0.7,0",
           "--screenshot", os.path.join(TMP, "sr.png")], {"BL_SHELLDBG": "599", "BL_PROFCSV": csvp})
sheets, areas = shell_blocks(out)
rows = list(csv.DictReader(open(csvp)))
z = float(rows[-1]["veh_z"])
check("sheet_run: car through all sheets", z > 140, f"car at z = {z:.1f} m after 10 s")
for i in (1, 2, 3):
    c, p = sheets.get(f"lead sheet {i}", (0, 0))
    check(f"sheet_run: sheet {i} cracked", c > 20, f"{c} cracks, {p} pieces")
check("sheet_run: pieces", sum(p for c, p in sheets.values()) >= 6, f"{sum(p for c, p in sheets.values())} pieces")
check("sheet_run: area kept", all(abs(a - 8.64) < 2e-3 for a in areas) and len(areas) >= 3, f"areas {areas}")
check("sheet_run: stable", unstable(out) == 0, f"{unstable(out)} warnings")
phys = [float(r["physics_ms"]) for r in rows[30:]]
print(f"      physics ms: avg {sum(phys) / len(phys):.2f}, max {max(phys):.1f}")

# ---- materials lab
out = run(["--scene", "materials", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--screenshot", os.path.join(TMP, "m.png")],
          {"BL_SHELLDBG": "299"})
sheets, areas = shell_blocks(out)
g = sheets.get("sheet: Glass", (0, 0))
check("materials: glass shatters", g[0] > 250 and g[1] > 20, f"{g[0]} cracks, {g[1]} pieces")
rb = sheets.get("sheet: Rubber", (0, 0))
check("materials: rubber holds", rb[0] == 0, f"{rb[0]} cracks")
for m in ("Steel", "Aluminium", "Lead", "Acrylic", "Plywood", "Cardboard", "Fabric"):
    c, p = sheets.get("sheet: " + m, (0, 0))
    check(f"materials: {m.lower()} cracked", c > 30, f"{c} cracks, {p} pieces")
check("materials: area kept", all(abs(a - 2.89) < 2e-3 or abs(a - 10.8) < 2e-3 for a in areas) and len(areas) >= 9, f"{len(areas)} sheets")
imp = {m.group(1): int(m.group(2)) for m in re.finditer(r"\s+(sheet: \w+)\s+shells.*\| impacts (\d+)", out)}
patterned = ["sheet: Glass", "sheet: Acrylic", "sheet: Steel", "sheet: Aluminium", "sheet: Plywood"]
check("materials: fracture patterns laid", all(imp.get(n, 0) >= 1 for n in patterned) and imp.get("sheet: Lead", 0) == 0,
      ", ".join(f"{n[7:]} {imp.get(n, 0)}" for n in patterned + ["sheet: Lead"]))
check("materials: stable", unstable(out) == 0, f"{unstable(out)} warnings")

# ---- physics lab: cannonballs into the steel sheet
out = run(["--scene", "lab", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--camera", "look:0,2.5,-44,0,1.6,-50", "--shoot",
           "3,60,0.4", "--screenshot", os.path.join(TMP, "l.png")], {"BL_SHELLDBG": "299"})
sheets, areas = shell_blocks(out)
st = sheets.get("steel sheet", (0, 0))
check("lab: cannonballs tear the steel sheet", st[0] > 20, f"{st[0]} cracks, {st[1]} pieces")
check("lab: stable", unstable(out) == 0, f"{unstable(out)} warnings")

# ---- sheet shapes: the car through the gate, the half-pipe and the dome; cannonballs; the laser
csvp = os.path.join(TMP, "sheet_shapes.csv")
out = run(["--scene", "sheet_shapes", "--size", "640x360", "--frames", "600", "--hidden", "--novsync", "--launch", "60", "--drive", "0.6,0",
           "--screenshot", os.path.join(TMP, "ss.png")], {"BL_SHELLDBG": "599", "BL_PROFCSV": csvp})
sheets, _ = shell_blocks(out)
z = float(list(csv.DictReader(open(csvp)))[-1]["veh_z"])
gate, dome = sheets.get("steel gate 6x4", (0, 0)), sheets.get("acrylic dome", (0, 0))
check("sheet_shapes: car through the gate and the dome", z > 120 and gate[1] > 0 and dome[0] > 20,
      f"car at z = {z:.1f} m; gate {gate[0]} cracks, {gate[1]} pieces; dome {dome[0]} cracks")
check("sheet_shapes: area kept (drive)", areas_kept(out), f"{len(sheet_areas(out)[-1])} sheets")
check("sheet_shapes: stable (drive)", unstable(out) == 0, f"{unstable(out)} warnings")
for name, cam in [("lead disc", "look:-1,1.9,16,-8,1.9,16"), ("glass ring", "look:1,2.75,16,8,2.75,16"),
                  ("plywood triangle", "look:-1,1.9,26,-8,1.9,26"), ("aluminium L", "look:1,2.3,26,8,2.3,26")]:
    out = run(["--scene", "sheet_shapes", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--camera", cam, "--shoot", "3,60,0.4",
               "--screenshot", os.path.join(TMP, "sp.png")], {"BL_SHELLDBG": "299"})
    c, p = shell_blocks(out)[0].get(name, (0, 0))
    check(f"sheet_shapes: cannonballs through the {name}", c > 20 and areas_kept(out) and unstable(out) == 0,
          f"{c} cracks, {p} pieces, area kept {areas_kept(out)}, {unstable(out)} warnings")
out = run(["--scene", "sheet_shapes", "--size", "640x360", "--frames", "240", "--hidden", "--novsync", "--camera", "look:1,2.5,33,0,2.3,40",
           "--laser", "0.1,0.55,0.9,0.45,30,40", "--screenshot", os.path.join(TMP, "sl.png")], {"BL_SHELLDBG": "239"})
m = re.search(r"laser: (\d+) cuts", out)
cuts = int(m.group(1)) if m else 0
check("sheet_shapes: laser across the gate", cuts > 100 and areas_kept(out) and unstable(out) == 0,
      f"{cuts} links cut, area kept {areas_kept(out)}, {unstable(out)} warnings")

# ---- sheet car: the car with a sheet body launched at 80 km/h into the parked one: its body refines and cracks, no beam
# of the frame breaks, no numerical trouble
out = run(["--scene", "sheet_car", "--size", "640x360", "--frames", "700", "--hidden", "--novsync", "--launch", "80", "--action", "zzz",
           "--screenshot", os.path.join(TMP, "sc.png")], {"BL_SHELLDBG": "690"})
cars = re.findall(r"Sheet Car\s+shells\s+(\d+) \(levels.*?refined\s+(\d+) cracks\s+(\d+)", out)
broken = re.findall(r"broken\s+(\d+)", out)
player = max(cars, key=lambda c: int(c[1])) if cars else ("0", "0", "0")
check("sheet_car: the body crumples in the crash", len(cars) >= 2 and int(player[1]) > 100 and int(player[2]) > 5,
      f"{len(cars)} cars, the hit one: {player[0]} triangles, {player[1]} splits, {player[2]} cracks")
check("sheet_car: the frame holds, stable", broken and int(broken[-1]) < 20 and unstable(out) == 0, f"{broken[-1] if broken else '?'} beams broken, {unstable(out)} warnings")

# ---- frame car: standing, then hung from a crane by the top of its cage, tilted about both axes: the wheels hang in
# their travel (a little lower, the same track, camber and toe within a few degrees), nothing yields or breaks
def wheels(out):
    return [tuple(float(v) for v in m) for m in re.findall(r"wheel at \(([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\) camber ([-+\d.]+) toe ([-+\d.]+)", out)]


stand = wheels(run(["--scene", "frame_car", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--screenshot", os.path.join(TMP, "fs.png")],
                   {"BL_SHOCKDBG": "290"}))
for roll, pitch in ((8, 6), (-12, -9)):
    out = run(["--scene", "frame_car", "--size", "640x360", "--frames", "600", "--hidden", "--novsync", "--crane", "1.0,-1,%d,%d" % (roll, pitch),
               "--screenshot", os.path.join(TMP, "fh.png")], {"BL_SHOCKDBG": "590"})
    hang = wheels(out)
    st = re.findall(r"frame: \d+ members, (\d+) splits, (\d+) torn", out)
    ok = len(stand) == 4 and len(hang) == 4
    drop = max(s_[1] - h[1] for s_, h in zip(stand, hang)) if ok else 9
    track = max(abs(abs(h[2]) - abs(s_[2])) for s_, h in zip(stand, hang)) if ok else 9
    ang = max(max(abs(h[3]), abs(h[4])) for h in hang) if ok else 99
    check("frame_car: hung tilted %+d/%+d deg, the wheels hang" % (roll, pitch), ok and drop < 0.04 and track < 0.02 and ang < 3 and st and st[-1] == ("0", "0") and unstable(out) == 0,
          "wheels %.1f cm lower, track %+.1f cm, camber/toe up to %.1f deg, %s splits, %s torn" % (drop * 100, track * 100, ang, st[-1][0] if st else "?", st[-1][1] if st else "?"))

# ---- steel barrels (a closed sheet: its volume in the BL_SHELLDBG block, 100% = 0.2232 m3): dropped on its bottom, side and
# rim, rolled down the ramp, thrown at another and stacked it dents but keeps its shape (a few percent); the 40 kg ball
# dents it deeper without tearing it or sticking in it; nothing cracks, nothing comes loose, the physics within a few
# milliseconds (15: a sanity bound, the machine may be busy). The same with frame rings (FEM): the frame holds (nothing
# torn, no failed solve). Tipped over it lands on its side and sleeps there (it slept balanced on its rim: the rest
# damping's rolling resistance); one stood on another stays and both sleep (they woke each other in turn). The scene's
# barrels at rest stay where they are (they drifted: the ground contact's push)
def barrels(out):
    """The last BL_SHELLDBG block: {barrel name: (volume %, centre, cracks, pieces, asleep)}."""
    last = out[out.rfind("\nt="):]
    return {m.group(5): (100 * float(m.group(1)) / 0.2232, tuple(float(m.group(i)) for i in (2, 3, 4)), int(m.group(6)), int(m.group(7)), "asleep" in m.group(8))
            for m in re.finditer(r"volume ([\d.]+) m3, centre \(([-\d.]+) ([-\d.]+) ([-\d.]+)\)\n\s+(barrel[^\n]*?)\s+shells.*?cracks\s+(\d+) pieces (\d+)([^\n]*)", last)}


BARREL_TESTS = [  # (action, frames, the least volume % each barrel keeps, the most (the dished ends pop out), what else, FEM rings)
    ("Drop it on its bottom from 5 m", 300, 96, 105, None, 0),
    ("Drop it on its side from 10 m", 360, 92, 104, None, 0),
    ("Drop it on the rim (45 deg) from 2 m", 300, 96, 104, None, 0),
    ("Roll it down the ramp", 420, 96, 103, "rolled", 0),
    ("Throw one barrel at another at 8 m/s", 300, 94, 102, None, 0),
    ("Stand one barrel on another", 480, 98, 102, "stacked", 0),
    ("Drop one barrel onto another from 3 m", 360, 95, 103, None, 0),
    ("Tip it over (a push at the top)", 480, 97, 103, "tipped", 0),
    ("Drop a 40 kg steel ball onto it from 3 m", 240, 65, 92, "ball", 0),
    ("Shoot a 40 kg steel ball at it against the wall at 20 m/s", 240, 30, 70, "ball", 0),
    ("Drop it on its side from 10 m", 360, 94, 106, None, 1),
    ("Throw one barrel at another at 8 m/s", 300, 94, 103, None, 1),
    ("Stand one barrel on another", 480, 98, 102, "stacked", 1),
    ("Tip it over (a push at the top)", 480, 97, 103, "tipped", 1),
    ("Drop a 40 kg steel ball onto it from 3 m", 240, 85, 100, "ball", 1),
] if not quick else [("Drop it on its side from 10 m", 360, 92, 104, None, 0), ("Stand one barrel on another", 480, 98, 102, "stacked", 0),
                     ("Tip it over (a push at the top)", 480, 97, 103, "tipped", 1), ("Drop a 40 kg steel ball onto it from 3 m", 240, 65, 92, "ball", 0)]
for act, frames, vmin, vmax, extra, fem in BARREL_TESTS:
    csvp = os.path.join(TMP, "barrels.csv")
    env = {"BL_SHELLDBG": str(frames - 5), "BL_PROFCSV": csvp}
    if fem:
        env["BL_BARREL_FEM"] = "1"
    out = run(["--scene", "barrels", "--size", "640x360", "--frames", str(frames), "--hidden", "--novsync", "--action", act,
               "--screenshot", os.path.join(TMP, "b.png")], env)
    bs = {k: v for k, v in barrels(out).items() if not re.match(r"barrel (fem )?\d", k)}  # (not the scene's own)
    phys = [float(r["physics_ms"]) for r in list(csv.DictReader(open(csvp)))[10:]]
    ok = bs and all(vmin <= v <= vmax and c == 0 and p == 0 for v, _, c, p, _ in bs.values()) and unstable(out) == 0 and sum(phys) / len(phys) < 15
    more = ""
    if extra == "rolled":  # off the ramp's foot (z 23.9) onto the ground
        ok = ok and bs.get("barrel", (0, (0, 0, 0)))[1][2] > 24
    if extra == "stacked":  # the one above standing on the one below (its centre a barrel and a half up), both asleep
        ok = ok and 1.3 < bs.get("barrel above", (0, (0, 0, 0)))[1][1] < 1.45 and abs(bs["barrel above"][1][0]) < 0.05 and all(v[4] for v in bs.values())
    if extra == "tipped":  # on its side (its centre a radius up, not balanced on the rim at 0.52 m) and asleep
        ok = ok and bs.get("barrel", (0, (0, 1, 0)))[1][1] < 0.33 and bs["barrel"][4]
    if extra == "ball":  # the ball not stuck in the drum: no node of it inside the ball
        m = re.findall(r"ball \S+: centre .*?radius ([\d.]+), the nearest sheet node ([\d.]+) m away", out)
        ok = ok and bool(m) and float(m[-1][1]) > float(m[-1][0]) - 0.01
        more = "; the ball's clearance %s m (radius %s)" % (m[-1][1], m[-1][0]) if m else "; no ball"
    if fem:  # the frame rings hold
        fr = re.findall(r"frame: (\d+) members, (\d+) splits, (\d+) torn, (\d+) failed solves", out)
        ok = ok and bool(fr) and all(int(t) == 0 and int(f) == 0 for _, _, t, f in fr)
        more += "; frames: %s" % (", ".join("%s torn, %s failed" % (t, f) for _, _, t, f in fr[-2:]) if fr else "none")
    check("barrels%s: %s" % (" (FEM)" if fem else "", act), bool(ok),
          "; ".join("%s: volume %.1f%%, centre (%.2f %.2f %.2f), %d cracks, %d pieces%s" % ((k, v) + c + (cr, p, ", asleep" if z else "")) for k, (v, c, cr, p, z) in bs.items())
          + more + "; physics %.2f ms avg, %d warnings" % (sum(phys) / max(1, len(phys)), unstable(out)))

# ---- dropped or knocked over, a barrel comes to rest where it lands: asleep by 12 s, not moved more than 5 cm after the
# first 4 s (a dented one trembled and walked off; one with frame rings threw itself about now and then)
for act, fem in ([("Drop it on its side from 2 m", 0), ("Drop it on the rim (45 deg) from 2 m", 1), ("Tip it over (a push at the top)", 1)] if not quick else
                 [("Drop it on its side from 2 m", 0)]):
    out = run(["--scene", "barrels", "--size", "640x360", "--frames", "720", "--hidden", "--novsync", "--action", act, "--screenshot", os.path.join(TMP, "b.png")],
              dict({"BL_SHELLDBG": "60"}, **({"BL_BARREL_FEM": "1"} if fem else {})))
    track = []
    for blk in out.split("\nt=")[1:]:
        m = re.search(r"centre \(([-\d.]+) ([-\d.]+) ([-\d.]+)\)\n\s+barrel\s+shells[^\n]*", blk)
        if m: track.append((float(m.group(1)), float(m.group(3)), "asleep" in m.group(0)))
    moved = ((track[-1][0] - track[4][0]) ** 2 + (track[-1][1] - track[4][1]) ** 2) ** 0.5 if len(track) > 5 else 9
    check("barrels%s: comes to rest: %s" % (" (FEM)" if fem else "", act), len(track) > 5 and track[-1][2] and moved < 0.05,
          "%s, moved %.1f cm after 4 s" % ("asleep" if track and track[-1][2] else "awake", 100 * moved))

# ---- the stress pile: 15 barrels, three thrown in one after another: the physics well within a 30 FPS frame
for fem in ((0, 1) if not quick else (0,)):
    csvp = os.path.join(TMP, "stress.csv")
    out = run(["--scene", "stress_barrels", "--size", "640x360", "--frames", "600", "--hidden", "--novsync", "--screenshot", os.path.join(TMP, "sb.png")],
              dict({"BL_PROFCSV": csvp, "BL_SHELLDBG": "595"}, **({"BL_BARREL_FEM": "1"} if fem else {})))
    phys = sorted(float(r["physics_ms"]) for r in list(csv.DictReader(open(csvp)))[10:])
    sheets, _ = shell_blocks(out)
    loose = sum(p for _, p in sheets.values())
    check("barrels%s: the stress pile" % (" (FEM)" if fem else ""), phys and sum(phys) / len(phys) < 25 and loose == 0 and unstable(out) == 0,
          "%d barrels, physics %.1f ms avg, 95%% %.1f, max %.1f, %d pieces, %d warnings" % (len(sheets), sum(phys) / max(1, len(phys)), phys[int(0.95 * len(phys))] if phys else 0,
                                                                                         phys[-1] if phys else 0, loose, unstable(out)))

# ---- the scene's barrels left alone stay where they are: asleep in place in a few seconds; kept awake, they hold still
for nosleep, secs, tol in ((0, 8, 0.012), (1, 20, 0.02)):
    out = run(["--scene", "barrels", "--size", "640x360", "--frames", str(int(secs * 60)), "--hidden", "--novsync", "--screenshot", os.path.join(TMP, "b.png")],
              dict({"BL_SHELLDBG": "60"}, **({"BL_NOSLEEP": "1"} if nosleep else {})))
    first, last, asleep = {}, {}, {}
    lines = out.split("\n")
    for i, l in enumerate(lines):
        mm = re.match(r"\s+(barrel[^\n]*?)\s+shells", l)
        if mm:
            c = re.search(r"centre \(([-\d.]+) ([-\d.]+) ([-\d.]+)", lines[i - 1])
            if c:
                pnt = (float(c.group(1)), float(c.group(3)))
                first.setdefault(mm.group(1), pnt)
                last[mm.group(1)] = pnt
                asleep[mm.group(1)] = "asleep" in l
    moved = {k: ((last[k][0] - first[k][0]) ** 2 + (last[k][1] - first[k][1]) ** 2) ** 0.5 for k in first}
    worst = max(moved.values()) if moved else 9
    ok = len(moved) >= 9 and worst < tol and (nosleep or all(asleep.values()))
    check("barrels: left alone %s" % ("(awake) %d s" % secs if nosleep else "they sleep where they stand"), ok,
          "%d barrels, the most one moved %.1f mm%s" % (len(moved), worst * 1000, "" if nosleep else ", %d asleep" % sum(asleep.values())))

# ---- the editor: a circle of sheet (16 and 48 sides) pulled into a drum and dropped: it lands and rests (it came apart
# on its first frame: the vehicle's sheet body was renumbered; the 48-sided one's centre overflowed a gather buffer)
for sides in (16, 48):
    p = subprocess.run([EXE, "--scene", "proving", "--editor", "--size", "640x360", "--frames", "240", "--hidden", "--novsync",
                        "--screenshot", os.path.join(TMP, "eb.png")], env=dict(os.environ, BL_EDITOR_EMPTY="1", BL_EDITOR_BARREL="0.3,0.9,%d" % sides,
                        BL_EDITOR_PHYSICS="1", BL_EDITOR_GRAVITY="1", BL_SHELLDBG="235"), capture_output=True, text=True, timeout=900)
    mm = re.findall(r"New model\s+shells\s+(\d+) .*?cracks\s+(\d+) pieces (\d+).*?(asleep)?\s*\|", p.stdout)
    ok = p.returncode == 0 and bool(mm) and mm[-1][1] == "0" and mm[-1][2] == "0" and mm[-1][3] == "asleep" and unstable(p.stdout) == 0
    check("editor: a drum of %d sides pulled from a circle" % sides, ok,
          "exit %d, %s" % (p.returncode, ("%s triangles, %s cracks, %s pieces, %s" % (mm[-1][0], mm[-1][1], mm[-1][2], mm[-1][3] or "awake")) if mm else "no sheet"))

# ---- model editor: the cart template saved, registered and spawned for a test drive drives off (--editor-test)
out = run(["--scene", "proving", "--size", "640x360", "--frames", "400", "--hidden", "--novsync", "--editor-test", "--drive", "0.5,0",
           "--screenshot", os.path.join(TMP, "ed.png")])
speeds = [float(v) for v in re.findall(r"speed\s+([-\d.]+) km/h", out)]
brk = re.findall(r"broken\s+(\d+)", out)
check("editor: the cart from the editor drives", speeds and speeds[-1] > 15 and brk and int(brk[-1]) == 0 and unstable(out) == 0,
      f"{speeds[-1] if speeds else '?'} km/h, {brk[-1] if brk else '?'} broken, {unstable(out)} warnings")
check("editor: the model file was written", os.path.exists(os.path.join(ROOT, "assets", "vehicles", "editor", "cart.truck")), "assets/vehicles/editor/cart.truck")

# ---- the Yaris body-in-white (a converted crash-test FE model): drives, and crumples against the wall without trouble
out = run(["--scene", "crash", "--vehicle", "yaris_biw/yaris_biw", "--size", "640x360", "--frames", "360", "--hidden", "--novsync", "--drive", "0.6,0",
           "--screenshot", os.path.join(TMP, "yd.png")])
speeds = [float(v) for v in re.findall(r"speed\s+([-\d.]+) km/h", out)]
check("yaris: drives", speeds and speeds[-1] > 5 and unstable(out) == 0, f"{speeds[-1] if speeds else '?'} km/h, {unstable(out)} warnings")
out = run(["--scene", "crash", "--vehicle", "yaris_biw/yaris_biw", "--spawn", "-6,260,0", "--launch", "80", "--action", "zzz", "--size", "640x360", "--frames", "300",
           "--hidden", "--novsync", "--screenshot", os.path.join(TMP, "yc.png")], {"BL_SHELLDBG": "299"})
yb = re.findall(r"shells\s+(\d+) \(levels.*?refined\s+(\d+) cracks\s+(\d+)", out)
check("yaris: the wall crash crumples the body, stable", yb and int(yb[-1][1]) > 30 and unstable(out) == 0,
      f"{yb[-1][0] if yb else '?'} triangles, {yb[-1][1] if yb else '?'} splits, {yb[-1][2] if yb else '?'} cracks, {unstable(out)} warnings")

# ---- all scenes: no numerical trouble
if not quick:
    for sc in ["proving", "forest", "canyon", "offroad", "crash", "vehicle_crash", "stress_vehicles", "stress_derby", "stress_crates",
               "stress_forest", "stress_bridge", "tape_maze", "rally", "rbr_verkiai"]:
        out = run(["--scene", sc, "--size", "640x360", "--frames", "600", "--hidden", "--novsync", "--screenshot", os.path.join(TMP, "x.png")])
        check(f"{sc}: stable", unstable(out) == 0, f"{unstable(out)} warnings")
    # ---- rally autopilot: vehicles behave as before
    out = run(["--scene", "rally", "--vehicle", "bmw_e36", "--size", "640x360", "--frames", "6600", "--hidden", "--novsync", "--action", "Autopilot",
               "--screenshot", os.path.join(TMP, "r.png")], timeout=1800)
    m = re.search(r"FINISH\s+(\d+):(\d+\.\d+)", out)
    t = int(m.group(1)) * 60 + float(m.group(2)) if m else 0
    check("rally: autopilot finishes as before", 93 < t < 100, f"stage time {t:.2f} s (was 96.2)")

fails = [r for r in results if not r[1]]
print(f"\n{len(results) - len(fails)} passed, {len(fails)} failed")
sys.exit(1 if fails else 0)
