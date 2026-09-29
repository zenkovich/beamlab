#!/usr/bin/env python3
"""Scenario checks through the application (headless): what the physics has to keep doing.

    python3 tools/test_scenes.py [--quick]

sheet_run: the car drives through all three lead sheets, each cracks, pieces fall off, nothing is lost (area);
materials: glass shatters, rubber holds, the metals punch through, the ball lays each material's fracture pattern;
physics lab: cannonballs through the steel sheet;
sheet shapes: the car through the 6 x 4 m gate and the dome, cannonballs through each panel, a laser cut across the gate;
frame car: hung from a crane tilted about both axes, its wheels hang in their travel; standing its welded panels hold,
it steers past 25 degrees and the right way, it accelerates, driving round nothing comes off, two of them head-on and
into the side stay apart (their frames' collision hulls), dropped 5 m its doors, hood and tailgate stay on, head-on the
bumpers and fenders come loose, the crash test scenes start their tests, into the wall its rear wheels stay straight;
steel barrels (and with FEM rings): dropped, rolled, thrown and stacked they dent but keep their shape, a 40 kg ball dents
them without sticking, nothing tears; tipped over or dropped they come to rest where they land; left alone they stay put;
the pile of 15 with three thrown in within a 30 FPS frame; the editor's drum from a circle lands and rests;
buggy: the Frame Car's tests on the desert racer, through the whoops and over the jump, the offroad hills;
FEM shells: the sheet, the cantilever and the hollow cube of triangle elements under their loads yield where they should
and come to rest; shell car: stands, steers, accelerates, drives round and through the crash tests stable;
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

# ---- the Frame Car (a hatchback's panels welded on its frame): standing 4 s nothing tears (no weld, no crack), the
# steering turns the front wheels past 25 degrees, and launched at the wall the rear wheels stay within 20 degrees of
# straight (their short toe links turned them through 90 degrees when the tail lifted; the toe stops)
out = run(["--scene", "frame_car", "--size", "640x360", "--frames", "240", "--hidden", "--novsync", "--drive", "0,1.0",
           "--screenshot", os.path.join(TMP, "fc2.png")], {"BL_SHELLDBG": "235", "BL_WHEELDBG": "1"})
wb = re.findall(r"(\d+) of (\d+) welds broken", out)
cr = re.findall(r"Frame Car\s+shells.*?cracks\s+(\d+)", out)
toe = [abs(float(t)) for t in re.findall(r"toe ([-\d.]+) camber", out)[-4:]]
check("frame car: standing, steering", bool(wb and cr and toe) and wb[-1][0] == "0" and int(wb[-1][1]) > 500 and cr[-1] == "0" and min(toe[:2]) > 25 and max(toe[2:]) < 2,
      "%s welds broken of %s, %s cracks, front toe %s, rear toe %s deg" % (wb[-1][0] if wb else "?", wb[-1][1] if wb else "?", cr[-1] if cr else "?",
                                                                         ", ".join("%.1f" % t for t in toe[:2]), ", ".join("%.1f" % t for t in toe[2:])))
# steering right turns it right (facing +z: x falls; the rack's hydro once turned the wheels against the input), full
# throttle is quick (the engine's inertia of 0.35 and a slipping clutch crawled to 26 km/h in 5 s), and driving round
# on the pad nothing comes off (the headlights' welds let go of the frame's own vibration)
out = run(["--scene", "frame_car", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--drive", "0.3,0.5",
           "--screenshot", os.path.join(TMP, "fc4.png")])
pos = re.findall(r"^t=\s*[\d.]+s\s+speed.*?pos \(([-\d.]+) [-\d.]+ ([-\d.]+)\)", out, re.M)
check("frame car: steering right turns right", bool(pos) and float(pos[-1][0]) < -1.5, "position %s after 5 s at half right lock" % (pos[-1] if pos else "?",))
out = run(["--scene", "frame_car", "--size", "640x360", "--frames", "250", "--hidden", "--novsync", "--drive", "1.0,0",
           "--screenshot", os.path.join(TMP, "fc5.png")])
spd = re.findall(r"^t=\s*([\d.]+)s\s+speed\s+([-\d.]+) km/h", out, re.M)
v4 = [float(v) for t, v in spd if abs(float(t) - 4.0) < 0.01]
check("frame car: 0-60 km/h within 4 s", bool(v4) and v4[0] > 60, "%.1f km/h at 4 s" % v4[0] if v4 else "no speed")
out = run(["--scene", "frame_car", "--size", "640x360", "--frames", "900", "--hidden", "--novsync", "--drive", "0.6,0.35",
           "--screenshot", os.path.join(TMP, "fc6.png")])
wb = re.findall(r"welds: (\d+) of (\d+) broken", out)
check("frame car: driving round, nothing comes off", bool(wb) and wb[-1][0] == "0" and unstable(out) == 0, "%s of %s welds broken" % wb[-1] if wb else "no weld count")
# two cars into each other: their frames' collision hulls (one-sided, solid hull triangles on the frame nodes) keep them
# apart - without them two Frame Cars went through each other (head-on their centres ended 0.3 m apart, ~300 frame
# nodes of one inside the other); a few nodes of the crushed fronts may stay in a little (the parts' frames count too)
for action, far, most_deep in (("Head-on into another", 1.8, 0.35), ("into its side", 1.4, 0.35)):
    out = run(["--scene", "frame_car", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--action", action,
               "--screenshot", os.path.join(TMP, "fc_in.png")])
    m = re.findall(r"frame nodes inside the other car: (\d+), (\d+) cm deep \(most (\d+), (\d+) cm\); centres ([\d.]+) m apart", out)
    ok = bool(m) and float(m[-1][4]) > far and int(m[-1][3]) < most_deep * 100 and unstable(out) == 0
    check("frame car: %s, the frames stay apart" % action, ok,
          "%s nodes inside at the end, %s cm deep (at most %s nodes, %s cm), centres %s m apart" % m[-1] if m else "no measure")
# the parts on their hinges and bolts (members with a break force; BL_FRAMEDBG names each that lets go and its force):
# dropped 5 m on its wheels the doors, the hood and the tailgate stay on (their hinges 20 and 15 kN); head-on the
# bumpers' plastic brackets (2.5 kN) and the fenders' bolts (9 kN) let go
def let_go(out):
    return [float(x) for x in re.findall(r"lets go at \d+ N \(breaks at (\d+)\)", out)]
out = run(["--scene", "frame_car", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--action", "Drop from 5", "--screenshot",
           os.path.join(TMP, "fc_d5.png")], {"BL_FRAMEDBG": "1"})
lg = let_go(out)
check("frame car: dropped 5 m, the doors, the hood and the tailgate stay on", bool(re.search(r"^t=", out, re.M)) and not [b for b in lg if b >= 15000] and unstable(out) == 0,
      "%d door hinges, %d lid hinges, %d fender bolts, %d bumper brackets, %d mirror stalks let go" % tuple(sum(1 for b in lg if b == v) for v in (20000, 15000, 9000, 2500, 500)))
out = run(["--scene", "frame_car", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--action", "Head-on into another", "--screenshot",
           os.path.join(TMP, "fc_h.png")], {"BL_FRAMEDBG": "1"})
lg = let_go(out)
check("frame car: head-on, the bumpers' brackets and the fenders' bolts let go", 2500 in lg and 9000 in lg and unstable(out) == 0,
      "%d door hinges, %d lid hinges, %d fender bolts, %d bumper brackets, %d mirror stalks let go" % tuple(sum(1 for b in lg if b == v) for v in (20000, 15000, 9000, 2500, 500)))
# the crash tests' scenes: each starts its test on the first frame (the second car, the slab, the axe come in) and runs
for sid, label in (("fc_headon", "Head-on"), ("fc_side", "into its side"), ("fc_wall", "wall"), ("fc_pole", "pole"), ("fc_drop", "Drop from 10"),
                   ("fc_slab", "slab"), ("fc_axe", "axe"), ("fc_roll", "Barrel roll")):
    out = run(["--scene", sid, "--size", "640x360", "--frames", "180", "--hidden", "--novsync", "--screenshot", os.path.join(TMP, sid + ".png")])
    started = re.findall(r"scene test: (.*)", out)
    check("scene %s: the test starts" % sid, bool(started) and label in started[0] and unstable(out) == 0 and os.path.exists(os.path.join(TMP, sid + ".png")),
          started[0].strip() if started else "not started")
out = run(["--scene", "frame_car", "--size", "640x360", "--frames", "400", "--hidden", "--novsync", "--drive", "0,0.0001", "--action", "Launch at the wall",
           "--screenshot", os.path.join(TMP, "fc3.png")], {"BL_WHEELDBG": "1", "BL_SHELLDBG": "395"})
toe = [abs(float(t)) for t in re.findall(r"toe ([-\d.]+) camber", out)[-4:]]
fr = re.findall(r"frame: (\d+) members, (\d+) splits, (\d+) torn, (\d+) failed solves", out)
check("frame car: into the wall at 60 km/h", bool(toe and fr) and max(toe[2:]) < 20 and fr[-1][3] == "0" and unstable(out) == 0,
      "rear toe %s deg, %s splits, %s torn, %s failed solves, %d warnings" % (", ".join("%.1f" % t for t in toe[2:]), fr[-1][1] if fr else "?", fr[-1][2] if fr else "?",
                                                                          fr[-1][3] if fr else "?", unstable(out)))

# ---- the Buggy (a desert racer's cage, long-travel wishbones and trailing arms, anti-roll bars of FEM tubes): the Frame
# Car's tests on it, and its own - the whoops and the jump on the gravel lane, a drive over the offroad scene's hills
def frame_line(out):
    m = re.findall(r"frame: (\d+) members, (\d+) splits, (\d+) torn, (\d+) failed solves, (\d+) clamps \| welds: (\d+) of (\d+) broken", out)
    return tuple(int(x) for x in m[-1]) if m else None


def speeds_at(out):
    return {round(float(t), 2): (float(v), float(x), float(y), float(z)) for t, v, x, y, z in
            re.findall(r"^t=\s*([\d.]+)s\s+speed\s+([-\d.]+) km/h.*?pos \(([-\d.]+) ([-\d.]+) ([-\d.]+)\)", out, re.M)}


# standing, then hung from the crane tilted: the wheels hang to full droop (18-23 cm, the springs still pushing at it),
# camber and toe within a few degrees, nothing torn
bstand = wheels(run(["--scene", "buggy", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--screenshot", os.path.join(TMP, "bs.png")],
                    {"BL_SHOCKDBG": "290"}))
for roll, pitch in ((8, 6), (-12, -9)):
    out = run(["--scene", "buggy", "--size", "640x360", "--frames", "600", "--hidden", "--novsync", "--crane", "1.0,-1,%d,%d" % (roll, pitch),
               "--screenshot", os.path.join(TMP, "bh.png")], {"BL_SHOCKDBG": "590"})
    hang, fl = wheels(out), frame_line(out)
    ok = len(bstand) == 4 and len(hang) == 4
    drops = [s_[1] - h[1] for s_, h in zip(bstand, hang)] if ok else [0]
    ang = max(max(abs(h[3]), abs(h[4])) for h in hang) if ok else 99
    check("buggy: hung tilted %+d/%+d deg, the wheels hang to full droop" % (roll, pitch),
          ok and min(drops) > 0.12 and max(drops) < 0.30 and ang < 6 and fl and fl[2] == 0 and fl[5] == 0 and unstable(out) == 0,
          "wheels %s cm lower, camber/toe up to %.1f deg, %s torn, %s welds broken" % (", ".join("%.0f" % (d * 100) for d in drops), ang, fl[2] if fl else "?",
                                                                                     fl[5] if fl else "?"))
# standing 4 s nothing tears, full lock turns the front wheels past 25 degrees, the rear stay straight
out = run(["--scene", "buggy", "--size", "640x360", "--frames", "240", "--hidden", "--novsync", "--drive", "0,1.0", "--screenshot", os.path.join(TMP, "b2.png")],
          {"BL_SHELLDBG": "235", "BL_WHEELDBG": "1"})
fl = frame_line(out)
cr = re.findall(r"Desert Buggy\s+shells.*?cracks\s+(\d+)", out)
toe = [abs(float(t)) for t in re.findall(r"toe ([-\d.]+) camber", out)[-4:]]
check("buggy: standing, steering", bool(fl and cr and toe) and fl[5] == 0 and fl[6] > 200 and cr[-1] == "0" and min(toe[0], toe[2]) > 25 and max(toe[1], toe[3]) < 2,
      "%s welds broken of %s, %s cracks, toe %s deg" % (fl[5] if fl else "?", fl[6] if fl else "?", cr[-1] if cr else "?", ", ".join("%.1f" % t for t in toe)))
out = run(["--scene", "buggy", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--drive", "0.3,0.5", "--screenshot", os.path.join(TMP, "b4.png")])
pos = re.findall(r"^t=\s*[\d.]+s\s+speed.*?pos \(([-\d.]+) [-\d.]+ ([-\d.]+)\)", out, re.M)
check("buggy: steering right turns right", bool(pos) and float(pos[-1][0]) < -1.5, "position %s after 5 s at half right lock" % (pos[-1] if pos else "?",))
# full throttle: 0-100 km/h in about 7 s (850 N m, 1460 kg, 39 inch tyres on asphalt: the grip holds it to ~0.45 g)
out = run(["--scene", "buggy", "--size", "640x360", "--frames", "480", "--hidden", "--novsync", "--drive", "1.0,0", "--screenshot", os.path.join(TMP, "b5.png")])
sp = speeds_at(out)
t100 = min([t for t, v in sp.items() if v[0] >= 100] or [99])
check("buggy: 0-60 km/h within 4 s, 0-100 within 8 s", sp.get(4.0, (0,))[0] > 60 and t100 < 8, "%.1f km/h at 4 s, 100 km/h at %.1f s" % (sp.get(4.0, (0,))[0], t100))
# the Frame Car's circle (0.6 throttle, 0.35 lock): the rear steps out at 0.8 g and it spins; it lands back on its
# wheels (up on two, anti-roll bars and 20-ray tyres; without them it rolled) and nothing more than a weld or two lets go
out = run(["--scene", "buggy", "--size", "640x360", "--frames", "900", "--hidden", "--novsync", "--drive", "0.6,0.35", "--screenshot", os.path.join(TMP, "b6.png")])
fl, sp = frame_line(out), speeds_at(out)
last = sp[max(sp)] if sp else (0, 0, 9, 0)
check("buggy: driving round, back on its wheels", bool(fl) and fl[2] == 0 and fl[5] <= 3 and last[2] < 0.8 and unstable(out) == 0,
      "%s welds broken, %s torn, it ends at %.1f km/h, %.2f m up" % (fl[5] if fl else "?", fl[2] if fl else "?", last[0], last[2]))
# two of them head-on and into the side: the frames stay apart (the front clip triangulated: its box of rails folded
# up and two cars went into each other); head-on the nose's bolts let go
for action, far, most_deep in (("Head-on into another", 1.8, 0.35), ("into its side", 1.4, 0.35)):
    out = run(["--scene", "buggy", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--action", action, "--screenshot", os.path.join(TMP, "b_in.png")],
              {"BL_FRAMEDBG": "1"})
    m = re.findall(r"frame nodes inside the other car: (\d+), (\d+) cm deep \(most (\d+), (\d+) cm\); centres ([\d.]+) m apart", out)
    lg = let_go(out)
    ok = bool(m) and float(m[-1][4]) > far and int(m[-1][3]) < most_deep * 100 and unstable(out) == 0 and (action[0] != "H" or 5000 in lg)
    check("buggy: %s, the frames stay apart" % action, ok, ("%s nodes inside at the end, %s cm deep (at most %s nodes, %s cm), centres %s m apart" % m[-1] if m else "no measure") +
          ", %d nose bolts let go, %d warnings" % (sum(1 for b in lg if b == 5000), unstable(out)))
# dropped 5 m and 10 m on its wheels: the panels' bolts hold (the cage flexing between them tore stiff ones)
for h, fr in ((5, 300), (10, 300)):
    out = run(["--scene", "buggy", "--size", "640x360", "--frames", str(fr), "--hidden", "--novsync", "--action", "Drop from %d" % h,
               "--screenshot", os.path.join(TMP, "bd.png")], {"BL_FRAMEDBG": "1"})
    lg, fl = let_go(out), frame_line(out)
    check("buggy: dropped %d m, the panels stay on" % h, not lg and bool(fl) and fl[2] == 0 and unstable(out) == 0,
          "%d bolts let go, %s torn, %s welds broken" % (len(lg), fl[2] if fl else "?", fl[5] if fl else "?"))
# the whoops (0.5 m every 8 m) at 80 km/h and the tabletop jump (a 2.2 m kicker, 12 degrees at the lip) at 90: through
# and on at speed, nothing breaks (before the rear was raised and its hoop moved the tail struck the kicker and the rear
# clip yielded; a yield of a milliradian or two in the frame's joints at the peaks stays)
for action, z_min, v_min in (("whoops", 100, 55), ("jump", 205, 60)):
    out = run(["--scene", "buggy", "--size", "640x360", "--frames", "400", "--hidden", "--novsync", "--drive", "0.8,0", "--action", action,
               "--screenshot", os.path.join(TMP, "bw.png")], {"BL_FRAMEDBG": "1"})
    fl, sp, lg = frame_line(out), speeds_at(out), let_go(out)
    last = sp[max(sp)] if sp else (0, 0, 0, 0)
    hinge = max([float(x) for pair in re.findall(r"splits: .*?\(([\d.]+) ([\d.]+)\)", out) for x in pair] or [0])
    check("buggy: %s, through at speed and nothing breaks" % action,
          bool(fl) and fl[2] == 0 and fl[5] == 0 and not lg and last[3] > z_min and last[0] > v_min and hinge < 0.005 and unstable(out) == 0,
          "at z %.0f m, %.0f km/h; %s welds broken, %s torn, %d bolts let go, the largest plastic rotation %.4f rad" % (last[3], last[0], fl[5] if fl else "?",
                                                                                                              fl[2] if fl else "?", len(lg), hinge))
# dropped on its roof from 1.5 m, rolled at 50 km/h: the cage holds (no member torn)
for action in ("Drop on the roof", "Barrel roll"):
    out = run(["--scene", "buggy", "--size", "640x360", "--frames", "400", "--hidden", "--novsync", "--action", action, "--screenshot", os.path.join(TMP, "br.png")])
    fl = frame_line(out)
    check("buggy: %s, the cage holds" % action.lower(), bool(fl) and fl[2] == 0 and unstable(out) == 0,
          "%s splits, %s torn, %s welds broken" % (fl[1] if fl else "?", fl[2] if fl else "?", fl[5] if fl else "?"))
# into the wall at 60 km/h: no failed solve, stable
out = run(["--scene", "buggy", "--size", "640x360", "--frames", "400", "--hidden", "--novsync", "--drive", "0,0.0001", "--action", "Launch at the wall",
           "--screenshot", os.path.join(TMP, "bwl.png")])
fl = frame_line(out)
check("buggy: into the wall at 60 km/h", bool(fl) and fl[3] == 0 and unstable(out) == 0,
      "%s splits, %s torn, %s failed solves, %d warnings" % (fl[1] if fl else "?", fl[2] if fl else "?", fl[3] if fl else "?", unstable(out)))
# the offroad scene's hills at half throttle: 6 s over the rough ground at 30 km/h, upright, nothing torn
out = run(["--scene", "offroad", "--vehicle", "buggy/buggy", "--size", "640x360", "--frames", "360", "--hidden", "--novsync", "--drive", "0.5,0",
           "--screenshot", os.path.join(TMP, "bo.png")])
fl, sp = frame_line(out), speeds_at(out)
last = sp[max(sp)] if sp else (0, 0, 0, 0)
check("buggy: offroad hills", bool(fl) and fl[2] == 0 and fl[5] < 10 and last[0] > 20 and unstable(out) == 0,
      "%.0f km/h at %.1f s, %s welds broken, %s torn" % (last[0], max(sp) if sp else 0, fl[5] if fl else "?", fl[2] if fl else "?"))
# the crash tests' scenes start their tests
for sid, label in (("bg_headon", "Head-on"), ("bg_side", "into its side"), ("bg_wall", "wall"), ("bg_drop", "Drop from 10"), ("bg_slab", "slab"),
                   ("bg_roll", "Barrel roll"), ("bg_whoops", "whoops"), ("bg_jump", "jump")):
    out = run(["--scene", sid, "--size", "640x360", "--frames", "180", "--hidden", "--novsync", "--screenshot", os.path.join(TMP, sid + ".png")])
    started = re.findall(r"scene test: (.*)", out)
    check("scene %s: the test starts" % sid, bool(started) and label in started[0] and unstable(out) == 0 and os.path.exists(os.path.join(TMP, sid + ".png")),
          started[0].strip() if started else "not started")

# ---- FEM shells (triangle elements of the frame): the steel sheet on its supports, the cantilever and the hollow cube
# under their loads yield where they should (the cantilever with 250 kg welded under its tip folds at its root, with 100
# kg it springs back up), nothing tears, no solve fails, and what rests comes to rest where it lies (a slab on the cube
# walked it off at the shells' old damping; it still creeps a few mm a second: the contacts are explicit)
def fem_objs(out):
    return {m.group(1).strip(): dict(tris=int(m.group(2)), torn=int(m.group(3)), dented=int(m.group(4)), peak=float(m.group(5)), failed=int(m.group(6)),
                                     sleep=int(m.group(7)), fast=float(m.group(8)), c=(float(m.group(9)), float(m.group(10)), float(m.group(11))),
                                     low=float(m.group(12)))
            for m in re.finditer(r"^fem f\d+ (.+?)\s+tris\s+(\d+) torn\s+(\d+) dented\s+(\d+) peak ([\d.]+) failed (\d+) sleep (\d) fastest ([\d.]+) "
                                 r"centre \(([-\d.]+) ([-\d.]+) ([-\d.]+)\) lowest ([-\d.]+)", out, re.M)}
fem_cases = [
    ("Reset", "all three rest, nothing yields", lambda o: all(x["dented"] == 0 and x["fast"] < 0.05 for x in o.values()) and o["fem cantilever"]["low"] > 1.1),
    ("ball on the sheet from 10 m", "the sheet dents", lambda o: o["fem sheet"]["dented"] > 20 and o["fem sheet"]["fast"] < 0.5),
    ("500 kg block on the sheet", "the sheet folds under it", lambda o: o["fem sheet"]["dented"] > 100 and o["fem sheet"]["low"] < 0.1),
    ("Weld 100 kg", "the cantilever springs", lambda o: o["fem cantilever"]["low"] > 0.6 and o["fem cantilever"]["dented"] < 20),
    ("Weld 250 kg", "the cantilever folds at its root", lambda o: o["fem cantilever"]["low"] < 0.1 and o["fem cantilever"]["dented"] > 5 and
     o["fem cantilever"]["fast"] < 0.1),
    ("Drop 1 t on the cantilever", "the cantilever folds", lambda o: o["fem cantilever"]["dented"] > 10),
    ("cube from 5 m on a face", "it dents a little and rests", lambda o: 0 < o["fem cube"]["dented"] < 60 and o["fem cube"]["fast"] < 0.05 and
     abs(o["fem cube"]["c"][1] - 0.5) < 0.03),
    ("cube from 5 m on a corner", "it dents a little and rests", lambda o: 0 < o["fem cube"]["dented"] < 60 and o["fem cube"]["fast"] < 0.05),
    ("Load the cube's lid", "it holds, elastic", lambda o: o["fem cube"]["dented"] == 0 and o["fem cube"]["fast"] < 0.05),
    ("1 t slab on the cube", "it dents and holds it", lambda o: o["fem cube"]["dented"] > 0 and abs(o["fem cube"]["c"][0] + 6) < 0.15 and
     abs(o["fem cube"]["c"][2]) < 0.15 and o["fem cube"]["c"][1] > 0.45),
    ("cube at the wall", "it crumples at the front", lambda o: o["fem cube"]["dented"] > 20 and o["fem cube"]["fast"] < 0.1),
]
for action, what, ok in fem_cases:
    out = run(["--scene", "fem_shells", "--size", "640x360", "--frames", "600", "--hidden", "--novsync", "--action", action,
               "--screenshot", os.path.join(TMP, "fs.png")], {"BL_FEMDBG": "599"})
    o = {k: v for k, v in fem_objs(out).items()}
    good = len(o) == 3 and all(x["torn"] == 0 and x["failed"] == 0 for x in o.values()) and unstable(out) == 0
    try:
        good = good and ok(o)
    except KeyError:
        good = False
    check("fem shells: %s: %s" % (action.lower(), what), good,
          "; ".join("%s %d dented, %d torn, fastest %.2f m/s, lowest %.2f m" % (k[4:], x["dented"], x["torn"], x["fast"], x["low"]) for k, x in o.items()) +
          ", %d warnings" % unstable(out))

# ---- shell car (its body of FEM triangles, no frame): it stands, steers the right way, accelerates, drives round and
# through the crash tests stable, no solve failing; standing and driving nothing yields
def shell_line(out):
    m = re.findall(r"(\d+) failed solves, \d+ clamps \| tris (\d+), (\d+) torn, (\d+) dented", out)
    return tuple(int(x) for x in m[-1]) if m else None
out = run(["--scene", "shell_car", "--size", "640x360", "--frames", "420", "--hidden", "--novsync", "--drive", "1.0,0", "--screenshot", os.path.join(TMP, "sc1.png")])
sl, sp = shell_line(out), speeds_at(out)
check("shell car: accelerates, nothing yields", bool(sl) and sl[0] == 0 and sl[2] == 0 and sl[3] == 0 and sp.get(6.0, (0,))[0] > 50 and unstable(out) == 0,
      "%.1f km/h at 6 s, %s" % (sp.get(6.0, (0,))[0], "%d failed, %d torn, %d dented" % (sl[0], sl[2], sl[3]) if sl else "no line"))
out = run(["--scene", "shell_car", "--size", "640x360", "--frames", "300", "--hidden", "--novsync", "--drive", "0.3,0.5", "--screenshot", os.path.join(TMP, "sc2.png")])
pos = re.findall(r"pos \(([-\d.]+) ([-\d.]+) ([-\d.]+)\)", out)
check("shell car: steering right turns right", bool(pos) and float(pos[-1][0]) < -1.5, "position %s after 5 s at half right lock" % (pos[-1] if pos else "?",))
out = run(["--scene", "shell_car", "--size", "640x360", "--frames", "900", "--hidden", "--novsync", "--drive", "0.6,0.35", "--screenshot", os.path.join(TMP, "sc3.png")])
sl = shell_line(out)
check("shell car: driving round, next to nothing yields", bool(sl) and sl[0] == 0 and sl[2] == 0 and sl[3] < 5 and unstable(out) == 0,
      "%d failed, %d torn, %d dented" % (sl[0], sl[2], sl[3]) if sl else "no line")
for action, frames in (("Head-on into another", 300), ("into its side", 300), ("Drop from 5", 300), ("Launch at the wall", 400), ("slab", 400), ("Barrel roll", 400)):
    out = run(["--scene", "shell_car", "--size", "640x360", "--frames", str(frames), "--hidden", "--novsync", "--action", action,
               "--screenshot", os.path.join(TMP, "sc4.png")])
    sl = shell_line(out)
    check("shell car: %s, stable" % action.lower(), bool(sl) and sl[0] == 0 and sl[2] < 150 and unstable(out) == 0,
          "%d failed, %d torn, %d dented, %d warnings" % (sl[0], sl[2], sl[3], unstable(out)) if sl else "no line")

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
