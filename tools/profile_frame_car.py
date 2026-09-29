#!/usr/bin/env python3
"""The Frame Car's stress tests, profiled (headless, fixed 60 Hz frames): per scenario the physics and frame time
(mean, 95th percentile, worst) after the action, the frame elements' share, the members torn and split.

    python3 tools/profile_frame_car.py [scenario ...]      (names: drive, drop, roof, roll, curb, wall, ramp, balls, steel,
                                                            drop10, headon, tbone, slab, pole, axe; BL_PROF_OUT=dir keeps the CSVs;
                                                            BL_PROF_SCENE=buggy: the Buggy's, with whoops and jump)
"""
import csv, os, re, shutil, subprocess, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = os.path.join(ROOT, "build", "beamlab")
TMP = tempfile.mkdtemp(prefix="bl_fcprof_")
SCEN = {
    "drive": (["--drive", "0.6,0.35"], 900),   # (circles on the pad: straight on, it reached the wall)
    "drop": (["--action", "Drop from 5"], 420),
    "roof": (["--action", "Drop on the roof"], 420),
    "roll": (["--action", "Barrel roll"], 480),
    "curb": (["--action", "Trip over"], 420),
    "wall": (["--action", "Launch at the wall"], 420),
    "ramp": (["--action", "Off the ramp"], 420),
    "balls": (["--shoot", "1,45,0.12", "--camera", "look:0,1.6,-7,0,0.6,0"], 600),
    "steel": (["--shoot", "0,40,0.1", "--camera", "look:0,6,-3,0,0.6,0"], 900),
    "drop10": (["--action", "Drop from 10"], 420),
    "headon": (["--action", "Head-on into another"], 420),
    "tbone": (["--action", "into its side"], 420),
    "slab": (["--action", "Drop a 5 t concrete slab"], 360),
    "pole": (["--action", "Launch at the pole"], 420),
    "axe": (["--action", "The giant axe"], 360),
    "whoops": (["--action", "whoops", "--drive", "0.8,0"], 420),   # (the Buggy's gravel lane)
    "jump": (["--action", "jump", "--drive", "0.8,0"], 420),
}
SCENE = os.environ.get("BL_PROF_SCENE", "frame_car")


def stats(xs):
    xs = sorted(xs)
    if not xs:
        return 0, 0, 0
    return sum(xs) / len(xs), xs[int(0.95 * (len(xs) - 1))], xs[-1]


def run(name):
    args, frames = SCEN[name]
    path = os.path.join(TMP, name + ".csv")
    env = dict(os.environ, BL_PROFCSV=path)
    out = subprocess.run([EXE, "--scene", SCENE, "--size", "1280x720", "--frames", str(frames), "--hidden", "--novsync",
                          "--screenshot", os.path.join(TMP, name + ".png")] + args, env=env, capture_output=True, text=True, timeout=1800)
    if os.environ.get("BL_PROF_OUT"):   # (the per-frame CSV kept: charts)
        os.makedirs(os.environ["BL_PROF_OUT"], exist_ok=True)
        shutil.copy(path, os.path.join(os.environ["BL_PROF_OUT"], name + ".csv"))
    rows = list(csv.DictReader(open(path)))[10:]
    phys = [float(r["physics_ms"]) for r in rows]
    cpu = [float(r["cpu_ms"]) for r in rows]
    fe = [float(r.get("Frame elements", 0) or 0) + float(r.get("Frame solve", 0) or 0) for r in rows]
    broken = re.findall(r"broken\s+(\d+)", out.stdout)
    fstat = re.findall(r"frame: (\d+) members, (\d+) splits, (\d+) torn, (\d+) failed solves, (\d+) clamps", out.stdout)
    welds = re.findall(r"welds: (\d+) of (\d+) broken", out.stdout)
    warn = len(re.findall(r"unstable|explod", out.stdout + out.stderr))
    p, c, f = stats(phys), stats(cpu), stats(fe)
    fs = fstat[-1] if fstat else ("?",) * 5
    print("%-6s physics %5.2f / %5.2f / %6.2f ms  frame %5.2f / %5.2f / %6.2f ms  FEM %5.2f / %5.2f ms  members %s splits %s torn %s fails %s clamps %s  welds broken %s  warnings %d" % (
        name, p[0], p[1], p[2], c[0], c[1], c[2], f[0], f[2], fs[0], fs[1], fs[2], fs[3], fs[4], "%s/%s" % welds[-1] if welds else "?", warn), flush=True)
    return p, c


if __name__ == "__main__":
    names = sys.argv[1:] or [n for n in SCEN if SCENE == "buggy" or n not in ("whoops", "jump")]
    for n in names:
        run(n)
