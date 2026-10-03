#!/usr/bin/env python3
"""Downloads the scenes' open assets (all CC0) into assets/textures and assets/models:

  textures  PBR materials of ambientCG (https://ambientcg.com): colour, normal (OpenGL), roughness, ambient occlusion
  sky       a panorama of Poly Haven (https://polyhaven.com), its tonemapped JPG at 4096 x 2048, and where its sun is
  models    Poly Haven models (glTF, 1k textures) converted to the engine's .blm (one mesh: position, normal, uv; its
            indices) with their diffuse / normal / ARM textures and a hull for collisions (support planes in 42
            directions: a k-DOP)

    python3 tools/fetch_assets.py [--force] [textures] [sky] [models]

The folders are not in the repository (as the vehicle mods): a scene without them falls back to plain materials.
"""
import io
import json
import math
import os
import struct
import sys
import urllib.request
import zipfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEX = os.path.join(ROOT, "assets", "textures")
MODELS = os.path.join(ROOT, "assets", "models")
# name in assets/textures: (ambientCG asset, resolution)
TEXTURES = {
    "road_two_lane": ("Road007", "2K"),       # two lanes: edge lines, a dashed centre line
    "road_multi_lane": ("Road006", "2K"),     # a wide road: double centre line, lane lines
    "asphalt": ("Road012A", "2K"),            # plain asphalt
    "asphalt_new": ("Asphalt025C", "1K"),     # dark fresh asphalt: repair patches
    "asphalt_worn": ("Asphalt019", "2K"),     # cracked, patched asphalt
    "asphalt_light": ("Asphalt031", "1K"),    # worn light asphalt
    "grass": ("Grass004", "2K"),
    "dirt": ("Ground054", "2K"),
    "gravel": ("Gravel022", "1K"),
    "concrete": ("Concrete034", "1K"),
    "concrete_dark": ("Concrete031", "1K"),
    "paving": ("PavingStones128", "1K"),
    "cobbles": ("PavingStones070", "2K"),
    "facade_brick": ("Facade018A", "1K"),
    "facade_flats": ("Facade020B", "1K"),
    "facade_office": ("Facade006", "1K"),
    "facade_glass": ("Facade001", "1K"),
    "rock": ("Rock030", "1K"),
    "metal_plate": ("DiamondPlate008C", "1K"),
    "metal": ("Metal032", "1K"),
}
SKY = "kloofendal_48d_partly_cloudy_puresky"
MODEL_IDS = ["concrete_road_barrier", "concrete_road_barrier_02", "street_lamp_01", "street_lamp_02", "fire_hydrant", "old_tyre", "Barrel_01",
             "rock_07", "rock_09", "boulder_01", "namaqualand_boulder_02", "namaqualand_boulder_04", "namaqualand_boulder_05", "rock_moss_set_01",
             "moon_rock_01", "moon_rock_03", "moon_rock_05", "modular_chainlink_fence", "shrub_02", "shrub_04"]


def get(url):
    req = urllib.request.Request(url, headers={"User-Agent": "beamlab-fetch-assets"})
    with urllib.request.urlopen(req, timeout=300) as r:
        return r.read()


def fetch_textures(force):
    os.makedirs(TEX, exist_ok=True)
    for name, (asset, res) in TEXTURES.items():
        if os.path.exists(os.path.join(TEX, name + "_color.jpg")) and not force:
            continue
        z = zipfile.ZipFile(io.BytesIO(get("https://ambientcg.com/get?file=%s_%s-JPG.zip" % (asset, res))))
        got = []
        for suffix, out in (("_Color.jpg", "color"), ("_NormalGL.jpg", "normal"), ("_Roughness.jpg", "rough"), ("_AmbientOcclusion.jpg", "ao")):
            src = [n for n in z.namelist() if n.endswith(suffix)]
            if src:
                with open(os.path.join(TEX, "%s_%s.jpg" % (name, out)), "wb") as f:
                    f.write(z.read(src[0]))
                got.append(out)
        print("texture %s <- %s (%s): %s" % (name, asset, res, ", ".join(got)))
    with open(os.path.join(TEX, "SOURCE.txt"), "w") as f:
        f.write("ambientCG (https://ambientcg.com), CC0 1.0: tools/fetch_assets.py\n")
        for name, (asset, res) in TEXTURES.items():
            f.write("  %s_*.jpg  <-  %s %s  (https://ambientcg.com/a/%s)\n" % (name, asset, res, asset))
        f.write("sky.jpg  <-  Poly Haven %s (https://polyhaven.com/a/%s), CC0\n" % (SKY, SKY))


def fetch_sky(force):
    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None
    dst = os.path.join(TEX, "sky.jpg")
    if os.path.exists(dst) and not force:
        return
    os.makedirs(TEX, exist_ok=True)
    files = json.loads(get("https://api.polyhaven.com/files/" + SKY))
    im = Image.open(io.BytesIO(get(files["tonemapped"]["url"]))).convert("RGB").resize((4096, 2048), Image.LANCZOS)
    im.save(dst, quality=92)
    # its sun: the brightest spot (of a small copy, blurred by the resize)
    small = np.asarray(im.resize((512, 256), Image.BILINEAR)).astype(np.float32).sum(axis=2)
    y, x = np.unravel_index(np.argmax(small), small.shape)
    az, polar = (x + 0.5) / 512.0 * 2 * math.pi - math.pi, (y + 0.5) / 256.0 * math.pi
    d = (math.sin(polar) * math.cos(az), math.cos(polar), math.sin(polar) * math.sin(az))   # (the sky shader's mapping, yaw 0)
    with open(os.path.join(TEX, "sky_sun.txt"), "w") as f:
        f.write("%.5f %.5f %.5f\n" % d)
    print("sky <- %s, its sun towards (%.3f %.3f %.3f)" % ((SKY,) + d))


# ------------------------------------------------------------------------------------------------ models
COMP = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def accessor(g, bins, i):
    a = g["accessors"][i]
    bv = g["bufferViews"][a["bufferView"]]
    dt, n = np.dtype(COMP[a["componentType"]]), NCOMP[a["type"]]
    off = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
    stride = bv.get("byteStride", 0) or dt.itemsize * n
    buf = bins[bv["buffer"]]
    if stride == dt.itemsize * n:
        return np.frombuffer(buf, dt, a["count"] * n, off).reshape(a["count"], n).copy()
    out = np.zeros((a["count"], n), dt)
    for k in range(a["count"]):
        out[k] = np.frombuffer(buf, dt, n, off + k * stride)
    return out


def node_matrix(nd):
    if "matrix" in nd:
        return np.array(nd["matrix"], np.float64).reshape(4, 4).T
    m = np.eye(4)
    t, r, s = nd.get("translation", [0, 0, 0]), nd.get("rotation", [0, 0, 0, 1]), nd.get("scale", [1, 1, 1])
    x, y, z, w = r
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    m[:3, :3] = R * np.array(s)
    m[:3, 3] = t
    return m


def kdop_dirs():
    """42 directions: an icosahedron's vertices and its edges' middles"""
    t = (1 + 5 ** 0.5) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    v = [np.array(p, np.float64) / np.linalg.norm(p) for p in v]
    out = list(v)
    for i in range(12):
        for j in range(i + 1, 12):
            if abs(np.linalg.norm(v[i] - v[j]) - 2 / np.linalg.norm((1, t, 0))) < 1e-6:
                out.append((v[i] + v[j]) / np.linalg.norm(v[i] + v[j]))
    return np.array(out)


def fetch_model(mid, force):
    out_dir = os.path.join(MODELS, mid)
    if os.path.exists(os.path.join(out_dir, mid + ".blm")) and not force:
        return
    os.makedirs(out_dir, exist_ok=True)
    files = json.loads(get("https://api.polyhaven.com/files/" + mid))
    gl = files["gltf"]["1k"]["gltf"]
    g = json.loads(get(gl["url"]))
    inc = {os.path.basename(k): v["url"] for k, v in gl["include"].items()}
    bins = [get(inc[os.path.basename(b["uri"])]) for b in g["buffers"]]
    P, N, UV, I = [], [], [], []
    base = 0

    def walk(ni, M):
        nonlocal base
        nd = g["nodes"][ni]
        M = M @ node_matrix(nd)
        if "mesh" in nd:
            for prim in g["meshes"][nd["mesh"]]["primitives"]:
                at = prim["attributes"]
                p = accessor(g, bins, at["POSITION"]).astype(np.float64)
                n = accessor(g, bins, at["NORMAL"]).astype(np.float64) if "NORMAL" in at else np.zeros_like(p)
                uv = accessor(g, bins, at["TEXCOORD_0"]).astype(np.float64) if "TEXCOORD_0" in at else np.zeros((len(p), 2))
                idx = accessor(g, bins, prim["indices"]).astype(np.int64).ravel() if "indices" in prim else np.arange(len(p))
                P.append(p @ M[:3, :3].T + M[:3, 3])
                nn = n @ np.linalg.inv(M[:3, :3])
                N.append(nn / np.maximum(np.linalg.norm(nn, axis=1, keepdims=True), 1e-12))
                UV.append(uv)
                I.append(idx + base)
                base += len(p)
        for c in nd.get("children", []):
            walk(c, M)

    for root in g["scenes"][g.get("scene", 0)]["nodes"]:
        walk(root, np.eye(4))
    P, N, UV, I = np.concatenate(P), np.concatenate(N), np.concatenate(UV), np.concatenate(I)
    with open(os.path.join(out_dir, mid + ".blm"), "wb") as f:
        f.write(struct.pack("<4sII", b"BLM1", len(P), len(I)))
        f.write(np.hstack([P, N, UV]).astype("<f4").tobytes())
        f.write(I.astype("<u4").tobytes())
    for kind in ("diff", "nor_gl", "arm"):
        src = [u for k, u in inc.items() if "_%s_" % kind in k]
        if src:
            with open(os.path.join(out_dir, "%s_%s.jpg" % (mid, kind)), "wb") as f:
                f.write(get(src[0]))
    # its hull: the support plane in each of 42 directions (n . p <= d inside)
    dirs = kdop_dirs()
    with open(os.path.join(out_dir, mid + ".hull"), "w") as f:
        f.write("bounds %.4f %.4f %.4f %.4f %.4f %.4f\n" % (tuple(P.min(axis=0)) + tuple(P.max(axis=0))))
        for d in dirs:
            f.write("plane %.5f %.5f %.5f %.5f\n" % (d[0], d[1], d[2], float((P @ d).max())))
    print("model %s: %d vertices, %d triangles, %.2f x %.2f x %.2f m" % ((mid, len(P), len(I) // 3) + tuple(P.max(axis=0) - P.min(axis=0))))


def main():
    force = "--force" in sys.argv
    what = [a for a in sys.argv[1:] if not a.startswith("--")] or ["textures", "sky", "models"]
    if "textures" in what:
        fetch_textures(force)
    if "sky" in what:
        fetch_sky(force)
    if "models" in what:
        os.makedirs(MODELS, exist_ok=True)
        for mid in MODEL_IDS:
            try:
                fetch_model(mid, force)
            except Exception as e:   # (one asset gone or changed: the others still)
                print("model %s failed: %s" % (mid, e))
        with open(os.path.join(MODELS, "SOURCE.txt"), "w") as f:
            f.write("Poly Haven (https://polyhaven.com), CC0: tools/fetch_assets.py\n" + "".join("  %s  (https://polyhaven.com/a/%s)\n" % (m, m) for m in MODEL_IDS))


if __name__ == "__main__":
    main()
