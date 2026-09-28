#!/usr/bin/env python3
"""Download Richard Burns Rally community stages and convert them into BeamLab stage bundles
(assets/stages/<name>/stage.bin + textures).

Stages (all by RALLY Guru, https://rallyguru-tracks.blogspot.com/):
    verkiai_sss   Verkiai 2010 Super Special (Lithuania, tarmac/gravel super special in a park)
    undva         Undva (Estonia, narrow fast gravel through forest and juniper bushes)
    travanca      Travanca do Monte (Portugal, gravel through pine and eucalyptus hills)
    fernet_branca Fernet Branca 2015 (Argentina, gravel through the Cordoba hills)
Author's terms (install guide and blog): free for non-commercial use; all models and textures may be used for
non-commercial purposes (vegetation textures reduced to 1024x1024); the stages themselves must not be modified or
redistributed as modified RBR stages; keep the author's credits. The converter therefore runs locally, reduces the
vegetation textures to 1024 px and writes a SOURCE.txt with the credits next to each bundle. Do not redistribute
the bundles.

Usage:
    python3 tools/fetch_rbr_stage.py [--stage NAME] [--archive file.7z] [--src extracted_dir] [--force]

Without --stage every stage is fetched (~1.5 GB of downloads). Needs numpy, Pillow and py7zr; when they are missing
the script creates build/rbr_venv and re-runs itself there.
"""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
VENV = os.path.join(ROOT, "build", "rbr_venv")


def ensure_deps():
    try:
        import numpy  # noqa: F401
        import PIL  # noqa: F401
        import py7zr  # noqa: F401
        from PIL import Image
        if tuple(int(x) for x in Image.__version__.split(".")[:2]) < (11, 2):
            raise ImportError("Pillow >= 11.2 is needed for DDS encoding")
        return
    except ImportError as e:
        if os.environ.get("BL_RBR_VENV"):
            sys.exit(f"missing dependency inside {VENV}: {e}")
    import subprocess
    import venv
    py = os.path.join(VENV, "bin", "python3")
    if not os.path.exists(py):
        print(f"[deps] creating {VENV}")
        venv.create(VENV, with_pip=True)
    print("[deps] installing numpy, Pillow, py7zr")
    subprocess.check_call([py, "-m", "pip", "install", "-q", "--disable-pip-version-check", "numpy", "Pillow>=11.2", "py7zr"])
    env = dict(os.environ, BL_RBR_VENV="1")
    os.execve(py, [py, os.path.abspath(__file__)] + sys.argv[1:], env)


GDRIVE = "https://drive.usercontent.google.com/download?id={}&export=download&confirm=t"
STAGES = {
    "verkiai_sss": dict(
        title="Verkiai 2010 Super Special (RBR)",
        archive="verkiai_sss.7z",
        url=GDRIVE.format("1bFBuRrIWxCspdzlS-NgWrikELpr-bKEz"),
        credits=(
            "Verkiai 2010 Super Special Stage v1.0 - Richard Burns Rally community stage (Lithuania)\n"
            "Made by RALLY Guru (https://rallyguru-tracks.blogspot.com/). Textures: RALLY Guru and Ivan Novozhilov.\n"
            "Particles by Martinez. Made with Blender and Wallaby.\n\n"
            "Author's terms: free for non-commercial use. All models and textures may be used for non-commercial\n"
            "purposes (vegetation textures reduced to 1024x1024 pixels). No modification of the stage itself is\n"
            "allowed without the author's permission; keep the author's credits.\n\n"
            "Converted locally for BeamLab by tools/fetch_rbr_stage.py (vegetation textures reduced to 1024 px).\n"
            "Personal, non-commercial use only. Do not redistribute.\n"),
    ),
    "undva": dict(
        title="Undva (RBR)",
        archive="undva.7z",
        url=GDRIVE.format("0BwUDAvN91_GyR25iQnhfXzhmZ2s"),
        credits=(
            "Undva v1.2 - Richard Burns Rally community stage (Estonia, gravel)\n"
            "Made by RALLY Guru (https://rallyguru-tracks.blogspot.com/); BTB version by Kytt.\n\n"
            "Author's terms: free for non-commercial use. All models and textures may be used for non-commercial\n"
            "purposes (vegetation textures reduced to 1024x1024 pixels). No modification of the stage itself is\n"
            "allowed without the author's permission; keep the author's credits.\n\n"
            "Converted locally for BeamLab by tools/fetch_rbr_stage.py (vegetation textures reduced to 1024 px).\n"
            "Personal, non-commercial use only. Do not redistribute.\n"),
    ),
    "travanca": dict(
        title="Travanca do Monte (RBR)",
        archive="travanca.7z",
        url=GDRIVE.format("0BwUDAvN91_GyVWxhemtOZnNFU1U"),
        credits=(
            "Travanca do Monte v1.0 - Richard Burns Rally community stage (Portugal, gravel)\n"
            "Made by RALLY Guru (https://rallyguru-tracks.blogspot.com/); BTB version by Zaxxon.\n\n"
            "Author's terms: free for non-commercial use. All models and textures may be used for non-commercial\n"
            "purposes (vegetation textures reduced to 1024x1024 pixels). No modification of the stage itself is\n"
            "allowed without the author's permission; keep the author's credits.\n\n"
            "Converted locally for BeamLab by tools/fetch_rbr_stage.py (vegetation textures reduced to 1024 px).\n"
            "Personal, non-commercial use only. Do not redistribute.\n"),
    ),
    "fernet_branca": dict(
        title="Fernet Branca 2015 (RBR)",
        archive="fernet_branca.7z",
        url=GDRIVE.format("0BwUDAvN91_GyMnVUZVJHWjBXSDQ"),
        credits=(
            "Fernet Branca 2015 v1.02 - Richard Burns Rally community stage (Argentina, gravel)\n"
            "Made by RALLY Guru (https://rallyguru-tracks.blogspot.com/). Particles by Martinez.\n\n"
            "Author's terms: free for non-commercial use. All models and textures may be used for non-commercial\n"
            "purposes (vegetation textures reduced to 1024x1024 pixels). No modification of the stage itself is\n"
            "allowed without the author's permission; keep the author's credits.\n\n"
            "Converted locally for BeamLab by tools/fetch_rbr_stage.py (vegetation textures reduced to 1024 px).\n"
            "Personal, non-commercial use only. Do not redistribute.\n"),
    ),
}


# ------------------------------------------------------------------------------------------------- download
def download(url, dst):
    import urllib.request
    tmp = dst + ".part"
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req, timeout=120) as r, open(tmp, "wb") as f:
        total = int(r.headers.get("Content-Length") or 0)
        ctype = r.headers.get("Content-Type", "")
        if "text/html" in ctype:
            raise RuntimeError("the download link returned a web page (Google Drive quota or confirmation); "
                               "download the archive in a browser and pass it with --archive")
        done = 0
        while True:
            chunk = r.read(1 << 20)
            if not chunk:
                break
            f.write(chunk)
            done += len(chunk)
            if total:
                print(f"\r[get ] {done / 1e6:7.1f} / {total / 1e6:.1f} MB", end="", flush=True)
        print()
    os.replace(tmp, dst)


def pick_track(names):
    """-> (folder, track stem) of the preferred lighting variant: M(orning), O(vercast), E(vening), N(ight) last"""
    lbs = [n for n in names if n.lower().endswith(".lbs")]
    if not lbs:
        raise RuntimeError("no .lbs file in the archive")

    def rank(n):
        stem = os.path.splitext(os.path.basename(n))[0]
        suf = stem.rsplit("_", 1)[-1].upper() if "_" in stem else ""
        return ({"M": 0, "O": 1, "E": 2, "N": 9}.get(suf, 5), n)

    best = sorted(lbs, key=rank)[0]
    return os.path.dirname(best), os.path.splitext(os.path.basename(best))[0]


def extract(archive, dst):
    import py7zr
    with py7zr.SevenZipFile(archive) as z:
        names = z.getnames()
    folder, track = pick_track(names)
    prefix = folder + "/" if folder else ""
    want = [prefix + track + ext for ext in (".col", ".ini", ".lbs", ".mat", ".trk", ".dls", "_textures.rbz")]
    want = [w for w in want if w in names] + [n for n in names if n.lower().endswith((".rtf", ".txt"))]
    print(f"[7z  ] extracting {track} ({len(want)} files) from {os.path.basename(archive)}")
    with py7zr.SevenZipFile(archive) as z:
        z.extract(path=dst, targets=want)
    return os.path.join(dst, folder), track


# ------------------------------------------------------------------------------------------------- conversion
SURF_GRASS, SURF_DIRT, SURF_ASPHALT, SURF_CONCRETE, SURF_GRAVEL, SURF_MUD, SURF_SAND, SURF_ROCK, SURF_WOOD, SURF_METAL, SURF_ICE = range(11)


def surface_of(mid):
    """RBR physical material id -> BeamLab surface"""
    m = int(mid)
    if 37 <= m <= 39 or 45 <= m <= 47 or 53 <= m <= 55 or 65 <= m <= 87:
        return SURF_ASPHALT
    if 1 <= m <= 23 or 33 <= m <= 35 or 41 <= m <= 43 or 49 <= m <= 51 or 153 <= m <= 155 or 177 <= m <= 199:
        return SURF_GRAVEL
    if 97 <= m <= 99 or 105 <= m <= 115 or 24 <= m <= 32 or 165 <= m <= 168:
        return SURF_GRASS
    if 89 <= m <= 95:
        return SURF_CONCRETE
    if 101 <= m <= 103 or 116 <= m <= 119 or m == 88:
        return SURF_ICE
    if 129 <= m <= 139 or 56 <= m <= 58:
        return SURF_ROCK
    if m == 157:
        return SURF_MUD
    return SURF_DIRT


def bl(p):
    """Blender (x, y, z-up) -> BeamLab (x, y-up, z): a proper rotation, triangle winding is kept"""
    import numpy as np
    p = np.asarray(p, dtype=np.float32)
    return np.stack([p[..., 0], p[..., 2], -p[..., 1]], axis=-1)


def lh_to_bl_matrix(m):
    """RBR instance matrix (row vectors, (x, up, y) coords) -> (R, t) acting on BeamLab column vectors"""
    import numpy as np
    S = np.diag([1.0, 1.0, -1.0])
    R = S @ m[:3, :3].T @ S
    t = S @ m[3, :3]
    return R.astype(np.float32), t.astype(np.float32)


def vertex_normals(pos, idx, weld=False):
    import numpy as np
    fn = np.cross(pos[idx[:, 1]] - pos[idx[:, 0]], pos[idx[:, 2]] - pos[idx[:, 0]])
    if weld:
        q = np.round(pos * 100.0).astype(np.int64)
        _, key = np.unique(q, axis=0, return_inverse=True)
        key = key.reshape(-1)
    else:
        key = np.arange(len(pos))
    acc = np.zeros((key.max() + 1 if len(key) else 0, 3), dtype=np.float64)
    for k in range(3):
        np.add.at(acc, key[idx[:, k]], fn)
    n = acc[key]
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.where(ln > 1e-12, n / np.maximum(ln, 1e-12), np.array([0.0, 1.0, 0.0]))
    return n.astype(np.float32)


class Texture:
    def __init__(self, name, file, alpha, foliage, ground):
        self.name, self.file, self.alpha, self.foliage, self.ground = name, file, alpha, foliage, ground


def read_ini(path):
    import configparser
    cp = configparser.ConfigParser(strict=False, interpolation=None)
    cp.optionxform = str
    cp.read(path, encoding="latin-1")
    ti = cp["TextureInfo"]
    names = [ti[f"Texture{i}"] for i in range(int(ti["NumTextures"]))]
    info = {}
    for n in names:
        s = cp[n] if cp.has_section(n) else {}
        info[n] = dict(alpha=str(s.get("OpacityMap", "false")).lower() == "true",
                       ground=str(s.get("IsGroundTexture", "false")).lower() == "true")
    return names, info


def is_foliage(name):
    import re
    return re.search(r"(?i)tree|veg|grass|bush|scenary", name) is not None


def write_dds_mips(img, path, alpha, keep_coverage):
    """Encodes a full mip chain (DXT1 / DXT5) with Pillow and writes one DDS file."""
    import io
    import struct
    import numpy as np
    from PIL import Image
    fmt = "DXT5" if alpha else "DXT1"
    ref = 0.5
    a0 = np.asarray(img.getchannel("A"), dtype=np.float32) / 255.0 if keep_coverage else None
    cov0 = float((a0 > ref).mean()) if keep_coverage else 0.0
    levels, header = [], None
    cur = img
    while True:
        im = cur
        if keep_coverage and cur is not img:
            # alpha-tested foliage thins out in the smaller mips: rescale alpha to keep the coverage of level 0
            a = np.asarray(cur.getchannel("A"), dtype=np.float32) / 255.0
            lo, hi = 0.5, 4.0
            for _ in range(14):
                s = 0.5 * (lo + hi)
                if float((np.clip(a * s, 0, 1) > ref).mean()) < cov0:
                    lo = s
                else:
                    hi = s
            im = cur.copy()
            im.putalpha(Image.fromarray((np.clip(a * hi, 0, 1) * 255 + 0.5).astype(np.uint8)))
        if not alpha:
            im = im.convert("RGB")
        buf = io.BytesIO()
        im.save(buf, "DDS", pixel_format=fmt)
        data = buf.getvalue()
        if data[84:88] != fmt.encode():
            raise RuntimeError(f"unexpected DDS header from Pillow for {path}")
        if header is None:
            header = bytearray(data[:128])
        levels.append(data[128:])
        w, h = cur.size
        if min(w, h) <= 4:
            break
        cur = cur.resize((max(1, w // 2), max(1, h // 2)), Image.Resampling.BOX)
    flags = struct.unpack_from("<I", header, 8)[0] | 0x20000  # DDSD_MIPMAPCOUNT
    struct.pack_into("<I", header, 8, flags)
    struct.pack_into("<I", header, 28, len(levels))
    caps = struct.unpack_from("<I", header, 108)[0] | 0x400008  # DDSCAPS_COMPLEX | DDSCAPS_MIPMAP
    struct.pack_into("<I", header, 108, caps)
    with open(path, "wb") as f:
        f.write(header)
        for l in levels:
            f.write(l)


def convert_textures(rbz_path, names, info, out_dir):
    import re
    import zipfile
    from PIL import Image
    zf = zipfile.ZipFile(rbz_path)
    by_lower = {n.lower(): n for n in zf.namelist()}
    os.makedirs(os.path.join(out_dir, "textures"), exist_ok=True)
    texs = []
    for i, n in enumerate(names):
        cand = [p for p in by_lower if p.endswith("/dry/new/" + n.lower()) or p.endswith("/dry/" + n.lower())]
        cand.sort(key=lambda p: (not p.endswith("/new/" + n.lower()) if info[n]["ground"] else p.endswith("/new/" + n.lower())))
        fol = is_foliage(n)
        out = "textures/" + re.sub(r"[^A-Za-z0-9_.-]", "_", os.path.splitext(n)[0]) + ".dds"
        texs.append(Texture(n, out, info[n]["alpha"], fol, info[n]["ground"]))
        if not cand:
            print(f"[tex ] {i:2d} {n}: not found")
            texs[-1].file = ""
            continue
        dst = os.path.join(out_dir, out)
        if os.path.exists(dst):
            continue
        with zf.open(by_lower[cand[0]]) as f:
            img = Image.open(f)
            img.load()
        img = img.convert("RGBA")
        limit = 1024 if fol else 4096  # author's terms: vegetation textures at most 1024 x 1024
        if max(img.size) > limit:
            s = limit / max(img.size)
            img = img.resize((max(4, int(img.size[0] * s)), max(4, int(img.size[1] * s))), Image.Resampling.LANCZOS)
        write_dds_mips(img, dst, info[n]["alpha"], keep_coverage=info[n]["alpha"])
        print(f"[tex ] {i:2d} {n:40s} {img.size[0]}x{img.size[1]} {'alpha' if info[n]['alpha'] else ''}")
    return texs


def hermite(p0, m0, p1, m1, t):
    t2, t3 = t * t, t * t * t
    return ((2 * t3 - 3 * t2 + 1)[:, None] * p0 + (t3 - 2 * t2 + t)[:, None] * m0 + (-2 * t3 + 3 * t2)[:, None] * p1 +
            (t3 - t2)[:, None] * m1)


def sample_driveline(dl):
    import numpy as np
    out = []
    for i in range(len(dl) - 1):
        p0, m0, _ = dl[i]
        p1, m1, _ = dl[i + 1]
        p0, m0, p1, m1 = (np.array(v, dtype=np.float64) for v in (p0, m0, p1, m1))
        n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / 1.0)))
        t = np.linspace(0, 1, n, endpoint=False)
        out.append(hermite(p0, m0, p1, m1, t))
    out.append(np.array([dl[-1][0]], dtype=np.float64))
    return bl(np.concatenate(out))


TILE = 64  # heightfield tile size in cells (matches phys::Heightfield::kTile)


def find_overpasses(line, min_gap=2.5, reach=6.0, min_ds=30.0):
    """driveline samples that pass over another part of the stage (a figure-of-eight crossing on a bridge):
    boolean mask of the upper samples"""
    import numpy as np
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(line[:, [0, 2]], axis=0), axis=1))]
    cell = 8.0
    keys = {}
    for i, p in enumerate(line):
        keys.setdefault((int(np.floor(p[0] / cell)), int(np.floor(p[2] / cell))), []).append(i)
    upper = np.zeros(len(line), dtype=bool)
    for (kx, kz), ids in keys.items():
        near = [j for dx in (-1, 0, 1) for dz in (-1, 0, 1) for j in keys.get((kx + dx, kz + dz), [])]
        near = np.array(near)
        for i in ids:
            d = np.linalg.norm(line[near][:, [0, 2]] - line[i, [0, 2]], axis=1)
            other = near[(d < reach) & (np.abs(s[near] - s[i]) > min_ds)]
            if len(other) and (line[i, 1] - line[other, 1]).max() > min_gap:
                upper[i] = True
    return upper


def deck_boxes(line, need, bridge, half_width=3.8, thick=0.35):
    """static boxes that carry the road where the heightfield cannot: along the driveline samples in `need`
    (1 m apart, overlapping); on real overpasses (`bridge`) also a 0.9 m parapet on both sides"""
    import numpy as np
    out = []
    grow = np.zeros(len(line), dtype=bool)
    for i in np.nonzero(need)[0]:
        grow[max(0, i - 2):min(len(line), i + 3)] = True
    for i in np.nonzero(grow)[0]:
        a, b = line[max(0, i - 1)], line[min(len(line) - 1, i + 1)]
        t = b - a
        t /= max(np.linalg.norm(t), 1e-6)
        lat = np.cross(np.array([0.0, 1.0, 0.0]), t)
        lat /= max(np.linalg.norm(lat), 1e-6)
        up = np.cross(t, lat)
        R = np.stack([lat, up, t], axis=1)  # columns: box x (across), y (up), z (along)
        out.append((line[i] - up * thick, np.array([half_width, thick, 0.8]), R, 2))
        if bridge[i]:
            for side in (-1.0, 1.0):
                c = line[i] + lat * (side * (half_width + 0.12)) + up * 0.45
                # (not on the lane of another pass over the same bridge)
                if np.min(np.linalg.norm(line[bridge][:, [0, 2]] - c[[0, 2]], axis=1)) < half_width - 0.3:
                    continue
                out.append((c, np.array([0.12, 0.45, 0.8]), R, 0))
    return out


def carve_under_decks(decks, tiles, TH, cell, ox, oz, clearance=0.8, margin=0.4):
    """keeps the heightfield at least `clearance` below every deck box (walls of a tunnel under the bridge would
    otherwise poke up to the deck surface: nodes riding on the deck hit their almost vertical faces)"""
    import numpy as np
    T = TH.shape[1]
    index = {(int(a), int(b)): k for k, (a, b) in enumerate(tiles)}
    carved = 0
    for (centre, half, R, kind) in decks:
        if kind != 2:
            continue
        top = centre + R[:, 1] * half[1]
        ext = np.abs(R) @ (half + margin)
        x0, x1 = int(np.floor((centre[0] - ext[0] - ox) / cell)), int(np.ceil((centre[0] + ext[0] - ox) / cell))
        z0, z1 = int(np.floor((centre[2] - ext[2] - oz) / cell)), int(np.ceil((centre[2] + ext[2] - oz) / cell))
        for iz in range(z0, z1 + 1):
            for ix in range(x0, x1 + 1):
                k = index.get((ix // T, iz // T))
                if k is None:
                    continue
                p = np.array([ox + ix * cell, 0.0, oz + iz * cell])
                l = R.T @ (p - centre)
                if abs(l[0]) > half[0] + margin or abs(l[2]) > half[2] + margin:
                    continue
                limit = top[1] - clearance
                if TH[k, iz % T, ix % T] > limit:
                    TH[k, iz % T, ix % T] = limit
                    carved += 1
    return carved


def floating_driveline(line, tiles, TH, cell, ox, oz, lateral=2.5, drop=1.0):
    """driveline samples where the heightfield (centre or +-lateral) lies more than `drop` below the road"""
    import numpy as np
    T = TH.shape[1]
    index = {(int(a), int(b)): k for k, (a, b) in enumerate(tiles)}

    def h(x, z):
        ix, iz = int(round((x - ox) / cell)), int(round((z - oz) / cell))
        k = index.get((ix // T, iz // T))
        return None if k is None else float(TH[k, iz % T, ix % T])

    need = np.zeros(len(line), dtype=bool)
    for i, p in enumerate(line):
        a, b = line[max(0, i - 1)], line[min(len(line) - 1, i + 1)]
        t = b - a
        t = t / max(np.linalg.norm(t[[0, 2]]), 1e-6)
        l = np.array([t[2], 0.0, -t[0]])
        for off in (-lateral, 0.0, lateral):
            q = p + l * off
            g = h(q[0], q[2])
            if g is None or g < p[1] - drop:
                need[i] = True
                break
    return need


def driveline_height_grid(line, ox, oz, k_cell, nkx, nkz, radius=20.0):
    """coarse grid (k_cell metres) of the driveline height near the road: (height, valid) per cell"""
    import numpy as np
    H = np.zeros((nkz, nkx), dtype=np.float32)
    D = np.full((nkz, nkx), np.inf, dtype=np.float32)
    r = int(np.ceil(radius / k_cell))
    for p in line[::2]:
        cx, cz = int((p[0] - ox) / k_cell), int((p[2] - oz) / k_cell)
        x0, x1, z0, z1 = max(0, cx - r), min(nkx, cx + r + 1), max(0, cz - r), min(nkz, cz + r + 1)
        if x0 >= x1 or z0 >= z1:
            continue
        gx = ox + (np.arange(x0, x1) + 0.5) * k_cell
        gz = oz + (np.arange(z0, z1) + 0.5) * k_cell
        d = np.sqrt((gx[None, :] - p[0]) ** 2 + (gz[:, None] - p[2]) ** 2).astype(np.float32)
        sub = D[z0:z1, x0:x1]
        better = d < sub
        sub[better] = d[better]
        H[z0:z1, x0:x1][better] = p[1]
    return H, D <= radius


def rasterize_ground(col, maps, line, max_cells=14e6):
    """COL ground triangles -> sparse heightfield: TILE x TILE tiles along the collision mesh (top surface) with the
    stage's surfaces. The cell is 0.25 m unless the collision mesh is so large that it would need more than
    max_cells cells (long stages). Returns (cell, (ox, oz), (nx, nz), tile coords, tile heights, tile surfaces)."""
    import numpy as np
    lut = np.array([surface_of(i) for i in range(256)], dtype=np.uint8)
    allv = np.concatenate([bl(v) for v, _ in col])
    mn, mx = allv.min(0), allv.max(0)
    # projected area of the collision mesh -> cell size
    area = 0.0
    for verts, tris in col:
        if len(tris):
            V = bl(verts).astype(np.float64)
            A, B, C = V[tris["a"].astype(np.int64)], V[tris["b"].astype(np.int64)], V[tris["c"].astype(np.int64)]
            area += 0.5 * np.abs((B[:, 0] - A[:, 0]) * (C[:, 2] - A[:, 2]) - (C[:, 0] - A[:, 0]) * (B[:, 2] - A[:, 2])).sum()
    cell = 0.25
    while area / (cell * cell) > max_cells:
        cell = round(cell + 0.05, 2)
    margin = 12.0
    ox, oz = np.floor(mn[0] - margin), np.floor(mn[2] - margin)
    nx = int(np.ceil((mx[0] + margin - ox) / cell)) + 1
    nz = int(np.ceil((mx[2] + margin - oz) / cell)) + 1
    cells, hs, ss = [], [], []
    maps = np.stack(maps) if maps else np.zeros((1, 16, 16), np.uint8)
    for verts, tris in col:
        if len(tris) == 0:
            continue
        V = bl(verts).astype(np.float64)
        ia, ib, ic = tris["a"].astype(np.int64), tris["b"].astype(np.int64), tris["c"].astype(np.int64)
        A, B, C = V[ia], V[ib], V[ic]
        # vertex attributes: 5-bit blending (material 2 weight), 4-bit packed material uvs
        blend = tris["blend"].astype(np.int64)
        bw = np.stack([(blend >> s) & 31 for s in (0, 5, 10)], axis=1) / 31.0
        uv1 = np.stack([tris[k] for k in ("a_uv1", "b_uv1", "c_uv1")], axis=1).astype(np.int64)
        uv2 = np.stack([tris[k] for k in ("a_uv2", "b_uv2", "c_uv2")], axis=1).astype(np.int64)
        m1 = np.minimum(tris["m1"].astype(np.int64), len(maps) - 1)
        m2 = np.minimum(tris["m2"].astype(np.int64), len(maps) - 1)
        x0 = np.ceil((np.minimum(np.minimum(A[:, 0], B[:, 0]), C[:, 0]) - ox) / cell).astype(np.int64)
        x1 = np.floor((np.maximum(np.maximum(A[:, 0], B[:, 0]), C[:, 0]) - ox) / cell).astype(np.int64)
        z0 = np.ceil((np.minimum(np.minimum(A[:, 2], B[:, 2]), C[:, 2]) - oz) / cell).astype(np.int64)
        z1 = np.floor((np.maximum(np.maximum(A[:, 2], B[:, 2]), C[:, 2]) - oz) / cell).astype(np.int64)
        ext = np.maximum(x1 - x0, z1 - z0) + 1
        tri_area = (B[:, 0] - A[:, 0]) * (C[:, 2] - A[:, 2]) - (C[:, 0] - A[:, 0]) * (B[:, 2] - A[:, 2])
        ok = (np.abs(tri_area) > 1e-9) & (x1 >= x0) & (z1 >= z0)
        for K in (2, 4, 8, 16, 32, 64, 128, 256, 1024):
            sel = np.nonzero(ok & (ext <= K) & (ext > K // 2 if K > 2 else ext <= K))[0]
            if len(sel) == 0:
                continue
            for s0 in range(0, len(sel), max(1, 400000 // (K * K))):
                s = sel[s0:s0 + max(1, 400000 // (K * K))]
                gx = x0[s][:, None, None] + np.arange(K)[None, None, :]
                gz = z0[s][:, None, None] + np.arange(K)[None, :, None]
                px = ox + gx * cell
                pz = oz + gz * cell
                a, b, c = A[s], B[s], C[s]
                ar = tri_area[s][:, None, None]
                w0 = ((b[:, 0, None, None] - px) * (c[:, 2, None, None] - pz) - (c[:, 0, None, None] - px) * (b[:, 2, None, None] - pz)) / ar
                w1 = ((c[:, 0, None, None] - px) * (a[:, 2, None, None] - pz) - (a[:, 0, None, None] - px) * (c[:, 2, None, None] - pz)) / ar
                w2 = 1.0 - w0 - w1
                inside = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6) & (gx <= x1[s][:, None, None]) & (gz <= z1[s][:, None, None])
                ti, zz, xx = np.nonzero(inside)
                W0, W1, W2 = w0[ti, zz, xx], w1[ti, zz, xx], w2[ti, zz, xx]
                t = s[ti]
                h = W0 * A[t, 1] + W1 * B[t, 1] + W2 * C[t, 1]
                bwt = W0 * bw[t, 0] + W1 * bw[t, 1] + W2 * bw[t, 2]
                use2 = bwt > 0.5
                uvp = np.where(use2[:, None], uv2[t], uv1[t])
                u = W0 * (uvp[:, 0] >> 4) + W1 * (uvp[:, 1] >> 4) + W2 * (uvp[:, 2] >> 4)
                v = W0 * (uvp[:, 0] & 15) + W1 * (uvp[:, 1] & 15) + W2 * (uvp[:, 2] & 15)
                col_ = np.clip((u / 15.0 * 16).astype(np.int64), 0, 15)
                row = np.clip((v / 15.0 * 16).astype(np.int64), 0, 15)
                mid = maps[np.where(use2, m2[t], m1[t]), row, col_]
                cells.append((gz[ti, zz, 0] * nx + gx[ti, 0, xx]).astype(np.int64))
                hs.append(h.astype(np.float32))
                ss.append(lut[mid])
    cells = np.concatenate(cells)
    hs = np.concatenate(hs)
    ss = np.concatenate(ss)
    order = np.lexsort((hs, cells))
    cells, hs, ss = cells[order], hs[order], ss[order]
    # one layer per cell: the top surface, except where the road passes under something (a bridge deck 2 m or more
    # above the driveline): there the highest layer near the driveline height
    starts = np.r_[0, np.nonzero(cells[1:] != cells[:-1])[0] + 1]
    top = np.r_[starts[1:], len(cells)] - 1
    k_cell = 2.0
    nkx, nkz = int(np.ceil(nx * cell / k_cell)) + 1, int(np.ceil(nz * cell / k_cell)) + 1
    HR, VR = driveline_height_grid(line, ox, oz, k_cell, nkx, nkz)
    ucx, ucz = cells[starts] % nx, cells[starts] // nx
    ki, kj = np.minimum((ucz * cell / k_cell).astype(np.int64), nkz - 1), np.minimum((ucx * cell / k_cell).astype(np.int64), nkx - 1)
    yref, valid = HR[ki, kj], VR[ki, kj]
    yref_s = np.repeat(yref, np.diff(np.r_[starts, len(cells)]))
    cand = np.where(hs <= yref_s + 1.5, np.arange(len(cells)), -1)
    best_low = np.maximum.reduceat(cand, starts)
    under = valid & (hs[top] - yref > 2.0) & (best_low >= 0)
    pick = np.where(under, best_low, top)
    print(f"[col ] {int(under.sum())} cells under bridges / overhangs use the lower layer")
    cells, hs, ss = cells[pick], hs[pick], ss[pick]
    cx, cz = cells % nx, cells // nx

    # ---- tiles: every tile with collision cells, plus a ring of neighbours for the margin and the walls
    T = TILE
    ntx, ntz = (nx + T - 1) // T, (nz + T - 1) // T
    occ = np.zeros((ntz + 2, ntx + 2), dtype=bool)
    occ[cz // T + 1, cx // T + 1] = True
    ring = occ.copy()
    for dz in (-1, 0, 1):
        for dx in (-1, 0, 1):
            ring[1 + dz:ntz + 1 + dz, 1 + dx:ntx + 1 + dx] |= occ[1:ntz + 1, 1:ntx + 1]
    ring[0, :] = ring[-1, :] = False
    ring[:, 0] = ring[:, -1] = False
    tz_, tx_ = np.nonzero(ring)
    tz_, tx_ = tz_ - 1, tx_ - 1
    nt = len(tz_)
    index = np.full((ntz + 2, ntx + 2), nt, dtype=np.int64)  # nt = blank tile
    index[tz_ + 1, tx_ + 1] = np.arange(nt)
    TH = np.full((nt + 1, T, T), np.nan, dtype=np.float32)
    TS = np.full((nt + 1, T, T), SURF_GRASS, dtype=np.uint8)
    ti = index[cz // T + 1, cx // T + 1]
    TH[ti, cz % T, cx % T] = hs
    TS[ti, cz % T, cx % T] = ss
    covered_cells = len(cells)

    # ---- fill: gaps next to the collision mesh get the average of their neighbours; RBR has no ground where the
    # collision mesh ends (the car falls through), so beyond 1.5 m a 3 m step walls those areas off
    P = int(np.ceil(2.0 / cell))
    W = T + 2 * P
    F = np.full((nt, W, W), np.nan, dtype=np.float32)
    FS = np.full((nt, W, W), SURF_GRASS, dtype=np.uint8)
    spans = {-1: (T - P, T, 0, P), 0: (0, T, P, P + T), 1: (0, P, P + T, W)}
    for dz in (-1, 0, 1):
        sz0, sz1, dz0, dz1 = spans[dz]
        for dx in (-1, 0, 1):
            sx0, sx1, dx0, dx1 = spans[dx]
            nb = index[tz_ + 1 + dz, tx_ + 1 + dx]
            F[:, dz0:dz1, dx0:dx1] = TH[nb, sz0:sz1, sx0:sx1]
            FS[:, dz0:dz1, dx0:dx1] = TS[nb, sz0:sz1, sx0:sx1]
    dist = np.where(np.isnan(F), 255, 0).astype(np.uint8)
    for k in range(1, P + 1):
        filled = dist < 255
        acc = np.zeros_like(F)
        cnt = np.zeros(F.shape, dtype=np.float32)
        srf = np.full(F.shape, 255, dtype=np.uint8)
        Fz = np.where(filled, F, 0.0)
        for sl_dst, sl_src in (((slice(None), slice(1, None), slice(None)), (slice(None), slice(None, -1), slice(None))),
                               ((slice(None), slice(None, -1), slice(None)), (slice(None), slice(1, None), slice(None))),
                               ((slice(None), slice(None), slice(1, None)), (slice(None), slice(None), slice(None, -1))),
                               ((slice(None), slice(None), slice(None, -1)), (slice(None), slice(None), slice(1, None)))):
            acc[sl_dst] += Fz[sl_src]
            cnt[sl_dst] += filled[sl_src]
            srf[sl_dst] = np.where((srf[sl_dst] == 255) & filled[sl_src], FS[sl_src], srf[sl_dst])
        new = ~filled & (cnt > 0)
        F[new] = acc[new] / cnt[new]
        FS[new] = srf[new]
        dist[new] = k
    F, FS, dist = F[:, P:P + T, P:P + T], FS[:, P:P + T, P:P + T], dist[:, P:P + T, P:P + T]
    keep = (dist < 255).any(axis=(1, 2))
    wall = (dist < 255) & (dist.astype(np.float32) * cell > 1.5)
    F[wall] += 3.0
    FS[wall] = SURF_ROCK
    tile_max = np.where(dist < 255, F, -np.inf).max(axis=(1, 2))
    far = dist == 255
    F = np.where(far, (tile_max + 3.0)[:, None, None], F).astype(np.float32)
    FS[far] = SURF_ROCK
    tiles = np.stack([tx_, tz_], axis=1)[keep]
    F, FS = F[keep], FS[keep]
    print(f"[col ] heightfield {nx} x {nz} @ {cell} m: {len(tiles)} tiles of {T}x{T} "
          f"({len(tiles) * T * T / 1e6:.1f} M cells, {covered_cells / 1e6:.1f} M covered)")
    return cell, (float(ox), float(oz)), (nx, nz), tiles.astype(np.int32), F, FS


def read_dls_events(path, length):
    """(start, finish) driveline locations from the pacenotes (EVENT_START 21 / EVENT_FINISH 22: id, flags, location)
    in the stage's DLS file; the records are found by scanning for a consistent run of pacenotes."""
    import struct
    if not os.path.exists(path):
        return -1.0, -1.0
    d = open(path, "rb").read()

    def rec(o):
        if o < 0 or o + 12 > len(d):
            return None
        i, f, loc = struct.unpack_from("<IIf", d, o)
        return (i, f, loc) if i < 256 and f < 0x10000 and -1.0 <= loc <= length + 1.0 else None

    start, finish = -1.0, -1.0
    for o in range(0, len(d) - 12, 4):
        r = rec(o)
        if not r or r[0] not in (21, 22):
            continue
        prev, nxt = rec(o - 12), rec(o + 12)
        in_run = (prev is not None and prev[2] <= r[2]) or (nxt is not None and nxt[2] >= r[2])
        if not in_run:
            continue
        if r[0] == 21 and start < 0:
            start = r[2]
        if r[0] == 22:
            finish = max(finish, r[2])
    return start, finish


def convert(src, track, out_dir, title, credits):
    import struct
    import numpy as np
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import rbr_formats as rf
    base = os.path.join(src, track)
    os.makedirs(out_dir, exist_ok=True)
    names, info = read_ini(base + ".ini")
    texs = convert_textures(base + "_textures.rbz", names, info, out_dir)

    seg = rf.segments(open(base + ".lbs", "rb").read())
    geom = rf.read_geom_blocks(seg[0x0])
    objs = rf.read_object_blocks(seg[0x1]) if 0x1 in seg else []
    ios = rf.read_interactive_objects(seg[0x6]) if 0x6 in seg else []
    trk = rf.segments(open(base + ".trk", "rb").read())
    driveline = rf.read_driveline(trk[0x14])
    scm = rf.read_shape_collision_meshes(trk[0x16]) if 0x16 in trk else []
    coldata = open(base + ".col", "rb").read()
    col = rf.read_col(coldata)
    mat = rf.read_mat(open(base + ".mat", "rb").read())
    cond = next((k for k in mat if "dry" in k and "new" in k), next(iter(mat)))

    batches = []  # (kind, flags, tex1, tex2, pos, nrm, uv1, uv2, rgba, idx)

    def rgba_of(verts):
        c = verts["color"]
        return np.stack([c["r"], c["g"], c["b"], c["a"]], axis=1).astype(np.uint8)

    def uv_of(verts, key):
        if key not in verts.dtype.names:
            return np.zeros((len(verts), 2), np.float32)
        return np.stack([verts[key]["u"], verts[key]["v"]], axis=1).astype(np.float32)

    # ground: normals welded across chunk seams
    gpos = [bl(rf.pos_of(c["verts"])) for c in geom]
    allp = np.concatenate(gpos)
    offs = np.cumsum([0] + [len(p) for p in gpos])
    alli = np.concatenate([c["idx"] + offs[k] for k, c in enumerate(geom)])
    gn = vertex_normals(allp, alli, weld=True)
    up_ratio = float((gn[:, 1] > 0).mean())
    flip = up_ratio < 0.5
    print(f"[lbs ] ground: {len(geom)} chunks, {len(alli)} triangles, normals up {up_ratio * 100:.0f}%{' (flipped winding)' if flip else ''}")
    if flip:
        gn = -gn
    for k, c in enumerate(geom):
        idx = c["idx"][:, ::-1] if flip else c["idx"]
        batches.append((0, 0, c["tex1"], c["tex2"], gpos[k], gn[offs[k]:offs[k + 1]], uv_of(c["verts"], "uv1"),
                        uv_of(c["verts"], "uv2"), rgba_of(c["verts"]), idx))

    def tex_ok(t):
        return t is not None and t < len(texs) and texs[t].file

    # object blocks: vegetation (alpha tested cards, up-facing normals like grass) and solid objects
    ntri_obj = 0
    for o in objs:
        p = bl(rf.pos_of(o["verts"]))
        idx = o["idx"][:, ::-1] if flip else o["idx"]
        t1 = o["tex1"]
        fol = tex_ok(t1) and texs[t1].foliage
        n = vertex_normals(p, idx)
        if fol:
            n = n * 0.35 + np.array([0, 1, 0], np.float32)
            n /= np.linalg.norm(n, axis=1, keepdims=True)
        alpha = tex_ok(t1) and texs[t1].alpha
        kind = 2 if alpha else 1
        flags = (1 if (o["no_cull"] or fol) else 0)
        batches.append((kind, flags, t1, None, p, n, uv_of(o["verts"], "uv1"), uv_of(o["verts"], "uv1"), rgba_of(o["verts"]), idx))
        ntri_obj += len(idx)
    print(f"[lbs ] objects: {len(objs)} blocks, {ntri_obj} triangles")

    # interactive objects: matched with shape collision meshes -> physical props; round bales -> BeamLab bales
    inst = []  # (io index, R, t)
    for i, io in enumerate(ios):
        for m in io["instances"]:
            R, t = lh_to_bl_matrix(m)
            inst.append((i, R, t))
    used = [False] * len(inst)
    bales = []
    for k, (i, R, t) in enumerate(inst):
        if ios[i]["name"].lower().startswith("rulonas"):
            p = np.concatenate([rf.pos_of(d["verts"]) for d in ios[i]["data"]])
            radius = float(np.median(np.linalg.norm(p[:, :2] - p[:, :2].mean(0), axis=1)))
            height = float(p[:, 2].max() - p[:, 2].min())
            yaw = float(np.degrees(np.arctan2(R[0, 2], R[0, 0])))
            bales.append((t, yaw, radius * 1.02, height))
            used[k] = True
    templates, props = {}, []
    for s in scm:
        if s["kind"] == 0 or s["name"].lower().startswith("rulonas") or len(s["verts"]) == 0:
            continue
        hull = bl(s["verts"])
        for (p, _scale, _q) in s["objects"]:
            pb = bl(np.array(p))
            best, bd = -1, 0.35
            for k, (i, R, t) in enumerate(inst):
                d = float(np.linalg.norm(t - pb))
                if not used[k] and d < bd:
                    best, bd = k, d
            if best < 0:
                continue
            used[best] = True
            i, R, t = inst[best]
            templates.setdefault(i, (s["kind"], hull.min(0), hull.max(0)))
            props.append((i, R, t))
    # masses of RBR's dynamic object kinds (kind 2 "traffic pig" = heavy concrete blocks and boulders)
    mass_of = {1: 5.0, 2: 2500.0, 3: 12.0, 6: 45.0, 7: 33.0, 8: 120.0, 9: 60.0, 10: 90.0}

    # static shape collision meshes (tree trunks, stumps, walls, bush columns) -> oriented static boxes; bendable
    # trees and bushes (material 121) yield, stumps / walls / trunks are solid; spectators (200) are ignored
    SOLID = {56, 57, 58, 59, 60, 61, 62, 63, 64, 120, 30, 31, 32}
    YIELD = {121, 24, 25, 26, 27, 28, 29, 165, 166, 167, 168}
    M = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)  # blender -> BeamLab
    statics = []
    for sm in scm:
        if sm["kind"] != 0 or len(sm["verts"]) == 0 or not (sm["material"] in SOLID or sm["material"] in YIELD):
            continue
        hv = sm["verts"].astype(np.float64)
        c_loc, h_loc = (hv.min(0) + hv.max(0)) * 0.5, (hv.max(0) - hv.min(0)) * 0.5
        for (p, sc, q) in sm["objects"]:
            x, y, z, w = q
            Rb = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                           [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                           [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            scv = np.array(sc, dtype=np.float64)
            centre = M @ (np.array(p) + Rb @ (c_loc * scv))
            half = np.abs(h_loc * scv)
            # local box axes: blender local (x, y, z) -> BeamLab local (x, z, -y) so that local "up" stays y
            R = M @ Rb @ M.T
            half_bl = np.array([half[0], half[2], half[1]])
            statics.append((centre, np.maximum(half_bl, 0.03), R, 1 if sm["material"] in YIELD else 0))
    print(f"[trk ] {len(statics)} static colliders ({sum(1 for x in statics if x[3])} yielding)")

    def io_meshes(i):
        out = []
        for d in ios[i]["data"]:
            if not tex_ok(d["tex1"]):
                continue
            p = bl(rf.pos_of(d["verts"]))
            idx = d["idx"][:, ::-1] if flip else d["idx"]
            if "normal" in d["verts"].dtype.names:
                nv = d["verts"]["normal"]
                n = bl(np.stack([nv["x"], nv["y"], nv["z"]], axis=1))
            else:
                n = vertex_normals(p, idx)
            out.append((d["tex1"], d["no_cull"], p, n, uv_of(d["verts"], "uv1"), uv_of(d["verts"], "uv2"), rgba_of(d["verts"]), idx))
        return out

    # everything else is static scenery
    static_io = {}
    for k, (i, R, t) in enumerate(inst):
        if used[k]:
            continue
        for (t1, nc, p, n, u1, u2, c, idx) in io_meshes(i):
            key = (t1, nc)
            wp = p @ R.T + t
            wn = n @ R.T
            static_io.setdefault(key, []).append((wp, wn, u1, u2, c, idx))
    for (t1, nc), parts in static_io.items():
        offs2 = np.cumsum([0] + [len(x[0]) for x in parts])
        alpha = texs[t1].alpha
        batches.append((2 if alpha else 1, 1 if nc else 0, t1, None, np.concatenate([x[0] for x in parts]),
                        np.concatenate([x[1] for x in parts]), np.concatenate([x[2] for x in parts]),
                        np.concatenate([x[2] for x in parts]), np.concatenate([x[4] for x in parts]),
                        np.concatenate([x[5] + offs2[j] for j, x in enumerate(parts)])))
    print(f"[io  ] {len(inst)} instances: {len(props)} physical props, {len(bales)} bales, {sum(1 for u in used if not u)} static")

    # ground collision
    line = sample_driveline(driveline)
    upper = find_overpasses(line)
    # the heightfield keeps the layer the road uses underneath; where the road then floats above it (the bridge of
    # a figure-of-eight, gaps in the collision mesh) static deck boxes carry it
    cell, (ox, oz), (nx, nz), tiles, TH, TS = rasterize_ground(col, mat[cond], line[~upper])
    need = floating_driveline(line, tiles, TH, cell, ox, oz) | upper
    if need.any():
        decks = deck_boxes(line, need, upper)
        statics.extend(decks)
        print(f"[col ] {carve_under_decks(decks, tiles, TH, cell, ox, oz)} cells lowered under the deck")
        print(f"[trk ] {int(upper.sum())} driveline samples on an overpass, {int(need.sum())} above the heightfield -> {len(decks)} deck boxes")
    start = line[0]
    fwd = line[min(8, len(line) - 1)] - line[0]
    heading = float(np.degrees(np.arctan2(fwd[0], fwd[2])))
    dl_len = float(driveline[-1][2])
    ev_start, ev_finish = read_dls_events(base + ".dls", dl_len)
    print(f"[dls ] start {ev_start:.1f} m, finish {ev_finish:.1f} m of {dl_len:.1f} m")

    # --------------------------------------------------------------------------------------- write
    def s16(b, s):
        e = s.encode("utf-8")
        b.append(struct.pack("<H", len(e)) + e)

    b = [b"BLSTAGE1", struct.pack("<I", 4)]
    s16(b, title)
    s16(b, credits)
    b.append(struct.pack("<I", len(texs)))
    for t in texs:
        s16(b, t.file)
        b.append(struct.pack("<B", (1 if t.alpha else 0) | (2 if t.foliage else 0) | (4 if t.ground else 0)))
    b.append(struct.pack("<IIfffII", nx, nz, cell, ox, oz, TILE, len(tiles)))  # v3: sparse tiles
    b.append(tiles.astype("<i4").tobytes())
    b.append(TH.astype("<f4").tobytes())
    b.append(TS.astype(np.uint8).tobytes())
    b.append(struct.pack("<I", len(batches)))
    vdt = np.dtype([("p", "<f4", 3), ("n", "<f4", 3), ("uv1", "<f4", 2), ("uv2", "<f4", 2), ("c", "u1", 4)])
    for (kind, flags, t1, t2, p, n, u1, u2, c, idx) in batches:
        v = np.zeros(len(p), vdt)
        v["p"], v["n"], v["uv1"], v["uv2"], v["c"] = p, n, u1, u2, c
        b.append(struct.pack("<BBiiII", kind, flags, -1 if t1 is None else t1, -1 if t2 is None else t2, len(v), idx.size))
        b.append(v.tobytes())
        b.append(idx.astype("<u4").tobytes())
    # prop templates (meshes in the prop's local frame) and instances
    tmpl_ids = {}
    b.append(struct.pack("<I", len(templates)))
    for i, (kind, hmin, hmax) in templates.items():
        tmpl_ids[i] = len(tmpl_ids)
        s16(b, ios[i]["name"])
        b.append(struct.pack("<If3f3f", kind, mass_of.get(kind, 50.0), *hmin, *hmax))
        ms = io_meshes(i)
        b.append(struct.pack("<I", len(ms)))
        for (t1, nc, p, n, u1, u2, c, idx) in ms:
            v = np.zeros(len(p), vdt)
            v["p"], v["n"], v["uv1"], v["uv2"], v["c"] = p, n, u1, u2, c
            b.append(struct.pack("<BBiiII", 2 if texs[t1].alpha else 1, 1 if nc else 0, t1, -1, len(v), idx.size))
            b.append(v.tobytes())
            b.append(idx.astype("<u4").tobytes())
    b.append(struct.pack("<I", len(props)))
    for (i, R, t) in props:
        b.append(struct.pack("<I9f3f", tmpl_ids[i], *R.reshape(-1), *t))
    b.append(struct.pack("<I", len(bales)))
    for (t, yaw, r, h) in bales:
        b.append(struct.pack("<3ffff", *t, yaw, r, h))
    b.append(struct.pack("<I", len(statics)))  # v4
    for (centre, half, R, soft) in statics:
        b.append(struct.pack("<3f3f9fB", *centre, *half, *R.reshape(-1), soft))
    b.append(struct.pack("<If", len(line), dl_len))
    b.append(line.astype("<f4").tobytes())
    b.append(struct.pack("<3ff", *start, heading))
    b.append(struct.pack("<ff", ev_start, ev_finish))  # v2: clock (driveline locations, < 0 unknown)
    with open(os.path.join(out_dir, "stage.bin"), "wb") as f:
        for x in b:
            f.write(x)
    with open(os.path.join(out_dir, "SOURCE.txt"), "w") as f:
        f.write(credits)
    size = os.path.getsize(os.path.join(out_dir, "stage.bin"))
    print(f"[out ] {out_dir}/stage.bin ({size / 1e6:.1f} MB), driveline {dl_len:.0f} m, {len(batches)} render batches")


def main():
    ensure_deps()
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", choices=sorted(STAGES), help="one stage (default: all)")
    ap.add_argument("--archive", help="already downloaded stage archive (.7z), with --stage")
    ap.add_argument("--src", help="already extracted stage folder (contains track-*.lbs), with --stage")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    for name in [a.stage] if a.stage else list(STAGES):
        st = STAGES[name]
        out_dir = os.path.join(ROOT, "assets", "stages", name)
        if os.path.exists(os.path.join(out_dir, "stage.bin")) and not a.force:
            print(f"[skip] {name}: {out_dir}/stage.bin exists (--force to rebuild)")
            continue
        print(f"==== {name}: {st['title']}")
        src = a.src if a.stage else None
        if src:
            track = pick_track([f for f in os.listdir(src)])[1]
        else:
            dl_dir = os.path.join(ROOT, "build", "downloads")
            os.makedirs(dl_dir, exist_ok=True)
            archive = (a.archive if a.stage else None) or os.path.join(dl_dir, st["archive"])
            if not os.path.exists(archive):
                print(f"[get ] {st['url']}")
                download(st["url"], archive)
            src, track = extract(archive, os.path.join(dl_dir, name))
        convert(src, track, out_dir, st["title"], st["credits"])


if __name__ == "__main__":
    main()
