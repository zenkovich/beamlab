"""Reader for Richard Burns Rally stage files (LBS geometry, TRK driveline / shape collision meshes, COL ground
collision, MAT surface maps). The layouts follow the writers of the rbr_track_formats package
(https://github.com/RichardBurnsRally/blender-track-exporter, GPLv3); this is an independent reader, no code is shared.

Coordinates: everything is returned in the exporter's Blender convention (x, y, z-up); LBS stores (x, z, y)."""
import struct
import numpy as np

# ---------------------------------------------------------------- dtypes
uv = np.dtype([("u", "<f4"), ("v", "<f4")])
v3lh = np.dtype([("x", "<f4"), ("z", "<f4"), ("y", "<f4")])  # file order: x, up, y (blender coords)
v3 = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
col = np.dtype([("b", "u1"), ("g", "u1"), ("r", "u1"), ("a", "u1")])
sway = np.dtype([("amp", "<f4"), ("freq", "<f4"), ("phase", "<f4")])
tri16 = np.dtype([("a", "<u2"), ("b", "<u2"), ("c", "<u2")])

GEOM_DTYPES = [
    np.dtype([("position", v3lh), ("color", col)]),
    np.dtype([("position", v3lh), ("color", col), ("uv1", uv)]),
    np.dtype([("position", v3lh), ("normal", v3lh), ("color", col), ("uv1", uv), ("specular_uv", uv), ("spec", "<f4")]),
    np.dtype([("position", v3lh), ("color", col), ("uv1", uv), ("shadow_uv", uv), ("shadow", "<f4")]),
    np.dtype([("position", v3lh), ("normal", v3lh), ("color", col), ("uv1", uv), ("specular_uv", uv), ("spec", "<f4"),
              ("shadow_uv", uv), ("shadow", "<f4")]),
    np.dtype([("position", v3lh), ("color", col), ("uv1", uv), ("uv2", uv)]),
    np.dtype([("position", v3lh), ("normal", v3lh), ("color", col), ("uv1", uv), ("uv2", uv), ("specular_uv", uv), ("spec", "<f4")]),
    np.dtype([("position", v3lh), ("color", col), ("uv1", uv), ("uv2", uv), ("shadow_uv", uv), ("shadow", "<f4")]),
    np.dtype([("position", v3lh), ("normal", v3lh), ("color", col), ("uv1", uv), ("uv2", uv), ("specular_uv", uv), ("spec", "<f4"),
              ("shadow_uv", uv), ("shadow", "<f4")]),
]
SWAY_DTYPES = {
    0: np.dtype([("position", v3lh), ("color", col), ("sway", sway)]),
    1: np.dtype([("position", v3lh), ("color", col), ("uv1", uv), ("sway", sway)]),
    2: np.dtype([("position", v3lh), ("color", col), ("uv1", uv), ("uv2", uv), ("sway", sway)]),
}
OBJ_DTYPES = {  # by vertex size
    16: np.dtype([("position", v3lh), ("color", col)]),
    24: np.dtype([("position", v3lh), ("color", col), ("uv1", uv)]),
    32: np.dtype([("position", v3lh), ("color", col), ("uv1", uv), ("uv2", uv)]),
    48: np.dtype([("position", v3lh), ("normal", v3lh), ("color", col), ("uv1", uv), ("specular_uv", uv), ("spec", "<f4")]),
    56: np.dtype([("position", v3lh), ("normal", v3lh), ("color", col), ("uv1", uv), ("uv2", uv), ("specular_uv", uv), ("spec", "<f4")]),
}

F_SINGLE, F_DOUBLE, F_SPEC, F_SHADER = 0x10, 0x20, 0x42, 0x80


class Reader:
    def __init__(self, data, off=0):
        self.d = data
        self.o = off

    def u(self, fmt):
        r = struct.unpack_from(fmt, self.d, self.o)
        self.o += struct.calcsize(fmt)
        return r

    def u1(self, fmt):
        return self.u(fmt)[0]

    def arr(self, dtype, n):
        a = np.frombuffer(self.d, dtype=dtype, count=n, offset=self.o)
        self.o += dtype.itemsize * n
        return a

    def cstr(self):
        e = self.d.index(b"\0", self.o)
        s = self.d[self.o:e].decode("latin-1")
        self.o = e + 1
        return s


def segments(data):
    out = {}
    o = 0
    while o + 16 <= len(data):
        _, cat, typ, ln = struct.unpack_from("<IIII", data, o)
        out[typ] = data[o + 16:o + 16 + ln]
        o += 16 + ln
    return out


def pos_of(verts):
    """blender coords (x, y, z-up) as an (n, 3) float array"""
    p = verts["position"]
    return np.stack([p["x"], p["y"], p["z"]], axis=1).astype(np.float32)


# ---------------------------------------------------------------- LBS
def read_geom_blocks(raw):
    r = Reader(raw)
    n, gloss = r.u("<If")
    chunks = []
    for _ in range(n):
        bufs = []
        for dt in GEOM_DTYPES:
            nt = r.u1("<I")
            tris = r.arr(tri16, nt // 3)
            nv = r.u1("<I")
            verts = r.arr(dt, nv)
            bufs.append((tris, verts))
        nrc = r.u1("<I")
        for _ in range(nrc):
            typ, vs, ps, ft3 = r.u("<IIII")
            ntri, nvert, fv = r.u("<III")
            bb = r.u("<6f")
            r.u("<BBBBI")
            r.u("<BBBBI")
            ntex, t1, t2 = r.u("<III")
            r.u("<BBBB")
            r.u("<6f")
            dist = r.u("<BBBB")[0]
            tris, verts = bufs[typ]
            ft = ft3 // 3
            t = tris[ft:ft + ntri]
            idx = np.stack([t["a"], t["b"], t["c"]], axis=1).astype(np.int64) - fv
            chunks.append(dict(type=typ, verts=verts[fv:fv + nvert], idx=idx, tex1=None if t1 == 0xFFFFFFFF else t1,
                               tex2=None if t2 == 0xFFFFFFFF else t2, dist=dist))
        r.u("<6f")
    return chunks


def read_object_blocks(raw):
    r = Reader(raw)
    n = r.u1("<I")
    out = []
    for _ in range(n):
        if not r.u1("<I"):
            continue
        for _half in range(2):
            nb = r.u1("<I")
            for _ in range(nb):
                flags = r.u1("<I")
                rstate = r.u1("<I")
                t1 = t2 = None
                if flags & (F_SINGLE | F_DOUBLE):
                    t1 = r.u1("<I")
                if flags & F_DOUBLE:
                    t2 = r.u1("<I")
                r.u1("<I")
                vsize = r.u1("<I")
                r.u1("<I")  # fvf
                ni = r.u1("<I")
                main = r.arr(np.dtype("<u2"), ni)
                lod = r.u1("<B")
                if lod == 2:
                    nf = r.u1("<I")
                    r.arr(np.dtype("<u2"), nf)
                kind = 2 if flags & F_DOUBLE else 1 if flags & F_SINGLE else 0
                dt = SWAY_DTYPES[kind]
                if dt.itemsize != vsize:
                    raise ValueError(f"object block vertex size {vsize} != {dt.itemsize}")
                nv = r.u1("<I")
                verts = r.arr(dt, nv)
                r.u("<6f")
                out.append(dict(verts=verts, idx=main.astype(np.int64).reshape(-1, 3), tex1=t1, tex2=t2, lod=lod,
                                no_cull=bool(rstate & 1)))
    return out


def read_interactive_objects(raw):
    r = Reader(raw)
    n = r.u1("<I")
    out = []
    for _ in range(n):
        name = r.cstr()
        kind, nd = r.u("<BI")
        datas = []
        for _ in range(nd):
            flags = r.u1("<I")
            rstate = r.u1("<I")
            t1 = t2 = None
            if flags & (F_SINGLE | F_DOUBLE):
                t1 = r.u1("<I")
                if flags & F_SHADER:
                    r.u("<ff")
            if flags & F_DOUBLE:
                t2 = r.u1("<I")
                if flags & F_SHADER:
                    r.u("<ff")
            if (flags & F_SPEC) == F_SPEC:
                r.u1("<I")
                if flags & F_SHADER:
                    r.u("<ff")
            vsize = r.u1("<I")
            r.u1("<I")
            ni = r.u1("<I")
            idx = r.arr(np.dtype("<u2"), ni).astype(np.int64).reshape(-1, 3)
            nv = r.u1("<I")
            verts = r.arr(OBJ_DTYPES[vsize], nv)
            datas.append(dict(verts=verts, idx=idx, tex1=t1, tex2=t2, no_cull=bool(rstate & 1)))
        ni = r.u1("<I")
        inst = []
        for _ in range(ni):
            key = r.u1("<I")
            m = np.array(r.u("<16f"), dtype=np.float32).reshape(4, 4)
            inst.append(m)
        out.append(dict(name=name, kind=kind, data=datas, instances=inst))
    return out


def read_car_location(raw):
    r = Reader(raw)
    m = np.array(r.u("<16f"), dtype=np.float32).reshape(4, 4)
    pos = r.u("<3f")
    euler = r.u("<3f")
    return dict(matrix=m, position=pos, euler=euler)


# ---------------------------------------------------------------- TRK
def read_driveline(raw):
    r = Reader(raw)
    n = r.u1("<I")
    pts = []
    for _ in range(n):
        p = r.u("<3f")
        d = r.u("<3f")
        loc = r.u("<fHH")[0]
        pts.append((p, d, loc))
    return pts


def read_shape_collision_meshes(raw):
    r = Reader(raw)
    n = r.u1("<I")
    out = []
    for _ in range(n):
        name = r.cstr()
        kind = r.u1("<I")
        material = r.u1("<B")
        r.u1("<B")
        vol = r.u1("<I")
        if vol == 2:
            r.u("<4f")
        elif vol == 1:
            r.u("<6f")
        nv = r.u1("<I")
        verts = np.array(r.u(f"<{3 * nv}f"), dtype=np.float32).reshape(-1, 3)
        nf = r.u1("<I")
        faces = []
        for _ in range(nf):
            r.u1("<B")
            faces.append(r.u("<III"))
        no = r.u1("<I")
        objs = []
        for _ in range(no):
            key = r.u1("<I")
            p = r.u("<3f")
            s = r.u("<3f")
            q = r.u("<4f")
            objs.append((p, s, q))
        out.append(dict(name=name, kind=kind, material=material, verts=verts, faces=faces, objects=objs))
    return out


# ---------------------------------------------------------------- COL
raw_tri = np.dtype([("a", "<u2"), ("b", "<u2"), ("c", "<u2"), ("blend", "<u2"), ("shade", "<u2"), ("m1", "u1"), ("m2", "u1"),
                    ("a_uv1", "u1"), ("b_uv1", "u1"), ("c_uv1", "u1"), ("a_uv2", "u1"), ("b_uv2", "u1"), ("c_uv2", "u1")])


def read_col(data):
    """-> list of (vertices (n,3) blender coords, raw triangles)"""
    if data[:4] != b"OC7R":
        raise ValueError("not a COL file")
    root_off, nsub, sub_off = struct.unpack_from("<III", data, 4)
    out = []
    for i in range(nsub):
        desc = struct.unpack_from("<I", data, sub_off + 4 * i)[0]
        _dt, _tt, _pad = struct.unpack_from("<BBH", data, desc)
        nv, voff, toff = struct.unpack_from("<III", data, desc + 8)
        verts = np.frombuffer(data, dtype=np.float32, count=nv * 3, offset=desc + voff).reshape(-1, 3)
        tris = []
        root = desc + toff

        def node(hdr, base):
            value, off = struct.unpack_from("<II", data, hdr + 24)
            ntri = value & 0x1FFFFF
            link = (value >> 21) & 1
            return ntri, link, off

        # root header: link bit set, offset relative to the header itself
        ntri, link, off = node(root, root)
        stack = [(root, ntri, off, True)]
        while stack:
            hdr, ntri, off, is_root = stack.pop()
            at = hdr + off if is_root else root + off
            if ntri > 0:
                tris.append(np.frombuffer(data, dtype=raw_tri, count=ntri, offset=at))
                continue
            # internal node: two child headers at `at`
            for k in range(2):
                h = at + 32 * k
                cnt, lk, co = node(h, root)
                stack.append((h, cnt, co, False))
        out.append((verts, np.concatenate(tris) if tris else np.zeros(0, raw_tri)))
    return out


# ---------------------------------------------------------------- MAT
def read_mat(data):
    r = Reader(data)
    n = r.u1("<I")
    out = {}
    for _ in range(n):
        name = r.cstr()
        nm = r.u1("<I")
        maps = []
        for _ in range(nm):
            w, h = r.u("<II")
            maps.append(np.frombuffer(r.d, dtype=np.uint8, count=w * h, offset=r.o).reshape(h, w).copy())
            r.o += w * h
        out[name] = maps
    return out


def read_brake_wall(data):
    """-> (n, 4) array of (inner x, inner y, outer x, outer y) in blender coords, or None"""
    root_off = struct.unpack_from("<I", data, 4)[0]
    bw_off = struct.unpack_from("<I", data, root_off + 20)[0]
    if bw_off == 0:
        return None
    base = root_off + bw_off
    size, npts, pp_off, tree_off = struct.unpack_from("<IIII", data, base)
    return np.frombuffer(data, dtype=np.float32, count=npts * 2, offset=base + pp_off).reshape(-1, 4).copy()
