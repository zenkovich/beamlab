// OGRE binary mesh reader.
// Walks the chunk stream the way OGRE's MeshSerializerImpl does: known chunks are read field by field (old
// exporters wrote unreliable chunk lengths), lengths are only used to skip data we don't need.
// Every read is bounds-checked; corrupt files fail with an error message instead of crashing.
#include "vehicle/ogre_mesh.h"

#include "core/util.h"

#include <cmath>
#include <cstring>
#include <unordered_map>

namespace bl {

namespace {

enum : uint16_t {
    M_HEADER = 0x1000,
    M_MESH = 0x3000,
    M_SUBMESH = 0x4000,
    M_SUBMESH_OPERATION = 0x4010,
    M_SUBMESH_BONE_ASSIGNMENT = 0x4100,
    M_SUBMESH_TEXTURE_ALIAS = 0x4200,
    M_GEOMETRY = 0x5000,
    M_GEOMETRY_VERTEX_DECLARATION = 0x5100, // legacy (<= v1.20): M_GEOMETRY_NORMALS
    M_GEOMETRY_VERTEX_ELEMENT = 0x5110,
    M_GEOMETRY_VERTEX_BUFFER = 0x5200,      // legacy: M_GEOMETRY_COLOURS
    M_GEOMETRY_VERTEX_BUFFER_DATA = 0x5210,
    M_GEOMETRY_TEXCOORDS = 0x5300,          // legacy only
    M_MESH_SKELETON_LINK = 0x6000,
    M_MESH_BONE_ASSIGNMENT = 0x7000,
    M_MESH_LOD = 0x8000,
    M_MESH_LOD_USAGE = 0x8100,
    M_MESH_LOD_MANUAL = 0x8110,
    M_MESH_LOD_GENERATED = 0x8120,
    M_MESH_BOUNDS = 0x9000,
    M_SUBMESH_NAME_TABLE = 0xA000,
    M_SUBMESH_NAME_TABLE_ELEMENT = 0xA100,
    M_EDGE_LISTS = 0xB000,
    M_POSES = 0xC000,
    M_ANIMATIONS = 0xD000,
    M_TABLE_EXTREMES = 0xE000,
};
constexpr uint32_t kChunkHeader = 6; // uint16 id + uint32 length (including the header)

// Format generations; only the differences that matter for reading are distinguished.
enum Ver { V1_1, V1_2, V1_3, V1_4, V1_41, V1_8, V1_10 };

enum : uint16_t { VES_POSITION = 1, VES_NORMAL = 4, VES_TEXTURE_COORDINATES = 7 };
enum : uint16_t { OT_TRIANGLE_LIST = 4, OT_TRIANGLE_STRIP = 5, OT_TRIANGLE_FAN = 6 };

struct Reader {
    const uint8_t* p = nullptr;
    size_t size = 0, pos = 0;
    bool big = false;  // big-endian file
    bool fail = false; // sticky: set by any out-of-bounds read

    size_t left() const { return pos < size ? size - pos : 0; }
    bool has(size_t n) const { return !fail && n <= left(); }
    bool skip(size_t n) {
        if (!has(n)) return fail = true, false;
        pos += n;
        return true;
    }
    uint32_t load(size_t n) { // unsigned integer of n (<= 4) bytes in file byte order
        if (!has(n)) { fail = true; return 0; }
        uint32_t v = 0;
        for (size_t i = 0; i < n; i++) v |= (uint32_t)p[pos + (big ? n - 1 - i : i)] << (8 * i);
        pos += n;
        return v;
    }
    uint16_t u16() { return (uint16_t)load(2); }
    uint32_t u32() { return load(4); }
    bool boolean() { return load(1) != 0; }
    float f32() {
        uint32_t u = load(4);
        float f;
        std::memcpy(&f, &u, 4);
        return f;
    }
    // '\n'-terminated string (OGRE DataStream::getLine).
    std::string str() {
        if (fail) return {};
        const uint8_t* b = p + pos;
        const void* nl = std::memchr(b, '\n', left());
        if (!nl) { fail = true; return {}; }
        size_t n = (const uint8_t*)nl - b;
        pos += n + 1;
        while (n > 0 && b[n - 1] == '\r') n--;
        return std::string((const char*)b, n);
    }
    // Chunk header; false at the end of the data.
    bool chunk(uint16_t& id, uint32_t& len) {
        if (fail || left() < kChunkHeader) return false;
        id = u16();
        len = u32();
        return true;
    }
    void backpedal() { pos -= kChunkHeader; }
    // Skips the rest of a chunk whose header was just read (start = header position).
    bool skip_chunk(size_t start, uint32_t len) {
        if (len < kChunkHeader || start + len > size) return fail = true, false;
        pos = start + len;
        return true;
    }
};

// ------------------------------------------------------------------ vertex data decoding

// Byte size of a VertexElementType (0 = unknown).
int elem_size(uint16_t t) {
    if (t <= 3) return 4 * (t + 1);                  // FLOAT1..4
    if (t >= 5 && t <= 8) return 2 * (t - 4);        // SHORT1..4
    if (t >= 12 && t <= 15) return 8 * (t - 11);     // DOUBLE1..4
    if (t >= 16 && t <= 19) return 2 * (t - 15);     // USHORT1..4
    if (t >= 20 && t <= 27) return 4 * ((t - 20) % 4 + 1); // (U)INT1..4
    if (t >= 36 && t <= 39) return 2 * (t - 35);     // HALF1..4
    switch (t) {
    case 4: case 9: case 10: case 11: case 28: case 29: case 30: case 31: case 33: case 35: return 4;
    case 32: case 34: return 8;
    }
    return 0;
}

float half_to_float(uint16_t h) {
    uint32_t s = (h >> 15) & 1, e = (h >> 10) & 31, m = h & 1023;
    float v;
    if (e == 0) v = std::ldexp((float)m, -24);
    else if (e == 31) v = m ? NAN : INFINITY;
    else v = std::ldexp((float)(m | 1024), (int)e - 25);
    return s ? -v : v;
}

// Decodes one element into up to 4 floats; returns the component count (0 = unsupported type).
int decode_elem(const uint8_t* s, uint16_t t, bool big, float out[4]) {
    auto rd = [&](int off, int n) {
        uint64_t v = 0;
        for (int i = 0; i < n; i++) v |= (uint64_t)s[off + (big ? n - 1 - i : i)] << (8 * i);
        return v;
    };
    auto f32 = [&](int off) { uint32_t u = (uint32_t)rd(off, 4); float f; std::memcpy(&f, &u, 4); return f; };
    auto f64 = [&](int off) { uint64_t u = rd(off, 8); double d; std::memcpy(&d, &u, 8); return (float)d; };
    int n = 0;
    if (t <= 3) {
        n = t + 1;
        for (int i = 0; i < n; i++) out[i] = f32(4 * i);
    } else if (t >= 5 && t <= 8) {
        n = t - 4;
        for (int i = 0; i < n; i++) out[i] = (float)(int16_t)rd(2 * i, 2);
    } else if (t >= 12 && t <= 15) {
        n = t - 11;
        for (int i = 0; i < n; i++) out[i] = f64(8 * i);
    } else if (t >= 16 && t <= 19) {
        n = t - 15;
        for (int i = 0; i < n; i++) out[i] = (float)(uint16_t)rd(2 * i, 2);
    } else if (t >= 20 && t <= 23) {
        n = t - 19;
        for (int i = 0; i < n; i++) out[i] = (float)(int32_t)rd(4 * i, 4);
    } else if (t >= 24 && t <= 27) {
        n = t - 23;
        for (int i = 0; i < n; i++) out[i] = (float)(uint32_t)rd(4 * i, 4);
    } else if (t >= 36 && t <= 39) {
        n = t - 35;
        for (int i = 0; i < n; i++) out[i] = half_to_float((uint16_t)rd(2 * i, 2));
    } else {
        switch (t) {
        case 4: case 9: case 10: case 11: case 30: // colours / UBYTE4(_NORM)
            n = 4;
            for (int i = 0; i < 4; i++) out[i] = s[i] * (t == 9 ? 1.0f : 1.0f / 255.0f);
            break;
        case 28: case 29: // BYTE4(_NORM)
            n = 4;
            for (int i = 0; i < 4; i++) out[i] = (float)(int8_t)s[i] * (t == 29 ? 1.0f / 127.0f : 1.0f);
            break;
        case 31: case 32: // SHORT2/4_NORM
            n = t == 31 ? 2 : 4;
            for (int i = 0; i < n; i++) out[i] = std::max(-1.0f, (float)(int16_t)rd(2 * i, 2) / 32767.0f);
            break;
        case 33: case 34: // USHORT2/4_NORM
            n = t == 33 ? 2 : 4;
            for (int i = 0; i < n; i++) out[i] = (float)(uint16_t)rd(2 * i, 2) / 65535.0f;
            break;
        case 35: { // INT_10_10_10_2_NORM (x in the low bits)
            uint32_t v = (uint32_t)rd(0, 4);
            n = 4;
            for (int i = 0; i < 3; i++) {
                int32_t c = (int32_t)((v >> (10 * i)) & 1023);
                if (c & 512) c -= 1024;
                out[i] = std::max(-1.0f, c / 511.0f);
            }
            out[3] = (float)(int32_t)(v >> 30);
            break;
        }
        }
    }
    for (int i = 0; i < n; i++)
        if (!std::isfinite(out[i])) out[i] = 0.0f;
    return n;
}

struct Geometry {
    uint32_t count = 0;
    std::vector<vec3> pos, nrm; // nrm empty = not present
    std::vector<vec2> uv;       // empty = not present
};

struct VertexElem {
    uint16_t source, type, semantic, offset, index;
};
struct VertexBuf {
    uint16_t bind, stride;
    size_t data; // file offset of count * stride bytes
};

// Legacy (v1.10 / v1.20) geometry: positions inline, then optional normal / colour / texcoord chunks.
bool read_geometry_legacy(Reader& r, Ver ver, Geometry& g, std::string& err) {
    auto read_vec3s = [&](std::vector<vec3>& dst) {
        if (!r.has((size_t)g.count * 12)) return false;
        dst.resize(g.count);
        for (auto& v : dst) {
            v.x = r.f32(), v.y = r.f32(), v.z = r.f32();
            for (int k = 0; k < 3; k++)
                if (!std::isfinite(v[k])) v[k] = 0;
        }
        return true;
    };
    if (!read_vec3s(g.pos)) return err = "vertex positions exceed file size", false;
    uint16_t id;
    uint32_t len;
    while (r.chunk(id, len)) {
        if (id == 0x5100) { // normals
            if (!read_vec3s(g.nrm)) return err = "vertex normals exceed file size", false;
        } else if (id == 0x5200) { // colours (RGBA8)
            if (!r.skip((size_t)g.count * 4)) return err = "vertex colours exceed file size", false;
        } else if (id == M_GEOMETRY_TEXCOORDS) {
            uint16_t dim = r.u16();
            if (dim < 1 || dim > 4 || !r.has((size_t)g.count * dim * 4)) return err = "bad texcoord set", false;
            bool first = g.uv.empty();
            if (first) g.uv.resize(g.count);
            for (uint32_t i = 0; i < g.count; i++) {
                float c[4] = {0, 0, 0, 0};
                for (int k = 0; k < dim; k++) c[k] = r.f32();
                if (!first) continue;
                float v = dim >= 2 ? c[1] : 0.0f;
                if (ver == V1_1 && dim == 2) v = 1.0f - v; // v1.10 stored flipped V
                g.uv[i] = {std::isfinite(c[0]) ? c[0] : 0.0f, std::isfinite(v) ? v : 0.0f};
            }
        } else {
            r.backpedal();
            break;
        }
    }
    return !r.fail || (err = "truncated geometry", false);
}

bool read_geometry(Reader& r, Ver ver, Geometry& g, std::string& err) {
    g.count = r.u32();
    if (r.fail) return err = "truncated geometry", false;
    if (ver <= V1_2) return read_geometry_legacy(r, ver, g, err);

    std::vector<VertexElem> elems;
    std::vector<VertexBuf> bufs;
    uint16_t id;
    uint32_t len;
    while (r.chunk(id, len)) {
        if (id == M_GEOMETRY_VERTEX_DECLARATION) {
            while (r.chunk(id, len)) {
                if (id != M_GEOMETRY_VERTEX_ELEMENT) {
                    r.backpedal();
                    break;
                }
                VertexElem e{r.u16(), r.u16(), r.u16(), r.u16(), r.u16()}; // braced init: evaluated in order
                elems.push_back(e);
            }
        } else if (id == M_GEOMETRY_VERTEX_BUFFER) {
            VertexBuf b{r.u16(), r.u16(), 0};
            uint16_t did;
            uint32_t dlen;
            if (!r.chunk(did, dlen) || did != M_GEOMETRY_VERTEX_BUFFER_DATA)
                return err = "missing vertex buffer data", false;
            size_t bytes = (size_t)g.count * b.stride;
            b.data = r.pos;
            if (!r.skip(bytes)) return err = "vertex buffer exceeds file size", false;
            bufs.push_back(b);
        } else {
            r.backpedal();
            break;
        }
    }
    if (r.fail) return err = "truncated vertex declaration", false;

    // Pick the lowest-index element of each semantic we use.
    const VertexElem* sel[3] = {nullptr, nullptr, nullptr}; // position, normal, uv
    for (auto& e : elems) {
        int k = e.semantic == VES_POSITION ? 0 : e.semantic == VES_NORMAL ? 1 : e.semantic == VES_TEXTURE_COORDINATES ? 2 : -1;
        if (k < 0 || elem_size(e.type) == 0) continue;
        if (!sel[k] || e.index < sel[k]->index) sel[k] = &e;
    }
    if (g.count == 0) return true;
    if (!sel[0]) return err = "geometry has no position element", false;

    for (int k = 0; k < 3; k++) {
        const VertexElem* e = sel[k];
        if (!e) continue;
        const VertexBuf* b = nullptr;
        for (auto& vb : bufs)
            if (vb.bind == e->source) { b = &vb; break; }
        if (!b || e->offset + elem_size(e->type) > b->stride) {
            if (k == 0) return err = "position element has no valid vertex buffer", false;
            continue; // ignore a broken normal / uv stream
        }
        if (k == 0) g.pos.resize(g.count);
        if (k == 1) g.nrm.resize(g.count);
        if (k == 2) g.uv.resize(g.count);
        for (uint32_t i = 0; i < g.count; i++) {
            float c[4] = {0, 0, 0, 0};
            decode_elem(r.p + b->data + (size_t)i * b->stride + e->offset, e->type, r.big, c);
            if (k == 0) g.pos[i] = {c[0], c[1], c[2]};
            else if (k == 1) g.nrm[i] = {c[0], c[1], c[2]};
            else g.uv[i] = {c[0], c[1]};
        }
    }
    return true;
}

// ------------------------------------------------------------------ mesh structure

struct RawSubmesh {
    std::string material;
    bool shared = true;
    uint16_t op = OT_TRIANGLE_LIST;
    std::vector<uint32_t> indices;
    Geometry geom;
};

bool read_submesh(Reader& r, Ver ver, RawSubmesh& sm, std::string& err) {
    sm.material = r.str();
    sm.shared = r.boolean();
    uint32_t n = r.u32();
    bool idx32 = r.boolean();
    if (r.fail || !r.has((size_t)n * (idx32 ? 4 : 2))) return err = "submesh index data exceeds file size", false;
    sm.indices.resize(n);
    for (auto& i : sm.indices) i = idx32 ? r.u32() : r.u16();

    uint16_t id;
    uint32_t len;
    if (!sm.shared) {
        if (!r.chunk(id, len) || id != M_GEOMETRY) return err = "submesh without geometry", false;
        if (!read_geometry(r, ver, sm.geom, err)) return false;
    }
    while (r.chunk(id, len)) {
        if (id == M_SUBMESH_OPERATION) {
            sm.op = r.u16();
        } else if (id == M_SUBMESH_BONE_ASSIGNMENT) {
            r.skip(10); // uint32 vertex, uint16 bone, float weight
        } else if (id == M_SUBMESH_TEXTURE_ALIAS) {
            r.str(); // alias name
            r.str(); // texture name
        } else {
            r.backpedal();
            break;
        }
    }
    return !r.fail || (err = "truncated submesh", false);
}

// Walks an M_MESH_LOD chunk by content (its length field is wrong in some old files).
bool walk_lod(Reader& r, Ver ver, size_t num_sub) {
    uint16_t id;
    uint32_t len;
    if (ver >= V1_10) {
        r.str(); // strategy
        uint16_t n = r.u16();
        for (int l = 1; l < n && !r.fail; l++) {
            if (!r.chunk(id, len)) return false;
            r.f32(); // user value
            if (id == M_MESH_LOD_MANUAL) {
                r.str();
            } else if (id == M_MESH_LOD_GENERATED) {
                for (size_t s = 0; s < num_sub && !r.fail; s++) {
                    r.u32(); // index count
                    r.u32(); // offset
                    if (r.u32() == 0xFFFFFFFFu) { // own buffer
                        bool i32 = r.boolean();
                        uint32_t cnt = r.u32();
                        r.skip((size_t)cnt * (i32 ? 4 : 2));
                    }
                }
            } else {
                return false;
            }
        }
        return !r.fail;
    }
    if (ver >= V1_8) r.str(); // strategy
    uint16_t n = r.u16();
    bool manual = r.boolean();
    for (int l = 1; l < n && !r.fail; l++) {
        if (!r.chunk(id, len) || id != M_MESH_LOD_USAGE) return false;
        r.f32(); // distance
        if (manual) {
            if (!r.chunk(id, len) || id != M_MESH_LOD_MANUAL) return false;
            r.str();
        } else {
            for (size_t s = 0; s < num_sub && !r.fail; s++) {
                if (!r.chunk(id, len) || id != M_MESH_LOD_GENERATED) return false;
                uint32_t cnt = r.u32();
                bool i32 = r.boolean();
                r.skip((size_t)cnt * (i32 ? 4 : 2));
            }
        }
    }
    return !r.fail;
}

bool is_mesh_level_chunk(uint16_t id) {
    switch (id) {
    case M_GEOMETRY: case M_SUBMESH: case M_MESH_SKELETON_LINK: case M_MESH_BONE_ASSIGNMENT: case M_MESH_LOD:
    case M_MESH_BOUNDS: case M_SUBMESH_NAME_TABLE: case M_EDGE_LISTS: case M_POSES: case M_ANIMATIONS:
    case M_TABLE_EXTREMES:
        return true;
    }
    return false;
}

struct RawMesh {
    Geometry shared;
    std::vector<RawSubmesh> subs;
    std::vector<std::string> sub_names; // from the name table (indexed by submesh)
    std::string warning;                 // non-fatal problem (e.g. unreadable trailing data)
};

bool read_mesh(Reader& r, Ver ver, RawMesh& m, std::string& err) {
    r.boolean(); // skeletally animated
    uint16_t id;
    uint32_t len;
    while (r.chunk(id, len)) {
        size_t start = r.pos - kChunkHeader;
        switch (id) {
        case M_GEOMETRY:
            if (!read_geometry(r, ver, m.shared, err)) return false;
            break;
        case M_SUBMESH:
            m.subs.emplace_back();
            if (!read_submesh(r, ver, m.subs.back(), err)) return err = format("submesh %zu: %s", m.subs.size() - 1, err.c_str()), false;
            break;
        case M_MESH_SKELETON_LINK:
            r.str();
            break;
        case M_MESH_BONE_ASSIGNMENT:
            r.skip(10);
            break;
        case M_MESH_LOD:
            if (!walk_lod(r, ver, m.subs.size())) {
                r.fail = false;
                r.skip_chunk(start, len);
            }
            break;
        case M_MESH_BOUNDS:
            r.skip(7 * 4); // min, max, radius (recomputed from the vertices)
            break;
        case M_SUBMESH_NAME_TABLE:
            while (r.chunk(id, len)) {
                if (id != M_SUBMESH_NAME_TABLE_ELEMENT) {
                    r.backpedal();
                    break;
                }
                uint16_t idx = r.u16();
                std::string name = r.str();
                if (!r.fail && idx < m.subs.size()) {
                    if (m.sub_names.size() < m.subs.size()) m.sub_names.resize(m.subs.size());
                    m.sub_names[idx] = name;
                }
            }
            break;
        default: // edge lists, poses, animations, extremes: not needed
            if (!is_mesh_level_chunk(id)) {
                r.backpedal();
                return true;
            }
            r.skip_chunk(start, len);
            break;
        }
        if (r.fail) {
            // Everything after the submeshes is optional: keep what we have.
            if (m.subs.empty()) return err = format("corrupt chunk 0x%04x", id), false;
            m.warning = format("stopped at corrupt chunk 0x%04x", id);
            return true;
        }
    }
    return true;
}

// ------------------------------------------------------------------ conversion

struct PosKey {
    uint32_t x, y, z;
    bool operator==(const PosKey& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct PosKeyHash {
    size_t operator()(const PosKey& k) const { return (k.x * 73856093u) ^ (k.y * 19349663u) ^ (k.z * 83492791u); }
};

// Area-weighted smooth normals; vertices at identical positions share one normal (smooth across UV seams).
void generate_normals(OgreSubmesh& sm) {
    size_t nv = sm.vertices.size();
    std::vector<uint32_t> canon(nv);
    std::unordered_map<PosKey, uint32_t, PosKeyHash> first;
    first.reserve(nv);
    for (size_t i = 0; i < nv; i++) {
        vec3 p = sm.vertices[i].pos + vec3(0.0f); // +0 folds -0 into 0
        PosKey k;
        std::memcpy(&k.x, &p.x, 4), std::memcpy(&k.y, &p.y, 4), std::memcpy(&k.z, &p.z, 4);
        canon[i] = first.emplace(k, (uint32_t)i).first->second;
    }
    std::vector<vec3> acc(nv, vec3(0.0f));
    for (size_t t = 0; t + 2 < sm.indices.size(); t += 3) {
        uint32_t a = sm.indices[t], b = sm.indices[t + 1], c = sm.indices[t + 2];
        vec3 n = cross(sm.vertices[b].pos - sm.vertices[a].pos, sm.vertices[c].pos - sm.vertices[a].pos);
        acc[canon[a]] += n, acc[canon[b]] += n, acc[canon[c]] += n;
    }
    for (size_t i = 0; i < nv; i++) sm.vertices[i].normal = normalize_or(acc[canon[i]], vec3(0, 1, 0));
}

bool convert(const RawMesh& raw, OgreMesh& out, std::string& err) {
    std::vector<uint32_t> remap, tri;
    for (size_t si = 0; si < raw.subs.size(); si++) {
        const RawSubmesh& rs = raw.subs[si];
        const Geometry& g = rs.shared ? raw.shared : rs.geom;
        OgreSubmesh sm;
        sm.material = rs.material;
        sm.has_normals = !g.nrm.empty();
        sm.has_uvs = !g.uv.empty();

        // Index list (non-indexed submeshes draw the vertices in order).
        const std::vector<uint32_t>* src = &rs.indices;
        std::vector<uint32_t> seq;
        if (rs.indices.empty()) {
            seq.resize(g.count);
            for (uint32_t i = 0; i < g.count; i++) seq[i] = i;
            src = &seq;
        }
        const std::vector<uint32_t>& ix = *src;

        // Triangulate; points and lines produce nothing.
        tri.clear();
        auto emit = [&](uint32_t a, uint32_t b, uint32_t c) {
            if (a >= g.count || b >= g.count || c >= g.count || a == b || b == c || a == c) return;
            tri.push_back(a), tri.push_back(b), tri.push_back(c);
        };
        if (rs.op == OT_TRIANGLE_LIST) {
            for (size_t i = 0; i + 2 < ix.size(); i += 3) emit(ix[i], ix[i + 1], ix[i + 2]);
        } else if (rs.op == OT_TRIANGLE_STRIP) {
            for (size_t i = 0; i + 2 < ix.size(); i++) {
                if (i & 1) emit(ix[i + 1], ix[i], ix[i + 2]);
                else emit(ix[i], ix[i + 1], ix[i + 2]);
            }
        } else if (rs.op == OT_TRIANGLE_FAN) {
            for (size_t i = 1; i + 1 < ix.size(); i++) emit(ix[0], ix[i], ix[i + 1]);
        }

        // Copy the referenced vertices (compacts shared geometry to what this submesh uses).
        remap.assign(g.count, UINT32_MAX);
        sm.indices.reserve(tri.size());
        for (uint32_t v : tri) {
            if (remap[v] == UINT32_MAX) {
                remap[v] = (uint32_t)sm.vertices.size();
                MeshVertex mv;
                mv.pos = g.pos[v];
                mv.normal = sm.has_normals ? normalize_or(g.nrm[v], vec3(0, 1, 0)) : vec3(0, 1, 0);
                mv.uv = sm.has_uvs ? g.uv[v] : vec2(0, 0);
                sm.vertices.push_back(mv);
            }
            sm.indices.push_back(remap[v]);
        }
        if (!sm.has_normals) generate_normals(sm);
        for (auto& v : sm.vertices) out.bounds.add(v.pos);
        out.submeshes.push_back(std::move(sm));
    }
    if (out.submeshes.empty()) return err = "mesh has no submeshes", false;
    return true;
}

} // namespace

bool load_ogre_mesh(const std::string& path, OgreMesh& out, std::string* error) {
    out = OgreMesh();
    std::string err;
    auto fail = [&](const std::string& msg) {
        if (error) *error = msg;
        return false;
    };
    std::vector<uint8_t> data;
    if (!read_file(path, data)) return fail("cannot read file");
    if (data.size() < 2) return fail("file too small");

    Reader r;
    r.p = data.data();
    r.size = data.size();
    if (data[0] == 0x00 && data[1] == 0x10) r.big = false;
    else if (data[0] == 0x10 && data[1] == 0x00) r.big = true;
    else return fail("not an OGRE mesh (bad header)");
    r.pos = 2;
    std::string ver_str = r.str();
    if (r.fail) return fail("truncated header");

    static const struct {
        const char* s;
        Ver v;
    } versions[] = {
        {"[MeshSerializer_v1.100]", V1_10}, {"[MeshSerializer_v1.8]", V1_8},   {"[MeshSerializer_v1.41]", V1_41},
        {"[MeshSerializer_v1.40]", V1_4},   {"[MeshSerializer_v1.30]", V1_3},  {"[MeshSerializer_v1.20]", V1_2},
        {"[MeshSerializer_v1.10]", V1_1},
    };
    Ver ver = V1_8;
    bool known = false;
    for (auto& v : versions)
        if (ver_str == v.s) ver = v.v, known = true;
    if (!known) {
        if (!starts_with_ci(ver_str, "[MeshSerializer_v")) return fail("unknown header '" + ver_str.substr(0, 40) + "'");
        err = "unknown version " + ver_str + ", reading as v1.8"; // best effort
    }
    out.version = ver_str.size() > 18 ? ver_str.substr(17, ver_str.size() - 18) : ver_str; // "1.41"

    RawMesh raw;
    bool got_mesh = false;
    uint16_t id;
    uint32_t len;
    while (r.chunk(id, len)) {
        size_t start = r.pos - kChunkHeader;
        if (id == M_MESH) {
            std::string merr;
            if (!read_mesh(r, ver, raw, merr)) return fail(merr);
            got_mesh = true;
            // Unknown trailing chunk inside M_MESH: continue after the M_MESH chunk if its length is sane.
            if (!r.fail && r.pos < start + len && start + len <= r.size && raw.warning.empty()) r.pos = start + len;
            if (!raw.warning.empty()) break;
        } else if (!r.skip_chunk(start, len)) {
            break;
        }
    }
    if (!got_mesh) return fail("no M_MESH chunk");
    if (!convert(raw, out, err)) return fail(err);
    return true;
}

} // namespace bl
