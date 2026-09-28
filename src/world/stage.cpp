#include "world/stage.h"
#include "core/profiler.h"
#include "core/util.h"
#include "gfx/texture.h"
#include "phys/static_world.h"

#include <cstring>

namespace bl {

namespace {

struct Cursor {
    const std::vector<uint8_t>& d;
    size_t o = 0;
    bool ok = true;
    bool need(size_t n) {
        if (!ok || o + n > d.size()) ok = false;
        return ok;
    }
    template <class T> T get() {
        T v{};
        if (need(sizeof(T))) std::memcpy(&v, d.data() + o, sizeof(T));
        o += sizeof(T);
        return v;
    }
    template <class T> void array(std::vector<T>& out, size_t n) {
        out.resize(n);
        if (need(n * sizeof(T)) && n) std::memcpy(out.data(), d.data() + o, n * sizeof(T));
        o += n * sizeof(T);
    }
    std::string str() {
        uint16_t n = get<uint16_t>();
        std::string s;
        if (need(n)) s.assign((const char*)d.data() + o, n);
        o += n;
        return s;
    }
};

#pragma pack(push, 1)
struct FileVertex {
    float p[3], n[3], uv1[2], uv2[2];
    uint8_t c[4];
};
#pragma pack(pop)
static_assert(sizeof(FileVertex) == 44, "stage vertex layout");

bool read_batch(Cursor& c, StageBundle::Batch& b) {
    b.kind = c.get<uint8_t>();
    b.flags = c.get<uint8_t>();
    b.tex1 = c.get<int32_t>();
    b.tex2 = c.get<int32_t>();
    uint32_t nv = c.get<uint32_t>(), ni = c.get<uint32_t>();
    std::vector<FileVertex> fv;
    c.array(fv, nv);
    c.array(b.idx, ni);
    if (!c.ok) return false;
    b.v.resize(nv);
    for (uint32_t i = 0; i < nv; i++) {
        const FileVertex& f = fv[i];
        VertexC& v = b.v[i];
        v.pos = vec3(f.p[0], f.p[1], f.p[2]);
        v.normal = vec3(f.n[0], f.n[1], f.n[2]);
        v.uv = vec2(f.uv1[0], f.uv1[1]);
        v.uv2 = vec2(f.uv2[0], f.uv2[1]);
        v.color = (uint32_t)f.c[0] | ((uint32_t)f.c[1] << 8) | ((uint32_t)f.c[2] << 16) | ((uint32_t)f.c[3] << 24);
    }
    for (uint32_t i : b.idx)
        if (i >= nv) return false;
    return true;
}

} // namespace

bool StageBundle::load(const std::string& path, std::string& error) {
    PROFILE_ZONE("Stage load");
    std::vector<uint8_t> data;
    if (!read_file(path, data)) {
        error = "missing " + path;
        return false;
    }
    Cursor c{data};
    if (data.size() < 12 || std::memcmp(data.data(), "BLSTAGE1", 8) != 0) {
        error = "not a stage bundle: " + path;
        return false;
    }
    c.o = 8;
    const uint32_t version = c.get<uint32_t>();
    if (version != 4) {
        error = "old stage bundle format: re-run  python3 tools/fetch_rbr_stage.py --force";
        return false;
    }
    title = c.str();
    credits = c.str();
    textures.resize(c.get<uint32_t>());
    for (auto& t : textures) {
        t.file = c.str();
        t.flags = c.get<uint8_t>();
    }
    nx = (int)c.get<uint32_t>();
    nz = (int)c.get<uint32_t>();
    cell = c.get<float>();
    origin.x = c.get<float>();
    origin.y = c.get<float>();
    tile = (int)c.get<uint32_t>();
    uint32_t nt = c.get<uint32_t>();
    std::vector<int32_t> tp;
    c.array(tp, (size_t)nt * 2);
    tiles.resize(nt);
    for (uint32_t i = 0; i < nt && c.ok; i++) tiles[i] = {tp[i * 2], tp[i * 2 + 1]};
    c.array(tile_height, (size_t)nt * tile * tile);
    c.array(tile_surface, (size_t)nt * tile * tile);
    batches.resize(c.get<uint32_t>());
    for (auto& b : batches)
        if (!read_batch(c, b)) break;
    templates.resize(c.ok ? c.get<uint32_t>() : 0);
    for (auto& t : templates) {
        t.name = c.str();
        t.kind = c.get<uint32_t>();
        t.mass = c.get<float>();
        for (int k = 0; k < 3; k++) t.hull_min[k] = c.get<float>();
        for (int k = 0; k < 3; k++) t.hull_max[k] = c.get<float>();
        t.meshes.resize(c.get<uint32_t>());
        for (auto& m : t.meshes)
            if (!read_batch(c, m)) break;
        if (!c.ok) break;
    }
    props.resize(c.ok ? c.get<uint32_t>() : 0);
    for (auto& p : props) {
        p.tmpl = (int)c.get<uint32_t>();
        float r[9];
        for (float& x : r) x = c.get<float>();
        // rows of the rotation matrix -> columns
        p.rot = transpose(mat3(vec3(r[0], r[1], r[2]), vec3(r[3], r[4], r[5]), vec3(r[6], r[7], r[8])));
        for (int k = 0; k < 3; k++) p.pos[k] = c.get<float>();
        if (p.tmpl < 0 || p.tmpl >= (int)templates.size()) c.ok = false;
    }
    bales.resize(c.ok ? c.get<uint32_t>() : 0);
    for (auto& b : bales) {
        for (int k = 0; k < 3; k++) b.pos[k] = c.get<float>();
        b.yaw = c.get<float>();
        b.radius = c.get<float>();
        b.height = c.get<float>();
    }
    statics.resize(c.ok ? c.get<uint32_t>() : 0);
    for (auto& st : statics) {
        for (int k = 0; k < 3; k++) st.centre[k] = c.get<float>();
        for (int k = 0; k < 3; k++) st.half[k] = c.get<float>();
        float r[9];
        for (float& x : r) x = c.get<float>();
        st.rot = transpose(mat3(vec3(r[0], r[1], r[2]), vec3(r[3], r[4], r[5]), vec3(r[6], r[7], r[8]))); // rows -> columns
        uint8_t kind = c.get<uint8_t>(); // 0 solid, 1 yielding, 2 bridge deck
        st.yielding = kind == 1;
        st.deck = kind == 2;
    }
    uint32_t nl = c.ok ? c.get<uint32_t>() : 0;
    line_length = c.get<float>();
    std::vector<float> lf;
    c.array(lf, (size_t)nl * 3);
    line.resize(nl);
    for (uint32_t i = 0; i < nl && c.ok; i++) line[i] = vec3(lf[i * 3], lf[i * 3 + 1], lf[i * 3 + 2]);
    for (int k = 0; k < 3; k++) start[k] = c.get<float>();
    heading = c.get<float>();
    clock_start = c.get<float>();
    clock_finish = c.get<float>();
    if (!c.ok || nx < 2 || nz < 2 || line.size() < 2 || tile != phys::Heightfield::kTile) {
        error = "truncated or corrupt stage bundle: " + path;
        return false;
    }
    return true;
}

MaterialPtr StageScenery::material(const StageBundle& b, const StageBundle::Batch& batch) {
    uint64_t key = ((uint64_t)(uint32_t)(batch.tex1 + 1) << 32) ^ ((uint64_t)(uint32_t)(batch.tex2 + 1) << 12) ^ ((uint64_t)batch.kind << 4) ^ batch.flags;
    for (auto& m : m_mats)
        if (m.first == key) return m.second;
    auto tex = [&](int i) -> TexturePtr { return i >= 0 && i < (int)m_tex.size() ? m_tex[(size_t)i] : nullptr; };
    auto m = std::make_shared<Material>();
    m->vertex_color = true;
    m->diffuse = tex(batch.tex1);
    uint8_t tf = batch.tex1 >= 0 && batch.tex1 < (int)b.textures.size() ? b.textures[(size_t)batch.tex1].flags : 0;
    m->name = batch.tex1 >= 0 && batch.tex1 < (int)b.textures.size() ? b.textures[(size_t)batch.tex1].file : "stage";
    m->double_sided = (batch.flags & 1) != 0;
    if (batch.kind == StageBundle::GROUND) {
        // (Wallaby stages often repeat the first layer as the second one: then a single texture fetch will do)
        m->diffuse2 = batch.tex2 >= 0 && batch.tex2 != batch.tex1 ? tex(batch.tex2) : nullptr;
        m->specular = 0.06f;
        m->gloss = 10.0f;
    } else if (batch.kind == StageBundle::ALPHA) {
        m->alpha_ref = 0.5f;
        m->specular = (tf & StageBundle::TEX_FOLIAGE) ? 0.03f : 0.12f;
        m->gloss = (tf & StageBundle::TEX_FOLIAGE) ? 6.0f : 16.0f;
    } else {
        m->specular = 0.15f;
        m->gloss = 20.0f;
    }
    m_mats.push_back({key, m});
    return m;
}

bool StageScenery::build(const StageBundle& b, const std::string& dir) {
    PROFILE_ZONE("Stage meshes");
    m_tex.clear();
    for (auto& t : b.textures) m_tex.push_back(t.file.empty() ? nullptr : TextureCache::get().load(dir + "/" + t.file));
    m_items.clear();
    triangles = 0;
    for (auto& batch : b.batches) {
        if (batch.idx.empty()) continue;
        Part p;
        p.mesh = std::make_unique<GpuMesh>();
        p.mesh->create(batch.v.data(), (int)batch.v.size(), batch.idx.data(), (int)batch.idx.size());
        p.mat = material(b, batch);
        triangles += batch.idx.size() / 3;
        m_items.push_back(std::move(p));
    }
    // ground first (large occluders), then the solid objects, alpha-tested foliage last
    std::stable_sort(m_items.begin(), m_items.end(), [](const Part& x, const Part& y) { return (x.mat->alpha_ref > 0) < (y.mat->alpha_ref > 0); });
    templates.clear();
    for (auto& t : b.templates) {
        std::vector<Part> parts;
        for (auto& m : t.meshes) {
            Part p;
            p.mesh = std::make_unique<GpuMesh>();
            p.mesh->create(m.v.data(), (int)m.v.size(), m.idx.data(), (int)m.idx.size());
            p.mat = material(b, m);
            parts.push_back(std::move(p));
        }
        templates.push_back(std::move(parts));
    }
    return true;
}

void StageScenery::draw(Renderer& r) const {
    for (auto& p : m_items) r.draw_mesh(p.mesh.get(), p.mat.get(), mat4());
}

} // namespace bl
