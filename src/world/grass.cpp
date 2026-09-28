#include "world/grass.h"
#include "core/profiler.h"
#include "gfx/texture.h"
#include "phys/softbody.h"
#include "vehicle/vehicle.h"

#include <algorithm>
#include <cmath>

namespace bl {

namespace {

// Blades grow from the texture's first row (v = 0 at the base, 1 at the tips: the wind sway scales with v).
TexturePtr make_grass_texture() {
    const int W = 256, H = 256;
    std::vector<vec4> px((size_t)W * H, vec4(0.2f, 0.3f, 0.1f, 0.0f));
    Rng rng(4711);
    for (int b = 0; b < 90; b++) {
        float x0 = rng.range(8.0f, W - 8.0f);
        float height = rng.range(0.45f, 0.98f) * H;
        float lean = rng.range(-0.28f, 0.28f) * W;
        float w0 = rng.range(2.5f, 5.5f);
        bool dry = rng.uniform() < 0.15f;
        vec3 base = dry ? vec3(0.40f, 0.37f, 0.20f) : vec3(0.20f, 0.32f, 0.09f);
        vec3 tip = dry ? vec3(0.66f, 0.60f, 0.36f) : lerp(vec3(0.36f, 0.50f, 0.15f), vec3(0.50f, 0.58f, 0.22f), rng.uniform());
        for (int y = 0; y < (int)height; y++) {
            float t = y / height;
            float cx = x0 + lean * t * t;
            float half = w0 * (1.0f - t) * 0.5f + 0.35f;
            for (int x = (int)std::floor(cx - half); x <= (int)std::ceil(cx + half); x++) {
                if (x < 0 || x >= W) continue;
                float edge = 1.0f - std::fabs((x + 0.5f - cx) / (half + 0.5f));
                if (edge <= 0) continue;
                vec3 col = lerp(base, tip, std::pow(t, 0.8f)) * (0.8f + 0.25f * edge);
                vec4& d = px[(size_t)y * W + x];
                d = vec4(col, std::max(d.w, clampf(edge * 2.0f, 0, 1)));
            }
        }
    }
    Image img;
    img.w = W;
    img.h = H;
    img.rgba.resize((size_t)W * H * 4);
    for (size_t i = 0; i < px.size(); i++)
        for (int k = 0; k < 4; k++) img.rgba[i * 4 + k] = (uint8_t)(clampf(px[i][k], 0, 1) * 255.0f + 0.5f);
    return TextureCache::get().from_image(img, "<grass>", true, true);
}

int64_t cell_key(int x, int z) { return ((int64_t)x << 32) ^ (uint32_t)z; }

} // namespace

GrassField::GrassField() = default;
GrassField::~GrassField() = default;

void GrassField::add(vec3 p, float scale, float tint, uint32_t seed) {
    // (linear chunk search while building: called before finalize() only, chunk lists stay short per scene)
    Chunk* c = nullptr;
    int cx = (int)std::floor(p.x / m_cell), cz = (int)std::floor(p.z / m_cell);
    int64_t key = cell_key(cx, cz);
    auto it = std::lower_bound(m_index.begin(), m_index.end(), std::make_pair(key, -1));
    if (it != m_index.end() && it->first == key) c = m_chunks[(size_t)it->second].get();
    else {
        m_index.insert(it, {key, (int)m_chunks.size()});
        m_chunks.push_back(std::make_unique<Chunk>());
        c = m_chunks.back().get();
    }
    Tuft t;
    t.p = p;
    t.yaw = (float)((seed * 2654435761u) % 6283) * 0.001f;
    t.scale = scale;
    vec3 green(1.0f, 1.0f, 1.0f), dry(1.35f, 1.15f, 0.75f);
    vec3 col = lerp(green, dry, clampf(tint, 0, 1)) * (0.85f + 0.3f * ((seed >> 7) % 100) * 0.01f);
    t.tint = vec4(col, 1);
    c->tufts.push_back(t);
    c->box.add(p);
    c->box.add(p + vec3(0, 0.5f * scale, 0));
    m_count++;
}

void GrassField::finalize() {
    // tuft: three crossed quads, normals up (lit like the ground they grow from)
    std::vector<Vertex> v;
    std::vector<uint32_t> idx;
    for (int q = 0; q < 3; q++) {
        float a = q * kPi / 3.0f;
        vec3 d(std::cos(a) * 0.3f, 0, std::sin(a) * 0.3f);
        uint32_t b = (uint32_t)v.size();
        v.push_back({-d, vec3(0, 1, 0), {0, 0}});
        v.push_back({d, vec3(0, 1, 0), {1, 0}});
        v.push_back({d + vec3(0, 0.45f, 0), vec3(0, 1, 0), {1, 1}});
        v.push_back({-d + vec3(0, 0.45f, 0), vec3(0, 1, 0), {0, 1}});
        idx.insert(idx.end(), {b, b + 1, b + 2, b, b + 2, b + 3});
    }
    m_mesh.create(v, idx, false);
    m_mat = std::make_shared<Material>();
    m_mat->diffuse = make_grass_texture();
    m_mat->alpha_ref = 0.4f;
    m_mat->double_sided = true;
    m_mat->cast_shadow = false;
    m_mat->specular = 0.04f;
    m_mat->gloss = 8.0f;
    m_mat->name = "grass";
    for (auto& c : m_chunks) {
        c->batch = std::make_unique<InstanceBatch>();
        c->batch->init(&m_mesh);
        c->dirty = true;
    }
}

void GrassField::rebuild(Chunk& c) {
    c.inst.resize(c.tufts.size());
    for (size_t i = 0; i < c.tufts.size(); i++) {
        const Tuft& t = c.tufts[i];
        mat4 m = mat4::from_trs(t.p, quat::axis_angle(vec3(0, 1, 0), t.yaw), vec3(t.scale, t.scale * (1.0f - 0.35f * t.bend), t.scale));
        if (t.bend > 0.001f) {
            // lay the tuft down towards `dir`: rotate about the horizontal axis across the push
            vec3 push(std::sin(t.dir), 0, std::cos(t.dir));
            vec3 axis = cross(vec3(0, 1, 0), push);
            mat4 lay = mat4::from_trs(vec3(0), quat::axis_angle(axis, t.bend * 1.35f));
            m = mat4::translate(t.p) * lay * mat4::translate(-t.p) * m;
        }
        c.inst[i] = InstanceData::from(m, t.tint);
    }
    c.batch->upload(c.inst.data(), (int)c.inst.size());
    c.dirty = false;
}

void GrassField::crush(vec2 c, float radius, float dir, float hold) {
    int x0 = (int)std::floor((c.x - radius) / m_cell), x1 = (int)std::floor((c.x + radius) / m_cell);
    int z0 = (int)std::floor((c.y - radius) / m_cell), z1 = (int)std::floor((c.y + radius) / m_cell);
    for (int z = z0; z <= z1; z++)
        for (int x = x0; x <= x1; x++) {
            int64_t key = cell_key(x, z);
            auto it = std::lower_bound(m_index.begin(), m_index.end(), std::make_pair(key, -1));
            if (it == m_index.end() || it->first != key) continue;
            Chunk& ch = *m_chunks[(size_t)it->second];
            for (Tuft& t : ch.tufts) {
                float r = radius + 0.12f * t.scale;
                float dx = t.p.x - c.x, dz = t.p.z - c.y;
                if (dx * dx + dz * dz > r * r) continue;
                if (t.bend < 0.99f || std::fabs(t.dir - dir) > 0.3f) ch.dirty = true;
                t.bend = 1.0f;
                t.dir = dir;
                t.hold = hold;
                ch.recovering = true;
            }
        }
}

void GrassField::update(const std::vector<std::unique_ptr<Vehicle>>& vehicles, float dt, const std::function<float(float, float)>& ground) {
    PROFILE_ZONE("Grass");
    if (m_chunks.empty()) return;
    for (const auto& vp : vehicles) {
        const phys::SoftBody& b = *vp->body;
        vec3 vel = vp->velocity();
        float sp = std::sqrt(vel.x * vel.x + vel.z * vel.z);
        if (sp < 0.3f) continue;
        float dir = std::atan2(vel.x, vel.z);
        // wheels: the tread nodes on the ground press a strip as wide as the tyre
        for (const phys::Wheel& w : b.wheels) {
            if (w.detached || w.nodes.empty()) continue;
            float lowest = 1e30f;
            vec3 lp;
            for (uint32_t ni : w.nodes)
                if (b.nodes[ni].p.y < lowest) {
                    lowest = b.nodes[ni].p.y;
                    lp = b.nodes[ni].p;
                }
            if (lowest - ground(lp.x, lp.z) > 0.12f) continue; // airborne
            crush(vec2(lp.x, lp.z), std::max(0.22f, 0.5f * w.width + 0.08f), dir, 6.0f);
        }
        // a low body (bumpers, sills) flattens what the wheels miss
        const float lo = b.aabb.mn.y;
        for (size_t i = 0; i < b.nodes.size(); i += 5) {
            const vec3& p = b.nodes[i].p;
            if (p.y - lo > 0.35f || p.y - ground(p.x, p.z) > 0.22f) continue;
            crush(vec2(p.x, p.z), 0.18f, dir, 4.0f);
        }
    }
    // recovery (in steps, so that a chunk is re-uploaded a few times per second at most)
    m_recover_clock += dt;
    if (m_recover_clock < 0.2f) return;
    float step = m_recover_clock;
    m_recover_clock = 0;
    for (auto& c : m_chunks) {
        if (!c->recovering) continue;
        bool any = false;
        for (Tuft& t : c->tufts) {
            if (t.bend <= 0) continue;
            if (t.hold > 0) t.hold -= step;
            else t.bend = std::max(0.0f, t.bend - step / 25.0f); // stands up again over ~25 s
            any = true;
        }
        c->recovering = any;
        c->dirty = true;
    }
}

void GrassField::draw(Renderer& r, vec3 cam) {
    if (!m_mat) return;
    for (auto& c : m_chunks) {
        vec3 center = c->box.center();
        float reach = view_distance + length(c->box.extent()) * 0.5f;
        if (length2(center - cam) > reach * reach) continue;
        if (c->dirty) rebuild(*c);
        r.draw_instanced(c->batch.get(), m_mat.get(), true);
    }
}

} // namespace bl
