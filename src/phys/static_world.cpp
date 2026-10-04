#include "phys/static_world.h"

#include <cmath>
#include <cstdlib>

namespace bl::phys {

vec3 primitive_collision(vec3 F, vec3 vel, float mass, vec3 normal, float dt, const GroundModel& gm, float pen, float friction_coef, float push_max,
                         float slop, float restitution) {
    vec3 out(0);
    float Vn = dot(vel, normal);
    float Fn = dot(F, normal);
    float sgl = gm.soft_depth;
    if (sgl > 0 && pen >= 0) {
        // fluid layer (mud): viscous drag + buoyancy
        float sp = length(vel);
        out += vel * (-gm.fluid_drag * std::min(1.0f, pen / sgl + 0.2f));
        float Fb = gm.fluid_density * std::min(pen, sgl) * 9.807f * 0.02f;
        if (Vn >= 0 && Fn < 0 && Fb > -Fn) Fb = -Fn;
        out += normal * Fb;
        (void)sp;
    }
    if (pen >= sgl) {
        float R = -Fn;
        // RoR: remove 80% of the approach speed and 20% of the penetration per step. The penetration
        // term is capped at 2 m/s so that nodes appearing deep inside a contact zone are not launched.
        // (push_max, slop: a body's own; see SoftBody::contact_push_max)
        const float take = restitution >= 0 ? 1.0f + std::min(restitution, 0.95f) : 0.8f;
        if (Vn < 0) R -= (take * Vn - std::min(0.2f * std::max(0.0f, pen - sgl - slop) / dt, push_max)) * mass / dt;
        if (R > 0) {
            vec3 slipF = F - normal * Fn;
            vec3 slip = vel - normal * Vn;
            float sv = length(slip);
            if (sv > 1e-9f) slip = slip / sv;
            else slip = vec3(0);
            float G = R * gm.strength * friction_coef;
            // Friction may stop the tangential motion within the step but never reverse it: the RoR terms scale
            // with the normal force, not with the node mass, and a light node squeezed hard (tape under a truck
            // tyre) would otherwise flip its velocity every substep with a growing amplitude.
            const float stop = mass * sv / dt;
            if (sv < gm.va && G > 0 && length(slipF) <= gm.ms * G) {
                // static friction (adhesion): cancel tangential force + smooth velocity damping
                out += normal * R - slip * std::min(gm.ms * G * (1.0f - std::exp(-sv / gm.va)), stop) - slipF;
            } else {
                float g = gm.mc + (gm.ms - gm.mc) * std::exp(-std::pow(sv / gm.vs, gm.alpha));
                out += normal * R - slip * std::min((g + std::min(gm.t2 * sv, 5.0f)) * G, std::max(0.0f, stop + dot(slipF, slip)));
            }
        }
    }
    return out;
}

TyreContact tyre_contact(vec3 F, vec3 vel, float mass, vec3 normal, float dt, const GroundModel& gm, float pen, float friction_coef) {
    TyreContact t;
    float Vn = dot(vel, normal);
    float Fn = dot(F, normal);
    float sgl = gm.soft_depth;
    if (sgl > 0 && pen >= 0) {
        t.normal_force += vel * (-gm.fluid_drag * std::min(1.0f, pen / sgl + 0.2f));
        float Fb = gm.fluid_density * std::min(pen, sgl) * 9.807f * 0.02f;
        if (Vn >= 0 && Fn < 0 && Fb > -Fn) Fb = -Fn;
        t.normal_force += normal * Fb;
    }
    if (pen < sgl) return t;
    float R = -Fn;
    if (Vn < 0) R -= (0.8f * Vn - std::min(0.2f * (pen - sgl) / dt, 2.0f)) * mass / dt;
    if (R <= 0) return t;
    t.touching = true;
    t.normal_force += normal * R;
    vec3 slipF = F - normal * Fn;
    vec3 slipV = vel - normal * Vn;
    float G = R * gm.strength * friction_coef;
    t.need = -slipF - slipV * (0.5f * mass / dt);
    t.cap = gm.ms * G;
    // sliding rubber keeps ~85% of its peak grip (the ground models' mc is tuned for RoR's softer adhesion)
    float sv = length(slipV);
    float mu_slide = std::max(gm.mc, 0.85f * gm.ms);
    float g = mu_slide + (gm.ms - mu_slide) * std::exp(-std::pow(sv / gm.vs, gm.alpha)) + std::min(gm.t2 * sv, 5.0f);
    float nl = length(t.need);
    vec3 dir = sv > 0.05f ? slipV * (-1.0f / sv) : (nl > 1e-6f ? t.need * (1.0f / nl) : vec3(0));
    t.slide = dir * (g * G);
    return t;
}

const std::vector<GroundModel>& ground_models() {
    static std::vector<GroundModel> g = [] {
        std::vector<GroundModel> v(SURF_COUNT);
        auto mk = [](const char* n, float va, float ms, float mc, float t2, float vs, vec3 col) {
            GroundModel m;
            m.name = n; m.va = va; m.ms = ms; m.mc = mc; m.t2 = t2; m.vs = vs; m.color = col;
            return m;
        };
        // values from RoR's stock ground_models.cfg (grass/asphalt/concrete/gravel/rock/sand/metal/ice)
        v[SURF_GRASS] = mk("grass", 3, 0.80f, 0.55f, 0.005f, 7, {0.33f, 0.47f, 0.20f});
        v[SURF_DIRT] = mk("dirt", 3, 0.80f, 0.58f, 0.006f, 4, {0.45f, 0.36f, 0.26f});
        v[SURF_ASPHALT] = mk("asphalt", 3, 1.20f, 0.75f, 0.010f, 6, {0.22f, 0.22f, 0.23f});
        v[SURF_CONCRETE] = mk("concrete", 3, 1.20f, 0.75f, 0.010f, 6, {0.58f, 0.57f, 0.55f});
        v[SURF_GRAVEL] = mk("gravel", 3, 0.85f, 0.60f, 0.006f, 3, {0.52f, 0.49f, 0.44f});
        v[SURF_MUD] = mk("mud", 2, 0.50f, 0.35f, 0.004f, 3, {0.30f, 0.23f, 0.16f});
        v[SURF_MUD].soft_depth = 0.12f;
        v[SURF_MUD].fluid_drag = 60.0f;
        v[SURF_MUD].fluid_density = 1500.0f;
        v[SURF_SAND] = mk("sand", 3, 0.70f, 0.55f, 0.0001f, 9, {0.76f, 0.68f, 0.50f});
        v[SURF_SAND].soft_depth = 0.03f;
        v[SURF_SAND].fluid_drag = 20.0f;
        v[SURF_SAND].fluid_density = 1600.0f;
        v[SURF_ROCK] = mk("rock", 3, 0.95f, 0.70f, 0.007f, 8, {0.45f, 0.43f, 0.40f});
        v[SURF_WOOD] = mk("wood", 3, 0.90f, 0.60f, 0.005f, 5, {0.55f, 0.40f, 0.25f});
        v[SURF_METAL] = mk("metal", 3, 0.70f, 0.40f, 0.001f, 3, {0.6f, 0.6f, 0.62f});
        v[SURF_ICE] = mk("ice", 1, 0.30f, 0.20f, 0.0001f, 3, {0.8f, 0.9f, 1.0f});
        return v;
    }();
    return g;
}

// ------------------------------------------------------------------ Heightfield
void Heightfield::create_sparse(int nx, int nz, float cell, vec2 origin) {
    m_nx = nx;
    m_nz = nz;
    m_cell = cell;
    m_origin = origin;
    m_tx = (nx + kTile - 1) >> kTileShift;
    m_tz = (nz + kTile - 1) >> kTileShift;
    m_tile_of.assign((size_t)m_tx * m_tz, -1);
    m_tile_pos.clear();
    m_th.clear();
    m_ts.clear();
    m_bx = m_bz = 0;
    m_block_max.clear();
}

void Heightfield::create(int nx, int nz, float cell, vec2 origin) {
    create_sparse(nx, nz, cell, origin);
    for (int tz = 0; tz < m_tz; tz++)
        for (int tx = 0; tx < m_tx; tx++) tile_heights(tx, tz);
    update_bounds();
}

float* Heightfield::tile_heights(int tx, int tz) {
    int32_t& t = m_tile_of[(size_t)tz * m_tx + tx];
    if (t < 0) {
        t = (int32_t)m_tile_pos.size();
        m_tile_pos.push_back({tx, tz});
        m_th.resize(m_th.size() + kTile * kTile, 0.0f);
        m_ts.resize(m_ts.size() + kTile * kTile, SURF_GRASS);
    }
    return &m_th[(size_t)t << (2 * kTileShift)];
}

uint8_t* Heightfield::tile_surfaces(int tx, int tz) {
    tile_heights(tx, tz);
    return &m_ts[(size_t)m_tile_of[(size_t)tz * m_tx + tx] << (2 * kTileShift)];
}

size_t Heightfield::slot(int x, int z) {
    float* base = tile_heights(x >> kTileShift, z >> kTileShift);
    return (size_t)(base - m_th.data()) + ((size_t)(z & (kTile - 1)) << kTileShift) + (x & (kTile - 1));
}

void Heightfield::update_bounds() {
    m_bx = (m_nx - 1 + kBlock - 1) / kBlock;
    m_bz = (m_nz - 1 + kBlock - 1) / kBlock;
    m_block_max.assign((size_t)m_bx * m_bz, -1e30f);
    for (size_t t = 0; t < m_tile_pos.size(); t++) {
        const float* th = &m_th[t << (2 * kTileShift)];
        const int x0 = m_tile_pos[t].first * kTile, z0 = m_tile_pos[t].second * kTile;
        for (int lz = 0; lz < kTile; lz++)
            for (int lx = 0; lx < kTile; lx++) {
                int x = x0 + lx, z = z0 + lz;
                if (x >= m_nx || z >= m_nz) continue;
                float v = th[(lz << kTileShift) + lx];
                // a vertex belongs to up to 4 blocks (block borders)
                int bx0 = std::max(0, (x - 1) / kBlock), bx1 = std::min(m_bx - 1, x / kBlock);
                int bz0 = std::max(0, (z - 1) / kBlock), bz1 = std::min(m_bz - 1, z / kBlock);
                for (int bz = bz0; bz <= bz1; bz++)
                    for (int bx = bx0; bx <= bx1; bx++) {
                        float& m = m_block_max[(size_t)bz * m_bx + bx];
                        m = std::max(m, v);
                    }
            }
    }
}

bool Heightfield::inside(float x, float z) const {
    float lx = (x - m_origin.x) / m_cell, lz = (z - m_origin.y) / m_cell;
    return lx >= 0 && lz >= 0 && lx < m_nx - 1 && lz < m_nz - 1;
}

bool Heightfield::sample(float x, float z, float& out_h, vec3& n) const {
    float lx = (x - m_origin.x) / m_cell, lz = (z - m_origin.y) / m_cell;
    if (!(lx >= 0 && lz >= 0 && lx < m_nx - 1 && lz < m_nz - 1)) return false;
    int ix = (int)lx, iz = (int)lz;
    float fx = lx - ix, fz = lz - iz;
    float h00 = h(ix, iz), h10 = h(ix + 1, iz), h01 = h(ix, iz + 1), h11 = h(ix + 1, iz + 1);
    if (std::min(std::min(h00, h10), std::min(h01, h11)) < -1e29f) return false; // no ground here
    // quad split along the (0,0)-(1,1) diagonal (matches the render mesh)
    if (fx > fz) {
        out_h = h00 + (h10 - h00) * fx + (h11 - h10) * fz;
        n = normalize(vec3(-(h10 - h00) / m_cell, 1.0f, -(h11 - h10) / m_cell));
    } else {
        out_h = h00 + (h11 - h01) * fx + (h01 - h00) * fz;
        n = normalize(vec3(-(h11 - h01) / m_cell, 1.0f, -(h01 - h00) / m_cell));
    }
    return true;
}

float Heightfield::height(float x, float z) const {
    float hh;
    vec3 n;
    return sample(x, z, hh, n) ? hh : -1e30f;
}

uint8_t Heightfield::surface_at(float x, float z) const {
    int ix = (int)std::lround((x - m_origin.x) / m_cell), iz = (int)std::lround((z - m_origin.y) / m_cell);
    ix = std::max(0, std::min(m_nx - 1, ix));
    iz = std::max(0, std::min(m_nz - 1, iz));
    return surf(ix, iz);
}

float Heightfield::max_height(float x0, float z0, float x1, float z1) const {
    int bx0 = (int)std::floor((x0 - m_origin.x) / m_cell) / kBlock, bx1 = (int)std::floor((x1 - m_origin.x) / m_cell) / kBlock;
    int bz0 = (int)std::floor((z0 - m_origin.y) / m_cell) / kBlock, bz1 = (int)std::floor((z1 - m_origin.y) / m_cell) / kBlock;
    bx0 = std::max(0, bx0);
    bz0 = std::max(0, bz0);
    bx1 = std::min(m_bx - 1, bx1);
    bz1 = std::min(m_bz - 1, bz1);
    float m = -1e30f;
    for (int bz = bz0; bz <= bz1; bz++)
        for (int bx = bx0; bx <= bx1; bx++) m = std::max(m, m_block_max[(size_t)bz * m_bx + bx]);
    return m;
}

vec3 Heightfield::vertex_normal(int x, int z) const {
    int x0 = std::max(0, x - 1), x1 = std::min(m_nx - 1, x + 1);
    int z0 = std::max(0, z - 1), z1 = std::min(m_nz - 1, z + 1);
    float dx = (h(x1, z) - h(x0, z)) / ((x1 - x0) * m_cell);
    float dz = (h(x, z1) - h(x, z0)) / ((z1 - z0) * m_cell);
    return normalize(vec3(-dx, 1.0f, -dz));
}

// ------------------------------------------------------------------ static world
void StaticBox::update_aabb() {
    vec3 e = vabs(rot.c[0]) * half.x + vabs(rot.c[1]) * half.y + vabs(rot.c[2]) * half.z;
    aabb.mn = center - e;
    aabb.mx = center + e;
}

void PavedLayer::create(vec2 origin, vec2 size, float cell) {
    clear();
    m_origin = origin, m_cell = cell;
    m_nx = (int)std::ceil(size.x / cell) + 1, m_nz = (int)std::ceil(size.y / cell) + 1;
    m_tx = (m_nx + kTile - 1) >> kTileShift, m_tz = (m_nz + kTile - 1) >> kTileShift;
    m_tile_of.assign((size_t)m_tx * m_tz, -1);
}

void PavedLayer::set(int ix, int iz, float dh, uint8_t surface) {
    if (ix < 0 || iz < 0 || ix >= m_nx || iz >= m_nz) return;
    int& t = m_tile_of[(size_t)(iz >> kTileShift) * m_tx + (ix >> kTileShift)];
    if (t < 0) {
        t = (int)(m_dh.size() >> (2 * kTileShift));
        m_dh.resize(m_dh.size() + (size_t)kTile * kTile, 0.0f);
        m_surf.resize(m_surf.size() + (size_t)kTile * kTile, kNone);
    }
    const size_t k = ((size_t)t << (2 * kTileShift)) + ((size_t)(iz & (kTile - 1)) << kTileShift) + (ix & (kTile - 1));
    m_dh[k] = dh, m_surf[k] = surface;
    max_raise = std::max(max_raise, dh);
}

bool PavedLayer::sample(float x, float z, float& dh_out, vec2& grad, uint8_t& surface) const {
    const float fx = (x - m_origin.x) / m_cell, fz = (z - m_origin.y) / m_cell;
    const int ix = (int)std::floor(fx), iz = (int)std::floor(fz);
    if (ix < 0 || iz < 0 || ix + 1 >= m_nx || iz + 1 >= m_nz) return false;
    const float u = fx - (float)ix, v = fz - (float)iz;
    surface = surf(ix + (u > 0.5f), iz + (v > 0.5f));
    if (surface == kNone) return false;
    const float h00 = dh(ix, iz), h10 = dh(ix + 1, iz), h01 = dh(ix, iz + 1), h11 = dh(ix + 1, iz + 1);
    dh_out = lerpf(lerpf(h00, h10, u), lerpf(h01, h11, u), v);
    grad = vec2(lerpf(h10 - h00, h11 - h01, v), lerpf(h01 - h00, h11 - h10, u)) * (1.0f / m_cell);
    return true;
}

float StaticWorld::ground_height(float x, float z) const {
    float h = has_terrain ? terrain.height(x, z) : 0.0f;
    if (h < -1e20f) h = 0.0f;
    if (road) {
        float s, lat;
        if (road->locate(x, z, s, lat)) h += road->detail(s, lat);
    }
    if (!paved.empty()) {
        float dh;
        vec2 g;
        uint8_t sf;
        if (paved.sample(x, z, dh, g, sf)) h += dh;
    }
    return h;
}

void StaticWorld::add_box(vec3 center, vec3 half, const quat& rot, uint8_t surface) {
    StaticBox b;
    b.center = center;
    b.rot = to_mat3(rot);
    b.half = half;
    b.surface = surface;
    b.update_aabb();
    boxes.push_back(b);
}

void StaticWorld::build_grid() {
    AABB all;
    for (auto& b : boxes) all.add(b.aabb);
    for (auto& c : cylinders) {
        all.add(c.base - vec3(c.radius, 0, c.radius));
        all.add(c.base + vec3(c.radius, c.height, c.radius));
    }
    m_grid_boxes.clear();
    m_big_boxes.clear();
    if (!all.valid()) {
        m_gx = m_gz = 0;
        return;
    }
    m_grid_origin = vec2(all.mn.x, all.mn.z);
    m_gx = std::max(1, (int)std::ceil((all.mx.x - all.mn.x) / m_grid_cell) + 1);
    m_gz = std::max(1, (int)std::ceil((all.mx.z - all.mn.z) / m_grid_cell) + 1);
    m_grid_boxes.assign((size_t)m_gx * m_gz, {});
    for (int i = 0; i < (int)boxes.size(); i++) {
        const AABB& a = boxes[i].aabb;
        int x0 = (int)((a.mn.x - m_grid_origin.x) / m_grid_cell), x1 = (int)((a.mx.x - m_grid_origin.x) / m_grid_cell);
        int z0 = (int)((a.mn.z - m_grid_origin.y) / m_grid_cell), z1 = (int)((a.mx.z - m_grid_origin.y) / m_grid_cell);
        if ((x1 - x0 + 1) * (z1 - z0 + 1) > 64) {
            m_big_boxes.push_back(i);
            continue;
        }
        for (int z = z0; z <= z1; z++)
            for (int x = x0; x <= x1; x++) m_grid_boxes[(size_t)z * m_gx + x].push_back(i);
    }
}

void StaticWorld::query(const AABB& b, std::vector<int>& out_boxes, std::vector<int>& out_cyl) const {
    size_t start = out_boxes.size();
    for (int i : m_big_boxes)
        if (boxes[i].aabb.overlaps(b)) out_boxes.push_back(i);
    if (m_gx > 0) {
        int x0 = std::max(0, (int)((b.mn.x - m_grid_origin.x) / m_grid_cell)), x1 = std::min(m_gx - 1, (int)((b.mx.x - m_grid_origin.x) / m_grid_cell));
        int z0 = std::max(0, (int)((b.mn.z - m_grid_origin.y) / m_grid_cell)), z1 = std::min(m_gz - 1, (int)((b.mx.z - m_grid_origin.y) / m_grid_cell));
        if (b.mx.x >= m_grid_origin.x && b.mx.z >= m_grid_origin.y)
            for (int z = z0; z <= z1; z++)
                for (int x = x0; x <= x1; x++)
                    for (int i : m_grid_boxes[(size_t)z * m_gx + x]) {
                        if (!boxes[i].aabb.overlaps(b)) continue;
                        bool dup = false;
                        for (size_t k = start; k < out_boxes.size(); k++)
                            if (out_boxes[k] == i) { dup = true; break; }
                        if (!dup) out_boxes.push_back(i);
                    }
    }
    for (int i = 0; i < (int)cylinders.size(); i++) {
        const auto& c = cylinders[i];
        if (b.mx.x >= c.base.x - c.radius && b.mn.x <= c.base.x + c.radius && b.mx.z >= c.base.z - c.radius &&
            b.mn.z <= c.base.z + c.radius && b.mx.y >= c.base.y && b.mn.y <= c.base.y + c.height)
            out_cyl.push_back(i);
    }
}

bool StaticWorld::collide_point(vec3 p, float r, const int* box_ids, int nbox, const int* cyl_ids, int ncyl, bool test_terrain,
                                ContactInfo& out) const {
    bool hit = false;
    out.depth = 0;
    out.max_force = 0;
    if (test_terrain && has_terrain) {
        float th;
        vec3 n;
        if (terrain.sample(p.x, p.z, th, n)) {
            uint8_t surf = 0xff;
            if (road) {
                float dh, lat;
                vec2 g;
                if (road->sample(p.x, p.z, dh, g, lat)) {
                    // bed slope + detail slope
                    float hx = -n.x / n.y + g.x, hz = -n.z / n.y + g.y;
                    th += dh;
                    n = normalize(vec3(-hx, 1.0f, -hz));
                    if (std::fabs(lat) < road->half_width + 0.3f) surf = road->surface;
                }
            }
            if (!paved.empty()) {
                float dh;
                vec2 g;
                uint8_t sf;
                if (paved.sample(p.x, p.z, dh, g, sf)) {
                    const float hx = -n.x / n.y + g.x, hz = -n.z / n.y + g.y;
                    th += dh;
                    n = normalize(vec3(-hx, 1.0f, -hz));
                    surf = sf;
                }
            }
            float d = (th - p.y) * n.y + r;
            if (d > 0) {
                out.depth = d;
                out.normal = n;
                out.surface = surf != 0xff ? surf : terrain.surface_at(p.x, p.z);
                hit = true;
            }
        }
    }
    for (int k = 0; k < nbox; k++) {
        const StaticBox& b = boxes[box_ids[k]];
        // (out of its bounds by more than the radius: out of it - most of a list are, a long body's list is long)
        if (p.x < b.aabb.mn.x - r || p.x > b.aabb.mx.x + r || p.y < b.aabb.mn.y - r || p.y > b.aabb.mx.y + r || p.z < b.aabb.mn.z - r || p.z > b.aabb.mx.z + r) continue;
        vec3 rel = p - b.center;
        vec3 l(dot(rel, b.rot.c[0]), dot(rel, b.rot.c[1]), dot(rel, b.rot.c[2]));
        vec3 q(clampf(l.x, -b.half.x, b.half.x), clampf(l.y, -b.half.y, b.half.y), clampf(l.z, -b.half.z, b.half.z));
        vec3 d = l - q;
        float dist2 = dot(d, d);
        float depth;
        vec3 nl;
        if (!b.planes.empty()) { // (a hull: the box and its planes - the signed distance the larger of the two's)
            float sd;
            if (dist2 > 1e-12f) {
                sd = std::sqrt(dist2), nl = d / sd;
            } else {
                const vec3 pen = b.half - vabs(l);
                if (pen.x <= pen.y && pen.x <= pen.z) sd = -pen.x, nl = vec3(signf(l.x), 0, 0);
                else if (pen.y <= pen.z) sd = -pen.y, nl = vec3(0, signf(l.y), 0);
                else sd = -pen.z, nl = vec3(0, 0, signf(l.z));
            }
            for (const vec4& pl : b.planes) {
                const float s = pl.x * l.x + pl.y * l.y + pl.z * l.z - pl.w;
                if (s > sd) sd = s, nl = vec3(pl.x, pl.y, pl.z);
            }
            if (sd >= r) continue;
            depth = r - sd;
        } else if (dist2 > 1e-12f) {
            if (dist2 >= r * r) continue;
            float dist = std::sqrt(dist2);
            depth = r - dist;
            nl = d / dist;
        } else {
            // inside: push out through the nearest face
            vec3 pen = b.half - vabs(l);
            if (pen.x <= pen.y && pen.x <= pen.z) { depth = pen.x; nl = vec3(signf(l.x), 0, 0); }
            else if (pen.y <= pen.z) { depth = pen.y; nl = vec3(0, signf(l.y), 0); }
            else { depth = pen.z; nl = vec3(0, 0, signf(l.z)); }
            depth += r;
        }
        if (depth > out.depth) {
            out.depth = depth;
            out.normal = b.rot * nl;
            out.surface = b.surface;
            out.max_force = b.max_force;
            hit = true;
        }
    }
    for (int k = 0; k < ncyl; k++) {
        const StaticCylinder& c = cylinders[cyl_ids[k]];
        vec3 rel = p - c.base;
        if (rel.y < -r || rel.y > c.height + r) continue;
        if (std::fabs(rel.x) >= c.radius + r || std::fabs(rel.z) >= c.radius + r) continue;
        float dxz = std::sqrt(rel.x * rel.x + rel.z * rel.z);
        float side = c.radius + r - dxz;
        float top = c.height + r - rel.y;
        if (side <= 0) continue;
        float depth;
        vec3 n;
        if (top < side && rel.y > c.height * 0.5f) { depth = top; n = vec3(0, 1, 0); }
        else { depth = side; n = dxz > 1e-6f ? vec3(rel.x / dxz, 0, rel.z / dxz) : vec3(1, 0, 0); }
        if (depth > out.depth) {
            out.depth = depth;
            out.normal = n;
            out.surface = c.surface;
            hit = true;
        }
    }
    return hit;
}

float StaticWorld::raycast(vec3 o, vec3 d, float max_t) const {
    float best = -1;
    // boxes (slab test in local space)
    for (const StaticBox& b : boxes) {
        vec3 rel = o - b.center;
        vec3 lo(dot(rel, b.rot.c[0]), dot(rel, b.rot.c[1]), dot(rel, b.rot.c[2]));
        vec3 ld(dot(d, b.rot.c[0]), dot(d, b.rot.c[1]), dot(d, b.rot.c[2]));
        float t0 = 0, t1 = max_t;
        bool miss = false;
        for (int a = 0; a < 3 && !miss; a++) {
            if (std::fabs(ld[a]) < 1e-9f) {
                if (lo[a] < -b.half[a] || lo[a] > b.half[a]) miss = true;
                continue;
            }
            float ta = (-b.half[a] - lo[a]) / ld[a], tb = (b.half[a] - lo[a]) / ld[a];
            if (ta > tb) std::swap(ta, tb);
            t0 = std::max(t0, ta);
            t1 = std::min(t1, tb);
            if (t0 > t1) miss = true;
        }
        if (!miss && (best < 0 || t0 < best)) best = t0;
    }
    if (has_terrain) {
        float step = terrain.cell() * 0.5f;
        float lim = best >= 0 ? best : max_t;
        float prev_t = 0;
        vec3 pp = o;
        float ph = terrain.height(pp.x, pp.z);
        for (float t = step; t <= lim; t += step) {
            vec3 p = o + d * t;
            float h = terrain.height(p.x, p.z);
            if (h > -1e29f && p.y <= h) {
                // refine by bisection
                float a = prev_t, bb = t;
                for (int i = 0; i < 12; i++) {
                    float m = (a + bb) * 0.5f;
                    vec3 q = o + d * m;
                    if (q.y <= terrain.height(q.x, q.z)) bb = m;
                    else a = m;
                }
                return bb;
            }
            prev_t = t;
            ph = h;
        }
        (void)ph;
    }
    return best;
}

} // namespace bl::phys
