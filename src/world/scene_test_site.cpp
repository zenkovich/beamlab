// The Test Site (scene `test_site`), a vehicle test site 1400 x 1000 m:
//   - a high-speed oval (two 700 m straights, turns of 200 m radius banked 14 degrees) round the test lanes;
//   - twenty test lanes inside it, each its own road 630 m long with 480 m to get up to speed before its test: walls
//     (full, 40% and 25% overlap), poles, posts that break, concrete barriers, blocks that slide, the two wedges, the
//     press, the vise, a jump, a rollover ramp, speed humps, waves, potholes, cobbles, kerbs;
//   - a skid pad (west infield), an off-road park with a rock crawl and graded slopes (east infield);
//   - a handling circuit over the hills (north-east), a worn country road (south), a city quarter (north-west).
// The terrain is earth and grass only: every road and pad is paved ground of its own (phys::PavedLayer, a 0.25 m grid
// over the terrain with the road's own shape - crown, waves, settled dips, ruts, potholes, humps - and surface), drawn as
// a mesh that samples the same grid.
// Assets (CC0, tools/fetch_assets.py): PBR materials of ambientCG, models and the sky of Poly Haven; without them the
// plain materials and boxes.
#include "core/util.h"
#include "game/game.h"
#include "vehicle/vehicle.h"
#include "world/static_model.h"

#include <algorithm>
#include <cmath>
#include <functional>

namespace bl {

using namespace phys;
namespace te = terrain_edit;

void scene_scatter_trees(Game& g, vec2 center, float radius, int count, uint32_t seed, float road_clear, const std::vector<vec2>& road); // (scenes.cpp)

namespace {

quat yaw_q(float deg) { return quat::axis_angle(vec3(0, 1, 0), deg * kDeg2Rad); }

struct MeshBuf {
    std::vector<Vertex> v;
    std::vector<uint32_t> idx;
    void quad(vec3 p0, vec3 p1, vec3 p2, vec3 p3, vec3 n, vec2 uv0, vec2 uv1, vec2 uv2, vec2 uv3) {
        const uint32_t b = (uint32_t)v.size();
        v.push_back({p0, n, uv0}), v.push_back({p1, n, uv1}), v.push_back({p2, n, uv2}), v.push_back({p3, n, uv3});
        idx.insert(idx.end(), {b, b + 1, b + 2, b, b + 2, b + 3});
    }
};

// A box's faces with its texture tiled at its real size (tile.x x tile.y metres): the four sides into `sides` (the
// texture upright), the top into `top`
void box_faces(MeshBuf* sides, MeshBuf* top, vec3 c, vec3 half, const quat& rot, vec2 tile, float top_tile) {
    const vec3 X = rot.rotate(vec3(1, 0, 0)), Y = rot.rotate(vec3(0, 1, 0)), Z = rot.rotate(vec3(0, 0, 1));
    auto face = [&](MeshBuf& m, vec3 n, float hn, vec3 a, float ha, vec3 b, float hb, vec2 t) {
        const vec3 o = c + n * hn;
        const float u1 = 2 * ha / t.x, v1 = 2 * hb / t.y;
        m.quad(o - a * ha - b * hb, o + a * ha - b * hb, o + a * ha + b * hb, o - a * ha + b * hb, n, vec2(0, v1), vec2(u1, v1), vec2(u1, 0), vec2(0, 0));
    };
    if (sides) {
        face(*sides, X, half.x, -Z, half.z, Y, half.y, tile);
        face(*sides, -X, half.x, Z, half.z, Y, half.y, tile);
        face(*sides, Z, half.z, X, half.x, Y, half.y, tile);
        face(*sides, -Z, half.z, -X, half.x, Y, half.y, tile);
    }
    if (top) face(*top, Y, half.y, X, half.x, -Z, half.z, vec2(top_tile));
}

// a static box drawn with its material tiled at its real size
void tiled_box(Game& g, vec3 c, vec3 half, const quat& rot, uint8_t surf, MaterialPtr mat, float tile) {
    g.add_static_box(c, half, rot, surf, mat, false);
    MeshBuf m;
    box_faces(&m, &m, c, half, rot, vec2(tile), tile);
    g.add_static_mesh(m.v, m.idx, mat);
}

// A model of assets/models stood at `pos` (its origin), turned `yaw_deg`, scaled; `solid`: its hull (the planes of its
// .hull inside its bounding box) collides. False when the model is not there.
bool place_model(Game& g, const char* id, vec3 pos, float yaw_deg, float scale, bool solid, uint8_t surf = SURF_CONCRETE) {
    const StaticModel* m = static_model(id);
    if (!m) return false;
    const quat q = yaw_q(yaw_deg);
    g.add_static_visual(&m->mesh, m->mat, mat4::from_trs(pos, q, vec3(scale)));
    if (solid) {
        const vec3 bc = m->bounds.center(), bh = m->bounds.extent() * 0.5f;
        g.world.statics.add_box(pos + q.rotate(bc * scale), bh * scale, q, surf);
        StaticBox& b = g.world.statics.boxes.back();
        for (const vec4& p : m->planes) b.planes.push_back(vec4(p.x, p.y, p.z, scale * (p.w - (p.x * bc.x + p.y * bc.y + p.z * bc.z))));
    }
    return true;
}

// ------------------------------------------------------------------------------------------------ roads
// How a road's surface departs from its bed (m): what makes it a road and not a billiard table
struct Profile {
    float lift = 0.03f;                         // over the ground beside it
    float crown = 0.02f;                        // its crossfall from the middle
    float wave = 0.03f, wave_len = 28.0f;       // long waves (the ground's settling)
    float ripple = 0.010f, ripple_len = 5.0f;   // short ones (the paver's, the traffic's)
    float dip = 0.045f, dip_every = 70.0f;      // a settled trench across it every so often
    float rut = 0.0f;                           // wheel ruts in each lane
    float pothole = 0.0f, pothole_every = 0.0f; // potholes: their depth, one per so many metres
    float patch = 0.0f;                         // repairs standing proud
    float cobble = 0.0f;                        // sets: each stone's own height
};
const Profile kSmooth{0.03f, 0.015f, 0.03f, 45.0f, 0.006f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
const Profile kRoad{0.03f, 0.02f, 0.07f, 26.0f, 0.022f, 4.5f, 0.08f, 55.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
const Profile kWorn{0.03f, 0.025f, 0.11f, 18.0f, 0.04f, 3.5f, 0.13f, 30.0f, 0.03f, 0.10f, 9.0f, 0.02f, 0.0f};
const Profile kLane{0.03f, 0.015f, 0.02f, 40.0f, 0.005f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

struct Road {
    std::vector<vec2> p;        // its centre line, a point every half metre
    std::vector<float> bed;
    bool closed = false;
    float half = 3.5f;
    Profile pr;
    uint32_t seed = 1;
    uint8_t surface = SURF_ASPHALT;
    std::function<float(float)> bank;                // the slope across at s (the side lat is positive on: up)
    std::function<float(float, float)> extra;        // more of its shape at (s, lat): humps, kerbs
    MaterialPtr mat;
    float tex_len = 7.0f;       // metres of road a tile
    float u_tiles = 1.0f;       // tiles across
    float mesh_step = 0.5f;
    float fall = 6.0f;          // the ground beside it brought to its edge over this
    float length() const { return p.size() < 2 ? 0.0f : 0.5f * (float)(p.size() - 1); }
    void at(float d, vec2& pos, vec2& tan, float& bedh) const {
        const int n = (int)p.size();
        const float f = clampf(d, 0.0f, length()) * 2.0f;
        const int i = std::min((int)f, n - 2);
        const float t = f - (float)i;
        pos = p[i] + (p[i + 1] - p[i]) * t, tan = normalize(p[i + 1] - p[i]), bedh = bed.empty() ? 0.0f : lerpf(bed[i], bed[i + 1], t);
    }
};

float hash1(uint32_t a, uint32_t b) {
    uint32_t h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u) * 0x85EBCA6Bu;
    h ^= h >> 15, h *= 0x2C1B3C6Du, h ^= h >> 12, h *= 0x297A2D39u, h ^= h >> 15;
    return (float)(h & 0xffffff) / (float)0x1000000;
}

// the road's height over its bed at (s, lat)
float road_shape(const Road& r, float s, float lat) {
    const Profile& q = r.pr;
    const float la = clampf(lat, -r.half, r.half);
    float h = q.lift + (r.bank ? r.bank(s) * la : -q.crown * std::fabs(la));
    h += q.wave * fbm2(s / q.wave_len, 1.7f, 2, 2.0f, 0.5f, r.seed) + q.ripple * fbm2(s / q.ripple_len, lat / (2.0f * q.ripple_len), 2, 2.0f, 0.5f, r.seed + 5);
    if (q.dip > 0 && q.dip_every > 0) { // (a trench settled across the road: 3-6 m long, deeper on one side)
        const int k = (int)std::floor(s / q.dip_every);
        for (int j = k - 1; j <= k + 1; j++) {
            const uint32_t id = (uint32_t)(j + 1000);
            const float c = ((float)j + 0.2f + 0.6f * hash1(r.seed, id)) * q.dip_every, len = 1.5f + 1.5f * hash1(r.seed + 1, id);
            const float u = (s - c) / len;
            if (std::fabs(u) < 2.5f) h -= q.dip * (0.6f + 0.8f * hash1(r.seed + 2, id)) * std::exp(-u * u) * (1.0f + 0.6f * la / r.half * (hash1(r.seed + 3, id) - 0.5f));
        }
    }
    if (q.rut > 0) // (two ruts a lane, 1.5 m apart)
        for (float lane : {-0.5f, 0.5f})
            for (float w : {-0.75f, 0.75f}) {
                const float u = (la - lane * r.half - w) / 0.28f;
                h -= q.rut * std::exp(-u * u) * (0.6f + 0.4f * fbm2(s / 15.0f, lane + w, 1, 2.0f, 0.5f, r.seed + 9));
            }
    if (q.pothole > 0 && q.pothole_every > 0) {
        const int k = (int)std::floor(s / q.pothole_every);
        for (int j = k - 1; j <= k + 1; j++) {
            const uint32_t id = (uint32_t)(j + 5000);
            const float c = ((float)j + hash1(r.seed + 11, id)) * q.pothole_every, cl = (hash1(r.seed + 12, id) - 0.5f) * 2.0f * std::max(0.0f, r.half - 1.6f);
            const float rad = 0.3f + 0.35f * hash1(r.seed + 13, id);
            const float d2 = ((s - c) * (s - c) + (la - cl) * (la - cl)) / (rad * rad);
            if (d2 < 1.0f) h -= q.pothole * (0.7f + 0.5f * hash1(r.seed + 14, id)) * (1.0f - smoothstepf(0.45f, 1.0f, d2));
        }
    }
    if (q.patch > 0) { // (repairs: rectangles a few metres long standing proud, one in three 12 m bays)
        const int k = (int)std::floor(s / 12.0f);
        const uint32_t id = (uint32_t)(k + 9000);
        if (hash1(r.seed + 21, id) < 0.34f) {
            const float c = ((float)k + 0.5f) * 12.0f, len = 1.5f + 3.0f * hash1(r.seed + 22, id), cl = (hash1(r.seed + 23, id) - 0.5f) * r.half, w = 0.8f + 0.9f * hash1(r.seed + 24, id);
            if (std::fabs(s - c) < len && std::fabs(la - cl) < w) h += q.patch;
        }
    }
    if (q.cobble > 0) h += q.cobble * (hash1((uint32_t)std::floor(s / 0.24f) + r.seed, (uint32_t)std::floor((la + 50.0f) / 0.24f)) - 0.5f) * 2.0f;
    if (r.extra) h += r.extra(s, lat);
    return h;
}

Road make_road(const std::vector<vec2>& ctrl, float half, const Profile& pr, uint32_t seed, MaterialPtr mat, bool closed = false) {
    Road r;
    r.half = half, r.closed = closed, r.pr = pr, r.seed = seed, r.mat = mat;
    float carry = 0;
    for (size_t i = 0; i + 1 < ctrl.size(); i++) {
        const vec2 d = ctrl[i + 1] - ctrl[i];
        const float L = length(d);
        if (L < 1e-4f) continue;
        float t = carry;
        for (; t < L; t += 0.5f) r.p.push_back(ctrl[i] + d * (t / L));
        carry = t - L;
    }
    if (!ctrl.empty()) r.p.push_back(closed ? r.p.front() : ctrl.back());
    return r;
}

// a smooth line through the points (Catmull-Rom), a point a metre
std::vector<vec2> spline(const std::vector<vec2>& c, bool closed) {
    std::vector<vec2> out;
    const int n = (int)c.size();
    auto at = [&](int i) { return closed ? c[((i % n) + n) % n] : c[std::clamp(i, 0, n - 1)]; };
    for (int i = 0; i < (closed ? n : n - 1); i++) {
        const vec2 p0 = at(i - 1), p1 = at(i), p2 = at(i + 1), p3 = at(i + 2);
        const int steps = std::max(2, (int)std::ceil(length(p2 - p1)));
        for (int k = 0; k < steps; k++) {
            const float t = (float)k / (float)steps, t2 = t * t, t3 = t2 * t;
            out.push_back((p1 * 2.0f + (p2 - p0) * t + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2 + (p1 * 3.0f - p0 - p2 * 3.0f + p3) * t3) * 0.5f);
        }
    }
    out.push_back(closed ? out.front() : c.back());
    return out;
}

std::vector<vec2> stadium(vec2 c, float half_straight, float radius) {
    std::vector<vec2> p;
    for (int i = 0; i <= 120; i++) { // (the east turn, the north straight westwards, the west turn, the south straight)
        const float a = -0.5f * kPi + kPi * (float)i / 120.0f;
        p.push_back(c + vec2(half_straight + radius * std::cos(a), radius * std::sin(a)));
    }
    for (int i = 0; i <= 120; i++) {
        const float a = 0.5f * kPi + kPi * (float)i / 120.0f;
        p.push_back(c + vec2(-half_straight + radius * std::cos(a), radius * std::sin(a)));
    }
    p.push_back(p.front());
    return p;
}

// Its bed into the terrain: the ground along the centre line, smoothed, with the bank across; the ground beside it
// brought up to its edge (the terrain stays earth: a strip of bare earth along its verge)
void road_bed(Game& g, Road& r) {
    Heightfield& hf = g.world.statics.terrain;
    const int nx = hf.nx(), nz = hf.nz(), n = (int)r.p.size();
    if (n < 2) return;
    r.bed.resize(n);
    for (int i = 0; i < n; i++) r.bed[i] = hf.height(r.p[i].x, r.p[i].y);
    for (int pass = 0; pass < 2; pass++) { // (twice a 40 m window)
        std::vector<float> pre(n + 1, 0.0f), sm(n);
        for (int i = 0; i < n; i++) pre[i + 1] = pre[i] + r.bed[i];
        for (int i = 0; i < n; i++) {
            if (r.closed) {
                float sum = 0;
                for (int k = -40; k <= 40; k++) sum += r.bed[(((i + k) % (n - 1)) + (n - 1)) % (n - 1)];
                sm[i] = sum / 81.0f;
            } else {
                const int a = std::max(0, i - 40), b = std::min(n - 1, i + 40);
                sm[i] = (pre[b + 1] - pre[a]) / (float)(b - a + 1);
            }
        }
        r.bed = sm;
    }
    const float reach = r.half + r.fall + 1.5f;
    std::vector<int> touched;
    static std::vector<float> best, tgt;
    best.assign((size_t)nx * nz, 1e9f), tgt.resize((size_t)nx * nz);
    for (int i = 0; i + 1 < n; i += 2) { // (a metre of road at a time)
        const int i1 = std::min(i + 2, n - 1);
        const vec2 a = r.p[i], d = r.p[i1] - a;
        const float L2 = dot(d, d);
        if (L2 < 1e-8f) continue;
        const int x0 = std::max(0, (int)std::floor((std::min(a.x, a.x + d.x) - reach - hf.origin().x) / hf.cell())),
                  x1 = std::min(nx - 1, (int)std::ceil((std::max(a.x, a.x + d.x) + reach - hf.origin().x) / hf.cell())),
                  z0 = std::max(0, (int)std::floor((std::min(a.y, a.y + d.y) - reach - hf.origin().y) / hf.cell())),
                  z1 = std::min(nz - 1, (int)std::ceil((std::max(a.y, a.y + d.y) + reach - hf.origin().y) / hf.cell()));
        for (int z = z0; z <= z1; z++)
            for (int x = x0; x <= x1; x++) {
                const vec2 q(hf.origin().x + x * hf.cell(), hf.origin().y + z * hf.cell());
                const float t = clampf(dot(q - a, d) / L2, 0.0f, 1.0f);
                const vec2 off = q - (a + d * t);
                const float dist = length(off);
                const size_t k = (size_t)z * nx + x;
                if (dist >= best[k] || dist > reach) continue;
                if (best[k] > 1e8f) touched.push_back((int)k);
                best[k] = dist;
                const float lat = clampf((d.y * off.x - d.x * off.y) / std::sqrt(L2), -r.half - 1.0f, r.half + 1.0f);
                const float s = 0.5f * ((float)i + t * (float)(i1 - i));
                tgt[k] = lerpf(r.bed[i], r.bed[i1], t) + (r.bank ? r.bank(s) * lat : 0.0f);
            }
    }
    if (g.terrain_drop.size() != (size_t)nx * nz) g.terrain_drop.assign((size_t)nx * nz, 0.0f);
    for (int k : touched) {
        const int x = k % nx, z = k / nx;
        const float w = 1.0f - smoothstepf(r.half + 0.5f, r.half + 0.5f + r.fall, best[k]);
        hf.h(x, z) = lerpf(hf.h(x, z), tgt[k], w);
        if (best[k] < r.half + 0.2f && hf.surf(x, z) == SURF_GRASS) hf.surf(x, z) = SURF_DIRT; // (the verge)
        if (best[k] < r.half - 1.3f) g.terrain_drop[k] = 0.35f;                                 // (the terrain's mesh well under the road's)
    }
}

// Its paving: the fine grid's cells under it get its height over the terrain and its surface. Where another road is
// paved already the two are blended - this one in full along its middle, the other's at its edges and its ends.
void road_pave(Game& g, const Road& r) {
    StaticWorld& st = g.world.statics;
    PavedLayer& pv = st.paved;
    const float c = pv.cell(), L = r.length();
    for (float s = 0; s <= L; s += 0.5f * c) {
        vec2 pos, tan;
        float bed;
        r.at(s, pos, tan, bed);
        const vec2 right(tan.y, -tan.x); // (lat positive: the side the bank raises)
        for (float lat = -r.half; lat <= r.half + 1e-3f; lat += 0.5f * c) {
            const vec2 q = pos + right * lat;
            const int ix = (int)std::lround((q.x - pv.origin().x) / c), iz = (int)std::lround((q.y - pv.origin().y) / c);
            const float gx = pv.origin().x + ix * c, gz = pv.origin().y + iz * c;
            const float ground = st.terrain.height(gx, gz);
            float h = bed + road_shape(r, s, lat);
            if (pv.surf(ix, iz) != PavedLayer::kNone) {
                float w = smoothstepf(0.0f, 2.0f, r.half - std::fabs(lat));
                if (!r.closed) w *= smoothstepf(0.0f, 4.0f, std::min(s, L - s));
                h = lerpf(ground + pv.dh(ix, iz), h, w);
            }
            pv.set(ix, iz, h - ground, r.surface);
        }
    }
}

// Its mesh, on the ground as it is (the terrain with the paving): the texture across its width (u_tiles times),
// tex_len metres of road a tile; a skirt down into the ground along each edge
void road_mesh(Game& g, const Road& r, float lift) {
    const StaticWorld& st = g.world.statics;
    const float L = r.length(), step = r.mesh_step;
    const int rows = (int)std::ceil(L / step) + 1, cols = (int)std::ceil(2 * r.half / step) + 1;
    MeshBuf m;
    m.v.reserve((size_t)rows * (cols + 2));
    for (int i = 0; i < rows; i++) {
        const float s = std::min(L, (float)i * step);
        vec2 pos, tan;
        float bed;
        r.at(s, pos, tan, bed);
        const vec2 right(tan.y, -tan.x);
        for (int c = -1; c <= cols; c++) {
            const bool skirt = c < 0 || c == cols;
            const float u = clampf((float)c / (float)(cols - 1), 0.0f, 1.0f), lat = (u - 0.5f) * 2.0f * r.half + (c < 0 ? -0.45f : c == cols ? 0.45f : 0.0f);
            const vec2 q = pos + right * lat;
            const float h = skirt ? st.terrain.height(q.x, q.y) - 0.12f : st.ground_height(q.x, q.y) + lift;
            const float e = 0.2f; // (its normal from the ground's slope there)
            const vec3 nrm = skirt ? vec3(0, 1, 0) : normalize(vec3(st.ground_height(q.x - e, q.y) - st.ground_height(q.x + e, q.y), 2 * e, st.ground_height(q.x, q.y - e) - st.ground_height(q.x, q.y + e)));
            m.v.push_back({vec3(q.x, h, q.y), nrm, vec2((skirt ? (c < 0 ? 0.004f : 0.996f) : u) * r.u_tiles, s / r.tex_len)});
        }
        if (i + 1 < rows)
            for (int c = 0; c < cols + 1; c++) {
                const uint32_t a = (uint32_t)(i * (cols + 2) + c), b = a + 1, a2 = a + cols + 2, b2 = a2 + 1;
                m.idx.insert(m.idx.end(), {a, b, a2, b, b2, a2});
            }
    }
    if (m.idx.size() >= 6) { // (facing up)
        const vec3 e1 = m.v[m.idx[4]].pos - m.v[m.idx[3]].pos, e2 = m.v[m.idx[5]].pos - m.v[m.idx[3]].pos;
        if (cross(e1, e2).y < 0)
            for (size_t k = 0; k + 2 < m.idx.size(); k += 3) std::swap(m.idx[k + 1], m.idx[k + 2]);
    }
    g.add_static_mesh(m.v, m.idx, r.mat);
}

// ------------------------------------------------------------------------------------------------ the crushers
// A platen moved by the scene: a heavy stiff box whose nodes are put on their way every substep (their place as built
// plus the stroke so far, at the stroke's speed), so what it meets is pushed by its contacts and cannot push it back.
// A cycle: closes at `speed` over `travel` along `axis`, holds, opens
struct Platen {
    SoftBody* body = nullptr;
    std::vector<vec3> rest;
    vec3 axis{0, -1, 0};
    float travel = 2.0f, speed = 0.5f, hold = 1.5f;
    float t = -1.0f;   // (the cycle's time; below 0: parked open)
    float off = 0, vel = 0;
    void advance(float dt) {
        float target = 0;
        if (t >= 0) {
            t += dt;
            const float close = travel / speed;
            if (t < close) target = t * speed;
            else if (t < close + hold) target = travel;
            else if (t < 2 * close + hold) target = travel - (t - close - hold) * speed;
            else t = -1.0f;
        }
        vel = (target - off) / dt, off = target;
    }
};

std::shared_ptr<Platen> add_platen(Game& g, const char* name, vec3 centre, vec3 size, vec3 axis, float travel, float speed, MaterialPtr mat) {
    SoftBoxDesc d;
    d.center = centre, d.size = size;
    d.nx = std::max(2, (int)std::round(size.x / 1.0f) + 1), d.ny = std::max(2, (int)std::round(size.y / 1.0f) + 1), d.nz = std::max(2, (int)std::round(size.z / 1.0f) + 1);
    d.mass = 20000.0f;
    d.beams = {1e9f, 4e4f, 1e12f, 1e12f, 0.0f};
    d.mat = mat;
    d.uv_scale = 0.5f;
    DynamicObject* o = g.add_object(build_soft_box(g.world, d, name));
    auto p = std::make_shared<Platen>();
    p->body = o->body, p->axis = axis, p->travel = travel, p->speed = speed;
    for (const Node& nd : o->body->nodes) p->rest.push_back(nd.p);
    o->body->can_sleep = false;
    o->body->air_drag = 0, o->body->aero_cda = 0;
    return p;
}

// (two platens of one machine - the vise's jaws - move in step: `lead` advances the cycle)
void drive_platen(const std::shared_ptr<Platen>& p, const std::shared_ptr<Platen>& lead = nullptr) {
    p->body->pre_substep = [p, lead](SoftBody& b, float dt) {
        if (lead) p->off = lead->off, p->vel = lead->vel;
        else p->advance(dt);
        const vec3 v = p->axis * p->vel;
        for (size_t i = 0; i < b.nodes.size() && i < p->rest.size(); i++) b.nodes[i].v = v + (p->rest[i] + p->axis * p->off - b.nodes[i].p) * 200.0f;
    };
}

} // namespace

void scene_test_site(Game& g) {
    auto& A = SharedAssets::get();
    const MaterialPtr m_road2 = pbr_material("road_two_lane", A.concrete), m_road4 = pbr_material("road_multi_lane", A.concrete), m_asphalt = pbr_material("asphalt_fine", A.concrete),
                      m_worn = pbr_material("asphalt_worn", A.concrete), m_cobbles = pbr_material("cobbles", A.stone), m_concrete = pbr_material("concrete", A.concrete),
                      m_concrete_dark = pbr_material("concrete_dark", A.concrete), m_paving = pbr_material("paving", A.concrete), m_rock = pbr_material("rock", A.stone),
                      m_plate = pbr_material("metal_plate", A.metal), m_metal = pbr_material("metal", A.metal);
    // ---- the sky and the light: the panorama's sun (every scene's: Game::load_scene), a clearer air
    if (g.light.sky_panorama) g.light.fog_density = 0.00045f;

    // ---- the ground: hills outside, level inside the oval
    g.create_terrain(1401, 1001, 1.0f, vec2(-700, -500));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 9.0f, 150.0f, 4, 11);
    const vec2 off_c(450, 0);       // the off-road park (east infield)
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) {
            const float wx = hf.origin().x + x, wz = hf.origin().y + z;
            const float d = length(vec2(wx - clampf(wx, -350.0f, 350.0f), wz)) - 200.0f; // (from the oval's line: inside below 0)
            hf.h(x, z) *= smoothstepf(10.0f, 120.0f, d);
            // the off-road park: rough ground
            const float k = 1.0f - smoothstepf(70.0f, 95.0f, length(vec2(wx, wz) - off_c));
            if (k > 0) {
                hf.h(x, z) += k * (0.5f + 0.9f * fbm2(wx * 0.11f, wz * 0.11f, 3, 2.0f, 0.5f, 71) + 0.35f * fbm2(wx * 0.4f, wz * 0.4f, 2, 2.0f, 0.5f, 72));
                if (k > 0.5f && fbm2(wx * 0.05f, wz * 0.05f, 2, 2.0f, 0.5f, 75) > -0.1f) hf.surf(x, z) = SURF_DIRT;
            }
            const int e = std::min(std::min(x, hf.nx() - 1 - x), std::min(z, hf.nz() - 1 - z)); // (a bank round the map's edge)
            if (e < 16) hf.h(x, z) += 8.0f * (1.0f - smoothstepf(0.0f, 16.0f, (float)e));
        }
    te::auto_surfaces(hf);
    te::flatten_circle(hf, off_c + vec2(40, -45), 14, -0.5f, 6.0f, SURF_MUD);
    te::flatten_rect(hf, vec2(-210, 370), vec2(135, 85), 0, hf.height(-210, 370), 25.0f); // the city quarter's ground

    // ---- the roads
    std::vector<Road> roads;
    // the oval: 16 m wide, its turns banked 14 degrees (eased in over their first and last 60 m)
    {
        Road r = make_road(stadium(vec2(0, 0), 350.0f, 200.0f), 8.0f, kSmooth, 40, m_road4, true);
        const float turn = kPi * 200.0f, straight = 700.0f;
        r.bank = [turn, straight](float s) {
            s = std::fmod(s, 2 * (turn + straight));
            const float in = s < turn ? s : (s >= turn + straight && s < 2 * turn + straight) ? s - turn - straight : -1.0f;
            return in < 0 ? 0.0f : 0.25f * smoothstepf(0.0f, 60.0f, std::min(in, turn - in));
        };
        r.tex_len = 16.0f, r.fall = 22.0f;
        roads.push_back(std::move(r));
    }
    // the feeder (west) and the return road (east) across the lanes' ends
    roads.push_back(make_road({vec2(-330, -262), vec2(-330, 300)}, 4.0f, kRoad, 21, m_road2));
    roads.push_back(make_road({vec2(300, -185), vec2(300, 255)}, 3.5f, kRoad, 22, m_road2));
    // the handling circuit over the hills (north-east) and the link to it
    roads.push_back(make_road(spline({vec2(100, 285), vec2(220, 262), vec2(330, 300), vec2(420, 258), vec2(540, 270), vec2(615, 330), vec2(595, 420), vec2(505, 458),
                                      vec2(440, 400), vec2(360, 442), vec2(260, 462), vec2(170, 432), vec2(205, 362), vec2(120, 345)}, true),
                              4.5f, kRoad, 23, m_road2, true));
    roads.push_back(make_road(spline({vec2(300, 255), vec2(303, 275), vec2(318, 292)}, false), 3.5f, kRoad, 24, m_road2));
    // the country road (south): worn - waves, settled trenches, ruts, potholes, patches
    roads.push_back(make_road(spline({vec2(-330, -262), vec2(-322, -300), vec2(-270, -345), vec2(-180, -330), vec2(-90, -390), vec2(20, -420), vec2(130, -360), vec2(230, -380),
                                      vec2(340, -440), vec2(460, -410), vec2(540, -330), vec2(620, -300)}, false),
                              3.5f, kWorn, 25, m_worn));
    roads.back().mesh_step = 0.25f;
    // the city's streets (north-west): three each way, the middle one across cobbled
    for (float x : {-300.0f, -200.0f, -100.0f}) roads.push_back(make_road({vec2(x, 300), vec2(x, 440)}, 3.5f, kRoad, 30 + (uint32_t)(-x), m_road2));
    for (float z : {300.0f, 370.0f, 440.0f}) {
        roads.push_back(make_road({vec2(-334, z), vec2(-92, z)}, 3.5f, kRoad, 60 + (uint32_t)z, z == 370.0f ? m_cobbles : m_road2));
        if (z == 370.0f) roads.back().pr.cobble = 0.012f, roads.back().surface = SURF_CONCRETE, roads.back().tex_len = 3.5f, roads.back().u_tiles = 2.0f, roads.back().mesh_step = 0.25f;
    }
    // the skid pad (west infield): a disc of 50 m, and its link
    {
        std::vector<vec2> ring;
        for (int i = 0; i <= 72; i++) ring.push_back(vec2(-450, 0) + vec2(std::cos(2 * kPi * i / 72.0f), std::sin(2 * kPi * i / 72.0f)) * 26.0f);
        Road r = make_road(ring, 24.0f, kSmooth, 90, m_asphalt, true);
        r.pr.crown = 0, r.tex_len = 6.0f, r.u_tiles = 8.0f, r.mesh_step = 1.0f, r.fall = 4.0f;
        roads.push_back(std::move(r));
        roads.push_back(make_road({vec2(-330, 0), vec2(-402, 0)}, 3.5f, kRoad, 91, m_road2));
    }
    // the off-road park's link (east infield)
    roads.push_back(make_road({vec2(300, 0), vec2(372, 0)}, 3.5f, kRoad, 92, m_road2));

    // the test lanes: each from the feeder (x -330) to the return road (x 300), its test at x 150 - 480 m to get up to speed
    struct Lane {
        const char* name;
        float z;
    };
    const float kTestX = 150.0f, kLaneX0 = -330.0f;
    std::vector<Lane> lanes;
    const char* lane_names[] = {"WALL", "OFFSET 40%", "SMALL OVERLAP 25%", "POLE 25 cm", "POLE 70 cm", "POSTS", "BARRIERS", "BLOCKS", "WEDGE: SIDES", "WEDGE: ROOF",
                                "PRESS",  "VISE", "JUMP", "ROLLOVER", "SPEED HUMPS", "WAVES", "POTHOLES", "COBBLES", "KERBS", "FREE"};
    for (int i = 0; i < 20; i++) lanes.push_back({lane_names[i], 171.0f - 18.0f * i});
    const size_t first_lane = roads.size();
    for (size_t i = 0; i < lanes.size(); i++) {
        Road r = make_road({vec2(kLaneX0, lanes[i].z), vec2(300, lanes[i].z)}, 3.0f, kLane, 100 + (uint32_t)i, m_road2);
        r.tex_len = 6.0f, r.fall = 3.0f;
        const float t0 = kTestX - kLaneX0; // (the test's place along the lane)
        const std::string name = lanes[i].name;
        if (name == "SPEED HUMPS") { // six humps 8 cm high, 1.6 m long, 14 m apart
            r.extra = [t0](float s, float) {
                const float u = std::fmod(s - (t0 - 70.0f) + 1400.0f, 14.0f) - 7.0f;
                return s > t0 - 77.0f && s < t0 + 7.0f && std::fabs(u) < 0.8f ? 0.08f * (0.5f + 0.5f * std::cos(u / 0.8f * kPi)) : 0.0f;
            };
            r.mesh_step = 0.25f;
        }
        if (name == "WAVES") { // 60 m of washboard (3 cm, 0.9 m), then 80 m of long waves out of step side to side (10 cm, 7 m)
            r.extra = [t0](float s, float lat) {
                if (s > t0 - 150.0f && s < t0 - 90.0f) return 0.015f * std::sin((s - t0) * 2 * kPi / 0.9f) * smoothstepf(0.0f, 4.0f, std::min(s - (t0 - 150.0f), t0 - 90.0f - s));
                if (s > t0 - 80.0f && s < t0)
                    return 0.05f * std::sin((s - t0) * 2 * kPi / 7.0f + (lat > 0 ? kPi : 0.0f)) * smoothstepf(0.0f, 6.0f, std::min(s - (t0 - 80.0f), t0 - s)) * smoothstepf(0.0f, 0.6f, std::fabs(lat));
                return 0.0f;
            };
            r.mesh_step = 0.25f;
        }
        if (name == "POTHOLES") r.pr = kWorn, r.pr.pothole = 0.09f, r.pr.pothole_every = 4.0f, r.pr.dip_every = 30.0f, r.mat = m_worn, r.mesh_step = 0.25f;
        if (name == "COBBLES") r.pr.cobble = 0.018f, r.pr.wave = 0.03f, r.pr.ripple = 0.02f, r.mat = m_cobbles, r.surface = SURF_CONCRETE, r.tex_len = 3.0f, r.u_tiles = 2.0f, r.mesh_step = 0.25f;
        if (name == "KERBS") { // kerbs across: 5, 8, 12, 16 cm up, a ramp back down over 3 m, 30 m apart
            r.extra = [t0](float s, float) {
                const float hs[4] = {0.05f, 0.08f, 0.12f, 0.16f};
                for (int k = 0; k < 4; k++) {
                    const float u = s - (t0 - 90.0f + 30.0f * k);
                    if (u >= 0 && u < 3.5f) return hs[k] * (u < 0.5f ? 1.0f : 1.0f - (u - 0.5f) / 3.0f);
                }
                return 0.0f;
            };
            r.mesh_step = 0.25f;
        }
        roads.push_back(std::move(r));
    }
    // beds into the terrain
    for (Road& r : roads) road_bed(g, r);
    // off-road park: three slopes up a mound (20%, 40%, 60%), concrete - the mound in the terrain, the slopes paved on it
    const vec2 mound(450, 55);
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) {
            const vec2 q(hf.origin().x + x - mound.x, hf.origin().y + z - mound.y);
            // (a plateau 6 m up, 16 m across each way; its west face three graded slopes side by side, the rest steep)
            const float west = -q.x - 8.0f; // (m out from the plateau's west edge)
            float h = 6.0f * (1.0f - smoothstepf(8.0f, 16.0f, std::max(std::fabs(q.x), std::fabs(q.y))));
            if (west > 0 && std::fabs(q.y) < 9.0f) h = std::max(0.0f, 6.0f - west * (q.y < -3.0f ? 0.2f : q.y < 3.0f ? 0.4f : 0.6f));
            if (h > 0.01f) hf.h(x, z) = std::max(hf.h(x, z), h);
        }
    const size_t first_slope = roads.size();
    for (int i = 0; i < 3; i++) {
        const float grade = 0.2f + 0.2f * i, zc = mound.y - 6.0f + 6.0f * i;
        Road r = make_road({vec2(mound.x - 8.0f - 6.0f / grade - 6.0f, zc), vec2(mound.x - 2.0f, zc)}, 2.6f, kLane, 95 + (uint32_t)i, m_concrete);
        r.pr.crown = 0, r.pr.wave = 0, r.pr.ripple = 0, r.surface = SURF_CONCRETE, r.tex_len = 3.0f, r.u_tiles = 2.0f;
        r.bed.resize(r.p.size());
        for (size_t k = 0; k < r.p.size(); k++) r.bed[k] = hf.height(r.p[k].x, r.p[k].y); // (the mound as shaped)
        roads.push_back(std::move(r));
    }
    g.world.statics.paved.create(hf.origin(), hf.size(), 0.25f);
    hf.update_bounds();
    // (the lanes, the links and the slopes first, the roads they meet over them: the oval's and the feeder's shape wins where they cross)
    std::vector<size_t> order;
    for (size_t i = first_lane; i < roads.size(); i++) order.push_back(i);
    for (size_t i = first_lane; i-- > 0;) order.push_back(i);
    for (size_t i : order) road_pave(g, roads[i]);
    g.finish_terrain();
    for (size_t k = 0; k < order.size(); k++) road_mesh(g, roads[order[k]], 0.004f + 0.0025f * (float)(order[k] < first_lane ? 8 + (first_lane - order[k]) % 8 : k % 3));
    (void)first_slope;
    for (float sx : {-1.0f, 1.0f}) g.add_static_box(vec3(sx * 701.0f, 10, 0), vec3(1.0f, 40, 502), quat(), SURF_CONCRETE, nullptr, false); // (walls behind the edge's bank)
    for (float sz : {-1.0f, 1.0f}) g.add_static_box(vec3(0, 10, sz * 501.0f), vec3(702, 40, 1.0f), quat(), SURF_CONCRETE, nullptr, false);
    auto ground = [&](float x, float z) { return g.world.statics.ground_height(x, z); };

    // ---- the lanes' tests (each at x 150 on its lane) and their names at the lane's start
    std::shared_ptr<Platen> press, jaw0;
    vec3 press_at(0), vise_at(0);
    int post = 0, block = 0;
    for (const Lane& ln : lanes) {
        const std::string name = ln.name;
        const float z = ln.z, X = kTestX;
        g.labels.push_back({vec3(kLaneX0 + 14.0f, 2.2f, z), name, vec4(1, 0.9f, 0.4f, 1)});
        g.labels.push_back({vec3(X - 30.0f, 3.0f, z), name, vec4(1, 1, 1, 1)});
        if (name == "WALL") tiled_box(g, vec3(X + 1.0f, 1.3f, z), vec3(1.0f, 1.3f, 3.4f), quat(), SURF_CONCRETE, m_concrete, 2.5f);
        if (name == "OFFSET 40%") tiled_box(g, vec3(X + 1.0f, 1.3f, z + 0.17f + 1.5f), vec3(1.0f, 1.3f, 1.5f), quat(), SURF_CONCRETE, m_concrete, 2.5f);
        if (name == "SMALL OVERLAP 25%") tiled_box(g, vec3(X + 1.0f, 1.3f, z + 0.44f + 1.3f), vec3(1.0f, 1.3f, 1.3f), quat(), SURF_CONCRETE, m_concrete_dark, 2.5f);
        if (name == "POLE 25 cm") g.add_static_cylinder(vec3(X, 0, z), 0.125f, 4.0f, SURF_METAL, m_metal);
        if (name == "POLE 70 cm") g.add_static_cylinder(vec3(X, 0, z), 0.35f, 4.5f, SURF_CONCRETE, m_concrete);
        if (name == "POSTS") // timber and steel posts that bend and break, stronger down the lane
            for (int i = 0; i < 5; i++) {
                PoleDesc pd;
                pd.base = vec3(X + 14.0f * i, ground(X + 14.0f * i, z), z + (i % 2 ? 0.7f : -0.7f));
                pd.length = 3.5f + 0.5f * i, pd.radius = 0.05f + 0.02f * i, pd.mass = 40 + 25.0f * i, pd.k_ang = 3e4f * (1 + i), pd.yield_deg = i % 2 ? 5.0f : 0.0f;
                pd.break_torque = 4000.0f * (1 + i), pd.mat = i % 2 ? A.metal : A.wood;
                g.add_object(build_pole(g.world, pd, format("post%d", post++)));
            }
        if (name == "BARRIERS") // concrete road barriers set at 20 degrees across the lane: a glancing blow
            for (int i = 0; i < 9; i++) {
                const vec3 p(X + 1.5f * i * std::cos(20 * kDeg2Rad), 0, z - 3.0f + 1.5f * i * std::sin(20 * kDeg2Rad) + 0.4f);
                if (!place_model(g, "concrete_road_barrier", vec3(p.x, ground(p.x, p.z), p.z), -20.0f, 1.0f, true))
                    tiled_box(g, vec3(p.x, 0.42f, p.z), vec3(0.77f, 0.42f, 0.3f), yaw_q(-20), SURF_CONCRETE, m_concrete, 2.0f);
            }
        if (name == "BLOCKS") // concrete blocks that slide when struck: 0.3 t to 2.5 t
            for (int i = 0; i < 4; i++) {
                SoftBoxDesc d;
                const float sz = 0.6f + 0.25f * i, px = X + 16.0f * i;
                d.center = vec3(px, ground(px, z) + 0.5f * sz + 0.02f, z + (i % 2 ? 0.5f : -0.5f)), d.size = vec3(sz);
                d.mass = 2300.0f * sz * sz * sz * 0.6f, d.beams = {4e7f, 8e3f, 1e12f, 1e12f, 0.0f}, d.mat = m_concrete, d.uv_scale = sz / 2.0f;
                g.add_object(build_soft_box(g.world, d, format("block%d", block++)));
            }
        if (name == "WEDGE: SIDES") { // two walls closing from 3.4 m to 1.0 m over 26 m
            const float len = 26.0f, w0 = 3.4f, w1 = 1.0f, ang = std::atan2(0.5f * (w0 - w1), len);
            for (float s : {-1.0f, 1.0f})
                tiled_box(g, vec3(X + 0.5f * len, 0.9f, z + s * (0.25f * (w0 + w1) + 0.35f)), vec3(0.5f * len / std::cos(ang), 0.9f, 0.35f), quat::axis_angle(vec3(0, 1, 0), s * ang),
                          SURF_CONCRETE, m_concrete, 2.5f);
        }
        if (name == "WEDGE: ROOF") { // a roof coming down from 2.3 m to 0.5 m over 26 m between two walls 3.6 m apart
            const float len = 26.0f, h0 = 2.3f, h1 = 0.5f, ang = std::atan2(h0 - h1, len);
            for (float s : {-1.0f, 1.0f}) tiled_box(g, vec3(X + 0.5f * len, 1.35f, z + s * 2.15f), vec3(0.5f * len, 1.35f, 0.35f), quat(), SURF_CONCRETE, m_concrete, 2.5f);
            const quat tilt = quat::axis_angle(vec3(0, 0, 1), -ang); // (lower towards +x)
            tiled_box(g, vec3(X + 0.5f * len, 0.5f * (h0 + h1), z) + tilt.rotate(vec3(0, 1, 0)) * 0.3f, vec3(0.5f * len / std::cos(ang), 0.3f, 1.8f), tilt, SURF_CONCRETE, m_concrete_dark, 2.5f);
        }
        if (name == "PRESS") { // a 20 t platen 7 x 5 m coming down from 3.2 m to 0.45 m
            press_at = vec3(X, ground(X, z), z);
            press = add_platen(g, "press", press_at + vec3(0, 3.4f, 0), vec3(7.0f, 0.4f, 5.0f), vec3(0, -1, 0), 2.75f, 0.45f, m_plate);
            drive_platen(press);
            for (float sx : {-1.0f, 1.0f})
                for (float sz : {-1.0f, 1.0f}) g.add_static_cylinder(press_at + vec3(sx * 3.9f, 0, sz * 3.1f), 0.22f, 4.4f, SURF_METAL, m_metal);
            for (float sx : {-1.0f, 1.0f}) tiled_box(g, press_at + vec3(sx * 3.9f, 4.65f, 0), vec3(0.25f, 0.25f, 3.4f), quat(), SURF_METAL, m_metal, 1.5f);
        }
        if (name == "VISE") { // two jaws 7 m long closing from 5 m to 0.7 m
            vise_at = vec3(X, ground(X, z), z);
            jaw0 = add_platen(g, "vise jaw", vise_at + vec3(0, 1.25f, -2.7f), vec3(7.0f, 2.3f, 0.4f), vec3(0, 0, 1), 2.15f, 0.4f, m_plate);
            auto jaw1 = add_platen(g, "vise jaw", vise_at + vec3(0, 1.25f, 2.7f), vec3(7.0f, 2.3f, 0.4f), vec3(0, 0, -1), 2.15f, 0.4f, m_plate);
            drive_platen(jaw0), drive_platen(jaw1, jaw0);
            for (float sz : {-1.0f, 1.0f}) tiled_box(g, vise_at + vec3(0, 1.3f, sz * 3.6f), vec3(3.7f, 1.3f, 0.5f), quat(), SURF_CONCRETE, m_concrete_dark, 2.5f); // (the jaws' housings)
        }
        if (name == "JUMP") // a kicker 10 m long to 1.5 m, a landing ramp 30 m on
            for (int k = 0; k < 2; k++) {
                const float len = k ? 14.0f : 10.0f, hgt = k ? 1.2f : 1.5f, ang = std::atan2(hgt, len), x0 = k ? X + 30.0f : X;
                const quat q = quat::axis_angle(vec3(0, 0, 1), k ? -ang : ang);
                tiled_box(g, vec3(x0 + 0.5f * len, 0.5f * hgt, z) - q.rotate(vec3(0, 1, 0)) * 0.25f, vec3(0.5f * std::sqrt(len * len + hgt * hgt), 0.25f, 2.8f), q, SURF_CONCRETE, m_concrete, 2.5f);
            }
        if (name == "ROLLOVER") { // a ramp under the left wheels alone: 9 m long to 1.1 m
            const float len = 9.0f, hgt = 1.1f, ang = std::atan2(hgt, len);
            const quat q = quat::axis_angle(vec3(0, 0, 1), ang);
            tiled_box(g, vec3(X + 0.5f * len, 0.5f * hgt, z + 1.0f) - q.rotate(vec3(0, 1, 0)) * 0.2f, vec3(0.5f * std::sqrt(len * len + hgt * hgt), 0.2f, 0.6f), q, SURF_METAL, m_plate, 1.0f);
        }
    }
    if (press && jaw0) {
        g.scene_actions.push_back({"Run the press", [press](Game&) { if (press->t < 0) press->t = 0; }});
        g.scene_actions.push_back({"Run the vise", [jaw0](Game&) { if (jaw0->t < 0) jaw0->t = 0; }});
        g.scene_update = [press, jaw0, press_at, vise_at, wait = vec2(0)](Game& gg, float dt) mutable {
            // (a vehicle standing inside for a second and a half starts the machine)
            bool in_press = false, in_vise = false;
            for (const auto& v : gg.vehicles) {
                const vec3 p = v->position();
                const bool still = length(v->body->average_velocity()) < 1.0f;
                in_press |= still && std::fabs(p.x - press_at.x) < 3.0f && std::fabs(p.z - press_at.z) < 2.0f && p.y < 3;
                in_vise |= still && std::fabs(p.x - vise_at.x) < 3.0f && std::fabs(p.z - vise_at.z) < 1.6f && p.y < 3;
            }
            wait.x = in_press && press->t < 0 ? wait.x + dt : 0.0f, wait.y = in_vise && jaw0->t < 0 ? wait.y + dt : 0.0f;
            if (wait.x > 1.5f) press->t = 0, wait.x = 0;
            if (wait.y > 1.5f) jaw0->t = 0, wait.y = 0;
        };
    }

    // ---- the Scene menu: the player's vehicle put in a zone or at a lane's start ("Go to"), or sent at a lane's test
    // from 60 m before it at a speed ("Crash test")
    {
        auto go = [](vec3 pos, float yaw, float kmh) {
            return [pos, yaw, kmh](Game& gg) {
                Vehicle* v = gg.player_vehicle();
                if (!v) return;
                v->reset(pos, yaw);
                if (kmh > 0) v->launch(yaw_q(yaw).rotate(vec3(0, 0, 1)) * (kmh / 3.6f));
            };
        };
        auto at = [&](float x, float z) { return vec3(x, ground(x, z), z); };
        const std::pair<const char*, vec4> zones[] = {{"Start (the feeder road)", vec4(-330, 9, 0, 0)},      {"Oval: the north straight", vec4(-300, 200, 90, 0)},
                                                      {"Skid pad", vec4(-450, 0, 0, 0)},                     {"Off-road park", vec4(385, 0, 90, 0)},
                                                      {"Rock crawl", vec4(390, -42, 90, 0)},                 {"Slopes 20 / 40 / 60%", vec4(395, 55, 90, 0)},
                                                      {"Handling circuit", vec4(330, 300, 60, 0)},           {"City quarter", vec4(-320, 300, 90, 0)},
                                                      {"Country road (worn)", vec4(-322, -300, 170, 0)}};
        for (const auto& [label, p] : zones) g.scene_actions.push_back({std::string("Go to/") + label, go(at(p.x, p.y), p.z, 0)});
        for (const Lane& ln : lanes) g.scene_actions.push_back({std::string("Go to/Lane: ") + ln.name, go(at(kLaneX0 + 12.0f, ln.z), 90, 0)});
        for (const Lane& ln : lanes) {
            const std::string name = ln.name;
            if (name == "PRESS" || name == "VISE") { // (put inside: the machine starts on its own)
                g.scene_actions.push_back({"Crash test/" + name, go(at(kTestX, ln.z), 90, 0)});
                continue;
            }
            if (name == "FREE" || name == "SPEED HUMPS" || name == "WAVES" || name == "POTHOLES" || name == "COBBLES" || name == "KERBS") continue;
            const bool slow = name == "WEDGE: SIDES" || name == "WEDGE: ROOF" || name == "ROLLOVER";
            for (float kmh : {slow ? 30.0f : 50.0f, slow ? 60.0f : 80.0f, slow ? 90.0f : 120.0f})
                g.scene_actions.push_back({"Crash test/" + name + format(" at %.0f km/h", kmh), go(at(kTestX - 60.0f, ln.z), 90, kmh)});
        }
        for (const char* name : {"SPEED HUMPS", "WAVES", "POTHOLES", "COBBLES", "KERBS"})
            for (const Lane& ln : lanes)
                if (name == std::string(ln.name))
                    for (float kmh : {40.0f, 80.0f}) g.scene_actions.push_back({std::string("Ride test/") + name + format(" at %.0f km/h", kmh), go(at(kTestX - 170.0f, ln.z), 90, kmh)});
    }

    // ---- the off-road park: boulders (the models' hulls collide), smaller rocks between
    {
        Rng rng(91);
        const char* big[] = {"namaqualand_boulder_04", "boulder_01", "namaqualand_boulder_02", "namaqualand_boulder_05"};
        const char* small[] = {"rock_07", "moon_rock_01", "rock_09", "moon_rock_05", "moon_rock_03"};
        // (a rock crawl 70 x 36 m thick with them, the rest of the park strewn)
        const vec2 crawl(430, -42), crawl_h(35, 18);
        for (int i = 0; i < 150; i++) {
            const bool dense = i < 100;
            vec2 p;
            if (dense) p = crawl + vec2(rng.range(-crawl_h.x, crawl_h.x), rng.range(-crawl_h.y, crawl_h.y));
            else {
                const float a = rng.range(0, 2 * kPi), rad = 12.0f + 70.0f * std::sqrt(rng.uniform());
                p = off_c + vec2(std::cos(a), std::sin(a)) * rad;
            }
            if (length(p - mound) < 34.0f || (std::fabs(p.y) < 6.0f && p.x < off_c.x - 20.0f) || length(p - (off_c + vec2(40, -45))) < 15.0f) continue; // (the mound, the way in, the mud)
            const bool b = i % 5 == 0;
            const char* id = b ? big[(i / 5) % 4] : small[i % 5];
            const StaticModel* m = static_model(id);
            const float want = b ? rng.range(1.8f, 3.4f) : rng.range(0.6f, 1.7f);
            if (m) {
                const vec3 e = m->bounds.extent();
                const float sc = want / std::max(e.x, e.z);
                place_model(g, id, vec3(p.x, hf.height(p.x, p.y) - m->bounds.mn.y * sc - 0.2f * e.y * sc, p.y), rng.range(0, 360), sc, true, SURF_ROCK);
            } else {
                tiled_box(g, vec3(p.x, hf.height(p.x, p.y) + 0.1f * want, p.y), vec3(0.5f * want, 0.3f * want, 0.4f * want), quat::axis_angle(rng.unit_vector(), rng.range(0, 0.6f)), SURF_ROCK, m_rock, 2.0f);
            }
        }
        g.labels.push_back({vec3(crawl.x - crawl_h.x, 3.0f, crawl.y), "ROCK CRAWL", vec4(1, 1, 1, 1)});
        g.labels.push_back({vec3(off_c.x - 70, 3.5f, 0), "OFF-ROAD PARK", vec4(1, 1, 1, 1)});
        g.labels.push_back({vec3(mound.x - 44, 2.5f, mound.y), "SLOPES 20 / 40 / 60%", vec4(1, 1, 1, 1)});
    }

    // ---- the city quarter: four blocks between the streets - a pavement on a 14 cm kerb, buildings, street lamps,
    // hydrants, barriers
    {
        struct Front {
            MaterialPtr mat;
            vec2 tile;      // (m a tile across, up)
            float floor;    // (m a floor: the heights whole floors)
        };
        const Front fronts[4] = {{pbr_material("facade_brick", A.concrete), vec2(19.2f, 18.0f), 3.0f},
                                 {pbr_material("facade_flats", A.concrete), vec2(17.0f, 21.0f), 3.0f},
                                 {pbr_material("facade_office", A.concrete), vec2(20.0f, 25.6f), 3.2f},
                                 {pbr_material("facade_glass", A.concrete), vec2(12.0f, 12.0f), 3.0f}};
        Rng rng(77);
        int k = 0, lamp = 0;
        for (int bx = 0; bx < 2; bx++)
            for (int bz = 0; bz < 2; bz++) {
                const vec2 c(-250.0f + 100.0f * bx, 335.0f + 70.0f * bz), half(46.3f, 31.3f); // (the block between the streets' edges)
                const float y0 = ground(c.x, c.y);
                tiled_box(g, vec3(c.x, y0 - 0.36f, c.y), vec3(half.x, 0.5f, half.y), quat(), SURF_CONCRETE, m_paving, 2.5f); // the pavement
                for (int b = 0; b < 3; b++) {
                    const Front& f = fronts[(k++ + bz) % 4];
                    const int floors = 3 + (int)(rng.uniform() * 7);
                    const float h = floors * f.floor;
                    const vec3 bc(c.x + (b - 1) * 29.5f, y0 + 0.14f + 0.5f * h, c.y), bh(13.0f - rng.uniform(), 0.5f * h, 26.0f - 3.0f * rng.uniform());
                    g.add_static_box(bc, bh, quat(), SURF_CONCRETE, nullptr, false);
                    MeshBuf walls, roof;
                    box_faces(&walls, &roof, bc, bh, quat(), f.tile, 6.0f);
                    for (size_t q = 0; q + 3 < walls.v.size(); q += 4) { // (whole tiles round each wall, whole floors up it)
                        const float w = length(walls.v[q + 1].pos - walls.v[q].pos);
                        walls.v[q + 1].uv.x = walls.v[q + 2].uv.x = std::max(1.0f, std::round(w / f.tile.x));
                        walls.v[q].uv.y = walls.v[q + 1].uv.y = (float)floors / std::round(f.tile.y / f.floor);
                    }
                    g.add_static_mesh(walls.v, walls.idx, f.mat);
                    g.add_static_mesh(roof.v, roof.idx, m_concrete_dark);
                }
                // street lamps on posts that bend and break off, hydrants at two corners, barriers at the others
                for (float sx : {-0.8f, -0.27f, 0.27f, 0.8f})
                    for (float sz : {-1.0f, 1.0f}) {
                        PoleDesc pd;
                        pd.base = vec3(c.x + sx * half.x, y0 + 0.14f, c.y + sz * (half.y - 0.8f));
                        pd.length = 6.5f, pd.radius = 0.06f, pd.mass = 70, pd.k_ang = 6e4f, pd.yield_deg = 6, pd.break_torque = 9000, pd.tip_mass = 6;
                        pd.mat = A.metal;
                        g.add_object(build_pole(g.world, pd, format("lamp%d", lamp++)));
                    }
                for (float sx : {-1.0f, 1.0f})
                    for (float sz : {-1.0f, 1.0f}) {
                        const vec3 p(c.x + sx * (half.x - 0.9f), y0 + 0.14f, c.y + sz * (half.y - 0.9f));
                        if (sx * sz > 0) {
                            if (place_model(g, "fire_hydrant", p, 0, 1.0f, false)) g.world.statics.cylinders.push_back({p, 0.13f, 0.75f, SURF_METAL});
                            else g.add_static_cylinder(p, 0.12f, 0.8f, SURF_METAL, A.red);
                        } else if (!place_model(g, "concrete_road_barrier_02", p + vec3(-sx * 2.2f, 0, 0), 0, 1.0f, true))
                            tiled_box(g, p + vec3(-sx * 2.2f, 0.4f, 0), vec3(0.8f, 0.4f, 0.25f), quat(), SURF_CONCRETE, m_concrete, 2.0f);
                    }
            }
        g.labels.push_back({vec3(-330, 6, 300), "CITY QUARTER", vec4(1, 1, 1, 1)});
    }

    // ---- trees outside the oval, clear of the roads
    {
        std::vector<vec2> clear;
        for (const Road& r : roads)
            for (size_t i = 0; i < r.p.size(); i += 8) clear.push_back(r.p[i]);
        for (const vec2 c : {vec2(-520, 330), vec2(-480, -340), vec2(0, -330), vec2(350, 360), vec2(560, -160), vec2(-600, 60), vec2(560, 200), vec2(120, 420)})
            scene_scatter_trees(g, c, 150, 110, 300 + (uint32_t)(c.x + c.y + 2000), 14.0f, clear);
    }
    g.labels.push_back({vec3(0, 3, 200), "OVAL", vec4(1, 1, 1, 1)});
    g.labels.push_back({vec3(-450, 3, 30), "SKID PAD", vec4(1, 1, 1, 1)});
    g.labels.push_back({vec3(318, 4, 292), "HANDLING CIRCUIT", vec4(1, 1, 1, 1)});
    g.labels.push_back({vec3(-330, 4, -262), "COUNTRY ROAD", vec4(1, 1, 1, 1)});
    g.set_spawn(vec3(-330, 0.0f, 9), 0);
    g.scene_hint = "Test lanes run east from the feeder road, 480 m to each test (their names at the start): walls, poles, barriers, wedges, "
                   "the press and the vise (stop inside), jump, humps, waves, potholes, cobbles, kerbs. Round them the banked oval; skid pad "
                   "(W), off-road park and slopes (E), handling circuit (NE), city (NW), a worn country road (S). Scene menu: Go to (zones, lanes), "
                   "Crash test and Ride test (the vehicle sent at a test at a speed).";
}

} // namespace bl
