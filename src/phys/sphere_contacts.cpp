// Sphere contacts of sheet bodies (SoftBody::sphere_contacts): a sheet's surface as small spheres at its nodes, at thirds
// along its edges and at the centres of its triangles, each 0.4 of that spacing across (the thickness the contact
// leaves between two sheets: about 3 cm on the barrel; one sphere a triangle, 0.4 of it across, left 8). Two spheres of different
// bodies that overlap push apart: two rims crossing edge to edge (which node against triangle misses) meet, a drum stands
// on another. (A sheet with itself: gone; its pushes on a dented drum drove it round on the ground.) The response is a projection after the short step's integration, as the membrane's: the
// spheres' nodes moved apart along the line of centres in proportion to their weights and inverse masses (positions
// only, a node at no more than 0.2 m/s), the closing velocity taken out, sliding resisted by Coulomb friction. A ball
// (SoftBody::sphere_ball: a projectile) is one sphere, the whole body. A sleeping sheet is an immovable obstacle, woken by a partner
// that moves (as the other contacts: a barrel standing on a sleeping one fell through it). Deterministic: one thread per
// island, sorted cells. A sheet with sphere_target meets only the balls so (the barrels: with each other node against
// triangle, their spheres between them cost a pile of barrels most of its step).
#include "phys/world.h"

#include "core/profiler.h"
#include "phys/shell_util.h"

#include <algorithm>
#include <cmath>

namespace bl::phys {

using namespace shell_detail;

namespace {

// a triangle's size: the leg of the right isosceles triangle of its rest area
inline float tri_size(const Shell& s) { return std::sqrt(2.0f * std::max(1e-10f, s.area0)); }

void build_spheres(SoftBody& b) {
    b.spheres.clear();
    b.spheres_version = b.topo_version;
    b.spheres_shells = b.shells.size();
    if (b.sphere_ball > 0) { // (a ball: one sphere, the whole body: k 0)
        b.spheres.push_back({{0, 0, 0}, {0, 0, 0}, 0, b.sphere_ball});
        b.sphere_first.assign(1, 0);
        b.sphere_rmax = b.sphere_ball;
        return;
    }
    const int m = std::clamp(b.sphere_div, 1, 4);
    const float f = b.sphere_scale / (float)m;
    // a node's and an edge's sphere: the smallest of its triangles'
    std::vector<float> rn(b.nodes.size(), 3.4e38f);
    std::vector<std::pair<uint64_t, float>> edges;
    auto edge_key = [](uint32_t a, uint32_t e) { return (uint64_t)std::min(a, e) << 32 | std::max(a, e); };
    for (const Shell& s : b.shells)
        for (int c = 0; c < 3; c++) {
            const uint32_t a = s.n[c], e = s.n[c == 2 ? 0 : c + 1];
            rn[a] = std::min(rn[a], f * tri_size(s));
            edges.push_back({edge_key(a, e), f * tri_size(s)});
        }
    std::sort(edges.begin(), edges.end());
    size_t ne = 0;
    for (size_t i = 0; i < edges.size(); i++)
        if (ne > 0 && edges[ne - 1].first == edges[i].first) edges[ne - 1].second = std::min(edges[ne - 1].second, edges[i].second);
        else edges[ne++] = edges[i];
    edges.resize(ne);
    // in the order of the triangles, each its own: the nodes and edges it is the first to have, its inner points of the
    // lattice (i, j, k > 0, i + j + k = m; below three its centre); a triangle's box then finds them (see collect)
    std::vector<uint8_t> node_done(b.nodes.size(), 0), edge_done(ne, 0);
    b.sphere_first.assign(b.shells.size() + 1, 0);
    b.sphere_rmax = 0;
    for (uint32_t si = 0; si < b.shells.size(); si++) {
        const Shell& s = b.shells[si];
        b.sphere_first[si] = (uint32_t)b.spheres.size();
        const float r = f * tri_size(s);
        for (int c = 0; c < 3; c++) {
            const uint32_t a = s.n[c], e = s.n[c == 2 ? 0 : c + 1];
            if (!node_done[a]) node_done[a] = 1, b.spheres.push_back({{a, a, a}, {1, 0, 0}, 1, rn[a]});
            const size_t ei = std::lower_bound(edges.begin(), edges.end(), std::make_pair(edge_key(a, e), -3.4e38f)) - edges.begin();
            if (m > 1 && !edge_done[ei]) {
                edge_done[ei] = 1;
                const uint32_t lo = std::min(a, e), hi = std::max(a, e);
                for (int q = 1; q < m; q++) b.spheres.push_back({{lo, hi, hi}, {1.0f - (float)q / m, (float)q / m, 0}, 2, edges[ei].second});
            }
        }
        if (m < 3) b.spheres.push_back({{s.n[0], s.n[1], s.n[2]}, {1.0f / 3, 1.0f / 3, 1.0f / 3}, 3, r});
        for (int i = 1; i < m; i++)
            for (int j = 1; i + j < m; j++)
                b.spheres.push_back({{s.n[0], s.n[1], s.n[2]}, {(float)i / m, (float)j / m, (float)(m - i - j) / m}, 3, r});
    }
    b.sphere_first.back() = (uint32_t)b.spheres.size();
    for (const SoftBody::Sphere& sp : b.spheres) b.sphere_rmax = std::max(b.sphere_rmax, sp.r);
    b.spheres_version = b.topo_version;
    b.spheres_shells = b.shells.size();
}

struct Cand {
    uint64_t key;       // cell
    uint32_t sphere;
    vec3 c;
    float r;
};

inline uint64_t cell_key(int x, int y, int z) {
    return ((uint64_t)(uint32_t)(x + (1 << 20)) << 42) | ((uint64_t)(uint32_t)(y + (1 << 20)) << 21) | (uint64_t)(uint32_t)(z + (1 << 20));
}

// candidates sorted by cell, their cells in an open-addressed table (key -> first of its run)
struct Grid {
    std::vector<Cand> c;
    std::vector<uint64_t> tkey;
    std::vector<uint32_t> tfirst;
    size_t cap = 0;
    float ic = 1;
    size_t slot_of(uint64_t key) const {
        // (all of the key mixed in: the bits of a product above 20 saw only z and y's low bits, and a row of cells along x
        // probed through one cluster, 0.7 ms a call for two barrels standing lid on lid)
        uint64_t x = key ^ (key >> 31);
        x *= 0xBF58476D1CE4E5B9ull;
        x ^= x >> 29;
        size_t h = (size_t)x & (cap - 1);
        while (tkey[h] != ~0ull && tkey[h] != key) h = (h + 1) & (cap - 1);
        return h;
    }
    void build(float cell) {
        ic = 1.0f / cell;
        for (Cand& x : c) x.key = cell_key((int)std::floor(x.c.x * ic), (int)std::floor(x.c.y * ic), (int)std::floor(x.c.z * ic));
        std::sort(c.begin(), c.end(), [](const Cand& a, const Cand& b) { return a.key != b.key ? a.key < b.key : a.sphere < b.sphere; });
        cap = 16;
        while (cap < c.size() * 2) cap <<= 1;
        tkey.assign(cap, ~0ull);
        tfirst.resize(cap);
        for (uint32_t i = 0; i < c.size(); i++)
            if (i == 0 || c[i].key != c[i - 1].key) {
                const size_t h = slot_of(c[i].key);
                tkey[h] = c[i].key, tfirst[h] = i;
            }
    }
    // f(j) for every candidate in the cells round p (27, or the half neighbourhood of p's own cell when half)
    template <class F> void near(vec3 p, bool half, uint64_t own, F&& f) const {
        static const int kHalf[14][3] = {{0, 0, 0}, {1, 0, 0}, {-1, 1, 0}, {0, 1, 0}, {1, 1, 0}, {-1, -1, 1}, {0, -1, 1}, {1, -1, 1},
                                         {-1, 0, 1}, {0, 0, 1}, {1, 0, 1}, {-1, 1, 1}, {0, 1, 1}, {1, 1, 1}};
        const int cx = (int)std::floor(p.x * ic), cy = (int)std::floor(p.y * ic), cz = (int)std::floor(p.z * ic);
        auto cellf = [&](int dx, int dy, int dz) {
            const uint64_t key = cell_key(cx + dx, cy + dy, cz + dz);
            const size_t h = slot_of(key);
            if (tkey[h] != key) return;
            for (size_t j = tfirst[h]; j < c.size() && c[j].key == key; j++) f(j, key == own);
        };
        if (half)
            for (const auto& o : kHalf) cellf(o[0], o[1], o[2]);
        else
            for (int dz = -1; dz <= 1; dz++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++) cellf(dx, dy, dz);
    }
};

using Contact = SpherePairs::Pair;

// a ball's centre of mass, velocity and inverse mass (this call's; the pushes keep them up to date)
struct Ball {
    vec3 c, v;
    float im = 0;
};

vec3 sphere_centre(const SoftBody& b, const SoftBody::Sphere& s) {
    const Node* nd = b.nodes.data();
    return s.k == 1 ? nd[s.n[0]].p : s.k == 2 ? nd[s.n[0]].p * s.w[0] + nd[s.n[1]].p * s.w[1] : nd[s.n[0]].p * s.w[0] + nd[s.n[1]].p * s.w[1] + nd[s.n[2]].p * s.w[2];
}

} // namespace

int sphere_contacts(std::vector<SoftBody*>& bodies, float h, std::vector<uint32_t>& group_of, float wake_speed, SpherePairs& cache,
                    bool detect, float span) {
    PROFILE_ACCUM("Sheet spheres");
    const size_t nb = bodies.size();
    if (nb == 0) return 0;
    thread_local Grid grid;
    thread_local std::vector<Cand> query;
    thread_local std::vector<Contact> contacts;
    thread_local std::vector<Ball> ball;
    contacts.clear();
    std::vector<AABB> box(nb);
    ball.assign(nb, Ball{});
    for (size_t i = 0; i < nb; i++) {
        SoftBody& b = *bodies[i];
        if (b.spheres_version != b.topo_version || b.spheres_shells != b.shells.size() || b.spheres.empty()) build_spheres(b), detect = true;
        box[i] = b.aabb;
        box[i].expand(0.05f);
        if (b.sphere_ball > 0) {
            float m = 0;
            vec3 c(0), p(0);
            for (const Node& x : b.nodes)
                if (x.inv_mass > 0) m += x.mass, c += x.p * x.mass, p += x.v * x.mass;
            if (m > 0) ball[i] = {c / m, p / m, 1.0f / m};
        }
    }
    // ---- two sheets whose boxes overlap: the spheres of one in the overlap in a grid, the other's there asking it; the
    // pairs closer than their radii and what the two bodies travel in `span` (the substep) kept for its short steps
    if (cache.bodies != bodies) detect = true;
    if (detect) cache.pairs.clear(), cache.bodies = bodies;
    for (size_t a = 0; a < nb && detect; a++)
        for (size_t o = a + 1; o < nb; o++) {
            if ((group_of[a] != 0 && group_of[a] == group_of[o]) || (bodies[a]->sleeping && bodies[o]->sleeping) || !box[a].overlaps(box[o])) continue;
            if (bodies[a]->shells.empty() && bodies[o]->shells.empty()) continue; // (two balls: their own contacts)
            if (bodies[a]->sphere_ball <= 0 && bodies[o]->sphere_ball <= 0 && !(bodies[a]->sphere_contacts && bodies[o]->sphere_contacts))
                continue; // (two sheets, one a sphere_target only: node against triangle)
            const float margin = std::min(0.04f, 1.5f * (bodies[a]->max_speed + bodies[o]->max_speed) * span + 0.002f);
            AABB q;
            q.mn = vmax(box[a].mn, box[o].mn) - vec3(margin);
            q.mx = vmin(box[a].mx, box[o].mx) + vec3(margin);
            // (the triangles whose box meets the overlap, then their spheres: a barrel has four times as many)
            auto collect = [&](size_t bi, std::vector<Cand>& out, float& rmax) {
                out.clear();
                const SoftBody& b = *bodies[bi];
                if (b.sphere_ball > 0) { // (the ball's one sphere)
                    const vec3 c = ball[bi].c;
                    const float r = b.sphere_ball;
                    if (ball[bi].im > 0 && c.x >= q.mn.x - r && c.x <= q.mx.x + r && c.y >= q.mn.y - r && c.y <= q.mx.y + r && c.z >= q.mn.z - r && c.z <= q.mx.z + r)
                        out.push_back({0, 0, c, r}), rmax = std::max(rmax, r);
                    return;
                }
                const Node* nd = b.nodes.data();
                const float rx = b.sphere_rmax;
                for (uint32_t si = 0; si < b.shells.size(); si++) {
                    const Shell& sh = b.shells[si];
                    const vec3 p0 = nd[sh.n[0]].p, p1 = nd[sh.n[1]].p, p2 = nd[sh.n[2]].p;
                    const vec3 mn = vmin(p0, vmin(p1, p2)), mx = vmax(p0, vmax(p1, p2));
                    if (mx.x < q.mn.x - rx || mn.x > q.mx.x + rx || mx.y < q.mn.y - rx || mn.y > q.mx.y + rx || mx.z < q.mn.z - rx || mn.z > q.mx.z + rx) continue;
                    for (uint32_t k = b.sphere_first[si]; k < b.sphere_first[si + 1]; k++) {
                        const SoftBody::Sphere& s = b.spheres[k];
                        const vec3 c = sphere_centre(b, s);
                        if (c.x < q.mn.x - s.r || c.x > q.mx.x + s.r || c.y < q.mn.y - s.r || c.y > q.mx.y + s.r || c.z < q.mn.z - s.r || c.z > q.mx.z + s.r) continue;
                        out.push_back({0, k, c, s.r});
                        rmax = std::max(rmax, s.r);
                    }
                }
            };
            float rg = 0, rq = 0;
            collect(o, grid.c, rg);
            collect(a, query, rq);
            if (grid.c.empty() || query.empty()) continue;
            grid.build(rg + rq);
            for (const Cand& A : query)
                grid.near(A.c, false, ~0ull, [&](size_t j, bool) {
                    const Cand& B = grid.c[j];
                    const float rr = A.r + B.r, rm = rr + margin;
                    if (length2(B.c - A.c) < rm * rm) cache.pairs.push_back({(uint32_t)a, A.sphere, (uint32_t)o, B.sphere, rr});
                });
        }
    contacts.insert(contacts.end(), cache.pairs.begin(), cache.pairs.end());
    if (contacts.empty()) return 0;
    // ---- the projection: two sweeps over the contacts (positions and velocities read fresh). The closing velocity is
    // taken out (no bounce), sliding resisted by Coulomb friction; an overlap is pushed apart in position only, each
    // node by no more than kOut h a call over all its contacts: the push adds no velocity, so nothing is driven (a
    // push with the velocity, pen / h, was the membrane's way, but bodies resting on each other then crept). A push per
    // pair of spheres, not per node, added up over a node's dozens of pairs: a barrel set a few centimetres down into
    // another (its lid's spheres into the other's, the lower one pressed into the ground) went off to five metres. An
    // impact's overlap is a step's travel deep: its velocity is taken out, the overlap undone in a few steps.
    static const float kOut = getenv("BL_SPHERE_OUT") ? (float)atof(getenv("BL_SPHERE_OUT")) : 0.2f; // m/s
    const float mu = 0.4f;
    thread_local std::vector<std::vector<float>> budget; // each node's push left this call
    budget.resize(nb);
    for (size_t i = 0; i < nb; i++) budget[i].assign(bodies[i]->nodes.size(), kOut * h);
    // one side of a pair: a sheet's sphere (its nodes, weighted) or a ball (the whole body, its centre of mass)
    auto side = [&](uint32_t bi, const SoftBody::Sphere& sp, float awake, vec3& c, vec3& v) {
        const SoftBody& b = *bodies[bi];
        if (sp.k == 0) {
            c = ball[bi].c, v = ball[bi].v;
            return ball[bi].im * awake;
        }
        float W = 0;
        c = v = vec3(0);
        for (int p = 0; p < sp.k; p++) {
            const Node& x = b.nodes[sp.n[p]];
            c += x.p * sp.w[p], v += x.v * sp.w[p], W += sp.w[p] * sp.w[p] * x.inv_mass * awake;
        }
        return W;
    };
    // the largest push (along the line of centres, as lam) its nodes have left this call
    auto room = [&](uint32_t bi, const SoftBody::Sphere& sp, float awake, float lam) {
        const SoftBody& b = *bodies[bi];
        for (int p = 0; p < sp.k; p++)
            if (const float f = sp.w[p] * awake * b.nodes[sp.n[p]].inv_mass; f > 0) lam = std::min(lam, budget[bi][sp.n[p]] / f);
        return lam;
    };
    // a push dp (lam n: positions) and an impulse dv (velocities) on one side
    auto apply = [&](uint32_t bi, const SoftBody::Sphere& sp, float awake, vec3 dp, vec3 dv) {
        SoftBody& b = *bodies[bi];
        if (sp.k == 0) {
            const float f = ball[bi].im * awake;
            for (Node& x : b.nodes)
                if (x.inv_mass > 0) x.p += dp * f, x.v += dv * f;
            ball[bi].c += dp * f, ball[bi].v += dv * f;
            return;
        }
        for (int p = 0; p < sp.k; p++) {
            Node& x = b.nodes[sp.n[p]];
            const float f = sp.w[p] * awake * x.inv_mass;
            x.p += dp * f, x.v += dv * f;
            budget[bi][sp.n[p]] -= length(dp) * f;
        }
    };
    for (int it = 0; it < 2; it++)
        for (const Contact& ct : contacts) {
            SoftBody& ba = *bodies[ct.ba];
            SoftBody& bb = *bodies[ct.bb];
            const SoftBody::Sphere& sa = ba.spheres[ct.sa];
            const SoftBody::Sphere& sb = bb.spheres[ct.sb];
            const float ia = ba.sleeping ? 0.0f : 1.0f, ib = bb.sleeping ? 0.0f : 1.0f; // (asleep: immovable)
            vec3 ca, cb, va, vb;
            const float W = side(ct.ba, sa, ia, ca, va) + side(ct.bb, sb, ib, cb, vb);
            if (!(W > 0)) continue;
            const vec3 d = cb - ca;
            const float l = length(d);
            if (ct.r <= l || !(l > 1e-7f)) continue;
            if (it == 0) ba.sphere_touches++, bb.sphere_touches++;
            // a sleeping one woken by a partner that moves: three times its rest threshold (a drum left awake on a
            // sleeping one trembles about that where they touch, and the two woke each other in turn for good)
            if (ba.sleeping && bb.max_speed > 3.0f * std::max(wake_speed, bb.sleep_speed)) ba.wake_request = true;
            if (bb.sleeping && ba.max_speed > 3.0f * std::max(wake_speed, ba.sleep_speed)) bb.wake_request = true;
            const vec3 n = d / l;
            const vec3 dv = vb - va;
            const float vn = dot(dv, n);
            // the push: the overlap, at most what leaves the two parting at kOut, at most what its nodes have left
            float lam = std::min(ct.r - l, std::max(0.0f, kOut - vn) * h) / W;
            lam = std::max(0.0f, room(ct.bb, sb, ib, room(ct.ba, sa, ia, lam)));
            static const bool push_v = getenv("BL_SPHERE_PUSHV") != nullptr; // (diagnostics: the push with velocity)
            const float pen = lam * W, lv = push_v ? lam / h : 0.0f;
            apply(ct.ba, sa, ia, n * -lam, n * -lv);
            apply(ct.bb, sb, ib, n * lam, n * lv);
            // the closing velocity (what the push left of it) out, the sliding one resisted by both impulses
            const float jn = std::max(0.0f, -(vn + (push_v ? pen / h : 0.0f))) / W;
            const vec3 vt = dv - n * vn;
            const float vtl = length(vt);
            const float jt = vtl > 1e-6f ? std::min(mu * (jn + lv), vtl / W) : 0.0f;
            if (jn <= 0 && jt <= 0) continue;
            const vec3 imp = n * jn - (vtl > 1e-6f ? vt * (jt / vtl) : vec3(0));
            apply(ct.ba, sa, ia, vec3(0), -imp);
            apply(ct.bb, sb, ib, vec3(0), imp);
        }
    return (int)contacts.size();
}

} // namespace bl::phys
