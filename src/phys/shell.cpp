// Triangle elements (phys::Shell): edge springs + bending hinges, adaptive longest-edge bisection and cracks by
// node splitting.
//
// Refinement is Rivara's longest-edge bisection: a triangle is split through the midpoint of its longest edge
// together with the neighbour across that edge (which is bisected first when the edge is not its own longest one),
// so the mesh never has T-junctions. On the right isosceles grid the sheets are built from, every child is again
// right isosceles: two bisections give the 4-triangle grid of the parent. Edge stiffness depends only on the shape
// (k * Lmin / L), so refinement keeps the sheet's stiffness; the node masses shrink with the area, so a refined
// body takes shorter steps of its own (SoftBody::dt_shift), each triangle evaluated at the rate its size needs.
//
// Cracks follow O'Brien & Hodgins (1999) in spirit: a node is duplicated and the triangles around it are divided
// between the copies along the crack line, which runs between triangles (along their edges). Each copy keeps the
// node's position and velocity; its mass is the share of its triangles.
#include "core/profiler.h"
#include "phys/shell_util.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace bl::phys {

using namespace shell_detail;

namespace {
inline float cross2(vec2 a, vec2 b) { return a.x * b.y - a.y * b.x; }
} // namespace

uint32_t SoftBody::add_shell(uint32_t a, uint32_t b, uint32_t c, vec2 ua, vec2 ub, vec2 uc) {
    Shell s{};
    s.n[0] = a;
    s.n[1] = b;
    s.n[2] = c;
    s.nb[0] = s.nb[1] = s.nb[2] = -1;
    s.uv[0] = ua;
    s.uv[1] = ub;
    s.uv[2] = uc;
    s.flaw = 1.0f;
    s.tri = UINT32_MAX;
    s.es[0] = s.es[1] = s.es[2] = s.hs[0] = s.hs[1] = s.hs[2] = 64;
    s.kscale = 1.0f;
    for (uint8_t& h : s.half) h = kNoHalf;
    shells.push_back(s);
    return (uint32_t)shells.size() - 1;
}

void SoftBody::finalize_shells(float areal_density, float dt, uint32_t seed, bool keep_mass) {
    const size_t nn = nodes.size();
    std::vector<uint8_t> used(nn, 0);
    for (Shell& s : shells)
        for (int c = 0; c < 3; c++) used[s.n[c]] = 1;
    node_base_mass.clear();
    if (keep_mass) {
        node_base_mass.resize(nn);
        for (size_t i = 0; i < nn; i++) node_base_mass[i] = nodes[i].mass;
    }
    for (size_t i = 0; i < nn; i++)
        if (used[i]) nodes[i].mass = base_mass((uint32_t)i);
    for (size_t si = 0; si < shells.size(); si++) {
        Shell& s = shells[si];
        const vec3 p0 = nodes[s.n[0]].p, p1 = nodes[s.n[1]].p, p2 = nodes[s.n[2]].p;
        for (int e = 0; e < 3; e++) s.L[e] = s.L0[e] = std::max(1e-4f, distance(nodes[s.n[e]].p, nodes[s.n[nx(e)]].p));
        s.area0 = s.area_nom = 0.5f * length(cross(p1 - p0, p2 - p0));
        s.n0 = normalize_or(cross(p1 - p0, p2 - p0), vec3(0, 1, 0));
        const ShellMaterial& sm = shell_material(s);
        s.mass = (sm.kg_m2 > 0 ? sm.kg_m2 : areal_density) * s.area0;
        s.level = 0;
        s.th0[0] = s.th0[1] = s.th0[2] = 0;
        s.nb[0] = s.nb[1] = s.nb[2] = -1;
        s.flaw = sm.flaw > 0 ? 1.0f + sm.flaw * rand_pm1(seed * 7919u + (uint32_t)si) : 1.0f;
        for (int c = 0; c < 3; c++) nodes[s.n[c]].mass += s.mass / 3.0f;
    }
    for (size_t i = 0; i < nn; i++)
        if (used[i]) nodes[i].inv_mass = (info[i].flags & NF_FIXED) || nodes[i].mass <= 0 ? 0.0f : 1.0f / nodes[i].mass;
    // neighbours across the edges
    std::unordered_map<uint64_t, std::pair<int, int>> open;
    for (int si = 0; si < (int)shells.size(); si++)
        for (int e = 0; e < 3; e++) {
            uint32_t a = shells[si].n[e], b = shells[si].n[nx(e)];
            uint64_t key = a < b ? ((uint64_t)a << 32 | b) : ((uint64_t)b << 32 | a);
            auto it = open.find(key);
            if (it == open.end()) {
                open.emplace(key, std::make_pair(si, e));
            } else if (it->second.first >= 0) {
                shells[si].nb[e] = it->second.first;
                shells[it->second.first].nb[it->second.second] = si;
                it->second.first = -1; // (a third triangle on the same edge stays unlinked)
            }
        }
    node_shells.assign(nn, {});
    for (uint32_t si = 0; si < shells.size(); si++)
        for (int c = 0; c < 3; c++) node_shells[shells[si].n[c]].push_back(si);
    for (Shell& s : shells) {
        s.edges = 0;
        for (int e = 0; e < 3; e++)
            if (s.nb[e] < 0) s.edges |= (uint8_t)(1u << e); // (the authored border: not a crack)
    }
    pattern_init(seed); // (the material plane; wood's fibres and veins in the edges' codes)
    // collision triangles (never "torn": the shell cracks instead)
    for (Shell& s : shells) {
        Triangle t;
        t.a = s.n[0];
        t.b = s.n[1];
        t.c = s.n[2];
        t.rest_edge2 = 0;
        s.tri = (uint32_t)tris.size();
        tris.push_back(t);
    }
    // stability caps: one stiffness for the whole sheet (a homogeneous material), from its most loaded node
    std::vector<float> ks(nn, 0.0f), ds(nn, 0.0f), hs(nn, 0.0f);
    for (Shell& s : shells) {
        const float lmin = std::min(s.L0[0], std::min(s.L0[1], s.L0[2]));
        for (int e = 0; e < 3; e++) {
            const float r = lmin / s.L0[e];
            ks[s.n[e]] += r;
            ks[s.n[nx(e)]] += r;
        }
    }
    for (int si = 0; si < (int)shells.size(); si++) {
        Shell& s = shells[si];
        for (int e = 0; e < 3; e++) {
            const int j = s.nb[e];
            if (j <= si) continue;
            Shell& t = shells[j];
            const int f = edge_of(t, s.n[e], s.n[nx(e)]);
            Hinge h;
            const uint32_t id[4] = {s.n[pv(e)], t.n[pv(f)], s.n[e], s.n[nx(e)]};
            if (!hinge_geometry(nodes[id[0]].p, nodes[id[1]].p, nodes[id[2]].p, nodes[id[3]].p, h)) continue;
            s.th0[e] = t.th0[f] = h.theta; // authored shape is the rest shape
            for (int k = 0; k < 4; k++) hs[id[k]] += h.fh * length2(h.g[k]);
        }
    }
    float kcap = 3.4e38f, dcap = 3.4e38f, bcap = 3.4e38f; // (what the nodes take, whatever the material)
    dt /= (float)(1 << std::clamp(shell_min_shift, 0, 2)); // (a sheet that takes short steps from the start: at that step)
    for (size_t i = 0; i < nn; i++) {
        if (!used[i] || nodes[i].inv_mass <= 0) continue;
        const float m = nodes[i].mass;
        if (ks[i] > 0) {
            kcap = std::min(kcap, edge_budget() * m / (dt * dt * ks[i]));
            dcap = std::min(dcap, kDampBudget * m / (dt * ks[i]));
        }
        if (hs[i] > 0) bcap = std::min(bcap, hinge_budget() * m / (dt * dt * hs[i]));
    }
    auto cap = [&](ShellMaterial& m) {
        m.k = std::min(m.k, kcap);
        m.damp = std::min(m.damp, dcap);
        m.bend = std::min(m.bend, bcap);
    };
    cap(shell_mat);
    for (ShellMaterial& m : shell_mat_extra) cap(m);
    shell_mat_base = shell_mat;
    shell_mat_extra_base = shell_mat_extra;
    for (Shell& s : shells) shell_springs(s, shell_material(s));
    shell_level = 0;
    const float budget = shell_mat.refine_budget > 0 ? shell_mat.refine_budget : (float)(1 << std::clamp(shell_max_level(), 0, 8));
    shell_cap = (size_t)((float)shells.size() * std::max(1.0f, budget)) + 16;
    force.resize(nn);
    topo_version++;
    shell_acc_stale = true;
    reorder_shells();
    ordered_shells = shells.size();
}

// ------------------------------------------------------------------------------------------------ topology
namespace {

struct ShellOps {
    SoftBody& b;
    std::vector<uint8_t>& touched; // shells changed in this batch (queued cracks on them are re-evaluated later)
    uint32_t counter;

    ShellMaterial& M() { return b.shell_mat; }
    void touch(uint32_t si) {
        if (touched.size() <= si) touched.resize(si + 1, 0);
        touched[si] = 1;
        b.shk.dirty_shells.push_back(si);
    }
    // for the kernel's arrays only (ShellKernel): changed links / fans, without the batch semantics of touch()
    void dirty(uint32_t si) { b.shk.dirty_shells.push_back(si); }
    void dirty_node(uint32_t v) { b.shk.dirty_nodes.push_back(v); }
    void log_tri(uint32_t t, uint32_t from) {
        if (b.topo_log.tris.size() < 100000) b.topo_log.tris.push_back({t, from});
        else b.topo_log.overflow = true;
    }
    void log_node(uint32_t v, uint32_t from) {
        if (b.topo_log.nodes.size() < 100000) b.topo_log.nodes.push_back({v, from});
        else b.topo_log.overflow = true;
    }
    bool can_refine(const Shell& s) {
        const float lmax = std::max(s.L0[0], std::max(s.L0[1], s.L0[2]));
        const ShellMaterial& m = b.shell_material(s);
        return s.level < m.max_level && lmax * 0.70710678f >= m.min_edge && b.shells.size() + 4 <= b.shell_cap;
    }
    bool budget_left() const { return b.refine_left > 0; } // (the frame's quota: the rest of the overloads queue again)
    // node on a free edge (sheet border or crack): a crack tip when the crack reaches it
    bool open_fan(uint32_t v) {
        for (uint32_t si : b.node_shells[v]) {
            const Shell& t = b.shells[si];
            const int c = corner_of(t, v);
            if (c >= 0 && (t.nb[c] < 0 || t.nb[pv(c)] < 0)) return true;
        }
        return false;
    }
    void set_node_mass(uint32_t v) {
        float m = b.base_mass(v);
        for (uint32_t si : b.node_shells[v]) m += b.shells[si].mass / 3.0f;
        Node& n = b.nodes[v];
        n.mass = m;
        n.inv_mass = (b.info[v].flags & NF_FIXED) || m <= 0 ? 0.0f : 1.0f / m;
    }
    uint32_t new_node(uint32_t like) {
        const uint32_t id = (uint32_t)b.nodes.size();
        b.nodes.push_back(b.nodes[like]);
        NodeInfo inf = b.info[like];
        inf.frame = -1;
        inf.ror_id = -1;
        inf.flags &= (uint16_t)~NF_FRAME;
        b.info.push_back(inf);
        b.force.push_back(vec3(0));
        if (!b.wind_area.empty()) b.wind_area.push_back(b.wind_area[like]);
        if (!b.node_base_mass.empty()) b.node_base_mass.push_back(0.0f); // (a copy or a midpoint: the sheet's alone)
        b.node_shells.emplace_back();
        dirty_node(id);
        return id;
    }
    void write_tri(const Shell& s) {
        Triangle& t = b.tris[s.tri];
        t.a = s.n[0];
        t.b = s.n[1];
        t.c = s.n[2];
    }
    void replace_in_fan(uint32_t v, uint32_t from, uint32_t to) {
        dirty_node(v);
        for (uint32_t& x : b.node_shells[v])
            if (x == from) x = to;
    }
    void remove_from_fan(uint32_t v, uint32_t si) {
        dirty_node(v);
        auto& fan = b.node_shells[v];
        fan.erase(std::remove(fan.begin(), fan.end(), si), fan.end());
    }
    // re-point the neighbour across edge (p, q) of shell `si` from `from` to `to`
    void relink(int nbr, uint32_t p, uint32_t q, int from, int to) {
        if (nbr < 0) return;
        Shell& t = b.shells[nbr];
        const int f = edge_of(t, p, q);
        if (f >= 0 && t.nb[f] == from) t.nb[f] = to;
        dirty((uint32_t)nbr);
    }

    // Bisect shell si through the point m at fraction t of its edge e (0.5: the midpoint; a fracture pattern puts it
    // on a line): si keeps the part at n[e], the returned new shell is the part at n[e+1]. The neighbour links across
    // the split edge are left to the caller.
    uint32_t bisect(uint32_t si, int e, uint32_t m, float t = 0.5f) {
        const Shell s = b.shells[si];
        const int e1 = nx(e), e2 = pv(e);
        const uint32_t bb = s.n[e1], c = s.n[e2];
        const uint32_t ni = (uint32_t)b.shells.size();
        // the cevian from the opposite corner to m (Stewart; the median at t = 0.5)
        auto cevian = [&](const float* L) {
            return std::sqrt(std::max(1e-10f, (1 - t) * L[e2] * L[e2] + t * L[e1] * L[e1] - t * (1 - t) * L[e] * L[e]));
        };
        const float lm = cevian(s.L), lm0 = cevian(s.L0);
        const vec2 uvm = s.uv[e] + (s.uv[e1] - s.uv[e]) * t;
        Shell A = s, B = s;
        A.n[e1] = m;
        A.L[e] = s.L[e] * t;
        A.L0[e] = s.L0[e] * t;
        A.L[e1] = lm;
        A.L0[e1] = lm0;
        A.th0[e1] = 0; // flat inside the parent
        A.nb[e1] = (int32_t)ni;
        A.uv[e1] = uvm;
        B.n[e] = m;
        B.L[e] = s.L[e] * (1 - t);
        B.L0[e] = s.L0[e] * (1 - t);
        B.L[e2] = lm;
        B.L0[e2] = lm0;
        B.th0[e2] = 0;
        B.nb[e2] = (int32_t)si;
        B.uv[e] = uvm;
        // the new edge inside the parent: no border, no cut, default strength (the pattern codes follow below)
        A.edges = (uint8_t)(s.edges & ~((1u << e1) | (8u << e1)));
        B.edges = (uint8_t)(s.edges & ~((1u << e2) | (8u << e2)));
        A.line = (uint8_t)(s.line & ~(1u << e1));
        B.line = (uint8_t)(s.line & ~(1u << e2));
        A.es[e1] = A.hs[e1] = B.es[e2] = B.hs[e2] = 64;
        // the bisection history: the children number their edges as the parent (A's e1 and B's e2 are new); an entry
        // of the parent goes to the child that still has that edge with its node
        for (int l = 0; l < 8; l++) {
            const uint8_t h = s.half[l];
            A.half[l] = B.half[l] = kNoHalf;
            if (h == kNoHalf) continue;
            const int he = h & 3;
            const bool second = (h & 4) != 0;
            if (he == e) (second ? B : A).half[l] = h; // (a-b: the node is a, in A, or b, in B)
            else if (he == e1) B.half[l] = h;          // (b-c: B's)
            else if (he == e2) A.half[l] = h;          // (c-a: A's)
        }
        A.half[std::min(7, s.level + 1)] = half_code(e, true);  // (A's edge e is a -> m)
        B.half[std::min(7, s.level + 1)] = half_code(e, false); // (B's edge e is m -> b)
        A.mass = s.mass * t;
        A.area0 = s.area0 * t;
        B.mass = s.mass * (1 - t);
        B.area0 = s.area0 * (1 - t);
        A.area_nom = B.area_nom = s.area_nom * 0.5f;
        for (Shell* x : {&A, &B}) {
            x->level = (uint8_t)(s.level + 1);
            x->pending = 0;
            const ShellMaterial& m = b.shell_material(*x);
            x->flaw = m.flaw > 0 ? 1.0f + m.flaw * rand_pm1(hash32(++counter) ^ (uint32_t)b.shells.size()) : 1.0f;
            shell_springs(*x, m);
        }
        B.tri = (uint32_t)b.tris.size();
        Triangle tri = b.tris[s.tri];
        b.tris.push_back(tri);
        log_tri(B.tri, s.tri);
        b.shells[si] = A;
        b.shells.push_back(B);
        write_tri(b.shells[si]);
        write_tri(b.shells[ni]);
        // the old neighbour across (b, c) now borders the new half
        relink(s.nb[e1], bb, c, (int)si, (int)ni);
        replace_in_fan(bb, si, ni);
        b.node_shells[c].push_back(ni);
        b.node_shells[m].push_back(si);
        b.node_shells[m].push_back(ni);
        dirty_node(c);
        dirty_node(m);
        b.shell_level = std::max(b.shell_level, (int)s.level + 1);
        if (b.pattern_on()) {
            b.pattern_codes(si);
            b.pattern_codes(ni);
        }
        touch(si);
        touch(ni);
        return ni;
    }

    // A new node of a patterned sheet moved (in the material plane, along the sheet) onto a pattern line close to it,
    // or a little at random inside a pattern zone: the lines get nodes on them and the crack edges are not straight.
    // The rest shape of the triangles around follows the move (no stress from it); a move that would squash or flip
    // one of them is halved, then dropped. Only inside the sheet (a border node stays on its border edge).
    void settle(uint32_t m, bool jitter = true) {
        if (!b.pattern_on() || (b.info[m].flags & NF_FIXED) || open_fan(m)) return;
        PROFILE_ACCUM("Sheet settle");
        const std::vector<uint32_t>& fan = b.node_shells[m];
        if (fan.size() < 3) return;
        const vec2 x = b.node_x(m);
        float lmin = 1e30f;
        for (uint32_t si : fan)
            for (int e = 0; e < 3; e++) lmin = std::min(lmin, b.shells[si].L0[e]);
        vec2 dx(0);
        const PatternLine ln = b.pattern_nearest(x, 0.35f * lmin);
        if (ln.d < 0.35f * lmin) {
            if (ln.d < 1e-6f) return;
            dx = ln.foot - x;
        } else if (jitter && b.pattern_zone(x)) {
            const uint32_t h = hash32(++counter * 747796405u);
            dx = vec2(rand_pm1(h), rand_pm1(h ^ 0x5bd1e995u)) * (0.1f * lmin);
        } else {
            return;
        }
        auto X = [&](uint32_t si, int c, vec2 d) { return b.shell_x(si, c) + (b.shells[si].n[c] == m ? d : vec2(0)); };
        auto fits = [&](vec2 d) {
            for (uint32_t si : fan) {
                const vec2 p0 = X(si, 0, vec2(0)), p1 = X(si, 1, vec2(0)), p2 = X(si, 2, vec2(0));
                const vec2 q0 = X(si, 0, d), q1 = X(si, 1, d), q2 = X(si, 2, d);
                const float a0 = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
                const float a1 = (q1.x - q0.x) * (q2.y - q0.y) - (q1.y - q0.y) * (q2.x - q0.x);
                if (a0 * a1 <= 0 || std::fabs(a1) < 0.7f * std::fabs(a0)) return false;
                if (b.shells[si].area0 * (a1 / a0) < 0.6f * b.shells[si].area_nom) return false; // (see Shell::area_nom)
                const vec2 P[3] = {p0, p1, p2}, Q[3] = {q0, q1, q2};
                for (int e = 0; e < 3; e++)
                    if (length(Q[nx(e)] - Q[e]) < 0.7f * length(P[nx(e)] - P[e])) return false;
                const float qn = tri_quality(std::fabs(a1), length(Q[1] - Q[0]), length(Q[2] - Q[1]), length(Q[0] - Q[2]));
                const float qo = tri_quality(std::fabs(a0), length(P[1] - P[0]), length(P[2] - P[1]), length(P[0] - P[2]));
                if (qn < std::min(kShapeMin, qo)) return false; // (no thinner than kShapeMin, or than it was)
            }
            return true;
        };
        if (!fits(dx)) {
            dx = dx * 0.5f;
            if (!fits(dx)) return;
        }
        // the same move on the current shape: the affine map of a triangle around from the material plane
        vec3 dp(0);
        {
            const uint32_t si = fan[0];
            const Shell& s = b.shells[si];
            const vec2 a = b.shell_x(si, 1) - b.shell_x(si, 0), c = b.shell_x(si, 2) - b.shell_x(si, 0);
            const float det = a.x * c.y - a.y * c.x;
            if (std::fabs(det) < 1e-14f) return;
            const float u = (dx.x * c.y - dx.y * c.x) / det, v = (a.x * dx.y - a.y * dx.x) / det; // dx = u a + v c
            const vec3 pa = b.nodes[s.n[1]].p - b.nodes[s.n[0]].p, pc = b.nodes[s.n[2]].p - b.nodes[s.n[0]].p;
            dp = pa * u + pc * v;
            // (only on a shape close to the rest shape: on a crumpled or badly stretched triangle the map is off and
            // the node would jump)
            const float ra = length(pa) / std::max(1e-9f, length(a)), rc = length(pc) / std::max(1e-9f, length(c));
            const float rb = length(pc - pa) / std::max(1e-9f, length(c - a));
            if (std::max(ra, std::max(rb, rc)) > 1.3f || std::min(ra, std::min(rb, rc)) < 0.77f || length(dp) > 1.5f * length(dx)) return;
        }
        const vec2 duv(b.shell_uvm.x > 0 ? dx.x / b.shell_uvm.x : 0.0f, b.shell_uvm.y > 0 ? dx.y / b.shell_uvm.y : 0.0f);
        double area_before = 0, area_after = 0;
        for (uint32_t si : fan) area_before += b.shells[si].area0;
        for (uint32_t si : fan) {
            Shell& s = b.shells[si];
            const int c = corner_of(s, m);
            const vec2 p0 = X(si, 0, vec2(0)), p1 = X(si, 1, vec2(0)), p2 = X(si, 2, vec2(0));
            const vec2 q0 = X(si, 0, dx), q1 = X(si, 1, dx), q2 = X(si, 2, dx);
            const float a0 = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
            const float a1 = (q1.x - q0.x) * (q2.y - q0.y) - (q1.y - q0.y) * (q2.x - q0.x);
            const vec2 P[3] = {p0, p1, p2}, Q[3] = {q0, q1, q2};
            for (int e : {c, pv(c)}) { // the two edges at the node
                const float r = length(Q[nx(e)] - Q[e]) / std::max(1e-9f, length(P[nx(e)] - P[e]));
                s.L[e] *= r;
                s.L0[e] *= r;
            }
            s.area0 *= a1 / a0;
            area_after += s.area0;
            s.uv[c] = s.uv[c] + duv;
        }
        // (the areas round an inner node add up to the same: exactly, whatever the rest areas were before)
        if (area_after > 1e-12)
            for (uint32_t si : fan) b.shells[si].area0 = (float)(b.shells[si].area0 * (area_before / area_after));
        b.nodes[m].p += dp;
        for (uint32_t si : fan) {
            b.pattern_codes(si);
            touch(si);
        }
    }

    // Longest-edge bisection keeping the mesh conforming. Returns false when the shell cannot be refined.
    bool refine(uint32_t si, int depth = 0) {
        if (depth > 16 || si >= b.shells.size()) return false;
        if (!can_refine(b.shells[si])) return false;
        const int e = longest_edge(b.shells[si]);
        const int j = b.shells[si].nb[e];
        int fe = -1;
        if (j >= 0) {
            const Shell& s = b.shells[si];
            const Shell& t = b.shells[j];
            fe = edge_of(t, s.n[e], s.n[nx(e)]);
            if (fe < 0) return false;
            if (t.L0[fe] < t.L0[longest_edge(t)] * 0.999f) {
                // the neighbour is coarser: split it first, then one of its halves shares our longest edge as its own
                if (!refine((uint32_t)j, depth + 1)) return false;
                return refine(si, depth + 1);
            }
            if (!can_refine(t)) return false;
        }
        const Shell s = b.shells[si];
        const uint32_t a = s.n[e], bb = s.n[nx(e)];
        // (a patterned sheet: on a line the edge crosses, or a little off the middle in a pattern zone; no half smaller
        // than 0.6 of its nominal size, see Shell::area_nom)
        float t = 0.5f;
        if (b.pattern_on()) {
            PROFILE_ACCUM("Sheet split point");
            float rho = s.area0 / std::max(1e-12f, s.area_nom);
            if (j >= 0) rho = std::min(rho, b.shells[j].area0 / std::max(1e-12f, b.shells[j].area_nom));
            t = b.pattern_split(b.shell_x(si, e), b.shell_x(si, nx(e)), hash32(++counter * 2246822519u), 0.5f - 0.3f / std::max(1e-3f, rho));
            // (no half of either triangle thinner than kShapeMin: else back towards the midpoint)
            auto halves_ok = [&](const Shell& x, int xe, float tx) {
                const int x1 = nx(xe), x2 = pv(xe);
                const float c = std::sqrt(std::max(1e-12f, (1 - tx) * x.L0[x2] * x.L0[x2] + tx * x.L0[x1] * x.L0[x1] - tx * (1 - tx) * x.L0[xe] * x.L0[xe]));
                return tri_quality(x.area0 * tx, x.L0[xe] * tx, c, x.L0[x2]) >= kShapeMin && tri_quality(x.area0 * (1 - tx), x.L0[xe] * (1 - tx), x.L0[x1], c) >= kShapeMin;
            };
            for (int k = 0; k < 4 && t != 0.5f; k++) {
                bool ok = halves_ok(s, e, t);
                if (ok && j >= 0) {
                    const Shell& nt = b.shells[j];
                    const int nfe = edge_of(nt, s.n[e], s.n[nx(e)]);
                    if (nfe >= 0) ok = halves_ok(nt, nfe, nt.n[nfe] == a ? t : 1 - t);
                }
                if (ok) break;
                t = k == 3 ? 0.5f : 0.5f + (t - 0.5f) * 0.5f;
            }
        }
        const uint32_t m = new_node(a);
        log_node(m, a);
        log_node(m, bb);
        {
            Node& nm = b.nodes[m];
            const Node &na = b.nodes[a], &nb = b.nodes[bb];
            nm.p = na.p + (nb.p - na.p) * t; // (mass and centre of mass kept: the halves' masses are t and 1 - t)
            nm.v = na.v + (nb.v - na.v) * t;
            NodeInfo& im = b.info[m];
            const uint16_t fixed = b.info[a].flags & b.info[bb].flags & NF_FIXED; // on a clamped border only
            im.flags = (uint16_t)((b.info[a].flags & ~NF_FIXED) | fixed);
        }
        const uint32_t sb = bisect(si, e, m, t); // si: (a, m, c), sb: (m, bb, c)
        if (j >= 0) {
            // neighbour (bb, a, c2) in its own winding: j keeps the half at bb, tb gets the half at a
            const uint32_t tb = bisect((uint32_t)j, fe, m, b.shells[j].n[fe] == a ? t : 1 - t);
            b.shells[si].nb[e] = (int32_t)tb;
            b.shells[tb].nb[fe] = (int32_t)si;
            b.shells[sb].nb[e] = j;
            b.shells[j].nb[fe] = (int32_t)sb;
        } else {
            b.shells[si].nb[e] = -1;
            b.shells[sb].nb[e] = -1;
        }
        // masses: every shell around the edge moved a third of its mass from the edge's ends onto the midpoint
        set_node_mass(a);
        set_node_mass(bb);
        set_node_mass(m);
        b.shell_stats.refined++;
        settle(m);
        return true;
    }

    // Would the part of v's fan with root r end up as a piece of at most `limit` triangles once the links `cut`
    // are gone? A piece is what hangs together through edges (a shared node alone is a hinge with no strength).
    bool small_piece(const std::vector<uint32_t>& fan, const std::vector<int>& root, int r, int limit,
                     const std::vector<std::pair<uint32_t, uint32_t>>& cut) {
        std::vector<uint32_t> seen, stack;
        for (size_t i = 0; i < fan.size(); i++)
            if (root[i] == r) {
                seen.push_back(fan[i]);
                stack.push_back(fan[i]);
            }
        if ((int)seen.size() > limit) return false;
        while (!stack.empty()) {
            const uint32_t si = stack.back();
            stack.pop_back();
            const Shell& s = b.shells[si];
            for (int e = 0; e < 3; e++) {
                const int t = s.nb[e];
                if (t < 0 || std::find(seen.begin(), seen.end(), (uint32_t)t) != seen.end()) continue;
                bool gone = false;
                for (auto& c : cut) gone |= (c.first == si && c.second == (uint32_t)t) || (c.second == si && c.first == (uint32_t)t);
                if (gone) continue;
                const auto it = std::find(fan.begin(), fan.end(), (uint32_t)t);
                if (it != fan.end() && root[it - fan.begin()] != r) return false; // joined to the rest around the node
                seen.push_back((uint32_t)t);
                if ((int)seen.size() > limit) return false;
                stack.push_back((uint32_t)t);
            }
        }
        return true;
    }

    // Divides the triangles around node v between copies of it. Links across the chosen crack spokes are cut: for a
    // closed fan the spokes best aligned with +dir and -dir, for an open fan (border or crack tip) the one best
    // aligned with the line; `through` forces a spoke (v, through). A fan already in pieces is only separated
    // (`pinch_only`: nothing else). No cut may leave a piece of min_piece triangles or fewer unless `dust_ok`.
    // The piece holding `keep` stays on v. Returns the number of new nodes.
    int split_node(uint32_t v, vec3 dir, int64_t through, int keep, bool dust_ok = false, bool pinch_only = false) {
        std::vector<uint32_t> fan = b.node_shells[v];
        const int nf = (int)fan.size();
        if (nf < 2) return 0;
        struct Link {
            int a, b; // fan positions
            uint32_t x; // spoke node
            float score;
            float on;   // the spoke lies on a fracture pattern line: the crack prefers it
            vec3 d;
        };
        std::vector<Link> links;
        const vec3 p = b.nodes[v].p;
        const float follow = b.pattern_on() ? 1.5f : 0.0f; // (a spoke on a line wins over any off it)
        for (int i = 0; i < nf; i++) {
            const Shell& s = b.shells[fan[i]];
            const int c = corner_of(s, v);
            if (c < 0) continue;
            const int j = s.nb[c]; // edge v -> n[c+1]
            if (j < 0) continue;
            const int k = (int)(std::find(fan.begin(), fan.end(), (uint32_t)j) - fan.begin());
            if (k >= nf) continue;
            const uint32_t x = s.n[nx(c)];
            const vec3 d = normalize_or(b.nodes[x].p - p, vec3(0));
            links.push_back({i, k, x, dot(d, dir), (s.line >> c) & 1u ? follow : 0.0f, d});
        }
        std::vector<int> parent(nf);
        auto find = [&](int x) {
            while (parent[x] != x) x = parent[x] = parent[parent[x]];
            return x;
        };
        auto components = [&](int skip1, int skip2) {
            for (int i = 0; i < nf; i++) parent[i] = i;
            int n = nf;
            for (int l = 0; l < (int)links.size(); l++) {
                if (l == skip1 || l == skip2) continue;
                int ra = find(links[l].a), rb = find(links[l].b);
                if (ra != rb) {
                    parent[ra] = rb;
                    n--;
                }
            }
            return n;
        };
        int cut1 = -1, cut2 = -1;
        if (components(-1, -1) == 1) {
            if (pinch_only) return 0;
            const bool closed = links.size() >= (size_t)nf;
            if (through >= 0) {
                for (int l = 0; l < (int)links.size(); l++)
                    if (links[l].x == (uint32_t)through) cut1 = l;
                if (cut1 < 0) return 0;
            } else {
                float best = -2;
                for (int l = 0; l < (int)links.size(); l++) {
                    const float sc = (closed ? links[l].score : std::fabs(links[l].score)) + links[l].on;
                    if (sc > best) {
                        best = sc;
                        cut1 = l;
                    }
                }
            }
            if (closed) {
                float best = 2;
                const float s1 = links[cut1].score;
                for (int pass = 0; pass < 2 && cut2 < 0; pass++)
                    for (int l = 0; l < (int)links.size(); l++) {
                        if (l == cut1) continue;
                        // the continuation of the crack: opposite to the first spoke (on a pattern line if one is
                        // there, but not bending back towards the first spoke)
                        if (pass == 0 && links[l].on > 0 && dot(links[l].d, links[cut1].d) > -0.3f) continue;
                        const float sc = (s1 >= 0 ? links[l].score : -links[l].score) - links[l].on;
                        if (sc < best) {
                            best = sc;
                            cut2 = l;
                        }
                    }
            }
            if (cut1 < 0 || components(cut1, cut2) < 2) return 0;
            if (!dust_ok && M().min_piece > 0) {
                // no splinters: every part must stay attached elsewhere or be a piece of more than min_piece triangles
                std::vector<int> rt(nf);
                for (int i = 0; i < nf; i++) rt[i] = find(i);
                std::vector<std::pair<uint32_t, uint32_t>> cut;
                for (int l : {cut1, cut2})
                    if (l >= 0) cut.push_back({fan[links[l].a], fan[links[l].b]});
                std::vector<int> done;
                for (int i = 0; i < nf; i++) {
                    if (std::find(done.begin(), done.end(), rt[i]) != done.end()) continue;
                    done.push_back(rt[i]);
                    if (small_piece(fan, rt, rt[i], M().min_piece, cut)) return 0;
                }
            }
            for (int l : {cut1, cut2}) {
                if (l < 0) continue;
                Shell& s = b.shells[fan[links[l].a]];
                Shell& t = b.shells[fan[links[l].b]];
                const int es = edge_of(s, v, links[l].x), et = edge_of(t, v, links[l].x);
                if (es >= 0) s.nb[es] = -1;
                if (et >= 0) t.nb[et] = -1;
                dirty(fan[links[l].a]);
                dirty(fan[links[l].b]);
            }
        }
        // group the fan by component; the one with `keep` (else the biggest) stays on v
        std::vector<int> root(nf), size(nf, 0);
        for (int i = 0; i < nf; i++) size[root[i] = find(i)]++;
        int keep_root = -1;
        for (int i = 0; i < nf; i++)
            if ((int)fan[i] == keep) keep_root = root[i];
        if (keep_root < 0) {
            int best = -1;
            for (int i = 0; i < nf; i++)
                if (size[root[i]] > best) best = size[keep_root = root[i]];
        }
        int created = 0;
        std::vector<int> copy_of(nf, -1);
        for (int i = 0; i < nf; i++) {
            const int r = root[i];
            if (r == keep_root) continue;
            if (copy_of[r] < 0) {
                copy_of[r] = (int)new_node(v);
                log_node((uint32_t)copy_of[r], v);
                created++;
            }
            const uint32_t nv = (uint32_t)copy_of[r];
            Shell& s = b.shells[fan[i]];
            const int c = corner_of(s, v);
            s.n[c] = nv;
            write_tri(s);
            remove_from_fan(v, fan[i]);
            b.node_shells[nv].push_back(fan[i]);
            dirty_node(nv);
            touch(fan[i]);
        }
        if (created) {
            set_node_mass(v);
            for (int i = 0; i < nf; i++)
                if (copy_of[i] >= 0) set_node_mass((uint32_t)copy_of[i]);
            b.shell_stats.cracks++;
        }
        return created;
    }
};

} // namespace

bool SoftBody::process_shell_events() {
    if (rigid) {
        shell_events.clear();
        shell_hit.speed = 0;
        return false;
    }
    if (shell_events.empty() && shell_hit.speed <= 0 && pattern_passes <= 0) return false;
    PROFILE_ACCUM("Sheet topology");
    shell_sync();
    std::vector<ShellEvent> ev;
    ev.swap(shell_events);
    for (const ShellEvent& e : ev)
        if (e.shell < shells.size()) shells[e.shell].pending = shk.aux[e.shell].pending = 0;
    std::vector<uint8_t> touched(shells.size(), 0);
    ShellOps ops{*this, touched, (uint32_t)shells.size() * 2654435761u + topo_version};
    const size_t ns0 = shells.size();
    int changes = 0;
    // the fastest contact of the substep lays a fracture pattern round its point
    if (shell_hit.speed > 0) {
        if (add_impact(shell_hit.x, shell_hit.speed, shell_hit.size, shell_hit.time)) changes++;
        shell_hit.speed = 0;
    }
    // refinements first; a crack queued on a shell that was refined in this batch is re-evaluated on the finer mesh
    {
        PROFILE_ACCUM("Sheet refine");
        for (const ShellEvent& e : ev) {
            if (!ops.budget_left()) break;
            if (e.kind == 0 && e.shell < ns0 && !(e.shell < touched.size() && touched[e.shell])) {
                const int before = shell_stats.refined;
                changes += ops.refine(e.shell) ? 1 : 0;
                refine_left -= shell_stats.refined - before;
            }
        }
    }
    // the impacts' lines refined, a level per substep (from the point of impact out: the detail grows with the
    // stress wave instead of all at once)
    if (pattern_passes > 0) {
        PROFILE_ACCUM("Sheet pattern");
        pattern_passes--;
        const int target = std::max(0, shell_max_level() - 1);
        for (ShellImpact& im : shell_impacts) {
            if (im.level >= target) continue;
            const uint32_t ns = (uint32_t)shells.size();
            for (uint32_t si = 0; si < ns; si++) {
                const Shell& s = shells[si];
                if (s.level > im.level || !ops.can_refine(s)) continue;
                const float lmax = std::max(s.L0[0], std::max(s.L0[1], s.L0[2]));
                const vec2 x0 = shell_x(si, 0), x1 = shell_x(si, 1), x2 = shell_x(si, 2), cx = (x0 + x1 + x2) * (1.0f / 3.0f);
                if (length(cx - im.c) > im.reach + lmax) continue;
                // (a line through the triangle: across one of its edges, or ending inside it)
                const bool on = pattern_cross(x0, x1, 0, 1, 1) >= 0 || pattern_cross(x1, x2, 0, 1, 1) >= 0 || pattern_cross(x2, x0, 0, 1, 1) >= 0 ||
                                pattern_nearest(cx, 0.3f * lmax, 1).d < 0.3f * lmax;
                // (not under the frame's quota: the lines must be ahead of the cracks, else they run off the pattern)
                if (on) changes += ops.refine(si) ? 1 : 0;
            }
            im.level++;
        }
        // the nodes of the refined triangles near a line put on it (the new ones are already)
        std::vector<uint32_t> vs;
        for (uint32_t si = 0; si < touched.size() && si < shells.size(); si++)
            if (touched[si])
                for (int c = 0; c < 3; c++) vs.push_back(shells[si].n[c]);
        std::sort(vs.begin(), vs.end());
        vs.erase(std::unique(vs.begin(), vs.end()), vs.end());
        for (uint32_t v : vs) ops.settle(v, false);
    }

    PROFILE_ACCUM("Sheet cracks"); // (the rest of the batch: picking and opening cracks)
    // cracks: the strongest overloads first, a crack tip (a node already on a free edge) before a new crack, at most
    // crack_rate per substep; the rest are queued again next substep if the load persists
    struct Crack {
        const ShellEvent* e;
        uint32_t first, second;
        float priority;
    };
    std::vector<Crack> cracks;
    for (const ShellEvent& e : ev) {
        if (e.kind == 0 || e.shell >= ns0 || (e.shell < touched.size() && touched[e.shell])) continue;
        const Shell& s = shells[e.shell];
        const uint32_t a = s.n[e.edge], bb = s.n[nx(e.edge)];
        const bool ta = ops.open_fan(a), tb = ops.open_fan(bb);
        uint32_t first = a;
        if (ta != tb) first = ta ? a : bb;
        else if (e.kind == 1) first = node_shells[a].size() >= node_shells[bb].size() ? a : bb;
        cracks.push_back({&e, first, first == a ? bb : a, (e.kind == 1 ? shk.aux[e.shell].strain : 1.0f) + ((ta || tb) ? 1.0f : 0.0f)});
    }
    std::sort(cracks.begin(), cracks.end(), [](const Crack& x, const Crack& y) { return x.priority > y.priority; });
    int budget = std::max(1, shell_mat.crack_rate);
    for (const Crack& ck : cracks) {
        if (budget <= 0) break;
        const ShellEvent& e = *ck.e;
        if (e.shell < touched.size() && touched[e.shell]) continue;
        const Shell& s = shells[e.shell];
        const vec3 pa = nodes[s.n[e.edge]].p, pb = nodes[s.n[nx(e.edge)]].p, pc = nodes[s.n[pv(e.edge)]].p;
        int c = 0;
        if (e.kind == 1) {
            // pulled apart along the edge: the crack runs across it
            const vec3 nrm = cross(pb - pa, pc - pa);
            vec3 across = normalize_or(cross(nrm, pb - pa), vec3(0));
            if (shell_mat.pattern == ShellPattern::Grain && pattern_on()) {
                // wood: a crack not far off the fibres runs along them (the fibres on the current shape: the triangle's
                // map from the material plane)
                const vec2 g(std::cos(shell_mat.grain_angle), std::sin(shell_mat.grain_angle));
                const vec2 a2 = shell_x(e.shell, 1) - shell_x(e.shell, 0), c2 = shell_x(e.shell, 2) - shell_x(e.shell, 0);
                const float det = a2.x * c2.y - a2.y * c2.x;
                if (std::fabs(det) > 1e-14f) {
                    const float u = (g.x * c2.y - g.y * c2.x) / det, w = (a2.x * g.y - a2.y * g.x) / det;
                    const vec3 g3 = normalize_or((nodes[s.n[1]].p - nodes[s.n[0]].p) * u + (nodes[s.n[2]].p - nodes[s.n[0]].p) * w, vec3(0));
                    const float c = dot(g3, across);
                    if (std::fabs(c) > 0.5f) across = c > 0 ? g3 : -g3;
                }
            }
            c = ops.split_node(ck.first, across, -1, (int)e.shell);
            if (!c) c = ops.split_node(ck.second, across, -1, (int)e.shell);
        } else {
            // folded too far: separate the two triangles along the edge, from the crack tip if there is one
            c = ops.split_node(ck.first, normalize_or(nodes[ck.second].p - nodes[ck.first].p, vec3(0)), ck.second, (int)e.shell);
        }
        if (c) budget--;
        else shells[e.shell].cool = shk.aux[e.shell].cool = 40; // cannot crack here now (e.g. it would cut off a splinter): wait a little
        changes += c;
    }
    if (!changes) {
        shell_sync(); // (refinements that failed can still have touched links)
        return false;
    }
    pieces_check = true;
    contacter_count = -1;
    topo_version++;
    shk.version++; // (the arrays follow incrementally: what the operations touched)
    {
        PROFILE_ACCUM("Sheet budget");
        std::vector<uint32_t> vs;
        for (uint32_t si = 0; si < touched.size() && si < shells.size(); si++)
            if (touched[si])
                for (int c = 0; c < 3; c++) vs.push_back(shells[si].n[c]);
        std::sort(vs.begin(), vs.end());
        vs.erase(std::unique(vs.begin(), vs.end()), vs.end());
        enforce_node_budget(&vs);
    }
    {
        PROFILE_ACCUM("Sheet sync");
        shell_sync();
    }
    shell_acc_stale = true;
    topo_changed = true;
    return true;
}

const SoftBody::TopoCounts& SoftBody::topo_counts() const {
    TopoCounts& c = topo_counts_cache;
    if (c.version == topo_version && c.n == shells.size()) return c;
    c = TopoCounts{};
    c.version = topo_version;
    c.n = shells.size();
    for (uint32_t si = 0; si < shells.size(); si++) {
        const Shell& s = shells[si];
        c.level[std::min(4, (int)s.level)]++;
        for (int e = 0; e < 3; e++) {
            if (s.nb[e] > (int)si) c.hinges++; // (each shared edge once)
            else if (s.nb[e] < 0) {
                if ((s.edges >> e) & 1u) c.border++;
                else if ((s.edges >> (3 + e)) & 1u) c.cuts++;
                else c.cracks++;
            }
        }
    }
    return c;
}

int SoftBody::shatter_shells(vec3 p, float radius) {
    if (shells.empty()) return 0;
    make_soft(); // (a rigid piece breaks up like a sheet again)
    AABB box = aabb;
    box.expand(radius);
    if (!box.contains(p)) return 0;
    shell_sync();
    std::vector<uint8_t> touched(shells.size(), 0);
    ShellOps ops{*this, touched, (uint32_t)shells.size() * 40503u + topo_version};
    auto centroid = [&](const Shell& s) { return (nodes[s.n[0]].p + nodes[s.n[1]].p + nodes[s.n[2]].p) / 3.0f; };
    auto near = [&](const Shell& s, float extra) {
        const float lmax = std::max(s.L0[0], std::max(s.L0[1], s.L0[2]));
        return length(centroid(s) - p) < radius + extra * lmax;
    };
    int changes = 0;
    // 1) refine everything near the point to the finest size
    for (int pass = 0; pass < shell_max_level() + 2; pass++) {
        bool any = false;
        const uint32_t ns = (uint32_t)shells.size();
        for (uint32_t si = 0; si < ns; si++)
            if (near(shells[si], 0.5f) && ops.can_refine(shells[si]) && ops.refine(si)) {
                any = true;
                changes++;
            }
        if (!any) break;
    }
    // 2) cut the links of the triangles inside the radius and split their nodes: loose fragments
    std::vector<uint32_t> hit;
    for (uint32_t si = 0; si < shells.size(); si++)
        if (near(shells[si], 0.0f)) hit.push_back(si);
    if (hit.empty() && !changes) return 0;
    std::vector<uint32_t> verts;
    for (uint32_t si : hit) {
        Shell& s = shells[si];
        for (int e = 0; e < 3; e++) {
            const int j = s.nb[e];
            if (j >= 0) {
                Shell& t = shells[j];
                const int f = edge_of(t, s.n[e], s.n[nx(e)]);
                if (f >= 0) t.nb[f] = -1;
                s.nb[e] = -1;
                ops.dirty((uint32_t)j);
                ops.dirty(si);
            }
            verts.push_back(s.n[e]);
        }
    }
    std::sort(verts.begin(), verts.end());
    verts.erase(std::unique(verts.begin(), verts.end()), verts.end());
    for (uint32_t v : verts) changes += ops.split_node(v, vec3(1, 0, 0), -1, -1, true);
    if (!changes) {
        shell_sync();
        return 0;
    }
    contacter_count = -1;
    topo_version++;
    shk.version++;
    enforce_node_budget();
    shell_sync();
    shell_acc_stale = true;
    topo_changed = true;
    pieces_check = true;
    wake();
    return (int)hit.size();
}

int SoftBody::cut_shells(vec3 o, vec3 d0, vec3 d1, float range) {
    if (shells.empty()) return 0;
    make_soft(); // (a rigid piece is cut like a sheet: soft again)
    vec3 m = cross(d0, d1);
    const float ml = length(m);
    if (ml < 1e-9f) return 0;
    m = m / ml;
    auto side = [&](vec3 p) { return dot(p - o, m); };
    {
        // (the body's box on one side of the plane: nothing to cut)
        int pos = 0, neg = 0;
        for (int k = 0; k < 8; k++) {
            const vec3 c((k & 1) ? aabb.mx.x : aabb.mn.x, (k & 2) ? aabb.mx.y : aabb.mn.y, (k & 4) ? aabb.mx.z : aabb.mn.z);
            (side(c) >= 0 ? pos : neg)++;
        }
        if (!pos || !neg) return 0;
    }
    // inside the sector the laser swept: between the two rays, in front, within range
    auto in_wedge = [&](vec3 x) {
        const vec3 w = x - o;
        return dot(cross(d0, w), m) >= 0 && dot(cross(w, d1), m) >= 0 && dot(w, d0 + d1) > 0 && dot(w, w) < range * range;
    };
    auto crossed = [&](const Shell& s) { // an edge of the triangle crosses the plane inside the sector
        float sd[3];
        for (int c = 0; c < 3; c++) sd[c] = side(nodes[s.n[c]].p);
        for (int e = 0; e < 3; e++) {
            const int a = e, b = nx(e);
            if ((sd[a] < 0) == (sd[b] < 0)) continue;
            const vec3 pa = nodes[s.n[a]].p, pb = nodes[s.n[b]].p;
            if (in_wedge(pa + (pb - pa) * (sd[a] / (sd[a] - sd[b])))) return true;
        }
        return false;
    };
    auto centroid = [&](const Shell& s) { return (nodes[s.n[0]].p + nodes[s.n[1]].p + nodes[s.n[2]].p) * (1.0f / 3.0f); };
    shell_sync();
    std::vector<uint8_t> touched(shells.size(), 0);
    ShellOps ops{*this, touched, (uint32_t)shells.size() * 2246822519u + topo_version};
    int changes = 0;
    // 1) the triangles the laser passed through, refined to the finest size: the cut runs along their edges
    for (int pass = 0; pass < shell_max_level() + 2; pass++) {
        bool any = false;
        const uint32_t ns = (uint32_t)shells.size();
        for (uint32_t si = 0; si < ns; si++)
            if (crossed(shells[si]) && ops.can_refine(shells[si]) && ops.refine(si)) {
                any = true;
                changes++;
            }
        if (!any) break;
    }
    // 2) every triangle is on the side of the plane its centroid is on: the links between the two sides are cut where
    // the laser passed between them
    int cuts = 0;
    std::vector<uint32_t> verts;
    for (uint32_t si = 0; si < shells.size(); si++) {
        Shell& s = shells[si];
        const float cs = side(centroid(s));
        for (int e = 0; e < 3; e++) {
            const int j = s.nb[e];
            if (j < (int)si) continue; // (each pair once; -1: no neighbour)
            Shell& t = shells[j];
            if ((cs < 0) == (side(centroid(t)) < 0)) continue;
            // (the link belongs to the step of a sweep that passes its edge's middle: the point where the edge meets the
            // plane jumps to an end once a node of it lies on the plane, and could fall to a step already done)
            const vec3 x = (nodes[s.n[e]].p + nodes[s.n[nx(e)]].p) * 0.5f;
            if (!in_wedge(x)) continue;
            const int f = edge_of(t, s.n[e], s.n[nx(e)]);
            if (f >= 0) {
                t.nb[f] = -1;
                t.edges |= (uint8_t)(8u << f); // (a laser cut: drawn scorched)
            }
            s.nb[e] = -1;
            s.edges |= (uint8_t)(8u << e);
            ops.dirty(si);
            ops.dirty((uint32_t)j);
            verts.push_back(s.n[e]);
            verts.push_back(s.n[nx(e)]);
            cuts++;
        }
    }
    // 3) nodes whose triangles are now on both sides: a copy for each part (position and velocity kept: no blast)
    std::sort(verts.begin(), verts.end());
    verts.erase(std::unique(verts.begin(), verts.end()), verts.end());
    const uint32_t n_before = (uint32_t)nodes.size();
    for (uint32_t v : verts)
        if (node_shells[v].size() >= 2) changes += ops.split_node(v, vec3(1, 0, 0), -1, -1, true, true);
    // 4) a straight cut: the nodes on it (a node and its copies together) onto the laser's plane, along the sheet. The cut
    // ran along triangle edges, a staircase of the finest triangles; each node moves by less than half a triangle. The
    // rest lengths of the triangles around a moved node follow the new shape (no stress from the move). A move that
    // would squash or flip a triangle of any of the copies is not made (the copies stay together: the two sides of the
    // cut keep fitting each other).
    if (cuts) {
        std::vector<std::vector<uint32_t>> groups;
        {
            std::vector<std::pair<uint32_t, uint32_t>> link; // (original, copy)
            for (const auto& [copy, from] : topo_log.nodes)
                if (copy >= n_before) link.push_back({from, copy});
            std::sort(link.begin(), link.end());
            for (uint32_t v : verts) {
                std::vector<uint32_t> g{v};
                for (auto it = std::lower_bound(link.begin(), link.end(), std::make_pair(v, 0u)); it != link.end() && it->first == v; ++it)
                    g.push_back(it->second);
                groups.push_back(std::move(g));
            }
        }
        // (a node the guard stops goes as far towards the plane as it allows; the second pass retries the stopped
        // ones: their neighbours on the cut have moved meanwhile)
        std::vector<uint8_t> done(groups.size(), 0);
        for (int pass = 0; pass < 2; pass++)
            for (size_t gk = 0; gk < groups.size(); gk++) {
                if (done[gk]) continue;
                const auto& g = groups[gk];
                const uint32_t v = g[0];
                if (v >= nodes.size() || (info[v].flags & NF_FIXED)) {
                    done[gk] = 1;
                    continue;
                }
                const vec3 p = nodes[v].p;
                const float dist = side(p);
                // the direction in the sheet across the cut (the plane's normal without its part along the sheet's normal)
                vec3 nrm(0);
                float lmin = 1e30f;
                for (uint32_t w : g)
                    for (uint32_t si : node_shells[w]) {
                        const Shell& t = shells[si];
                        nrm += cross(nodes[t.n[1]].p - nodes[t.n[0]].p, nodes[t.n[2]].p - nodes[t.n[0]].p);
                        for (int e = 0; e < 3; e++) lmin = std::min(lmin, t.L0[e]);
                    }
                nrm = normalize_or(nrm, vec3(0));
                const vec3 dir = m - nrm * dot(m, nrm);
                const float dm = dot(dir, m);
                if (dm < 0.5f || std::fabs(dist) < 1e-6f) {
                    done[gk] = 1;
                    continue;
                }
                const vec3 full = p - dir * (dist / dm);
                if (!in_wedge(full)) {
                    done[gk] = 1;
                    continue;
                }
                // (a squashed triangle's hinges get stiff, ~1 / height^2: the explicit step would not hold them)
                auto fits = [&](vec3 q) {
                    bool ok = true;
                    for (uint32_t w : g)
                        for (uint32_t si : node_shells[w]) {
                            const Shell& t = shells[si];
                            vec3 a[3];
                            for (int c = 0; c < 3; c++) a[c] = t.n[c] == w ? q : nodes[t.n[c]].p;
                            const vec3 n1 = cross(a[1] - a[0], a[2] - a[0]);
                            const vec3 n0 = cross(nodes[t.n[1]].p - nodes[t.n[0]].p, nodes[t.n[2]].p - nodes[t.n[0]].p);
                            ok &= dot(n1, n0) > 0 && length(n1) > 0.75f * length(n0);
                            ok &= t.area0 * length(n1) >= 0.6f * t.area_nom * length(n0); // (see Shell::area_nom)
                            // (every triangle stays on its side of the plane: the next steps of a sweep sort the
                            // triangles by side again, a triangle moved across would never be cut from its old side)
                            const float c0 = side((nodes[t.n[0]].p + nodes[t.n[1]].p + nodes[t.n[2]].p) * (1.0f / 3.0f)), c1 = side((a[0] + a[1] + a[2]) * (1.0f / 3.0f));
                            ok &= (c0 < 0) == (c1 < 0) && std::fabs(c1) > 1e-4f * lmin;
                            for (int e = 0; e < 3; e++) ok &= length(a[nx(e)] - a[e]) > 0.75f * length(nodes[t.n[nx(e)]].p - nodes[t.n[e]].p);
                        }
                    return ok;
                };
                const float reach = std::min(1.0f, 0.4f * lmin / std::max(1e-9f, length(full - p)));
                float f = 0;
                for (float k : {1.0f, 0.7f, 0.45f, 0.25f})
                    if (fits(p + (full - p) * (k * reach))) {
                        f = k * reach;
                        break;
                    }
                if (f < 1.0f && pass == 0) continue; // (not all the way: the second pass, after the neighbours)
                done[gk] = 1;
                if (f <= 0) continue;
                const vec3 q = p + (full - p) * f;
                // The rest shape follows the move. In the material plane (uv in metres) when the sheet has one: the
                // areas of all the triangles around the node and its copies (the whole star it had before the cut)
                // add up to the same, however the sheet is deformed now; else by the current shape.
                vec2 dx(0);
                bool plane = shell_uvm.x > 0 && shell_uvm.y > 0;
                if (plane) {
                    const uint32_t s0 = node_shells[v][0];
                    const Shell& t = shells[s0];
                    const vec3 e1 = nodes[t.n[1]].p - nodes[t.n[0]].p, e2 = nodes[t.n[2]].p - nodes[t.n[0]].p, dp = q - p;
                    const float a11 = dot(e1, e1), a12 = dot(e1, e2), a22 = dot(e2, e2), det = a11 * a22 - a12 * a12;
                    if (det > 1e-18f) {
                        const float b1 = dot(e1, dp), b2 = dot(e2, dp);
                        const float u = (b1 * a22 - b2 * a12) / det, w = (a11 * b2 - a12 * b1) / det; // dp ~ u e1 + w e2
                        dx = (shell_x(s0, 1) - shell_x(s0, 0)) * u + (shell_x(s0, 2) - shell_x(s0, 0)) * w;
                    } else {
                        plane = false;
                    }
                }
                if (plane) {
                    // (only a node that was inside the sheet: the triangles around it and its copies close round it; a
                    // node on the border or on an older cut would take area with it)
                    float ang = 0;
                    for (uint32_t w : g)
                        for (uint32_t si : node_shells[w]) {
                            const int c = corner_of(shells[si], w);
                            const vec2 a = shell_x(si, nx(c)) - shell_x(si, c), b2 = shell_x(si, pv(c)) - shell_x(si, c);
                            ang += std::atan2(std::fabs(cross2(a, b2)), dot(a, b2));
                        }
                    if (std::fabs(ang - 6.28318531f) > 1e-3f) continue;
                }
                const vec2 duv = plane ? vec2(dx.x / shell_uvm.x, dx.y / shell_uvm.y) : vec2(0);
                double area_before = 0, area_after = 0;
                for (uint32_t w : g)
                    for (uint32_t si : node_shells[w]) area_before += shells[si].area0;
                for (uint32_t w : g) {
                    for (uint32_t si : node_shells[w]) {
                        Shell& t = shells[si];
                        const int c = corner_of(t, w);
                        if (plane) {
                            vec2 P[3], Q[3];
                            for (int k = 0; k < 3; k++) {
                                P[k] = shell_x(si, k);
                                Q[k] = P[k] + (k == c ? dx : vec2(0));
                            }
                            for (int e : {c, pv(c)}) { // the two edges at the node
                                const float r = length(Q[nx(e)] - Q[e]) / std::max(1e-9f, length(P[nx(e)] - P[e]));
                                t.L[e] *= r;
                                t.L0[e] *= r;
                            }
                            const float a0 = cross2(P[1] - P[0], P[2] - P[0]), a1 = cross2(Q[1] - Q[0], Q[2] - Q[0]);
                            if (std::fabs(a0) > 1e-14f) t.area0 *= a1 / a0;
                            t.uv[c] = t.uv[c] + duv;
                        } else {
                            for (int e : {c, pv(c)}) {
                                const vec3 pa = nodes[t.n[e]].p, pb = nodes[t.n[nx(e)]].p;
                                const vec3 qa = t.n[e] == w ? q : pa, qb = t.n[nx(e)] == w ? q : pb;
                                const float r = length(qb - qa) / std::max(1e-6f, length(pb - pa));
                                t.L[e] *= r;
                                t.L0[e] *= r;
                            }
                            const vec3 a0 = t.n[0] == w ? q : nodes[t.n[0]].p, a1 = t.n[1] == w ? q : nodes[t.n[1]].p, a2 = t.n[2] == w ? q : nodes[t.n[2]].p;
                            const float before = 0.5f * length(cross(nodes[t.n[1]].p - nodes[t.n[0]].p, nodes[t.n[2]].p - nodes[t.n[0]].p));
                            t.area0 *= 0.5f * length(cross(a1 - a0, a2 - a0)) / std::max(1e-12f, before);
                        }
                        shell_springs(t, shell_material(t));
                        ops.dirty(si);
                        area_after += t.area0;
                    }
                    nodes[w].p = q;
                }
                if (plane && area_after > 1e-12) // (the star round the node keeps its area exactly)
                    for (uint32_t w : g)
                        for (uint32_t si : node_shells[w]) shells[si].area0 = (float)(shells[si].area0 * (area_before / area_after));
                if (pattern_on())
                    for (uint32_t w : g)
                        for (uint32_t si : node_shells[w]) pattern_codes(si);
            }
    }
    if (!cuts && !changes) {
        shell_sync();
        return 0;
    }
    contacter_count = -1;
    topo_version++;
    shk.version++;
    enforce_node_budget();
    shell_sync();
    shell_acc_stale = true;
    topo_changed = true;
    pieces_check = true;
    wake();
    return cuts;
}

void SoftBody::enforce_node_budget(const std::vector<uint32_t>* subset) {
    if (shells.empty() || rigid) return;
    const float h = kDefaultDt / (float)(1 << dt_shift());
    std::vector<uint32_t> all;
    if (!subset) {
        all.resize(nodes.size());
        for (uint32_t v = 0; v < nodes.size(); v++) all[v] = v;
        subset = &all;
    }
    // the springs and hinges a node carries (the kernel: every triangle its own three springs, a hinge per shared
    // edge with the stiffness min(kb) le0^2 / (A1 + A2) acting through the angle's gradient at each of its four nodes).
    // The hinges (a geometry each) only where they can matter: a node of few triangles, or one whose springs are
    // already near the budget; an inner node of a regular fan is far below either
    auto load = [&](uint32_t v, float& ks, float& hs) {
        ks = hs = 0;
        for (uint32_t si : node_shells[v]) {
            const Shell& s = shells[si];
            for (int e = 0; e < 3; e++)
                if (s.n[e] == v || s.n[nx(e)] == v) ks += s.k[e];
        }
        if (node_shells[v].size() > 3 && ks * h * h * nodes[v].inv_mass < 0.5f * edge_budget()) return;
        for (uint32_t si : node_shells[v]) {
            const Shell& s = shells[si];
            for (int e = 0; e < 3; e++) {
                const int j = s.nb[e];
                if (j < 0) continue;
                const Shell& t = shells[j];
                const int fe = edge_of(t, s.n[e], s.n[nx(e)]);
                if (fe < 0) continue;
                const uint32_t id[4] = {s.n[pv(e)], t.n[pv(fe)], s.n[e], s.n[nx(e)]};
                int k = -1;
                for (int q = 0; q < 4; q++)
                    if (id[q] == v) k = q;
                if (k < 0) continue;
                Hinge hg;
                if (!hinge_geometry(nodes[id[0]].p, nodes[id[1]].p, nodes[id[2]].p, nodes[id[3]].p, hg)) continue;
                hs += std::min(s.kb, t.kb) * (s.L0[e] * s.L0[e] / (s.area0 + t.area0)) * length2(hg.g[k]);
            }
        }
    };
    for (int pass = 0; pass < 2; pass++) {
        bool any = false;
        for (uint32_t v : *subset) {
            if (v >= nodes.size() || nodes[v].inv_mass <= 0) continue;
            float ks, hs;
            load(v, ks, hs);
            // (a frame node moves at the frame's step, the substep: FemFrame::hold)
            const float hv = fem.slot(v) >= 0 ? kDefaultDt : h;
            const float re = ks * hv * hv * nodes[v].inv_mass, rh = hs * hv * hv * nodes[v].inv_mass;
            const float tol = pass == 0 ? 1.0f : 1.05f;
            if (re <= edge_budget() * tol && rh <= hinge_budget() * tol) continue;
            const float f = std::min(re > 0 ? edge_budget() / re : 1.0f, rh > 0 ? hinge_budget() / rh : 1.0f);
            for (uint32_t si : node_shells[v]) {
                Shell& s = shells[si];
                const float ns = pass == 0 ? std::min(s.kscale, f) : s.kscale * std::min(1.0f, f);
                if (ns < s.kscale) {
                    s.kscale = ns;
                    shell_springs(s, shell_material(s));
                    shk.dirty_shells.push_back(si);
                    any = true;
                }
            }
        }
        if (!any) break; // (a softened triangle changed its other nodes' sums: one more pass settles them)
    }
}

// ------------------------------------------------------------------------------------------------ membrane
// Every edge (each triangle's own) back within the yield strain of its plastic rest length L: the ends moved apart or
// together along it in proportion to their inverse masses, their velocities with them (a position projection, Gauss-
// Seidel over the triangles: stiff at any step, it takes the energy of the stretch as an inelastic collision would).
// The correction a step may make is that of the membrane's yield force on the edge's strip (membrane x area / L0 per
// triangle: the two triangles of an inner edge carry its width); beyond it the rest length follows, the edge flows
// plastically, and the fracture strain (from L0, the kernel's) is reached in the end.
int SoftBody::project_membrane(float h) {
    if (shells.empty() || rigid) return 0;
    bool any = shell_mat.membrane > 0;
    for (const ShellMaterial& m : shell_mat_extra) any |= m.membrane > 0;
    if (!any) return 0;
    shell_sync();
    if (mem_version != topo_version || mem_shells != shells.size()) {
        // each edge once: the two triangles on it each had its own copy with its own plastic rest length, the one that
        // flowed parted from the other past the band, and the sweeps pulled the edge to one and to the other for good
        // (8 J a frame into a dented drum's buckled end: it trembled and walked on the ground)
        mem_edges.clear();
        for (uint32_t si = 0; si < shells.size(); si++) {
            const Shell& s = shells[si];
            const ShellMaterial& m = shell_material(s);
            if (m.membrane <= 0) continue;
            for (int e = 0; e < 3; e++) {
                const int j = s.nb[e];
                if (j >= 0 && (uint32_t)j < si && shell_material(shells[j]).membrane > 0) continue; // (the one with the lower index has it)
                MemEdge me;
                me.a = s.n[e], me.b = s.n[nx(e)], me.shell = si, me.e = (uint32_t)e, me.hot = 0;
                me.L = s.L[e], me.band = m.yield * s.L0[e], me.force = m.membrane * s.area0 / std::max(1e-6f, s.L0[e]);
                if (j >= 0) {
                    const Shell& t = shells[j];
                    const int f = edge_of(t, s.n[e], s.n[nx(e)]);
                    if (f >= 0 && shell_material(t).membrane > 0) {
                        me.shell2 = (uint32_t)j, me.e2 = (uint32_t)f;
                        me.L = 0.5f * (s.L[e] + t.L[f]); // (copies that had parted meet halfway)
                        me.band = std::min(me.band, shell_material(t).yield * t.L0[f]);
                        me.force += shell_material(t).membrane * t.area0 / std::max(1e-6f, t.L0[f]);
                    }
                }
                mem_edges.push_back(me);
            }
        }
        for (MemEdge& me : mem_edges) { // (the copies set to the shared rest length)
            shells[me.shell].L[me.e] = me.L;
            shk.hot[me.shell >> 2].L[me.e][me.shell & 3] = me.L;
            if (me.shell2 != UINT32_MAX) {
                shells[me.shell2].L[me.e2] = me.L;
                shk.hot[me.shell2 >> 2].L[me.e2][me.shell2 & 3] = me.L;
            }
        }
        mem_version = topo_version;
        mem_shells = shells.size();
    }
    Node* nd = nodes.data();
    const float h2 = h * h, ih = 1.0f / h;
    int moved = 0;
    // a node on the ground stays where the contact left it: the other end of its edges takes the correction (the
    // projection pressed the lowest nodes of a lying drum into the ground, past the contact's slop, and the contact's
    // push out of it kicked them back up every step: the drum drummed and walked)
    if (ground_touch.size() != nodes.size()) ground_touch.assign(nodes.size(), 0);
    const uint8_t* touch = ground_touch.data();
    // Gauss-Seidel sweeps, forth and back, until one moves nothing worth a twentieth of the band (at rest, rolling: the
    // first), at most two (BL_MEMBRANE_ITERS): one sweep carries a push a few triangles on, the drum dropped on its
    // bottom folded its wall before the rows above felt it; four cost a third more and look the same on the drum. (A frame node's double position follows in FemFrame::sync_positions, next: it
    // advances by h v, the velocity with the correction in it.)
    static const int kIters = getenv("BL_MEMBRANE_ITERS") ? atoi(getenv("BL_MEMBRANE_ITERS")) : 2;
    const size_t ne = mem_edges.size();
    // (at rest no plastic flow, as in the hinges (shell_kernel's P.deform): a drum with rings, bent elastically with
    // its dents, pressed on its sheet for good, and the sheet flowed on under it, crept and walked)
    const bool flow = allow_deform && !(resting && max_speed < rest_plastic);
    int out_band = 0;
    // the membrane's damping (membrane_damp): the rate of stretch of the edges that left their band lately taken down by
    // that part, in the first sweep (a pass of its own over the edges cost as much as the projection)
    const float edamp = std::min(membrane_damp, 1.0f);
    constexpr int kHot = 64; // (short steps an edge is damped after it left its band: damping every edge in a pass of its own
                             // cost 2 ms a frame on the Frame Car)
    for (int it = 0; it < kIters; it++) {
        const int before = moved;
        for (size_t k0 = 0; k0 < ne; k0++) {
            MemEdge& me = mem_edges[(it & 1) ? ne - 1 - k0 : k0];
            Node& A = nd[me.a];
            Node& B = nd[me.b];
            const vec3 d = B.p - A.p;
            const float lo = me.L - me.band, hi = me.L + me.band, len2 = length2(d);
            const bool inside = len2 >= lo * lo && len2 <= hi * hi;
            if (it == 0 && edamp > 0) {
                // (the edges that left their band in the last few steps: the rattle is theirs, and the rest of the sheet
                // skipped cheaply)
                if (!inside) me.hot = kHot;
                if (me.hot > 0 && len2 > 1e-14f) {
                    const float wa = A.inv_mass, wb = B.inv_mass, w = wa + wb;
                    if (w > 0) {
                        const float j = edamp * dot(B.v - A.v, d) / (w * len2); // (along d / |d|: no root)
                        A.v += d * (j * wa), B.v -= d * (j * wb);
                    }
                    if (inside) me.hot = me.hot - 1;
                }
            }
            if (inside) continue; // (within the band: most edges, no root)
            if (it == 0) out_band++;
            float wa = (touch[me.a] & 1) ? 0.0f : A.inv_mass, wb = (touch[me.b] & 1) ? 0.0f : B.inv_mass;
            if (wa + wb <= 0) wa = A.inv_mass, wb = B.inv_mass; // (both on the ground: as before)
            const float w = wa + wb;
            const float len = std::sqrt(len2);
            if (!(w > 0) || !(len > 1e-7f)) continue;
            const float over = len > hi ? len - hi : len - lo;
            // (the most the yield force moves it in this step; the rest: plastic flow)
            const float cap = flow ? me.force * h2 * w : 1e30f;
            float c = over;
            if (std::fabs(over) > cap) {
                c = over > 0 ? cap : -cap;
                me.L += over - c;
                shells[me.shell].L[me.e] = me.L;
                shk.hot[me.shell >> 2].L[me.e][me.shell & 3] = me.L;
                if (me.shell2 != UINT32_MAX) {
                    shells[me.shell2].L[me.e2] = me.L;
                    shk.hot[me.shell2 >> 2].L[me.e2][me.shell2 & 3] = me.L;
                }
            }
            const vec3 dir = d * (c / (len * w));
            A.p += dir * wa, A.v += dir * (wa * ih);
            B.p -= dir * wb, B.v -= dir * (wb * ih);
            moved += std::fabs(c) > 0.05f * me.band;
        }
        if (moved == before) break;
    }
    mem_moved = moved, mem_edges_out = out_band;
    return moved;
}

// ------------------------------------------------------------------------------------------------ coarsening
// A bisection put node m on edge a-b and left four triangles round it, (a m c) (m b c) (b m d) (m a d), two on a border.
// Where the sheet has settled they go back to (a b c) (b a d): the node is dropped, the rest lengths along a-b add up,
// the plastic and pattern state of the outer edges is kept, the two parents get the masses and areas of their halves,
// the momentum of m goes to a and b. Nothing is merged across a crack that is not straight, into a thin triangle, or
// where a triangle is loaded (strain, a queued event): a sheet hit again refines the same way as before.
int SoftBody::coarsen_shells(int max_merges, float quiet_frac) {
    if (!shells_only() || shell_level == 0 || max_merges <= 0 || rigid) return 0; // (compaction renumbers the nodes)
    shell_sync();
    const ShellMaterial& M = shell_mat;
    const float quiet = quiet_frac * M.refine;
    std::vector<uint8_t> dead(shells.size(), 0), gone(nodes.size(), 0);
    int merges = 0;
    auto planar = [&](uint32_t si, int c) { return shell_x(si, c); };
    // a shell's data for the merged parent: copies edge `se` of `src` onto edge `de` of `dst`
    auto copy_edge = [&](Shell& dst, int de, const Shell& src, int se) {
        dst.nb[de] = src.nb[se];
        dst.L[de] = src.L[se];
        dst.L0[de] = src.L0[se];
        dst.th0[de] = src.th0[se];
        dst.es[de] = src.es[se];
        dst.hs[de] = src.hs[se];
        dst.edges = (uint8_t)((dst.edges & ~((1u << de) | (8u << de))) | (((src.edges >> se) & 1u) << de) | (((src.edges >> (3 + se)) & 1u) << (3 + de)));
        dst.line = (uint8_t)((dst.line & ~(1u << de)) | (((src.line >> se) & 1u) << de));
    };
    auto relink = [&](int nbr, uint32_t p, uint32_t q, int from, int to) {
        if (nbr < 0) return;
        Shell& t = shells[nbr];
        const int f = edge_of(t, p, q);
        if (f >= 0 && t.nb[f] == from) t.nb[f] = to;
    };
    auto remove_from_fan = [&](uint32_t v, uint32_t si) {
        auto& fan = node_shells[v];
        fan.erase(std::remove(fan.begin(), fan.end(), si), fan.end());
    };
    auto node_mass = [&](uint32_t v) {
        float m = base_mass(v);
        for (uint32_t si : node_shells[v]) m += shells[si].mass / 3.0f;
        nodes[v].mass = m;
        nodes[v].inv_mass = (info[v].flags & NF_FIXED) || m <= 0 ? 0.0f : 1.0f / m;
    };
    // the two halves along a-b (m on it) sewn into one triangle: `keep` (a m c) becomes (a b c), `drop` (m b c) goes.
    // The rest lengths along a-b add up, scaled by `len_scale` (the straight distance a-b over the path through m in
    // the material plane: a node the pattern moved off the line leaves no stress); the areas add up (exactly: the
    // sheet's area is kept whatever the rest shape).
    auto sew = [&](uint32_t keep, uint32_t drop, uint32_t m, uint32_t a, uint32_t b, float len_scale) {
        Shell& K = shells[keep];
        const Shell D = shells[drop];
        const int km = corner_of(K, m), dm = corner_of(D, m);
        const int e_am = pv(km) == corner_of(K, a) ? pv(km) : km; // K's edge a-m (a -> m or m -> a)
        const int e_mb = corner_of(D, b) == nx(dm) ? dm : pv(dm); // D's edge m-b
        // outer edge of D (b-c): onto K's edge that had m and c
        const int e_kc = e_am == km ? pv(km) : km; // K's edge between m and c (the one at m that is not a-m)
        const int e_bc = nx(dm);                    // D's edge b-c (the one not at m)
        // the merged edge a-b keeps K's slot e_am; the edge m-c slot of K takes D's edge b-c
        Shell N = K;
        N.n[km] = b;
        N.L[e_am] = (K.L[e_am] + D.L[e_mb]) * len_scale;
        N.L0[e_am] = (K.L0[e_am] + D.L0[e_mb]) * len_scale;
        N.th0[e_am] = 0.5f * (K.th0[e_am] + D.th0[e_mb]);
        N.es[e_am] = std::min(K.es[e_am], D.es[e_mb]);
        N.hs[e_am] = std::min(K.hs[e_am], D.hs[e_mb]);
        N.edges = (uint8_t)(K.edges & ~((1u << e_am) | (8u << e_am)));
        N.edges |= (uint8_t)(((((K.edges >> e_am) & 1u) & ((D.edges >> e_mb) & 1u)) << e_am) | ((((K.edges >> (3 + e_am)) & 1u) & ((D.edges >> (3 + e_mb)) & 1u)) << (3 + e_am)));
        N.line = (uint8_t)((K.line & ~(1u << e_am)) | ((((K.line >> e_am) & 1u) & ((D.line >> e_mb) & 1u)) << e_am));
        N.nb[e_am] = -1; // (set by the caller: the other parent, or a free edge)
        copy_edge(N, e_kc, D, e_bc);
        N.uv[km] = D.uv[corner_of(D, b)];
        N.mass = K.mass + D.mass;
        N.area0 = K.area0 + D.area0;
        N.area_nom = K.area_nom + D.area_nom;
        N.level = (uint8_t)(std::max(K.level, D.level) - 1);
        // the history: the entries of both halves (each kept the parent's entry for the edge it still had; both number
        // their edges as the parent), without this level's
        for (int l = 0; l < 8; l++)
            if (N.half[l] == kNoHalf) N.half[l] = D.half[l];
        N.half[std::min(7, (int)std::max(K.level, D.level))] = kNoHalf;
        N.flaw = 0.5f * (K.flaw + D.flaw);
        N.pending = N.cool = 0;
        shells[keep] = N;
        // the neighbour across b-c is at edge e_kc of N now: was D's
        {
            const uint32_t p = N.n[e_kc], q = N.n[nx(e_kc)];
            relink(N.nb[e_kc], p, q, (int)drop, (int)keep);
        }
        dead[drop] = 1;
        for (int c = 0; c < 3; c++) remove_from_fan(D.n[c], drop);
        remove_from_fan(m, keep);
        node_shells[b].push_back(keep);
        return e_am;
    };
    const uint32_t nn0 = (uint32_t)nodes.size();
    for (uint32_t m = 0; m < nn0 && merges < max_merges; m++) {
        if (gone[m]) continue;
        const std::vector<uint32_t> fan = node_shells[m];
        const int nf = (int)fan.size();
        if (nf != 4 && nf != 2) continue;
        bool ok = true;
        for (uint32_t si : fan) {
            if (dead[si]) ok = false;
            const Shell& s = shells[si];
            if (s.level == 0 || s.pending || s.cool || (si < shk.aux.size() && shk.aux[si].strain > quiet)) ok = false;
            // (a plastically stretched or bent triangle asks to be refined again at once: it keeps its detail)
            for (int e = 0; e < 3; e++) {
                if (std::fabs(s.L[e] - s.L0[e]) > 0.5f * M.refine_yield * s.L0[e]) ok = false;
                if (std::fabs(s.th0[e]) * bend_equivalent(s.level) > 0.5f * std::min(M.refine_angle, M.bend_yield)) ok = false;
            }
        }
        if (!ok) continue;
        // the bisection that put m in: every shell round m has a half edge at m (its level's entry), ending at a or b
        uint32_t a = UINT32_MAX, b = UINT32_MAX, c = UINT32_MAX, d = UINT32_MAX;
        auto half_of = [&](const Shell& s, uint32_t& far) { // the shell's half edge at its level: its edge index if it ends at m
            const uint8_t h = s.half[std::min(7, (int)s.level)];
            if (h == kNoHalf) return -1;
            const int he = h & 3;
            const uint32_t node = (h & 4) ? s.n[nx(he)] : s.n[he];
            if (node != m) return -1;
            far = (h & 4) ? s.n[he] : s.n[nx(he)];
            return he;
        };
        for (uint32_t si : fan) {
            const Shell& s = shells[si];
            uint32_t far;
            if (half_of(s, far) < 0) { ok = false; break; }
            if (a == UINT32_MAX || far == a) a = far;
            else if (b == UINT32_MAX || far == b) b = far;
            else { ok = false; break; }
        }
        if (!ok || a == UINT32_MAX || b == UINT32_MAX) continue;
        {
            int na = 0, nb2 = 0;
            for (uint32_t si : fan) {
                uint32_t far = 0;
                half_of(shells[si], far);
                (far == a ? na : nb2)++;
            }
            if (na != nf / 2 || nb2 != nf / 2) continue;
        }
        auto shell_with = [&](uint32_t p, uint32_t q) -> int {
            for (uint32_t si : fan)
                if (corner_of(shells[si], p) >= 0 && corner_of(shells[si], q) >= 0) return (int)si;
            return -1;
        };
        // the other spokes: c (shared by the a-side and b-side shells of one parent), d (of the other)
        for (uint32_t si : fan) {
            const Shell& s = shells[si];
            for (int k = 0; k < 3; k++) {
                const uint32_t v = s.n[k];
                if (v == m || v == a || v == b) continue;
                if (c == UINT32_MAX || c == v) c = v;
                else if (d == UINT32_MAX || d == v) d = v;
                else ok = false;
            }
        }
        if (!ok || c == UINT32_MAX || (nf == 4 && d == UINT32_MAX) || (nf == 2 && d != UINT32_MAX)) continue;
        if (nf == 2) {
            // border: the half edges are free, of the same kind (both border, both cut, both crack)
            uint32_t kind[2];
            int k = 0;
            for (uint32_t si : fan) {
                const Shell& s = shells[si];
                uint32_t far;
                const int h = half_of(s, far);
                if (h < 0 || s.nb[h] >= 0) { ok = false; break; }
                kind[k++] = ((s.edges >> h) & 1u) | (((s.edges >> (3 + h)) & 1u) << 1);
            }
            if (!ok || kind[0] != kind[1]) continue;
        }
        // the parents' shapes in the material plane: well formed, and the scale of the sewn edge (a node the pattern moved
        // off the line: the parent takes the straight edge)
        float len_scale = 1;
        {
            const int sa = shell_with(a, m), sb = shell_with(b, m);
            if (sa < 0 || sb < 0) continue;
            const vec2 xm = planar((uint32_t)sa, corner_of(shells[sa], m)), xa = planar((uint32_t)sa, corner_of(shells[sa], a)),
                       xb = planar((uint32_t)sb, corner_of(shells[sb], b));
            const vec2 u = xa - xm, v = xb - xm;
            const float lu = length(u), lv = length(v), lab = length(xb - xa);
            if (lu < 1e-7f || lv < 1e-7f || lab < 1e-7f || dot(u, v) > 0) continue; // (m beside the edge, not on it)
            len_scale = lab / (lu + lv);
            const uint32_t far[2] = {c, d};
            for (int i = 0; i < (nf == 4 ? 2 : 1); i++) {
                const int sx = shell_with(far[i], m);
                if (sx < 0) continue;
                const vec2 xx = planar((uint32_t)sx, corner_of(shells[sx], far[i]));
                const float area = 0.5f * std::fabs(cross2(xb - xa, xx - xa));
                // (the parent has the heavier nodes, its edge^2 / area is that of its halves: thinner than kShapeMin is
                // still within the step; only a sliver is refused)
                if (tri_quality(area, lab, length(xx - xa), length(xb - xx)) < 0.4f) ok = false;
            }
            if (!ok) continue;
        }
        if ((info[m].flags & NF_FIXED) && !((info[a].flags & info[b].flags) & NF_FIXED)) continue;
        // sew: (a m c)+(m b c) -> (a b c), and (b m d)+(m a d) -> (b a d)
        const vec3 pm = nodes[m].v * nodes[m].mass;
        const int s_ac = shell_with(a, c), s_cb = shell_with(c, b);
        if (s_ac < 0 || s_cb < 0) continue;
        const int e1 = sew((uint32_t)s_ac, (uint32_t)s_cb, m, a, b, len_scale);
        if (nf == 4) {
            const int s_bd = shell_with(b, d), s_da = shell_with(d, a);
            if (s_bd < 0 || s_da < 0) { // (cannot happen after the checks; the sheet is left consistent enough to compact)
                dead[s_ac] = 0;
                continue;
            }
            const int e2 = sew((uint32_t)s_bd, (uint32_t)s_da, m, b, a, len_scale);
            shells[s_ac].nb[e1] = s_bd;
            shells[s_bd].nb[e2] = s_ac;
        } else {
            shells[s_ac].nb[e1] = -1;
        }
        node_shells[m].clear();
        gone[m] = 1;
        for (uint32_t v : {a, b, c}) node_mass(v);
        if (nf == 4) node_mass(d);
        // the momentum of m onto a and b
        for (uint32_t v : {a, b})
            if (nodes[v].inv_mass > 0) nodes[v].v += pm * (0.5f * nodes[v].inv_mass);
        merges++;
    }
    if (!merges) return 0;
    // compact: the dead shells and the dropped nodes out of the arrays
    const uint32_t ns = (uint32_t)shells.size(), nn = (uint32_t)nodes.size();
    std::vector<uint32_t> sidx(ns, UINT32_MAX), nidx(nn, UINT32_MAX);
    uint32_t k = 0;
    for (uint32_t si = 0; si < ns; si++)
        if (!dead[si]) sidx[si] = k++;
    uint32_t q = 0;
    for (uint32_t v = 0; v < nn; v++)
        if (!gone[v]) nidx[v] = q++;
    std::vector<Shell> shells2;
    shells2.reserve(k);
    for (uint32_t si = 0; si < ns; si++) {
        if (dead[si]) continue;
        Shell s = shells[si];
        for (int c = 0; c < 3; c++) {
            s.n[c] = nidx[s.n[c]];
            if (s.nb[c] >= 0) s.nb[c] = dead[s.nb[c]] ? -1 : (int32_t)sidx[s.nb[c]];
        }
        shells2.push_back(s);
    }
    std::vector<Node> nodes2;
    std::vector<NodeInfo> info2;
    std::vector<float> wind2;
    nodes2.reserve(q);
    info2.reserve(q);
    for (uint32_t v = 0; v < nn; v++) {
        if (gone[v]) continue;
        nodes2.push_back(nodes[v]);
        info2.push_back(info[v]);
        if (!wind_area.empty()) wind2.push_back(wind_area[v]);
    }
    if (grab_node >= 0) grab_node = gone[grab_node] ? -1 : (int)nidx[grab_node];
    remap_node_refs([&](uint32_t i) { return gone[i] ? (int64_t)-1 : (int64_t)nidx[i]; });
    nodes.swap(nodes2);
    info.swap(info2);
    wind_area.swap(wind2);
    shells.swap(shells2);
    tris.clear();
    node_shells.assign(nodes.size(), {});
    int level = 0;
    for (uint32_t si = 0; si < shells.size(); si++) {
        Shell& s = shells[si];
        for (int c = 0; c < 3; c++) node_shells[s.n[c]].push_back(si);
        Triangle t;
        t.a = s.n[0];
        t.b = s.n[1];
        t.c = s.n[2];
        t.rest_edge2 = 0;
        s.tri = (uint32_t)tris.size();
        tris.push_back(t);
        level = std::max(level, (int)s.level);
        if (pattern_on()) pattern_codes(si);
        else shell_springs(s, shell_material(s));
    }
    shell_level = level;
    force.assign(nodes.size(), vec3(0));
    ext_force.clear();
    shell_events.clear();
    node_wheel.clear();
    contacter_count = -1;
    topo_version++;
    shk.dirty_shells.clear();
    shk.dirty_nodes.clear();
    shk.version = ~0u; // (renumbered: the kernel's arrays are rebuilt)
    enforce_node_budget(); // (a coarser sheet takes a longer step: the light nodes left at the cracks)
    shk.dirty_shells.clear();
    shell_acc_stale = true;
    topo_changed = true;
    topo_log.overflow = true;
    compute_aabb();
    ordered_shells = shells.size();
    return merges;
}

int SoftBody::detach_pieces(std::vector<std::unique_ptr<SoftBody>>& out) {
    if (shells.empty() || !beams.empty() || !joints.empty() || !frames.empty() || !wheels.empty() || !slides.empty() || !capsules.empty())
        return 0;
    {
        // parts joined only at a node (no shared edge) have no strength there: separate them first, so that a piece
        // held by a single corner falls off instead of swinging on it forever
        std::vector<uint8_t> touched(shells.size(), 0);
        ShellOps ops{*this, touched, 777u};
        const uint32_t n0 = (uint32_t)nodes.size();
        int split = 0;
        for (uint32_t v = 0; v < n0; v++)
            if (node_shells[v].size() >= 2) split += ops.split_node(v, vec3(1, 0, 0), -1, -1, true, true);
        if (split) topo_version++;
    }
    const uint32_t nn = (uint32_t)nodes.size(), ns = (uint32_t)shells.size();
    std::vector<uint32_t> par(nn);
    for (uint32_t i = 0; i < nn; i++) par[i] = i;
    auto find = [&](uint32_t x) {
        while (par[x] != x) x = par[x] = par[par[x]];
        return x;
    };
    for (const Shell& s : shells) {
        const uint32_t a = find(s.n[0]);
        for (int c = 1; c < 3; c++) {
            const uint32_t r = find(s.n[c]);
            if (r != a) par[r] = a;
        }
    }
    for (const FrameElement& e : fem.elems) { // (a sheet on a frame: its members hold the parts together too)
        if (e.broken || e.a >= fem.node.size() || e.b >= fem.node.size()) continue;
        const uint32_t a = find(fem.node[e.a]), r = find(fem.node[e.b]);
        if (a != r && a < nn && r < nn) par[r] = a;
    }
    std::vector<uint32_t> count(nn, 0);
    std::vector<uint8_t> anchored(nn, 0);
    for (const Shell& s : shells) count[find(s.n[0])]++;
    bool any_anchor = false;
    for (uint32_t i = 0; i < nn; i++)
        if (info[i].flags & NF_FIXED) any_anchor |= (anchored[find(i)] = 1) != 0;
    uint32_t biggest = UINT32_MAX;
    for (uint32_t i = 0; i < nn; i++)
        if (par[i] == i && count[i] > 0 && (biggest == UINT32_MAX || count[i] > count[biggest])) biggest = i;
    std::vector<int> piece(nn, -1);
    int np = 0;
    for (uint32_t i = 0; i < nn; i++)
        if (par[i] == i && count[i] > 0 && !(any_anchor ? anchored[i] != 0 : i == biggest)) piece[i] = np++;
    if (np == 0) return 0;
    // the pieces and the rest of the sheet share a collision group: they were one body (no self collision) and
    // leave each other along the crack faces, touching everywhere
    if (collision_group == 0) collision_group = 1000000 + id;
    std::vector<SoftBody*> pb(np);
    for (int k = 0; k < np; k++) {
        auto b = std::make_unique<SoftBody>();
        b->name = name + " piece";
        b->is_piece = true;
        b->shell_mat = shell_mat;
        b->shell_mat_base = shell_mat_base;
        b->shell_mat_extra = shell_mat_extra;
        b->shell_mat_extra_base = shell_mat_extra_base;
        b->collision_radius = collision_radius;
        b->ground_friction = ground_friction;
        b->air_drag = air_drag;
        b->can_sleep = can_sleep;
        b->allow_break = allow_break;
        b->allow_deform = allow_deform;
        b->ductile = ductile;
        b->collision_group = collision_group;
        b->sleep_speed = sleep_speed;
        b->shell_uvm = shell_uvm; // (the pattern goes on in the piece)
        b->shell_impacts = shell_impacts;
        b->pattern_seed = pattern_seed;
        pb[k] = b.get();
        out.push_back(std::move(b));
    }
    // split the nodes and shells between this body (kept pieces) and the new ones
    std::vector<uint32_t> nidx(nn), sidx(ns);
    std::vector<Node> keep_nodes;
    std::vector<NodeInfo> keep_info;
    std::vector<float> keep_wind;
    for (uint32_t i = 0; i < nn; i++) {
        const int p = piece[find(i)];
        SoftBody& t = p < 0 ? *this : *pb[p];
        std::vector<Node>& tn = p < 0 ? keep_nodes : t.nodes;
        nidx[i] = (uint32_t)tn.size();
        tn.push_back(nodes[i]);
        (p < 0 ? keep_info : t.info).push_back(info[i]);
        if (!wind_area.empty()) (p < 0 ? keep_wind : t.wind_area).push_back(wind_area[i]);
    }
    std::vector<Shell> keep_shells;
    for (uint32_t si = 0; si < ns; si++) {
        const int p = piece[find(shells[si].n[0])];
        std::vector<Shell>& ts = p < 0 ? keep_shells : pb[p]->shells;
        sidx[si] = (uint32_t)ts.size();
        ts.push_back(shells[si]);
    }
    if (grab_node >= 0) grab_node = piece[find((uint32_t)grab_node)] < 0 ? (int)nidx[grab_node] : -1;
    remap_node_refs([&](uint32_t i) { return piece[find(i)] < 0 ? (int64_t)nidx[i] : (int64_t)-1; }); // (a volume on a piece: off)
    std::vector<int> part_of;
    if (!fem.empty()) {
        part_of.resize(nn);
        for (uint32_t i = 0; i < nn; i++) part_of[i] = piece[find(i)];
    }
    nodes.swap(keep_nodes);
    info.swap(keep_info);
    wind_area.swap(keep_wind);
    shells.swap(keep_shells);
    auto relink = [&](SoftBody& b) {
        b.tris.clear();
        b.node_shells.assign(b.nodes.size(), {});
        int level = 0;
        for (uint32_t si = 0; si < b.shells.size(); si++) {
            Shell& s = b.shells[si];
            for (int c = 0; c < 3; c++) {
                s.n[c] = nidx[s.n[c]];
                if (s.nb[c] >= 0) s.nb[c] = (int32_t)sidx[s.nb[c]];
                b.node_shells[s.n[c]].push_back(si);
            }
            Triangle t;
            t.a = s.n[0];
            t.b = s.n[1];
            t.c = s.n[2];
            t.rest_edge2 = 0;
            s.tri = (uint32_t)b.tris.size();
            b.tris.push_back(t);
            s.pending = 0;
            level = std::max(level, (int)s.level);
        }
        b.shell_level = level;
        b.force.assign(b.nodes.size(), vec3(0));
        b.contacter_count = -1;
        b.topo_version++;
        b.shk.dirty_shells.clear(); // (renumbered: the kernel's arrays are rebuilt)
        b.shk.dirty_nodes.clear();
        b.shk.version = ~0u;
        b.compute_aabb();
        b.max_speed = 0;
        for (const Node& x : b.nodes) b.max_speed = std::max(b.max_speed, length(x.v));
    };
    relink(*this);
    if (!fem.empty()) fem.split_off(*this, part_of, nidx, pb); // (the frame's members go with their pieces)
    enforce_node_budget();
    shk.dirty_shells.clear();
    for (SoftBody* b : pb) {
        relink(*b);
        b->enforce_node_budget(); // (its own level, its own step)
        b->shk.dirty_shells.clear();
        b->shell_cap = std::max(shell_cap, b->shells.size() * 2 + 16); // (a piece hit again refines as the sheet did)
        b->reorder_shells();
        b->ordered_shells = b->shells.size();
    }
    return np;
}

namespace {
inline uint32_t spread10(uint32_t x) { // 10 bits -> every third bit
    x &= 0x3ff;
    x = (x | (x << 16)) & 0x030000ff;
    x = (x | (x << 8)) & 0x0300f00f;
    x = (x | (x << 4)) & 0x030c30c3;
    x = (x | (x << 2)) & 0x09249249;
    return x;
}
} // namespace

bool SoftBody::reorder_shells() {
    const size_t ns = shells.size(), nn = nodes.size();
    if (ns < 64 || keep_node_order || tris.size() != ns || !beams.empty() || !joints.empty() || !frames.empty() || !wheels.empty() || !slides.empty() ||
        !capsules.empty())
        return false;
    vec3 mn(1e30f), mx(-1e30f);
    for (const Node& x : nodes) {
        mn = vmin(mn, x.p);
        mx = vmax(mx, x.p);
    }
    const vec3 ext = vmax(mx - mn, vec3(1e-6f));
    const float q = 1023.0f / maxc(ext); // (one scale on all axes: a thin sheet keeps its shape in the curve)
    std::vector<std::pair<uint32_t, uint32_t>> key(ns);
    for (uint32_t si = 0; si < ns; si++) {
        const Shell& s = shells[si];
        const vec3 c = (nodes[s.n[0]].p + nodes[s.n[1]].p + nodes[s.n[2]].p) * (1.0f / 3.0f) - mn;
        const uint32_t x = (uint32_t)std::clamp(c.x * q, 0.0f, 1023.0f), y = (uint32_t)std::clamp(c.y * q, 0.0f, 1023.0f),
                       z = (uint32_t)std::clamp(c.z * q, 0.0f, 1023.0f);
        key[si] = {spread10(x) | (spread10(y) << 1) | (spread10(z) << 2), si};
    }
    std::sort(key.begin(), key.end());
    std::vector<uint32_t> sidx(ns), nidx(nn, UINT32_MAX);
    uint32_t next = 0;
    for (uint32_t k = 0; k < ns; k++) {
        sidx[key[k].second] = k;
        const Shell& s = shells[key[k].second];
        for (int c = 0; c < 3; c++)
            if (nidx[s.n[c]] == UINT32_MAX) nidx[s.n[c]] = next++;
    }
    for (uint32_t i = 0; i < nn; i++)
        if (nidx[i] == UINT32_MAX) nidx[i] = next++; // (nodes of no shell keep their order at the end)
    std::vector<Node> nodes2(nn);
    std::vector<NodeInfo> info2(nn);
    std::vector<float> wind2(wind_area.empty() ? 0 : nn);
    for (uint32_t i = 0; i < nn; i++) {
        nodes2[nidx[i]] = nodes[i];
        info2[nidx[i]] = info[i];
        if (!wind_area.empty()) wind2[nidx[i]] = wind_area[i];
    }
    std::vector<Shell> shells2(ns);
    std::vector<Triangle> tris2(ns);
    for (uint32_t si = 0; si < ns; si++) {
        Shell s = shells[si];
        const Triangle t = tris[s.tri];
        const uint32_t k = sidx[si];
        for (int c = 0; c < 3; c++) {
            s.n[c] = nidx[s.n[c]];
            if (s.nb[c] >= 0) s.nb[c] = (int32_t)sidx[s.nb[c]];
        }
        s.tri = k;
        shells2[k] = s;
        Triangle& t2 = tris2[k];
        t2 = t;
        t2.a = s.n[0];
        t2.b = s.n[1];
        t2.c = s.n[2];
    }
    // Rotate every triangle's corners (the winding stays) so that the edges whose hinge it evaluates come first: the
    // kernel runs the hinges of 4 triangles per edge index as one vector, with the owners aligned most lanes are busy
    // on edge 0 and 1 and edge 2 is mostly skipped.
    std::vector<uint8_t> rot(ns, 0);
    for (uint32_t si = 0; si < ns; si++) {
        const Shell& s = shells2[si];
        int own = 0;
        for (int e = 0; e < 3; e++) {
            const int j = s.nb[e];
            if (j < 0 || edge_of(shells2[j], s.n[e], s.n[nx(e)]) < 0) continue;
            const int cs = rate_class(s.level), ct = rate_class(shells2[j].level);
            if (cs > ct || (cs == ct && si < (uint32_t)j)) own |= 1 << e;
        }
        int r = 0; // (new edge 0 = old edge r)
        if (own == 2 || own == 6) r = 1;
        else if (own == 4 || own == 5) r = 2;
        if (!r) continue;
        rot[si] = (uint8_t)r;
        const Shell o = s;
        Shell& t = shells2[si];
        t.line = 0;
        t.edges = 0;
        for (int c = 0; c < 3; c++) {
            const int q = (c + r) % 3;
            t.n[c] = o.n[q];
            t.uv[c] = o.uv[q];
            t.nb[c] = o.nb[q];
            t.L[c] = o.L[q];
            t.L0[c] = o.L0[q];
            t.th0[c] = o.th0[q];
            t.k[c] = o.k[q];
            t.d[c] = o.d[q];
            t.es[c] = o.es[q];
            t.hs[c] = o.hs[q];
            t.line |= (uint8_t)(((o.line >> q) & 1u) << c);
            for (int l = 0; l < 8; l++)
                if (o.half[l] != kNoHalf && (o.half[l] & 3) == q) t.half[l] = half_code(c, o.half[l] & 4);
            t.edges |= (uint8_t)((((o.edges >> q) & 1u) << c) | (((o.edges >> (3 + q)) & 1u) << (3 + c)));
        }
        Triangle& tr = tris2[si];
        tr.a = t.n[0];
        tr.b = t.n[1];
        tr.c = t.n[2];
    }
    nodes.swap(nodes2);
    info.swap(info2);
    wind_area.swap(wind2);
    shells.swap(shells2);
    tris.swap(tris2);
    force.assign(nn, vec3(0));
    ext_force.clear();
    node_shells.assign(nn, {});
    for (uint32_t si = 0; si < ns; si++)
        for (int c = 0; c < 3; c++) node_shells[shells[si].n[c]].push_back(si);
    for (ShellEvent& e : shell_events)
        if (e.shell < ns) {
            e.shell = sidx[e.shell];
            e.edge = (uint8_t)((e.edge + 3 - rot[e.shell]) % 3); // (the edges were rotated with the corners)
        }
    if (grab_node >= 0 && grab_node < (int)nn) grab_node = (int)nidx[grab_node];
    remap_node_refs([&](uint32_t i) { return i < nn ? (int64_t)nidx[i] : (int64_t)-1; });
    if (!fem.empty()) fem.renumber(*this, nidx); // (a sheet on a frame: the frame's nodes renumbered with it)
    node_wheel.clear();
    topo_log.clear();
    topo_version++;
    shk.dirty_shells.clear();
    shk.dirty_nodes.clear();
    shk.version = ~0u; // (rebuilt: every index changed)
    shell_acc_stale = true;
    ordered_shells = ns;
    return true;
}

} // namespace bl::phys
