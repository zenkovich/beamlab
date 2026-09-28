// Reference copy of the triangle-element force kernel as it was before the optimisation (2026-09-25): the optimised
// kernel must give the same forces and side effects (plastic state, strain, events). Generated from src/phys/shell.cpp.
#include "reference_shell.h"
#include "core/profiler.h"
#include "phys/shell_util.h"

#include <algorithm>
#include <cmath>

namespace bl::phys::ref {
namespace {

// stability budgets of the shells' share of a node (the explicit limit is about 2; see SoftBody::stabilize)
constexpr float kEdgeBudget = 0.45f;  // sum(k) dt^2 / m of the edge springs
constexpr float kHingeBudget = 0.2f;  // the same for the bending hinges
constexpr float kDampBudget = 0.4f;   // sum(d) dt / m

inline int nx(int e) { return e == 2 ? 0 : e + 1; }
inline int pv(int e) { return e == 0 ? 2 : e - 1; }
inline uint32_t hash32(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}
inline float rand_pm1(uint32_t h) { return (float)(hash32(h) & 0xffffff) / (float)0x800000 - 1.0f; }

// edge of t with endpoints a, b (either direction), -1 if none
inline int edge_of(const Shell& t, uint32_t a, uint32_t b) {
    for (int f = 0; f < 3; f++) {
        uint32_t p = t.n[f], q = t.n[nx(f)];
        if ((p == a && q == b) || (p == b && q == a)) return f;
    }
    return -1;
}
inline int corner_of(const Shell& s, uint32_t v) { return s.n[0] == v ? 0 : s.n[1] == v ? 1 : s.n[2] == v ? 2 : -1; }
inline int longest_edge(const Shell& s) {
    int e = 0;
    for (int f = 1; f < 3; f++)
        if (s.L0[f] > s.L0[e] * 1.0005f) e = f;
    return e;
}
// Bending thresholds are moments, expressed as the fold angle of an authored-size triangle: a finer triangle at the
// same moment folds by a different angle (shorter lever: smaller angle per curvature; softer hinge, see
// shell_springs: larger angle). Without this the softer refined zone would crack under loads the sheet carries.
inline float bend_equivalent(int level) {
    static const float table[8] = {1.0f, 1.41421356f, 0.5f, 0.70710678f, 0.25f, 0.35355339f, 0.125f, 0.1767767f};
    return table[std::min(level, 7)]; // 0.25^(level/2) * sqrt(2)^level
}
// atan2 to ~1e-5 rad (the hinge angle is needed for every edge in every short step)
inline float fast_atan2(float y, float x) {
    const float ax = std::fabs(x), ay = std::fabs(y);
    const float mx = std::max(ax, ay), mn = std::min(ax, ay);
    const float a = mn / std::max(mx, 1e-30f);
    const float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    if (ay > ax) r = 1.57079637f - r;
    if (x < 0) r = 3.14159274f - r;
    return y < 0 ? -r : r;
}
inline float wrap_pi(float a) {
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
}

// Dihedral angle across edge x3-x4 between triangles (x3, x4, x1) and (x4, x3, x2), 0 = flat, and its gradient
// (Bridson et al. 2003, "Simulation of clothing with folds and wrinkles").
struct Hinge {
    float theta;
    float fh;   // |E|^2 / (A1 + A2): the hinge stiffness of a continuous bending modulus, the same at every size
    float le, h1, h2; // edge length, heights of x1 and x2 over the edge
    vec3 g[4];  // d theta / d x1, x2, x3, x4
};
inline bool hinge_geometry(vec3 x1, vec3 x2, vec3 x3, vec3 x4, Hinge& h) {
    const vec3 e = x4 - x3;
    const float le2 = dot(e, e);
    if (le2 < 1e-12f) return false;
    const float le = std::sqrt(le2);
    const vec3 N1 = cross(e, x1 - x3), N2 = cross(x3 - x4, x2 - x4);
    const float a1 = dot(N1, N1), a2 = dot(N2, N2);
    if (a1 < 1e-16f || a2 < 1e-16f) return false;
    const float l1 = std::sqrt(a1), l2 = std::sqrt(a2);
    const vec3 n1 = N1 / l1, n2 = N2 / l2;
    h.theta = std::atan2(dot(cross(n1, n2), e) / le, dot(n1, n2)); // (exact: the kernel's own angle error is measured)
    h.le = le;
    h.h1 = l1 / le;
    h.h2 = l2 / le;
    const vec3 w1 = N1 / a1, w2 = N2 / a2;
    h.g[0] = w1 * (-le);
    h.g[1] = w2 * (-le);
    h.g[2] = -(w1 * (dot(x1 - x4, e) / le) + w2 * (dot(x2 - x4, e) / le));
    h.g[3] = w1 * (dot(x1 - x3, e) / le) + w2 * (dot(x2 - x3, e) / le);
    h.fh = le2 / (0.5f * (l1 + l2));
    return true;
}

} // namespace

void RefBody::ref_forces(float h, int step, int sub) {
    if (shells.empty()) return;
    Node* nd = nodes.data();
    const size_t nn = nodes.size();
    // Multi-rate: a shell is evaluated at the rate its size needs (class c = ceil(level / 2): every sub >> c short
    // steps) and its forces are held in between; the nodes all move with the short step. A damaged sheet pays the
    // short step only for its fine triangles. After a topology change everything is evaluated again (a held force
    // would push a split node with the force of triangles it no longer carries).
    int maxc = 0;
    while ((2 << maxc) <= sub && maxc < 2) maxc++;
    bool due[3] = {false, false, false};
    for (int c = 0; c <= maxc; c++) {
        due[c] = step % (sub >> c) == 0 || shell_acc_stale;
        if (due[c]) shell_acc[c].assign(nn, vec3(0));
        else if (shell_acc[c].size() < nn) shell_acc[c].resize(nn, vec3(0));
    }
    shell_acc_stale = false;
    auto cls = [&](const Shell& x) { return std::min(maxc, ((int)x.level + 1) / 2); };
    const ShellMaterial& M = shell_mat;
    const bool deform = allow_deform, brk = allow_break;
    const int nsh = (int)shells.size();
    vec3 vmean(0);
    if (M.flex_damp > 0) {
        float m = 0;
        for (size_t i = 0; i < nn; i++) {
            vmean += nd[i].v * nd[i].mass;
            m += nd[i].mass;
        }
        vmean = m > 0 ? vmean / m : vec3(0);
    }
    const bool budget_left = shells.size() + 4 <= shell_cap;
    auto can_refine = [&](const Shell& s) {
        const float lmax = std::max(s.L0[0], std::max(s.L0[1], s.L0[2]));
        return budget_left && s.level < M.max_level && lmax * 0.70710678f >= M.min_edge;
    };
    auto queue = [&](int si, uint8_t kind, int e) {
        shell_events.push_back({(uint32_t)si, kind, (uint8_t)e});
        shells[si].pending = 1;
    };
    {
    PROFILE_ACCUM("Sheet edges"); // (per triangle: edge springs, air drag, clamped edges, overload checks)
    for (int si = 0; si < nsh; si++) {
        Shell& s = shells[si];
        const int sc = cls(s);
        if (!due[sc]) continue;
        vec3* f = shell_acc[sc].data();
        const float dt = h * (float)(sub >> sc);
        float worst = 0, plastic = 0;
        int worst_e = 0;
        for (int e = 0; e < 3; e++) {
            const uint32_t a = s.n[e], b = s.n[nx(e)];
            const vec3 dis = nd[a].p - nd[b].p;
            const float len2 = dot(dis, dis);
            if (len2 < 1e-14f) continue;
            const float inv = 1.0f / std::sqrt(len2);
            const float len = len2 * inv;
            float k = s.k[e], d = s.d[e];
            if (M.tension_only && len < s.L[e]) {
                k = 0;
                d *= 0.1f;
            }
            const float lim = s.L0[e] * (M.brk * s.flaw * ((float)s.es[e] * (1.0f / 64.0f))); // elongation at fracture (x the pattern's code)
            if (deform && k > 0 && len - s.L0[e] < lim) {
                // plastic yield: beyond the yield strain the rest length follows (not past the fracture strain: a
                // material that has used up its ductility only loads its neighbours further, it does not flow forever)
                const float ylen = M.yield * s.L0[e];
                const float diff = len - s.L[e];
                if (diff > ylen) s.L[e] = len - ylen;
                else if (diff < -ylen) s.L[e] = len + ylen;
            }
            const float slen = -k * (len - s.L[e]) - d * dot(nd[a].v - nd[b].v, dis) * inv;
            const vec3 fv = dis * (slen * inv);
            f[a] += fv;
            f[b] -= fv;
            const float r = (len - s.L0[e]) / lim;
            if (r > worst && len > s.L[e]) { // (only while it is being pulled: a relaxed, stretched flap is no crack)
                worst = r;
                worst_e = e;
            }
            plastic = std::max(plastic, std::fabs(s.L[e] - s.L0[e]) / s.L0[e]);
        }
        s.strain = worst;
        {
            const vec3 p0 = nd[s.n[0]].p, p1 = nd[s.n[1]].p, p2 = nd[s.n[2]].p;
            const vec3 N = cross(p1 - p0, p2 - p0);
            const float n2 = dot(N, N);
            if (n2 > 1e-16f) {
                const vec3 nrm = N / std::sqrt(n2);
                const float vn = dot(nd[s.n[0]].v + nd[s.n[1]].v + nd[s.n[2]].v, nrm) * (1.0f / 3.0f);
                // air drag on the face: -0.5 rho Cd A |vn| vn; internal friction of the motion across the sheet
                // relative to the body; a third of each on every corner
                const float drag = -0.5f * 1.225f * 1.2f * M.aero * 0.5f * std::sqrt(n2) * std::fabs(vn) * vn;
                const float flex = -M.flex_damp * s.mass * (vn - dot(vmean, nrm));
                const vec3 fa = nrm * ((drag + flex) * (1.0f / 3.0f));
                for (int c = 0; c < 3; c++) f[s.n[c]] += fa;
                for (int e = 0; e < 3; e++) {
                    // clamped edge (both ends fixed, nothing beyond): a hinge against the rest plane, acting on the
                    // opposite corner (a pinned border would let torn flaps swing forever)
                    const uint32_t a = s.n[e], bb = s.n[nx(e)], c = s.n[pv(e)];
                    if (s.nb[e] >= 0 || !(info[a].flags & info[bb].flags & NF_FIXED) || (info[c].flags & NF_FIXED)) continue;
                    const vec3 ev = nd[bb].p - nd[a].p;
                    const float le = length(ev);
                    if (le < 1e-6f) continue;
                    const vec3 eh = ev / le;
                    const float h1 = std::sqrt(n2) / le;
                    const float th = std::atan2(dot(cross(s.n0, nrm), eh), dot(s.n0, nrm));
                    float dth = wrap_pi(th - s.th0[e]);
                    const float eq = bend_equivalent(s.level);
                    const float lim = std::min(M.bend_yield, M.bend_break * s.flaw);
                    if (deform && std::fabs(dth) * eq > lim) {
                        const float ex = dth - std::copysign(lim / eq, dth);
                        s.th0[e] = wrap_pi(s.th0[e] + ex);
                        dth -= ex;
                    }
                    const float kh = s.kb * (s.L0[e] * s.L0[e] / (2.0f * s.area0)) * clampf((kPi - std::fabs(th)) / 0.4f, 0.0f, 1.0f);
                    const float rate = dot(nd[c].v, nrm) / h1;
                    f[c] += nrm * (-kh * (dth + M.bend_damp * dt * rate) / h1);
                }
            }
        }
        if (s.cool) s.cool--;
        if (!brk || s.pending) continue;
        if (worst > 1.0f) {
            if (can_refine(s)) queue(si, 0, 0);
            else if (!s.cool) queue(si, 1, worst_e);
        }
        else if ((worst > M.refine || plastic > M.refine_yield) && can_refine(s)) queue(si, 0, 0);
    }
    }
    // bending hinges (each pair once, from the lower index)
    {
    PROFILE_ACCUM("Sheet hinges");
    for (int si = 0; si < nsh; si++) {
        Shell& s = shells[si];
        for (int e = 0; e < 3; e++) {
            const int j = s.nb[e];
            if (j <= si) continue;
            Shell& t = shells[j];
            const int hc = std::max(cls(s), cls(t));
            if (!due[hc]) continue;
            vec3* f = shell_acc[hc].data();
            const float dt = h * (float)(sub >> hc);
            const int fe = edge_of(t, s.n[e], s.n[nx(e)]);
            if (fe < 0) continue;
            const uint32_t id[4] = {s.n[pv(e)], t.n[pv(fe)], s.n[e], s.n[nx(e)]};
            Hinge hg;
            if (!hinge_geometry(nd[id[0]].p, nd[id[1]].p, nd[id[2]].p, nd[id[3]].p, hg)) continue;
            float dth = wrap_pi(hg.theta - s.th0[e]);
            const float eq = bend_equivalent(std::max(s.level, t.level));
            if (deform && std::fabs(dth) * eq > M.bend_yield) {
                const float ex = dth - std::copysign(M.bend_yield / eq, dth);
                s.th0[e] = t.th0[fe] = wrap_pi(s.th0[e] + ex);
                dth -= ex;
            }
            if (brk && !s.pending && !t.pending) {
                const float lim = M.bend_break * std::min(s.flaw, t.flaw) * ((float)std::min(s.hs[e], t.hs[fe]) * (1.0f / 64.0f));
                const bool refine_s = budget_left && s.level < M.max_level, refine_t = budget_left && t.level < M.max_level;
                if (std::fabs(dth) * eq > lim) {
                    // a brittle sheet folded too sharply: refine first, crack along this edge at the finest size
                    if (refine_s || refine_t) queue(refine_s && (!refine_t || s.level <= t.level) ? si : j, 0, 0);
                    else if (!s.cool) queue(si, 2, e);
                } else if (std::fabs(dth) * eq > M.refine_angle && (refine_s || refine_t)) {
                    queue(refine_s && (!refine_t || s.level <= t.level) ? si : j, 0, 0);
                }
            }
            // Stiffness from the rest shape (a constant: the force stays the gradient of an energy). Faded out for a
            // triangle squashed towards a line (its angle gradient ~ 1/height blows up) and near a full fold (the
            // angle wraps at +-pi): both used to pump energy into flapping, cracked edges.
            const float le0 = s.L0[e];
            float w = clampf((kPi - std::fabs(hg.theta)) / 0.4f, 0.0f, 1.0f);
            w *= sqr(clampf(std::min(hg.h1 * shell_detail::hinge_fade_c(le0, s.area0), hg.h2 * shell_detail::hinge_fade_c(le0, t.area0)), 0.0f, 1.0f));
            const float kh = std::min(s.kb, t.kb) * (le0 * le0 / (s.area0 + t.area0)) * w;
            if (kh <= 0) continue;
            float rate = 0;
            for (int k = 0; k < 4; k++) rate += dot(hg.g[k], nd[id[k]].v);
            const float fm = -kh * (dth + M.bend_damp * dt * rate);
            for (int k = 0; k < 4; k++) f[id[k]] += hg.g[k] * fm;
        }
    }
    }
    vec3* F = force.data();
    for (int c = 0; c <= maxc; c++) {
        const vec3* a = shell_acc[c].data();
        const size_t n = std::min(nn, shell_acc[c].size());
        for (size_t i = 0; i < n; i++) F[i] += a[i];
    }
}


} // namespace bl::phys::ref
