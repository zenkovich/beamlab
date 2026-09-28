// Helpers shared by the triangle-element code: topology (shell.cpp) and the force kernel (shell_kernel.cpp).
#pragma once

#include "phys/softbody.h"

#include <algorithm>
#include <cmath>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace bl::phys::shell_detail {

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
// Rate class of a triangle: evaluated every sub >> class short steps (see SoftBody::dt_shift).
inline int rate_class(int level) { return std::min(2, (level + 1) / 2); }

// Stiffness of an edge from its fracture pattern code (Shell::es): the edges across a pattern line a little softer,
// the ones holding the pieces between the lines a little stiffer, so the strain gathers on the lines (within the
// explicit stability budget: at most 1.15 of the capped stiffness).
inline float pattern_stiffness(uint8_t es) { return std::clamp(1.0f + 0.15f * ((float)es * (1.0f / 64.0f) - 1.0f), 0.9f, 1.15f); }

// Shape of a triangle: 4 sqrt3 area / sum of the squared edges (1 equilateral, 0.87 right isosceles, 0 a line). The
// fracture patterns move nodes off the 4-8 grid only as far as every triangle keeps at least kShapeMin: a thin
// triangle's hinges are stiff (edge^2 / area, the angle gradients ~ 1 / height), the explicit step would not hold them.
constexpr float kShapeMin = 0.72f;
// A hinge's stiffness is faded out for a triangle squashed towards a line during the step: its angle gradient grows
// as 1 / height, the force as 1 / height^2, and the stability budget was met at the rest shape. Below kHingeFade of
// the rest height (2 / kHingeFadeC) the fade is the squared height ratio, so the force stays at the rest value: a
// linear fade left it growing as 1 / height and flung the light nodes of a crushed car body.
constexpr float kHingeFadeC = 1.34f; // the fade: clamp(2 (h / h0) / kHingeFadeC, 0, 1)^2, i.e. from h0 * 0.67 down
inline float hinge_fade_c(float le0, float area0) { return le0 / (kHingeFadeC * area0); }
inline float tri_quality(float area, float l0, float l1, float l2) { return 6.9282032f * area / std::max(1e-20f, l0 * l0 + l1 * l1 + l2 * l2); }

// Springs of a shell from the material's (stability clamped) stiffness: longer edges of the shape are softer
// (k * Lmin / L), the damping follows the size (the substep shrinks with it), the hinges soften with the level
// (bending waves need a step ~ size^2, the island only halves it per two levels).
inline void shell_springs(Shell& s, const ShellMaterial& m) {
    const float lmin = std::min(s.L0[0], std::min(s.L0[1], s.L0[2]));
    const float ds = std::pow(0.70710678f, (float)s.level);
    for (int e = 0; e < 3; e++) {
        const float r = lmin / std::max(1e-6f, s.L0[e]) * pattern_stiffness(s.es[e]) * s.kscale; // (d / k the same on every edge)
        s.k[e] = m.k * r;
        s.d[e] = m.damp * r * ds;
    }
    s.kb = m.bend * std::pow(0.25f, (float)(s.level / 2)) * s.kscale;
}

// ---------------------------------------------------------------------------------------------- fast math
// 1 / sqrt(x) to ~1 ulp: the hardware estimate (8 bits) and two Newton steps. No division, no square root unit.
inline float rsqrt(float x) {
#if defined(__aarch64__) && !defined(BL_NO_RSQRT)
    float e = vrsqrtes_f32(x);
    e *= vrsqrtss_f32(x * e, e);
    e *= vrsqrtss_f32(x * e, e);
    return e;
#else
    return 1.0f / std::sqrt(x);
#endif
}

// atan by linear interpolation in a table of 256 intervals on [0, 1] (error < 1.3e-6 rad; the table is 2 KB and
// stays in L1). Each entry holds the value and the step to the next one: one 8-byte load per call.
struct AtanTable {
    float v[257][2];
    AtanTable() {
        for (int i = 0; i <= 256; i++) {
            const double a = std::atan(i / 256.0), b = std::atan((i + 1) / 256.0);
            v[i][0] = (float)a;
            v[i][1] = (float)(b - a);
        }
    }
};
inline const AtanTable& atan_table() {
    static const AtanTable t;
    return t;
}
inline float lut_atan2(float y, float x) {
    const float ax = std::fabs(x), ay = std::fabs(y);
    const float mx = std::max(ax, ay), mn = std::min(ax, ay);
    const float a = mn / std::max(mx, 1e-30f);
    const float q = a * 256.0f;
    const int i = (int)q;
    const float* e = atan_table().v[i];
    float r = e[0] + (q - (float)i) * e[1];
    if (ay > ax) r = 1.57079637f - r;
    if (x < 0) r = 3.14159274f - r;
    return y < 0 ? -r : r;
}
// atan2 to ~1e-5 rad (polynomial; set-up code that must not depend on the table's initialisation order)
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
    h.theta = fast_atan2(dot(cross(n1, n2), e) / le, dot(n1, n2));
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

} // namespace bl::phys::shell_detail
