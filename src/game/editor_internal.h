// Helpers shared by the model editor's files (editor*.cpp).
#pragma once

#include "core/math.h"
#include "gfx/renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace bl::edit_detail {

inline void sorted_insert(std::vector<int>& v, int i) {
    auto it = std::lower_bound(v.begin(), v.end(), i);
    if (it == v.end() || *it != i) v.insert(it, i);
}
inline void sorted_toggle(std::vector<int>& v, int i) {
    auto it = std::lower_bound(v.begin(), v.end(), i);
    if (it != v.end() && *it == i) v.erase(it);
    else v.insert(it, i);
}

inline float seg_dist2(vec2 p, vec2 a, vec2 b, float* t_out = nullptr) {
    const vec2 ab = b - a;
    const float l2 = ab.x * ab.x + ab.y * ab.y;
    float t = l2 > 1e-9f ? ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / l2 : 0.0f;
    t = clampf(t, 0, 1);
    if (t_out) *t_out = t;
    const vec2 q = a + ab * t;
    return (p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y);
}

inline bool ray_tri(vec3 ro, vec3 rd, vec3 a, vec3 b, vec3 c, float& t) {
    const vec3 e1 = b - a, e2 = c - a, p = cross(rd, e2);
    const float det = dot(e1, p);
    if (std::fabs(det) < 1e-12f) return false;
    const float inv = 1.0f / det;
    const vec3 s = ro - a;
    const float u = dot(s, p) * inv;
    if (u < 0 || u > 1) return false;
    const vec3 q = cross(s, e1);
    const float v = dot(rd, q) * inv;
    if (v < 0 || u + v > 1) return false;
    t = dot(e2, q) * inv;
    return t > 0;
}

// the parameter along a line (o + d t) of the point nearest to a ray
inline float line_param_near_ray(vec3 o, vec3 d, vec3 ro, vec3 rd) {
    const vec3 w = o - ro;
    const float a = dot(d, d), b = dot(d, rd), c = dot(rd, rd), dd = dot(d, w), e = dot(rd, w);
    const float den = a * c - b * b;
    if (std::fabs(den) < 1e-12f) return 0;
    return (b * e - c * dd) / den;
}

inline bool ray_plane(vec3 ro, vec3 rd, vec3 n, float d, vec3& out) { // plane dot(n, p) = d
    const float den = dot(n, rd);
    if (std::fabs(den) < 1e-7f) return false;
    const float t = (d - dot(n, ro)) / den;
    if (t < 0) return false;
    out = ro + rd * t;
    return true;
}

inline vec3 mirror_z(vec3 p) { return vec3(p.x, p.y, -p.z); }

// two axes spanning the plane of an axis-aligned normal
inline void plane_axes(vec3 n, vec3& u, vec3& v) {
    if (std::fabs(n.y) > 0.9f) u = vec3(1, 0, 0), v = vec3(0, 0, 1);
    else if (std::fabs(n.z) > 0.9f) u = vec3(1, 0, 0), v = vec3(0, 1, 0);
    else u = vec3(0, 0, 1), v = vec3(0, 1, 0);
}

inline std::string fmt_len(float m) {
    char b[32];
    snprintf(b, sizeof b, "%.3f m", m);
    return b;
}
inline std::string fmt_vec(vec3 p) {
    char b[64];
    snprintf(b, sizeof b, "(%.3f, %.3f, %.3f)", p.x, p.y, p.z);
    return b;
}

inline uint32_t col(vec4 c) { return Renderer::rgba(c.x, c.y, c.z, c.w); }
inline uint32_t dim(uint32_t c, float f) {
    const float a = ((c >> 24) & 255) / 255.0f * f;
    return (c & 0x00ffffffu) | ((uint32_t)(a * 255) << 24);
}
inline const uint32_t kSelCol = Renderer::rgba(1, 0.6f, 0.15f, 1), kHoverCol = Renderer::rgba(1, 1, 1, 1);
inline const uint32_t kAxisCol[3] = {Renderer::rgba(1, 0.3f, 0.3f, 1), Renderer::rgba(0.35f, 1, 0.35f, 1), Renderer::rgba(0.35f, 0.55f, 1, 1)};

// thick lines and rings for the highlights: parallel lines offset in the view plane
struct ViewFrame {
    vec3 pos, fwd, right, up;
    float px_world;  // world size of one pixel at 1 m (perspective)
    float px_ortho;  // world size of one pixel (orthographic), 0 for perspective
    float px_at(vec3 p) const { return px_ortho > 0 ? px_ortho : std::max(0.1f, dot(p - pos, fwd)) * px_world; }
};
inline ViewFrame view_frame(const Camera& c, float view_h) {
    ViewFrame v;
    v.pos = c.pos;
    v.fwd = c.forward();
    v.right = normalize_or(cross(v.fwd, c.up), vec3(1, 0, 0));
    v.up = normalize_or(cross(v.right, v.fwd), vec3(0, 1, 0));
    v.px_world = 2.0f * std::tan(c.fov_deg * kDeg2Rad * 0.5f) / std::max(1.0f, view_h);
    v.px_ortho = c.ortho_half > 0 ? 2.0f * c.ortho_half / std::max(1.0f, view_h) : 0.0f;
    return v;
}
inline void thick_line(Renderer& r, const ViewFrame&, vec3 a, vec3 b, uint32_t c, float px) { r.thick_line(a, b, c, px); }
inline void ring(Renderer& r, const ViewFrame& v, vec3 p, uint32_t c, float px, int n = 14, float width = 1.5f) {
    const float rad = v.px_at(p) * px;
    for (int k = 0; k < n; k++) {
        const float t0 = 2 * kPi * k / n, t1 = 2 * kPi * (k + 1) / n;
        r.thick_line(p + (v.right * std::cos(t0) + v.up * std::sin(t0)) * rad, p + (v.right * std::cos(t1) + v.up * std::sin(t1)) * rad, c, width);
    }
}
inline void square(Renderer& r, const ViewFrame& v, vec3 p, uint32_t c, float px) {
    const float h = v.px_at(p) * px;
    const vec3 a = v.right * h, b = v.up * h;
    r.line(p - a - b, p + a - b, c), r.line(p + a - b, p + a + b, c), r.line(p + a + b, p - a + b, c), r.line(p - a + b, p - a - b, c);
}
inline void dashed(Renderer& r, vec3 a, vec3 b, uint32_t c, float dash, float width = 1.0f) {
    const float L = length(b - a);
    if (L < 1e-6f) return;
    const vec3 d = (b - a) / L;
    for (float t = 0; t < L; t += 2 * dash) r.thick_line(a + d * t, a + d * std::min(L, t + dash), c, width);
}

} // namespace bl::edit_detail
