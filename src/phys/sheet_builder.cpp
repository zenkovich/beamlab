#include "phys/sheet_builder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>

namespace bl::phys {

float add_sheet_mesh(SoftBody& b, const SheetMeshDesc& d) {
    const int nu = std::max(2, d.nu), nv = std::max(2, d.nv);
    const float hw = d.width * 0.5f, hh = d.height * 0.5f;
    auto gx = [&](int i) { return ((float)i / (nu - 1) - 0.5f) * d.width; };
    auto gy = [&](int j) { return ((float)j / (nv - 1) - 0.5f) * d.height; };
    const SheetShape shape = d.shape;
    const float hole = std::clamp(d.hole, 0.05f, 0.95f);
    auto ell = [&](float x, float y, float s) { return (x * x) / (hw * hw * s * s) + (y * y) / (hh * hh * s * s); };
    auto inside = [&](float x, float y) {
        switch (shape) {
        case SheetShape::Disc: return ell(x, y, 1.0f) <= 1.0f;
        case SheetShape::Ring: return ell(x, y, 1.0f) <= 1.0f && ell(x, y, hole) >= 1.0f;
        case SheetShape::Triangle: return y >= -hh && std::fabs(x) <= hw * (hh - y) / d.height;
        case SheetShape::LShape: return !(x > 1e-4f && y > 1e-4f);
        default: return true;
        }
    };
    // triangles of the "4-8" grid (the diagonals alternate: every vertex sees 4 or 8 triangles, the pattern bisection
    // produces), kept where their centroid is inside the shape
    auto id = [&](int i, int j) { return j * nu + i; };
    std::vector<std::array<int, 3>> tris;
    for (int j = 0; j + 1 < nv; j++)
        for (int i = 0; i + 1 < nu; i++) {
            const int a = id(i, j), bb = id(i + 1, j), c = id(i + 1, j + 1), e = id(i, j + 1);
            const std::array<int, 3> t2[2] = {((i + j) & 1) == 0 ? std::array<int, 3>{a, bb, c} : std::array<int, 3>{a, bb, e},
                                              ((i + j) & 1) == 0 ? std::array<int, 3>{a, c, e} : std::array<int, 3>{bb, c, e}};
            for (const auto& t : t2) {
                float cx = 0, cy = 0;
                for (int k : t) {
                    cx += gx(k % nu);
                    cy += gy(k / nu);
                }
                if (inside(cx / 3, cy / 3)) tris.push_back(t);
            }
        }
    // planar positions of the nodes in use
    std::vector<int> map(nu * nv, -1);
    std::vector<vec2> P;
    std::vector<int> gi; // grid index of each node
    for (auto& t : tris)
        for (int& k : t) {
            if (map[k] < 0) {
                map[k] = (int)P.size();
                P.push_back(vec2(gx(k % nu), gy(k / nu)));
                gi.push_back(k);
            }
            k = map[k];
        }
    const int nn = (int)P.size();
    // border: edges of one triangle only
    std::map<std::pair<int, int>, int> edges;
    for (auto& t : tris)
        for (int e = 0; e < 3; e++) {
            int a = t[e], c = t[(e + 1) % 3];
            edges[{std::min(a, c), std::max(a, c)}]++;
        }
    std::vector<uint8_t> border(nn, 0);
    for (auto& [k, n] : edges)
        if (n == 1) border[k.first] = border[k.second] = 1;
    auto outer = [&](int v) { // (a ring's hole border is not outer)
        if (shape != SheetShape::Ring) return border[v] != 0;
        return border[v] && ell(P[v].x, P[v].y, 1.0f) > (1.0f + hole) * (1.0f + hole) * 0.25f;
    };
    // curved outlines: the border nodes onto the outline, where the triangles around them keep their shape
    if (shape == SheetShape::Disc || shape == SheetShape::Ring || shape == SheetShape::Triangle) {
        std::vector<std::vector<int>> fan(nn);
        for (int t = 0; t < (int)tris.size(); t++)
            for (int k : tris[t]) fan[k].push_back(t);
        const float cell = 0.5f * (d.width / (nu - 1)) * (d.height / (nv - 1));
        auto area = [&](const std::array<int, 3>& t) {
            const vec2 a = P[t[0]], c = P[t[1]], e = P[t[2]];
            return 0.5f * ((c.x - a.x) * (e.y - a.y) - (c.y - a.y) * (e.x - a.x));
        };
        auto project = [&](int v) {
            const vec2 p = P[v];
            if (shape == SheetShape::Triangle) {
                const vec2 A(-hw, -hh), B(hw, -hh), C(0, hh);
                vec2 best = p;
                float bd = 1e30f;
                for (auto [s0, s1] : {std::make_pair(A, B), std::make_pair(B, C), std::make_pair(C, A)}) {
                    const vec2 e = s1 - s0;
                    const float t = std::clamp(((p.x - s0.x) * e.x + (p.y - s0.y) * e.y) / (e.x * e.x + e.y * e.y), 0.0f, 1.0f);
                    const vec2 q = s0 + e * t;
                    const float dd = (q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y);
                    if (dd < bd) {
                        bd = dd;
                        best = q;
                    }
                }
                return best;
            }
            const float s = (shape == SheetShape::Ring && !outer(v)) ? hole : 1.0f;
            const float r = std::sqrt(ell(p.x, p.y, s));
            return r > 1e-6f ? vec2(p.x / r, p.y / r) : p;
        };
        for (int v = 0; v < nn; v++) {
            if (!border[v]) continue;
            const vec2 old = P[v];
            P[v] = project(v);
            bool ok = true;
            for (int t : fan[v]) ok &= area(tris[t]) > 0.35f * cell;
            if (!ok) P[v] = old;
        }
    }
    // fixed nodes
    std::vector<uint8_t> fixed(nn, 0);
    if (shape == SheetShape::Rect) {
        for (int v = 0; v < nn; v++) {
            const int i = gi[v] % nu, j = gi[v] / nu;
            const bool side = i == 0 || i == nu - 1, top = j == nv - 1, bottom = j == 0;
            fixed[v] = d.clamp == 1 ? (side || top || bottom) : d.clamp == 2 ? (side || top) : d.clamp == 3 ? top : d.clamp == 4 ? (top && side) : false;
        }
    } else if (d.clamp == 4) {
        int l = -1, r = -1;
        for (int v = 0; v < nn; v++) {
            if (!outer(v)) continue;
            if (l < 0 || P[v].y - P[v].x > P[l].y - P[l].x) l = v;
            if (r < 0 || P[v].y + P[v].x > P[r].y + P[r].x) r = v;
        }
        if (l >= 0) fixed[l] = 1;
        if (r >= 0) fixed[r] = 1;
    } else if (d.clamp > 0) {
        for (int v = 0; v < nn; v++) {
            if (!outer(v)) continue;
            const float y = P[v].y / hh;
            fixed[v] = d.clamp == 1 || (d.clamp == 2 && y > -0.5f) || (d.clamp == 3 && y > 0.5f);
        }
    }
    // onto the surface
    const vec3 U = normalize(d.u), V = normalize(d.v), N = normalize(cross(U, V));
    auto place = [&](vec2 q) {
        if (d.curve > 0) {
            const float phi = q.x / d.curve;
            return d.center + U * (d.curve * std::sin(phi)) + V * q.y + N * (d.curve * (1.0f - std::cos(phi)));
        }
        if (d.dome > 0) {
            const float r = std::sqrt(q.x * q.x + q.y * q.y);
            if (r < 1e-6f) return d.center;
            const float psi = r / d.dome;
            return d.center + (U * q.x + V * q.y) * (d.dome * std::sin(psi) / r) - N * (d.dome * (1.0f - std::cos(psi)));
        }
        return d.center + U * q.x + V * q.y;
    };
    const uint32_t base = (uint32_t)b.nodes.size();
    for (int v = 0; v < nn; v++) b.add_node(place(P[v]), 1.0f, NF_GROUND | NF_CONTACTER | (fixed[v] ? NF_FIXED : 0));
    auto uv = [&](int v) { return vec2(P[v].x / d.width + 0.5f, P[v].y / d.height + 0.5f) * d.uv_scale; };
    float area = 0;
    for (const auto& t : tris) {
        b.add_shell(base + t[0], base + t[1], base + t[2], uv(t[0]), uv(t[1]), uv(t[2]));
        const vec3 p0 = b.nodes[base + t[0]].p, p1 = b.nodes[base + t[1]].p, p2 = b.nodes[base + t[2]].p;
        area += 0.5f * length(cross(p1 - p0, p2 - p0));
    }
    return area;
}

} // namespace bl::phys
