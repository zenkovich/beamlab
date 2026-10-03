#include "world/objects.h"
#include "phys/fem_shell.h"
#include "phys/sheet_builder.h"
#include "core/profiler.h"
#include "core/util.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#include <unordered_map>

namespace bl {

using namespace phys;

// ====================================================================================== visuals
void SurfaceVisual::update(const SoftBody& b, bool first) {
    const int ntri = (int)corner_node.size() / 3;
    auto corner = [&](int t, int k) { return b.nodes[corner_node[t * 3 + k]].p; };
    if (first || (int)rest_edge2.size() != ntri) {
        rest_edge2.resize(ntri);
        torn.assign(ntri, 0);
        for (int t = 0; t < ntri; t++) rest_edge2[t] = max_edge2(corner(t, 0), corner(t, 1), corner(t, 2));
        // map to the body's collision triangles (same node triplet), so tearing them hides the face too
        auto key = [](uint32_t a, uint32_t b, uint32_t c) {
            if (a > b) std::swap(a, b);
            if (b > c) std::swap(b, c);
            if (a > b) std::swap(a, b);
            return (uint64_t)a | ((uint64_t)b << 21) | ((uint64_t)c << 42); // exact below 2M nodes
        };
        std::unordered_map<uint64_t, int32_t> lookup;
        for (int32_t i = 0; i < (int32_t)b.tris.size(); i++) lookup[key(b.tris[i].a, b.tris[i].b, b.tris[i].c)] = i;
        body_tri.assign(ntri, -1);
        for (int t = 0; t < ntri && !lookup.empty(); t++) {
            auto it = lookup.find(key(corner_node[t * 3], corner_node[t * 3 + 1], corner_node[t * 3 + 2]));
            if (it != lookup.end()) body_tri[t] = it->second;
        }
    }
    int newly_torn = 0;
    for (int t = 0; t < ntri; t++)
        if (!torn[t] && ((body_tri[t] >= 0 && b.tris[body_tri[t]].torn) ||
                         (rest_edge2[t] > 0 && max_edge2(corner(t, 0), corner(t, 1), corner(t, 2)) > kTornStretch2 * rest_edge2[t]))) {
            torn[t] = 1;
            newly_torn++;
        }
    if (smooth) {
        if (first) {
            // one vertex per (node, uv) pair: texture seams (a wrapped cylinder) get their own vertices, while
            // normals are still accumulated per node (normal_group) so the shading stays smooth across the seam
            std::map<std::tuple<uint32_t, float, float>, uint32_t> remap;
            std::map<uint32_t, uint32_t> node_group;
            corner_vertex.resize(corner_node.size());
            unique_nodes.clear();
            normal_group.clear();
            for (size_t i = 0; i < corner_node.size(); i++) {
                auto key = std::make_tuple(corner_node[i], corner_uv[i].x, corner_uv[i].y);
                auto it = remap.find(key);
                if (it == remap.end()) {
                    uint32_t vi = (uint32_t)unique_nodes.size();
                    unique_nodes.push_back(corner_node[i]);
                    auto g = node_group.emplace(corner_node[i], vi).first;
                    normal_group.push_back(g->second);
                    remap[key] = vi;
                    corner_vertex[i] = vi;
                } else {
                    corner_vertex[i] = it->second;
                }
            }
            verts.resize(unique_nodes.size());
            for (size_t i = 0; i < corner_node.size(); i++) verts[corner_vertex[i]].uv = corner_uv[i];
            idx.assign(corner_vertex.begin(), corner_vertex.end());
        }
        if (newly_torn) {
            for (int t = 0; t < ntri; t++)
                if (torn[t]) idx[t * 3 + 1] = idx[t * 3 + 2] = idx[t * 3]; // degenerate
            indices_dirty = true;
        }
        for (size_t v = 0; v < unique_nodes.size(); v++) {
            verts[v].pos = b.nodes[unique_nodes[v]].p;
            verts[v].normal = vec3(0);
        }
        for (int t = 0; t < ntri; t++) {
            if (torn[t]) continue;
            const uint32_t* tri = &corner_vertex[(size_t)t * 3];
            vec3 n = cross(verts[tri[1]].pos - verts[tri[0]].pos, verts[tri[2]].pos - verts[tri[0]].pos);
            for (int k = 0; k < 3; k++) verts[normal_group[tri[k]]].normal += n;
        }
        for (size_t v = 0; v < verts.size(); v++)
            if (normal_group[v] == v) verts[v].normal = normalize_or(verts[v].normal, vec3(0, 1, 0));
        for (size_t v = 0; v < verts.size(); v++)
            if (normal_group[v] != v) verts[v].normal = verts[normal_group[v]].normal;
    } else {
        if (first) {
            verts.resize(corner_node.size());
            idx.resize(corner_node.size());
            for (size_t i = 0; i < idx.size(); i++) idx[i] = (uint32_t)i;
        }
        for (int t = 0; t < ntri; t++) {
            vec3 p0 = b.nodes[corner_node[t * 3]].p, p1 = b.nodes[corner_node[t * 3 + 1]].p, p2 = b.nodes[corner_node[t * 3 + 2]].p;
            if (torn[t]) p1 = p2 = p0; // degenerate
            vec3 n = normalize_or(cross(p1 - p0, p2 - p0), vec3(0, 1, 0));
            const vec3 ps[3] = {p0, p1, p2};
            for (int k = 0; k < 3; k++) {
                Vertex& v = verts[t * 3 + k];
                v.pos = ps[k];
                v.normal = n;
                v.uv = corner_uv[t * 3 + k];
            }
        }
    }
}

void SurfaceVisual::upload(bool first) {
    if (first || !mesh.valid()) mesh.create(verts, idx, true);
    else {
        mesh.update_vertices(verts.data(), (int)verts.size());
        if (indices_dirty) mesh.update_indices(idx.data(), (int)idx.size());
    }
    indices_dirty = false;
}

void ShellVisual::update(const SoftBody& b, bool first) {
    PROFILE_ACCUM("Sheet mesh");
    const size_t ns = b.shells.size();
    const bool thick = thickness > 0;
    if (first || topo != b.topo_version) {
        PROFILE_ACCUM("Sheet mesh rebuild"); // (new layout after refinement / cracks)
        walls.clear();
        if (thick)
            for (uint32_t si = 0; si < ns; si++)
                for (uint8_t e = 0; e < 3; e++)
                    if (b.shells[si].nb[e] < 0) walls.push_back({si, e});
        const size_t faces = ns * (thick ? 2 : 1);
        verts.resize(faces * 3 + walls.size() * 4);
        idx.clear();
        idx.reserve(faces * 3 + walls.size() * 6);
        // grouped by material (a draw each): the front faces, the back faces, the walls of the free edges
        int nm = 1;
        for (uint32_t si = 0; si < ns; si++) nm = std::max(nm, (int)b.shells[si].mat + 1);
        ranges.assign(nm, {0, 0});
        const uint32_t back = (uint32_t)ns * 3, w0 = (uint32_t)faces * 3;
        for (int m = 0; m < nm; m++) {
            const int first = (int)idx.size();
            for (uint32_t si = 0; si < ns; si++)
                if (b.shells[si].mat == m || (nm == 1)) idx.insert(idx.end(), {si * 3, si * 3 + 1, si * 3 + 2});
            if (thick) {
                for (uint32_t si = 0; si < ns; si++)
                    if (b.shells[si].mat == m || (nm == 1)) idx.insert(idx.end(), {back + si * 3, back + si * 3 + 2, back + si * 3 + 1});
                for (uint32_t k = 0; k < walls.size(); k++) {
                    if (b.shells[walls[k].shell].mat != m && nm != 1) continue;
                    const uint32_t q = w0 + k * 4;
                    idx.insert(idx.end(), {q, q + 1, q + 2, q, q + 2, q + 3});
                }
            }
            ranges[m] = {first, (int)idx.size() - first};
        }
        topo = b.topo_version;
        rebuilt = true;
    }
    // smooth normals per node: the copies of a split node get their own, so cracks stay sharp
    node_normal.assign(b.nodes.size(), vec3(0));
    for (const Shell& s : b.shells) {
        const vec3 p0 = b.nodes[s.n[0]].p;
        const vec3 n = cross(b.nodes[s.n[1]].p - p0, b.nodes[s.n[2]].p - p0);
        for (int c = 0; c < 3; c++) node_normal[s.n[c]] += n;
    }
    for (const Shell& s : b.shells)
        for (int c = 0; c < 3; c++) {
            vec3& n = node_normal[s.n[c]];
            if (length2(n) != 1.0f) n = normalize_or(n, vec3(0, 1, 0));
        }
    // the shading normals: split at creases (a corner's normal from the faces round its node that turn less than the
    // crease angle from its own), the thickness still along the node's smooth one (the surface stays closed)
    const bool creased = crease_cos >= -1.0f && b.node_shells.size() == b.nodes.size();
    if (creased) {
        face_normal.resize(ns);
        for (size_t si = 0; si < ns; si++) {
            const Shell& s = b.shells[si];
            const vec3 p0 = b.nodes[s.n[0]].p;
            face_normal[si] = cross(b.nodes[s.n[1]].p - p0, b.nodes[s.n[2]].p - p0); // (area-weighted)
        }
    }
    const float h = thickness * 0.5f;
    const size_t back = ns * 3;
    for (size_t si = 0; si < ns; si++) {
        const Shell& s = b.shells[si];
        const vec3 own = creased ? normalize_or(face_normal[si], vec3(0, 1, 0)) : vec3(0);
        for (int c = 0; c < 3; c++) {
            const vec3 p = b.nodes[s.n[c]].p, n = node_normal[s.n[c]];
            vec3 sn = n;
            if (creased) {
                vec3 acc(0);
                for (uint32_t sj : b.node_shells[s.n[c]])
                    if (sj < ns && dot(normalize_or(face_normal[sj], vec3(0)), own) >= crease_cos) acc += face_normal[sj];
                sn = normalize_or(acc, n);
            }
            Vertex& f = verts[si * 3 + c];
            f.pos = p + n * h;
            f.normal = sn;
            f.uv = s.uv[c];
            if (thick) {
                Vertex& k = verts[back + si * 3 + c];
                k.pos = p - n * h;
                k.normal = -sn;
                k.uv = s.uv[c];
            }
        }
    }
    const size_t w0 = ns * (thick ? 6 : 3);
    for (size_t k = 0; k < walls.size(); k++) {
        const Shell& s = b.shells[walls[k].shell];
        const int e = walls[k].edge, e1 = e == 2 ? 0 : e + 1, e2 = e == 0 ? 2 : e - 1;
        const uint32_t a = s.n[e], c = s.n[e1];
        const vec3 pa = b.nodes[a].p, pc = b.nodes[c].p;
        const vec3 fn = cross(pc - pa, b.nodes[s.n[e2]].p - pa);
        const vec3 out = normalize_or(cross(pc - pa, fn), vec3(0, 1, 0));
        const vec3 na = node_normal[a] * h, nc = node_normal[c] * h;
        Vertex* q = &verts[w0 + k * 4];
        q[0] = {pa + na, out, s.uv[e]};
        q[1] = {pa - na, out, s.uv[e] + vec2(0, thickness)};
        q[2] = {pc - nc, out, s.uv[e1] + vec2(0, thickness)};
        q[3] = {pc + nc, out, s.uv[e1]};
    }
    update_fx(b, rebuilt || first);
}

namespace {
inline uint32_t fx_hash(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}
inline float fx_r01(uint32_t h) { return (float)(fx_hash(h) & 0xffffff) * (1.0f / 16777216.0f); }
} // namespace

void ShellVisual::update_fx(const SoftBody& b, bool rebuild) {
    PROFILE_ACCUM("Sheet fx");
    const ShellPattern style = b.shell_mat.pattern;
    fx_style = (int)style;
    const bool glass = style == ShellPattern::Radial, metal = style == ShellPattern::Punch, wood = style == ShellPattern::Grain;
    const size_t ns = b.shells.size();
    // (the size of an impact's mark: crushed glass a few cm, the scuff round a metal plug, a bruise on wood)
    auto mark_radius = [&](const ShellImpact& im) { return glass ? std::clamp(0.35f * im.r, 0.04f, 0.15f) : metal ? im.r * 1.35f : im.r * 0.5f; };
    static const int kVerts[FX_COUNT] = {8, 8, 8, 3, 6};
    if (rebuild) {
        fx_items.clear();
        for (uint32_t si = 0; si < ns; si++) {
            const Shell& s = b.shells[si];
            for (uint8_t e = 0; e < 3; e++) {
                const int j = s.nb[e];
                // (from where the edge is on the material: the same splinters after every rebuild)
                const vec2 ua = s.uv[e], ub = s.uv[e == 2 ? 0 : e + 1];
                const uint32_t seed = fx_hash((uint32_t)std::lround((ua.x + ub.x) * 8192.0f) * 2654435761u ^ (uint32_t)std::lround((ua.y + ub.y) * 8192.0f));
                if (j >= 0) continue;
                if ((s.edges >> (3 + e)) & 1u) {
                    fx_items.push_back({si, e, FX_CHAR, 0, seed});
                    continue;
                }
                if ((s.edges >> e) & 1u) continue; // (the authored border)
                fx_items.push_back({si, e, FX_RIM, 0, seed});
                if ((wood || metal) && !b.is_piece) { // (a piece: its rim only; a mesh per kind costs on every new piece)
                    const int n = wood ? 1 + (int)(fx_r01(seed) * 3.0f) : (fx_r01(seed) < 0.6f ? 1 : 0);
                    for (int k = 0; k < n; k++) fx_items.push_back({si, e, FX_SLIVER, 0, fx_hash(seed + (uint32_t)k * 7919u)});
                }
            }
        }
        // the pattern's lines on the material: glass shows its web of cracks, metal the crease of the ring. Points
        // every centimetre, each in its triangle (barycentric in the material plane: it follows the sheet); a line
        // over a gap (the pieces moved apart, a hole) is not drawn there.
        web.clear();
        if ((glass || metal) && !b.is_piece && !b.shell_impacts.empty() && b.shell_uvm.x > 0 && ns > 0) {
            vec2 mn(1e30f), mx(-1e30f);
            for (uint32_t si = 0; si < ns; si++)
                for (int c = 0; c < 3; c++) {
                    const vec2 x = b.shell_x(si, c);
                    mn = vec2(std::min(mn.x, x.x), std::min(mn.y, x.y));
                    mx = vec2(std::max(mx.x, x.x), std::max(mx.y, x.y));
                }
            const int gn = std::clamp((int)std::sqrt((float)ns * 0.5f), 1, 128);
            const vec2 ext = vec2(std::max(1e-6f, mx.x - mn.x), std::max(1e-6f, mx.y - mn.y));
            std::vector<std::vector<uint32_t>> grid((size_t)gn * gn);
            auto cell = [&](vec2 x) {
                return vec2(std::clamp((x.x - mn.x) / ext.x * gn, 0.0f, gn - 0.001f), std::clamp((x.y - mn.y) / ext.y * gn, 0.0f, gn - 0.001f));
            };
            for (uint32_t si = 0; si < ns; si++) {
                vec2 lo(1e30f), hi(-1e30f);
                for (int c = 0; c < 3; c++) {
                    const vec2 x = cell(b.shell_x(si, c));
                    lo = vec2(std::min(lo.x, x.x), std::min(lo.y, x.y));
                    hi = vec2(std::max(hi.x, x.x), std::max(hi.y, x.y));
                }
                for (int y = (int)lo.y; y <= (int)hi.y; y++)
                    for (int x = (int)lo.x; x <= (int)hi.x; x++) grid[(size_t)y * gn + x].push_back(si);
            }
            auto inside = [&](uint32_t si, vec2 x, float& u, float& v) {
                const vec2 a = b.shell_x(si, 0), e1 = b.shell_x(si, 1) - a, e2 = b.shell_x(si, 2) - a, d = x - a;
                const float det = e1.x * e2.y - e1.y * e2.x;
                if (std::fabs(det) < 1e-14f) return false;
                u = (d.x * e2.y - d.y * e2.x) / det;
                v = (e1.x * d.y - e1.y * d.x) / det;
                return u >= -1e-4f && v >= -1e-4f && u + v <= 1.0001f;
            };
            const float ds = 0.01f;
            int32_t hint = -1;
            for (const ShellImpact& im : b.shell_impacts) {
                uint32_t prev_line = 0xffffffffu;
                vec2 prev_b(1e30f);
                for (const auto& sg : im.segs) {
                    if (metal && sg.line != 0) continue; // (metal: the ring only; its tears are cracks or nothing)
                    const bool cont = sg.line == prev_line && length(sg.a - prev_b) < 1e-5f;
                    if (!cont) web.push_back({-2, 0, 0}); // (a new polyline)
                    const int n = std::max(1, (int)std::ceil(length(sg.b - sg.a) / ds));
                    for (int k = cont ? 1 : 0; k <= n; k++) {
                        const vec2 x = sg.a + (sg.b - sg.a) * ((float)k / (float)n);
                        WebPt w{-1, 0, 0};
                        if (hint >= 0 && inside((uint32_t)hint, x, w.u, w.v)) w.shell = hint;
                        else {
                            const vec2 c = cell(x);
                            for (uint32_t si : grid[(size_t)(int)c.y * gn + (int)c.x])
                                if (inside(si, x, w.u, w.v)) {
                                    w.shell = (int32_t)si;
                                    break;
                                }
                        }
                        hint = w.shell;
                        web.push_back(w);
                    }
                    prev_line = sg.line;
                    prev_b = sg.b;
                }
            }
            for (size_t k = 0; k + 1 < web.size(); k++)
                if (web[k].shell >= 0 && web[k + 1].shell >= 0) fx_items.push_back({(uint32_t)k, 0, FX_LINE, 0, 0});
        }
        if (style != ShellPattern::None && b.shell_uvm.x > 0 && !b.is_piece)
            for (size_t i = 0; i < b.shell_impacts.size(); i++) {
                const ShellImpact& im = b.shell_impacts[i];
                const float R = mark_radius(im);
                for (uint32_t si = 0; si < ns; si++) {
                    const Shell& s = b.shells[si];
                    const vec2 cx = (b.shell_x(si, 0) + b.shell_x(si, 1) + b.shell_x(si, 2)) * (1.0f / 3.0f);
                    const float lmax = std::max(s.L0[0], std::max(s.L0[1], s.L0[2]));
                    if (length(cx - im.c) < R + lmax) fx_items.push_back({si, 0, FX_MARK, (uint16_t)i, 0});
                }
            }
        for (int k = 0; k < FX_COUNT; k++) {
            fx[k].v.clear();
            fx[k].i.clear();
            fx[k].rebuilt = true;
        }
        for (const FxItem& it : fx_items) {
            Fx& f = fx[it.kind];
            const uint32_t base = (uint32_t)f.i.size() / (it.kind == FX_SLIVER ? 3 : it.kind == FX_MARK ? 6 : 12) * (uint32_t)kVerts[it.kind];
            if (it.kind == FX_SLIVER) f.i.insert(f.i.end(), {base, base + 1, base + 2});
            else if (it.kind == FX_MARK) f.i.insert(f.i.end(), {base, base + 1, base + 2, base + 3, base + 5, base + 4});
            else f.i.insert(f.i.end(), {base, base + 1, base + 2, base, base + 2, base + 3, base + 4, base + 6, base + 5, base + 4, base + 7, base + 6});
        }
        for (int k = 0; k < FX_COUNT; k++) fx[k].v.resize(fx[k].i.size() / (k == FX_SLIVER ? 3 : k == FX_MARK ? 6 : 12) * kVerts[k]);
    }
    // positions: on the faces (half the thickness out, a little more against z-fighting), following the nodes
    const float h = thickness * 0.5f, lift = 0.0006f + thickness * 0.05f;
    const float w_line = 0.003f, w_rim = glass ? 0.003f : 0.004f, w_char = 0.006f;
    size_t cursor[FX_COUNT] = {0, 0, 0, 0, 0};
    auto web_point = [&](const WebPt& w, vec3& n) {
        const Shell& s = b.shells[(uint32_t)w.shell];
        const vec3 p0 = b.nodes[s.n[0]].p, p1 = b.nodes[s.n[1]].p, p2 = b.nodes[s.n[2]].p;
        n = normalize_or(cross(p1 - p0, p2 - p0), vec3(0, 1, 0));
        return p0 + (p1 - p0) * w.u + (p2 - p0) * w.v;
    };
    for (const FxItem& it : fx_items) {
        Vertex* q = &fx[it.kind].v[cursor[it.kind]];
        cursor[it.kind] += kVerts[it.kind];
        if (it.kind == FX_LINE) {
            // a piece of a pattern line: a ribbon on both faces, gone where the two points have come apart
            const WebPt &w0 = web[it.shell], &w1 = web[it.shell + 1];
            if ((uint32_t)w0.shell >= ns || (uint32_t)w1.shell >= ns) {
                for (int k = 0; k < 8; k++) q[k] = {vec3(0), vec3(0, 1, 0), vec2(0)};
                continue;
            }
            vec3 n0, n1;
            const vec3 a = web_point(w0, n0), c = web_point(w1, n1);
            const vec3 nn = normalize_or(n0 + n1, n0);
            if (length(c - a) > 0.03f) {
                for (int k = 0; k < 8; k++) q[k] = {a, nn, vec2(0)};
                continue;
            }
            const vec3 side = normalize_or(cross(nn, c - a), vec3(0)) * (0.5f * w_line);
            for (int f = 0; f < 2; f++) {
                const vec3 o = nn * ((f == 0 ? 1.0f : -1.0f) * (h + lift));
                Vertex* r = q + f * 4;
                r[0] = {a + o - side, nn, vec2(0, 0)};
                r[1] = {c + o - side, nn, vec2(1, 0)};
                r[2] = {c + o + side, nn, vec2(1, 1)};
                r[3] = {a + o + side, nn, vec2(0, 1)};
            }
            continue;
        }
        if (it.shell >= ns) continue;
        const Shell& s = b.shells[it.shell];
        const int e = it.edge, e1 = e == 2 ? 0 : e + 1, e2 = e == 0 ? 2 : e - 1;
        const vec3 pa = b.nodes[s.n[e]].p, pb = b.nodes[s.n[e1]].p, pc = b.nodes[s.n[e2]].p;
        const vec3 na = node_normal[s.n[e]], nb = node_normal[s.n[e1]];
        const vec3 fn = normalize_or(cross(pb - pa, pc - pa), na);
        if (it.kind == FX_MARK) {
            const ShellImpact& im = b.shell_impacts[it.impact];
            const float R = mark_radius(im);
            const vec3 p[3] = {b.nodes[s.n[0]].p, b.nodes[s.n[1]].p, b.nodes[s.n[2]].p};
            const vec3 n[3] = {node_normal[s.n[0]], node_normal[s.n[1]], node_normal[s.n[2]]};
            for (int c = 0; c < 3; c++) {
                const vec2 uv = vec2(0.5f) + (b.shell_x(it.shell, c) - im.c) * (0.5f / R);
                q[c] = {p[c] + n[c] * (h + lift * 1.5f), n[c], uv};
                q[3 + c] = {p[c] - n[c] * (h + lift * 1.5f), -n[c], uv};
            }
            continue;
        }
        const vec3 along = pb - pa;
        vec3 in = normalize_or(cross(fn, along), vec3(0));
        if (dot(in, pc - pa) < 0) in = -in;
        if (it.kind == FX_SLIVER) {
            const float len = length(along);
            if (len < 1e-6f) {
                q[0] = q[1] = q[2] = {pa, fn, vec2(0)};
                continue;
            }
            const uint32_t r = it.seed;
            const float u = 0.12f + 0.76f * fx_r01(r), bw = (wood ? 0.0015f + 0.003f * fx_r01(r ^ 1u) : 0.001f + 0.0015f * fx_r01(r ^ 1u)) / len;
            const float side = (2.0f * fx_r01(r ^ 2u) - 1.0f) * 0.8f;
            vec3 dir = -in;
            if (wood) {
                // along the fibres (the triangle's map from the material plane), pointing out of the crack
                const vec2 g(std::cos(b.shell_mat.grain_angle), std::sin(b.shell_mat.grain_angle));
                const vec2 a2 = b.shell_x(it.shell, 1) - b.shell_x(it.shell, 0), c2 = b.shell_x(it.shell, 2) - b.shell_x(it.shell, 0);
                const float det = a2.x * c2.y - a2.y * c2.x;
                if (std::fabs(det) > 1e-14f) {
                    const float gu = (g.x * c2.y - g.y * c2.x) / det, gv = (a2.x * g.y - a2.y * g.x) / det;
                    vec3 g3 = normalize_or((b.nodes[s.n[1]].p - b.nodes[s.n[0]].p) * gu + (b.nodes[s.n[2]].p - b.nodes[s.n[0]].p) * gv, dir);
                    if (dot(g3, dir) < 0) g3 = -g3;
                    dir = normalize_or(dir * 0.6f + g3 * 1.4f, dir);
                }
            }
            const float L = wood ? 0.006f + 0.03f * fx_r01(r ^ 3u) * fx_r01(r ^ 4u) : 0.003f + 0.007f * fx_r01(r ^ 3u);
            const float up = (fx_r01(r ^ 5u) - 0.5f) * thickness * (wood ? 1.2f : 3.0f);
            const vec3 n0 = normalize_or(na + nb, fn);
            const vec3 b0 = pa + along * std::max(0.0f, u - bw) + n0 * (h * side), b1 = pa + along * std::min(1.0f, u + bw) + n0 * (h * side);
            const vec3 tip = (b0 + b1) * 0.5f + dir * L + n0 * up;
            q[0] = {b0, n0, vec2(0, 0)};
            q[1] = {b1, n0, vec2(1, 0)};
            q[2] = {tip, n0, vec2(0.5f, 1)};
            continue;
        }
        // ribbons on both faces: a line centred on the edge, or a rim / scorch from the edge inwards
        const float w = it.kind == FX_LINE ? w_line : it.kind == FX_RIM ? w_rim : w_char;
        const vec3 o0 = it.kind == FX_LINE ? in * (-0.5f * w) : vec3(0), o1 = o0 + in * w;
        for (int f = 0; f < 2; f++) {
            const float sg = f == 0 ? 1.0f : -1.0f;
            const vec3 da = na * (sg * (h + lift)), db = nb * (sg * (h + lift));
            const vec3 nn = fn * sg;
            Vertex* r = q + f * 4;
            r[0] = {pa + da + o0, nn, vec2(0, 0)};
            r[1] = {pb + db + o0, nn, vec2(1, 0)};
            r[2] = {pb + db + o1, nn, vec2(1, 1)};
            r[3] = {pa + da + o1, nn, vec2(0, 1)};
        }
    }
}

void ShellVisual::upload(bool first) {
    PROFILE_ACCUM("Sheet upload");
    // (a rebuilt layout goes into the same buffers, orphaned: creating new ones every time a crack opened made the
    // driver wait for the frame still drawing the old ones)
    if (first || !mesh.valid()) mesh.create(verts, idx, true);
    else {
        mesh.update_vertices(verts.data(), (int)verts.size());
        if (rebuilt) mesh.update_indices(idx.data(), (int)idx.size());
    }
    rebuilt = false;
    for (int k = 0; k < FX_COUNT; k++) {
        Fx& f = fx[k];
        if (f.v.empty()) {
            f.rebuilt = false;
            continue;
        }
        if (!f.mat) f.mat = make_sheet_fx_material(k, fx_style);
        if (first || !f.mesh.valid()) f.mesh.create(f.v, f.i, true);
        else {
            f.mesh.update_vertices(f.v.data(), (int)f.v.size());
            if (f.rebuilt || f.mesh.index_count() != (int)f.i.size()) f.mesh.update_indices(f.i.data(), (int)f.i.size());
        }
        f.rebuilt = false;
    }
}

void TreeVisual::update(const SoftBody& b, bool first) {
    // bark: 8-sided rings at both ends of each segment, oriented by the parent frame
    const int sides = 7;
    bv.clear();
    if (first) bi.clear();
    for (const Segment& s : segments) {
        const Joint& j = b.joints[s.joint];
        const Frame& pf = b.frames[j.parent_frame];
        vec3 p0 = b.nodes[pf.node].p, p1 = b.nodes[j.child_node].p;
        quat q0 = pf.q;
        quat q1 = j.child_frame >= 0 ? b.frames[j.child_frame].q : pf.q;
        if (j.broken) {
            // hide broken connection by collapsing the segment onto the child
            p0 = p1;
        }
        vec3 axis = normalize_or(p1 - p0, q0.rotate(vec3(0, 1, 0)));
        float len = length(p1 - p0);
        uint32_t base = (uint32_t)bv.size();
        for (int e = 0; e < 2; e++) {
            quat q = e == 0 ? q0 : q1;
            vec3 c = e == 0 ? p0 : p1;
            float r = e == 0 ? s.r0 : s.r1;
            // ring plane perpendicular to the segment, twist from the frame
            vec3 ref = q.rotate(vec3(1, 0, 0));
            vec3 u = normalize_or(ref - axis * dot(ref, axis), any_perpendicular(axis));
            vec3 w = cross(axis, u);
            for (int k = 0; k <= sides; k++) {
                float a = 2 * kPi * k / sides;
                vec3 n = u * std::cos(a) + w * std::sin(a);
                bv.push_back({c + n * r, n, vec2((float)k / sides, e ? len * 0.5f : 0.0f)});
            }
        }
        if (first)
            for (int k = 0; k < sides; k++) {
                uint32_t a0 = base + k, a1 = base + k + 1, b0 = base + sides + 1 + k, b1 = b0 + 1;
                bi.insert(bi.end(), {a0, b0, a1, a1, b0, b1});
            }
    }
    // leaves: two crossed quads per cluster
    lv.clear();
    if (first) li.clear();
    for (const Leaf& l : leaves) {
        const Frame& f = b.frames[l.frame];
        vec3 c = b.nodes[f.node].p + f.q.rotate(l.offset);
        quat q = f.q * l.rot;
        float s = l.size * 0.5f;
        for (int k = 0; k < 2; k++) {
            vec3 ax = q.rotate(k == 0 ? vec3(1, 0, 0) : vec3(0, 0, 1));
            vec3 ay = q.rotate(vec3(0, 1, 0));
            vec3 nrm = normalize(cross(ax, ay) + ay * 0.8f); // bent normals: softer lighting
            uint32_t base = (uint32_t)lv.size();
            lv.push_back({c - ax * s - ay * s, nrm, {0, 1}});
            lv.push_back({c + ax * s - ay * s, nrm, {1, 1}});
            lv.push_back({c + ax * s + ay * s, nrm, {1, 0}});
            lv.push_back({c - ax * s + ay * s, nrm, {0, 0}});
            if (first) li.insert(li.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
        }
    }
}

void TreeVisual::upload(bool first) {
    if (first || !bark_mesh.valid()) {
        bark_mesh.create(bv, bi, true);
        if (!lv.empty()) leaf_mesh.create(lv, li, true);
    } else {
        bark_mesh.update_vertices(bv.data(), (int)bv.size());
        if (!lv.empty()) leaf_mesh.update_vertices(lv.data(), (int)lv.size());
    }
}

mat4 RigidVisual::frame(const SoftBody& b) const {
    vec3 o = b.nodes[n0].p;
    vec3 x = normalize_or(b.nodes[na].p - o, vec3(1, 0, 0));
    vec3 y = b.nodes[nb].p - o;
    y = normalize_or(y - x * dot(y, x), any_perpendicular(x));
    return mat4(vec4(x, 0), vec4(y, 0), vec4(cross(x, y), 0), vec4(o, 1));
}

bool DynamicObject::prepare_visuals() {
    bool first = visuals_dirty;
    if (!first && body && body->sleeping && !(sheet && sheet->topo != body->topo_version)) return false;
    if (rigid) rigid->model = rigid->frame(*body) * rigid->local;
    for (auto& s : surfaces) s->update(*body, first);
    if (sheet) sheet->update(*body, first);
    if (frame) frame->update(*body);
    if (tree) tree->update(*body, first);
    // sticks: box between beam nodes
    stick_instances.clear();
    stick_instance_mat.clear();
    for (const StickVisual& sv : sticks) {
        const Beam& bm = body->beams[sv.beam];
        if (bm.flags & BF_BROKEN) continue;
        vec3 a = body->nodes[bm.a].p, b = body->nodes[bm.b].p;
        vec3 d = b - a;
        float len = length(d);
        if (len < 1e-4f) continue;
        vec3 y = d / len;
        vec3 x = any_perpendicular(y);
        vec3 z = cross(x, y);
        float w = sv.radius * 2.0f;
        mat4 m(vec4(x * w, 0), vec4(y * len, 0), vec4(z * w, 0), vec4(a, 1));
        // tint by stress (subtle): red when loaded close to the break limit
        float s = bm.strength > 0 ? std::fabs(bm.stress) / bm.strength : 0;
        vec4 tint(1, 1 - s * 0.6f, 1 - s * 0.7f, 1);
        stick_instances.push_back(InstanceData::from(m, tint));
        stick_instance_mat.push_back(sv.mat);
    }
    m_upload_first = first;
    visuals_dirty = false;
    return true;
}

void DynamicObject::upload_visuals() {
    for (auto& s : surfaces) s->upload(m_upload_first);
    if (sheet) sheet->upload(m_upload_first);
    if (frame) frame->upload();
    if (tree) tree->upload(m_upload_first);
    m_upload_first = false;
}

void FrameVisual::update(const phys::SoftBody& b) {
    const phys::FemFrame& f = b.fem;
    constexpr int kSides = 8;
    if (built != f.elems.size()) {
        built = f.elems.size();
        verts.assign(f.elems.size() * kSides * 2, Vertex{});
        idx.clear();
        for (size_t e = 0; e < f.elems.size(); e++) {
            const uint32_t o = (uint32_t)(e * kSides * 2);
            for (int k = 0; k < kSides; k++) {
                const uint32_t a0 = o + k, a1 = o + (k + 1) % kSides, b0 = a0 + kSides, b1 = a1 + kSides;
                idx.insert(idx.end(), {a0, b1, b0, a0, a1, b1}); // (counter-clockwise seen from outside: the faces out)
            }
        }
        rebuilt = true;
    }
    for (size_t e = 0; e < f.elems.size(); e++) {
        const phys::FrameElement& m = f.elems[e];
        Vertex* v = &verts[e * kSides * 2];
        const vec3 a = b.nodes[f.node[m.a]].p, c = b.nodes[f.node[m.b]].p;
        const float L = length(c - a);
        if (m.broken || m.hidden || !(L > 1e-5f)) {
            for (int k = 0; k < kSides * 2; k++) v[k].pos = a; // (nothing to draw)
            continue;
        }
        const vec3 d = (c - a) / L;
        const float r = f.sections[m.section].half;
        const vec3 u = normalize(any_perpendicular(d)), w = cross(d, u);
        const vec3 p0 = a - d * std::min(r, 0.3f * L), p1 = c + d * std::min(r, 0.3f * L); // (into the joint)
        for (int k = 0; k < kSides; k++) {
            const float t = 2.0f * kPi * k / kSides;
            const vec3 n = u * std::cos(t) + w * std::sin(t);
            v[k] = {p0 + n * r, n, vec2((float)k / kSides, 0)};
            v[k + kSides] = {p1 + n * r, n, vec2((float)k / kSides, L)};
        }
    }
    // the plates: per triangle its two faces (6 vertices, the back one wound the other way), rebuilt as triangles tear out;
    // after the frame's triangles its loose ones (FemFrame::loose_tris: fragments torn off, on body nodes, flat)
    const size_t nt = f.tris.size(), nl = f.loose_tris.size();
    if (nt == 0 && nl == 0) return;
    // (the index lists follow the layout: each slot's section, or gone - a count of them alone missed a fragment let
    // loose, its triangles moved from tris to loose_tris and the rest renumbered: slots drawn in another section's
    // material, a bumper's in the body's colour)
    uint64_t layout = 1469598103934665603ull ^ (nt * 1000003 + nl);
    for (const phys::FrameTri& t : f.tris) layout = (layout ^ (t.broken ? 0xffffu : t.section)) * 1099511628211ull;
    for (const phys::FemFrame::LooseTri& t : f.loose_tris) layout = (layout ^ t.section) * 1099511628211ull;
    if (plate_built != layout) {
        plate_built = layout;
        plate_verts.assign((nt + nl) * 6, Vertex{});
        plate_idx.clear();
        plate_ranges.assign(f.shell_sections.size(), {0, 0});
        for (size_t sec = 0; sec < f.shell_sections.size(); sec++) { // (by section: each drawn in its material)
            plate_ranges[sec].first = (int)plate_idx.size();
            for (size_t i = 0; i < nt + nl; i++) {
                if (i < nt ? (f.tris[i].broken || f.tris[i].section != sec) : f.loose_tris[i - nt].section != sec) continue;
                const uint32_t o = (uint32_t)(i * 6);
                plate_idx.insert(plate_idx.end(), {o, o + 1, o + 2, o + 3, o + 5, o + 4});
            }
            plate_ranges[sec].second = (int)plate_idx.size() - plate_ranges[sec].first;
        }
        plate_rebuilt = true;
    }
    node_normal.assign(f.node.size(), vec3(0));
    std::vector<vec3> fn(nt, vec3(0));
    for (size_t i = 0; i < nt; i++) {
        const phys::FrameTri& t = f.tris[i];
        if (t.broken) continue;
        const vec3 a = b.nodes[f.node[t.n[0]]].p, c = b.nodes[f.node[t.n[1]]].p, d = b.nodes[f.node[t.n[2]]].p;
        const vec3 n = cross(c - a, d - a); // (area-weighted)
        fn[i] = normalize_or(n, vec3(0, 1, 0));
        for (uint32_t v : t.n) node_normal[v] += n;
    }
    for (vec3& n : node_normal) n = normalize_or(n, vec3(0, 1, 0));
    const float kCrease = 0.82f; // (cos 35 degrees)
    for (size_t i = 0; i < nt; i++) {
        const phys::FrameTri& t = f.tris[i];
        if (t.broken) continue;
        const float h = 0.5f * f.shell_sections[t.section].t;
        Vertex* v = &plate_verts[i * 6];
        for (int k = 0; k < 3; k++) {
            const vec3 p = b.nodes[f.node[t.n[k]]].p, nn = node_normal[t.n[k]];
            const vec3 n = dot(nn, fn[i]) > kCrease ? nn : fn[i];
            const vec2 uv(p.x + p.y * 0.3f, p.z + p.y * 0.7f);
            v[k] = {p + fn[i] * h, n, uv};
            v[3 + k] = {p - fn[i] * h, -n, uv};
        }
    }
    for (size_t i = 0; i < nl; i++) {
        const phys::FemFrame::LooseTri& t = f.loose_tris[i];
        if (t.n[0] >= b.nodes.size() || t.n[1] >= b.nodes.size() || t.n[2] >= b.nodes.size()) continue;
        const vec3 a = b.nodes[t.n[0]].p, c = b.nodes[t.n[1]].p, d = b.nodes[t.n[2]].p;
        const vec3 n = normalize_or(cross(c - a, d - a), vec3(0, 1, 0));
        const float h = t.section < f.shell_sections.size() ? 0.5f * f.shell_sections[t.section].t : 0.0f;
        Vertex* v = &plate_verts[(nt + i) * 6];
        for (int k = 0; k < 3; k++) {
            const vec3 p = b.nodes[t.n[k]].p;
            const vec2 uv(p.x + p.y * 0.3f, p.z + p.y * 0.7f);
            v[k] = {p + n * h, n, uv};
            v[3 + k] = {p - n * h, -n, uv};
        }
    }
}

MaterialPtr frame_plate_material() {
    static MaterialPtr mat = [] {
        auto m = std::make_shared<Material>();
        m->name = "frame_plates";
        m->color = vec4(0.62f, 0.64f, 0.67f, 1.0f);
        m->specular = 0.5f;
        m->gloss = 40.0f;
        return m;
    }();
    return mat;
}

MaterialPtr frame_tube_material() {
    static MaterialPtr mat = [] {
        auto m = std::make_shared<Material>();
        m->name = "frame_tubes";
        m->color = vec4(0.30f, 0.32f, 0.35f, 1.0f);
        m->specular = 0.6f;
        m->gloss = 48.0f;
        return m;
    }();
    return mat;
}

void FrameVisual::upload() {
    if (!verts.empty()) {
        if (!mesh.valid() || rebuilt) mesh.create(verts, idx, true);
        else mesh.update_vertices(verts.data(), (int)verts.size());
        rebuilt = false;
    }
    if (!plate_verts.empty() && !plate_idx.empty()) {
        if (!plate_mesh.valid() || plate_rebuilt) plate_mesh.create(plate_verts, plate_idx, true);
        else plate_mesh.update_vertices(plate_verts.data(), (int)plate_verts.size());
        plate_rebuilt = false;
    }
}

namespace {
// a see-through copy of a material (cached in `ghosts`), alpha 1 the material itself
const Material* see_through(std::unordered_map<const Material*, std::unique_ptr<Material>>& ghosts, const Material* m, float alpha) {
    if (!m || alpha >= 0.999f) return m;
    auto& g = ghosts[m];
    if (!g) g = std::make_unique<Material>(*m);
    g->blend = true, g->cast_shadow = false, g->double_sided = true, g->alpha_ref = 0.0f;
    g->color = vec4(m->color.x, m->color.y, m->color.z, m->color.w * alpha);
    return g.get();
}
} // namespace

void FrameVisual::draw(Renderer& r, float alpha, float plate_alpha) const {
    if (mesh.valid() && mat && !verts.empty()) r.draw_mesh(&mesh, see_through(ghosts, mat.get(), alpha), mat4());
    if (!plate_mesh.valid() || plate_idx.empty() || plates_hidden) return;
    const float pa = plate_alpha < 0 ? alpha : plate_alpha;
    auto see = [&](const Material* m) { return see_through(ghosts, m, pa); };
    const Material* base = see((plate_mat ? plate_mat : frame_plate_material()).get());
    bool own = false;
    for (size_t sec = 0; sec < plate_ranges.size(); sec++) own |= sec < section_mats.size() && section_mats[sec] && plate_ranges[sec].second > 0;
    if (!own) {
        r.draw_mesh(&plate_mesh, base, mat4());
        return;
    }
    for (size_t sec = 0; sec < plate_ranges.size(); sec++) {
        if (plate_ranges[sec].second <= 0) continue;
        const Material* m = sec < section_mats.size() && section_mats[sec] ? see(section_mats[sec].get()) : base;
        r.draw_mesh(&plate_mesh, m, mat4(), plate_ranges[sec].first, plate_ranges[sec].second);
    }
}

void ShellVisual::draw(Renderer& r, float alpha) const {
    if (ranges.size() <= 1) {
        r.draw_mesh(&mesh, see_through(ghosts, mat.get(), alpha), mat4());
    } else {
        for (size_t m = 0; m < ranges.size(); m++) {
            if (ranges[m].second <= 0) continue;
            const Material* mm = m < mats.size() && mats[m] ? mats[m].get() : mat.get();
            r.draw_mesh(&mesh, see_through(ghosts, mm, alpha), mat4(), ranges[m].first, ranges[m].second);
        }
    }
    for (const auto& f : fx)
        if (!f.v.empty() && f.mesh.valid() && f.mat) r.draw_mesh(&f.mesh, f.mat.get(), mat4());
}

void DynamicObject::update_visuals() {
    if (prepare_visuals()) upload_visuals();
}

void DynamicObject::draw(Renderer& r, InstanceCollector& ic, float xray) {
    if (rigid)
        for (auto& p : rigid->parts) r.draw_mesh(p.mesh, p.mat, rigid->model);
    for (auto& s : surfaces) r.draw_mesh(&s->mesh, s->mat.get(), mat4());
    const float see = xray > 0 ? xray : 1.0f;
    if (sheet) sheet->draw(r, see);
    if (frame) frame->draw(r, 1.0f, see);
    if (tree) {
        r.draw_mesh(&tree->bark_mesh, tree->bark.get(), mat4());
        if (tree->leaf_mesh.valid() && tree->leaf) r.draw_mesh(&tree->leaf_mesh, tree->leaf.get(), mat4());
    }
    const GpuMesh* stick = &SharedAssets::get().stick;
    for (size_t i = 0; i < stick_instances.size(); i++)
        ic.add(stick, stick_mats[stick_instance_mat[i]].get(), stick_instances[i]);
}

void InstanceCollector::begin() {
    for (auto& b : batches) b.data.clear();
}

void InstanceCollector::add(const GpuMesh* mesh, const Material* mat, const InstanceData& d) {
    for (auto& b : batches)
        if (b.mesh == mesh && b.mat == mat) {
            b.data.push_back(d);
            return;
        }
    Batch nb;
    nb.mesh = mesh;
    nb.mat = mat;
    nb.data.push_back(d);
    batches.push_back(std::move(nb));
}

void InstanceCollector::flush(Renderer& r) {
    for (auto& b : batches) {
        if (b.data.empty()) continue;
        if (!b.gpu) {
            b.gpu = std::make_unique<InstanceBatch>();
            b.gpu->init(b.mesh);
        }
        b.gpu->upload(b.data);
        r.draw_instanced(b.gpu.get(), b.mat);
    }
}

// ====================================================================================== helpers
namespace {

void add_face_surface(SurfaceVisual& sv, uint32_t a, uint32_t b, uint32_t c, vec2 ua, vec2 ub, vec2 uc) {
    sv.corner_node.insert(sv.corner_node.end(), {a, b, c});
    sv.corner_uv.insert(sv.corner_uv.end(), {ua, ub, uc});
}

// stable time step for a spring between masses: k*dt^2/m < limit
float clamp_k(float k, float m, float dt = kDefaultDt, float limit = 0.3f) { return std::min(k, limit * m / (dt * dt)); }

} // namespace

// ====================================================================================== soft box
std::unique_ptr<DynamicObject> build_soft_box(World& w, const SoftBoxDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    const int nx = std::max(2, d.nx), ny = std::max(2, d.ny), nz = std::max(2, d.nz);
    const int total = nx * ny * nz;
    const float nm = d.mass / total;
    auto id = [&](int x, int y, int z) { return (uint32_t)((z * ny + y) * nx + x); };
    mat3 R = to_mat3(d.rot);
    for (int z = 0; z < nz; z++)
        for (int y = 0; y < ny; y++)
            for (int x = 0; x < nx; x++) {
                vec3 l((float)x / (nx - 1) - 0.5f, (float)y / (ny - 1) - 0.5f, (float)z / (nz - 1) - 0.5f);
                vec3 p = d.center + R * (l * d.size);
                uint16_t fl = NF_GROUND | NF_CONTACTER;
                if (d.fixed_bottom && y == 0) fl |= NF_FIXED;
                body->add_node(p, nm, fl);
            }
    // beams to all neighbours in the 3x3x3 neighbourhood (each pair once)
    float kk = clamp_k(d.beams.k, nm);
    for (int z = 0; z < nz; z++)
        for (int y = 0; y < ny; y++)
            for (int x = 0; x < nx; x++)
                for (int dz = -1; dz <= 1; dz++)
                    for (int dy = -1; dy <= 1; dy++)
                        for (int dx = -1; dx <= 1; dx++) {
                            int x2 = x + dx, y2 = y + dy, z2 = z + dz;
                            if (x2 < 0 || y2 < 0 || z2 < 0 || x2 >= nx || y2 >= ny || z2 >= nz) continue;
                            uint32_t a = id(x, y, z), b = id(x2, y2, z2);
                            if (b <= a) continue;
                            uint32_t bi = body->add_beam(a, b, kk, d.beams.d, d.beams.strength, d.beams.deform);
                            body->beams[bi].plastic = d.beams.plastic;
                        }
    auto sv = std::make_unique<SurfaceVisual>();
    sv->mat = d.mat ? d.mat : SharedAssets::get().orange;
    // 6 faces: generate quads (CCW from outside)
    auto face = [&](int axis, int side) {
        // axis: 0 x, 1 y, 2 z ; side 0 min, 1 max
        int dims[3] = {nx, ny, nz};
        int u_ax = (axis + 1) % 3, v_ax = (axis + 2) % 3;
        int fixed = side ? dims[axis] - 1 : 0;
        for (int v = 0; v < dims[v_ax] - 1; v++)
            for (int u = 0; u < dims[u_ax] - 1; u++) {
                int c[4][3];
                const int du[4] = {0, 1, 1, 0}, dv[4] = {0, 0, 1, 1};
                for (int k = 0; k < 4; k++) {
                    c[k][axis] = fixed;
                    c[k][u_ax] = u + du[k];
                    c[k][v_ax] = v + dv[k];
                }
                uint32_t q[4];
                vec2 uv[4];
                for (int k = 0; k < 4; k++) {
                    q[k] = id(c[k][0], c[k][1], c[k][2]);
                    uv[k] = vec2((float)(u + du[k]) / (dims[u_ax] - 1), (float)(v + dv[k]) / (dims[v_ax] - 1)) * d.uv_scale;
                }
                // orientation: (u,v,axis) is right-handed; outward normal +axis for side 1
                if (side) {
                    add_face_surface(*sv, q[0], q[1], q[2], uv[0], uv[1], uv[2]);
                    add_face_surface(*sv, q[0], q[2], q[3], uv[0], uv[2], uv[3]);
                    body->add_triangle(q[0], q[1], q[2]);
                    body->add_triangle(q[0], q[2], q[3]);
                } else {
                    add_face_surface(*sv, q[0], q[2], q[1], uv[0], uv[2], uv[1]);
                    add_face_surface(*sv, q[0], q[3], q[2], uv[0], uv[3], uv[2]);
                    body->add_triangle(q[0], q[2], q[1]);
                    body->add_triangle(q[0], q[3], q[2]);
                }
            }
    };
    for (int a = 0; a < 3; a++)
        for (int s = 0; s < 2; s++) face(a, s);
    body->collision_radius = 0.04f;
    body->finalize();
    body->stabilize(kDefaultDt);
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->surfaces.push_back(std::move(sv));
    return obj;
}

// ====================================================================================== FEM shells
namespace {
std::unique_ptr<DynamicObject> fem_shell_object(World& w, std::unique_ptr<SoftBody> body, MaterialPtr visual, const std::string& name) {
    body->collision_radius = 0.01f;
    body->fem.finalize(*body);
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->frame = std::make_unique<FrameVisual>();
    obj->frame->mat = frame_tube_material();
    obj->frame->plate_mat = visual ? visual : frame_plate_material();
    return obj;
}
} // namespace

std::unique_ptr<DynamicObject> build_fem_plate(World& w, const FemPlateDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    ShellSection sec = make_shell_section(d.material, d.thickness);
    if (d.damping >= 0) sec.damping = d.damping;
    const uint16_t si = body->fem.add_shell_section(sec);
    ShellMesher m(*body);
    m.grid(d.origin, d.du, d.dv, d.nu, d.nv, si);
    if (d.fixed)
        for (size_t i = 0; i < body->nodes.size(); i++)
            if (d.fixed(body->nodes[i].p)) body->info[i].flags |= NF_FIXED;
    if (d.more) d.more(*body, m, si);
    m.finish();
    for (Node& x : body->nodes) x.v = d.velocity;
    return fem_shell_object(w, std::move(body), d.visual, name);
}

std::unique_ptr<DynamicObject> build_fem_box(World& w, const FemBoxDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    const uint16_t si = body->fem.add_shell_section(make_shell_section(d.material, d.thickness));
    ShellMesher m(*body);
    // six faces of n x n cells, their normals out (the grid's normal is du x dv), shared edges and corners
    const vec3 h = d.size * 0.5f, X(d.size.x / d.n, 0, 0), Y(0, d.size.y / d.n, 0), Z(0, 0, d.size.z / d.n);
    const vec3 o = -h;
    m.grid(o, Z, X, d.n, d.n, si);                              // bottom
    const std::vector<uint32_t> lid = m.grid(o + vec3(0, d.size.y, 0), X, Z, d.n, d.n, si); // top
    m.grid(o, X, Y, d.n, d.n, si);                              // front
    m.grid(o + vec3(0, 0, d.size.z), Y, X, d.n, d.n, si);      // back
    m.grid(o, Y, Z, d.n, d.n, si);                              // left
    m.grid(o + vec3(d.size.x, 0, 0), Z, Y, d.n, d.n, si);      // right
    for (uint32_t i : lid) body->nodes[i].mass += d.lid_load / (float)lid.size();
    m.finish();
    for (Node& x : body->nodes) {
        const vec3 r = d.rot.rotate(x.p);
        x.p = d.center + r;
        x.v = d.velocity + cross(d.spin, r);
    }
    body->fem.set_orientation(d.rot);
    for (vec3& om : body->fem.w) om = d.spin;
    return fem_shell_object(w, std::move(body), d.visual, name);
}

// ====================================================================================== axe
std::unique_ptr<DynamicObject> build_axe(World& w, const AxeDesc& d, const std::string& name, uint32_t* edge_node, uint32_t* edge_node2) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    auto sv = std::make_unique<SurfaceVisual>();
    sv->mat = d.mat ? d.mat : SharedAssets::get().metal;
    // (hanging straight down from the pivot, then turned by the release angle about the axis)
    const float c = std::cos(d.angle), sn = std::sin(d.angle), cy = std::cos(d.yaw), sy = std::sin(d.yaw);
    auto place = [&](vec3 l) {
        const vec3 q(l.x * c + l.y * sn, -l.x * sn + l.y * c, l.z);
        return d.pivot + vec3(q.x * cy + q.z * sy, q.y, -q.x * sy + q.z * cy);
    };
    const float h = d.handle * 0.5f;
    struct Box {
        int nx, ny, nz;
        uint32_t first;
        uint32_t id(int x, int y, int z) const { return first + (uint32_t)((z * ny + y) * nx + x); }
    };
    // a box of nodes (lo..hi in the axe's own frame), its beams to all neighbours, its six faces; `taper`: its
    // thickness (z) at hi.x against lo.x's, over the last `bevel` of it (a wedge's edge)
    // (a wedge's nodes collide through its faces alone: as contacters, 4 cm round, its edge's pushed the plates beside
    // its cut away; it has no face at its edge - the two sides meet there, a 2 mm strip of a face caught the cut's
    // nodes beside it and swept them along ahead)
    auto box = [&](int nx, int ny, int nz, vec3 lo, vec3 hi, float mass, float taper = 1.0f, float bevel = 0) {
        Box b{nx, ny, nz, (uint32_t)body->nodes.size()};
        const float nm = mass / (float)(nx * ny * nz);
        for (int z = 0; z < nz; z++)
            for (int y = 0; y < ny; y++)
                for (int x = 0; x < nx; x++) {
                    const vec3 t((float)x / (nx - 1), (float)y / (ny - 1), (float)z / (nz - 1));
                    vec3 l = lo + (hi - lo) * t;
                    if (taper != 1.0f) {
                        const float u = bevel > 0 ? std::clamp(1.0f - (hi.x - l.x) / bevel, 0.0f, 1.0f) : t.x; // (0 behind the bevel, 1 at the edge)
                        l.z = 0.5f * (lo.z + hi.z) + (l.z - 0.5f * (lo.z + hi.z)) * (1.0f + (taper - 1.0f) * u);
                    }
                    body->add_node(place(l), nm, taper != 1.0f ? NF_GROUND : NF_GROUND | NF_CONTACTER);
                }
        const float kk = clamp_k(5e8f, nm);
        for (int z = 0; z < nz; z++)
            for (int y = 0; y < ny; y++)
                for (int x = 0; x < nx; x++)
                    for (int dz = -1; dz <= 1; dz++)
                        for (int dy = -1; dy <= 1; dy++)
                            for (int dx = -1; dx <= 1; dx++) {
                                const int x2 = x + dx, y2 = y + dy, z2 = z + dz;
                                if (x2 < 0 || y2 < 0 || z2 < 0 || x2 >= nx || y2 >= ny || z2 >= nz || b.id(x2, y2, z2) <= b.id(x, y, z)) continue;
                                body->add_beam(b.id(x, y, z), b.id(x2, y2, z2), kk, 0.02f * kk * kDefaultDt, 1e12f, 1e12f);
                            }
        const int dims[3] = {nx, ny, nz};
        for (int axis = 0; axis < 3; axis++)
            for (int side = 0; side < 2; side++) {
                if (taper != 1.0f && axis == 0 && side == 1) continue;
                const int u_ax = (axis + 1) % 3, v_ax = (axis + 2) % 3, fixed = side ? dims[axis] - 1 : 0;
                for (int v = 0; v < dims[v_ax] - 1; v++)
                    for (int u = 0; u < dims[u_ax] - 1; u++) {
                        uint32_t q[4];
                        vec2 uv[4];
                        const int du[4] = {0, 1, 1, 0}, dv[4] = {0, 0, 1, 1};
                        for (int k = 0; k < 4; k++) {
                            int cc[3];
                            cc[axis] = fixed, cc[u_ax] = u + du[k], cc[v_ax] = v + dv[k];
                            q[k] = b.id(cc[0], cc[1], cc[2]);
                            uv[k] = vec2((float)(u + du[k]), (float)(v + dv[k])) * 0.5f;
                        }
                        const int o[6] = {0, 1, 2, 0, 2, 3}, r[6] = {0, 2, 1, 0, 3, 2};
                        const int* t = side ? o : r;
                        for (int k = 0; k < 6; k += 3) {
                            add_face_surface(*sv, q[t[k]], q[t[k + 1]], q[t[k + 2]], uv[t[k]], uv[t[k + 1]], uv[t[k + 2]]);
                            body->add_triangle(q[t[k]], q[t[k + 1]], q[t[k + 2]], false);
                        }
                    }
            }
        return b;
    };
    const float top = -(d.length - d.blade_h); // (the blade's top, the handle's foot)
    const Box hb = box(2, 7, 2, vec3(-h, top, -h), vec3(h, -0.35f, h), 0.25f * d.mass);
    // (a wedge to its edge, the +x side: it leads. Flat, 12 cm thick at the edge, it met what it had cut inside a car
    // face on and pushed it along, stuck in its floor)
    const Box bb = box(5, 3, 2, vec3(-0.5f * d.blade_w, -d.length, -0.5f * d.thick), vec3(0.5f * d.blade_w, top, 0.5f * d.thick), 0.75f * d.mass,
                       d.thick > 0 ? std::min(1.0f, d.edge / d.thick) : 1.0f, d.bevel);
    // the handle's foot into the blade's top; its head on the pivot (two fixed nodes on the axis: it swings about it)
    const float kj = clamp_k(5e8f, 0.25f * d.mass / 28.0f);
    for (int z = 0; z < 2; z++)
        for (int x = 0; x < 2; x++)
            for (int bz = 0; bz < 2; bz++)
                for (int bx = 0; bx < 5; bx++) body->add_beam(hb.id(x, 0, z), bb.id(bx, 2, bz), kj, 0.02f * kj * kDefaultDt, 1e12f, 1e12f);
    for (float z : {-0.45f, 0.45f}) {
        const uint32_t p = body->add_node(place(vec3(0, 0, z)), 1.0f, NF_FIXED);
        for (int hz = 0; hz < 2; hz++)
            for (int x = 0; x < 2; x++) body->add_beam(p, hb.id(x, 6, hz), kj, 0.02f * kj * kDefaultDt, 1e12f, 1e12f);
    }
    if (edge_node) *edge_node = bb.id(4, 1, 0); // (released towards -x it swings towards +x: the blade's +x side leads)
    if (edge_node2) *edge_node2 = bb.id(4, 1, 1); // (the edge's other side)
    body->collision_radius = 0.04f;
    body->contact_friction = d.friction;
    // (a solid: its faces push out what is in front of them, or just behind - inside its 2 mm edge, two-sided, its two
    // faces pushed a cut's nodes back and forth in turn, and a sheet's cut edge flew apart)
    body->hull_depth = 0.001f, body->faces_only = true, body->face_skin = 0.002f; // (behind no deeper than half its edge)
    body->finalize();
    body->stabilize(kDefaultDt);
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->surfaces.push_back(std::move(sv));
    return obj;
}

// ====================================================================================== soft sphere
std::unique_ptr<DynamicObject> build_soft_sphere(World& w, const SoftSphereDesc& d, const std::string& name) {
    // icosphere
    std::vector<vec3> verts;
    std::vector<uint32_t> tris;
    const float t = (1.0f + std::sqrt(5.0f)) / 2.0f;
    vec3 iv[12] = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t}, {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    for (auto& v : iv) verts.push_back(normalize(v));
    uint32_t it[] = {0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
                     3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9, 4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1};
    tris.assign(std::begin(it), std::end(it));
    for (int s = 0; s < d.subdiv; s++) {
        std::map<std::pair<uint32_t, uint32_t>, uint32_t> mid;
        auto midpoint = [&](uint32_t a, uint32_t b) {
            auto key = std::make_pair(std::min(a, b), std::max(a, b));
            auto f = mid.find(key);
            if (f != mid.end()) return f->second;
            uint32_t i = (uint32_t)verts.size();
            verts.push_back(normalize(verts[a] + verts[b]));
            mid[key] = i;
            return i;
        };
        std::vector<uint32_t> nt;
        for (size_t k = 0; k < tris.size(); k += 3) {
            uint32_t a = tris[k], b = tris[k + 1], c = tris[k + 2];
            uint32_t ab = midpoint(a, b), bc = midpoint(b, c), ca = midpoint(c, a);
            nt.insert(nt.end(), {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca});
        }
        tris.swap(nt);
    }
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    const float nm = d.mass * 0.7f / verts.size();
    for (auto& v : verts) body->add_node(d.center + v * d.radius, nm);
    uint32_t center = body->add_node(d.center, d.mass * 0.3f, NF_GROUND | NF_CONTACTER); // (rests on the ground once the ball breaks)
    float kk = clamp_k(d.beams.k, nm);
    std::map<std::pair<uint32_t, uint32_t>, bool> edges;
    for (size_t k = 0; k < tris.size(); k += 3)
        for (int e = 0; e < 3; e++) {
            uint32_t a = tris[k + e], b = tris[k + (e + 1) % 3];
            auto key = std::make_pair(std::min(a, b), std::max(a, b));
            if (edges.count(key)) continue;
            edges[key] = true;
            body->add_beam(a, b, kk, d.beams.d, d.beams.strength, d.beams.deform);
        }
    for (uint32_t i = 0; i < verts.size(); i++) body->add_beam(i, center, kk * 0.5f, d.beams.d, d.beams.strength, d.beams.deform);
    // cross links to opposite-ish vertices for volume preservation
    for (uint32_t i = 0; i < verts.size(); i += 2) {
        uint32_t best = i;
        float bd = 1e9f;
        for (uint32_t j = 0; j < verts.size(); j++) {
            float dd = dot(verts[i], verts[j]);
            if (dd < bd) { bd = dd; best = j; }
        }
        if (best != i) body->add_beam(i, best, kk * 0.3f, d.beams.d, d.beams.strength, d.beams.deform);
    }
    auto sv = std::make_unique<SurfaceVisual>();
    sv->mat = d.mat ? d.mat : SharedAssets::get().jelly;
    sv->smooth = true;
    for (size_t k = 0; k < tris.size(); k += 3) {
        uint32_t a = tris[k], b = tris[k + 1], c = tris[k + 2];
        auto uvf = [&](uint32_t i) { vec3 v = verts[i]; return vec2(std::atan2(v.z, v.x) / (2 * kPi) + 0.5f, v.y * 0.5f + 0.5f); };
        add_face_surface(*sv, a, b, c, uvf(a), uvf(b), uvf(c));
        body->add_triangle(a, b, c);
    }
    body->collision_radius = 0.04f;
    body->finalize();
    body->stabilize(kDefaultDt);
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->surfaces.push_back(std::move(sv));
    return obj;
}

std::unique_ptr<DynamicObject> build_ball(World& w, const BallDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    const uint32_t c = body->add_node(d.center, d.mass, NF_GROUND | NF_CONTACTER);
    body->capsules.push_back({c, c, d.radius});
    body->collision_radius = 0.01f; // (its node against triangles: the capsule's sphere does that)
    body->sphere_ball = d.radius;
    body->ground_friction = d.friction;
    body->bounce = d.bounce;
    body->finalize();
    body->info[c].radius = d.radius; // (the ground and the static world: a sphere)
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    MaterialPtr mat = d.mat ? d.mat : SharedAssets::get().metal;
    obj->stick_mats.push_back(mat); // (kept alive: the visual holds it by pointer)
    auto rv = std::make_unique<RigidVisual>();
    rv->parts = {{&SharedAssets::get().sphere, mat.get()}};
    rv->n0 = rv->na = rv->nb = c;
    rv->local = mat4::scale(vec3(d.radius));
    rv->model = mat4::translate(d.center) * rv->local;
    obj->rigid = std::move(rv);
    return obj;
}

// ====================================================================================== rope
std::unique_ptr<DynamicObject> build_rope(World& w, const RopeDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    const int n = std::max(2, d.segments + 1);
    const float nm = d.mass / n;
    for (int i = 0; i < n; i++) {
        float t = (float)i / (n - 1);
        uint16_t fl = NF_GROUND | NF_CONTACTER;
        if ((i == 0 && d.fix_a) || (i == n - 1 && d.fix_b)) fl |= NF_FIXED;
        float m = nm + (i == n - 1 ? d.end_mass : 0.0f);
        body->add_node(lerp(d.a, d.b, t), m, fl);
    }
    float kk = clamp_k(d.beams.k, nm);
    auto obj = std::make_unique<DynamicObject>();
    obj->stick_mats.push_back(d.mat ? d.mat : SharedAssets::get().rust);
    for (int i = 0; i + 1 < n; i++) {
        uint32_t bi = body->add_beam(i, i + 1, kk, d.beams.d, d.beams.strength, d.beams.deform, BT_ROPE);
        body->capsules.push_back({(uint32_t)i, (uint32_t)i + 1, d.radius, -1});
        obj->sticks.push_back({bi, d.radius, 0});
    }
    if (d.end_size > 0) {
        // heavy block at the free end (pendulum / wrecking ball)
        vec3 c = d.b;
        float s = d.end_size * 0.5f;
        uint32_t base = (uint32_t)body->nodes.size();
        float bm = std::max(1.0f, d.end_mass) / 8.0f;
        for (int k = 0; k < 8; k++) {
            vec3 o((k & 1) ? s : -s, (k & 2) ? -2 * s : 0, (k & 4) ? s : -s);
            body->add_node(c + o, bm);
        }
        for (uint32_t a = 0; a < 8; a++)
            for (uint32_t b = a + 1; b < 8; b++) body->add_beam(base + a, base + b, clamp_k(5e6f, bm), 3000, 1e9f, 1e9f);
        for (uint32_t k = 0; k < 4; k++) body->add_beam(n - 1, base + k, kk, d.beams.d, d.beams.strength * 4, d.beams.deform * 4);
        auto sv = std::make_unique<SurfaceVisual>();
        sv->mat = SharedAssets::get().metal;
        const int f[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
        for (auto& q : f) {
            uint32_t a = base + q[0], b = base + q[1], cc = base + q[2], dd = base + q[3];
            // faces defined with outward CCW for this local layout (y down = -)
            add_face_surface(*sv, a, cc, b, {0, 0}, {1, 1}, {1, 0});
            add_face_surface(*sv, a, dd, cc, {0, 0}, {0, 1}, {1, 1});
            body->add_triangle(a, cc, b);
            body->add_triangle(a, dd, cc);
        }
        obj->surfaces.push_back(std::move(sv));
    }
    body->collision_radius = 0.05f;
    body->finalize();
    body->stabilize(kDefaultDt);
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    return obj;
}

// ====================================================================================== bridge
std::unique_ptr<DynamicObject> build_bridge(World& w, const BridgeDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    vec3 axis = d.end - d.start;
    float L = length(axis);
    vec3 fwd = axis / L;
    vec3 side = normalize(cross(vec3(0, 1, 0), fwd)); // left
    const int S = std::max(2, d.segments);
    const float half_w = d.width * 0.5f;
    const int stations = S + 1;
    // node layout per station: deck left, deck right, (truss) top left, top right, outrigger left/right
    std::vector<uint32_t> dl(stations), dr(stations), tl(stations, UINT32_MAX), tr(stations, UINT32_MAX), ol(stations), orr(stations);
    float total_mass = d.mass_per_m * L;
    int per_station = d.truss ? 6 : 2;
    float nm = total_mass / (stations * per_station);
    auto obj = std::make_unique<DynamicObject>();
    obj->stick_mats.push_back(d.truss_mat ? d.truss_mat : SharedAssets::get().rust);
    obj->stick_mats.push_back(d.deck_mat ? d.deck_mat : SharedAssets::get().dark_wood);
    for (int i = 0; i < stations; i++) {
        float t = (float)i / S;
        vec3 c = d.start + axis * t;
        bool anchor = (i == 0 || i == S);
        uint16_t fl = NF_GROUND | NF_CONTACTER | (anchor ? NF_FIXED : 0);
        dl[i] = body->add_node(c + side * half_w, nm, fl);
        dr[i] = body->add_node(c - side * half_w, nm, fl);
        if (d.truss) {
            ol[i] = body->add_node(c + side * (half_w + 0.9f), nm, fl);
            orr[i] = body->add_node(c - side * (half_w + 0.9f), nm, fl);
            if (!anchor || true) {
                float h = d.truss_height;
                tl[i] = body->add_node(c + side * (half_w + 0.15f) + vec3(0, h, 0), nm, NF_GROUND | NF_CONTACTER | (anchor ? NF_FIXED : 0));
                tr[i] = body->add_node(c - side * (half_w + 0.15f) + vec3(0, h, 0), nm, NF_GROUND | NF_CONTACTER | (anchor ? NF_FIXED : 0));
            }
        }
    }
    const BeamParams& db = d.deck;
    const BeamParams& tb = d.truss_beams;
    float kd = clamp_k(db.k, nm), kt = clamp_k(tb.k, nm);
    auto beam = [&](uint32_t a, uint32_t b, const BeamParams& p, float k, int vis_mat, float vis_r) {
        uint32_t bi = body->add_beam(a, b, k, p.d, p.strength, p.deform);
        body->beams[bi].plastic = p.plastic;
        if (vis_mat >= 0) obj->sticks.push_back({bi, vis_r, vis_mat});
        return bi;
    };
    for (int i = 0; i < stations; i++) {
        beam(dl[i], dr[i], db, kd, 1, 0.09f); // cross beam
        if (d.truss) {
            beam(dl[i], ol[i], db, kd, 1, 0.08f);
            beam(dr[i], orr[i], db, kd, 1, 0.08f);
            beam(dl[i], tl[i], tb, kt, 0, 0.07f); // verticals
            beam(dr[i], tr[i], tb, kt, 0, 0.07f);
            beam(ol[i], tl[i], tb, kt, 0, 0.05f); // knee braces
            beam(orr[i], tr[i], tb, kt, 0, 0.05f);
            beam(dl[i], tr[i], tb, kt * 0.5f, -1, 0); // hidden stiffeners through the deck plane
            beam(dr[i], tl[i], tb, kt * 0.5f, -1, 0);
        }
        if (i + 1 < stations) {
            int j = i + 1;
            beam(dl[i], dl[j], db, kd, 1, 0.1f); // stringers
            beam(dr[i], dr[j], db, kd, 1, 0.1f);
            beam(dl[i], dr[j], db, kd, -1, 0);   // deck X bracing
            beam(dr[i], dl[j], db, kd, -1, 0);
            if (d.truss) {
                beam(ol[i], ol[j], db, kd, -1, 0);
                beam(orr[i], orr[j], db, kd, -1, 0);
                beam(tl[i], tl[j], tb, kt, 0, 0.08f); // top chords
                beam(tr[i], tr[j], tb, kt, 0, 0.08f);
                // diagonals (Pratt: towards the center)
                bool left_half = i < S / 2;
                if (left_half) {
                    beam(tl[i], dl[j], tb, kt, 0, 0.06f);
                    beam(tr[i], dr[j], tb, kt, 0, 0.06f);
                } else {
                    beam(dl[i], tl[j], tb, kt, 0, 0.06f);
                    beam(dr[i], tr[j], tb, kt, 0, 0.06f);
                }
                beam(tl[i], ol[j], tb, kt * 0.5f, -1, 0);
                beam(tr[i], orr[j], tb, kt * 0.5f, -1, 0);
            }
        }
    }
    if (d.girder_depth > 0) {
        // Warren girder under both deck edges: lower chord + verticals + alternating diagonals
        std::vector<uint32_t> gl(stations), gr(stations);
        for (int i = 0; i < stations; i++) {
            bool anchor = (i == 0 || i == S);
            vec3 c = d.start + axis * ((float)i / S);
            uint16_t fl = NF_GROUND | NF_CONTACTER | (anchor ? NF_FIXED : 0);
            gl[i] = body->add_node(c + side * half_w * 0.9f - vec3(0, d.girder_depth, 0), nm * 0.5f, fl);
            gr[i] = body->add_node(c - side * half_w * 0.9f - vec3(0, d.girder_depth, 0), nm * 0.5f, fl);
        }
        for (int i = 0; i < stations; i++) {
            beam(dl[i], gl[i], tb, kt, 1, 0.07f);
            beam(dr[i], gr[i], tb, kt, 1, 0.07f);
            beam(gl[i], gr[i], tb, kt, -1, 0);
            if (i + 1 < stations) {
                beam(gl[i], gl[i + 1], tb, kt, 1, 0.09f);
                beam(gr[i], gr[i + 1], tb, kt, 1, 0.09f);
                if (i % 2 == 0) {
                    beam(dl[i], gl[i + 1], tb, kt, 1, 0.06f);
                    beam(dr[i], gr[i + 1], tb, kt, 1, 0.06f);
                } else {
                    beam(gl[i], dl[i + 1], tb, kt, 1, 0.06f);
                    beam(gr[i], dr[i + 1], tb, kt, 1, 0.06f);
                }
                beam(gl[i], gr[i + 1], tb, kt * 0.5f, -1, 0);
                beam(gr[i], gl[i + 1], tb, kt * 0.5f, -1, 0);
            }
        }
    }
    // deck surface (collision + render): planks between stations
    auto sv = std::make_unique<SurfaceVisual>();
    sv->mat = d.deck_mat ? d.deck_mat : SharedAssets::get().wood;
    for (int i = 0; i < S; i++) {
        uint32_t a = dr[i], b = dr[i + 1], c = dl[i + 1], e = dl[i];
        float u0 = 0, u1 = 1, v0 = (float)i * L / S / d.width, v1 = (float)(i + 1) * L / S / d.width;
        // CCW seen from above: right->forward->left
        add_face_surface(*sv, a, b, c, {u0, v0}, {u0, v1}, {u1, v1});
        add_face_surface(*sv, a, c, e, {u0, v0}, {u1, v1}, {u1, v0});
        body->add_triangle(a, b, c);
        body->add_triangle(a, c, e);
        body->tris[body->tris.size() - 1].surface = d.surface;
        body->tris[body->tris.size() - 2].surface = d.surface;
    }
    sv->mat->double_sided = true;
    obj->surfaces.push_back(std::move(sv));
    body->collision_radius = 0.06f;
    body->is_static_like = true;
    body->finalize();
    body->stabilize(kDefaultDt);
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    return obj;
}

// ====================================================================================== tree
namespace {

struct TreeBuild {
    SoftBody* body;
    TreeVisual* vis;
    const TreeDesc* d;
    Rng rng{1};
    float wood_density = 650.0f;
    float E = 1.5e9f;        // effective Young's modulus (softer than real wood -> visible sway)
    float rupture = 45e6f;   // modulus of rupture (Pa)
    std::vector<float> wind;

    // Adds a node with a frame looking along `dir` (local +Y = dir).
    uint32_t add_frame_node(vec3 p, vec3 dir, float seg_len, float radius, bool fixed) {
        float mass = std::max(0.3f, kPi * radius * radius * seg_len * wood_density);
        uint32_t n = body->add_node(p, fixed ? 0.0f : mass, NF_GROUND | (fixed ? NF_FIXED : 0));
        if (fixed) {
            body->nodes[n].mass = mass;
            body->nodes[n].inv_mass = 0;
        }
        quat q = quat_from_to(vec3(0, 1, 0), normalize(dir));
        // twist around the axis randomly so leaf placement varies
        q = q * quat::axis_angle(vec3(0, 1, 0), rng.range(0, 2 * kPi));
        float r_eff = std::max(seg_len * 0.5f, 0.15f);
        body->add_frame(n, q, mass * r_eff * r_eff);
        wind.push_back(0);
        return n;
    }

    void connect(uint32_t parent_node, uint32_t child_node, float r_parent, float r_child) {
        const Node& pn = body->nodes[parent_node];
        const Node& cn = body->nodes[child_node];
        float len = distance(pn.p, cn.p);
        float r = 0.5f * (r_parent + r_child);
        float I = kPi * r * r * r * r / 4.0f;
        float k_ang = d->stiffness * E * I / std::max(len, 0.2f);
        float mc = cn.mass, mp = pn.inv_mass > 0 ? pn.mass : mc * 10.0f;
        float m_eff = std::min(mc, mp);
        float k_lin = 0.25f * m_eff / (kDefaultDt * kDefaultDt);
        // stability: angular spring vs frame inertia of the lighter side
        const Frame& cf = body->frames[body->info[child_node].frame];
        float Ic = cf.inv_inertia > 0 ? 1.0f / cf.inv_inertia : 1e9f;
        k_ang = std::min(k_ang, 0.2f * Ic / (kDefaultDt * kDefaultDt));
        float d_ang = 2.0f * 0.35f * std::sqrt(k_ang * Ic);
        float d_lin = 2.0f * 0.5f * std::sqrt(k_lin * m_eff);
        uint32_t j = body->add_joint(body->info[parent_node].frame, child_node, body->info[child_node].frame, k_lin, d_lin, k_ang, d_ang);
        Joint& jj = body->joints[j];
        jj.radius = r;
        jj.break_torque = d->strength * rupture * kPi * r * r * r / 4.0f;
        jj.break_force = d->strength * 40e6f * kPi * r * r; // axial/shear pull-out
        jj.yield_angle = 0.18f;
        body->capsules.push_back({parent_node, child_node, r, (int32_t)j});
        vis->segments.push_back({j, r_parent, r_child});
    }

    void add_leaves(uint32_t node, float size, int count, float spread, MaterialPtr) {
        int f = body->info[node].frame;
        if (f < 0) return;
        for (int i = 0; i < count; i++) {
            TreeVisual::Leaf l;
            l.frame = (uint32_t)f;
            l.offset = vec3(rng.range(-spread, spread), rng.range(-spread * 0.3f, spread * 0.8f), rng.range(-spread, spread));
            l.rot = quat::axis_angle(vec3(0, 1, 0), rng.range(0, 2 * kPi)) * quat::axis_angle(vec3(1, 0, 0), rng.range(-0.5f, 0.5f));
            l.size = size * rng.range(0.8f, 1.2f);
            l.tint = vec4(1);
            vis->leaves.push_back(l);
        }
        wind[node] += size * size * count * 0.35f;
    }

    // Recursive branch: returns nothing, creates `segs` segments starting at parent node.
    void branch(uint32_t parent, vec3 dir, float length, float r0, float r1, int segs, int depth, bool leafy) {
        float seg_len = length / segs;
        uint32_t prev = parent;
        float prev_r = r0;
        vec3 p = body->nodes[parent].p;
        vec3 dcur = normalize(dir);
        for (int s = 1; s <= segs; s++) {
            float t = (float)s / segs;
            float r = lerpf(r0, r1, t);
            // gentle curvature + upward/downward tendency
            vec3 bend = rng.unit_vector() * 0.18f;
            if (d->kind == TreeKind::Pine && depth > 0) bend += vec3(0, -0.12f, 0);
            else bend += vec3(0, depth > 0 ? 0.08f : 0.02f, 0);
            dcur = normalize(dcur + bend);
            p += dcur * seg_len;
            uint32_t n = add_frame_node(p, dcur, seg_len, r, false);
            connect(prev, n, prev_r, r);
            // children
            if (depth < 2 && s < segs) {
                int children = 0;
                if (d->kind == TreeKind::Deciduous) children = (depth == 0 ? (s >= segs / 3 ? 2 : 0) : (rng.uniform() < 0.6f ? 1 : 0));
                else if (d->kind == TreeKind::Birch) children = (depth == 0 ? (s >= segs / 2 ? 1 : 0) : (rng.uniform() < 0.4f ? 1 : 0));
                else if (d->kind == TreeKind::Dead) children = (depth == 0 ? (s >= segs / 3 ? 1 : 0) : 0);
                else if (d->kind == TreeKind::Bush) children = depth == 0 ? 1 : 0;
                for (int c = 0; c < children; c++) {
                    vec3 perp = normalize(cross(dcur, rng.unit_vector()));
                    float ang = rng.range(0.6f, 1.0f);
                    vec3 cd = normalize(dcur * std::cos(ang) + perp * std::sin(ang));
                    float cl = length * rng.range(0.35f, 0.55f) * (1.0f - t * 0.4f);
                    int cs = std::max(2, segs / 2);
                    branch(n, cd, cl, r * 0.6f, r * 0.2f, cs, depth + 1, leafy);
                }
            }
            if (leafy && (depth >= 1 || s >= segs - 1)) {
                float sz = d->kind == TreeKind::Birch ? 1.1f : (d->kind == TreeKind::Bush ? 0.9f : 1.6f);
                add_leaves(n, sz, depth == 0 ? 3 : 2, seg_len * 0.6f, nullptr);
            }
            prev = n;
            prev_r = r;
        }
    }
};

} // namespace

std::unique_ptr<DynamicObject> build_tree(World& w, const TreeDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    auto vis = std::make_unique<TreeVisual>();
    TreeBuild tb;
    tb.body = body.get();
    tb.vis = vis.get();
    tb.d = &d;
    tb.rng = Rng(d.seed * 7919u + 13u);
    auto& A = SharedAssets::get();
    vis->bark = A.bark;
    bool leafy = d.leaves && d.kind != TreeKind::Dead;
    // root plate: an anchor in the ground and a free root node whose frame can rotate. A joint on a fixed node
    // cannot tilt its child (the frame of a fixed node never rotates and the linear spring pins the child), so
    // the soil is this extra joint: stiff in position, soft and plastic in angle, torn out after a long lean.
    uint32_t anchor = tb.add_frame_node(d.base - vec3(0, 0.3f, 0), vec3(0, 1, 0), 0.5f, d.trunk_radius, true);
    uint32_t root = tb.add_frame_node(d.base, vec3(0, 1, 0), 0.6f, d.trunk_radius, false);
    body->info[root].flags &= ~NF_GROUND; // it sits 10 cm below the surface: no ground contact fighting the soil joint
    {
        const float r = d.trunk_radius;
        float trunk = d.strength * tb.rupture * kPi * r * r * r / 4.0f;
        float root_yield = 0.22f * trunk;
        float m = body->nodes[root].mass;
        const Frame& rf = body->frames[body->info[root].frame];
        float Ir = 1.0f / rf.inv_inertia;
        float k_lin = 0.25f * m / (kDefaultDt * kDefaultDt);
        float k_ang = std::min(root_yield / 0.08f, 0.2f * Ir / (kDefaultDt * kDefaultDt)); // ~4.5 deg elastic
        uint32_t j = body->add_joint(body->info[anchor].frame, root, body->info[root].frame, k_lin, 2 * 0.5f * std::sqrt(k_lin * m), k_ang,
                                     2 * 0.35f * std::sqrt(k_ang * Ir));
        Joint& jt = body->joints[j];
        jt.radius = r;
        jt.break_torque = root_yield;
        jt.yield_angle = 0.08f;
        jt.max_bend = 1.1f; // leans ~60 deg before it is uprooted
    }
    switch (d.kind) {
    case TreeKind::Pine: {
        vis->leaf = A.needles;
        tb.E = 2.0e9f;
        int segs = std::max(5, (int)(d.height / 1.2f));
        float seg_len = d.height / segs;
        uint32_t prev = root;
        float prev_r = d.trunk_radius;
        vec3 p = d.base;
        for (int s = 1; s <= segs; s++) {
            float t = (float)s / segs;
            float r = lerpf(d.trunk_radius, d.trunk_radius * 0.15f, t);
            p += normalize(vec3(tb.rng.range(-0.04f, 0.04f), 1, tb.rng.range(-0.04f, 0.04f))) * seg_len;
            uint32_t n = tb.add_frame_node(p, vec3(0, 1, 0), seg_len, r, false);
            tb.connect(prev, n, prev_r, r);
            // whorl of branches (skip the lowest 25%)
            if (t > 0.22f && s < segs) {
                int nb = 4;
                float blen = (1.0f - t) * d.height * 0.42f + 0.4f;
                for (int b = 0; b < nb; b++) {
                    float a = (2 * kPi * b) / nb + tb.rng.range(-0.3f, 0.3f) + s * 0.7f;
                    vec3 bd = normalize(vec3(std::cos(a), 0.05f, std::sin(a)));
                    // short 2-segment branch with needles
                    uint32_t bp = n;
                    float bpr = r * 0.4f;
                    vec3 q = p;
                    for (int k = 1; k <= 2; k++) {
                        q += normalize(bd + vec3(0, -0.12f * k, 0)) * (blen / 2);
                        float br = bpr * (1.0f - 0.4f * k);
                        uint32_t bn = tb.add_frame_node(q, bd, blen / 2, br, false);
                        tb.connect(bp, bn, bpr, br);
                        if (leafy) tb.add_leaves(bn, 1.5f + blen * 0.35f, 2, blen * 0.25f, nullptr);
                        bp = bn;
                        bpr = br;
                    }
                }
            }
            if (leafy && s >= segs - 1) tb.add_leaves(n, 1.3f, 3, 0.3f, nullptr);
            prev = n;
            prev_r = r;
        }
        break;
    }
    case TreeKind::Bush: {
        vis->leaf = A.bush;
        vis->bark = A.dark_wood;
        tb.E = 1.0e9f;
        int stems = 4 + (int)(tb.rng.uniform() * 3);
        for (int s = 0; s < stems; s++) {
            vec3 dir = normalize(vec3(tb.rng.range(-0.6f, 0.6f), 1, tb.rng.range(-0.6f, 0.6f)));
            tb.branch(root, dir, d.height * tb.rng.range(0.7f, 1.0f), d.trunk_radius, d.trunk_radius * 0.3f, 3, 1, leafy);
        }
        break;
    }
    default: {
        vis->leaf = d.kind == TreeKind::Birch ? A.leaves2 : A.leaves;
        if (d.kind == TreeKind::Birch) vis->bark = A.birch_bark;
        int segs = std::max(4, (int)(d.height / 1.4f));
        tb.branch(root, vec3(0, 1, 0), d.height * 0.8f, d.trunk_radius, d.trunk_radius * 0.3f, segs, 0, leafy);
        break;
    }
    }
    body->wind_area = tb.wind;
    body->wind_area.resize(body->nodes.size(), 0);
    body->collision_radius = 0.03f;
    body->is_static_like = true;
    body->air_drag = 0.0f;
    body->finalize();
    body->stabilize(kDefaultDt);
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->tree = std::move(vis);
    return obj;
}

// ====================================================================================== pole (cantilever)
std::unique_ptr<DynamicObject> build_pole(World& w, const PoleDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    auto vis = std::make_unique<TreeVisual>();
    vis->bark = d.mat ? d.mat : SharedAssets::get().metal;
    vec3 dir = normalize(d.dir);
    const int S = std::max(1, d.segments);
    float seg = d.length / S;
    float nm = d.mass / S;
    quat q = quat_from_to(vec3(0, 1, 0), dir);
    std::vector<uint32_t> nodes;
    for (int i = 0; i <= S; i++) {
        bool fixed = i == 0;
        float m = nm + (i == S ? d.tip_mass : 0.0f);
        uint32_t n = body->add_node(d.base + dir * (seg * i), fixed ? 0.0f : m, NF_GROUND | NF_CONTACTER | (fixed ? NF_FIXED : 0));
        body->nodes[n].mass = m;
        float r_eff = std::max(seg * 0.5f, 0.1f);
        body->add_frame(n, q, m * r_eff * r_eff);
        nodes.push_back(n);
    }
    for (int i = 0; i < S; i++) {
        uint32_t p = nodes[i], c = nodes[i + 1];
        float mc = body->nodes[c].mass;
        float k_lin = 0.25f * mc / (kDefaultDt * kDefaultDt);
        const Frame& cf = body->frames[body->info[c].frame];
        float Ic = 1.0f / cf.inv_inertia;
        float k_ang = std::min(d.k_ang, 0.2f * Ic / (kDefaultDt * kDefaultDt));
        uint32_t j = body->add_joint(body->info[p].frame, c, body->info[c].frame, k_lin, 2 * 0.5f * std::sqrt(k_lin * mc), k_ang,
                                     2 * 0.15f * std::sqrt(k_ang * Ic));
        body->joints[j].yield_angle = d.yield_deg * kDeg2Rad;
        body->joints[j].break_torque = d.break_torque;
        body->joints[j].max_bend = 0.25f; // brittle: snaps soon after yielding
        body->joints[j].radius = d.radius;
        body->capsules.push_back({p, c, d.radius, (int32_t)j});
        vis->segments.push_back({j, d.radius, d.radius});
    }
    body->collision_radius = 0.03f;
    body->finalize();
    body->stabilize(kDefaultDt);
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->tree = std::move(vis);
    return obj;
}

// ====================================================================================== traffic cone
std::unique_ptr<DynamicObject> build_traffic_cone(World& w, const ConeDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    const int ring = 6;
    float nm = d.mass / (ring * 2 + 1);
    std::vector<uint32_t> base_ring, mid_ring;
    for (int i = 0; i < ring; i++) {
        float a = 2 * kPi * i / ring;
        base_ring.push_back(body->add_node(d.base + vec3(std::cos(a) * d.radius * 1.4f, 0.02f, std::sin(a) * d.radius * 1.4f), nm));
    }
    for (int i = 0; i < ring; i++) {
        float a = 2 * kPi * i / ring;
        mid_ring.push_back(body->add_node(d.base + vec3(std::cos(a) * d.radius * 0.55f, d.height * 0.55f, std::sin(a) * d.radius * 0.55f), nm));
    }
    uint32_t top = body->add_node(d.base + vec3(0, d.height, 0), nm);
    float k = clamp_k(4e5f, nm);
    for (int i = 0; i < ring; i++) {
        int j = (i + 1) % ring;
        body->add_beam(base_ring[i], base_ring[j], k, 150, 1e9f, 1e9f);
        body->add_beam(base_ring[i], base_ring[(i + ring / 2) % ring], k, 150, 1e9f, 1e9f);
        body->add_beam(mid_ring[i], mid_ring[j], k, 150, 1e9f, 1e9f);
        body->add_beam(mid_ring[i], mid_ring[(i + ring / 2) % ring], k, 150, 1e9f, 1e9f);
        body->add_beam(base_ring[i], mid_ring[i], k, 150, 1e9f, 1e9f);
        body->add_beam(base_ring[i], mid_ring[j], k, 150, 1e9f, 1e9f);
        body->add_beam(base_ring[j], mid_ring[i], k, 150, 1e9f, 1e9f);
        body->add_beam(mid_ring[i], top, k, 150, 1e9f, 1e9f);
        body->add_beam(base_ring[i], top, k * 0.5f, 150, 1e9f, 1e9f);
    }
    auto sv = std::make_unique<SurfaceVisual>();
    auto m = std::make_shared<Material>(*SharedAssets::get().cone_mat);
    sv->mat = m;
    sv->smooth = false;
    for (int i = 0; i < ring; i++) {
        int j = (i + 1) % ring;
        // sides CCW from outside
        add_face_surface(*sv, base_ring[i], mid_ring[j], base_ring[j], {0, 1}, {1, 0}, {1, 1});
        add_face_surface(*sv, base_ring[i], mid_ring[i], mid_ring[j], {0, 1}, {0, 0}, {1, 0});
        add_face_surface(*sv, mid_ring[i], top, mid_ring[j], {0, 1}, {0.5f, 0}, {1, 1});
        body->add_triangle(base_ring[i], mid_ring[j], base_ring[j]);
        body->add_triangle(base_ring[i], mid_ring[i], mid_ring[j]);
        body->add_triangle(mid_ring[i], top, mid_ring[j]);
    }
    body->collision_radius = 0.03f;
    body->finalize();
    body->stabilize(kDefaultDt);
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->surfaces.push_back(std::move(sv));
    return obj;
}

// ====================================================================================== tower
std::unique_ptr<DynamicObject> build_tower(World& w, const TowerDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    const int L = std::max(1, d.levels);
    float nm = d.mass / ((L + 1) * 4);
    std::vector<std::array<uint32_t, 4>> lv(L + 1);
    float hw = d.width * 0.5f;
    for (int l = 0; l <= L; l++) {
        float y = d.height * l / L;
        float s = hw * (1.0f - 0.25f * (float)l / L);
        const vec2 c[4] = {{-s, -s}, {s, -s}, {s, s}, {-s, s}};
        for (int k = 0; k < 4; k++) lv[l][k] = body->add_node(d.base + vec3(c[k].x, y, c[k].y), nm, NF_GROUND | NF_CONTACTER | (l == 0 ? NF_FIXED : 0));
    }
    auto obj = std::make_unique<DynamicObject>();
    obj->stick_mats.push_back(d.mat ? d.mat : SharedAssets::get().red);
    float k = clamp_k(d.beams.k, nm);
    auto beam = [&](uint32_t a, uint32_t b, float r) {
        uint32_t bi = body->add_beam(a, b, k, d.beams.d, d.beams.strength, d.beams.deform);
        body->beams[bi].plastic = d.beams.plastic;
        obj->sticks.push_back({bi, r, 0});
        body->capsules.push_back({a, b, r, -1});
    };
    for (int l = 0; l <= L; l++)
        for (int k2 = 0; k2 < 4; k2++) {
            int n2 = (k2 + 1) % 4;
            if (l > 0) beam(lv[l][k2], lv[l][n2], 0.05f);
            if (l < L) {
                beam(lv[l][k2], lv[l + 1][k2], 0.08f);
                beam(lv[l][k2], lv[l + 1][n2], 0.04f);
                beam(lv[l][n2], lv[l + 1][k2], 0.04f);
            }
        }
    for (int l = 1; l <= L; l++) {
        uint32_t bi = body->add_beam(lv[l][0], lv[l][2], k, d.beams.d, d.beams.strength, d.beams.deform);
        (void)bi;
        body->add_beam(lv[l][1], lv[l][3], k, d.beams.d, d.beams.strength, d.beams.deform);
    }
    body->collision_radius = 0.05f;
    body->is_static_like = true;
    body->finalize();
    body->stabilize(kDefaultDt);
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    return obj;
}

// ====================================================================================== net
std::unique_ptr<DynamicObject> build_net(World& w, const NetDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    const int n = std::max(3, d.n);
    float nm = d.mass / (n * n);
    auto id = [&](int x, int z) { return (uint32_t)(z * n + x); };
    for (int z = 0; z < n; z++)
        for (int x = 0; x < n; x++) {
            bool border = x == 0 || z == 0 || x == n - 1 || z == n - 1;
            vec3 p = d.center + vec3(((float)x / (n - 1) - 0.5f) * d.size, 0, ((float)z / (n - 1) - 0.5f) * d.size);
            body->add_node(p, nm, NF_GROUND | NF_CONTACTER | (border ? NF_FIXED : 0));
        }
    float k = clamp_k(d.beams.k, nm);
    auto obj = std::make_unique<DynamicObject>();
    for (int z = 0; z < n; z++)
        for (int x = 0; x < n; x++) {
            if (x + 1 < n) body->add_beam(id(x, z), id(x + 1, z), k, d.beams.d, d.beams.strength, d.beams.deform, BT_ROPE);
            if (z + 1 < n) body->add_beam(id(x, z), id(x, z + 1), k, d.beams.d, d.beams.strength, d.beams.deform, BT_ROPE);
            if (x + 1 < n && z + 1 < n) {
                body->add_beam(id(x, z), id(x + 1, z + 1), k * 0.3f, d.beams.d, d.beams.strength, d.beams.deform, BT_ROPE);
                body->add_beam(id(x + 1, z), id(x, z + 1), k * 0.3f, d.beams.d, d.beams.strength, d.beams.deform, BT_ROPE);
            }
        }
    auto sv = std::make_unique<SurfaceVisual>();
    auto m = std::make_shared<Material>(*SharedAssets::get().blue);
    m->double_sided = true;
    m->diffuse = SharedAssets::get().tex_checker;
    sv->mat = m;
    sv->smooth = true;
    for (int z = 0; z + 1 < n; z++)
        for (int x = 0; x + 1 < n; x++) {
            uint32_t a = id(x, z), b = id(x + 1, z), c = id(x + 1, z + 1), e = id(x, z + 1);
            vec2 ua((float)x / (n - 1), (float)z / (n - 1)), ub((float)(x + 1) / (n - 1), ua.y), uc(ub.x, (float)(z + 1) / (n - 1)), ue(ua.x, uc.y);
            add_face_surface(*sv, a, c, b, ua * 4, uc * 4, ub * 4);
            add_face_surface(*sv, a, e, c, ua * 4, ue * 4, uc * 4);
            body->add_triangle(a, c, b);
            body->add_triangle(a, e, c);
        }
    body->collision_radius = 0.06f;
    body->finalize();
    body->stabilize(kDefaultDt);
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->surfaces.push_back(std::move(sv));
    return obj;
}

// ====================================================================================== cloth
std::unique_ptr<DynamicObject> build_cloth(World& w, const ClothDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    const int nu = std::max(2, d.nu), nv = std::max(2, d.nv);
    const float nm = d.mass / (nu * nv);
    vec3 U = normalize(d.u) * d.width, V = normalize(d.v) * d.height;
    auto id = [&](int i, int j) { return (uint32_t)(j * nu + i); };
    for (int j = 0; j < nv; j++)
        for (int i = 0; i < nu; i++) {
            bool pin = false;
            if (d.pin == 0) pin = j == 0;
            else if (d.pin == 1) pin = j == 0 && (i == 0 || i == nu - 1);
            else if (d.pin == 2) pin = (j == 0 || j == nv - 1) && (i == 0 || i == nu - 1);
            else if (d.pin == 3) pin = j == 0 || j == nv - 1 || i == 0 || i == nu - 1;
            else pin = j == 0 || i == 0 || i == nu - 1;
            vec3 p = d.origin + U * ((float)i / (nu - 1)) + V * ((float)j / (nv - 1));
            body->add_node(p, nm, NF_GROUND | NF_CONTACTER | (pin ? NF_FIXED : 0));
        }
    // stretch (structural), shear (diagonal) and bend (skip one) links; all are rope beams so the cloth folds freely
    auto link = [&](uint32_t a, uint32_t b, float k, float strain_scale) {
        uint32_t bi = body->add_beam(a, b, k, d.damping * std::sqrt(k * nm), strain_scale, 1e9f, BT_ROPE); // strength set below
        body->beams[bi].flags |= BF_NO_DEFORM;
    };
    for (int j = 0; j < nv; j++)
        for (int i = 0; i < nu; i++) {
            if (i + 1 < nu) link(id(i, j), id(i + 1, j), d.stretch_k, 1.0f);
            if (j + 1 < nv) link(id(i, j), id(i, j + 1), d.stretch_k, 1.0f);
            if (i + 1 < nu && j + 1 < nv) {
                link(id(i, j), id(i + 1, j + 1), d.stretch_k * 0.3f, 1.5f);
                link(id(i + 1, j), id(i, j + 1), d.stretch_k * 0.3f, 1.5f);
            }
            if (i + 2 < nu) link(id(i, j), id(i + 2, j), d.stretch_k * 0.05f, 3.0f);
            if (j + 2 < nv) link(id(i, j), id(i, j + 2), d.stretch_k * 0.05f, 3.0f);
        }
    auto sv = std::make_unique<SurfaceVisual>();
    sv->mat = d.mat ? d.mat : SharedAssets::get().blue;
    sv->smooth = true;
    for (int j = 0; j + 1 < nv; j++)
        for (int i = 0; i + 1 < nu; i++) {
            uint32_t a = id(i, j), b = id(i + 1, j), cc = id(i + 1, j + 1), e = id(i, j + 1);
            vec2 ua((float)i / (nu - 1), (float)j / (nv - 1)), ub((float)(i + 1) / (nu - 1), ua.y), uc(ub.x, (float)(j + 1) / (nv - 1)),
                ue(ua.x, uc.y);
            add_face_surface(*sv, a, cc, b, ua * 3.0f, uc * 3.0f, ub * 3.0f);
            add_face_surface(*sv, a, e, cc, ua * 3.0f, ue * 3.0f, uc * 3.0f);
            body->add_triangle(a, cc, b);
            body->add_triangle(a, e, cc);
        }
    body->wind_area.assign(body->nodes.size(), d.width * d.height / (nu * nv) * 0.6f);
    body->collision_radius = 0.04f;
    body->finalize();
    body->stabilize(kDefaultDt);
    // tear threshold from strain (after the stiffness has been clamped): strength field held the strain scale
    for (Beam& b : body->beams) b.strength = b.k * b.L0 * d.tear_strain * b.strength;
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->surfaces.push_back(std::move(sv));
    return obj;
}

// ====================================================================================== rally tape
std::unique_ptr<DynamicObject> build_tape_line(World& w, const TapeLineDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    auto& A = SharedAssets::get();
    const vec3 up(0, 1, 0);
    const int P = (int)d.posts.size();
    if (P < 2) return nullptr;
    // ---- stakes: a free stick (bottom + top node, rigid beam) standing in a fixed foot. The foot holds it with an
    // orientation joint (bends, then snaps) and a weak beam; once knocked out the stick falls and is dragged
    // along by the tape instead of vanishing.
    auto obj = std::make_unique<DynamicObject>();
    obj->stick_mats.push_back(d.stake_mat ? d.stake_mat : A.stake);
    std::vector<uint32_t> top(P), low(P);
    std::vector<uint32_t> stake_beams;
    for (int i = 0; i < P; i++) {
        vec3 foot = d.posts[i];
        uint32_t f = body->add_node(foot, d.stake_mass, NF_GROUND | NF_FIXED);
        body->add_frame(f, quat(), d.stake_mass * 0.25f);
        uint32_t t = body->add_node(foot + up * d.height, d.stake_mass, NF_GROUND | NF_CONTACTER);
        uint32_t bot = body->add_node(foot + up * 0.06f, d.stake_mass * 0.5f, NF_GROUND | NF_CONTACTER);
        float r_eff = std::max(d.height * 0.5f, 0.1f);
        body->add_frame(t, quat(), d.stake_mass * r_eff * r_eff);
        float Ic = d.stake_mass * r_eff * r_eff;
        float k_lin = 0.25f * d.stake_mass / (kDefaultDt * kDefaultDt);
        float k_ang = std::min(d.stake_k_ang, 0.2f * Ic / (kDefaultDt * kDefaultDt));
        // light damping: the damper is not capped by the yield force and a stiff one would hold a creeping car
        uint32_t j = body->add_joint(body->info[f].frame, t, body->info[t].frame, k_lin, 0.2f * std::sqrt(k_lin * d.stake_mass), k_ang,
                                     2 * 0.3f * std::sqrt(k_ang * Ic));
        body->joints[j].yield_angle = d.stake_yield_deg * kDeg2Rad;
        body->joints[j].break_torque = d.stake_break_torque;
        body->joints[j].max_bend = 0.3f;
        body->joints[j].break_force = 6.0f; // the linear part too: a push moves the top, 5 cm later it is out
        body->joints[j].max_stretch = 0.05f;
        body->joints[j].radius = d.stake_radius;
        // the stick itself (never breaks) and its seat in the ground (pulls out easily)
        uint32_t sb = body->add_beam(bot, t, 2e5f, 20.0f, 1e9f, 1e9f);
        body->beams[sb].flags |= BF_NO_DEFORM | BF_NO_BREAK;
        stake_beams.push_back(sb);
        uint32_t seat = body->add_beam(f, bot, 2e4f, 10.0f, 1e9f, 1e9f);
        body->beams[seat].strength = 10.0f; // N: pulls out of the soil
        body->beams[seat].flags |= BF_NO_DEFORM | BF_INVISIBLE;
        body->capsules.push_back({bot, t, d.stake_radius + 0.02f, -1});
        top[i] = t;
        // the tape's lower edge where it is tied to the stake
        low[i] = body->add_node(foot + up * (d.height - d.tape_width), d.tape_node_mass, NF_GROUND | NF_CONTACTER);
    }
    // ---- tape columns (top, bottom) along the whole line; stake columns are shared by both spans
    std::vector<std::pair<uint32_t, uint32_t>> cols;
    std::vector<float> col_u;
    float dist = 0;
    for (int i = 0; i + 1 < P; i++) {
        vec3 a = body->nodes[top[i]].p, b = body->nodes[top[i + 1]].p;
        float span = length(b - a);
        int n = std::max(2, (int)std::ceil(span / d.node_spacing));
        if (i == 0) {
            cols.push_back({top[0], low[0]});
            col_u.push_back(0);
        }
        for (int k = 1; k <= n; k++) {
            float t = (float)k / n;
            float u = dist + span * t;
            if (k == n) {
                cols.push_back({top[i + 1], low[i + 1]});
            } else {
                vec3 p = lerp(a, b, t) - up * (d.sag * 4.0f * t * (1.0f - t));
                uint32_t tn = body->add_node(p, d.tape_node_mass, NF_GROUND | NF_CONTACTER);
                uint32_t bn = body->add_node(p - up * d.tape_width, d.tape_node_mass, NF_GROUND | NF_CONTACTER);
                cols.push_back({tn, bn});
            }
            col_u.push_back(u);
        }
        dist += span;
    }
    const float k_tape = 2.0e4f;
    auto link = [&](uint32_t a, uint32_t b, float k, float strain_scale) {
        uint32_t bi = body->add_beam(a, b, k, 0.05f * std::sqrt(k * d.tape_node_mass), strain_scale, 1e9f, BT_ROPE);
        body->beams[bi].flags |= BF_NO_DEFORM;
    };
    const int C = (int)cols.size();
    for (int c = 0; c < C; c++) {
        link(cols[c].first, cols[c].second, k_tape, 1.0f);
        if (c + 1 < C) {
            link(cols[c].first, cols[c + 1].first, k_tape, 1.0f);
            link(cols[c].second, cols[c + 1].second, k_tape, 1.0f);
            link(cols[c].first, cols[c + 1].second, k_tape * 0.3f, 1.5f);
            link(cols[c].second, cols[c + 1].first, k_tape * 0.3f, 1.5f);
        }
        if (c + 2 < C) link(cols[c].first, cols[c + 2].first, k_tape * 0.03f, 3.0f);
    }
    auto sv = std::make_unique<SurfaceVisual>();
    sv->mat = d.tape_mat ? d.tape_mat : A.tape;
    sv->smooth = true;
    const float u_per_m = 1.0f / 0.5f; // one stripe repeat per half metre
    for (int c = 0; c + 1 < C; c++) {
        uint32_t a = cols[c].first, b = cols[c + 1].first, cc = cols[c + 1].second, e = cols[c].second;
        vec2 ua(col_u[c] * u_per_m, 0), ub(col_u[c + 1] * u_per_m, 0), uc(col_u[c + 1] * u_per_m, 1), ue(col_u[c] * u_per_m, 1);
        add_face_surface(*sv, a, cc, b, ua, uc, ub);
        add_face_surface(*sv, a, e, cc, ua, ue, uc);
        body->add_triangle(a, cc, b);
        body->add_triangle(a, e, cc);
    }
    body->wind_area.assign(body->nodes.size(), 0.0f);
    for (auto& cp : cols) body->wind_area[cp.first] = body->wind_area[cp.second] = d.node_spacing * d.tape_width * 0.5f;
    body->collision_radius = 0.03f;
    body->is_static_like = true;
    body->finalize();
    body->stabilize(kDefaultDt);
    // tear thresholds from strain (after the stiffness clamp); the strength field carried the strain scale
    for (Beam& b : body->beams)
        if (b.type == BT_ROPE) b.strength = std::min(b.k * b.L0 * d.tear_strain * b.strength, 12.0f); // N: thin plastic tape
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->surfaces.push_back(std::move(sv));
    for (uint32_t sb : stake_beams) obj->sticks.push_back({sb, d.stake_radius, 0});
    return obj;
}

// ====================================================================================== lathe (hay)
std::unique_ptr<DynamicObject> build_lathe(World& w, const LatheDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    auto& A = SharedAssets::get();
    const int S = std::max(6, d.segments);
    const int R = (int)d.profile.size();
    if (R < 2) return nullptr;
    vec3 ax = normalize(d.axis);
    vec3 e1 = normalize(cross(std::fabs(ax.y) < 0.9f ? vec3(0, 1, 0) : vec3(1, 0, 0), ax));
    vec3 e2 = cross(ax, e1);
    // rings: outer (S nodes), inner (S/2 nodes at half radius), centre; radius 0 -> apex node only
    struct Ring {
        std::vector<uint32_t> outer, inner;
        uint32_t center;
    };
    std::vector<Ring> rings(R);
    std::vector<vec3> pos;
    std::vector<int> ring_of;
    int count = 0;
    for (int r = 0; r < R; r++) count += d.profile[r].y <= 1e-3f ? 1 : 1 + S + S / 2;
    const float nm = d.mass / count;
    int cur_ring = 0;
    auto node = [&](vec3 p) {
        pos.push_back(p);
        ring_of.push_back(cur_ring);
        uint32_t n = body->add_node(p, nm);
        body->info[n].friction = d.friction;
        return n;
    };
    for (int r = 0; r < R; r++) {
        vec3 c = d.base + ax * d.profile[r].x;
        float rad = d.profile[r].y;
        cur_ring = r;
        rings[r].center = node(c);
        if (rad <= 1e-3f) continue;
        for (int i = 0; i < S; i++) {
            float a = 2 * kPi * i / S;
            rings[r].outer.push_back(node(c + (e1 * std::cos(a) + e2 * std::sin(a)) * rad));
        }
        for (int i = 0; i < S / 2; i++) {
            float a = 2 * kPi * (i + 0.5f) / (S / 2);
            rings[r].inner.push_back(node(c + (e1 * std::cos(a) + e2 * std::sin(a)) * (rad * 0.5f)));
        }
    }
    const int N = (int)pos.size();
    // volumetric beams: every pair closer than ~2.3x the local nearest-neighbour spacing (includes the diagonals
    // between rings, which carry shear and torsion)
    std::vector<float> nn(N, 1e9f);
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            if (i != j) nn[i] = std::min(nn[i], length(pos[i] - pos[j]));
    std::vector<std::pair<int, int>> pairs;
    std::vector<int> degree(N, 0);
    // ... and every node to the nearby nodes of the neighbouring rings: rings spaced wider than the in-ring spacing
    // would otherwise hang on axial beams alone, a shear mechanism that folds the body flat under its own weight
    std::vector<float> nn_up(N, 1e9f);
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            if (ring_of[j] == ring_of[i] + 1) nn_up[i] = std::min(nn_up[i], length(pos[i] - pos[j]));
    for (int i = 0; i < N; i++)
        for (int j = i + 1; j < N; j++) {
            float dij = length(pos[i] - pos[j]);
            bool near = dij <= 2.3f * std::max(nn[i], nn[j]);
            if (!near && std::abs(ring_of[i] - ring_of[j]) == 1) {
                int lo = ring_of[i] < ring_of[j] ? i : j;
                near = dij <= 1.6f * nn_up[lo];
            }
            if (near) {
                pairs.push_back({i, j});
                degree[i]++;
                degree[j]++;
            }
        }
    // one stiffness for all beams, inside the explicit budget of the best connected node: a per-node clamp
    // (stabilize) would make the inner rings softer than the ends and the body would sag into a waist
    int max_deg = *std::max_element(degree.begin(), degree.end());
    float k = std::min(d.beams.k, 0.5f * nm / (kDefaultDt * kDefaultDt * std::max(1, max_deg)));
    float damp = std::min(d.beams.d, 0.4f * nm / (kDefaultDt * std::max(1, max_deg)));
    for (auto [i, j] : pairs) {
        uint32_t bi = body->add_beam(i, j, k, damp, d.beams.strength, d.beams.deform);
        body->beams[bi].plastic = d.beams.plastic;
    }
    // surfaces: side between consecutive outer rings (or apex), flat caps at the ends
    auto side = std::make_unique<SurfaceVisual>();
    side->mat = d.side_mat ? d.side_mat : A.bale_side;
    side->smooth = true;
    auto cap = std::make_unique<SurfaceVisual>();
    cap->mat = d.cap_mat ? d.cap_mat : side->mat;
    cap->smooth = false;
    float along = 0;
    for (int r = 0; r + 1 < R; r++) {
        float seg_len = length(vec2(d.profile[r + 1].x - d.profile[r].x, d.profile[r + 1].y - d.profile[r].y));
        float v0 = along * d.v_scale, v1 = (along + seg_len) * d.v_scale;
        along += seg_len;
        const Ring &ra = rings[r], &rb = rings[r + 1];
        for (int i = 0; i < S; i++) {
            int j = i + 1;
            float u0 = (float)i / S * d.u_repeat, u1 = (float)j / S * d.u_repeat;
            j %= S;
            if (!ra.outer.empty() && !rb.outer.empty()) {
                // CCW seen from outside: (along the ring, then along the axis)
                uint32_t a = ra.outer[i], b = ra.outer[j], c = rb.outer[j], e = rb.outer[i];
                add_face_surface(*side, a, b, c, {u0, v0}, {u1, v0}, {u1, v1});
                add_face_surface(*side, a, c, e, {u0, v0}, {u1, v1}, {u0, v1});
                body->add_triangle(a, b, c);
                body->add_triangle(a, c, e);
            } else if (!ra.outer.empty()) { // apex above
                uint32_t a = ra.outer[i], b = ra.outer[j];
                add_face_surface(*side, a, b, rb.center, {u0, v0}, {u1, v0}, {(u0 + u1) * 0.5f, v1});
                body->add_triangle(a, b, rb.center);
            } else if (!rb.outer.empty()) { // apex below
                uint32_t c = rb.outer[j], e = rb.outer[i];
                add_face_surface(*side, ra.center, c, e, {(u0 + u1) * 0.5f, v0}, {u1, v1}, {u0, v1});
                body->add_triangle(ra.center, c, e);
            }
        }
    }
    for (int end = 0; end < 2; end++) {
        const Ring& rg = rings[end ? R - 1 : 0];
        if (rg.outer.empty()) continue;
        for (int i = 0; i < S; i++) {
            int j = (i + 1) % S;
            float a0 = 2 * kPi * i / S, a1 = 2 * kPi * (i + 1) / S;
            vec2 uc(0.5f, 0.5f), ui(0.5f + 0.5f * std::cos(a0), 0.5f + 0.5f * std::sin(a0)), uj(0.5f + 0.5f * std::cos(a1), 0.5f + 0.5f * std::sin(a1));
            // outward: -axis at the first ring, +axis at the last
            if (end) {
                add_face_surface(*cap, rg.center, rg.outer[i], rg.outer[j], uc, ui, uj);
                body->add_triangle(rg.center, rg.outer[i], rg.outer[j]);
            } else {
                add_face_surface(*cap, rg.center, rg.outer[j], rg.outer[i], uc, uj, ui);
                body->add_triangle(rg.center, rg.outer[j], rg.outer[i]);
            }
        }
    }
    body->collision_radius = 0.04f;
    body->sleep_speed = 0.3f; // a soft cylinder keeps rocking a little on its contact nodes
    body->finalize();
    body->stabilize(kDefaultDt);
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->surfaces.push_back(std::move(side));
    if (!cap->corner_node.empty()) obj->surfaces.push_back(std::move(cap));
    return obj;
}

std::unique_ptr<DynamicObject> build_round_bale(World& w, vec3 center, float yaw_deg, const std::string& name, float radius, float width, float mass) {
    auto& A = SharedAssets::get();
    LatheDesc d;
    d.axis = quat::axis_angle(vec3(0, 1, 0), yaw_deg * kDeg2Rad).rotate(vec3(1, 0, 0));
    d.base = center;
    d.profile = {{-width * 0.5f, radius}, {0.0f, radius}, {width * 0.5f, radius}};
    d.segments = 14;
    d.mass = mass;
    d.side_mat = A.bale_side;
    d.cap_mat = A.bale_end;
    d.u_repeat = 3.0f;
    d.v_scale = 0.8f;
    d.friction = 1.2f;
    return build_lathe(w, d, name);
}

std::unique_ptr<DynamicObject> build_standing_bale(World& w, vec3 base, float yaw_deg, float radius, float height, const std::string& name, float mass) {
    auto& A = SharedAssets::get();
    LatheDesc d;
    d.axis = vec3(0, 1, 0);
    d.base = base;
    d.profile = {{0.0f, radius}, {height * 0.5f, radius}, {height, radius}};
    d.segments = 14;
    d.mass = mass;
    d.side_mat = A.bale_side;
    d.cap_mat = A.bale_end;
    d.u_repeat = 3.0f;
    d.v_scale = 0.8f;
    d.friction = 1.2f;
    (void)yaw_deg; // (rotationally symmetric)
    return build_lathe(w, d, name);
}

std::unique_ptr<DynamicObject> build_mesh_prop(World& w, const MeshPropDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    vec3 lo = d.hull_min, hi = d.hull_max;
    // thin boards: at least 8 cm thick, nodes of a thinner box pass through each other's faces
    for (int k = 0; k < 3; k++)
        if (hi[k] - lo[k] < 0.08f) {
            float c = 0.5f * (lo[k] + hi[k]);
            lo[k] = c - 0.04f;
            hi[k] = c + 0.04f;
        }
    const float nm = d.mass / 8.0f;
    vec3 corner[8];
    for (int i = 0; i < 8; i++) {
        corner[i] = vec3(i & 1 ? hi.x : lo.x, i & 2 ? hi.y : lo.y, i & 4 ? hi.z : lo.z);
        body->add_node(d.xform.transform_point(corner[i]), nm, NF_GROUND | NF_CONTACTER);
    }
    const float kk = clamp_k(1.5e6f, nm);
    for (uint32_t a = 0; a < 8; a++)
        for (uint32_t b = a + 1; b < 8; b++) body->add_beam(a, b, kk, 400.0f, 1e7f, 1e6f);
    // faces, wound outwards
    const int quads[6][4] = {{0, 2, 6, 4}, {1, 3, 7, 5}, {0, 1, 5, 4}, {2, 3, 7, 6}, {0, 1, 3, 2}, {4, 5, 7, 6}};
    vec3 c = (lo + hi) * 0.5f;
    for (auto& q : quads)
        for (int t = 0; t < 2; t++) {
            uint32_t a = (uint32_t)q[0], b = (uint32_t)q[1 + t], e = (uint32_t)q[2 + t];
            vec3 n = cross(corner[b] - corner[a], corner[e] - corner[a]);
            if (dot(n, corner[a] - c) < 0) std::swap(b, e);
            body->add_triangle(a, b, e);
        }
    body->collision_radius = 0.03f;
    body->finalize();
    body->stabilize(kDefaultDt);
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    auto rv = std::make_unique<RigidVisual>();
    rv->parts = d.parts;
    // frame along the two longest box edges (a 4 cm edge would wobble)
    vec3 ext = hi - lo;
    int ax[3] = {0, 1, 2};
    std::sort(ax, ax + 3, [&](int i, int j) { return ext[i] > ext[j]; });
    rv->n0 = 0;
    rv->na = 1u << ax[0];
    rv->nb = 1u << ax[1];
    rv->local = inverse(rv->frame(*obj->body)) * d.xform;
    rv->model = d.xform;
    obj->rigid = std::move(rv);
    return obj;
}

std::unique_ptr<DynamicObject> build_haystack(World& w, vec3 base, float height, float radius, const std::string& name, float mass) {
    auto& A = SharedAssets::get();
    LatheDesc d;
    d.axis = vec3(0, 1, 0);
    d.base = base;
    const float H = height, Rr = radius;
    d.profile = {{0.05f, Rr * 0.95f}, {H * 0.22f, Rr}, {H * 0.5f, Rr * 0.9f}, {H * 0.75f, Rr * 0.6f}, {H * 0.92f, Rr * 0.25f}, {H, 0.0f}};
    d.segments = 12;
    d.mass = mass;
    d.side_mat = A.straw;
    d.cap_mat = A.straw;
    d.u_repeat = 4.0f;
    d.v_scale = 0.5f;
    d.beams = {2e5f, 200.0f, 6000.0f, 1e5f, 0.4f};
    d.friction = 1.3f;
    return build_lathe(w, d, name);
}

// ====================================================================================== sheet (triangle elements)
std::unique_ptr<DynamicObject> build_sheet(World& w, const SheetDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    phys::SheetMeshDesc md;
    md.center = d.center;
    md.u = d.u;
    md.v = d.v;
    md.width = d.width;
    md.height = d.height;
    md.nu = d.nu;
    md.nv = d.nv;
    md.shape = d.shape;
    md.hole = d.hole;
    md.curve = d.curve;
    md.dome = d.dome;
    md.clamp = d.clamp;
    md.uv_scale = d.uv_scale;
    const float area = phys::add_sheet_mesh(*body, md);
    body->shell_mat = d.mat;
    if (const char* ml = getenv("BL_SHEET_MAXLEVEL")) body->shell_mat.max_level = atoi(ml); // diagnostics: refinement depth
    body->collision_radius = 0.04f;
    body->finalize();
    body->finalize_shells(d.kg_m2 > 0 ? d.kg_m2 : d.mass / std::max(1e-3f, area), kDefaultDt, d.seed);
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    auto sv = std::make_unique<ShellVisual>();
    sv->mat = d.visual ? d.visual : SharedAssets::get().metal;
    sv->thickness = d.thickness;
    obj->body = w.add_body(std::move(body));
    obj->sheet = std::move(sv);
    // start sagged under its own weight and asleep (a flat sheet dropped under gravity would overshoot)
    if (d.clamp != 0) presettle(*obj->body, w, 0.6f, false);
    return obj;
}

// ====================================================================================== barrel
std::unique_ptr<DynamicObject> build_barrel(World& w, const BarrelDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    int S = std::max(6, d.segments & ~1), R = std::max(3, d.rows / 3 * 3);
    if (const char* e = getenv("BL_BARREL_SEG")) S = std::max(6, atoi(e) & ~1), R = std::max(3, (S / 2) / 3 * 3); // (diagnostics)
    const vec3 Y = normalize(d.rot.rotate(vec3(0, 1, 0))), X = normalize(d.rot.rotate(vec3(1, 0, 0))), Z = cross(X, Y);
    const float H = d.height, Rr = d.radius;
    const uint16_t fl = NF_GROUND | NF_CONTACTER;
    auto at = [&](float r, float ang, float y) { return d.center + Y * y + (X * std::cos(ang) + Z * std::sin(ang)) * r; };
    // the wall: rings of S nodes, 0 at the bottom
    std::vector<uint32_t> wall((R + 1) * S);
    const float dish = getenv("BL_BARREL_DISH") ? (float)atof(getenv("BL_BARREL_DISH")) : d.lid_dish; // (diagnostics)
    const float imp = getenv("BL_BARREL_IMP") ? (float)atof(getenv("BL_BARREL_IMP")) : d.imperfection;
    uint32_t seed = 2166136261u;
    for (char ch : name) seed = (seed ^ (uint8_t)ch) * 16777619u;
    auto noise = [&]() { // (-1..1, the same for the same barrel)
        seed = seed * 1664525u + 1013904223u;
        return (float)(seed >> 8) / 8388608.0f - 1.0f;
    };
    for (int j = 0; j <= R; j++) {
        const float r = Rr + (d.hoops && (j == R / 3 || j == 2 * R / 3) ? d.hoop_depth : 0.0f); // (the rolling hoops pressed out)
        for (int i = 0; i < S; i++) {
            const float ri = r + (j == 0 || j == R ? 0.0f : imp * noise()); // (the chimes true)
            wall[j * S + i] = body->add_node(at(ri, 2.0f * kPi * i / S, (j / (float)R - 0.5f) * H), 0.0f, fl);
        }
    }
    auto wid = [&](int i, int j) { return wall[j * S + ((i % S) + S) % S]; };
    // a triangle wound so its front faces `out`
    auto tri = [&](uint32_t a, uint32_t b, uint32_t c, vec2 ua, vec2 ub, vec2 uc, vec3 out) {
        const vec3 pa = body->nodes[a].p, pb = body->nodes[b].p, pc = body->nodes[c].p;
        if (dot(cross(pb - pa, pc - pa), out) < 0) std::swap(b, c), std::swap(ub, uc);
        body->add_shell(a, b, c, ua, ub, uc);
    };
    const float cell = 2.0f * kPi * Rr / S;
    for (int j = 0; j < R; j++)
        for (int i = 0; i < S; i++) {
            // the 4-8 grid (the diagonals alternate: the pattern bisection keeps), unrolled round the wall
            const uint32_t a = wid(i, j), b = wid(i + 1, j), c = wid(i + 1, j + 1), e = wid(i, j + 1);
            const vec2 ua(i * cell, j * H / R), ub((i + 1) * cell, ua.y), uc(ub.x, (j + 1) * H / R), ue(ua.x, uc.y);
            const vec3 out = normalize(body->nodes[a].p + body->nodes[c].p - d.center * 2.0f - Y * dot(body->nodes[a].p + body->nodes[c].p - d.center * 2.0f, Y));
            if (((i + j) & 1) == 0) tri(a, b, c, ua, ub, uc, out), tri(a, c, e, ua, uc, ue, out);
            else tri(a, b, e, ua, ub, ue, out), tri(b, c, e, ub, uc, ue, out);
        }
    // the ends: rings of 24, 24, 12, 6 nodes and the centre (the outer ring is the wall's), halving where the ring is
    // too short for them
    for (int end = 0; end < 2; end++) {
        const float y = (end ? 0.5f : -0.5f) * H;
        const vec3 out = end ? Y : -Y;
        std::vector<std::vector<uint32_t>> rings;
        std::vector<uint32_t> first;
        for (int i = 0; i < S; i++) first.push_back(wid(i, end ? R : 0));
        rings.push_back(first);
        const float step = cell;
        std::vector<float> radii;
        for (float r = Rr - step; r > 0.6f * step; r -= step) radii.push_back(r);
        int n = S;
        for (float r : radii) {
            while (n > 6 && 2.0f * kPi * r / n < 0.7f * step) n /= 2;
            std::vector<uint32_t> ring;
            // (dished in: a shallow bowl inside the first ring, flat out to the rim: a drum set on another rests on the
            // flat band, not on a slope down to the middle that pushed it off sideways)
            const float r1 = Rr - step, in = dish * std::max(0.0f, 1.0f - (r * r) / (r1 * r1));
            for (int i = 0; i < n; i++) ring.push_back(body->add_node(at(r, 2.0f * kPi * i / n, y) - out * (in + imp * noise()), 0.0f, fl));
            rings.push_back(ring);
        }
        const uint32_t centre = body->add_node(d.center + Y * y - out * dish, 0.0f, fl);
        auto uvp = [&](uint32_t k) { // (the ends drawn in their own place beside the unrolled wall)
            const vec3 q = body->nodes[k].p - d.center;
            return vec2(dot(q, X) + (end ? 3.0f : 4.0f) * Rr + S * cell, dot(q, Z) + Rr);
        };
        for (size_t k = 0; k + 1 < rings.size(); k++) {
            const auto& o = rings[k];
            const auto& in = rings[k + 1];
            const int no = (int)o.size(), ni = (int)in.size();
            if (no == ni) {
                for (int i = 0; i < no; i++) {
                    const uint32_t a = o[i], b = o[(i + 1) % no], c = in[(i + 1) % ni], e = in[i];
                    if (((i + (int)k) & 1) == 0) tri(a, b, c, uvp(a), uvp(b), uvp(c), out), tri(a, c, e, uvp(a), uvp(c), uvp(e), out);
                    else tri(a, b, e, uvp(a), uvp(b), uvp(e), out), tri(b, c, e, uvp(b), uvp(c), uvp(e), out);
                }
            } else { // (halving: each inner segment under two outer ones)
                for (int i = 0; i < ni; i++) {
                    const uint32_t o0 = o[(2 * i) % no], o1 = o[(2 * i + 1) % no], o2 = o[(2 * i + 2) % no], i0 = in[i], i1 = in[(i + 1) % ni];
                    tri(o0, o1, i0, uvp(o0), uvp(o1), uvp(i0), out);
                    tri(o1, i1, i0, uvp(o1), uvp(i1), uvp(i0), out);
                    tri(o1, o2, i1, uvp(o1), uvp(o2), uvp(i1), out);
                }
            }
        }
        const auto& last = rings.back();
        for (size_t i = 0; i < last.size(); i++) {
            const uint32_t a = last[i], b = last[(i + 1) % last.size()];
            tri(a, b, centre, uvp(a), uvp(b), uvp(centre), out);
        }
    }
    // the rings of frame elements: the chimes at the ends, the rolling hoops at a third and two thirds of the height
    const uint16_t chime = body->fem.add_section(make_frame_section("Steel", FrameShape::Rod, d.chime, 0.0f));
    const uint16_t hoop = body->fem.add_section(make_frame_section("Steel", FrameShape::Tube, d.hoop, 0.1f * d.hoop));
    // (elastic: a ring's plastic hinges, flowing in a step taken four times a substep, fed a dented drum's trembling
    // until it tumbled about the pad)
    // The ring's stretch is the sheet's: the membrane along the same edges holds it, and a ring's own axial stiffness
    // (1e8 N/m: a micrometre past the membrane's band, 100 N) fought the projection on
    // every edge: after a dent the drum trembled with a few joules for good and rolled and turned about on its own. A
    // hundredth of it keeps the ring in line and leaves the bending, what the ring is there for.
    for (uint16_t sec : {chime, hoop}) body->fem.sections[sec].Np = 0, body->fem.sections[sec].axial = 0.01f;
    // (optional, the Scene menu's "with FEM rings": the rings keep the ends and hoops round, the wall between dents; a
    // sheet holds no rotation of its nodes, so a ring's nodes turn about its line with nothing but the ring against it)
    std::vector<std::pair<int, uint16_t>> rings;
    if (d.frame_rings) {
        rings = {{0, chime}, {R, chime}};
        if (d.hoops) rings.push_back({R / 3, hoop}), rings.push_back({2 * R / 3, hoop});
    }
    for (auto [j, sec] : rings) {
        const float m = body->fem.sections[sec].mass_per_m() * cell;
        for (int i = 0; i < S; i++) {
            body->nodes[wid(i, j)].mass += m * 0.5f;
            body->nodes[wid(i + 1, j)].mass += m * 0.5f;
            body->fem.add_element(wid(i, j), wid(i + 1, j), sec);
        }
    }
    body->shell_mat = d.mat;
    if (const char* ml = getenv("BL_SHEET_MAXLEVEL")) body->shell_mat.max_level = atoi(ml); // diagnostics: refinement depth
    body->shell_min_shift = d.min_shift;
    // node against triangle with the other bodies (their sphere contacts, phys/sphere_contacts.cpp, are off:
    // BL_BARREL_SPHERES=1); a ball meets it as a sphere (node against triangle, the drum's nodes slipped between the
    // ball's faces and held it stuck in the wall)
    body->sphere_contacts = getenv("BL_BARREL_SPHERES") != nullptr;
    body->sphere_target = true;
    if (const char* ms = getenv("BL_BARREL_SHIFT")) body->shell_min_shift = atoi(ms); // (diagnostics)
    // at rest it stays: no push out of the ground up to 3 mm deep (each touch kicked a node off again, and the drum
    // drummed and rocked), and the ground's rolling resistance and the metal's damping once slow (SoftBody::rest_damp)
    body->contact_slop = 0.003f;
    body->contact_push_max = 0.5f;
    body->rest_damp = 3.0f;
    if (const char* e = getenv("BL_BARREL_SLOP")) body->contact_slop = (float)atof(e); // (diagnostics)
    if (const char* e = getenv("BL_BARREL_PUSHMAX")) body->contact_push_max = (float)atof(e);
    body->rest_vib_damp = d.frame_rings ? 40.0f : 10.0f; // (with frame rings the skin trembles harder: see fem_every_step)
    body->rest_roll = 0.4f; // (a dented drum on concrete: rolling at 2 m/s it stops in 5 m)
    // (the ground's grip at rest: eight times; a drum crushed flat by a fall trembles a while, and on its friction alone
    // its trembling nodes walked it about at a few cm/s)
    body->rest_friction = getenv("BL_BARREL_RESTFRIC") ? (float)atof(getenv("BL_BARREL_RESTFRIC")) : 8.0f;
    // the rings share the sheet's nodes: solved in its every short step (once a substep, the sheet's forces on them held
    // over its short steps and set right a substep late, a dented drum with rings trembled, walked and threw itself about)
    body->fem_every_step = true;
    body->energy_guard = d.frame_rings && !getenv("BL_NOGUARD"); // (see SoftBody::energy_guard: the sheet alone needs none)
    if (const char* e = getenv("BL_BARREL_ROLLRES")) body->rest_roll = (float)atof(e);
    if (const char* e = getenv("BL_BARREL_REST")) body->rest_damp = (float)atof(e);
    if (const char* e = getenv("BL_BARREL_RESTVIB")) body->rest_vib_damp = (float)atof(e);
    body->collision_radius = 0.04f;
    for (size_t i = 0; i < body->nodes.size(); i++) body->nodes[i].inv_mass = body->nodes[i].mass > 0 ? 1.0f / body->nodes[i].mass : 0.0f;
    body->finalize();
    body->finalize_shells(d.kg_m2, kDefaultDt, 1, true);
    body->fem.finalize(*body);
    // its initial motion
    const vec3 c = body->center_of_mass();
    for (Node& n : body->nodes) n.v = d.velocity + cross(d.spin, n.p - c);
    for (vec3& x : body->fem.w) x = d.spin;
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    auto sv = std::make_unique<ShellVisual>();
    sv->mat = d.visual ? d.visual : SharedAssets::get().metal;
    sv->thickness = 0.003f;
    sv->crease_cos = getenv("BL_NOCREASE") ? -2.0f : 0.7071f; // (the ends meet the wall at a sharp edge; dents stay smooth)
    obj->sheet = std::move(sv);
    if (!rings.empty()) {
        obj->frame = std::make_unique<FrameVisual>();
        obj->frame->mat = obj->sheet->mat;
    }
    obj->body = w.add_body(std::move(body));
    return obj;
}

// ====================================================================================== plate
std::unique_ptr<DynamicObject> build_plate(World& w, const PlateDesc& d, const std::string& name) {
    auto body = std::make_unique<SoftBody>();
    body->name = name;
    const int nu = std::max(2, d.nu), nv = std::max(2, d.nv);
    const float nm = d.mass / (nu * nv * 2);
    vec3 U = normalize(d.u), V = normalize(d.v), N = normalize(cross(U, V));
    auto id = [&](int i, int j, int l) { return (uint32_t)((l * nv + j) * nu + i); };
    for (int l = 0; l < 2; l++)
        for (int j = 0; j < nv; j++)
            for (int i = 0; i < nu; i++) {
                bool border = i == 0 || j == 0 || i == nu - 1 || j == nv - 1;
                bool fixed = (d.clamp_edges && border && (d.clamp_bottom || j > 0)) || (d.hang_top && j == nv - 1);
                vec3 p = d.center + U * (((float)i / (nu - 1) - 0.5f) * d.width) + V * (((float)j / (nv - 1) - 0.5f) * d.height) +
                         N * ((l - 0.5f) * d.thickness);
                body->add_node(p, nm, NF_GROUND | NF_CONTACTER | (fixed ? NF_FIXED : 0));
            }
    auto beam = [&](uint32_t a, uint32_t b, float scale) { body->add_beam(a, b, d.k * scale, d.damp, 1e12f, 1e12f); };
    for (int l = 0; l < 2; l++)
        for (int j = 0; j < nv; j++)
            for (int i = 0; i < nu; i++) {
                if (i + 1 < nu) beam(id(i, j, l), id(i + 1, j, l), 1.0f);
                if (j + 1 < nv) beam(id(i, j, l), id(i, j + 1, l), 1.0f);
                if (i + 1 < nu && j + 1 < nv) {
                    beam(id(i, j, l), id(i + 1, j + 1, l), 0.7f);
                    beam(id(i + 1, j, l), id(i, j + 1, l), 0.7f);
                }
            }
    // through-thickness links: verticals + diagonals (these carry the bending)
    for (int j = 0; j < nv; j++)
        for (int i = 0; i < nu; i++) {
            beam(id(i, j, 0), id(i, j, 1), 1.0f);
            if (i + 1 < nu) {
                beam(id(i, j, 0), id(i + 1, j, 1), 0.8f);
                beam(id(i + 1, j, 0), id(i, j, 1), 0.8f);
            }
            if (j + 1 < nv) {
                beam(id(i, j, 0), id(i, j + 1, 1), 0.8f);
                beam(id(i, j + 1, 0), id(i, j, 1), 0.8f);
            }
        }
    auto sv = std::make_unique<SurfaceVisual>();
    sv->mat = d.mat ? d.mat : SharedAssets::get().metal;
    sv->smooth = true;
    for (int l = 0; l < 2; l++)
        for (int j = 0; j + 1 < nv; j++)
            for (int i = 0; i + 1 < nu; i++) {
                uint32_t a = id(i, j, l), b = id(i + 1, j, l), cc = id(i + 1, j + 1, l), e = id(i, j + 1, l);
                vec2 ua((float)i / (nu - 1), (float)j / (nv - 1)), ub((float)(i + 1) / (nu - 1), ua.y), uc(ub.x, (float)(j + 1) / (nv - 1)),
                    ue(ua.x, uc.y);
                if (l == 1) { // front (+N)
                    add_face_surface(*sv, a, b, cc, ua, ub, uc);
                    add_face_surface(*sv, a, cc, e, ua, uc, ue);
                    body->add_triangle(a, b, cc);
                    body->add_triangle(a, cc, e);
                } else {      // back (-N)
                    add_face_surface(*sv, a, cc, b, ua, uc, ub);
                    add_face_surface(*sv, a, e, cc, ua, ue, uc);
                    body->add_triangle(a, cc, b);
                    body->add_triangle(a, e, cc);
                }
            }
    body->collision_radius = 0.05f;
    body->ductile = d.ductile;
    body->finalize();
    body->stabilize(kDefaultDt);
    // yield / fracture as strains of the (stability clamped) stiffness
    for (Beam& b : body->beams) {
        b.max_pos = b.k * b.L0 * d.yield_strain;
        b.max_neg = -b.max_pos;
        b.strength = b.k * b.L0 * d.break_strain;
    }
    auto obj = std::make_unique<DynamicObject>();
    obj->name = name;
    obj->body = w.add_body(std::move(body));
    obj->surfaces.push_back(std::move(sv));
    return obj;
}

// ====================================================================================== presettle
void presettle(SoftBody& b, World& w, float seconds, bool compensate) {
    PROFILE_ZONE("Presettle");
    const float dt = w.settings.dt;
    const vec3 g = w.settings.gravity;
    const int steps = (int)(seconds / dt);
    std::vector<Node> start = b.nodes;
    std::vector<Frame> start_f = b.frames;
    // authored joint targets
    std::vector<vec3> auth_off;
    std::vector<quat> auth_rot;
    for (auto& j : b.joints) {
        auth_off.push_back(j.local_offset);
        auth_rot.push_back(j.rel_rot);
    }
    bool deform = b.allow_deform, brk = b.allow_break;
    b.allow_deform = false;
    b.allow_break = false;
    // static geometry the body may rest on (bridge piers, abutments): simple projection contacts
    std::vector<int> box_ids, cyl_ids;
    {
        AABB a = b.aabb;
        a.expand(1.0f);
        w.statics.query(a, box_ids, cyl_ids);
    }
    int iters = compensate ? 3 : 1;
    for (int it = 0; it < iters; it++) {
        b.nodes = start;
        b.frames = start_f;
        const int sub = 1 << b.dt_shift(); // refined sheets need shorter steps
        for (int s = 0; s < steps * sub; s++) {
            b.clear_forces(g);
            b.compute_beam_forces();
            if (!b.joints.empty()) b.compute_joint_forces();
            if (!b.shells.empty()) b.compute_shell_forces(dt / sub, s % sub, sub);
            b.integrate(dt / sub);
            if (!box_ids.empty() || !cyl_ids.empty())
                for (size_t i = 0; i < b.nodes.size(); i++) {
                    Node& n = b.nodes[i];
                    if (n.inv_mass <= 0 || !(b.info[i].flags & NF_GROUND)) continue;
                    ContactInfo ci;
                    if (w.statics.collide_point(n.p, 0.0f, box_ids.data(), (int)box_ids.size(), cyl_ids.data(), (int)cyl_ids.size(), false, ci)) {
                        n.p += ci.normal * ci.depth;
                        float vn = dot(n.v, ci.normal);
                        if (vn < 0) n.v -= ci.normal * vn;
                    }
                }
            // heavy artificial damping for fast convergence
            const float damp = sub == 1 ? 0.995f : std::pow(0.995f, 1.0f / sub);
            for (auto& n : b.nodes) n.v *= damp;
            for (auto& f : b.frames) f.w *= 0.99f;
        }
        if (compensate && it + 1 < iters) {
            for (size_t k = 0; k < b.joints.size(); k++) {
                Joint& j = b.joints[k];
                const Frame& pf = b.frames[j.parent_frame];
                vec3 settled_off = conj(pf.q).rotate(b.nodes[j.child_node].p - b.nodes[pf.node].p);
                j.local_offset += auth_off[k] - settled_off;
                if (j.child_frame >= 0) {
                    quat settled_rel = normalize(conj(pf.q) * b.frames[j.child_frame].q);
                    // rel' = authored * conj(settled) * rel
                    j.rel_rot = normalize(auth_rot[k] * conj(settled_rel) * j.rel_rot);
                }
            }
        }
    }
    if (compensate) {
        // keep the authored geometry as the start pose; the rest params now hold it against gravity
        b.nodes = start;
        b.frames = start_f;
    }
    for (auto& n : b.nodes) n.v = vec3(0);
    for (auto& f : b.frames) f.w = vec3(0);
    b.clear_shell_events();
    b.allow_deform = deform;
    b.allow_break = brk;
    b.compute_aabb();
    b.sleeping = true;
}

} // namespace bl
