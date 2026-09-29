// Collision volumes (see CollisionVolume in softbody.h): the hull built once, placed on its anchors every substep, and
// the forces it takes spread onto them. The contacts themselves are World::collide_volumes.
#include "phys/softbody.h"
#include "core/util.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace bl::phys {

int SoftBody::add_volume(const std::string& name, const std::vector<uint32_t>& anchors, const std::vector<vec3>& points, float break_rms) {
    std::vector<uint32_t> an;
    for (uint32_t a : anchors)
        if (a < nodes.size() && std::find(an.begin(), an.end(), a) == an.end()) an.push_back(a);
    if (an.size() < 3 || points.size() < 4 || points.size() > 64) return -1;
    CollisionVolume cv;
    cv.name = name;
    cv.anchors = an;
    cv.break_rms = break_rms > 0 ? break_rms : 0.12f;
    vec3 c0(0);
    for (uint32_t a : an) c0 += nodes[a].p;
    c0 = c0 / (float)an.size();
    for (uint32_t a : an) cv.rest.push_back(nodes[a].p - c0);
    for (vec3 p : points) cv.verts.push_back(p - c0);
    // the hull's faces: the planes through three of its points with all the others behind (brute force: a few dozen)
    const std::vector<vec3>& V = cv.verts;
    const int n = (int)V.size();
    float span = 0;
    for (const vec3& p : V) span = std::max(span, length(p));
    const float eps = 1e-4f * std::max(1.0f, span);
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            for (int k = j + 1; k < n; k++) {
                vec3 nn = cross(V[j] - V[i], V[k] - V[i]);
                const float l = length(nn);
                if (l < 1e-8f) continue;
                nn = nn / l;
                float d = dot(nn, V[i]);
                int above = 0, below = 0;
                for (int m = 0; m < n; m++) {
                    const float s = dot(nn, V[m]) - d;
                    above += s > eps, below += s < -eps;
                }
                if (above && below) continue;
                if (above) nn = -nn, d = -d; // (all behind: the outward normal)
                bool dup = false;
                for (const vec4& pl : cv.planes) dup |= dot(pl.xyz(), nn) > 0.9999f && std::fabs(pl.w - d) < eps;
                if (!dup) cv.planes.push_back(vec4(nn, d));
            }
    if (cv.planes.size() < 4) return -1;
    // each face's vertices in order round its centre (the debug view's outline)
    for (const vec4& pl : cv.planes) {
        const vec3 nn = pl.xyz();
        std::vector<uint8_t> on;
        vec3 fc(0);
        for (int m = 0; m < n; m++)
            if (std::fabs(dot(nn, V[m]) - pl.w) <= eps) on.push_back((uint8_t)m), fc += V[m];
        if (on.size() < 3) continue;
        fc = fc / (float)on.size();
        const vec3 u = normalize(any_perpendicular(nn)), t = cross(nn, u);
        std::sort(on.begin(), on.end(), [&](uint8_t a, uint8_t b) {
            const vec3 da = V[a] - fc, db = V[b] - fc;
            return std::atan2(dot(da, t), dot(da, u)) < std::atan2(dot(db, t), dot(db, u));
        });
        cv.faces.push_back(on);
    }
    volumes.push_back(std::move(cv));
    return (int)volumes.size() - 1;
}

void SoftBody::place_volumes() {
    for (CollisionVolume& cv : volumes) {
        if (cv.broken) {
            cv.placed = false;
            continue;
        }
        const size_t na = cv.anchors.size();
        vec3 c(0), v(0);
        float m = 0;
        for (uint32_t a : cv.anchors) c += nodes[a].p, v += nodes[a].v, m += nodes[a].mass;
        c = c / (float)na, v = v / (float)na;
        cv.cur.resize(na);
        mat3 A(vec3(0), vec3(0), vec3(0)), J(vec3(0), vec3(0), vec3(0));
        for (size_t i = 0; i < na; i++) {
            const vec3 d = nodes[cv.anchors[i]].p - c;
            cv.cur[i] = d;
            A = A + outer(d, cv.rest[i]);
            J = J + mat3::diag(vec3(dot(d, d))) + outer(d, d) * -1.0f;
        }
        // the rotation of the best fit: Mueller et al. 2016, from the last step's (a few iterations: it moves little)
        quat q = cv.q;
        for (int it = 0; it < 20; it++) {
            const mat3 R = to_mat3(q);
            const vec3 om = cross(R.c[0], A.c[0]) + cross(R.c[1], A.c[1]) + cross(R.c[2], A.c[2]);
            const float den = std::fabs(dot(R.c[0], A.c[0]) + dot(R.c[1], A.c[1]) + dot(R.c[2], A.c[2])) + 1e-9f;
            const vec3 wv = om / den;
            const float wl = length(wv);
            if (!(wl > 1e-7f)) break;
            q = normalize(quat::axis_angle(wv / wl, wl) * q);
        }
        if (!std::isfinite(q.x + q.y + q.z + q.w)) q = quat();
        cv.q = q;
        const mat3 R = to_mat3(q);
        float r2 = 0;
        vec3 L(0);
        for (size_t i = 0; i < na; i++) {
            r2 += length2(cv.cur[i] - R * cv.rest[i]);
            L += cross(cv.cur[i], nodes[cv.anchors[i]].v - v);
        }
        cv.rms = std::sqrt(r2 / (float)na);
        if (!(cv.rms <= cv.break_rms)) { // crushed or torn apart: off for good
            cv.broken = true, cv.placed = false;
            continue;
        }
        // (a flat or thin set of anchors: its spread nearly singular across it; a little of its size keeps it invertible)
        float sz = 0;
        for (const vec3& d : cv.cur) sz += length2(d);
        J = J + mat3::diag(vec3(0.05f * sz / (float)na + 1e-6f));
        cv.Jinv = inverse(J);
        cv.c = c, cv.v = v, cv.w = cv.Jinv * L, cv.mass = m;
        if (!std::isfinite(cv.w.x + cv.w.y + cv.w.z)) cv.w = vec3(0);
        cv.wverts.resize(cv.verts.size());
        vec3 mn(1e30f), mx(-1e30f);
        for (size_t k = 0; k < cv.verts.size(); k++) {
            const vec3 p = c + R * cv.verts[k];
            cv.wverts[k] = p, mn = vmin(mn, p), mx = vmax(mx, p);
        }
        cv.mn = mn, cv.mx = mx;
        cv.wplanes.resize(cv.planes.size());
        for (size_t k = 0; k < cv.planes.size(); k++) {
            const vec3 nn = R * cv.planes[k].xyz();
            cv.wplanes[k] = vec4(nn, cv.planes[k].w + dot(nn, c));
        }
        cv.placed = true;
    }
}

int SoftBody::find_volume_parts() {
    static const bool off = getenv("BL_VOLPARTS_OFF") != nullptr; // (diagnostics: the volumes against the other bodies alone)
    if (volumes.empty() || off) return 0;
    // the parts: components with triangles (panels, not the suspension's tubes) with a part's side of a mount in them
    const int nc = fem.components();
    std::vector<char> part(std::max(nc, 0), 0);
    for (const FrameMount& m : fem.mounts)
        if (const int c = fem.component_of(m.b); c >= 0 && fem.component_tris(c) > 0) part[c] = 1;
    std::vector<int> comp(nodes.size(), -1);
    for (uint32_t i = 0; i < nodes.size(); i++) comp[i] = fem.component_of(i);
    static const bool dbg = getenv("BL_VOLDBG") != nullptr; // (diagnostics: the components and what each volume holds off)
    if (dbg)
        for (int c = 0; c < nc; c++) {
            vec3 mn(1e30f), mx(-1e30f);
            for (uint32_t i = 0; i < nodes.size(); i++)
                if (comp[i] == c) mn = vmin(mn, nodes[i].p), mx = vmax(mx, nodes[i].p);
            log_info("  component %d: %d nodes, %d members, %d tris%s, (%.2f %.2f %.2f) - (%.2f %.2f %.2f)", c, fem.component_nodes(c), fem.component_members(c),
                     fem.component_tris(c), part[c] ? ", a part" : "", mn.x, mn.y, mn.z, mx.x, mx.y, mx.z);
        }
    place_volumes();
    int inside = 0;
    for (CollisionVolume& cv : volumes) {
        cv.parts.clear();
        std::vector<char> own(part.size(), 0);
        for (uint32_t a : cv.anchors)
            if (comp[a] >= 0) own[comp[a]] = 1;
        int in = 0;
        for (uint32_t i = 0; i < nodes.size(); i++) {
            if (comp[i] < 0 || !part[comp[i]] || own[comp[i]] || !(info[i].flags & NF_CONTACTER)) continue;
            int face;
            if (cv.placed && cv.depth(nodes[i].p, face) < 0.0f) { // (inside as built: it would be flung out)
                in++;
                continue;
            }
            cv.parts.push_back(i);
        }
        inside += in;
        if (dbg) {
            log_info("  volume '%s': holds off %d nodes of the parts, %d inside it left out", cv.name.c_str(), (int)cv.parts.size(), in);
            std::vector<float> gap(part.size(), 1e30f);
            for (uint32_t i : cv.parts) {
                int face;
                gap[comp[i]] = std::min(gap[comp[i]], cv.depth(nodes[i].p, face));
            }
            for (size_t c = 0; c < gap.size(); c++)
                if (gap[c] < 0.1f) log_info("    component %d: %.3f m off it", (int)c, gap[c]);
        }
    }
    return inside;
}

void SoftBody::remap_node_refs(const std::function<int64_t(uint32_t)>& map) {
    for (CollisionVolume& cv : volumes) {
        for (uint32_t& a : cv.anchors) {
            const int64_t k = map(a);
            if (k < 0) cv.broken = true, cv.placed = false;
            a = k < 0 ? 0 : (uint32_t)k;
        }
        size_t n = 0;
        for (uint32_t i : cv.parts)
            if (const int64_t k = map(i); k >= 0) cv.parts[n++] = (uint32_t)k;
        cv.parts.resize(n);
    }
    if (grab_offsets.size() != grab_nodes.size() || grab_w.size() != grab_nodes.size()) return;
    size_t n = 0;
    for (size_t j = 0; j < grab_nodes.size(); j++)
        if (const int64_t k = map(grab_nodes[j]); k >= 0) grab_nodes[n] = (uint32_t)k, grab_offsets[n] = grab_offsets[j], grab_w[n] = grab_w[j], n++;
    grab_nodes.resize(n), grab_offsets.resize(n), grab_w.resize(n);
}

void SoftBody::push_volume(CollisionVolume& cv, vec3 p, vec3 f) {
    if (!cv.placed || cv.anchors.empty() || force.size() != nodes.size()) return;
    cv.hits++, cv.peak = std::max(cv.peak, length(f));
    // the force and its moment about the centre, spread as over a rigid body: each anchor f / N + om x d_i, om = J^-1 tau
    const float inv_n = 1.0f / (float)cv.anchors.size();
    const vec3 om = cv.Jinv * cross(p - cv.c, f);
    for (size_t i = 0; i < cv.anchors.size(); i++) force[cv.anchors[i]] += f * inv_n + cross(om, cv.cur[i]);
}

} // namespace bl::phys
