// Welds: a sheet held on a frame at points (SoftBody::Weld), the pull on each spread over the sheet's nodes round it.
#include "phys/softbody.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace bl::phys {

bool SoftBody::add_weld(uint32_t anchor, uint32_t node, float radius, float brk, float k, float h, int anchor2, float t) {
    if (anchor >= nodes.size() || node >= nodes.size() || node_shells.size() != nodes.size() || node_shells[node].empty()) return false;
    if (anchor2 < 0 || anchor2 >= (int)nodes.size() || !(t > 0)) anchor2 = (int)anchor, t = 0;
    radius = std::max(radius, 1e-3f);
    // the sheet's nodes within the radius, reached across its triangles (not a panel beside it across a gap)
    const vec3 p0 = nodes[node].p;
    std::vector<uint32_t> group{node}, stack{node};
    std::vector<float> w{1.0f};
    while (!stack.empty()) {
        const uint32_t v = stack.back();
        stack.pop_back();
        for (uint32_t si : node_shells[v])
            for (uint32_t u : shells[si].n) {
                if (std::find(group.begin(), group.end(), u) != group.end()) continue;
                const float d = length(nodes[u].p - p0);
                if (d >= radius) continue;
                group.push_back(u), w.push_back((1.0f - d / radius) * (1.0f - d / radius));
                stack.push_back(u);
            }
    }
    const uint32_t a2 = (uint32_t)anchor2;
    const float ia = (1 - t) * (1 - t) * nodes[anchor].inv_mass + t * t * nodes[a2].inv_mass; // (the member's point)
    float ws = 0, inv_m = ia;
    for (float x : w) ws += x;
    vec3 c(0);
    for (size_t i = 0; i < group.size(); i++) w[i] /= ws, c += nodes[group[i]].p * w[i], inv_m += w[i] * w[i] * nodes[group[i]].inv_mass;
    if (!(inv_m > 0)) return false;
    Weld wd;
    wd.anchor = anchor;
    wd.anchor2 = a2;
    wd.t = t;
    wd.slot = fem.slot(anchor);
    wd.first = (uint32_t)weld_nodes.size();
    wd.count = (uint32_t)group.size();
    const vec3 off = nodes[anchor].p * (1 - t) + nodes[a2].p * t - c;
    wd.off = wd.slot >= 0 && wd.slot < (int)fem.q.size() ? conj(fem.q[wd.slot]).rotate(off) : off;
    // (explicit: a quarter of the stable stiffness on the effective mass of the pair, anchor and centre)
    const float m_eff = 1.0f / inv_m, k_max = 0.25f * m_eff / (h * h);
    wd.k = k > 0 ? std::min(k, k_max) : k_max;
    // (held off the anchor: the pull's moment turns the anchor, a stiffness k |off|^2 on its rotation, explicit - kept
    // within a tenth of what the node's rotational inertia takes in a step; a lamp 20 cm off a tube's node at the
    // full stiffness rang the node to pieces)
    static const bool no_cap = getenv("BL_WELD_NOCAP") != nullptr; // (diagnostics)
    if (!no_cap && wd.slot >= 0 && wd.slot < (int)fem.inertia.size() && length2(off) > 1e-8f) wd.k = std::min(wd.k, 0.1f * fem.inertia[wd.slot] / (h * h * length2(off)));
    wd.c = 2.0f * 0.33f * std::sqrt(wd.k * m_eff);
    wd.brk = brk;
    weld_nodes.insert(weld_nodes.end(), group.begin(), group.end());
    weld_w.insert(weld_w.end(), w.begin(), w.end());
    welds.push_back(wd);
    return true;
}

void SoftBody::compute_weld_forces(bool fem_torque) {
    Node* nd = nodes.data();
    vec3* F = force.data();
    const uint32_t* gn = weld_nodes.data();
    const float* gw = weld_w.data();
    for (Weld& wd : welds) {
        if (wd.broken) continue;
        vec3 c(0), cv(0);
        for (uint32_t i = wd.first; i < wd.first + wd.count; i++) c += nd[gn[i]].p * gw[i], cv += nd[gn[i]].v * gw[i];
        const vec3 off = wd.slot >= 0 && wd.slot < (int)fem.q.size() ? fem.q[wd.slot].rotate(wd.off) : wd.off;
        const float t = wd.t;
        const vec3 ap = nd[wd.anchor].p * (1 - t) + nd[wd.anchor2].p * t, av = nd[wd.anchor].v * (1 - t) + nd[wd.anchor2].v * t;
        const vec3 d = ap - off - c; // (where the anchor holds the centre, from where it is)
        const vec3 fs = d * wd.k;
        if (wd.brk > 0 && length2(fs) > wd.brk * wd.brk) {
            wd.broken = true;
            stats.broken_welds++;
            static const bool dbg = getenv("BL_WELDDBG") != nullptr; // (which weld let go, and how far it was pulled)
            if (dbg)
                printf("weld %d broke: anchor %u node %u (%.2f %.2f %.2f) pulled %.1f mm, %.0f N of %.0f\n", (int)(&wd - welds.data()), wd.anchor, gn[wd.first],
                       nd[gn[wd.first]].p.x, nd[gn[wd.first]].p.y, nd[gn[wd.first]].p.z, length(d) * 1000.0f, length(fs), wd.brk);
            continue;
        }
        const vec3 f = fs + (av - cv) * wd.c; // (on the sheet; the anchor takes it back)
        F[wd.anchor] -= f * (1 - t);
        if (t > 0) F[wd.anchor2] -= f * t;
        // (and the moment: -dU/dtheta of 1/2 k |ap - q off - c|^2 is (q off) x f on the anchor's rotation)
        static const bool no_torque = getenv("BL_WELD_NOTORQUE") != nullptr; // (diagnostics)
        if (fem_torque && !no_torque && wd.slot >= 0 && wd.slot < (int)fem.torque.size()) fem.torque[wd.slot] += cross(off, f);
        for (uint32_t i = wd.first; i < wd.first + wd.count; i++) F[gn[i]] += f * gw[i];
    }
}

} // namespace bl::phys
