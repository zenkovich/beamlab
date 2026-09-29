#include "phys/fem_shell.h"

#include <cmath>

namespace bl::phys {

uint32_t ShellMesher::node(vec3 p) {
    const auto key = std::make_tuple(std::lround(p.x * 1000.0), std::lround(p.y * 1000.0), std::lround(p.z * 1000.0));
    if (auto it = m_at.find(key); it != m_at.end()) return it->second;
    const uint32_t id = m_b.add_node(p, 0.0f, m_flags);
    m_at[key] = id;
    return id;
}

uint32_t ShellMesher::tri(uint32_t a, uint32_t b, uint32_t c, uint16_t section, bool collide, int32_t tag) {
    int32_t coll = -1;
    if (collide) {
        Triangle t;
        t.a = a, t.b = b, t.c = c;
        t.two_sided = true;
        const vec3 pa = m_b.nodes[a].p, pb = m_b.nodes[b].p, pc = m_b.nodes[c].p;
        t.rest_edge2 = std::max(length2(pb - pa), std::max(length2(pc - pb), length2(pa - pc)));
        coll = (int32_t)m_b.tris.size();
        m_b.tris.push_back(t);
    }
    const uint32_t ti = m_b.fem.add_tri(a, b, c, section, tag, coll);
    const float area = 0.5f * length(cross(m_b.nodes[b].p - m_b.nodes[a].p, m_b.nodes[c].p - m_b.nodes[a].p));
    const float m = m_b.fem.shell_sections[section].mass_per_m2() * area;
    m_b.fem.tris[ti].mass = m;
    for (uint32_t v : {a, b, c}) m_b.nodes[v].mass += m / 3.0f;
    return ti;
}

std::vector<uint32_t> ShellMesher::grid(vec3 origin, vec3 du, vec3 dv, int nu, int nv, uint16_t section, bool collide) {
    std::vector<uint32_t> id((size_t)(nu + 1) * (nv + 1));
    for (int j = 0; j <= nv; j++)
        for (int i = 0; i <= nu; i++) id[(size_t)j * (nu + 1) + i] = node(origin + du * (float)i + dv * (float)j);
    for (int j = 0; j < nv; j++)
        for (int i = 0; i < nu; i++) {
            const uint32_t n00 = id[(size_t)j * (nu + 1) + i], n10 = id[(size_t)j * (nu + 1) + i + 1];
            const uint32_t n01 = id[(size_t)(j + 1) * (nu + 1) + i], n11 = id[(size_t)(j + 1) * (nu + 1) + i + 1];
            if ((i + j) & 1) tri(n00, n10, n11, section, collide), tri(n00, n11, n01, section, collide);
            else tri(n00, n10, n01, section, collide), tri(n10, n11, n01, section, collide);
        }
    return id;
}

void ShellMesher::finish() {
    for (size_t i = 0; i < m_b.nodes.size(); i++) {
        Node& n = m_b.nodes[i];
        n.inv_mass = (m_b.info[i].flags & NF_FIXED) || n.mass <= 0 ? 0.0f : 1.0f / n.mass;
    }
}

} // namespace bl::phys
