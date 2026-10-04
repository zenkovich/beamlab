#include "phys/frame_fem.h"
#include <string>

#include "phys/softbody.h"
#include "core/profiler.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace bl::phys {

namespace {
// a triangle's local dofs (per corner u, v, w, theta_x, theta_y, theta_z) in its stiffness's two parts, the membrane's
// (u, v, theta_z) and the plate's (w, theta_x, theta_y)
constexpr int kMemDof[3] = {0, 1, 5}, kPlateDof[3] = {2, 3, 4};
// the stretch a triangle bisected `level` times tears at, against the authored one's (sqrt(2) a level: the tear's band)
inline float tear_band(int level) {
    static const float table[4] = {1.0f, 1.41421356f, 2.0f, 2.82842712f};
    return table[std::min(level, 3)];
}
} // namespace

// ------------------------------------------------------------------------------------------------ materials, sections
static const FrameMaterial kMaterials[] = {
    // name, E, G, density, yield, elongation at fracture
    {"Steel", 2.10e11f, 8.1e10f, 7850.0f, 3.5e8f, 0.20f},      // mild structural steel (S355)
    {"MildSteel", 2.10e11f, 8.1e10f, 7850.0f, 1.8e8f, 0.40f},  // deep-drawing sheet steel (DC04): a car body's, yields early, stretches far
    {"Chromoly", 2.05e11f, 8.0e10f, 7850.0f, 4.6e8f, 0.15f},   // 4130, normalized: roll cages, race frames
    {"ChromolyHT", 2.05e11f, 8.0e10f, 7850.0f, 9.0e8f, 0.12f}, // 4130, quenched and tempered: an off-road racer's cage
    {"Aluminium", 6.9e10f, 2.6e10f, 2700.0f, 2.75e8f, 0.10f},  // 6061-T6
    {"Titanium", 1.14e11f, 4.4e10f, 4430.0f, 8.8e8f, 0.10f},   // Ti-6Al-4V
    {"Carbon", 1.2e11f, 2.5e10f, 1600.0f, 6.0e8f, 0.015f},     // CFRP tube: strong, light and brittle
    {"Wood", 1.1e10f, 7.0e8f, 500.0f, 4.0e7f, 0.02f},          // pine along the grain
    {"Plastic", 1.6e9f, 5.8e8f, 950.0f, 3.0e7f, 0.08f},        // polypropylene: a bumper's absorber and brackets, a mirror's arm
    {"SpringSteel", 2.06e11f, 7.9e10f, 7850.0f, 1.2e9f, 0.06f}, // 51CrV4, quenched and tempered: torsion and anti-roll bars
    {"Maraging", 1.9e11f, 7.1e10f, 8000.0f, 1.9e9f, 0.08f},    // maraging steel 300, aged: race suspension uprights and arms
};

const FrameMaterial* frame_materials(int& count) {
    count = (int)(sizeof kMaterials / sizeof kMaterials[0]);
    return kMaterials;
}

const FrameMaterial& frame_material(const std::string& name) {
    for (const FrameMaterial& m : kMaterials) {
        const char* a = m.name;
        size_t i = 0;
        for (; a[i] && i < name.size() && std::tolower((unsigned char)a[i]) == std::tolower((unsigned char)name[i]); i++) {}
        if (!a[i] && i == name.size()) return m;
    }
    return kMaterials[0];
}

FrameShape frame_shape(const std::string& name) {
    std::string s;
    for (char c : name) s += (char)std::tolower((unsigned char)c);
    if (s == "box" || s == "square" || s == "rhs" || s == "shs") return FrameShape::Box;
    if (s == "rod" || s == "round") return FrameShape::Rod;
    if (s == "bar" || s == "solid") return FrameShape::Bar;
    return FrameShape::Tube;
}

const char* frame_shape_name(FrameShape s) {
    switch (s) {
    case FrameShape::Box: return "box";
    case FrameShape::Rod: return "rod";
    case FrameShape::Bar: return "bar";
    default: return "tube";
    }
}

static const char* const kJointNames[FJ_COUNT] = {"rigid", "ball", "hinge_v", "hinge_h", "swivel", "elastic"};

const char* frame_joint_name(FrameJoint j) { return kJointNames[j < FJ_COUNT ? j : 0]; }

FrameJoint frame_joint(const std::string& name) {
    std::string s;
    for (char c : name) s += (char)std::tolower((unsigned char)c);
    if (s == "pinned" || s == "pin") return FJ_BALL;
    if (s == "hinge") return FJ_HINGE_V;
    for (int i = 0; i < FJ_COUNT; i++)
        if (s == kJointNames[i]) return (FrameJoint)i;
    return FJ_RIGID;
}

FrameSection make_frame_section(const std::string& material, FrameShape shape, float outer, float wall) {
    const FrameMaterial& m = frame_material(material);
    FrameSection s;
    s.E = m.E, s.G = m.G, s.rho = m.rho;
    s.max_strain = m.elongation;
    s.hinge_capacity = 10.0f * m.elongation; // (steel 2.0 rad, chromoly 1.5, aluminium 1.0, carbon 0.15)
    const double D = std::max(1e-3f, outer);
    const double ty = m.yield, tau = m.yield / std::sqrt(3.0);
    double A, I, J, Zp, Tp, kappa;
    if (shape == FrameShape::Tube || shape == FrameShape::Rod) {
        const double t = shape == FrameShape::Rod ? D * 0.5 : std::clamp((double)wall, 1e-4, D * 0.5);
        const double d = D - 2 * t, R = D * 0.5, r = d * 0.5;
        A = kPi / 4 * (D * D - d * d);
        I = kPi / 64 * (D * D * D * D - d * d * d * d);
        J = 2 * I;
        Zp = (D * D * D - d * d * d) / 6;
        Tp = 2 * kPi / 3 * tau * (R * R * R - r * r * r);
        kappa = shape == FrameShape::Rod ? 0.9 : 0.53;
    } else {
        const double t = shape == FrameShape::Bar ? D * 0.5 : std::clamp((double)wall, 1e-4, D * 0.5);
        const double bi = D - 2 * t;
        A = D * D - bi * bi;
        I = (D * D * D * D - bi * bi * bi * bi) / 12;
        if (shape == FrameShape::Bar || bi <= 0) {
            J = 0.1406 * D * D * D * D;
            Tp = tau * D * D * D / 3;
            kappa = 0.833;
        } else {
            J = t * (D - t) * (D - t) * (D - t); // (thin-walled closed section: 4 Am^2 t / perimeter)
            Tp = 2 * (D - t) * (D - t) * t * tau;
            kappa = 0.44;
        }
        Zp = (D * D * D - bi * bi * bi) / 4;
    }
    s.A = (float)A;
    s.Iy = s.Iz = (float)I;
    s.J = (float)J;
    s.As_y = s.As_z = (float)(kappa * A);
    s.Np = (float)(ty * A);
    s.Mp = (float)(ty * Zp);
    s.Tp = (float)Tp;
    s.half = (float)(D * 0.5);
    return s;
}

ShellSection make_shell_section(const std::string& material, float thickness) {
    const FrameMaterial& m = frame_material(material);
    ShellSection s;
    s.E = m.E, s.nu = std::clamp(m.E / (2.0f * m.G) - 1.0f, 0.0f, 0.49f), s.rho = m.rho;
    s.t = std::max(1e-4f, thickness);
    s.yield = m.yield, s.elongation = m.elongation;
    if (const char* d = getenv("BL_SHELL_DAMP")) s.damping = (float)atof(d); // (diagnostics: the shells' damping)
    // its fracture pattern: metals and plastics tear out a ring and radial tears round a blow (a plug), the brittle
    // carbon cracks in a web, wood splits along its grain
    const std::string n = m.name;
    s.pattern = n == "Carbon" ? ShellPattern::Radial : n == "Wood" ? ShellPattern::Grain : ShellPattern::Punch;
    if (s.pattern == ShellPattern::Radial) s.pattern_size = 0.2f, s.pattern_weak = 0.4f;
    if (const char* l = getenv("BL_FEM_LEVEL")) s.max_level = atoi(l); // (diagnostics: the refinement's depth, 0: none)
    if (getenv("BL_FEM_NOPATTERN")) s.pattern = ShellPattern::None;   // (diagnostics: no fracture patterns)
    if (const char* p = getenv("BL_FEM_MINPIECE")) s.min_piece = (float)atof(p); // (diagnostics: the smallest piece a tear cuts off, m2)
    return s;
}

// ------------------------------------------------------------------------------------------------ building
uint32_t FemFrame::add_node(uint32_t body_node) {
    if (body_node >= slot_.size()) slot_.resize(body_node + 1, -1);
    if (slot_[body_node] >= 0) return (uint32_t)slot_[body_node];
    slot_[body_node] = (int32_t)node.size();
    node.push_back(body_node);
    q.push_back(quat());
    w.push_back(vec3(0));
    inertia.push_back(0.0f);
    torque.push_back(vec3(0));
    member_f.push_back(vec3(0));
    xd.insert(xd.end(), {0.0, 0.0, 0.0});
    contact_n.push_back(vec3(0));
    contact_f.push_back(vec3(0));
    ready_ = false;
    return (uint32_t)node.size() - 1;
}

uint32_t FemFrame::add_mount(uint32_t body_a, uint32_t body_b, float brk, float k, float damp, MountKind kind, float param, uint32_t body_b2) {
    FrameMount m;
    add_node(body_a), add_node(body_b);
    m.a = body_a, m.b = body_b;
    m.brk = brk, m.k = k, m.damp = damp;
    m.kind = kind;
    if (kind == MountKind::Clamp) m.brk_m = std::max(0.0f, param);
    if (kind == MountKind::Strap) m.len = std::max(1.0f, param);
    if (kind == MountKind::Hinge) add_node(body_b2), m.b2 = body_b2;
    mounts.push_back(m);
    ready_ = false;
    return (uint32_t)mounts.size() - 1;
}

uint16_t FemFrame::add_section(const FrameSection& s) {
    sections.push_back(s);
    return (uint16_t)(sections.size() - 1);
}

uint16_t FemFrame::add_shell_section(const ShellSection& s) {
    shell_sections.push_back(s);
    if (s.pattern != ShellPattern::None) pattern_speed_ = patterned_ ? std::min(pattern_speed_, s.pattern_speed) : s.pattern_speed, patterned_ = true;
    return (uint16_t)(shell_sections.size() - 1);
}

uint32_t FemFrame::add_tri(uint32_t body_a, uint32_t body_b, uint32_t body_c, uint16_t section, int32_t tag, int32_t coll) {
    FrameTri t;
    t.n[0] = add_node(body_a), t.n[1] = add_node(body_b), t.n[2] = add_node(body_c);
    t.section = section;
    t.tag = tag, t.coll = coll;
    tris.push_back(t);
    ready_ = false;
    return (uint32_t)tris.size() - 1;
}

uint32_t FemFrame::add_element(uint32_t body_a, uint32_t body_b, uint16_t section, uint8_t end_a, uint8_t end_b, int32_t tag) {
    FrameElement e;
    e.a = add_node(body_a);
    e.b = add_node(body_b);
    e.section = section;
    e.end_a = end_a < FJ_COUNT ? end_a : FJ_RIGID;
    e.end_b = end_b < FJ_COUNT ? end_b : FJ_RIGID;
    e.tag = tag;
    elems.push_back(e);
    ready_ = false;
    return (uint32_t)elems.size() - 1;
}

void FemFrame::finalize(const SoftBody& b) {
    body_ = &b;
    const size_t n = node.size();
    q.resize(n, quat());
    w.resize(n, vec3(0));
    torque.assign(n, vec3(0));
    member_f.assign(n, vec3(0));
    contact_n.assign(n, vec3(0));
    contact_f.assign(n, vec3(0));
    inertia.assign(n, 0.0f);
    xd.resize(n * 3);
    for (size_t i = 0; i < n; i++) {
        const vec3 p = b.nodes[node[i]].p;
        xd[i * 3] = p.x, xd[i * 3 + 1] = p.y, xd[i * 3 + 2] = p.z;
    }
    for (FrameElement& e : elems) {
        const vec3 xa = b.nodes[node[e.a]].p, xb = b.nodes[node[e.b]].p;
        const vec3 d = xb - xa;
        e.L0 = length(d);
        if (e.L0 < 1e-5f || e.a == e.b) {
            e.broken = true;
            continue;
        }
        // the rest frame: x along the member, y towards the world's up (a vertical member: towards x)
        const vec3 e1 = d / e.L0;
        const vec3 ref = std::fabs(e1.y) < 0.9f ? vec3(0, 1, 0) : vec3(1, 0, 0);
        const vec3 e2 = normalize(ref - e1 * dot(e1, ref)), e3 = cross(e1, e2);
        const quat E0 = from_mat3(mat3(e1, e2, e3));
        e.qa = normalize(conj(q[e.a]) * E0);
        e.qb = normalize(conj(q[e.b]) * E0);
        // (the mass it put on its nodes: the caller's, else its section's, assumed on the nodes already)
        if (e.mass <= 0) e.mass = element_mass(e);
    }
    // the triangles' rest shape (once: a later pattern change keeps it): the corners in the plane of the element, from
    // its centroid, x along its first edge; the element's rest frame in each corner's frame
    for (FrameTri& t : tris) {
        if (t.area0 > 0 || t.broken) continue;
        const vec3 x0 = b.nodes[node[t.n[0]]].p, x1 = b.nodes[node[t.n[1]]].p, x2 = b.nodes[node[t.n[2]]].p;
        const vec3 nn = cross(x1 - x0, x2 - x0);
        const float a2 = length(nn);
        if (!(a2 > 1e-8f) || t.n[0] == t.n[1] || t.n[1] == t.n[2] || t.n[0] == t.n[2]) {
            t.broken = true;
            continue;
        }
        const vec3 e3 = nn / a2, e1 = normalize(x1 - x0), e2 = cross(e3, e1);
        const vec3 c = (x0 + x1 + x2) / 3.0f;
        const vec3 xs[3] = {x0, x1, x2};
        for (int i = 0; i < 3; i++) t.X[i][0] = t.X0[i][0] = dot(xs[i] - c, e1), t.X[i][1] = t.X0[i][1] = dot(xs[i] - c, e2);
        t.area0 = 0.5f * a2;
        const quat E0 = from_mat3(mat3(e1, e2, e3));
        for (int i = 0; i < 3; i++) t.r0[i] = normalize(conj(q[t.n[i]]) * E0), t.th0[i] = vec3(0);
        if (t.mass <= 0) t.mass = tri_mass(t);
    }
    tri_k_.assign(tris.size() * kTriK, 0.0f);
    tri_b_.assign(tris.size() * kTriB, 0.0f);
    tri_k_ok_.assign(tris.size(), 0);
    if (tri_cap_ == 0) tri_cap_ = 3 * tris.size() + 16; // (the refinement's budget, of the authored shell)
    for (size_t i = 0; i < n; i++) node_inertia((uint32_t)i, b);
    // the mounts made once, where their nodes stand now: b's point in a's frame, the spring the nodes' masses take at
    // the step unless given (a quarter of the explicit limit), the damping near a third of critical
    std::vector<std::vector<uint32_t>> nbr;
    for (FrameMount& m : mounts) {
        if (m.made) continue;
        const int sa = slot(m.a), sb = slot(m.b);
        if (sa < 0 || sb < 0) continue;
        m.off = conj(q[sa]).rotate(b.nodes[m.b].p - b.nodes[m.a].p);
        m.na = 0;
        const float h = kDefaultDt;
        if (m.kind == MountKind::Stop || m.kind == MountKind::Strap) {
            // (a stop, a strap: the two nodes' distance alone)
            const float iab = b.nodes[m.a].inv_mass + b.nodes[m.b].inv_mass;
            m.m_eff = iab > 0 ? 1.0f / iab : 1.0f;
            if (!(m.k > 0)) m.k = 0.25f * m.m_eff / (h * h);
            m.c = 2.0f * 0.3f * std::sqrt(m.k * m.m_eff);
            m.L0 = length(b.nodes[m.b].p - b.nodes[m.a].p);
            m.nb = 1, m.bn[0] = m.b;
            m.made = true;
            m.members[0] = (uint8_t)std::min(255, members_at((uint32_t)sa));
            m.members[4] = (uint8_t)std::min(255, members_at((uint32_t)sb));
            continue;
        }
        // the anchor's nodes: a and, of the frame nodes a few members (or triangles' edges) round it (within 0.9 m), the
        // farthest from a, from their line and from their plane
        if (nbr.empty()) {
            nbr.assign(n, {});
            for (const FrameElement& e : elems)
                if (!e.broken) nbr[e.a].push_back(e.b), nbr[e.b].push_back(e.a);
            for (const FrameTri& t : tris)
                if (!t.broken)
                    for (int i = 0; i < 3; i++) nbr[t.n[i]].push_back(t.n[(i + 1) % 3]), nbr[t.n[(i + 1) % 3]].push_back(t.n[i]);
        }
        // (of candidates round node c within reach: the farthest from it, then off their line, then off their plane)
        auto spread = [&](int c, float reach, int out[3], int rings = 6) {
            const vec3 x0 = b.nodes[node[c]].p;
            std::vector<uint32_t> cand{(uint32_t)c};
            for (size_t r0 = 0, ring = 0; ring < (size_t)rings && r0 < cand.size(); ring++) {
                const size_t r1 = cand.size();
                for (size_t i = r0; i < r1; i++)
                    for (uint32_t u : nbr[cand[i]])
                        if (std::find(cand.begin(), cand.end(), u) == cand.end() && length(b.nodes[node[u]].p - x0) < reach) cand.push_back(u);
                r0 = r1;
            }
            out[0] = out[1] = out[2] = -1;
            float best = 0;
            for (uint32_t u : cand)
                if (u != (uint32_t)c && length(b.nodes[node[u]].p - x0) > best) best = length(b.nodes[node[u]].p - x0), out[0] = (int)u;
            if (out[0] >= 0) {
                const vec3 d1 = normalize(b.nodes[node[out[0]]].p - x0);
                best = 0;
                for (uint32_t u : cand) {
                    const vec3 r = b.nodes[node[u]].p - x0;
                    const float l = length(r - d1 * dot(r, d1));
                    if (u != (uint32_t)c && l > best) best = l, out[1] = (int)u;
                }
            }
            if (out[1] >= 0) {
                const vec3 nrm = normalize(cross(b.nodes[node[out[0]]].p - x0, b.nodes[node[out[1]]].p - x0));
                best = 0;
                for (uint32_t u : cand) {
                    const float l = std::fabs(dot(b.nodes[node[u]].p - x0, nrm));
                    if (u != (uint32_t)c && l > best) best = l, out[2] = (int)u;
                }
                if (best < 0.02f) out[2] = -1;
            }
        };
        const vec3 x0 = b.nodes[m.a].p;
        int ai[3];
        spread(sa, 0.9f, ai);
        vec3 e1, e2, e3;
        float det = 0;
        if (ai[2] >= 0) {
            e1 = b.nodes[node[ai[0]]].p - x0, e2 = b.nodes[node[ai[1]]].p - x0, e3 = b.nodes[node[ai[2]]].p - x0;
            det = dot(e1, cross(e2, e3));
            if (std::fabs(det) > 1e-9f) {
                m.an[0] = m.a, m.an[1] = node[ai[0]], m.an[2] = node[ai[1]], m.an[3] = node[ai[2]];
                m.na = 4;
            }
        }
        // the part's nodes held: b; a hinge's second node on its line; a clamp's three more round b (within 0.5 m, the
        // part's own: its members' and triangles' ring round b)
        m.nb = 1, m.bn[0] = m.b;
        if (m.kind == MountKind::Hinge && slot(m.b2) >= 0 && m.b2 != m.b) m.bn[m.nb++] = m.b2;
        if (m.kind == MountKind::Clamp) {
            // (its flange: the ring of the part's nodes next to b - farther out a flexible part's own bending read as the
            // bolt turned, a bumper's plastic clip twisted off by a drive round a circle; more rings for a sparse one)
            int bi[3];
            spread(sb, 0.5f, bi, 1);
            if (bi[1] < 0) spread(sb, 0.5f, bi);
            for (int j = 0; j < 3; j++)
                if (bi[j] >= 0) m.bn[m.nb++] = node[bi[j]];
        }
        // each one's point: its affine weights on the anchor's nodes (or its offset in a's frame), its spring - a quarter
        // of what it and its share of the anchor take at the step, the anchor shared by them all
        float ksum = 0;
        for (int j = 0; j < m.nb; j++) {
            const vec3 r = b.nodes[m.bn[j]].p - x0;
            float ia = 0;
            if (m.na > 0) {
                const float l1 = dot(r, cross(e2, e3)) / det, l2 = dot(e1, cross(r, e3)) / det, l3 = dot(e1, cross(e2, r)) / det;
                m.bw[j][0] = 1.0f - l1 - l2 - l3, m.bw[j][1] = l1, m.bw[j][2] = l2, m.bw[j][3] = l3;
                for (int i = 0; i < 4; i++) ia += m.bw[j][i] * m.bw[j][i] * b.nodes[m.an[i]].inv_mass;
            } else {
                m.boff[j] = conj(q[sa]).rotate(r);
                ia = b.nodes[m.a].inv_mass;
            }
            const float ib = b.nodes[m.bn[j]].inv_mass;
            const float me = ia * m.nb + ib > 0 ? 1.0f / (ia * m.nb + ib) : 1.0f;
            m.bm[j] = me;
            m.bk[j] = m.k > 0 ? m.k / m.nb : 0.25f * me / (h * h);
            m.bc[j] = 2.0f * 0.3f * std::sqrt(m.bk[j] * me);
            ksum += m.bk[j];
            if (j == 0) m.m_eff = me;
        }
        // (a clamp with a break moment gives a little before it lets go: the nodes round b on springs that turn the part
        // kClampGive radians at that moment - a bolted flange's give and slip - b's own spring the step's)
        if (m.kind == MountKind::Clamp && m.brk_m > 0 && m.nb > 1) {
            float r2 = 0;
            for (int j = 1; j < m.nb; j++) r2 += length2(b.nodes[m.bn[j]].p - b.nodes[m.b].p);
            if (r2 > 1e-6f)
                for (int j = 1; j < m.nb; j++) {
                    ksum -= m.bk[j];
                    m.bk[j] = std::min(m.bk[j], m.brk_m / kClampGive / r2);
                    m.bc[j] = 2.0f * 0.3f * std::sqrt(m.bk[j] * m.bm[j]);
                    ksum += m.bk[j];
                }
        }
        for (int i = 0; i < 4; i++) m.aw[i] = m.bw[0][i];
        m.k = ksum;
        m.c = m.bc[0];
        m.made = true;
        {
            const int na = std::max(1, m.na);
            for (int j = 0; j < na; j++) m.members[j] = (uint8_t)std::min(255, members_at((uint32_t)slot(m.na > 0 ? m.an[j] : m.a)));
            m.members[4] = (uint8_t)std::min(255, members_at((uint32_t)sb));
        }
        static const bool dbg = getenv("BL_FRAMEDBG") != nullptr;
        if (dbg)
            printf("frame: mount %u-%u (kind %d, %d nodes held): %.0f mm off, anchor of %d nodes (%.2f %.2f %.2f %.2f), k %.3g N/m, c %.3g N s/m, masses %.2f %.2f kg\n",
                   m.a, m.b, (int)m.kind, m.nb, length(m.off) * 1000.0f, m.na, m.aw[0], m.aw[1], m.aw[2], m.aw[3], m.k, m.c, b.nodes[m.a].mass, b.nodes[m.b].mass);
    }
    analyse();
    tan_.assign(elems.size(), Tangent());
    tri_tan_.assign(tris.size(), TriTan());
    ready_ = true;
    authored_hinges = vertex_hinges();
}

// The mounts' springs: b (and a clamp's, a hinge's other nodes) held at a's points (explicit; acting for the frame's
// step, as the members); past the break force for kOverloadTime a mount lets go, a clamp past its break moment (about
// its bolt) for kTwistTime. A stop pushes b off a, a strap pulls it back, past their distances
void FemFrame::mount_forces(SoftBody& b) {
    vec3* F = b.force.data();
    static const bool dbg = getenv("BL_FRAMEDBG") != nullptr;
    const float h = last_h_ > 0 ? last_h_ : kDefaultDt;
    for (size_t i = 0; i < mounts.size(); i++) {
        FrameMount& m = mounts[i];
        if (m.broken || !m.made) continue;
        const int sa = slot(m.a), sb = slot(m.b);
        if (sa < 0 || sb < 0) continue;
        // (the load against its limit: past it for kOverloadTime, it lets go)
        auto lets_go = [&](float over) {
            if (!b.allow_break) return false;
            m.overload = std::max(0.0f, m.overload + (over - 1.0f) * h);
            if (m.overload <= kOverloadTime) return false;
            if (dbg) printf("frame: mount %zu (%u-%u) lets go at %.0f N (breaks at %.0f), kind %d, %.0f N m (breaks at %.0f N m)\n", i, m.a, m.b, m.f, m.brk, (int)m.kind, m.m,
                            m.brk_m);
            m.broken = true;
            mounts_broken++;
            b.stats.broken_beams++;
            debris_check = true;
            return true;
        };
        // (explicit for the frame's step: at a longer step than the substep at most what the nodes' masses take at it,
        // omega h 1 - 2e6 N/m on half-kilo nodes rang at 1 kHz and let a part go on a 1 ms peak)
        auto capped = [&](float k, float c, float me, float& kk, float& cc) {
            kk = k, cc = c;
            if (h > 1.5f * kDefaultDt && me > 0 && k > me / (h * h)) {
                const float kmax = me / (h * h);
                cc = c * std::sqrt(kmax / k), kk = kmax;
            }
        };
        const Node& A = b.nodes[m.a];
        if (m.kind == MountKind::Stop || m.kind == MountKind::Strap) {
            const Node& B = b.nodes[m.b];
            const vec3 dx = B.p - A.p;
            const float L = length(dx);
            const float lim = m.kind == MountKind::Stop ? m.L0 : m.L0 * m.len;
            if (L < 1e-6f || (m.kind == MountKind::Stop ? L >= lim : L <= lim)) {
                m.f = 0, m.overload = std::max(0.0f, m.overload - h);
                continue;
            }
            const vec3 nrm = dx / L;
            float k, c;
            capped(m.k, m.c, m.m_eff, k, c);
            const float fs = k * (lim - L); // (on b along a->b: a stop's out, a strap's in)
            float fn = fs - c * dot(B.v - A.v, nrm);
            fn = m.kind == MountKind::Stop ? std::max(0.0f, fn) : std::min(0.0f, fn);
            m.f = std::fabs(fs), m.m = 0;
            if (m.brk > 0 && lets_go(std::fabs(fs) / m.brk)) continue;
            const vec3 f = nrm * fn;
            F[m.b] += f, member_f[sb] += f;
            F[m.a] -= f, member_f[sa] -= f;
            continue;
        }
        // a point, a clamp, a hinge: each held node on its spring to its point of the anchor
        vec3 fj[4], rj[4], xm(0), fsum(0);
        for (int j = 0; j < m.nb; j++) {
            const Node& Bj = b.nodes[m.bn[j]];
            vec3 P(0), Pv(0);
            if (m.na > 0) {
                for (int k = 0; k < m.na; k++) P += b.nodes[m.an[k]].p * m.bw[j][k], Pv += b.nodes[m.an[k]].v * m.bw[j][k];
            } else {
                rj[j] = q[sa].rotate(m.boff[j]);
                P = A.p + rj[j], Pv = A.v + cross(w[sa], rj[j]);
            }
            float k, c;
            capped(m.bk[j], m.bc[j], m.bm[j], k, c);
            const vec3 fs = (P - Bj.p) * k;
            fj[j] = fs + (Pv - Bj.v) * c; // (on the part's node; the anchor's nodes take it back)
            fsum += fs, xm += Bj.p / (float)m.nb;
        }
        // (a clamp's moment about its bolt, b: about the held nodes' middle b's own spring carrying the part's weight and
        // its jolts made one - a bumper's brackets were twisted off by a drive round a circle, 69 N m of it, more than
        // the bumper hung on one of them made)
        vec3 msum(0);
        const vec3 about = m.kind == MountKind::Clamp ? b.nodes[m.b].p : xm;
        for (int j = 0; j < m.nb; j++) msum += cross(b.nodes[m.bn[j]].p - about, fj[j]);
        m.f = length(fsum), m.m = length(msum);
        if (m.kind == MountKind::Clamp && m.brk_m > 0) { // (twisted past its break moment for kTwistTime: it lets go)
            m.twist = std::max(0.0f, m.twist + (m.m / m.brk_m - 1.0f) * h);
            if (m.twist > kTwistTime && b.allow_break) {
                if (dbg) printf("frame: mount %zu (%u-%u) twisted off at %.0f N m (breaks at %.0f N m)\n", i, m.a, m.b, m.m, m.brk_m);
                m.broken = true;
                mounts_broken++;
                b.stats.broken_beams++;
                debris_check = true;
                continue;
            }
        }
        if (m.brk > 0 && lets_go(m.f / m.brk)) continue;
        for (int j = 0; j < m.nb; j++) {
            const vec3 f = fj[j];
            F[m.bn[j]] += f;
            if (const int sj = slot(m.bn[j]); sj >= 0) member_f[sj] += f;
            if (m.na > 0) {
                for (int k = 0; k < m.na; k++) {
                    F[m.an[k]] -= f * m.bw[j][k];
                    if (const int sk = slot(m.an[k]); sk >= 0) member_f[sk] -= f * m.bw[j][k];
                }
            } else {
                F[m.a] -= f;
                member_f[sa] -= f;
                torque[sa] += cross(rj[j], -f);
                if ((size_t)sa < member_t_.size()) member_t_[sa] += cross(rj[j], -f);
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------ the sparse pattern
void FemFrame::analyse() {
    PROFILE_ACCUM("Frame analyse");
    const int n = (int)node.size();
    // the components: frame nodes joined by members (a part on mounts is one of its own)
    std::vector<int> up(n);
    for (int i = 0; i < n; i++) up[i] = i;
    auto find = [&](int x) {
        while (up[x] != x) x = up[x] = up[up[x]];
        return x;
    };
    for (const FrameElement& e : elems) up[find((int)e.a)] = find((int)e.b);
    for (const FrameTri& t : tris)
        if (!t.broken) up[find((int)t.n[0])] = find((int)t.n[1]), up[find((int)t.n[1])] = find((int)t.n[2]);
    // (the graph's edges collected, then a node's neighbours each once: adj_ptr / adj_idx)
    std::vector<std::pair<int, int>> edge;
    edge.reserve(elems.size() + 3 * tris.size() + 64);
    auto link = [&](int a, int b) {
        if (a != b) edge.push_back({a, b});
    };
    for (const FrameElement& e : elems) link((int)e.a, (int)e.b);
    for (const FrameTri& t : tris)
        if (!t.broken) link((int)t.n[0], (int)t.n[1]), link((int)t.n[1], (int)t.n[2]), link((int)t.n[2], (int)t.n[0]);
    // the body's springs on frame nodes: in their component's system (both ends in one: a block of its pattern); one
    // between two components acts explicitly, as the mounts do
    links_.clear();
    if (body_) {
        const SoftBody& b = *body_;
        std::vector<int32_t> shock_of(b.beams.size(), -1);
        for (size_t k = 0; k < b.shocks.size(); k++)
            if (b.shocks[k].beam < b.beams.size()) shock_of[b.shocks[k].beam] = (int32_t)k;
        for (size_t k = 0; k < b.beams.size(); k++) {
            const Beam& bm = b.beams[k];
            const int fa = slot(bm.a), fb = slot(bm.b);
            if (fa < 0 && fb < 0) continue;
            if (fa >= 0 && fb >= 0 && find(fa) != find(fb)) continue;
            links_.push_back({(uint32_t)k, fa, fb, shock_of[k], -1, 0});
            if (fa >= 0 && fb >= 0) link(fa, fb);
        }
    }
    // components in the order of their first node; minimum degree ordering on each one's elimination graph, the
    // components one after another (no block joins two: each factors on its own)
    std::vector<int> comp_of(n, -1), roots;
    for (int i = 0; i < n; i++) {
        const int r = find(i);
        if (comp_of[r] < 0) comp_of[r] = (int)roots.size(), roots.push_back(r);
        comp_of[i] = comp_of[r];
    }
    const int nc = (int)roots.size();
    std::vector<int> adj_ptr(n + 1, 0), adj_idx;
    {
        for (const auto& [a, b] : edge) adj_ptr[a + 1]++, adj_ptr[b + 1]++;
        for (int i = 0; i < n; i++) adj_ptr[i + 1] += adj_ptr[i];
        std::vector<int> raw(adj_ptr[n]), fill(adj_ptr.begin(), adj_ptr.end() - 1);
        for (const auto& [a, b] : edge) raw[fill[a]++] = b, raw[fill[b]++] = a;
        std::vector<int> stamp(n, -1);
        adj_idx.reserve(raw.size());
        std::vector<int> ptr2(n + 1, 0);
        for (int i = 0; i < n; i++) {
            for (int q = adj_ptr[i]; q < adj_ptr[i + 1]; q++)
                if (stamp[raw[q]] != i) stamp[raw[q]] = i, adj_idx.push_back(raw[q]);
            ptr2[i + 1] = (int)adj_idx.size();
        }
        adj_ptr.swap(ptr2);
    }
    if (prof::g_trace) prof::trace_mark("analyse", 1);
    std::vector<std::vector<int>> comp_nodes(nc);
    for (int i = 0; i < n; i++) comp_nodes[comp_of[i]].push_back(i);
    perm_.assign(n, 0);
    iperm_.assign(n, 0);
    comp_range_.assign(nc, {0, 0});
    // (the elimination graph of a component as bit rows: a node's neighbours after the ones before it were eliminated,
    // each elimination joining its neighbours to each other; the pick is the first node of least degree in the
    // component's order. With lists and searches it was a third of a millisecond a pattern, 30 of them in a crash's frame)
    std::vector<int> cols_ptr(n + 1, 0), cols_buf, loc(n, -1), deg;
    std::vector<uint64_t> bits, bucket; // (bucket d: the nodes of degree d, a bit each - the least degree's first in order)
    cols_buf.reserve((size_t)n * 8);
    int k = 0;
    for (int c = 0; c < nc; c++) {
        comp_range_[c].first = k;
        const std::vector<int>& cn = comp_nodes[c];
        const int m = (int)cn.size(), W = (m + 63) / 64;
        for (int x = 0; x < m; x++) loc[cn[x]] = x;
        bits.assign((size_t)m * W, 0ull);
        bucket.assign((size_t)m * W, 0ull);
        deg.assign(m, 0);
        int mind = m;
        for (int x = 0; x < m; x++) {
            uint64_t* rx = &bits[(size_t)x * W];
            for (int q = adj_ptr[cn[x]]; q < adj_ptr[cn[x] + 1]; q++) {
                const int y = loc[adj_idx[q]];
                rx[y >> 6] |= 1ull << (y & 63);
            }
            deg[x] = adj_ptr[cn[x] + 1] - adj_ptr[cn[x]];
            bucket[(size_t)deg[x] * W + (x >> 6)] |= 1ull << (x & 63);
            mind = std::min(mind, deg[x]);
        }
        for (int step = 0; step < m; step++, k++) {
            int v = -1;
            while (v < 0) {
                const uint64_t* bd = &bucket[(size_t)mind * W];
                for (int w = 0; w < W; w++)
                    if (bd[w]) {
                        v = (w << 6) + __builtin_ctzll(bd[w]);
                        break;
                    }
                if (v < 0) mind++;
            }
            bucket[(size_t)deg[v] * W + (v >> 6)] &= ~(1ull << (v & 63));
            perm_[k] = cn[v];
            iperm_[cn[v]] = k;
            uint64_t* rv = &bits[(size_t)v * W];
            for (int w = 0; w < W; w++)
                for (uint64_t b = rv[w]; b; b &= b - 1) {
                    const int y = (w << 6) + __builtin_ctzll(b);
                    cols_buf.push_back(cn[y]);
                    uint64_t* ry = &bits[(size_t)y * W];
                    int d = 0;
                    for (int q = 0; q < W; q++) ry[q] |= rv[q];
                    ry[y >> 6] &= ~(1ull << (y & 63));
                    ry[v >> 6] &= ~(1ull << (v & 63));
                    for (int q = 0; q < W; q++) d += __builtin_popcountll(ry[q]);
                    bucket[(size_t)deg[y] * W + (y >> 6)] &= ~(1ull << (y & 63));
                    bucket[(size_t)d * W + (y >> 6)] |= 1ull << (y & 63);
                    deg[y] = d;
                    mind = std::min(mind, d);
                }
            for (int w = 0; w < W; w++) rv[w] = 0;
            cols_ptr[k + 1] = (int)cols_buf.size();
        }
        comp_range_[c].second = k;
    }
    if (prof::g_trace) prof::trace_mark("analyse", 2);
    col_ptr_.assign(n + 1, 0);
    row_.resize(cols_buf.size());
    size_t nupd = 0;
    for (int kk = 0; kk < n; kk++) {
        const int a0 = cols_ptr[kk], a1 = cols_ptr[kk + 1];
        for (int q = a0; q < a1; q++) row_[q] = iperm_[cols_buf[q]];
        std::sort(row_.begin() + a0, row_.begin() + a1);
        col_ptr_[kk + 1] = a1;
        nupd += (size_t)(a1 - a0) * (a1 - a0 + 1) / 2;
    }
    auto find_block = [&](int col, int row) { // (a column's rows are sorted)
        const int* a = row_.data() + col_ptr_[col];
        const int* e = row_.data() + col_ptr_[col + 1];
        const int* it = std::lower_bound(a, e, row);
        return it != e && *it == row ? (int)(it - row_.data()) : -1;
    };
    upd_ptr_.assign(n + 1, 0);
    upd_.clear();
    upd_.reserve(nupd);
    for (int kk = 0; kk < n; kk++) {
        for (int pi = col_ptr_[kk]; pi < col_ptr_[kk + 1]; pi++) {
            // (the targets in column row_[pi], rows ascending as pj's: a walk down its rows, not a search each)
            const int c = row_[pi], qe = col_ptr_[c + 1];
            int q = col_ptr_[c];
            for (int pj = pi; pj < col_ptr_[kk + 1]; pj++) {
                Update u;
                u.pi = pi, u.pj = pj;
                if (pi == pj) {
                    u.target = -(c + 1);
                } else {
                    const int rr = row_[pj];
                    while (q < qe && row_[q] < rr) q++;
                    u.target = q < qe && row_[q] == rr ? q : -1;
                }
                upd_.push_back(u);
            }
        }
        upd_ptr_[kk + 1] = (int)upd_.size();
    }
    if (prof::g_trace) prof::trace_mark("analyse", 3);
    elem_block_.assign(elems.size(), -1);
    elem_swap_.assign(elems.size(), 0);
    comp_elems_.assign(nc, {});
    for (size_t i = 0; i < elems.size(); i++) {
        comp_elems_[comp_of[elems[i].a]].push_back((uint32_t)i);
        const int ka = iperm_[elems[i].a], kb = iperm_[elems[i].b];
        if (ka == kb) continue;
        elem_block_[i] = find_block(std::min(ka, kb), std::max(ka, kb));
        elem_swap_[i] = ka > kb;
    }
    // the triangles: the blocks of their corner pairs (01, 12, 20; the row node the later one in the order)
    tri_block_.assign(tris.size(), {-1, -1, -1});
    tri_swap_.assign(tris.size(), {0, 0, 0});
    comp_tris_.assign(nc, {});
    for (size_t ti = 0; ti < tris.size(); ti++) {
        const FrameTri& t = tris[ti];
        if (t.broken) continue;
        comp_tris_[comp_of[t.n[0]]].push_back((uint32_t)ti);
        for (int e = 0; e < 3; e++) {
            const int ki = iperm_[t.n[e]], kj = iperm_[t.n[(e + 1) % 3]];
            if (ki == kj) continue;
            tri_block_[ti][e] = find_block(std::min(ki, kj), std::max(ki, kj));
            tri_swap_[ti][e] = ki > kj;
        }
    }
    comp_links_.assign(nc, {});
    onesided_n_.assign(body_ ? body_->nodes.size() : 0, 0);
    for (size_t li = 0; li < links_.size(); li++) {
        Link& l = links_[li];
        comp_links_[comp_of[l.fa >= 0 ? l.fa : l.fb]].push_back((uint32_t)li);
        if ((l.fa < 0) != (l.fb < 0) && body_ && l.beam < body_->beams.size()) {
            const Beam& bm = body_->beams[l.beam];
            const uint32_t o = l.fa >= 0 ? bm.b : bm.a;
            if (o < onesided_n_.size() && onesided_n_[o] < 65535) onesided_n_[o]++;
        }
        if (l.fa < 0 || l.fb < 0 || l.fa == l.fb) continue;
        const int ka = iperm_[l.fa], kb = iperm_[l.fb];
        l.block = find_block(std::min(ka, kb), std::max(ka, kb));
        l.swap = ka > kb;
    }
    // the damped mounts' ends, in the component of each
    comp_mdamp_.assign(nc, {});
    for (size_t mi = 0; mi < mounts.size(); mi++) {
        const FrameMount& m = mounts[mi];
        const int sa = slot(m.a), sb = slot(m.b);
        if (m.broken || !(m.damp > 0) || sa < 0 || sb < 0 || sa >= n || sb >= n) continue;
        comp_mdamp_[comp_of[sa]].push_back({sa, sb, m.damp, (uint32_t)mi});
        comp_mdamp_[comp_of[sb]].push_back({sb, sa, m.damp, (uint32_t)mi});
    }
    comp_changed_.assign(nc, {});
    comp_stats_.assign(nc, CompStats());
    if (getenv("BL_FACTORDBG")) { // (diagnostics: the factor's size, the largest components')
        printf("frame pattern: %d nodes, %zu members, %zu triangles, %d components, %zu off-diagonal blocks, %zu block updates\n", n, elems.size(), tris.size(), nc, row_.size(),
               upd_.size());
        for (int c = 0; c < nc; c++) {
            const int a = comp_range_[c].first, b = comp_range_[c].second;
            if (b - a >= 60) printf("  component %d: %d nodes, %d off-diagonal blocks, %d block updates\n", c, b - a, col_ptr_[b] - col_ptr_[a], upd_ptr_[b] - upd_ptr_[a]);
        }
    }
    if (prof::g_trace) prof::trace_mark("analyse", 4);
    gather_lists();
    if (prof::g_trace) prof::trace_mark("analyse", 5);
    par_plan();
    if (prof::g_trace) prof::trace_mark("analyse", 6);
    // (diagnostics, BL_FACTORDUMP=<file>: the largest component's pattern as JSON, once: the nodes in the elimination
    // order, the members, triangles and springs between them, and the factor's blocks per column)
    static bool dumped = false;
    if (const char* path = getenv("BL_FACTORDUMP"); path && !dumped && body_) {
        int big = -1;
        for (int c = 0; c < nc; c++)
            if (big < 0 || comp_range_[c].second - comp_range_[c].first > comp_range_[big].second - comp_range_[big].first) big = c;
        const int a = big >= 0 ? comp_range_[big].first : 0, e = big >= 0 ? comp_range_[big].second : 0;
        if (e - a >= 100) {
            dumped = true;
            if (FILE* f = fopen(path, "w")) {
                fprintf(f, "{\"nodes\": %d, \"blocks\": %d, \"updates\": %d,\n\"pos\": [", e - a, col_ptr_[e] - col_ptr_[a], upd_ptr_[e] - upd_ptr_[a]);
                for (int kk = a; kk < e; kk++) {
                    const vec3 q = body_->nodes[node[perm_[kk]]].p;
                    fprintf(f, "%s[%.3f,%.3f,%.3f]", kk > a ? "," : "", q.x, q.y, q.z);
                }
                fprintf(f, "],\n\"members\": [");
                bool first = true;
                for (const FrameElement& m : elems)
                    if (comp_of[m.a] == big && m.a != m.b)
                        fprintf(f, "%s[%d,%d,%d]", first ? "" : ",", iperm_[m.a] - a, iperm_[m.b] - a, m.hidden ? 1 : 0), first = false;
                fprintf(f, "],\n\"tris\": [");
                first = true;
                for (const FrameTri& t : tris)
                    if (!t.broken && comp_of[t.n[0]] == big)
                        fprintf(f, "%s[%d,%d,%d]", first ? "" : ",", iperm_[t.n[0]] - a, iperm_[t.n[1]] - a, iperm_[t.n[2]] - a), first = false;
                fprintf(f, "],\n\"links\": [");
                first = true;
                for (const Link& l : links_)
                    if (l.fa >= 0 && l.fb >= 0 && l.fa != l.fb && comp_of[l.fa] == big)
                        fprintf(f, "%s[%d,%d]", first ? "" : ",", iperm_[l.fa] - a, iperm_[l.fb] - a), first = false;
                fprintf(f, "],\n\"cols\": [");
                for (int kk = a; kk < e; kk++) {
                    fprintf(f, "%s[", kk > a ? "," : "");
                    for (int p = col_ptr_[kk]; p < col_ptr_[kk + 1]; p++) fprintf(f, "%s%d", p > col_ptr_[kk] ? "," : "", row_[p] - a);
                    fprintf(f, "]");
                }
                // (its plan of the team's stages, if any: the subtree groups' columns, the columns above)
                fprintf(f, "],\n\"groups\": [");
                const int pl = big < (int)comp_plan_.size() ? comp_plan_[big] : -1;
                if (pl >= 0) {
                    const ParPlan& P = plans_[pl];
                    for (size_t g = 0; g + 1 < P.gptr.size(); g++) {
                        fprintf(f, "%s[", g ? "," : "");
                        for (int x = P.gptr[g]; x < P.gptr[g + 1]; x++) fprintf(f, "%s%d", x > P.gptr[g] ? "," : "", P.gcols[x] - a);
                        fprintf(f, "]");
                    }
                    fprintf(f, "],\n\"top\": [");
                    for (size_t x = 0; x < P.top.size(); x++) fprintf(f, "%s%d", x ? "," : "", P.top[x] - a);
                    fprintf(f, "],\n\"deferred\": %zu", P.dupd.size());
                } else {
                    fprintf(f, "],\n\"top\": [], \"deferred\": 0");
                }
                fprintf(f, "}\n");
                fclose(f);
            }
        }
    }
    diag_.assign((size_t)n * 36, 0.0);
    off_.assign(row_.size() * 36, 0.0);
    rhs_.assign((size_t)n * 6, 0.0);
    dinv_.assign((size_t)n * 6, 0.0);
    diagA_.resize(diag_.size()), offA_.resize(off_.size()), rhsA_.resize(rhs_.size()); // (copies of the assembly: no zeroing)
}

// The assembly's lists (see Gather): the members' blocks, then the triangles', each in its order
void FemFrame::gather_lists() {
    const int n = (int)node.size();
    gdiag_ptr_.assign(n + 2, 0);
    goff_ptr_.assign(row_.size() + 2, 0);
    // (counted, then filled in the elements' order: the members, then the triangles)
    for (size_t i = 0; i < elems.size(); i++) {
        gdiag_ptr_[iperm_[elems[i].a] + 2]++, gdiag_ptr_[iperm_[elems[i].b] + 2]++;
        if (elem_block_[i] >= 0) goff_ptr_[elem_block_[i] + 2]++;
    }
    for (size_t ti = 0; ti < tris.size(); ti++) {
        if (tris[ti].broken) continue;
        for (int c = 0; c < 3; c++) gdiag_ptr_[iperm_[tris[ti].n[c]] + 2]++;
        for (int e = 0; e < 3; e++)
            if (tri_block_[ti][e] >= 0) goff_ptr_[tri_block_[ti][e] + 2]++;
    }
    for (size_t k = 2; k < gdiag_ptr_.size(); k++) gdiag_ptr_[k] += gdiag_ptr_[k - 1];
    for (size_t k = 2; k < goff_ptr_.size(); k++) goff_ptr_[k] += goff_ptr_[k - 1];
    gdiag_.resize(gdiag_ptr_.back());
    goff_.resize(goff_ptr_.back());
    // (the fill cursors: ptr[k + 1] runs from the start of k's entries to their end)
    for (size_t i = 0; i < elems.size(); i++) {
        const FrameElement& e = elems[i];
        gdiag_[gdiag_ptr_[iperm_[e.a] + 1]++] = {(uint32_t)i, 0, 0, 0, 0};
        gdiag_[gdiag_ptr_[iperm_[e.b] + 1]++] = {(uint32_t)i, 0, 1, 0, 1};
        if (elem_block_[i] >= 0) goff_[goff_ptr_[elem_block_[i] + 1]++] = {(uint32_t)i, 0, (uint8_t)(elem_swap_[i] ? 3 : 2), 0, 0}; // (the row node's: b, or a)
    }
    for (size_t ti = 0; ti < tris.size(); ti++) {
        const FrameTri& t = tris[ti];
        if (t.broken) continue;
        for (int c = 0; c < 3; c++) gdiag_[gdiag_ptr_[iperm_[t.n[c]] + 1]++] = {(uint32_t)ti, 1, (uint8_t)c, (uint8_t)c, (uint8_t)c};
        for (int e = 0; e < 3; e++)
            if (tri_block_[ti][e] >= 0) { // (the block's row is the later corner in the order)
                const uint8_t i = (uint8_t)e, j = (uint8_t)((e + 1) % 3);
                goff_[goff_ptr_[tri_block_[ti][e] + 1]++] = {(uint32_t)ti, 1, tri_swap_[ti][e] ? i : j, tri_swap_[ti][e] ? j : i, (uint8_t)e};
            }
    }
    gdiag_ptr_.pop_back();
    goff_ptr_.pop_back();
}

// The assembly of columns [chunk * kAsmCols, ...) of a component: their diagonal blocks and right sides (the node's
// mass, inertia, forces, ground contact; its members' and triangles' blocks) and their off-diagonal blocks
void FemFrame::solve_gather(SoftBody& b, int comp, int chunk) {
    if (comp < (int)comp_blocks_.size() && !comp_blocks_[comp]) gather_part(b, comp, chunk, 0);
    gather_part(b, comp, chunk, 1);
}

void FemFrame::gather_blocks(SoftBody& b, int comp) {
    for (int ch = 0, n = asm_chunks(comp); ch < n; ch++) gather_part(b, comp, ch, 0);
}

// part 0: the blocks - the masses, the members' and the triangles' (and their damping's part of the right side, apart:
// rdamp_); part 1: the rest of the right side, the ground contacts' terms, rdamp_ in
void FemFrame::gather_part(SoftBody& b, int comp, int chunk, int part) {
    PROFILE_ACCUM("Frame assemble");
    const float h = sv_.h, step = sv_.step, theta = sv_.theta, dissipation = sv_.dissipation;
    const double h2 = (double)theta * h * h, hd = (double)dissipation * h * h;
    const vec3* F = b.force.data();
    const vec3* E = sv_.ext;
    const int k0 = comp_range_[comp].first, k1 = comp_range_[comp].second;
    const int ka = k0 + chunk * kAsmCols, kb = std::min(k1, ka + kAsmCols);
    auto is_fixed = [&](int k) { return b.nodes[node[perm_[k]]].inv_mass <= 0; };
    // a member's block blk (0 aa, 1 bb, 2 ba) times c, with the geometric stiffness of its tension (sign gs)
    auto member_block = [&](const Gather& g, double* D, double gs) {
        const FrameElement& e = elems[g.id];
        const Tangent& t = tan_[g.id];
        const double c = h2 + (double)sections[e.section].damping * h;
        const double* B = &mem_kw_[(size_t)g.id * kMemW + (size_t)g.blk * 36];
        for (int x = 0; x < 36; x++) D[x] += c * B[x];
        // (as add_member: the product first, then signed - the same sums to the bit)
        const double gg = h2 * t.ngeo;
        if (gg != 0) {
            double P[9];
            const double ee[3] = {t.e1.x, t.e1.y, t.e1.z};
            for (int r = 0; r < 3; r++)
                for (int q = 0; q < 3; q++) P[r * 3 + q] = gg * ((r == q ? 1.0 : 0.0) - ee[r] * ee[q]);
            for (int r = 0; r < 3; r++)
                for (int q = 0; q < 3; q++) D[r * 6 + q] += gs * P[r * 3 + q];
        }
    };
    // a triangle's block of corners (i, j) in the world's axes (R K_ij R^T, R the element's axes for the translations and
    // the rotations alike) times c, with the geometric stiffness of its tension
    auto tri_block = [&](const Gather& g, double* D) {
        const FrameTri& t = tris[g.id];
        const TriTan& tg = tri_tan_[g.id];
        const int i = g.blk, j = g.tr;
        const double c = h2 + (double)shell_sections[t.section].damping * h;
        const double E[3][3] = {{tg.e1.x, tg.e2.x, tg.e3.x}, {tg.e1.y, tg.e2.y, tg.e3.y}, {tg.e1.z, tg.e2.z, tg.e3.z}}; // (E[p][a]: axis a)
        // (the local block's nothing between the membrane's dofs and the plate's left out of E K_ij: of its 9 products
        // per row 4 or 5 - the same sums, the products by nothing dropped)
        const float* Km = &tri_k_[(size_t)g.id * kTriK];
        const float* Kp = Km + 81;
        auto m = [&](int s2, int t2) { return (double)Km[(3 * i + s2) * 9 + 3 * j + t2]; };
        auto pl = [&](int s2, int t2) { return (double)Kp[(3 * i + s2) * 9 + 3 * j + t2]; };
        for (int ra = 0; ra < 2; ra++)
            for (int cb = 0; cb < 2; cb++) {
                double T[3][3];
                for (int p = 0; p < 3; p++) {
                    const double E0 = E[p][0], E1 = E[p][1], E2 = E[p][2];
                    if (ra == 0 && cb == 0) { // (u v w against u v w)
                        T[p][0] = E0 * m(0, 0) + E1 * m(1, 0);
                        T[p][1] = E0 * m(0, 1) + E1 * m(1, 1);
                        T[p][2] = E2 * pl(0, 0);
                    } else if (ra == 0) { // (u v w against theta_x theta_y theta_z)
                        T[p][0] = E2 * pl(0, 1);
                        T[p][1] = E2 * pl(0, 2);
                        T[p][2] = E0 * m(0, 2) + E1 * m(1, 2);
                    } else if (cb == 0) { // (theta against u v w)
                        T[p][0] = E2 * m(2, 0);
                        T[p][1] = E2 * m(2, 1);
                        T[p][2] = E0 * pl(1, 0) + E1 * pl(2, 0);
                    } else { // (theta against theta)
                        T[p][0] = E0 * pl(1, 1) + E1 * pl(2, 1);
                        T[p][1] = E0 * pl(1, 2) + E1 * pl(2, 2);
                        T[p][2] = E2 * m(2, 2);
                    }
                }
                for (int p = 0; p < 3; p++)
                    for (int q = 0; q < 3; q++) D[(3 * ra + p) * 6 + 3 * cb + q] += c * (T[p][0] * E[q][0] + T[p][1] * E[q][1] + T[p][2] * E[q][2]);
            }
        const double gg = h2 * tg.g[i][j];
        if (gg != 0)
            for (int p = 0; p < 3; p++) D[p * 7] += gg;
    };
    auto member_on = [&](uint32_t ei) { return tan_[ei].on && !elems[ei].broken; };
    auto tri_on = [&](uint32_t ti) { return tri_tan_[ti].on && !tris[ti].broken && tri_k_ok_[ti] && tri_kv_.size() >= ((size_t)ti + 1) * 18; };
    for (int k = ka; k < kb; k++) {
        const int i = perm_[k];
        const Node& x = b.nodes[node[i]];
        double* D = &diag_[(size_t)k * 36];
        double* r = &rhs_[(size_t)k * 6];
        double* rd = &rdamp_[(size_t)k * 6];
        if (part == 0) {
            std::fill(D, D + 36, 0.0);
            if (x.inv_mass <= 0) {
                fixed_[i] = 1;
                for (int j = 0; j < 6; j++) D[j * 7] = 1.0, r[j] = 0.0;
            } else {
                D[0] = D[7] = D[14] = x.mass;
                D[21] = D[28] = D[35] = inertia[i];
                for (int j = 0; j < 6; j++) rd[j] = 0;
                // its members' and triangles' blocks, and their damping's part of the right side: -(theta_d h^2 + beta h) K v
                for (int q = gdiag_ptr_[k]; q < gdiag_ptr_[k + 1]; q++) {
                    const Gather& g = gdiag_[q];
                    if (g.kind == 0) {
                        if (!member_on(g.id)) continue;
                        member_block(g, D, 1.0);
                        const double cr = hd + (double)sections[elems[g.id].section].damping * h;
                        if (cr != 0) {
                            const double* kv = &mem_kv_[(size_t)g.id * 12 + 6 * g.corner];
                            for (int j = 0; j < 6; j++) rd[j] -= cr * kv[j];
                        }
                    } else {
                        if (!tri_on(g.id)) continue;
                        tri_block(g, D);
                        const double cr = hd + (double)shell_sections[tris[g.id].section].damping * h;
                        if (cr != 0) {
                            const double* kv = &tri_kv_[(size_t)g.id * 18 + 6 * g.corner];
                            for (int j = 0; j < 6; j++) rd[j] -= cr * kv[j];
                        }
                    }
                }
            }
        } else if (x.inv_mass > 0) {
            const vec3 f = F[node[i]], m = torque[i], J = impulse[i];
            // (all of it for h but the contacts; the smooth forces' guess for the later short steps comes off the held
            // impulse, which then tells the next step how far off it was)
            vec3 fc = i < (int)contact_f.size() ? contact_f[i] : vec3(0);
            if (h > step && E) fc += E[node[i]];
            const double hs = (double)h - (double)step;
            const vec3 fm = i < (int)member_f.size() ? member_f[i] : vec3(0);
            const vec3 tm = i < (int)member_t_.size() ? member_t_[i] : vec3(0);
            // (the smooth forces taken for the whole of h, corrected by what they turn out to be in the steps between:
            // held_component at once, hold at the next step; taken for this step alone they loaded the structure every
            // other step, and a cantilever's static twist came out 14% short)
            r[0] = h * (double)f.x - hs * fc.x + J.x, r[1] = h * (double)f.y - hs * fc.y + J.y, r[2] = h * (double)f.z - hs * fc.z + J.z;
            impulse[i] = (f - fm - fc) * -(float)hs;
            pred_[i] = f - fm - fc;
            r[3] = h * (double)m.x, r[4] = h * (double)m.y, r[5] = h * (double)m.z;
            tpred_[i] = m - tm;
            if (i < (int)contact_n.size() && length2(contact_n[i]) > 0) {
                // a ground contact: kappa n n^T towards the normal velocity change the contact asked for (step f.n / m, its
                // force being in f: for its short step, the later ones hold theirs); kappa 200 m makes it 99.5% of it
                // whatever the members do
                const bool stick = length2(contact_n[i]) > 2.0f;
                const vec3 cn = stick ? contact_n[i] * 0.5f : contact_n[i];
                const double kap = 200.0 * x.mass, want = step * (double)dot(f, cn) / x.mass;
                const double nn[3] = {cn.x, cn.y, cn.z};
                for (int p = 0; p < 3; p++) {
                    for (int q2 = 0; q2 < 3; q2++) D[p * 6 + q2] += kap * nn[p] * nn[q2];
                    r[p] += kap * want * nn[p];
                }
                // sticking (see World::collide_static): along the ground too, towards what its friction asked (a ring's node
                // on the ground slid a little every step under its members' forces, and a lying drum with frame rings
                // crept along at a centimetre a second)
                if (stick) { // (its sliding stopped)
                    const vec3 vt = x.v - cn * dot(x.v, cn);
                    const double wt[3] = {-vt.x, -vt.y, -vt.z};
                    for (int p = 0; p < 3; p++) {
                        for (int q2 = 0; q2 < 3; q2++) D[p * 6 + q2] += kap * ((p == q2 ? 1.0 : 0.0) - nn[p] * nn[q2]);
                        r[p] += kap * wt[p];
                    }
                }
            }
            for (int j = 0; j < 6; j++) r[j] += rd[j];
        }
        if (part != 0) continue;
        // the column's off-diagonal blocks (none to or from a pinned node)
        const bool fk = x.inv_mass <= 0;
        for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) {
            double* O = &off_[(size_t)p * 36];
            std::fill(O, O + 36, 0.0);
            if (fk || goff_ptr_[p] == goff_ptr_[p + 1] || is_fixed(row_[p])) continue;
            for (int q = goff_ptr_[p]; q < goff_ptr_[p + 1]; q++) {
                const Gather& g = goff_[q];
                if (g.kind == 0) {
                    if (member_on(g.id)) member_block(g, O, -1.0);
                } else if (tri_on(g.id)) {
                    tri_block(g, O);
                }
            }
        }
    }
}

// The large components' factorization in the team's stages (see ParPlan): the elimination tree (a column's parent the
// first row below its diagonal) cut into subtrees and the columns above them. The cut: from the roots down, the
// costliest subtree replaced by its children while the estimate of the stages' time (the slowest group of subtrees,
// the slowest group of the updates above them, the columns above in order) goes down. Costs in block updates.
void FemFrame::par_plan() {
    plans_.clear();
    comp_plan_.assign(comp_range_.size(), -1);
    const int n = (int)col_ptr_.size() - 1;
    upd_split_.assign(std::max(0, n), 0);
    row_split_.assign(std::max(0, n), 0);
    col_group_.assign(std::max(0, n), -1);
    for (int k = 0; k < n; k++) upd_split_[k] = upd_ptr_[k + 1], row_split_[k] = col_ptr_[k + 1];
    par_fail_ptr_.clear();
    par_fail_.clear();
    par_clamps_.clear();
    static const bool off = getenv("BL_NOPARFACTOR") != nullptr; // (diagnostics: every component factored in order)
    if (off) return;
    for (int c = 0; c < (int)comp_range_.size(); c++) {
        const int k0 = comp_range_[c].first, k1 = comp_range_[c].second, m = k1 - k0;
        if (m < kParNodes) continue;
        std::vector<int> par(m, -1), cptr(m + 1, 0), clist(m);
        // (costs in tenths of a block update - integers: the sums exact in any order)
        std::vector<int64_t> cost(m), sub;
        int64_t total = 0;
        for (int k = k0; k < k1; k++) {
            const int64_t r = col_ptr_[k + 1] - col_ptr_[k];
            if (r > 0) par[k - k0] = row_[col_ptr_[k]] - k0;
            cost[k - k0] = 15 + 6 * r + 5 * r * (r + 1);
            total += cost[k - k0];
        }
        sub = cost;
        for (int k = 0; k < m; k++)
            if (par[k] >= 0) sub[par[k]] += sub[k], cptr[par[k] + 1]++;
        for (int k = 0; k < m; k++) cptr[k + 1] += cptr[k];
        {
            std::vector<int> fill(cptr.begin(), cptr.end() - 1);
            for (int k = 0; k < m; k++)
                if (par[k] >= 0) clist[fill[par[k]]++] = k;
        }
        std::vector<int> front, owner(m, -1), stack;
        for (int k = 0; k < m; k++)
            if (par[k] < 0) front.push_back(k);
        std::vector<int64_t> local(m, 0), dcount(m, 0);
        auto rows_in = [&](int k, int r) { // (column k's rows in its subtree r: the first ones, its ancestors up to r)
            const int* a = &row_[col_ptr_[k + k0]];
            const int* e = &row_[col_ptr_[k + k0 + 1]];
            return (int)(std::upper_bound(a, e, r + k0) - a);
        };
        // (the costliest first, each to the lightest bin: the slowest bin's load)
        std::vector<int64_t> loc, dc;
        loc.reserve(m), dc.reserve(m);
        auto lpt = [](std::vector<int64_t>& items, int bins) {
            std::sort(items.begin(), items.end(), std::greater<int64_t>());
            int64_t load[64] = {};
            bins = std::clamp(bins, 1, 64);
            for (int64_t x : items) {
                int j = 0;
                for (int q = 1; q < bins; q++)
                    if (load[q] < load[j]) j = q;
                load[j] += x;
            }
            int64_t mx = 0;
            for (int q = 0; q < bins; q++) mx = std::max(mx, load[q]);
            return mx;
        };
        // the estimate of a cut from scratch (owner, local, dcount of it)
        auto estimate = [&](const std::vector<int>& fr) {
            std::fill(owner.begin(), owner.end(), -1);
            std::fill(dcount.begin(), dcount.end(), 0);
            loc.clear();
            int64_t topc = 0;
            for (int r : fr) {
                int64_t l = 0;
                stack.assign(1, r);
                while (!stack.empty()) {
                    const int x = stack.back();
                    stack.pop_back();
                    owner[x] = r;
                    const int rr = col_ptr_[x + k0 + 1] - col_ptr_[x + k0], s2 = rows_in(x, r);
                    const int64_t q = rr - s2;
                    l += cost[x] - 5 * q * (q + 1);
                    for (int i = s2; i < rr; i++) dcount[row_[col_ptr_[x + k0] + i] - k0] += rr - i;
                    for (int y = cptr[x]; y < cptr[x + 1]; y++) stack.push_back(clist[y]);
                }
                local[r] = l;
                loc.push_back(l);
            }
            dc.clear();
            for (int k = 0; k < m; k++) {
                if (owner[k] < 0) topc += cost[k];
                if (dcount[k] > 0) dc.push_back(dcount[k]);
            }
            return lpt(loc, kParGroups) + 10 * lpt(dc, kParDefers) + topc;
        };
        // the cut moved down one subtree root r at a time, incrementally: r's column goes above the cut (its updates of
        // the blocks above leave the deferred ones), and of the columns below it only those with a block in row r change -
        // that row is above the cut now (one more deferred update set, the local cost less); their subtrees' sums along
        // the path up to r (a whole subtree walked again for each move was a third of a millisecond a pattern)
        std::vector<int> tptr(m + 1, 0), tcol, tpos;          // (per row r: the columns with a block in it, the block's place)
        for (int k = 0; k < m; k++)
            for (int p = col_ptr_[k + k0]; p < col_ptr_[k + k0 + 1]; p++) tptr[row_[p] - k0 + 1]++;
        for (int k = 0; k < m; k++) tptr[k + 1] += tptr[k];
        tcol.resize(tptr[m]), tpos.resize(tptr[m]);
        {
            std::vector<int> fill(tptr.begin(), tptr.end() - 1);
            for (int k = 0; k < m; k++)
                for (int p = col_ptr_[k + k0]; p < col_ptr_[k + k0 + 1]; p++) {
                    const int r = row_[p] - k0;
                    tcol[fill[r]] = k, tpos[fill[r]] = p - col_ptr_[k + k0], fill[r]++;
                }
        }
        std::vector<int64_t> qa(m, 0);                        // (per column: its rows above the cut)
        // (the subtrees' local sums: a Fenwick tree over the tree's preorder, a subtree an interval of it)
        std::vector<int> tin(m, 0), tout(m, 0);
        {
            int t = 0;
            for (int r : front) {
                stack.assign(1, r);
                while (!stack.empty()) {
                    const int x = stack.back();
                    stack.pop_back();
                    if (x < 0) { tout[-x - 1] = t; continue; }
                    tin[x] = t++;
                    stack.push_back(-x - 1);
                    for (int y = cptr[x]; y < cptr[x + 1]; y++) stack.push_back(clist[y]);
                }
            }
        }
        std::vector<int64_t> fen(m + 1, 0);
        auto fen_add = [&](int i, int64_t v) { for (++i; i <= m; i += i & -i) fen[i] += v; };
        auto fen_sum = [&](int i) { int64_t r = 0; for (; i > 0; i -= i & -i) r += fen[i]; return r; }; // ([0, i))
        for (int x = 0; x < m; x++) fen_add(tin[x], cost[x]);
        auto subtree_local = [&](int x) { return fen_sum(tout[x]) - fen_sum(tin[x]); };
        std::vector<int> topl;
        std::fill(dcount.begin(), dcount.end(), 0);
        for (int r : front) local[r] = subtree_local(r);
        // (the deferred groups' slowest taken as the bound LPT keeps near: the largest one or an even share - the
        // search's every step sorted and binned them, a third of an analysis)
        auto value = [&]() {
            loc.clear();
            for (int r : front) loc.push_back(local[r]);
            int64_t topc = 0, dsum = 0, dmax = 0;
            for (int k : topl) {
                topc += cost[k];
                if (dcount[k] > 0) dsum += dcount[k], dmax = std::max(dmax, dcount[k]);
            }
            return lpt(loc, kParGroups) + 10 * std::max(dmax, (dsum + kParDefers - 1) / kParDefers) + topc;
        };
        int64_t best = INT64_MAX;
        std::vector<int> best_front;
        static const bool sdbg = getenv("BL_SEARCHDBG") != nullptr;
        int best_it = 0, its = 0;
        for (int it = 0; it < 64; it++) {
            const int64_t e = value();
            its = it + 1;
            if (e < best) best = e, best_front = front, best_it = it;
            int big = -1;
            for (size_t i = 0; i < front.size(); i++)
                if (big < 0 || local[front[i]] > local[front[big]]) big = (int)i;
            const int r = front[big];
            if (cptr[r] == cptr[r + 1]) break;
            // r above the cut: its own updates of the blocks above it are the top's now
            {
                const int rr = col_ptr_[r + k0 + 1] - col_ptr_[r + k0];
                for (int i = (int)(rr - qa[r]); i < rr; i++) dcount[row_[col_ptr_[r + k0] + i] - k0] -= rr - i;
            }
            topl.insert(std::upper_bound(topl.begin(), topl.end(), r), r);
            for (int t = tptr[r]; t < tptr[r + 1]; t++) {
                const int x = tcol[t];
                const int rr = col_ptr_[x + k0 + 1] - col_ptr_[x + k0];
                qa[x]++;
                dcount[r] += rr - tpos[t];
                fen_add(tin[x], -10 * qa[x]);
            }
            fen_add(tin[r], -cost[r]); // (r's own column: above the cut)
            front.erase(front.begin() + big);
            for (int y = cptr[r]; y < cptr[r + 1]; y++) front.push_back(clist[y]), local[clist[y]] = subtree_local(clist[y]);
        }
        if (sdbg) printf("search: comp %d of %d nodes: best at %d of %d iterations, %lld\n", c, m, best_it, its, (long long)best / 10);
        if (best_front.size() < 2 || best > 0.8 * total) continue; // (not worth the stages)
        estimate(best_front); // (owner, local of the cut chosen)
        if (prof::g_trace) prof::trace_mark("plan", 3);
        ParPlan pl;
        pl.comp = c;
        // the subtrees into groups (the costliest first, each to the lightest group)
        std::vector<int> order = best_front;
        std::sort(order.begin(), order.end(), [&](int a, int b2) { return local[a] > local[b2] || (local[a] == local[b2] && a < b2); });
        const int ng = std::min<int>(kParGroups, (int)order.size());
        std::vector<double> load(ng, 0.0);
        std::vector<int> group_of(m, -1);
        for (int r : order) {
            const int g = (int)(std::min_element(load.begin(), load.end()) - load.begin());
            load[g] += local[r];
            group_of[r] = g;
        }
        std::vector<std::vector<int>> gc(ng);
        for (int k = 0; k < m; k++) {
            if (owner[k] < 0) {
                pl.top.push_back(k + k0);
                continue;
            }
            gc[group_of[owner[k]]].push_back(k + k0);
            // (its updates of the blocks above its subtree: those of its rows from there on, the last of its list)
            const int rr = col_ptr_[k + k0 + 1] - col_ptr_[k + k0], s2 = rows_in(k, owner[k]);
            row_split_[k + k0] = col_ptr_[k + k0] + s2, col_group_[k + k0] = group_of[owner[k]];
            int u = upd_ptr_[k + k0];
            for (int a = 0; a < s2; a++) u += rr - a;
            upd_split_[k + k0] = u;
        }
        pl.gptr.push_back(0);
        for (const auto& v : gc) {
            pl.gcols.insert(pl.gcols.end(), v.begin(), v.end());
            pl.gptr.push_back((int)pl.gcols.size());
        }
        // (the rows of the columns above: their blocks, by column - the forward substitution's for them, left-looking)
        {
            std::vector<int> tpos(m, -1);
            for (size_t t = 0; t < pl.top.size(); t++) tpos[pl.top[t] - k0] = (int)t;
            pl.tptr.assign(pl.top.size() + 1, 0);
            for (int k = k0; k < k1; k++)
                for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++)
                    if (tpos[row_[p] - k0] >= 0) pl.tptr[tpos[row_[p] - k0] + 1]++;
            for (size_t t = 0; t < pl.top.size(); t++) pl.tptr[t + 1] += pl.tptr[t];
            pl.tblk.resize(pl.tptr.back()), pl.tcol.resize(pl.tptr.back());
            std::vector<int> fill(pl.tptr.begin(), pl.tptr.end() - 1);
            for (int k = k0; k < k1; k++)
                for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++)
                    if (const int t = tpos[row_[p] - k0]; t >= 0) pl.tblk[fill[t]] = p, pl.tcol[fill[t]] = k, fill[t]++;
        }
        if (prof::g_trace) prof::trace_mark("plan", 4);
        // the deferred updates by their block's column, the columns into groups by their count
        std::vector<std::vector<int>> by_col(m);
        for (int k = 0; k < m; k++) {
            if (owner[k] < 0) continue;
            for (int u = upd_split_[k + k0]; u < upd_ptr_[k + k0 + 1]; u++) by_col[row_[upd_[u].pi] - k0].push_back(u);
        }
        std::vector<int> cols;
        for (int k = 0; k < m; k++)
            if (!by_col[k].empty()) cols.push_back(k);
        std::sort(cols.begin(), cols.end(), [&](int a, int b2) { return by_col[a].size() > by_col[b2].size() || (by_col[a].size() == by_col[b2].size() && a < b2); });
        const int nd = std::max(1, std::min<int>(kParDefers, (int)cols.size()));
        std::vector<size_t> dload(nd, 0);
        std::vector<std::vector<int>> dg(nd);
        for (int k : cols) {
            const int g = (int)(std::min_element(dload.begin(), dload.end()) - dload.begin());
            dload[g] += by_col[k].size();
            dg[g].push_back(k);
        }
        pl.dptr.push_back(0);
        for (auto& v : dg) {
            std::sort(v.begin(), v.end());
            for (int k : v) pl.dupd.insert(pl.dupd.end(), by_col[k].begin(), by_col[k].end());
            pl.dptr.push_back((int)pl.dupd.size());
        }
        if (prof::g_trace) prof::trace_mark("plan", 5);
        par_fail_ptr_.push_back((int)par_fail_.size());
        par_fail_.resize(par_fail_.size() + ng, 0);
        par_clamps_.resize(par_fail_.size(), 0);
        comp_plan_[c] = (int)plans_.size();
        plans_.push_back(std::move(pl));
        if (getenv("BL_FACTORDBG")) {
            printf("  component %d in the team's stages: %d groups (%zu subtrees), %d deferred groups (%zu updates), %zu columns above, estimate %lld of %lld\n", c, ng,
                   best_front.size(), nd, plans_.back().dupd.size(), plans_.back().top.size(), (long long)best / 10, (long long)total / 10);
            if (getenv("BL_TOPDBG")) {
                int64_t tc = 0;
                for (int k : plans_.back().top) tc += cost[k - k0];
                printf("    top cost %lld, groups:", (long long)tc / 10);
                for (double x : load) printf(" %.0f", x / 10);
                printf("\n    top (col rows parent children):");
                for (int k : plans_.back().top) {
                    int nch = 0;
                    for (int y = cptr[k - k0]; y < cptr[k - k0 + 1]; y++) nch += owner[clist[y]] < 0;
                    printf(" %d:%d>%d/%d", k - k0, col_ptr_[k + 1] - col_ptr_[k], par[k - k0], nch);
                }
                printf("\n");
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------ element mechanics
namespace {

// where the co-rotated frame takes its twist from: 0 end a, 1 end b, between them (see kinematics)
float twist_weight(const FrameElement& e) {
    const bool fa = e.end_a == FJ_BALL || e.end_a == FJ_SWIVEL, fb = e.end_b == FJ_BALL || e.end_b == FJ_SWIVEL;
    return fa == fb ? 0.5f : fa ? 1.0f : 0.0f;
}

struct Kin {
    vec3 e1, e2, e3;
    float L = 0;
    double dL = 0;   // elongation (double: see FemFrame::xd)
    vec3 ra, rb; // end rotations against the co-rotated frame (element axes: x twist, y and z bending)
};

bool kinematics(const FemFrame& f, const FrameElement& e, Kin& k) {
    const double* xa = &f.xd[e.a * 3];
    const double* xb = &f.xd[e.b * 3];
    const double dx = xb[0] - xa[0], dy = xb[1] - xa[1], dz = xb[2] - xa[2];
    const double L = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(L > 1e-6) || !std::isfinite(L)) return false;
    k.L = (float)L;
    k.dL = L - (double)e.L0;
    k.e1 = vec3((float)(dx / L), (float)(dy / L), (float)(dz / L));
    // the frames the two nodes give the member, their mean turned onto the chord: the co-rotated frame (an end whose
    // joint frees the twist has no say in it: the member's energy would change as that node turns, with no torque
    // there to show for it, and such a pull out of nowhere drove the uprights on their ball joints round in circles)
    const quat Ea = normalize(f.q[e.a] * e.qa), Eb = normalize(f.q[e.b] * e.qb);
    const float t = twist_weight(e);
    const quat Em = t <= 0.0f ? Ea : t >= 1.0f ? Eb : slerp(Ea, Eb, t);
    const quat E = normalize(quat_from_to(normalize(Em.rotate(vec3(1, 0, 0))), k.e1) * Em);
    const quat Ec = conj(E);
    k.ra = quat_log(Ec * Ea);
    k.rb = quat_log(Ec * Eb);
    k.e2 = E.rotate(vec3(0, 1, 0));
    k.e3 = E.rotate(vec3(0, 0, 1));
    return true;
}

// The member's elastic stiffness (the generalized one of its deformations: elongation, twist, bending at the ends)
struct Stiff {
    float ka, kt;
    float cy[3], cz[3]; // [aa, ab, bb]
};

void condense(float c[3], bool pin_a, bool pin_b) {
    if (pin_a && pin_b) c[0] = c[1] = c[2] = 0;
    else if (pin_a) c[2] = c[0] > 0 ? c[2] - c[1] * c[1] / c[0] : 0, c[0] = c[1] = 0;
    else if (pin_b) c[0] = c[2] > 0 ? c[0] - c[1] * c[1] / c[2] : 0, c[1] = c[2] = 0;
}

// what a joint frees at a member's end: the twist, the rotation about y, about z; and its flexibility (1 / N m/rad)
struct EndFlex {
    bool rel[3] = {false, false, false};
    float f = 0;
};
EndFlex end_flex(uint8_t j, const FrameSection& s) {
    EndFlex e;
    switch (j) {
    case FJ_BALL: e.rel[0] = e.rel[1] = e.rel[2] = true; break;
    case FJ_HINGE_V: e.rel[2] = true; break; // (free about z: it swings in the x-y plane, the vertical one)
    case FJ_HINGE_H: e.rel[1] = true; break;
    case FJ_SWIVEL: e.rel[0] = true; break;
    case FJ_ELASTIC: e.f = s.joint_k > 0 ? 1.0f / s.joint_k : 0.0f; break;
    default: break;
    }
    return e;
}

// the 2 x 2 end stiffness of one bending plane with the ends' releases and joint flexibilities: a released end is
// condensed out, a flexible one adds its flexibility to the member's (K^-1 + diag(fa, fb))^-1
void ends2(float c[3], bool ra, bool rb, float fa, float fb) {
    if (ra || rb) {
        condense(c, ra, rb);
        if (ra != rb) {
            float& k = ra ? c[2] : c[0];
            const float f = ra ? fb : fa;
            if (f > 0 && k > 0) k = 1.0f / (1.0f / k + f);
        }
        return;
    }
    if (fa <= 0 && fb <= 0) return;
    const double det = (double)c[0] * c[2] - (double)c[1] * c[1];
    if (!(det > 0)) return;
    const double F00 = c[2] / det + fa, F01 = -c[1] / det, F11 = c[0] / det + fb;
    const double d2 = F00 * F11 - F01 * F01;
    c[0] = (float)(F11 / d2), c[1] = (float)(-F01 / d2), c[2] = (float)(F00 / d2);
}

Stiff stiffness(const FrameSection& s, const FrameElement& e) {
    Stiff k;
    const float L = e.L0;
    const EndFlex fa = end_flex(e.end_a, s), fb = end_flex(e.end_b, s);
    k.ka = s.axial * s.E * s.A / L;
    k.kt = s.G * s.J / L;
    if (fa.rel[0] || fb.rel[0]) k.kt = 0;
    else if (fa.f + fb.f > 0) k.kt = 1.0f / (1.0f / k.kt + fa.f + fb.f);
    auto bend = [&](float I, float As, float c[3], int axis) {
        const float phi = As > 0 ? 12.0f * s.E * I / (s.G * As * L * L) : 0.0f; // shear deformation (Timoshenko)
        const float k0 = s.E * I / (L * (1.0f + phi));
        c[0] = c[2] = (4.0f + phi) * k0;
        c[1] = (2.0f - phi) * k0;
        ends2(c, fa.rel[axis], fb.rel[axis], fa.f, fb.f);
    };
    bend(s.Iy, s.As_z, k.cy, 1);
    bend(s.Iz, s.As_y, k.cz, 2);
    return k;
}

// ------------------------------------------------------------------------------------------------ triangle elements
// The Discrete Kirchhoff Triangle's curvatures (Batoz, Bathe, Ho 1980): kappa = B U at (xi, eta) of the triangle's
// area coordinates, U = [w, theta_x, theta_y] at its corners (theta_x = w_y, theta_y = -w_x: the rotations about the
// element's x and y axes), kappa = [beta_x,x, beta_y,y, beta_x,y + beta_y,x]; A2 twice the area
void dkt_b(const double x[3], const double y[3], double xi, double eta, double B[3][9]) {
    const double x23 = x[1] - x[2], x31 = x[2] - x[0], x12 = x[0] - x[1];
    const double y23 = y[1] - y[2], y31 = y[2] - y[0], y12 = y[0] - y[1];
    double P[7], q[7], r[7], t[7];
    const double xs[3] = {x23, x31, x12}, ys[3] = {y23, y31, y12};
    for (int k = 4; k <= 6; k++) {
        const double xij = xs[k - 4], yij = ys[k - 4], l2 = xij * xij + yij * yij;
        P[k] = -6 * xij / l2, q[k] = 3 * xij * yij / l2, t[k] = -6 * yij / l2, r[k] = 3 * yij * yij / l2;
    }
    const double a = 1 - 2 * xi, bb = 1 - 2 * eta;
    const double Hx_xi[9] = {P[6] * a + (P[5] - P[6]) * eta, q[6] * a - (q[5] + q[6]) * eta, -4 + 6 * (xi + eta) + r[6] * a - eta * (r[5] + r[6]),
                             -P[6] * a + eta * (P[4] + P[6]), q[6] * a - eta * (q[6] - q[4]), -2 + 6 * xi + r[6] * a + eta * (r[4] - r[6]),
                             -eta * (P[5] + P[4]), eta * (q[4] - q[5]), -eta * (r[5] - r[4])};
    const double Hy_xi[9] = {t[6] * a + eta * (t[5] - t[6]), 1 + r[6] * a - eta * (r[5] + r[6]), -q[6] * a + eta * (q[5] + q[6]),
                             -t[6] * a + eta * (t[4] + t[6]), -1 + r[6] * a + eta * (r[4] - r[6]), -q[6] * a - eta * (q[4] - q[6]),
                             -eta * (t[4] + t[5]), eta * (r[4] - r[5]), -eta * (q[4] - q[5])};
    const double Hx_eta[9] = {-P[5] * bb - xi * (P[6] - P[5]), q[5] * bb - xi * (q[5] + q[6]), -4 + 6 * (xi + eta) + r[5] * bb - xi * (r[5] + r[6]),
                              xi * (P[4] + P[6]), xi * (q[4] - q[6]), -xi * (r[6] - r[4]),
                              P[5] * bb - xi * (P[4] + P[5]), q[5] * bb + xi * (q[4] - q[5]), -2 + 6 * eta + r[5] * bb + xi * (r[4] - r[5])};
    const double Hy_eta[9] = {-t[5] * bb - xi * (t[6] - t[5]), 1 + r[5] * bb - xi * (r[5] + r[6]), -q[5] * bb + xi * (q[5] + q[6]),
                              xi * (t[4] + t[6]), xi * (r[4] - r[6]), -xi * (q[4] - q[6]),
                              t[5] * bb - xi * (t[4] + t[5]), -1 + r[5] * bb + xi * (r[4] - r[5]), -q[5] * bb - xi * (q[4] - q[5])};
    const double A2 = x31 * y12 - x12 * y31, iA = 1.0 / A2;
    for (int j = 0; j < 9; j++) {
        B[0][j] = (y31 * Hx_xi[j] + y12 * Hx_eta[j]) * iA;
        B[1][j] = (-x31 * Hy_xi[j] - x12 * Hy_eta[j]) * iA;
        B[2][j] = (-x31 * Hx_xi[j] - x12 * Hx_eta[j] + y31 * Hy_xi[j] + y12 * Hy_eta[j]) * iA;
    }
}
constexpr double kGauss[3][2] = {{1.0 / 6, 1.0 / 6}, {2.0 / 3, 1.0 / 6}, {1.0 / 6, 2.0 / 3}}; // (weights 1/6: the area's 1/2)

// The membrane's shape gradients: dN_i/dx = b_i / A2, dN_i/dy = c_i / A2
void tri_grad(const float X[3][2], double b[3], double c[3], double& A2) {
    for (int i = 0; i < 3; i++) {
        const int j = (i + 1) % 3, k = (i + 2) % 3;
        b[i] = (double)X[j][1] - X[k][1];
        c[i] = (double)X[k][0] - X[j][0];
    }
    A2 = ((double)X[1][0] - X[0][0]) * ((double)X[2][1] - X[0][1]) - ((double)X[2][0] - X[0][0]) * ((double)X[1][1] - X[0][1]);
}

// The element's stiffness in its own frame: 18 dofs, per corner [u, v, w, theta_x, theta_y, theta_z]
void tri_local_k(const float X[3][2], const ShellSection& s, double K[18][18]) {
    for (int i = 0; i < 18; i++)
        for (int j = 0; j < 18; j++) K[i][j] = 0;
    double b[3], c[3], A2;
    tri_grad(X, b, c, A2);
    if (!(A2 > 0)) return;
    const double A = 0.5 * A2, nu = s.nu;
    // the membrane (constant strain): A B^T D B, D the plane stress elasticity times the thickness
    const double Dm = (double)s.E * s.t / (1 - nu * nu);
    double Bm[3][6];
    for (int i = 0; i < 3; i++) {
        Bm[0][2 * i] = b[i] / A2, Bm[1][2 * i] = 0, Bm[2][2 * i] = c[i] / A2;
        Bm[0][2 * i + 1] = 0, Bm[1][2 * i + 1] = c[i] / A2, Bm[2][2 * i + 1] = b[i] / A2;
    }
    const double D3[3][3] = {{1, nu, 0}, {nu, 1, 0}, {0, 0, 0.5 * (1 - nu)}};
    for (int p = 0; p < 6; p++)
        for (int q = 0; q < 6; q++) {
            double v = 0;
            for (int r = 0; r < 3; r++)
                for (int t = 0; t < 3; t++) v += Bm[r][p] * D3[r][t] * Bm[t][q];
            K[6 * (p / 2) + p % 2][6 * (q / 2) + q % 2] += A * Dm * v;
        }
    // the bending (DKT, three points)
    const double x[3] = {X[0][0], X[1][0], X[2][0]}, y[3] = {X[0][1], X[1][1], X[2][1]};
    const double Db = s.D();
    for (const auto& g : kGauss) {
        double B[3][9];
        dkt_b(x, y, g[0], g[1], B);
        for (int p = 0; p < 9; p++)
            for (int q = 0; q < 9; q++) {
                double v = 0;
                for (int r = 0; r < 3; r++)
                    for (int t = 0; t < 3; t++) v += B[r][p] * D3[r][t] * B[t][q];
                K[6 * (p / 3) + 2 + p % 3][6 * (q / 3) + 2 + q % 3] += A2 / 6.0 * Db * v;
            }
    }
    // the drilling: each corner's theta_z against the element's in-plane turning omega = (v_x - u_y) / 2, a spring of
    // kd each (its energy sum kd (theta_z - omega)^2 / 2: no stiffness in a rigid turn, forces in balance)
    const double G = (double)s.E / (2 * (1 + nu)), kd = (double)s.drill * G * s.t * A;
    double gu[3], gv[3];
    for (int i = 0; i < 3; i++) gu[i] = -c[i] / (2 * A2), gv[i] = b[i] / (2 * A2);
    for (int i = 0; i < 3; i++) {
        K[6 * i + 5][6 * i + 5] += kd;
        for (int j = 0; j < 3; j++) {
            K[6 * i + 5][6 * j] -= kd * gu[j], K[6 * j][6 * i + 5] -= kd * gu[j];
            K[6 * i + 5][6 * j + 1] -= kd * gv[j], K[6 * j + 1][6 * i + 5] -= kd * gv[j];
            K[6 * i][6 * j] += 3 * kd * gu[i] * gu[j], K[6 * i][6 * j + 1] += 3 * kd * gu[i] * gv[j];
            K[6 * i + 1][6 * j] += 3 * kd * gv[i] * gu[j], K[6 * i + 1][6 * j + 1] += 3 * kd * gv[i] * gv[j];
        }
    }
}

// The co-rotated frame and the deformation: the normal of the current triangle, and in its plane the turning that
// best fits the rest shape onto the current one (least squares: the displacements carry no turn); the corners'
// displacements (w = 0: the plane goes through them) and their rotations against that frame, the plastic rest ones off
struct TriKin {
    vec3 e1, e2, e3;
    double d[18];
};

bool tri_kinematics(const FemFrame& f, const FrameTri& t, TriKin& k) {
    double x[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) x[i][j] = f.xd[t.n[i] * 3 + j];
    const double a[3] = {x[1][0] - x[0][0], x[1][1] - x[0][1], x[1][2] - x[0][2]}, c[3] = {x[2][0] - x[0][0], x[2][1] - x[0][1], x[2][2] - x[0][2]};
    double n[3] = {a[1] * c[2] - a[2] * c[1], a[2] * c[0] - a[0] * c[2], a[0] * c[1] - a[1] * c[0]};
    const double nl = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]), al = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (!(nl > 1e-12) || !(al > 1e-9) || !std::isfinite(nl)) return false;
    for (double& v : n) v /= nl;
    const double t1[3] = {a[0] / al, a[1] / al, a[2] / al};
    const double t2[3] = {n[1] * t1[2] - n[2] * t1[1], n[2] * t1[0] - n[0] * t1[2], n[0] * t1[1] - n[1] * t1[0]};
    double cen[3];
    for (int j = 0; j < 3; j++) cen[j] = (x[0][j] + x[1][j] + x[2][j]) / 3.0;
    double p[3][2];
    for (int i = 0; i < 3; i++) {
        const double r[3] = {x[i][0] - cen[0], x[i][1] - cen[1], x[i][2] - cen[2]};
        p[i][0] = r[0] * t1[0] + r[1] * t1[1] + r[2] * t1[2];
        p[i][1] = r[0] * t2[0] + r[1] * t2[1] + r[2] * t2[2];
    }
    double sn = 0, cs = 0;
    for (int i = 0; i < 3; i++) sn += t.X[i][0] * p[i][1] - t.X[i][1] * p[i][0], cs += t.X[i][0] * p[i][0] + t.X[i][1] * p[i][1];
    const double phi = std::atan2(sn, cs), cp = std::cos(phi), sp = std::sin(phi);
    double e1[3], e2[3];
    for (int j = 0; j < 3; j++) e1[j] = cp * t1[j] + sp * t2[j], e2[j] = -sp * t1[j] + cp * t2[j];
    k.e1 = vec3((float)e1[0], (float)e1[1], (float)e1[2]);
    k.e2 = vec3((float)e2[0], (float)e2[1], (float)e2[2]);
    k.e3 = vec3((float)n[0], (float)n[1], (float)n[2]);
    const quat Ec = conj(from_mat3(mat3(k.e1, k.e2, k.e3)));
    for (int i = 0; i < 3; i++) {
        // (in the element's axes: the fit turned the rest shape by phi)
        k.d[6 * i] = cp * p[i][0] + sp * p[i][1] - t.X[i][0];
        k.d[6 * i + 1] = -sp * p[i][0] + cp * p[i][1] - t.X[i][1];
        k.d[6 * i + 2] = 0;
        const vec3 th = quat_log(Ec * normalize(f.q[t.n[i]] * t.r0[i])) - t.th0[i];
        k.d[6 * i + 3] = th.x, k.d[6 * i + 4] = th.y, k.d[6 * i + 5] = th.z;
    }
    return true;
}

} // namespace

void FemFrame::tri_build_k(uint32_t ti) {
    double K[18][18];
    const FrameTri& t = tris[ti];
    tri_local_k(t.X, shell_sections[t.section], K);
    float* P = &tri_k_[(size_t)ti * kTriK];
    for (int a = 0; a < 9; a++)
        for (int c = 0; c < 9; c++) {
            const int im = 6 * (a / 3) + kMemDof[a % 3], jm = 6 * (c / 3) + kMemDof[c % 3];
            const int ip = 6 * (a / 3) + kPlateDof[a % 3], jp = 6 * (c / 3) + kPlateDof[c % 3];
            P[a * 9 + c] = (float)K[std::min(im, jm)][std::max(im, jm)];
            P[81 + a * 9 + c] = (float)K[std::min(ip, jp)][std::max(ip, jp)];
        }
    // (the bending's curvatures at the three points from the corners' rotations alone: the plastic check's)
    const double x[3] = {t.X[0][0], t.X[1][0], t.X[2][0]}, y[3] = {t.X[0][1], t.X[1][1], t.X[2][1]};
    float* Bc = &tri_b_[(size_t)ti * kTriB];
    for (int g = 0; g < 3; g++) {
        double B[3][9];
        dkt_b(x, y, kGauss[g][0], kGauss[g][1], B);
        for (int r = 0; r < 3; r++)
            for (int i = 0; i < 3; i++) Bc[g * 18 + r * 6 + 2 * i] = (float)B[r][3 * i + 1], Bc[g * 18 + r * 6 + 2 * i + 1] = (float)B[r][3 * i + 2];
    }
    tri_k_ok_[ti] = 1;
}

void FemFrame::tri_stiffness(uint32_t ti, double K[18][18]) const {
    tri_local_k(tris[ti].X, shell_sections[tris[ti].section], K);
}

bool FemFrame::tri_state(uint32_t ti, float d[18], vec3 axes[3]) const {
    TriKin k;
    if (ti >= tris.size() || !tri_kinematics(*this, tris[ti], k)) return false;
    for (int i = 0; i < 18; i++) d[i] = (float)k.d[i];
    axes[0] = k.e1, axes[1] = k.e2, axes[2] = k.e3;
    return true;
}

void FemFrame::eval_tris(SoftBody& b, int chunk, std::vector<Event>& evs) {
    const size_t t_end = std::min(tris.size(), (size_t)(chunk + 1) * kTriChunk);
    for (size_t ti = (size_t)chunk * kTriChunk; ti < t_end; ti++) {
        FrameTri& t = tris[ti];
        TriOut& o = tri_out_[ti];
        TriTan& tg = tri_tan_[ti];
        o.on = tg.on = false;
        if (t.broken) continue;
        TriKin k;
        if (!tri_kinematics(*this, t, k)) continue;
        const ShellSection& s = shell_sections[t.section];
        double b3[3], c3[3], A2;
        tri_grad(t.X, b3, c3, A2);
        if (!(A2 > 0)) continue;
        const double nu = s.nu, Dm = (double)s.E / (1 - nu * nu);
        // the membrane's strain and stress (Pa)
        auto membrane = [&](double eps[3], double sig[3]) {
            eps[0] = eps[1] = eps[2] = 0;
            for (int i = 0; i < 3; i++) {
                eps[0] += b3[i] * k.d[6 * i] / A2;
                eps[1] += c3[i] * k.d[6 * i + 1] / A2;
                eps[2] += (c3[i] * k.d[6 * i] + b3[i] * k.d[6 * i + 1]) / A2;
            }
            sig[0] = Dm * (eps[0] + nu * eps[1]), sig[1] = Dm * (nu * eps[0] + eps[1]), sig[2] = Dm * 0.5 * (1 - nu) * eps[2];
        };
        double eps[3], sig[3];
        membrane(eps, sig);
        auto von_mises = [](const double m[3]) { return std::sqrt(std::max(0.0, m[0] * m[0] - m[0] * m[1] + m[1] * m[1] + 3 * m[2] * m[2])); };
        float util = 0;
        if (s.yield > 0 && b.allow_deform) {
            // the membrane flows: radially back to the yield stress, the rest shape following the stretch (the element's
            // stiffness rebuilt once it has flowed a little)
            const double seq = von_mises(sig);
            util = (float)(seq / s.yield);
            if (seq > s.yield) {
                const double r = 1.0 - s.yield / seq;
                for (int i = 0; i < 3; i++) t.X[i][0] += (float)(r * k.d[6 * i]), t.X[i][1] += (float)(r * k.d[6 * i + 1]), k.d[6 * i] *= 1 - r, k.d[6 * i + 1] *= 1 - r;
                const float mx = (t.X[0][0] + t.X[1][0] + t.X[2][0]) / 3.0f, my = (t.X[0][1] + t.X[1][1] + t.X[2][1]) / 3.0f;
                for (int i = 0; i < 3; i++) t.X[i][0] -= mx, t.X[i][1] -= my;
                // (the plastic stretch: the rest shape against the authored one, F = [dX] [dX0]^-1, its largest principal
                // stretch; crushed it folds and does not tear)
                const double a0x = t.X0[1][0] - t.X0[0][0], a0y = t.X0[1][1] - t.X0[0][1], b0x = t.X0[2][0] - t.X0[0][0], b0y = t.X0[2][1] - t.X0[0][1];
                const double ax = t.X[1][0] - t.X[0][0], ay = t.X[1][1] - t.X[0][1], bx = t.X[2][0] - t.X[0][0], by = t.X[2][1] - t.X[0][1];
                const double det = a0x * b0y - b0x * a0y;
                if (std::fabs(det) > 1e-12) {
                    const double i00 = b0y / det, i01 = -b0x / det, i10 = -a0y / det, i11 = a0x / det;
                    const double F00 = ax * i00 + bx * i10, F01 = ax * i01 + bx * i11, F10 = ay * i00 + by * i10, F11 = ay * i01 + by * i11;
                    const double C00 = F00 * F00 + F10 * F10, C01 = F00 * F01 + F10 * F11, C11 = F01 * F01 + F11 * F11;
                    const double lmax = 0.5 * (C00 + C11) + std::sqrt(0.25 * (C00 - C11) * (C00 - C11) + C01 * C01);
                    t.dmg = std::max(t.dmg, (float)(std::sqrt(lmax) - 1.0));
                }
                tri_k_ok_[ti] = 0;
                tri_grad(t.X, b3, c3, A2);
                membrane(eps, sig);
            }
            // the bending: the moments at the three points, the largest against the plastic moment of the plate; past it
            // the corners' rest rotations follow theirs, radially back to it
            if (!tri_k_ok_[ti]) tri_build_k((uint32_t)ti);
            const float* Bc = &tri_b_[ti * kTriB];
            const double D = s.D(), Mp = 0.25 * s.yield * (double)s.t * s.t;
            double meq = 0;
            for (int g = 0; g < 3; g++) {
                double kap[3] = {0, 0, 0};
                for (int r = 0; r < 3; r++)
                    for (int i = 0; i < 3; i++) kap[r] += Bc[g * 18 + r * 6 + 2 * i] * k.d[6 * i + 3] + Bc[g * 18 + r * 6 + 2 * i + 1] * k.d[6 * i + 4];
                const double m[3] = {D * (kap[0] + nu * kap[1]), D * (nu * kap[0] + kap[1]), D * 0.5 * (1 - nu) * kap[2]};
                meq = std::max(meq, von_mises(m));
            }
            util = std::max(util, (float)(meq / Mp));
            if (meq > Mp) {
                const double r = 1.0 - Mp / meq;
                for (int i = 0; i < 3; i++) {
                    t.th0[i].x += (float)(r * k.d[6 * i + 3]), t.th0[i].y += (float)(r * k.d[6 * i + 4]);
                    k.d[6 * i + 3] *= 1 - r, k.d[6 * i + 4] *= 1 - r;
                }
                t.dmg = std::max(t.dmg, 1e-5f); // (yielded: shown so)
            }
            // (the plastic rotations into the rest frames past a few hundredths: the rotations measured stay small)
            for (int i = 0; i < 3; i++)
                if (dot(t.th0[i], t.th0[i]) > 0.02f * 0.02f) t.r0[i] = normalize(t.r0[i] * quat_exp(-t.th0[i])), t.th0[i] = vec3(0);
            // (past the elongation it tears an edge free, each further tear half the elongation later; a pattern's line
            // parts sooner, the zone between the lines later: its weakest edge's; a bisected triangle a sqrt(2) later a
            // level - its stretch is the mean over a smaller piece of the tear's band, the energy a tear takes the
            // same on a finer shell: halved it tore at once, shreds. On its way there it is bisected: the zone that tears
            // gets the detail first)
            const float el = s.elongation * (1.0f + 0.5f * t.tears) * tear_band(t.level) * (float)std::min(t.es[0], std::min(t.es[1], t.es[2])) * (1.0f / 64.0f);
            if (t.wait && t.dmg <= 2.0f * el) t.wait--; // (far past its tear it goes now, shard or not: see tear_tri)
            else if (t.tears < 3 && t.dmg > el && b.allow_break) t.wait = 0, evs.push_back({(uint32_t)ti, (uint8_t)4, 0.0f});
            // (one that cannot tear along an edge - a piece of it hanging on a fixed or held node - lets go of its corners
            // far past its tear: perfectly plastic, it flowed on without end, its free corners flung round the node)
            else if (t.tears >= 3 && !t.held && t.dmg > 3.0f * el && b.allow_break) evs.push_back({(uint32_t)ti, (uint8_t)6, 0.0f});
            else if (!t.whole && t.level < s.max_level && t.dmg > s.refine_at * el * ((t.line & 8u) ? 0.1f : 1.0f) && b.allow_break)
                evs.push_back({(uint32_t)ti, (uint8_t)5, 0.0f}); // (a line across it: as soon as it yields, the line resolved)
        }
        t.util = util;
        if (!tri_k_ok_[ti]) tri_build_k((uint32_t)ti);
        // the corners' forces and moments: -K d in the element's axes; in the same pass K v at the corners'
        // velocities in those axes (the material damping's part of the step's right side: tri_kv_)
        const float* Km = &tri_k_[ti * kTriK];
        const float* Kp = Km + 81;
        double vl[18];
        {
            const double E[3][3] = {{k.e1.x, k.e2.x, k.e3.x}, {k.e1.y, k.e2.y, k.e3.y}, {k.e1.z, k.e2.z, k.e3.z}}; // (E[p][a]: axis a)
            for (int i = 0; i < 3; i++) {
                const vec3 v = b.nodes[node[t.n[i]]].v, om = w[t.n[i]];
                for (int a2 = 0; a2 < 3; a2++) {
                    vl[6 * i + a2] = E[0][a2] * v.x + E[1][a2] * v.y + E[2][a2] * v.z;
                    vl[6 * i + 3 + a2] = E[0][a2] * om.x + E[1][a2] * om.y + E[2][a2] * om.z;
                }
            }
        }
        // (each part on its own dofs: the sums of the whole of K's rows, its nothing left out - in the same order)
        double f[18], fl[18];
        {
            double dm[9], dp[9], vm[9], vp[9];
            for (int a = 0; a < 9; a++) {
                const int ci = 6 * (a / 3);
                dm[a] = k.d[ci + kMemDof[a % 3]], dp[a] = k.d[ci + kPlateDof[a % 3]];
                vm[a] = vl[ci + kMemDof[a % 3]], vp[a] = vl[ci + kPlateDof[a % 3]];
            }
            double fm[9] = {}, fp[9] = {}, um[9] = {}, up[9] = {};
            for (int c = 0; c < 9; c++)
                for (int a = 0; a < 9; a++) { // (symmetric: row a's c-th the c-th row's a-th)
                    const double km = (double)Km[c * 9 + a], kp = (double)Kp[c * 9 + a];
                    fm[a] += km * dm[c], um[a] += km * vm[c];
                    fp[a] += kp * dp[c], up[a] += kp * vp[c];
                }
            for (int a = 0; a < 9; a++) {
                const int ci = 6 * (a / 3);
                f[ci + kMemDof[a % 3]] = fm[a], fl[ci + kMemDof[a % 3]] = um[a];
                f[ci + kPlateDof[a % 3]] = fp[a], fl[ci + kPlateDof[a % 3]] = up[a];
            }
        }
        o.on = true;
        for (int i = 0; i < 3; i++) {
            o.f[i] = -(k.e1 * (float)f[6 * i] + k.e2 * (float)f[6 * i + 1] + k.e3 * (float)f[6 * i + 2]);
            o.t[i] = -(k.e1 * (float)f[6 * i + 3] + k.e2 * (float)f[6 * i + 4] + k.e3 * (float)f[6 * i + 5]);
        }
        // the tangent: the axes, and the geometric stiffness of the membrane's tension (its compression left out, as the
        // members' is) between the corners, A grad N_i^T (t sigma)+ grad N_j
        tg.on = true;
        tg.e1 = k.e1, tg.e2 = k.e2, tg.e3 = k.e3;
        {
            const double sx = sig[0] * s.t, sy = sig[1] * s.t, sxy = sig[2] * s.t;
            const double tr = 0.5 * (sx + sy), df = std::sqrt(0.25 * (sx - sy) * (sx - sy) + sxy * sxy);
            const double l1 = std::max(0.0, tr + df), l2 = std::max(0.0, tr - df);
            // (the positive part: its eigenvectors)
            double vx = sxy, vy = l1 - sx;
            if (std::fabs(vx) + std::fabs(vy) < 1e-12 * (std::fabs(sx) + std::fabs(sy) + 1)) vx = sx >= sy ? 1 : 0, vy = sx >= sy ? 0 : 1;
            const double vl = std::sqrt(vx * vx + vy * vy);
            vx /= vl, vy /= vl;
            const double S[2][2] = {{l1 * vx * vx + l2 * vy * vy, (l1 - l2) * vx * vy}, {(l1 - l2) * vx * vy, l1 * vy * vy + l2 * vx * vx}};
            const double A = 0.5 * A2;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                    const double gi[2] = {b3[i] / A2, c3[i] / A2}, gj[2] = {b3[j] / A2, c3[j] / A2};
                    tg.g[i][j] = (float)(A * (gi[0] * (S[0][0] * gj[0] + S[0][1] * gj[1]) + gi[1] * (S[1][0] * gj[0] + S[1][1] * gj[1])));
                }
        }
        // K v (above) turned back into the world's axes
        {
            const double E[3][3] = {{tg.e1.x, tg.e2.x, tg.e3.x}, {tg.e1.y, tg.e2.y, tg.e3.y}, {tg.e1.z, tg.e2.z, tg.e3.z}}; // (E[p][a]: axis a)
            double* kv = &tri_kv_[(size_t)ti * 18];
            for (int i = 0; i < 3; i++)
                for (int p = 0; p < 3; p++) {
                    kv[6 * i + p] = E[p][0] * fl[6 * i] + E[p][1] * fl[6 * i + 1] + E[p][2] * fl[6 * i + 2];
                    kv[6 * i + 3 + p] = E[p][0] * fl[6 * i + 3] + E[p][1] * fl[6 * i + 4] + E[p][2] * fl[6 * i + 5];
                }
        }
    }
}

// a member's matrix at its tangent now (the plastic one where it yields: effective) for the assembly, and K v
void FemFrame::member_world_k(const SoftBody& b, uint32_t ei) {
    const FrameElement& e = elems[ei];
    double K[12][12];
    element_matrix(effective(tan_[ei]), K);
    double* W = &mem_kw_[(size_t)ei * kMemW];
    for (int r = 0; r < 6; r++)
        for (int q = 0; q < 6; q++) W[r * 6 + q] = K[r][q], W[36 + r * 6 + q] = K[6 + r][6 + q], W[72 + r * 6 + q] = K[6 + r][q], W[108 + r * 6 + q] = K[r][6 + q];
    const Node& na = b.nodes[node[e.a]];
    const Node& nb = b.nodes[node[e.b]];
    const double u[12] = {na.v.x, na.v.y, na.v.z, w[e.a].x, w[e.a].y, w[e.a].z, nb.v.x, nb.v.y, nb.v.z, w[e.b].x, w[e.b].y, w[e.b].z};
    double* kv = &mem_kv_[(size_t)ei * 12];
    for (int r = 0; r < 12; r++) {
        double s2 = 0;
        for (int q = 0; q < 12; q++) s2 += K[r][q] * u[q];
        kv[r] = s2;
    }
}


void FemFrame::compute_forces(SoftBody& b) {
    const int chunks = begin_forces(b);
    for (int c = 0; c < chunks; c++) eval_forces(b, c);
    end_forces(b);
}

int FemFrame::begin_forces(SoftBody& b) {
    if (!ready_) finalize(b);
    steps_++;
    static const int etan = getenv("BL_REPASS") ? atoi(getenv("BL_REPASS")) : -1; // (diagnostics: 0 never again, 1 always as before)
    repass_credit_ = std::min(kRepassBurst, repass_credit_ + kRepassRefill);
    elastic_step_ = etan == 0 || (etan != 1 && repass_credit_ < 1.0f);
    if (tan_.size() != elems.size()) tan_.assign(elems.size(), Tangent());
    // a node that is not where its double position rounds to was moved by something else (a reset, a repair)
    xd.resize(node.size() * 3);
    for (size_t i = 0; i < node.size(); i++) {
        const vec3 p = b.nodes[node[i]].p;
        double* x = &xd[i * 3];
        if ((float)x[0] != p.x || (float)x[1] != p.y || (float)x[2] != p.z) x[0] = p.x, x[1] = p.y, x[2] = p.z;
    }
    member_f.assign(node.size(), vec3(0));
    member_t_.assign(node.size(), vec3(0));
    // (the frame nodes some member is welded to: a released end's damping turns its node against the member only where
    // the node's turning is someone's - a node held by released ends alone has the inertia of a few grams)
    bool any_damp = false;
    for (const FrameSection& s : sections) any_damp |= s.joint_damp > 0;
    if (any_damp) {
        welded_.assign(node.size(), 0);
        for (const FrameElement& e : elems)
            if (!e.broken) welded_[e.a] |= e.end_a == FJ_RIGID, welded_[e.b] |= e.end_b == FJ_RIGID;
    }
    out_.resize(elems.size());
    tri_out_.resize(tris.size());
    if (tri_tan_.size() != tris.size()) tri_tan_.assign(tris.size(), TriTan());
    if (tri_k_ok_.size() != tris.size()) tri_k_.assign(tris.size() * kTriK, 0.0f), tri_b_.assign(tris.size() * kTriB, 0.0f), tri_k_ok_.assign(tris.size(), 0);
    if (tri_kv_.size() != tris.size() * 18) tri_kv_.assign(tris.size() * 18, 0.0);
    if (mem_kw_.size() != elems.size() * kMemW) mem_kw_.assign(elems.size() * kMemW, 0.0), mem_kv_.assign(elems.size() * 12, 0.0);
    const int chunks = (int)((elems.size() + kElemChunk - 1) / kElemChunk) + (int)((tris.size() + kTriChunk - 1) / kTriChunk);
    chunk_events_.resize(std::max<size_t>(chunk_events_.size(), (size_t)chunks));
    for (int c = 0; c < chunks; c++) chunk_events_[c].clear();
    return chunks;
}

void FemFrame::eval_forces(SoftBody& b, int chunk) {
    static const bool dbg = getenv("BL_FRAMEDBG") != nullptr;
    std::vector<Event>& evs = chunk_events_[chunk];
    const int mchunks = (int)((elems.size() + kElemChunk - 1) / kElemChunk);
    if (chunk >= mchunks) { // (the triangles' chunks after the members')
        eval_tris(b, chunk - mchunks, evs);
        return;
    }
    const size_t e_end = std::min(elems.size(), (size_t)(chunk + 1) * kElemChunk);
    for (size_t ei = (size_t)chunk * kElemChunk; ei < e_end; ei++) {
        FrameElement& e = elems[ei];
        Tangent& t = tan_[ei];
        ElemOut& o = out_[ei];
        t.on = false, o.on = false;
        if (e.broken) continue;
        Kin k;
        if (!kinematics(*this, e, k)) continue;
        const FrameSection& s = sections[e.section];
        const Stiff st = stiffness(s, e);
        const bool plastic = s.Np > 0 && b.allow_deform;
        // axial: tension yields at Np, compression at the lower of Np and the Euler load (ends between pinned and
        // clamped: an effective length of 0.7 L)
        const float delta = (float)k.dL;
        float N = st.ka * (delta - e.up), ka_t = st.ka;
        float Nt = s.Np, Nc = s.Np;
        bool axial_yield = false;
        float flow_ax = 0, flow_t = 0;
        if (plastic) {
            const float Le = 0.7f * e.L0;
            Nc = std::min(s.Np, kPi * kPi * s.E * std::min(s.Iy, s.Iz) / (Le * Le));
            if (N > Nt || N < -Nc) {
                const float lim = N > 0 ? Nt : -Nc;
                const float dup = (N - lim) / st.ka;
                e.up += dup;
                flow_ax = std::fabs(dup) / e.L0;
                // (the damage: the plastic stretch reached; crushed, it folds and does not tear)
                e.damage = std::max(e.damage, e.up / e.L0);
                N = lim;
                ka_t = st.ka * 0.01f;
                axial_yield = true;
            }
        }
        // torsion
        const float tw = k.rb.x - k.ra.x;
        float T = st.kt * (tw - e.tp), kt_t = st.kt;
        if (plastic && s.Tp > 0 && st.kt > 0 && std::fabs(T) > s.Tp) {
            const float lim = T > 0 ? s.Tp : -s.Tp;
            const float dtp = (T - lim) / st.kt;
            e.tp += dtp;
            flow_t = std::fabs(dtp);
            e.damage = std::max(e.damage, std::fabs(e.tp + e.bt) * s.half / e.L0); // (the shear strain at the surface reached)
            T = lim;
            kt_t = st.kt * 0.01f;
        }
        // bending: the moments at the ends (y and z components), the hinges' plastic rotations taken off
        auto moments = [&](vec2& Ma, vec2& Mb) {
            const vec2 da(k.ra.y - e.pa.x, k.ra.z - e.pa.y), db(k.rb.y - e.pb.x, k.rb.z - e.pb.y);
            Ma = vec2(st.cy[0] * da.x + st.cy[1] * db.x, st.cz[0] * da.y + st.cz[1] * db.y);
            Mb = vec2(st.cy[1] * da.x + st.cy[2] * db.x, st.cz[1] * da.y + st.cz[2] * db.y);
        };
        vec2 Ma, Mb;
        moments(Ma, Mb);
        bool hinge_a = false, hinge_b = false;
        float flow_a = 0, flow_b = 0;
        if (plastic && s.Mp > 0) {
            // radial return at each end in turn (twice: a hinge at one end changes the moment at the other); a joint's
            // freed rotation takes no moment and gets no plastic part
            // (the yield moment hardens with the hinge's rotation: dmg is that rotation, accumulated)
            auto yield_end = [&](vec2 M, float cyy, float czz, vec2& p, const vec2& base, float& dmg, float& flow) {
                const float Mp = s.Mp * (1.0f + s.hardening * std::min(1.0f, dmg));
                const float l = length(M);
                if (l <= Mp) return false;
                const float r = 1.0f - Mp / l;
                const vec2 dp(cyy > 0 ? M.x * r / cyy : 0.0f, czz > 0 ? M.y * r / czz : 0.0f);
                p += dp;
                flow += length(dp);
                // (the damage: the largest plastic rotation the hinge reached, not the path: yielding back and forth in the
                // shaking after an impact does not wear it through)
                dmg = std::max(dmg, length(p + base));
                return true;
            };
            for (int it = 0; it < 2; it++) {
                if (yield_end(Ma, st.cy[0], st.cz[0], e.pa, e.ba, e.dmg_a, flow_a)) hinge_a = true, moments(Ma, Mb);
                if (yield_end(Mb, st.cy[2], st.cz[2], e.pb, e.bb, e.dmg_b, flow_b)) hinge_b = true, moments(Ma, Mb);
            }
        }
        float util = 0;
        if (s.Np > 0) util = std::max(util, N > 0 ? N / Nt : -N / Nc);
        if (s.Mp > 0) util = std::max(util, std::max(length(Ma) / (1.0f + s.hardening * std::min(1.0f, e.dmg_a)), length(Mb) / (1.0f + s.hardening * std::min(1.0f, e.dmg_b))) / s.Mp);
        if (s.Tp > 0) util = std::max(util, std::fabs(T) / s.Tp);
        e.util = util;
        e.N = N;
        // failure: torn off the joint where a hinge ran out of ductility, split and torn in the middle when stretched,
        // crushed or twisted beyond it; a member forming a hinge is split to bend along a curve there (queued: the
        // topology changes after the integration)
        if (plastic && b.allow_break) {
            int kind = -1;
            if (e.dmg_a > s.hinge_capacity && !(e.torn & 1)) kind = 1;
            else if (e.dmg_b > s.hinge_capacity && !(e.torn & 2)) kind = 2;
            else if (e.damage > s.max_strain && !(e.torn & 3)) kind = e.dmg_b > e.dmg_a ? 2 : 1; // (pulled apart: off the weaker joint, once)
            else if ((hinge_a || hinge_b || axial_yield) && e.level < s.max_level && e.L0 > 2.0f * s.min_len()) kind = 0;
            if (kind >= 0) {
                if (dbg && kind == 0)
                    printf("frame: member %zu (%u-%u) splits: level %d, L0 %.3f, hinges %d %d (%.4f %.4f), axial %d, N %.0f\n", ei, node[e.a], node[e.b], e.level, e.L0,
                           (int)hinge_a, (int)hinge_b, e.dmg_a, e.dmg_b, (int)axial_yield, N);
                if (dbg && kind > 0)
                    printf("frame: member %zu (%u-%u) %s: damage %.3f, hinges %.3f %.3f, N %.0f, T %.1f\n", ei, node[e.a], node[e.b],
                           "tears off its joint", e.damage, e.dmg_a, e.dmg_b, N, T);
                bool dup = false;
                for (const Event& ev : evs) dup |= ev.elem == ei;
                if (!dup) evs.push_back({(uint32_t)ei, (uint8_t)kind, 0.5f});
            }
        }
        // nodal forces (-B^T g) and moments: the shear of the end moments acts across the member at its current length,
        // so forces and moments balance exactly
        const float Sy = Ma.x + Mb.x, Sz = Ma.y + Mb.y;
        const vec3 fa = k.e1 * N + k.e3 * (Sy / k.L) - k.e2 * (Sz / k.L);
        o.on = true;
        o.fa = fa;
        o.ta = k.e1 * T - k.e2 * Ma.x - k.e3 * Ma.y;
        o.tb = k.e1 * (-T) - k.e2 * Mb.x - k.e3 * Mb.y;
        // a mount that lets go: past the section's break force at its ends (axial and shear) for a while it tears off
        // its end a (the overload over time: a bolt yields before it breaks. On the peak alone the stiff frame's ringing
        // as a car dropped 5 m landed on its wheels tore all four doors off, the hood and the fenders)
        if (s.break_force > 0 && b.allow_break && !(e.torn & 1)) {
            const float fl = length(fa);
            e.overload = std::max(0.0f, e.overload + (fl / s.break_force - 1.0f) * (last_h_ > 0 ? last_h_ : 5e-4f));
            if (e.overload > kOverloadTime) {
                bool dup = false;
                for (const Event& ev : evs) dup |= ev.elem == ei;
                if (!dup) evs.push_back({(uint32_t)ei, (uint8_t)1, 0.5f});
                if (dbg) printf("frame: member %zu (%u-%u) lets go at %.0f N (breaks at %.0f)\n", ei, node[e.a], node[e.b], fl, s.break_force);
            }
        }
        // a released end's damping: its node's turning against the member's (the welded end's node), both ways
        if (s.joint_damp > 0 && (e.end_a != FJ_RIGID) != (e.end_b != FJ_RIGID)) {
            const uint32_t fr = e.end_a != FJ_RIGID ? e.a : e.b, fw = fr == e.a ? e.b : e.a;
            if (welded_[fr] && welded_[fw]) {
                const vec3 tq = (w[fr] - w[fw]) * -s.joint_damp;
                if (fr == e.a) o.ta += tq, o.tb -= tq;
                else o.tb += tq, o.ta -= tq;
            }
        }
        // the plastic rotations into the rest frames (E_a = E exp(r_a): turned by -p there, it measures r_a - p; the
        // co-rotated frame, their mean on the chord, moves only by a second-order twist); the twist half into each end
        const float kBake = 0.02f;
        if (dot(e.pa, e.pa) > kBake * kBake) e.qa = normalize(e.qa * quat_exp(vec3(0, -e.pa.x, -e.pa.y))), e.ba += e.pa, e.pa = vec2(0, 0);
        if (dot(e.pb, e.pb) > kBake * kBake) e.qb = normalize(e.qb * quat_exp(vec3(0, -e.pb.x, -e.pb.y))), e.bb += e.pb, e.pb = vec2(0, 0);
        if (std::fabs(e.tp) > kBake) {
            e.qa = normalize(e.qa * quat_exp(vec3(0.5f * e.tp, 0, 0)));
            e.qb = normalize(e.qb * quat_exp(vec3(-0.5f * e.tp, 0, 0)));
            e.bt += e.tp, e.tp = 0;
        }
        // the tangent for the implicit step: elastic, and where the member yields, the plastic one unless the step
        // unloads it (solve checks: a plastic tangent that stays on while unloading is integrated as if explicitly,
        // and on a light node of short members at many times its stable step)
        (void)ka_t, (void)kt_t, (void)hinge_a, (void)hinge_b;
        t.on = true;
        t.e1 = k.e1, t.e2 = k.e2, t.e3 = k.e3;
        t.L = k.L;
        t.ka = st.ka;
        t.kt = st.kt;
        for (int i = 0; i < 3; i++) t.cy[i] = st.cy[i], t.cz[i] = st.cz[i];
        t.ngeo = N > 0 ? N / k.L : 0.0f;
        // (the plastic tangent where it flows this step, not where it creeps along the yield surface: a damaged frame at
        // rest under its weight, the shaking after an impact; there the elastic one)
        const float kFlow = 1e-4f;
        t.yield = (uint8_t)((flow_a > kFlow ? 1 : 0) | (flow_b > kFlow ? 2 : 0) | (flow_ax > 0.1f * kFlow ? 4 : 0) | (flow_t > kFlow ? 8 : 0));
        t.unload = 0;
        if (t.elastic_hold > 0) t.elastic_hold--, t.unload = t.yield;
        if (elastic_step_) t.unload = t.yield; // (the passes' budget spent: see kRepassBurst)
        t.ma = Ma, t.mb = Mb, t.N = N, t.T = T;
        member_world_k(b, (uint32_t)ei);
    }
}

void FemFrame::end_forces(SoftBody& b) {
    if (!gdiag_ptr_.empty() && gdiag_ptr_.size() == node.size() + 1) gather_forces(b, 0, node.size());
    else end_forces_scatter(b);
    end_forces_rest(b);
}

// The members' and triangles' forces onto frame nodes [f0, f1): each node's in their order, as the members' loop and
// then the triangles' added them (the assembly's lists: Gather) - a team's chunks
void FemFrame::gather_forces(SoftBody& b, size_t f0, size_t f1) {
    if (gdiag_ptr_.size() != node.size() + 1 || iperm_.size() != node.size()) { // (no lists yet: the first chunk, in order)
        if (f0 == 0) end_forces_scatter(b);
        return;
    }
    vec3* F = b.force.data();
    for (size_t fn = f0; fn < f1 && fn < node.size(); fn++) {
        const int k = iperm_[fn];
        vec3& Fn = F[node[fn]];
        for (int q = gdiag_ptr_[k]; q < gdiag_ptr_[k + 1]; q++) {
            const Gather& g = gdiag_[q];
            if (g.kind == 0) {
                if (g.id >= out_.size()) continue;
                const ElemOut& o = out_[g.id];
                if (!o.on) continue;
                if (g.corner == 0) Fn += o.fa, member_f[fn] += o.fa, torque[fn] += o.ta, member_t_[fn] += o.ta;
                else Fn -= o.fa, member_f[fn] -= o.fa, torque[fn] += o.tb, member_t_[fn] += o.tb;
            } else {
                if (g.id >= tri_out_.size()) continue;
                const TriOut& o = tri_out_[g.id];
                if (!o.on) continue;
                Fn += o.f[g.corner], member_f[fn] += o.f[g.corner], torque[fn] += o.t[g.corner], member_t_[fn] += o.t[g.corner];
            }
        }
    }
}

// (the same in the elements' order, the lists not made yet)
void FemFrame::end_forces_scatter(SoftBody& b) {
    vec3* F = b.force.data();
    for (size_t ei = 0; ei < elems.size() && ei < out_.size(); ei++) {
        const ElemOut& o = out_[ei];
        if (!o.on) continue;
        const FrameElement& e = elems[ei];
        F[node[e.a]] += o.fa;
        F[node[e.b]] -= o.fa;
        member_f[e.a] += o.fa;
        member_f[e.b] -= o.fa;
        torque[e.a] += o.ta;
        torque[e.b] += o.tb;
        member_t_[e.a] += o.ta;
        member_t_[e.b] += o.tb;
    }
    for (size_t ti = 0; ti < tris.size() && ti < tri_out_.size(); ti++) {
        const TriOut& o = tri_out_[ti];
        if (!o.on) continue;
        const FrameTri& t = tris[ti];
        for (int i = 0; i < 3; i++) {
            F[node[t.n[i]]] += o.f[i];
            member_f[t.n[i]] += o.f[i];
            torque[t.n[i]] += o.t[i];
            member_t_[t.n[i]] += o.t[i];
        }
    }
}

void FemFrame::end_forces_rest(SoftBody& b) {
    if (!mounts.empty()) mount_forces(b);
    const size_t chunks = (elems.size() + kElemChunk - 1) / kElemChunk + (tris.size() + kTriChunk - 1) / kTriChunk;
    for (size_t c = 0; c < chunks && c < chunk_events_.size(); c++)
        for (const Event& x : chunk_events_[c]) {
            bool dup = false;
            for (const Event& ev : events_) dup |= ev.elem == x.elem && (ev.kind == 4) == (x.kind == 4);
            if (!dup) events_.push_back(x);
        }
}

FemFrame::Tangent FemFrame::effective(const Tangent& t) const {
    const uint8_t y = (uint8_t)(t.yield & ~t.unload);
    if (!y) return t;
    Tangent e = t;
    if (y & 3) condense(e.cy, y & 1, y & 2), condense(e.cz, y & 1, y & 2); // (a flowing hinge turns like a pin)
    if (y & 4) e.ka *= 0.01f;
    if (y & 8) e.kt *= 0.01f;
    return e;
}

void FemFrame::element_matrix(const Tangent& t, double K[12][12]) const {
    // K = B^T D B written out in 3 x 3 blocks of the dofs [ua, pa, ub, pb] (B: the elongation, twist and the end
    // rotations about y and z against the chord; D: ka, kt and the two bending pairs): sums of outer products of the
    // member's axes, g = e3 / L and h = e2 / L across it
    const double iL = 1.0 / t.L;
    const double e1[3] = {t.e1.x, t.e1.y, t.e1.z}, e2[3] = {t.e2.x, t.e2.y, t.e2.z}, e3[3] = {t.e3.x, t.e3.y, t.e3.z};
    const double g[3] = {e3[0] * iL, e3[1] * iL, e3[2] * iL}, h[3] = {e2[0] * iL, e2[1] * iL, e2[2] * iL};
    const double cy0 = t.cy[0], cy1 = t.cy[1], cy2 = t.cy[2], cz0 = t.cz[0], cz1 = t.cz[1], cz2 = t.cz[2];
    const double Sy = cy0 + 2 * cy1 + cy2, Sz = cz0 + 2 * cz1 + cz2, ay = cy0 + cy1, by = cy1 + cy2, az = cz0 + cz1, bz = cz1 + cz2;
    const double ka = t.ka, kt = t.kt;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            const double uu = ka * e1[i] * e1[j] + Sy * g[i] * g[j] + Sz * h[i] * h[j];
            const double up = -ay * g[i] * e2[j] + az * h[i] * e3[j];                           // [ua, pa]
            const double ub = -by * g[i] * e2[j] + bz * h[i] * e3[j];                           // [ua, pb]
            const double tt = kt * e1[i] * e1[j];
            const double pa = tt + cy0 * e2[i] * e2[j] + cz0 * e3[i] * e3[j];                 // [pa, pa]
            const double pb = -tt + cy1 * e2[i] * e2[j] + cz1 * e3[i] * e3[j];                // [pa, pb]
            const double qb = tt + cy2 * e2[i] * e2[j] + cz2 * e3[i] * e3[j];                 // [pb, pb]
            K[i][j] = uu, K[i][3 + j] = up, K[i][6 + j] = -uu, K[i][9 + j] = ub;
            K[3 + i][3 + j] = pa, K[3 + i][9 + j] = pb;
            K[6 + i][6 + j] = uu, K[6 + i][9 + j] = -ub;
            K[9 + i][9 + j] = qb;
        }
    // [pa, ub] = -[ua, pa]^T and the lower triangle by symmetry
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) K[3 + i][6 + j] = -K[j][3 + i];
    for (int r = 0; r < 12; r++)
        for (int c = r + 1; c < 12; c++) K[c][r] = K[r][c];
}

// ------------------------------------------------------------------------------------------------ the implicit step
namespace {

// 6 x 6 blocks, row-major. Cholesky of a symmetric positive definite block (lower triangle, in place; the inverse
// diagonal in dinv).
bool chol6(double* A, double* dinv) {
    for (int j = 0; j < 6; j++) {
        double s = A[j * 6 + j];
        for (int k = 0; k < j; k++) s -= A[j * 6 + k] * A[j * 6 + k];
        if (!(s > 0)) return false;
        const double d = std::sqrt(s), id = 1.0 / d;
        A[j * 6 + j] = d;
        dinv[j] = id;
        for (int i = j + 1; i < 6; i++) {
            double t = A[i * 6 + j];
            for (int k = 0; k < j; k++) t -= A[i * 6 + k] * A[j * 6 + k];
            A[i * 6 + j] = t * id;
        }
        for (int i = 0; i < j; i++) A[i * 6 + j] = 0;
    }
    return true;
}
// X := X L^-T (each row x solves L x^T = row)
void right_solve_lt(const double* L, const double* dinv, double* X) {
    for (int r = 0; r < 6; r++) {
        double* x = X + r * 6;
        for (int j = 0; j < 6; j++) {
            double s = x[j];
            for (int m = 0; m < j; m++) s -= L[j * 6 + m] * x[m];
            x[j] = s * dinv[j];
        }
    }
}
// T -= P Q^T (Q transposed first: the inner loop runs along T's rows, which vectorizes; kept transposed for the whole
// factorization instead, the blocks crowd the cache)
#if defined(__aarch64__)
// (NEON: Q's columns in registers, two of T's columns a lane pair; the same sums in the same order as below)
void sub_abt(double* __restrict T, const double* __restrict P, const double* __restrict Q) {
    float64x2_t q[6][3]; // q[m][j] = (Q[2j][m], Q[2j + 1][m])
    for (int j = 0; j < 3; j++) {
        const double* a = Q + 12 * j;
        const double* b = a + 6;
        for (int m = 0; m < 6; m += 2) {
            const float64x2_t va = vld1q_f64(a + m), vb = vld1q_f64(b + m);
            q[m][j] = vtrn1q_f64(va, vb);
            q[m + 1][j] = vtrn2q_f64(va, vb);
        }
    }
    for (int r = 0; r < 6; r++) {
        const float64x2_t p0 = vld1q_f64(P + r * 6), p1 = vld1q_f64(P + r * 6 + 2), p2 = vld1q_f64(P + r * 6 + 4);
        float64x2_t a0 = vdupq_n_f64(0.0), a1 = a0, a2 = a0;
#define BL_SUB_M(m, pv, lane)                          \
    a0 = vfmaq_laneq_f64(a0, q[m][0], pv, lane);       \
    a1 = vfmaq_laneq_f64(a1, q[m][1], pv, lane);       \
    a2 = vfmaq_laneq_f64(a2, q[m][2], pv, lane);
        BL_SUB_M(0, p0, 0) BL_SUB_M(1, p0, 1) BL_SUB_M(2, p1, 0) BL_SUB_M(3, p1, 1) BL_SUB_M(4, p2, 0) BL_SUB_M(5, p2, 1)
#undef BL_SUB_M
        double* t = T + r * 6;
        vst1q_f64(t, vsubq_f64(vld1q_f64(t), a0));
        vst1q_f64(t + 2, vsubq_f64(vld1q_f64(t + 2), a1));
        vst1q_f64(t + 4, vsubq_f64(vld1q_f64(t + 4), a2));
    }
}
// r -= L y (each row's sum in order: two rows a lane pair)
void sub_mv(const double* __restrict L, const double* __restrict y, double* __restrict r) {
    for (int i = 0; i < 6; i += 2) {
        float64x2_t acc = vdupq_n_f64(0.0);
        for (int j = 0; j < 6; j += 2) {
            const float64x2_t va = vld1q_f64(L + i * 6 + j), vb = vld1q_f64(L + i * 6 + 6 + j);
            acc = vfmaq_n_f64(acc, vtrn1q_f64(va, vb), y[j]);
            acc = vfmaq_n_f64(acc, vtrn2q_f64(va, vb), y[j + 1]);
        }
        vst1q_f64(r + i, vsubq_f64(vld1q_f64(r + i), acc));
    }
}
// x -= L^T xr (each column's sum in order: two columns a lane pair)
void sub_mtv(const double* __restrict L, const double* __restrict xr, double* __restrict x) {
    float64x2_t a0 = vdupq_n_f64(0.0), a1 = a0, a2 = a0;
    for (int i = 0; i < 6; i++) {
        a0 = vfmaq_n_f64(a0, vld1q_f64(L + i * 6), xr[i]);
        a1 = vfmaq_n_f64(a1, vld1q_f64(L + i * 6 + 2), xr[i]);
        a2 = vfmaq_n_f64(a2, vld1q_f64(L + i * 6 + 4), xr[i]);
    }
    vst1q_f64(x, vsubq_f64(vld1q_f64(x), a0));
    vst1q_f64(x + 2, vsubq_f64(vld1q_f64(x + 2), a1));
    vst1q_f64(x + 4, vsubq_f64(vld1q_f64(x + 4), a2));
}
#else
void sub_abt(double* __restrict T, const double* __restrict P, const double* __restrict Q) {
    double Qt[36];
    for (int c = 0; c < 6; c++)
        for (int m = 0; m < 6; m++) Qt[m * 6 + c] = Q[c * 6 + m];
    for (int r = 0; r < 6; r++) {
        double acc[6] = {0, 0, 0, 0, 0, 0};
        for (int m = 0; m < 6; m++) {
            const double p = P[r * 6 + m];
            for (int c = 0; c < 6; c++) acc[c] += p * Qt[m * 6 + c];
        }
        for (int c = 0; c < 6; c++) T[r * 6 + c] -= acc[c];
    }
}
void sub_mv(const double* __restrict L, const double* __restrict y, double* __restrict r) {
    for (int i = 0; i < 6; i++) {
        double s = 0;
        for (int j = 0; j < 6; j++) s += L[i * 6 + j] * y[j];
        r[i] -= s;
    }
}
void sub_mtv(const double* __restrict L, const double* __restrict xr, double* __restrict x) {
    for (int j = 0; j < 6; j++) {
        double s = 0;
        for (int i = 0; i < 6; i++) s += L[i * 6 + j] * xr[i];
        x[j] -= s;
    }
}
#endif
void forward6(const double* L, const double* dinv, double* y) { // L y = b in place
    for (int j = 0; j < 6; j++) {
        double s = y[j];
        for (int m = 0; m < j; m++) s -= L[j * 6 + m] * y[m];
        y[j] = s * dinv[j];
    }
}
void backward6(const double* L, const double* dinv, double* x) { // L^T x = b in place
    for (int j = 5; j >= 0; j--) {
        double s = x[j];
        for (int m = j + 1; m < 6; m++) s -= L[m * 6 + j] * x[m];
        x[j] = s * dinv[j];
    }
}

} // namespace

void FemFrame::hold(SoftBody& b, float step) {
    if (!ready_) return;
    impulse.resize(node.size(), vec3(0));
    vec3* F = b.force.data();
    for (size_t i = 0; i < node.size(); i++) {
        Node& x = b.nodes[node[i]];
        if (x.inv_mass <= 0) continue;
        // the ground's contact acts in its own short step (held over to the next implicit step with the rest, a ring's
        // node on the ground sank for three short steps and was then thrown back out: a drum with frame rings trembled
        // and walked); the rest waits for the frame's step
        const vec3 fc = i < contact_f.size() ? contact_f[i] : vec3(0);
        impulse[i] += (F[node[i]] - fc) * step;
        x.v += fc * (step * x.inv_mass);
        F[node[i]] = vec3(0);
    }
    for (vec3& t : torque) t = vec3(0);
    for (vec3& c : contact_n) c = vec3(0);
    for (vec3& c : contact_f) c = vec3(0);
    for (TriPress& t : tri_press) t.kap = 0;
}

int FemFrame::held_begin(SoftBody& b, float step) {
    if (!ready_ || node.empty() || comp_factored_.size() != comp_range_.size() || pred_.size() != node.size() || tpred_.size() != node.size() ||
        fixed_.size() != node.size() || rhs_.size() < node.size() * 6)
        return 0;
    bool any = false;
    for (char c : comp_factored_) any |= c != 0;
    if (!any) return 0;
    held_step_ = step;
    impulse.resize(node.size(), vec3(0));
    for (CompStats& cs : comp_stats_) cs.clamps = 0; // (the step's were counted by solve_end)
    std::fill(par_clamps_.begin(), par_clamps_.end(), 0);
    comp_held_par_.assign(comp_range_.size(), 0);
    return (int)comp_range_.size();
}

void FemFrame::held_rhs_col(const SoftBody& b, int kk) {
    const vec3* F = b.force.data();
    const float step = held_step_;
    const int i = perm_[kk];
    double* r = &rhs_[(size_t)kk * 6];
    if (fixed_[i] || b.nodes[node[i]].inv_mass <= 0) {
        for (int j = 0; j < 6; j++) r[j] = 0;
        return;
    }
    const vec3 d = (F[node[i]] - pred_[i]) * step, t = (torque[i] - tpred_[i]) * step;
    r[0] = d.x, r[1] = d.y, r[2] = d.z, r[3] = t.x, r[4] = t.y, r[5] = t.z;
    impulse[i] += pred_[i] * step; // (the prediction's debt for this step paid)
}

void FemFrame::held_vel_col(SoftBody& b, int kk, int& clamps) {
    vec3* F = b.force.data();
    const float step = held_step_;
    const int i = perm_[kk];
    if (fixed_[i]) return;
    const double* x = &rhs_[(size_t)kk * 6];
    const Node& nd = b.nodes[node[i]];
    vec3 dv((float)x[0], (float)x[1], (float)x[2]);
    const vec3 dw((float)x[3], (float)x[4], (float)x[5]);
    if (!std::isfinite(dv.x + dv.y + dv.z + dw.x + dw.y + dw.z)) {
        F[node[i]] = vec3(0);
        return;
    }
    const float kMaxDv = 40.0f, kMaxW = 3000.0f;
    static const bool dbg = getenv("BL_CLAMPDBG") != nullptr; // (diagnostics: which nodes are cut, and how)
    if (const float l = length(dv); l > kMaxDv) {
        if (dbg) printf("clamp dv node %u %.0f m/s\n", node[i], l);
        dv *= kMaxDv / l, clamps++;
    }
    F[node[i]] = dv * (nd.mass / step);
    w[i] += dw;
    if (const float l = length(w[i]); l > kMaxW) {
        if (dbg) printf("clamp w node %u %.0f rad/s\n", node[i], l);
        w[i] *= kMaxW / l, clamps++;
    }
}

// held_component in the team's stages: group g's right side, its one-sided springs' share, its forward substitution
void FemFrame::held_front(SoftBody& b, int comp, int g) {
    PROFILE_ACCUM("Frame held");
    const ParPlan& pl = plans_[comp_plan_[comp]];
    for (int x = pl.gptr[g]; x < pl.gptr[g + 1]; x++) held_rhs_col(b, pl.gcols[x]);
    if (comp < (int)comp_onesided_.size())
        for (const OneSided& o : comp_onesided_[comp])
            if (col_group_[iperm_[o.frame_node]] == g) one_sided_entry(b, o, held_step_);
    for (int x = pl.gptr[g]; x < pl.gptr[g + 1]; x++) {
        const int k = pl.gcols[x];
        double* y = &rhs_[(size_t)k * 6];
        forward6(&diag_[(size_t)k * 36], &dinv_[(size_t)k * 6], y);
        for (int p = col_ptr_[k]; p < row_split_[k]; p++) sub_mv(&off_[(size_t)p * 36], y, &rhs_[(size_t)row_[p] * 6]);
    }
}

// ... the columns above: their right side, both substitutions, their velocities
void FemFrame::held_top(SoftBody& b, int comp) {
    PROFILE_ACCUM("Frame held");
    static const bool femprof = getenv("BL_FEMPROF") != nullptr;
    const uint64_t t0 = femprof ? prof::now() : 0;
    const ParPlan& pl = plans_[comp_plan_[comp]];
    for (int k : pl.top) held_rhs_col(b, k);
    if (comp < (int)comp_onesided_.size())
        for (const OneSided& o : comp_onesided_[comp])
            if (col_group_[iperm_[o.frame_node]] < 0) one_sided_entry(b, o, held_step_);
    for (size_t t = 0; t < pl.top.size(); t++) {
        const int r = pl.top[t];
        double* y = &rhs_[(size_t)r * 6];
        for (int q = pl.tptr[t]; q < pl.tptr[t + 1]; q++) sub_mv(&off_[(size_t)pl.tblk[q] * 36], &rhs_[(size_t)pl.tcol[q] * 6], y);
        forward6(&diag_[(size_t)r * 36], &dinv_[(size_t)r * 6], y);
    }
    for (int t = (int)pl.top.size() - 1; t >= 0; t--) {
        const int k = pl.top[t];
        double* x = &rhs_[(size_t)k * 6];
        for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) sub_mtv(&off_[(size_t)p * 36], &rhs_[(size_t)row_[p] * 6], x);
        backward6(&diag_[(size_t)k * 36], &dinv_[(size_t)k * 6], x);
    }
    CompStats& st = comp_stats_[comp];
    for (int k : pl.top) held_vel_col(b, k, st.clamps);
    if (comp < (int)comp_held_par_.size()) comp_held_par_[comp] = 1; // (its one-sided springs' reaction in held_end)
    if (femprof) st.t[5] += prof::ticks_to_ms(prof::now() - t0), st.held++;
}

// ... group g's backward substitution, last column first, and its nodes' velocities
void FemFrame::held_back(SoftBody& b, int comp, int g) {
    PROFILE_ACCUM("Frame held");
    const int pi = comp_plan_[comp];
    const ParPlan& pl = plans_[pi];
    for (int x = pl.gptr[g + 1] - 1; x >= pl.gptr[g]; x--) {
        const int k = pl.gcols[x];
        double* xk = &rhs_[(size_t)k * 6];
        for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) sub_mtv(&off_[(size_t)p * 36], &rhs_[(size_t)row_[p] * 6], xk);
        backward6(&diag_[(size_t)k * 36], &dinv_[(size_t)k * 6], xk);
    }
    int& clamps = par_clamps_[(size_t)par_fail_ptr_[pi] + g];
    for (int x = pl.gptr[g]; x < pl.gptr[g + 1]; x++) held_vel_col(b, pl.gcols[x], clamps);
}

void FemFrame::held_component(SoftBody& b, int comp) {
    PROFILE_ACCUM("Frame held");
    static const bool femprof = getenv("BL_FEMPROF") != nullptr;
    struct Lap {
        double* t = nullptr;
        uint64_t t0 = 0;
        ~Lap() { if (t) *t += prof::ticks_to_ms(prof::now() - t0); }
    } lap;
    if (femprof) lap.t = &comp_stats_[comp].t[5], lap.t0 = prof::now(), comp_stats_[comp].held++;
    const int k0 = comp_range_[comp].first, k1 = comp_range_[comp].second;
    const float step = held_step_;
    vec3* F = b.force.data();
    if (!comp_factored_[comp]) { // (its step failed: the forces held for the next, as hold)
        for (int kk = k0; kk < k1; kk++) {
            const int i = perm_[kk];
            Node& x = b.nodes[node[i]];
            if (x.inv_mass <= 0) continue;
            const vec3 fc = i < (int)contact_f.size() ? contact_f[i] : vec3(0);
            impulse[i] += (F[node[i]] - fc) * step;
            x.v += fc * (step * x.inv_mass);
            F[node[i]] = vec3(0);
        }
        return;
    }
    // the right side: this step's force beyond the prediction (the step took it for its whole length), its torque
    for (int kk = k0; kk < k1; kk++) held_rhs_col(b, kk);
    one_sided_rhs(b, comp, step);
    // L L^T x = r with the last step's factor
    for (int k = k0; k < k1; k++) {
        double* y = &rhs_[(size_t)k * 6];
        forward6(&diag_[(size_t)k * 36], &dinv_[(size_t)k * 6], y);
        for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) sub_mv(&off_[(size_t)p * 36], y, &rhs_[(size_t)row_[p] * 6]);
    }
    for (int k = k1 - 1; k >= k0; k--) {
        double* x = &rhs_[(size_t)k * 6];
        for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) sub_mtv(&off_[(size_t)p * 36], &rhs_[(size_t)row_[p] * 6], x);
        backward6(&diag_[(size_t)k * 36], &dinv_[(size_t)k * 6], x);
    }
    one_sided_react(b, comp, step); // (the held step's share of the one-sided springs, as the step's)
    CompStats& st = comp_stats_[comp];
    for (int kk = k0; kk < k1; kk++) held_vel_col(b, kk, st.clamps);
}

void FemFrame::held_end(SoftBody& b) {
    prof_flush();
    for (int c = 0; c < (int)comp_held_par_.size(); c++)
        if (comp_held_par_[c]) one_sided_react(b, c, held_step_), comp_held_par_[c] = 0;
    for (int& n : par_clamps_) clamps += n, n = 0;
    apply_reacts(b, held_step_);
    for (const CompStats& cs : comp_stats_) clamps += cs.clamps;
    for (CompStats& cs : comp_stats_) cs.clamps = 0;
    for (vec3& t : torque) t = vec3(0);
    for (vec3& c : contact_n) c = vec3(0);
    for (vec3& c : contact_f) c = vec3(0);
    for (TriPress& t : tri_press) t.kap = 0;
}

void FemFrame::solve(SoftBody& b, float h, float step, float theta, float dissipation) {
    const int nc = solve_begin(b, h, step, theta, dissipation);
    for (int c = 0; c < nc; c++) solve_component(b, c);
    solve_end(b);
}

int FemFrame::solve_begin(SoftBody& b, float h, float step, float theta, float dissipation) {
    if (!ready_ || node.empty()) return 0;
    last_h_ = h;
    impulse.resize(node.size(), vec3(0));
    dissipation = std::clamp(dissipation, 0.0f, 1.0f);
    theta = std::clamp(theta, 0.25f + 0.5f * dissipation, 1.0f); // (unconditionally stable)
    fixed_.assign(node.size(), 0);
    sv_.h = h, sv_.step = step, sv_.theta = theta, sv_.dissipation = dissipation;
    // (a sub-cycled body's contacts with other bodies, held over its short steps: World::step_island)
    sv_.ext = h > step && b.ext_force.size() == b.nodes.size() ? b.ext_force.data() : nullptr;
    w0_ = w;
    pred_.resize(node.size(), vec3(0));
    tpred_.resize(node.size(), vec3(0));
    comp_factored_.assign(comp_range_.size(), 0);
    comp_pass_.assign(comp_range_.size(), 0);
    comp_repass_.assign(comp_range_.size(), 0);
    comp_back_.assign(comp_range_.size(), 0);
    comp_held_par_.assign(comp_range_.size(), 0);
    comp_blocks_.assign(comp_range_.size(), 0);
    rdamp_.resize(node.size() * 6);
    comp_stats_.assign(comp_range_.size(), CompStats());
    comp_onesided_.resize(comp_range_.size());
    comp_react_.resize(comp_range_.size());
    for (auto& r : comp_react_) r.clear();
    return (int)comp_range_.size();
}

// the one-sided springs (OneSided): their other ends' forces over the step (their shares), along the springs, into
// the frame nodes' right sides (h: the step's length)
void FemFrame::one_sided_rhs(const SoftBody& b, int comp, float h) {
    if (comp >= (int)comp_onesided_.size()) return;
    for (const OneSided& o : comp_onesided_[comp]) one_sided_entry(b, o, h);
}

void FemFrame::one_sided_entry(const SoftBody& b, const OneSided& o, float h) {
    if (fixed_[o.frame_node] || o.other >= b.force.size()) return;
    const double rb = (double)dot(o.e, b.force[o.other]) * h * o.share;
    double* R = &rhs_[(size_t)iperm_[o.frame_node] * 6];
    const double s = o.c / ((double)o.mb + o.c) * rb;
    R[0] += s * o.e.x, R[1] += s * o.e.y, R[2] += s * o.e.z;
}

// ... and after the solve, the other ends' change along them: c (mb dv - rb) / (mb + c), what their frame nodes gave up
// (dv: the solution in rhs_)
void FemFrame::one_sided_react(const SoftBody& b, int comp, float h) {
    if (comp >= (int)comp_onesided_.size() || comp >= (int)comp_react_.size()) return;
    auto& out = comp_react_[comp];
    out.clear();
    for (const OneSided& o : comp_onesided_[comp]) {
        if (fixed_[o.frame_node] || o.other >= b.force.size()) continue;
        const double* x = &rhs_[(size_t)iperm_[o.frame_node] * 6];
        const double dva = (double)o.e.x * x[0] + (double)o.e.y * x[1] + (double)o.e.z * x[2];
        const double rb = (double)dot(o.e, b.force[o.other]) * h * o.share;
        const double J = o.c * ((double)o.mb * dva - rb) / ((double)o.mb + o.c);
        if (!std::isfinite(J)) continue;
        out.push_back({o.other, o.e * (float)J});
    }
}

void FemFrame::apply_reacts(SoftBody& b, float step) {
    if (!(step > 0)) return;
    for (auto& list : comp_react_) {
        for (const auto& [n, J] : list)
            if (n < b.force.size() && b.nodes[n].inv_mass > 0) b.force[n] += J * (1.0f / step);
        list.clear();
    }
}

void FemFrame::solve_end(SoftBody& b) {
    apply_reacts(b, sv_.step);
    for (const CompStats& cs : comp_stats_) solve_failures += cs.failures, clamps += cs.clamps, passes_ += cs.passes;
    for (const CompStats& cs : comp_stats_) repasses += cs.repasses, repass_credit_ -= (float)cs.repasses;
    prof_flush();
    for (vec3& t : torque) t = vec3(0); // (torques from outside, e.g. the tests, add up until the next step)
    for (vec3& c : contact_n) c = vec3(0);
    for (vec3& c : contact_f) c = vec3(0);
    for (TriPress& t : tri_press) t.kap = 0;
}

void FemFrame::solve_component(SoftBody& b, int comp) {
    static const bool dbg = getenv("BL_SERIALDBG") != nullptr;
    if (dbg && comp_range_[comp].second - comp_range_[comp].first >= kParNodes)
        printf("serial: %p comp %d of %d nodes, plan %d (plans %zu)\n", (void*)this, comp, comp_range_[comp].second - comp_range_[comp].first, comp < (int)comp_plan_.size() ? comp_plan_[comp] : -9, plans_.size());
    solve_impl(b, comp, 0);
}
void FemFrame::solve_assemble(SoftBody& b, int comp) { solve_impl(b, comp, par_component(comp) ? 1 : 0); }
void FemFrame::solve_finish(SoftBody& b, int comp) { solve_impl(b, comp, 2); }
void FemFrame::solve_repass(SoftBody& b, int comp) { solve_impl(b, comp, 3); }
void FemFrame::solve_finish_end(SoftBody& b, int comp) { solve_impl(b, comp, 4); }

// A large component's subtrees (group g of its plan): their columns factored in order, each one's updates of the
// blocks of its own subtree (the ones above it: solve_defer)
void FemFrame::solve_factor(int comp, int g) {
    PROFILE_ACCUM("Frame factor");
    const int pi = comp_plan_[comp];
    const ParPlan& pl = plans_[pi];
    char& fail = par_fail_[(size_t)par_fail_ptr_[pi] + g];
    fail = 0;
    for (int x = pl.gptr[g]; x < pl.gptr[g + 1]; x++) {
        const int k = pl.gcols[x];
        double* Lkk = &diag_[(size_t)k * 36];
        double* dk = &dinv_[(size_t)k * 6];
        if (!chol6(Lkk, dk)) {
            fail = 1;
            return;
        }
        for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) right_solve_lt(Lkk, dk, &off_[(size_t)p * 36]);
        for (int u = upd_ptr_[k]; u < upd_split_[k]; u++) {
            const Update& up = upd_[u];
            double* T = up.target >= 0 ? &off_[(size_t)up.target * 36] : &diag_[(size_t)(-up.target - 1) * 36];
            sub_abt(T, &off_[(size_t)up.pj * 36], &off_[(size_t)up.pi * 36]);
        }
        // (the forward substitution's column: its rows in the subtree; those above take it in solve_finish)
        double* y = &rhs_[(size_t)k * 6];
        forward6(Lkk, dk, y);
        for (int p = col_ptr_[k]; p < row_split_[k]; p++) sub_mv(&off_[(size_t)p * 36], y, &rhs_[(size_t)row_[p] * 6]);
    }
}

// ... and after solve_finish's columns above, the subtrees' backward substitution: group g's columns, last first
void FemFrame::solve_back(int comp, int g) {
    PROFILE_ACCUM("Frame factor");
    const ParPlan& pl = plans_[comp_plan_[comp]];
    for (int x = pl.gptr[g + 1] - 1; x >= pl.gptr[g]; x--) {
        const int k = pl.gcols[x];
        double* xk = &rhs_[(size_t)k * 6];
        for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) sub_mtv(&off_[(size_t)p * 36], &rhs_[(size_t)row_[p] * 6], xk);
        backward6(&diag_[(size_t)k * 36], &dinv_[(size_t)k * 6], xk);
    }
}

// ... the subtrees' updates of the blocks above them: group g's columns' blocks, each one's in the order of the
// columns they come from
void FemFrame::solve_defer(int comp, int g) {
    PROFILE_ACCUM("Frame factor");
    const ParPlan& pl = plans_[comp_plan_[comp]];
    for (int x = pl.dptr[g]; x < pl.dptr[g + 1]; x++) {
        const Update& up = upd_[pl.dupd[x]];
        double* T = up.target >= 0 ? &off_[(size_t)up.target * 36] : &diag_[(size_t)(-up.target - 1) * 36];
        sub_abt(T, &off_[(size_t)up.pj * 36], &off_[(size_t)up.pi * 36]);
    }
}

// One component's implicit step: its nodes are the range [k0, k1) of the permuted order, its pattern closed in it.
// stage 0: the whole step; 1: the first assembly alone (the team factors the subtrees next: solve_factor, solve_defer);
// 2: the rest of it after those
void FemFrame::solve_impl(SoftBody& b, int comp, int stage) {
    const float h = sv_.h, step = sv_.step, theta = sv_.theta, dissipation = sv_.dissipation;
    std::vector<char>& fixed = fixed_;
    const int k0 = comp_range_[comp].first, k1 = comp_range_[comp].second;
    const std::vector<uint32_t>& celems = comp_elems_[comp];
    std::vector<Changed>& changed = comp_changed_[comp];
    CompStats& st = comp_stats_[comp];
    static const bool femprof = getenv("BL_FEMPROF") != nullptr;
    uint64_t tp = femprof ? prof::now() : 0;
    auto tlap = [&](int i) {
        if (!femprof) return;
        const uint64_t t = prof::now();
        st.t[i] += prof::ticks_to_ms(t - tp);
        tp = t;
    };
    const size_t d0 = (size_t)k0 * 36, d1 = (size_t)k1 * 36, o0 = (size_t)col_ptr_[k0] * 36, o1 = (size_t)col_ptr_[k1] * 36, r0 = (size_t)k0 * 6, r1 = (size_t)k1 * 6;
    bool any_yield = false;
    for (uint32_t ei : celems) any_yield |= tan_[ei].on && (tan_[ei].yield & ~tan_[ei].unload); // (one the step may unload)
    const double h2 = (double)theta * h * h, hd = (double)dissipation * h * h;
    // a member into the system (D, O, R: the diagonal and off-diagonal blocks and the right side): the matrix takes
    // (theta h^2 + beta h) K + theta h^2 Kg, the right side -(theta_d h^2 + beta h) K v (theta_d: numerical dissipation;
    // 0 keeps a linear frame's energy exactly, only the material damping beta takes it); sign -1 takes it out again
    // (geo: with the geometric stiffness)
    auto add_member = [&](size_t ei, const Tangent& t, double sign, bool geo, double* Dg, double* Of, double* R) {
        double K[12][12];
        const FrameElement& e = elems[ei];
        element_matrix(t, K);
        const double beta = (double)sections[e.section].damping * h;
        const double c = sign * (h2 + beta), cr = sign * (hd + beta);
        const bool fa = fixed[e.a], fb = fixed[e.b];
        const int ka = iperm_[e.a], kb = iperm_[e.b];
        if (cr != 0) {
            const Node& na = b.nodes[node[e.a]];
            const Node& nb = b.nodes[node[e.b]];
            const double u[12] = {na.v.x, na.v.y, na.v.z, w[e.a].x, w[e.a].y, w[e.a].z, nb.v.x, nb.v.y, nb.v.z, w[e.b].x, w[e.b].y, w[e.b].z};
            double* ra = R + (size_t)ka * 6;
            double* rb = R + (size_t)kb * 6;
            for (int r = 0; r < 12; r++) {
                double s2 = 0;
                for (int q = 0; q < 12; q++) s2 += K[r][q] * u[q];
                if (r < 6) {
                    if (!fa) ra[r] -= cr * s2;
                } else if (!fb) {
                    rb[r - 6] -= cr * s2;
                }
            }
        }
        // the geometric stiffness of tension across the member between the displacements, theta h^2 (N / L)(I - e1 e1^T)
        // (on the matrix only: against the velocities it would brake a spinning frame)
        const double g = geo ? sign * h2 * t.ngeo : 0.0;
        double P[9];
        const double ee[3] = {t.e1.x, t.e1.y, t.e1.z};
        for (int r = 0; r < 3; r++)
            for (int q = 0; q < 3; q++) P[r * 3 + q] = g * ((r == q ? 1.0 : 0.0) - ee[r] * ee[q]);
        auto add_block = [&](double* D, int ro, int co, double sign_geo) {
            for (int r = 0; r < 6; r++)
                for (int q = 0; q < 6; q++) D[r * 6 + q] += c * K[ro + r][co + q];
            if (g != 0)
                for (int r = 0; r < 3; r++)
                    for (int q = 0; q < 3; q++) D[r * 6 + q] += sign_geo * P[r * 3 + q];
        };
        if (!fa) add_block(Dg + (size_t)ka * 36, 0, 0, 1.0);
        if (!fb) add_block(Dg + (size_t)kb * 36, 6, 6, 1.0);
        const int p = elem_block_[ei];
        if (p >= 0 && !fa && !fb) {
            // the block (row node, column node) of the lower triangle
            const int ro = elem_swap_[ei] ? 0 : 6, co = elem_swap_[ei] ? 6 : 0;
            add_block(Of + (size_t)p * 36, ro, co, -1.0);
        }
    };
    if (stage <= 1) {
        changed.clear();
        if (comp < (int)comp_pass_.size()) comp_pass_[comp] = 0;
    }
    if (stage == 2 && comp < (int)comp_repass_.size()) comp_repass_[comp] = 0;
    if ((stage == 2 || stage == 4) && comp < (int)comp_back_.size()) comp_back_[comp] = 0;
    // (stage 2 and 3 of a large component: from the pass it is at - a pass again goes through the team's stages too;
    // 4: on from its backward substitution's subtrees, solve_back)
    bool resume = stage == 4;
    const bool staged = (stage == 2 || stage == 4) && par_component(comp);
    for (int pass = stage >= 2 ? comp_pass_[comp] : 0; pass < 3; pass++) {
    const ParPlan* plan = staged ? &plans_[comp_plan_[comp]] : nullptr;
    if (!resume) {
    PROFILE_ACCUM("Frame assemble");
    if (stage != 2) st.passes++;
    if (pass > 0) {
        if (stage != 2) {
        // again with the unloaded hinges' elastic tangents: those members' change into the system assembled before
        // (kept: the factorization works in place), not the whole assembly again
        for (const Changed& ch : changed) {
            Tangent was = tan_[ch.elem];
            was.unload = ch.unload;
            add_member(ch.elem, effective(was), -1.0, false, diagA_.data(), offA_.data(), rhsA_.data());
            add_member(ch.elem, effective(tan_[ch.elem]), 1.0, false, diagA_.data(), offA_.data(), rhsA_.data());
        }
        changed.clear();
        std::copy(diagA_.begin() + d0, diagA_.begin() + d1, diag_.begin() + d0);
        std::copy(offA_.begin() + o0, offA_.begin() + o1, off_.begin() + o0);
        std::copy(rhsA_.begin() + r0, rhsA_.begin() + r1, rhs_.begin() + r0);
        }
    } else if (stage != 2) {
    // the nodes' masses, the members' and the triangles' blocks (solve_gather: a large component's in the team's chunks,
    // before this)
    if (stage == 0)
        for (int ch = 0, nch = asm_chunks(comp); ch < nch; ch++) solve_gather(b, comp, ch);
    // the body's springs on frame nodes: (theta h^2 k + h d) e e^T, their tangent now (a shock past or near its bound:
    // the bound's stiffness; a slack rope: none)
    if (pass == 0 && comp < (int)comp_onesided_.size()) comp_onesided_[comp].clear();
    for (uint32_t li : comp_links_[comp]) {
        const Link& l = links_[li];
        if (l.beam >= b.beams.size()) continue;
        const Beam& bm = b.beams[l.beam];
        if (bm.flags & BF_BROKEN) continue;
        const vec3 d = b.nodes[bm.b].p - b.nodes[bm.a].p;
        const float len = length(d);
        if (!(len > 1e-6f)) continue;
        const vec3 e = d / len;
        float k = bm.k, dmp = bm.d;
        if (l.shock >= 0 && l.shock < (int)b.shocks.size()) {
            const Shock& sh = b.shocks[l.shock];
            const float diff = len - bm.L, margin = 0.02f;
            k = std::max(sh.spring, std::max(sh.spring_in, sh.spring_out));
            dmp = std::max(sh.damp, std::max(sh.damp_in, sh.damp_out));
            if (diff > sh.long_bound * bm.L - margin || diff < -sh.short_bound * bm.L + margin) k = std::max(k, sh.bound_spring), dmp = std::max(dmp, sh.bound_damp);
        } else if (bm.type == BT_ROPE && len < bm.L) {
            k = 0;
        } else if (bm.type == BT_SUPPORT && len > bm.L) {
            k = 0;
        }
        const double c = (double)theta * h * h * k + (double)h * dmp;
        if (!(c > 0)) continue;
        const double ee[3] = {e.x, e.y, e.z};
        // one end off the frame: that end's share of its mass condensed in (a pinned one: an anchor, as it is)
        double cf = c;
        static const bool old_one_sided = getenv("BL_ONESIDED_OLD") != nullptr; // (diagnostics: the frame's side alone, as before)
        if ((l.fa < 0) != (l.fb < 0) && !old_one_sided) {
            const uint32_t o = l.fa >= 0 ? bm.b : bm.a;
            const Node& xo = b.nodes[o];
            if (xo.inv_mass > 0) {
                const float share = 1.0f / (float)std::max<int>(1, o < onesided_n_.size() ? onesided_n_[o] : 1);
                const double mb = (double)xo.mass * share;
                cf = c * mb / (mb + c);
                if (pass == 0 && comp < (int)comp_onesided_.size())
                    comp_onesided_[comp].push_back({(uint32_t)(l.fa >= 0 ? l.fa : l.fb), o, e, (float)c, (float)mb, share});
            }
        }
        auto add_diag = [&](int fn) {
            if (fn < 0 || fixed[fn]) return;
            double* D = &diag_[(size_t)iperm_[fn] * 36];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) D[i * 6 + j] += cf * ee[i] * ee[j];
        };
        add_diag(l.fa);
        add_diag(l.fb);
        if (l.block >= 0 && !fixed[l.fa] && !fixed[l.fb]) {
            double* O = &off_[(size_t)l.block * 36];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) O[i * 6 + j] -= c * ee[i] * ee[j];
        }
    }
    // the plates pressed at their middles (tri_press): kappa c c^T on their corners' translations, c the middle along the
    // contact's normal, the right side kappa (step a) c
    if (!tri_press.empty())
        for (uint32_t ti : comp_tris_[comp]) {
            if (ti >= tri_press.size() || !(tri_press[ti].kap > 0) || tris[ti].broken) continue;
            const TriPress& tp = tri_press[ti];
            const FrameTri& t = tris[ti];
            const double kk = (double)tp.kap / 9.0, rr = (double)tp.kap * step * tp.a / 3.0;
            const double ee[3] = {tp.n.x, tp.n.y, tp.n.z};
            for (int q = 0; q < 3; q++) {
                const uint32_t fn = t.n[q], f2 = t.n[q == 2 ? 0 : q + 1];
                if (fixed[fn]) continue;
                double* D = &diag_[(size_t)iperm_[fn] * 36];
                double* R = &rhs_[(size_t)iperm_[fn] * 6];
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) D[i * 6 + j] += kk * ee[i] * ee[j];
                    R[i] += rr * ee[i];
                }
                if (const int p = tri_block_[ti][q]; p >= 0 && !fixed[f2]) {
                    double* O = &off_[(size_t)p * 36];
                    for (int i = 0; i < 3; i++)
                        for (int j = 0; j < 3; j++) O[i * 6 + j] += kk * ee[i] * ee[j];
                }
            }
        }
    // the released ends' damping (compute_forces put its torque from the velocities before the step in): h c on the
    // two nodes' turning, against each other, so it takes the turning after the step (explicit, a hinge of 2 N m s/rad
    // on a lid's light frame node spun it up and the lid's hinges tore off as the car stood)
    if (welded_.size() == node.size())
        for (uint32_t ei : celems) {
            const FrameElement& e = elems[ei];
            const FrameSection& s = sections[e.section];
            if (e.broken || !(s.joint_damp > 0) || (e.end_a != FJ_RIGID) == (e.end_b != FJ_RIGID) || !welded_[e.a] || !welded_[e.b]) continue;
            const double c = (double)h * s.joint_damp;
            for (uint32_t fn : {e.a, e.b})
                if (!fixed[fn])
                    for (int i = 3; i < 6; i++) diag_[(size_t)iperm_[fn] * 36 + i * 7] += c;
            if (const int p = elem_block_[ei]; p >= 0 && !fixed[e.a] && !fixed[e.b])
                for (int i = 3; i < 6; i++) off_[(size_t)p * 36 + i * 7] -= c;
        }
    // the damped mounts' ends: h c on the node's turning against the other end's at the step's start (that end in another
    // component's system: each side implicit on its own)
    for (const MountDamp& md : comp_mdamp_[comp]) {
        if (fixed[md.self] || md.mount >= mounts.size() || mounts[md.mount].broken) continue;
        const double c = (double)h * md.damp;
        const size_t ks = (size_t)iperm_[md.self];
        const vec3 dw = w0_[md.self] - w0_[md.other];
        for (int i = 3; i < 6; i++) diag_[ks * 36 + i * 7] += c;
        rhs_[ks * 6 + 3] -= c * dw.x, rhs_[ks * 6 + 4] -= c * dw.y, rhs_[ks * 6 + 5] -= c * dw.z;
    }
    one_sided_rhs(b, comp, h);
    if (any_yield) {
        std::copy(diag_.begin() + d0, diag_.begin() + d1, diagA_.begin() + d0);
        std::copy(off_.begin() + o0, off_.begin() + o1, offA_.begin() + o0);
        std::copy(rhs_.begin() + r0, rhs_.begin() + r1, rhsA_.begin() + r0);
    }
    }
    if (stage == 1 || stage == 3) { // (the factor's subtrees next, in the team's stages)
        tlap(0);
        return;
    }
    // block Cholesky, right-looking over the precomputed pattern (the diagonal blocks' inverse diagonals kept); after
    // the team's stages the columns above the subtrees alone
    PROFILE_ACCUM("Frame factor");
    tlap(0);
    if (plan) {
        bool failed = false;
        for (int g = 0; g + 1 < (int)plan->gptr.size(); g++) failed |= par_fail_[(size_t)par_fail_ptr_[comp_plan_[comp]] + g] != 0;
        if (failed) {
            st.failures++;
            for (int kk = k0; kk < k1; kk++) impulse[perm_[kk]] = vec3(0);
            return; // (as below)
        }
    }
    const int nk = plan ? (int)plan->top.size() : k1 - k0;
    for (int ik = 0; ik < nk; ik++) {
        const int k = plan ? plan->top[ik] : k0 + ik;
        double* Lkk = &diag_[(size_t)k * 36];
        double* dk = &dinv_[(size_t)k * 6];
        if (!chol6(Lkk, dk)) {
            st.failures++;
            for (int kk = k0; kk < k1; kk++) impulse[perm_[kk]] = vec3(0);
            return; // (the members' forces stay in b.force: an explicit step this once)
        }
        for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) {
            double* X = &off_[(size_t)p * 36];
            right_solve_lt(Lkk, dk, X);
        }
        for (int u = upd_ptr_[k]; u < upd_ptr_[k + 1]; u++) {
            const Update& up = upd_[u];
            double* T = up.target >= 0 ? &off_[(size_t)up.target * 36] : &diag_[(size_t)(-up.target - 1) * 36];
            sub_abt(T, &off_[(size_t)up.pj * 36], &off_[(size_t)up.pi * 36]);
        }
    }
    comp_factored_[comp] = 1;
    tlap(1);
    if (plan) {
        // L L^T x = r above the subtrees (theirs forward in solve_factor, backward in solve_back next): forward, each
        // column's row from the columns before it in their order (the same subtractions as the sweep's), then backward
        for (size_t t = 0; t < plan->top.size(); t++) {
            const int r = plan->top[t];
            double* y = &rhs_[(size_t)r * 6];
            for (int q = plan->tptr[t]; q < plan->tptr[t + 1]; q++) sub_mv(&off_[(size_t)plan->tblk[q] * 36], &rhs_[(size_t)plan->tcol[q] * 6], y);
            forward6(&diag_[(size_t)r * 36], &dinv_[(size_t)r * 6], y);
        }
        for (int t = (int)plan->top.size() - 1; t >= 0; t--) {
            const int k = plan->top[t];
            double* x = &rhs_[(size_t)k * 6];
            for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) sub_mtv(&off_[(size_t)p * 36], &rhs_[(size_t)row_[p] * 6], x);
            backward6(&diag_[(size_t)k * 36], &dinv_[(size_t)k * 6], x);
        }
        if (subst_staged(comp)) {
            comp_back_[comp] = 1;
            tlap(2);
            return;
        }
        for (int g = 0; g + 1 < (int)plan->gptr.size(); g++) solve_back(comp, g); // (a smaller one's subtrees here, in turn)
    } else {
        // L L^T x = r
        for (int k = k0; k < k1; k++) {
            double* y = &rhs_[(size_t)k * 6];
            forward6(&diag_[(size_t)k * 36], &dinv_[(size_t)k * 6], y);
            for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) sub_mv(&off_[(size_t)p * 36], y, &rhs_[(size_t)row_[p] * 6]);
        }
        for (int k = k1 - 1; k >= k0; k--) {
            double* x = &rhs_[(size_t)k * 6];
            for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) sub_mtv(&off_[(size_t)p * 36], &rhs_[(size_t)row_[p] * 6], x);
            backward6(&diag_[(size_t)k * 36], &dinv_[(size_t)k * 6], x);
        }
    }
    tlap(2);
    }
    resume = false;
    // did the step unload a yielding hinge (bar, twist)? then its elastic tangent, and the step again
    if (!any_yield || pass == 2) break;
    bool again = false;
    auto vel = [&](uint32_t fn, vec3& v, vec3& om) {
        if (fixed[fn]) {
            v = om = vec3(0);
            return;
        }
        const double* x = &rhs_[(size_t)iperm_[fn] * 6];
        v = b.nodes[node[fn]].v + vec3((float)x[0], (float)x[1], (float)x[2]);
        om = w[fn] + vec3((float)x[3], (float)x[4], (float)x[5]);
    };
    for (uint32_t ei : celems) {
        Tangent& t = tan_[ei];
        const uint8_t y = (uint8_t)(t.yield & ~t.unload);
        if (!t.on || !y || elems[ei].broken) continue;
        vec3 va, wa, vb, wb;
        vel(elems[ei].a, va, wa), vel(elems[ei].b, vb, wb);
        const float iL = 1.0f / t.L;
        const vec3 dv = vb - va;
        // the rates of the deformations (the rows of B)
        const vec2 ra(dot(t.e3, dv) * iL + dot(t.e2, wa), -dot(t.e2, dv) * iL + dot(t.e3, wa));
        const vec2 rb(dot(t.e3, dv) * iL + dot(t.e2, wb), -dot(t.e2, dv) * iL + dot(t.e3, wb));
        uint8_t u = 0;
        if ((y & 1) && dot(ra, t.ma) < 0) u |= 1;
        if ((y & 2) && dot(rb, t.mb) < 0) u |= 2;
        if ((y & 4) && dot(t.e1, dv) * t.N < 0) u |= 4;
        if ((y & 8) && dot(t.e1, wb - wa) * t.T < 0) u |= 8;
        if (u) changed.push_back({(uint32_t)ei, t.unload}), t.unload |= u, t.elastic_hold = 16, again = true;
    }
    tlap(3);
    if (again && k1 - k0 >= 60) st.repasses++; // (a large component's: the cost the budget is for)
    if (again && plan) { // (the pass again in the team's stages: solve_repass, solve_factor, solve_defer, solve_finish)
        comp_pass_[comp] = (uint8_t)(pass + 1);
        comp_repass_[comp] = 1;
        return;
    }
    if (!again) break;
    }
    one_sided_react(b, comp, h);
    // the new velocities: to the body's integrator as a force, the rotations advanced here
    vec3* Fw = b.force.data();
    for (int kk = k0; kk < k1; kk++) {
        const int i = perm_[kk];
        if (fixed[i]) {
            w[i] = vec3(0);
            continue;
        }
        const double* x = &rhs_[(size_t)kk * 6];
        const Node& nd = b.nodes[node[i]];
        vec3 dv((float)x[0], (float)x[1], (float)x[2]);
        const vec3 dw((float)x[3], (float)x[4], (float)x[5]);
        if (!std::isfinite(dv.x + dv.y + dv.z + dw.x + dw.y + dw.z)) {
            st.failures++;
            continue;
        }
        // (safety: no real impact changes a node's velocity by 40 m/s in a step or spins it past 3000 rad/s; a node
        // squeezed between heavy bodies by stiff contacts could, and would fling the frame apart)
        const float kMaxDv = 40.0f, kMaxW = 3000.0f;
        if (const float l = length(dv); l > kMaxDv) dv *= kMaxDv / l, st.clamps++;
        Fw[node[i]] = dv * (nd.mass / step);
        w[i] += dw;
        if (const float l = length(w[i]); l > kMaxW) w[i] *= kMaxW / l, st.clamps++;
        static const bool dbg = getenv("BL_FRAMEDBG") != nullptr;
        if (dbg && (length(dv) >= kMaxDv * 0.999f || length(w[i]) >= kMaxW * 0.999f))
            printf("frame: node %u (component %d of %d nodes) clamped: dv %.1f m/s, spin %.0f rad/s\n", node[i], comp, k1 - k0, length(dv), length(w[i]));
    }
    tlap(4);
}

// (diagnostics, BL_FEMPROF: the components' times summed over the run, printed at the exit)
namespace {
struct FemProfRow {
    const void* frame = nullptr;
    int comp = 0, nodes = 0, blocks = 0, updates = 0, members = 0, tris = 0;
    long long calls = 0, passes = 0, held = 0;
    double t[6] = {};
};
std::mutex g_femprof_mx;
std::vector<FemProfRow> g_femprof;
void femprof_print() {
    std::lock_guard<std::mutex> lk(g_femprof_mx);
    std::sort(g_femprof.begin(), g_femprof.end(), [](const FemProfRow& a, const FemProfRow& b) { return a.t[0] + a.t[1] + a.t[2] + a.t[5] > b.t[0] + b.t[1] + b.t[2] + b.t[5]; });
    printf("femprof: frame comp nodes blocks updates members tris calls passes held assemble_ms factor_ms subst_ms unload_ms rest_ms held_ms\n");
    for (const FemProfRow& r : g_femprof)
        if (r.calls + r.held > 0)
            printf("femprof: %p %d %d %d %d %d %d %lld %lld %lld %.3f %.3f %.3f %.3f %.3f %.3f\n", r.frame, r.comp, r.nodes, r.blocks, r.updates, r.members, r.tris, r.calls,
                   r.passes, r.held, r.t[0], r.t[1], r.t[2], r.t[3], r.t[4], r.t[5]);
}
} // namespace

void FemFrame::prof_flush() {
    static const bool femprof = getenv("BL_FEMPROF") != nullptr;
    if (!femprof) return;
    std::lock_guard<std::mutex> lk(g_femprof_mx);
    static bool registered = (atexit(femprof_print), true);
    (void)registered;
    for (size_t c = 0; c < comp_stats_.size() && c < comp_range_.size(); c++) {
        CompStats& cs = comp_stats_[c];
        if (cs.passes == 0 && cs.held == 0) continue;
        const int a = comp_range_[c].first, e = comp_range_[c].second;
        FemProfRow* row = nullptr;
        for (FemProfRow& r : g_femprof)
            if (r.frame == this && r.comp == (int)c && r.nodes == e - a) row = &r;
        if (!row) {
            g_femprof.push_back({});
            row = &g_femprof.back();
            row->frame = this, row->comp = (int)c, row->nodes = e - a, row->blocks = col_ptr_[e] - col_ptr_[a], row->updates = upd_ptr_[e] - upd_ptr_[a];
            row->members = (int)comp_elems_[c].size(), row->tris = c < comp_tris_.size() ? (int)comp_tris_[c].size() : 0;
        }
        row->calls += cs.passes > 0, row->passes += cs.passes, row->held += cs.held;
        for (int i = 0; i < 6; i++) row->t[i] += cs.t[i], cs.t[i] = 0;
        cs.passes = 0, cs.held = 0;
    }
}

void FemFrame::node_inertia(uint32_t fn, const SoftBody& b) {
    // lumped rotational inertia of the members' halves (HRZ lumping of the consistent mass: m L^2 / 78 per end), with
    // a floor from the node's own mass
    float I = 0;
    for (const FrameElement& e : elems)
        if (!e.broken && (e.a == fn || e.b == fn)) I += element_mass(e) * e.L0 * e.L0 / 78.0f;
    // (a triangle's corner: its third of the mass over its third of the area, as a patch turning about its middle)
    for (const FrameTri& t : tris)
        if (!t.broken && (t.n[0] == fn || t.n[1] == fn || t.n[2] == fn)) I += t.mass * t.area0 / 108.0f;
    const float mn = node[fn] < b.nodes.size() ? b.nodes[node[fn]].mass : 0.0f;
    inertia[fn] = std::max(I, std::max(1e-6f, mn * 1e-4f));
}

int FemFrame::members_at(uint32_t fn) const {
    int n = 0;
    for (const FrameElement& e : elems) n += !e.broken && (e.a == fn || e.b == fn);
    for (const FrameTri& t : tris) n += !t.broken && (t.n[0] == fn || t.n[1] == fn || t.n[2] == fn); // (a shell's corner)
    return n;
}

namespace {

// a copy of body node `like` (appended), as the sheets' topology operations make them
uint32_t clone_node(SoftBody& b, uint32_t like) {
    const size_t n0 = b.nodes.size();
    const uint32_t id = (uint32_t)n0;
    b.nodes.push_back(b.nodes[like]);
    NodeInfo inf = b.info[like];
    inf.frame = -1;
    inf.flags &= (uint16_t)~NF_FRAME;
    b.info.push_back(inf);
    b.force.push_back(vec3(0));
    if (b.wind_area.size() == n0) b.wind_area.push_back(b.wind_area[like]);
    if (b.node_base_mass.size() == n0) b.node_base_mass.push_back(0.0f);
    if (b.node_shells.size() == n0) b.node_shells.emplace_back();
    if (b.node_wheel.size() == n0) b.node_wheel.push_back(-1);
    if (b.topo_log.nodes.size() < 100000) b.topo_log.nodes.push_back({id, like});
    else b.topo_log.overflow = true;
    b.contacter_count = -1;
    b.copy_node_refs(like, id); // (a part's node: its copy held off the volumes too)
    return id;
}

void set_mass(Node& n, float m, bool fixed) {
    n.mass = m;
    n.inv_mass = fixed || m <= 0 ? 0.0f : 1.0f / m;
}

} // namespace

int FemFrame::split(SoftBody& b, uint32_t ei, float t) {
    PROFILE_ACCUM("Frame split");
    if (ei >= elems.size() || elems[ei].broken) return -1;
    Kin k;
    if (!kinematics(*this, elems[ei], k)) return -1;
    FrameElement e = elems[ei];
    t = std::clamp(t, 0.05f, 0.95f);
    if (e.L0 * std::min(t, 1.0f - t) < 0.005f) return -1;
    // the member's bent shape: the Hermite cubic of its end rotations against the chord (bending in each plane),
    // the twist between them linear; the new node on it, turned as the shape is there
    const float ay = k.ra.y - e.pa.x, by = k.rb.y - e.pb.x, az = k.ra.z - e.pa.y, bz = k.rb.z - e.pb.y;
    const float L = k.L, u = t;
    const float N1 = u * (1 - u) * (1 - u), N2 = u * u * (1 - u);                 // displacement shapes
    const float S1 = (1 - u) * (1 - 3 * u), S2 = u * (2 - 3 * u);                  // their slopes
    const float dv = L * (az * N1 - bz * N2), dw = -L * (ay * N1 - by * N2);       // across: along e2, e3
    const vec3 rm(k.ra.x + (k.rb.x - k.ra.x) * u, ay * S1 - by * S2, az * S1 - bz * S2); // (element axes)
    const double* xa = &xd[e.a * 3];
    const double px = xa[0] + (double)k.e1.x * u * L + (double)k.e2.x * dv + (double)k.e3.x * dw;
    const double py = xa[1] + (double)k.e1.y * u * L + (double)k.e2.y * dv + (double)k.e3.y * dw;
    const double pz = xa[2] + (double)k.e1.z * u * L + (double)k.e2.z * dv + (double)k.e3.z * dw;
    // the co-rotated frame of the member (as kinematics makes it) and the new node's orientation in it
    const quat Ea = normalize(q[e.a] * e.qa), Eb = normalize(q[e.b] * e.qb);
    const float tw = twist_weight(e);
    const quat Em = tw <= 0.0f ? Ea : tw >= 1.0f ? Eb : slerp(Ea, Eb, tw);
    const quat E = normalize(quat_from_to(normalize(Em.rotate(vec3(1, 0, 0))), k.e1) * Em);
    const quat qm = normalize(E * quat_exp(rm));
    // the body node: between the ends, its velocity interpolated; the member's mass moves with its halves
    const uint32_t na = node[e.a], nb = node[e.b];
    const uint32_t nm = clone_node(b, u < 0.5f ? na : nb);
    Node& m = b.nodes[nm];
    m.p = vec3((float)px, (float)py, (float)pz);
    m.v = b.nodes[na].v * (1 - u) + b.nodes[nb].v * u;
    b.info[nm].flags &= (uint16_t)~NF_FIXED;
    // (from the ends at most a quarter of what they carry: other members and springs lean on them too)
    const float take_a = std::min(e.mass * (1 - u) * 0.5f, b.nodes[na].mass * 0.25f), take_b = std::min(e.mass * u * 0.5f, b.nodes[nb].mass * 0.25f);
    set_mass(m, std::max(1e-3f, take_a + take_b), false);
    if (b.nodes[na].inv_mass > 0) set_mass(b.nodes[na], b.nodes[na].mass - take_a, false);
    if (b.nodes[nb].inv_mass > 0) set_mass(b.nodes[nb], b.nodes[nb].mass - take_b, false);
    const uint32_t fm = add_node(nm);
    q[fm] = qm;
    w[fm] = w[e.a] * (1 - u) + w[e.b] * u;
    xd[fm * 3] = px, xd[fm * 3 + 1] = py, xd[fm * 3 + 2] = pz;
    // the two halves: the rest frames at the new node are its own (it was made in the member's frame), the plastic
    // state goes with each end, the elongation and twist in proportion
    FrameElement c1 = e, c2 = e;
    c1.b = fm, c1.qb = quat(), c1.end_b = FJ_RIGID, c1.L0 = e.L0 * u, c1.mass = e.mass * u;
    c1.up = e.up * u, c1.tp = e.tp * u, c1.pb = vec2(0, 0), c1.bb = vec2(0, 0), c1.bt = e.bt * u, c1.dmg_b = 0, c1.torn = (uint8_t)(e.torn & 1);
    c2.a = fm, c2.qa = quat(), c2.end_a = FJ_RIGID, c2.L0 = e.L0 * (1 - u), c2.mass = e.mass * (1 - u);
    c2.up = e.up * (1 - u), c2.tp = e.tp * (1 - u), c2.pa = vec2(0, 0), c2.ba = vec2(0, 0), c2.bt = e.bt * (1 - u), c2.dmg_a = 0, c2.torn = (uint8_t)(e.torn & 2);
    c1.level = c2.level = (uint8_t)std::min(255, e.level + 1);
    elems[ei] = c1;
    elems.push_back(c2);
    node_inertia(e.a, b), node_inertia(e.b, b), node_inertia(fm, b);
    splits++;
    ready_ = false; // (the pattern: analysed again)
    return (int)fm;
}

bool FemFrame::tear(SoftBody& b, uint32_t ei, int end) {
    PROFILE_ACCUM("Frame tear");
    if (ei >= elems.size() || elems[ei].broken) return false;
    FrameElement& e = elems[ei];
    e.torn |= (uint8_t)(end ? 2 : 1);
    const uint32_t fn = end ? e.b : e.a;
    const int k = members_at(fn);
    const uint32_t v = node[fn];
    if (k <= 1 && b.nodes[v].inv_mass > 0) return false; // (a free end already; one on a fixed anchor tears off it)
    const uint32_t c = clone_node(b, v);
    // the copy takes the member's share of the node (a fixed anchor keeps its place: the torn end is free)
    const bool fixed = b.nodes[v].inv_mass <= 0;
    b.info[c].flags &= (uint16_t)~NF_FIXED;
    const float share = fixed ? std::max(0.1f, e.mass * 0.5f) : b.nodes[v].mass / (float)k;
    set_mass(b.nodes[c], share, false);
    if (!fixed) set_mass(b.nodes[v], b.nodes[v].mass - share, false);
    const uint32_t fc = add_node(c);
    q[fc] = q[fn];
    w[fc] = w[fn];
    xd[fc * 3] = xd[fn * 3], xd[fc * 3 + 1] = xd[fn * 3 + 1], xd[fc * 3 + 2] = xd[fn * 3 + 2];
    (end ? e.b : e.a) = fc;
    (end ? e.end_b : e.end_a) = FJ_RIGID;
    node_inertia(fn, b), node_inertia(fc, b);
    debris_check = true;
    broken++;
    b.stats.broken_beams++;
    ready_ = false;
    return true;
}

bool FemFrame::process_events(SoftBody& b) {
    if (events_.empty() && !(hit_.speed > 0)) return false;
    const size_t n0 = b.nodes.size();
    const int torn0 = tris_torn;
    // the substep's fastest contact on a plate lays a fracture pattern (first: the bisections below put their nodes on
    // its lines)
    if (hit_.speed > 0) {
        const int s0 = slot(hit_.n[0]), s1 = slot(hit_.n[1]), s2 = slot(hit_.n[2]);
        int at = -1;
        for (size_t u = 0; u < tris.size() && at < 0 && s0 >= 0; u++) {
            const FrameTri& t = tris[u];
            if (t.broken) continue;
            auto has = [&](int x) { return x >= 0 && ((int)t.n[0] == x || (int)t.n[1] == x || (int)t.n[2] == x); };
            if (has(s0) && has(s1) && has(s2)) at = (int)u;
        }
        if (at >= 0) {
            const vec3 p = b.nodes[hit_.n[0]].p * hit_.bary.x + b.nodes[hit_.n[1]].p * hit_.bary.y + b.nodes[hit_.n[2]].p * hit_.bary.z;
            add_impact(b, (uint32_t)at, p, hit_.speed, hit_.size, hit_.time);
        }
        hit_.speed = 0;
    }
    // the bisections, before the tears (a tear queued on a triangle bisected now waits for the finer shell), a few a
    // substep (each is the solver's pattern again: the rest queue again while they yield)
    std::vector<uint8_t> refined; // (the triangles bisected now, the halves that kept the index and the new ones)
    {
        int budget = steps_ - refine_step_ >= (uint32_t)kRefineEvery ? kRefinePerStep : 0;
        const size_t nt = tris.size();
        std::vector<uint8_t> lv;
        for (const Event& x : events_) {
            if (x.kind != 5 || budget <= 0 || x.elem >= nt || tris[x.elem].broken || tris[x.elem].whole) continue;
            refine_step_ = steps_;
            if (lv.empty()) {
                lv.resize(nt);
                for (size_t u = 0; u < nt; u++) lv[u] = tris[u].level;
            }
            if (tris[x.elem].level != lv[x.elem]) continue; // (bisected already, as a neighbour)
            const int k = refine_tri(b, x.elem);
            if (k) budget -= k;
            else tris[x.elem].whole = true;
        }
        if (!lv.empty()) {
            refined.assign(tris.size(), 1);
            for (size_t u = 0; u < nt; u++) refined[u] = tris[u].level != lv[u];
        }
    }
    if (events_.empty()) {
        if (!ready_) {
            torque.resize(node.size(), vec3(0));
            member_f.resize(node.size(), vec3(0));
            body_ = &b;
            analyse();
            tan_.assign(elems.size(), Tangent());
            tri_tan_.assign(tris.size(), TriTan());
            ready_ = true;
        }
        return b.nodes.size() != n0;
    }
    // the shocks' seats before the first tear: the members at each end's frame node
    for (Shock& s : b.shocks)
        if (s.seat[0] == 255 && s.beam < b.beams.size()) {
            const int sa = slot(b.beams[s.beam].a), sb = slot(b.beams[s.beam].b);
            s.seat[0] = sa < 0 ? 0 : (uint8_t)std::clamp(members_at((uint32_t)sa), 1, 254);
            s.seat[1] = sb < 0 ? 0 : (uint8_t)std::clamp(members_at((uint32_t)sb), 1, 254);
        }
    std::vector<Event> ev;
    ev.swap(events_);
    static const bool tdbg = getenv("BL_FRAMEDBG") != nullptr;
    for (const Event& x : ev) {
        switch (x.kind) {
        case 5: break; // (above)
        case 6: // a triangle that could not tear along an edge, far past it: its corners copied for it alone - if no edge
                // of it is joined (a piece hanging on a node); one still joined by an edge stays (pulled from its corners it
                // left a hole: two hundred of them on the curb)
            if (x.elem < tris.size() && !tris[x.elem].broken) {
                bool joined = false;
                for (int e = 0; e < 3 && !joined; e++) {
                    const uint32_t p0 = tris[x.elem].n[e], p1 = tris[x.elem].n[(e + 1) % 3];
                    for (size_t u = 0; u < tris.size() && !joined; u++) {
                        const FrameTri& o = tris[u];
                        if (u == x.elem || o.broken) continue;
                        int hit = 0;
                        for (uint32_t c : o.n) hit += c == p0 || c == p1;
                        joined = hit == 2;
                    }
                }
                if (joined) {
                    tris[x.elem].held = true;
                    break;
                }
                for (uint32_t c : tris[x.elem].n) {
                    int users = 0;
                    for (const FrameTri& u : tris) users += !u.broken && (u.n[0] == c || u.n[1] == c || u.n[2] == c);
                    const Node& nc = b.nodes[node[c]];
                    if (users > 1 || nc.inv_mass <= 0 || members_at(c) > 1) detach_tris(b, c, {x.elem});
                }
                tris[x.elem].dmg = 0; // (free now: not again)
                tris_torn++;
            }
            break;
        case 4: // a triangle torn: a crack along its edge across the plastic stretch, the nodes there duplicated
            if (x.elem < tris.size() && !tris[x.elem].broken && !(x.elem < refined.size() && refined[x.elem])) {
                const int k = tear_tri(b, x.elem);
                if (tdbg) {
                    const FrameTri& t = tris[x.elem];
                    printf("frame: triangle %u (%u %u %u) %s: plastic strain %.3f, %d edges torn\n", x.elem, node[t.n[0]], node[t.n[1]], node[t.n[2]],
                           k ? "tears" : "free all round", t.dmg, t.tears);
                }
            }
            break;
        case 0: split(b, x.elem, x.t); break;
        case 1: tear(b, x.elem, 0); break;
        case 2: tear(b, x.elem, 1); break;
        default: // split, and the half of the given end torn off the new node
            if (split(b, x.elem, x.t) >= 0) tear(b, x.elem, 1);
            else elems[x.elem].damage = 0;
            break;
        }
    }
    // a mount whose seat tore (a member torn off one of its nodes) lets go (the members at each frame node counted once:
    // members_at for every mount's node was a millisecond a crash's substep)
    static const bool dbg = getenv("BL_FRAMEDBG") != nullptr;
    std::vector<int> at(node.size(), 0);
    for (const FrameElement& e : elems)
        if (!e.broken) {
            if (e.a < at.size()) at[e.a]++;
            if (e.b < at.size() && e.b != e.a) at[e.b]++;
        }
    for (const FrameTri& t : tris)
        if (!t.broken)
            for (int c = 0; c < 3; c++) {
                const uint32_t f = t.n[c];
                if (f < at.size() && (c == 0 || f != t.n[0]) && (c < 2 || f != t.n[1])) at[f]++;
            }
    auto members_now = [&](uint32_t fn) { return fn < at.size() ? at[fn] : 0; };
    for (size_t i = 0; i < mounts.size(); i++) {
        FrameMount& m = mounts[i];
        if (m.broken || !m.made) continue;
        bool torn = false;
        const int na = std::max(1, m.na);
        for (int j = 0; j < na && !torn; j++) {
            const int sj = slot(m.na > 0 ? m.an[j] : m.a);
            torn = sj < 0 || members_now((uint32_t)sj) < m.members[j];
        }
        const int sb = slot(m.b);
        torn |= sb < 0 || members_now((uint32_t)sb) < m.members[4];
        if (!torn) continue;
        if (dbg) printf("frame: mount %zu (%u-%u) lets go at %.0f N (breaks at %.0f): its seat tore\n", i, m.a, m.b, m.f, m.brk);
        m.broken = true;
        mounts_broken++;
        b.stats.broken_beams++;
        debris_check = true;
        ready_ = false;
    }
    // and a shock whose seat tore down to a member or none: the node left on it hung on its spring and bump stop (the
    // Buggy's shock towers torn off in a head-on: a top, half a kilo on a single tube and the stop's 2e7 N/m, flung at
    // 400 m/s); a hub that lost its steering arm keeps its tyre's beams
    for (size_t i = 0; i < b.shocks.size(); i++) {
        const Shock& s = b.shocks[i];
        if (s.beam >= b.beams.size() || (b.beams[s.beam].flags & BF_BROKEN)) continue;
        bool torn = false;
        for (int e = 0; e < 2 && !torn; e++) {
            if (s.seat[e] == 0 || s.seat[e] == 255) continue;
            const int sl = slot(e ? b.beams[s.beam].b : b.beams[s.beam].a);
            torn = sl < 0 || (members_now((uint32_t)sl) < s.seat[e] && members_now((uint32_t)sl) <= 1);
        }
        if (!torn) continue;
        if (dbg) printf("frame: shock %zu (%u-%u) lets go: its seat tore\n", i, b.beams[s.beam].a, b.beams[s.beam].b);
        b.beams[s.beam].flags |= BF_BROKEN;
        b.stats.broken_beams++;
    }
    // the fragments the tears cut off loose at once (their frame's step no longer theirs)
    if (tris_torn != torn0 && loose_mass_ > 0 && !b.rigid) {
        std::vector<std::unique_ptr<SoftBody>> none;
        detach_debris(b, none, loose_mass_, true);
    }
    if (!ready_) {
        // the frame nodes' pattern again (their orientation, velocities and positions are kept)
        torque.resize(node.size(), vec3(0));
        member_f.resize(node.size(), vec3(0));
        body_ = &b;
        analyse();
        tan_.assign(elems.size(), Tangent());
        tri_tan_.assign(tris.size(), TriTan());
        ready_ = true;
    }
    return b.nodes.size() != n0;
}

void FemFrame::sync_positions(SoftBody& b, float h) { sync_positions(b, h, 0, node.size()); }

void FemFrame::sync_positions(SoftBody& b, float h, size_t i0, size_t i1) {
    for (size_t i = i0; i < i1 && i < node.size(); i++) {
        Node& n = b.nodes[node[i]];
        double* x = &xd[i * 3];
        // (the step the integrator made, x += h v', in double; the orientation turned as far)
        x[0] += (double)h * n.v.x, x[1] += (double)h * n.v.y, x[2] += (double)h * n.v.z;
        if (length2(w[i]) > 0) q[i] = normalize(quat_exp(w[i] * h) * q[i]);
        // (where the integrator put it, within the float's rounding: the double position; else something moved it)
        const double tol = 1e-4 * (1.0 + std::fabs(n.p.x) + std::fabs(n.p.y) + std::fabs(n.p.z));
        if (n.inv_mass > 0 && std::fabs(x[0] - n.p.x) <= tol && std::fabs(x[1] - n.p.y) <= tol && std::fabs(x[2] - n.p.z) <= tol)
            n.p = vec3((float)x[0], (float)x[1], (float)x[2]);
        else
            x[0] = n.p.x, x[1] = n.p.y, x[2] = n.p.z;
    }
}

// ------------------------------------------------------------------------------------------------ debris
void FemFrame::compact(SoftBody& b) {
    std::vector<int32_t> keep(node.size(), -1);
    std::vector<FrameElement> es;
    for (const FrameElement& e : elems)
        if (!e.broken) es.push_back(e);
    for (const FrameElement& e : es) keep[e.a] = keep[e.b] = 0;
    std::vector<FrameTri> ts;
    std::vector<int32_t> tri_new(tris.size(), -1); // (a sheet over the triangles: its shells' elements renumbered too)
    for (size_t u = 0; u < tris.size(); u++)
        if (!tris[u].broken) tri_new[u] = (int32_t)ts.size(), ts.push_back(tris[u]);
    for (Shell& sh : b.shells)
        if (sh.host >= 0) sh.host = (size_t)sh.host < tri_new.size() ? tri_new[sh.host] : -1;
    // (the plates' mid points' pairs follow: World::inherit_pairs)
    std::vector<int32_t>& rm = b.topo_log.mid_remap;
    if (rm.empty()) rm = tri_new;
    else
        for (int32_t& x : rm) x = x >= 0 && (size_t)x < tri_new.size() ? tri_new[x] : -1;
    for (const FrameTri& t : ts) keep[t.n[0]] = keep[t.n[1]] = keep[t.n[2]] = 0;
    std::vector<uint32_t> nn;
    std::vector<quat> qq;
    std::vector<vec3> ww, tt, cc, jj;
    std::vector<float> ii;
    std::vector<double> xx;
    for (size_t i = 0; i < node.size(); i++) {
        if (keep[i] < 0) continue;
        keep[i] = (int32_t)nn.size();
        nn.push_back(node[i]), qq.push_back(q[i]), ww.push_back(w[i]), ii.push_back(inertia[i]);
        tt.push_back(vec3(0)), cc.push_back(vec3(0)), jj.push_back(i < impulse.size() ? impulse[i] : vec3(0));
        xx.insert(xx.end(), {xd[i * 3], xd[i * 3 + 1], xd[i * 3 + 2]});
    }
    for (FrameElement& e : es) e.a = (uint32_t)keep[e.a], e.b = (uint32_t)keep[e.b];
    for (FrameTri& t : ts)
        for (uint32_t& v : t.n) v = (uint32_t)keep[v];
    for (uint32_t v : node)
        if (v < slot_.size()) slot_[v] = -1;
    node.swap(nn), q.swap(qq), w.swap(ww), inertia.swap(ii), torque.swap(tt), contact_n.swap(cc), impulse.swap(jj), xd.swap(xx);
    member_f.assign(node.size(), vec3(0));
    contact_f.assign(node.size(), vec3(0));
    tri_press.clear();
    elems.swap(es);
    tris.swap(ts);
    tri_k_.assign(tris.size() * kTriK, 0.0f), tri_b_.assign(tris.size() * kTriB, 0.0f), tri_k_ok_.assign(tris.size(), 0);
    for (size_t i = 0; i < node.size(); i++) slot_[node[i]] = (int32_t)i;
    events_.clear();
    body_ = &b;
    analyse();
    tan_.assign(elems.size(), Tangent());
    tri_tan_.assign(tris.size(), TriTan());
    ready_ = true;
}

int FemFrame::detach_debris(SoftBody& b, std::vector<std::unique_ptr<SoftBody>>& out, float max_mass, bool loose_only) {
    if (!loose_only) debris_check = false, loose_mass_ = max_mass;
    const size_t n = b.nodes.size();
    // the body's parts: nodes joined by anything (members, beams, sheets, triangles, joints, slides, capsules)
    std::vector<uint32_t> up(n);
    for (uint32_t i = 0; i < n; i++) up[i] = i;
    auto find = [&](uint32_t x) {
        while (up[x] != x) x = up[x] = up[up[x]];
        return x;
    };
    auto join = [&](uint32_t a, uint32_t c) {
        if (a < n && c < n) up[find(a)] = find(c);
    };
    std::vector<char> other(n, 0); // (held by something else than frame members)
    std::vector<char> held(n, 0);  // (... than frame members, triangle elements and their collision triangles)
    for (const FrameElement& e : elems)
        if (!e.broken) join(node[e.a], node[e.b]);
    // (a part of members alone leaves as debris; one with triangle elements too, if nothing else holds it: a panel torn
    // off the shell, a corner of it - a rigid body of its own then, and the shell's step no longer pays for it)
    std::vector<char> fem_coll(b.tris.size(), 0);
    for (const FrameTri& t : tris)
        if (!t.broken) {
            join(node[t.n[0]], node[t.n[1]]), join(node[t.n[1]], node[t.n[2]]);
            for (uint32_t v : t.n) other[node[v]] = 1;
            if (t.coll >= 0 && t.coll < (int)fem_coll.size()) fem_coll[t.coll] = 1;
        }
    auto hold = [&](uint32_t v) {
        if (v < n) other[v] = held[v] = 1;
    };
    // (a part on mounts, a sheet on welds: held by them; a part's frame and its skin go together)
    for (const FrameMount& m : mounts)
        if (!m.broken) {
            join(m.a, m.b), hold(m.a), hold(m.b);
            for (int k = 1; k < m.nb; k++) join(m.a, m.bn[k]), hold(m.bn[k]);
        }
    for (const Weld& wd : b.welds) {
        if (wd.broken) continue;
        for (uint32_t i = wd.first; i < wd.first + wd.count && i < b.weld_nodes.size(); i++) join(wd.anchor, b.weld_nodes[i]);
        hold(wd.anchor);
    }
    for (const Beam& bm : b.beams)
        if (!(bm.flags & BF_BROKEN)) join(bm.a, bm.b), hold(bm.a), hold(bm.b);
    // (a sheet over the triangle elements goes with them; any other holds its nodes)
    for (const Shell& sh : b.shells) {
        join(sh.n[0], sh.n[1]), join(sh.n[1], sh.n[2]);
        if (!b.on_plate(sh)) hold(sh.n[0]), hold(sh.n[1]), hold(sh.n[2]);
        else
            for (uint32_t v : sh.n)
                if (v < n) other[v] = 1;
    }
    const std::vector<char> of_shell = b.shell_tri_mask(); // (the sheet's own: with its shells above)
    for (size_t ti = 0; ti < b.tris.size(); ti++) { // (a hull triangle holds nothing: it goes with the debris, torn)
        const Triangle& t = b.tris[ti];
        if (t.torn || !t.two_sided || of_shell[ti]) continue;
        join(t.a, t.b), join(t.b, t.c);
        if (fem_coll[ti]) other[t.a] = other[t.b] = other[t.c] = 1; // (a triangle element's: it goes with it)
        else hold(t.a), hold(t.b), hold(t.c);                        // (the body's skin, its mesh's: it stays)
    }
    for (const Joint& j : b.joints)
        if (!j.broken && j.parent_frame < b.frames.size()) join(b.frames[j.parent_frame].node, j.child_node), hold(j.child_node);
    for (const Capsule& c : b.capsules) join(c.a, c.b), hold(c.a), hold(c.b);
    for (const SlideNode& sl : b.slides) {
        hold(sl.node);
        for (uint32_t r : sl.rail) join(sl.node, r), hold(r);
    }
    for (const Wheel& wh : b.wheels) {
        for (uint32_t v : wh.nodes) hold(v);
        for (uint32_t v : wh.rim) hold(v);
        hold(wh.axle0), hold(wh.axle1);
    }
    // (a collision volume's anchors: its part stays - the volume fit to the debris' nodes left behind pushed the other
    // car off with its engine's mass, MN)
    for (const CollisionVolume& cv : b.volumes)
        if (!cv.broken)
            for (uint32_t a : cv.anchors) hold(a);
    struct Part {
        float mass = 0;
        int members = 0, tris = 0;
        bool other = false, held = false, fixed = false;
    };
    std::vector<Part> parts(n);
    for (uint32_t i = 0; i < n; i++) {
        Part& p = parts[find(i)];
        p.mass += b.nodes[i].mass;
        p.other |= other[i] != 0;
        p.held |= held[i] != 0;
        p.fixed |= b.nodes[i].inv_mass <= 0 && b.nodes[i].mass > 0 && (b.info[i].flags & NF_FIXED);
    }
    for (const FrameElement& e : elems)
        if (!e.broken) parts[find(node[e.a])].members++;
    for (const FrameTri& t : tris)
        if (!t.broken) parts[find(node[t.n[0]])].tris++;
    uint32_t main = 0;
    for (uint32_t i = 0; i < n; i++)
        if (parts[find(i)].mass > parts[main].mass) main = find(i);
    static const bool dbg = getenv("BL_FRAMEDBG") != nullptr;
    if (dbg) {
        // (the frame's own parts: its members alone)
        std::vector<uint32_t> fu(node.size());
        for (uint32_t i = 0; i < fu.size(); i++) fu[i] = i;
        auto ff = [&](uint32_t x) {
            while (fu[x] != x) x = fu[x] = fu[fu[x]];
            return x;
        };
        for (const FrameElement& e : elems)
            if (!e.broken) fu[ff(e.a)] = ff(e.b);
        std::vector<int> cnt(node.size(), 0);
        for (const FrameElement& e : elems)
            if (!e.broken) cnt[ff(e.a)]++;
        std::string sizes;
        for (size_t i = 0; i < cnt.size(); i++)
            if (cnt[i] > 0) sizes += " " + std::to_string(cnt[i]);
        printf("frame: %s: members alone make parts of%s members\n", b.name.c_str(), sizes.c_str());
    }
    if (dbg)
        for (uint32_t i = 0; i < n; i++)
            if (find(i) == i && parts[i].members > 0)
                printf("frame: part of %s: %.1f kg, %d members%s%s%s\n", b.name.c_str(), parts[i].mass, parts[i].members, parts[i].other ? ", held by other elements" : "",
                       parts[i].fixed ? ", fixed" : "", i == main ? " (main)" : "");
    // a fragment torn off the shell - a few triangle elements, no members, held by nothing else: out of the frame's
    // step (each was a component of its own, a few light nodes at a steel plate's stiffness), its triangles kept to be
    // drawn (loose_tris) and its shape held by stiff springs of the body along their edges and across the shared ones;
    // its nodes stay the body's (its contacts, its group, the volumes as before - a torn-off panel made a rigid body
    // of its own, wedged between two cars, pushed them apart)
    static const bool no_loose = getenv("BL_NOLOOSE") != nullptr; // (diagnostics: fragments stay in the frame's step)
    bool changed = false;
    if (!no_loose) {
        std::vector<char> loose(n, 0);
        for (uint32_t i = 0; i < n; i++) {
            const uint32_t r = find(i);
            const Part& p = parts[r];
            loose[i] = r != main && p.tris > 0 && p.tris <= kLooseTris && p.members == 0 && !p.held && !p.fixed && p.mass <= max_mass;
        }
        const float h = std::max(last_h_, 5e-4f); // (the frame's step or the substep's, the longer: a body stepping alone after its sub-cycling)
        std::vector<std::pair<uint32_t, uint32_t>> edges;   // (a body node pair each, its triangle's third node)
        std::vector<uint32_t> third;
        std::vector<uint32_t> roots_done;
        for (size_t ti = 0; ti < tris.size(); ti++) {
            FrameTri& t = tris[ti];
            if (t.broken || !loose[node[t.n[0]]]) continue;
            const uint32_t r = find(node[t.n[0]]);
            if (std::find(roots_done.begin(), roots_done.end(), r) != roots_done.end()) continue;
            roots_done.push_back(r);
            // (the fragment's triangles, its edges)
            edges.clear(), third.clear();
            float mmin = 1e30f;
            for (size_t tj = ti; tj < tris.size(); tj++) {
                FrameTri& u = tris[tj];
                if (u.broken || find(node[u.n[0]]) != r) continue;
                const uint32_t v[3] = {node[u.n[0]], node[u.n[1]], node[u.n[2]]};
                loose_tris.push_back({{v[0], v[1], v[2]}, u.section, u.area0});
                for (int e = 0; e < 3; e++) {
                    const uint32_t x = std::min(v[e], v[(e + 1) % 3]), y = std::max(v[e], v[(e + 1) % 3]);
                    edges.push_back({x, y}), third.push_back(v[(e + 2) % 3]);
                    mmin = std::min(mmin, b.nodes[v[e]].mass);
                }
                u.broken = true;
            }
            // (springs at a fifth of the explicit limit of its lightest node, a tenth of critical damping)
            const float k = 0.2f * mmin / (h * h), d = 0.1f * mmin / h;
            for (size_t e = 0; e < edges.size(); e++) {
                bool first = true;
                for (size_t f2 = 0; f2 < e; f2++)
                    if (edges[f2] == edges[e]) {
                        first = false;
                        if (third[f2] != third[e]) b.add_beam(third[f2], third[e], 0.5f * k, d, 1e30f, 1e30f, BT_NORMAL, BF_NO_DEFORM); // (across it)
                    }
                if (first) b.add_beam(edges[e].first, edges[e].second, k, d, 1e30f, 1e30f, BT_NORMAL, BF_NO_DEFORM);
            }
            loose_count++;
            changed = true;
        }
    }
    // the debris: parts of frame members alone, light, not the body's main part
    std::vector<int32_t> piece_of(n, -1);
    std::vector<uint32_t> roots;
    for (uint32_t i = 0; i < n; i++) {
        const uint32_t r = find(i);
        const Part& p = parts[r];
        if (r == main || p.members == 0 || p.other || p.fixed || p.mass > max_mass || b.nodes[i].inv_mass <= 0) continue;
        if (piece_of[r] < 0) piece_of[r] = (int32_t)roots.size(), roots.push_back(r);
        piece_of[i] = piece_of[r];
    }
    if (roots.empty() || loose_only) { // (the debris: the world's, between frames)
        if (changed) {
            compact(b); // (the plates' mid points' pairs renumbered: TopoLog::mid_remap)
            b.topo_changed = true;
            b.topo_version++;
            b.shk.version++;
        }
        return 0;
    }
    const size_t first = out.size();
    std::vector<int32_t> new_idx(n, -1);
    for (size_t k = 0; k < roots.size(); k++) {
        auto nb = std::make_unique<SoftBody>();
        nb->name = b.name + " (debris)";
        nb->is_piece = true;
        nb->collision_radius = b.collision_radius;
        nb->ground_friction = b.ground_friction;
        nb->collision_group = b.collision_group;
        for (uint32_t i = 0; i < n; i++) {
            if (piece_of[find(i)] != (int32_t)k || piece_of[i] != (int32_t)k) continue;
            const Node& x = b.nodes[i];
            new_idx[i] = (int32_t)nb->add_node(x.p, x.mass, (uint16_t)(b.info[i].flags & (NF_GROUND | NF_CONTACTER)));
            nb->nodes.back().v = x.v;
            nb->info.back().friction = b.info[i].friction;
        }
        FemFrame& f = nb->fem;
        f.sections = sections;
        for (size_t ei = 0; ei < elems.size(); ei++) {
            const FrameElement& e = elems[ei];
            if (e.broken || new_idx[node[e.a]] < 0 || piece_of[node[e.a]] != (int32_t)k) continue;
            FrameElement c = e;
            c.a = f.add_node((uint32_t)new_idx[node[e.a]]);
            c.b = f.add_node((uint32_t)new_idx[node[e.b]]);
            f.q[c.a] = q[e.a], f.q[c.b] = q[e.b], f.w[c.a] = w[e.a], f.w[c.b] = w[e.b];
            f.elems.push_back(c);
            elems[ei].broken = true; // (gone from this body)
        }
        for (size_t i = 0; i < f.node.size(); i++) {
            const vec3 p = nb->nodes[f.node[i]].p;
            f.xd[i * 3] = p.x, f.xd[i * 3 + 1] = p.y, f.xd[i * 3 + 2] = p.z;
            f.node_inertia((uint32_t)i, *nb);
        }
        f.body_ = nb.get();
        f.analyse();
        f.tan_.assign(f.elems.size(), Tangent());
        f.ready_ = true;
        nb->compute_aabb();
        out.push_back(std::move(nb));
    }
    // in this body: the debris' nodes switched off (their indices stay: vehicles and meshes refer to nodes by number),
    // the hull triangles on them torn
    for (Triangle& t : b.tris)
        if (!t.two_sided && ((t.a < n && new_idx[t.a] >= 0) || (t.b < n && new_idx[t.b] >= 0) || (t.c < n && new_idx[t.c] >= 0))) t.torn = true;
    for (uint32_t i = 0; i < n; i++) {
        if (new_idx[i] < 0) continue;
        Node& x = b.nodes[i];
        x.inv_mass = 0;
        x.v = vec3(0);
        b.info[i].flags = 0;
        b.force[i] = vec3(0);
    }
    compact(b);
    b.contacter_count = -1;
    b.topo_log.overflow = true; // (the contact pairs: searched again)
    b.topo_changed = true;
    b.topo_version++;
    b.shk.version++;
    return (int)(out.size() - first);
}

void FemFrame::renumber(const SoftBody& b, const std::vector<uint32_t>& nidx) {
    for (uint32_t& v : node)
        if (v < nidx.size()) v = nidx[v];
    for (LooseTri& t : loose_tris)
        for (uint32_t& v : t.n)
            if (v < nidx.size()) v = nidx[v];
    for (FrameMount& m : mounts) {
        auto re = [&](uint32_t& v) {
            if (v < nidx.size()) v = nidx[v];
        };
        re(m.a), re(m.b), re(m.b2);
        for (int k = 0; k < 4; k++) re(m.an[k]), re(m.bn[k]);
    }
    slot_.assign(b.nodes.size(), -1);
    for (size_t i = 0; i < node.size(); i++)
        if (node[i] < slot_.size()) slot_[node[i]] = (int32_t)i;
    if (!ready_) return; // (finalize: at the first step)
    body_ = &b; // (the links and the pattern again; the members' state is kept)
    analyse();
    tan_.assign(elems.size(), Tangent());
}

void FemFrame::split_off(SoftBody& b, const std::vector<int>& part_of, const std::vector<uint32_t>& nidx, const std::vector<SoftBody*>& pieces) {
    { // (the loose fragments on the kept nodes follow them; the others are dropped)
        size_t m = 0;
        for (const LooseTri& t : loose_tris) {
            bool keep = true;
            for (uint32_t v : t.n) keep &= v < part_of.size() && part_of[v] < 0 && v < nidx.size();
            if (!keep) continue;
            LooseTri c = t;
            for (uint32_t& v : c.n) v = nidx[v];
            loose_tris[m++] = c;
        }
        loose_tris.resize(m);
    }
    if (elems.empty()) return;
    std::vector<uint8_t> moved(pieces.size(), 0);
    for (FrameElement& e : elems) {
        if (e.broken || node[e.a] >= part_of.size() || node[e.b] >= part_of.size()) continue;
        const int k = part_of[node[e.a]];
        if (k < 0 || k >= (int)pieces.size() || part_of[node[e.b]] != k) continue;
        FemFrame& f = pieces[k]->fem;
        if (f.sections.empty()) f.sections = sections;
        FrameElement c = e;
        const uint32_t oa = e.a, ob = e.b;
        c.a = f.add_node(nidx[node[oa]]);
        c.b = f.add_node(nidx[node[ob]]);
        f.q[c.a] = q[oa], f.q[c.b] = q[ob], f.w[c.a] = w[oa], f.w[c.b] = w[ob];
        f.elems.push_back(c);
        e.broken = true; // (gone from this body)
        moved[k] = 1;
    }
    for (size_t k = 0; k < pieces.size(); k++) {
        if (!moved[k]) continue;
        SoftBody& pb = *pieces[k];
        FemFrame& f = pb.fem;
        f.xd.assign(f.node.size() * 3, 0.0);
        f.inertia.assign(f.node.size(), 0.0f);
        f.torque.assign(f.node.size(), vec3(0));
        f.member_f.assign(f.node.size(), vec3(0));
        f.contact_n.assign(f.node.size(), vec3(0));
        f.contact_f.assign(f.node.size(), vec3(0));
        f.tri_press.clear();
        f.impulse.assign(f.node.size(), vec3(0));
        for (size_t i = 0; i < f.node.size(); i++) {
            const vec3 p = pb.nodes[f.node[i]].p;
            f.xd[i * 3] = p.x, f.xd[i * 3 + 1] = p.y, f.xd[i * 3 + 2] = p.z;
            f.node_inertia((uint32_t)i, pb);
        }
        f.body_ = &pb;
        f.analyse();
        f.tan_.assign(f.elems.size(), Tangent());
        f.ready_ = true;
    }
    // (a triangle with a corner gone to a piece is torn out: the frame's triangles stay with the kept body)
    for (FrameTri& t : tris)
        for (uint32_t v : t.n)
            if (!t.broken && node[v] < part_of.size() && part_of[node[v]] >= 0) {
                t.broken = true;
                if (t.coll >= 0 && t.coll < (int)b.tris.size()) b.tris[t.coll].torn = true;
            }
    // the kept frame: its nodes renumbered (those gone are unused now), then compacted; a mount with an end gone lets go
    for (FrameMount& m : mounts) {
        auto gone = [&](uint32_t v) { return v >= part_of.size() || part_of[v] >= 0; };
        bool off = gone(m.a) || gone(m.b) || (m.kind == MountKind::Hinge && gone(m.b2));
        for (int k = 0; k < m.na; k++) off |= gone(m.an[k]);
        for (int k = 0; k < m.nb; k++) off |= gone(m.bn[k]);
        if (off) {
            m.broken = true;
            continue;
        }
        m.a = nidx[m.a], m.b = nidx[m.b];
        if (m.kind == MountKind::Hinge) m.b2 = nidx[m.b2];
        for (int k = 0; k < m.na; k++) m.an[k] = nidx[m.an[k]];
        for (int k = 0; k < m.nb; k++) m.bn[k] = nidx[m.bn[k]];
    }
    for (uint32_t& v : node) v = v < part_of.size() && part_of[v] < 0 ? nidx[v] : 0;
    slot_.assign(b.nodes.size(), -1);
    compact(b);
}

// ------------------------------------------------------------------------------------------------ the rest
void FemFrame::rotate(const quat& r) {
    for (size_t i = 0; i < q.size(); i++) {
        q[i] = normalize(r * q[i]);
        w[i] = r.rotate(w[i]);
    }
}

void FemFrame::set_orientation(const quat& r) {
    for (size_t i = 0; i < q.size(); i++) q[i] = r, w[i] = vec3(0);
}

void FemFrame::place_exact(SoftBody& b, const std::vector<vec3>& rel, double tx, double ty, double tz) {
    if (xd.size() != node.size() * 3 || rel.size() != b.nodes.size()) return;
    for (size_t i = 0; i < node.size(); i++) {
        const uint32_t n = node[i];
        xd[i * 3] = (double)rel[n].x + tx, xd[i * 3 + 1] = (double)rel[n].y + ty, xd[i * 3 + 2] = (double)rel[n].z + tz;
        b.nodes[n].p = vec3((float)xd[i * 3], (float)xd[i * 3 + 1], (float)xd[i * 3 + 2]);
    }
}

void FemFrame::stop() {
    for (vec3& x : w) x = vec3(0);
}

int FemFrame::break_near(SoftBody& b, vec3 p, float r) {
    // each member within reach is torn where it passes closest: off a joint near its ends, else split there first
    int n = 0;
    const size_t count = elems.size();
    for (size_t ei = 0; ei < count; ei++) {
        const FrameElement& e = elems[ei];
        if (e.broken) continue;
        const vec3 a = b.nodes[node[e.a]].p, c = b.nodes[node[e.b]].p;
        const vec3 d = c - a;
        const float t = std::clamp(dot(p - a, d) / std::max(1e-12f, dot(d, d)), 0.0f, 1.0f);
        if (length2(a + d * t - p) > r * r) continue;
        n += cut(b, (uint32_t)ei, t);
    }
    // the triangles within reach: torn out
    for (size_t ti = 0; ti < tris.size(); ti++) {
        const FrameTri& t = tris[ti];
        if (t.broken) continue;
        vec3 bary;
        const vec3 c = closest_on_triangle(p, b.nodes[node[t.n[0]]].p, b.nodes[node[t.n[1]]].p, b.nodes[node[t.n[2]]].p, bary);
        if (length2(c - p) > r * r) continue;
        const vec3 mid = (b.nodes[node[t.n[0]]].p + b.nodes[node[t.n[1]]].p + b.nodes[node[t.n[2]]].p) / 3.0f;
        n += tear_tri(b, (uint32_t)ti, normalize_or(mid - p, vec3(1, 0, 0)));
    }
    finish_cuts(b, n);
    return n;
}

int FemFrame::component_of(uint32_t body_node) const {
    const int sl = slot(body_node);
    if (sl < 0 || (size_t)sl >= iperm_.size()) return -1;
    const int k = iperm_[sl];
    for (size_t c = 0; c < comp_range_.size(); c++)
        if (k >= comp_range_[c].first && k < comp_range_[c].second) return (int)c;
    return -1;
}

int FemFrame::release_latches(SoftBody& b) {
    std::vector<int> hinged;
    for (const FrameMount& m : mounts)
        if (!m.broken && m.kind == MountKind::Hinge) hinged.push_back(component_of(m.b));
    int n = 0;
    for (FrameMount& m : mounts) {
        if (m.broken || m.kind != MountKind::Point) continue;
        const int c = component_of(m.b);
        if (c < 0 || std::find(hinged.begin(), hinged.end(), c) == hinged.end()) continue;
        m.broken = true;
        mounts_broken++;
        b.stats.broken_beams++;
        n++;
    }
    if (n) b.wake();
    return n;
}

void FemFrame::mass_floor(SoftBody& b, uint32_t fn) {
    if (fn >= node.size()) return;
    const uint32_t v = node[fn];
    Node& n = b.nodes[v];
    if (n.inv_mass <= 0 && n.mass > 0) return; // (fixed)
    float share = 0;
    for (const FrameTri& t : tris)
        if (!t.broken && (t.n[0] == fn || t.n[1] == fn || t.n[2] == fn)) share += t.mass / 3.0f;
    for (const FrameElement& e : elems)
        if (!e.broken && (e.a == fn || e.b == fn)) share += 0.5f * e.mass;
    const float need = std::max(1e-3f, 0.5f * share);
    if (v < b.node_base_mass.size() && !b.shells.empty()) {
        if (b.node_base_mass[v] < need) n.mass += need - b.node_base_mass[v], b.node_base_mass[v] = need;
    } else if (n.mass < need) {
        n.mass = need;
    }
    n.inv_mass = (b.info[v].flags & NF_FIXED) ? 0.0f : 1.0f / n.mass;
}

uint32_t FemFrame::detach_tris(SoftBody& b, uint32_t fn, const std::vector<uint32_t>& moved, const std::vector<uint32_t>* members) {
    // the copy: the moved triangles' share of the node's mass, a third at least (a light copy on a small piece rang at
    // the step); the node keeps the rest - its members, loads, springs. A sheet over the triangles goes with them
    // (SoftBody::sheet_follow): the shares are of the node's own mass, its shells' thirds go with the shells
    const uint32_t v = node[fn];
    const bool sheet = !b.shells.empty() && v < b.node_base_mass.size();
    const float own = sheet ? b.node_base_mass[v] : b.nodes[v].mass;
    float share = 0;
    for (uint32_t u : moved) share += tris[u].mass / 3.0f;
    if (members)
        for (uint32_t ei : *members) share += 0.5f * elems[ei].mass;
    const bool fixed = b.nodes[v].inv_mass <= 0;
    // (no floor above the node's own: a refined shell's nodes weigh a gram or two, a gram's floor left one negative)
    share = fixed ? std::max(share, 1e-3f) : std::max(1e-6f, std::min(std::max(share, own / 3.0f), 0.7f * own));
    const uint32_t c = clone_node(b, v);
    b.info[c].flags &= (uint16_t)~NF_FIXED;
    set_mass(b.nodes[c], share, false);
    if (!fixed) set_mass(b.nodes[v], b.nodes[v].mass - share, false);
    if (sheet) {
        b.node_base_mass[c] = share;
        if (!fixed) b.node_base_mass[v] = std::max(0.0f, own - share);
    }
    const uint32_t fc = add_node(c);
    q[fc] = q[fn];
    w[fc] = w[fn];
    xd[fc * 3] = xd[fn * 3], xd[fc * 3 + 1] = xd[fn * 3 + 1], xd[fc * 3 + 2] = xd[fn * 3 + 2];
    if (members) // (the members' ends: on the copy, their rest frames in its frame the same - it has the node's turning)
        for (uint32_t ei : *members) {
            FrameElement& e = elems[ei];
            if (e.a == fn) e.a = fc;
            if (e.b == fn) e.b = fc;
        }
    for (uint32_t u : moved) {
        FrameTri& x = tris[u];
        for (uint32_t& k : x.n)
            if (k == fn) k = fc;
        if (x.coll >= 0 && x.coll < (int)b.tris.size()) {
            Triangle& ct = b.tris[x.coll];
            if (ct.a == v) ct.a = c;
            if (ct.b == v) ct.b = c;
            if (ct.c == v) ct.c = c;
        }
    }
    if (!b.shells.empty()) b.sheet_follow(v, c, moved);
    mass_floor(b, fn), mass_floor(b, fc);
    node_inertia(fn, b), node_inertia(fc, b);
    debris_check = true;
    ready_ = false;
    b.topo_changed = true, b.topo_version++;
    return fc;
}

void FemFrame::bind_sheet(SoftBody& b) {
    auto key = [](uint32_t x, uint32_t y, uint32_t z) {
        std::array<uint32_t, 3> k{x, y, z};
        std::sort(k.begin(), k.end());
        return k;
    };
    std::map<std::array<uint32_t, 3>, int> shell_of, tri_of;
    for (size_t si = 0; si < b.shells.size(); si++) shell_of.emplace(key(b.shells[si].n[0], b.shells[si].n[1], b.shells[si].n[2]), (int)si);
    const std::vector<char> of_shell = b.shell_tri_mask();
    for (size_t i = 0; i < b.tris.size(); i++)
        if (!of_shell[i]) tri_of.emplace(key(b.tris[i].a, b.tris[i].b, b.tris[i].c), (int)i);
    for (size_t u = 0; u < tris.size(); u++) {
        FrameTri& t = tris[u];
        const auto k = key(node[t.n[0]], node[t.n[1]], node[t.n[2]]);
        if (auto s = shell_of.find(k); s != shell_of.end()) {
            b.shells[s->second].host = (int32_t)u;
            t.coll = -1;
            continue;
        }
        const auto c = tri_of.find(k);
        t.coll = c != tri_of.end() ? c->second : -1;
    }
    b.sheet_on_plates = !b.shells.empty() && std::all_of(b.shells.begin(), b.shells.end(), [](const Shell& s) { return s.host >= 0; });
}

uint32_t FemFrame::split_tri(SoftBody& b, uint32_t ti, int e, uint32_t fm, float t) {
    const FrameTri s = tris[ti];
    const int e1 = (e + 1) % 3;
    FrameTri A = s, B = s;
    A.n[e1] = fm;
    B.n[e] = fm;
    // the rest shapes: the parent's in its own axes (the children's rest frame is the parent's), the new corner on the
    // edge, each about its centroid again; the plastic rest rotations of the new corner between its ends'
    auto lerp2 = [&](const float (&X)[3][2], int k) {
        const float x = X[e][k] + (X[e1][k] - X[e][k]) * t;
        return x;
    };
    const float xm[2] = {lerp2(s.X, 0), lerp2(s.X, 1)}, x0m[2] = {lerp2(s.X0, 0), lerp2(s.X0, 1)};
    for (int k = 0; k < 2; k++) A.X[e1][k] = B.X[e][k] = xm[k], A.X0[e1][k] = B.X0[e][k] = x0m[k];
    for (FrameTri* x : {&A, &B})
        for (float (*X)[2] : {x->X, x->X0}) {
            const float cx = (X[0][0] + X[1][0] + X[2][0]) / 3.0f, cy = (X[0][1] + X[1][1] + X[2][1]) / 3.0f;
            for (int i = 0; i < 3; i++) X[i][0] -= cx, X[i][1] -= cy;
        }
    // (the new node's frame is the element's rest frame there - the ends' views of it between them -, so its rest frame
    // in that node's: the identity for the triangle whose views made it, their turning against it for the other)
    const quat Ea = normalize(q[s.n[e]] * s.r0[e]), Eb = normalize(q[s.n[e1]] * s.r0[e1]);
    const quat rm = normalize(conj(q[fm]) * slerp(Ea, Eb, t));
    const vec3 thm = s.th0[e] + (s.th0[e1] - s.th0[e]) * t;
    A.r0[e1] = B.r0[e] = rm;
    A.th0[e1] = B.th0[e] = thm;
    A.area0 = s.area0 * t, B.area0 = s.area0 * (1 - t);
    A.mass = s.mass * t, B.mass = s.mass * (1 - t);
    A.level = B.level = (uint8_t)(s.level + 1);
    const vec2 pm = s.px[e] + (s.px[e1] - s.px[e]) * t;
    A.px[e1] = B.px[e] = pm;
    // the corners' masses: the edge's ends each give the new node their share of the half they left
    auto add_mass = [&](uint32_t fv, float dm) {
        Node& n = b.nodes[node[fv]];
        const bool fixed = n.inv_mass <= 0 && n.mass > 0;
        n.mass = std::max(1e-4f, n.mass + dm);
        n.inv_mass = fixed || (b.info[node[fv]].flags & NF_FIXED) ? 0.0f : 1.0f / n.mass;
        if (node[fv] < b.node_base_mass.size()) b.node_base_mass[node[fv]] = std::max(0.0f, b.node_base_mass[node[fv]] + dm);
    };
    add_mass(s.n[e], -(1 - t) * s.mass / 3.0f);
    add_mass(s.n[e1], -t * s.mass / 3.0f);
    add_mass(fm, s.mass / 3.0f);
    // the collision triangle: the parent's is A's, B a copy of it (its contacts inherited: topo_log)
    B.coll = -1;
    if (s.coll >= 0 && s.coll < (int)b.tris.size()) {
        Triangle tb = b.tris[s.coll];
        Triangle& ta = b.tris[s.coll];
        ta.a = node[A.n[0]], ta.b = node[A.n[1]], ta.c = node[A.n[2]];
        tb.a = node[B.n[0]], tb.b = node[B.n[1]], tb.c = node[B.n[2]];
        B.coll = (int32_t)b.tris.size();
        b.tris.push_back(tb);
        if (b.topo_log.tris.size() < 100000) b.topo_log.tris.push_back({(uint32_t)B.coll, (uint32_t)s.coll});
        else b.topo_log.overflow = true;
    }
    const uint32_t bi = (uint32_t)tris.size();
    tris[ti] = A;
    tris.push_back(B);
    if (b.topo_log.mids.size() < 100000) b.topo_log.mids.push_back({bi, ti}); // (its mid point's pairs: the parent's)
    else b.topo_log.overflow = true;
    tri_codes(ti), tri_codes(bi);
    return bi;
}

int FemFrame::refine_tri(SoftBody& b, uint32_t ti, int depth) {
    if (depth > 8 || ti >= tris.size() || tris[ti].broken || tris.size() + 2 > tri_cap_) return 0;
    auto len0 = [&](const FrameTri& t, int e) { return std::hypot(t.X0[(e + 1) % 3][0] - t.X0[e][0], t.X0[(e + 1) % 3][1] - t.X0[e][1]); };
    auto longest = [&](const FrameTri& t) {
        int e = 0;
        for (int f = 1; f < 3; f++)
            if (len0(t, f) > len0(t, e) * 1.0005f) e = f;
        return e;
    };
    auto can = [&](const FrameTri& t) {
        const ShellSection& s = shell_sections[t.section];
        return t.level < s.max_level + level_bonus_ && 0.5f * len0(t, longest(t)) >= s.min_edge;
    };
    if (!can(tris[ti])) return 0;
    const int e = longest(tris[ti]);
    const uint32_t a = tris[ti].n[e], c = tris[ti].n[(e + 1) % 3];
    // the neighbour across the edge (none: the shell's border; more: a seam of three, left whole)
    int tj = -1, across = 0;
    for (size_t u = 0; u < tris.size(); u++) {
        if (u == ti || tris[u].broken) continue;
        int hit = 0;
        for (uint32_t x : tris[u].n) hit += x == a || x == c;
        if (hit == 2) tj = (int)u, across++;
    }
    if (across > 1) return 0;
    int fe = -1;
    if (tj >= 0) {
        const FrameTri& tn = tris[tj];
        for (int f = 0; f < 3; f++)
            if ((tn.n[f] == a && tn.n[(f + 1) % 3] == c) || (tn.n[f] == c && tn.n[(f + 1) % 3] == a)) fe = f;
        if (fe < 0 || !can(tn)) return 0;
        if (len0(tn, fe) < len0(tn, longest(tn)) * 0.999f) {
            // (the neighbour is coarser: split it first, then a half of it has our edge as its longest)
            const int k = refine_tri(b, (uint32_t)tj, depth + 1);
            return k ? k + refine_tri(b, ti, depth + 1) : 0;
        }
    }
    // the new node: on a pattern's line the edge crosses near its middle (its halves not too thin), else the middle
    float t = 0.5f;
    if (split_side_) { // (a cut: on its plane)
        const float sa = (*split_side_)(b.nodes[node[a]].p), sc = (*split_side_)(b.nodes[node[c]].p);
        if ((sa < 0) != (sc < 0)) t = std::clamp(sa / (sa - sc), 0.15f, 0.85f);
    } else {
        const FrameTri& x = tris[ti];
        if (x.imp >= 0 && (size_t)x.imp < impacts.size()) {
            const float tc = pattern_cross(impacts[x.imp].pat, x.px[e], x.px[(e + 1) % 3], 0.25f, 0.75f);
            if (tc >= 0) t = tc;
        }
    }
    const uint32_t va = node[a], vc = node[c];
    const uint32_t m = clone_node(b, va);
    {
        Node& nm = b.nodes[m];
        const Node &na = b.nodes[va], &nc = b.nodes[vc];
        nm.p = na.p + (nc.p - na.p) * t;
        nm.v = na.v + (nc.v - na.v) * t;
        nm.mass = 0, nm.inv_mass = 0;
        const uint16_t fixed = b.info[va].flags & b.info[vc].flags & NF_FIXED; // (on a clamped border only)
        b.info[m].flags = (uint16_t)((b.info[va].flags & ~NF_FIXED) | fixed);
    }
    if (b.topo_log.nodes.size() < 100000) b.topo_log.nodes.push_back({m, vc}); // (between a, logged by clone_node, and c)
    else b.topo_log.overflow = true;
    const uint32_t fm = add_node(m);
    for (int k = 0; k < 3; k++) xd[fm * 3 + k] = xd[a * 3 + k] + (xd[c * 3 + k] - xd[a * 3 + k]) * t;
    w[fm] = w[a] + (w[c] - w[a]) * t;
    {
        const FrameTri& x = tris[ti];
        q[fm] = slerp(normalize(q[a] * x.r0[e]), normalize(q[c] * x.r0[(e + 1) % 3]), t);
    }
    const int32_t host = (int32_t)ti, nhost = tj;
    const uint32_t bi = split_tri(b, ti, e, fm, t);
    uint32_t bj = 0;
    if (tj >= 0) bj = split_tri(b, (uint32_t)tj, fe, fm, tris[tj].n[fe] == a ? t : 1 - t);
    // the sheet over them halved the same way (Shell::host: each half on its element)
    if (!b.shells.empty()) {
        b.sheet_bisect(host, va, vc, m, t, (int32_t)ti, (int32_t)bi);
        if (tj >= 0) {
            const bool a_first = tris[tj].n[fe] == a; // (its half at a: the one that kept its index, else the new one)
            b.sheet_bisect(nhost, va, vc, m, t, a_first ? tj : (int32_t)bj, a_first ? (int32_t)bj : tj);
        }
    }
    for (uint32_t v : {a, c, fm, tris[ti].n[(e + 2) % 3]}) mass_floor(b, v);
    if (tj >= 0)
        for (uint32_t v : tris[tj].n) mass_floor(b, v);
    for (uint32_t u : {ti, bi})
        for (uint32_t v : tris[u].n) node_inertia(v, b);
    if (tj >= 0)
        for (uint32_t v : tris[tj].n) node_inertia(v, b);
    if (tj >= 0)
        for (uint32_t v : tris[bj].n) node_inertia(v, b);
    tris_refined++;
    ready_ = false;
    b.topo_changed = true, b.topo_version++;
    return 1;
}

void FemFrame::tri_codes(uint32_t ti) {
    FrameTri& t = tris[ti];
    t.line = 0;
    t.es[0] = t.es[1] = t.es[2] = 64;
    if (t.imp < 0 || (size_t)t.imp >= impacts.size()) return;
    const ShellImpact& im = impacts[t.imp].pat;
    const ShellSection& s = shell_sections[t.section];
    const float zr = pattern_zone_radius(im);
    for (int e = 0; e < 3; e++) {
        const vec2 xa = t.px[e], xb = t.px[(e + 1) % 3], mid = (xa + xb) * 0.5f;
        const float len = length(xb - xa);
        if (len < 1e-6f || length(mid - im.c) > im.reach + len) continue;
        // (how near a line it runs: its ends and its middle within half its length of one - the same one for both ends,
        // else half as near; on the coarse shell an edge seldom lies on a line, the tears follow it in steps of the
        // edges nearest it), the edges near a line weaker, those of the zone far from them stronger
        const float reach = 0.5f * len + 1e-4f;
        const PatternLine la = pattern_nearest(im, xa, reach), lb = pattern_nearest(im, xb, reach), lm = pattern_nearest(im, mid, reach);
        float near = 0;
        if (la.d < reach && lb.d < reach && lm.d < reach) near = std::clamp(1.0f - std::max(la.d, std::max(lb.d, lm.d)) / reach, 0.0f, 1.0f) * (la.id == lb.id ? 1.0f : 0.5f);
        const float far = length(mid - im.c) < zr ? s.pattern_strong : 1.0f;
        const float f = far + (s.pattern_weak - far) * std::min(1.0f, 1.5f * near);
        if (near > 0.4f) t.line |= (uint8_t)(1u << e);
        t.es[e] = (uint8_t)std::clamp((int)std::lround(f * 64.0f), 4, 255);
        if (length(mid - im.c) < zr && pattern_cross(im, xa, xb, 0.02f, 0.98f) >= 0) t.line |= 8u;
    }
}

void FemFrame::note_hit(uint32_t a, uint32_t b, uint32_t c, vec3 bary, float speed, float size, double time) {
    if (!patterned_ || !(speed > hit_.speed) || speed < pattern_speed_) return;
    hit_.n[0] = a, hit_.n[1] = b, hit_.n[2] = c;
    hit_.bary = bary, hit_.speed = speed, hit_.size = size, hit_.time = time;
}

bool FemFrame::add_impact(SoftBody& b, uint32_t ti, vec3 p, float speed, float size, double time) {
    if (ti >= tris.size() || tris[ti].broken || !b.allow_break || b.rigid) return false;
    const ShellSection& s = shell_sections[tris[ti].section];
    if (s.pattern == ShellPattern::None || speed < s.pattern_speed) return false;
    for (const Impact& im : impacts) {
        const float d = length(p - im.o);
        if (d < std::max(im.pat.r, 0.1f)) return false;                       // (the same spot again)
        if (d < 3.0f * im.pat.r && time - im.pat.time < 0.25) return false;   // (the same body going on through)
    }
    if (impacts.size() >= 12) return false;
    auto pos = [&](uint32_t fn) { return b.nodes[node[fn]].p; };
    const FrameTri& t0 = tris[ti];
    const vec3 x0 = pos(t0.n[0]), x1 = pos(t0.n[1]), x2 = pos(t0.n[2]);
    const vec3 n = normalize_or(cross(x1 - x0, x2 - x0), vec3(0, 1, 0));
    Impact im;
    im.o = p;
    im.u = normalize_or((x1 - x0) - n * dot(x1 - x0, n), vec3(1, 0, 0));
    im.v = cross(n, im.u);
    uint32_t seed = 0x9e3779b9u * (uint32_t)(impacts.size() + 1);
    for (float f : {p.x, p.y, p.z}) {
        uint32_t u;
        std::memcpy(&u, &f, 4);
        seed = (seed ^ u) * 0x85ebca6bu;
    }
    // (wood: the grain along the plate's first edge there)
    im.pat = make_pattern(s.pattern, s.pattern_size, vec2(1, 0), vec2(0), speed, size, seed);
    im.pat.time = time;
    const int k = (int)impacts.size();
    impacts.push_back(im);
    const Impact& I = impacts.back();
    const float zr = pattern_zone_radius(I.pat);
    // its triangles: those round it on this plate (joined by their edges, within reach, turned less than 70 degrees
    // from it - not round a corner onto another face, not the other wall of a closed section); a triangle in an older
    // pattern only when inside this one's zone
    std::vector<std::vector<uint32_t>> fan(node.size());
    for (uint32_t u = 0; u < tris.size(); u++)
        if (!tris[u].broken)
            for (uint32_t v : tris[u].n) fan[v].push_back(u);
    std::vector<uint8_t> seen(tris.size(), 0);
    std::vector<uint32_t> queue{ti};
    seen[ti] = 1;
    for (size_t qi = 0; qi < queue.size(); qi++) {
        FrameTri& t = tris[queue[qi]];
        const vec3 y0 = pos(t.n[0]), y1 = pos(t.n[1]), y2 = pos(t.n[2]), cen = (y0 + y1 + y2) / 3.0f;
        const float lmax = std::max(length(y1 - y0), std::max(length(y2 - y1), length(y0 - y2)));
        if (length(cen - p) > I.pat.reach + lmax) continue;
        if (std::fabs(dot(normalize_or(cross(y1 - y0, y2 - y0), vec3(0)), n)) < 0.34f) continue;
        if (t.imp < 0 || length(cen - p) < zr) {
            t.imp = (int8_t)k;
            const vec3 ys[3] = {y0, y1, y2};
            for (int i = 0; i < 3; i++) t.px[i] = vec2(dot(ys[i] - p, I.u), dot(ys[i] - p, I.v));
            tri_codes(queue[qi]);
        }
        for (int i = 0; i < 3; i++) {
            const uint32_t va = t.n[i], vb = t.n[(i + 1) % 3];
            for (uint32_t u : fan[va])
                if (!seen[u] && (tris[u].n[0] == vb || tris[u].n[1] == vb || tris[u].n[2] == vb)) seen[u] = 1, queue.push_back(u);
        }
    }
    return true;
}

std::vector<uint32_t> FemFrame::refine_cut(SoftBody& b, const std::vector<uint32_t>& crossed, const std::function<float(vec3)>& side,
                                           const std::function<bool(uint32_t)>& crosses, int levels) {
    std::vector<uint32_t> now = crossed;
    split_side_ = &side, level_bonus_ = levels;
    for (int pass = 0; pass < levels + 2 && !now.empty(); pass++) {
        const size_t nt = tris.size();
        int made = 0;
        for (uint32_t ti : now)
            if (ti < tris.size() && !tris[ti].broken) made += refine_tri(b, ti);
        if (!made) break;
        // (the triangles crossing it now: the ones crossing it before or their halves - and the halves of their
        // neighbours halved with them)
        std::vector<uint32_t> next;
        for (uint32_t ti : now)
            if (ti < tris.size() && !tris[ti].broken && crosses(ti)) next.push_back(ti);
        for (uint32_t ti = (uint32_t)nt; ti < tris.size(); ti++)
            if (!tris[ti].broken && crosses(ti)) next.push_back(ti);
        std::sort(next.begin(), next.end());
        next.erase(std::unique(next.begin(), next.end()), next.end());
        now.swap(next);
    }
    split_side_ = nullptr, level_bonus_ = 0;
    // (and any other triangle crossing it: a neighbour halved there)
    std::vector<uint32_t> out;
    for (uint32_t ti = 0; ti < tris.size(); ti++)
        if (!tris[ti].broken && crosses(ti)) out.push_back(ti);
    return out;
}

int FemFrame::part_tris(SoftBody& b, const std::vector<uint32_t>& crossed, const std::function<float(vec3)>& side) {
    auto mid = [&](const FrameTri& t) { return (b.nodes[node[t.n[0]]].p + b.nodes[node[t.n[1]]].p + b.nodes[node[t.n[2]]].p) / 3.0f; };
    std::vector<uint32_t> corners;
    for (uint32_t ti : crossed)
        if (ti < tris.size() && !tris[ti].broken)
            for (uint32_t k : tris[ti].n)
                if (std::find(corners.begin(), corners.end(), k) == corners.end()) corners.push_back(k);
    int n = 0;
    std::vector<uint32_t> copies;
    // (the fans as they were round the corners and round their fans' other nodes: parted by side, a corner parts the
    // edges from it between triangles on the two sides too, and those edges' far ends may be left joining the two by
    // their node alone)
    std::vector<uint32_t> near = corners;
    for (size_t u = 0; u < tris.size(); u++) {
        const FrameTri& t = tris[u];
        if (t.broken) continue;
        bool touches = false;
        for (uint32_t v : t.n) touches |= std::find(corners.begin(), corners.end(), v) != corners.end();
        if (touches)
            for (uint32_t v : t.n)
                if (std::find(near.begin(), near.end(), v) == near.end()) near.push_back(v);
    }
    std::vector<std::vector<uint32_t>> fan0(near.size());
    std::vector<std::vector<int>> g0(near.size());
    for (size_t q = 0; q < near.size(); q++) fan_groups(near[q], {}, fan0[q], g0[q]);
    std::vector<int> copy_of;
    for (uint32_t fn : corners) {
        std::vector<uint32_t> pos;
        bool neg = false;
        for (size_t u = 0; u < tris.size(); u++) {
            const FrameTri& t = tris[u];
            if (t.broken || (t.n[0] != fn && t.n[1] != fn && t.n[2] != fn)) continue;
            if (side(mid(t)) >= 0) pos.push_back((uint32_t)u);
            else neg = true;
        }
        if (pos.empty() || !neg) continue;
        // (the members at the corner on the positive side go with them: left on the node, they joined the two sides)
        std::vector<uint32_t> pm;
        for (uint32_t ei = 0; ei < (uint32_t)elems.size(); ei++) {
            const FrameElement& e = elems[ei];
            if (e.broken || (e.a != fn && e.b != fn) || e.a == e.b) continue;
            if (side((b.nodes[node[e.a]].p + b.nodes[node[e.b]].p) * 0.5f) >= 0) pm.push_back(ei);
        }
        copies.push_back(detach_tris(b, fn, pos, &pm));
        copy_of.push_back((int)(std::find(corners.begin(), corners.end(), fn) - corners.begin()));
        n++;
    }
    // (and no triangle left on the rest by one corner where the cut split a group: split_refine)
    for (size_t q = 0; q < near.size(); q++) n += split_refine(b, near[q], fan0[q], g0[q], {});
    for (size_t q = 0; q < copies.size(); q++) n += split_refine(b, copies[q], fan0[copy_of[q]], g0[copy_of[q]], {});
    for (uint32_t ti : crossed)
        if (ti < tris.size()) tris[ti].tears = (uint8_t)std::min(3, tris[ti].tears + 1);
    tris_torn += (int)crossed.size();
    b.stats.broken_beams += n;
    return n;
}

int FemFrame::tear_tri(SoftBody& b, uint32_t ti, vec3 pull) {
    PROFILE_ACCUM("Frame tear tri");
    if (ti >= tris.size() || tris[ti].broken || tris[ti].tears >= 3) return 0;
    const FrameTri& t0 = tris[ti];
    const vec3 P[3] = {b.nodes[node[t0.n[0]]].p, b.nodes[node[t0.n[1]]].p, b.nodes[node[t0.n[2]]].p};
    // the edges in the order to try: the one most across the pull first (a crack runs across the stretch)
    float score[3];
    if (length2(pull) > 1e-12f) {
        const vec3 nn = normalize_or(cross(P[1] - P[0], P[2] - P[0]), vec3(0, 1, 0));
        const vec3 d = normalize_or(pull - nn * dot(pull, nn), vec3(0));
        for (int k = 0; k < 3; k++) score[k] = std::fabs(dot(normalize_or(P[(k + 1) % 3] - P[k], vec3(0)), d));
    } else {
        // (the plastic stretch F = [dX][dX0]^-1 in the authored plane: C = F^T F, its largest eigenvector v)
        const double a0x = t0.X0[1][0] - t0.X0[0][0], a0y = t0.X0[1][1] - t0.X0[0][1], b0x = t0.X0[2][0] - t0.X0[0][0], b0y = t0.X0[2][1] - t0.X0[0][1];
        const double ax = t0.X[1][0] - t0.X[0][0], ay = t0.X[1][1] - t0.X[0][1], bx = t0.X[2][0] - t0.X[0][0], by = t0.X[2][1] - t0.X[0][1];
        const double det = a0x * b0y - b0x * a0y;
        double vx = 1, vy = 0;
        if (std::fabs(det) > 1e-12) {
            const double i00 = b0y / det, i01 = -b0x / det, i10 = -a0y / det, i11 = a0x / det;
            const double F00 = ax * i00 + bx * i10, F01 = ax * i01 + bx * i11, F10 = ay * i00 + by * i10, F11 = ay * i01 + by * i11;
            const double C00 = F00 * F00 + F10 * F10, C01 = F00 * F01 + F10 * F11, C11 = F01 * F01 + F11 * F11;
            const double lmax = 0.5 * (C00 + C11) + std::sqrt(0.25 * (C00 - C11) * (C00 - C11) + C01 * C01);
            if (std::fabs(C01) > 1e-12) vx = C01, vy = lmax - C00;
            else if (C11 > C00) vx = 0, vy = 1;
            const double l = std::sqrt(vx * vx + vy * vy);
            vx /= l, vy /= l;
        }
        for (int k = 0; k < 3; k++) {
            const double ex = t0.X0[(k + 1) % 3][0] - t0.X0[k][0], ey = t0.X0[(k + 1) % 3][1] - t0.X0[k][1], el = std::sqrt(ex * ex + ey * ey);
            score[k] = el > 1e-9 ? (float)std::fabs((ex * vx + ey * vy) / el) : 1.0f;
        }
    }
    // (a pattern's weak edges - near its lines - first whatever the stretch, its strong ones - in the zone between them -
    // last: the cracks run along the lines)
    auto band = [&](int k) { return t0.es[k] < 56 ? 0 : t0.es[k] <= 72 ? 1 : 2; };
    int order[3] = {0, 1, 2};
    std::sort(order, order + 3, [&](int x, int y) { return band(x) != band(y) ? band(x) < band(y) : score[x] < score[y]; });
    // The edge parts all along, both its ends duplicated: an end inside the sheet (its fan still one round it with the
    // edge cut) takes the crack on along its edge straightest on from the torn one - shared by two triangles, not the
    // two parted ones' third corners - and that edge's far end is the crack's tip, shared by both sides. A node is
    // split only by its fan's groups, every group of triangles joined round it by their edges a node of its own: no
    // triangle is left on the rest by one corner (the crack's two sides hung on a node and swung there, a hole beside).
    constexpr uint32_t kNone = ~0u;
    std::vector<uint32_t> fan;
    std::vector<int> grp;
    auto pos = [&](uint32_t fn) { return b.nodes[node[fn]].p; };
    auto onward = [&](uint32_t v, uint32_t from, uint32_t skip1, uint32_t skip2) -> uint32_t {
        fan_groups(v, {}, fan, grp);
        const vec3 p = pos(v), dir = normalize_or(p - pos(from), vec3(0));
        float best = -2.0f;
        uint32_t next = kNone;
        for (uint32_t u : fan)
            for (uint32_t e : tris[u].n) {
                if (e == v || e == from || e == skip1 || e == skip2) continue;
                int users = 0;
                float code = 64; // (its pattern code, the weaker side's: along a line the crack runs on along it)
                for (uint32_t w2 : fan) {
                    const FrameTri& x = tris[w2];
                    if (x.n[0] != e && x.n[1] != e && x.n[2] != e) continue;
                    users++;
                    for (int g = 0; g < 3; g++)
                        if ((x.n[g] == v && x.n[(g + 1) % 3] == e) || (x.n[g] == e && x.n[(g + 1) % 3] == v)) code = std::min(code, (float)x.es[g]);
                }
                if (users != 2) continue;
                const float c = dot(normalize_or(pos(e) - p, vec3(0)), dir) + std::clamp(1.5f * (64.0f - code) / 64.0f, -0.5f, 0.6f);
                if (c > best) best = c, next = e;
            }
        return next;
    };
    // (a piece smaller than the section's min_piece is not cut off - the edge stays, another may part, or it waits -
    // unless the triangle is far past its tear: then it goes as a shard rather than stretch on)
    const ShellSection& sec = shell_sections[t0.section];
    const float past = sec.elongation * (1.0f + 0.5f * t0.tears) * tear_band(t0.level) * (float)std::min(t0.es[0], std::min(t0.es[1], t0.es[2])) * (1.0f / 64.0f);
    const bool shard_ok = t0.dmg > 2.0f * past;
    bool waits = false;
    int weak_held = 0;
    for (int k : order) {
        const uint32_t a = t0.n[k], c = t0.n[(k + 1) % 3], t3 = t0.n[(k + 2) % 3];
        // (the triangle across: none - the sheet's edge already, nothing to tear there; more - a seam of three, left)
        int tj = -1, across = 0;
        for (size_t u = 0; u < tris.size(); u++) {
            if (u == ti || tris[u].broken) continue;
            int hit = 0;
            for (uint32_t x : tris[u].n) hit += x == a || x == c;
            if (hit == 2) tj = (int)u, across++;
        }
        if (across != 1) continue;
        weak_held += t0.es[k] < 56; // (a weak edge still joined: the tear had it to part)
        uint32_t j3 = kNone;
        for (uint32_t x : tris[tj].n)
            if (x != a && x != c) j3 = x;
        // (an end parts when the two triangles fall in different groups round it with the cut: one inside the sheet
        // needs the crack on along another edge for that)
        auto parted = [&](uint32_t v, const std::vector<uint32_t>& cut) {
            fan_groups(v, cut, fan, grp);
            int gi = -1, gj = -2;
            for (size_t i = 0; i < fan.size(); i++) {
                if (fan[i] == ti) gi = grp[i];
                if ((int)fan[i] == tj) gj = grp[i];
            }
            return gi != gj;
        };
        std::vector<uint32_t> cut_a{c}, cut_c{a};
        uint32_t xa = kNone, xc = kNone;
        bool ok = true;
        if (!parted(a, cut_a)) {
            xa = onward(a, c, t3, j3);
            if (xa == kNone) ok = false;
            else cut_a.push_back(xa), ok = parted(a, cut_a);
        }
        if (ok && !parted(c, cut_c)) {
            xc = onward(c, a, t3, j3);
            if (xc == kNone) ok = false;
            else cut_c.push_back(xc), ok = parted(c, cut_c);
        }
        if (!ok) continue; // (it would part at one end alone: another edge, or none)
        if (!shard_ok && cuts_off_small(ti, (uint32_t)tj, a, c, xa, xc)) { // (a shard: another edge, or later)
            waits = true;
            continue;
        }
        // the nodes' fans as they were (a node where the authored mesh joins triangles by a corner alone - a panel's
        // corner on another's - stays so: only the groups the crack splits part)
        std::vector<uint32_t> fan0[4];
        std::vector<int> g0[4];
        const uint32_t at[4] = {a, c, xa, xc};
        for (int q = 0; q < 4; q++)
            if (at[q] != kNone) fan_groups(at[q], {}, fan0[q], g0[q]);
        split_refine(b, a, fan0[0], g0[0], cut_a);
        split_refine(b, c, fan0[1], g0[1], cut_c);
        // (the tips: on the sheet's edge or on another crack they part as well)
        if (xa != kNone) split_refine(b, xa, fan0[2], g0[2], {});
        if (xc != kNone) split_refine(b, xc, fan0[3], g0[3], {});
        FrameTri& t = tris[ti];
        if (weak_held) tears_by_line++, tears_on_line += t.es[k] < 56;
        t.tears++;
        tris_torn++;
        b.stats.broken_beams++;
        debris_check = true;
        ready_ = false;
        b.topo_changed = true, b.topo_version++;
        return 1;
    }
    if (!waits) tris[ti].tears = 3; // (free all round, or no edge of it parts whole)
    else tris[ti].wait = 64;        // (it tries again a little later: the stretch grows, or its neighbours tear)
    return 0;
}

// The triangles round frame node fn, grouped by their edges across it: two of them are joined where they share an edge
// fn-x with x not in `cut`. The fan, each one's group; the groups' count.
bool FemFrame::cuts_off_small(uint32_t ti, uint32_t tj, uint32_t a, uint32_t c, uint32_t xa, uint32_t xc) const {
    const float limit = shell_sections[tris[ti].section].min_piece;
    if (!(limit > 0)) return false;
    constexpr uint32_t kNone = ~0u;
    auto is_cut = [&](uint32_t p, uint32_t q) {
        auto same = [&](uint32_t x, uint32_t y) { return (p == x && q == y) || (p == y && q == x); };
        return same(a, c) || (xa != kNone && same(a, xa)) || (xc != kNone && same(c, xc));
    };
    std::vector<std::vector<uint32_t>> fan(node.size());
    for (uint32_t u = 0; u < tris.size(); u++)
        if (!tris[u].broken)
            for (uint32_t v : tris[u].n) fan[v].push_back(u);
    // (the side of `from` with the cut: big enough, or joined to `to` round the cut - nothing cut off)
    auto big = [&](uint32_t from, uint32_t to) {
        std::vector<uint32_t> seen{from}, stack{from};
        double area = tris[from].area0;
        while (!stack.empty()) {
            const FrameTri& t = tris[stack.back()];
            stack.pop_back();
            for (int e = 0; e < 3; e++) {
                const uint32_t p = t.n[e], q = t.n[(e + 1) % 3];
                if (is_cut(p, q)) continue;
                for (uint32_t u : fan[p]) {
                    const FrameTri& x = tris[u];
                    if (x.n[0] != q && x.n[1] != q && x.n[2] != q) continue;
                    if (std::find(seen.begin(), seen.end(), u) != seen.end()) continue;
                    if (u == to) return true;
                    area += x.area0;
                    if (area >= limit) return true;
                    seen.push_back(u), stack.push_back(u);
                }
            }
        }
        return area >= limit;
    };
    return !big(ti, tj) || !big(tj, ti);
}

int FemFrame::fan_groups(uint32_t fn, const std::vector<uint32_t>& cut, std::vector<uint32_t>& fan, std::vector<int>& group) const {
    fan.clear();
    for (size_t u = 0; u < tris.size(); u++)
        if (!tris[u].broken && (tris[u].n[0] == fn || tris[u].n[1] == fn || tris[u].n[2] == fn)) fan.push_back((uint32_t)u);
    group.assign(fan.size(), -1);
    auto shares = [&](uint32_t u, uint32_t w) { // (an edge fn-x of both, x not cut)
        for (uint32_t x : tris[u].n) {
            if (x == fn || std::find(cut.begin(), cut.end(), x) != cut.end()) continue;
            if (tris[w].n[0] == x || tris[w].n[1] == x || tris[w].n[2] == x) return true;
        }
        return false;
    };
    int ng = 0;
    std::vector<size_t> stack;
    for (size_t i = 0; i < fan.size(); i++) {
        if (group[i] >= 0) continue;
        group[i] = ng;
        stack.assign(1, i);
        while (!stack.empty()) {
            const size_t x = stack.back();
            stack.pop_back();
            for (size_t y = 0; y < fan.size(); y++)
                if (group[y] < 0 && shares(fan[x], fan[y])) group[y] = ng, stack.push_back(y);
        }
        ng++;
    }
    return ng;
}

// fn's triangles that were one group round it (fan0, g0: before) and are several now (by their edges, `cut` taken as
// parted): the largest of each keeps the node, every other one takes a copy. The copies made.
int FemFrame::split_refine(SoftBody& b, uint32_t fn, const std::vector<uint32_t>& fan0, const std::vector<int>& g0, const std::vector<uint32_t>& cut) {
    std::vector<uint32_t> fan;
    std::vector<int> grp;
    const int ng = fan_groups(fn, cut, fan, grp);
    if (ng < 2) return 0;
    int made = 0;
    const int n0 = g0.empty() ? 0 : *std::max_element(g0.begin(), g0.end()) + 1;
    for (int q = 0; q < n0; q++) {
        std::vector<int> cnt(ng, 0);
        for (size_t i = 0; i < fan.size(); i++) {
            const auto it = std::find(fan0.begin(), fan0.end(), fan[i]);
            if (it != fan0.end() && g0[it - fan0.begin()] == q) cnt[grp[i]]++;
        }
        int parts = 0, keep = 0;
        for (int g = 0; g < ng; g++)
            if (cnt[g] > 0) {
                parts++;
                if (cnt[g] > cnt[keep] || cnt[keep] == 0) keep = g;
            }
        if (parts < 2) continue;
        for (int g = 0; g < ng; g++) {
            if (g == keep || cnt[g] == 0) continue;
            std::vector<uint32_t> moved;
            for (size_t i = 0; i < fan.size(); i++) {
                const auto it = std::find(fan0.begin(), fan0.end(), fan[i]);
                if (grp[i] == g && it != fan0.end() && g0[it - fan0.begin()] == q) moved.push_back(fan[i]);
            }
            detach_tris(b, fn, moved);
            made++;
        }
    }
    return made;
}

int FemFrame::vertex_hinges() const {
    std::vector<char> seen(node.size(), 0);
    std::vector<uint32_t> fan;
    std::vector<int> grp;
    int n = 0;
    static const bool dbg = getenv("BL_HINGEDBG") != nullptr; // (diagnostics: each such node, its groups' sections)
    for (const FrameTri& t : tris)
        if (!t.broken)
            for (uint32_t v : t.n)
                if (v < seen.size() && !seen[v]) {
                    seen[v] = 1;
                    const int ng = fan_groups(v, {}, fan, grp);
                    n += ng > 1;
                    if (dbg && ng > 1 && body_) {
                        printf("hinge: frame node %u (body %u at %.3f %.3f %.3f): %d groups:", v, node[v], body_->nodes[node[v]].p.x, body_->nodes[node[v]].p.y,
                               body_->nodes[node[v]].p.z, ng);
                        for (size_t i = 0; i < fan.size(); i++) printf(" %d/s%d/t%d", grp[i], tris[fan[i]].section, tris[fan[i]].tears);
                        printf("\n");
                    }
                }
    return n;
}

int FemFrame::cut(SoftBody& b, uint32_t ei, float t) {
    if (ei >= elems.size() || elems[ei].broken) return 0;
    const FrameElement& e = elems[ei];
    const float L = length(b.nodes[node[e.b]].p - b.nodes[node[e.a]].p);
    if (t * L < 0.06f || t < 0.12f) return tear(b, ei, 0);
    if ((1 - t) * L < 0.06f || t > 0.88f) return tear(b, ei, 1);
    return split(b, ei, t) >= 0 ? tear(b, ei, 1) : 0;
}

void FemFrame::cap_loose(SoftBody& b, float cap) const {
    if (loose_tris.empty()) return;
    // (against the body's mean velocity, but only a node faster than the body's speed and the cap: one lying on the
    // ground was dragged after a car faster than the cap; against its own fragment's mean a shard ran away whole)
    const vec3 vm = b.average_velocity();
    const float top = length(vm) + cap;
    for (const LooseTri& t : loose_tris)
        for (uint32_t v : t.n) {
            if (v >= b.nodes.size()) continue;
            Node& x = b.nodes[v];
            if (length2(x.v) <= top * top) continue;
            const vec3 d = x.v - vm;
            const float l2 = length2(d);
            if (l2 > cap * cap) x.v = vm + d * (cap / std::sqrt(l2));
        }
}

void FemFrame::finish_cuts(SoftBody& b, int tears) {
    if (!ready_) {
        torque.resize(node.size(), vec3(0));
        member_f.resize(node.size(), vec3(0));
        body_ = &b;
        analyse();
        tan_.assign(elems.size(), Tangent());
        tri_tan_.assign(tris.size(), TriTan());
        ready_ = true;
    }
    if (tears) b.topo_changed = true, b.topo_version++, b.shk.version++;
}

int FemFrame::repair() {
    int n = 0;
    for (size_t i = 0; i < q.size(); i++) {
        if (!std::isfinite(q[i].x + q[i].y + q[i].z + q[i].w)) q[i] = quat(), n++;
        if (!std::isfinite(w[i].x + w[i].y + w[i].z)) w[i] = vec3(0), n++;
    }
    return n;
}

double FemFrame::tri_energy(double* parts) const {
    double U = 0;
    if (parts) parts[0] = parts[1] = parts[2] = 0;
    for (size_t ti = 0; ti < tris.size(); ti++) {
        const FrameTri& t = tris[ti];
        TriKin k;
        if (t.broken || !tri_kinematics(*this, t, k)) continue;
        double K[18][18];
        tri_local_k(t.X, shell_sections[t.section], K);
        // the parts: the dofs of the membrane (u, v), of the bending (w, theta_x, theta_y); the drilling the rest
        auto quad = [&](int mask) {
            double e = 0;
            for (int i = 0; i < 18; i++)
                for (int j = 0; j < 18; j++)
                    if ((mask >> (i % 6) & 1) && (mask >> (j % 6) & 1)) e += k.d[i] * K[i][j] * k.d[j];
            return 0.5 * e;
        };
        const double all = quad(63), mem = quad(3), bend = quad(28);
        U += all;
        if (parts) parts[0] += mem, parts[1] += bend, parts[2] += all - mem - bend;
    }
    return U;
}

double FemFrame::strain_energy(const SoftBody& b, double* parts) const {
    double U = 0;
    if (parts) parts[0] = parts[1] = parts[2] = 0;
    for (const FrameElement& e : elems) {
        if (e.broken) continue;
        Kin k;
        if (!kinematics(*this, e, k)) continue;
        const Stiff st = stiffness(sections[e.section], e);
        const double d = k.dL - e.up, tw = k.rb.x - k.ra.x - e.tp;
        const double ay = k.ra.y - e.pa.x, by = k.rb.y - e.pb.x, az = k.ra.z - e.pa.y, bz = k.rb.z - e.pb.y;
        const double ua = 0.5 * st.ka * d * d, ut = 0.5 * st.kt * tw * tw;
        const double ub = 0.5 * (st.cy[0] * ay * ay + 2 * st.cy[1] * ay * by + st.cy[2] * by * by) +
                          0.5 * (st.cz[0] * az * az + 2 * st.cz[1] * az * bz + st.cz[2] * bz * bz);
        U += ua + ut + ub;
        if (parts) parts[0] += ua, parts[1] += ut, parts[2] += ub;
    }
    return U;
}

} // namespace bl::phys
