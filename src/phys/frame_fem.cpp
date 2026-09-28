#include "phys/frame_fem.h"
#include <string>

#include "phys/softbody.h"
#include "core/profiler.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace bl::phys {

// ------------------------------------------------------------------------------------------------ materials, sections
static const FrameMaterial kMaterials[] = {
    // name, E, G, density, yield, elongation at fracture
    {"Steel", 2.10e11f, 8.1e10f, 7850.0f, 3.5e8f, 0.20f},      // mild structural steel (S355)
    {"Chromoly", 2.05e11f, 8.0e10f, 7850.0f, 4.6e8f, 0.15f},   // 4130, normalized: roll cages, race frames
    {"Aluminium", 6.9e10f, 2.6e10f, 2700.0f, 2.75e8f, 0.10f},  // 6061-T6
    {"Titanium", 1.14e11f, 4.4e10f, 4430.0f, 8.8e8f, 0.10f},   // Ti-6Al-4V
    {"Carbon", 1.2e11f, 2.5e10f, 1600.0f, 6.0e8f, 0.015f},     // CFRP tube: strong, light and brittle
    {"Wood", 1.1e10f, 7.0e8f, 500.0f, 4.0e7f, 0.02f},          // pine along the grain
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

uint16_t FemFrame::add_section(const FrameSection& s) {
    sections.push_back(s);
    return (uint16_t)(sections.size() - 1);
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
    for (size_t i = 0; i < n; i++) node_inertia((uint32_t)i, b);
    analyse();
    tan_.assign(elems.size(), Tangent());
    ready_ = true;
}

// ------------------------------------------------------------------------------------------------ the sparse pattern
void FemFrame::analyse() {
    const int n = (int)node.size();
    std::vector<std::vector<int>> adj(n);
    auto link = [&](int a, int b) {
        if (a == b) return;
        if (std::find(adj[a].begin(), adj[a].end(), b) == adj[a].end()) adj[a].push_back(b);
        if (std::find(adj[b].begin(), adj[b].end(), a) == adj[b].end()) adj[b].push_back(a);
    };
    for (const FrameElement& e : elems) link((int)e.a, (int)e.b);
    // the body's springs on frame nodes
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
            links_.push_back({(uint32_t)k, fa, fb, shock_of[k], -1, 0});
            if (fa >= 0 && fb >= 0) link(fa, fb);
        }
    }
    // minimum degree ordering on the elimination graph; a column's rows are the node's neighbours when it goes
    perm_.assign(n, 0);
    iperm_.assign(n, 0);
    std::vector<char> done(n, 0);
    std::vector<std::vector<int>> cols(n);
    for (int k = 0; k < n; k++) {
        int v = -1;
        for (int i = 0; i < n; i++)
            if (!done[i] && (v < 0 || adj[i].size() < adj[v].size())) v = i;
        perm_[k] = v;
        iperm_[v] = k;
        done[v] = 1;
        cols[k] = adj[v];
        for (size_t x = 0; x < adj[v].size(); x++)
            for (size_t y = x + 1; y < adj[v].size(); y++) link(adj[v][x], adj[v][y]);
        for (int u : adj[v]) adj[u].erase(std::find(adj[u].begin(), adj[u].end(), v));
        adj[v].clear();
    }
    col_ptr_.assign(n + 1, 0);
    row_.clear();
    for (int k = 0; k < n; k++) {
        std::vector<int> r;
        for (int u : cols[k]) r.push_back(iperm_[u]);
        std::sort(r.begin(), r.end());
        row_.insert(row_.end(), r.begin(), r.end());
        col_ptr_[k + 1] = (int)row_.size();
    }
    auto find_block = [&](int col, int row) {
        for (int p = col_ptr_[col]; p < col_ptr_[col + 1]; p++)
            if (row_[p] == row) return p;
        return -1;
    };
    upd_ptr_.assign(n + 1, 0);
    upd_.clear();
    for (int k = 0; k < n; k++) {
        for (int pi = col_ptr_[k]; pi < col_ptr_[k + 1]; pi++)
            for (int pj = pi; pj < col_ptr_[k + 1]; pj++) {
                Update u;
                u.pi = pi, u.pj = pj;
                u.target = pi == pj ? -(row_[pi] + 1) : find_block(row_[pi], row_[pj]);
                upd_.push_back(u);
            }
        upd_ptr_[k + 1] = (int)upd_.size();
    }
    elem_block_.assign(elems.size(), -1);
    elem_swap_.assign(elems.size(), 0);
    for (size_t i = 0; i < elems.size(); i++) {
        const int ka = iperm_[elems[i].a], kb = iperm_[elems[i].b];
        if (ka == kb) continue;
        elem_block_[i] = find_block(std::min(ka, kb), std::max(ka, kb));
        elem_swap_[i] = ka > kb;
    }
    for (Link& l : links_) {
        if (l.fa < 0 || l.fb < 0 || l.fa == l.fb) continue;
        const int ka = iperm_[l.fa], kb = iperm_[l.fb];
        l.block = find_block(std::min(ka, kb), std::max(ka, kb));
        l.swap = ka > kb;
    }
    diag_.assign((size_t)n * 36, 0.0);
    off_.assign(row_.size() * 36, 0.0);
    rhs_.assign((size_t)n * 6, 0.0);
    dinv_.assign((size_t)n * 6, 0.0);
    diagA_.clear(), offA_.clear(), rhsA_.clear();
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

} // namespace

void FemFrame::compute_forces(SoftBody& b) {
    if (!ready_) finalize(b);
    if (tan_.size() != elems.size()) tan_.assign(elems.size(), Tangent());
    // a node that is not where its double position rounds to was moved by something else (a reset, a repair)
    xd.resize(node.size() * 3);
    for (size_t i = 0; i < node.size(); i++) {
        const vec3 p = b.nodes[node[i]].p;
        double* x = &xd[i * 3];
        if ((float)x[0] != p.x || (float)x[1] != p.y || (float)x[2] != p.z) x[0] = p.x, x[1] = p.y, x[2] = p.z;
    }
    vec3* F = b.force.data();
    member_f.assign(node.size(), vec3(0));
    static const bool dbg = getenv("BL_FRAMEDBG") != nullptr;
    for (size_t ei = 0; ei < elems.size(); ei++) {
        FrameElement& e = elems[ei];
        Tangent& t = tan_[ei];
        t.on = false;
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
                for (const Event& ev : events_) dup |= ev.elem == ei;
                if (!dup) events_.push_back({(uint32_t)ei, (uint8_t)kind, 0.5f});
            }
        }
        // nodal forces (-B^T g) and moments: the shear of the end moments acts across the member at its current length,
        // so forces and moments balance exactly
        const float Sy = Ma.x + Mb.x, Sz = Ma.y + Mb.y;
        const vec3 fa = k.e1 * N + k.e3 * (Sy / k.L) - k.e2 * (Sz / k.L);
        F[node[e.a]] += fa;
        F[node[e.b]] -= fa;
        member_f[e.a] += fa;
        member_f[e.b] -= fa;
        torque[e.a] += k.e1 * T - k.e2 * Ma.x - k.e3 * Ma.y;
        torque[e.b] += k.e1 * (-T) - k.e2 * Mb.x - k.e3 * Mb.y;
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
        t.ma = Ma, t.mb = Mb, t.N = N, t.T = T;
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
}

void FemFrame::solve(SoftBody& b, float h, float step, float theta, float dissipation) {
    if (!ready_ || node.empty()) return;
    const int n = (int)node.size();
    impulse.resize(node.size(), vec3(0));
    dissipation = std::clamp(dissipation, 0.0f, 1.0f);
    theta = std::clamp(theta, 0.25f + 0.5f * dissipation, 1.0f); // (unconditionally stable)
    std::vector<char>& fixed = fixed_;
    fixed.assign(n, 0);
    const vec3* F = b.force.data();
    // (a sub-cycled body's contacts with other bodies, held over its short steps: World::step_island)
    const vec3* E = h > step && b.ext_force.size() == b.nodes.size() ? b.ext_force.data() : nullptr;
    bool any_yield = false;
    for (const Tangent& t : tan_) any_yield |= t.on && t.yield;
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
    changed_.clear();
    for (int pass = 0; pass < 3; pass++) {
    PROFILE_ACCUM("Frame assemble");
    passes_++;
    if (pass > 0) {
        // again with the unloaded hinges' elastic tangents: those members' change into the system assembled before
        // (kept: the factorization works in place), not the whole assembly again
        for (const Changed& ch : changed_) {
            Tangent was = tan_[ch.elem];
            was.unload = ch.unload;
            add_member(ch.elem, effective(was), -1.0, false, diagA_.data(), offA_.data(), rhsA_.data());
            add_member(ch.elem, effective(tan_[ch.elem]), 1.0, false, diagA_.data(), offA_.data(), rhsA_.data());
        }
        changed_.clear();
        std::copy(diagA_.begin(), diagA_.end(), diag_.begin());
        std::copy(offA_.begin(), offA_.end(), off_.begin());
        std::copy(rhsA_.begin(), rhsA_.end(), rhs_.begin());
    } else {
    std::fill(diag_.begin(), diag_.end(), 0.0);
    std::fill(off_.begin(), off_.end(), 0.0);
    for (int i = 0; i < n; i++) {
        const Node& x = b.nodes[node[i]];
        const int k = iperm_[i];
        double* D = &diag_[(size_t)k * 36];
        double* r = &rhs_[(size_t)k * 6];
        if (x.inv_mass <= 0) {
            fixed[i] = 1;
            for (int j = 0; j < 6; j++) D[j * 7] = 1.0, r[j] = 0.0;
            continue;
        }
        D[0] = D[7] = D[14] = x.mass;
        D[21] = D[28] = D[35] = inertia[i];
        const vec3 f = F[node[i]], m = torque[i], J = impulse[i];
        // (all of it for h but the contacts; the smooth forces' guess for the later short steps comes off the held
        // impulse, which then tells the next step how far off it was)
        vec3 fc = i < (int)contact_f.size() ? contact_f[i] : vec3(0);
        if (h > step && E) fc += E[node[i]];
        const double hs = (double)h - (double)step;
        const vec3 fm = i < (int)member_f.size() ? member_f[i] : vec3(0);
        r[0] = h * (double)f.x - hs * fc.x + J.x, r[1] = h * (double)f.y - hs * fc.y + J.y, r[2] = h * (double)f.z - hs * fc.z + J.z;
        impulse[i] = (f - fm - fc) * -(float)hs;
        r[3] = h * (double)m.x, r[4] = h * (double)m.y, r[5] = h * (double)m.z;
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
    }
    for (size_t ei = 0; ei < elems.size(); ei++) {
        if (!tan_[ei].on || elems[ei].broken) continue;
        add_member(ei, effective(tan_[ei]), 1.0, true, diag_.data(), off_.data(), rhs_.data());
    }
    // the body's springs on frame nodes: (theta h^2 k + h d) e e^T, their tangent now (a shock past or near its bound:
    // the bound's stiffness; a slack rope: none)
    for (const Link& l : links_) {
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
        auto add_diag = [&](int fn) {
            if (fn < 0 || fixed[fn]) return;
            double* D = &diag_[(size_t)iperm_[fn] * 36];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) D[i * 6 + j] += c * ee[i] * ee[j];
        };
        add_diag(l.fa);
        add_diag(l.fb);
        if (l.block >= 0 && !fixed[l.fa] && !fixed[l.fb]) {
            double* O = &off_[(size_t)l.block * 36];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) O[i * 6 + j] -= c * ee[i] * ee[j];
        }
    }
    if (any_yield) diagA_ = diag_, offA_ = off_, rhsA_ = rhs_;
    }
    // block Cholesky, right-looking over the precomputed pattern (the diagonal blocks' inverse diagonals kept)
    PROFILE_ACCUM("Frame factor");
    for (int k = 0; k < n; k++) {
        double* Lkk = &diag_[(size_t)k * 36];
        double* dk = &dinv_[(size_t)k * 6];
        if (!chol6(Lkk, dk)) {
            solve_failures++;
            for (vec3& t : torque) t = vec3(0);
            for (vec3& c : contact_n) c = vec3(0);
            for (vec3& c : contact_f) c = vec3(0);
            for (vec3& J : impulse) J = vec3(0);
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
    // L L^T x = r
    for (int k = 0; k < n; k++) {
        double* y = &rhs_[(size_t)k * 6];
        forward6(&diag_[(size_t)k * 36], &dinv_[(size_t)k * 6], y);
        for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) {
            const double* L = &off_[(size_t)p * 36];
            double* r = &rhs_[(size_t)row_[p] * 6];
            for (int i = 0; i < 6; i++) {
                double s = 0;
                for (int j = 0; j < 6; j++) s += L[i * 6 + j] * y[j];
                r[i] -= s;
            }
        }
    }
    for (int k = n - 1; k >= 0; k--) {
        double* x = &rhs_[(size_t)k * 6];
        for (int p = col_ptr_[k]; p < col_ptr_[k + 1]; p++) {
            const double* L = &off_[(size_t)p * 36];
            const double* xr = &rhs_[(size_t)row_[p] * 6];
            for (int j = 0; j < 6; j++) {
                double s = 0;
                for (int i = 0; i < 6; i++) s += L[i * 6 + j] * xr[i];
                x[j] -= s;
            }
        }
        backward6(&diag_[(size_t)k * 36], &dinv_[(size_t)k * 6], x);
    }
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
    for (size_t ei = 0; ei < elems.size(); ei++) {
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
        if (u) changed_.push_back({(uint32_t)ei, t.unload}), t.unload |= u, t.elastic_hold = 16, again = true;
    }
    if (!again) break;
    }
    // the new velocities: to the body's integrator as a force, the rotations advanced here
    vec3* Fw = b.force.data();
    for (int i = 0; i < n; i++) {
        if (fixed[i]) {
            w[i] = vec3(0);
            continue;
        }
        const double* x = &rhs_[(size_t)iperm_[i] * 6];
        const Node& nd = b.nodes[node[i]];
        vec3 dv((float)x[0], (float)x[1], (float)x[2]);
        const vec3 dw((float)x[3], (float)x[4], (float)x[5]);
        if (!std::isfinite(dv.x + dv.y + dv.z + dw.x + dw.y + dw.z)) {
            solve_failures++;
            continue;
        }
        // (safety: no real impact changes a node's velocity by 40 m/s in a step or spins it past 3000 rad/s; a node
        // squeezed between heavy bodies by stiff contacts could, and would fling the frame apart)
        const float kMaxDv = 40.0f, kMaxW = 3000.0f;
        if (const float l = length(dv); l > kMaxDv) dv *= kMaxDv / l, clamps++;
        Fw[node[i]] = dv * (nd.mass / step);
        w[i] += dw;
        if (const float l = length(w[i]); l > kMaxW) w[i] *= kMaxW / l, clamps++;
    }
    for (vec3& t : torque) t = vec3(0); // (torques from outside, e.g. the tests, add up until the next step)
    for (vec3& c : contact_n) c = vec3(0);
    for (vec3& c : contact_f) c = vec3(0);
}

void FemFrame::node_inertia(uint32_t fn, const SoftBody& b) {
    // lumped rotational inertia of the members' halves (HRZ lumping of the consistent mass: m L^2 / 78 per end), with
    // a floor from the node's own mass
    float I = 0;
    for (const FrameElement& e : elems)
        if (!e.broken && (e.a == fn || e.b == fn)) I += element_mass(e) * e.L0 * e.L0 / 78.0f;
    const float mn = node[fn] < b.nodes.size() ? b.nodes[node[fn]].mass : 0.0f;
    inertia[fn] = std::max(I, std::max(1e-6f, mn * 1e-4f));
}

int FemFrame::members_at(uint32_t fn) const {
    int n = 0;
    for (const FrameElement& e : elems) n += !e.broken && (e.a == fn || e.b == fn);
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
    return id;
}

void set_mass(Node& n, float m, bool fixed) {
    n.mass = m;
    n.inv_mass = fixed || m <= 0 ? 0.0f : 1.0f / m;
}

} // namespace

int FemFrame::split(SoftBody& b, uint32_t ei, float t) {
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
    if (events_.empty()) return false;
    const size_t n0 = b.nodes.size();
    std::vector<Event> ev;
    ev.swap(events_);
    for (const Event& x : ev) {
        switch (x.kind) {
        case 0: split(b, x.elem, x.t); break;
        case 1: tear(b, x.elem, 0); break;
        case 2: tear(b, x.elem, 1); break;
        default: // split, and the half of the given end torn off the new node
            if (split(b, x.elem, x.t) >= 0) tear(b, x.elem, 1);
            else elems[x.elem].damage = 0;
            break;
        }
    }
    if (!ready_) {
        // the frame nodes' pattern again (their orientation, velocities and positions are kept)
        torque.resize(node.size(), vec3(0));
        member_f.resize(node.size(), vec3(0));
        body_ = &b;
        analyse();
        tan_.assign(elems.size(), Tangent());
        ready_ = true;
    }
    return b.nodes.size() != n0;
}

void FemFrame::sync_positions(SoftBody& b, float h) {
    for (size_t i = 0; i < node.size(); i++) {
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
    for (uint32_t v : node)
        if (v < slot_.size()) slot_[v] = -1;
    node.swap(nn), q.swap(qq), w.swap(ww), inertia.swap(ii), torque.swap(tt), contact_n.swap(cc), impulse.swap(jj), xd.swap(xx);
    member_f.assign(node.size(), vec3(0));
    contact_f.assign(node.size(), vec3(0));
    elems.swap(es);
    for (size_t i = 0; i < node.size(); i++) slot_[node[i]] = (int32_t)i;
    events_.clear();
    body_ = &b;
    analyse();
    tan_.assign(elems.size(), Tangent());
    ready_ = true;
}

int FemFrame::detach_debris(SoftBody& b, std::vector<std::unique_ptr<SoftBody>>& out, float max_mass) {
    debris_check = false;
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
    for (const FrameElement& e : elems)
        if (!e.broken) join(node[e.a], node[e.b]);
    for (const Beam& bm : b.beams)
        if (!(bm.flags & BF_BROKEN)) join(bm.a, bm.b), other[bm.a] = other[bm.b] = 1;
    for (const Shell& sh : b.shells) join(sh.n[0], sh.n[1]), join(sh.n[1], sh.n[2]), other[sh.n[0]] = other[sh.n[1]] = other[sh.n[2]] = 1;
    for (const Triangle& t : b.tris) join(t.a, t.b), join(t.b, t.c), other[t.a] = other[t.b] = other[t.c] = 1;
    for (const Joint& j : b.joints)
        if (!j.broken && j.parent_frame < b.frames.size()) join(b.frames[j.parent_frame].node, j.child_node), other[j.child_node] = 1;
    for (const Capsule& c : b.capsules) join(c.a, c.b), other[c.a] = other[c.b] = 1;
    for (const SlideNode& sl : b.slides) {
        other[sl.node] = 1;
        for (uint32_t r : sl.rail) join(sl.node, r), other[r] = 1;
    }
    for (const Wheel& wh : b.wheels) {
        for (uint32_t v : wh.nodes) other[v] = 1;
        for (uint32_t v : wh.rim) other[v] = 1;
        other[wh.axle0] = other[wh.axle1] = 1;
    }
    struct Part {
        float mass = 0;
        int members = 0;
        bool other = false, fixed = false;
    };
    std::vector<Part> parts(n);
    for (uint32_t i = 0; i < n; i++) {
        Part& p = parts[find(i)];
        p.mass += b.nodes[i].mass;
        p.other |= other[i] != 0;
        p.fixed |= b.nodes[i].inv_mass <= 0 && b.nodes[i].mass > 0 && (b.info[i].flags & NF_FIXED);
    }
    for (const FrameElement& e : elems)
        if (!e.broken) parts[find(node[e.a])].members++;
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
    if (roots.empty()) return 0;
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
    // in this body: the debris' nodes switched off (their indices stay: vehicles and meshes refer to nodes by number)
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
    slot_.assign(b.nodes.size(), -1);
    for (size_t i = 0; i < node.size(); i++)
        if (node[i] < slot_.size()) slot_[node[i]] = (int32_t)i;
    if (!ready_) return; // (finalize: at the first step)
    body_ = &b; // (the links and the pattern again; the members' state is kept)
    analyse();
    tan_.assign(elems.size(), Tangent());
}

void FemFrame::split_off(SoftBody& b, const std::vector<int>& part_of, const std::vector<uint32_t>& nidx, const std::vector<SoftBody*>& pieces) {
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
    // the kept frame: its nodes renumbered (those gone are unused now), then compacted
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
    finish_cuts(b, n);
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

void FemFrame::finish_cuts(SoftBody& b, int tears) {
    if (!ready_) {
        torque.resize(node.size(), vec3(0));
        member_f.resize(node.size(), vec3(0));
        body_ = &b;
        analyse();
        tan_.assign(elems.size(), Tangent());
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
