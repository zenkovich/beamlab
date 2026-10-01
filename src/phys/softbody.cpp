#include "phys/softbody.h"
#include "core/profiler.h"

#include <cmath>

namespace bl::phys {

uint32_t SoftBody::add_node(vec3 p, float mass, uint16_t flags) {
    Node n;
    n.p = p;
    n.v = vec3(0);
    n.mass = mass;
    n.inv_mass = (flags & NF_FIXED) || mass <= 0 ? 0.0f : 1.0f / mass;
    nodes.push_back(n);
    force.push_back(vec3(0));
    NodeInfo inf;
    inf.flags = flags;
    info.push_back(inf);
    return (uint32_t)nodes.size() - 1;
}

uint32_t SoftBody::add_beam(uint32_t a, uint32_t b, float k, float d, float strength, float deform, uint8_t type, uint8_t flags) {
    Beam bm{};
    bm.a = a;
    bm.b = b;
    bm.L = bm.L0 = distance(nodes[a].p, nodes[b].p);
    bm.k = k;
    bm.d = d;
    bm.max_pos = deform;
    bm.max_neg = -deform;
    bm.strength = strength;
    bm.plastic = 0;
    bm.stress = 0;
    bm.support_limit = 4.0f;
    bm.type = type;
    bm.flags = flags;
    bm.group = 0;
    beams.push_back(bm);
    return (uint32_t)beams.size() - 1;
}

uint32_t SoftBody::add_frame(uint32_t node, quat q, float inertia) {
    Frame f;
    f.node = node;
    f.q = q;
    f.w = vec3(0);
    f.inv_inertia = inertia > 0 && nodes[node].inv_mass > 0 ? 1.0f / inertia : 0.0f;
    f.torque = vec3(0);
    frames.push_back(f);
    info[node].flags |= NF_FRAME;
    info[node].frame = (int32_t)frames.size() - 1;
    return (uint32_t)frames.size() - 1;
}

uint32_t SoftBody::add_joint(uint32_t parent_frame, uint32_t child_node, int32_t child_frame, float k_lin, float d_lin, float k_ang, float d_ang) {
    Joint j{};
    const Frame& pf = frames[parent_frame];
    j.parent_frame = parent_frame;
    j.child_node = child_node;
    j.child_frame = child_frame;
    j.local_offset = conj(pf.q).rotate(nodes[child_node].p - nodes[pf.node].p);
    j.rel_rot = child_frame >= 0 ? normalize(conj(pf.q) * frames[child_frame].q) : quat();
    j.k_lin = k_lin;
    j.d_lin = d_lin;
    j.k_ang = k_ang;
    j.d_ang = d_ang;
    j.break_force = 0;
    j.break_torque = 0;
    j.yield_angle = 0;
    j.radius = 0.05f;
    j.stress = 0;
    joints.push_back(j);
    return (uint32_t)joints.size() - 1;
}

void SoftBody::finalize() {
    for (auto& b : beams) {
        b.L = b.L0 = std::max(1e-3f, distance(nodes[b.a].p, nodes[b.b].p));
    }
    for (Triangle& t : tris) t.rest_edge2 = max_edge2(nodes[t.a].p, nodes[t.b].p, nodes[t.c].p);
    force.resize(nodes.size());
    compute_aabb();
}

int SoftBody::stabilize(float dt, float k_budget, float d_budget) {
    const size_t n = nodes.size();
    std::vector<float> ksum(n, 0.0f), dsum(n, 0.0f);
    for (const Beam& b : beams) {
        ksum[b.a] += b.k;
        ksum[b.b] += b.k;
        dsum[b.a] += b.d;
        dsum[b.b] += b.d;
    }
    std::vector<float> kscale(n, 1.0f), dscale(n, 1.0f);
    for (size_t i = 0; i < n; i++) {
        float m = nodes[i].mass;
        if (m <= 0) continue;
        float kl = ksum[i] * dt * dt / m;
        if (kl > k_budget) kscale[i] = k_budget / kl;
        float dl = dsum[i] * dt / m;
        if (dl > d_budget) dscale[i] = d_budget / dl;
    }
    int changed = 0;
    for (Beam& b : beams) {
        float ks = std::min(kscale[b.a], kscale[b.b]);
        float ds = std::min(dscale[b.a], dscale[b.b]);
        if (ks < 1.0f || ds < 1.0f) changed++;
        b.k *= ks;
        b.d *= ds;
    }
    if (joints.empty()) return changed;
    // ---- orientation preserving joints
    // Translational budget per node (same rule as beams, including beams already scaled above).
    std::vector<float> jk(n, 0.0f), jd(n, 0.0f);
    for (const Beam& b : beams) {
        jk[b.a] += b.k;
        jk[b.b] += b.k;
        jd[b.a] += b.d;
        jd[b.b] += b.d;
    }
    for (const Joint& j : joints) {
        uint32_t pn = frames[j.parent_frame].node;
        jk[pn] += j.k_lin;
        jk[j.child_node] += j.k_lin;
        jd[pn] += j.d_lin;
        jd[j.child_node] += j.d_lin;
    }
    for (Joint& j : joints) {
        uint32_t pn = frames[j.parent_frame].node;
        float s = 1.0f, sd = 1.0f;
        for (uint32_t ni : {pn, j.child_node}) {
            float m = nodes[ni].mass;
            if (m <= 0 || nodes[ni].inv_mass <= 0) continue;
            float kl = jk[ni] * dt * dt / m;
            if (kl > k_budget) s = std::min(s, k_budget / kl);
            float dl = jd[ni] * dt / m;
            if (dl > d_budget) sd = std::min(sd, d_budget / dl);
        }
        j.k_lin *= s;
        j.d_lin *= sd;
    }
    // Rotational budget per frame: the linear spring acts on the parent frame through the lever arm
    // (stiffness k_lin * L^2) in addition to the angular springs. When the frame's inertia is too
    // small for explicit integration, the inertia is inflated (slower but stable rotation).
    std::vector<float> ka(frames.size(), 0.0f), da(frames.size(), 0.0f);
    for (const Joint& j : joints) {
        float L2 = length2(j.local_offset);
        ka[j.parent_frame] += j.k_lin * L2 + j.k_ang;
        da[j.parent_frame] += j.d_lin * L2 + j.d_ang;
        if (j.child_frame >= 0) {
            ka[j.child_frame] += j.k_ang;
            da[j.child_frame] += j.d_ang;
        }
    }
    for (size_t f = 0; f < frames.size(); f++) {
        Frame& fr = frames[f];
        if (fr.inv_inertia <= 0) continue;
        float I = 1.0f / fr.inv_inertia;
        float need = std::max(ka[f] * dt * dt / (0.5f * k_budget), da[f] * dt / d_budget);
        if (need > I) {
            fr.inv_inertia = 1.0f / need;
            changed++;
        }
    }
    return changed;
}

float SoftBody::total_mass() const {
    float m = 0;
    for (auto& n : nodes) m += n.mass;
    return m;
}

vec3 SoftBody::center_of_mass() const {
    vec3 c(0);
    float m = 0;
    for (auto& n : nodes) {
        c += n.p * n.mass;
        m += n.mass;
    }
    return m > 0 ? c / m : c;
}

vec3 SoftBody::average_velocity() const {
    vec3 c(0);
    float m = 0;
    for (auto& n : nodes) {
        c += n.v * n.mass;
        m += n.mass;
    }
    return m > 0 ? c / m : c;
}

void SoftBody::compute_aabb() {
    AABB b;
    for (auto& n : nodes) b.add(n.p);
    aabb = b;
}

void SoftBody::translate(vec3 d) {
    for (auto& n : nodes) n.p += d;
    compute_aabb();
}

void SoftBody::set_velocity(vec3 v) {
    for (auto& n : nodes)
        if (n.inv_mass > 0) n.v = v;
    for (auto& f : frames) f.w = vec3(0);
    fem.stop();
}

void SoftBody::transform(const quat& r, vec3 pivot, vec3 t) {
    for (auto& n : nodes) {
        n.p = r.rotate(n.p - pivot) + pivot + t;
        n.v = r.rotate(n.v);
    }
    for (auto& f : frames) {
        f.q = normalize(r * f.q);
        f.w = r.rotate(f.w);
    }
    fem.rotate(r);
    compute_aabb();
}

void SoftBody::clear_forces(vec3 g) {
    const size_t n = nodes.size();
    vec3* f = force.data();
    const Node* nd = nodes.data();
    if (air_drag > 0 || aero_cda > 0) {
        // RoR applies -0.05*|v|*v to every node (~9 kN at 75 km/h on a 440-node car, 50x a real car's drag).
        // Here: the RoR term on each node's velocity relative to the body mean (it still damps flapping parts),
        // plus real aerodynamic drag 0.5*rho*CdA*|v|*v on the mean velocity, shared by mass.
        vec3 mv(0);
        float M = 0;
        for (size_t i = 0; i < n; i++) {
            mv += nd[i].v * nd[i].mass;
            M += nd[i].mass;
        }
        vec3 vm = M > 0 ? mv / M : vec3(0);
        vec3 aero_acc = M > 0 ? vm * (-0.5f * 1.225f * aero_cda * length(vm) / M) : vec3(0);
        const NodeInfo* inf = info.data();
        for (size_t i = 0; i < n; i++) {
            f[i] = (g + aero_acc) * nd[i].mass;
            if (inf[i].flags & (NF_TYRE | NF_RIM)) continue; // spinning tread: not a flapping part
            vec3 vr = nd[i].v - vm;
            f[i] -= vr * (air_drag * length(vr));
        }
    } else {
        for (size_t i = 0; i < n; i++) f[i] = g * nd[i].mass;
    }
    for (auto& fr : frames) fr.torque = vec3(0);
    if (grab_node >= 0 && grab_node < (int)n) {
        const float gs = clampf(grab_scale, 0.01f, 100.0f);
        const bool many = !grab_nodes.empty() && grab_nodes.size() == grab_offsets.size() && grab_nodes.size() == grab_w.size();
        float msum = 0; // (weight x mass)
        if (many)
            for (size_t j = 0; j < grab_nodes.size(); j++) msum += grab_nodes[j] < n ? nodes[grab_nodes[j]].mass * grab_w[j] : 0.0f;
        const size_t cnt = many ? grab_nodes.size() : 1;
        for (size_t j = 0; j < cnt; j++) {
            const uint32_t i = many ? grab_nodes[j] : (uint32_t)grab_node;
            if (i >= n) continue;
            Node& gn = nodes[i];
            if (gn.inv_mass <= 0) continue;
            const float share = many ? (msum > 0 ? gn.mass * grab_w[j] / msum : 0.0f) : 1.0f; // (of the pull and of its force cap's constant)
            if (!(share > 0)) continue;
            vec3 d = grab_target + (many ? grab_offsets[j] : vec3(0)) - gn.p;
            float k = (grab_k > 0 ? grab_k * share : gn.mass * 400.0f) * gs;
            // critically damped pull, force capped for robustness
            vec3 F = d * k - gn.v * (2.0f * std::sqrt(k * gn.mass));
            // (and capped by acceleration: a 5 g cloth node pulled with 20 kN would jump kilometres per second; a stronger
            // grab raises the cap, up to 40000 m/s2)
            float fl = length(F), fmax = std::min(std::min(gn.mass * 400.0f + 20000.0f * share, gn.mass * 5000.0f) * gs, gn.mass * 40000.0f);
            if (fl > fmax) F *= fmax / fl;
            f[i] += F;
        }
    }
}

// RoR beam model: linear spring-damper along the beam axis with plastic yield and breaking.
void SoftBody::compute_beam_forces() {
    Node* nd = nodes.data();
    vec3* f = force.data();
    const bool deform = allow_deform, brk = allow_break;
    for (Beam& b : beams) {
        if (b.flags & (BF_BROKEN | BF_SHOCK)) continue;
        const Node& na = nd[b.a];
        const Node& nb = nd[b.b];
        vec3 dis = na.p - nb.p;
        float len2 = dot(dis, dis);
        if (len2 < 1e-12f) continue;
        float inv_len = 1.0f / std::sqrt(len2);
        float len = len2 * inv_len;
        float diff = len - b.L;
        vec3 dv = na.v - nb.v;
        float k = b.k, d = b.d;
        if (b.type == BT_ROPE) {
            if (diff < 0) { k = 0; d *= 0.1f; }
        } else if (b.type == BT_SUPPORT) {
            if (diff > 0) {
                k = 0;
                d *= 0.1f;
                if (brk && diff > b.L * b.support_limit) { b.flags |= BF_BROKEN; stats.broken_beams++; continue; }
            }
        }
        float slen = -k * diff - d * dot(dv, dis) * inv_len; // > 0: compression pushes nodes apart
        if (deform && !(b.flags & BF_NO_DEFORM) && k != 0) {
            if (slen > b.max_pos && diff < 0) {
                // compressive yield: the rest length follows, leaving the yield strain (work hardening)
                float yield_len = b.max_pos / k;
                float def = diff + yield_len * (1.0f - b.plastic);
                float Lold = b.L;
                b.L = std::max(std::min(0.1f, 0.3f * b.L0), b.L + def); // RoR floor (0.1 m), relative for short beams
                slen = slen - (slen - b.max_pos) * 0.5f;
                if (b.L < Lold) b.max_pos *= Lold / b.L;
            } else if (slen < b.max_neg && diff > 0) {
                // tensile yield: lengthen, harden and weaken
                float yield_len = b.max_neg / k;
                float def = diff + yield_len * (1.0f - b.plastic);
                float Lold = b.L;
                b.L = b.L + def;
                slen = slen - (slen - b.max_neg) * 0.5f;
                if (Lold > 0 && b.L > Lold) b.max_neg *= b.L / Lold;
                if (!ductile) b.strength -= def * k; // RoR: tension weakens the beam
            }
        }
        if (brk && std::fabs(slen) > b.strength && !(b.flags & BF_NO_BREAK)) {
            b.flags |= BF_BROKEN;
            stats.broken_beams++;
            static const bool dbg = getenv("BL_BEAMDBG") != nullptr; // (diagnostics: which beam broke)
            if (dbg) printf("beam %u-%u broke at %.0f N (strength %.0f)\n", b.a, b.b, std::fabs(slen), b.strength);
            continue;
        }
        b.stress = slen;
        vec3 fv = dis * (slen * inv_len);
        f[b.a] += fv;
        f[b.b] -= fv;
    }
}

void SoftBody::compute_shock_forces() {
    Node* nd = nodes.data();
    vec3* f = force.data();
    for (const Shock& s : shocks) {
        Beam& b = beams[s.beam];
        if (b.flags & BF_BROKEN) continue;
        const Node& na = nd[b.a];
        const Node& nb = nd[b.b];
        vec3 dis = na.p - nb.p;
        float len2 = dot(dis, dis);
        if (len2 < 1e-12f) continue;
        float inv_len = 1.0f / std::sqrt(len2);
        float len = len2 * inv_len;
        const float L = b.L;
        float diff = len - L;
        float v = dot(na.v - nb.v, dis) * inv_len; // > 0 lengthening
        float k, d;
        const float lb = s.long_bound * L, sb = s.short_bound * L;
        if (s.type == 1) {
            k = s.spring;
            d = s.damp;
            float r = 0;
            if (diff > lb) r = diff - lb;
            else if (diff < -sb) r = -diff - sb;
            if (r != 0) {
                r = std::min(r, 1.0f);
                k += (s.bound_spring - k) * r;
                d += (s.bound_damp - d) * r;
            }
        } else if (s.type == 2) {
            auto prog = [&](bool out, float& kk, float& dd) {
                if (out) {
                    kk = s.spring_out;
                    dd = s.damp_out;
                    float fr = s.long_bound != 0 ? std::min(sqr(diff / (s.long_bound * L)), 1.0f) : 1.0f;
                    kk += s.prog_spring_out * kk * fr;
                    dd += s.prog_damp_out * dd * fr;
                } else {
                    kk = s.spring_in;
                    dd = s.damp_in;
                    float fr = s.short_bound != 0 ? std::min(sqr(diff / (s.short_bound * L)), 1.0f) : 1.0f;
                    kk += s.prog_spring_in * kk * fr;
                    dd += s.prog_damp_in * dd * fr;
                }
            };
            prog(v > 0, k, d);
            if (s.soft_bump) {
                float Lp = 0.8f * L, lpre = s.long_bound * Lp, spre = -s.short_bound * Lp;
                if (diff > lpre) {
                    prog(true, k, d);
                    float f2 = s.long_bound != 0 ? std::min(sqr((diff - lpre) * 5.0f / (s.long_bound * L)), 1.0f) : 1.0f;
                    k += (k + 100) * s.prog_spring_out * f2;
                    d += (d + 100) * s.prog_damp_out * f2;
                    if (v < 0) { k = s.spring_in; d = s.damp_in; }
                } else if (diff < spre) {
                    prog(false, k, d);
                    float f2 = s.short_bound != 0 ? std::min(sqr((diff - spre) * 5.0f / (s.short_bound * L)), 1.0f) : 1.0f;
                    k += (k + 100) * s.prog_spring_out * f2;
                    d += (d + 100) * s.prog_damp_out * f2;
                    if (v > 0) { k = s.spring_out; d = s.damp_out; }
                }
                if (diff > lb || diff < -sb) {
                    k = std::max(k, s.bound_spring);
                    d = std::max(d, s.bound_damp);
                }
            } else if (diff > lb || diff < -sb) {
                k = s.bound_spring;
                d = s.bound_damp;
            }
        } else {
            k = s.spring_in;
            d = s.damp_in;
            if (diff > lb) {
                float r = std::min(diff - lb, 1.0f);
                k += (s.bound_spring - k) * r;
                d += (s.bound_damp - d) * r;
            } else if (diff < -sb) {
                float r = std::min(-diff - sb, 1.0f);
                k += (s.bound_spring - k) * r;
                d += (s.bound_damp - d) * r;
            } else if (v > 0) {
                float w = clampf(std::fabs(v), 0.15f, 20.0f);
                k = s.spring_out;
                d = (s.damp_out * s.damp_out_slow * std::min(w, s.split_out) + s.damp_out * s.damp_out_fast * std::max(0.0f, w - s.split_out)) / w;
            } else if (v < 0) {
                float w = clampf(std::fabs(v), 0.15f, 20.0f);
                k = s.spring_in;
                d = (s.damp_in * s.damp_in_slow * std::min(w, s.split_in) + s.damp_in * s.damp_in_fast * std::max(0.0f, w - s.split_in)) / w;
            }
        }
        float slen = -k * diff - d * v;
        if (allow_break && std::fabs(slen) > b.strength && !(b.flags & BF_NO_BREAK)) {
            b.flags |= BF_BROKEN;
            stats.broken_beams++;
            continue;
        }
        b.stress = slen;
        vec3 fv = dis * (slen * inv_len);
        f[b.a] += fv;
        f[b.b] -= fv;
    }
}

// Slide nodes (RoR SlideNode): spring from the node to the nearest point of its rail segment;
// the reaction is split between the segment's end nodes by the position along the segment.
void SoftBody::compute_slide_forces() {
    Node* nd = nodes.data();
    vec3* f = force.data();
    for (SlideNode& s : slides) {
        if (s.broken || s.rail.size() < 2) continue;
        const vec3 p = nd[s.node].p;
        auto seg_dist2 = [&](int i, float& t) {
            vec3 a = nd[s.rail[i]].p, b = nd[s.rail[i + 1]].p, ab = b - a;
            float l2 = dot(ab, ab);
            t = l2 > 1e-12f ? clampf(dot(p - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
            return length2(a + ab * t - p);
        };
        const int nseg = (int)s.rail.size() - 1;
        float t, best_t;
        if (s.seg < 0) {
            // initial search over the whole rail
            float best = 1e30f;
            for (int i = 0; i < nseg; i++) {
                float d = seg_dist2(i, t);
                if (d < best) { best = d; s.seg = i; best_t = t; }
            }
        }
        // like RoR: only the current segment and its neighbours are checked each step
        float best = seg_dist2(s.seg, best_t);
        int bi = s.seg;
        if (s.seg > 0) {
            float d = seg_dist2(s.seg - 1, t);
            if (d < best) { best = d; bi = s.seg - 1; best_t = t; }
        }
        if (s.seg + 1 < nseg) {
            float d = seg_dist2(s.seg + 1, t);
            if (d < best) { best = d; bi = s.seg + 1; best_t = t; }
        }
        s.seg = bi;
        uint32_t na = s.rail[bi], nb = s.rail[bi + 1];
        vec3 ideal = nd[na].p + (nd[nb].p - nd[na].p) * best_t;
        vec3 d = ideal - p;
        float dist = length(d);
        float stretch = dist - s.threshold;
        if (stretch <= 0 || dist < 1e-9f) continue;
        vec3 F = d * (s.k * stretch / dist); // pulls the node onto the rail
        if (s.break_force > 0 && length(F) > s.break_force) {
            s.broken = true;
            continue;
        }
        f[s.node] += F;
        f[na] -= F * (1.0f - best_t);
        f[nb] -= F * best_t;
    }
}

// 6-DOF orientation preserving joints (parent frame -> child node / frame).
void SoftBody::compute_joint_forces() {
    Node* nd = nodes.data();
    vec3* f = force.data();
    for (Joint& j : joints) {
        if (j.broken) continue;
        Frame& pf = frames[j.parent_frame];
        const Node& pn = nd[pf.node];
        const Node& cn = nd[j.child_node];
        vec3 r = pf.q.rotate(j.local_offset);     // lever arm in world space
        vec3 err = cn.p - (pn.p + r);
        // linear yield: beyond the yield force the rest offset follows (plastic stretch)
        if (j.break_force > 0 && allow_deform) {
            float ylen = j.break_force / j.k_lin;
            float el = length(err);
            if (el > ylen) {
                float excess = el - ylen;
                vec3 dir = err / el;
                j.local_offset += conj(pf.q).rotate(dir * excess);
                r = pf.q.rotate(j.local_offset);
                err = dir * ylen;
                j.plastic_lin += excess;
            }
        }
        vec3 vrel = cn.v - (pn.v + cross(pf.w, r));
        vec3 F = err * (-j.k_lin) - vrel * j.d_lin;  // force on child
        f[j.child_node] += F;
        f[pf.node] -= F;
        pf.torque += cross(r, -F);
        float s = j.break_force > 0 ? length(err) * j.k_lin / j.break_force : 0.0f;
        if (j.child_frame >= 0) {
            Frame& cf = frames[j.child_frame];
            quat qe = cf.q * conj(pf.q * j.rel_rot);
            vec3 theta = quat_log(qe);              // world-space rotation error
            float ang = length(theta);
            float yield = j.yield_angle > 0 ? j.yield_angle : 1e9f;
            if (j.break_torque > 0) yield = std::min(yield, j.break_torque / j.k_ang);
            if (ang > yield && allow_deform) {
                // plastic bend: move the rest orientation towards the current one
                float excess = ang - yield;
                vec3 axis = theta / ang;
                quat bend = quat::axis_angle(axis, excess);
                // rel_rot' such that pf.q * rel_rot' = bend * pf.q * rel_rot
                j.rel_rot = normalize(conj(pf.q) * bend * pf.q * j.rel_rot);
                theta = axis * yield;
                j.plastic_ang += excess;
            }
            vec3 T = theta * (-j.k_ang) - (cf.w - pf.w) * j.d_ang;
            cf.torque += T;
            pf.torque -= T;
            if (j.break_torque > 0) s = std::max(s, length(theta) * j.k_ang / j.break_torque);
        }
        j.stress = s;
        bool snap = (j.break_force > 0 && j.plastic_lin > j.max_stretch) || (j.break_torque > 0 && j.plastic_ang > j.max_bend);
        if (allow_break && snap) {
            j.broken = true;
            stats.broken_joints++;
        }
    }
}

// RoR CalcWheels: brake "stop torque", torque -> tangential tread forces, speed measurement,
// reaction torque on the suspension arm.
void SoftBody::compute_wheel_forces(float dt, bool first) {
    Node* nd = nodes.data();
    vec3* f = force.data();
    float speed_sum = 0, spin_sum = 0;
    int nprop = 0;
    for (Wheel& w : wheels) {
        if (w.detached) continue;
        if (w.ring) { // (a ring tyre: its spin its own - World::ring_tyres takes the torque and the brake)
            if (first) w.drive_torque = w.torque;
            else w.torque = w.drive_torque;
            w.speed = (w.propulsed == 2 ? -w.spin : w.spin) * w.radius;
            w.avg_speed = 0.99f * w.avg_speed + 0.1f * w.speed;
            if (w.propulsed == 1) {
                speed_sum += w.speed;
                spin_sum += w.speed / std::max(0.05f, w.radius);
                nprop++;
            }
            w.last_torque = w.torque;
            continue;
        }
        if (w.nodes.empty()) continue;
        if (first) w.drive_torque = w.torque;
        else w.torque = w.drive_torque;
        if (w.brake > 0) {
            float stop = -w.avg_speed * w.radius * w.mass / dt - w.last_retorque;
            w.torque += w.speed > 0 ? clampf(stop, -w.brake, 0.0f) : clampf(stop, 0.0f, w.brake);
        }
        const Node& h0 = nd[w.axle0];
        const Node& h1 = nd[w.axle1];
        vec3 axis = normalize(h1.p - h0.p);
        const int nn = (int)w.nodes.size();
        float per = w.torque / (float)nn;
        float expected = w.speed;
        float speed = 0;
        for (int j = 0; j < nn; j++) {
            const Node& hub = (j & 1) ? h1 : h0;
            uint32_t ni = w.nodes[j];
            vec3 rad = nd[ni].p - hub.p;
            float rl2 = dot(rad, rad);
            if (rl2 < 1e-8f) continue;
            float inv = 1.0f / std::sqrt(rl2);
            if (w.propulsed == 2) rad = -rad;
            vec3 dir = cross(axis, rad) * inv;
            f[ni] += dir * (per * inv);
            speed += dot(nd[ni].v - hub.v, dir);
        }
        w.speed = speed / (float)nn;
        w.avg_speed = 0.99f * w.avg_speed + 0.1f * w.speed;
        if (w.propulsed == 1) {
            speed_sum += w.speed;
            spin_sum += w.speed / std::max(0.05f, w.radius);
            nprop++;
        }
        if (w.mass > 0 && w.radius > 0) {
            expected += (w.last_torque / w.radius) / w.mass * dt;
            w.last_retorque = w.mass * (w.speed - expected) / dt;
        }
        // reaction torque on the wheel support
        if (w.arm >= 0 && w.near_attach >= 0 && std::fabs(w.torque) > 0.01f) {
            vec3 rr = nd[w.arm].p - nd[w.near_attach].p;
            vec3 r = rr - axis * dot(rr, axis);
            float off = length(rr - r), rl = length(r);
            if (rl > 0.01f && 2 * off < rl) {
                r /= rl;
                vec3 cf = cross(axis, r) * (0.5f * w.torque / rl) * (1.0f - 2.0f * off / rl);
                f[w.arm] -= cf;
                f[w.near_attach] += cf;
            }
        }
        w.last_torque = w.torque;
        w.torque = 0;
    }
    if (nprop > 0) {
        wheel_speed = speed_sum / nprop;
        wheel_spin = spin_sum / nprop;
    }
}

void SoftBody::integrate_nodes(size_t n0, size_t n1, float dt, vec3& mn_out, vec3& mx_out, float& max_v2_out) {
    Node* nd = nodes.data();
    const vec3* f = force.data();
    float max_v2 = max_v2_out;
    vec3 mn = mn_out, mx = mx_out;
    for (size_t i = n0; i < n1; i++) {
        Node& x = nd[i];
        if (x.inv_mass > 0) {
            x.v += f[i] * (x.inv_mass * dt);
            float v2 = dot(x.v, x.v);
            if (v2 > kMaxNodeSpeed * kMaxNodeSpeed) { // runaway: clamp (a NaN fails both tests and is reset below)
                x.v *= kMaxNodeSpeed / std::sqrt(v2);
                v2 = kMaxNodeSpeed * kMaxNodeSpeed;
            } else if (!(v2 >= 0.0f)) {
                x.v = vec3(0);
                v2 = 0;
            }
            x.p += x.v * dt;
            max_v2 = v2 > max_v2 ? v2 : max_v2;
        }
        mn = vmin(mn, x.p);
        mx = vmax(mx, x.p);
    }
    mn_out = mn;
    mx_out = mx;
    max_v2_out = max_v2;
}

void SoftBody::motion_energy(vec3 g, double& ke, double& pe) const {
    ke = pe = 0;
    for (const Node& x : nodes)
        if (x.inv_mass > 0) ke += 0.5 * x.mass * length2(x.v), pe -= x.mass * (double)dot(g, x.p);
    for (size_t i = 0; i < fem.w.size() && i < fem.inertia.size(); i++) ke += 0.5 * fem.inertia[i] * length2(fem.w[i]);
}

void SoftBody::guard_energy(vec3 g) {
    double ke, pe;
    motion_energy(g, ke, pe);
    const double excess = (ke + pe) - (guard_ke + guard_pe) - (2.0 + 0.05 * guard_ke);
    if (!(excess > 0) || !(ke > 0)) return;
    const float s = (float)std::sqrt(std::max(0.0, ke - excess) / ke);
    for (Node& x : nodes) x.v *= s;
    for (vec3& w : fem.w) w *= s;
    guard_cuts++;
    static const bool dbg = getenv("BL_GUARDDBG") != nullptr;
    if (dbg) printf("guard %s: +%.2f J over its %.2f J (kinetic %.2f J at the start), velocities x %.3f\n", name.c_str(), excess, guard_ke + guard_pe, guard_ke, s);
}

bool SoftBody::over_support(vec3 c, float spare) const {
    if (ground_touch.size() != nodes.size()) return true; // (not known: as if it were)
    thread_local std::vector<vec2> pts, hull;
    pts.clear();
    // the nodes on the ground in this frame's short steps, and those as low (5 mm): a drum lying on its four corners
    // rocks on three, the fourth a hair above the ground
    float low = 1e30f;
    for (size_t i = 0; i < nodes.size(); i++)
        if (ground_touch[i] & 2) low = std::min(low, nodes[i].p.y);
    for (size_t i = 0; i < nodes.size(); i++)
        if ((ground_touch[i] & 2) || nodes[i].p.y < low + 0.005f) pts.push_back(vec2(nodes[i].p.x, nodes[i].p.z));
    if (pts.empty()) return true; // (on another body, or in the air: not the ground's to say)
    // the convex hull (Andrew's monotone chain), then the centre's distance inside each edge
    std::sort(pts.begin(), pts.end(), [](const vec2& a, const vec2& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    auto turn = [](const vec2& o, const vec2& a, const vec2& b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); };
    hull.assign(2 * pts.size(), vec2(0));
    size_t k = 0;
    for (size_t i = 0; i < pts.size(); i++) {
        while (k >= 2 && turn(hull[k - 2], hull[k - 1], pts[i]) <= 0) k--;
        hull[k++] = pts[i];
    }
    for (size_t i = pts.size() - 1, t = k + 1; i-- > 0;) {
        while (k >= t && turn(hull[k - 2], hull[k - 1], pts[i]) <= 0) k--;
        hull[k++] = pts[i];
    }
    hull.resize(k > 1 ? k - 1 : k);
    const vec2 p(c.x, c.z);
    if (hull.size() < 3) { // (a point or a line: within `spare` of it)
        const vec2 a = hull[0], b = hull.back(), ab = b - a;
        const float l2 = dot(ab, ab), t = l2 > 0 ? std::clamp(dot(p - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
        return length(p - (a + ab * t)) <= spare;
    }
    float worst = 1e9f;
    for (size_t i = 0; i < hull.size(); i++) { // (counter-clockwise: inside is to the left of each edge)
        const vec2 a = hull[i], b = hull[(i + 1) % hull.size()], e = b - a;
        const float l = length(e);
        if (l > 0) worst = std::min(worst, turn(a, b, p) / l);
    }
    return worst >= -spare;
}

void SoftBody::apply_rest_damping(float dt, float substep) {
    // the motion as a whole: the centre's velocity and the spin (L = I w about the centre of mass)
    double m = 0;
    vec3 c(0), p(0);
    for (const Node& x : nodes)
        if (x.inv_mass > 0) m += x.mass, c += x.p * x.mass, p += x.v * x.mass;
    if (m <= 0) return;
    c = c / (float)m;
    const vec3 V = p / (float)m;
    vec3 L(0);
    float Ixx = 0, Iyy = 0, Izz = 0, Ixy = 0, Ixz = 0, Iyz = 0, rmax2 = 0;
    for (const Node& x : nodes) {
        if (x.inv_mass <= 0) continue;
        const vec3 r = x.p - c;
        L += cross(r, x.v - V) * x.mass;
        Ixx += x.mass * (r.y * r.y + r.z * r.z), Iyy += x.mass * (r.x * r.x + r.z * r.z), Izz += x.mass * (r.x * r.x + r.y * r.y);
        Ixy -= x.mass * r.x * r.y, Ixz -= x.mass * r.x * r.z, Iyz -= x.mass * r.y * r.z;
        rmax2 = std::max(rmax2, length2(r));
    }
    const mat3 I(vec3(Ixx, Ixy, Ixz), vec3(Ixy, Iyy, Iyz), vec3(Ixz, Iyz, Izz));
    const float det = determinant(I);
    const vec3 w = std::fabs(det) > 1e-12f ? inverse(I) * L : vec3(0);
    const float s = std::max(length(V), length(w) * std::sqrt(rmax2));
    // (its motion as a whole: the trembling of a dented drum is not motion, and reads as a spin of a few tenths of a
    // radian a second: the centre's speed, the spin's at the rim with three times the room)
    const float sr = std::max(length(V), length(w) * std::sqrt(rmax2) / 3.0f);
    // held only where it could stand: its centre of mass over what it stands on (the nodes on the ground seen from above,
    // their hull, 5 mm to spare). Tipping over its rim, a drum came up to the top slowly, the rolling resistance took
    // the last of it there, and it slept balanced on its edge; off its support it neither rests nor sleeps, and rolls
    // freely (the damping below slows its fall, a tip's growth three times its rate, but does not stop it)
    const bool held = over_support(c, 0.005f);
    resting = held && sr < rest_rigid;
    rigid_speed = held ? sr : 1e9f;
    // rolling resistance: on the ground the motion as a whole slows at rest_roll (m/s2) whatever its speed (a drum tipped
    // over or dropped rolled on, round and round, for ten seconds); slow, the damping below as well
    float f = held && rest_roll > 0 && static_contacts > 0 && s > 1e-6f ? std::min(1.0f, rest_roll * dt / s) : 0.0f, fv = 0;
    if (s < rest_speed) {
        const float ramp = 1.0f - s / rest_speed;
        f = 1.0f - (1.0f - f) * (1.0f - (1.0f - std::exp(-rest_damp * dt)) * ramp);
        fv = (1.0f - std::exp(-rest_vib_damp * dt)) * ramp;
    }
    if (f <= 0 && fv <= 0) return;
    for (Node& x : nodes)
        if (x.inv_mass > 0) {
            const vec3 rigid = V + cross(w, x.p - c);
            x.v -= rigid * f + (x.v - rigid) * fv; // (the motion as a whole, and the vibration about it)
        }
    // (a frame's nodes' spins are left to its implicit step: damped here as a torque, the rings' bending in them was
    // kicked once a frame and a drum with frame rings rolled off after a drop)
    (void)substep;
}

void SoftBody::make_rigid() {
    if (rigid || nodes.empty()) return;
    Rigid& r = rb;
    r.mass = 0;
    r.com = vec3(0);
    vec3 mv(0);
    for (const Node& n : nodes) {
        r.mass += n.mass;
        r.com += n.p * n.mass;
        mv += n.v * n.mass;
    }
    if (r.mass <= 0) return;
    r.com = r.com / r.mass;
    r.v = mv / r.mass;
    // inertia of the point masses about the centre, the angular velocity from their angular momentum
    mat3 I;
    I.c[0] = I.c[1] = I.c[2] = vec3(0);
    vec3 L(0);
    for (const Node& n : nodes) {
        const vec3 d = n.p - r.com;
        const float d2 = dot(d, d);
        I = I + (mat3::diag(vec3(d2)) + outer(d, d) * -1.0f) * n.mass;
        L += cross(d, n.v - r.v) * n.mass;
    }
    // (a flat piece has next to no inertia about its normal-in-plane axes numerically: a floor keeps the spin bounded)
    const float floor_i = 1e-4f * r.mass;
    for (int k = 0; k < 3; k++) I.c[k][k] = std::max(I.c[k][k], floor_i);
    r.inv_inertia = inverse(I);
    r.w = r.inv_inertia * L;
    r.q = quat();
    r.local.resize(nodes.size());
    for (size_t i = 0; i < nodes.size(); i++) {
        r.local[i] = nodes[i].p - r.com;
        nodes[i].v = r.v + cross(r.w, r.local[i]);
    }
    rigid = true;
    shell_events.clear();
    for (Shell& sh : shells) sh.pending = 0;
}

void SoftBody::make_soft() {
    if (!rigid) return;
    rigid = false;
    rb.local.clear();
    shell_acc_stale = true;
}

void SoftBody::rigid_step(float dt, bool touching, vec3& mn_out, vec3& mx_out, float& max_v2_out) {
    Rigid& r = rb;
    if (r.mass <= 0 || r.local.size() != nodes.size()) return;
    vec3 F(0), T(0);
    for (size_t i = 0; i < nodes.size(); i++) {
        F += force[i];
        T += cross(nodes[i].p - r.com, force[i]);
    }
    r.v += F * (dt / r.mass);
    const mat3 R = to_mat3(r.q);
    const mat3 inv_i = R * r.inv_inertia * transpose(R);
    r.w += inv_i * T * dt;
    // angular damping: a little in flight; on the ground a shard loses its spin fast (it tumbles, chatters and rolls
    // on an edge otherwise: the penalty contacts give back the energy a real impact swallows)
    r.w *= 1.0f - (touching ? 12.0f : 0.2f) * dt;
    const float w2 = dot(r.w, r.w);
    if (w2 > 400.0f * 400.0f) r.w *= 400.0f / std::sqrt(w2);
    float v2 = dot(r.v, r.v);
    if (v2 > kMaxNodeSpeed * kMaxNodeSpeed) r.v *= kMaxNodeSpeed / std::sqrt(v2);
    if (!(v2 >= 0.0f) || !(w2 >= 0.0f)) r.v = r.w = vec3(0);
    r.com += r.v * dt;
    const quat dq{r.w.x, r.w.y, r.w.z, 0};
    r.q = normalize(r.q + (dq * r.q) * (0.5f * dt));
    const mat3 R2 = to_mat3(r.q);
    vec3 mn = mn_out, mx = mx_out;
    float max_v2 = max_v2_out;
    for (size_t i = 0; i < nodes.size(); i++) {
        Node& n = nodes[i];
        const vec3 d = R2 * r.local[i];
        n.p = r.com + d;
        n.v = r.v + cross(r.w, d);
        max_v2 = std::max(max_v2, dot(n.v, n.v));
        mn = vmin(mn, n.p);
        mx = vmax(mx, n.p);
    }
    mn_out = mn;
    mx_out = mx;
    max_v2_out = max_v2;
}

quat SoftBody::triad_orientation(uint32_t node, int32_t ref_x, int32_t ref_y) const {
    const vec3 p = nodes[node].p;
    const vec3 e1 = normalize_or(nodes[ref_x].p - p, vec3(1, 0, 0));
    vec3 e3 = normalize_or(cross(e1, nodes[ref_y].p - p), vec3(0, 0, 1));
    if (length2(cross(e1, e3)) < 1e-12f) e3 = normalize_or(cross(e1, vec3(0, 1, 0)), vec3(0, 0, 1));
    const vec3 e2 = cross(e3, e1);
    return normalize(from_mat3(mat3(e1, e2, e3)));
}

void SoftBody::integrate_frames(float dt) {
    for (Frame& fr : frames) {
        if (fr.ref_x >= 0 && fr.ref_y >= 0 && fr.ref_x < (int32_t)nodes.size() && fr.ref_y < (int32_t)nodes.size()) {
            fr.q = triad_orientation(fr.node, fr.ref_x, fr.ref_y);
            fr.w = vec3(0);
            fr.torque = vec3(0);
            continue;
        }
        if (fr.inv_inertia <= 0) continue;
        fr.w += fr.torque * (fr.inv_inertia * dt);
        fr.w *= (1.0f - 0.5f * dt); // tiny angular damping for robustness
        vec3 w = fr.w;
        quat dq{w.x, w.y, w.z, 0};
        quat q = fr.q + (dq * fr.q) * (0.5f * dt);
        fr.q = normalize(q);
    }
}

void SoftBody::integrate(float dt) {
    vec3 mn(1e30f), mx(-1e30f);
    float max_v2 = 0;
    integrate_nodes(0, nodes.size(), dt, mn, mx, max_v2);
    aabb.mn = mn;
    aabb.mx = mx;
    float ms = std::sqrt(max_v2);
    if (ms > max_speed) max_speed = ms;
    integrate_frames(dt);
}

} // namespace bl::phys
