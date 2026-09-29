// Builds a physics SoftBody from a parsed RoR definition following ActorSpawner rules
// (node numbering, wheel generation, beam defaults, shocks, node masses, collision cabs).
#include "vehicle/builder.h"
#include "core/profiler.h"
#include "core/util.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace bl {

using namespace phys;

// Incremental 3D convex hull (small point sets). Returns CCW-outward triangles as point indices.
std::vector<std::array<int, 3>> convex_hull(const std::vector<vec3>& pts) {
    std::vector<std::array<int, 3>> faces;
    const int n = (int)pts.size();
    if (n < 4) return faces;
    // initial tetrahedron
    int i0 = 0, i1 = -1, i2 = -1, i3 = -1;
    for (int i = 1; i < n; i++)
        if (length2(pts[i] - pts[i0]) > 1e-6f) { i1 = i; break; }
    if (i1 < 0) return faces;
    for (int i = 0; i < n; i++)
        if (length2(cross(pts[i1] - pts[i0], pts[i] - pts[i0])) > 1e-8f) { i2 = i; break; }
    if (i2 < 0) return faces;
    vec3 nrm = cross(pts[i1] - pts[i0], pts[i2] - pts[i0]);
    float best = 0;
    for (int i = 0; i < n; i++) {
        float d = std::fabs(dot(pts[i] - pts[i0], nrm));
        if (d > best) { best = d; i3 = i; }
    }
    if (i3 < 0 || best < 1e-8f) return faces;
    auto outward = [&](std::array<int, 3> f, vec3 inside) {
        vec3 fn = cross(pts[f[1]] - pts[f[0]], pts[f[2]] - pts[f[0]]);
        if (dot(fn, inside - pts[f[0]]) > 0) std::swap(f[1], f[2]);
        return f;
    };
    vec3 c = (pts[i0] + pts[i1] + pts[i2] + pts[i3]) * 0.25f;
    faces = {outward({i0, i1, i2}, c), outward({i0, i1, i3}, c), outward({i0, i2, i3}, c), outward({i1, i2, i3}, c)};
    for (int p = 0; p < n; p++) {
        if (p == i0 || p == i1 || p == i2 || p == i3) continue;
        std::vector<bool> visible(faces.size());
        bool any = false;
        for (size_t f = 0; f < faces.size(); f++) {
            const auto& F = faces[f];
            vec3 fn = cross(pts[F[1]] - pts[F[0]], pts[F[2]] - pts[F[0]]);
            visible[f] = dot(fn, pts[p] - pts[F[0]]) > 1e-6f * length(fn);
            any |= visible[f];
        }
        if (!any) continue;
        std::map<std::pair<int, int>, int> edges;
        for (size_t f = 0; f < faces.size(); f++)
            if (visible[f])
                for (int e = 0; e < 3; e++) edges[{faces[f][e], faces[f][(e + 1) % 3]}]++;
        std::vector<std::array<int, 3>> nf;
        for (size_t f = 0; f < faces.size(); f++)
            if (!visible[f]) nf.push_back(faces[f]);
        for (auto& [e, cnt] : edges)
            if (!edges.count({e.second, e.first})) nf.push_back({e.first, e.second, p});
        faces.swap(nf);
    }
    return faces;
}

namespace {

bool has_opt(const std::string& s, char c) { return s.find(c) != std::string::npos; }

// Ogre Vector3::perpendicular
vec3 ogre_perpendicular(vec3 a) {
    vec3 p = cross(a, vec3(1, 0, 0));
    if (length2(p) < 1e-12f) p = cross(a, vec3(0, 1, 0));
    return normalize(p);
}


} // namespace

// ================================================================================================
bool VehicleBuilder::build(const ror::Document& d, SoftBody& body, Drivetrain& drive, std::vector<std::string>& warnings) {
    PROFILE_ZONE("Vehicle build");
    const int N = (int)d.nodes.size();
    if (N == 0) {
        warnings.push_back("no nodes");
        return false;
    }
    std::vector<vec3> pos(N);
    std::vector<uint16_t> flags(N, NF_GROUND);
    std::vector<float> friction(N, 1.0f);
    std::vector<bool> tyre(N, false), loaded(N, false), override_w(N, false), virtual_node(N, false);
    std::vector<float> weight(N, 0.0f), minimass(N, d.minimass);
    std::vector<float> fixed_mass(N, -1.0f);

    // ---- explicit and cinecam nodes
    for (int i = 0; i < N; i++) {
        const auto& s = d.nodes[i];
        if (s.kind == ror::NodeSlot::EXPLICIT) {
            const auto& nd = d.nodes_explicit[s.ref];
            pos[i] = nd.pos;
            friction[i] = nd.defaults.friction > 0 ? nd.defaults.friction : 1.0f;
            if (has_opt(nd.options, 'c')) flags[i] &= ~NF_GROUND;
            if (has_opt(nd.options, 'm')) flags[i] |= NF_NO_MOUSE;
            if (nd.loaded) {
                loaded[i] = true;
                flags[i] |= NF_LOAD;
                if (nd.load_weight >= 0) {
                    override_w[i] = true;
                    weight[i] = nd.load_weight;
                }
            }
            if (nd.minimass >= 0) minimass[i] = nd.minimass;
        } else if (s.kind == ror::NodeSlot::CINECAM) {
            const auto& cc = d.cinecams[s.ref];
            pos[i] = cc.pos;
            flags[i] = NF_NO_MOUSE; // no ground contact
            fixed_mass[i] = cc.node_mass > 0 ? cc.node_mass : 20.0f;
        }
    }
    for (int f : d.fixes)
        if (f >= 0 && f < N) flags[f] |= NF_FIXED;

    struct PendingBeam {
        int a, b;
        float k, d, strength, deform, plastic;
        uint8_t type = BT_NORMAL;
        uint8_t flags = 0;
        bool virt = false;       // excluded from mass distribution
        float refL = -1;         // rest length override (shocks precompression)
        int shock = -1;          // index into pending shocks
        float support_limit = 4.0f;
        int group = 0;
    };
    std::vector<PendingBeam> pb;
    std::vector<Shock> shocks;
    auto add_beam = [&](int a, int b, float k, float dmp, float strength, float deform, float plastic) -> PendingBeam& {
        PendingBeam x{a, b, k, dmp, strength, deform, plastic};
        pb.push_back(x);
        return pb.back();
    };

    // ---- wheels: generate nodes + beams
    body.wheels.clear();
    for (const auto& w : d.wheels) {
        int A = w.n1, B = w.n2;
        if (A < 0 || B < 0 || A >= N || B >= N) continue;
        if (pos[A].z > pos[B].z) std::swap(A, B); // RoR: axis node 0 has the smaller z
        vec3 axis = normalize(pos[B] - pos[A]);
        const int R = std::max(3, w.rays);
        const int base = w.first_node;
        Wheel wh;
        wh.tag = (int)(&w - &d.wheels[0]);
        wh.axle0 = A;
        wh.axle1 = B;
        wh.radius = w.radius;
        wh.rim_radius = w.rim_radius;
        wh.width = length(pos[B] - pos[A]);
        wh.braked = (w.braking >= 0 && w.braking <= 4) ? w.braking : 0;
        wh.propulsed = (w.propulsion == 1 || w.propulsion == 2) ? w.propulsion : 0;
        wh.arm = w.arm;
        if (w.arm >= 0 && w.arm < N) wh.near_attach = length2(pos[w.arm] - pos[A]) < length2(pos[w.arm] - pos[B]) ? A : B;
        float node_fric = w.nd.friction > 0 ? w.nd.friction : 1.0f;
        const auto& bd = w.bd;
        float strength = bd.brk; // unscaled
        float deform = bd.deform_threshold();
        auto set_node = [&](int idx, vec3 p, float mass, bool is_tyre, float fric) {
            if (idx < 0 || idx >= N) return;
            pos[idx] = p;
            flags[idx] = NF_GROUND | NF_CONTACTER | (is_tyre ? NF_TYRE : NF_RIM);
            friction[idx] = fric;
            if (is_tyre) {
                tyre[idx] = true;
                fixed_mass[idx] = mass;
            }
        };
        auto spoke_shock = [&](int a, int b, float k, float dmp, float sb, float lb) {
            PendingBeam& x = add_beam(a, b, k, dmp, strength, deform, bd.plastic);
            Shock s;
            s.type = 1;
            s.spring = k;
            s.damp = dmp;
            s.short_bound = sb;
            s.long_bound = lb;
            s.bound_spring = 9000000.0f; // NORMAL beams stiffen towards DEFAULT_SPRING/DAMP
            s.bound_damp = 12000.0f;
            x.shock = (int)shocks.size();
            shocks.push_back(s);
        };
        auto rigid_beams = [&](const std::vector<int>& ring_a, const std::vector<int>& ring_b, float k, float dmp) {
            if (w.rigidity < 0 || w.rigidity >= N) return;
            bool near_a = length2(pos[w.rigidity] - pos[A]) < length2(pos[w.rigidity] - pos[B]);
            for (int i = 0; i < R; i++) {
                PendingBeam& x = add_beam(w.rigidity, near_a ? ring_a[i] : ring_b[i], k, dmp, strength, deform, bd.plastic);
                x.virt = true;
                x.flags |= BF_INVISIBLE;
            }
        };
        if (w.type == ror::WheelDef::WHEELS || w.type == ror::WheelDef::MESHWHEELS || w.type == ror::WheelDef::MESHWHEELS2) {
            vec3 ray = ogre_perpendicular(axis) * w.radius;
            quat rot = quat::axis_angle(axis, -2.0f * kPi / (2 * R));
            std::vector<int> outer(R), inner(R);
            float nm = w.mass / (2.0f * R);
            for (int i = 0; i < R; i++) {
                outer[i] = base + 2 * i;
                inner[i] = base + 2 * i + 1;
                set_node(outer[i], pos[A] + ray, nm, true, node_fric);
                ray = rot.rotate(ray);
                set_node(inner[i], pos[B] + ray, nm, true, node_fric);
                ray = rot.rotate(ray);
                wh.nodes.push_back(outer[i]);
                wh.nodes.push_back(inner[i]);
            }
            float kt = w.spring, dt_ = w.damp;
            float kr = w.spring, dr = w.damp;
            float lext = 0.0f;
            if (w.type == ror::WheelDef::MESHWHEELS2) {
                kr = bd.spring;
                dr = bd.damp;
                lext = 0.15f;
            }
            for (int i = 0; i < R; i++) {
                int j = (i + 1) % R;
                spoke_shock(A, outer[i], kt, dt_, 0.66f, lext);
                spoke_shock(B, inner[i], kt, dt_, 0.66f, lext);
                add_beam(B, outer[i], kt, dt_, strength, deform, bd.plastic);
                add_beam(A, inner[i], kt, dt_, strength, deform, bd.plastic);
                add_beam(outer[i], inner[i], kr, dr, strength, deform, bd.plastic);
                add_beam(outer[i], outer[j], kr, dr, strength, deform, bd.plastic);
                add_beam(inner[i], inner[j], kr, dr, strength, deform, bd.plastic);
                add_beam(inner[i], outer[j], kr, dr, strength, deform, bd.plastic);
            }
            rigid_beams(outer, inner, kt, dt_);
            wh.type = w.type == ror::WheelDef::WHEELS ? Wheel::W_WHEELS : (w.type == ror::WheelDef::MESHWHEELS ? Wheel::W_MESHWHEELS : Wheel::W_MESHWHEELS2);
        } else if (w.type == ror::WheelDef::WHEELS2) {
            std::vector<int> ro(R), ri(R), to(R), ti(R);
            vec3 rim_ray(0, w.rim_radius, 0);
            quat rot = quat::axis_angle(axis, -2.0f * kPi / R);
            for (int i = 0; i < R; i++) {
                ro[i] = base + 2 * i;
                ri[i] = base + 2 * i + 1;
                set_node(ro[i], pos[A] + rim_ray, 0, false, node_fric);
                set_node(ri[i], pos[B] + rim_ray, 0, false, node_fric);
                rim_ray = rot.rotate(rim_ray);
            }
            vec3 tyre_ray = quat::axis_angle(axis, -kPi / R).rotate(vec3(0, w.radius, 0));
            for (int i = 0; i < R; i++) {
                to[i] = base + 2 * R + 2 * i;
                ti[i] = base + 2 * R + 2 * i + 1;
                set_node(to[i], pos[A] + tyre_ray, 0.67f * w.mass / (2.0f * R), true, 2.0f);
                set_node(ti[i], pos[B] + tyre_ray, 0.33f * w.mass / (2.0f * R), true, 2.0f);
                tyre_ray = rot.rotate(tyre_ray);
                wh.nodes.push_back(to[i]);
                wh.nodes.push_back(ti[i]);
                wh.rim.push_back(ro[i]);
                wh.rim.push_back(ri[i]);
            }
            float kr = w.rim_spring, dr = w.rim_damp, kt = w.spring, dt_ = w.damp;
            for (int i = 0; i < R; i++) {
                int j = (i + 1) % R;
                add_beam(A, ro[i], kr, dr, strength, deform, bd.plastic);
                add_beam(B, ri[i], kr, dr, strength, deform, bd.plastic);
                add_beam(B, ro[i], kr, dr, strength, deform, bd.plastic);
                add_beam(A, ri[i], kr, dr, strength, deform, bd.plastic);
                add_beam(ro[i], ri[i], kr, dr, strength, deform, bd.plastic);
                add_beam(ro[i], ro[j], kr, dr, strength, deform, bd.plastic);
                add_beam(ri[i], ri[j], kr, dr, strength, deform, bd.plastic);
                add_beam(ro[i], ri[j], kr, dr, strength, deform, bd.plastic);
                add_beam(ri[i], ro[j], kr, dr, strength, deform, bd.plastic);
                // tyre
                add_beam(to[i], to[j], kt, dt_, strength, deform, bd.plastic);
                add_beam(to[i], ti[j], kt, dt_, strength, deform, bd.plastic);
                add_beam(ti[i], to[j], kt, dt_, strength, deform, bd.plastic);
                add_beam(ti[i], ti[j], kt, dt_, strength, deform, bd.plastic);
                add_beam(to[i], ro[i], kt, dt_, strength, deform, bd.plastic);
                add_beam(to[i], ro[j], kt, dt_, strength, deform, bd.plastic);
                add_beam(ti[i], ri[i], kt, dt_, strength, deform, bd.plastic);
                add_beam(ti[i], ri[j], kt, dt_, strength, deform, bd.plastic);
                add_beam(to[i], ri[i], kt, dt_, strength, deform, bd.plastic);
                add_beam(to[i], ri[j], kt, dt_, strength, deform, bd.plastic);
                add_beam(ti[i], ro[i], kt, dt_, strength, deform, bd.plastic);
                add_beam(ti[i], ro[j], kt, dt_, strength, deform, bd.plastic);
                add_beam(A, to[i], kt, dt_, strength, deform, bd.plastic);
                add_beam(B, ti[i], kt, dt_, strength, deform, bd.plastic);
            }
            rigid_beams(ro, ri, kr, dr);
            wh.type = Wheel::W_WHEELS2;
        } else { // FLEXBODYWHEELS
            std::vector<int> ro(R), ri(R), to(R), ti(R);
            quat step = quat::axis_angle(axis, -2.0f * kPi / (2 * R));
            vec3 perp = ogre_perpendicular(axis);
            vec3 rim_ray = perp * w.rim_radius;
            float nm = w.mass / (4.0f * R);
            for (int i = 0; i < R; i++) {
                ro[i] = base + 2 * i;
                ri[i] = base + 2 * i + 1;
                set_node(ro[i], pos[A] + rim_ray, 0, false, node_fric);
                rim_ray = step.rotate(rim_ray);
                set_node(ri[i], pos[B] + rim_ray, 0, false, node_fric);
                rim_ray = step.rotate(rim_ray);
            }
            vec3 tyre_ray = step.rotate(perp * w.radius);
            for (int i = 0; i < R; i++) {
                to[i] = base + 2 * R + 2 * i;
                ti[i] = base + 2 * R + 2 * i + 1;
                set_node(to[i], pos[A] + tyre_ray, nm, true, node_fric);
                tyre_ray = step.rotate(tyre_ray);
                set_node(ti[i], pos[B] + tyre_ray, nm, true, node_fric);
                tyre_ray = step.rotate(tyre_ray);
                wh.nodes.push_back(to[i]);
                wh.nodes.push_back(ti[i]);
                wh.rim.push_back(ro[i]);
                wh.rim.push_back(ri[i]);
            }
            float kr = w.rim_spring, dr = w.rim_damp, kt = w.spring, dt_ = w.damp;
            float sb = 1.0f - 0.95f * w.rim_radius / std::max(0.01f, w.radius);
            for (int i = 0; i < R; i++) {
                int j = (i + 1) % R, p = (i + R - 1) % R;
                add_beam(A, ro[i], kr, dr, strength, deform, bd.plastic);
                add_beam(B, ri[i], kr, dr, strength, deform, bd.plastic);
                add_beam(B, ro[i], kr, dr, strength, deform, bd.plastic);
                add_beam(A, ri[i], kr, dr, strength, deform, bd.plastic);
                add_beam(ro[i], ri[i], kr, dr, strength, deform, bd.plastic);
                add_beam(ro[i], ro[j], kr, dr, strength, deform, bd.plastic);
                add_beam(ri[i], ri[j], kr, dr, strength, deform, bd.plastic);
                add_beam(ri[i], ro[j], kr, dr, strength, deform, bd.plastic);
                add_beam(ro[i], to[i], kt * 0.5f, dt_, strength, deform, bd.plastic);
                add_beam(ro[i], ti[p], kt * 0.5f, dt_, strength, deform, bd.plastic);
                add_beam(ro[i], to[p], kt * 0.5f, dt_, strength, deform, bd.plastic);
                add_beam(ri[i], to[i], kt * 0.5f, dt_, strength, deform, bd.plastic);
                add_beam(ri[i], ti[i], kt * 0.5f, dt_, strength, deform, bd.plastic);
                add_beam(ri[i], ti[p], kt * 0.5f, dt_, strength, deform, bd.plastic);
                add_beam(to[i], ti[i], bd.spring, bd.damp, strength, deform, bd.plastic);
                add_beam(to[i], to[j], bd.spring, bd.damp, strength, deform, bd.plastic);
                add_beam(ti[i], ti[j], bd.spring, bd.damp, strength, deform, bd.plastic);
                add_beam(ti[i], to[j], bd.spring, bd.damp, strength, deform, bd.plastic);
                spoke_shock(A, to[i], kt * 0.5f, dt_, sb, 0.0f);
                spoke_shock(B, ti[i], kt * 0.5f, dt_, sb, 0.0f);
            }
            rigid_beams(to, ti, kt, dt_);
            wh.type = Wheel::W_FLEXBODY;
        }
        body.wheels.push_back(std::move(wh));
    }

    // ---- beams (the frame elements, option F, are kept for the FEM frame below)
    std::vector<int> frame_beams;
    std::vector<char> frame_node(N, 0);
    for (size_t bi = 0; bi < d.beams.size(); bi++) {
        const auto& b = d.beams[bi];
        if (b.n1 == b.n2) continue;
        if (b.frame >= 0 && b.frame < (int)d.frame_sections.size() && b.n1 >= 0 && b.n2 >= 0 && b.n1 < N && b.n2 < N) {
            frame_beams.push_back((int)bi);
            frame_node[b.n1] = frame_node[b.n2] = 1;
            continue;
        }
        const auto& bd = b.bd;
        PendingBeam& x = add_beam(b.n1, b.n2, bd.k(), bd.d(), bd.break_scaled(), bd.deform_threshold(), bd.plastic);
        x.group = b.detacher_group;
        if (has_opt(b.options, 'i')) x.flags |= BF_INVISIBLE;
        if (has_opt(b.options, 'r')) x.type = BT_ROPE;
        else if (has_opt(b.options, 's')) {
            x.type = BT_SUPPORT;
            x.support_limit = b.support_limit > 0 ? b.support_limit : 4.0f;
        }
    }
    // ---- cinecam beams
    for (int i = 0; i < N; i++) {
        const auto& s = d.nodes[i];
        if (s.kind != ror::NodeSlot::CINECAM) continue;
        const auto& cc = d.cinecams[s.ref];
        for (int k = 0; k < 8; k++) {
            if (cc.nodes[k] == i) continue;
            PendingBeam& x = add_beam(i, cc.nodes[k], cc.spring, cc.damp, cc.bd.brk, cc.bd.deform_threshold(), cc.bd.plastic);
            x.flags |= BF_INVISIBLE | BF_NO_DEFORM;
        }
    }
    // ---- shocks
    for (const auto& s : d.shocks) {
        if (s.n1 == s.n2) continue;
        float spawn_len = length(pos[s.n1] - pos[s.n2]);
        Shock sh;
        sh.type = (uint8_t)s.type;
        float sbd_spring = s.bd.spring, sbd_damp = s.bd.damp;
        sh.bound_spring = sbd_spring;
        sh.bound_damp = sbd_damp;
        float sb = s.shortbound, lb = s.longbound;
        if (has_opt(s.options, 'm') && spawn_len > 0) {
            sb /= spawn_len;
            lb /= spawn_len;
        }
        if (s.type >= 2 && has_opt(s.options, 'M') && spawn_len > 0) {
            sb = std::min(1.0f, (spawn_len - s.shortbound) / spawn_len);
            lb = std::max(0.0f, (s.longbound - spawn_len) / spawn_len);
        }
        sh.short_bound = sb;
        sh.long_bound = lb;
        float k, dmp;
        if (s.type == 1) {
            sh.spring = k = s.spring;
            sh.damp = dmp = s.damp;
        } else {
            k = s.spring_in;
            dmp = s.damp_in;
            sh.spring_in = s.spring_in;
            sh.damp_in = s.damp_in;
            sh.prog_spring_in = s.prog_spring_in;
            sh.prog_damp_in = s.prog_damp_in;
            sh.spring_out = s.spring_out;
            sh.damp_out = s.damp_out;
            sh.prog_spring_out = s.prog_spring_out;
            sh.prog_damp_out = s.prog_damp_out;
            sh.damp_in_slow = s.damp_in_slow;
            sh.split_in = s.split_vel_in > 0 ? s.split_vel_in : 1;
            sh.damp_in_fast = s.damp_in_fast;
            sh.damp_out_slow = s.damp_out_slow;
            sh.split_out = s.split_vel_out > 0 ? s.split_vel_out : 1;
            sh.damp_out_fast = s.damp_out_fast;
            sh.soft_bump = s.type == 2 && has_opt(s.options, 's');
            if (s.type == 3) sh.spring = s.spring_in;
        }
        PendingBeam& x = add_beam(s.n1, s.n2, k, dmp, 4.0f * s.bd.brk, s.bd.deform_threshold(), 0);
        x.flags |= BF_NO_DEFORM;
        if (has_opt(s.options, 'i')) x.flags |= BF_INVISIBLE;
        x.refL = spawn_len * (s.precompression > 0 ? s.precompression : 1.0f);
        x.shock = (int)shocks.size();
        shocks.push_back(sh);
    }
    // ---- hydros
    std::vector<std::pair<int, HydroCtl>> hyd;
    for (const auto& h : d.hydros) {
        if (h.n1 == h.n2) continue;
        PendingBeam& x = add_beam(h.n1, h.n2, h.bd.k(), h.bd.d(), h.bd.break_scaled(), h.bd.deform_threshold(), 0);
        x.flags |= BF_NO_DEFORM | BF_HYDRO;
        const std::string& o = h.options;
        HydroCtl c{0, h.factor, true, false};
        bool airplane = o.find_first_of("aeruvxygh") != std::string::npos;
        bool j = has_opt(o, 'j');
        c.steer = !airplane && !j;
        if (has_opt(o, 'n') || (!o.empty() && o[0] == 'i')) c.steer = true;
        c.speed_dep = has_opt(o, 's');
        if (has_opt(o, 'i') || j) x.flags |= BF_INVISIBLE;
        hyd.push_back({(int)pb.size() - 1, c});
    }
    // ---- commands
    std::vector<std::pair<int, CommandCtl>> cmds;
    for (const auto& c : d.commands) {
        if (c.n1 == c.n2) continue;
        PendingBeam& x = add_beam(c.n1, c.n2, c.bd.k(), c.bd.d(), c.bd.break_scaled(), c.bd.deform_threshold(), 0);
        x.flags |= BF_NO_DEFORM | BF_HYDRO;
        if (has_opt(c.options, 'r')) x.type = BT_ROPE;
        if (has_opt(c.options, 'i')) x.flags |= BF_INVISIBLE;
        CommandCtl cc{0, c.rate_short, c.rate_long, c.shortbound, c.longbound, c.key_contract, c.key_extend, has_opt(c.options, 'c')};
        cmds.push_back({(int)pb.size() - 1, cc});
    }
    // ---- ropes
    std::set<int> rope_ends;
    for (const auto& r : d.ropes) {
        if (r.root == r.end) continue;
        PendingBeam& x = add_beam(r.root, r.end, r.bd.k(), r.bd.d(), r.bd.break_scaled(), r.bd.deform_threshold(), 0);
        x.type = BT_ROPE;
        x.flags |= BF_NO_DEFORM;
        rope_ends.insert(r.end);
    }

    // ---- node masses (RoR Actor::recalculateNodeMasses)
    std::vector<float> mass(N, 0.0f);
    int masscount = 0;
    for (int i = 0; i < N; i++)
        if (!tyre[i] && loaded[i] && !override_w[i]) masscount++;
    float cargo = std::max(0.0f, d.cargo_mass);
    for (int i = 0; i < N; i++) {
        if (tyre[i]) continue;
        if (!loaded[i]) mass[i] = 0;
        else if (!override_w[i]) mass[i] = masscount > 0 ? cargo / masscount : 0;
        else mass[i] = weight[i];
    }
    double total_len = 0;
    for (const auto& x : pb) {
        if (x.virt || x.a < 0 || x.b < 0 || x.a >= N || x.b >= N) continue;
        float L = x.refL > 0 ? x.refL : length(pos[x.a] - pos[x.b]);
        if (!tyre[x.a]) total_len += L * 0.5;
        if (!tyre[x.b]) total_len += L * 0.5;
    }
    if (total_len > 0)
        for (const auto& x : pb) {
            if (x.virt || x.a < 0 || x.b < 0 || x.a >= N || x.b >= N) continue;
            float L = x.refL > 0 ? x.refL : length(pos[x.a] - pos[x.b]);
            float half = (float)(L * d.dry_mass / total_len * 0.5);
            if (!tyre[x.a]) mass[x.a] += half;
            if (!tyre[x.b]) mass[x.b] += half;
        }
    for (int e : rope_ends)
        if (e >= 0 && e < N) mass[e] = 100.0f;
    // frame elements: their share of the dry mass (they are the structure's members, as beams are in RoR, so they
    // count when no beams do) and on top their section's own mass, rho A L, half at each end
    std::vector<phys::FrameSection> frame_secs;
    std::vector<float> frame_mass(d.beams.size(), 0.0f); // (per frame element: the mass it puts on its nodes)
    for (const auto& fs : d.frame_sections) frame_secs.push_back(phys::make_frame_section(fs.material, phys::frame_shape(fs.shape), fs.outer, fs.wall));
    {
        double flen = 0;
        for (int bi : frame_beams) flen += length(pos[d.beams[bi].n1] - pos[d.beams[bi].n2]);
        const double all = total_len + flen;
        for (int bi : frame_beams) {
            const auto& b = d.beams[bi];
            const float L = length(pos[b.n1] - pos[b.n2]);
            const float share = all > 0 ? (float)(L * d.dry_mass / all * 0.5) : 0.0f;
            const float own = frame_secs[b.frame].mass_per_m() * L * 0.5f;
            if (!tyre[b.n1]) mass[b.n1] += share + own;
            if (!tyre[b.n2]) mass[b.n2] += share + own;
            frame_mass[bi] = 2.0f * (share + own);
        }
        // triangle elements (fem_tris): their own mass, rho t A, a third at each corner; the dry mass over them by their
        // area when nothing else carries it (a body that is a shell alone)
        double tri_area = 0;
        for (const auto& t : d.fem_tris) {
            if (t.n1 < 0 || t.n2 < 0 || t.n3 < 0 || t.n1 >= N || t.n2 >= N || t.n3 >= N || t.shell < 0 || t.shell >= (int)d.fem_shells.size()) continue;
            const float A = 0.5f * length(cross(pos[t.n2] - pos[t.n1], pos[t.n3] - pos[t.n1]));
            const auto& sh = d.fem_shells[t.shell];
            const float own = phys::make_shell_section(sh.material, sh.thickness).mass_per_m2() * A / 3.0f;
            for (int v : {t.n1, t.n2, t.n3}) mass[v] += own, frame_node[v] = 1;
            tri_area += A;
        }
        if (all <= 0 && tri_area > 0)
            for (const auto& t : d.fem_tris) {
                if (t.n1 < 0 || t.n2 < 0 || t.n3 < 0 || t.n1 >= N || t.n2 >= N || t.n3 >= N) continue;
                const float A = 0.5f * length(cross(pos[t.n2] - pos[t.n1], pos[t.n3] - pos[t.n1]));
                for (int v : {t.n1, t.n2, t.n3}) mass[v] += (float)(d.dry_mass * A / tri_area / 3.0);
            }
        // (the plain beams' share of the dry mass was given out against their length alone: scaled down to theirs)
        if (flen > 0 && total_len > 0)
            for (const auto& x : pb) {
                if (x.virt || x.a < 0 || x.b < 0 || x.a >= N || x.b >= N) continue;
                const float L = x.refL > 0 ? x.refL : length(pos[x.a] - pos[x.b]);
                const float over = (float)(L * d.dry_mass / total_len * 0.5) - (float)(L * d.dry_mass / all * 0.5);
                if (!tyre[x.a]) mass[x.a] -= over;
                if (!tyre[x.b]) mass[x.b] -= over;
            }
    }
    // (a node held by frame elements alone needs no minimum mass: the frame is solved implicitly)
    std::vector<char> plain_node(N, 0);
    for (const auto& x : pb)
        if (x.a >= 0 && x.b >= 0 && x.a < N && x.b < N) plain_node[x.a] = plain_node[x.b] = 1;
    for (int i = 0; i < N; i++) {
        if (tyre[i]) {
            mass[i] = fixed_mass[i] > 0 ? fixed_mass[i] : 1.0f;
            continue;
        }
        if (fixed_mass[i] > 0) mass[i] = fixed_mass[i]; // cinecam
        const bool frame_only = frame_node[i] && !plain_node[i];
        if (!frame_only && !(d.minimass_skip_loaded && loaded[i]) && mass[i] < minimass[i]) mass[i] = minimass[i];
        if (mass[i] <= 0) mass[i] = 1.0f;
    }

    // ---- collision cabs / contact flags
    std::vector<bool> cab_node(N, false);
    int ncoll = 0;
    for (const auto& sm : d.submeshes)
        for (const auto& c : sm.cabs) {
            if (c.options.find_first_of("cpuDFSh") == std::string::npos) continue;
            if (c.n1 == c.n2 || c.n2 == c.n3 || c.n1 == c.n3) continue;
            if (c.n1 >= N || c.n2 >= N || c.n3 >= N) continue;
            // (option h: a hull triangle - one-sided and solid, see Triangle::two_sided; it tears when an edge is stretched
            // 2.5 times, a car cut in two leaves no triangle across the gap)
            const bool hull = c.options.find('h') != std::string::npos;
            body.tris.push_back({(uint32_t)c.n1, (uint32_t)c.n2, (uint32_t)c.n3, SURF_METAL, !hull, false, hull ? max_edge2(pos[c.n1], pos[c.n2], pos[c.n3]) : 0.0f});
            cab_node[c.n1] = cab_node[c.n2] = cab_node[c.n3] = true;
            ncoll++;
        }
    // (the triangle elements are collision triangles too: two-sided; torn with them)
    std::vector<int32_t> fem_tri_coll(d.fem_tris.size(), -1);
    for (size_t ti = 0; ti < d.fem_tris.size(); ti++) {
        const auto& t = d.fem_tris[ti];
        if (t.n1 < 0 || t.n2 < 0 || t.n3 < 0 || t.n1 >= N || t.n2 >= N || t.n3 >= N || t.n1 == t.n2 || t.n2 == t.n3 || t.n1 == t.n3) continue;
        fem_tri_coll[ti] = (int32_t)body.tris.size();
        body.tris.push_back({(uint32_t)t.n1, (uint32_t)t.n2, (uint32_t)t.n3, SURF_METAL, true, false, max_edge2(pos[t.n1], pos[t.n2], pos[t.n3])});
        cab_node[t.n1] = cab_node[t.n2] = cab_node[t.n3] = true;
        ncoll++;
    }
    for (int c : d.contacters)
        if (c >= 0 && c < N) flags[c] |= NF_CONTACTER;
    for (int i = 0; i < N; i++) {
        bool ground = (flags[i] & NF_GROUND) != 0;
        if (flags[i] & (NF_TYRE | NF_RIM)) continue;
        bool contactable = ground && (cab_node[i] || ncoll == 0);
        if (contactable) flags[i] |= NF_CONTACTER;
        else if (!(flags[i] & NF_CONTACTER)) flags[i] &= ~NF_CONTACTER;
    }
    if (ncoll == 0) {
        // Fallback: vehicles without collision cabs get a convex hull shell made of their own nodes so
        // they can still be hit by other bodies (RoR has no inter-vehicle collision for them).
        std::vector<vec3> pts;
        std::vector<int> ids;
        for (int i = 0; i < N; i++)
            if ((flags[i] & NF_GROUND) && !tyre[i] && !(flags[i] & NF_RIM)) {
                pts.push_back(pos[i]);
                ids.push_back(i);
            }
        auto hull = convex_hull(pts);
        for (auto& f : hull) body.tris.push_back({(uint32_t)ids[f[0]], (uint32_t)ids[f[1]], (uint32_t)ids[f[2]], SURF_METAL, true});
        if (!hull.empty()) warnings.push_back(format("no collision cabs: generated a %d-triangle convex hull shell", (int)hull.size()));
    }

    // ---- emit nodes
    body.nodes.clear();
    body.info.clear();
    body.force.clear();
    for (int i = 0; i < N; i++) {
        uint32_t id = body.add_node(pos[i], mass[i], flags[i]);
        body.info[id].friction = friction[i];
        body.info[id].ror_id = i;
        if (flags[i] & NF_FIXED) {
            body.nodes[id].inv_mass = 0;
            body.nodes[id].mass = mass[i];
        }
    }
    for (auto& w : body.wheels) {
        w.mass = 0;
        for (uint32_t ni : w.nodes) w.mass += body.nodes[ni].mass;
        // contact radius of the tread nodes ~1.5x the sag of the polygon chord: the patch always has 2-3 nodes
        // (a bare node ring alternates between one vertex and one edge on the ground and slips at each change)
        int rays = std::max(3, (int)w.nodes.size() / 2);
        float sag = w.radius * (1.0f - std::cos(kPi / rays));
        for (uint32_t ni : w.nodes) body.info[ni].radius = 1.5f * sag + 0.005f;
    }
    // ---- emit beams
    std::vector<int> beam_index(pb.size(), -1);
    for (size_t i = 0; i < pb.size(); i++) {
        const auto& x = pb[i];
        if (x.a < 0 || x.b < 0 || x.a >= N || x.b >= N || x.a == x.b) continue;
        uint32_t bi = body.add_beam(x.a, x.b, x.k, x.d, x.strength, x.deform, x.type, x.flags);
        Beam& bm = body.beams[bi];
        bm.plastic = x.plastic;
        bm.support_limit = x.support_limit;
        bm.group = (uint16_t)std::max(0, x.group);
        if (x.refL > 0) bm.L = bm.L0 = x.refL;
        if (bm.L < 1e-3f) bm.L = bm.L0 = 1e-3f;
        if (x.shock >= 0) {
            bm.flags |= BF_SHOCK;
            Shock s = shocks[x.shock];
            s.beam = bi;
            body.shocks.push_back(s);
        }
        beam_index[i] = (int)bi;
    }
    // ---- joints (BeamLab): the child node is held at its rest offset in the parent node's frame, a bending-stiff
    // link without bracing. The parent's frame is kinematic: the triad of the parent and two of its beam
    // neighbours (not the child), so it turns with the structure; a parent with no such neighbours keeps its world
    // orientation. The stiffness takes a small share of the nodes' stability budget.
    // (the frame's reference pair: the most perpendicular pair of the parent's neighbours that are not held by it;
    // failing that, of all its neighbours: two held nodes define the frame and a third is held in it, so a star of
    // rigid beams is rigid; a node joined to one other node only has the held one as its second reference, which
    // holds the angle between the two)
    std::vector<std::vector<int>> held(N);
    for (const auto& j : d.joints)
        if (j.parent >= 0 && j.child >= 0 && j.parent < N && j.child < N) held[j.parent].push_back(j.child);
    for (const auto& j : d.joints) {
        if (j.parent < 0 || j.child < 0 || j.parent >= N || j.child >= N || j.parent == j.child) continue;
        int rx = j.ref_x, ry = j.ref_y;
        if (rx < 0 || ry < 0 || rx >= N || ry >= N) {
            std::vector<int> nb, own;
            for (const auto& b : body.beams) {
                const int o = (int)b.a == j.parent ? (int)b.b : (int)b.b == j.parent ? (int)b.a : -1;
                if (o < 0 || std::find(nb.begin(), nb.end(), o) != nb.end()) continue;
                nb.push_back(o);
                if (std::find(held[j.parent].begin(), held[j.parent].end(), o) == held[j.parent].end()) own.push_back(o);
            }
            const vec3 p = body.nodes[j.parent].p;
            auto best_pair = [&](const std::vector<int>& c) {
                float best = 1e-4f;
                rx = ry = -1;
                for (size_t a = 0; a < c.size(); a++)
                    for (size_t k = a + 1; k < c.size(); k++) {
                        const vec3 u = normalize_or(body.nodes[c[a]].p - p, vec3(0)), v = normalize_or(body.nodes[c[k]].p - p, vec3(0));
                        const float s = length2(cross(u, v));
                        if (s > best) best = s, rx = c[a], ry = c[k];
                    }
            };
            best_pair(own);
            if (rx < 0 && own.size() == 1) rx = own[0], ry = j.child;
            if (rx < 0) best_pair(nb);
            if (rx < 0) continue; // (a free end: nothing to hold the child against)
        }
        if (body.info[j.parent].frame < 0) {
            const quat q = rx >= 0 ? body.triad_orientation((uint32_t)j.parent, rx, ry) : quat();
            const uint32_t f = body.add_frame((uint32_t)j.parent, q, rx >= 0 ? 1.0f : 0.0f);
            body.frames[f].ref_x = rx;
            body.frames[f].ref_y = ry;
            if (rx < 0) body.frames[f].inv_inertia = 0; // (world orientation)
        }
        const int pf = body.info[j.parent].frame;
        const float dt = kDefaultDt;
        const float mc = body.nodes[j.child].mass, mp = body.nodes[j.parent].inv_mass > 0 ? body.nodes[j.parent].mass : mc * 10.0f;
        const float m_eff = std::max(1e-3f, std::min(mc, mp));
        const float k_lin = std::min(j.k, 0.08f * m_eff / (dt * dt)), d_lin = 2.0f * 0.5f * std::sqrt(k_lin * m_eff);
        const uint32_t id = body.add_joint((uint32_t)pf, (uint32_t)j.child, -1, k_lin, d_lin, 0.0f, 0.0f);
        body.joints[id].break_force = j.brk;
    }
    // ---- frame elements (FEM, phys/frame_fem.h): members welded to their nodes, solved implicitly
    if (!frame_beams.empty()) {
        std::vector<int> sec_id(frame_secs.size(), -1);
        for (int bi : frame_beams) {
            const auto& b = d.beams[bi];
            const auto& fs = d.frame_sections[b.frame];
            if (sec_id[b.frame] < 0) {
                phys::FrameSection sec = frame_secs[b.frame];
                sec.joint_k = fs.joint_k;
                sec.break_force = fs.brk;
                sec.joint_damp = fs.joint_damp;
                sec_id[b.frame] = body.fem.add_section(sec);
            }
            const uint32_t e = body.fem.add_element((uint32_t)b.n1, (uint32_t)b.n2, (uint16_t)sec_id[b.frame], (uint8_t)(b.end_a >= 0 ? b.end_a : fs.end_a),
                                                    (uint8_t)(b.end_b >= 0 ? b.end_b : fs.end_b), bi);
            // (the mass it put on its nodes: its share of the dry mass and its own; moved along when it splits or tears)
            body.fem.elems[e].mass = frame_mass[bi];
        }
    }
    // ---- triangle elements (FEM shells: phys::FrameTri) on frame nodes of their own or the members'
    if (!d.fem_tris.empty()) {
        std::vector<int> shell_id(d.fem_shells.size(), -1);
        for (size_t ti = 0; ti < d.fem_tris.size(); ti++) {
            const auto& t = d.fem_tris[ti];
            if (fem_tri_coll[ti] < 0 || t.shell < 0 || t.shell >= (int)d.fem_shells.size()) {
                warnings.push_back(format("fem triangle %d %d %d: degenerate or no such nodes", t.n1, t.n2, t.n3));
                continue;
            }
            if (shell_id[t.shell] < 0) {
                phys::ShellSection sec = phys::make_shell_section(d.fem_shells[t.shell].material, d.fem_shells[t.shell].thickness);
                sec.color = d.fem_shells[t.shell].color;
                shell_id[t.shell] = body.fem.add_shell_section(sec);
            }
            const uint32_t k = body.fem.add_tri((uint32_t)t.n1, (uint32_t)t.n2, (uint32_t)t.n3, (uint16_t)shell_id[t.shell], (int32_t)ti, fem_tri_coll[ti]);
            body.fem.tris[k].mass = body.fem.shell_sections[shell_id[t.shell]].mass_per_m2() * 0.5f * length(cross(pos[t.n2] - pos[t.n1], pos[t.n3] - pos[t.n1]));
        }
    }
    // ---- the parts on the frame at a distance (`mounts`): both ends frame nodes (the members' or the triangles')
    for (const auto& m : d.mounts) {
        const phys::MountKind kind = m.kind == 'c' ? phys::MountKind::Clamp : m.kind == 'h' ? phys::MountKind::Hinge : m.kind == 's' ? phys::MountKind::Stop
                                     : m.kind == 'r' ? phys::MountKind::Strap : phys::MountKind::Point;
        const bool b2_ok = kind != phys::MountKind::Hinge || (m.b2 >= 0 && m.b2 < N && body.fem.slot((uint32_t)m.b2) >= 0);
        if (m.a >= 0 && m.b >= 0 && m.a < N && m.b < N && body.fem.slot((uint32_t)m.a) >= 0 && body.fem.slot((uint32_t)m.b) >= 0 && b2_ok)
            body.fem.add_mount((uint32_t)m.a, (uint32_t)m.b, m.brk, m.k, m.damp, kind, m.param, (uint32_t)std::max(0, m.b2));
        else
            warnings.push_back(format("mount %d-%d: not between frame nodes", m.a, m.b));
    }
    if (!body.fem.empty()) body.fem.finalize(body);
    // ---- the collision volumes (`collision_volumes`: hulls riding on frame nodes, phys::CollisionVolume)
    for (const auto& v : d.volumes) {
        std::vector<uint32_t> an;
        for (int a : v.anchors)
            if (a >= 0 && a < N) an.push_back((uint32_t)a);
        if (body.add_volume(v.name, an, v.verts, v.break_rms) < 0)
            warnings.push_back(format("collision volume '%s': needs 3 anchors and a hull of 4 points or more", v.name.c_str()));
    }
    if (const int in = body.find_volume_parts(); in > 0) // (the parts they hold off: nodes inside one as built are left out)
        warnings.push_back(format("collision volumes: %d nodes of the parts inside them as built (not held off)", in));
    drive.hydros.clear();
    for (auto& [pi, c] : hyd)
        if (beam_index[pi] >= 0) {
            c.beam = beam_index[pi];
            drive.hydros.push_back(c);
        }
    drive.commands.clear();
    for (auto& [pi, c] : cmds)
        if (beam_index[pi] >= 0) {
            c.beam = beam_index[pi];
            drive.commands.push_back(c);
        }
    // ---- slide nodes
    for (const auto& s : d.slidenodes) {
        SlideNode sn;
        if (s.node < 0 || s.node >= N) continue;
        sn.node = (uint32_t)s.node;
        std::vector<int> rail = s.rail;
        if (rail.size() < 2 && s.railgroup >= 0)
            for (const auto& g : d.railgroups)
                if (g.id == s.railgroup) rail = g.nodes;
        for (int n : rail)
            if (n >= 0 && n < N) sn.rail.push_back((uint32_t)n);
        if (sn.rail.size() < 2) {
            warnings.push_back(format("slidenode %d: no rail, ignored", s.node));
            continue;
        }
        if (s.spring > 0) sn.k = s.spring;
        if (s.tolerance > 0) sn.threshold = s.tolerance;
        if (s.break_force > 0) sn.break_force = s.break_force;
        body.slides.push_back(std::move(sn));
    }
    body.air_drag = 0.05f;
    {
        // frontal area from the node cloud (width x height), Cd by shape: cars ~0.35, tall vehicles ~0.65
        body.compute_aabb();
        vec3 e = body.aabb.extent();
        float width = std::min(e.x, e.z), height = e.y;
        float cd = height > 2.4f ? 0.65f : 0.35f;
        body.aero_cda = cd * width * height * 0.85f;
    }
    body.collision_radius = d.collision_range > 0 ? std::max(0.03f, d.collision_range) : 0.05f;
    body.force.resize(body.nodes.size());
    body.compute_aabb();
    return true;
}

} // namespace bl
