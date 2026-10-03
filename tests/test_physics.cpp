// Physics regression tests (no GL): triangle-element sheets and the world stepping.
//
//   ./build/test_physics            all tests
//   ./build/test_physics bench      + timing of the sheet force kernel
//
// The force kernel is compared with a frozen copy of the pre-optimisation kernel (reference_shell.cpp): same forces,
// plastic state, strains and overload events. The rest checks invariants that any implementation has to keep.
#include "core/jobs.h"
#include "core/profiler.h"
#include "core/util.h"
#include "phys/cache_sim.h"
#include "phys/fem_shell.h"
#include "phys/sheet_builder.h"
#include "phys/world.h"
#include "reference_shell.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <set>
#include <string>

using namespace bl;
using namespace bl::phys;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...)                                                   \
    do {                                                                   \
        if (cond) {                                                        \
            g_pass++;                                                      \
        } else {                                                           \
            g_fail++;                                                      \
            printf("    FAIL line %d: %s: ", __LINE__, #cond);            \
            printf(__VA_ARGS__);                                           \
            printf("\n");                                                 \
        }                                                                  \
    } while (0)

struct TRng {
    uint32_t s;
    explicit TRng(uint32_t seed) : s(seed * 2654435761u + 1) {}
    uint32_t next() {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return s;
    }
    float uni() { return (next() & 0xffffff) / float(0x1000000); }
    float range(float a, float b) { return a + (b - a) * uni(); }
};

// ------------------------------------------------------------------------------------------ materials (as in the lab)
static ShellMaterial mat_lead() {
    ShellMaterial m;
    m.max_level = 4, m.min_edge = 0.02f, m.bend_damp = 2.0f, m.min_piece = 12;
    m.k = 4e5f, m.damp = 150, m.yield = 0.006f, m.brk = 0.45f, m.refine = 0.3f, m.refine_yield = 0.05f;
    m.bend = 300, m.bend_yield = 0.03f, m.refine_angle = 0.35f;
    return m;
}
static ShellMaterial mat_glass() {
    ShellMaterial m;
    m.max_level = 4, m.min_edge = 0.02f, m.bend_damp = 2.0f, m.min_piece = 6;
    m.k = 8e6f, m.damp = 80, m.brk = 0.01f, m.refine = 0.7f, m.flaw = 0.3f;
    m.bend = 1e9f, m.bend_break = 0.15f, m.refine_angle = 0.12f;
    return m;
}
static ShellMaterial mat_steel() {
    ShellMaterial m;
    m.max_level = 4, m.min_edge = 0.02f, m.bend_damp = 2.0f, m.min_piece = 12;
    m.k = 1e7f, m.damp = 200, m.yield = 0.004f, m.brk = 0.35f, m.refine = 0.3f, m.refine_yield = 0.02f;
    m.bend = 1e9f, m.bend_yield = 0.1f, m.refine_angle = 0.35f;
    return m;
}
static ShellMaterial mat_plywood() {
    ShellMaterial m;
    m.max_level = 4, m.min_edge = 0.02f, m.bend_damp = 2.0f, m.min_piece = 6;
    m.k = 3e6f, m.damp = 80, m.yield = 0.012f, m.brk = 0.025f, m.refine = 0.4f, m.flaw = 0.3f;
    m.bend = 1e9f, m.bend_yield = 0.15f, m.bend_break = 0.25f, m.refine_angle = 0.12f;
    return m;
}
// the materials with their fracture patterns (as sheet_material() in the scenes)
static ShellMaterial patterned(ShellMaterial m, ShellPattern p, float size, float speed, float grain = 0.0f) {
    m.pattern = p;
    m.pattern_size = size;
    m.pattern_speed = speed;
    m.grain_angle = grain;
    return m;
}
static ShellMaterial mat_fabric() {
    ShellMaterial m;
    m.max_level = 4, m.min_edge = 0.02f, m.bend_damp = 2.0f, m.min_piece = 12;
    m.k = 4e4f, m.damp = 30, m.brk = 0.25f, m.refine = 0.5f, m.tension_only = true;
    m.bend = 0, m.refine_angle = 1e9f;
    return m;
}

// Sheet as build_sheet() makes it (4-8 grid, clamp 0 free, 1 all edges, 2 top + sides, 3 top).
static std::unique_ptr<SoftBody> make_sheet(vec3 c, vec3 u, vec3 v, float w, float h, int nu, int nv, float mass, int clamp, const ShellMaterial& m,
                                           uint32_t seed = 1) {
    auto b = std::make_unique<SoftBody>();
    b->name = "test sheet";
    auto id = [&](int i, int j) { return (uint32_t)(j * nu + i); };
    for (int j = 0; j < nv; j++)
        for (int i = 0; i < nu; i++) {
            const bool side = i == 0 || i == nu - 1, top = j == nv - 1, bottom = j == 0;
            const bool fixed = clamp == 1 ? (side || top || bottom) : clamp == 2 ? (side || top) : clamp == 3 ? top : false;
            vec3 p = c + u * (((float)i / (nu - 1) - 0.5f) * w) + v * (((float)j / (nv - 1) - 0.5f) * h);
            b->add_node(p, 1.0f, NF_GROUND | NF_CONTACTER | (fixed ? NF_FIXED : 0));
        }
    auto uv = [&](int i, int j) { return vec2((float)i / (nu - 1), (float)j / (nv - 1)); };
    for (int j = 0; j + 1 < nv; j++)
        for (int i = 0; i + 1 < nu; i++) {
            const uint32_t a = id(i, j), bb = id(i + 1, j), cc = id(i + 1, j + 1), e = id(i, j + 1);
            if (((i + j) & 1) == 0) {
                b->add_shell(a, bb, cc, uv(i, j), uv(i + 1, j), uv(i + 1, j + 1));
                b->add_shell(a, cc, e, uv(i, j), uv(i + 1, j + 1), uv(i, j + 1));
            } else {
                b->add_shell(a, bb, e, uv(i, j), uv(i + 1, j), uv(i, j + 1));
                b->add_shell(bb, cc, e, uv(i + 1, j), uv(i + 1, j + 1), uv(i, j + 1));
            }
        }
    b->shell_mat = m;
    b->collision_radius = 0.04f;
    b->finalize();
    b->finalize_shells(mass / (w * h), kDefaultDt, seed);
    return b;
}

// A soft ball (icosahedron + centre) of radius r.
static std::unique_ptr<SoftBody> make_ball(vec3 c, float r, float mass, vec3 vel) {
    auto b = std::make_unique<SoftBody>();
    b->name = "test ball";
    const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    const vec3 ico[12] = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t}, {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    const int faces[20][3] = {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
                              {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
    for (auto& p : ico) b->add_node(c + normalize(p) * r, mass / 13.0f);
    const uint32_t centre = b->add_node(c, mass / 13.0f);
    std::set<std::pair<int, int>> edges;
    for (auto& f : faces) {
        for (int k = 0; k < 3; k++) {
            int a = f[k], bb = f[(k + 1) % 3];
            edges.insert({std::min(a, bb), std::max(a, bb)});
        }
        b->add_triangle(f[0], f[2], f[1]);
    }
    for (auto& e : edges) b->add_beam(e.first, e.second, 2e6f, 400, 1e12f, 1e12f);
    for (uint32_t i = 0; i < 12; i++) b->add_beam(i, centre, 2e6f, 400, 1e12f, 1e12f);
    b->finalize();
    b->stabilize(kDefaultDt);
    b->set_velocity(vel);
    b->collision_radius = 0.05f;
    return b;
}

static void jitter(SoftBody& b, TRng& r, float dp, float dv) {
    for (size_t i = 0; i < b.nodes.size(); i++) {
        if (b.nodes[i].inv_mass <= 0) continue;
        b.nodes[i].p += vec3(r.range(-dp, dp), r.range(-dp, dp), r.range(-dp, dp));
        b.nodes[i].v += vec3(r.range(-dv, dv), r.range(-dv, dv), r.range(-dv, dv));
    }
}

// Random refinements and cracks through the event queue (as the simulation does it).
static void random_topology(SoftBody& b, TRng& r, int rounds, int refine_per_round, int cracks_per_round) {
    for (int k = 0; k < rounds; k++) {
        for (int i = 0; i < refine_per_round; i++) b.shell_events.push_back({(uint32_t)(r.next() % b.shells.size()), 0, 0});
        b.process_shell_events();
        for (int i = 0; i < cracks_per_round; i++) {
            uint32_t s = r.next() % b.shells.size();
            b.shell_strain(s) = 5.0f; // (crack priority)
            b.shell_events.push_back({s, 1, (uint8_t)(r.next() % 3)});
        }
        b.process_shell_events();
    }
}

static double shell_area(const SoftBody& b) {
    double a = 0;
    for (const auto& s : b.shells) a += s.area0;
    return a;
}

// ------------------------------------------------------------------------------------------ invariants
static void check_topology(const SoftBody& b, const char* what) {
    int bad_link = 0, bad_fan = 0, bad_tri = 0, bad_mass = 0, nonfinite = 0, bad_code = 0, bad_uv = 0;
    for (uint32_t si = 0; si < b.shells.size(); si++) {
        const Shell& s = b.shells[si];
        for (int e = 0; e < 3; e++) {
            const int j = s.nb[e];
            if (j < 0) continue;
            const Shell& t = b.shells[j];
            const uint32_t a = s.n[e], c = s.n[e == 2 ? 0 : e + 1];
            int f = -1;
            for (int q = 0; q < 3; q++) {
                uint32_t p = t.n[q], w = t.n[q == 2 ? 0 : q + 1];
                if ((p == a && w == c) || (p == c && w == a)) f = q;
            }
            if (f < 0 || t.nb[f] != (int)si) bad_link++;
            else if (t.es[f] != s.es[e] || t.hs[f] != s.hs[e] || ((t.line >> f) & 1) != ((s.line >> e) & 1)) bad_code++;
        }
        const Triangle& tr = b.tris[s.tri];
        if (tr.a != s.n[0] || tr.b != s.n[1] || tr.c != s.n[2]) bad_tri++;
    }
    std::vector<std::set<uint32_t>> fan(b.nodes.size());
    for (uint32_t si = 0; si < b.shells.size(); si++)
        for (int c = 0; c < 3; c++) fan[b.shells[si].n[c]].insert(si);
    for (size_t v = 0; v < b.nodes.size() && v < b.node_shells.size(); v++) {
        std::set<uint32_t> listed(b.node_shells[v].begin(), b.node_shells[v].end());
        if (listed != fan[v] || listed.size() != b.node_shells[v].size()) bad_fan++;
        double m = 0;
        for (uint32_t si : fan[v]) m += b.shells[si].mass / 3.0;
        if (!fan[v].empty() && std::fabs(m - b.nodes[v].mass) > 1e-4 * std::max(1e-3, m)) bad_mass++;
        const Node& n = b.nodes[v];
        if (!std::isfinite(n.p.x + n.p.y + n.p.z + n.v.x + n.v.y + n.v.z)) nonfinite++;
        vec2 uv0(0);
        bool first = true, same = true;
        for (uint32_t si : fan[v]) {
            const Shell& s = b.shells[si];
            for (int c = 0; c < 3; c++)
                if (s.n[c] == v) {
                    if (first) uv0 = s.uv[c];
                    else same &= std::fabs(s.uv[c].x - uv0.x) < 1e-5f && std::fabs(s.uv[c].y - uv0.y) < 1e-5f;
                    first = false;
                }
        }
        if (!same) bad_uv++;
    }
    CHECK(bad_link == 0, "%s: %d asymmetric neighbour links", what, bad_link);
    CHECK(bad_fan == 0, "%s: %d node fans differ from the shells", what, bad_fan);
    CHECK(bad_tri == 0, "%s: %d collision triangles differ from the shells", what, bad_tri);
    CHECK(bad_mass == 0, "%s: %d node masses differ from the shells' shares", what, bad_mass);
    CHECK(nonfinite == 0, "%s: %d non-finite nodes", what, nonfinite);
    CHECK(bad_code == 0, "%s: %d edges with different pattern codes on their two sides", what, bad_code);
    CHECK(bad_uv == 0, "%s: %d nodes with different uv in their triangles", what, bad_uv);
}

// ------------------------------------------------------------------------------------------ force kernel vs reference
static void compare_kernels(const SoftBody& body, const char* what, float h = kDefaultDt) {
    ref::RefBody A(body);
    SoftBody B(body);
    A.clear_forces(vec3(0));
    B.clear_forces(vec3(0));
    A.shell_acc_stale = B.shell_acc_stale = true;
    A.ref_forces(h, 0, 1);
    B.compute_shell_forces(h, 0, 1);
    double fmax = 0, dmax = 0;
    for (size_t i = 0; i < A.force.size(); i++) {
        fmax = std::max(fmax, (double)length(A.force[i]));
        dmax = std::max(dmax, (double)length(A.force[i] - B.force[i]));
    }
    // (the kernel's atan table is 1.3e-6 rad off: on the stiff hinges of a folded metal sheet that is a few 1e-4 of the force)
    CHECK(dmax <= 5e-4 * std::max(1.0, fmax), "%s: forces differ by %.3g (max force %.3g)", what, dmax, fmax);
    double dl = 0, dth = 0, dst = 0;
    for (size_t s = 0; s < A.shells.size(); s++)
        for (int e = 0; e < 3; e++) {
            dl = std::max(dl, (double)std::fabs(A.shells[s].L[e] - B.shells[s].L[e]) / std::max(1e-6f, A.shells[s].L0[e]));
            dth = std::max(dth, (double)std::fabs(A.shells[s].th0[e] - B.shells[s].th0[e]));
            dst = std::max(dst, (double)std::fabs(A.shells[s].strain - B.shell_strain((uint32_t)s)) / std::max(1.0f, A.shells[s].strain));
        }
    CHECK(dl < 1e-4, "%s: plastic rest lengths differ by %.3g (relative)", what, dl);
    CHECK(dth < 1e-3, "%s: plastic rest angles differ by %.3g rad", what, dth);
    CHECK(dst < 1e-3, "%s: strains differ by %.3g", what, dst);
    auto events = [](const SoftBody& b) {
        std::set<std::tuple<uint32_t, int, int>> s;
        for (auto& e : b.shell_events) s.insert({e.shell, e.kind, e.kind == 0 ? 0 : e.edge});
        return s;
    };
    const auto ea = events(A), eb = events(B);
    size_t common = 0;
    for (auto& e : ea) common += eb.count(e);
    // (borderline overloads may flip with rounding; nearly all events have to match)
    CHECK(ea.size() == eb.size() || common + 2 >= std::max(ea.size(), eb.size()), "%s: overload events differ (%zu vs %zu, %zu in common)", what,
          ea.size(), eb.size(), common);
}

static void test_kernel_equivalence() {
    printf("kernel vs reference\n");
    for (int variant = 0; variant < 4; variant++) {
        const ShellMaterial m = variant == 0 ? mat_lead() : variant == 1 ? mat_glass() : variant == 2 ? mat_fabric() : mat_lead();
        auto s = make_sheet(vec3(0, 1.4f, 0), vec3(1, 0, 0), vec3(0, 1, 0), 3.6f, 2.4f, 15, 11, 500.0f, variant == 3 ? 0 : 2, m, 7 + variant);
        TRng r(100 + variant);
        char name[64];
        jitter(*s, r, 0.01f, 0.3f);
        snprintf(name, sizeof name, "material %d, authored", variant);
        compare_kernels(*s, name);
        random_topology(*s, r, 12, 30, 6);
        jitter(*s, r, 0.02f, 0.8f);
        snprintf(name, sizeof name, "material %d, refined + cracked (%zu tris)", variant, s->shells.size());
        compare_kernels(*s, name);
        jitter(*s, r, 0.08f, 3.0f); // large: plastic flow, overloads, folds
        snprintf(name, sizeof name, "material %d, heavily deformed", variant);
        compare_kernels(*s, name);
    }
}

static void test_momentum() {
    printf("momentum and angular momentum of the internal forces\n");
    ShellMaterial m = mat_lead();
    m.aero = 0;
    m.flex_damp = 0;
    auto s = make_sheet(vec3(0.3f, 1.0f, -0.2f), normalize(vec3(1, 0.2f, 0)), normalize(vec3(0, 0.3f, 1)), 2.0f, 2.0f, 12, 12, 300, 0, m, 3);
    TRng r(5);
    random_topology(*s, r, 8, 20, 3);
    jitter(*s, r, 0.05f, 1.0f);
    s->clear_forces(vec3(0));
    s->shell_acc_stale = true;
    s->allow_break = false;
    s->compute_shell_forces(kDefaultDt, 0, 1);
    vec3 sum(0), torque(0);
    double mag = 0;
    for (size_t i = 0; i < s->nodes.size(); i++) {
        sum += s->force[i];
        torque += cross(s->nodes[i].p, s->force[i]);
        mag += length(s->force[i]);
    }
    CHECK(length(sum) < 1e-4 * mag, "net force %.3g of %.3g", length(sum), mag);
    CHECK(length(torque) < 1e-3 * mag, "net torque %.3g (force scale %.3g)", length(torque), mag);
}

static void test_reorder() {
    printf("Morton renumbering keeps the sheet and its forces\n");
    auto s = make_sheet(vec3(0, 1.4f, 0), vec3(1, 0, 0), vec3(0, 1, 0), 3.6f, 2.4f, 15, 11, 500.0f, 2, mat_lead(), 3);
    s->shell_cap = 1u << 30;
    TRng r(9);
    for (int k = 0; k < 6; k++) {
        for (int i = 0; i < 60; i++) s->shell_events.push_back({(uint32_t)(r.next() % s->shells.size()), 0, 0});
        s->process_shell_events();
    }
    jitter(*s, r, 0.01f, 0.3f);
    s->allow_break = false;
    SoftBody a(*s), b(*s);
    CHECK(b.reorder_shells(), "not reordered");
    check_topology(b, "reordered");
    a.clear_forces(vec3(0, -9.81f, 0));
    b.clear_forces(vec3(0, -9.81f, 0));
    a.compute_shell_forces(kDefaultDt, 0, 1);
    b.compute_shell_forces(kDefaultDt, 0, 1);
    // (refined only: every node has a position of its own)
    auto sorted = [](const SoftBody& x) {
        std::vector<std::pair<std::tuple<float, float, float>, vec3>> v;
        for (size_t i = 0; i < x.nodes.size(); i++) v.push_back({{x.nodes[i].p.x, x.nodes[i].p.y, x.nodes[i].p.z}, x.force[i]});
        std::sort(v.begin(), v.end(), [](auto& p, auto& q) { return p.first < q.first; });
        return v;
    };
    const auto fa = sorted(a), fb = sorted(b);
    double d = 0, m = 0;
    for (size_t i = 0; i < fa.size(); i++) {
        d = std::max(d, (double)length(fa[i].second - fb[i].second));
        m = std::max(m, (double)length(fa[i].second));
    }
    CHECK(fa.size() == fb.size() && d <= 2e-4 * std::max(1.0, m), "forces differ by %.3g (max %.3g)", d, m);
    CHECK(std::fabs(shell_area(a) - shell_area(b)) < 1e-6 * shell_area(a), "area changed");
}

static void test_topology() {
    printf("topology invariants after random refinement and cracks\n");
    for (int variant = 0; variant < 3; variant++) {
        const ShellMaterial m = variant == 0 ? mat_lead() : variant == 1 ? mat_glass() : mat_fabric();
        auto s = make_sheet(vec3(0), vec3(1, 0, 0), vec3(0, 0, -1), 1.7f, 1.7f, 14, 14, 100.0f, 1, m, 11 + variant);
        const double area0 = shell_area(*s);
        double mass0 = 0;
        for (auto& sh : s->shells) mass0 += sh.mass;
        TRng r(40 + variant);
        random_topology(*s, r, 25, 40, 12);
        char name[64];
        snprintf(name, sizeof name, "material %d (%zu tris, %d cracks)", variant, s->shells.size(), s->shell_stats.cracks);
        check_topology(*s, name);
        CHECK(std::fabs(shell_area(*s) - area0) < 1e-4 * area0, "%s: area %.6f vs %.6f", name, shell_area(*s), area0);
        double mass = 0;
        for (auto& sh : s->shells) mass += sh.mass;
        CHECK(std::fabs(mass - mass0) < 1e-4 * mass0, "%s: mass %.6f vs %.6f", name, mass, mass0);
        CHECK(s->shell_stats.cracks > 0 && s->shell_stats.refined > 0, "%s: nothing happened", name);
        // loose pieces become bodies of their own: nothing is lost
        std::vector<std::unique_ptr<SoftBody>> pieces;
        s->pieces_check = true;
        s->detach_pieces(pieces);
        double a = shell_area(*s);
        for (auto& p : pieces) {
            a += shell_area(*p);
            check_topology(*p, "piece");
        }
        check_topology(*s, "after detaching");
        CHECK(std::fabs(a - area0) < 1e-4 * area0, "%s: area after detaching %.6f vs %.6f (%zu pieces)", name, a, area0, pieces.size());
    }
}

// ------------------------------------------------------------------------------------------ world level
struct RunResult {
    double hash = 0;
    int cracks = 0;
    size_t bodies = 0;
    double area = 0;
    float ball_min_y = 1e9f;
    bool finite = true;
    double ms = 0;
};

static RunResult run_impact(bool multithreaded, int frames, const ShellMaterial& m, float speed = 10.0f, double team_cost = -1) {
    World w;
    w.settings.multithreaded = multithreaded;
    if (team_cost >= 0) w.settings.team_cost = team_cost;
    auto sheet = make_sheet(vec3(0, 1.3f, 0), vec3(1, 0, 0), vec3(0, 0, -1), 1.7f, 1.7f, 14, 14, 68.0f * 2.89f, 1, m, 21);
    const double area0 = shell_area(*sheet);
    w.add_body(std::move(sheet));
    w.add_body(make_ball(vec3(0.05f, 1.9f, 0.03f), 0.22f, 60.0f, vec3(0, -speed, 0)));
    const auto t0 = std::chrono::steady_clock::now();
    for (int f = 0; f < frames; f++) w.step_substeps(33);
    RunResult r;
    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    r.bodies = w.bodies().size();
    for (auto& b : w.bodies()) {
        for (size_t i = 0; i < b->nodes.size(); i++) {
            const Node& n = b->nodes[i];
            r.hash += (n.p.x * 1.3 + n.p.y * 2.7 + n.p.z * 3.1) * (double)((i % 97) + 1);
            r.finite &= std::isfinite(n.p.x + n.p.y + n.p.z);
            if (b->name == "test ball") r.ball_min_y = std::min(r.ball_min_y, n.p.y);
        }
        r.cracks += b->shell_stats.cracks;
        r.area += shell_area(*b);
    }
    r.area /= area0;
    return r;
}

static void test_world() {
    printf("world: a ball through a sheet\n");
    const RunResult a = run_impact(false, 45, mat_lead(), 14.0f);
    CHECK(a.finite, "non-finite state");
    CHECK(a.cracks > 0, "the sheet did not crack");
    CHECK(a.ball_min_y < 1.3f - 0.3f, "the ball did not pass (lowest node y %.2f)", a.ball_min_y);
    CHECK(std::fabs(a.area - 1.0) < 1e-4, "area of the sheet and its pieces %.6f of the authored one", a.area);
    const RunResult b = run_impact(false, 45, mat_lead(), 14.0f);
    CHECK(a.hash == b.hash, "single-threaded runs differ (%.9g vs %.9g)", a.hash, b.hash);
    const RunResult c = run_impact(true, 45, mat_lead(), 14.0f);
    CHECK(a.hash == c.hash, "multithreaded run differs from the single-threaded one (%.9g vs %.9g)", a.hash, c.hash);
    const RunResult c2 = run_impact(true, 45, mat_lead(), 14.0f, 1e30);
    CHECK(a.hash == c2.hash, "multithreaded run without teams differs from the single-threaded one (%.9g vs %.9g)", a.hash, c2.hash);
    const RunResult c3 = run_impact(true, 45, mat_lead(), 14.0f);
    CHECK(c3.hash == c.hash, "two multithreaded runs differ (%.9g vs %.9g)", c.hash, c3.hash);
    printf("    %d cracks, %zu bodies, %.1f ms single / %.1f ms multithreaded for %d frames\n", a.cracks, a.bodies, a.ms, c.ms, 45);
    const RunResult g = run_impact(false, 45, mat_glass(), 14.0f);
    CHECK(g.finite && g.cracks > 0, "glass: finite %d, cracks %d", (int)g.finite, g.cracks);
}

// Two balls into one sheet, one after the other at different spots: the second impact has to refine the sheet around
// it as the first one did (the refinement budget used to be spent by the first impact).
struct ImpactStats {
    int fine1 = 0, fine2 = 0;   // triangles of level >= 3 within 0.35 m of each impact point, after each impact
    size_t shells = 0, cap = 0;
    int cracks = 0;
};
static ImpactStats run_two_impacts(const ShellMaterial& m, float w, float h, int nu, int nv, float kg_m2, float speed, int frames_between) {
    World w0;
    World& W = w0;
    W.settings.coarsen_per_frame = 0; // (the refinement budget is under test: the detail must stay)
    auto sheet = make_sheet(vec3(0, 1.5f, 0), vec3(1, 0, 0), vec3(0, 0, -1), w, h, nu, nv, kg_m2 * w * h, 1, m, 31);
    SoftBody* sb = W.add_body(std::move(sheet));
    const vec3 p1(-w * 0.25f, 1.5f, 0.1f * h), p2(w * 0.25f, 1.5f, -0.1f * h);
    W.add_body(make_ball(p1 + vec3(0, 0.5f, 0), 0.22f, 60.0f, vec3(0, -speed, 0)));
    auto fine_near = [&](vec3 c) {
        int n = 0;
        for (const SoftBody* b : [&] {
                 std::vector<const SoftBody*> v;
                 for (auto& x : W.bodies())
                     if (!x->shells.empty()) v.push_back(x.get());
                 return v;
             }())
            for (const Shell& s : b->shells) {
                const vec3 q = (b->nodes[s.n[0]].p + b->nodes[s.n[1]].p + b->nodes[s.n[2]].p) / 3.0f;
                if (s.level >= 3 && length(vec3(q.x - c.x, 0, q.z - c.z)) < 0.35f) n++;
            }
        return n;
    };
    for (int f = 0; f < frames_between; f++) W.step_substeps(33);
    ImpactStats r;
    r.fine1 = fine_near(p1);
    W.add_body(make_ball(p2 + vec3(0, 0.5f, 0), 0.22f, 60.0f, vec3(0, -speed, 0)));
    for (int f = 0; f < 40; f++) W.step_substeps(33);
    r.fine2 = fine_near(p2);
    r.shells = sb->shells.size();
    r.cap = sb->shell_cap;
    for (auto& b : W.bodies()) r.cracks += b->shell_stats.cracks;
    return r;
}

static void test_repeat_impacts() {
    printf("repeated impacts: the second ball refines the sheet like the first\n");
    struct Case {
        const char* name;
        ShellMaterial m;
        float w, h;
        int nu, nv;
        float kg_m2, speed;
    };
    const Case cases[] = {{"lead 1.7 m", mat_lead(), 1.7f, 1.7f, 14, 14, 68.0f, 12.0f},
                          {"lead 3.6 m", mat_lead(), 3.6f, 2.4f, 15, 11, 68.0f, 12.0f},
                          {"glass 1.7 m", mat_glass(), 1.7f, 1.7f, 14, 14, 25.0f, 8.0f}};
    for (const Case& c : cases) {
        const ImpactStats r = run_two_impacts(c.m, c.w, c.h, c.nu, c.nv, c.kg_m2, c.speed, 40);
        printf("    %-12s first impact: %d fine triangles, second: %d; %zu triangles (cap %zu), %d cracks\n", c.name, r.fine1, r.fine2, r.shells, r.cap,
               r.cracks);
        CHECK(r.fine1 > 0, "%s: the first impact did not refine", c.name);
        CHECK(r.fine2 * 2 >= r.fine1, "%s: the second impact refined %d triangles, the first %d", c.name, r.fine2, r.fine1);
    }
}

// ------------------------------------------------------------------------------------------ shapes and sizes
struct ShapeCase {
    const char* name;
    SheetMeshDesc d;
    float kg_m2;
    double area; // expected (analytic)
};
static std::vector<ShapeCase> shape_cases(vec3 c, int clamp) {
    const float pi = 3.14159265f;
    std::vector<ShapeCase> v;
    auto base = [&](SheetShape sh, float w, float h, int nu, int nv) {
        SheetMeshDesc d;
        d.center = c;
        d.u = vec3(1, 0, 0);
        d.v = vec3(0, 0, -1); // (lying: normal up)
        d.width = w;
        d.height = h;
        d.nu = nu;
        d.nv = nv;
        d.shape = sh;
        d.clamp = clamp;
        return d;
    };
    v.push_back({"big 6x4 m", base(SheetShape::Rect, 6.0f, 4.0f, 31, 21), 40.0f, 24.0});
    v.push_back({"disc 2.4 m", base(SheetShape::Disc, 2.4f, 2.4f, 21, 21), 40.0f, pi * 1.2 * 1.2});
    {
        ShapeCase r{"ring 2.4 m", base(SheetShape::Ring, 2.4f, 2.4f, 25, 25), 40.0f, pi * 1.44 * (1 - 0.45 * 0.45)};
        v.push_back(r);
    }
    v.push_back({"triangle 2.6 m", base(SheetShape::Triangle, 2.6f, 2.2f, 21, 17), 40.0f, 0.5 * 2.6 * 2.2});
    v.push_back({"L 2.4 m", base(SheetShape::LShape, 2.4f, 2.4f, 17, 17), 40.0f, 2.4 * 2.4 * 0.75});
    {
        ShapeCase h{"half-pipe", base(SheetShape::Rect, pi * 1.0f, 2.0f, 23, 15), 40.0f, pi * 1.0 * 2.0};
        h.d.curve = 1.0f;
        v.push_back(h);
    }
    {
        ShapeCase dm{"dome", base(SheetShape::Disc, 2.4f, 2.4f, 21, 21), 40.0f, 0};
        dm.d.dome = 1.6f;
        const double psi = 1.2 / 1.6; // cap area 2 pi R^2 (1 - cos(psi)) for an arc length of 1.2 m
        dm.area = 2 * pi * 1.6 * 1.6 * (1 - std::cos(psi));
        v.push_back(dm);
    }
    return v;
}
static std::unique_ptr<SoftBody> make_shape(const ShapeCase& c, const ShellMaterial& m, uint32_t seed) {
    auto b = std::make_unique<SoftBody>();
    b->name = std::string("shape ") + c.name;
    const float area = add_sheet_mesh(*b, c.d);
    b->shell_mat = m;
    b->collision_radius = 0.04f;
    b->finalize();
    b->finalize_shells(c.kg_m2, kDefaultDt, seed);
    (void)area;
    return b;
}

static void test_shapes() {
    printf("sheets of other shapes and sizes: mesh, kernel, topology\n");
    uint32_t seed = 50;
    for (const ShapeCase& c : shape_cases(vec3(0, 1.5f, 0), 0)) {
        for (int mat = 0; mat < 2; mat++) {
            const ShellMaterial m = mat == 0 ? mat_lead() : mat_glass();
            auto s = make_shape(c, m, seed++);
            char name[96];
            snprintf(name, sizeof name, "%s, %s", c.name, mat == 0 ? "lead" : "glass");
            const double a0 = shell_area(*s);
            double mass0 = 0;
            for (auto& n : s->nodes) mass0 += n.mass;
            if (mat == 0) {
                check_topology(*s, name);
                CHECK(std::fabs(a0 - c.area) < 0.04 * c.area, "%s: area %.3f, expected %.3f", name, a0, c.area);
                CHECK(std::fabs(mass0 - c.kg_m2 * a0) < 1e-3 * mass0, "%s: node masses %.3f, expected %.3f", name, mass0, c.kg_m2 * a0);
            }
            TRng r(seed * 7);
            jitter(*s, r, 0.01f, 0.3f);
            compare_kernels(*s, name);
            random_topology(*s, r, 10, 40, 10);
            char name2[128];
            snprintf(name2, sizeof name2, "%s after refinement and cracks (%zu tris)", name, s->shells.size());
            check_topology(*s, name2);
            CHECK(std::fabs(shell_area(*s) - a0) < 1e-4 * a0, "%s: area %.6f vs %.6f", name2, shell_area(*s), a0);
            jitter(*s, r, 0.03f, 1.0f);
            compare_kernels(*s, name2);
            std::vector<std::unique_ptr<SoftBody>> pieces;
            s->pieces_check = true;
            s->detach_pieces(pieces);
            double a = shell_area(*s);
            for (auto& p : pieces) {
                a += shell_area(*p);
                check_topology(*p, "piece");
            }
            CHECK(std::fabs(a - a0) < 1e-4 * a0, "%s: area after detaching %.6f vs %.6f", name2, a, a0);
        }
    }
}

// A ball through each shape (clamped at its border, or held at two corners), then a second one elsewhere
static void test_shape_impacts() {
    printf("sheets of other shapes: two balls through each, settling\n");
    for (int clamp : {1, 4}) {
        for (const ShapeCase& c : shape_cases(vec3(0, 1.5f, 0), clamp)) {
            if (clamp == 4 && std::string(c.name) != "disc 2.4 m" && std::string(c.name) != "triangle 2.6 m" && std::string(c.name) != "dome") continue;
            World W;
            SoftBody* sb = W.add_body(make_shape(c, mat_lead(), 77));
            const double a0 = shell_area(*sb);
            const vec3 p1 = vec3(-0.3f, 2.1f, 0.2f), p2 = vec3(0.35f, 2.1f, -0.25f);
            W.add_body(make_ball(p1, 0.2f, 50.0f, vec3(0, -12.0f, 0)));
            for (int f = 0; f < 35; f++) W.step_substeps(33);
            W.add_body(make_ball(p2, 0.2f, 50.0f, vec3(0, -12.0f, 0)));
            for (int f = 0; f < 45; f++) W.step_substeps(33);
            double area = 0;
            int cracks = 0, fine = 0;
            bool finite = true;
            float ball_y = 1e9f;
            for (auto& b : W.bodies()) {
                area += shell_area(*b);
                cracks += b->shell_stats.cracks;
                for (auto& s : b->shells) fine += s.level >= 3;
                for (auto& n : b->nodes) {
                    finite &= std::isfinite(n.p.x + n.p.y + n.p.z);
                    if (b->name == "test ball") ball_y = std::min(ball_y, n.p.y);
                }
            }
            char name[96];
            snprintf(name, sizeof name, "%s, %s", c.name, clamp == 1 ? "clamped border" : "hung at two corners");
            printf("    %-34s %5zu triangles, %4d at level 3+, %3d cracks, %zu bodies\n", name, sb->shells.size(), fine, cracks, W.bodies().size());
            CHECK(finite, "%s: non-finite state", name);
            CHECK(std::fabs(area - a0) < 1e-4 * a0, "%s: area %.6f vs %.6f", name, area, a0);
            CHECK(fine > 20, "%s: hardly refined (%d triangles at level 3+)", name, fine);
            CHECK(cracks > 0 || clamp == 4, "%s: no cracks", name);
        }
    }
}

// ------------------------------------------------------------------------------------------ laser
// ------------------------------------------------------------------------------------------ fracture patterns
// Crack edges (free edges that are neither the border nor a laser cut) of a sheet and its pieces: their total length,
// the length within `near` of the lines of the sheet's impacts, and the length running along direction g (|cos| > 0.9).
struct CrackStats {
    double len = 0, near_line = 0, along = 0;
    int edges = 0;
};
static void crack_stats(const SoftBody& b, CrackStats& r, vec2 g) {
    for (uint32_t si = 0; si < b.shells.size(); si++) {
        const Shell& s = b.shells[si];
        for (int e = 0; e < 3; e++) {
            if (s.nb[e] >= 0 || (s.edges & (9u << e))) continue;
            const vec2 xa = b.shell_x(si, e), xb = b.shell_x(si, (e + 1) % 3);
            const float l = length(xb - xa);
            if (l < 1e-6f) continue;
            r.len += l;
            r.edges++;
            const float tol = 0.25f * l + 0.002f;
            const float d = std::max(b.pattern_nearest(xa, tol, 1).d, std::max(b.pattern_nearest(xb, tol, 1).d, b.pattern_nearest((xa + xb) * 0.5f, tol, 1).d));
            if (d < tol) r.near_line += l;
            if (std::fabs(dot((xb - xa) * (1.0f / l), g)) > 0.9f) r.along += l;
        }
    }
}

struct PatternRun {
    double hash = 0, area = 0, area0 = 0;
    bool finite = true;
    int cracks = 0, impacts = 0;
    size_t bodies = 0;
    CrackStats cs;
};
static PatternRun run_pattern(const ShellMaterial& m, float kg_m2, float speed, bool mt, int frames, vec2 at, vec2 grain = vec2(1, 0),
                              std::unique_ptr<SoftBody>* keep = nullptr, const std::vector<ShellImpact>* measure = nullptr) {
    World w;
    w.settings.multithreaded = mt;
    auto sheet = make_sheet(vec3(0, 1.3f, 0), vec3(1, 0, 0), vec3(0, 0, -1), 1.7f, 1.7f, 14, 14, kg_m2 * 2.89f, 1, m, 21);
    PatternRun r;
    r.area0 = shell_area(*sheet);
    SoftBody* sb = w.add_body(std::move(sheet));
    w.add_body(make_ball(vec3(at.x, 1.9f, -at.y), 0.22f, 60.0f, vec3(0, -speed, 0)));
    for (int f = 0; f < frames; f++) w.step_substeps(33);
    r.bodies = w.bodies().size();
    r.impacts = (int)sb->shell_impacts.size();
    for (auto& b : w.bodies()) {
        for (size_t i = 0; i < b->nodes.size(); i++) {
            const Node& n = b->nodes[i];
            r.hash += (n.p.x * 1.3 + n.p.y * 2.7 + n.p.z * 3.1) * (double)((i % 97) + 1);
            r.finite &= std::isfinite(n.p.x + n.p.y + n.p.z);
        }
        if (b->shells.empty()) continue;
        r.cracks += b->shell_stats.cracks;
        r.area += shell_area(*b);
        if (measure) b->shell_impacts = *measure; // (a sheet without a pattern measured against another run's lines)
        crack_stats(*b, r.cs, grain);
        check_topology(*b, "pattern run");
    }
    if (keep)
        for (auto& b : w.bodies())
            if (b.get() == sb) *keep = std::make_unique<SoftBody>(*sb);
    return r;
}

static void test_patterns() {
    printf("fracture patterns: the cracks follow the material's lines round the point of impact\n");
    // the lines of an impact laid directly: refined level by level, nodes put on the lines, codes on both sides of
    // every edge the same, the kernel as the reference on the patterned sheet
    {
        const ShellMaterial m = patterned(mat_glass(), ShellPattern::Radial, 0.35f, 3.0f);
        auto s = make_sheet(vec3(0, 1.5f, 0), vec3(1, 0, 0), vec3(0, 0, -1), 1.7f, 1.7f, 14, 14, 25.0f * 2.89f, 0, m, 5);
        const double a0 = shell_area(*s);
        CHECK(s->pattern_on() && std::fabs(s->shell_uvm.x - 1.7f) < 1e-3f && std::fabs(s->shell_uvm.y - 1.7f) < 1e-3f, "material plane: %.4f x %.4f m per uv",
              s->shell_uvm.x, s->shell_uvm.y);
        CHECK(s->add_impact(vec2(0.9f, 0.8f), 12.0f), "the impact was not laid");
        CHECK(!s->add_impact(vec2(0.92f, 0.8f), 12.0f), "a second impact on the same spot");
        for (int k = 0; k < 8 && s->pattern_passes > 0; k++) s->process_shell_events();
        int line_edges = 0, on_nodes = 0, near_nodes = 0;
        for (uint32_t si = 0; si < s->shells.size(); si++)
            for (int e = 0; e < 3; e++) line_edges += (s->shells[si].line >> e) & 1;
        for (uint32_t v = 0; v < s->nodes.size(); v++) {
            float lmin = 1e9f;
            for (uint32_t si : s->node_shells[v])
                for (int e = 0; e < 3; e++) lmin = std::min(lmin, s->shells[si].L0[e]);
            const PatternLine ln = s->pattern_nearest(s->node_x(v), 0.3f * lmin, 1);
            if (ln.d < 0.3f * lmin) {
                near_nodes++;
                on_nodes += ln.d < 1e-4f;
            }
        }
        printf("    glass web laid: %zu triangles (%zu authored), %d edges on the lines, %d of %d nodes near a line on it\n", s->shells.size(), (size_t)338,
               line_edges / 2, on_nodes, near_nodes);
        CHECK(line_edges > 100, "only %d edges on the lines", line_edges);
        CHECK(on_nodes * 3 > near_nodes * 2, "%d of %d nodes near a line are on it", on_nodes, near_nodes);
        CHECK(std::fabs(shell_area(*s) - a0) < 1e-4 * a0, "area %.6f vs %.6f", shell_area(*s), a0);
        check_topology(*s, "glass web");
        {
            // (the kernels compared on a strained copy: at rest the forces are rounding noise)
            SoftBody x(*s);
            TRng r(77);
            for (auto& n : x.nodes) n.p += vec3(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1)) * 0.002f;
            compare_kernels(x, "glass web");
        }
    }
    // balls into sheets of three materials
    struct Case {
        const char* name;
        ShellMaterial m;
        float kg_m2, speed;
        int frames;
        vec2 grain;
    };
    const float ga = 0.5f; // (wood: fibres at 0.5 rad from u)
    const Case cases[] = {
        {"glass", patterned(mat_glass(), ShellPattern::Radial, 0.35f, 3.0f), 25.0f, 9.0f, 40, vec2(1, 0)},
        {"steel", patterned(mat_steel(), ShellPattern::Punch, 0.14f, 8.0f), 47.0f, 24.0f, 40, vec2(1, 0)},
        {"plywood", patterned(mat_plywood(), ShellPattern::Grain, 0.2f, 5.0f, ga), 11.0f, 12.0f, 40, vec2(std::cos(ga), std::sin(ga))},
    };
    for (const Case& c : cases) {
        std::unique_ptr<SoftBody> sheet;
        const PatternRun a = run_pattern(c.m, c.kg_m2, c.speed, false, c.frames, vec2(0.1f, 0.05f), c.grain, &sheet);
        const double on = a.cs.len > 0 ? a.cs.near_line / a.cs.len : 0, along = a.cs.len > 0 ? a.cs.along / a.cs.len : 0;
        printf("    %-8s %d impacts, %d cracks, %zu bodies, %d crack edges: %.0f%% on the impact's lines, %.0f%% along the grain\n", c.name, a.impacts, a.cracks,
               a.bodies, a.cs.edges, on * 100, along * 100);
        CHECK(a.finite, "%s: non-finite state", c.name);
        CHECK(a.impacts >= 1, "%s: no pattern laid", c.name);
        CHECK(a.cracks > 10, "%s: %d cracks", c.name, a.cracks);
        CHECK(std::fabs(a.area - a.area0) < 1e-4 * a.area0, "%s: area %.6f vs %.6f", c.name, a.area, a.area0);
        if (c.m.pattern != ShellPattern::Grain && sheet) {
            // the same impact on the sheet without a pattern, measured against the same lines
            ShellMaterial plain = c.m;
            plain.pattern = ShellPattern::None;
            const PatternRun b = run_pattern(plain, c.kg_m2, c.speed, false, c.frames, vec2(0.1f, 0.05f), c.grain, nullptr, &sheet->shell_impacts);
            const double base = b.cs.len > 0 ? b.cs.near_line / b.cs.len : 0;
            printf("             without the pattern: %d cracks, %.0f%% on the same lines\n", b.cracks, base * 100);
            CHECK(on > 1.4 * base, "%s: %.0f%% of the crack length on the pattern's lines, %.0f%% without the pattern", c.name, on * 100, base * 100);
        }
        if (sheet) compare_kernels(*sheet, c.name);
        const PatternRun b = run_pattern(c.m, c.kg_m2, c.speed, true, c.frames, vec2(0.1f, 0.05f), c.grain);
        CHECK(a.hash == b.hash, "%s: multithreaded run differs from the single-threaded one (%.9g vs %.9g)", c.name, a.hash, b.hash);
    }
    // wood: the cracks run along the fibres, whichever way they go (against the same sheet without a pattern: the
    // mesh's own directions)
    for (float gang : {0.5f, 1.5707963f}) {
        const vec2 g(std::cos(gang), std::sin(gang));
        const PatternRun a = run_pattern(patterned(mat_plywood(), ShellPattern::Grain, 0.2f, 5.0f, gang), 11.0f, 12.0f, false, 40, vec2(0.1f, 0.05f), g);
        const PatternRun b = run_pattern(mat_plywood(), 11.0f, 12.0f, false, 40, vec2(0.1f, 0.05f), g);
        const double along = a.cs.len > 0 ? a.cs.along / a.cs.len : 0, base = b.cs.len > 0 ? b.cs.along / b.cs.len : 0;
        printf("    plywood, fibres at %.2f rad: %.0f%% of the crack length along them (%.0f%% without the grain)\n", gang, along * 100, base * 100);
        CHECK(along > 1.5 * base && along > 0.35, "plywood (fibres at %.2f): %.0f%% of the crack length along the grain, %.0f%% without", gang, along * 100,
              base * 100);
    }
}

// ------------------------------------------------------------------------------------------ coarsening
static void test_coarsen() {
    printf("coarsening: a settled sheet goes back to its authored triangles\n");
    for (int depth = 1; depth <= 3; depth++) {
        // uniform: every triangle refined `depth` times, then coarsened back
        auto s = make_sheet(vec3(0, 1.5f, 0), vec3(1, 0, 0), vec3(0, 0, -1), 1.7f, 1.7f, 14, 14, 100.0f, 0, mat_lead(), 7);
        const size_t n0 = s->shells.size();
        for (int d = 0; d < depth; d++) {
            const size_t ns = s->shells.size();
            for (uint32_t si = 0; si < ns; si++)
                if (s->shells[si].level < d + 1) s->shell_events.push_back({si, 0, 0});
            s->process_shell_events();
            // (a pass may leave some at the old level: the neighbour was split first)
            for (int k = 0; k < 4; k++) {
                bool any = false;
                for (uint32_t si = 0; si < s->shells.size(); si++)
                    if (s->shells[si].level < d + 1) { s->shell_events.push_back({si, 0, 0}); any = true; }
                if (!any) break;
                s->process_shell_events();
            }
        }
        const size_t n1 = s->shells.size();
        int rounds = 0;
        for (; rounds < 20 && s->coarsen_shells(100000); rounds++) check_topology(*s, "uniform coarsening");
        printf("    uniform depth %d: %zu -> %zu -> %zu triangles in %d rounds\n", depth, n0, n1, s->shells.size(), rounds);
        CHECK(s->shells.size() == n0, "uniform depth %d: %zu triangles left of %zu", depth, s->shells.size(), n0);
    }
    for (int variant = 0; variant < 4; variant++) {
        // 0: refined only; 1: refined + cracks; 2: glass with a fracture pattern laid; 3: clamped border (fixed nodes)
        const ShellMaterial m = variant == 2 ? patterned(mat_glass(), ShellPattern::Radial, 0.35f, 3.0f) : mat_lead();
        auto s = make_sheet(vec3(0, 1.5f, 0), vec3(1, 0, 0), vec3(0, 0, -1), 1.7f, 1.7f, 14, 14, 100.0f, variant == 3 ? 1 : 0, m, 40 + variant);
        const size_t n0 = s->shells.size();
        const double a0 = shell_area(*s);
        double mass0 = 0;
        for (auto& n : s->nodes) mass0 += n.mass;
        TRng r(9 + variant);
        if (variant == 2) {
            s->add_impact(vec2(0.9f, 0.8f), 12.0f);
            for (int k = 0; k < 8 && s->pattern_passes > 0; k++) s->process_shell_events();
        }
        random_topology(*s, r, 10, 40, variant == 1 ? 6 : 0);
        const size_t n1 = s->shells.size();
        int merges = 0, rounds = 0;
        for (; rounds < 40; rounds++) {
            const int k = s->coarsen_shells(100000);
            if (!k) break;
            merges += k;
            check_topology(*s, "coarsening");
        }
        double mass = 0;
        for (auto& n : s->nodes) mass += n.mass;
        char name[64];
        snprintf(name, sizeof name, "coarsen variant %d", variant);
        printf("    %s: %zu -> %zu -> %zu triangles, %d merges in %d rounds, finest level %d\n", name, n0, n1, s->shells.size(), merges, rounds, s->shell_level);
        CHECK(merges > 0, "%s: nothing merged", name);
        CHECK(std::fabs(shell_area(*s) - a0) < 1e-4 * a0, "%s: area %.6f vs %.6f", name, shell_area(*s), a0);
        CHECK(std::fabs(mass - mass0) < 1e-4 * mass0, "%s: mass %.6f vs %.6f", name, mass, mass0);
        if (variant != 1) CHECK(s->shells.size() <= n0 + n0 / 10, "%s: %zu triangles left of %zu authored", name, s->shells.size(), n0);
        else CHECK(s->shells.size() < n1 / 2, "%s: %zu triangles left of %zu refined", name, s->shells.size(), n1);
        {
            SoftBody x(*s);
            TRng r2(3);
            for (auto& n : x.nodes) n.p += vec3(r2.range(-1, 1), r2.range(-1, 1), r2.range(-1, 1)) * 0.002f;
            compare_kernels(x, name);
        }
        // and it refines again like a fresh sheet
        random_topology(*s, r, 3, 40, 0);
        check_topology(*s, name);
        CHECK(std::fabs(shell_area(*s) - a0) < 1e-4 * a0, "%s: area after refining again %.6f vs %.6f", name, shell_area(*s), a0);
    }
}

// ------------------------------------------------------------------------------------------ rigid pieces
static void test_rigid() {
    printf("rigid pieces: a shard moves as one body, keeps its momentum, comes to rest\n");
    // a small free sheet made rigid while spinning: momentum and angular momentum kept, the shape kept
    {
        auto s = make_sheet(vec3(0, 3, 0), vec3(1, 0, 0), vec3(0, 0, -1), 0.4f, 0.4f, 5, 5, 2.0f, 0, mat_glass(), 3);
        const vec3 w0(0, 0, 3.0f), v0(1, 0, 0);
        vec3 com(0);
        float M = 0;
        for (auto& n : s->nodes) com += n.p * n.mass, M += n.mass;
        com = com / M;
        for (auto& n : s->nodes) n.v = v0 + cross(w0, n.p - com);
        vec3 P0(0), L0(0);
        for (auto& n : s->nodes) P0 += n.v * n.mass, L0 += cross(n.p - com, n.v) * n.mass;
        s->make_rigid();
        vec3 P1(0), L1(0);
        for (auto& n : s->nodes) P1 += n.v * n.mass, L1 += cross(n.p - com, n.v) * n.mass;
        CHECK(length(P1 - P0) < 1e-4f * length(P0) && length(L1 - L0) < 1e-3f * length(L0), "make_rigid: momentum %.4f -> %.4f, angular %.4f -> %.4f",
              length(P0), length(P1), length(L0), length(L1));
        CHECK(std::fabs(length(s->rb.w) - 3.0f) < 0.05f, "make_rigid: spin %.3f rad/s (3 expected)", length(s->rb.w));
        // a second of free flight with gravity: the centre falls like a point mass, the shape does not change
        const vec3 c0 = s->rb.com;
        const float d01 = length(s->nodes[0].p - s->nodes[7].p);
        for (auto& n : s->nodes) (void)n;
        for (int k = 0; k < 2000; k++) {
            s->clear_forces(vec3(0, -9.81f, 0));
            vec3 mn(1e30f), mx(-1e30f);
            float v2 = 0;
            s->rigid_step(0.0005f, false, mn, mx, v2);
        }
        const vec3 expect = c0 + v0 * 1.0f + vec3(0, -0.5f * 9.81f, 0);
        CHECK(length(s->rb.com - expect) < 0.01f, "rigid flight: centre at (%.3f %.3f %.3f), expected (%.3f %.3f %.3f)", s->rb.com.x, s->rb.com.y, s->rb.com.z,
              expect.x, expect.y, expect.z);
        CHECK(std::fabs(length(s->nodes[0].p - s->nodes[7].p) - d01) < 1e-4f, "rigid flight: the shape changed (%.5f -> %.5f)", d01,
              length(s->nodes[0].p - s->nodes[7].p));
    }
    if (getenv("BL_RIGIDDBG")) {
        // a rigid shard spinning flat on the ground: friction stops it
        World w;
        w.statics.terrain.create(21, 21, 1.0f, vec2(-10, -10));
        w.statics.has_terrain = true;
        w.statics.terrain.update_bounds();
        auto s = make_sheet(vec3(0, 0.03f, 0), vec3(1, 0, 0), vec3(0, 0, -1), 0.3f, 0.3f, 4, 4, 0.5f, 0, mat_glass(), 3);
        s->collision_radius = 0.04f;
        vec3 com(0); float M = 0;
        for (auto& n : s->nodes) com += n.p * n.mass, M += n.mass;
        com = com / M;
        for (auto& n : s->nodes) n.v = cross(vec3(0, 6, 0), n.p - com);
        s->make_rigid();
        SoftBody* b = w.add_body(std::move(s));
        for (int f = 0; f < 60; f++) {
            w.step_substeps(33);
            if (f % 6 == 0) printf("DBG spin t=%.2f w=%.3f v=%.3f com y=%.4f contacts %d sleeping %d\n", (f + 1) / 60.0, length(b->rb.w), length(b->rb.v), b->rb.com.y, b->static_contacts, (int)b->sleeping);
        }
    }
    // a glass sheet in the world with rigid pieces: the pieces that fall off are rigid, land, sleep; area kept
    {
        World w;
        w.statics.terrain.create(21, 21, 1.0f, vec2(-10, -10)); // (flat ground at y = 0)
        w.statics.has_terrain = true;
        w.statics.terrain.update_bounds();
        auto sheet = make_sheet(vec3(0, 1.3f, 0), vec3(1, 0, 0), vec3(0, 0, -1), 1.7f, 1.7f, 14, 14, 25.0f * 2.89f, 1, mat_glass(), 21);
        const double a0 = shell_area(*sheet);
        w.add_body(std::move(sheet));
        w.add_body(make_ball(vec3(0.05f, 1.9f, 0.03f), 0.22f, 60.0f, vec3(0, -9.0f, 0)));
        for (int f = 0; f < 180; f++) w.step_substeps(33);
        int pieces = 0, rigid = 0, asleep = 0, off = 0;
        double area = 0;
        bool finite = true;
        float vmax = 0;
        for (auto& b : w.bodies()) {
            if (b->shells.empty()) continue;
            area += shell_area(*b);
            for (auto& n : b->nodes) finite &= std::isfinite(n.p.x + n.p.y + n.p.z);
            if (!b->is_piece) continue;
            pieces++;
            rigid += b->rigid;
            asleep += b->sleeping;
            for (auto& n : b->nodes) {
                vmax = std::max(vmax, length(n.v));
                if (n.p.y < -0.5f) off++;
            }
        }
        printf("    %d pieces, %d rigid, %d asleep after 3 s, top speed %.2f m/s\n", pieces, rigid, asleep, vmax);
        if (getenv("BL_RIGIDDBG")) {
            int hist[6] = {0};
            for (auto& b : w.bodies()) {
                if (!b->is_piece || b->sleeping) continue;
                float v = 0, ymin = 1e9f;
                for (auto& n : b->nodes) v = std::max(v, length(n.v)), ymin = std::min(ymin, n.p.y);
                hist[std::min(5, (int)(v / 0.1f))]++;
                if (v > 0.3f) printf("DBG piece %s: %zu nodes, speed %.2f, lowest y %.3f, com y %.3f, w %.2f, timer %.2f\n", b->name.c_str(), b->nodes.size(), v, ymin, b->rb.com.y, length(b->rb.w), b->sleep_timer);
            }
            printf("DBG awake piece speeds: <0.1: %d, <0.2: %d, <0.3: %d, <0.4: %d, <0.5: %d, more: %d\n", hist[0], hist[1], hist[2], hist[3], hist[4], hist[5]);
        }
        CHECK(pieces > 5 && rigid == pieces, "%d pieces, %d rigid", pieces, rigid);
        CHECK(finite && off == 0, "finite %d, %d nodes below the ground", (int)finite, off);
        CHECK(asleep * 2 >= pieces, "%d of %d pieces asleep after 3 s", asleep, pieces);
        CHECK(std::fabs(area - a0) < 1e-4 * a0, "area %.6f vs %.6f", area, a0);
    }
}

static void test_laser() {
    printf("laser: cuts along the swept plane, nothing moves\n");
    // a free lead sheet lying at y 1.5, a laser from above sweeping across it along x = 0.1
    for (int variant = 0; variant < 3; variant++) {
        const ShellMaterial m = variant == 0 ? mat_lead() : variant == 1 ? mat_glass() : mat_fabric();
        auto s = make_sheet(vec3(0, 1.5f, 0), vec3(1, 0, 0), vec3(0, 0, -1), 1.7f, 1.7f, 14, 14, 100.0f, 0, m, 90 + variant);
        s->compute_aabb();
        const double a0 = shell_area(*s);
        std::vector<vec3> p0;
        for (auto& n : s->nodes) p0.push_back(n.p);
        const vec3 o(0.1f, 5.0f, 0.0f);
        // full cut: the sector reaches past both edges of the sheet
        const int cuts = s->cut_shells(o, normalize(vec3(0, -3.5f, -2.0f)), normalize(vec3(0, -3.5f, 2.0f)), 50.0f);
        char name[64];
        snprintf(name, sizeof name, "full cut, material %d", variant);
        CHECK(cuts > 10, "%s: %d links cut", name, cuts);
        {
            // every free edge is the border or the cut (drawn scorched)
            int unmarked = 0, marked = 0;
            for (const Shell& sh : s->shells)
                for (int e = 0; e < 3; e++)
                    if (sh.nb[e] < 0) {
                        if ((sh.edges >> (3 + e)) & 1) marked++;
                        else if (!((sh.edges >> e) & 1)) unmarked++;
                    }
            CHECK(unmarked == 0 && marked >= 2 * cuts, "%s: %d cut edges marked, %d free edges neither border nor cut", name, marked, unmarked);
        }
        check_topology(*s, name);
        float moved = 0;
        for (size_t i = 0; i < p0.size(); i++) moved = std::max(moved, length(s->nodes[i].p - p0[i]));
        CHECK(moved == 0.0f, "%s: nodes moved by %.3g m", name, moved);
        std::vector<std::unique_ptr<SoftBody>> pieces;
        s->pieces_check = true;
        s->detach_pieces(pieces);
        CHECK(pieces.size() == 1, "%s: %zu pieces cut off (1 expected)", name, pieces.size());
        double a = shell_area(*s);
        for (auto& p : pieces) a += shell_area(*p);
        CHECK(std::fabs(a - a0) < 1e-4 * a0, "%s: area %.6f vs %.6f", name, a, a0);
        // each part on its own side of the cut (triangle centroids against x = 0.1, within a finest triangle)
        auto one_side = [](const SoftBody& b) {
            int neg = 0, pos = 0;
            for (auto& sh : b.shells) {
                const float x = (b.nodes[sh.n[0]].p.x + b.nodes[sh.n[1]].p.x + b.nodes[sh.n[2]].p.x) / 3.0f - 0.1f;
                if (x < -0.04f) neg++;
                if (x > 0.04f) pos++;
            }
            return neg == 0 || pos == 0;
        };
        CHECK(one_side(*s) && (pieces.empty() || one_side(*pieces[0])), "%s: a part lies on both sides of the cut", name);
    }
    // cuts along (and very near) the grid lines of the mesh: the plane through rows of nodes, or a hair beside them,
    // at a small slope; the sheet must come apart all the same
    for (int variant = 0; variant < 4; variant++) {
        auto s = make_sheet(vec3(0, 1.5f, 0), vec3(1, 0, 0), vec3(0, 0, -1), 3.0f, 2.0f, 13, 9, 47.0f * 6.0f, 0, mat_lead(), 11);
        s->compute_aabb();
        const float off[4] = {0.0f, 1e-5f, -1e-5f, 0.002f}, slope[4] = {0.0f, 0.0f, 0.003f, 0.01f};
        // the plane through the camera above x = 0, z = off, tilted by `slope` across the sheet
        const vec3 o(0.0f, 5.0f, off[variant]);
        const int cuts = s->cut_shells(o, normalize(vec3(-3.0f, -3.5f, -3.0f * slope[variant])), normalize(vec3(3.0f, -3.5f, 3.0f * slope[variant])), 50.0f);
        std::vector<std::unique_ptr<SoftBody>> pieces;
        s->pieces_check = true;
        s->detach_pieces(pieces);
        CHECK(pieces.size() == 1, "cut along the grid, variant %d: %d links cut, %zu pieces cut off (1 expected)", variant, cuts, pieces.size());
    }
    // a sweep in narrow steps (the mouse over 40 frames) across a gate-like sheet: no link left across the cut
    for (int variant = 0; variant < 2; variant++) {
        auto s = make_sheet(vec3(0, 2.3f, 0), vec3(1, 0, 0), vec3(0, 1, 0), 6.0f, 4.0f, 25, 17, 47.0f * 24.0f, 2, mat_steel(), 12);
        s->compute_aabb();
        const vec3 o(0.5f, 2.6f, -2.4f);
        auto target = [&](float t) { return vec3(-3.2f + 6.4f * t, (variant == 0 ? 2.4f : 2.3f) - (variant == 0 ? 0.2f : 0.0f) * t, 0.0f); };
        int cuts = 0;
        vec3 prev = normalize(target(0) - o);
        for (int k = 1; k <= 40; k++) {
            const vec3 d = normalize(target((float)k / 40.0f) - o);
            cuts += s->cut_shells(o, prev, d, 150.0f);
            prev = d;
        }
        // the parts joined by edges: the cut runs from side to side, the part above and the part below are apart
        std::vector<int> comp(s->shells.size(), -1);
        int ncomp = 0;
        for (uint32_t s0 = 0; s0 < s->shells.size(); s0++) {
            if (comp[s0] >= 0) continue;
            std::vector<uint32_t> st{s0};
            comp[s0] = ncomp;
            while (!st.empty()) {
                const uint32_t x = st.back();
                st.pop_back();
                for (int e = 0; e < 3; e++) {
                    const int j = s->shells[x].nb[e];
                    if (j >= 0 && comp[j] < 0) {
                        comp[j] = ncomp;
                        st.push_back((uint32_t)j);
                    }
                }
            }
            ncomp++;
        }
        auto part_at = [&](vec3 p) {
            float best = 1e9f;
            int c = -1;
            for (uint32_t si = 0; si < s->shells.size(); si++) {
                const Shell& sh = s->shells[si];
                const float d = length((s->nodes[sh.n[0]].p + s->nodes[sh.n[1]].p + s->nodes[sh.n[2]].p) / 3.0f - p);
                if (d < best) best = d, c = comp[si];
            }
            return c;
        };
        const int up = part_at(vec3(0, 3.5f, 0)), down = part_at(vec3(0, 1.0f, 0));
        CHECK(up != down, "sweep in steps across a gate, variant %d: %d links cut, the parts above and below still joined (%d parts)", variant, cuts, ncomp);
    }
    {
        // a slit: the sector ends in the middle of the sheet, the sheet stays in one piece
        auto s = make_sheet(vec3(0, 1.5f, 0), vec3(1, 0, 0), vec3(0, 0, -1), 1.7f, 1.7f, 14, 14, 100.0f, 0, mat_lead(), 95);
        s->compute_aabb();
        const int cuts = s->cut_shells(vec3(0.1f, 5, 0), normalize(vec3(0, -3.5f, -2.0f)), normalize(vec3(0, -3.5f, 0)), 50.0f);
        check_topology(*s, "slit");
        std::vector<std::unique_ptr<SoftBody>> pieces;
        s->pieces_check = true;
        s->detach_pieces(pieces);
        CHECK(cuts > 3 && pieces.empty(), "slit: %d links cut, %zu pieces (a slit, not a cut through)", cuts, pieces.size());
    }
    {
        // in the world: a clamped sheet cut down the middle keeps hanging in two halves; a ring cut twice gives two pieces
        World W;
        SoftBody* sb = W.add_body(make_sheet(vec3(0, 1.3f, 0), vec3(1, 0, 0), vec3(0, 1, 0), 3.6f, 2.4f, 15, 11, 500.0f, 3, mat_lead(), 96));
        for (int f = 0; f < 5; f++) W.step_substeps(33);
        const double a0 = shell_area(*sb);
        const int cuts = W.laser_cut(vec3(0.05f, 1.3f, 6.0f), normalize(vec3(0, 3, -6)), normalize(vec3(0, -3, -6)), 50.0f);
        for (int f = 0; f < 90; f++) W.step_substeps(33);
        double area = 0;
        bool finite = true;
        for (auto& b : W.bodies()) {
            area += shell_area(*b);
            for (auto& n : b->nodes) finite &= std::isfinite(n.p.x + n.p.y + n.p.z);
        }
        CHECK(cuts > 10 && finite, "hanging sheet: %d links cut, finite %d", cuts, (int)finite);
        CHECK(std::fabs(area - a0) < 1e-4 * a0, "hanging sheet: area %.6f vs %.6f", area, a0);
        // two halves (both hang from the bar, so they stay one body): two parts joined by no edge
        std::vector<int> part(sb->shells.size(), -1);
        int parts = 0;
        for (size_t s0 = 0; s0 < sb->shells.size(); s0++) {
            if (part[s0] >= 0) continue;
            std::vector<size_t> stack{s0};
            part[s0] = parts;
            while (!stack.empty()) {
                const size_t k = stack.back();
                stack.pop_back();
                for (int e = 0; e < 3; e++) {
                    const int j = sb->shells[k].nb[e];
                    if (j >= 0 && part[j] < 0) {
                        part[j] = parts;
                        stack.push_back((size_t)j);
                    }
                }
            }
            parts++;
        }
        printf("    hanging lead sheet cut top to bottom: %d links cut, %d parts\n", cuts, parts);
        CHECK(parts == 2, "hanging sheet: %d parts after the cut", parts);
    }
    {
        // beams: a ball cut through its middle loses the beams that cross the plane
        auto ball = make_ball(vec3(0, 1, 0), 0.3f, 20, vec3(0));
        World W;
        SoftBody* b = W.add_body(std::move(ball));
        const int cut = W.laser_cut(vec3(0.02f, 1, 5), normalize(vec3(0, 2, -5)), normalize(vec3(0, -2, -5)), 50.0f);
        int broken = 0;
        for (auto& bm : b->beams) broken += (bm.flags & BF_BROKEN) ? 1 : 0;
        int torn = 0;
        for (auto& t : b->tris) torn += t.torn ? 1 : 0;
        CHECK(broken > 0 && cut == broken + torn, "ball: %d cut, %d beams broken, %d surface triangles torn", cut, broken, torn);
    }
    // many random cuts into hanging sheets, then the dynamics: nothing blows up
    for (int variant = 0; variant < 3; variant++) {
        const ShellMaterial m = variant == 0 ? mat_lead() : variant == 1 ? mat_glass() : mat_fabric();
        World W;
        SoftBody* sb = W.add_body(make_sheet(vec3(0, 1.4f, 0), vec3(1, 0, 0), vec3(0, 1, 0), 3.6f, 2.4f, 15, 11, 300.0f, 3, m, 120 + variant));
        const double a0 = shell_area(*sb);
        TRng r(700 + variant);
        int cuts = 0;
        for (int k = 0; k < 12; k++) {
            for (int f = 0; f < 3; f++) W.step_substeps(33);
            const vec3 o(r.range(-1.5f, 1.5f), r.range(0.5f, 2.3f), 5.0f);
            const vec3 t0(r.range(-2.0f, 2.0f), r.range(0.2f, 2.6f), 0), t1(r.range(-2.0f, 2.0f), r.range(0.2f, 2.6f), 0);
            cuts += W.laser_cut(o, normalize(t0 - o), normalize(t1 - o), 50.0f);
        }
        float vmax = 0;
        bool finite = true;
        for (int f = 0; f < 60; f++) W.step_substeps(33);
        double area = 0;
        for (auto& b : W.bodies()) {
            area += shell_area(*b);
            check_topology(*b, "after random cuts");
            for (auto& n : b->nodes) {
                finite &= std::isfinite(n.p.x + n.p.y + n.p.z);
                vmax = std::max(vmax, length(n.v));
            }
        }
        printf("    12 random cuts, material %d: %d links cut, %zu bodies, top speed after 1 s %.1f m/s\n", variant, cuts, W.bodies().size(), vmax);
        CHECK(finite && vmax < 30.0f, "random cuts, material %d: finite %d, top speed %.1f m/s", variant, (int)finite, vmax);
        CHECK(std::fabs(area - a0) < 2e-4 * a0, "random cuts, material %d: area %.5f vs %.5f", variant, area, a0);
    }
    // shapes: a ring and a dome cut through
    for (const ShapeCase& c : shape_cases(vec3(0, 1.5f, 0), 0)) {
        if (std::string(c.name) != "ring 2.4 m" && std::string(c.name) != "dome" && std::string(c.name) != "L 2.4 m") continue;
        auto s = make_shape(c, mat_lead(), 97);
        s->compute_aabb();
        const double a0 = shell_area(*s);
        const int cuts = s->cut_shells(vec3(0.07f, 6, 0), normalize(vec3(0, -4.5f, -3.0f)), normalize(vec3(0, -4.5f, 3.0f)), 50.0f);
        std::vector<std::unique_ptr<SoftBody>> pieces;
        s->pieces_check = true;
        s->detach_pieces(pieces);
        double a = shell_area(*s);
        for (auto& p : pieces) {
            a += shell_area(*p);
            check_topology(*p, c.name);
        }
        check_topology(*s, c.name);
        const size_t expect = std::string(c.name) == "ring 2.4 m" ? 2 : 1; // (a ring cut across falls into halves: the kept one and ... )
        printf("    %-10s %d links cut, %zu pieces\n", c.name, cuts, pieces.size());
        CHECK(pieces.size() >= 1 && pieces.size() <= expect, "%s: %zu pieces", c.name, pieces.size());
        CHECK(std::fabs(a - a0) < 1e-4 * a0, "%s: area %.6f vs %.6f", c.name, a, a0);
    }
}

static void test_stability() {
    printf("stability: a clamped sheet refined to the finest level settles\n");
    World w;
    w.settings.coarsen_per_frame = 0; // (the sheet is refined by hand: it must stay so)
    auto s = make_sheet(vec3(0, 2, 0), vec3(1, 0, 0), vec3(0, 0, -1), 1.7f, 1.7f, 14, 14, 136.0f, 1, mat_lead(), 5);
    SoftBody* b = w.add_body(std::move(s));
    b->shell_cap = 1u << 30;
    for (int pass = 0; pass < 4; pass++) {
        const uint32_t n = (uint32_t)b->shells.size();
        for (uint32_t i = 0; i < n; i++) b->shell_events.push_back({i, 0, 0});
        b->process_shell_events();
    }
    b->allow_break = false; // (only the dynamics)
    size_t mid = 0;
    for (size_t i = 0; i < b->nodes.size(); i++)
        if (length(b->nodes[i].p - vec3(0, 2, 0)) < length(b->nodes[mid].p - vec3(0, 2, 0))) mid = i;
    const vec3 p0 = b->nodes[mid].p;
    for (int f = 0; f < 150; f++) w.step_substeps(33);
    float vmax = 0, drift = 0;
    bool finite = true;
    for (auto& n : b->nodes) {
        vmax = std::max(vmax, length(n.v));
        finite &= std::isfinite(n.p.x + n.p.y + n.p.z);
    }
    // (the world renumbers a sheet that grew: the node is found again by where it hangs, not by its index)
    size_t mid2 = 0;
    for (size_t i = 0; i < b->nodes.size(); i++) {
        auto dxz = [&](size_t k) { return (b->nodes[k].p.x - p0.x) * (b->nodes[k].p.x - p0.x) + (b->nodes[k].p.z - p0.z) * (b->nodes[k].p.z - p0.z); };
        if (dxz(i) < dxz(mid2)) mid2 = i;
    }
    drift = length(b->nodes[mid2].p - p0);
    CHECK(finite, "non-finite nodes");
    CHECK(b->shell_level == 4, "refinement level %d", b->shell_level);
    CHECK(vmax < 0.5f || b->sleeping, "still moving at %.3f m/s after 2.5 s (%zu tris)", vmax, b->shells.size());
    CHECK(drift < 0.3f, "centre sagged %.3f m", drift);
    printf("    %zu triangles, max speed %.4f m/s, sag %.3f m\n", b->shells.size(), vmax, drift);
}

// ------------------------------------------------------------------------------------------ timing
static void bench() {
    printf("bench: sheet force kernel\n");
    auto s = make_sheet(vec3(0, 1.4f, 0), vec3(1, 0, 0), vec3(0, 1, 0), 3.6f, 2.4f, 15, 11, 588.0f, 2, mat_lead(), 9);
    s->shell_cap = 1u << 30;
    for (int pass = 0; pass < 4; pass++) {
        const uint32_t n = (uint32_t)s->shells.size();
        for (uint32_t i = 0; i < n; i++) s->shell_events.push_back({i, 0, 0});
        s->process_shell_events();
    }
    TRng r(3);
    jitter(*s, r, 0.002f, 0.05f);
    s->allow_break = false;
    s->allow_deform = false;
    int hinges = 0;
    for (uint32_t si = 0; si < s->shells.size(); si++)
        for (int e = 0; e < 3; e++) hinges += s->shells[si].nb[e] > (int)si;
    const int reps = 400;
    ref::RefBody A(*s);
    for (int pass = 0; pass < 2; pass++) {
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < reps; i++) {
            if (pass == 0) {
                A.clear_forces(vec3(0));
                A.ref_forces(kDefaultDt, 0, 1);
            } else {
                s->clear_forces(vec3(0));
                s->compute_shell_forces(kDefaultDt, 0, 1);
            }
        }
        double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / reps;
        printf("    %s: %zu triangles, %d hinges, %zu nodes: %.1f us per evaluation, %.1f ns per triangle\n", pass == 0 ? "reference" : "current",
               s->shells.size(), hinges, s->nodes.size(), us, us * 1000.0 / s->shells.size());
    }
    {
        const KernelCacheReport cr = measure_kernel_cache(*s);
        printf("%s", cr.text().c_str());
    }
    {
        // the same sheet renumbered along a Morton curve (as the world does it once a fifth of a sheet is new)
        SoftBody o(*s);
        o.reorder_shells();
        o.clear_forces(vec3(0));
        o.compute_shell_forces(kDefaultDt, 0, 1);
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < reps; i++) {
            o.clear_forces(vec3(0));
            o.compute_shell_forces(kDefaultDt, 0, 1);
        }
        double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / reps;
        printf("    current, Morton order: %.1f us per evaluation, %.1f ns per triangle\n", us, us * 1000.0 / o.shells.size());
        const KernelCacheReport cr = measure_kernel_cache(o);
        printf("%s", cr.text().c_str());
    }
    {
        // parts of the current kernel
        double t_begin = 0, t_eval = 0, t_end = 0, t_gather = 0;
        for (int i = 0; i < reps; i++) {
            s->clear_forces(vec3(0));
            auto t0 = std::chrono::steady_clock::now();
            const int ch = s->shell_begin(kDefaultDt, 0, 1);
            auto t1 = std::chrono::steady_clock::now();
            for (int c = 0; c < ch; c++) s->shell_eval(c);
            auto t2 = std::chrono::steady_clock::now();
            s->shell_end();
            auto t3 = std::chrono::steady_clock::now();
            s->shell_gather(0, s->nodes.size(), s->force.data());
            auto t4 = std::chrono::steady_clock::now();
            t_begin += std::chrono::duration<double, std::micro>(t1 - t0).count();
            t_eval += std::chrono::duration<double, std::micro>(t2 - t1).count();
            t_end += std::chrono::duration<double, std::micro>(t3 - t2).count();
            t_gather += std::chrono::duration<double, std::micro>(t4 - t3).count();
        }
        printf("    current parts (us): begin %.1f, elements %.1f, events %.1f, gather %.1f\n", t_begin / reps, t_eval / reps, t_end / reps,
               t_gather / reps);
    }
}

// ./build/test_physics kernel [seconds]: only the kernel in a loop (for sampling profilers)
static void kernel_loop(double seconds) {
    auto s = make_sheet(vec3(0, 1.4f, 0), vec3(1, 0, 0), vec3(0, 1, 0), 3.6f, 2.4f, 15, 11, 588.0f, 2, mat_lead(), 9);
    s->shell_cap = 1u << 30;
    for (int pass = 0; pass < 4; pass++) {
        const uint32_t n = (uint32_t)s->shells.size();
        for (uint32_t i = 0; i < n; i++) s->shell_events.push_back({i, 0, 0});
        s->process_shell_events();
    }
    TRng r(3);
    jitter(*s, r, 0.002f, 0.05f);
    s->allow_break = false;
    s->allow_deform = false;
    s->reorder_shells();
    const auto t0 = std::chrono::steady_clock::now();
    long n = 0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
        for (int i = 0; i < 100; i++) {
            s->clear_forces(vec3(0));
            s->compute_shell_forces(kDefaultDt, 0, 1);
        }
        n += 100;
    }
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / n;
    printf("kernel: %.1f us per evaluation, %.1f ns per triangle\n", us, us * 1000 / s->shells.size());
}

// ./build/test_physics team: cost of a parallel phase (Team) with trivial and with real chunks
static void team_bench() {
    std::vector<double> sink(4096, 0.0);
    for (int work_ns : {0, 2000, 10000}) {
        for (int chunks : {1, 4, 12, 48}) {
            const int phases = 20000;
            JobSystem::get().parallel_items(1, [&](int, int) {
                Team team(true);
                auto t0 = std::chrono::steady_clock::now();
                for (int p = 0; p < phases; p++)
                    team.run(chunks, [&](int c) {
                        if (work_ns == 0) return;
                        auto s0 = std::chrono::steady_clock::now();
                        double x = sink[c];
                        while (std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - s0).count() < work_ns) x += 1.0;
                        sink[c] = x;
                    });
                double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / phases;
                double ideal = (double)work_ns * 1e-3 * chunks / JobSystem::get().num_threads();
                printf("    %2d chunks of %5d ns: %.2f us per phase (ideal %.2f)\n", chunks, work_ns, us, std::max(ideal, work_ns * 1e-3));
            });
        }
    }
}

// ------------------------------------------------------------------------------------------------ frame elements
// The FEM frame against the engineering results it has to reproduce: a cantilever's tip deflection and rotation, its
// twist and stretch, a portal frame's sway, the first bending frequency; a free frame spinning keeps its shape and
// energy; plastic hinges hold the plastic moment, keep their set and break past the material's ductility; a frame
// with a sheet skin dropped on the ground stays sound and steps alike on one thread and many.
namespace {

FrameSection frame_tube(float damping) {
    FrameSection s = make_frame_section("Steel", FrameShape::Tube, 0.04f, 0.002f);
    s.damping = damping;
    return s;
}

// n members from p0 to p1, the first node clamped (position and rotation), lumped masses of the members; the joint
// of the first member at the root
std::unique_ptr<SoftBody> frame_line(int n, vec3 p0, vec3 p1, const FrameSection& s, uint8_t root = FJ_RIGID) {
    auto b = std::make_unique<SoftBody>();
    b->name = "frame line";
    b->can_sleep = false;
    const uint16_t si = b->fem.add_section(s);
    const float m = s.mass_per_m() * length(p1 - p0) / n;
    for (int i = 0; i <= n; i++) b->add_node(p0 + (p1 - p0) * ((float)i / n), i == 0 || i == n ? m * 0.5f : m, i == 0 ? NF_FIXED : NF_NONE);
    for (int i = 0; i < n; i++) b->fem.add_element(i, i + 1, si, i == 0 ? root : FJ_RIGID);
    b->fem.finalize(*b);
    return b;
}

// a straight chain of n members between two nodes that exist already (the frame's nodes added between them)
void frame_chain(SoftBody& b, uint32_t a, uint32_t c, int n, uint16_t si, std::vector<uint32_t>* made = nullptr) {
    const FrameSection& s = b.fem.sections[si];
    const vec3 p0 = b.nodes[a].p, p1 = b.nodes[c].p;
    const float m = s.mass_per_m() * length(p1 - p0) / n;
    uint32_t prev = a;
    b.nodes[a].mass += m * 0.5f;
    b.nodes[c].mass += m * 0.5f;
    for (int i = 1; i <= n; i++) {
        uint32_t cur = c;
        if (i < n) {
            cur = b.add_node(p0 + (p1 - p0) * ((float)i / n), m, b.info[a].flags & ~NF_FIXED);
            if (made) made->push_back(cur);
        }
        b.fem.add_element(prev, cur, si);
        prev = cur;
    }
}

void fix_inv_mass(SoftBody& b) {
    for (size_t i = 0; i < b.nodes.size(); i++) b.nodes[i].inv_mass = (b.info[i].flags & NF_FIXED) || b.nodes[i].mass <= 0 ? 0.0f : 1.0f / b.nodes[i].mass;
}

float tip_rotation(const SoftBody& b, uint32_t n, int axis) {
    const vec3 r = quat_log(b.fem.q[b.fem.slot(n)]);
    return axis == 0 ? r.x : axis == 1 ? r.y : r.z;
}

double frame_energy(const SoftBody& b) {
    double E = 0;
    for (const Node& n : b.nodes) E += 0.5 * n.mass * length2(n.v);
    for (size_t i = 0; i < b.fem.node.size(); i++) E += 0.5 * b.fem.inertia[i] * length2(b.fem.w[i]);
    return E + b.fem.strain_energy(b);
}

} // namespace

// The membrane's damping (SoftBody::membrane_damp) takes the edges' rates of stretch down and keeps the momentum, the
// angular momentum and a rigid turn
static void test_membrane_damp() {
    printf("membrane damping\n");
    ShellMaterial m = mat_steel();
    m.membrane = 1.0e5f, m.yield = 0.0015f;
    auto rate = [](const SoftBody& b) {
        double s2 = 0;
        int n = 0;
        for (const Shell& sh : b.shells)
            for (int e = 0; e < 3; e++) {
                const Node& A = b.nodes[sh.n[e]];
                const Node& B = b.nodes[sh.n[(e + 1) % 3]];
                const vec3 u = normalize(B.p - A.p);
                const double r = dot(B.v - A.v, u);
                s2 += r * r, n++;
            }
        return std::sqrt(s2 / std::max(1, n));
    };
    auto momenta = [](const SoftBody& b, vec3& P, vec3& L) {
        P = L = vec3(0);
        for (const Node& x : b.nodes) P += x.v * x.mass, L += cross(x.p, x.v * x.mass);
    };
    for (int jit = 0; jit < 2; jit++) {
        auto s = make_sheet(vec3(0.2f, 1.0f, 0.1f), normalize(vec3(1, 0.1f, 0)), normalize(vec3(0, 0.2f, 1)), 1.2f, 1.0f, 9, 8, 20.0f, 0, m, 5);
        TRng r(9);
        const vec3 om(0.3f, 2.0f, -0.7f), c(0.2f, 1.0f, 0.1f);
        // (the damping takes the edges that left their band lately: stretched 1% and projected back once, then the sheet
        // as it was)
        std::vector<vec3> p0;
        for (Node& x : s->nodes) p0.push_back(x.p), x.p = c + (x.p - c) * 1.01f;
        s->allow_deform = false;
        s->membrane_damp = 0.3f;
        s->project_membrane(kDefaultDt);
        for (size_t i = 0; i < s->nodes.size(); i++) s->nodes[i].p = p0[i];
        for (Node& x : s->nodes) {
            x.v = vec3(0.5f, -0.2f, 0.1f) + cross(om, x.p - c);
            if (jit) x.v += vec3(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1)) * 0.3f;
        }
        std::vector<vec3> v0;
        for (const Node& x : s->nodes) v0.push_back(x.v);
        vec3 P0, L0, P1, L1;
        momenta(*s, P0, L0);
        const double r0 = rate(*s);
        s->membrane_damp = 0.3f;
        s->project_membrane(kDefaultDt);
        momenta(*s, P1, L1);
        const double r1 = rate(*s);
        float dv = 0;
        for (size_t i = 0; i < s->nodes.size(); i++) dv = std::max(dv, length(s->nodes[i].v - v0[i]));
        printf("    %s: stretch rate %.4f -> %.4f m/s rms, momentum off by %.2g, angular by %.2g, speeds changed by %.2g m/s at most\n",
               jit ? "a turning sheet shaken" : "a turning sheet", r0, r1, length(P1 - P0), length(L1 - L0), dv);
        CHECK(length(P1 - P0) < 1e-4f * (1 + length(P0)) && length(L1 - L0) < 1e-3f * (1 + length(L0)), "momentum %.3g, angular momentum %.3g changed", length(P1 - P0),
              length(L1 - L0));
        if (jit) CHECK(r1 < 0.85 * r0, "the stretch rate %.4f -> %.4f", r0, r1);
        else CHECK(dv < 1e-4f, "a rigid turn was damped (%.3g m/s)", dv);
    }
}

// ------------------------------------------------------------------------------------------------ triangle elements
// The shell of FEM triangles against the plate and beam results it has to reproduce: a cantilever strip's tip under a
// load across it and along it, a clamped square plate under pressure (Timoshenko's table), a free plate spun about
// three axes keeps its shape and energy, a rigid turn of an element makes no force; past its yield a strip keeps a
// set and holds about its plastic moment, pulled far it tears; a hollow steel cube dropped on the ground stays sound.
namespace {

// a strip or plate of nu x nv cells in the x-z plane at height y, the corners' nodes on the x = 0 edge clamped
std::unique_ptr<SoftBody> tri_plate(float L, float W, int nu, int nv, const ShellSection& s, bool clamp_root, float y = 1.0f) {
    auto b = std::make_unique<SoftBody>();
    b->name = "tri plate";
    b->can_sleep = false;
    const uint16_t si = b->fem.add_shell_section(s);
    ShellMesher m(*b, NF_NONE);
    m.grid(vec3(0, y, 0), vec3(L / nu, 0, 0), vec3(0, 0, W / nv), nu, nv, si, false);
    if (clamp_root)
        for (size_t i = 0; i < b->nodes.size(); i++)
            if (b->nodes[i].p.x < 1e-4f) b->info[i].flags |= NF_FIXED;
    m.finish();
    b->fem.finalize(*b);
    return b;
}

} // namespace

static void test_fem_tris() {
    printf("triangle elements (FEM shells)\n");
    // (the shell's authored area: its triangles' and its loose fragments' - refined or torn, none of it lost)
    // (a node still on the part held at `root` - a loaded node torn off with a small piece would be flung by the load)
    auto held_on = [](const SoftBody& x, uint32_t v, uint32_t root) {
        return x.fem.slot(v) >= 0 && x.fem.slot(root) >= 0 && x.fem.component_of(v) == x.fem.component_of(root);
    };
    auto shell_area = [](const FemFrame& f) {
        double a = 0;
        for (const FrameTri& t : f.tris) a += t.broken ? 0.0 : (double)t.area0;
        for (const FemFrame::LooseTri& t : f.loose_tris) a += t.area0;
        return a;
    };
    auto world = [](bool settle) {
        auto w = std::make_unique<World>();
        w->settings.gravity = vec3(0);
        if (settle) w->settings.frame_theta = 1.0f, w->settings.frame_dissipation = 1.0f;
        return w;
    };
    ShellSection steel = make_shell_section("Steel", 0.005f);
    ShellSection elastic = steel;
    elastic.yield = 0;
    printf("    steel 5 mm: %.1f kg/m2, D %.0f N m, nu %.3f\n", steel.mass_per_m2(), steel.D(), steel.nu);
    // one element turned and moved as a whole (its nodes' orientations with it): no force, no moment
    {
        auto b = tri_plate(0.5f, 0.3f, 1, 1, elastic, false);
        const quat r = normalize(quat::axis_angle(normalize(vec3(0.3f, 0.8f, -0.5f)), 1.1f));
        for (Node& n : b->nodes) n.p = r.rotate(n.p - vec3(0.2f, 1, 0.1f)) + vec3(3, 2, -1);
        b->fem.rotate(r);
        b->fem.compute_forces(*b);
        float fmax = 0, tmax = 0;
        for (size_t i = 0; i < b->nodes.size(); i++) fmax = std::max(fmax, length(b->force[i]));
        for (const vec3& t : b->fem.torque) tmax = std::max(tmax, length(t));
        // (against what a micrometre's stretch or a microradian's bend makes: the float positions' rounding is about that)
        // (the moments through the drilling spring, G t A / 20 per corner: its microradian's)
        const float fu = elastic.E * elastic.t * 1e-6f, tu = elastic.drill * elastic.E / (2 * (1 + elastic.nu)) * elastic.t * 0.075f * 1e-6f;
        printf("    a rigid turn of 63 degrees: largest force %.2e N (a micrometre's %.2e), moment %.2e N m (a microradian's %.2e)\n", fmax, fu, tmax, tu);
        CHECK(fmax < fu && tmax < tu, "rigid motion made forces: %g N, %g N m", fmax, tmax);
    }
    // a cantilever strip, 1 m x 0.2 m of 5 mm steel in 10 x 2 cells: the tip under a load across (beam theory, E I
    // between the beam's and the plate's E / (1 - nu^2)), then along it (E t b)
    {
        const float L = 1.0f, W = 0.2f, P = 20.0f;
        const double EI = (double)elastic.E * W * elastic.t * elastic.t * elastic.t / 12.0;
        for (int load = 0; load < 2; load++) {
            auto w = world(true);
            SoftBody* b = w->add_body(tri_plate(L, W, 10, 2, elastic, true));
            std::vector<uint32_t> tip;
            for (uint32_t i = 0; i < b->nodes.size(); i++)
                if (b->nodes[i].p.x > L - 1e-4f) tip.push_back(i);
            vec3 p0(0);
            for (uint32_t i : tip) p0 += b->nodes[i].p / (float)tip.size();
            const vec3 F = load == 0 ? vec3(0, -P, 0) : vec3(2e5f, 0, 0);
            b->pre_substep = [&](SoftBody& x, float) {
                for (uint32_t i : tip) x.force[i] += F * ((x.nodes[i].p.z < 1e-4f || x.nodes[i].p.z > W - 1e-4f ? 0.5f : 1.0f) / (float)(tip.size() - 1));
                for (size_t i = 0; i < x.nodes.size(); i++) x.force[i] -= x.nodes[i].v * (x.nodes[i].mass * 40.0f); // (settles: its 4 Hz mode rang)
            };
            for (int i = 0; i < 4000; i++) w->step_substeps(1);
            vec3 p1(0);
            for (uint32_t i : tip) p1 += b->nodes[i].p / (float)tip.size();
            if (load == 0) {
                const double want = P * L * L * L / (3 * EI), got = p0.y - p1.y, plate = want * (1 - elastic.nu * elastic.nu);
                printf("    cantilever strip, across: tip %.3f mm (beam %.3f, plate %.3f)\n", got * 1e3, want * 1e3, plate * 1e3);
                CHECK(got > plate * 0.97 && got < want * 1.03, "strip tip %.5g, beam %.5g, plate %.5g", got, want, plate);
            } else {
                const double want = F.x * L / ((double)elastic.E * elastic.t * W), got = p1.x - p0.x;
                printf("    cantilever strip, along: stretch %.4f mm (theory %.4f)\n", got * 1e3, want * 1e3);
                CHECK(std::fabs(got / want - 1) < 0.03, "strip stretch %.5g vs %.5g", got, want);
            }
            CHECK(b->fem.solve_failures == 0, "strip: %d failed solves", b->fem.solve_failures);
        }
    }
    // a clamped square plate, 1 m of 10 mm steel in 10 x 10 cells, 1 kPa: the middle w = 0.00126 q a^4 / D
    {
        ShellSection s10 = make_shell_section("Steel", 0.01f);
        s10.yield = 0;
        auto w = world(true);
        auto body = tri_plate(1.0f, 1.0f, 10, 10, s10, false);
        for (size_t i = 0; i < body->nodes.size(); i++) {
            const vec3 p = body->nodes[i].p;
            if (p.x < 1e-4f || p.x > 1 - 1e-4f || p.z < 1e-4f || p.z > 1 - 1e-4f) body->info[i].flags |= NF_FIXED, body->nodes[i].inv_mass = 0;
        }
        SoftBody* b = w->add_body(std::move(body));
        std::vector<vec3> load(b->nodes.size(), vec3(0));
        const float q = 1000.0f;
        for (const FrameTri& t : b->fem.tris)
            for (uint32_t v : t.n) load[b->fem.node[v]] += vec3(0, -q * t.area0 / 3.0f, 0);
        uint32_t mid = 0;
        for (uint32_t i = 0; i < b->nodes.size(); i++)
            if (length(b->nodes[i].p - vec3(0.5f, 1, 0.5f)) < 1e-3f) mid = i;
        const float y0 = b->nodes[mid].p.y;
        b->pre_substep = [&](SoftBody& x, float) {
            for (size_t i = 0; i < x.nodes.size(); i++) x.force[i] += load[i];
        };
        for (int i = 0; i < 4000; i++) w->step_substeps(1);
        const double want = 0.00126 * q / s10.D(), got = y0 - b->nodes[mid].p.y;
        printf("    clamped square plate: middle %.4f mm (Timoshenko %.4f)\n", got * 1e3, want * 1e3);
        CHECK(std::fabs(got / want - 1) < 0.05, "clamped plate %.5g vs %.5g", got, want);
    }
    // a free plate spun about three axes: after a second it keeps its shape (little strain energy) and its energy; the
    // frame's step at 2 kHz and at 1 kHz (the linearization over twice the step strains it a little more)
    for (int fe : {1, 2}) {
        ShellSection s2 = make_shell_section("Steel", 0.002f);
        s2.yield = 0;
        auto w = world(false);
        w->settings.frame_every = fe;
        auto body = tri_plate(1.0f, 1.0f, 6, 6, s2, false);
        const vec3 om(1.5f, 5.0f, 1.0f), c(0.5f, 1, 0.5f);
        for (Node& x : body->nodes) x.v = cross(om, x.p - c);
        for (vec3& x : body->fem.w) x = om;
        SoftBody* b = w->add_body(std::move(body));
        auto kinetic = [&]() {
            double ke = 0;
            for (const Node& x : b->nodes) ke += 0.5 * x.mass * length2(x.v);
            for (size_t i = 0; i < b->fem.node.size(); i++) ke += 0.5 * b->fem.inertia[i] * length2(b->fem.w[i]);
            return ke;
        };
        const double ke0 = kinetic();
        double umax = 0;
        for (int i = 0; i < 2000; i++) {
            w->step_substeps(1);
            umax = std::max(umax, b->fem.tri_energy());
        }
        const double ke1 = kinetic();
        printf("    free plate spun (its step at %d Hz): energy %.3f -> %.3f J, strain energy at most %.2e J\n", 2000 / fe, ke0, ke1, umax);
        CHECK(std::fabs(ke1 / ke0 - 1) < 0.02 && umax < (fe == 1 ? 1e-3 : 2e-2) * ke0, "spun plate (%d): energy %.4g -> %.4g, strain %.3g", fe, ke0, ke1, umax);
        CHECK(b->fem.solve_failures == 0 && b->fem.clamps == 0, "spun plate: %d failures, %d clamps", b->fem.solve_failures, b->fem.clamps);
    }
    // plastic: a strip 0.5 m x 0.2 m of 10 mm steel. Loaded to half its first yield it springs back; to 0.9 of its
    // plastic collapse load (the root's plastic moment sigma_y t^2 b / 4) it keeps a set when unloaded
    {
        ShellSection s10 = make_shell_section("Steel", 0.01f);
        const float L = 0.5f, W = 0.2f;
        const double Pe = s10.yield * s10.t * s10.t * W / 6.0 / L, Pp = s10.yield * s10.t * s10.t * W / 4.0 / L;
        for (int k = 0; k < 2; k++) {
            auto w = world(true);
            SoftBody* b = w->add_body(tri_plate(L, W, 10, 4, s10, true));
            std::vector<uint32_t> tip;
            for (uint32_t i = 0; i < b->nodes.size(); i++)
                if (b->nodes[i].p.x > L - 1e-4f) tip.push_back(i);
            const float P = (float)(k == 0 ? 0.5 * Pe : 0.9 * Pp);
            bool on = true;
            b->pre_substep = [&](SoftBody& x, float) {
                if (on)
                    for (uint32_t i : tip) x.force[i] += vec3(0, -P / (float)tip.size(), 0);
                for (size_t i = 0; i < x.nodes.size(); i++) x.force[i] -= x.nodes[i].v * (x.nodes[i].mass * 40.0f);
            };
            const float y0 = b->nodes[tip[tip.size() / 2]].p.y;
            for (int i = 0; i < 4000; i++) w->step_substeps(1);
            const float loaded = y0 - b->nodes[tip[tip.size() / 2]].p.y;
            on = false;
            for (int i = 0; i < 4000; i++) w->step_substeps(1);
            const float set = y0 - b->nodes[tip[tip.size() / 2]].p.y;
            printf("    plastic strip at %s: tip %.2f mm loaded, %.2f mm set\n", k == 0 ? "half its first yield" : "0.9 of its collapse load", loaded * 1e3, set * 1e3);
            if (k == 0) CHECK(set < 0.02f * loaded, "elastic strip kept a set: %.4g of %.4g", set, loaded);
            else CHECK(set > 0.2f * loaded && b->fem.tris_torn == 0, "plastic strip: set %.4g of %.4g, %d torn", set, loaded, b->fem.tris_torn);
        }
        // beyond the collapse load it folds at the root (keeps going)
        auto w = world(true);
        SoftBody* b = w->add_body(tri_plate(L, W, 10, 4, s10, true));
        std::vector<uint32_t> tip;
        for (uint32_t i = 0; i < b->nodes.size(); i++)
            if (b->nodes[i].p.x > L - 1e-4f) tip.push_back(i);
        b->pre_substep = [&](SoftBody& x, float) {
            for (uint32_t i : tip) x.force[i] += vec3(0, (float)(-1.3 * Pp) / (float)tip.size(), 0);
        };
        const float y0 = b->nodes[tip[0]].p.y;
        for (int i = 0; i < 2000; i++) w->step_substeps(1);
        const float d = y0 - b->nodes[tip[0]].p.y;
        printf("    plastic strip at 1.3 its collapse load: tip %.0f mm down after a second\n", d * 1e3);
        CHECK(d > 0.1f && b->fem.solve_failures == 0, "collapsing strip: %.4g m, %d failures", d, b->fem.solve_failures);
    }
    // pulled far past its yield a strip stretches and tears
    {
        ShellSection s2 = make_shell_section("Steel", 0.002f);
        auto w = world(false);
        SoftBody* b = w->add_body(tri_plate(0.5f, 0.1f, 10, 2, s2, true));
        std::vector<uint32_t> tip;
        for (uint32_t i = 0; i < b->nodes.size(); i++)
            if (b->nodes[i].p.x > 0.5f - 1e-4f) tip.push_back(i);
        const float P = 2.0f * s2.yield * s2.t * 0.1f;
        b->pre_substep = [&](SoftBody& x, float) {
            for (uint32_t i : tip) x.force[i] += vec3(P / (float)tip.size(), 0, 0);
        };
        const size_t nodes0 = b->nodes.size(), tris0 = b->fem.tris.size();
        const double area0 = shell_area(b->fem);
        bool pulling = true;
        uint32_t root = 0;
        for (uint32_t i = 0; i < b->nodes.size(); i++)
            if (b->info[i].flags & NF_FIXED) root = i;
        b->pre_substep = [&](SoftBody& x, float) { // (the tip still on the clamped strip: a piece torn off is let go)
            if (pulling)
                for (uint32_t i : tip)
                    if (held_on(x, i, root)) x.force[i] += vec3(P / (float)tip.size(), 0, 0);
        };
        for (int i = 0; i < 4000 && b->fem.components() < 2 && b->fem.loose_count == 0; i++) w->step_substeps(1);
        pulling = false; // (the load off once it is in two: the piece it tore off flies)
        for (int i = 0; i < 400; i++) w->step_substeps(1);
        bool finite = true;
        for (const Node& x : b->nodes) finite &= std::isfinite(x.p.x + x.p.y + x.p.z);
        int gone = 0;
        for (const FrameTri& t : b->fem.tris) gone += t.broken;
        // (torn, a triangle stays: the crack opens along its edges, the nodes there duplicated - none disappears; a small
        // piece torn off loose, FemFrame::loose_tris, out of the frame but kept)
        const double kept = shell_area(b->fem) / area0;
        printf("    strip pulled at twice its yield: %d tears, %d bisections, %zu -> %zu nodes, %zu -> %zu triangles (%d gone), area kept %.6f, %d components, "
               "%d fragments loose\n", b->fem.tris_torn, b->fem.tris_refined, nodes0, b->nodes.size(), tris0, b->fem.tris.size(), gone, kept, b->fem.components(),
               b->fem.loose_count);
        // (a loose fragment keeps its shape: its springs near their lengths at the tear)
        float worst = 0;
        for (const Beam& bm : b->beams)
            if (bm.flags & BF_NO_DEFORM) worst = std::max(worst, std::fabs(distance(b->nodes[bm.a].p, b->nodes[bm.b].p) - bm.L) / std::max(bm.L, 1e-6f));
        printf("    its loose fragments' shape: springs within %.2f%% of their lengths\n", worst * 100);
        CHECK(worst < 0.2f, "loose fragment stretched %.1f%%", worst * 100); // (a shard cut off far past its tear flies off spinning)
        CHECK(b->fem.vertex_hinges() == 0, "pulled strip: %d triangles' groups left on one corner", b->fem.vertex_hinges());
        CHECK(b->fem.tris_torn > 0 && finite && gone == 0 && b->nodes.size() > nodes0 && std::fabs(kept - 1) < 1e-5 && (b->fem.components() >= 2 || b->fem.loose_count > 0),
              "pulled strip: %d torn, finite %d, %d gone, %zu nodes, %d components, %d loose", b->fem.tris_torn, (int)finite, gone, b->nodes.size(), b->fem.components(),
              b->fem.loose_count);
    }
    // a plate cut across by the laser: it parts along its edges into two pieces, no triangle removed, the nodes along the
    // cut duplicated
    {
        auto w = world(false);
        SoftBody* b = w->add_body(tri_plate(0.6f, 0.4f, 6, 4, make_shell_section("Steel", 0.002f), false));
        const size_t nodes0 = b->nodes.size();
        w->step_substeps(1);
        const int cut = w->laser_cut(vec3(0.31f, 2.0f, 0.2f), normalize(vec3(0, -1, 0.3f)), normalize(vec3(0, -1, -0.3f)), 5.0f);
        b->fem.finish_cuts(*b, cut);
        for (int i = 0; i < 200; i++) w->step_substeps(1);
        int gone = 0;
        for (const FrameTri& t : b->fem.tris) gone += t.broken;
        bool finite = true;
        for (const Node& x : b->nodes) finite &= std::isfinite(x.p.x + x.p.y + x.p.z);
        printf("    plate cut across by the laser: %d tears, %zu -> %zu nodes, %d triangles gone, %d components\n", cut, nodes0, b->nodes.size(), gone,
               b->fem.components());
        CHECK(cut > 0 && gone == 0 && b->nodes.size() > nodes0 && b->fem.components() == 2 && finite, "laser cut: %d tears, %d gone, %d components", cut, gone,
              b->fem.components());
        CHECK(b->fem.vertex_hinges() == 0, "laser cut: %d nodes with triangles on a corner alone", b->fem.vertex_hinges());
    }
    // a plate held at its root torn at a corner, pulled far past its yield: the crack runs in along the triangles' edges,
    // each torn edge parting whole (both its nodes duplicated); no triangle is left on the rest by one corner, none is
    // removed, the pieces torn off loose or whole
    {
        auto w = world(false);
        ShellSection s2 = make_shell_section("Steel", 0.002f);
        SoftBody* b = w->add_body(tri_plate(0.4f, 0.4f, 8, 8, s2, true));
        uint32_t corner = 0;
        for (uint32_t i = 0; i < b->nodes.size(); i++)
            if (b->nodes[i].p.x + b->nodes[i].p.z > b->nodes[corner].p.x + b->nodes[corner].p.z) corner = i;
        const size_t tris0 = b->fem.tris.size(), nodes0 = b->nodes.size();
        const double area0 = shell_area(b->fem);
        const float P = 1.5f * s2.yield * s2.t * 0.05f;
        bool pulling = true;
        const vec3 p0 = b->nodes[corner].p;
        b->pre_substep = [&](SoftBody& x, float) { // (until it tears off: a piece pulled on flies off for good)
            if (pulling && corner < x.force.size() && x.fem.slot(corner) >= 0 && distance(x.nodes[corner].p, p0) < 0.25f) x.force[corner] += vec3(P, 0.3f * P, P);
        };
        for (int i = 0; i < 4000 && b->fem.tris_torn < 6; i++) w->step_substeps(1);
        pulling = false;
        for (int i = 0; i < 400; i++) w->step_substeps(1);
        bool finite = true;
        for (const Node& x : b->nodes) finite &= std::isfinite(x.p.x + x.p.y + x.p.z);
        int gone = 0;
        for (const FrameTri& t : b->fem.tris) gone += t.broken;
        const double kept = shell_area(b->fem) / area0;
        printf("    plate torn at a corner: %d tears, %d bisections, %zu -> %zu nodes, %zu -> %zu triangles (%zu loose), area kept %.6f, %d left on one corner\n",
               b->fem.tris_torn, b->fem.tris_refined, nodes0, b->nodes.size(), tris0, b->fem.tris.size() - gone, b->fem.loose_tris.size(), kept, b->fem.vertex_hinges());
        CHECK(b->fem.tris_torn > 0 && finite && std::fabs(kept - 1) < 1e-5 && b->fem.vertex_hinges() == 0, "corner tear: %d tears, finite %d, area kept %.6f, %d hinges",
              b->fem.tris_torn, (int)finite, kept, b->fem.vertex_hinges());
    }
    // the same plate under a sheet laid over its elements (a car's skin over its body-in-white, Vehicle::make_sheet_body):
    // the sheet parts with them (SoftBody::sheet_follow) - each shell stays on its element's corners, none is linked
    // across a torn edge or drawn out over the crack -, it does not crack on its own; the mass is kept
    {
        auto w = world(false);
        ShellSection s2 = make_shell_section("Steel", 0.002f);
        auto pb = tri_plate(0.4f, 0.4f, 8, 8, s2, true);
        auto uv = [&](uint32_t n) { return vec2(pb->nodes[n].p.x, pb->nodes[n].p.z); };
        for (const FrameTri& t : pb->fem.tris) {
            const uint32_t a = pb->fem.node[t.n[0]], c = pb->fem.node[t.n[1]], d = pb->fem.node[t.n[2]];
            pb->add_shell(a, c, d, uv(a), uv(c), uv(d));
        }
        pb->shell_mat = mat_steel();
        pb->finalize_shells(4.7f, kDefaultDt, 7, true);
        pb->fem.bind_sheet(*pb);
        int hosted0 = 0;
        for (const Shell& s : pb->shells) hosted0 += pb->on_plate(s);
        const size_t tris0 = pb->fem.tris.size();
        SoftBody* b = w->add_body(std::move(pb));
        uint32_t corner = 0;
        for (uint32_t i = 0; i < b->nodes.size(); i++)
            if (b->nodes[i].p.x + b->nodes[i].p.z > b->nodes[corner].p.x + b->nodes[corner].p.z) corner = i;
        double mass0 = 0;
        for (const Node& x : b->nodes) mass0 += x.mass;
        const float P = 1.5f * s2.yield * s2.t * 0.05f;
        bool pulling = true;
        const vec3 p0 = b->nodes[corner].p;
        b->pre_substep = [&](SoftBody& x, float) { // (until it tears off: a piece pulled on flies off for good)
            if (pulling && corner < x.force.size() && x.fem.slot(corner) >= 0 && distance(x.nodes[corner].p, p0) < 0.25f) x.force[corner] += vec3(P, 0.3f * P, P);
        };
        for (int i = 0; i < 4000 && b->fem.tris_torn < 6; i++) w->step_substeps(1);
        pulling = false;
        for (int i = 0; i < 400; i++) w->step_substeps(1);
        bool finite = true;
        double mass = 0;
        for (const Node& x : b->nodes) finite &= std::isfinite(x.p.x + x.p.y + x.p.z), mass += x.mass;
        const FemFrame& f = b->fem;
        int off = 0, across = 0, hosted = 0;
        float worst = 0;
        for (const Shell& s : b->shells) {
            for (int e = 0; e < 3; e++) worst = std::max(worst, distance(b->nodes[s.n[e]].p, b->nodes[s.n[(e + 1) % 3]].p) / s.L0[e]);
            if (!b->on_plate(s)) continue;
            hosted++;
            const FrameTri& h = f.tris[s.host];
            for (uint32_t n : s.n)
                off += f.slot(n) >= 0 && f.node[h.n[0]] != n && f.node[h.n[1]] != n && f.node[h.n[2]] != n;
            for (int e = 0; e < 3; e++) {
                const int j = s.nb[e];
                if (j < 0 || !b->on_plate(b->shells[j]) || b->shells[j].host == s.host) continue;
                int same = 0;
                for (uint32_t x : h.n)
                    for (uint32_t y : f.tris[b->shells[j].host].n) same += f.node[x] == f.node[y];
                across += same < 2;
            }
        }
        printf("    ... under a sheet (%d shells, one on each): %d tears, %d of the %zu shells now on their elements, %d corners off them, %d links across a tear, "
               "longest edge x%.2f, mass %.4f -> %.4f kg\n", hosted0, f.tris_torn, hosted, b->shells.size(), off, across, worst, mass0, mass);
        CHECK(hosted0 == (int)tris0 && f.tris_torn > 0 && finite && off == 0 && across == 0 && worst < 2.5f && f.vertex_hinges() == 0 &&
                  std::fabs(mass - mass0) < 1e-3 * mass0,
              "corner tear under a sheet: %d of %zu bound, %d tears, finite %d, %d off, %d across, x%.2f, %d hinges, mass %.4f -> %.4f", hosted0, tris0,
              f.tris_torn, (int)finite, off, across, worst, f.vertex_hinges(), mass0, mass);
    }
    // a plate yielding at a corner is bisected there before it tears (ShellSection::max_level, refine_at): the shell
    // stays conforming - no edge on more than two triangles, its border as long and its area as large as before (a
    // neighbour left whole beside a halved edge would leave a slit) -, the mass kept, nothing torn
    {
        auto w = world(false);
        ShellSection s3 = make_shell_section("Steel", 0.002f);
        s3.elongation = 10.0f, s3.refine_at = 0.005f, s3.pattern = ShellPattern::None, s3.min_edge = 0.01f; // (bisected from a plastic stretch of 5%)
        SoftBody* b = w->add_body(tri_plate(0.4f, 0.4f, 8, 8, s3, true));
        const FemFrame& f = b->fem;
        uint32_t corner = 0;
        for (uint32_t i = 0; i < b->nodes.size(); i++)
            if (b->nodes[i].p.x + b->nodes[i].p.z > b->nodes[corner].p.x + b->nodes[corner].p.z) corner = i;
        auto border = [&]() { // (the edges on one triangle: their authored lengths)
            std::map<std::pair<uint32_t, uint32_t>, std::pair<int, float>> edges;
            for (const FrameTri& t : f.tris) {
                if (t.broken) continue;
                for (int e = 0; e < 3; e++) {
                    const uint32_t x = f.node[t.n[e]], y = f.node[t.n[(e + 1) % 3]];
                    auto& v = edges[{std::min(x, y), std::max(x, y)}];
                    v.first++;
                    v.second = std::hypot(t.X0[(e + 1) % 3][0] - t.X0[e][0], t.X0[(e + 1) % 3][1] - t.X0[e][1]);
                }
            }
            double len = 0;
            int most = 0;
            for (const auto& [k, v] : edges) {
                most = std::max(most, v.first);
                if (v.first == 1) len += v.second;
            }
            return std::make_pair(len, most);
        };
        double mass0 = 0;
        for (const Node& x : b->nodes) mass0 += x.mass;
        const double area0 = shell_area(f), border0 = border().first;
        const size_t tris0 = f.tris.size(), nodes0 = b->nodes.size();
        const float P = 1.5f * s3.yield * s3.t * 0.05f;
        const vec3 p0 = b->nodes[corner].p;
        b->pre_substep = [&](SoftBody& x, float) {
            if (corner < x.force.size() && distance(x.nodes[corner].p, p0) < 0.08f) x.force[corner] += vec3(P, 0.3f * P, P);
        };
        for (int i = 0; i < 3000; i++) w->step_substeps(1);
        bool finite = true;
        double mass = 0;
        for (const Node& x : b->nodes) finite &= std::isfinite(x.p.x + x.p.y + x.p.z), mass += x.mass;
        int top = 0, in_frame = 0;
        for (const FrameTri& t : f.tris) top = std::max(top, (int)t.level);
        for (uint32_t i = (uint32_t)nodes0; i < b->nodes.size(); i++) in_frame += f.slot(i) >= 0;
        const auto [border1, most] = border();
        printf("    plate refined where it yields: %d bisections, %zu -> %zu triangles (levels to %d), %zu -> %zu nodes (%d in the frame), the border %.4f -> %.4f m, "
               "at most %d triangles on an edge, area %.6f -> %.6f m2, mass %.4f -> %.4f kg, %d torn\n", f.tris_refined, tris0, f.tris.size(), top, nodes0,
               b->nodes.size(), in_frame, border0, border1, most, area0, shell_area(f), mass0, mass, f.tris_torn);
        CHECK(f.tris_refined > 0 && top == s3.max_level && in_frame == (int)(b->nodes.size() - nodes0) && std::fabs(border1 - border0) < 1e-4 && most <= 2 &&
                  std::fabs(shell_area(f) - area0) < 1e-6 * area0 && std::fabs(mass - mass0) < 1e-4 * mass0 && f.tris_torn == 0 && finite,
              "refined plate: %d bisections, level %d, border %.4f -> %.4f, %d on an edge, mass %.4f -> %.4f, %d torn, finite %d", f.tris_refined, top, border0,
              border1, most, mass0, mass, f.tris_torn, (int)finite);
    }
    // a 0.8 mm steel plate held round its border, a punch pattern laid in its middle (a ring and radial tears round it,
    // FemFrame::impacts: its lines resolved by bisections, their edges weaker) and the middle pressed through: a
    // triangle with an edge near a line tears along it (the strain decides where the plate tears - here inside the
    // ring, at the pressed middle's rim -, the pattern which way); nothing of it lost. And a fast ball on it lays one
    // (the contacts' FemFrame::note_hit)
    {
        struct Punch {
            size_t impacts = 0;
            int coded = 0, lines = 0, refined = 0, torn = 0;
            std::vector<vec3> edges; // (the middles of the edges its tears left free inside it)
            int by_line = 0, on_line = 0; // (tears of triangles with an edge near a line, those that parted it)
            FemFrame::Impact im;
            bool finite = true;
            double kept = 0;
        };
        auto plate = [&](bool patterned) {
            ShellSection s4 = make_shell_section("Steel", 0.0008f);
            s4.min_edge = 0.02f; // (its 5 cm cells halved once)
            if (!patterned) s4.pattern = ShellPattern::None;
            auto pb = std::make_unique<SoftBody>(); // (its nodes contacters, its triangles' collision triangles)
            pb->name = "punched plate";
            pb->can_sleep = false;
            const uint16_t sec = pb->fem.add_shell_section(s4);
            ShellMesher m(*pb);
            m.grid(vec3(0, 1, 0), vec3(0.05f, 0, 0), vec3(0, 0, 0.05f), 12, 12, sec, true);
            for (uint32_t i = 0; i < pb->nodes.size(); i++) {
                const vec3 p = pb->nodes[i].p;
                if (p.x < 1e-4f || p.z < 1e-4f || p.x > 0.6f - 1e-4f || p.z > 0.6f - 1e-4f) pb->info[i].flags |= NF_FIXED;
            }
            m.finish();
            pb->fem.finalize(*pb);
            return pb;
        };
        const vec3 mid(0.31f, 1, 0.29f), offset(0.03f, 0, 0); // (the pattern a little off the pressed middle)
        auto punch = [&](bool patterned) {
            Punch r;
            auto w = world(false);
            SoftBody* b = w->add_body(plate(patterned));
            FemFrame& f = b->fem;
            const double area0 = shell_area(f);
            if (patterned) {
                // (the pattern of a 10 cm body's blow at 20 m/s, its lines resolved as the world's contacts do it)
                uint32_t at = 0;
                for (uint32_t u = 0; u < f.tris.size(); u++) {
                    const vec3 c = (b->nodes[f.node[f.tris[u].n[0]]].p + b->nodes[f.node[f.tris[u].n[1]]].p + b->nodes[f.node[f.tris[u].n[2]]].p) / 3.0f;
                    const vec3 c0 = (b->nodes[f.node[f.tris[at].n[0]]].p + b->nodes[f.node[f.tris[at].n[1]]].p + b->nodes[f.node[f.tris[at].n[2]]].p) / 3.0f;
                    if (distance(c, mid + offset) < distance(c0, mid + offset)) at = u;
                }
                f.note_hit(f.node[f.tris[at].n[0]], f.node[f.tris[at].n[1]], f.node[f.tris[at].n[2]], vec3(1.0f / 3.0f), 20.0f, 0.1f, 0.0);
            }
            // (the middle pressed down by a force growing 60 kN a second, off once it is through)
            std::vector<uint32_t> press;
            for (uint32_t i = 0; i < b->nodes.size(); i++)
                if (std::hypot(b->nodes[i].p.x - mid.x, b->nodes[i].p.z - mid.z) < 0.075f) press.push_back(i);
            double t = 0;
            bool through = false;
            b->pre_substep = [&](SoftBody& x, float dt) { // (on the nodes still on the held plate: a torn-off one would be flung)
                t += dt;
                if (!through)
                    for (uint32_t i : press)
                        if (held_on(x, i, 0)) x.force[i] += vec3(0, -60000.0f * (float)t / (float)press.size(), 0);
            };
            for (int i = 0; i < 6000 && !through; i++) {
                w->step_substeps(1);
                through = b->nodes[press[0]].p.y < 0.8f || length(b->nodes[press[0]].v) > 15.0f; // (the plug off: it would be flung)
            }
            for (int i = 0; i < 200; i++) w->step_substeps(1);
            std::map<std::pair<uint32_t, uint32_t>, int> edges;
            for (const FrameTri& tr : f.tris) {
                if (tr.broken) continue;
                r.coded += tr.imp >= 0, r.lines += (tr.line & 7u) != 0;
                for (int e = 0; e < 3; e++) {
                    const uint32_t x = f.node[tr.n[e]], y = f.node[tr.n[(e + 1) % 3]];
                    edges[{std::min(x, y), std::max(x, y)}]++;
                }
            }
            for (const auto& [k, n] : edges) {
                if (n != 1) continue;
                const vec3 m = (b->nodes[k.first].p + b->nodes[k.second].p) * 0.5f;
                if (std::min(std::min(m.x, m.z), std::min(0.6f - m.x, 0.6f - m.z)) > 0.01f) r.edges.push_back(m); // (off the border)
            }
            r.by_line = f.tears_by_line, r.on_line = f.tears_on_line;
            for (const Node& x : b->nodes) r.finite &= std::isfinite(x.p.x + x.p.y + x.p.z);
            r.impacts = f.impacts.size();
            if (!f.impacts.empty()) r.im = f.impacts[0];
            r.refined = f.tris_refined, r.torn = f.tris_torn;
            r.kept = shell_area(f) / area0;
            return r;
        };
        const Punch on = punch(true), off = punch(false);
        // (its tears along the lines: a triangle with an edge near one parts that edge)
        printf("    0.8 mm of steel pressed through a punch pattern (ring %.0f mm): %d triangles in it, %d on its lines, %d bisections, %d tears, %d of the %d "
               "tears of triangles on its lines along them (without the pattern: %d tears, %d bisections); area kept %.6f\n",
               on.impacts ? on.im.pat.r * 1000.0f : 0.0f, on.coded, on.lines, on.refined, on.torn, on.on_line, on.by_line, off.torn, off.refined, on.kept);
        CHECK(on.impacts == 1 && on.im.pat.kind == (uint8_t)ShellPattern::Punch && on.coded > 0 && on.lines > 0 && on.refined > off.refined && on.torn > 0 &&
                  off.torn > 0 && off.impacts == 0 && on.by_line > 0 && on.on_line * 10 >= on.by_line * 7 && off.by_line == 0 && on.finite && off.finite &&
                  std::fabs(on.kept - 1) < 1e-5,
              "punched plate: %zu patterns, %d coded, %d on lines, %d bisections (%d without), %d tears, %d of %d along the lines; finite %d %d", on.impacts, on.coded,
              on.lines, on.refined, off.refined, on.torn, on.on_line, on.by_line, (int)on.finite, (int)off.finite);
        // (a fast ball's contact lays one: the world's contacts call note_hit)
        {
            auto w = world(false);
            SoftBody* b = w->add_body(plate(true));
            w->add_body(make_ball(mid + vec3(0, 0.16f, 0), 0.1f, 2.0f, vec3(0, -12.0f, 0)));
            for (int i = 0; i < 100 && b->fem.impacts.empty(); i++) w->step_substeps(1);
            printf("    a ball at 12 m/s on it: %zu pattern laid (ring %.0f mm)\n", b->fem.impacts.size(), b->fem.impacts.empty() ? 0.0f : b->fem.impacts[0].pat.r * 1000.0f);
            CHECK(b->fem.impacts.size() == 1, "ball on the plate: %zu patterns", b->fem.impacts.size());
        }
    }
    // a hollow steel cube (1 m, 2 mm, 5 x 5 cells a face) dropped 1 m on its face: it lands and rests, sound
    {
        World w;
        w.statics.terrain.create(21, 21, 1.0f, vec2(-10, -10));
        w.statics.has_terrain = true;
        w.statics.terrain.update_bounds();
        auto body = std::make_unique<SoftBody>();
        body->name = "tri cube";
        const uint16_t si = body->fem.add_shell_section(make_shell_section("Steel", 0.002f));
        ShellMesher m(*body);
        const float a = 1.0f, y0 = 1.0f;
        const vec3 o(-0.5f, y0, -0.5f);
        const vec3 X(a / 5, 0, 0), Y(0, a / 5, 0), Z(0, 0, a / 5);
        m.grid(o, Z, X, 5, 5, si);                          // bottom (normals out: down)
        m.grid(o + vec3(0, a, 0), X, Z, 5, 5, si);         // top
        m.grid(o, X, Y, 5, 5, si);                          // front z = 0
        m.grid(o + vec3(0, 0, a), Y, X, 5, 5, si);         // back
        m.grid(o, Y, Z, 5, 5, si);                          // left
        m.grid(o + vec3(a, 0, 0), Z, Y, 5, 5, si);         // right
        m.finish();
        body->fem.finalize(*body);
        SoftBody* b = w.add_body(std::move(body));
        float mass = 0;
        for (const Node& x : b->nodes) mass += x.mass;
        double ke_max = 0;
        for (int i = 0; i < 8000; i++) {
            w.step_substeps(1);
            double ke = 0;
            for (const Node& x : b->nodes) ke += 0.5 * x.mass * length2(x.v);
            ke_max = std::max(ke_max, ke);
        }
        double ke = 0;
        for (const Node& x : b->nodes) ke += 0.5 * x.mass * length2(x.v);
        const vec3 c = b->center_of_mass();
        float vmax = 0;
        for (const Node& x : b->nodes) vmax = std::max(vmax, length(x.v));
        printf("    hollow cube dropped 1 m: %zu nodes, %zu triangles, %.1f kg; rests at %.3f m; after 4 s its walls ring with %.3f J of the %.0f J it "
               "landed with (fastest node %.3f m/s), %d failed solves\n", b->nodes.size(), b->fem.tris.size(), mass, c.y, ke, ke_max, vmax, b->fem.solve_failures);
        CHECK(std::fabs(c.y - 0.5f) < 0.03f && ke < 1e-3 * ke_max && b->fem.solve_failures == 0 && b->fem.tris_torn == 0, "cube: centre %.3f, %.3g J of %.3g, %d failures, %d torn",
              c.y, ke, ke_max, b->fem.solve_failures, b->fem.tris_torn);
    }
}

static void test_frame() {
    printf("frame elements (FEM)\n");
    const FrameSection tube = frame_tube(2e-4f);
    const double EI = (double)tube.E * tube.Iz, GJ = (double)tube.G * tube.J, EA = (double)tube.E * tube.A, kGA = (double)tube.G * tube.As_y;
    printf("    steel tube 40 x 2: %.2f kg/m, EA %.3g N, EI %.0f N m2, GJ %.0f N m2, Mp %.0f N m, Np %.0f N\n", tube.mass_per_m(), EA, EI, GJ, tube.Mp, tube.Np);
    // (quasi-static cases: backward Euler with full dissipation settles fast; dynamic ones: the defaults)
    auto world = [](bool settle) {
        auto w = std::make_unique<World>();
        w->settings.gravity = vec3(0);
        if (settle) w->settings.frame_theta = 1.0f, w->settings.frame_dissipation = 1.0f;
        return w;
    };
    const float L = 1.0f;
    // a cantilever: tip load, then tip torque, then a pull along it (quasi-static: backward Euler, damped)
    {
        struct Case {
            const char* name;
            vec3 force, torque;
        };
        const Case cases[] = {{"bending", vec3(0, -50, 0), vec3(0)}, {"torsion", vec3(0), vec3(20, 0, 0)}, {"tension", vec3(1e4f, 0, 0), vec3(0)}};
        for (const Case& c : cases) {
            auto w = world(true);
            SoftBody* b = w->add_body(frame_line(10, vec3(0, 1, 0), vec3(L, 1, 0), tube));
            const uint32_t tip = 10;
            const vec3 p0 = b->nodes[tip].p;
            b->pre_substep = [&](SoftBody& x, float) {
                x.force[tip] += c.force;
                x.fem.torque[x.fem.slot(tip)] += c.torque;
            };
            for (int i = 0; i < 4000; i++) w->step_substeps(1);
            const vec3 d = b->nodes[tip].p - p0;
            if (c.force.y != 0) {
                const double P = -c.force.y;
                const double want = P * L * L * L / (3 * EI) + P * L / kGA, got = -d.y;
                const double want_r = P * L * L / (2 * EI), got_r = -tip_rotation(*b, tip, 2);
                printf("    cantilever %s: tip %.4f mm (theory %.4f), rotation %.5f rad (theory %.5f)\n", c.name, got * 1e3, want * 1e3, got_r, want_r);
                CHECK(std::fabs(got / want - 1) < 0.01, "cantilever deflection %.5g vs %.5g", got, want);
                CHECK(std::fabs(got_r / want_r - 1) < 0.01, "cantilever tip rotation %.5g vs %.5g", got_r, want_r);
            } else if (c.torque.x != 0) {
                const double want = c.torque.x * L / GJ, got = tip_rotation(*b, tip, 0);
                printf("    cantilever %s: twist %.5f rad (theory %.5f)\n", c.name, got, want);
                CHECK(std::fabs(got / want - 1) < 0.01, "twist %.5g vs %.5g", got, want);
            } else {
                const double want = c.force.x * L / EA, got = d.x;
                printf("    cantilever %s: stretch %.4f mm (theory %.4f)\n", c.name, got * 1e3, want * 1e3);
                CHECK(std::fabs(got / want - 1) < 0.02, "stretch %.5g vs %.5g", got, want);
            }
            CHECK(b->fem.solve_failures == 0, "%s: %d failed solves", c.name, b->fem.solve_failures);
        }
    }
    // a portal frame with clamped feet and rigid corners: sway under a side load, K = 24 EI/H^3 (6k + 1)/(6k + 4),
    // k = (Ib / W) / (Ic / H)
    {
        auto w = world(true);
        auto body = std::make_unique<SoftBody>();
        body->name = "portal";
        body->can_sleep = false;
        const uint16_t si = body->fem.add_section(tube);
        const uint32_t f0 = body->add_node(vec3(0, 0, 0), 0.0f, NF_FIXED), f1 = body->add_node(vec3(1, 0, 0), 0.0f, NF_FIXED);
        const uint32_t c0 = body->add_node(vec3(0, 1, 0), 0.0f, NF_NONE), c1 = body->add_node(vec3(1, 1, 0), 0.0f, NF_NONE);
        frame_chain(*body, f0, c0, 4, si);
        frame_chain(*body, f1, c1, 4, si);
        frame_chain(*body, c0, c1, 4, si);
        fix_inv_mass(*body);
        body->fem.finalize(*body);
        SoftBody* b = w->add_body(std::move(body));
        const vec3 a0 = b->nodes[c0].p, a1 = b->nodes[c1].p;
        const float P = 200;
        b->pre_substep = [&](SoftBody& x, float) { x.force[c0] += vec3(P, 0, 0); };
        for (int i = 0; i < 4000; i++) w->step_substeps(1);
        const double got = 0.5 * ((b->nodes[c0].p - a0).x + (b->nodes[c1].p - a1).x);
        const double k = 1.0, K = 24 * EI * (6 * k + 1) / (6 * k + 4), want = P / K;
        printf("    portal frame: sway %.4f mm (theory %.4f)\n", got * 1e3, want * 1e3);
        CHECK(std::fabs(got / want - 1) < 0.02, "portal sway %.5g vs %.5g", got, want);
    }
    // the joints: a portal on ball feet sways PH^3 (2k + 1) / (12 EI k); a cantilever on a hinge swings freely in its
    // plane and stands as a cantilever across it, on a swivel twists freely and bends as a cantilever; on an elastic
    // joint its tip goes PL^3 / 3EI + PL / kGA + PL^2 / kj
    {
        auto w = world(true);
        auto body = std::make_unique<SoftBody>();
        body->name = "portal on balls";
        body->can_sleep = false;
        const uint16_t si = body->fem.add_section(tube);
        const uint32_t f0 = body->add_node(vec3(0, 0, 0), 0.0f, NF_FIXED), f1 = body->add_node(vec3(1, 0, 0), 0.0f, NF_FIXED);
        const uint32_t c0 = body->add_node(vec3(0, 1, 0), 0.0f, NF_NONE), c1 = body->add_node(vec3(1, 1, 0), 0.0f, NF_NONE);
        frame_chain(*body, f0, c0, 4, si);
        frame_chain(*body, f1, c1, 4, si);
        frame_chain(*body, c0, c1, 4, si);
        for (FrameElement& e : body->fem.elems)
            if (body->fem.node[e.a] == f0 || body->fem.node[e.a] == f1) e.end_a = FJ_BALL;
        fix_inv_mass(*body);
        body->fem.finalize(*body);
        SoftBody* b = w->add_body(std::move(body));
        const vec3 a0 = b->nodes[c0].p, a1 = b->nodes[c1].p;
        const float P = 100;
        b->pre_substep = [&](SoftBody& x, float) { x.force[c0] += vec3(P, 0, 0); };
        for (int i = 0; i < 8000; i++) w->step_substeps(1);
        const double got = 0.5 * ((b->nodes[c0].p - a0).x + (b->nodes[c1].p - a1).x), want = P * 3.0 / (12 * EI);
        printf("    portal on ball joints: sway %.4f mm (theory %.4f)\n", got * 1e3, want * 1e3);
        CHECK(std::fabs(got / want - 1) < 0.02, "ball-foot portal sway %.5g vs %.5g", got, want);
    }
    {
        struct Case {
            const char* name;
            uint8_t root;
            vec3 force, torque;
            int kind; // 0 cantilever deflection along the force, 1 free (moves far), 2 free twist, 3 elastic joint
        };
        const Case cases[] = {{"hinge, across its swing", FJ_HINGE_V, vec3(0, 0, 50), vec3(0), 0}, {"hinge, in its swing", FJ_HINGE_V, vec3(0, -20, 0), vec3(0), 1},
                              {"swivel, bent", FJ_SWIVEL, vec3(0, -50, 0), vec3(0), 0},        {"swivel, twisted", FJ_SWIVEL, vec3(0), vec3(5, 0, 0), 2},
                              {"elastic joint", FJ_ELASTIC, vec3(0, -50, 0), vec3(0), 3}};
        for (const Case& c : cases) {
            auto w = world(true);
            FrameSection s = tube;
            s.joint_k = 5.0e4f;
            SoftBody* b = w->add_body(frame_line(10, vec3(0, 1, 0), vec3(L, 1, 0), s, c.root));
            const uint32_t tip = 10;
            const vec3 p0 = b->nodes[tip].p;
            b->pre_substep = [&](SoftBody& x, float) {
                x.force[tip] += c.force;
                x.fem.torque[x.fem.slot(tip)] += c.torque;
            };
            for (int i = 0; i < 4000; i++) w->step_substeps(1);
            const vec3 d = b->nodes[tip].p - p0;
            const double P = length(c.force);
            if (c.kind == 0 || c.kind == 3) {
                const double got = std::fabs(dot(d, normalize(c.force)));
                double want = P * L * L * L / (3 * EI) + P * L / kGA;
                if (c.kind == 3) want += P * L * L / s.joint_k;
                printf("    cantilever on a %s: tip %.4f mm (theory %.4f)\n", c.name, got * 1e3, want * 1e3);
                CHECK(std::fabs(got / want - 1) < 0.01, "%s: %.5g vs %.5g", c.name, got, want);
            } else if (c.kind == 1) {
                printf("    cantilever on a %s: the tip swings %.0f mm\n", c.name, length(d) * 1e3);
                CHECK(length(d) > 0.3f, "%s: moved only %.4f m", c.name, length(d));
            } else {
                // (it spins on: the rate about its axis, the angle would wrap round)
                const float spin = std::fabs(b->fem.w[b->fem.slot(tip)].x);
                const float bend = length(vec3(0, b->fem.w[b->fem.slot(tip)].y, b->fem.w[b->fem.slot(tip)].z));
                printf("    cantilever on a %s: spins on its axis at %.0f rad/s (%.2g across)\n", c.name, spin, bend);
                CHECK(spin > 10.0f && bend < 0.01f * spin, "%s: spin %.3f rad/s, %.3g across", c.name, spin, bend);
            }
        }
    }
    // a member split while loaded keeps its state (the new node on its bent shape: the strain energy and the tip stay)
    {
        auto w = world(true);
        FrameSection s = tube;
        s.Np = s.Mp = s.Tp = 0; // (elastic)
        SoftBody* b = w->add_body(frame_line(4, vec3(0, 1, 0), vec3(L, 1, 0), s));
        const uint32_t tip = 4;
        b->pre_substep = [&](SoftBody& x, float) {
            x.force[tip] += vec3(0, -50, 30);
            x.fem.torque[x.fem.slot(tip)] += vec3(20, 0, 0);
        };
        for (int i = 0; i < 4000; i++) w->step_substeps(1);
        const double U0 = b->fem.strain_energy(*b);
        const vec3 t0 = b->nodes[tip].p;
        const size_t n0 = b->nodes.size();
        const int fm = b->fem.split(*b, 0, 0.5f);
        b->fem.process_events(*b); // (analysed again)
        b->fem.break_near(*b, vec3(1e3f), 1e-3f);
        const double U1 = b->fem.strain_energy(*b);
        for (int i = 0; i < 2000; i++) w->step_substeps(1);
        const vec3 t1 = b->nodes[tip].p;
        printf("    split under load: strain energy %.6f -> %.6f J, tip moved %.2g mm after\n", U0, U1, length(t1 - t0) * 1e3);
        CHECK(fm >= 0 && b->nodes.size() == n0 + 1 && b->fem.elems.size() == 5, "split: node %d, %zu nodes, %zu members", fm, b->nodes.size(), b->fem.elems.size());
        CHECK(std::fabs(U1 / U0 - 1) < 1e-3, "split changed the strain energy %.6g -> %.6g", U0, U1);
        CHECK(length(t1 - t0) < 2e-5f, "split moved the tip by %.3g m", length(t1 - t0));
    }
    // the destroy tool cuts a member where it passes: split there and torn apart (nothing removed)
    {
        auto w = world(true);
        SoftBody* b = w->add_body(frame_line(2, vec3(0, 1, 0), vec3(L, 1, 0), tube));
        const size_t n0 = b->nodes.size(), e0 = b->fem.elems.size();
        const int torn = b->fem.break_near(*b, vec3(0.25f, 1, 0), 0.02f);
        const vec3 v0 = b->nodes[2].p;
        b->pre_substep = [&](SoftBody& x, float) { x.force[2] += vec3(0, -20, 0); };
        for (int i = 0; i < 2000; i++) w->step_substeps(1);
        printf("    cut by the destroy tool: %d tear, %zu -> %zu nodes, %zu -> %zu members, the free part fell %.0f mm\n", torn, n0, b->nodes.size(), e0,
               b->fem.elems.size(), (v0.y - b->nodes[2].p.y) * 1e3);
        CHECK(torn == 1 && b->nodes.size() == n0 + 2 && b->fem.elems.size() == e0 + 1, "cut: %d tears, %zu nodes, %zu members", torn, b->nodes.size(), b->fem.elems.size());
        CHECK(v0.y - b->nodes[2].p.y > 0.2f, "the cut part did not come off (%.4f m)", v0.y - b->nodes[2].p.y);
    }
    // the first bending frequency of the cantilever (released from its loaded shape; the least numerical damping):
    // f1 = 1.8751^2 / (2 pi) sqrt(EI / (m L^4))
    {
        auto w = world(true);
        SoftBody* b = w->add_body(frame_line(10, vec3(0, 1, 0), vec3(L, 1, 0), tube));
        const uint32_t tip = 10;
        bool load = true;
        b->pre_substep = [&](SoftBody& x, float) {
            if (load) x.force[tip] += vec3(0, -50, 0);
        };
        for (int i = 0; i < 4000; i++) w->step_substeps(1);
        const float y0 = 1.0f;
        load = false;
        b->fem.sections[0].damping = 0;
        w->settings = World().settings;
        w->settings.gravity = vec3(0);
        std::vector<double> ups;
        float prev = b->nodes[tip].p.y - y0, amp0 = std::fabs(prev), amp_last = 0;
        const int steps = 1200;
        for (int i = 1; i <= steps; i++) {
            w->step_substeps(1);
            const float y = b->nodes[tip].p.y - y0;
            if (prev < 0 && y >= 0) ups.push_back(i - y / (y - prev));
            if (i > steps - 200) amp_last = std::max(amp_last, std::fabs(y));
            prev = y;
        }
        const double f = ups.size() >= 2 ? (ups.size() - 1) / ((ups.back() - ups.front()) * kDefaultDt) : 0;
        const double want = 1.8751 * 1.8751 / (2 * kPi) * std::sqrt(EI / (tube.mass_per_m() * (double)L * L * L * L));
        printf("    cantilever frequency %.2f Hz (theory %.2f), amplitude after %.1f s: %.0f%%\n", f, want, steps * kDefaultDt, 100.0 * amp_last / amp0);
        CHECK(std::fabs(f / want - 1) < 0.03, "frequency %.4g vs %.4g", f, want);
        CHECK(amp_last > 0.75f * amp0, "the free vibration died out too fast (%.0f%% left)", 100.0 * amp_last / amp0);
    }
    // a free square frame spinning and flying: it keeps its shape, and its energy (no damping)
    {
        auto w = world(false);
        w->settings.frame_dissipation = 0;
        auto body = std::make_unique<SoftBody>();
        body->name = "spinner";
        body->can_sleep = false;
        FrameSection s = tube;
        s.damping = 0;
        const uint16_t si = body->fem.add_section(s);
        const uint32_t c[4] = {body->add_node(vec3(0, 2, 0), 0.0f, NF_NONE), body->add_node(vec3(1, 2, 0), 0.0f, NF_NONE),
                               body->add_node(vec3(1, 2, 1), 0.0f, NF_NONE), body->add_node(vec3(0, 2, 1), 0.0f, NF_NONE)};
        for (int i = 0; i < 4; i++) frame_chain(*body, c[i], c[(i + 1) % 4], 2, si);
        fix_inv_mass(*body);
        body->fem.finalize(*body);
        SoftBody* b = w->add_body(std::move(body));
        const vec3 ctr(0.5f, 2, 0.5f), om(0.7f, 3.0f, -0.4f), v0(1, 0.5f, 0);
        for (Node& n : b->nodes) n.v = v0 + cross(om, n.p - ctr);
        for (vec3& x : b->fem.w) x = om;
        const double E0 = frame_energy(*b);
        const float d0 = length(b->nodes[c[0]].p - b->nodes[c[2]].p);
        double umax = 0;
        for (int i = 0; i < 4000; i++) {
            w->step_substeps(1);
            umax = std::max(umax, b->fem.strain_energy(*b));
        }
        const double E1 = frame_energy(*b);
        const float d1 = length(b->nodes[c[0]].p - b->nodes[c[2]].p);
        printf("    spinning frame: energy %.4f -> %.4f J, strain energy up to %.2g J, diagonal %.5f -> %.5f m\n", E0, E1, umax, d0, d1);
        CHECK(std::fabs(E1 / E0 - 1) < 0.01, "energy %.6g -> %.6g", E0, E1);
        CHECK(std::fabs(d1 - d0) < 1e-3f, "shape: diagonal %.6f -> %.6f", d0, d1);
        CHECK(umax < 1e-3 * E0, "strain energy %.3g in a rigid spin", umax);
    }
    // plastic hinges: below the collapse load the cantilever springs back, above it a hinge forms at the root and
    // holds the plastic moment (the tip sinks until the lever is short enough: cos a = Mp / (P L)) and keeps its set
    // when unloaded; far above it the hinge breaks
    {
        const double Pc = tube.Mp / L;
        struct Case {
            double f;
            double set_min, set_max;
            bool breaks;
        };
        // (with hardening the balance is P L cos a = Mp (1 + h a): 1.05 Pc sinks some 150 mm; a follower load across the
        // member keeps turning the hinge until it tears at its capacity)
        const Case cases[] = {{0.8, 0.0, 0.002, false}, {1.05, 0.08, 0.22, false}, {1.4, 0, 1e9, true}};
        for (const Case& c : cases) {
            auto w = world(true);
            FrameSection sec = tube;
            if (c.breaks) sec.hardening = 0; // (the root hinge alone takes the rotation, up to its capacity)
            SoftBody* b = w->add_body(frame_line(10, vec3(0, 1, 0), vec3(L, 1, 0), sec));
            const uint32_t tip = 10;
            double P = 0;
            // (a dashpot on the tip: the collapse is a mechanism, without it the tip would overshoot the balance)
            b->pre_substep = [&](SoftBody& x, float) {
                if (x.fem.broken) return;
                vec3 dir(0, -1, 0);
                if (c.breaks) { // (across the root member: it keeps bending the root hinge)
                    const vec3 d = normalize(x.nodes[1].p - x.nodes[0].p);
                    dir = vec3(d.y, -d.x, 0);
                }
                x.force[tip] += dir * (float)P - x.nodes[tip].v * 40.0f;
            };
            float peak_util = 0, max_drop = 0;
            for (int i = 0; i < 8000; i++) {
                const double t = i * kDefaultDt; // 1 s up, 1 s held, 1 s down, 1 s at rest
                P = c.f * Pc * (t < 1 ? t : t < 2 || c.breaks ? 1 : t < 3 ? 3 - t : 0);
                w->step_substeps(1);
                if (!b->fem.elems[0].broken) peak_util = std::max(peak_util, b->fem.elems[0].util);
                max_drop = std::max(max_drop, 1.0f - b->nodes[tip].p.y);
            }
            const float set = 1.0f - b->nodes[tip].p.y;
            const bool broke = b->fem.broken > 0;
            if (broke) CHECK(b->fem.elems.size() >= 10 && b->nodes.size() > 11, "torn, not removed: %zu members, %zu nodes", b->fem.elems.size(), b->nodes.size());
            printf("    plastic, %.2f x collapse load%s: peak %.3f of the yield at the root, sank %.1f mm, set %.1f mm, hinge %.2f rad%s\n", c.f, c.breaks ? " (following)" : "",
                   peak_util, max_drop * 1e3, set * 1e3, b->fem.elems[0].dmg_a, broke ? ", torn off" : "");
            CHECK(broke == c.breaks, "%.2f Pc: broken %d", c.f, (int)broke);
            if (!c.breaks) {
                CHECK(set >= c.set_min && set <= c.set_max, "%.2f Pc: set %.4f m (%.3f - %.3f)", c.f, set, c.set_min, c.set_max);
                CHECK(peak_util <= 1.02f, "%.2f Pc: the root moment reached %.3f of the (hardened) yield", c.f, peak_util);
            } else {
                CHECK(b->fem.elems[0].dmg_a >= tube.hinge_capacity * 0.99f, "torn before the joint's capacity: %.3f of %.3f rad", b->fem.elems[0].dmg_a,
                      tube.hinge_capacity);
            }
        }
    }
    // a frame box with a sheet-metal roof, dropped on the ground: sound, and the same on one thread and on many
    {
        auto run = [&](bool mt, double& hash, float& vmax, int& failures, int& broken, bool& finite) {
            World w;
            w.settings.multithreaded = mt;
            w.statics.terrain.create(21, 21, 1.0f, vec2(-10, -10));
            w.statics.has_terrain = true;
            w.statics.terrain.update_bounds();
            auto body = std::make_unique<SoftBody>();
            body->name = "frame box";
            const uint16_t si = body->fem.add_section(frame_tube(2e-5f));
            const float y0 = 0.4f, H = 0.6f, S = 1.0f;
            const int n = 4; // (roof edges split to take the sheet)
            uint32_t k[8];
            for (int i = 0; i < 8; i++) k[i] = body->add_node(vec3((i & 1) ? S : 0, y0 + ((i & 4) ? H : 0), (i & 2) ? S : 0), 0.0f);
            // the roof grid (n x n cells) with its border on the roof's members
            std::vector<uint32_t> g((n + 1) * (n + 1), ~0u);
            auto gid = [&](int i, int j) -> uint32_t& { return g[j * (n + 1) + i]; };
            gid(0, 0) = k[4], gid(n, 0) = k[5], gid(0, n) = k[6], gid(n, n) = k[7];
            auto edge = [&](uint32_t a, uint32_t b, int i0, int j0, int di, int dj) {
                std::vector<uint32_t> made;
                frame_chain(*body, a, b, n, si, &made);
                for (int t = 1; t < n; t++) gid(i0 + di * t, j0 + dj * t) = made[t - 1];
            };
            edge(k[4], k[5], 0, 0, 1, 0);
            edge(k[6], k[7], 0, n, 1, 0);
            edge(k[4], k[6], 0, 0, 0, 1);
            edge(k[5], k[7], n, 0, 0, 1);
            for (int j = 1; j < n; j++)
                for (int i = 1; i < n; i++) gid(i, j) = body->add_node(vec3(S * i / n, y0 + H, S * j / n), 0.0f);
            // the posts and the floor
            frame_chain(*body, k[0], k[1], 2, si), frame_chain(*body, k[2], k[3], 2, si);
            frame_chain(*body, k[0], k[2], 2, si), frame_chain(*body, k[1], k[3], 2, si);
            for (int i = 0; i < 4; i++) frame_chain(*body, k[i], k[i + 4], 2, si);
            for (int j = 0; j < n; j++)
                for (int i = 0; i < n; i++) {
                    const uint32_t a = gid(i, j), bb = gid(i + 1, j), cc = gid(i + 1, j + 1), e = gid(i, j + 1);
                    auto uv = [&](int x, int y) { return vec2((float)x / n, (float)y / n); };
                    body->add_shell(a, e, cc, uv(i, j), uv(i, j + 1), uv(i + 1, j + 1));
                    body->add_shell(a, cc, bb, uv(i, j), uv(i + 1, j + 1), uv(i + 1, j));
                }
            body->shell_mat = mat_steel();
            fix_inv_mass(*body);
            body->finalize();
            body->finalize_shells(8.0f, kDefaultDt, 3, true);
            body->fem.finalize(*body);
            for (Node& x : body->nodes) x.v = vec3(0.3f, -3.0f, 0);
            SoftBody* b = w.add_body(std::move(body));
            vmax = 0;
            for (int f = 0; f < 60; f++) {
                w.step_substeps(33);
                if (f > 40)
                    for (const Node& x : b->nodes) vmax = std::max(vmax, length(x.v));
            }
            hash = 0;
            finite = true;
            for (size_t i = 0; i < b->nodes.size(); i++) {
                const Node& x = b->nodes[i];
                hash += (x.p.x * 1.3 + x.p.y * 2.7 + x.p.z * 3.1) * (double)((i % 97) + 1);
                finite &= std::isfinite(x.p.x + x.p.y + x.p.z);
            }
            failures = b->fem.solve_failures;
            broken = b->fem.broken;
        };
        double h1, h2;
        float v1, v2;
        int f1, f2, b1, b2;
        bool ok1, ok2;
        run(false, h1, v1, f1, b1, ok1);
        run(true, h2, v2, f2, b2, ok2);
        printf("    frame box with a roof dropped: speed after 1.4 s %.3f m/s, %d failed solves, %d members broken\n", v1, f1, b1);
        CHECK(ok1 && ok2, "non-finite state");
        CHECK(f1 == 0 && b1 == 0, "failed solves %d, broken members %d", f1, b1);
        CHECK(v1 < 0.5f, "still moving at %.3f m/s", v1);
        CHECK(h1 == h2, "multithreaded run differs (%.9g vs %.9g)", h1, h2);
    }
    // a small piece torn off a frame leaves it as a rigid body (the frame goes on without those members) and falls
    {
        World w;
        w.statics.terrain.create(21, 21, 1.0f, vec2(-10, -10));
        w.statics.has_terrain = true;
        w.statics.terrain.update_bounds();
        auto body = std::make_unique<SoftBody>();
        body->name = "frame box";
        const uint16_t si = body->fem.add_section(frame_tube(2e-5f));
        uint32_t k[8];
        for (int i = 0; i < 8; i++) k[i] = body->add_node(vec3((i & 1) ? 1.0f : 0, 0.3f + ((i & 4) ? 0.6f : 0), (i & 2) ? 1.0f : 0), 0.0f);
        for (int i = 0; i < 8; i += 4) {
            frame_chain(*body, k[i], k[i + 1], 2, si), frame_chain(*body, k[i + 2], k[i + 3], 2, si);
            frame_chain(*body, k[i], k[i + 2], 2, si), frame_chain(*body, k[i + 1], k[i + 3], 2, si);
        }
        for (int i = 0; i < 4; i++) frame_chain(*body, k[i], k[i + 4], 2, si);
        const uint32_t tip = body->add_node(vec3(1.6f, 0.9f, 0), 0.0f);
        const uint32_t stick = (uint32_t)body->fem.elems.size();
        frame_chain(*body, k[5], tip, 2, si);
        fix_inv_mass(*body);
        body->finalize();
        body->fem.finalize(*body);
        SoftBody* b = w.add_body(std::move(body));
        for (int f = 0; f < 30; f++) w.step_substeps(33);
        const size_t members0 = b->fem.elems.size(), bodies0 = w.bodies().size();
        const bool torn = b->fem.tear(*b, stick, 0);
        b->topo_changed = true, b->topo_version++, b->shk.version++;
        for (int f = 0; f < 90; f++) w.step_substeps(33);
        const SoftBody* piece = nullptr;
        for (size_t i = bodies0; i < w.bodies().size(); i++)
            if (!w.bodies()[i]->fem.empty()) piece = w.bodies()[i].get();
        float ymin = 1e9f, vmax = 0;
        if (piece)
            for (const Node& x : piece->nodes) ymin = std::min(ymin, x.p.y), vmax = std::max(vmax, length(x.v));
        printf("    stick torn off a frame box: %s, %zu members (the box %zu -> %zu), at rest %.3f m/s, lowest point %.3f m\n", piece ? "a rigid piece" : "no piece",
               piece ? piece->fem.elems.size() : 0, members0, b->fem.elems.size(), vmax, ymin);
        CHECK(torn, "the tear was refused");
        CHECK(piece && piece->rigid && piece->fem.elems.size() == 2, "the stick did not leave as a rigid piece of 2 members");
        CHECK(b->fem.elems.size() == members0 - 2 && b->fem.solve_failures == 0, "the box has %zu members (want %zu), %d failed solves", b->fem.elems.size(), members0 - 2,
              b->fem.solve_failures);
        CHECK(piece && ymin > -0.05f && vmax < 0.3f, "the piece is not resting on the ground (lowest %.3f, speed %.3f)", ymin, vmax);
    }    // a mount (a section with a break force) lets go when its force has stood over it for FemFrame::kOverloadTime, not
    // on a short peak: a cantilever whose first member is the mount, a load at its tip
    {
        struct Case {
            const char* name;
            float P;
            int steps_on; // (substeps the load stays on)
            bool tears;
        };
        const Case cases[] = {{"at 0.8 of its force", 400, 800, false}, {"a peak of 4 times it for 1 ms", 2000, 2, false}, {"at twice it", 1000, 800, true}};
        for (const Case& c : cases) {
            auto w = world(true);
            auto body = std::make_unique<SoftBody>();
            body->name = "mount";
            body->can_sleep = false;
            FrameSection ms = tube;
            ms.break_force = 500;
            const uint16_t sm = body->fem.add_section(ms), st = body->fem.add_section(tube);
            const float m = tube.mass_per_m() * 0.1f;
            for (int i = 0; i <= 4; i++) body->add_node(vec3(0.1f * i, 1, 0), i == 0 || i == 4 ? m * 0.5f : m, i == 0 ? NF_FIXED : NF_NONE);
            for (int i = 0; i < 4; i++) body->fem.add_element(i, i + 1, i == 0 ? sm : st);
            body->fem.finalize(*body);
            SoftBody* b = w->add_body(std::move(body));
            int step = 0, torn_at = -1;
            b->pre_substep = [&](SoftBody& x, float) {
                if (step < c.steps_on && torn_at < 0) x.force[4] += vec3(0, -c.P, 0); // (off once it let go)
            };
            for (; step < 1000; step++) {
                w->step_substeps(1);
                if (torn_at < 0 && (b->fem.elems[0].torn & 1)) torn_at = step;
            }
            const float dt_ms = w->settings.dt * 1000.0f;
            printf("    mount (breaks at 500 N) %s: %s\n", c.name, torn_at >= 0 ? "let go" : "held");
            if (torn_at >= 0) printf("      after %.1f ms\n", torn_at * dt_ms);
            CHECK((torn_at >= 0) == c.tears, "mount %s: %s", c.name, torn_at >= 0 ? "let go" : "held");
            // (within a step of the frame's: the overload is counted at its steps)
            if (c.tears)
                CHECK(torn_at * dt_ms >= 1000.0f * FemFrame::kOverloadTime - dt_ms * (float)w->settings.frame_every, "let go after %.1f ms, before the overload time",
                      torn_at * dt_ms);
        }
    }
    // a released joint's damping (FrameSection::joint_damp) is implicit: an arm on a ball joint (its node held by a
    // welded stub) swings on under gravity undamped, comes to rest hanging with a little damping, and with a huge one
    // hardly moves and stays sound (explicit, 2 N m s/rad on a lid's light node tore its hinges off)
    {
        const float damps[] = {0.0f, 0.5f, 1.0e5f};
        for (float c : damps) {
            auto w = world(false);
            w->settings.gravity = vec3(0, -9.81f, 0);
            auto body = std::make_unique<SoftBody>();
            body->name = "damped arm";
            body->can_sleep = false;
            FrameSection s = tube;
            s.joint_damp = c;
            const uint16_t si = body->fem.add_section(s);
            const float m = tube.mass_per_m() * 0.25f;
            const uint32_t a = body->add_node(vec3(0, 1, 0), 0.0f, NF_FIXED), pv = body->add_node(vec3(0.1f, 1, 0), m * 0.5f, NF_NONE);
            const uint32_t mid = body->add_node(vec3(0.35f, 1, 0), m, NF_NONE), tip = body->add_node(vec3(0.6f, 1, 0), m * 0.5f, NF_NONE);
            body->fem.add_element(a, pv, si);
            body->fem.add_element(pv, mid, si, FJ_BALL, FJ_RIGID); // (the joint: released at the pivot)
            body->fem.add_element(mid, tip, si);
            fix_inv_mass(*body);
            body->fem.finalize(*body);
            SoftBody* b = w->add_body(std::move(body));
            float vlast = 0, ymin = 1e9f;
            bool finite = true;
            for (int step = 0; step < 6000; step++) {
                w->step_substeps(1);
                const Node& t = b->nodes[tip];
                finite &= std::isfinite(t.p.x + t.p.y + t.v.x + t.v.y);
                ymin = std::min(ymin, t.p.y);
                if (step >= 5000) vlast = std::max(vlast, length(t.v));
            }
            printf("    arm on a ball joint damped %g N m s/rad: the tip went down to %.3f m, its speed in the last 0.5 s %.3f m/s\n", c, ymin, vlast);
            CHECK(finite && b->fem.solve_failures == 0, "damping %g: not finite or %d failed solves", c, b->fem.solve_failures);
            if (c == 0) CHECK(vlast > 1.0f, "undamped arm stopped (%.3f m/s)", vlast);
            else if (c < 100) CHECK(vlast < 0.05f && b->nodes[tip].p.y < 0.6f, "damped arm still moving at %.3f m/s, tip at %.3f", vlast, b->nodes[tip].p.y);
            else CHECK(ymin > 0.98f && vlast < 0.01f, "the heavily damped arm moved: down to %.3f m, %.3f m/s", ymin, vlast);
        }
    }    // mounts (FrameMount): a part on a fixed frame at a distance, no member between them - a component of its own; it
    // hangs there, lets go when its load stands past the break force for kOverloadTime; on two mounts on a line it swings
    // as on a hinge, their turning damping stills it, a huge one stays sound
    {
        auto make = [&](float brk, float damp, bool hinge) {
            auto body = std::make_unique<SoftBody>();
            body->name = "mounted";
            body->can_sleep = false;
            const uint16_t si = body->fem.add_section(tube);
            // the fixed frame: a tetrahedron of members
            const vec3 fp[4] = {vec3(0, 1, 0), vec3(0.5f, 1, 0), vec3(0, 1, 0.5f), vec3(0.2f, 1.5f, 0.2f)};
            uint32_t f[4];
            for (int i = 0; i < 4; i++) f[i] = body->add_node(fp[i], 0.0f, i < 3 ? NF_FIXED : NF_NONE);
            for (int i = 0; i < 4; i++)
                for (int j = i + 1; j < 4; j++) frame_chain(*body, f[i], f[j], 2, si);
            // the part: a square with a diagonal, 4 cm under the frame (or hanging from its x edge: a hinge)
            const float y = 0.96f;
            const vec3 qp[4] = {vec3(0.05f, y, 0.05f), vec3(0.45f, y, 0.05f), vec3(0.45f, y, 0.45f), vec3(0.05f, y, 0.45f)};
            uint32_t q[4];
            for (int i = 0; i < 4; i++) q[i] = body->add_node(qp[i], 0.5f, NF_NONE);
            for (int i = 0; i < 4; i++) frame_chain(*body, q[i], q[(i + 1) % 4], 2, si);
            frame_chain(*body, q[0], q[2], 2, si);
            fix_inv_mass(*body);
            if (hinge) {
                body->fem.add_mount(f[0], q[0], brk, 2.0e6f, damp);
                body->fem.add_mount(f[1], q[1], brk, 2.0e6f, damp);
            } else {
                body->fem.add_mount(f[0], q[0], brk, 2.0e6f, damp);
                body->fem.add_mount(f[1], q[1], brk, 2.0e6f, damp);
                body->fem.add_mount(f[2], q[3], brk, 2.0e6f, damp);
            }
            body->fem.finalize(*body);
            return std::make_pair(std::move(body), q[2]);
        };
        {
            auto w = world(false);
            w->settings.gravity = vec3(0, -9.81f, 0);
            auto [body, far] = make(0.0f, 0.0f, false);
            SoftBody* b = w->add_body(std::move(body));
            const vec3 p0 = b->nodes[far].p;
            float vlast = 0;
            for (int step = 0; step < 4000; step++) {
                w->step_substeps(1);
                if (step >= 3000) vlast = std::max(vlast, length(b->nodes[far].v));
            }
            const float sag = p0.y - b->nodes[far].p.y;
            printf("    a part on three mounts 4 cm under a frame: %d components, the far corner sank %.2f mm, %.4f m/s at the end\n", b->fem.components(), sag * 1e3f, vlast);
            CHECK(b->fem.components() == 2, "%d components (want the frame and the part)", b->fem.components());
            CHECK(sag > 0 && sag < 0.003f && vlast < 0.01f && b->fem.solve_failures == 0, "the mounted part: sank %.4f m, %.4f m/s, %d failed solves", sag, vlast, b->fem.solve_failures);
        }
        {
            struct Case {
                const char* name;
                float P;      // N down on each mounted node of the part
                int steps_on;
                bool lets_go;
            };
            const Case cases[] = {{"at 0.6 of its force", 0.6f * 200, 800, false}, {"a peak of 4 times it for 1 ms", 4 * 200.0f, 2, false}, {"at twice it", 2 * 200.0f, 800, true}};
            for (const Case& c : cases) {
                auto w = world(true);
                auto [body, far] = make(200.0f, 0.0f, false);
                SoftBody* b = w->add_body(std::move(body));
                int step = 0;
                b->pre_substep = [&](SoftBody& x, float) {
                    if (step < c.steps_on && x.fem.mounts_broken == 0)
                        for (const FrameMount& m : x.fem.mounts) x.force[m.b] += vec3(0, -c.P, 0);
                };
                for (; step < 1000; step++) w->step_substeps(1);
                printf("    mounted part (3 x 200 N) loaded on its mounts %s: %d of 3 let go\n", c.name, b->fem.mounts_broken);
                CHECK((b->fem.mounts_broken > 0) == c.lets_go, "mounts %s: %d let go", c.name, b->fem.mounts_broken);
            }
        }
        {
            const float damps[] = {0.0f, 1.0f, 1.0e5f};
            for (float dmp : damps) {
                auto w = world(false);
                w->settings.gravity = vec3(0, -9.81f, 0);
                auto [body, far] = make(0.0f, dmp, true);
                SoftBody* b = w->add_body(std::move(body));
                float ymin = 1e9f, vlast = 0;
                bool finite = true;
                for (int step = 0; step < 6000; step++) {
                    w->step_substeps(1);
                    const Node& t = b->nodes[far];
                    finite &= std::isfinite(t.p.x + t.p.y + t.p.z + t.v.x + t.v.y + t.v.z);
                    ymin = std::min(ymin, t.p.y);
                    if (step >= 5000) vlast = std::max(vlast, length(t.v));
                }
                printf("    a square on two mounts (a hinge) damped %g N m s/rad: its far corner down to %.3f m, %.3f m/s in the last 0.5 s\n", dmp, ymin, vlast);
                CHECK(finite && b->fem.solve_failures == 0, "hinged on mounts, damping %g: not finite or %d failed solves", dmp, b->fem.solve_failures);
                if (dmp == 0) CHECK(ymin < 0.7f && vlast > 0.5f, "the undamped hinge: down to %.3f, %.3f m/s", ymin, vlast);
                else if (dmp < 100) CHECK(ymin < 0.7f && vlast < 0.05f, "the damped hinge: down to %.3f, still %.3f m/s", ymin, vlast);
                else CHECK(ymin > 0.9f, "the heavily damped hinge fell to %.3f", ymin);
            }
        }
    }
    // mount kinds (FrameMount): the same frame and square part. A clamp alone holds it (the part's nodes round the bolt
    // held: it does not swing about the bolt, as it does on a point) and lets go past its break moment below its break
    // force; a hinge alone (two of the part's nodes on its line) lets it turn about that line only; a stop keeps a lid
    // hinged over the frame from falling through it; a strap ends a part's swing
    {
        struct MountSpec {
            int f, q;           // the frame's node, the part's (indices in the tetrahedron, the square)
            MountKind kind;
            float brk, param;
            int q2;             // a hinge's second node of the part
        };
        auto make = [&](float y, std::vector<MountSpec> specs, uint32_t* fn, uint32_t* qn) {
            auto body = std::make_unique<SoftBody>();
            body->name = "mounted";
            body->can_sleep = false;
            const uint16_t si = body->fem.add_section(tube);
            const vec3 fp[4] = {vec3(0, 1, 0), vec3(0.5f, 1, 0), vec3(0, 1, 0.5f), vec3(0.2f, 1.5f, 0.2f)};
            for (int i = 0; i < 4; i++) fn[i] = body->add_node(fp[i], 0.0f, i < 3 ? NF_FIXED : NF_NONE);
            for (int i = 0; i < 4; i++)
                for (int j = i + 1; j < 4; j++) frame_chain(*body, fn[i], fn[j], 2, si);
            const vec3 qp[4] = {vec3(0.05f, y, 0.05f), vec3(0.45f, y, 0.05f), vec3(0.45f, y, 0.45f), vec3(0.05f, y, 0.45f)};
            for (int i = 0; i < 4; i++) qn[i] = body->add_node(qp[i], 0.5f, NF_NONE);
            for (int i = 0; i < 4; i++) frame_chain(*body, qn[i], qn[(i + 1) % 4], 2, si);
            frame_chain(*body, qn[0], qn[2], 2, si);
            fix_inv_mass(*body);
            for (const MountSpec& m : specs)
                body->fem.add_mount(fn[m.f], qn[m.q], m.brk, 0.0f, 0.0f, m.kind, m.param, m.q2 >= 0 ? qn[m.q2] : 0);
            body->fem.finalize(*body);
            return body;
        };
        auto run = [&](std::unique_ptr<SoftBody> body, int steps, const std::function<void(SoftBody&, int)>& load, const std::function<void(const SoftBody&)>& watch) {
            auto w = world(false);
            w->settings.gravity = vec3(0, -9.81f, 0);
            SoftBody* b = w->add_body(std::move(body));
            int step = 0;
            b->pre_substep = [&](SoftBody& x, float) { load(x, step); };
            bool finite = true;
            for (; step < steps; step++) {
                w->step_substeps(1);
                for (const Node& n : b->nodes) finite &= std::isfinite(n.p.x + n.p.y + n.p.z);
                watch(*b);
            }
            CHECK(finite && b->fem.solve_failures == 0, "mount kinds: not finite or %d failed solves", b->fem.solve_failures);
            return b->fem.mounts_broken;
        };
        const auto none = [](SoftBody&, int) {};
        uint32_t f[4], q[4];
        // hanging from one mount 4 cm under the frame: a point (it swings down), a clamp (it holds)
        for (MountKind k : {MountKind::Point, MountKind::Clamp}) {
            float ymin = 1e9f;
            run(make(0.96f, {{0, 0, k, 0.0f, 0.0f, -1}}, f, q), 4000, none, [&](const SoftBody& b) { ymin = std::min(ymin, b.nodes[q[2]].p.y); });
            printf("    a square hung from one %s: its far corner down to %.3f m\n", k == MountKind::Point ? "point mount" : "clamp", ymin);
            if (k == MountKind::Point) CHECK(ymin < 0.7f, "on a point mount the part did not swing down (%.3f)", ymin);
            else CHECK(ymin > 0.93f, "on a clamp the part swung down to %.3f", ymin);
        }
        // a clamp (1000 N, 55 N m) and a load at the far corner: 20 N holds (11 N m, the square's own 7.8 kg 27 N m more
        // about the bolt), 80 N twists it off (46 N m and that)
        for (float P : {20.0f, 80.0f}) {
            float mmax = 0;
            const int broken = run(make(0.96f, {{0, 0, MountKind::Clamp, 1000.0f, 55.0f, -1}}, f, q), 1500,
                                   [&](SoftBody& x, int) { x.force[q[2]] += vec3(0, -P, 0); }, [&](const SoftBody& b) {
                                       if (!b.fem.mounts.empty() && !b.fem.mounts[0].broken) mmax = std::max(mmax, b.fem.mounts[0].m);
                                   });
            printf("    a clamp (1000 N, 55 N m) with %.0f N at the far corner: %s (%.0f N m about its bolt)\n", P, broken ? "let go" : "held", mmax);
            CHECK((broken > 0) == (P > 50.0f), "the clamp with %.0f N at the far corner: %d let go", P, broken);
        }
        // one hinge (the part's two nodes on the x line): the far edge swings down, the hinge's line stays
        {
            float ymin = 1e9f, ymax_line = -1e9f, ymin_line = 1e9f;
            run(make(0.96f, {{0, 0, MountKind::Hinge, 0.0f, 0.0f, 1}}, f, q), 4000, none, [&](const SoftBody& b) {
                ymin = std::min(ymin, b.nodes[q[2]].p.y);
                ymin_line = std::min(ymin_line, b.nodes[q[1]].p.y), ymax_line = std::max(ymax_line, b.nodes[q[1]].p.y);
            });
            printf("    a square on one hinge: its far corner down to %.3f m, the hinge's far node within %.1f mm\n", ymin, (ymax_line - ymin_line) * 1e3f);
            CHECK(ymin < 0.7f && ymax_line - ymin_line < 0.01f, "one hinge: far corner %.3f, the line's node moved %.4f", ymin, ymax_line - ymin_line);
        }
        // a lid hinged (two points on the x line) 4 cm over the frame: it falls through it, or rests on a stop
        for (bool stop : {false, true}) {
            std::vector<MountSpec> ms = {{0, 0, MountKind::Point, 0.0f, 0.0f, -1}, {1, 1, MountKind::Point, 0.0f, 0.0f, -1}};
            if (stop) ms.push_back({2, 3, MountKind::Stop, 0.0f, 0.0f, -1});
            float ymin = 1e9f;
            run(make(1.04f, ms, f, q), 3000, none, [&](const SoftBody& b) { ymin = std::min(ymin, b.nodes[q[3]].p.y); });
            printf("    a lid hinged over the frame %s: its free edge down to %.3f m\n", stop ? "on a stop" : "with no stop", ymin);
            if (stop) CHECK(ymin > 1.02f, "the lid on its stop sank to %.3f", ymin);
            else CHECK(ymin < 0.8f, "the lid with no stop stayed at %.3f", ymin);
        }
        // a part swinging on a point mount, a strap from the frame's far corner to its far corner (1.2 times the rest)
        for (bool strap : {false, true}) {
            std::vector<MountSpec> ms = {{0, 0, MountKind::Point, 0.0f, 0.0f, -1}};
            if (strap) ms.push_back({1, 2, MountKind::Strap, 0.0f, 1.2f, -1});
            float dmax = 0;
            float L0 = 0;
            run(make(0.96f, ms, f, q), 3000, none, [&](const SoftBody& b) {
                const float d = length(b.nodes[q[2]].p - b.nodes[f[1]].p);
                if (L0 == 0) L0 = d;
                dmax = std::max(dmax, d);
            });
            printf("    a swinging part %s: its far corner out to %.2f times its rest distance from the frame's\n", strap ? "on a strap (1.2)" : "with no strap", dmax / L0);
            if (strap) CHECK(dmax < 1.25f * L0, "the strap let it out to %.2f", dmax / L0);
            else CHECK(dmax > 1.4f * L0, "with no strap it went out to %.2f only", dmax / L0);
        }
    }
}

// A collision hull's triangle (Triangle::two_sided false: one-sided, solid behind to the body's hull_depth): a ball
// pushed 15 cm behind it comes back out in front, no faster than the push-out speed (2 m/s: RoR's contact, held for
// many steps, pumped it out at 6.5 m/s). Behind an ordinary two-sided triangle the same ball stays where it is (out of
// the thin surface's reach: a node that gets past its middle is pushed on through).
struct HullRun {
    float centre = 0, speed = 0; // (the ball's centre from the plane after 1 s, its fastest speed)
};
static HullRun run_hull(bool hull) {
    World W;
    W.settings.gravity = vec3(0);
    auto plate = std::make_unique<SoftBody>();
    plate->name = "hull plate";
    const vec3 o(0, 5, 0);
    const uint32_t a = plate->add_node(o + vec3(-2, 0, -2), 50), b = plate->add_node(o + vec3(2, 0, -2), 50), c = plate->add_node(o + vec3(0, 0, 3), 50);
    plate->add_beam(a, b, 1e6f, 100, 1e12f, 1e12f), plate->add_beam(b, c, 1e6f, 100, 1e12f, 1e12f), plate->add_beam(c, a, 1e6f, 100, 1e12f, 1e12f);
    plate->add_triangle(a, c, b); // (facing +y)
    plate->finalize();
    plate->tris[0].two_sided = !hull;
    for (Node& n : plate->nodes) n.inv_mass = 0;
    W.add_body(std::move(plate));
    SoftBody* ball = W.add_body(make_ball(o + vec3(0, -0.15f, 0), 0.08f, 4.0f, vec3(0)));
    if (getenv("BL_HULLDBG"))
        for (int k = 0; k < 12; k++) {
            W.step_substeps(1);
            float vmax = 0, vmin = 1e9f;
            for (const Node& n : ball->nodes) vmax = std::max(vmax, n.v.y), vmin = std::min(vmin, n.v.y);
            printf("    substep %2d: node vy %.2f .. %.2f\n", k, vmin, vmax);
        }
    HullRun r;
    for (int f = 0; f < 60; f++) {
        W.step_substeps(33);
        vec3 v(0);
        float m = 0;
        for (const Node& n : ball->nodes) v += n.v * n.mass, m += n.mass;
        r.speed = std::max(r.speed, length(v / m));
        if (getenv("BL_HULLDBG") && (f < 6 || f % 10 == 0)) printf("    frame %2d: centre %.3f m, velocity %.2f m/s\n", f, ball->center_of_mass().y - o.y, v.y / m);
    }
    r.centre = ball->center_of_mass().y - o.y;
    return r;
}

static void test_hull() {
    printf("collision hull: a ball pushed behind a hull triangle comes back out\n");
    const HullRun out = run_hull(true), stays = run_hull(false);
    CHECK(out.centre > 0.05f, "behind a hull triangle the ball stayed at %.3f m", out.centre);
    CHECK(out.speed < 2.1f, "the ball left the hull at %.2f m/s (pumped out)", out.speed);
    CHECK(stays.centre < -0.1f, "behind a two-sided triangle the ball moved to %.3f m", stays.centre);
    printf("    the ball's centre from the plane: %.3f m (hull, left at %.2f m/s), %.3f m (two-sided)\n", out.centre, out.speed, stays.centre);
}

// ---- one-sided springs, balls, collision volumes: a frame falling freely with a node hung on a spring under it falls at
// g as a whole (the spring's implicit part taken on the frame's side alone - the other end held still for the step -
// took momentum out: a driven FEM car lost a third of its tyres' push); a ball of one node and a capsule rests on the
// ground at its radius; a node thrown at a body's collision volume is kept out of it, the momentum kept
// A ring tyre (Wheel::ring) on a quarter car: two axle nodes carrying 300 kg on flat ground - its deflection at rest
// against the one its stiffness was given for (the patch as a circle pressed into the flat), still; driven, it rolls
// the load on with the tread nearly not slipping; braked, it stops
static void test_ring_tyre() {
    printf("ring tyres\n");
    auto make = [](World& w) {
        w.settings.gravity = vec3(0, -9.81f, 0);
        w.statics.terrain.create(41, 81, 1.0f, vec2(-20, -20));
        w.statics.has_terrain = true;
        w.statics.terrain.update_bounds();
        auto body = std::make_unique<SoftBody>();
        body->name = "quarter car";
        body->can_sleep = false;
        const uint32_t a0 = body->add_node(vec3(0, 0.35f, -0.12f), 150.0f, NF_NONE), a1 = body->add_node(vec3(0, 0.35f, 0.12f), 150.0f, NF_NONE);
        body->add_beam(a0, a1, 1e7f, 1e3f, 1e12f, 1e12f);
        Wheel wh;
        wh.ring = true, wh.type = Wheel::W_RING;
        wh.axle0 = a0, wh.axle1 = a1;
        wh.radius = 0.32f, wh.rim_radius = 0.2f, wh.width = 0.22f, wh.mass = 20;
        wh.inertia = 0.5f * 20 * 0.32f * 0.32f;
        const float patch = (4.0f / 3.0f) * 0.22f * std::sqrt(2.0f * 0.32f * 0.02f);
        wh.k_area = 2.0e5f / patch, wh.c_area = 1500.0f / patch, wh.k_shear = 1.5f * wh.k_area, wh.c_shear = 2.5f * wh.c_area;
        wh.propulsed = 1, wh.braked = 1;
        body->wheels.push_back(wh);
        body->finalize();
        return w.add_body(std::move(body));
    };
    World w;
    SoftBody* b = make(w);
    for (int f = 0; f < 120; f++) w.step_substeps(33);
    const float y = 0.5f * (b->nodes[0].p.y + b->nodes[1].p.y), defl = 0.32f - y;
    // (the load F = k_area w (4/3) d sqrt(2 R d) for the deflection d: solved for 300 kg)
    const float kw = b->wheels[0].k_area * 0.22f * (4.0f / 3.0f) * std::sqrt(2.0f * 0.32f), want = std::pow(300.0f * 9.81f / kw, 1.0f / 1.5f);
    CHECK(std::fabs(defl - want) < 0.003f && length(b->nodes[0].v) < 0.01f, "a ring tyre under 300 kg: pressed in %.4f m (want %.4f), %.3f m/s", defl, want, length(b->nodes[0].v));
    printf("    300 kg on a ring tyre: pressed in %.1f mm (the patch's %.1f), load %.0f N\n", defl * 1e3f, want * 1e3f, b->wheels[0].load);
    // driven: 400 N m for 2 s
    for (int f = 0; f < 120; f++) {
        for (int k = 0; k < 33; k++) b->wheels[0].torque = 400.0f, w.step_substeps(1);
    }
    const float v = -b->nodes[0].v.x, tread = b->wheels[0].spin * 0.32f; // (spin about +z: it rolls towards -x)
    CHECK(v > 3.0f && std::fabs(tread - v) < 0.1f * v, "driven: the load at %.2f m/s, the tread at %.2f", v, tread);
    printf("    driven 400 N m for 2 s: %.2f m/s, the tread at %.2f m/s (slip %.1f%%)\n", v, tread, 100.0f * (tread - v) / std::max(v, 1e-3f));
    for (int f = 0; f < 180; f++) {
        for (int k = 0; k < 33; k++) b->wheels[0].brake = 3000.0f, w.step_substeps(1);
    }
    CHECK(length(b->nodes[0].v) < 0.05f && std::fabs(b->wheels[0].spin) < 0.05f, "braked: %.3f m/s, spin %.3f rad/s", length(b->nodes[0].v), b->wheels[0].spin);
    printf("    braked 3 kN m: stopped at %.3f m/s\n", length(b->nodes[0].v));
}

// A ring tyre's wheel as the builder makes it (vehicle/builder.cpp): R 0.32, rim 0.2, 0.22 wide, 20 kg, the tyre
// 200 kN/m pressed 2 cm into the flat; on the axle nodes a0 -> a1
static Wheel ring_wheel(uint32_t a0, uint32_t a1) {
    Wheel wh;
    wh.ring = true, wh.type = Wheel::W_RING;
    wh.axle0 = a0, wh.axle1 = a1;
    wh.radius = 0.32f, wh.rim_radius = 0.2f, wh.width = 0.22f, wh.mass = 20;
    wh.inertia = 0.5f * 20 * 0.32f * 0.32f;
    wh.inertia_t = 0.5f * wh.inertia + 20 * 0.22f * 0.22f / 12.0f;
    const float patch = (4.0f / 3.0f) * 0.22f * std::sqrt(2.0f * 0.32f * 0.02f);
    wh.k_area = 2.0e5f / patch, wh.c_area = 1500.0f / patch, wh.k_shear = 1.5f * wh.k_area, wh.c_shear = 2.5f * wh.c_area;
    wh.k_lat = 0.8f * wh.k_area * patch, wh.c_lat = 0.002f * wh.k_lat;
    wh.k_long = wh.k_area * patch, wh.c_long = 0.002f * wh.k_long;
    wh.ref = vec3(0, 1, 0);
    return wh;
}

// A cart: four ring wheels (two axles 1.6 m apart, the wheels 1.1 m apart on each) on a rigid frame (the axle nodes and
// one above them, every pair a beam), `load` kg on each wheel's axle nodes; each wheel's axle turned by `camber` (rad: its
// axle1 end raised - its axle0 side lower)
static SoftBody* ring_cart(World& w, float load, float camber) {
    w.settings.gravity = vec3(0, -9.81f, 0);
    w.statics.terrain.create(41, 81, 1.0f, vec2(-20, -20));
    w.statics.has_terrain = true;
    w.statics.terrain.update_bounds();
    auto body = std::make_unique<SoftBody>();
    body->name = "ring cart";
    body->can_sleep = false;
    const vec3 a(0, std::sin(camber), std::cos(camber));
    std::vector<uint32_t> fr;
    for (float x : {-0.8f, 0.8f})
        for (float z : {-0.55f, 0.55f}) {
            const vec3 c(x, 0.322f, z);
            fr.push_back(body->add_node(c - a * 0.15f, 0.5f * load, NF_NONE));
            fr.push_back(body->add_node(c + a * 0.15f, 0.5f * load, NF_NONE));
        }
    fr.push_back(body->add_node(vec3(0, 0.9f, 0), 40.0f, NF_NONE));
    for (size_t i = 0; i < fr.size(); i++)
        for (size_t j = i + 1; j < fr.size(); j++) body->add_beam(fr[i], fr[j], 1e7f, 2e3f, 1e12f, 1e12f);
    for (int k = 0; k < 4; k++) body->wheels.push_back(ring_wheel(fr[2 * k], fr[2 * k + 1]));
    body->finalize();
    body->seat_wheels();
    return w.add_body(std::move(body));
}

// The ring tyre's wheel a body of its own on its bearings; its sidewalls each side their own, folding over and
// crushed; the tread's strips shifted across the rim by a side load (the tyre bending sideways)
static void test_ring_wheel() {
    printf("ring tyres: the wheel's body, the sidewalls, the side load\n");
    auto side_most = [](const Wheel& wh, int sd, const std::vector<float>& v) {
        float m = 0;
        for (size_t k = sd; k < v.size(); k += 2) m = std::max(m, v[k]);
        return m;
    };
    {
        World w;
        SoftBody* b = ring_cart(w, 200.0f, 0.0f);
        for (int f = 0; f < 120; f++) w.step_substeps(33);
        const Wheel& wh = b->wheels[0];
        const vec3 mid = (b->nodes[wh.axle0].p + b->nodes[wh.axle1].p) * 0.5f;
        const float want = (200.0f + 20.0f + 10.0f) * 9.81f; // (its axle nodes', its own, a quarter of the frame's top)
        CHECK(std::fabs(wh.load - want) < 0.04f * want, "the ground carries %.0f N (want %.0f: the wheel's own 20 kg on it)", wh.load, want);
        CHECK(length(wh.pos - mid) < 1e-3f && length(wh.vel) < 0.01f, "the wheel %.2f mm off its axle's middle, at %.3f m/s", 1e3f * length(wh.pos - mid), length(wh.vel));
        CHECK(std::fabs(b->total_mass() - (8 * 100.0f + 40.0f + 4 * 20.0f)) < 0.01f, "the cart weighs %.1f kg (its wheels with it)", b->total_mass());
        const float l = side_most(wh, 0, wh.side_sq), r = side_most(wh, 1, wh.side_sq);
        CHECK(std::fabs(l - r) < 1e-3f && l > 0.01f, "upright: its sidewalls pressed in %.1f and %.1f mm (alike)", l * 1e3f, r * 1e3f);
        printf("    220 kg on a wheel: %.0f N on the ground, the wheel %.2f mm off its axle, its sidewalls %.1f / %.1f mm\n", wh.load, 1e3f * length(wh.pos - mid), l * 1e3f, r * 1e3f);
        // a side load (0.4 of the weight, along the axles: towards axle1) - the strips shifted on the rims, the sidewall
        // the rim leans over pressed further; it stands, not sliding
        const float Fs = 0.4f * 4 * want;
        const float z0 = b->nodes[0].p.z;
        float drift = 0;
        b->pre_substep = [&](SoftBody& x, float) {
            for (int q = 0; q < 8; q++) x.force[q] += vec3(0, 0, 0.125f * Fs);
        };
        for (int f = 0; f < 150; f++) {
            w.step_substeps(33);
            if (f == 120) drift = b->nodes[0].p.z;
        }
        b->pre_substep = nullptr;
        (void)z0;
        drift = std::fabs(b->nodes[0].p.z - drift);
        const Wheel& wo = b->wheels[1]; // (an outer one: the cart's weight leans on them)
        const float l2 = side_most(wo, 0, wo.side_sq), r2 = side_most(wo, 1, wo.side_sq);
        float shift = 0;
        for (const vec2& S : wo.carcass) shift = std::min(shift, S.y);
        CHECK(shift < -0.002f && shift > -0.05f, "the belt shifted across the rim by %.1f mm (want the tread behind the rim: -2 .. -50 mm)", shift * 1e3f);
        CHECK(r2 > l2 + 0.002f, "the side the rim leans over pressed in %.1f mm, the other %.1f mm", r2 * 1e3f, l2 * 1e3f);
        CHECK(drift < 0.005f, "it slides on at %.1f mm in 0.5 s under %.0f N", drift * 1e3f, Fs);
        printf("    %.0f N across the cart: an outer wheel's belt %.1f mm across its rim, its sidewalls %.1f / %.1f mm, %.1f mm in the last 0.5 s\n", Fs, shift * 1e3f, l2 * 1e3f,
               r2 * 1e3f, drift * 1e3f);
    }
    {
        // cambered 5 deg (its axle0 side lower): that sidewall pressed further and folding over first
        World w;
        SoftBody* b = ring_cart(w, 1800.0f, 5.0f * kPi / 180.0f);
        for (int f = 0; f < 240; f++) w.step_substeps(33);
        const Wheel& wh = b->wheels[0];
        const float l = side_most(wh, 0, wh.side_sq), r = side_most(wh, 1, wh.side_sq), fl = side_most(wh, 0, wh.fold), fr2 = side_most(wh, 1, wh.fold);
        CHECK(l > r + 0.005f, "cambered: the lower sidewall pressed in %.1f mm, the upper %.1f mm", l * 1e3f, r * 1e3f);
        CHECK(fl > 0.5f && fr2 < 0.5f, "cambered under 1.8 t: the lower sidewall folded %.2f, the upper %.2f", fl, fr2);
        CHECK(length(b->nodes[0].v) < 0.02f && length(wh.vel) < 0.02f, "it stands: %.3f m/s, the wheel %.3f m/s", length(b->nodes[0].v), length(wh.vel));
        printf("    1.8 t on a wheel cambered 5 deg: its sidewalls %.1f mm (folded %.2f) / %.1f mm (%.2f)\n", l * 1e3f, fl, r * 1e3f, fr2);
    }
    {
        // overloaded (5 t a wheel): both sidewalls folded, the rim's flange on the ground through them - crushed, held
        World w;
        SoftBody* b = ring_cart(w, 5000.0f, 0.0f);
        for (int f = 0; f < 240; f++) w.step_substeps(33);
        const Wheel& wh = b->wheels[0];
        const float l = side_most(wh, 0, wh.side_sq), fl = side_most(wh, 0, wh.fold), fr2 = side_most(wh, 1, wh.fold);
        const float y = wh.pos.y;
        CHECK(fl > 0.5f && fr2 > 0.5f, "5 t on a wheel: its sidewalls folded %.2f, %.2f", fl, fr2);
        CHECK(wh.pinches > 0 && y > 0.2f && length(wh.vel) < 0.02f, "crushed %d times, the axle %.3f m up (the rim 0.2), at %.3f m/s", wh.pinches, y, length(wh.vel));
        printf("    5 t on a wheel: pressed in %.1f mm, both sidewalls folded, crushed (%d), the axle %.0f mm up\n", l * 1e3f, wh.pinches, y * 1e3f);
    }
    {
        // driven: the work of the drive on the wheels (its torque times their spin on their hubs) covers what the cart
        // gains - its motion and its wheels' spin - nothing from nowhere (the bearings, the belt, the brush)
        World w;
        SoftBody* b = ring_cart(w, 200.0f, 0.0f);
        for (int f = 0; f < 60; f++) w.step_substeps(33);
        auto energy = [&]() {
            double e = 0;
            for (const Node& n : b->nodes) e += 0.5 * n.mass * length2(n.v) + n.mass * 9.81 * n.p.y;
            for (const Wheel& wh : b->wheels) e += 0.5 * wh.mass * length2(wh.vel) + wh.mass * 9.81 * wh.pos.y + 0.5 * dot(wh.omega(), wh.mom);
            return e;
        };
        const double e0 = energy();
        double work = 0;
        const float T = 350.0f, h = 1.0f / (60.0f * 33.0f);
        for (int f = 0; f < 180; f++)
            for (int k = 0; k < 33; k++) {
                for (Wheel& wh : b->wheels) wh.torque = T;
                w.step_substeps(1);
                for (const Wheel& wh : b->wheels) work += (double)T * wh.spin * h;
            }
        const double gain = energy() - e0;
        float v = 0, tread = 0;
        for (const Wheel& wh : b->wheels) v += length(wh.vel) / 4, tread += std::fabs(wh.spin) * wh.radius / 4;
        CHECK(gain < work && gain > 0.5 * work, "driven 3 s: the cart gained %.0f J of the drive's %.0f J", gain, work);
        printf("    driven 4 x %.0f N m for 3 s: %.2f m/s, the tread at %.2f m/s; %.1f kJ gained of the drive's %.1f kJ\n", T, v, tread, gain * 1e-3, work * 1e-3);
    }
    {
        // the body moved: its wheels with it
        World w;
        SoftBody* b = ring_cart(w, 200.0f, 0.0f);
        for (int f = 0; f < 30; f++) w.step_substeps(33);
        b->translate(vec3(4, 0.5f, 0));
        b->set_velocity(vec3(0, 0, 0));
        const Wheel& wh = b->wheels[1];
        const vec3 mid = (b->nodes[wh.axle0].p + b->nodes[wh.axle1].p) * 0.5f;
        CHECK(length(wh.pos - mid) < 1e-4f, "moved: the wheel %.3f mm off its axle", 1e3f * length(wh.pos - mid));
        for (int f = 0; f < 120; f++) w.step_substeps(33);
        CHECK(std::fabs(b->nodes[0].p.x - 3.2f) < 0.01f && length(wh.vel) < 0.05f, "and fell back onto its wheels where it was put: x %.3f (want 3.2), %.3f m/s", b->nodes[0].p.x,
              length(wh.vel));
    }
}

static void test_volumes() {
    printf("one-sided springs, a one-node ball, collision volumes\n");
    const FrameSection tube = frame_tube(2e-4f);
    {
        auto w = std::make_unique<World>();
        w->settings.gravity = vec3(0, -9.81f, 0);
        auto body = std::make_unique<SoftBody>();
        body->name = "hung";
        body->can_sleep = false;
        const uint16_t si = body->fem.add_section(tube);
        const uint32_t f0 = body->add_node(vec3(0, 50, 0), 1.0f, NF_NONE), f1 = body->add_node(vec3(0.5f, 50, 0), 1.0f, NF_NONE),
                       f2 = body->add_node(vec3(0, 50, 0.5f), 1.0f, NF_NONE);
        body->fem.add_element(f0, f1, si), body->fem.add_element(f1, f2, si), body->fem.add_element(f2, f0, si);
        const uint32_t p = body->add_node(vec3(0, 49.5f, 0), 2.0f, NF_NONE);
        body->add_beam(f0, p, 1e5f, 900.0f, 1e12f, 1e12f);
        fix_inv_mass(*body);
        body->fem.finalize(*body);
        SoftBody* b = w->add_body(std::move(body));
        const int steps = 1000;
        for (int k = 0; k < steps; k++) w->step_substeps(1);
        double P = 0, M = 0, pf = 0, mf = 0;
        for (size_t i = 0; i < b->nodes.size(); i++) {
            P += b->nodes[i].mass * b->nodes[i].v.y, M += b->nodes[i].mass;
            if (b->fem.slot((uint32_t)i) >= 0) pf += b->nodes[i].mass * b->nodes[i].v.y, mf += b->nodes[i].mass;
        }
        const double t = steps * w->settings.dt, expect = -9.81 * t;
        CHECK(std::fabs(P / (M * expect) - 1.0) < 0.01, "a frame with a hung node fell at %.3f of g as a whole", P / (M * expect));
        CHECK(std::fabs(pf / (mf * expect) - 1.0) < 0.02, "its frame fell at %.3f of g", pf / (mf * expect));
        printf("    in free fall: the whole at %.4f of g, the frame's nodes at %.4f\n", P / (M * expect), pf / (mf * expect));
    }
    {
        auto w = std::make_unique<World>();
        w->settings.gravity = vec3(0, -9.81f, 0);
        w->statics.terrain.create(21, 21, 1.0f, vec2(-10, -10)); // (flat ground at y = 0)
        w->statics.has_terrain = true;
        w->statics.terrain.update_bounds();
        auto body = std::make_unique<SoftBody>();
        body->name = "ball";
        const uint32_t c = body->add_node(vec3(0, 2, 0), 40.0f, NF_GROUND | NF_CONTACTER);
        body->capsules.push_back({c, c, 0.22f});
        body->sphere_ball = 0.22f, body->bounce = 0.2f, body->collision_radius = 0.01f;
        body->finalize();
        body->info[c].radius = 0.22f;
        SoftBody* b = w->add_body(std::move(body));
        for (int f = 0; f < 90; f++) w->step_substeps(33);
        const float y = b->nodes[0].p.y, v = length(b->nodes[0].v);
        CHECK(std::fabs(y - 0.22f) < 0.02f && v < 0.1f, "a ball of 0.22 m dropped on the ground rests at %.3f m, %.2f m/s", y, v);
        printf("    a ball of 0.22 m dropped from 2 m: at %.3f m, %.3f m/s\n", y, v);
    }
    {
        auto w = std::make_unique<World>();
        w->settings.gravity = vec3(0);
        auto cube = std::make_unique<SoftBody>();
        cube->name = "cube";
        std::vector<uint32_t> an;
        for (int k = 0; k < 8; k++) an.push_back(cube->add_node(vec3((float)(k & 1), 5.0f + (float)((k >> 1) & 1), (float)((k >> 2) & 1)), 10.0f, NF_NONE));
        for (int i = 0; i < 8; i++)
            for (int j = i + 1; j < 8; j++) cube->add_beam(an[i], an[j], 1e6f, 1000.0f, 1e12f, 1e12f);
        cube->finalize();
        std::vector<vec3> hull;
        for (int k = 0; k < 8; k++) hull.push_back(vec3(0.1f + 0.8f * (float)(k & 1), 5.1f + 0.8f * (float)((k >> 1) & 1), 0.1f + 0.8f * (float)((k >> 2) & 1)));
        const int vi = cube->add_volume("box", an, hull, 0.2f);
        CHECK(vi == 0 && cube->volumes[0].planes.size() == 6, "a box's hull: volume %d, %zu faces", vi, vi == 0 ? cube->volumes[0].planes.size() : 0);
        SoftBody* A = w->add_body(std::move(cube));
        auto shot = std::make_unique<SoftBody>();
        shot->name = "shot";
        shot->add_node(vec3(-1.0f, 5.5f, 0.5f), 2.0f, NF_GROUND | NF_CONTACTER);
        shot->finalize();
        shot->nodes[0].v = vec3(5, 0, 0);
        SoftBody* B = w->add_body(std::move(shot));
        float deepest = -1e9f;
        for (int f = 0; f < 36; f++) {
            w->step_substeps(33);
            deepest = std::max(deepest, B->nodes[0].p.x);
        }
        vec3 P(0);
        for (const Node& n : A->nodes) P += n.v * n.mass;
        P += B->nodes[0].v * B->nodes[0].mass;
        CHECK(deepest < 0.15f, "a node thrown at a collision volume went in to x %.3f (its face at 0.1)", deepest);
        CHECK(std::fabs(P.x - 10.0f) < 0.3f && std::fabs(P.y) < 0.3f && std::fabs(P.z) < 0.3f, "the momentum of the node and the cube: (%.2f %.2f %.2f), was (10 0 0)", P.x, P.y, P.z);
        CHECK(A->volumes[0].hits > 0 && !A->volumes[0].broken, "the volume: %d contacts, %s", A->volumes[0].hits, A->volumes[0].broken ? "off" : "on");
        printf("    a 2 kg node at 5 m/s into a box's volume: in to %.3f m past its face, momentum (%.2f %.2f %.2f), %d contacts\n", deepest - 0.1f, P.x, P.y, P.z,
               A->volumes[0].hits);
    }
    // a volume's fit after its body turned at once (a spawn's heading, a reset): half a turn stalled the fit from the last
    for (float deg : {90.0f, 180.0f, 179.0f}) {
        SoftBody body;
        std::vector<uint32_t> an;
        for (int k = 0; k < 8; k++) an.push_back(body.add_node(vec3(2.0f * (float)(k & 1), (float)((k >> 1) & 1), 1.5f * (float)((k >> 2) & 1)), 10.0f, NF_NONE));
        body.finalize();
        std::vector<vec3> hull;
        for (int k = 0; k < 8; k++) hull.push_back(vec3(0.1f + 1.8f * (float)(k & 1), 0.1f + 0.8f * (float)((k >> 1) & 1), 0.1f + 1.3f * (float)((k >> 2) & 1)));
        body.add_volume("box", an, hull, 0.12f);
        body.place_volumes();
        const quat t = quat::axis_angle(vec3(0, 1, 0), deg * kDeg2Rad);
        for (Node& n : body.nodes) n.p = vec3(5, 0, 5) + to_mat3(t) * n.p;
        body.place_volumes();
        const CollisionVolume& cv = body.volumes[0];
        float err = 0;
        for (size_t k = 0; k < hull.size() && k < cv.wverts.size(); k++) err = std::max(err, length(cv.wverts[k] - (vec3(5, 0, 5) + to_mat3(t) * hull[k])));
        CHECK(!cv.broken && cv.placed && err < 1e-3f, "a volume turned %.0f deg at once: %s, its hull %.4f m off", deg, cv.broken ? "off" : "on", err);
    }
    // the body's own parts (the frame's components with triangles on mounts: a hood, a door) held off its volumes, the
    // frame it rides on not: a plate on three soft mounts 10 cm over a wide box volume on a heavy frame, thrown down at
    // 8 m/s - its first blow (the mounts alone let it 20 cm into the box)
    for (int held = 1; held >= 0; held--) {
        auto w = std::make_unique<World>();
        w->settings.gravity = vec3(0);
        auto body = std::make_unique<SoftBody>();
        body->name = "parted";
        body->can_sleep = false;
        const uint16_t si = body->fem.add_section(tube);
        const vec3 fp[4] = {vec3(-0.5f, 1, -0.5f), vec3(1.5f, 1, -0.5f), vec3(-0.5f, 1, 1.5f), vec3(0.5f, 0.2f, 0.5f)};
        uint32_t f[4];
        for (int i = 0; i < 4; i++) f[i] = body->add_node(fp[i], 50.0f, NF_CONTACTER);
        for (int i = 0; i < 4; i++)
            for (int j = i + 1; j < 4; j++) body->fem.add_element(f[i], f[j], si);
        ShellMesher m(*body, NF_CONTACTER);
        const uint16_t ss = body->fem.add_shell_section(make_shell_section("Steel", 0.002f));
        const std::vector<uint32_t> q = m.grid(vec3(0.2f, 1.55f, 0.2f), vec3(0.3f, 0, 0), vec3(0, 0, 0.3f), 2, 2, ss, false);
        m.finish();
        fix_inv_mass(*body);
        body->fem.add_mount(f[0], q[0], 0.0f, 1e3f, 0.0f);
        body->fem.add_mount(f[1], q[2], 0.0f, 1e3f, 0.0f);
        body->fem.add_mount(f[2], q[6], 0.0f, 1e3f, 0.0f);
        body->fem.finalize(*body);
        std::vector<vec3> hull;
        for (int k = 0; k < 8; k++) hull.push_back(vec3(-0.4f + 1.8f * (float)(k & 1), 1.05f + 0.4f * (float)((k >> 1) & 1), -0.4f + 1.8f * (float)((k >> 2) & 1)));
        body->add_volume("box", {f[0], f[1], f[2], f[3]}, hull, 0.2f);
        const int inside = body->find_volume_parts();
        const std::vector<uint32_t> parts = body->volumes[0].parts;
        if (held) {
            bool frame_in = false;
            for (uint32_t i : parts) frame_in |= i < 4;
            CHECK(inside == 0 && parts.size() == q.size() && !frame_in, "the volume's parts: %zu nodes (want the plate's %zu), %d inside, the frame's %s", parts.size(),
                  q.size(), inside, frame_in ? "in" : "not in");
        } else {
            body->volumes[0].parts.clear();
        }
        vec3 P0(0);
        for (uint32_t i : q) body->nodes[i].v = vec3(0, -8, 0);
        for (const Node& n : body->nodes) P0 += n.v * n.mass;
        SoftBody* b = w->add_body(std::move(body));
        float low = 1e9f, vrel = 0; // (the deepest of the plate's nodes into it; the plate's speed to it at the end)
        for (int k = 0; k < 120; k++) {
            w->step_substeps(1);
            int face;
            vrel = 0;
            for (uint32_t i : q) low = std::min(low, b->volumes[0].depth(b->nodes[i].p, face)), vrel += (b->nodes[i].v.y - b->volumes[0].v.y) / (float)q.size();
        }
        vec3 P(0);
        for (const Node& n : b->nodes) P += n.v * n.mass;
        if (held) {
            CHECK(low > -0.03f && vrel > -0.5f && b->volumes[0].hits > 0 && b->fem.solve_failures == 0,
                  "a part thrown at its body's volume went %.3f m into it, at %.2f m/s to it after 60 ms, %d contacts, %d failed solves", -low, vrel, b->volumes[0].hits,
                  b->fem.solve_failures);
            CHECK(length(P - P0) < 0.02f * length(P0), "the parted body's momentum (%.2f %.2f %.2f), was (%.2f %.2f %.2f)", P.x, P.y, P.z, P0.x, P0.y, P0.z);
            printf("    a plate on mounts thrown at 8 m/s at its body's volume: %.3f m into it, %.2f m/s to it after 60 ms, %d contacts, momentum %.3f of it\n", -low, vrel,
                   b->volumes[0].hits, P.y / P0.y);
        } else {
            CHECK(low < -0.1f, "the plate not held off went only %.3f m into it", -low);
            printf("    ... not held off (its nodes out of the volume's parts): %.3f m into it\n", -low);
        }
    }
}

// the FEM plates' mid points (SoftBody::tri_mids): a plate of 2 mm steel (1 x 1 m, three cells along x: six triangles,
// their mid points a sixth of a metre either side of its middle line, its corners half a metre) dropped 0.3 m flat on
// a beam that runs under its middle between its corners, and on two blades (another body's triangles standing on edge
// under its mid points, their corners far out). Held on them; without its mid points it falls through to the ground
static void test_tri_mids() {
    printf("the FEM plates' mid points\n");
    const ShellSection s2 = make_shell_section("Steel", 0.002f);
    for (int blades = 0; blades < 2; blades++)
        for (int on = 1; on >= 0; on--) {
            auto w = std::make_unique<World>();
            w->statics.terrain.create(21, 21, 1.0f, vec2(-10, -10)); // (flat ground at y = 0)
            w->statics.has_terrain = true;
            w->statics.terrain.update_bounds();
            if (!blades) {
                w->statics.add_box(vec3(0, 0.45f, 0), vec3(1.0f, 0.05f, 0.2f), quat(), SURF_METAL);
                w->statics.build_grid();
            } else {
                auto blade = std::make_unique<SoftBody>();
                blade->name = "blades";
                for (float z : {-1.0f / 6, 1.0f / 6}) {
                    const uint32_t a = blade->add_node(vec3(-1, 0.5f, z), 10.0f, NF_FIXED | NF_CONTACTER), b = blade->add_node(vec3(1, 0.5f, z), 10.0f, NF_FIXED | NF_CONTACTER),
                                   c = blade->add_node(vec3(0, -0.5f, z), 10.0f, NF_FIXED | NF_CONTACTER);
                    Triangle t;
                    t.a = a, t.b = b, t.c = c;
                    t.rest_edge2 = 4.0f;
                    blade->tris.push_back(t);
                }
                blade->finalize();
                fix_inv_mass(*blade);
                w->add_body(std::move(blade));
            }
            auto body = std::make_unique<SoftBody>();
            body->name = "plate";
            body->can_sleep = false;
            const uint16_t si = body->fem.add_shell_section(s2);
            ShellMesher m(*body);
            m.grid(vec3(-0.5f, 0.8f, -0.5f), vec3(1.0f / 3, 0, 0), vec3(0, 0, 1), 3, 1, si, true);
            m.finish();
            body->fem.finalize(*body);
            body->tri_mids = on != 0;
            SoftBody* b = w->add_body(std::move(body));
            int touched = 0;
            float mass = 0;
            for (const Node& x : b->nodes) mass += x.mass;
            for (int f = 0; f < 60; f++) {
                w->step_substeps(33);
                touched += b->mid_contacts;
            }
            // (it lands with m g 0.3 and rings on: a thin plate on points; its ringing against that, its bulk's speed)
            float low = 1e9f;
            double ke = 0;
            vec3 vm(0);
            for (const Node& x : b->nodes) low = std::min(low, x.p.y), ke += 0.5 * x.mass * length2(x.v), vm += x.v * (x.mass / mass);
            const double ring = ke / (mass * 9.81 * 0.3);
            const char* on_what = blades ? "two blades (another body's triangles)" : "a beam";
            if (on)
                CHECK(low > 0.45f && ring < 0.01 && length(vm) < 0.1f && touched > 0 && b->fem.solve_failures == 0,
                      "a plate dropped on %s under its mid points: its corners at %.3f m (the top 0.5), %.2f%% of its landing energy left, bulk %.3f m/s, %d mid contacts, %d failed solves",
                      on_what, low, 100 * ring, length(vm), touched, b->fem.solve_failures);
            else
                CHECK(low < 0.1f, "without its mid points a plate fell through %s: its corners at %.3f m", on_what, low);
            printf("    a 1 x 1 m plate dropped 0.3 m on %s under its middle, %s mid points: its corners at %.3f m (the top 0.5), %.2f%% of its landing energy left after 1 s, %d mid contacts\n",
                   on_what, on ? "with" : "without", low, 100 * ring, touched);
        }
}

// a sheet across two supports (the FEM Shells scene's: 2 x 2 m of 3 mm steel, 16 x 16 cells) cut through between them
// by a plane swept from above (the giant axe's, World::laser_cut): the triangles it crosses bisected along it, the sheet
// in two halves of its mass, no triangle lost; the halves swing down over the supports' inner edges and come to rest on
// the ground. Pressed at their middles against an edge its plates stay quiet (each corner held to its own explicit pull,
// its members' stiff one, they shook up to 150 m/s in a dozen steps and flew apart)
static void test_sheet_cut_hinge() {
    printf("a sheet on two supports cut in two between them\n");
    auto w = std::make_unique<World>();
    w->statics.terrain.create(21, 21, 1.0f, vec2(-10, -10)); // (flat ground at y = 0)
    w->statics.has_terrain = true;
    w->statics.terrain.update_bounds();
    for (float x : {-0.85f, 0.85f}) w->statics.add_box(vec3(x, 0.4f, 0), vec3(0.12f, 0.4f, 1.3f), quat(), SURF_CONCRETE);
    w->statics.build_grid();
    auto body = std::make_unique<SoftBody>();
    body->name = "sheet";
    body->collision_radius = 0.01f;
    const uint16_t si = body->fem.add_shell_section(make_shell_section("Steel", 0.003f));
    ShellMesher m(*body);
    m.grid(vec3(-1, 0.82f, -1), vec3(2.0f / 16, 0, 0), vec3(0, 0, 2.0f / 16), 16, 16, si);
    m.finish();
    body->fem.finalize(*body);
    SoftBody* b = w->add_body(std::move(body));
    for (int f = 0; f < 15; f++) w->step_substeps(33);
    double area0 = 0;
    for (const FrameTri& t : b->fem.tris) area0 += t.area0;
    const size_t tris0 = b->fem.tris.size();
    // (off the grid's middle line by 4 cm: through the triangles, not along their edges)
    const vec3 o(0.04f, 8.0f, 0);
    const int cut = w->laser_cut(o, normalize(vec3(0, -8, -1.3f)), normalize(vec3(0, -8, 1.3f)), 10.0f);
    float p0, p1;
    b->largest_parts(p0, p1);
    float fastest = 0, lowest = 1e9f;
    for (int f = 0; f < 240; f++) { // (4 s)
        w->step_substeps(33);
        for (const Node& x : b->nodes) fastest = std::max(fastest, length(x.v)), lowest = std::min(lowest, x.p.y);
    }
    // (at the end: its motion's rms speed, the mass's - a light node on the ground may still tremble)
    double mv2 = 0, mass = 0;
    for (const Node& x : b->nodes) mv2 += x.mass * length2(x.v), mass += x.mass;
    const float rest = (float)std::sqrt(mv2 / std::max(mass, 1e-9));
    double area = 0;
    for (const FrameTri& t : b->fem.tris) area += t.broken ? 0.0 : (double)t.area0;
    for (const FemFrame::LooseTri& t : b->fem.loose_tris) area += t.area0;
    float q0, q1;
    b->largest_parts(q0, q1);
    printf("    %d links cut, %zu -> %zu triangles (%d bisections), parts %.0f%% %.0f%% (after 4 s %.0f%% %.0f%%), area kept %.6f; fastest %.2f m/s, lowest %.3f m, "
           "%.3f m/s rms at the end, %d failed solves\n",
           cut, tris0, b->fem.tris.size(), b->fem.tris_refined, 100 * p0, 100 * p1, 100 * q0, 100 * q1, area / area0, fastest, lowest, rest, b->fem.solve_failures);
    CHECK(cut > 0 && b->fem.tris_refined >= 16 && p0 < 0.6f && p1 > 0.4f && std::fabs(area / area0 - 1) < 1e-5,
          "the cut: %d links, %d bisections, parts %.2f %.2f, area kept %.6f", cut, b->fem.tris_refined, p0, p1, area / area0);
    // (falling 0.8 m into the gap they land at 4-5 m/s, edge first: a light node on the cut bounces off at about three
    // times that; flying apart, as they did, at 150)
    CHECK(fastest < 20.0f && lowest > -0.03f && rest < 0.05f && q1 > 0.4f && b->fem.solve_failures == 0,
          "the halves falling: fastest %.2f m/s, lowest %.3f m, %.3f m/s rms at the end, parts %.2f %.2f, %d failed solves", fastest, lowest, rest, q0, q1,
          b->fem.solve_failures);
}

// a sheet's triangles against another body's collision volume (SoftBody::tri_mids: the triangle clipped by the hull):
// a 1 x 1 m sheet of two triangles (8 kg/m2 of steel) thrown flat at 3 m/s at a bar (a volume on a heavy body) that
// runs under it between its corners - under its middle (0.4 m wide), or two off its triangles' middles (0.2 m wide,
// either side of it) - it stops on it; without it goes through
// A fragment torn off lies still while its body drives away: the RoR air drag is relative to each part's mean velocity
// (against the body's mean it pulled the fragment after the car) and the loose shards' cap leaves a slow one alone.
static void test_loose_drag() {
    printf("a torn-off fragment against its driving body\n");
    SoftBody b;
    b.name = "car and fragment";
    b.can_sleep = false;
    b.air_drag = 0.05f, b.aero_cda = 0.7f;
    // the car: a 400 kg rod at 25 m/s; the fragment: a 3-node shard of 60 g at rest, 5 m off
    const uint32_t c0 = b.add_node(vec3(0, 1, 0), 200.0f, NF_NONE), c1 = b.add_node(vec3(0, 1, 2), 200.0f, NF_NONE);
    b.add_beam(c0, c1, 1e6f, 1e3f, 1e12f, 1e12f);
    const uint32_t f0 = b.add_node(vec3(5, 0, 0), 0.02f, NF_NONE), f1 = b.add_node(vec3(5.1f, 0, 0), 0.02f, NF_NONE),
                   f2 = b.add_node(vec3(5, 0, 0.1f), 0.02f, NF_NONE);
    b.add_beam(f0, f1, 1e3f, 1.0f, 1e12f, 1e12f), b.add_beam(f1, f2, 1e3f, 1.0f, 1e12f, 1e12f), b.add_beam(f2, f0, 1e3f, 1.0f, 1e12f, 1e12f);
    b.finalize();
    b.nodes[c0].v = b.nodes[c1].v = vec3(0, 0, 25);
    int np = 0;
    b.part_labels(&np);
    CHECK(np == 2, "%d parts (want the car and the fragment)", np);
    b.clear_forces(vec3(0));
    float worst = 0;
    for (uint32_t q : {f0, f1, f2}) worst = std::max(worst, length(b.force[q]));
    CHECK(worst < 1e-6f, "the fragment at rest pulled by %.3g N", worst);
    const float fc = length(b.force[c0] + b.force[c1]), want = 0.5f * 1.225f * 0.7f * 25 * 25 * (400.0f / 400.06f);
    CHECK(std::fabs(fc - want) < 0.01f * want, "the car's drag %.1f N (want %.1f, its share)", fc, want);
    // the shard cap: a car at 60 m/s leaves the shard lying, a shard flung at 120 m/s is caught at 50 against the car
    FemFrame& fr = b.fem;
    FemFrame::LooseTri lt{};
    lt.n[0] = f0, lt.n[1] = f1, lt.n[2] = f2;
    fr.loose_tris.push_back(lt);
    b.nodes[c0].v = b.nodes[c1].v = vec3(0, 0, 60);
    fr.cap_loose(b);
    CHECK(length(b.nodes[f0].v) < 1e-6f, "the shard at rest dragged to %.2f m/s after a 60 m/s car", length(b.nodes[f0].v));
    b.nodes[c0].v = b.nodes[c1].v = vec3(0);
    b.nodes[f1].v = vec3(120, 0, 0);
    fr.cap_loose(b);
    CHECK(std::fabs(length(b.nodes[f1].v) - FemFrame::kLooseCap) < 0.5f, "the flung shard node at %.1f m/s (want the cap %.0f)", length(b.nodes[f1].v), FemFrame::kLooseCap);
    printf("    the fragment's pull %.2g N (a 25 m/s car's drag %.0f N); a 120 m/s shard kept to %.1f m/s\n", worst, fc, length(b.nodes[f1].v));
}

static void test_sheet_mids() {
    printf("a sheet's triangles against a collision volume\n");
    for (int on = 2; on >= 0; on--) {
        auto w = std::make_unique<World>();
        w->settings.gravity = vec3(0);
        auto bar = std::make_unique<SoftBody>();
        bar->name = "bar";
        bar->can_sleep = false;
        const std::vector<std::pair<float, float>> zs = on == 2 ? std::vector<std::pair<float, float>>{{0.25f, 0.45f}, {-0.45f, -0.25f}}
                                                                : std::vector<std::pair<float, float>>{{-0.2f, 0.2f}};
        for (auto [z0, z1] : zs) {
            std::vector<uint32_t> an;
            for (int k = 0; k < 8; k++)
                an.push_back(bar->add_node(vec3(-1.0f + 2.0f * (float)(k & 1), 0.1f + 0.3f * (float)((k >> 1) & 1), z0 + (z1 - z0) * (float)((k >> 2) & 1)), 500.0f, NF_NONE));
            for (int i = 0; i < 8; i++)
                for (int j = i + 1; j < 8; j++) bar->add_beam(an[i], an[j], 1e8f, 1e4f, 1e12f, 1e12f);
        }
        bar->finalize();
        for (size_t k = 0; k < zs.size(); k++) {
            std::vector<uint32_t> an;
            std::vector<vec3> hull;
            for (int i = 0; i < 8; i++) {
                an.push_back((uint32_t)(k * 8 + i));
                hull.push_back(vec3(-1.0f + 2.0f * (float)(i & 1), 0.1f + 0.4f * (float)((i >> 1) & 1), zs[k].first + (zs[k].second - zs[k].first) * (float)((i >> 2) & 1)));
            }
            bar->add_volume("bar", an, hull, 0.2f);
        }
        w->add_body(std::move(bar));
        auto sheet = std::make_unique<SoftBody>();
        sheet->name = "sheet";
        sheet->can_sleep = false;
        uint32_t n[4];
        for (int k = 0; k < 4; k++) n[k] = sheet->add_node(vec3(-0.5f + (float)(k & 1), 1.0f, -0.5f + (float)(k >> 1)), 0.0f);
        sheet->add_shell(n[0], n[2], n[3], vec2(0, 0), vec2(0, 1), vec2(1, 1));
        sheet->add_shell(n[0], n[3], n[1], vec2(0, 0), vec2(1, 1), vec2(1, 0));
        sheet->shell_mat = mat_steel();
        sheet->tri_mids = on != 0;
        sheet->finalize();
        sheet->finalize_shells(8.0f, kDefaultDt, 3, true);
        for (Node& x : sheet->nodes) x.v = vec3(0, -3.0f, 0);
        SoftBody* s = w->add_body(std::move(sheet));
        float low = 1e9f;
        int touched = 0;
        for (int f = 0; f < 30; f++) {
            w->step_substeps(33);
            touched += s->mid_contacts;
            for (const Node& x : s->nodes) low = std::min(low, x.p.y);
        }
        const char* where = on == 2 ? "(two) off its triangles' middles" : "under its middle";
        if (on)
            CHECK(low > 0.45f && touched > 0, "a sheet thrown at a bar %s went down to %.3f m (the bar's top 0.5), %d contacts", where, low, touched);
        else
            CHECK(low < 0.2f, "without its triangles a sheet went through the bar under its middle: down to %.3f m", low);
        printf("    a 1 x 1 m sheet (two triangles) thrown at 3 m/s at a bar %s, %s its triangles: its corners down to %.3f m (the bar's top 0.5)\n", where,
               on ? "with" : "without", low);
    }
}

// ---- FEM frame benchmark (`test_physics fembench [steps]`): the frame's cost per substep against its members, for
// three kinds of structure and one split into parts: a planar truss (a ladder with diagonals: little fill), a car-like
// cage (square rings of four tubes, longitudinals, the faces' diagonals), a cubic lattice (the most fill), and the cage
// cut into 1-8 components (parts on mounts). Free in space, no gravity, a spin to keep the members working; one line of
// CSV per case: kind, members, nodes, components, factor blocks, block updates, ms per substep (all threads, one thread),
// ms for the members' forces and the solve (one thread)
namespace {
std::unique_ptr<SoftBody> fem_structure(int kind, int n, int parts) {
    auto b = std::make_unique<SoftBody>();
    b->name = "fembench";
    b->can_sleep = false;
    const uint16_t si = b->fem.add_section(frame_tube(2e-4f));
    auto node = [&](vec3 p) { return b->add_node(p, 1.0f, NF_NONE); };
    auto mem = [&](uint32_t a, uint32_t c) { b->fem.add_element(a, c, si); };
    if (kind == 0) { // ladder: n bays of 0.25 m, 0.5 m wide
        std::vector<uint32_t> l, r;
        for (int i = 0; i <= n; i++) l.push_back(node(vec3(0.25f * i, 1, 0))), r.push_back(node(vec3(0.25f * i, 1, 0.5f)));
        for (int i = 0; i <= n; i++) {
            mem(l[i], r[i]);
            if (i < n) mem(l[i], l[i + 1]), mem(r[i], r[i + 1]), mem(l[i], r[i + 1]);
        }
    } else if (kind == 1) { // cage: n bays of 0.3 m, rings of 4 (0.6 x 0.6), the faces' diagonals; `parts` pieces along it
        const int per = std::max(1, n / parts);
        std::vector<std::array<uint32_t, 4>> ring;
        for (int i = 0; i <= n + parts; i++) {
            std::array<uint32_t, 4> rr;
            for (int c = 0; c < 4; c++) rr[c] = node(vec3(0.3f * i, 1 + 0.6f * (c == 1 || c == 2), 0.6f * (c >= 2)));
            ring.push_back(rr);
        }
        for (int p = 0, i0 = 0; p < parts; p++) {
            const int i1 = p == parts - 1 ? n + parts : i0 + per;
            for (int i = i0; i <= i1; i++)
                for (int c = 0; c < 4; c++) {
                    mem(ring[i][c], ring[i][(c + 1) % 4]);
                    if (i < i1) mem(ring[i][c], ring[i + 1][c]), mem(ring[i][c], ring[i + 1][(c + 1) % 4]);
                }
            i0 = i1 + 1;
        }
    } else { // cubic lattice n x n x n, 0.3 m
        std::vector<uint32_t> id(n * n * n);
        for (int x = 0; x < n; x++)
            for (int y = 0; y < n; y++)
                for (int z = 0; z < n; z++) id[(x * n + y) * n + z] = node(vec3(0.3f * x, 1 + 0.3f * y, 0.3f * z));
        for (int x = 0; x < n; x++)
            for (int y = 0; y < n; y++)
                for (int z = 0; z < n; z++) {
                    const uint32_t a = id[(x * n + y) * n + z];
                    if (x + 1 < n) mem(a, id[((x + 1) * n + y) * n + z]);
                    if (y + 1 < n) mem(a, id[(x * n + y + 1) * n + z]);
                    if (z + 1 < n) mem(a, id[(x * n + y) * n + z + 1]);
                }
    }
    fix_inv_mass(*b);
    b->fem.finalize(*b);
    for (Node& x : b->nodes) x.v = cross(vec3(0.3f, 1.0f, 0.2f), x.p - vec3(0, 1, 0)); // (a spin)
    return b;
}

void fem_bench(int steps) {
    printf("kind,members,nodes,components,blocks,updates,ms_mt,ms_st,ms_forces_st,ms_solve_st\n");
    struct Case {
        const char* name;
        int kind, n, parts;
    };
    std::vector<Case> cases;
    for (int p : {1, 2, 4, 8, 13}) cases.push_back({"cage_parts", 1, 128, p});   // (first: the lattice's long runs heat the machine)
    for (int n : {10, 25, 50, 100, 200, 400, 800}) cases.push_back({"ladder", 0, n, 1});
    for (int n : {4, 8, 16, 32, 64, 128, 250}) cases.push_back({"cage", 1, n, 1});
    for (int n : {3, 4, 5, 6, 7, 8, 9, 10}) cases.push_back({"lattice", 2, n, 1});
    for (const Case& c : cases) {
        double ms[2] = {0, 0}, fst = 0, sst = 0;
        size_t members = 0, nodes = 0, blocks = 0, updates = 0;
        int comps = 0;
        for (int mt = 1; mt >= 0; mt--) {
            World w;
            w.settings.gravity = vec3(0);
            w.settings.multithreaded = mt != 0;
            SoftBody* b = w.add_body(fem_structure(c.kind, c.n, c.parts));
            for (int i = 0; i < 20; i++) w.step_substeps(1);
            ms[mt] = 1e9;
            for (int rep = 0; rep < 3; rep++) { // (the least of three: the machine's other work off it)
                const auto t0 = std::chrono::steady_clock::now();
                for (int i = 0; i < steps; i++) w.step_substeps(1);
                ms[mt] = std::min(ms[mt], std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / steps);
            }
            members = b->fem.elems.size(), nodes = b->fem.node.size(), blocks = b->fem.factor_blocks(), updates = b->fem.factor_updates(), comps = b->fem.components();
            if (!mt) { // (the frame's own calls, one thread)
                fst = sst = 1e9;
                for (int rep = 0; rep < 3; rep++) {
                    const auto t1 = std::chrono::steady_clock::now();
                    for (int i = 0; i < steps; i++) b->clear_forces(vec3(0)), b->fem.compute_forces(*b);
                    const auto t2 = std::chrono::steady_clock::now();
                    for (int i = 0; i < steps; i++) b->fem.solve(*b, kDefaultDt, kDefaultDt, w.settings.frame_theta, w.settings.frame_dissipation);
                    const auto t3 = std::chrono::steady_clock::now();
                    fst = std::min(fst, std::chrono::duration<double, std::milli>(t2 - t1).count() / steps);
                    sst = std::min(sst, std::chrono::duration<double, std::milli>(t3 - t2).count() / steps);
                }
            }
        }
        printf("%s,%zu,%zu,%d,%zu,%zu,%.4f,%.4f,%.4f,%.4f\n", c.name, members, nodes, comps, blocks, updates, ms[1], ms[0], fst, sst);
    }
}
} // namespace

int main(int argc, char** argv) {
    JobSystem::get().init(std::max(1, JobSystem::performance_cores() - 1)); // (as the application: performance cores only)
    if (argc > 1 && std::strcmp(argv[1], "fembench") == 0) {
        setvbuf(stdout, nullptr, _IOLBF, 0);
        fem_bench(argc > 2 ? atoi(argv[2]) : 300);
        JobSystem::get().shutdown();
        return 0;
    }
    if (argc > 1 && std::strcmp(argv[1], "team") == 0) {
        team_bench();
        JobSystem::get().shutdown();
        return 0;
    }
    if (argc > 1 && std::strcmp(argv[1], "kernel") == 0) {
        kernel_loop(argc > 2 ? atof(argv[2]) : 3.0);
        JobSystem::get().shutdown();
        return 0;
    }
    const bool do_bench = argc > 1 && std::strcmp(argv[1], "bench") == 0;
    setvbuf(stdout, nullptr, _IOLBF, 0); // (lines in order with the warnings on stderr)
    // `only <name>`: one section (kernel, momentum, topology, reorder, repeat, shapes, shape_impacts, patterns, laser,
    // stability, world, membrane, frame, hull)
    const char* only = argc > 2 && std::strcmp(argv[1], "only") == 0 ? argv[2] : nullptr;
    auto run = [&](const char* name, void (*f)()) {
        if (!only || std::strcmp(only, name) == 0) f();
    };
    run("kernel", test_kernel_equivalence);
    run("momentum", test_momentum);
    run("topology", test_topology);
    run("reorder", test_reorder);
    run("repeat", test_repeat_impacts);
    run("shapes", test_shapes);
    run("shape_impacts", test_shape_impacts);
    run("patterns", test_patterns);
    run("coarsen", test_coarsen);
    run("rigid", test_rigid);
    run("laser", test_laser);
    run("stability", test_stability);
    run("world", test_world);
    run("membrane", test_membrane_damp);
    run("frame", test_frame);
    run("fem_tri", test_fem_tris);
    run("hull", test_hull);
    run("volumes", test_volumes);
    run("ring", test_ring_tyre);
    run("ring_wheel", test_ring_wheel);
    run("mids", test_tri_mids);
    run("sheet_cut", test_sheet_cut_hinge);
    run("sheet_mids", test_sheet_mids);
    run("loose_drag", test_loose_drag);
    if (do_bench) bench();
    printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
    JobSystem::get().shutdown();
    return g_fail == 0 ? 0 : 1;
}
