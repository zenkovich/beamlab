// The scenes' builders and what they share (src/world/scenes*.cpp, scene_test_site.cpp).
#pragma once
#include "game/game.h"
#include "phys/fem_shell.h"
#include "world/objects.h"

#include <string>
#include <vector>

namespace bl {

using namespace phys; // (as every scene file does)

// ---- scenes
void scene_proving_ground(Game& g);
void scene_test_site(Game& g);
void scene_forest(Game& g);
void scene_canyon(Game& g);
void scene_offroad(Game& g);
void scene_crash(Game& g);
void scene_vehicle_crash(Game& g);
void scene_rally(Game& g);
void scene_tape_maze(Game& g);
void scene_rbr_stage(Game& g, const std::string& id);
void scene_lab(Game& g);
void scene_materials(Game& g);
void scene_sheet_run(Game& g);
void scene_sheet_shapes(Game& g);
void scene_fem_shells(Game& g);
void scene_barrels(Game& g);
void scene_stress_barrels(Game& g);
void scene_editor(Game& g);
void scene_sheet_car(Game& g);
void scene_frame_car(Game& g);
void scene_buggy(Game& g);
void scene_shell_car(Game& g);
void scene_shell_car_m3(Game& g);
void scene_frame_car_test(Game& g, const char* action, vec3 eye, vec3 target, const char* hint, void (*scene)(Game&) = scene_frame_car);
void scene_stress_vehicles(Game& g);
void scene_stress_crates(Game& g);
void scene_stress_forest(Game& g);
void scene_stress_derby(Game& g);
void scene_stress_bridge(Game& g);

// ---- shared pieces (scenes_common.cpp)
quat yaw_q(float deg);
void add_cone_line(Game& g, vec3 a, vec3 b, int n);
void add_ramp(Game& g, vec3 foot, float yaw_deg, float length, float width, float height, MaterialPtr mat, uint8_t surf = phys::SURF_CONCRETE);
std::vector<vec2> ellipse(vec2 c, vec2 r, int n, float phase = 0);
void scatter_trees(Game& g, vec2 center, float radius, int count, uint32_t seed, float road_clear, const std::vector<vec2>& road, float scale = 1.0f, bool bushes = true);
void flat_arena(Game& g, float half, uint8_t surf = phys::SURF_CONCRETE, bool walls = true);

// ---- the lab's materials (scenes_lab.cpp)
// ------------------------------------------------------------------------------------------- materials lab
// The same tests for sheets, shapes and bars of different materials: every material is a stiffness (clamped by the
// explicit stability budget of its nodes) plus yield and fracture strains, ductile or brittle.
struct MatSpec {
    const char* name;
    vec4 color;
    float spec, gloss;
    bool glass;          // transparent
    int tex;             // 0 none, 1 wood, 2 fabric, 3 concrete
    float density;       // kg/m3 (shapes, bars)
    float sheet_kg_m2;   // areal mass of the sheets
    // sheets and solids get their own numbers: the stiffness a node can carry is capped by the explicit stability
    // budget (a light sheet is far softer than the real material), so the yield strains are tuned per form
    float k, yield, brk;              // sheets: target stiffness (N/m), yield / fracture strain (>= 1000: never)
    float bulk_k, bulk_yield, bulk_brk; // solids and bars
    float damp;
    bool ductile;        // metals: stretch instead of tearing
    float flaw;          // brittle: random spread of the strength (+-), cracks run through the weak links: chunks, not dust
};
const MatSpec& mat_spec(const char* name);
MaterialPtr mat_visual(const MatSpec& m);
phys::ShellMaterial sheet_material(const std::string& name);
MaterialPtr fabric_visual();

// The giant axe (build_axe): each frame the sector its edge swept about the pivot (a plane across what it swings
// through: the swing's own) cuts what it crosses (World::laser_cut), out to its edge's reach - a little ahead of the
// blade, to where the edge will be in a frame and a half, so the edge meets what it cut. The rest is the collisions:
// its wedge of a blade, 12 cm at the back, pushes the cut faces apart and is slowed by them, as the halves' falling
// is by it.
struct AxeSwing {
    DynamicObject* obj = nullptr;
    uint32_t edge = 0, edge2 = 0;       // (its edge's middle, the two sides of the blade)
    vec3 pivot{0};
    vec3 axis{0, 0, 1}, ahead{1, 0, 0}; // (its swing's axis - the cut's normal -, the way it swings through the bottom)
    vec3 dir{0};                        // (the edge's direction from the pivot last frame: the sector it swept is cut)
    float reach = 0;                    // (the pivot to the edge's lower end)
    float thick = 0;                    // (the blade's at its back)
    float blade_h = 0;                  // (its leading face's length, along the handle)
    float r0 = 0;                       // (the edge's distance from the pivot as it was let go: its stretch since)
    float angle = 0, most = -180;       // (its edge's angle from straight down, towards `ahead`, degrees; the most so far)
    int cuts = 0;                       // (the links cut so far)
    bool caught = false;                // (a ratchet at its pivot holds it at the top of its swing through: swinging back,
                                        // its blunt back hit the halves it had cut and threw them off)
    void make(Game& g, const AxeDesc& d, const std::string& name) {
        pivot = d.pivot;
        reach = std::sqrt(0.25f * d.blade_w * d.blade_w + d.length * d.length), thick = d.thick, blade_h = d.blade_h;
        axis = vec3(std::sin(d.yaw), 0, std::cos(d.yaw)), ahead = vec3(std::cos(d.yaw), 0, -std::sin(d.yaw));
        obj = g.add_object(build_axe(g.world, d, name, &edge, &edge2));
    }
    // this frame's cut (the links cut)
    int step(Game& g, float dt) {
        if (!obj || !obj->body) return 0;
        const SoftBody& ab = *obj->body;
        const vec3 e = ab.nodes[edge].p - pivot, v = ab.nodes[edge].v;
        const float r = length(e);
        angle = std::atan2(dot(e, ahead), -e.y) * 57.2958f, most = std::max(most, angle);
        if (caught) return 0;
        if (most > 20.0f && angle < most - 0.5f) { // (through and at the top of its swing: held there)
            caught = true;
            for (Node& x : obj->body->nodes) x.v = vec3(0), x.inv_mass = 0;
            return 0;
        }
        // (cutting as long as it moves at all: stopped below 2 m/s, it stuck in the car for good)
        if (r < 1e-3f || length(v) < 0.05f) return 0;
        // (the edge's way this frame and 5 cm: what it meets it cuts there, in its collisions - half a metre ahead the
        // car was cut before the blade came, and it went through as a laser)
        const float vl = length(v), lead = vl * std::max(dt, 1.0f / 60.0f) + 0.05f;
        // (in its swing's plane, across the pivot's axis: the cut's plane is that one - from two directions nearly the
        // same and a little out of it, a slow swing's plane tilted a metre off at the car)
        auto flat = [this](vec3 x) { return normalize_or(x - axis * dot(x, axis), vec3(0, -1, 0)); };
        const vec3 d0 = flat(e), d1 = flat(e + v * (lead / vl));
        const vec3 from = length2(dir) > 0 && dot(dir, d0) < 0.99999f ? dir : d0;
        if (r0 <= 0) r0 = r;
        // (through the blade's edge where it is now, swayed off its swing's plane a few millimetres: the kerf its cut
        // leaves about the plane - 2 mm either side - on its middle, not one side's under its face)
        const vec3 o = pivot + axis * dot((ab.nodes[edge].p + ab.nodes[edge2].p) * 0.5f - pivot, axis);
        const int n = g.world.laser_cut(o, from, d1, reach * r / r0 + 0.02f, &ab, 0.5f * thick); // (to its edge, as far
        // as it stretches: what lies under it it does not reach)
        if (getenv("BL_AXEDBG"))
            printf("axe: edge (%.2f %.2f %.2f) v %.1f from (%.3f %.3f %.3f) to (%.3f %.3f %.3f), %d cut\n", ab.nodes[edge].p.x, ab.nodes[edge].p.y, ab.nodes[edge].p.z, vl,
                   from.x, from.y, from.z, d1.x, d1.y, d1.z, n);
        dir = d1;
        cuts += n;
        if (n > 0) work(g, n, vl);
        return n;
    }
    // The blow: what the blade meets - the nodes round its edge in what it cuts - is struck on to a third of the edge's
    // speed along its way (an inelastic blow on the material it has to cut through: none faster after it, a light sheet's
    // nodes struck again each frame flew off), the momentum that takes off its turn, and each link cut takes kCutWork
    // of its energy besides (the material's tearing). (Cut for nothing, it went through a car as a laser at the free
    // swing's speed, the car unmoved.)
    static constexpr float kCutWork = 100.0f;  // J a link
    static constexpr float kCarry = 0.33f;     // (of the edge's speed: what it strikes on)
    static constexpr float kPushReach = 0.35f; // m round its edge: the nodes the blow takes
    void work(Game& g, int n, float vl) {
        SoftBody& ab = *obj->body;
        const vec3 ee = ab.nodes[edge].p - pivot, er = ee - axis * dot(ee, axis), ve = ab.nodes[edge].v;
        const float arm = length(er), ves = length(ve);
        if (arm < 0.1f || ves < 0.05f) return;
        const vec3 way = ve / ves;
        // (its moment of inertia about the pivot's axis, its turn's rate)
        double I = 0, Lw = 0, ke = 0;
        for (const Node& x : ab.nodes)
            if (x.inv_mass > 0) {
                const vec3 r = x.p - pivot, rp = r - axis * dot(r, axis);
                I += x.mass * length2(rp), Lw += x.mass * dot(cross(r, x.v), axis), ke += 0.5 * x.mass * length2(x.v);
            }
        if (!(I > 0) || !(ke > 1.0)) return;
        // (the nodes round its edge - the leading face's line from its corner up the blade - in what it cut, struck on)
        const vec3 u = er / arm, c0 = pivot + u * (reach - blade_h), c1 = pivot + u * reach;
        double J = 0;
        for (const auto& bp : g.world.bodies()) {
            SoftBody& b = *bp;
            if (&b == &ab) continue;
            AABB box = b.aabb;
            box.expand(kPushReach);
            if (!box.contains(ab.nodes[edge].p)) continue;
            bool struck = false;
            for (Node& x : b.nodes) {
                if (x.inv_mass <= 0) continue;
                const float t = std::clamp(dot(x.p - c0, c1 - c0) / std::max(1e-6f, length2(c1 - c0)), 0.0f, 1.0f);
                if (length2(x.p - (c0 + (c1 - c0) * t)) >= kPushReach * kPushReach) continue;
                const float dv = kCarry * vl - dot(x.v, way);
                if (dv > 0) x.v += way * dv, J += (double)x.mass * dv, struck = true;
            }
            if (struck) b.wake();
        }
        // (its turn slowed by the blow's angular impulse and by the cut's work, no more than a third a frame)
        const double w0 = Lw / I, w1 = w0 - J * arm / I * (w0 >= 0 ? 1.0 : -1.0);
        double f = std::fabs(w0) > 1e-6 ? std::max(0.0, w1 / w0) : 1.0;
        f *= std::sqrt(std::max(0.0, 1.0 - std::min((double)n * kCutWork, 0.3 * ke) / std::max(1e-9, ke * f * f)));
        f = std::max(f, 0.67);
        for (Node& x : ab.nodes)
            if (x.inv_mass > 0) x.v *= (float)f;
        pushed += (float)J;
    }
    float pushed = 0; // (the momentum it gave what it cut, N s)
    std::string status() const {
        float v = 0, low = 1e9f;
        if (obj && obj->body) {
            v = length(obj->body->nodes[edge].v);
            for (const Node& x : obj->body->nodes) low = std::min(low, x.p.y);
        }
        return format("axe at %.0f deg (most %.0f%s), edge %.1f m/s, %d cut, %.0f N s given, lowest %.3f", angle, most, caught ? ", caught" : "", v, cuts, pushed, low);
    }
};

} // namespace bl
