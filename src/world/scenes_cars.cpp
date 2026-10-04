// The FEM cars' scenes: the Sheet Car, the pad of the Frame Car, the Buggy and the Shell Car with its tests.
#include "core/util.h"
#include "game/game.h"
#include "phys/fem_shell.h"
#include "vehicle/vehicle.h"
#include "world/ai.h"
#include "world/scenes_internal.h"
#include "world/stage.h"

#include <algorithm>
#include <cmath>

namespace bl {

using namespace phys;
namespace te = terrain_edit;


void apply_vehicle_sheet_body(Vehicle* v, bool player) {
    const std::string& spec = v->def().cab_material;
    if (spec.rfind("sheet/", 0) != 0 || getenv("BL_NOSHEETBODY")) return; // (BL_NOSHEETBODY=1: the plain frame, for diagnostics)
    std::vector<std::string> f;                                                // (':' is a separator of the truck format)
    for (size_t a = 0; a <= spec.size();) {
        size_t b = spec.find('/', a);
        if (b == std::string::npos) b = spec.size();
        f.push_back(spec.substr(a, b - a));
        a = b + 1;
    }
    const std::string name = f.size() > 1 && !f[1].empty() ? f[1] : "Steel";
    const float kg_m2 = f.size() > 2 ? (float)atof(f[2].c_str()) : 15.7f;      // (2 mm steel)
    const float thick = f.size() > 3 ? (float)atof(f[3].c_str()) : 0.006f;     // (drawn thicker than it is)
    const int max_level = f.size() > 4 && !f[4].empty() ? atoi(f[4].c_str()) : -1; // (refinement depth, -1: the body's default)
    // a body's sheet of a material: metals more ductile than the lab's plate of the same name (they yield early and
    // keep the dent, a long plastic range before they tear, refine only for a real crumple, no finer than 3 levels:
    // level 4 tore in a shower of small pieces); glass, plastics, wood keep their own brittleness (a window shatters)
    auto body_material = [&](const std::string& mname, int level) {
        phys::ShellMaterial m = sheet_material(mname);
        m.pattern = phys::ShellPattern::None; // (the pattern's material plane is flat: not for a car body)
        m.max_level = level >= 0 ? level : 3;
        if (mname == "Steel" || mname == "Aluminium" || mname == "Lead") {
            m.min_piece = 30; // (a crumpled body sheds a few big panels, not shards)
            m.yield = std::min(m.yield, 0.006f), m.brk = std::max(m.brk, 0.7f), m.refine = 0.35f, m.refine_yield = 0.06f;
            m.bend_yield = std::min(m.bend_yield, 0.15f), m.refine_angle = 0.6f;
        } else {
            m.min_piece = std::max(m.min_piece, 12);
        }
        // (a car's glass is set in its frame: the frame's twist bends it, a windscreen driven at full lock in place
        // cracked; it takes three times the plate's bend before it breaks, still shatters when hit)
        if (mname == "Glass") m.bend_break *= getenv("BL_GLASS_BEND") ? (float)atof(getenv("BL_GLASS_BEND")) : 3.0f;
        if (const char* ml = getenv("BL_SHEET_MAXLEVEL")) m.max_level = atoi(ml); // diagnostics: refinement depth
        return m;
    };
    // A body of the sheet alone (no beam holds it: the model editor's drum from a circle): a metal's plane held by the
    // membrane projection (its yield force sigma_y t), as the Steel Barrels' (with the springs the step allows, a car's
    // panels on their frame, a 2 mm steel drum dropped 1.5 m lay flat)
    // Panels welded on a frame (`welds`: nodes of their own, no beam) are the same: the welds hold them, their plane
    // held by the projection, and the sheet takes 4 short steps a substep (its hinges the budget of 1 mm of steel
    // bending: on its own light nodes at one step the budget left a panel as limp as cloth, the glass cracked standing)
    const bool sheet_alone = v->body && v->body->beams.empty();
    const bool welded = v->body && !v->def().welds.empty();
    if (welded) v->body->shell_min_shift = std::max(v->body->shell_min_shift, getenv("BL_CAR_SHIFT") ? atoi(getenv("BL_CAR_SHIFT")) : 0); // (diagnostics: 2, 4 short steps: more cracks standing, no faster)
    // (welded panels: the membrane damped, SoftBody::membrane_damp - 0.6 of the rate of stretch of the edges that left
    // their band lately took the rattle of the floors and the door glass (frames that moved a node group over 0.3 mm) to a
    // third, 0.3 to a half; BL_MEM_DAMP to try others)
    if (welded) v->body->membrane_damp = getenv("BL_MEM_DAMP") ? (float)atof(getenv("BL_MEM_DAMP")) : 0.6f;
    auto membrane = [&](phys::ShellMaterial& m, const std::string& mname, float kgm2) {
        if (!sheet_alone && !welded) return;
        // (glass: its tensile strength, it flows past it only to crack at its fracture strain soon after)
        const float sy = mname == "Steel" ? 250e6f : mname == "Aluminium" ? 150e6f : mname == "Lead" ? 12e6f : mname == "Glass" ? 40e6f : mname == "Plastic" ? 30e6f : 0.0f;
        const float rho = mname == "Steel" ? 7850.0f : mname == "Aluminium" ? 2700.0f : mname == "Glass" ? 2500.0f : mname == "Plastic" ? 950.0f : 11340.0f;
        if (sy <= 0) return;
        m.membrane = sy * kgm2 / rho;
        m.yield = mname == "Glass" ? 0.0008f : mname == "Plastic" ? 0.003f : 0.0015f; // (polypropylene gives more)
        // (welded panels bend as the plate they are, D = E t^3 / 12 (1 - nu^2), t from the areal density, the step's
        // budget above it; at the budget alone a sheet of even triangles had every hinge at its limit at once and its
        // skin rang from step to step, 1.2 mm aluminium on the Buggy eight times stiffer than it is)
        if (welded) {
            const float E = mname == "Steel" ? 200e9f : mname == "Aluminium" ? 70e9f : mname == "Glass" ? 70e9f : mname == "Plastic" ? 1.5e9f : 16e9f;
            const float t = kgm2 / rho;
            m.bend = std::min(m.bend, E * t * t * t / (12.0f * (1.0f - 0.3f * 0.3f)) * (getenv("BL_PLATE_BEND") ? (float)atof(getenv("BL_PLATE_BEND")) : 1.0f));
        }
    };
    auto look = [&](const std::string& mname, vec3 color) {
        auto vis = mat_visual(mat_spec(mname.c_str()));
        const bool metal = mname == "Steel" || mname == "Aluminium" || mname == "Lead";
        if (color.x >= 0) vis->color = vec4(color.x, color.y, color.z, vis->color.w);
        else if (metal) vis->color = player ? vec4(0.85f, 0.15f, 0.12f, 1) : vec4(0.2f, 0.45f, 0.85f, 1);
        return vis;
    };
    phys::ShellMaterial m = body_material(name, max_level);
    membrane(m, name, kg_m2);
    auto vis = look(name, vec3(-1));
    std::vector<std::array<int, 3>> only;
    std::vector<int> only_mat;
    for (const auto& s : v->def().shells) only.push_back({s.n1, s.n2, s.n3}), only_mat.push_back(s.mat);
    // the shells' own materials (`set_shell_material` in the shells section)
    std::vector<Vehicle::SheetMaterial> extra;
    for (const auto& sm : v->def().shell_materials) {
        Vehicle::SheetMaterial e;
        e.mat = body_material(sm.material, sm.max_level >= 0 ? sm.max_level : max_level);
        e.mat.kg_m2 = sm.kg_m2;
        membrane(e.mat, sm.material, sm.kg_m2 > 0 ? sm.kg_m2 : kg_m2);
        e.visual = look(sm.material, sm.color);
        extra.push_back(e);
    }
    v->make_sheet_body(m, kg_m2, vis, thick, only.empty() ? nullptr : &only, &only_mat, extra.empty() ? nullptr : &extra);
    if (sheet_alone && m.membrane > 0) { // (at rest it stays, as the barrels: see build_barrel)
        v->body->contact_slop = 0.003f;
        v->body->contact_push_max = 0.5f;
        v->body->rest_damp = 3.0f;
        v->body->rest_vib_damp = 10.0f;
    }
}

void scene_sheet_car(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(121, 361, 1.0f, vec2(-60, -100));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 2.0f, 50.0f, 3, 23);
    te::flatten_rect(hf, vec2(0, 60), vec2(24, 200), 0, 0.0f, 16, SURF_ASPHALT);
    g.finish_terrain();
    auto sheet_car = [&](vec3 pos, float yaw, bool player) { return g.spawn_vehicle("sheet_car/sheet_car", pos, yaw, player); }; // (the body: apply_vehicle_sheet_body)
    // the concrete wall, the poles beside the lane, a second sheet car across it
    g.add_static_box(vec3(-3, 1.5f, 160), vec3(5, 1.5f, 0.6f), quat(), SURF_CONCRETE, A.concrete);
    for (int i = 0; i < 3; i++) {
        PoleDesc pd;
        pd.base = vec3(6, 0, 110 + i * 12.0f);
        pd.segments = 6;
        pd.length = 6;
        pd.radius = 0.09f;
        pd.mass = 120;
        pd.k_ang = 2e5f;
        pd.yield_deg = 6;
        pd.break_torque = 9000;
        pd.mat = A.metal;
        g.add_object(build_pole(g.world, pd, format("pole%d", i)));
    }
    add_cone_line(g, vec3(-8, 0, 0), vec3(-8, 0, 150), 12);
    add_cone_line(g, vec3(8, 0, 0), vec3(8, 0, 100), 8);
    g.no_player_vehicle = true; // (the scene spawns its own: the sheet car is the player's)
    g.set_spawn(vec3(0, 0, 0), 0);
    sheet_car(vec3(0, 0, 0), 0, true);
    sheet_car(vec3(2, 0, 130), 90, false);
    g.labels.push_back({vec3(2, 2.5f, 130), "SHEET CAR (parked)"});
    g.labels.push_back({vec3(-3, 3.6f, 160), "WALL"});
    g.scene_hint = "The car's body is a sheet of steel triangle elements on a node-beam frame: it dents, tears and cracks (F3 shows the mesh). "
                   "Drive into the parked car, the poles or the wall, or Scene > Launch at 50/80 km/h. R resets the body.";
}

// ------------------------------------------------------------------------------------------- frame car, buggy
// The FEM cars' test pad: the Frame Car (assets/vehicles/frame_car, tools/make_frame_car.py), a welded space frame of
// FEM frame elements with sheet metal panels on it, double wishbones on ball joints; the Buggy (assets/vehicles/buggy,
// tools/make_buggy.py), a desert racer's tube cage with aluminium panels on it, long-travel wishbones and trailing arms.
// A flat test pad: a lane to a concrete wall, a ramp, a curb to trip over (the Buggy's: a gravel lane with whoops and a
// tabletop jump); the Scene menu's stress tests drop the car, turn it over, trip it, launch it at the wall and off the ramp.

// the player's car put back at pos (facing +z turned by yaw), then moved and set spinning: lifted by `lift`, turned
// by `turn` about its centre, moving at v and turning at w (rad/s, world); the wheels roll with the ground speed
void frame_car_stunt(Game& g, vec3 pos, float yaw, float lift, const quat& turn, vec3 v, vec3 w) {
    Vehicle* car = g.player_vehicle();
    if (!car) return;
    car->reset(pos, yaw);
    SoftBody& b = *car->body;
    const vec3 c = b.center_of_mass();
    b.transform(turn, c, vec3(0, lift, 0));
    car->launch(v);
    const vec3 c2 = b.center_of_mass();
    for (Node& n : b.nodes)
        if (n.inv_mass > 0) n.v += cross(w, n.p - c2);
    for (vec3& x : b.fem.w) x += w;
    b.seat_wheels(); // (the ring tyres' wheels turning with it)
    b.wake();
}

// as the grab tool does: the top node over the centre of mass pulled `lift` up at `strength` times the tool's pull
void frame_car_grab(Game& g, float lift, float strength) {
    Vehicle* car = g.player_vehicle();
    if (!car) return;
    car->reset(vec3(0, 0, 20), 0);
    SoftBody& b = *car->body;
    const vec3 c = b.center_of_mass();
    float top = -1e9f;
    for (uint32_t i = 0; i < (uint32_t)b.nodes.size(); i++)
        if (b.fem.slot(i) >= 0) top = std::max(top, b.nodes[i].p.y);
    // (BL_GRAB_AT=x,y,z: the frame node nearest that point from the centre of mass instead; checks)
    vec3 at(0, 0, 0);
    bool any = false;
    if (const char* e = getenv("BL_GRAB_AT")) any = sscanf(e, "%f,%f,%f", &at.x, &at.y, &at.z) == 3;
    int best = -1;
    float bd = 1e9f;
    for (uint32_t i = 0; i < (uint32_t)b.nodes.size(); i++) {
        const vec3 p = b.nodes[i].p;
        const float d = any ? length2(p - (c + at)) : (p.x - c.x) * (p.x - c.x) + (p.z - c.z) * (p.z - c.z);
        if (b.fem.slot(i) >= 0 && (any || p.y > top - 0.05f) && d < bd) bd = d, best = (int)i; // (a frame node: a sheet's would tear out)
    }
    if (best < 0) return;
    // (the node itself, not a pick along a ray: that finds the roof's sheet)
    g.grab_end();
    g.grab_strength = strength;
    g.grab_active = true;
    g.grab_body = &b;
    g.grab_node = best;
    g.grab_depth = 3.0f + lift;
    g.grab_bodies = {&b};
    b.grab_node = best;
    b.grab_target = b.nodes[best].p + vec3(0, lift, 0);
    b.grab_k = std::max(2000.0f, 60000.0f * std::pow(std::max(b.total_mass(), 50.0f) / 3000.0f, 0.75f));
    b.grab_scale = strength;
    b.wake();
}


// what the stress tests put into the scene besides the player's car (a second car, the slab, the axe): cleared by
// the next test
struct FrameCarExtras {
    Vehicle* other = nullptr;
    DynamicObject* slab = nullptr;
    AxeSwing axe;
    // two cars: how far one car's frame got into the other - its frame nodes inside the other's collision hull (the
    // generalized winding number of the hull's triangles about the node), the deepest from the hull's surface
    vec3 axis{0};
    int inside = 0, inside_max = 0;
    float depth = 0, depth_max = 0;
};

// the frame nodes of `a` inside the collision hull (the one-sided triangles, not torn) of `h`: how many, the deepest (m)
void frame_nodes_inside(const SoftBody& a, const SoftBody& h, int& count, float& deepest) {
    count = 0, deepest = 0;
    std::vector<const Triangle*> hull;
    for (const Triangle& t : h.tris)
        if (!t.two_sided && !t.torn) hull.push_back(&t);
    if (hull.empty()) return;
    const AABB box = h.aabb;
    for (uint32_t n : a.fem.node) {
        const vec3 p = a.nodes[n].p;
        if (p.x < box.mn.x || p.y < box.mn.y || p.z < box.mn.z || p.x > box.mx.x || p.y > box.mx.y || p.z > box.mx.z) continue;
        double w = 0; // (the solid angles, Van Oosterom and Strackee)
        float dmin = 1e9f;
        for (const Triangle* t : hull) {
            const vec3 A = h.nodes[t->a].p - p, B = h.nodes[t->b].p - p, C = h.nodes[t->c].p - p;
            const double la = length(A), lb = length(B), lc = length(C);
            const double num = dot(A, cross(B, C)), den = la * lb * lc + dot(A, B) * lc + dot(A, C) * lb + dot(B, C) * la;
            w += 2.0 * std::atan2(num, den);
            vec3 bary;
            dmin = std::min(dmin, length(closest_on_triangle(p, h.nodes[t->a].p, h.nodes[t->b].p, h.nodes[t->c].p, bary) - p));
        }
        if (std::fabs(w) / (4.0 * kPi) > 0.5) {
            count++, deepest = std::max(deepest, dmin);
            static const bool dbg = getenv("BL_INSIDEDBG") != nullptr; // (diagnostics: which nodes)
            if (dbg) printf("  inside: node %u of %s, %.0f cm deep\n", n, a.name.c_str(), dmin * 100.0f);
        }
    }
}
FrameCarExtras s_fc;

// the pad's car: the player's vehicle, the one the crash tests spawn a second of
struct PadCar {
    std::string id = "frame_car/frame_car", name = "Frame Car";
};
PadCar s_pad;

// the player's car put back on its wheels at pos and what the drops need of it: its top (m) and its centre of mass
bool frame_car_measure(Game& g, vec3 pos, float& top, vec3& com) {
    Vehicle* car = g.player_vehicle();
    if (!car) return false;
    car->reset(pos, 0);
    top = car->body->aabb.mx.y, com = car->body->center_of_mass();
    return true;
}

void frame_car_clear(Game& g) {
    if (s_fc.other) g.remove_vehicle(s_fc.other);
    if (s_fc.slab) g.remove_object(s_fc.slab);
    if (s_fc.axe.obj) g.remove_object(s_fc.axe.obj);
    s_fc = FrameCarExtras();
}

// a gravel lane on the pad (the Buggy's): whoops, 0.5 m high every 8 m, from z = 20 to 100, and a tabletop jump
// (a 2.2 m kicker, its face curving up over 6 m to 12 degrees at the lip, a 16 m table, a 20 m landing); the
// terrain's grid is 1 m
void pad_dirt_lane(phys::Heightfield& hf, float x0, float half) {
    auto bump = [](float z) {
        if (z > 20 && z < 100) return 0.25f * (1.0f - std::cos(2.0f * kPi * (z - 20) / 8.0f)); // (the whoops)
        const float g = std::tan(12.0f * kDeg2Rad), arc = 6.0f, kick = arc + (2.2f - 0.5f * g * arc) / g; // (13.4 m)
        if (z > 150 && z < 150 + arc) return 0.5f * g * (z - 150) * (z - 150) / arc;
        if (z >= 150 + arc && z < 150 + kick) return 0.5f * g * arc + g * (z - 150 - arc);
        if (z >= 150 + kick && z < 166 + kick) return 2.2f;
        if (z >= 166 + kick && z < 186 + kick) return 2.2f * (0.5f + 0.5f * std::cos(kPi * (z - 166 - kick) / 20.0f));
        return 0.0f;
    };
    for (int iz = 0; iz < hf.nz(); iz++)
        for (int ix = 0; ix < hf.nx(); ix++) {
            const vec2 w = hf.origin() + vec2((float)ix, (float)iz) * hf.cell();
            const float dx = std::fabs(w.x - x0);
            if (dx > half + 3 || w.y < 0 || w.y > 230) continue;
            const float edge = dx < half ? 1.0f : 0.5f + 0.5f * std::cos(kPi * (dx - half) / 3.0f); // (3 m shoulders)
            hf.h(ix, iz) += bump(w.y) * edge;
            if (dx < half + 1) hf.surf(ix, iz) = SURF_GRAVEL;
        }
}

// A rough field on the dirt pad, x -84..-40, z 105..195, and an earth bank beyond it along the terrain's edge (a car
// in figure eights wandered off the edge): ground rolling in 3-9 m
// waves up to half a metre (sums of sines at fixed phases) with smaller bumps on them, dirt, 3 m shoulders to the flat
const vec3 kRoughField(-62.0f, 0.0f, 150.0f);
void pad_rough_field(phys::Heightfield& hf) {
    for (int iz = 0; iz < hf.nz(); iz++)
        for (int ix = 0; ix < hf.nx(); ix++) {
            const vec2 w = hf.origin() + vec2((float)ix, (float)iz) * hf.cell();
            const float dx = std::fabs(w.x - kRoughField.x) - 22.0f, dz = std::fabs(w.y - kRoughField.z) - 45.0f;
            const float d = std::max(dx, dz);
            if (w.x < -104.0f && w.y > 60.0f && w.y < 240.0f) hf.h(ix, iz) += 2.0f * std::min(1.0f, (-104.0f - w.x) / 8.0f); // (the bank)
            if (d > 3.0f) continue;
            const float edge = d < 0 ? 1.0f : 0.5f + 0.5f * std::cos(kPi * d / 3.0f);
            const float x = w.x, z = w.y;
            float h = 0.16f * std::sin(x * 0.71f + 1.3f) * std::sin(z * 0.83f + 0.4f) + 0.12f * std::sin(x * 0.37f - z * 0.29f + 2.1f) +
                      0.10f * std::sin(x * 1.13f + z * 0.97f + 0.7f) + 0.07f * std::sin(x * 1.9f - 0.5f) * std::sin(z * 2.3f + 1.1f);
            hf.h(ix, iz) += (h + 0.2f) * edge;
            hf.surf(ix, iz) = SURF_DIRT;
        }
}

void scene_fem_pad(Game& g, const char* vid, const char* name, bool dirt) {
    s_pad.id = vid, s_pad.name = name;
    auto& A = SharedAssets::get();
    g.create_terrain(241, 401, 1.0f, vec2(-120, -100));
    auto& hf = g.world.statics.terrain;
    te::flatten_rect(hf, vec2(0, 100), vec2(116, 196), 0, 0.0f, 3, SURF_ASPHALT);
    if (dirt) pad_dirt_lane(hf, 80, 5), pad_rough_field(hf);
    g.finish_terrain();
    // the wall across the main lane, a 15 degree ramp 1.5 m high on the right lane, a curb along the left lane
    g.add_static_box(vec3(0, 1.5f, 150), vec3(4, 1.5f, 0.6f), quat(), SURF_CONCRETE, A.concrete);
    const float ra = 15.0f * kDeg2Rad, rl = 6.0f;
    g.add_static_box(vec3(40, rl * 0.5f * std::sin(ra) - 0.1f, 80 + rl * 0.5f * std::cos(ra)), vec3(2.5f, 0.1f, rl * 0.5f), quat::axis_angle(vec3(1, 0, 0), -ra), SURF_CONCRETE,
                     A.concrete);
    g.add_static_box(vec3(-40, 0.12f, 60), vec3(0.15f, 0.12f, 12), quat(), SURF_CONCRETE, A.concrete);
    add_cone_line(g, vec3(-5, 0, 0), vec3(-5, 0, 140), 12);
    add_cone_line(g, vec3(5, 0, 0), vec3(5, 0, 140), 12);
    g.no_player_vehicle = true;
    g.set_spawn(vec3(0, 0, 0), 0);
    g.spawn_vehicle(s_pad.id, vec3(0, 0, 0), 0, true);
    g.add_static_cylinder(vec3(16, 0, 150), 0.16f, 4.0f, SURF_CONCRETE, A.concrete); // (a pole beside the wall)
    g.labels.push_back({vec3(0, 3.6f, 150), "WALL"});
    g.labels.push_back({vec3(16, 4.4f, 150), "POLE"});
    g.labels.push_back({vec3(40, 3.0f, 84), "RAMP"});
    g.labels.push_back({vec3(-40, 1.0f, 48), "CURB"});
    if (dirt) g.labels.push_back({vec3(80, 1.6f, 18), "WHOOPS"}), g.labels.push_back({vec3(80, 3.6f, 148), "JUMP"});
    const float kmh = 1.0f / 3.6f;
    frame_car_clear(g);
    s_fc = FrameCarExtras();
    // the axe's cut: each frame, the sector its edge swept about the pivot (a plane across the car) cuts what it crosses
    // (ahead of the blade: from where the edge is to where it will be in a frame and a half, so the edge meets what it
    // cut, not the car it has to push through first)
    g.scene_update = [](Game& gg, float dt) {
        if (s_fc.other && length2(s_fc.axis) > 0)
            if (const Vehicle* car = gg.player_vehicle(); car && !car->body->fem.empty() && !s_fc.other->body->fem.empty()) {
                int n1, n2;
                float d1, d2;
                frame_nodes_inside(*car->body, *s_fc.other->body, n1, d1);
                frame_nodes_inside(*s_fc.other->body, *car->body, n2, d2);
                s_fc.inside = n1 + n2, s_fc.depth = std::max(d1, d2);
                s_fc.inside_max = std::max(s_fc.inside_max, s_fc.inside), s_fc.depth_max = std::max(s_fc.depth_max, s_fc.depth);
                gg.scene_status = format("frame nodes inside the other car: %d, %.0f cm deep (most %d, %.0f cm); centres %.2f m apart", s_fc.inside, s_fc.depth * 100.0f,
                                         s_fc.inside_max, s_fc.depth_max * 100.0f, length(car->body->center_of_mass() - s_fc.other->body->center_of_mass()));
            }
        if (!s_fc.axe.obj) return;
        // (the brake lines cut: held by their brakes the halves leaned on each other at the cut, level)
        if (s_fc.axe.step(gg, dt) > 0)
            if (Vehicle* car = gg.player_vehicle()) car->cut_brakes();
        gg.scene_status = s_fc.axe.status();
    };
    g.scene_actions.push_back({"Drop from 5 m", [](Game& gg) { frame_car_clear(gg), frame_car_stunt(gg, vec3(0, 0, 20), 0, 5.0f, quat(), vec3(0), vec3(0)); }});
    g.scene_actions.push_back({"Drop from 10 m", [](Game& gg) { frame_car_clear(gg), frame_car_stunt(gg, vec3(0, 0, 20), 0, 10.0f, quat(), vec3(0), vec3(0)); }});
    g.scene_actions.push_back({format("Head-on into another %s (50 km/h each)", name), [kmh](Game& gg) {
                                   frame_car_clear(gg);
                                   frame_car_stunt(gg, vec3(0, 0, 50), 0, 0.0f, quat(), vec3(0, 0, 50 * kmh), vec3(0));
                                   s_fc.other = gg.spawn_vehicle(s_pad.id, vec3(0.25f, 0, 72), 180, false);
                                   if (s_fc.other) s_fc.other->launch(vec3(0, 0, -50 * kmh)), s_fc.axis = vec3(0, 0, 1);
                               }});
    g.scene_actions.push_back({format("Another %s into its side at 50 km/h", name), [kmh](Game& gg) {
                                   frame_car_clear(gg);
                                   frame_car_stunt(gg, vec3(0, 0, 60), 0, 0.0f, quat(), vec3(0), vec3(0));
                                   s_fc.other = gg.spawn_vehicle(s_pad.id, vec3(-9, 0, 59.6f), 90, false);
                                   if (s_fc.other) s_fc.other->launch(vec3(50 * kmh, 0, 0)), s_fc.axis = vec3(-1, 0, 0);
                               }});
    g.scene_actions.push_back({"Drop a 5 t concrete slab on it from 2.5 m", [](Game& gg) {
                                   frame_car_clear(gg);
                                   float top;
                                   vec3 com;
                                   if (!frame_car_measure(gg, vec3(0, 0, 20), top, com)) return;
                                   frame_car_stunt(gg, vec3(0, 0, 20), 0, 0.0f, quat(), vec3(0), vec3(0));
                                   SoftBoxDesc d;
                                   d.center = vec3(0, top + 2.5f + 0.15f, 20);
                                   d.size = vec3(2.2f, 0.3f, 4.2f);
                                   d.nx = 3, d.ny = 2, d.nz = 5;
                                   d.mass = 5000.0f;
                                   d.beams = {1e9f, 4e4f, 1e12f, 1e12f, 0.0f};
                                   d.mat = SharedAssets::get().concrete;
                                   s_fc.slab = gg.add_object(build_soft_box(gg.world, d, "slab"));
                               }});
    g.scene_actions.push_back({"Launch at the pole at 50 km/h", [kmh](Game& gg) {
                                   frame_car_clear(gg), frame_car_stunt(gg, vec3(16.4f, 0, 115), 0, 0.0f, quat(), vec3(0, 0, 50 * kmh), vec3(0));
                               }});
    g.scene_actions.push_back({"The giant axe (it swings down and cuts the car in two)", [](Game& gg) {
                                   frame_car_clear(gg);
                                   frame_car_stunt(gg, vec3(-25, 0, 120), 0, 0.0f, quat(), vec3(0), vec3(0));
                                   AxeDesc d;
                                   d.pivot = vec3(-25, 8.33f, 120); // (its edge's corners 8 cm over the ground, 5 cm swinging:
                                   // lower, it pressed the shards lying under it through the ground)
                                   s_fc.axe.make(gg, d, "axe");
                               }});
    g.scene_actions.push_back({"Drop on the roof from 1.5 m", [](Game& gg) {
                                   // (turned over about its centre of mass its top goes to 2 com - top: lifted to 1.5 m)
                                   float top;
                                   vec3 com;
                                   if (!frame_car_measure(gg, vec3(0, 0, 20), top, com)) return;
                                   frame_car_stunt(gg, vec3(0, 0, 20), 0, 1.5f + top - 2.0f * com.y, quat::axis_angle(vec3(0, 0, 1), kPi), vec3(0), vec3(0));
                               }});
    g.scene_actions.push_back({"Lay it on its side (from 0.3 m)", [](Game& gg) {
                                   frame_car_clear(gg);
                                   Vehicle* car = gg.player_vehicle();
                                   if (!car) return;
                                   frame_car_stunt(gg, vec3(0, 0, 20), 0, 0.0f, quat::axis_angle(vec3(0, 0, 1), 0.5f * kPi), vec3(0), vec3(0));
                                   SoftBody& b = *car->body;
                                   float lo = 1e9f;
                                   for (const Node& n : b.nodes) lo = std::min(lo, n.p.y);
                                   b.transform(quat(), b.center_of_mass(), vec3(0, 0.3f - lo, 0));
                               }});
    g.scene_actions.push_back({"Barrel roll at 50 km/h", [kmh](Game& gg) {
                                   frame_car_stunt(gg, vec3(0, 0, 10), 0, 0.6f, quat(), vec3(0, 3.5f, 50 * kmh), vec3(0, 0, 5.0f));
                               }});
    g.scene_actions.push_back({"Trip over the curb at 40 km/h", [kmh](Game& gg) {
                                   frame_car_stunt(gg, vec3(-44, 0, 60), 0, 0.0f, quat(), vec3(40 * kmh, 0, 0), vec3(0));
                               }});
    g.scene_actions.push_back({"Launch at the wall at 60 km/h", [kmh](Game& gg) {
                                   frame_car_stunt(gg, vec3(0, 0, 110), 0, 0.0f, quat(), vec3(0, 0, 60 * kmh), vec3(0));
                               }});
    g.scene_actions.push_back({"Off the ramp at 70 km/h", [kmh](Game& gg) {
                                   frame_car_stunt(gg, vec3(40, 0, 40), 0, 0.0f, quat(), vec3(0, 0, 70 * kmh), vec3(0));
                               }});
    if (dirt) {
        g.scene_actions.push_back({"Through the whoops at 80 km/h", [kmh](Game& gg) {
                                       frame_car_clear(gg), frame_car_stunt(gg, vec3(80, 0, 2), 0, 0.0f, quat(), vec3(0, 0, 80 * kmh), vec3(0));
                                   }});
        g.scene_actions.push_back({"Over the jump at 90 km/h", [kmh](Game& gg) {
                                       frame_car_clear(gg), frame_car_stunt(gg, vec3(80, 0, 115), 0, 0.0f, quat(), vec3(0, 0, 90 * kmh), vec3(0));
                                   }});
        g.scene_actions.push_back({"To the rough field", [](Game& gg) {
                                       frame_car_clear(gg), frame_car_stunt(gg, kRoughField + vec3(0, 0.6f, -40), 0, 0.0f, quat(), vec3(0), vec3(0));
                                   }});
    }
    g.scene_actions.push_back({"Let the latches go (hood, trunk lid, doors)", [](Game& gg) {
                                   Vehicle* car = gg.player_vehicle();
                                   if (!car) return;
                                   SoftBody& b = *car->body;
                                   int n = b.fem.release_latches(b);
                                   // (the Frame Car's latches are beams between a part's frame and the body's: they go too)
                                   for (Beam& bm : b.beams) {
                                       if (bm.flags & BF_BROKEN) continue;
                                       const int ca = b.fem.component_of(bm.a), cb = b.fem.component_of(bm.b);
                                       if (ca >= 0 && cb >= 0 && ca != cb) bm.flags |= BF_BROKEN, n++;
                                   }
                                   log_info("scene: %d latches let go", n);
                               }});
    g.scene_actions.push_back({"Let every part go (bolts, hinges, latches)", [](Game& gg) {
                                   Vehicle* car = gg.player_vehicle();
                                   if (!car) return;
                                   SoftBody& b = *car->body;
                                   int n = 0;
                                   for (FrameMount& m : b.fem.mounts)
                                       if (!m.broken) m.broken = true, b.fem.mounts_broken++, n++;
                                   for (Beam& bm : b.beams) {
                                       if (bm.flags & BF_BROKEN) continue;
                                       const int ca = b.fem.component_of(bm.a), cb = b.fem.component_of(bm.b);
                                       if (ca >= 0 && cb >= 0 && ca != cb) bm.flags |= BF_BROKEN, n++;
                                   }
                                   b.wake();
                                   log_info("scene: %d mounts and latches let go", n);
                               }});
    g.scene_actions.push_back({"Hang it from a crane (1 m up)", [](Game& gg) {
                                   if (Vehicle* car = gg.player_vehicle()) car->reset(vec3(0, 0, 20), 0), gg.crane_vehicle(car, 1.0f);
                               }});
    g.scene_actions.push_back({"Hang it tilted from a crane (8 deg roll, 6 deg pitch)", [](Game& gg) {
                                   if (Vehicle* car = gg.player_vehicle()) car->reset(vec3(0, 0, 20), 0), gg.crane_vehicle(car, 1.0f, 8.0f, 6.0f);
                               }});
    g.scene_actions.push_back({"Grab the roof and lift it 1.5 m", [](Game& gg) {
                                   const char* s = getenv("BL_GRAB_STRENGTH");
                                   frame_car_grab(gg, 1.5f, s ? (float)atof(s) : 5.0f);
                               }});
    g.scene_actions.push_back({"Let go (the crane, the grab)", [](Game& gg) { gg.crane_release(), gg.grab_end(); }});
    g.scene_hint = dirt ? "A desert racer's cage of welded tubes (FEM frame elements) with aluminium panels, long-travel wishbones and trailing "
                          "arms. F3: the frame, orange where bent for good. Scene menu: the whoops and the jump on the gravel lane, drop it, "
                          "turn it over, launch it at the wall. R resets it."
                        : "A space frame of welded tubes (FEM frame elements) with sheet panels on it, double wishbones on ball joints. F3: the "
                          "frame, orange where bent for good. Scene menu: drop it, turn it over, trip it on the curb, launch it at the wall or off "
                          "the ramp. R resets it.";
    if (std::string(vid).find("shell_car") != std::string::npos)
        g.scene_hint = "A saloon on the BMW E36's lines, all FEM: its body-in-white members (sills, pillars, rails) with sheets of FEM "
                       "triangles between them (floor, roof, firewall, aprons, quarters); the hood, fenders, doors (with their window "
                       "frames), trunk lid, bumpers and tail lights FEM triangles on hinges, latches, clamped bolts, buffers and stays that "
                       "let go; the Frame Car's suspension. F3: the elements, orange where the steel has yielded. Scene menu: crashes, "
                       "drops, rolls, on its side, the whoops, the jump, the rough field, the latches. R resets it.";
}

void scene_frame_car(Game& g) { scene_fem_pad(g, "frame_car/frame_car", "Frame Car", false); }
void scene_buggy(Game& g) { scene_fem_pad(g, "buggy/buggy", "Buggy", true); }
void scene_shell_car(Game& g) { scene_fem_pad(g, "shell_car/shell_car", "Shell Car", true); } // (the dirt lane and the rough field: its offroad tests)
// (the Shell Car with the BMW E36 Lightweight's - the M3's - meshes on it, flexbodies skinned to its parts' nodes:
// tools/make_shell_car.py SC_M3=1)
void scene_shell_car_m3(Game& g) { scene_fem_pad(g, "shell_car/shell_car_m3", "Shell Car M3", true); }

// The pad's crash tests as scenes of their own (the reports' videos): the Frame Car's (or the Buggy's) scene with one
// of its Scene menu tests started on the first frame and the free camera where the report's was; F5 (a reload keeps
// the camera) or the Scene menu runs it again, the other tests stay in the menu
void scene_frame_car_test(Game& g, const char* action, vec3 eye, vec3 target, const char* hint, void (*scene)(Game&)) {
    scene(g);
    std::function<void(Game&)> run;
    std::string label;
    for (const SceneAction& a : g.scene_actions)
        if (a.label.find(action) != std::string::npos) {
            run = a.run, label = a.label;
            break;
        }
    if (!run) return;
    auto base = g.scene_update;
    g.scene_update = [base, run, label, started = false](Game& gg, float dt) mutable {
        if (!started) started = true, log_info("scene test: %s", label.c_str()), run(gg);
        if (base) base(gg, dt);
    };
    g.scene_actions.insert(g.scene_actions.begin(), {"Run the test again (F5)", run});
    g.cam.look_free(eye, target);
    g.scene_hint = std::string(hint) + " F5 runs it again; the Scene menu has the other tests, the " + s_pad.name + " scene is the pad to drive on.";
}

} // namespace bl
