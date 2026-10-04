// The physics lab: primitives, materials, sheets, the FEM shells, the steel barrels.
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


void scene_lab(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(241, 241, 1.0f, vec2(-120, -120));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 4.0f, 60.0f, 3, 5);
    te::flatten_rect(hf, vec2(0, 0), vec2(80, 80), 0, 0.0f, 20, SURF_CONCRETE);
    g.finish_terrain();

    // A: crate pyramid (stacking / collisions)
    {
        int id = 0;
        for (int row = 0; row < 4; row++)
            for (int i = 0; i < 4 - row; i++) {
                SoftBoxDesc d;
                d.center = vec3(-30 + i * 1.05f + row * 0.52f, 0.5f + row * 1.02f, -30);
                d.size = vec3(1.0f);
                d.nx = d.ny = d.nz = 3;
                d.mass = 60;
                d.mat = std::make_shared<Material>(*A.wood);
                d.mat->diffuse = A.tex_crate;
                d.beams = {1.2e6f, 1500, 3e4f, 1.5e5f, 0.0f};
                g.add_object(build_soft_box(g.world, d, format("crate%d", id++)));
            }
    }
    // B: jelly spheres on a ramp
    {
        g.add_static_box(vec3(-10, 1.5f, -30), vec3(3, 0.3f, 6), quat::axis_angle(vec3(1, 0, 0), 0.35f), SURF_CONCRETE, A.concrete);
        for (int i = 0; i < 3; i++) {
            SoftSphereDesc d;
            d.center = vec3(-11 + i * 1.2f, 5.0f + i * 1.8f, -34);
            d.radius = 0.55f;
            d.mass = 40;
            d.beams = {5e4f, 90, 1e9f, 1e9f, 0};
            g.add_object(build_soft_sphere(g.world, d, format("jelly%d", i)));
        }
    }
    // C: plastic deformation - metal blocks dropped on a pillar
    {
        g.add_static_cylinder(vec3(10, 0, -30), 0.35f, 1.2f, SURF_METAL, A.metal);
        for (int i = 0; i < 2; i++) {
            SoftBoxDesc d;
            d.center = vec3(10 + i * 0.3f, 6.0f + i * 3.5f, -30);
            d.size = vec3(2.0f, 0.6f, 1.2f);
            d.nx = 6;
            d.ny = 3;
            d.nz = 4;
            d.mass = 500;
            d.mat = A.metal;
            d.beams = {5e6f, 2500, 6e4f, 1e7f, 0.0f};
            g.add_object(build_soft_box(g.world, d, format("metal%d", i)));
        }
    }
    // D: breakable tower + wrecking ball
    {
        TowerDesc td;
        td.base = vec3(30, 0, -30);
        td.height = 9;
        td.levels = 5;
        td.width = 2.2f;
        td.mass = 700;
        td.beams = {8e6f, 3000, 3e5f, 1.4e6f, 0.0f};
        g.add_object(build_tower(g.world, td, "tower"));
        // gallows
        g.add_static_box(vec3(38, 6, -30), vec3(0.3f, 6, 0.3f), quat(), SURF_METAL, A.metal);
        g.add_static_box(vec3(35, 12, -30), vec3(3.4f, 0.3f, 0.3f), quat(), SURF_METAL, A.metal);
        RopeDesc rd;
        rd.a = vec3(33.5f, 11.7f, -30);
        rd.b = vec3(33.5f + 7.0f, 11.7f + 0.5f, -30); // start horizontal -> swings into the tower
        rd.segments = 10;
        rd.mass = 30;
        rd.end_mass = 1500;
        rd.end_size = 1.0f;
        rd.beams = {5e7f, 4000, 1e12f, 1e12f, 0};
        g.add_object(build_rope(g.world, rd, "wrecking ball"));
    }
    // E: orientation preserving joints - cantilevers (elastic / plastic / breakable) + a tree
    {
        const float x0 = -30;
        for (int i = 0; i < 3; i++) {
            PoleDesc pd;
            pd.base = vec3(x0 + i * 6.0f, 1.0f, 0);
            g.add_static_box(pd.base - vec3(0, 0.5f, 0.4f), vec3(0.4f, 1.0f, 0.4f), quat(), SURF_CONCRETE, A.concrete);
            pd.base.z += 0.0f;
            pd.dir = vec3(0, 0.15f, 1);
            pd.segments = 8;
            pd.length = 4.0f;
            pd.radius = 0.07f;
            pd.mass = 30;
            pd.k_ang = 6e4f;
            pd.tip_mass = 0;
            pd.mat = i == 0 ? A.metal : (i == 1 ? A.yellow : A.red);
            if (i == 1) pd.yield_deg = 4.0f;
            if (i == 2) pd.break_torque = 3500.0f;
            g.add_object(build_pole(g.world, pd, i == 0 ? "elastic pole" : (i == 1 ? "plastic pole" : "breakable pole")));
            // weight dropped onto the tip
            SoftBoxDesc bdsc;
            bdsc.center = pd.base + normalize(pd.dir) * 3.8f + vec3(0, 3.5f + i, 0);
            bdsc.size = vec3(0.6f);
            bdsc.mass = 120;
            bdsc.mat = A.metal;
            bdsc.beams = {3e6f, 1000, 1e9f, 1e9f, 0};
            g.add_object(build_soft_box(g.world, bdsc, format("weight%d", i)));
        }
        TreeDesc tdsc;
        tdsc.base = vec3(-10, 0, 3);
        tdsc.kind = TreeKind::Deciduous;
        tdsc.height = 7;
        tdsc.trunk_radius = 0.14f;
        g.add_object(build_tree(g.world, tdsc, "lab tree"));
    }
    // F: trampoline net + things to bounce
    {
        g.add_static_box(vec3(10, 0.75f, 0), vec3(4.3f, 0.75f, 0.2f), quat(), SURF_METAL, nullptr);
        NetDesc nd;
        nd.center = vec3(10, 1.5f, 5);
        nd.size = 8;
        nd.n = 14;
        nd.mass = 80;
        nd.beams = {3e5f, 300, 1e9f, 2.5e4f, 0};
        g.add_object(build_net(g.world, nd, "trampoline"));
        for (int i = 0; i < 4; i++) g.drop_primitive(i % 2 ? 1 : 0, vec3(9 + i * 0.7f, 5 + i * 2.0f, 4 + i * 0.5f));
    }
    // G: hanging ropes / chain pendulums (Newton cradle-ish)
    {
        g.add_static_box(vec3(32, 6, 4), vec3(4, 0.25f, 0.25f), quat(), SURF_METAL, A.metal);
        for (int i = 0; i < 5; i++) {
            RopeDesc rd;
            rd.a = vec3(29 + i * 1.02f, 5.75f, 4);
            rd.b = rd.a + vec3(i == 0 ? -3.0f : 0, i == 0 ? -1.8f : -3.5f, 0);
            rd.segments = 6;
            rd.mass = 5;
            rd.end_mass = 60;
            rd.end_size = 0.5f;
            rd.beams = {2e7f, 1000, 1e12f, 1e12f, 0};
            g.add_object(build_rope(g.world, rd, format("pendulum%d", i)));
        }
    }
    // H: dominoes (planks)
    {
        for (int i = 0; i < 16; i++) {
            SoftBoxDesc d;
            d.center = vec3(-30 + i * 1.3f, 1.0f, 30);
            d.size = vec3(0.2f, 2.0f, 1.0f);
            d.nx = 2;
            d.ny = 4;
            d.nz = 2;
            d.mass = 40;
            d.mat = i % 2 ? A.red : A.white;
            d.beams = {3e6f, 1500, 1e9f, 1e9f, 0};
            if (i == 0) d.rot = quat::axis_angle(vec3(0, 0, 1), -0.35f);
            g.add_object(build_soft_box(g.world, d, format("domino%d", i)));
        }
    }
    // I: breakable plank bridge between two blocks + heavy weight
    {
        g.add_static_box(vec3(14, 1.0f, 30), vec3(1, 1, 1.5f), quat(), SURF_CONCRETE, A.concrete);
        g.add_static_box(vec3(24, 1.0f, 30), vec3(1, 1, 1.5f), quat(), SURF_CONCRETE, A.concrete);
        SoftBoxDesc d;
        d.center = vec3(19, 2.1f, 30);
        d.size = vec3(11.0f, 0.2f, 1.2f);
        d.nx = 16;
        d.ny = 2;
        d.nz = 2;
        d.mass = 120;
        d.mat = A.wood;
        d.beams = {3e6f, 1500, 4e4f, 9e4f, 0.0f};
        g.add_object(build_soft_box(g.world, d, "plank"));
        SoftBoxDesc w;
        w.center = vec3(19, 6.0f, 30);
        w.size = vec3(0.9f);
        w.mass = 900;
        w.mat = A.metal;
        w.beams = {6e6f, 2000, 1e9f, 1e9f, 0};
        g.add_object(build_soft_box(g.world, w, "anvil"));
    }
    // J: sheets of fabric, metal and plastic in frames (shooting / driving targets)
    {
        const float z = -50.0f;
        auto frame = [&](float x, float w2, float top, MaterialPtr m) {
            g.add_static_box(vec3(x - w2 - 0.15f, top * 0.5f, z), vec3(0.12f, top * 0.5f, 0.12f), quat(), SURF_METAL, m);
            g.add_static_box(vec3(x + w2 + 0.15f, top * 0.5f, z), vec3(0.12f, top * 0.5f, 0.12f), quat(), SURF_METAL, m);
            g.add_static_box(vec3(x, top + 0.1f, z), vec3(w2 + 0.3f, 0.1f, 0.12f), quat(), SURF_METAL, m);
        };
        // sheets of triangle elements: hits refine them and open cracks between the finer pieces
        auto sheet = [&](const char* mat, float x, float mass, int clamp, MaterialPtr visual, const char* name) {
            SheetDesc sd;
            sd.center = vec3(x, 1.65f, z);
            sd.u = vec3(1, 0, 0);
            sd.v = vec3(0, 1, 0);
            sd.width = sd.height = 2.6f;
            sd.nu = sd.nv = 18;
            sd.mass = mass;
            sd.thickness = clamp == 3 ? 0.004f : 0.02f;
            sd.clamp = clamp;
            sd.mat = sheet_material(mat);
            sd.visual = visual;
            sd.uv_scale = clamp == 3 ? 3.0f : 1.0f;
            sd.seed = (uint32_t)(x * 10 + 50);
            g.add_object(build_sheet(g.world, sd, name));
        };
        // fabric curtain hanging from the bar (tears when hit hard)
        frame(-6.5f, 1.8f, 3.8f, A.dark_wood);
        sheet("Fabric", -6.5f, 8.0f, 3, fabric_visual(), "fabric");
        // steel plate clamped in a frame: dents plastically, tears when shot hard
        frame(0.0f, 1.3f, 3.1f, A.metal);
        auto steel = std::make_shared<Material>(*A.metal);
        steel->double_sided = true;
        sheet("Steel", 0.0f, 450.0f, 1, steel, "steel sheet"); // ~9 mm armour plate
        // plastic plate: flexible, cracks
        frame(6.5f, 1.3f, 3.1f, A.dark_wood);
        auto plastic = std::make_shared<Material>(*A.yellow);
        plastic->color = vec4(0.2f, 0.55f, 0.95f, 1);
        plastic->specular = 0.8f;
        plastic->gloss = 64;
        plastic->double_sided = true;
        sheet("Acrylic", 6.5f, 90.0f, 1, plastic, "plastic sheet");
    }
    g.set_spawn(vec3(0, 0, -75), 0);
    g.scene_hint = "J (front): fabric / steel / plastic sheets - press B and shoot them.  A crates  B jelly  C plastic metal  D tower + wrecking ball  E poles: elastic / plastic / breakable  F trampoline  G pendulums  H dominoes  I plank bridge. Left-drag any node to pull it. Scene menu: drop objects.";
}


const MatSpec kMats[] = {
    // name        color                          spec  gloss glass tex dens  kg/m2 | sheet k yield    break | bulk k  yield   break | damp ductile flaw
    {"Steel", {0.40f, 0.42f, 0.46f, 1}, 0.9f, 64, false, 0, 7850, 47, 1.0e7f, 0.05f, 0.5f, 2.0e7f, 0.1f, 1.0f, 200, true, 0},
    {"Aluminium", {0.60f, 0.62f, 0.66f, 1}, 0.8f, 48, false, 0, 2700, 22, 1.0e7f, 0.02f, 0.3f, 1.0e7f, 0.05f, 0.5f, 120, true, 0},
    {"Lead", {0.36f, 0.38f, 0.42f, 1}, 0.3f, 16, false, 0, 11300, 68, 4.0e5f, 0.002f, 0.8f, 3.0e6f, 0.003f, 1.0f, 150, true, 0},
    {"Glass", {0.55f, 0.8f, 0.85f, 0.35f}, 1.4f, 160, true, 0, 2500, 20, 8.0e6f, 0.006f, 0.006f, 1.0e7f, 0.012f, 0.012f, 80, false, 0.4f},
    {"Acrylic", {0.95f, 0.45f, 0.12f, 1}, 0.9f, 80, false, 0, 1190, 9.5f, 1.5e6f, 0.05f, 0.08f, 3.0e6f, 0.05f, 0.08f, 40, false, 0.2f},
    {"Plywood", {1, 1, 1, 1}, 0.1f, 8, false, 1, 600, 11, 3.0e6f, 0.01f, 0.018f, 4.0e6f, 0.012f, 0.02f, 80, false, 0.3f},
    {"Rubber", {0.1f, 0.1f, 0.11f, 1}, 0.25f, 12, false, 0, 1100, 11, 1.5e5f, 1e4f, 3.0f, 1.5e5f, 1e4f, 3.0f, 10, false, 0},
    {"Plastic", {0.13f, 0.13f, 0.14f, 1}, 0.5f, 24, false, 0, 950, 2.8f, 1.5e6f, 0.03f, 0.3f, 2.0e6f, 0.04f, 0.3f, 40, true, 0},
    {"Cardboard", {0.72f, 0.56f, 0.36f, 1}, 0.05f, 6, false, 0, 250, 1.5f, 4.0e5f, 0.01f, 0.05f, 4.0e5f, 0.02f, 0.1f, 20, false, 0.2f},
    {"Foam", {0.95f, 0.85f, 0.3f, 1}, 0.05f, 6, false, 0, 30, 1, 3.0e4f, 0.05f, 0.6f, 2.0e4f, 0.05f, 0.6f, 30, false, 0},
    {"Concrete", {1, 1, 1, 1}, 0.1f, 8, false, 3, 2400, 50, 1.0e7f, 0.002f, 0.002f, 1.0e7f, 0.01f, 0.01f, 300, false, 0.4f},
};
const MatSpec& mat_spec(const char* name) {
    for (const MatSpec& m : kMats)
        if (std::string(m.name) == name) return m;
    return kMats[0];
}

MaterialPtr mat_visual(const MatSpec& m) {
    auto& A = SharedAssets::get();
    auto v = std::make_shared<Material>(*A.metal);
    v->color = m.color;
    v->specular = m.spec;
    v->gloss = m.gloss;
    v->reflect = m.glass ? 0.25f : (m.spec > 0.7f ? 0.15f : 0.0f);
    v->double_sided = true;
    v->diffuse = m.tex == 1 ? A.tex_wood : m.tex == 2 ? A.tex_fabric : m.tex == 3 ? A.tex_concrete : nullptr;
    if (m.glass) {
        v->blend = true;
        v->alpha_ref = 0.02f;
        v->cast_shadow = false;
    }
    v->name = m.name;
    return v;
}

// yield / fracture strains on the (stability clamped) beam stiffness, as for the plates; brittle materials get a
// random flaw per link
void apply_mat(DynamicObject& o, const MatSpec& m, bool bulk, uint32_t seed) {
    phys::SoftBody& b = *o.body;
    b.ductile = m.ductile;
    const float ys = bulk ? m.bulk_yield : m.yield, bs = bulk ? m.bulk_brk : m.brk;
    Rng rng(seed * 2654435761u + 7);
    for (phys::Beam& bm : b.beams) {
        if (bm.type == phys::BT_ROPE) continue;
        float f = m.flaw > 0 ? 1.0f + m.flaw * rng.range(-1.0f, 1.0f) : 1.0f;
        bm.max_pos = ys >= 1000 ? 1e12f : bm.k * bm.L0 * ys * f;
        bm.max_neg = -bm.max_pos;
        bm.strength = bs >= 1000 ? 1e12f : bm.k * bm.L0 * bs * f;
    }
}

// Sheets are triangle elements (phys::Shell): overloaded triangles are bisected into finer ones, the finest ones crack
// apart by splitting nodes. Stiffness and damping are capped by the stability budget (bend 1e9 = as stiff as the
// budget allows); strains are total strains from the authored size.

phys::ShellMaterial sheet_material(const std::string& name) {
    phys::ShellMaterial m;
    m.max_level = 4;
    m.min_edge = 0.02f;
    m.bend_damp = 2.0f;
    m.min_piece = 20; // ductile: tears in flaps (no crack may cut off fewer finest triangles: no dust, fewer bodies)
    if (name == "Glass" || name == "Acrylic" || name == "Plywood") m.min_piece = 10; // brittle: shards
    if (name == "Steel") {
        m.k = 1e7f, m.damp = 200, m.yield = 0.004f, m.brk = 0.35f, m.refine = 0.3f, m.refine_yield = 0.02f;
        m.bend = 1e9f, m.bend_yield = 0.1f, m.refine_angle = 0.35f;
    } else if (name == "Aluminium") {
        m.k = 1e7f, m.damp = 120, m.yield = 0.003f, m.brk = 0.22f, m.refine = 0.3f, m.refine_yield = 0.015f;
        m.bend = 1e9f, m.bend_yield = 0.08f, m.refine_angle = 0.35f;
    } else if (name == "Lead") {
        m.k = 4e5f, m.damp = 150, m.yield = 0.006f, m.brk = 0.45f, m.refine = 0.3f, m.refine_yield = 0.05f;
        m.bend = 300, m.bend_yield = 0.03f, m.refine_angle = 0.35f;
    } else if (name == "Glass") {
        m.k = 8e6f, m.damp = 80, m.brk = 0.01f, m.refine = 0.7f, m.flaw = 0.3f;
        m.bend = 1e9f, m.bend_break = 0.15f, m.refine_angle = 0.12f;
    } else if (name == "Acrylic") {
        m.k = 1.5e6f, m.damp = 40, m.brk = 0.05f, m.refine = 0.4f, m.flaw = 0.2f;
        m.bend = 1e9f, m.bend_break = 0.3f, m.refine_angle = 0.15f;
    } else if (name == "Plywood") {
        m.k = 3e6f, m.damp = 80, m.yield = 0.012f, m.brk = 0.025f, m.refine = 0.4f, m.flaw = 0.3f;
        m.bend = 1e9f, m.bend_yield = 0.15f, m.bend_break = 0.25f, m.refine_angle = 0.12f;
    } else if (name == "Plastic") { // (polypropylene: a bumper's cover; it bends far, keeps some of the dent, tears late)
        m.k = 1.5e6f, m.damp = 40, m.yield = 0.03f, m.brk = 0.3f, m.refine = 0.4f, m.refine_yield = 0.1f;
        m.bend = 1e9f, m.bend_yield = 0.25f, m.refine_angle = 0.5f;
    } else if (name == "Rubber") {
        m.k = 1.5e5f, m.damp = 10, m.brk = 2.5f, m.refine = 0.3f;
        m.bend = 5, m.refine_angle = 0.5f;
    } else if (name == "Cardboard") {
        m.k = 4e5f, m.damp = 20, m.yield = 0.01f, m.brk = 0.06f, m.refine = 0.4f, m.flaw = 0.2f;
        m.bend = 20, m.bend_yield = 0.05f, m.refine_angle = 0.3f;
    } else { // Fabric
        m.k = 4e4f, m.damp = 30, m.brk = 0.25f, m.refine = 0.5f, m.tension_only = true;
        m.bend = 0, m.refine_angle = 1e9f;
    }
    // fracture patterns (phys/shell_pattern.h): glass and acrylic crack in a web round the point of impact, metals
    // punch a ring with radial tears, plywood splits along its grain
    // (a rag of fabric, a strip of rubber or cardboard stays soft after it tears off; every other piece is a rigid shard)
    if (name == "Fabric" || name == "Rubber" || name == "Cardboard") m.rigid_pieces = false;
    if (name == "Glass") m.pattern = phys::ShellPattern::Radial, m.pattern_speed = 3.0f, m.pattern_size = 0.5f;
    else if (name == "Acrylic") m.pattern = phys::ShellPattern::Radial, m.pattern_speed = 4.0f, m.pattern_size = 0.4f, m.pattern_strong = 1.4f;
    else if (name == "Steel") m.pattern = phys::ShellPattern::Punch, m.pattern_speed = 8.0f, m.pattern_size = 0.14f, m.pattern_weak = 0.6f, m.pattern_strong = 1.3f;
    else if (name == "Aluminium") m.pattern = phys::ShellPattern::Punch, m.pattern_speed = 7.0f, m.pattern_size = 0.16f, m.pattern_weak = 0.6f, m.pattern_strong = 1.3f;
    else if (name == "Plywood") m.pattern = phys::ShellPattern::Grain, m.pattern_speed = 5.0f, m.pattern_size = 0.2f, m.grain_ratio = 2.5f;
    return m;
}

MaterialPtr fabric_visual() {
    auto& A = SharedAssets::get();
    auto m = std::make_shared<Material>(*A.red);
    m->diffuse = A.tex_fabric;
    m->color = vec4(1.0f);
    m->specular = 0.05f;
    m->double_sided = true;
    return m;
}

void scene_materials(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(181, 221, 1.0f, vec2(-90, -110));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 3.0f, 50.0f, 3, 17);
    te::flatten_rect(hf, vec2(-8, -20), vec2(66, 80), 0, 0.0f, 16, SURF_CONCRETE);
    g.finish_terrain();
    const vec4 kTitle(1.0f, 0.78f, 0.35f, 1.0f);

    // ---- row A: sheets on stands, a 60 kg steel ball dropped from 6 m onto each
    const char* sheet_mats[] = {"Steel", "Aluminium", "Lead", "Glass", "Acrylic", "Plywood", "Rubber", "Cardboard", "Fabric"};
    const float zA = -40.0f, hA = 1.3f, sA = 1.7f;
    std::vector<vec3> ball_at;
    for (int i = 0; i < 9; i++) {
        const float x = -24.0f + i * 6.0f;
        for (int cx : {-1, 1})
            for (int cz : {-1, 1})
                g.add_static_box(vec3(x + cx * (sA * 0.5f + 0.06f), hA * 0.5f, zA + cz * (sA * 0.5f + 0.06f)), vec3(0.06f, hA * 0.5f, 0.06f), quat(),
                                 SURF_METAL, A.dark_wood);
        std::string name = sheet_mats[i];
        SheetDesc sd;
        sd.center = vec3(x, hA, zA);
        sd.u = vec3(1, 0, 0);
        sd.v = vec3(0, 0, -1); // front faces up
        sd.width = sd.height = sA;
        sd.nu = sd.nv = 14;
        sd.clamp = 1;
        sd.mat = sheet_material(name);
        sd.seed = 100 + i;
        if (name == "Fabric") {
            sd.mass = 0.3f * sA * sA * 3.0f;
            sd.thickness = 0.004f;
            sd.visual = fabric_visual();
            sd.uv_scale = 3.0f;
        } else {
            const MatSpec& m = mat_spec(name.c_str());
            sd.mass = m.sheet_kg_m2 * sA * sA;
            sd.thickness = std::clamp(m.sheet_kg_m2 / m.density, 0.004f, 0.03f);
            sd.visual = mat_visual(m);
        }
        g.add_object(build_sheet(g.world, sd, "sheet: " + name));
        g.labels.push_back({vec3(x, hA + 2.2f, zA), name});
        ball_at.push_back(vec3(x, hA + 6.0f, zA));
    }
    g.labels.push_back({vec3(-24.0f - 5.5f, 3.2f, zA), "SHEETS: 60 kg ball from 6 m", kTitle});

    // ---- row B: shapes: a cube dropped from 5 m, a ball from 8 m, a cylinder crushed by a 1.5 t press
    const char* shape_mats[] = {"Steel", "Aluminium", "Lead", "Plywood", "Rubber", "Foam", "Glass", "Concrete"};
    const float zB = -18.0f;
    struct Drop {
        int kind; // 0 cube, 1 ball, 2 press block
        vec3 at;
        int mat;
    };
    std::vector<Drop> shape_drops;
    for (int i = 0; i < 8; i++) {
        const float x = -21.0f + i * 6.0f;
        const MatSpec& m = mat_spec(shape_mats[i]);
        // cylinder standing on the ground, the press comes down on it
        LatheDesc ld;
        ld.axis = vec3(0, 1, 0);
        ld.base = vec3(x + 1.5f, 0.02f, zB);
        ld.profile = {{0.0f, 0.3f}, {0.35f, 0.3f}, {0.7f, 0.3f}};
        ld.segments = 12;
        ld.mass = m.density * kPi * 0.3f * 0.3f * 0.7f;
        ld.beams = {m.bulk_k, m.damp, 1e12f, 1e12f, 0.0f};
        ld.side_mat = ld.cap_mat = mat_visual(m);
        ld.u_repeat = 2.0f;
        auto cyl = build_lathe(g.world, ld, std::string("cylinder: ") + m.name);
        apply_mat(*cyl, m, true, 200 + i);
        g.add_object(std::move(cyl));
        shape_drops.push_back({0, vec3(x - 1.5f, 5.0f, zB), (int)(&m - kMats)});
        shape_drops.push_back({1, vec3(x, 8.0f, zB), (int)(&m - kMats)});
        shape_drops.push_back({2, vec3(x + 1.5f, 4.0f, zB), (int)(&m - kMats)});
        g.labels.push_back({vec3(x, 3.0f, zB), m.name});
    }
    g.labels.push_back({vec3(-21.0f - 5.5f, 3.4f, zB), "SHAPES: cube 5 m, ball 8 m, press 1.5 t", kTitle});

    // ---- row C: bars on two supports, 300 kg dropped from 3 m on the middle
    const float zC = 2.0f;
    std::vector<vec3> weight_at;
    for (int i = 0; i < 8; i++) {
        const float x = -21.0f + i * 6.0f;
        const MatSpec& m = mat_spec(shape_mats[i]);
        for (int sd : {-1, 1}) g.add_static_box(vec3(x + sd * 1.35f, 0.4f, zC), vec3(0.25f, 0.4f, 0.35f), quat(), SURF_CONCRETE, A.concrete);
        SoftBoxDesc bd;
        bd.center = vec3(x, 0.8f + 0.09f, zC);
        bd.size = vec3(3.2f, 0.18f, 0.18f);
        bd.nx = 14;
        bd.ny = bd.nz = 2;
        bd.mass = m.density * 3.2f * 0.18f * 0.18f;
        bd.beams = {m.bulk_k, m.damp, 1e12f, 1e12f, 0.0f};
        bd.mat = mat_visual(m);
        auto bar = build_soft_box(g.world, bd, std::string("bar: ") + m.name);
        apply_mat(*bar, m, true, 300 + i);
        g.add_object(std::move(bar));
        weight_at.push_back(vec3(x, 0.89f + 3.0f, zC));
        g.labels.push_back({vec3(x, 2.4f, zC), m.name});
    }
    g.labels.push_back({vec3(-21.0f - 5.5f, 2.8f, zC), "BARS: 300 kg from 3 m", kTitle});

    // ---- driving lane: gates with a panel clamped in the frame, break through them (or drive around)
    const char* lane_mats[] = {"Cardboard", "Fabric", "Plywood", "Acrylic", "Glass", "Rubber", "Aluminium", "Steel"};
    const float xL = -45.0f;
    for (int i = 0; i < 8; i++) {
        const float z = 20.0f - i * 14.0f;
        for (int sd : {-1, 1}) g.add_static_box(vec3(xL + sd * 2.12f, 1.6f, z), vec3(0.1f, 1.6f, 0.1f), quat(), SURF_METAL, A.metal);
        g.add_static_box(vec3(xL, 3.19f, z), vec3(2.22f, 0.08f, 0.1f), quat(), SURF_METAL, A.metal);
        std::string name = lane_mats[i];
        SheetDesc sd;
        sd.center = vec3(xL, 1.75f, z);
        sd.u = vec3(1, 0, 0);
        sd.v = vec3(0, 1, 0);
        sd.width = 4.0f;
        sd.height = 2.7f;
        sd.nu = 16;
        sd.nv = 11; // (square cells: 4.0 / 15 ~ 2.7 / 10)
        sd.clamp = 2;
        sd.mat = sheet_material(name);
        sd.seed = 400 + i;
        if (name == "Fabric") {
            sd.mass = 3.0f;
            sd.thickness = 0.004f;
            sd.visual = fabric_visual();
            sd.uv_scale = 3.0f;
        } else {
            const MatSpec& m = mat_spec(name.c_str());
            sd.mass = m.sheet_kg_m2 * 4.0f * 2.7f;
            sd.thickness = std::clamp(m.sheet_kg_m2 / m.density, 0.004f, 0.03f);
            sd.visual = mat_visual(m);
        }
        g.add_object(build_sheet(g.world, sd, "panel: " + name));
        g.labels.push_back({vec3(xL, 3.8f, z), name});
    }
    g.labels.push_back({vec3(xL, 4.6f, 30.0f), "PANELS IN GATES: shoot them (B)", kTitle});
    // no vehicle here: the free camera looks over all the rows (WASD / QE to fly, right mouse to look)
    g.no_player_vehicle = true;
    g.set_spawn(vec3(0, 0, 30.0f), 180.0f);
    g.cam.look_free(vec3(-2.0f, 13.0f, 24.0f), vec3(-2.0f, 0.0f, -22.0f));

    // ---- the tests run one after the other (Scene menu: again)
    struct Tests {
        double t0 = 0;
        int stage = 0;
    };
    auto tests = std::make_shared<Tests>();
    auto spawn = [ball_at, shape_drops, weight_at](Game& gg, int stage) {
        auto& A2 = SharedAssets::get();
        if (stage == 0)
            for (size_t i = 0; i < ball_at.size(); i++) {
                SoftSphereDesc d;
                d.center = ball_at[i];
                d.radius = 0.22f;
                d.subdiv = 1;
                d.mass = 60;
                d.mat = A2.metal;
                d.beams = {3e6f, 400, 1e12f, 1e12f, 0};
                gg.add_object(build_soft_sphere(gg.world, d, format("test ball %zu", i)));
            }
        if (stage == 1)
            for (const auto& dr : shape_drops) {
                const MatSpec& m = kMats[dr.mat];
                if (dr.kind == 0) {
                    SoftBoxDesc d;
                    d.center = dr.at;
                    d.size = vec3(0.6f);
                    d.nx = d.ny = d.nz = 3;
                    d.mass = m.density * 0.216f;
                    d.rot = quat::axis_angle(normalize(vec3(1, 0.3f, 0.6f)), 0.5f); // lands on an edge
                    d.beams = {m.bulk_k, m.damp, 1e12f, 1e12f, 0.0f};
                    d.mat = mat_visual(m);
                    auto o = build_soft_box(gg.world, d, std::string("cube: ") + m.name);
                    apply_mat(*o, m, true, 500 + dr.mat);
                    gg.add_object(std::move(o));
                } else if (dr.kind == 1) {
                    SoftSphereDesc d;
                    d.center = dr.at;
                    d.radius = 0.35f;
                    d.subdiv = 2;
                    d.mass = m.density * 4.0f / 3.0f * kPi * 0.35f * 0.35f * 0.35f;
                    d.beams = {m.bulk_k, m.damp, 1e12f, 1e12f, 0.0f};
                    d.mat = mat_visual(m);
                    auto o = build_soft_sphere(gg.world, d, std::string("ball: ") + m.name);
                    apply_mat(*o, m, true, 600 + dr.mat);
                    gg.add_object(std::move(o));
                } else {
                    SoftBoxDesc d;
                    d.center = dr.at;
                    d.size = vec3(1.0f, 0.5f, 1.0f);
                    d.nx = d.nz = 3;
                    d.ny = 2;
                    d.mass = 1500;
                    d.beams = {8e6f, 3000, 1e12f, 1e12f, 0.0f};
                    d.mat = A2.rust;
                    gg.add_object(build_soft_box(gg.world, d, "press"));
                }
            }
        if (stage == 2)
            for (size_t i = 0; i < weight_at.size(); i++) {
                SoftBoxDesc d;
                d.center = weight_at[i];
                d.size = vec3(0.45f);
                d.mass = 300;
                d.beams = {6e6f, 2000, 1e12f, 1e12f, 0.0f};
                d.mat = A2.rust;
                gg.add_object(build_soft_box(gg.world, d, format("bar weight %zu", i)));
            }
    };
    g.scene_actions.push_back({"Drop the test weights again", [tests](Game& gg) {
                                   tests->stage = 0;
                                   tests->t0 = gg.world.time();
                               }});
    g.scene_update = [tests, spawn](Game& gg, float) {
        const double t = gg.world.time() - tests->t0;
        const double at[3] = {1.0, 5.0, 10.0};
        while (tests->stage < 3 && t >= at[tests->stage]) spawn(gg, tests->stage++);
        const char* names[3] = {"sheets", "shapes", "bars"};
        gg.scene_status = tests->stage < 3 ? format("Next test: %s in %.0f s", names[tests->stage], at[tests->stage] - t)
                                           : "Tests done. Scene menu: drop the weights again; F5 resets the samples.";
    };
    g.scene_hint = "Materials lab. The same tests for every material: sheets (steel, aluminium, lead, glass, acrylic, plywood, rubber, "
                   "cardboard, fabric) take a 60 kg ball from 6 m; cubes, balls and cylinders (steel ... concrete) are dropped and "
                   "pressed; bars take 300 kg from 3 m. The sheets are triangle elements: where they are overloaded the triangles "
                   "split into finer ones and cracks open between them, torn-off pieces fall as bodies of their own (F3 shows the "
                   "mesh). On the left: 8 panels clamped in gates. B shoots at anything, X destroys, G drags. Free camera: WASD, "
                   "Q/E, right mouse; F5 reruns the tests and keeps the camera.";
}

// ------------------------------------------------------------------------------------------- sheet run
// Profiling / crash test: a straight lane with three lead sheets (6 mm) clamped in gates; drive (or launch) through
// them.
void scene_sheet_run(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(81, 361, 1.0f, vec2(-40, -60));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 2.0f, 50.0f, 3, 31);
    te::flatten_rect(hf, vec2(0, 110), vec2(14, 175), 0, 0.0f, 12, SURF_CONCRETE);
    g.finish_terrain();
    const MatSpec& lead = mat_spec("Lead");
    const float w = 3.6f, h = 2.4f;
    for (int i = 0; i < 3; i++) {
        const float z = 60.0f + i * 30.0f;
        for (int sd : {-1, 1}) g.add_static_box(vec3(sd * (w * 0.5f + 0.12f), 1.4f, z), vec3(0.1f, 1.4f, 0.1f), quat(), SURF_METAL, A.metal);
        g.add_static_box(vec3(0, 2.85f, z), vec3(w * 0.5f + 0.22f, 0.08f, 0.1f), quat(), SURF_METAL, A.metal);
        SheetDesc sd;
        sd.center = vec3(0, 0.2f + h * 0.5f, z);
        sd.u = vec3(1, 0, 0);
        sd.v = vec3(0, 1, 0);
        sd.width = w;
        sd.height = h;
        sd.nu = 15;
        sd.nv = 11; // (square cells: 3.6 / 14 ~ 2.4 / 10)
        sd.clamp = 2;
        sd.mat = sheet_material("Lead");
        sd.mass = lead.sheet_kg_m2 * w * h;
        sd.thickness = std::clamp(lead.sheet_kg_m2 / lead.density, 0.004f, 0.03f);
        sd.visual = mat_visual(lead);
        sd.seed = 700 + i;
        g.add_object(build_sheet(g.world, sd, format("lead sheet %d", i + 1)));
        g.labels.push_back({vec3(0, 3.4f, z), format("LEAD 6 mm #%d", i + 1)});
    }
    add_cone_line(g, vec3(-4, 0, 20), vec3(-4, 0, 150), 10);
    add_cone_line(g, vec3(4, 0, 20), vec3(4, 0, 150), 10);
    g.set_spawn(vec3(0, 0, 0), 0);
    g.scene_hint = "Three lead sheets (6 mm, 590 kg each) clamped in gates, 30 m apart. Drive through them or use Scene > Launch. "
                   "F2: performance widget with the physics breakdown (sheet forces, cracks, contacts, the heaviest island).";
}

// ------------------------------------------------------------------------------------------- sheet shapes
// Triangle-element sheets of other shapes and sizes: a 6 x 4 m steel gate across the road, an aluminium half-pipe to
// drive through, panels of four shapes on stands along the road, an acrylic dome at the end. For the balls (B) and
// the laser (L).
void scene_sheet_shapes(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(81, 221, 1.0f, vec2(-40, -40));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 2.0f, 50.0f, 3, 41);
    te::flatten_rect(hf, vec2(0, 60), vec2(18, 130), 0, 0.0f, 12, SURF_CONCRETE);
    g.finish_terrain();
    const vec4 kTitle(1.0f, 0.78f, 0.35f, 1.0f);
    auto sheet = [&](SheetDesc sd, const char* mat, const std::string& name) {
        const MatSpec& m = mat_spec(mat);
        sd.mat = sheet_material(mat);
        sd.kg_m2 = m.sheet_kg_m2;
        sd.thickness = std::clamp(m.sheet_kg_m2 / m.density, 0.004f, 0.03f);
        sd.visual = mat_visual(m);
        g.add_object(build_sheet(g.world, sd, name));
    };
    // the gate: 6 x 4 m of 6 mm steel (1100 kg), top and sides held
    {
        const float w = 6.0f, h = 4.0f, z = 40.0f;
        for (int sd : {-1, 1}) g.add_static_box(vec3(sd * (w * 0.5f + 0.14f), 2.2f, z), vec3(0.12f, 2.2f, 0.12f), quat(), SURF_METAL, A.metal);
        g.add_static_box(vec3(0, 4.45f, z), vec3(w * 0.5f + 0.26f, 0.1f, 0.12f), quat(), SURF_METAL, A.metal);
        SheetDesc sd;
        sd.center = vec3(0, 0.3f + h * 0.5f, z);
        sd.width = w;
        sd.height = h;
        sd.nu = 25;
        sd.nv = 17; // (0.25 m cells)
        sd.clamp = 2;
        sd.seed = 801;
        sheet(sd, "Steel", "steel gate 6x4");
        g.labels.push_back({vec3(0, 5.1f, z), "STEEL 6 x 4 m", kTitle});
    }
    // the half-pipe: 2 mm aluminium bent round the road (radius 2.6 m, 6 m long), held along its edges
    {
        const float r = 2.6f, z = 75.0f;
        SheetDesc sd;
        sd.center = vec3(0, 0.05f + r, z);
        sd.u = vec3(1, 0, 0);
        sd.v = vec3(0, 0, 1); // (front normal down: the arch's inside)
        sd.width = 3.14159265f * r;
        sd.height = 6.0f;
        sd.nu = 27;
        sd.nv = 20;
        sd.curve = r;
        sd.clamp = 1;
        sd.seed = 802;
        sheet(sd, "Aluminium", "aluminium half-pipe");
        g.labels.push_back({vec3(0, 2 * r + 0.8f, z - 3.0f), "ALUMINIUM HALF-PIPE", kTitle});
    }
    // panels on stands along the road, facing it: lead disc, glass ring, plywood triangle, aluminium L
    struct Panel {
        const char* mat;
        phys::SheetShape shape;
        int clamp;
        float x, z;
        const char* name;
        const char* label;
    };
    const Panel panels[] = {
        {"Lead", phys::SheetShape::Disc, 1, -8.0f, 16.0f, "lead disc", "LEAD DISC"},
        {"Glass", phys::SheetShape::Ring, 1, 8.0f, 16.0f, "glass ring", "GLASS RING"},
        {"Plywood", phys::SheetShape::Triangle, 2, -8.0f, 26.0f, "plywood triangle", "PLYWOOD TRIANGLE"},
        {"Aluminium", phys::SheetShape::LShape, 3, 8.0f, 26.0f, "aluminium L", "ALUMINIUM L"},
    };
    for (const Panel& pn : panels) {
        const float s = 2.4f, y = 1.9f;
        const float face = pn.x < 0 ? 1.0f : -1.0f; // (the front towards the road)
        for (int e : {-1, 1}) g.add_static_box(vec3(pn.x, 1.6f, pn.z + e * (s * 0.5f + 0.1f)), vec3(0.07f, 1.6f, 0.07f), quat(), SURF_METAL, A.dark_wood);
        g.add_static_box(vec3(pn.x, 3.25f, pn.z), vec3(0.07f, 0.07f, s * 0.5f + 0.17f), quat(), SURF_METAL, A.dark_wood);
        SheetDesc sd;
        sd.center = vec3(pn.x, y, pn.z);
        sd.u = vec3(0, 0, -face);
        sd.v = vec3(0, 1, 0);
        sd.width = sd.height = s;
        sd.nu = sd.nv = 17; // (0.15 m cells)
        sd.shape = pn.shape;
        sd.clamp = pn.clamp;
        sd.seed = 810 + (uint32_t)(&pn - panels);
        sheet(sd, pn.mat, pn.name);
        g.labels.push_back({vec3(pn.x, 3.8f, pn.z), pn.label});
    }
    // the dome: 4 mm acrylic, a spherical cap 4 m across (radius 2.6 m), its rim on the ground
    {
        const float r = 2.6f, a = 2.0f, z = 110.0f;
        SheetDesc sd;
        sd.center = vec3(0, 0.05f + r * (1.0f - std::cos(a / r)), z);
        sd.u = vec3(1, 0, 0);
        sd.v = vec3(0, 0, -1); // (front up)
        sd.width = sd.height = 2 * a;
        sd.nu = sd.nv = 21;
        sd.shape = phys::SheetShape::Disc;
        sd.dome = r;
        sd.clamp = 1;
        sd.seed = 820;
        sheet(sd, "Acrylic", "acrylic dome");
        g.labels.push_back({vec3(0, 2.2f, z), "ACRYLIC DOME", kTitle});
    }
    add_cone_line(g, vec3(-4.5f, 0, 5), vec3(-4.5f, 0, 125), 12);
    add_cone_line(g, vec3(4.5f, 0, 5), vec3(4.5f, 0, 125), 12);
    g.set_spawn(vec3(0, 0, 0), 0);
    g.scene_hint = "Sheets of other shapes and sizes: a 6 x 4 m steel gate, an aluminium half-pipe, panels (lead disc, glass ring, plywood "
                   "triangle, aluminium L), an acrylic dome. B: shoot balls; L: laser, hold the left button and sweep to cut along the cursor.";
}

// ------------------------------------------------------------------------------------------- FEM shells
// Triangle elements of the FEM frame (phys::FrameTri: a co-rotational thin shell, membrane and Kirchhoff bending, in
// the frame's implicit step) in three prototypes on a concrete pad: a steel sheet of 3 mm, 2 x 2 m, lying across two
// supports; a cantilever, 1.5 x 0.5 m of 8 mm steel clamped in a concrete block; a hollow steel cube of 1 m, 2 mm,
// 5 x 5 cells a face. The Scene menu drops balls and slabs on them, throws the cube about.

MaterialPtr fem_plate_visual(vec3 color) {
    auto vis = mat_visual(mat_spec("Steel"));
    vis->color = vec4(color, 1.0f);
    vis->specular = 0.5f;
    return vis;
}

AxeSwing s_fs_axe; // (the giant axe through the sheet)

void fem_shells_clear(Game& g) {
    std::vector<DynamicObject*> gone;
    for (auto& o : g.objects)
        if (o->name.rfind("fem ", 0) == 0 || o->name.rfind("weight", 0) == 0) gone.push_back(o.get());
    for (DynamicObject* o : gone) g.remove_object(o);
    s_fs_axe = AxeSwing();
}

SoftBody* fem_object(Game& g, const char* name) {
    for (auto& o : g.objects)
        if (o->name == name && o->body) return o->body;
    return nullptr;
}

void add_fem_sheet(Game& g, float y, vec3 v = vec3(0), vec3 at = vec3(0), const char* name = "fem sheet") {
    FemPlateDesc d;
    d.origin = at + vec3(-1, y, -1), d.du = vec3(2.0f / 16, 0, 0), d.dv = vec3(0, 0, 2.0f / 16), d.nu = d.nv = 16;
    d.thickness = 0.003f;
    d.visual = fem_plate_visual(vec3(0.62f, 0.64f, 0.67f));
    d.velocity = v;
    g.add_object(build_fem_plate(g.world, d, name));
}

// the axe's bench: the same sheet on two blocks with a slot 30 cm wide between them along z, the axe's way (across
// the sheet's supports its halves fell into the gap under the blade and it carried them off)
const vec3 kAxeBench(0, 0, 7.0f);

const vec3 kCantRoot(6.0f, 1.2f, -0.25f);
// the cantilever, with a box of `load` kg welded under its tip: a steel box 0.125 x 0.5 x 0.25 m (5 mm) on the last
// row of cells, open at the top (the tip is its lid), filled with lead (the mass on its nodes)
void add_fem_cantilever(Game& g, float load = 0) {
    FemPlateDesc d;
    d.origin = kCantRoot, d.du = vec3(1.5f / 12, 0, 0), d.dv = vec3(0, 0, 0.5f / 4), d.nu = 12, d.nv = 4;
    d.thickness = 0.008f;
    d.damping = 1e-3f; // (bolted at its root: about 1 % of critical in its first mode, not the bare steel's 0.1 %)
    d.fixed = [](vec3 p) { return p.x < kCantRoot.x + 1e-3f; };
    d.visual = fem_plate_visual(vec3(0.20f, 0.36f, 0.62f));
    if (load > 0)
        d.more = [load](phys::SoftBody& b, phys::ShellMesher& m, uint16_t) {
            const uint16_t s = b.fem.add_shell_section(phys::make_shell_section("Steel", 0.005f));
            const vec3 o = kCantRoot + vec3(1.5f - 0.125f, -0.25f, 0), X(0.125f, 0, 0), Y(0, 0.125f, 0), Z(0, 0, 0.125f);
            std::vector<uint32_t> box;
            for (const auto& ns : {m.grid(o, Z, X, 4, 1, s), m.grid(o, X, Y, 1, 2, s), m.grid(o + Z * 4.0f, Y, X, 2, 1, s), m.grid(o, Y, Z, 2, 4, s),
                                   m.grid(o + X, Z, Y, 4, 2, s)})
                box.insert(box.end(), ns.begin(), ns.end());
            std::sort(box.begin(), box.end());
            box.erase(std::unique(box.begin(), box.end()), box.end());
            for (uint32_t i : box) b.nodes[i].mass += load / (float)box.size();
        };
    g.add_object(build_fem_plate(g.world, d, "fem cantilever"));
}

const vec3 kCubeAt(-6.0f, 0.51f, 0.0f);
void add_fem_cube(Game& g, vec3 at, const quat& rot = quat(), vec3 v = vec3(0), vec3 spin = vec3(0), float lid_load = 0) {
    FemBoxDesc d;
    d.center = at, d.rot = rot, d.velocity = v, d.spin = spin, d.lid_load = lid_load;
    d.n = 5, d.thickness = 0.002f;
    d.visual = fem_plate_visual(vec3(0.78f, 0.30f, 0.12f));
    g.add_object(build_fem_box(g.world, d, "fem cube"));
}

void remove_named(Game& g, const std::string& name) {
    std::vector<DynamicObject*> gone;
    for (auto& o : g.objects)
        if (o->name == name) gone.push_back(o.get());
    for (DynamicObject* o : gone) g.remove_object(o);
}

void add_weight(Game& g, vec3 at, vec3 size, float mass) {
    SoftBoxDesc d;
    d.center = at, d.size = size;
    d.nx = d.ny = d.nz = 2;
    d.mass = mass;
    d.beams = {1e9f, 4e4f, 1e12f, 1e12f, 0.0f};
    d.mat = SharedAssets::get().concrete;
    g.add_object(build_soft_box(g.world, d, "weight"));
}

void scene_fem_shells(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(81, 81, 1.0f, vec2(-40, -40));
    auto& hf = g.world.statics.terrain;
    te::flatten_rect(hf, vec2(0, 0), vec2(34, 34), 0, 0.0f, 4, SURF_CONCRETE);
    g.finish_terrain();
    // the sheet's two supports, the cantilever's block, a wall to throw the cube at
    for (float x : {-0.85f, 0.85f}) g.add_static_box(vec3(x, 0.4f, 0), vec3(0.12f, 0.4f, 1.3f), quat(), SURF_CONCRETE, A.concrete);
    for (float x : {-0.6f, 0.6f}) g.add_static_box(kAxeBench + vec3(x, 0.4f, 0), vec3(0.45f, 0.4f, 1.3f), quat(), SURF_CONCRETE, A.concrete);
    g.add_static_box(vec3(kCantRoot.x - 0.5f, 0.9f, 0), vec3(0.5f, 0.9f, 0.6f), quat(), SURF_CONCRETE, A.concrete);
    g.add_static_box(vec3(-12.0f, 1.0f, 0), vec3(0.3f, 1.0f, 3.0f), quat(), SURF_CONCRETE, A.concrete);
    g.labels.push_back({vec3(0, 1.9f, 0), "SHEET 2 x 2 m, 3 mm"});
    g.labels.push_back({kAxeBench + vec3(0, 1.9f, 0), "AXE BENCH"});
    g.labels.push_back({vec3(6.7f, 2.2f, 0), "CANTILEVER 1.5 m, 8 mm"});
    g.labels.push_back({vec3(-6, 1.9f, 0), "CUBE 1 m, 2 mm"});
    g.no_player_vehicle = true;
    g.set_spawn(vec3(0, 0, -9), 0);
    s_fs_axe = AxeSwing();
    auto all = [](Game& gg) { add_fem_sheet(gg, 0.82f), add_fem_cantilever(gg), add_fem_cube(gg, kCubeAt); };
    all(g);
    // the axe's cut, each frame; the sheet's parts (the two largest' shares of its mass) and its fastest node
    g.scene_update = [](Game& gg, float dt) {
        if (!s_fs_axe.obj) return;
        s_fs_axe.step(gg, dt);
        gg.scene_status = s_fs_axe.status();
        if (const SoftBody* sh = fem_object(gg, "fem bench sheet")) {
            float p0, p1, fast = 0;
            sh->largest_parts(p0, p1);
            for (const Node& n : sh->nodes) fast = std::max(fast, length(n.v));
            gg.scene_status += format(" | sheet: parts %.0f%% %.0f%%, %zu triangles, %d torn, %d bisections, fastest %.1f m/s", 100.0f * p0, 100.0f * p1,
                                      sh->fem.tris.size(), sh->fem.tris_torn, sh->fem.tris_refined, fast);
        }
    };
    auto test = [&](const char* label, std::function<void(Game&)> f) {
        g.scene_actions.push_back({label, [all, f](Game& gg) {
                                       fem_shells_clear(gg);
                                       all(gg);
                                       f(gg);
                                   }});
    };
    test("Reset", [](Game&) {});
    test("Drop a 40 kg steel ball on the sheet from 3 m", [](Game& gg) {
        gg.projectile_kind = 0;
        gg.projectile_speed = std::sqrt(2.0f * 9.81f * 3.0f);
        gg.shoot(vec3(0.3f, 0.82f + 0.25f + 2.0f, 0.2f), vec3(0, -1, 0));
    });
    test("Drop a 40 kg steel ball on the sheet from 10 m", [](Game& gg) {
        gg.projectile_kind = 0;
        gg.projectile_speed = std::sqrt(2.0f * 9.81f * 10.0f);
        gg.shoot(vec3(0.0f, 0.82f + 0.25f + 2.0f, 0.0f), vec3(0, -1, 0));
    });
    test("Drop a 500 kg block on the sheet from 1 m", [](Game& gg) { add_weight(gg, vec3(0, 0.82f + 1.2f, 0), vec3(0.5f, 0.4f, 0.5f), 500.0f); });
    // (the cars' axe: 5 t, 8.2 m from its pivot, swinging along z through the bench's slot, through the triangles a
    // little off their grid's middle line: they bisect along the cut)
    test("The giant axe (it swings down and cuts a sheet on the bench in two)", [](Game& gg) {
        add_fem_sheet(gg, 0.82f, vec3(0), kAxeBench, "fem bench sheet");
        AxeDesc d;
        d.pivot = kAxeBench + vec3(0.04f, 8.33f, 0);
        d.yaw = 0.5f * kPi;
        s_fs_axe.make(gg, d, "fem axe");
    });
    test("Weld 100 kg under the cantilever's tip", [](Game& gg) { remove_named(gg, "fem cantilever"), add_fem_cantilever(gg, 100.0f); });
    test("Weld 250 kg under the cantilever's tip", [](Game& gg) { remove_named(gg, "fem cantilever"), add_fem_cantilever(gg, 250.0f); });
    test("Drop 1 t on the cantilever's tip from 0.5 m", [](Game& gg) { add_weight(gg, vec3(7.25f, 1.2f + 0.8f, 0), vec3(0.4f, 0.5f, 0.45f), 1000.0f); });
    test("Drop the cube from 5 m on a face", [](Game& gg) {
        remove_named(gg, "fem cube");
        add_fem_cube(gg, kCubeAt + vec3(0, 5, 0));
    });
    test("Drop the cube from 5 m on a corner", [](Game& gg) {
        remove_named(gg, "fem cube");
        const quat r = quat::axis_angle(normalize(vec3(1, 0, -1)), std::atan(std::sqrt(2.0f)));
        add_fem_cube(gg, kCubeAt + vec3(0, 5.4f, 0), r);
    });
    test("Load the cube's lid with 1 t (spread on it)", [](Game& gg) { remove_named(gg, "fem cube"), add_fem_cube(gg, kCubeAt, quat(), vec3(0), vec3(0), 1000.0f); });
    test("Drop a 1 t slab on the cube from 1 m", [](Game& gg) { add_weight(gg, kCubeAt + vec3(0, 0.5f + 1.0f + 0.15f, 0), vec3(1.4f, 0.3f, 1.4f), 1000.0f); });
    test("Throw the cube at the wall at 50 km/h", [](Game& gg) {
        remove_named(gg, "fem cube");
        add_fem_cube(gg, kCubeAt + vec3(-1.5f, 0.6f, 0), quat(), vec3(-50 / 3.6f, 1.0f, 0), vec3(0, 0, 2.0f));
    });
    g.scene_hint = "Triangle elements of the FEM frame: a thin shell (membrane and Kirchhoff bending, co-rotational) solved implicitly "
                   "with the members. A steel sheet across two supports, a cantilever clamped in a block, a hollow steel cube. Scene "
                   "menu: balls, weights, drops. F3: the elements (orange where yielded), F4 stress.";
}

// ------------------------------------------------------------------------------------------- steel barrels
// 200 l steel drums (build_barrel: a sheet of triangle elements on rings of frame elements at the chimes and the
// rolling hoops) on a concrete pad with a 15 degree ramp and a wall. Scene menu: drops on the bottom, the side and the
// rim from different heights, rolling down the ramp and into the wall, one barrel thrown at another and dropped onto
// another; every test starts from a clear pad.

phys::ShellMaterial barrel_material() {
    // the body sheet's steel (as the cars': yields early and keeps the dent, a long plastic range before it tears)
    phys::ShellMaterial m = sheet_material("Steel");
    m.pattern = phys::ShellPattern::None;
    m.max_level = 2;         // (a dent's detail down to a quarter of the authored triangles' size, at most twice as many)
    m.refine_budget = 2.0f;
    m.min_piece = 30;
    m.yield = std::min(m.yield, 0.006f), m.brk = std::max(m.brk, 0.7f), m.refine = 0.35f, m.refine_yield = 0.06f;
    m.bend_yield = std::min(m.bend_yield, 0.15f), m.refine_angle = 0.6f;
    // the membrane: the yield force per width, drum steel (DC01, ~200 MPa) 1.0 mm thick 2e5 N/m, taken at a quarter:
    // the triangles' 9 cm edges stand for a wall that buckles in folds of a few centimetres and gives long before its
    // plane yields (at the full force a drum thrown at another or dropped on its edge kept its shape); elastic to 0.15%
    m.membrane = 5.0e4f;
    m.yield = 0.0015f;
    // bending: a plate's D = E t^3 / 12 (1 - nu^2) = 19 N m for 1.0 mm, the hinge's 3 D (its stiffness at the authored
    // size: the short steps give it room), yielding early (a 9 cm triangle's hinge stands for a strip that would
    // buckle in waves of a few centimetres: at 0.05 rad, and still at 0.02, drops and knocks left no mark)
    m.bend = 58.0f;
    m.bend_yield = 0.01f;
    m.bend_harden = getenv("BL_BARREL_HARDEN") ? (float)atof(getenv("BL_BARREL_HARDEN")) : 1.0f; // (work hardening: see ShellMaterial)
    m.bend_harden_max = getenv("BL_BARREL_HARDMAX") ? (float)atof(getenv("BL_BARREL_HARDMAX")) : 2.0f;
    if (const char* e = getenv("BL_BARREL_MEMBRANE")) m.membrane = (float)atof(e); // (diagnostics: 0 the springs alone)
    if (const char* e = getenv("BL_BARREL_BENDYIELD")) m.bend_yield = (float)atof(e);
    if (const char* e = getenv("BL_BARREL_BEND")) m.bend = (float)atof(e);
    return m;
}

void barrels_clear(Game& g) {
    std::vector<DynamicObject*> gone;
    for (auto& o : g.objects)
        if (o->name.rfind("barrel", 0) == 0) gone.push_back(o.get());
    for (DynamicObject* o : gone) g.remove_object(o);
}

// a barrel standing (axis up) turned by `tilt` about x, then `turn` about y; `at` the centre
// the scene's barrel type (Scene menu): a sheet alone, or with rings of frame elements (FEM) at the chimes and hoops
bool s_barrel_fem = false;

DynamicObject* add_barrel(Game& g, const std::string& name, vec3 at, const quat& rot, vec3 v = vec3(0), vec3 w = vec3(0), vec3 color = vec3(0.10f, 0.28f, 0.62f),
                          int fem = -1) {
    BarrelDesc d;
    d.frame_rings = fem < 0 ? s_barrel_fem : fem > 0;
    d.center = at;
    d.rot = rot;
    d.velocity = v;
    d.spin = w;
    d.mat = barrel_material();
    auto vis = mat_visual(mat_spec("Steel"));
    vis->color = vec4(color, 1.0f);
    vis->specular = 0.45f;
    d.visual = vis;
    return g.add_object(build_barrel(g.world, d, name));
}

const float kBarrelR = 0.286f, kBarrelH = 0.88f;
const quat kUpright = quat();
// (lying: turned half a segment of the 20 about its axis first, so it lies on a flat between two nodes, not on one)
const quat kHalfSegment = quat::axis_angle(vec3(0, 1, 0), kPi / 20.0f);
const quat kOnSideX = quat::axis_angle(vec3(0, 0, 1), 0.5f * kPi) * kHalfSegment; // (the axis along x: it rolls along z)
const quat kOnSideZ = quat::axis_angle(vec3(1, 0, 0), 0.5f * kPi) * kHalfSegment; // (the axis along z: it rolls along x)
const float kRampA = 15.0f * kDeg2Rad, kRampX = 12.0f, kRampZ = 20.0f, kRampHalf = 4.0f, kRampYc = 1.0f;

float ramp_surface(float z) { // (its top face's height along it)
    return kRampYc + 0.1f / std::cos(kRampA) - (z - kRampZ) * std::tan(kRampA);
}

void scene_barrels(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(161, 161, 1.0f, vec2(-80, -80));
    auto& hf = g.world.statics.terrain;
    te::flatten_rect(hf, vec2(0, 0), vec2(70, 70), 0, 0.0f, 5, SURF_CONCRETE);
    g.finish_terrain();
    g.add_static_box(vec3(kRampX, kRampYc, kRampZ), vec3(2.0f, 0.1f, kRampHalf), quat::axis_angle(vec3(1, 0, 0), kRampA), SURF_CONCRETE, A.concrete);
    g.add_static_box(vec3(-12.3f, 1.0f, 0), vec3(0.3f, 1.0f, 5.0f), quat(), SURF_CONCRETE, A.concrete);
    g.labels.push_back({vec3(kRampX, 3.2f, kRampZ - 3.5f), "RAMP 15 deg"});
    g.labels.push_back({vec3(-12.3f, 2.6f, 0), "WALL"});
    g.no_player_vehicle = true;
    g.set_spawn(vec3(0, 0, -8), 0);
    s_barrel_fem = getenv("BL_BARREL_FEM") != nullptr; // (scripted checks: the scenarios with FEM barrels)
    // a few to play with: a row standing, two lying
    // a few to play with: a row standing, two lying; behind them the same with frame rings (FEM), green
    auto display = [](Game& gg) {
        for (int i = 0; i < 3; i++) add_barrel(gg, "barrel " + std::to_string(i + 1), vec3(-1.0f + i * 0.7f, kBarrelH * 0.5f + 0.01f, 6.0f), kUpright, vec3(0), vec3(0), vec3(0.10f, 0.28f, 0.62f), 0);
        add_barrel(gg, "barrel 4", vec3(3.0f, kBarrelR + 0.01f, 6.0f), kOnSideZ, vec3(0), vec3(0), vec3(0.75f, 0.12f, 0.08f), 0);
        add_barrel(gg, "barrel 5", vec3(4.2f, kBarrelR + 0.01f, 6.0f), kOnSideX, vec3(0), vec3(0), vec3(0.85f, 0.62f, 0.10f), 0);
        const vec3 green(0.16f, 0.42f, 0.22f);
        for (int i = 0; i < 3; i++) add_barrel(gg, "barrel fem " + std::to_string(i + 1), vec3(-1.0f + i * 0.7f, kBarrelH * 0.5f + 0.01f, 8.0f), kUpright, vec3(0), vec3(0), green, 1);
        add_barrel(gg, "barrel fem 4", vec3(3.0f, kBarrelR + 0.01f, 8.0f), kOnSideZ, vec3(0), vec3(0), green, 1);
    };
    display(g);
    g.scene_actions.push_back({"Barrels: the sheet alone", [display](Game& gg) { s_barrel_fem = false, barrels_clear(gg), display(gg); }});
    g.scene_actions.push_back({"Barrels: with FEM rings (chimes and hoops)", [display](Game& gg) { s_barrel_fem = true, barrels_clear(gg), display(gg); }});
    auto test = [&](const char* label, std::function<void(Game&)> f) {
        g.scene_actions.push_back({label, [f](Game& gg) {
                                       barrels_clear(gg);
                                       f(gg);
                                   }});
    };
    test("Tip it over (a push at the top)", [](Game& gg) {
        // standing, turning over its bottom rim at 3.5 rad/s: it falls on its side (and should then lie still)
        const vec3 w(0, 0, 3.5f), pivot(kBarrelR, 0, 0), c(0, kBarrelH * 0.5f + 0.01f, 0);
        add_barrel(gg, "barrel", c, kUpright, cross(w, c - pivot), w);
    });
    test("Drop it on its bottom from 1 m", [](Game& gg) { add_barrel(gg, "barrel", vec3(0, 1.0f + kBarrelH * 0.5f, 0), kUpright); });
    test("Drop it on its bottom from 5 m", [](Game& gg) { add_barrel(gg, "barrel", vec3(0, 5.0f + kBarrelH * 0.5f, 0), kUpright); });
    test("Drop it on its side from 2 m", [](Game& gg) { add_barrel(gg, "barrel", vec3(0, 2.0f + kBarrelR, 0), kOnSideZ); });
    test("Drop it on its side from 10 m", [](Game& gg) { add_barrel(gg, "barrel", vec3(0, 10.0f + kBarrelR, 0), kOnSideZ); });
    test("Drop it on the rim (45 deg) from 2 m", [](Game& gg) {
        const float a = 0.25f * kPi;
        add_barrel(gg, "barrel", vec3(0, 2.0f + kBarrelR * std::sin(a) + 0.5f * kBarrelH * std::cos(a), 0), quat::axis_angle(vec3(1, 0, 0), a));
    });
    test("Roll it down the ramp", [](Game& gg) {
        const float z = kRampZ - kRampHalf * std::cos(kRampA) + 0.6f;
        add_barrel(gg, "barrel", vec3(kRampX, ramp_surface(z) + kBarrelR / std::cos(kRampA) + 0.03f, z), kOnSideX);
    });
    test("Kick it rolling into the wall at 5 m/s", [](Game& gg) {
        add_barrel(gg, "barrel", vec3(-4.0f, kBarrelR + 0.01f, 0), kOnSideZ, vec3(-5, 0, 0), vec3(0, 0, 5.0f / kBarrelR));
    });
    test("Throw one barrel at another at 8 m/s", [](Game& gg) {
        add_barrel(gg, "barrel target", vec3(0, kBarrelH * 0.5f + 0.01f, 0), kUpright);
        add_barrel(gg, "barrel thrown", vec3(-2.5f, 0.55f, 0), kOnSideZ, vec3(8, 0, 0), vec3(0), vec3(0.75f, 0.12f, 0.08f));
    });
    test("Stand one barrel on another", [](Game& gg) { // (the lid under a barrel's weight: it holds, it does not creep)
        add_barrel(gg, "barrel below", vec3(0, kBarrelH * 0.5f + 0.01f, 0), kUpright);
        // (4 cm apart: their spheres overlap, they part slowly and it settles)
        add_barrel(gg, "barrel above", vec3(0, kBarrelH * 1.5f + 0.04f, 0), kUpright, vec3(0), vec3(0), vec3(0.75f, 0.12f, 0.08f));
    });
    test("Drop one barrel onto another from 1.5 m", [](Game& gg) {
        add_barrel(gg, "barrel below", vec3(0, kBarrelH * 0.5f + 0.01f, 0), kUpright);
        add_barrel(gg, "barrel above", vec3(0.08f, kBarrelH + 1.5f + kBarrelH * 0.5f, 0), quat::axis_angle(vec3(0, 0, 1), 0.08f), vec3(0), vec3(0),
                   vec3(0.75f, 0.12f, 0.08f));
    });
    test("Drop one barrel onto another from 3 m", [](Game& gg) {
        add_barrel(gg, "barrel below", vec3(0, kBarrelH * 0.5f + 0.01f, 0), kUpright);
        add_barrel(gg, "barrel above", vec3(0.08f, kBarrelH + 3.0f + kBarrelH * 0.5f, 0), quat::axis_angle(vec3(0, 0, 1), 0.08f), vec3(0), vec3(0),
                   vec3(0.75f, 0.12f, 0.08f));
    });
    // a 40 kg steel ball (the Shoot tool's) onto a barrel lying on the pad (it cannot get away: a dent), as if dropped from
    // h (it starts just above at the speed of that fall); and one shot at a barrel standing against the wall
    for (float hgt : {3.0f, 8.0f}) {
        const std::string label = "Drop a 40 kg steel ball onto it from " + std::to_string((int)hgt) + " m";
        test(label.c_str(), [hgt](Game& gg) {
            add_barrel(gg, "barrel", vec3(0, kBarrelR + 0.01f, 0), kOnSideZ);
            gg.projectile_kind = 0;
            gg.projectile_speed = std::sqrt(2.0f * 9.81f * hgt);
            gg.shoot(vec3(0, 2.0f * kBarrelR + 0.25f + 2.0f, 0), vec3(0, -1, 0)); // (the shot starts 2 m along its line)
        });
    }
    test("Shoot a 40 kg steel ball at it against the wall at 20 m/s", [](Game& gg) {
        add_barrel(gg, "barrel", vec3(-11.7f + kBarrelR, kBarrelH * 0.5f + 0.01f, 0), kUpright);
        gg.projectile_kind = 0;
        gg.projectile_speed = 20.0f;
        gg.shoot(vec3(-11.7f + kBarrelR + 0.8f + 2.0f, 0.5f, 0), vec3(-1, 0, 0));
    });
    g.scene_hint = "Steel drums (200 l, 1 mm wall): a sheet of triangle elements on FEM rings at the chimes and the rolling hoops. "
                   "Scene menu: drop one on its bottom, side or rim, roll it down the ramp or into the wall, throw one at another. "
                   "F3: the mesh.";
}

// A pile of 15 barrels lying five, four, three, two, one (chocks at the ends of the bottom row), and three more thrown
// into it one after another at 10 m/s, low, middle, high (the Stress menu: the barrels' performance)
void scene_stress_barrels(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(81, 81, 1.0f, vec2(-40, -40));
    te::flatten_rect(g.world.statics.terrain, vec2(0, 0), vec2(34, 34), 0, 0.0f, 5, SURF_CONCRETE);
    g.finish_terrain();
    g.no_player_vehicle = true;
    g.set_spawn(vec3(0, 0, -10), 0);
    s_barrel_fem = getenv("BL_BARREL_FEM") != nullptr;
    const float r = kBarrelR + 0.008f, d = 2.0f * r + 0.012f, dy = d * 0.8660254f;
    int n = 0;
    for (int row = 0; row < 5; row++)
        for (int i = 0; i < 5 - row; i++) {
            const float x = (i - (4 - row) * 0.5f) * d, y = r + 0.005f + row * dy;
            const vec3 col = n % 3 == 0 ? vec3(0.10f, 0.28f, 0.62f) : n % 3 == 1 ? vec3(0.75f, 0.12f, 0.08f) : vec3(0.16f, 0.42f, 0.22f);
            add_barrel(g, "barrel pile " + std::to_string(++n), vec3(x, y, 0), kOnSideZ, vec3(0), vec3(0), col);
        }
    for (float sx : {-1.0f, 1.0f}) // (the chocks: the bottom row would roll apart)
        g.add_static_box(vec3(sx * (2.5f * d + 0.1f), 0.1f, 0), vec3(0.1f, 0.1f, 0.6f), quat(), SURF_CONCRETE, A.concrete);
    // the three thrown in: from 6 m in front, at 1.5, 3.5 and 5.5 s
    auto t = std::make_shared<float>(0.0f);
    auto thrown = std::make_shared<int>(0);
    g.scene_update = [t, thrown, r](Game& gg, float dt) {
        *t += dt;
        const float at[3] = {1.5f, 3.5f, 5.5f}, h[3] = {r + 0.05f, 1.3f, 2.3f}, x[3] = {-0.6f, 0.3f, 0.0f};
        while (*thrown < 3 && *t >= at[*thrown]) {
            const int k = (*thrown)++;
            add_barrel(gg, "barrel thrown " + std::to_string(k + 1), vec3(x[k], h[k], -6.0f), kOnSideX, vec3(0, k ? 2.0f : 0.0f, 10.0f), vec3(0),
                       vec3(0.85f, 0.62f, 0.10f));
        }
    };
    g.scene_hint = "15 steel barrels in a pile, three more thrown in (sheet barrels; BL_BARREL_FEM=1: with frame rings).";
}

} // namespace bl
