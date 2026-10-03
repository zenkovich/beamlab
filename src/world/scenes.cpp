// Scene registry: test worlds for the vehicle and physics demos.
#include "core/util.h"
#include "game/game.h"
#include "phys/fem_shell.h"
#include "vehicle/vehicle.h"
#include "world/ai.h"
#include "world/stage.h"

#include <algorithm>
#include <cmath>

namespace bl {

using namespace phys;
namespace te = terrain_edit;

namespace {

quat yaw_q(float deg) { return quat::axis_angle(vec3(0, 1, 0), deg * kDeg2Rad); }

// Wedge ramp made of a rotated box: `pos` = center of the ramp foot line, rises towards `yaw` direction.
void add_ramp(Game& g, vec3 foot, float yaw_deg, float length, float width, float height, MaterialPtr mat, uint8_t surf = SURF_CONCRETE) {
    float ang = std::atan2(height, length);
    float slope_len = std::sqrt(length * length + height * height);
    float thick = 0.6f;
    quat q = yaw_q(yaw_deg) * quat::axis_angle(vec3(1, 0, 0), -ang);
    vec3 fwd = yaw_q(yaw_deg).rotate(vec3(0, 0, 1));
    // box center: middle of the slope, pushed down by half thickness along the slope normal
    vec3 up = q.rotate(vec3(0, 1, 0));
    vec3 c = foot + fwd * (length * 0.5f) + vec3(0, height * 0.5f, 0) - up * (thick * 0.5f);
    g.add_static_box(c, vec3(width * 0.5f, thick * 0.5f, slope_len * 0.5f), q, surf, mat);
}

void add_cone_line(Game& g, vec3 a, vec3 b, int n) {
    for (int i = 0; i < n; i++) {
        float t = n > 1 ? (float)i / (n - 1) : 0;
        ConeDesc d;
        d.base = lerp(a, b, t);
        d.base.y = g.world.statics.terrain.height(d.base.x, d.base.z);
        g.add_object(build_traffic_cone(g.world, d, format("cone%d", i)));
    }
}

void scatter_trees(Game& g, vec2 center, float radius, int count, uint32_t seed, float road_clear, const std::vector<vec2>& road, float scale = 1.0f,
                   bool bushes = true) {
    Rng rng(seed);
    auto& hf = g.world.statics.terrain;
    int placed = 0, tries = 0;
    std::vector<vec2> used;
    while (placed < count && tries < count * 30) {
        tries++;
        float a = rng.range(0, 2 * kPi), r = radius * std::sqrt(rng.uniform());
        vec2 p = center + vec2(std::cos(a), std::sin(a)) * r;
        if (!hf.inside(p.x, p.y)) continue;
        bool bad = false;
        for (size_t i = 0; i + 1 < road.size() && !bad; i++) {
            vec2 ab = road[i + 1] - road[i];
            float t = clampf(dot(p - road[i], ab) / std::max(1e-6f, dot(ab, ab)), 0, 1);
            if (length(p - (road[i] + ab * t)) < road_clear) bad = true;
        }
        for (auto& u : used)
            if (length(u - p) < 3.2f) bad = true;
        vec3 n;
        float h;
        hf.sample(p.x, p.y, h, n);
        if (n.y < 0.8f) bad = true;
        if (bad) continue;
        used.push_back(p);
        TreeDesc d;
        d.base = vec3(p.x, h - 0.1f, p.y);
        d.seed = seed * 1000 + placed;
        float k = rng.uniform();
        if (bushes && k < 0.3f) {
            d.kind = TreeKind::Bush;
            d.height = rng.range(1.2f, 2.2f);
            d.trunk_radius = 0.035f;
            d.strength = 0.6f;
        } else if (k < 0.6f) {
            d.kind = TreeKind::Pine;
            d.height = rng.range(8, 14) * scale;
            d.trunk_radius = rng.range(0.14f, 0.22f) * scale;
        } else if (k < 0.8f) {
            d.kind = TreeKind::Deciduous;
            d.height = rng.range(7, 11) * scale;
            d.trunk_radius = rng.range(0.14f, 0.2f) * scale;
        } else if (k < 0.95f) {
            d.kind = TreeKind::Birch;
            d.height = rng.range(7, 10) * scale;
            d.trunk_radius = rng.range(0.08f, 0.12f) * scale;
        } else {
            d.kind = TreeKind::Dead;
            d.height = rng.range(5, 8);
            d.trunk_radius = rng.range(0.1f, 0.15f);
            d.strength = 0.5f;
        }
        g.add_object(build_tree(g.world, d, format("tree%d", placed)));
        placed++;
    }
}

std::vector<vec2> ellipse(vec2 c, vec2 r, int n, float phase = 0) {
    std::vector<vec2> p;
    for (int i = 0; i <= n; i++) {
        float a = phase + 2 * kPi * i / n;
        p.push_back(c + vec2(std::cos(a) * r.x, std::sin(a) * r.y));
    }
    return p;
}

// ------------------------------------------------------------------------------------------- scenes
void scene_proving_ground(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(401, 401, 1.0f, vec2(-200, -200));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 6.0f, 90.0f, 4, 11);
    te::flatten_rect(hf, vec2(0, 0), vec2(75, 55), 0, 0.0f, 25.0f, SURF_ASPHALT);
    // hill climb (south-east): 20% and 40% grades
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) {
            float wx = hf.origin().x + x, wz = hf.origin().y + z;
            if (wx > 90 && wx < 185 && wz > -48 && wz < 8) {
                // hill climb: 20% then 40% grade, plateau on top, smooth side slopes
                float t = clampf((wx - 95) / 80.0f, 0, 1);
                float hgt = t < 0.5f ? t * 2 * 8.0f : 8.0f + (t - 0.5f) * 2 * 16.0f;
                float side = std::max(0.0f, std::fabs(wz + 20.0f) - 7.0f); // 14 m wide road
                float k = clampf(1.0f - side / 14.0f, 0, 1);
                float endk = clampf((185 - wx) / 10.0f, 0, 1);
                float target = hgt * smoothstepf(0, 1, k) * endk;
                if (target > hf.h(x, z)) {
                    hf.h(x, z) = target;
                    hf.surf(x, z) = side < 0.5f ? SURF_DIRT : SURF_GRASS;
                }
            }
            // twister / axle articulation bumps (north-east)
            if (wx > 90 && wx < 150 && wz > 25 && wz < 37) {
                float s = std::sin((wx - 90) * 0.35f) * (wz < 31 ? 1 : -1);
                hf.h(x, z) = std::max(0.0f, s) * 0.8f;
                hf.surf(x, z) = SURF_DIRT;
            }
        }
    te::flatten_rect(hf, vec2(-120, 0), vec2(20, 20), 0, 0.2f, 6.0f, SURF_MUD);   // mud pit
    te::flatten_rect(hf, vec2(-120, 50), vec2(20, 18), 0, 0.0f, 6.0f, SURF_ICE);  // ice skid pad
    te::flatten_rect(hf, vec2(-120, -50), vec2(20, 18), 0, 0.0f, 6.0f, SURF_GRAVEL);
    te::paint_road(hf, ellipse(vec2(0, 0), vec2(150, 110), 64), 6.0f, SURF_ASPHALT, 0.9f);
    te::auto_surfaces(hf);
    g.finish_terrain();

    // slalom
    add_cone_line(g, vec3(-55, 0, 20), vec3(35, 0, 20), 10);
    add_cone_line(g, vec3(-55, 0, 40), vec3(-55, 0, 46), 2);
    // ramps: small / medium / jump
    add_ramp(g, vec3(-40, 0, -25), 90, 6, 5, 0.6f, A.concrete);
    add_ramp(g, vec3(-10, 0, -25), 90, 8, 5, 1.4f, A.concrete);
    add_ramp(g, vec3(25, 0, -25), 90, 10, 6, 2.6f, A.concrete);
    add_ramp(g, vec3(67, 0, -25), -90, 16, 6, 2.6f, A.concrete); // landing
    // speed bumps
    for (int i = 0; i < 4; i++)
        g.add_static_box(vec3(-30 + i * 9.0f, 0.0f, -45), vec3(0.35f, 0.12f, 4.0f), quat(), SURF_ASPHALT, A.yellow);
    // washboard
    for (int i = 0; i < 30; i++) g.add_static_box(vec3(10 + i * 0.7f, -0.02f, -45), vec3(0.12f, 0.08f, 3.5f), quat(), SURF_CONCRETE, A.concrete);
    // stairs
    for (int i = 0; i < 6; i++) g.add_static_box(vec3(50 + i * 1.2f, 0.12f + i * 0.18f, -45), vec3(0.6f + (5 - i) * 0.0f, 0.12f + i * 0.18f, 3.0f), quat(), SURF_CONCRETE, A.concrete);
    g.add_static_box(vec3(60, 0.5f, -45), vec3(2.0f, 1.0f, 3.0f), quat(), SURF_CONCRETE, A.concrete);
    // curbs around the pad
    for (int i = -3; i <= 3; i++) g.add_static_box(vec3(i * 20.0f, 0.1f, 56), vec3(9.5f, 0.12f, 0.25f), quat(), SURF_CONCRETE, A.white);
    // a few dynamic crates to push around
    for (int i = 0; i < 6; i++) g.drop_primitive(0, vec3(-60 + i * 1.5f, 0.6f, -5));
    g.set_spawn(vec3(-60, 0.0f, 30), 90);
    g.scene_hint = "Slalom, ramps, bumps, washboard, stairs, hill climb (E), articulation twister (NE), mud/ice/gravel pads (W).";
}

void scene_forest(Game& g) {
    g.create_terrain(351, 351, 1.0f, vec2(-175, -175));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 14.0f, 70.0f, 5, 3);
    te::flatten_circle(hf, vec2(0, 0), 14, hf.height(0, 0), 10, SURF_GRASS);
    std::vector<vec2> road;
    for (int i = 0; i <= 40; i++) {
        float t = i / 40.0f;
        road.push_back(vec2(-150 + 300 * t, 40 * std::sin(t * 6.0f) + 10 * std::sin(t * 17)));
    }
    te::paint_road(hf, road, 3.5f, SURF_DIRT, 0.8f);
    te::auto_surfaces(hf);
    g.finish_terrain();
    scatter_trees(g, vec2(0, 0), 150, 220, 17, 6.0f, road);
    vec2 sp = road[18];
    g.set_spawn(vec3(sp.x, hf.height(sp.x, sp.y), sp.y), 90);
    g.world.settings.wind = vec3(2.0f, 0, 1.0f);
    g.scene_hint = "Trees and bushes are node/joint bodies with orientation-preserving joints. Drive into them, grab branches with the mouse, raise the wind in Physics menu.";
}

void scene_canyon(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(401, 301, 1.0f, vec2(-200, -150));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 3.0f, 60.0f, 4, 21);
    te::flatten_rect(hf, vec2(-80, 0), vec2(55, 70), 0, 0.0f, 15, -1);
    te::flatten_rect(hf, vec2(80, 0), vec2(55, 70), 0, 0.0f, 15, -1);
    te::carve_channel(hf, vec2(0, -160), vec2(0, 160), -32.0f, 20.0f, 7.0f);
    te::paint_road(hf, {vec2(-160, 0), vec2(-27, 0)}, 4.0f, SURF_ASPHALT, 1.0f);
    te::paint_road(hf, {vec2(27, 0), vec2(160, 0)}, 4.0f, SURF_ASPHALT, 1.0f);
    te::paint_road(hf, {vec2(-160, 45), vec2(-26, 45)}, 3.0f, SURF_DIRT, 1.0f);
    te::paint_road(hf, {vec2(26, 45), vec2(160, 45)}, 3.0f, SURF_DIRT, 1.0f);
    te::auto_surfaces(hf);
    g.finish_terrain();
    // abutments
    for (float s : {-1.0f, 1.0f}) {
        g.add_static_box(vec3(s * 29.5f, -1.5f, 0), vec3(3.0f, 1.5f, 3.6f), quat(), SURF_CONCRETE, A.concrete);
        g.add_static_box(vec3(s * 28.0f, -1.2f, 45), vec3(2.0f, 1.2f, 2.2f), quat(), SURF_ROCK, A.stone);
    }
    // steel truss bridge (breaks under heavy trucks)
    BridgeDesc bd;
    bd.start = vec3(-27, 0.05f, 0);
    bd.end = vec3(27, 0.05f, 0);
    bd.width = 5.0f;
    bd.segments = 18;
    bd.truss_height = 2.2f;
    bd.mass_per_m = 350.0f;
    // tuned: cars (<= ~8 t RoR mass) cross, heavy trucks (Tatra, bus, wrecker) break it
    bd.deck = {8e6f, 6000, 2.2e5f, 2.5e5f, 0.0f};
    bd.truss_beams = {1.2e7f, 8000, 3.1e5f, 3.5e5f, 0.0f};
    bd.deck_mat = A.wood;
    bd.truss_mat = A.rust;
    g.add_object(build_bridge(g.world, bd, "truss bridge"));
    // weak wooden bridge (no truss)
    BridgeDesc wb;
    wb.start = vec3(-26, 0.05f, 45);
    wb.end = vec3(26, 0.05f, 45);
    wb.width = 4.0f;
    wb.segments = 20;
    wb.truss = false;
    wb.mass_per_m = 150.0f;
    wb.girder_depth = 1.4f;
    wb.deck = {3e6f, 2500, 2.0e5f, 2.3e5f, 0.0f}; // holds light cars, breaks under vans / trucks / buses
    wb.truss_beams = {1.5e7f, 5000, 2.0e5f, 2.3e5f, 0.0f};
    wb.truss_mat = A.dark_wood;
    wb.deck_mat = A.wood;
    // two wooden trestle piers from the canyon floor split the rope bridge into ~17 m spans
    for (float px : {-8.7f, 8.7f}) {
        float floor_y = hf.height(px, 45.0f);
        float top = 0.05f - 1.4f - 0.02f;
        g.add_static_box(vec3(px, (top + floor_y) * 0.5f, 45.0f), vec3(0.35f, (top - floor_y) * 0.5f, 2.3f), quat(), SURF_WOOD, A.dark_wood);
    }
    g.add_object(build_bridge(g.world, wb, "wooden bridge"));
    g.set_spawn(vec3(-70, 0.0f, 0), 90);
    g.scene_hint = "Steel truss bridge (south) holds cars and vans but breaks under heavy trucks and buses; the wooden rope bridge (north) holds only light cars. Use Scene > 'Send heavy truck' for a demo.";
    g.scene_update = [](Game& gg, float dt) { ai_update_all(gg, dt); };
}

phys::ShellMaterial sheet_material(const std::string& name);
MaterialPtr fabric_visual();

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

void scene_crash(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(221, 701, 1.0f, vec2(-110, -350));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 4.0f, 60.0f, 3, 9);
    te::flatten_rect(hf, vec2(0, 0), vec2(20, 330), 0, 0.0f, 20, SURF_ASPHALT);
    g.finish_terrain();
    // concrete wall at the end
    g.add_static_box(vec3(-6, 1.5f, 300), vec3(4, 1.5f, 0.6f), quat(), SURF_CONCRETE, A.concrete);
    // deformable barrier: wall of crates
    for (int y = 0; y < 3; y++)
        for (int x = 0; x < 5; x++) {
            SoftBoxDesc d;
            d.center = vec3(4 + x * 1.02f, 0.5f + y * 1.02f, 300);
            d.size = vec3(1.0f);
            d.nx = d.ny = d.nz = 2;
            d.mass = 50;
            d.mat = std::make_shared<Material>(*A.wood);
            d.mat->diffuse = A.tex_crate;
            d.beams = {1e6f, 1000, 2e4f, 8e4f, 0.0f};
            g.add_object(build_soft_box(g.world, d, format("barrier%d", y * 5 + x)));
        }
    // breakable poles (street lights)
    for (int i = 0; i < 4; i++) {
        PoleDesc pd;
        pd.base = vec3(-12, 0, 200 + i * 12.0f);
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
    add_cone_line(g, vec3(-2, 0, 150), vec3(-2, 0, 280), 12);
    add_cone_line(g, vec3(12, 0, 150), vec3(12, 0, 280), 12);
    g.set_spawn(vec3(0, 0, -300), 0);
    g.scene_hint = "Scene > 'Launch at 50/80 km/h' fires the vehicle into the wall/barrier.";
}

void scene_offroad(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(401, 401, 1.0f, vec2(-200, -200));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 18.0f, 60.0f, 5, 41);
    te::add_noise(hf, 1.2f, 6.0f, 3, 43);
    te::flatten_circle(hf, vec2(0, 0), 12, hf.height(0, 0), 8, SURF_GRAVEL);
    // mud pits & sand
    Rng rng(4);
    for (int i = 0; i < 8; i++) {
        vec2 c(rng.range(-150, 150), rng.range(-150, 150));
        te::paint_circle(hf, c, rng.range(6, 14), i % 2 ? SURF_MUD : SURF_SAND);
    }
    te::auto_surfaces(hf);
    g.finish_terrain();
    // rocks
    for (int i = 0; i < 90; i++) {
        vec2 p(rng.range(-180, 180), rng.range(-180, 180));
        if (length(p) < 20) continue;
        float h = hf.height(p.x, p.y);
        vec3 half(rng.range(0.4f, 1.6f), rng.range(0.3f, 1.2f), rng.range(0.4f, 1.6f));
        quat q = quat::axis_angle(rng.unit_vector(), rng.range(0, 1.2f));
        g.add_static_box(vec3(p.x, h, p.y), half, q, SURF_ROCK, A.stone);
    }
    // logs to drive over
    for (int i = 0; i < 12; i++) {
        vec2 p(rng.range(-60, 60), rng.range(-60, 60));
        if (length(p) < 15) continue;
        SoftBoxDesc d;
        d.center = vec3(p.x, hf.height(p.x, p.y) + 0.4f, p.y);
        d.size = vec3(5.0f, 0.4f, 0.4f);
        d.nx = 6;
        d.ny = 2;
        d.nz = 2;
        d.mass = 250;
        d.rot = quat::axis_angle(vec3(0, 1, 0), rng.range(0, 3));
        d.mat = A.dark_wood;
        d.beams = {4e6f, 2000, 1e9f, 1e9f, 0};
        g.add_object(build_soft_box(g.world, d, format("log%d", i)));
    }
    scatter_trees(g, vec2(0, 0), 190, 90, 29, 0, {}, 1.0f, true);
    g.set_spawn(vec3(0, hf.height(0, 0), 0), 30);
    g.scene_hint = "Rough terrain with rocks, mud (soft ground) and sand pits, logs.";
}

void scene_vehicle_crash(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(301, 301, 1.0f, vec2(-150, -150));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 5.0f, 70.0f, 3, 61);
    te::flatten_rect(hf, vec2(0, 0), vec2(110, 110), 0, 0.0f, 25, SURF_ASPHALT);
    g.finish_terrain();
    // impact marker: a painted cross made of flat concrete slabs (drive-over height)
    g.add_static_box(vec3(0, -0.045f, 0), vec3(6.0f, 0.05f, 0.15f), quat(), SURF_ASPHALT, A.yellow);
    g.add_static_box(vec3(0, -0.045f, 0), vec3(0.15f, 0.05f, 6.0f), quat(), SURF_ASPHALT, A.yellow);
    // default pair: two different cars
    if (g.crash.vehicle_a.empty() || !find_vehicle(g.crash.vehicle_a) || g.crash.vehicle_b.empty() || !find_vehicle(g.crash.vehicle_b)) {
        const auto& reg = vehicle_registry();
        std::vector<std::string> pick;
        for (const char* pref : {"bmw_e36", "dodge_viper", "audi_quattro", "mercedes_clk"})
            for (auto& e : reg)
                if (e.folder == pref) {
                    pick.push_back(e.id);
                    break;
                }
        for (auto& e : reg)
            if (pick.size() < 2 && e.drivable) pick.push_back(e.id);
        if (pick.size() >= 2) {
            g.crash.vehicle_a = pick[0];
            g.crash.vehicle_b = pick[1];
        }
    }
    g.set_spawn(vec3(0, 0, -40), 0);
    g.no_player_vehicle = true;
    g.scene_hint = "Two vehicles collide at the marked point. Choose the vehicles, speeds and crash layout in the Scene menu, F5 restarts.";
    g.scene_update = [](Game& gg, float dt) { gg.crash_update(dt); };
    g.run_crash();
}

// ---------------------------------------------------------------------------- stress scenes
// ------------------------------------------------------------------------------------------- rally stage
// Centre line of a stage: Catmull-Rom through control points, resampled every `step` metres.
struct StageRoute {
    std::vector<vec2> p;
    std::vector<float> s; // arc length at each sample
    float length() const { return s.back(); }
    int index_at(float d) const {
        int i = (int)(std::upper_bound(s.begin(), s.end(), d) - s.begin()) - 1;
        return std::clamp(i, 0, (int)p.size() - 2);
    }
    vec2 at(float d) const {
        int i = index_at(d);
        float t = clampf((d - s[i]) / std::max(1e-4f, s[i + 1] - s[i]), 0, 1);
        return p[i] + (p[i + 1] - p[i]) * t;
    }
    vec2 tangent(float d) const {
        int i = index_at(d);
        return normalize(p[i + 1] - p[i]);
    }
    // left of the driving direction (yaw +90 turns +z into +x, so left of +z is +x)
    vec2 left(float d) const {
        vec2 t = tangent(d);
        return vec2(t.y, -t.x);
    }
    float nearest_s(vec2 q, int from = 0, int to = -1) const {
        if (to < 0) to = (int)p.size() - 1;
        float best = 1e30f, bs = 0;
        for (int i = std::max(0, from); i < std::min(to, (int)p.size() - 1); i++) {
            vec2 ab = p[i + 1] - p[i];
            float t = clampf(dot(q - p[i], ab) / std::max(1e-6f, dot(ab, ab)), 0, 1);
            vec2 d = p[i] + ab * t - q;
            float dd = dot(d, d);
            if (dd < best) {
                best = dd;
                bs = s[i] + (s[i + 1] - s[i]) * t;
            }
        }
        return bs;
    }
    float dist(vec2 q) const {
        float best = 1e30f;
        for (size_t i = 0; i + 1 < p.size(); i++) {
            vec2 ab = p[i + 1] - p[i];
            float t = clampf(dot(q - p[i], ab) / std::max(1e-6f, dot(ab, ab)), 0, 1);
            vec2 d = p[i] + ab * t - q;
            best = std::min(best, dot(d, d));
        }
        return std::sqrt(best);
    }
    // signed curvature at sample i (+ = turning left), over +-w samples
    float curvature(int i, int w = 3) const {
        int a = std::max(0, i - w), b = std::min((int)p.size() - 1, i + w);
        if (b - a < 2) return 0;
        vec2 t0 = normalize(p[i] - p[a]), t1 = normalize(p[b] - p[i]);
        float turn = std::asin(clampf(t0.y * t1.x - t0.x * t1.y, -1, 1)); // cross(t0, t1) in (x, z): > 0 left
        return turn / std::max(1e-3f, 0.5f * (s[b] - s[a]));
    }
};

StageRoute make_stage_route(const std::vector<vec2>& ctrl, float step) {
    StageRoute r;
    std::vector<vec2> q = {ctrl.front()};
    q.insert(q.end(), ctrl.begin(), ctrl.end());
    q.push_back(ctrl.back());
    auto cr = [](vec2 p0, vec2 p1, vec2 p2, vec2 p3, float t) {
        float t2 = t * t, t3 = t2 * t;
        return (p1 * 2.0f + (p2 - p0) * t + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2 + (p1 * 3.0f - p0 - p2 * 3.0f + p3) * t3) * 0.5f;
    };
    std::vector<vec2> dense;
    for (size_t i = 1; i + 2 < q.size(); i++) {
        int n = std::max(8, (int)(length(q[i + 1] - q[i]) / 0.5f));
        for (int k = 0; k < n; k++) dense.push_back(cr(q[i - 1], q[i], q[i + 1], q[i + 2], (float)k / n));
    }
    dense.push_back(ctrl.back());
    // resample at even spacing
    r.p.push_back(dense[0]);
    r.s.push_back(0);
    float acc = 0;
    for (size_t i = 1; i < dense.size(); i++) {
        acc += length(dense[i] - dense[i - 1]);
        if (acc >= step || i + 1 == dense.size()) {
            r.s.push_back(r.s.back() + length(dense[i] - r.p.back()));
            r.p.push_back(dense[i]);
            acc = 0;
        }
    }
    return r;
}

// Rounded crest across the road (jump): rises over `up` metres, falls over `down`, full height within the road.
void add_crest(Heightfield& hf, vec2 c, vec2 dir, float height, float up, float down, float half_width) {
    vec2 n(dir.y, -dir.x);
    const vec2 o = hf.origin();
    float reach = std::max(up, down) + half_width + 10;
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) {
            vec2 p(o.x + x * hf.cell(), o.y + z * hf.cell());
            if (std::fabs(p.x - c.x) > reach || std::fabs(p.y - c.y) > reach) continue;
            float a = dot(p - c, dir), l = std::fabs(dot(p - c, n));
            float prof = 0;
            if (a > -up && a <= 0) prof = smoothstepf(0, 1, (a + up) / up);
            else if (a > 0 && a < down) prof = 1.0f - smoothstepf(0, 1, a / down);
            float lat = 1.0f - smoothstepf(0, 1, (l - half_width) / 10.0f);
            hf.h(x, z) += height * prof * clampf(lat, 0, 1);
        }
}

std::string stage_time(double t) {
    int m = (int)(t / 60.0);
    return format("%d:%05.2f", m, t - m * 60.0);
}

// Stage clock between two arc lengths of the route (banner + status line) and the Autopilot scene action
// (the AI drives the player's car along `line`).
void install_stage_timer(Game& g, std::shared_ptr<StageRoute> route, float s_start, float s_finish, std::vector<vec3> line, float ai_speed,
                         float ai_lat_acc, std::string wait_text) {
    struct Timer {
        int state = 0; // 0 before the start, 1 running, 2 finished
        double t0 = 0, result = 0, best = 0;
        int idx = 0;
    };
    auto timer = std::make_shared<Timer>();
    g.scene_actions.push_back({"Autopilot: AI drives the stage", [line, ai_speed, ai_lat_acc](Game& gg) {
                                   Vehicle* v = gg.player_vehicle();
                                   if (!v) return;
                                   if (v->ai && v->ai_state.autopilot) {
                                       v->ai = false;
                                       v->ai_state.autopilot = false;
                                       return;
                                   }
                                   v->ai = true;
                                   v->ai_state.autopilot = true;
                                   ai_set_race_route(*v, line, ai_speed, ai_lat_acc, 4.0f, false);
                               }});
    g.scene_update = [route, timer, s_start, s_finish, wait_text](Game& gg, float dt) {
        ai_update_all(gg, dt);
        Vehicle* v = gg.player_vehicle();
        if (!v) return;
        const StageRoute& R = *route;
        vec3 p3 = v->position();
        vec2 p(p3.x, p3.z);
        int i0 = std::max(0, timer->idx - 20), i1 = std::min((int)R.p.size() - 1, timer->idx + 60);
        float s = R.nearest_s(p, i0, i1);
        if (length(R.at(s) - p) > 20.0f) s = R.nearest_s(p); // reset / off the stage: global search
        timer->idx = R.index_at(s);
        double now = gg.world.time();
        std::string pilot = v->ai_state.autopilot && v->ai ? "  [autopilot]" : "";
        if (s < s_start - 5.0f && timer->state != 0) timer->state = 0;
        if (timer->state == 0) {
            gg.scene_banner.clear();
            gg.scene_status = wait_text + pilot;
            if (s >= s_start && s < s_start + 30) {
                timer->state = 1;
                timer->t0 = now;
            }
        }
        if (timer->state == 1) {
            double t = now - timer->t0;
            gg.scene_banner = format("%s   %.2f / %.2f km", stage_time(t).c_str(), std::max(0.0f, s - s_start) / 1000.0f, (s_finish - s_start) / 1000.0f);
            gg.scene_status = "Stage running" + pilot;
            if (s >= s_finish) {
                timer->state = 2;
                timer->result = t;
                if (timer->best <= 0 || t < timer->best) timer->best = t;
            }
        }
        if (timer->state == 2) {
            gg.scene_banner = "FINISH  " + stage_time(timer->result);
            gg.scene_status = format("Stage time %s, best %s. F5 restarts the stage.", stage_time(timer->result).c_str(),
                                     stage_time(timer->best).c_str()) + pilot;
        }
    };
}

void scene_rally(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(701, 701, 1.0f, vec2(-350, -350));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 11.0f, 130.0f, 4, 23);
    const std::vector<vec2> ctrl = {
        {-300, -255}, {-250, -255}, {-190, -255}, {-150, -250}, {-126, -232}, {-118, -195}, {-124, -150}, {-112, -110}, {-80, -86},
        {-35, -78},   {10, -86},    {52, -104},   {92, -100},   {122, -72},   {132, -30},   {118, 10},    {124, 50},    {152, 78},
        {190, 84},    {222, 100},   {240, 128},   {244, 152},   {236, 170},   {220, 176},   {204, 168},   {198, 150},   {190, 128},
        {168, 120},   {140, 130},   {112, 152},   {92, 184},    {60, 200},    {20, 196},    {-20, 180},   {-60, 184},   {-100, 204},
        {-140, 212},  {-190, 208},  {-240, 214},  {-290, 214}};
    auto route = std::make_shared<StageRoute>(make_stage_route(ctrl, 3.0f));
    const StageRoute& R = *route;
    const float half_w = 3.6f;
    // sections (arc length): fields at both ends, a forest in the middle (twisty part + hairpin)
    const float forest0 = R.nearest_s(ctrl[12]) - 25, forest1 = R.nearest_s(ctrl[29]) + 30;
    auto in_forest = [&](float s) { return s > forest0 && s < forest1; };
    const float s_start = R.nearest_s(ctrl[1]), s_finish = R.length() - 45.0f;
    // crests on the fast field straights
    for (int k : {9, 34}) {
        float s = R.nearest_s(ctrl[k]);
        add_crest(hf, R.at(s), R.tangent(s), 1.5f, 11.0f, 8.0f, half_w + 2);
    }
    for (float s : {0.0f, R.length()}) {
        vec2 c = R.at(s);
        te::flatten_circle(hf, c, 22, hf.height(c.x, c.y), 12, SURF_GRASS);
    }
    te::paint_road(hf, R.p, half_w, SURF_GRAVEL, 0.85f);
    // (the terrain is finished after the bales and haystacks: each gets a levelled patch, a round bale on a
    // cross slope would roll away)
    auto ground = [&](vec2 p) { return vec3(p.x, hf.height(p.x, p.y), p.y); };

    Rng rng(4242);
    std::vector<std::pair<vec2, float>> taken; // occupied spots (centre, radius): stakes, bales, trees
    auto free_at = [&](vec2 p, float r) {
        for (auto& t : taken)
            if (length(t.first - p) < t.second + r) return false;
        return true;
    };
    int n_tape = 0, n_bale = 0, n_tree = 0, n_stack = 0;

    // ---- start / finish gates: two posts and a chequered cross bar
    auto gate = [&](float s) {
        vec2 c = R.at(s), t = R.tangent(s), l = R.left(s);
        float h = hf.height(c.x, c.y);
        quat q = yaw_q(std::atan2(t.x, t.y) * kRad2Deg);
        for (int side : {-1, 1}) {
            vec2 pp = c + l * (side * (half_w + 1.6f));
            g.add_static_box(vec3(pp.x, h + 2.6f, pp.y), vec3(0.15f, 2.6f, 0.15f), q, SURF_METAL, A.metal);
            taken.push_back({pp, 1.0f});
        }
        g.add_static_box(vec3(c.x, h + 5.0f, c.y), vec3(half_w + 1.75f, 0.4f, 0.08f), q, SURF_METAL, A.banner);
    };
    gate(s_start);
    gate(s_finish);

    // ---- corners: tape on the outside + round bales at the apex (fields), bales in front of the trees (forest)
    std::vector<int> apexes;
    for (int i = 3; i + 3 < (int)R.p.size(); i++) {
        float k = std::fabs(R.curvature(i));
        if (k < 1.0f / 70.0f) continue;
        bool peak = true;
        for (int j = std::max(0, i - 6); j <= std::min((int)R.p.size() - 1, i + 6); j++)
            if (std::fabs(R.curvature(j)) > k) peak = false;
        if (peak && (apexes.empty() || R.s[i] - R.s[apexes.back()] > 30.0f)) apexes.push_back(i);
    }
    auto tape_line = [&](float s0, float s1, float offset, float spacing) {
        TapeLineDesc td;
        for (float s = s0; s <= s1 + 0.01f; s += spacing) {
            vec2 p = R.at(s) + R.left(s) * offset;
            if (R.dist(p) < half_w + 1.2f || !free_at(p, 0.6f)) {
                if (td.posts.size() >= 3) {
                    g.add_object(build_tape_line(g.world, td, format("tape%d", n_tape++)));
                }
                td.posts.clear();
                continue;
            }
            td.posts.push_back(ground(p) - vec3(0, 0.05f, 0));
        }
        if (td.posts.size() >= 3) g.add_object(build_tape_line(g.world, td, format("tape%d", n_tape++)));
        for (float s = s0; s <= s1; s += spacing) taken.push_back({R.at(s) + R.left(s) * offset, 0.5f});
    };
    // round bale lying on its side; `h`: ground height of the levelled patch (shared by a row of bales)
    auto round_bale = [&](vec2 p, float yaw, float h) {
        if (!free_at(p, 0.9f) || R.dist(p) < half_w + 0.6f) return;
        te::flatten_circle(hf, p, 1.0f, h, 0.9f);
        g.add_object(build_round_bale(g.world, vec3(p.x, h + 0.78f, p.y), yaw, format("bale%d", n_bale++)));
        taken.push_back({p, 0.9f});
    };
    for (int ai : apexes) {
        float s = R.s[ai], k = R.curvature(ai);
        if (s < s_start + 20 || s > s_finish - 10) continue;
        float out = k > 0 ? -1.0f : 1.0f; // outside of the corner
        float radius = 1.0f / std::fabs(k);
        vec2 t = R.tangent(s);
        float yaw = std::atan2(t.x, t.y) * kRad2Deg + 90.0f; // bale axis along the road
        // round bales at the apex on the outside
        int nb = radius < 25 ? 4 : 3;
        float row_h = hf.height(R.at(s).x + R.left(s).x * out * (half_w + 1.6f), R.at(s).y + R.left(s).y * out * (half_w + 1.6f));
        for (int b = 0; b < nb; b++) {
            float sb = s + (b - (nb - 1) * 0.5f) * 1.7f;
            round_bale(R.at(sb) + R.left(sb) * (out * (half_w + 1.6f)), std::atan2(R.tangent(sb).x, R.tangent(sb).y) * kRad2Deg + 90.0f, row_h);
        }
        (void)yaw;
        if (!in_forest(s) || radius < 25) tape_line(s - 26, s + 26, out * (half_w + 4.2f), 4.0f);
        if (radius < 25) {
            // hairpin: bales on the inside too, and a second tape line further out for the spectators
            vec2 pin = R.at(s) - R.left(s) * (out * (half_w + 1.5f));
            round_bale(pin, std::atan2(t.x, t.y) * kRad2Deg + 90.0f, hf.height(pin.x, pin.y));
            tape_line(s - 20, s + 20, out * (half_w + 9.0f), 4.0f);
        }
    }
    // spectator tape along the start straight (both sides)
    tape_line(s_start + 8, s_start + 70, half_w + 4.0f, 4.0f);
    tape_line(s_start + 8, s_start + 70, -(half_w + 4.0f), 4.0f);

    // ---- chicane of small square bales on the return straight (two stacked walls, alternating sides)
    auto square_bale = [&](vec3 c, float yaw) {
        SoftBoxDesc d;
        d.center = c;
        d.size = vec3(0.9f, 0.4f, 0.46f); // length (x), height, width
        d.rot = yaw_q(yaw);
        d.nx = 3;
        d.ny = 2;
        d.nz = 2;
        d.mass = 22;
        d.mat = A.straw;
        d.beams = {3e5f, 120, 1500, 6e4f, 0.35f};
        g.add_object(build_soft_box(g.world, d, format("sqbale%d", n_stack++)));
    };
    {
        float sc = R.nearest_s(ctrl[37]);
        for (int w = 0; w < 2; w++) {
            float s = sc + w * 24.0f;
            vec2 c = R.at(s), l = R.left(s), t = R.tangent(s);
            float side = w == 0 ? 1.0f : -1.0f;
            float yaw = std::atan2(t.x, t.y) * kRad2Deg; // local z along the road -> bale length (x) across it
            // gaps of ~0.1 m: neighbours must not start inside each other's collision radius
            for (int layer = 0; layer < 2; layer++)
                for (int row = 0; row < 2; row++)
                    for (int b = 0; b < 4 - layer; b++) {
                        float lat = side * (0.3f + 0.5f + b * 1.0f + layer * 0.5f);
                        vec2 p = c + l * lat + t * ((row - 0.5f) * 0.58f);
                        square_bale(ground(p) + vec3(0, 0.23f + layer * 0.48f, 0), yaw);
                    }
            taken.push_back({c + l * (side * 2.0f), 2.5f});
        }
    }
    // ---- haystacks in the fields (a couple close to fast corners), a round bale pyramid at the start
    for (int i = 0, tries = 0; i < 9 && tries < 400; tries++) {
        float s = rng.range(s_start + 30, s_finish - 30);
        if (in_forest(s)) continue;
        float lat = (rng.uniform() < 0.5f ? -1.0f : 1.0f) * rng.range(i < 3 ? 10.0f : 18.0f, i < 3 ? 13.0f : 45.0f);
        vec2 p = R.at(s) + R.left(s) * lat;
        if (!hf.inside(p.x, p.y) || R.dist(p) < 9.0f || !free_at(p, 3.5f)) continue;
        float rad = rng.range(1.9f, 2.4f), hgt = rng.range(3.4f, 4.2f);
        te::flatten_circle(hf, p, rad + 0.4f, hf.height(p.x, p.y), 2.0f);
        g.add_object(build_haystack(g.world, ground(p) - vec3(0, 0.05f, 0), hgt, rad, format("haystack%d", i)));
        taken.push_back({p, 3.0f});
        i++;
    }
    {
        vec2 c = R.at(4.0f), l = R.left(4.0f), t = R.tangent(4.0f);
        float yaw = std::atan2(t.x, t.y) * kRad2Deg + 90.0f;
        vec2 base = c + l * 14.0f;
        // 3-2-1 pyramid; centres 1.62 m apart (0.12 m gaps), rows at the touching height + 6 cm
        const int rows[3] = {3, 2, 1};
        const float pitch = 1.62f, rise = std::sqrt(pitch * pitch - 0.25f * pitch * pitch) + 0.06f;
        float h0 = hf.height(base.x, base.y);
        te::flatten_circle(hf, base, 3.6f, h0, 2.0f);
        for (int r = 0; r < 3; r++)
            for (int b = 0; b < rows[r]; b++) {
                vec2 p = base + t * ((b - (rows[r] - 1) * 0.5f) * pitch);
                g.add_object(build_round_bale(g.world, vec3(p.x, std::max(h0, hf.height(p.x, p.y)) + 0.78f + r * rise, p.y), yaw,
                                              format("bale%d", n_bale++)));
            }
        taken.push_back({base, 3.0f});
    }

    // ---- detailed road surface: ruts, camber, bumps, potholes, washboard before the tighter corners
    {
        auto road = std::make_shared<RoadSurface>();
        StageRoute dense = make_stage_route(ctrl, 1.0f);
        std::vector<vec2> wash;
        for (int ai : apexes)
            if (1.0f / std::max(1e-4f, std::fabs(R.curvature(ai))) < 45.0f) wash.push_back({R.s[ai] - 55.0f, R.s[ai] - 12.0f});
        road->build(dense.p, half_w, 1.2f, SURF_GRAVEL, 77, wash);
        g.world.statics.road = road;
        // the terrain under the frayed road edge is grass (the road texture's alpha lets it through)
        const vec2 o = hf.origin();
        for (int z = 0; z < hf.nz(); z++)
            for (int x = 0; x < hf.nx(); x++) {
                float s, lat;
                if (hf.surf(x, z) == SURF_GRAVEL && road->locate(o.x + x * hf.cell(), o.y + z * hf.cell(), s, lat) && std::fabs(lat) > half_w - 0.9f)
                    hf.surf(x, z) = SURF_GRASS;
            }
    }
    te::auto_surfaces(hf);
    g.finish_terrain();

    // ---- trees: the forest section (dense at the verge), a birch line along the first field, a few field trees
    auto tree = [&](vec2 p, float verge) {
        if (!hf.inside(p.x, p.y) || R.dist(p) < verge || !free_at(p, 1.6f)) return false;
        vec3 n;
        float h;
        hf.sample(p.x, p.y, h, n);
        if (n.y < 0.8f) return false;
        TreeDesc d;
        d.base = vec3(p.x, h - 0.1f, p.y);
        d.seed = 7000 + n_tree;
        float k = rng.uniform();
        if (k < 0.18f) {
            d.kind = TreeKind::Bush;
            d.height = rng.range(1.2f, 2.2f);
            d.trunk_radius = 0.035f;
            d.strength = 0.6f;
        } else if (k < 0.62f) {
            d.kind = TreeKind::Pine;
            d.height = rng.range(9, 15);
            d.trunk_radius = rng.range(0.14f, 0.22f);
        } else if (k < 0.82f) {
            d.kind = TreeKind::Deciduous;
            d.height = rng.range(7, 11);
            d.trunk_radius = rng.range(0.14f, 0.2f);
        } else {
            d.kind = TreeKind::Birch;
            d.height = rng.range(7, 10);
            d.trunk_radius = rng.range(0.08f, 0.12f);
        }
        g.add_object(build_tree(g.world, d, format("tree%d", n_tree++)));
        taken.push_back({p, 1.2f});
        return true;
    };
    for (float s = forest0; s < forest1; s += 5.5f)
        for (int side : {-1, 1})
            if (rng.uniform() < 0.8f) tree(R.at(s) + R.left(s) * (side * rng.range(half_w + 1.9f, half_w + 5.5f)), half_w + 1.8f);
    for (int i = 0, tries = 0; i < 170 && tries < 3000; tries++) {
        float s = rng.range(forest0 - 25, forest1 + 25);
        vec2 p = R.at(s) + R.left(s) * ((rng.uniform() < 0.5f ? -1.0f : 1.0f) * rng.range(half_w + 6.0f, 42.0f));
        if (tree(p, half_w + 5.0f)) i++;
    }
    for (float s = s_start + 20; s < R.nearest_s(ctrl[5]); s += 14.0f) tree(R.at(s) + R.left(s) * 12.0f, 8.0f);
    for (int i = 0, tries = 0; i < 16 && tries < 400; tries++) {
        float s = rng.range(0, R.length());
        if (in_forest(s)) continue;
        vec2 p = R.at(s) + R.left(s) * ((rng.uniform() < 0.5f ? -1.0f : 1.0f) * rng.range(9.0f, 50.0f));
        if (tree(p, 8.0f)) i++;
    }

    // ---- decorative grass: dense on the verges (hides the seam), thinning out into the fields, a few tufts on the
    // crown of the road; vehicles flatten it
    {
        g.grass = std::make_unique<GrassField>();
        Rng gr(9090);
        const RoadSurface& road = *g.world.statics.road;
        uint32_t seed = 1;
        auto put = [&](vec2 p, float scale, float tint) {
            if (!hf.inside(p.x, p.y)) return;
            for (auto& t : taken)
                if (t.second >= 1.0f && length(t.first - p) < t.second * 0.8f) return; // not inside bales / stacks
            g.grass->add(vec3(p.x, g.ground_height(p.x, p.y) - 0.02f, p.y), scale, tint, seed++);
        };
        for (float s = 0; s < road.length(); s += 0.3f) {
            vec2 c = road.point(s), l = road.left(s);
            for (int side : {-1, 1}) {
                // verge band: from the frayed road edge 2.5 m out; short near the gravel, patchy, taller further out
                float patch = value_noise2(s * 0.08f, side * 3.0f, 21) * 0.5f + 0.5f;
                for (int k = 0; k < 7; k++) {
                    float lat = half_w + 0.2f + k * 0.36f + gr.range(-0.15f, 0.15f);
                    if (gr.uniform() < 0.3f + 0.4f * (1.0f - patch)) continue;
                    float sc = (k == 0 ? gr.range(0.45f, 0.8f) : gr.range(0.6f, 1.25f)) * (0.8f + 0.4f * patch);
                    put(c + l * (side * lat), sc, gr.uniform() * 0.5f);
                }
                // outer band: sparse, taller, drier
                if (gr.uniform() < 0.6f) put(c + l * (side * gr.range(half_w + 2.6f, half_w + 9.0f)), gr.range(0.9f, 1.6f), gr.range(0.2f, 0.8f));
            }
            // grass on the crown between the ruts, here and there
            if (gr.uniform() < 0.12f && value_noise2(s * 0.05f, 0.0f, 5) > 0.2f) put(c + l * gr.range(-0.25f, 0.25f), gr.range(0.4f, 0.7f), 0.3f);
        }
        // patches in the fields around the stage
        for (int i = 0; i < 260; i++) {
            float s = gr.range(0, R.length());
            vec2 base = R.at(s) + R.left(s) * ((gr.uniform() < 0.5f ? -1.0f : 1.0f) * gr.range(10.0f, 30.0f));
            int n = (int)gr.range(20, 60);
            for (int k = 0; k < n; k++) put(base + vec2(gr.range(-3, 3), gr.range(-3, 3)), gr.range(0.8f, 1.5f), gr.uniform());
        }
        g.grass->finalize();
        log_info("grass: %zu tufts", g.grass->count());
    }

    g.world.settings.wind = vec3(1.5f, 0, 1.0f); // a breeze: the tapes flutter, the trees sway a little
    g.world.settings.wind_gusts = 0.5f;
    g.world.settings.wind_radius = 80.0f; // trees and tapes sway around the camera, the rest of the stage sleeps
    vec2 sp = R.at(3.0f), st = R.tangent(3.0f);
    g.set_spawn(ground(sp), std::atan2(st.x, st.y) * kRad2Deg);
    g.scene_hint = format("Rally stage, %.1f km of gravel: fields, a forest with a hairpin, two crests and a bale chicane. "
                          "Trees, tape stakes, hay bales and haystacks are all soft bodies. Timer starts at the START gate.",
                          (s_finish - s_start) / 1000.0f);
    log_info("rally: %.0f m, %d corners, %d trees, %d tape lines, %d round bales, %d square bales", R.length(), (int)apexes.size(), n_tree,
             n_tape, n_bale, n_stack);

    // ---- stage timer + autopilot
    std::vector<vec3> line;
    for (vec2 p : R.p) line.push_back(ground(p));
    install_stage_timer(g, route, s_start, s_finish, line, 26.0f, 4.5f, "Drive through the START gate to start the clock.");
}

void flat_arena(Game& g, float half, uint8_t surf = SURF_CONCRETE, bool walls = true) {
    auto& A = SharedAssets::get();
    int n = (int)(half * 2) + 41;
    g.create_terrain(n, n, 1.0f, vec2(-half - 20, -half - 20));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 3.0f, 40.0f, 3, 77);
    te::flatten_rect(hf, vec2(0, 0), vec2(half, half), 0, 0.0f, 15, surf);
    g.finish_terrain();
    if (walls) {
        g.add_static_box(vec3(0, 1.0f, half), vec3(half, 1.0f, 0.5f), quat(), SURF_CONCRETE, A.concrete);
        g.add_static_box(vec3(0, 1.0f, -half), vec3(half, 1.0f, 0.5f), quat(), SURF_CONCRETE, A.concrete);
        g.add_static_box(vec3(half, 1.0f, 0), vec3(0.5f, 1.0f, half), quat(), SURF_CONCRETE, A.concrete);
        g.add_static_box(vec3(-half, 1.0f, 0), vec3(0.5f, 1.0f, half), quat(), SURF_CONCRETE, A.concrete);
    }
}

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
} // namespace

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

namespace {

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

// ------------------------------------------------------------------------------------------- sheet car
// A car whose body is a sheet: the frame is nodes and beams as in any vehicle, the body panels are triangle elements
// (assets/vehicles/sheet_car, tools/make_sheet_car.py; Vehicle::make_sheet_body). A wall, poles and a second sheet
// car parked across the lane to crash into.
// ------------------------------------------------------------------------------------------- model editor
// The model editor's stage: a flat concrete square 800 m across (test drives, the physics test's floor), nothing else;
// a slate background and little haze, so the ground's edge draws a clear horizon and white and yellow lines read.
void scene_editor(Game& g) {
    g.create_terrain(401, 401, 2.0f, vec2(-400, -400));
    auto& hf = g.world.statics.terrain;
    te::flatten_rect(hf, vec2(0, 0), vec2(390, 390), 0, 0.0f, 5, SURF_ASPHALT); // (dark: the lines show)
    g.finish_terrain();
    g.light.sky_color = vec3(0.42f, 0.47f, 0.55f);
    g.light.ground_color = vec3(0.2f, 0.2f, 0.21f);
    g.light.fog_color = vec3(0.30f, 0.34f, 0.40f);
    g.light.fog_density = 0.00025f;
    g.no_player_vehicle = true;
    g.set_spawn(vec3(0, 0, 0), 0);
    g.scene_hint = "The model editor's stage (Editor menu, Ctrl+E).";
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
namespace {

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

} // namespace

// a gravel lane on the pad (the Buggy's): whoops, 0.5 m high every 8 m, from z = 20 to 100, and a tabletop jump
// (a 2.2 m kicker, its face curving up over 6 m to 12 degrees at the lip, a 16 m table, a 20 m landing); the
// terrain's grid is 1 m
static void pad_dirt_lane(phys::Heightfield& hf, float x0, float half) {
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
static void pad_rough_field(phys::Heightfield& hf) {
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

static void scene_fem_pad(Game& g, const char* vid, const char* name, bool dirt) {
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



// The pad's crash tests as scenes of their own (the reports' videos): the Frame Car's (or the Buggy's) scene with one
// of its Scene menu tests started on the first frame and the free camera where the report's was; F5 (a reload keeps
// the camera) or the Scene menu runs it again, the other tests stay in the menu
void scene_frame_car_test(Game& g, const char* action, vec3 eye, vec3 target, const char* hint, void (*scene)(Game&) = scene_frame_car) {
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

// ------------------------------------------------------------------------------------------- FEM shells
// Triangle elements of the FEM frame (phys::FrameTri: a co-rotational thin shell, membrane and Kirchhoff bending, in
// the frame's implicit step) in three prototypes on a concrete pad: a steel sheet of 3 mm, 2 x 2 m, lying across two
// supports; a cantilever, 1.5 x 0.5 m of 8 mm steel clamped in a concrete block; a hollow steel cube of 1 m, 2 mm,
// 5 x 5 cells a face. The Scene menu drops balls and slabs on them, throws the cube about.
namespace {

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

} // namespace

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
namespace {

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

} // namespace

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

// ------------------------------------------------------------------------------------------- tape maze
// A gymkhana course marked only with tape: five lanes joined by hairpins, then a chicane to the finish. Every tape
// line is a soft body (stakes on orientation joints, tearing tape), split into short sections.
void scene_tape_maze(Game& g) {
    auto& A = SharedAssets::get();
    g.create_terrain(181, 161, 1.0f, vec2(-90, -80));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 0.6f, 35.0f, 3, 515);
    te::flatten_rect(hf, vec2(0, -5), vec2(75, 70), 0, hf.height(0, -5), 10.0f, SURF_GRASS);
    // centre line: lanes along x, 14 m apart, joined by hairpins (radius 7 m)
    const int lanes = 5;
    const float x0 = -38, x1 = 38, z0 = -56, pitch = 14.0f, half_w = 3.3f;
    std::vector<vec2> ctrl = {{x0 - 18, z0}, {x0 - 8, z0}};
    for (int l = 0; l < lanes; l++) {
        float z = z0 + l * pitch;
        bool east = (l % 2) == 0;
        float xa = east ? x0 : x1, xb = east ? x1 : x0, dir = east ? 1.0f : -1.0f;
        ctrl.push_back({xa, z});
        ctrl.push_back({(xa + xb) * 0.5f, z});
        ctrl.push_back({xb, z});
        if (l + 1 < lanes) { // hairpin
            ctrl.push_back({xb + dir * 5.0f, z + 1.8f});
            ctrl.push_back({xb + dir * 7.0f, z + pitch * 0.5f});
            ctrl.push_back({xb + dir * 5.0f, z + pitch - 1.8f});
        }
    }
    // (lane 4 ends in the east) chicane northwards, then the finish straight to the west
    const float zc = z0 + (lanes - 1) * pitch;
    for (vec2 q : std::initializer_list<vec2>{{x1 + 10, zc + 6}, {x1 + 6, zc + 16}, {x1 - 6, zc + 20}, {x1 - 16, zc + 16}, {x1 - 26, zc + 22},
                                               {x1 - 36, zc + 17}, {x1 - 46, zc + 22}, {x1 - 60, zc + 20}, {x1 - 80, zc + 20}, {x1 - 92, zc + 20}})
        ctrl.push_back(q);
    auto route = std::make_shared<StageRoute>(make_stage_route(ctrl, 1.0f));
    const StageRoute& R = *route;
    te::paint_road(hf, R.p, half_w + 0.3f, SURF_DIRT, 0.7f);
    g.finish_terrain();
    auto ground = [&](vec2 p) { return vec3(p.x, hf.height(p.x, p.y), p.y); };
    const float s_start = R.nearest_s(vec2(x0 - 8, z0)) + 2.0f, s_finish = R.length() - 14.0f;

    // tape on both sides, a stake every 3 m, sections of 8 stakes (short bodies: a knocked section sleeps again soon)
    int n_tape = 0, n_stakes = 0;
    for (int side : {-1, 1}) {
        TapeLineDesc td;
        auto flush = [&]() {
            if (td.posts.size() >= 2) g.add_object(build_tape_line(g.world, td, format("maze_tape%d", n_tape++)));
            n_stakes += (int)td.posts.size();
            td.posts.clear();
        };
        for (float d = s_start - 6.0f; d <= s_finish + 6.0f; d += 3.0f) {
            vec2 p = R.at(d) + R.left(d) * (side * half_w);
            // keep clear of the other lanes (the inside of the hairpins folds back onto itself)
            bool clear = R.dist(p) > half_w - 0.4f;
            if (!clear) {
                flush();
                continue;
            }
            td.posts.push_back(ground(p) - vec3(0, 0.05f, 0));
            if (td.posts.size() >= 8) {
                vec3 last = td.posts.back();
                flush();
                td.posts.push_back(last + vec3(0.25f * R.tangent(d).x, 0, 0.25f * R.tangent(d).y)); // next section starts at the same stake
            }
        }
        flush();
    }
    // start / finish gates
    auto gate = [&](float d, MaterialPtr bar) {
        vec2 c = R.at(d), t = R.tangent(d), l = R.left(d);
        float h = hf.height(c.x, c.y);
        quat q = yaw_q(std::atan2(t.x, t.y) * kRad2Deg);
        for (int side : {-1, 1}) {
            vec2 pp = c + l * (side * (half_w + 1.2f));
            g.add_static_box(vec3(pp.x, h + 1.6f, pp.y), vec3(0.12f, 1.6f, 0.12f), q, SURF_METAL, A.metal);
        }
        g.add_static_box(vec3(c.x, h + 3.3f, c.y), vec3(half_w + 1.3f, 0.3f, 0.06f), q, SURF_METAL, bar);
    };
    gate(s_start, A.banner);
    gate(s_finish, A.banner);
    vec2 sp = R.at(1.0f), st = R.tangent(1.0f);
    g.set_spawn(ground(sp), std::atan2(st.x, st.y) * kRad2Deg);
    std::vector<vec3> line;
    for (vec2 p : R.p) line.push_back(ground(p));
    install_stage_timer(g, route, s_start, s_finish, line, 13.0f, 4.0f, "Drive through the START gate to start the clock.");
    g.world.settings.wind = vec3(1.2f, 0, 0.8f);
    g.world.settings.wind_gusts = 0.5f;
    g.world.settings.wind_radius = 60.0f;
    g.scene_hint = format("Tape maze: %.0f m between tapes only, 5 lanes, 4 hairpins and a chicane (%d stakes in %d tape sections). "
                          "The stakes bend and snap, the tape tears; the clock runs from START to FINISH.",
                          s_finish - s_start, n_stakes, n_tape);
    log_info("tape maze: %.0f m, %d stakes, %d tape sections", s_finish - s_start, n_stakes, n_tape);
}

// ------------------------------------------------------------------------------------------- imported RBR stage
// A Richard Burns Rally community stage converted by tools/fetch_rbr_stage.py: the original meshes and textures,
// the collision mesh as a 25 cm heightfield with the stage's surfaces, signs / banners / boards as light bodies with
// their own meshes, the round bales as soft bales, and the RBR driveline for the clock and the autopilot.
void scene_rbr_stage(Game& g, const std::string& id) {
    const std::string dir = asset_path("stages/" + id);
    auto bundle = std::make_shared<StageBundle>();
    std::string err;
    if (!bundle->load(dir + "/stage.bin", err)) {
        log_warn("RBR stage: %s", err.c_str());
        flat_arena(g, 60, SURF_ASPHALT, false);
        g.set_spawn(vec3(0, 0, -30), 0);
        g.scene_hint = "This RBR stage is not installed. Run  python3 tools/fetch_rbr_stage.py --stage " + id +
                       "  (downloads the stage from the author's Google Drive and converts it locally), then reload this scene.";
        g.scene_status = err;
        return;
    }
    const StageBundle& B = *bundle;
    auto& hf = g.world.statics.terrain;
    hf.create_sparse(B.nx, B.nz, B.cell, B.origin);
    g.world.statics.has_terrain = true;
    const size_t tn = (size_t)B.tile * B.tile;
    for (size_t t = 0; t < B.tiles.size(); t++) {
        std::copy_n(&B.tile_height[t * tn], tn, hf.tile_heights(B.tiles[t].first, B.tiles[t].second));
        uint8_t* ts = hf.tile_surfaces(B.tiles[t].first, B.tiles[t].second);
        for (size_t i = 0; i < tn; i++) ts[i] = B.tile_surface[t * tn + i] < SURF_COUNT ? B.tile_surface[t * tn + i] : (uint8_t)SURF_DIRT;
    }
    hf.update_bounds();
    // the stage's static shape collision: solid trunks, stumps and walls; bendable trees and bushes yield
    for (const auto& st : B.statics) {
        phys::StaticBox box;
        box.center = st.centre;
        box.rot = st.rot;
        box.half = st.half;
        box.surface = st.deck ? SURF_ASPHALT : SURF_WOOD;
        box.max_force = st.yielding ? 2500.0f : 0.0f; // N per node: a creeping car stops, a fast one pushes through
        box.update_aabb();
        g.world.statics.boxes.push_back(box);
    }
    g.scenery = std::make_unique<StageScenery>();
    g.scenery->build(B, dir);

    // props: the stage's own meshes on light box bodies (knocked over by the cars)
    int n_prop = 0;
    float s_clock = -1, s_finish_sign = -1, s_finish_board = -1;
    StageRoute R0;
    for (const vec3& p : B.line) {
        R0.s.push_back(R0.p.empty() ? 0.0f : R0.s.back() + length(vec2(p.x, p.z) - R0.p.back()));
        R0.p.push_back(vec2(p.x, p.z));
    }
    for (const auto& pr : B.props) {
        const auto& t = B.templates[(size_t)pr.tmpl];
        MeshPropDesc d;
        d.xform = mat4::from_mat3(pr.rot, pr.pos);
        d.hull_min = t.hull_min;
        d.hull_max = t.hull_max;
        d.mass = t.mass;
        for (auto& part : g.scenery->templates[(size_t)pr.tmpl]) d.parts.push_back({part.mesh.get(), part.mat.get()});
        DynamicObject* o = g.add_object(build_mesh_prop(g.world, d, format("%s #%d", t.name.c_str(), n_prop++)));
        // like in RBR the props stay put until something hits them (narrow bases on uneven ground would tip over)
        o->body->sleeping = true;
        std::string lname = to_lower(t.name);
        float s = R0.nearest_s(vec2(pr.pos.x, pr.pos.z));
        if (lname.find("start_clock") != std::string::npos) s_clock = s;
        if (lname.find("finish r") != std::string::npos) s_finish_sign = std::max(s_finish_sign, s); // red FINISH sign
        if (lname.find("finish") != std::string::npos && (lname.find("left") != std::string::npos || lname.find("right") != std::string::npos))
            s_finish_board = std::max(s_finish_board, s);
    }
    int n_bale = 0;
    for (const auto& b : B.bales) // (asleep like the props: some stand on slopes and embankment edges)
        g.add_object(build_standing_bale(g.world, b.pos - vec3(0, 0.02f, 0), b.yaw, b.radius, b.height, format("bale%d", n_bale++), 260.0f))
            ->body->sleeping = true;

    // clock: the stage's own start / finish pacenotes; without them the start clock and the red FINISH sign
    // (the car stands on the start line: the clock starts once it has rolled 2 m)
    auto route = std::make_shared<StageRoute>(std::move(R0));
    float s_start = B.clock_start >= 0 ? B.clock_start : (s_clock >= 0 ? s_clock : 2.0f);
    s_start = std::clamp(s_start + 2.0f, 2.0f, route->length() * 0.5f);
    float s_finish = route->length() - 5.0f;
    if (B.clock_finish > s_start + 50) s_finish = std::min(B.clock_finish, route->length() - 1.0f);
    else if (s_finish_sign > s_start + 50) s_finish = s_finish_sign;
    else if (s_finish_board > s_start + 50) s_finish = s_finish_board;
    vec3 sp = B.start;
    sp.y = g.ground_height(sp.x, sp.z);
    g.set_spawn(sp, B.heading);
    // the autopilot stops a little after the finish (at the STOP control; the collision mesh ends soon after)
    std::vector<vec3> line;
    for (size_t i = 0; i < B.line.size() && route->s[i] < s_finish + 20.0f; i++) line.push_back(vec3(B.line[i].x, g.ground_height(B.line[i].x, B.line[i].z), B.line[i].z));
    // autopilot limits by the surface under the driveline: tarmac corners faster than gravel
    int tarmac = 0;
    for (const vec3& p : line) tarmac += hf.surface_at(p.x, p.z) == SURF_ASPHALT ? 1 : 0;
    const bool on_tarmac = tarmac * 2 > (int)line.size();
    install_stage_timer(g, route, s_start, s_finish, line, on_tarmac ? 26.0f : 24.0f, on_tarmac ? 7.0f : 4.5f,
                        "Cross the start line to start the clock.");

    // morning light (the M variant of the stage)
    g.light.sun_dir = normalize(vec3(0.75f, 0.55f, -0.45f));
    g.light.sun_color = vec3(2.6f, 2.35f, 2.05f);
    g.light.fog_color = vec3(0.7f, 0.76f, 0.84f);
    g.light.fog_density = 0.0008f;
    g.world.settings.wind = vec3(1.0f, 0, 0.6f);
    g.world.settings.wind_radius = 60.0f;
    g.scene_hint = format("%s: a Richard Burns Rally community stage by RALLY Guru (rallyguru-tracks.blogspot.com), %.2f km. "
                          "Original meshes and textures; %d signs, banners and boards are knockable props%s. "
                          "Where the original has no collision (buildings, far scenery) invisible walls close the stage. "
                          "Personal non-commercial use (see assets/stages/%s/SOURCE.txt).",
                          B.title.c_str(), (s_finish - s_start) / 1000.0f, n_prop, n_bale ? ", the hay bales are soft bodies" : "", id.c_str());
    log_info("rbr stage: %zu triangles, %d props, %d bales, route %.0f m (clock %.0f..%.0f m)", g.scenery->triangles, n_prop, n_bale,
             route->length(), s_start, s_finish);
}

// One drivable variant per mod folder (the pack has many variants of some models).
static std::vector<const VehicleEntry*> one_per_model(const std::vector<VehicleEntry>& reg) {
    std::vector<const VehicleEntry*> pool;
    for (auto& e : reg) {
        if (!e.drivable) continue;
        bool seen = false;
        for (auto* p : pool) seen = seen || p->folder == e.folder;
        if (!seen) pool.push_back(&e);
    }
    return pool;
}

void scene_stress_vehicles(Game& g) {
    flat_arena(g, 80);
    // spawn a grid of mixed vehicles, dropped from a small height
    const auto& reg = vehicle_registry();
    std::vector<const VehicleEntry*> pool = one_per_model(reg);
    int count = 16;
    for (int i = 0; i < count && !pool.empty(); i++) {
        const VehicleEntry* e = pool[i % pool.size()];
        vec3 p(-45 + (i % 4) * 30.0f, 1.0f, -45 + (i / 4) * 30.0f);
        if (Vehicle* v = g.spawn_vehicle(e->id, p, (float)(i * 37 % 360), false)) v->ai = true;
    }
    g.set_spawn(vec3(0, 0, -70), 0);
    g.scene_hint = "16 AI vehicles driving around (every model in the pack).";
    g.scene_update = [](Game& gg, float dt) { ai_update_all(gg, dt); };
}

void scene_stress_crates(Game& g) {
    flat_arena(g, 40);
    auto& A = SharedAssets::get();
    int id = 0;
    for (int y = 0; y < 8; y++)
        for (int z = 0; z < 8; z++)
            for (int x = 0; x < 8; x++) {
                SoftBoxDesc d;
                d.center = vec3(-8 + x * 2.2f, 1.0f + y * 1.6f, -8 + z * 2.2f);
                d.size = vec3(1.0f);
                d.nx = d.ny = d.nz = 2;
                d.mass = 40;
                d.rot = quat::axis_angle(vec3(0.3f, 1, 0.1f), (x + y + z) * 0.3f);
                d.mat = std::make_shared<Material>(*A.wood);
                d.mat->diffuse = A.tex_crate;
                d.beams = {1e6f, 1000, 5e4f, 2e5f, 0.0f};
                g.add_object(build_soft_box(g.world, d, format("c%d", id++)));
            }
    g.set_spawn(vec3(0, 0, -30), 0);
    g.scene_hint = "512 soft crates falling into a pile (inter-body collisions, islands, sleeping).";
}

void scene_stress_forest(Game& g) {
    g.create_terrain(301, 301, 1.0f, vec2(-150, -150));
    auto& hf = g.world.statics.terrain;
    te::add_noise(hf, 6.0f, 60.0f, 4, 8);
    te::flatten_circle(hf, vec2(0, 0), 10, hf.height(0, 0), 6, SURF_GRASS);
    te::auto_surfaces(hf);
    g.finish_terrain();
    scatter_trees(g, vec2(0, 0), 140, 400, 91, 0, {}, 1.0f, false);
    g.world.settings.wind = vec3(9.0f, 0, 4.0f);
    g.world.settings.wind_gusts = 0.8f;
    g.set_spawn(vec3(0, hf.height(0, 0), 0), 0);
    g.scene_hint = "400 trees swaying in strong gusty wind: every tree stays awake (worst case for the joint solver).";
}

void scene_stress_derby(Game& g) {
    flat_arena(g, 45, SURF_DIRT);
    const auto& reg = vehicle_registry();
    std::vector<const VehicleEntry*> pool = one_per_model(reg);
    for (int i = 0; i < 12 && !pool.empty(); i++) {
        float a = 2 * kPi * i / 12;
        vec3 p(std::cos(a) * 32, 1.0f, std::sin(a) * 32);
        float yaw = std::atan2(-p.x, -p.z) * kRad2Deg;
        if (Vehicle* v = g.spawn_vehicle(pool[(i * 5) % pool.size()]->id, p, yaw, false)) {
            v->ai = true;
        }
    }
    g.set_spawn(vec3(0, 0, -40), 0);
    g.scene_hint = "Demolition derby: 12 AI vehicles ram each other (heavy inter-body collisions + deformation).";
    g.scene_update = [](Game& gg, float dt) { ai_update_all(gg, dt, true); };
}

void scene_stress_bridge(Game& g) {
    scene_canyon(g);
    g.scene_hint = "Convoy of heavy trucks over the truss bridge.";
    const char* heavy[] = {"tatra_815_6x6", "kenworth_wrecker", "freightliner_fla", "thomas_hdx_bus"};
    int k = 0;
    for (auto& e : vehicle_registry()) {
        for (auto* h : heavy)
            if (e.folder == h && k < 4) {
                if (Vehicle* v = g.spawn_vehicle(e.id, vec3(-60 - k * 16.0f, 0, 0), 90, false)) {
                    v->ai = true;
                    ai_set_route(*v, {vec3(200, 0, 0)}, 25.0f);
                }
                k++;
                break;
            }
    }
}

} // namespace

const std::vector<SceneInfo>& scene_registry() {
    static std::vector<SceneInfo> s = {
        {"proving", "Proving Ground", "Driving", "Vehicle handling test course", scene_proving_ground},
        {"forest", "Forest", "Driving", "Hills with trees and bushes", scene_forest},
        {"offroad", "Offroad Trail", "Driving", "Rocks, mud, sand, logs", scene_offroad},
        {"canyon", "Canyon Bridges", "Driving", "Breakable beam bridges over a chasm", scene_canyon},
        {"tape_maze", "Tape Maze", "Driving", "Gymkhana course marked with tape: lanes, hairpins, chicane, timer", scene_tape_maze},
        {"rally", "Rally Stage", "Rally", "Gravel stage: trees, tape, hay bales, crests, timer", scene_rally},
        {"rbr_verkiai", "RBR: Verkiai SSS", "Rally", "RBR community stage (RALLY Guru): Lithuanian super special in a park",
         [](Game& g) { scene_rbr_stage(g, "verkiai_sss"); }},
        {"rbr_undva", "RBR: Undva", "Rally", "RBR community stage (RALLY Guru): narrow fast Estonian gravel through forest",
         [](Game& g) { scene_rbr_stage(g, "undva"); }},
        {"rbr_travanca", "RBR: Travanca do Monte", "Rally", "RBR community stage (RALLY Guru): Portuguese gravel in wooded hills",
         [](Game& g) { scene_rbr_stage(g, "travanca"); }},
        {"rbr_fernet", "RBR: Fernet Branca", "Rally", "RBR community stage (RALLY Guru): Argentine gravel through the hills",
         [](Game& g) { scene_rbr_stage(g, "fernet_branca"); }},
        {"crash", "Crash Test", "Crash", "Wall, barrier, poles", scene_crash},
        {"vehicle_crash", "Vehicle vs Vehicle", "Crash", "Configurable crash of two vehicles (Scene menu)", scene_vehicle_crash},
        {"frame_car", "Frame Car", "Test cars/Frame Car", "A car on a welded space frame of FEM tubes with sheet panels: drive it, drop it, roll it, crash it",
         scene_frame_car},
        {"fc_headon", "Frame Car: Head-on", "Test cars/Frame Car", "Two Frame Cars head-on at 50 km/h each",
         [](Game& g) { scene_frame_car_test(g, "Head-on into another", vec3(4.6f, 1.9f, 57.5f), vec3(0, 0.6f, 62.5f), "Two Frame Cars meet head-on at 50 km/h each."); }},
        {"fc_side", "Frame Car: Side impact", "Test cars/Frame Car", "Another Frame Car into its side at 50 km/h",
         [](Game& g) {
             scene_frame_car_test(g, "into its side", vec3(6.5f, 2.8f, 53.5f), vec3(-1.5f, 0.7f, 60), "A second Frame Car hits the standing one in the side at 50 km/h.");
         }},
        {"fc_wall", "Frame Car: Wall", "Test cars/Frame Car", "Into a concrete wall at 60 km/h",
         [](Game& g) { scene_frame_car_test(g, "Launch at the wall", vec3(6.5f, 2.4f, 139.5f), vec3(0, 0.7f, 146), "The Frame Car into a concrete wall at 60 km/h."); }},
        {"fc_pole", "Frame Car: Pole", "Test cars/Frame Car", "Into a concrete pole at 50 km/h",
         [](Game& g) { scene_frame_car_test(g, "Launch at the pole", vec3(22, 2.4f, 141), vec3(16, 0.7f, 147), "The Frame Car into a concrete pole at 50 km/h."); }},
        {"fc_drop", "Frame Car: Drop 10 m", "Test cars/Frame Car", "Dropped on its wheels from 10 m",
         [](Game& g) { scene_frame_car_test(g, "Drop from 10", vec3(8, 4.5f, 12), vec3(0, 1.8f, 20), "The Frame Car dropped on its wheels from 10 m."); }},
        {"fc_slab", "Frame Car: Slab", "Test cars/Frame Car", "A 5 t concrete slab dropped on it from 2.5 m",
         [](Game& g) { scene_frame_car_test(g, "Drop a 5 t concrete slab", vec3(6, 3.6f, 14.5f), vec3(0, 1.2f, 20), "A 5 t concrete slab falls on the roof from 2.5 m."); }},
        {"fc_axe", "Frame Car: Giant axe", "Test cars/Frame Car", "A 5 t pendulum axe swings down and cuts it in two",
         [](Game& g) { scene_frame_car_test(g, "The giant axe", vec3(-15.5f, 5.0f, 110.5f), vec3(-25, 3.0f, 120), "A 5 t axe on a pendulum swings down and cuts the car in two."); }},
        {"fc_roll", "Frame Car: Barrel roll", "Test cars/Frame Car", "Thrown up and spun at 50 km/h: it rolls over",
         [](Game& g) { scene_frame_car_test(g, "Barrel roll", vec3(9, 4, 18), vec3(0, 1, 32), "Thrown up and spun at 50 km/h: the Frame Car rolls over."); }},
        {"buggy", "Buggy", "Test cars/Buggy", "A desert racer on a welded tube cage (FEM) with aluminium panels: whoops, a jump, drops, rolls, crashes",
         scene_buggy},
        {"bg_headon", "Buggy: Head-on", "Test cars/Buggy", "Two Buggies head-on at 50 km/h each",
         [](Game& g) { scene_frame_car_test(g, "Head-on into another", vec3(4.6f, 1.9f, 57.5f), vec3(0, 0.6f, 62.5f), "Two Buggies meet head-on at 50 km/h each.", scene_buggy); }},
        {"bg_side", "Buggy: Side impact", "Test cars/Buggy", "Another Buggy into its side at 50 km/h",
         [](Game& g) {
             scene_frame_car_test(g, "into its side", vec3(6.5f, 2.8f, 53.5f), vec3(-1.5f, 0.7f, 60), "A second Buggy hits the standing one in the side at 50 km/h.", scene_buggy);
         }},
        {"bg_wall", "Buggy: Wall", "Test cars/Buggy", "Into a concrete wall at 60 km/h",
         [](Game& g) { scene_frame_car_test(g, "Launch at the wall", vec3(6.5f, 2.4f, 139.5f), vec3(0, 0.7f, 146), "The Buggy into a concrete wall at 60 km/h.", scene_buggy); }},
        {"bg_drop", "Buggy: Drop 10 m", "Test cars/Buggy", "Dropped on its wheels from 10 m",
         [](Game& g) { scene_frame_car_test(g, "Drop from 10", vec3(8, 4.5f, 12), vec3(0, 1.8f, 20), "The Buggy dropped on its wheels from 10 m.", scene_buggy); }},
        {"bg_slab", "Buggy: Slab", "Test cars/Buggy", "A 5 t concrete slab dropped on it from 2.5 m",
         [](Game& g) { scene_frame_car_test(g, "Drop a 5 t concrete slab", vec3(6, 3.9f, 14.5f), vec3(0, 1.4f, 20), "A 5 t concrete slab falls on the cage from 2.5 m.", scene_buggy); }},
        {"bg_roll", "Buggy: Barrel roll", "Test cars/Buggy", "Thrown up and spun at 50 km/h: it rolls over",
         [](Game& g) { scene_frame_car_test(g, "Barrel roll", vec3(9, 4, 18), vec3(0, 1, 32), "Thrown up and spun at 50 km/h: the Buggy rolls over.", scene_buggy); }},
        {"bg_whoops", "Buggy: Whoops", "Test cars/Buggy", "Through the whoops at 80 km/h",
         [](Game& g) { scene_frame_car_test(g, "whoops", vec3(88, 2.5f, 30), vec3(80, 0.8f, 45), "Through ten 0.5 m whoops at 80 km/h.", scene_buggy); }},
        {"bg_jump", "Buggy: Jump", "Test cars/Buggy", "Over the tabletop jump at 90 km/h",
         [](Game& g) { scene_frame_car_test(g, "jump", vec3(95, 4.0f, 160), vec3(80, 2.2f, 172), "Over the 2.2 m tabletop at 90 km/h.", scene_buggy); }},
        {"shell_car", "Shell Car", "Test cars/Shell Car", "A saloon on the BMW E36's lines, all FEM: a body-in-white of members and sheets, parts on hinges, bolts, buffers",
         scene_shell_car},
        {"sc_headon", "Shell Car: Head-on", "Test cars/Shell Car", "Two Shell Cars head-on at 50 km/h each",
         [](Game& g) { scene_frame_car_test(g, "Head-on into another", vec3(5.2f, 2.0f, 56.5f), vec3(0, 0.6f, 62.5f), "Two Shell Cars meet head-on at 50 km/h each.", scene_shell_car); }},
        {"sc_side", "Shell Car: Side impact", "Test cars/Shell Car", "Another Shell Car into its side at 50 km/h",
         [](Game& g) {
             scene_frame_car_test(g, "into its side", vec3(7.0f, 3.0f, 53.0f), vec3(-1.5f, 0.7f, 60), "A second Shell Car hits the standing one in the side at 50 km/h.", scene_shell_car);
         }},
        {"sc_wall", "Shell Car: Wall", "Test cars/Shell Car", "Into a concrete wall at 60 km/h",
         [](Game& g) { scene_frame_car_test(g, "Launch at the wall", vec3(7.0f, 2.5f, 139.0f), vec3(0, 0.7f, 146), "The Shell Car into a concrete wall at 60 km/h.", scene_shell_car); }},
        {"sc_pole", "Shell Car: Pole", "Test cars/Shell Car", "Into a concrete pole at 50 km/h",
         [](Game& g) { scene_frame_car_test(g, "Launch at the pole", vec3(22.5f, 2.5f, 140.5f), vec3(16, 0.7f, 147), "The Shell Car into a concrete pole at 50 km/h.", scene_shell_car); }},
        {"sc_drop", "Shell Car: Drop 10 m", "Test cars/Shell Car", "Dropped on its wheels from 10 m",
         [](Game& g) { scene_frame_car_test(g, "Drop from 10", vec3(8.5f, 4.5f, 12), vec3(0, 1.8f, 20), "The Shell Car dropped on its wheels from 10 m.", scene_shell_car); }},
        {"sc_roof", "Shell Car: On the roof", "Test cars/Shell Car", "Turned over and dropped on its roof from 1.5 m",
         [](Game& g) { scene_frame_car_test(g, "Drop on the roof", vec3(8, 3.0f, 13), vec3(0, 0.8f, 20), "The Shell Car turned over and dropped on its roof from 1.5 m.", scene_shell_car); }},
        {"sc_side_lay", "Shell Car: On its side", "Test cars/Shell Car", "Laid on its side from 0.3 m: the doors rest on their openings, the wheels keep their toe",
         [](Game& g) { scene_frame_car_test(g, "Lay it on its side", vec3(0, 2.2f, 13), vec3(0, 0.8f, 20), "The Shell Car laid on its side from 0.3 m.", scene_shell_car); }},
        {"sc_slab", "Shell Car: Slab", "Test cars/Shell Car", "A 5 t concrete slab dropped on it from 2.5 m",
         [](Game& g) { scene_frame_car_test(g, "Drop a 5 t concrete slab", vec3(6.5f, 3.8f, 14), vec3(0, 1.2f, 20), "A 5 t concrete slab falls on the roof from 2.5 m.", scene_shell_car); }},
        {"sc_axe", "Shell Car: Giant axe", "Test cars/Shell Car", "A 5 t pendulum axe swings down and cuts it in two",
         [](Game& g) { scene_frame_car_test(g, "The giant axe", vec3(-15.5f, 5.0f, 110.0f), vec3(-25, 3.0f, 120), "A 5 t axe on a pendulum swings down and cuts the car in two.", scene_shell_car); }},
        {"sc_roll", "Shell Car: Barrel roll", "Test cars/Shell Car", "Thrown up and spun at 50 km/h: it rolls over",
         [](Game& g) { scene_frame_car_test(g, "Barrel roll", vec3(9.5f, 4, 18), vec3(0, 1, 32), "Thrown up and spun at 50 km/h: the Shell Car rolls over.", scene_shell_car); }},
        {"sc_curb", "Shell Car: Curb", "Test cars/Shell Car", "Sideways into a curb at 40 km/h",
         [](Game& g) { scene_frame_car_test(g, "Trip over the curb", vec3(-32, 3.0f, 51), vec3(-41, 0.8f, 60), "The Shell Car slides sideways into a curb at 40 km/h.", scene_shell_car); }},
        {"sc_whoops", "Shell Car: Whoops", "Test cars/Shell Car", "Through the whoops at 80 km/h",
         [](Game& g) { scene_frame_car_test(g, "whoops", vec3(88, 2.5f, 30), vec3(80, 0.8f, 45), "Through ten 0.5 m whoops at 80 km/h.", scene_shell_car); }},
        {"sc_jump", "Shell Car: Jump", "Test cars/Shell Car", "Over the tabletop jump at 90 km/h",
         [](Game& g) { scene_frame_car_test(g, "jump", vec3(95, 4.0f, 160), vec3(80, 2.2f, 172), "Over the 2.2 m tabletop at 90 km/h.", scene_shell_car); }},
        {"sc_latches", "Shell Car: Latches let go", "Test cars/Shell Car", "The hood's, the lid's and the doors' latches let go: they rest on their buffers and stays",
         [](Game& g) { scene_frame_car_test(g, "Let the latches go", vec3(7, 2.6f, -5.5f), vec3(0, 0.8f, 0), "The latches let go: the hood and the lid rest on their buffers.", scene_shell_car); }},
        {"sheet_car", "Sheet Car", "Test cars", "A car whose body panels are a steel sheet of triangle elements: crash it into a parked one, poles or a wall",
         scene_sheet_car},
        {"lab", "Primitives", "Physics lab", "Primitive tests: collisions, deformation, breaking, joints", scene_lab},
        {"materials", "Materials Lab", "Physics lab", "Sheets, shapes and bars of 10 materials: drop, press and bending tests, drive-through panels",
         scene_materials},
        {"sheet_shapes", "Sheet Shapes", "Physics lab", "Sheets of other shapes and sizes: steel gate 6 x 4 m, half-pipe, disc, ring, triangle, L, dome",
         scene_sheet_shapes},
        {"sheet_run", "Sheet Run", "Physics lab", "Drive through three lead sheets clamped in gates (profiling of sheet fracture)", scene_sheet_run},
        {"barrels", "Steel Barrels", "Physics lab", "200 l steel drums of triangle elements: drop, roll, stack and crash them", scene_barrels},
        {"fem_shells", "FEM Shells", "Physics lab", "Triangle elements of the FEM frame: a steel sheet, a cantilever and a hollow cube", scene_fem_shells},
        {"stress_vehicles", "Stress: 16 vehicles", "Stress", "Many AI vehicles", scene_stress_vehicles},
        {"stress_derby", "Stress: Demolition derby", "Stress", "12 AI vehicles colliding", scene_stress_derby},
        {"stress_crates", "Stress: 512 crates", "Stress", "Pile of soft boxes", scene_stress_crates},
        {"stress_forest", "Stress: Windy forest", "Stress", "400 trees in wind", scene_stress_forest},
        {"stress_bridge", "Stress: Bridge convoy", "Stress", "Heavy trucks break the bridge", scene_stress_bridge},
        {"stress_barrels", "Stress: Barrel pile", "Stress", "15 steel barrels in a pile, three more thrown into it", scene_stress_barrels},
        {"editor", "Model Editor", "Tools", "The model editor's stage: a flat square, a neutral background (Editor menu, Ctrl+E)", scene_editor},
    };
    return s;
}

} // namespace bl
