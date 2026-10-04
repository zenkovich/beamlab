// The driving scenes: the Proving Ground, the forest, the canyon's bridges, the off-road trail, the crash tests.
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


// The start scene: a small handling course on painted terrain (the large test site with its roads, crash lanes and
// machines is the Test Site: scene_test_site.cpp)
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
    // (diagnostics, BL_CRASH=<vehicle a>,<vehicle b>[,<km/h a>,<km/h b>[,<layout>[,<slow motion 0/1>]]]: the pair and
    // the run without the menu - profiling a crash at full speed)
    if (const char* e = getenv("BL_CRASH")) {
        const std::vector<std::string> f = split_any(e, ",");
        if (f.size() >= 2) g.crash.vehicle_a = f[0], g.crash.vehicle_b = f[1];
        if (f.size() >= 4) g.crash.speed_a = (float)atof(f[2].c_str()), g.crash.speed_b = (float)atof(f[3].c_str());
        if (f.size() >= 5) g.crash.layout = atoi(f[4].c_str());
        if (f.size() >= 6) g.crash.slow_motion = atoi(f[5].c_str()) != 0;
    }
    g.set_spawn(vec3(0, 0, -40), 0);
    g.no_player_vehicle = true;
    g.scene_hint = "Two vehicles collide at the marked point. Choose the vehicles, speeds and crash layout in the Scene menu, F5 restarts.";
    g.scene_update = [](Game& gg, float dt) { gg.crash_update(dt); };
    g.run_crash();
}

} // namespace bl
