// The stress scenes: many vehicles, crates, trees, a bridge under a convoy.
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


// One drivable variant per mod folder (the pack has many variants of some models).
std::vector<const VehicleEntry*> one_per_model(const std::vector<VehicleEntry>& reg) {
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

} // namespace bl
