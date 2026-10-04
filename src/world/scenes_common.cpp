// What the scenes share: cones, ramps, trees, a flat arena; the model editor's stage.
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


quat yaw_q(float deg) { return quat::axis_angle(vec3(0, 1, 0), deg * kDeg2Rad); }

void add_cone_line(Game& g, vec3 a, vec3 b, int n) {
    for (int i = 0; i < n; i++) {
        float t = n > 1 ? (float)i / (n - 1) : 0;
        ConeDesc d;
        d.base = lerp(a, b, t);
        d.base.y = g.world.statics.terrain.height(d.base.x, d.base.z);
        g.add_object(build_traffic_cone(g.world, d, format("cone%d", i)));
    }
}

void scatter_trees(Game& g, vec2 center, float radius, int count, uint32_t seed, float road_clear, const std::vector<vec2>& road, float scale,
                   bool bushes) {
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

// ------------------------------------------------------------------------------------------- scenes
void add_ramp(Game& g, vec3 foot, float yaw_deg, float length, float width, float height, MaterialPtr mat, uint8_t surf) {
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

std::vector<vec2> ellipse(vec2 c, vec2 r, int n, float phase) {
    std::vector<vec2> p;
    for (int i = 0; i <= n; i++) {
        float a = phase + 2 * kPi * i / n;
        p.push_back(c + vec2(std::cos(a) * r.x, std::sin(a) * r.y));
    }
    return p;
}

void flat_arena(Game& g, float half, uint8_t surf, bool walls) {
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

// (the trees' scatter for the scenes of other files)
void scene_scatter_trees(Game& g, vec2 center, float radius, int count, uint32_t seed, float road_clear, const std::vector<vec2>& road) {
    scatter_trees(g, center, radius, count, seed, road_clear, road);
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

} // namespace bl
