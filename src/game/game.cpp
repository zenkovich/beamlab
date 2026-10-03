#include "game/game.h"
#include "core/jobs.h"
#include "core/profiler.h"
#include "core/util.h"
#include "vehicle/vehicle.h"

#include <algorithm>
#include <unordered_map>

namespace bl {

Game::Game() = default;
Game::~Game() = default;

bool Game::init() {
    SharedAssets::get().init();
    return true;
}

Vehicle* Game::player_vehicle() const { return player >= 0 && player < (int)vehicles.size() ? vehicles[player].get() : nullptr; }

void Game::create_terrain(int nx, int nz, float cell, vec2 origin) {
    world.statics.terrain.create(nx, nz, cell, origin);
    world.statics.has_terrain = true;
}

float Game::ground_height(float x, float z) const {
    const auto& st = world.statics;
    float h = st.has_terrain ? st.terrain.height(x, z) : 0.0f;
    if (h < -1e20f) h = 0.0f;
    if (st.road) {
        float s, lat;
        if (st.road->locate(x, z, s, lat)) h += st.road->detail(s, lat);
    }
    return h;
}

void Game::finish_terrain() {
    world.statics.terrain.update_bounds();
    if (world.statics.road) {
        std::vector<float> drop = RoadRender::terrain_drop(*world.statics.road, world.statics.terrain);
        terrain_render.build(world.statics.terrain, &drop);
        road_render = std::make_unique<RoadRender>();
        road_render->build(*world.statics.road, world.statics.terrain, make_road_material(world.statics.road->half_width, world.statics.road->edge));
    } else {
        terrain_render.build(world.statics.terrain);
    }
}

void Game::add_static_box(vec3 center, vec3 half, const quat& rot, uint8_t surface, MaterialPtr mat, bool render) {
    world.statics.add_box(center, half, rot, surface);
    if (render && mat) m_static_visuals.push_back({&SharedAssets::get().box, mat, mat4::from_trs(center, rot, half * 2.0f)});
}

void Game::add_static_cylinder(vec3 base, float radius, float height, uint8_t surface, MaterialPtr mat) {
    world.statics.cylinders.push_back({base, radius, height, surface});
    if (mat) m_static_visuals.push_back({&SharedAssets::get().cylinder, mat, mat4::translate(base) * mat4::scale(vec3(radius, height, radius))});
}

DynamicObject* Game::add_object(std::unique_ptr<DynamicObject> o) {
    objects.push_back(std::move(o));
    return objects.back().get();
}

void Game::presettle_static_objects() {
    PROFILE_ZONE("Presettle all");
    std::vector<DynamicObject*> list;
    for (auto& o : objects)
        if (o->body && o->body->is_static_like) list.push_back(o.get());
    JobSystem::get().parallel_items((int)list.size(), [&](int i, int) {
        DynamicObject* o = list[i];
        bool trees = !o->body->joints.empty();
        presettle(*o->body, world, trees ? 1.2f : 2.0f, trees);
    });
}

Vehicle* Game::spawn_vehicle(const std::string& vid, vec3 pos, float yaw, bool make_player) {
    const VehicleEntry* e = find_vehicle(vid);
    if (!e) {
        log_error("unknown vehicle '%s'", vid.c_str());
        return nullptr;
    }
    std::string err;
    auto v = Vehicle::create(*e, world, pos, yaw, &err);
    if (!v) {
        log_error("failed to spawn '%s': %s", vid.c_str(), err.c_str());
        return nullptr;
    }
    vehicles.push_back(std::move(v));
    Vehicle* ptr = vehicles.back().get();
    apply_vehicle_sheet_body(ptr, make_player);
    if (make_player) {
        for (auto& o : vehicles) o->is_player = false;
        player = (int)vehicles.size() - 1;
        ptr->is_player = true;
        cam.mode = CameraController::CHASE;
        cam.snap_behind(ptr->forward());
    }
    return ptr;
}

void Game::remove_vehicle(Vehicle* v) {
    for (size_t i = 0; i < vehicles.size(); i++)
        if (vehicles[i].get() == v) {
            if (grab_holds(v->body)) grab_end();
            world.remove_body(v->body);
            vehicles.erase(vehicles.begin() + i);
            if (player == (int)i) player = -1;
            else if (player > (int)i) player--;
            if (m_crash_a == v) m_crash_a = nullptr;
            if (m_crash_b == v) m_crash_b = nullptr;
            return;
        }
}

void Game::clear_vehicles() {
    grab_end();
    for (auto& v : vehicles) world.remove_body(v->body);
    vehicles.clear();
    player = -1;
}

void Game::load_scene(int index) {
    PROFILE_ZONE("Load scene");
    const auto& reg = scene_registry();
    if (index < 0 || index >= (int)reg.size()) return;
    grab_end();
    vehicles.clear();
    player = -1;
    objects.clear();
    m_static_visuals.clear();
    world.clear();
    world.statics = phys::StaticWorld();
    // a piece cracked off a sheet becomes its own object with the sheet's look
    world.on_piece = [this](phys::SoftBody* parent, phys::SoftBody* piece) {
        if (!piece->fem.empty() && piece->shells.empty()) { // (debris of a torn frame: its tubes, a torn-off panel's plates)
            auto po = std::make_unique<DynamicObject>();
            po->name = piece->name;
            po->body = piece;
            po->frame = std::make_unique<FrameVisual>();
            po->frame->mat = frame_tube_material();
            for (const auto& v : vehicles) // (a car's: in its paint)
                if (v->body == parent && v->frame_visual()) {
                    const FrameVisual& fv = *v->frame_visual();
                    if (fv.mat) po->frame->mat = fv.mat;
                    po->frame->plate_mat = fv.plate_mat;
                    po->frame->section_mats = fv.section_mats;
                }
            objects.push_back(std::move(po));
            return;
        }
        for (auto& o : objects)
            if (o->body == parent && o->sheet) {
                auto po = std::make_unique<DynamicObject>();
                po->name = o->name + " (piece)";
                po->body = piece;
                po->sheet = std::make_unique<ShellVisual>();
                po->sheet->mat = o->sheet->mat;
                po->sheet->thickness = o->sheet->thickness;
                if (!piece->fem.empty()) { // (a piece of a sheet on a frame: its part of the frame too)
                    po->frame = std::make_unique<FrameVisual>();
                    po->frame->mat = o->frame ? o->frame->mat : frame_tube_material();
                }
                objects.push_back(std::move(po));
                return;
            }
    };
    road_render.reset();
    grass.reset();
    scenery.reset(); // (after the objects: props draw the scenery's template meshes)
    world.settings.wind = vec3(0);
    world.settings.wind_radius = 0;
    light = LightSettings();
    scene_update = nullptr;
    scene_hint.clear();
    scene_status.clear();
    scene_banner.clear();
    scene_actions.clear();
    labels.clear();
    no_player_vehicle = false;
    projectiles.clear();
    m_crash_a = m_crash_b = nullptr;
    m_crash_phase = 0;
    spawn_pos = vec3(0, 0, 0);
    spawn_yaw = 0;
    scene_index = index;
    double t0 = time_seconds();
    reg[index].build(*this);
    world.statics.build_grid();
    presettle_static_objects();
    for (auto& o : objects) o->visuals_dirty = true;
    log_info("scene '%s' built in %.0f ms: %d bodies, %d objects", reg[index].name.c_str(), (time_seconds() - t0) * 1000.0,
             (int)world.bodies().size(), (int)objects.size());
    // spawn the player vehicle
    if (spawn_override) {
        spawn_pos = spawn_override_pos;
        spawn_pos.y = world.statics.has_terrain ? world.statics.terrain.height(spawn_pos.x, spawn_pos.z) : 0.0f;
        spawn_yaw = spawn_override_yaw;
    }
    if (!no_player_vehicle && !selected_vehicle.empty() && selected_vehicle != "none" && !player_vehicle()) {
        vec3 p = spawn_pos;
        spawn_vehicle(selected_vehicle, p, spawn_yaw, true);
    }
    if (!player_vehicle() && !no_player_vehicle) {
        vec3 to = vec3(0, 0, 0) - spawn_pos;
        to.y = 0;
        vec3 dir = normalize_or(to, vec3(0, 0, 1));
        cam.look_free(spawn_pos - dir * 6.0f + vec3(0, 7, 0), spawn_pos + dir * 30.0f);
    }
}

vec3 Game::camera_focus() const {
    if (Vehicle* v = player_vehicle()) return v->position();
    return m_last_cam.pos + m_last_cam.forward() * 12.0f;
}

void Game::update(float dt, const VehicleInput& in, const CameraInput& cam_in) {
    Vehicle* pv = player_vehicle();
    {
        PROFILE_ZONE("Game logic");
        for (auto& v : vehicles)
            if (v.get() == pv) v->set_input(in);
        if (scene_update) scene_update(*this, dt);
    }
    world.settings.wind_focus = m_last_cam.pos;
    if (!paused || step_once) {
        float fdt = paused ? world.settings.dt * 20 / std::max(0.01f, world.settings.time_scale) : dt;
        uint64_t t0 = prof::now();
        world.step_frame(fdt);
        frame_physics_ms = (float)prof::ticks_to_ms(prof::now() - t0);
        step_once = false;
    } else {
        frame_physics_ms = 0;
    }
    {
        PROFILE_ZONE("Vehicles frame");
        // vehicles get the simulated time of this frame (slow motion / pause aware)
        float sim_dt = world.stats().substeps * world.settings.dt;
        for (auto& v : vehicles) v->update_frame(paused && !step_once ? 0.0f : sim_dt);
        if (grass) grass->update(vehicles, sim_dt, [this](float x, float z) { return ground_height(x, z); });
    }
    // camera
    if (pv) cam.update(dt, cam_in, true, pv->position(), pv->forward(), pv->velocity(), pv->cockpit_pos(), pv->up());
    else cam.update(dt, cam_in, false, vec3(0), vec3(0, 0, 1), vec3(0), vec3(0), vec3(0, 1, 0));
}

void Game::render(Renderer& r, int w, int h, const Camera* cam_override, bool visuals) {
    Camera c = cam_override ? *cam_override : cam.camera();
    r.begin_frame(w, h, c, light, (float)world.time());
    m_last_cam = r.camera();
    if (visuals) {
        PROFILE_ZONE("Visual update");
        // skin/update dynamic visuals in parallel (CPU work only; GPU upload happens below on this thread)
        std::vector<DynamicObject*> objs;
        for (auto& o : objects)
            if (o->needs_visual_update()) objs.push_back(o.get());
        // CPU skinning for vehicles and objects in parallel, GL uploads afterwards on this thread
        std::vector<uint8_t> need((size_t)objs.size(), 0);
        const int nv = (int)vehicles.size();
        JobSystem::get().parallel_for(nv + (int)objs.size(), 1, [&](int b0, int b1, int) {
            for (int i = b0; i < b1; i++) {
                if (i < nv) vehicles[i]->update_visuals();
                else need[i - nv] = objs[i - nv]->prepare_visuals() ? 1 : 0;
            }
        });
        for (size_t i = 0; i < objs.size(); i++)
            if (need[i]) objs[i]->upload_visuals();
    }
    {
        PROFILE_ZONE("Draw submit");
        if (scenery) scenery->draw(r);
        else if (!debug.hide_terrain) terrain_render.draw(r);
        if (road_render && !getenv("BL_NOROAD")) road_render->draw(r);
        if (grass && !debug.hide_meshes) grass->draw(r, c.pos);
        for (auto& s : m_static_visuals) r.draw_mesh(s.mesh, s.mat.get(), s.model);
        m_instances.begin();
        if (!debug.hide_meshes) {
            for (auto& o : objects) o->draw(r, m_instances, debug.xray ? debug.xray_alpha : 0.0f);
        }
        for (auto& v : vehicles) v->draw(r, m_instances, debug);
        m_instances.flush(r);
    }
    draw_debug(r);
}

void Game::draw_debug(Renderer& r) {
    PROFILE_ZONE("Debug geometry");
    // tool indicator under the cursor
    if (tool == Tool::Laser && laser_active) {
        // the beam: from below the eye to what the cursor points at
        const vec3 end = cursor_valid ? cursor_point : laser_origin + laser_dir * laser_range;
        const vec3 side = normalize_or(cross(laser_dir, vec3(0, 1, 0)), vec3(1, 0, 0));
        const vec3 muzzle = laser_origin + laser_dir * 0.8f - vec3(0, 0.35f, 0) + side * 0.25f;
        r.line(muzzle, end, Renderer::rgba(1.0f, 0.12f, 0.08f, 0.95f));
        r.line(muzzle + vec3(0, 0.004f, 0), end, Renderer::rgba(1.0f, 0.55f, 0.45f, 0.6f));
        r.point(end, Renderer::rgba(1.0f, 0.9f, 0.7f));
    }
    // a wire sphere: three circles round p
    auto sphere = [&](vec3 p, float rad, uint32_t col) {
        const int seg = 32;
        for (int axis = 0; axis < 3; axis++)
            for (int i = 0; i < seg; i++) {
                float a0 = 2 * kPi * i / seg, a1 = 2 * kPi * (i + 1) / seg;
                auto pt = [&](float a) {
                    vec3 o(std::cos(a), std::sin(a), 0);
                    if (axis == 1) o = vec3(o.x, 0, o.y);
                    if (axis == 2) o = vec3(0, o.x, o.y);
                    return p + o * rad;
                };
                r.line(pt(a0), pt(a1), col);
            }
    };
    // the grab: its line, the sphere it holds at the target and its nodes, brighter the harder they are pulled (the
    // pull falls off to the sphere's edge); not pulling, its sphere under the cursor
    if (grab_active && grab_body && grab_body->grab_node >= 0 && grab_body->grab_node < (int)grab_body->nodes.size()) {
        r.line(grab_body->nodes[grab_body->grab_node].p, grab_body->grab_target, Renderer::rgba(1, 1, 0.2f));
        r.point(grab_body->grab_target, Renderer::rgba(1, 1, 0.2f));
        for (const phys::SoftBody* b : grab_bodies)
            for (size_t j = 0; j < b->grab_nodes.size() && j < b->grab_w.size(); j++)
                if (b->grab_nodes[j] < b->nodes.size()) r.point(b->nodes[b->grab_nodes[j]].p, Renderer::rgba(1, 1, 0.2f, 0.15f + 0.85f * b->grab_w[j]));
        if (grab_radius > 0.01f) sphere(grab_body->grab_target, grab_radius, Renderer::rgba(1, 1, 0.2f, 0.6f));
    } else if (tool == Tool::Grab && cursor_valid && grab_radius > 0.01f) {
        sphere(cursor_point, grab_radius, Renderer::rgba(1, 1, 0.2f, 0.5f));
    }
    if (cursor_valid && tool != Tool::Grab) {
        vec3 p = cursor_point;
        if (tool == Tool::Laser) {
            float s = 0.06f + 0.004f * length(p - m_last_cam.pos);
            uint32_t col = Renderer::rgba(1.0f, 0.2f, 0.15f, 0.9f);
            r.line(p - vec3(s, 0, 0), p + vec3(s, 0, 0), col);
            r.line(p - vec3(0, s, 0), p + vec3(0, s, 0), col);
        } else if (tool == Tool::Destroy) {
            sphere(p, destroy_radius, Renderer::rgba(1.0f, 0.35f, 0.2f, 0.9f));
        } else {
            float s = 0.15f + 0.01f * length(p - m_last_cam.pos);
            uint32_t col = Renderer::rgba(0.3f, 1.0f, 0.4f, 0.9f);
            r.line(p - vec3(s, 0, 0), p + vec3(s, 0, 0), col);
            r.line(p - vec3(0, s, 0), p + vec3(0, s, 0), col);
            r.line(p - vec3(0, 0, s), p + vec3(0, 0, s), col);
        }
    }
    debug_labels.clear();
    if (!(debug.beams || debug.nodes || debug.collision || debug.islands || debug.wheels || debug.volumes)) {
        m_vol_hits.clear();
        return;
    }
    const float cull2 = 120.0f * 120.0f;
    const bool labels = debug.labels && (debug.beams || debug.wheels || debug.volumes);
    // a ring tyre (Wheel::ring): its wheel's own pose - its rim (steel blue, the rigid disc; five spokes turn with it), its
    // bearings on the axle nodes (yellow: the axle, the wheel's axle through them) - its tread's rows of points as the
    // physics has them - pressed in towards the axle, shifted on the rim and sheared along the ground - white, orange to
    // red as far as they are pressed in (of the sidewall's height), the ring as it would be unpressed faint over the
    // patch, each sidewall from the rim to the tread coloured by its own side's pressing-in (magenta: folded over); the
    // patch's points and their shift and shear (x10, green); its load and slip
    auto ring_tyre = [&](const phys::SoftBody& b, const phys::Wheel& w) {
        const vec3 a0 = b.nodes[w.axle0].p, a1 = b.nodes[w.axle1].p;
        const mat3 Rw = to_mat3(w.rot);
        const vec3 c = w.pos, ax = Rw.c[0], e1 = Rw.c[1], e2 = Rw.c[2];
        const float R = w.radius, W = w.width, rim = std::min(w.rim_radius, 0.95f * R), wall = std::max(0.01f, R - rim);
        const int NP = (int)w.squash.size(), N = NP > 0 ? NP : std::max(8, w.ring_n);
        const bool sides = NP > 0 && (int)w.side_sq.size() == 2 * NP && (int)w.fold.size() == 2 * NP;
        auto dir = [&](int i) {
            const float th = 2.0f * kPi * (float)i / (float)N;
            return e1 * std::cos(th) + e2 * std::sin(th);
        };
        auto sq = [&](int i) { return NP > 0 ? w.squash[i % N] : 0.0f; };
        auto sqs = [&](int i, int sd) { return sides ? w.side_sq[2 * (i % N) + sd] : sq(i); };
        auto fold = [&](int i, int sd) { return sides ? w.fold[2 * (i % N) + sd] : 0.0f; };
        auto sh = [&](int i) { return NP > 0 && (int)w.shift.size() == NP ? w.shift[i % N] : vec3(0); };
        auto tread = [&](int i, float lat) { return c + ax * lat + dir(i) * (R - std::min(wall, sqs(i, lat < 0 ? 0 : 1))) + sh(i); };
        auto press_col = [&](float s, float a) {
            const float u = clampf(s / wall, 0, 1);
            return s > 1e-4f ? Renderer::rgba(1.0f, 0.62f - 0.5f * u, 0.15f, a) : Renderer::rgba(0.88f, 0.9f, 0.92f, 0.75f * a);
        };
        const uint32_t steel = Renderer::rgba(0.55f, 0.72f, 1.0f);
        r.line(a0, a1, Renderer::rgba(1, 1, 0));
        r.line(c - ax * w.bearing_half, c + ax * w.bearing_half, Renderer::rgba(1, 0.8f, 0.2f));
        for (int side = 0; side < 2; side++) {
            const vec3 face = c + ax * ((side ? 0.46f : -0.46f) * W);
            for (int i = 0; i < N; i++) r.thick_line(face + dir(i) * rim, face + dir(i + 1) * rim, steel, 2.0f);
            for (int k = 0; k < 5; k++) r.line(face, face + dir(k * N / 5) * rim, steel);
        }
        constexpr int kRows = phys::Wheel::kRingRows;
        for (int j = 0; j < kRows; j++) {
            const float lat = ((float)j - 0.5f * (kRows - 1)) * W / (float)kRows;
            const int sd = lat < 0 ? 0 : 1;
            for (int i = 0; i < N; i++) {
                const float s = std::max(sqs(i, sd), sqs(i + 1, sd));
                r.thick_line(tread(i, lat), tread(i + 1, lat), press_col(s, 1.0f), s > 1e-4f ? 2.5f : 1.5f);
                if (j == kRows / 2 && std::max(sq(i), sq(i + 1)) > 1e-4f) r.line(c + dir(i) * R, c + dir(i + 1) * R, Renderer::rgba(1, 1, 1, 0.3f));
            }
        }
        for (int i = 0; i < N; i += 2) // (the sidewalls: one each side on every other point)
            for (int side = 0; side < 2; side++) {
                const float f = side ? 1.0f : -1.0f;
                const uint32_t col = fold(i, side) > 0.5f ? Renderer::rgba(1.0f, 0.2f, 0.9f) : press_col(sqs(i, side), 0.55f);
                r.line(c + ax * (0.46f * f * W) + dir(i) * rim, tread(i, f * 0.375f * W), col);
            }
        vec3 patch(0);
        int np = 0;
        for (int i = 0; i < NP; i++) {
            if (w.squash[i] <= 1e-4f) continue;
            const vec3 p = tread(i, 0.0f);
            r.point(p, press_col(w.squash[i], 1.0f));
            if (i < (int)w.shift.size()) r.line(p, p + w.shift[i] * 10.0f, Renderer::rgba(0.3f, 1.0f, 0.4f));
            patch += p, np++;
        }
        if (np > 0) r.line(patch / (float)np, patch / (float)np + vec3(0, 1, 0) * std::min(1.0f, w.load * 1e-4f), Renderer::rgba(1.0f, 0.4f, 0.3f));
        if (labels) {
            const vec3 vc = w.vel, f = normalize_or(cross(ax, vec3(0, 1, 0)), vec3(1, 0, 0)); // (rolling: vc = spin R f)
            const float vx = dot(vc, f), vt = w.spin * R;
            char t[96];
            if (std::fabs(vx) > 0.5f || std::fabs(vt) > 0.5f)
                snprintf(t, sizeof(t), "%.1f kN  slip %+.0f%%  side %.0f mm", w.load * 1e-3f, 100.0f * (vt - vx) / std::max(std::fabs(vx), 1.0f), w.lat_most * 1e3f);
            else
                snprintf(t, sizeof(t), "%.1f kN  side %.0f mm", w.load * 1e-3f, w.lat_most * 1e3f);
            debug_labels.push_back({c + vec3(0, R + 0.12f, 0), t, vec4(1.0f, 0.85f, 0.5f, 1)});
        }
    };
    // a body's collision volumes (the volumes view, F7: a car's engine, its bays, seats, trunk): their hulls' edges,
    // magenta (a drawn one - an engine - pinker), yellow while something is pressing into it, a ring tyre's drum (other
    // bodies' only) pale blue; their anchors (the nodes they follow) dots; one crushed off: its name in red at its anchors
    auto volumes = [&](const phys::SoftBody& b) {
        std::vector<int>& seen = m_vol_hits[&b];
        seen.resize(b.volumes.size(), -1);
        for (size_t k = 0; k < b.volumes.size(); k++) {
            const phys::CollisionVolume& cv = b.volumes[k];
            const bool hit = seen[k] >= 0 && cv.hits > seen[k];
            seen[k] = cv.hits;
            if (cv.broken) {
                vec3 m(0);
                for (uint32_t a : cv.anchors) m += b.nodes[a].p;
                if (labels && !cv.anchors.empty()) debug_labels.push_back({m / (float)cv.anchors.size(), cv.name + " (off)", vec4(1.0f, 0.35f, 0.3f, 1)});
                continue;
            }
            if (!cv.placed || cv.wverts.size() != cv.verts.size()) continue;
            const uint32_t col = cv.tyre ? Renderer::rgba(0.45f, 0.8f, 1.0f, 0.6f)
                               : hit     ? Renderer::rgba(1.0f, 0.95f, 0.3f, 1.0f)
                               : cv.color.x >= 0 ? Renderer::rgba(1.0f, 0.55f, 0.95f, 0.95f) : Renderer::rgba(1.0f, 0.25f, 0.85f, 0.95f);
            const float px = cv.tyre ? 1.5f : hit ? 3.5f : 2.5f;
            for (const auto& f : cv.faces)
                for (size_t e = 0; e < f.size(); e++) r.thick_line(cv.wverts[f[e]], cv.wverts[f[(e + 1) % f.size()]], col, px);
            if (cv.tyre) continue;
            for (uint32_t a : cv.anchors) r.point(b.nodes[a].p, Renderer::rgba(1.0f, 0.4f, 0.9f, 0.7f));
            if (labels) debug_labels.push_back({cv.c, cv.name, vec4(1.0f, 0.6f, 0.95f, 1)});
        }
    };
    // Sheets are drawn as slabs of their thickness around the nodes' plane, so their edges and collision triangles (in that
    // plane) would be hidden inside the slab: they are lifted to the face towards the camera (along the node normals of the
    // sheet's mesh), a little further with distance for the depth buffer's precision.
    std::unordered_map<const phys::SoftBody*, const ShellVisual*> sheet_visual;
    for (const auto& o : objects)
        if (o->sheet && o->body) sheet_visual[o->body] = o->sheet.get();
    std::vector<vec3> lift;
    auto lift_nodes = [&](const phys::SoftBody& b) {
        lift.assign(b.nodes.size(), vec3(0));
        auto it = sheet_visual.find(&b);
        const ShellVisual* sv = it != sheet_visual.end() ? it->second : nullptr;
        const float h = sv ? sv->thickness * 0.5f : 0.0f;
        const bool have = sv && sv->node_normal.size() == b.nodes.size();
        if (!have)
            for (const phys::Shell& sh : b.shells) {
                const vec3 p0 = b.nodes[sh.n[0]].p;
                const vec3 n = cross(b.nodes[sh.n[1]].p - p0, b.nodes[sh.n[2]].p - p0);
                for (int c = 0; c < 3; c++) lift[sh.n[c]] += n;
            }
        for (size_t i = 0; i < b.nodes.size(); i++) {
            const vec3 n = have ? sv->node_normal[i] : normalize_or(lift[i], vec3(0));
            if (length2(n) == 0) {
                lift[i] = vec3(0);
                continue;
            }
            const vec3 to_cam = m_last_cam.pos - b.nodes[i].p;
            const float d = length(to_cam);
            lift[i] = n * ((dot(to_cam, n) >= 0 ? 1.0f : -1.0f) * (h + 0.002f + 0.0008f * d));
        }
    };
    // the beam view's forces and stresses: the most loaded elements near the camera (a quarter of their limit or more)
    // named by their load - a member its axial force and its load against its yield (or buckling) limit, a triangle
    // element its stress and that, a beam its force, a sheet's triangle its stretch against its fracture strain
    struct Loaded {
        float util;
        vec3 at;
        std::string text;
    };
    std::vector<Loaded> loaded;
    // the load colours (the beam view; F8: its deformation's instead): grey unloaded, full at its limit - a member's and
    // a beam's blue in tension, red in compression, amber loaded by its bending; a plate's (a triangle element's, a
    // sheet's) grey to yellow to red
    const vec3 kIdle(0.52f, 0.55f, 0.60f), kTension(0.15f, 0.45f, 1.0f), kCompression(1.0f, 0.15f, 0.1f), kBending(1.0f, 0.68f, 0.1f);
    auto ramp = [&](float u, vec3 hi) {
        u = clampf(u, 0, 1);
        const vec3 c = kIdle + (hi - kIdle) * u;
        return Renderer::rgba(c.x, c.y, c.z, 0.6f + 0.4f * u);
    };
    auto heat = [&](float u) {
        u = clampf(u, 0, 1);
        const vec3 y(1.0f, 0.85f, 0.15f), rd(1.0f, 0.12f, 0.08f), c = u < 0.5f ? kIdle + (y - kIdle) * (2.0f * u) : y + (rd - y) * (2.0f * u - 1.0f);
        return Renderer::rgba(c.x, c.y, c.z, 0.6f + 0.4f * u);
    };
    auto load_label = [&](float util, vec3 at, const char* fmt, float v) {
        if (!(labels && debug.beams) || util < 0.25f || length2(at - m_last_cam.pos) > 10.0f * 10.0f) return;
        char t[64];
        snprintf(t, sizeof(t), fmt, v, std::min(999.0f, 100.0f * util));
        loaded.push_back({util, at, t});
    };
    for (auto& bp : world.bodies()) {
        const phys::SoftBody& b = *bp;
        // (an orthographic camera stands far back: nothing is culled by distance)
        if (m_last_cam.ortho_half <= 0 && length2(b.aabb.center() - m_last_cam.pos) > cull2 + length2(b.aabb.extent())) continue;
        const bool sheet = !b.shells.empty() && (debug.beams || debug.collision);
        if (sheet) lift_nodes(b);
        if (debug.beams) {
            for (const auto& bm : b.beams) {
                if (bm.flags & phys::BF_BROKEN) continue;
                vec3 a = b.nodes[bm.a].p, c = b.nodes[bm.b].p;
                uint32_t col;
                if (debug.stress) { // (its force against its limit: positive compression)
                    const float s = bm.strength > 0 ? bm.stress / std::max(1.0f, std::min(bm.strength, bm.max_pos)) : 0;
                    col = ramp(std::fabs(s), s > 0 ? kCompression : kTension);
                } else {
                    float def = std::fabs(bm.L - bm.L0) / std::max(0.01f, bm.L0);
                    col = def > 0.01f ? Renderer::rgba(1, 0.5f, 0.1f) : ((bm.flags & phys::BF_SHOCK) ? Renderer::rgba(0.3f, 0.6f, 1) : Renderer::rgba(0.85f, 0.85f, 0.85f, 0.8f));
                }
                debug.beam_px > 1.0f ? r.thick_line(a, c, col, debug.beam_px) : r.line(a, c, col);
                if (bm.strength > 0) load_label(std::fabs(bm.stress) / std::max(1.0f, std::min(bm.strength, bm.max_pos)), (a + c) * 0.5f, "%.1f kN  %.0f%%", -bm.stress * 1e-3f);
            }
            // frame elements (FEM): thicker; steel blue, orange once bent for good, by stress their load against the
            // yield (or buckling) limit, green -> red
            for (const phys::FrameElement& e : b.fem.elems) {
                if (e.broken) continue;
                const vec3 a = b.nodes[b.fem.node[e.a]].p, c = b.nodes[b.fem.node[e.b]].p;
                uint32_t col;
                if (debug.stress) { // (its load against its yield or buckling limit; by its axial force's sign, or its bending)
                    const float np = b.fem.sections[e.section].Np;
                    const bool bend = np > 0 && std::fabs(e.N) / np < 0.5f * e.util;
                    col = ramp(e.util, bend ? kBending : e.N >= 0 ? kTension : kCompression);
                } else {
                    col = e.damage > 1e-4f ? Renderer::rgba(1.0f, 0.55f, 0.15f) : Renderer::rgba(0.55f, 0.72f, 1.0f);
                }
                r.thick_line(a, c, col, std::max(2.5f, debug.beam_px * 2.0f));
                { // (its axial force; a member loaded by its bending more than by that: named so)
                    const float np = b.fem.sections[e.section].Np;
                    const bool bend = np > 0 && std::fabs(e.N) / np < 0.5f * e.util;
                    load_label(e.util, (a + c) * 0.5f, bend ? "bending, %+.1f kN  %.0f%%" : "%+.1f kN  %.0f%%", e.N * 1e-3f);
                }
                // the joints other than welded: a dot near the end, in the editor's colours
                static const uint32_t jcol[] = {0, Renderer::rgba(1.0f, 0.6f, 0.2f), Renderer::rgba(0.45f, 1.0f, 0.5f), Renderer::rgba(0.3f, 0.9f, 1.0f),
                                                Renderer::rgba(1.0f, 0.45f, 0.9f), Renderer::rgba(1.0f, 0.9f, 0.3f)};
                const float Lm = length(c - a);
                if (e.end_a > 0 && e.end_a < 6 && Lm > 1e-4f) r.point(a + (c - a) * std::min(0.2f, 0.06f / Lm), jcol[e.end_a]);
                if (e.end_b > 0 && e.end_b < 6 && Lm > 1e-4f) r.point(c + (a - c) * std::min(0.2f, 0.06f / Lm), jcol[e.end_b]);
            }
            // triangle elements (FEM shells): their edges, blue, orange where the plate has yielded; by stress their load
            // against the yield, green -> red. On the plate's face towards the camera (the plate is drawn as a slab of
            // its thickness round the elements' plane: 1 px lines in that plane were inside it, hidden), a little further
            // with distance for the depth buffer's precision
            const float tw = std::max(2.0f, debug.beam_px * 1.5f);
            for (const phys::FrameTri& t : b.fem.tris) {
                if (t.broken) continue;
                uint32_t col;
                if (debug.stress) {
                    col = heat(t.util);
                } else {
                    col = t.dmg > 0 ? Renderer::rgba(1.0f, 0.5f, 0.05f) : Renderer::rgba(0.15f, 0.45f, 1.0f);
                }
                const vec3 p0 = b.nodes[b.fem.node[t.n[0]]].p, p1 = b.nodes[b.fem.node[t.n[1]]].p, p2 = b.nodes[b.fem.node[t.n[2]]].p;
                const vec3 n = normalize_or(cross(p1 - p0, p2 - p0), vec3(0));
                const vec3 to_cam = m_last_cam.pos - (p0 + p1 + p2) * (1.0f / 3);
                const vec3 up = n * ((dot(to_cam, n) >= 0 ? 1.0f : -1.0f) *
                                     (0.5f * b.fem.shell_sections[t.section].t + 0.002f + 0.0008f * length(to_cam)));
                r.thick_line(p0 + up, p1 + up, col, tw), r.thick_line(p1 + up, p2 + up, col, tw), r.thick_line(p2 + up, p0 + up, col, tw);
                if (const float y = b.fem.shell_sections[t.section].yield; y > 0) load_label(t.util, (p0 + p1 + p2) * (1.0f / 3) + up, "%.0f MPa  %.0f%%", t.util * y * 1e-6f);
            }
            // triangle elements: each edge once, on the face towards the camera. Pale blue (not the beams' grey: they are
            // no beams), the authored border brighter; plastically stretched or shortened (> 1%) orange; cracks and cuts
            // red; by stress: the edge strain against the fracture strain, green -> red
            for (size_t si = 0; si < b.shells.size(); si++) {
                const auto& sh = b.shells[si];
                for (int e = 0; e < 3; e++) {
                    if (sh.nb[e] >= 0 && sh.nb[e] < (int)si) continue;
                    const uint32_t ia = sh.n[e], ic = sh.n[e == 2 ? 0 : e + 1];
                    vec3 a = b.nodes[ia].p + lift[ia], c = b.nodes[ic].p + lift[ic];
                    uint32_t col;
                    if (debug.stress) {
                        col = heat(si < b.shk.aux.size() ? b.shk.aux[si].strain : 0.0f);
                    } else if (sh.nb[e] < 0 && !((sh.edges >> e) & 1u)) {
                        col = Renderer::rgba(1.0f, 0.25f, 0.2f, 0.95f);
                    } else {
                        const float def = std::fabs(sh.L[e] - sh.L0[e]) / std::max(1e-4f, sh.L0[e]);
                        col = def > 0.01f ? Renderer::rgba(1, 0.5f, 0.1f)
                            : sh.nb[e] < 0 ? Renderer::rgba(0.55f, 0.85f, 1.0f, 0.95f) : Renderer::rgba(0.6f, 0.78f, 0.9f, 0.6f);
                    }
                    debug.beam_px > 1.0f ? r.thick_line(a, c, col, debug.beam_px) : r.line(a, c, col);
                }
                if (si < b.shk.aux.size() && b.shk.aux[si].strain >= 0.25f && b.shell_mat.brk < 10.0f) // (its stretch against its fracture strain)
                    load_label(b.shk.aux[si].strain, (b.nodes[sh.n[0]].p + lift[sh.n[0]] + b.nodes[sh.n[1]].p + lift[sh.n[1]] + b.nodes[sh.n[2]].p + lift[sh.n[2]]) * (1.0f / 3),
                               "sheet stretched %.0f%%  %.0f%%", 100.0f * b.shk.aux[si].strain * b.shell_mat.brk);
            }
            for (const auto& j : b.joints) {
                if (j.broken) continue;
                const auto& pf = b.frames[j.parent_frame];
                float s = clampf(j.stress, 0, 1);
                {
                    const uint32_t jc = Renderer::rgba(0.3f + s * 0.7f, 1 - s * 0.7f, 0.3f);
                    debug.beam_px > 1.0f ? r.thick_line(b.nodes[pf.node].p, b.nodes[j.child_node].p, jc, debug.beam_px) : r.line(b.nodes[pf.node].p, b.nodes[j.child_node].p, jc);
                }
            }
        }
        if (debug.beams || debug.wheels)
            for (const auto& wh : b.wheels)
                if (wh.ring && !wh.detached && wh.axle0 < b.nodes.size() && wh.axle1 < b.nodes.size()) ring_tyre(b, wh);
        if (debug.volumes && !b.volumes.empty()) volumes(b);
        // the FEM plates' mid points (SoftBody::tri_mids): the collision view all of them (pale blue), the beam view the
        // ones that touched something in the last frame - orange the static world, yellow another body, magenta a
        // collision volume; on the plate's face towards the camera, as its edges
        if ((debug.beams || debug.collision) && b.tri_mids && !b.fem.tris.empty())
            for (size_t k = 0; k < b.fem.tris.size(); k++) {
                const phys::FrameTri& t = b.fem.tris[k];
                const uint8_t m = k < b.mid_touch.size() ? b.mid_touch[k] : 0;
                if (t.broken || (!m && !debug.collision) || t.n[0] >= b.fem.node.size() || t.n[1] >= b.fem.node.size() || t.n[2] >= b.fem.node.size()) continue;
                const vec3 p0 = b.nodes[b.fem.node[t.n[0]]].p, p1 = b.nodes[b.fem.node[t.n[1]]].p, p2 = b.nodes[b.fem.node[t.n[2]]].p, c = (p0 + p1 + p2) * (1.0f / 3);
                const vec3 n = normalize_or(cross(p1 - p0, p2 - p0), vec3(0)), to_cam = m_last_cam.pos - c;
                const vec3 up = n * ((dot(to_cam, n) >= 0 ? 1.0f : -1.0f) * (0.5f * b.fem.shell_sections[t.section].t + 0.003f + 0.0008f * length(to_cam)));
                const uint32_t col = m & 4 ? Renderer::rgba(1.0f, 0.3f, 0.9f) : m & 2 ? Renderer::rgba(1.0f, 0.95f, 0.3f) : m & 1 ? Renderer::rgba(1.0f, 0.55f, 0.1f)
                                                                                                       : Renderer::rgba(0.55f, 0.85f, 1.0f, 0.7f);
                r.point(c + up, col);
            }
        if (debug.nodes) {
            for (size_t i = 0; i < b.nodes.size(); i++) {
                uint16_t f = b.info[i].flags;
                uint32_t col = (f & phys::NF_FIXED) ? Renderer::rgba(1, 0.2f, 0.2f) : (f & phys::NF_TYRE) ? Renderer::rgba(0.2f, 0.4f, 1) : Renderer::rgba(1, 0.85f, 0.2f);
                r.point(b.nodes[i].p, col);
            }
            // frames: small axes
            for (const auto& fr : b.frames) {
                if (!debug.frames) break;
                vec3 p = b.nodes[fr.node].p;
                r.line(p, p + fr.q.rotate(vec3(0.15f, 0, 0)), Renderer::rgba(1, 0, 0));
                r.line(p, p + fr.q.rotate(vec3(0, 0.15f, 0)), Renderer::rgba(0, 1, 0));
                r.line(p, p + fr.q.rotate(vec3(0, 0, 0.15f)), Renderer::rgba(0, 0, 1));
            }
        }
        if (debug.collision) {
            static const bool hull_only = getenv("BL_COLLISION") && atoi(getenv("BL_COLLISION")) == 2; // (the hulls alone)
            for (const auto& t : b.tris) {
                if (t.torn || (hull_only && t.two_sided)) continue; // (no longer colliding)
                vec3 a = b.nodes[t.a].p, c = b.nodes[t.b].p, d = b.nodes[t.c].p;
                if (sheet && t.two_sided) {
                    a += lift[t.a];
                    c += lift[t.b];
                    d += lift[t.c];
                }
                // (a hull triangle amber, with a tick along its outward normal from its centre)
                uint32_t col = t.two_sided ? Renderer::rgba(0.2f, 1.0f, 0.9f, 0.5f) : Renderer::rgba(1.0f, 0.62f, 0.15f, 0.9f);
                if (!t.two_sided) {
                    const vec3 m = (a + c + d) / 3.0f;
                    r.line(m, m + normalize_or(cross(c - a, d - a), vec3(0)) * 0.12f, col);
                }
                r.line(a, c, col);
                r.line(c, d, col);
                r.line(d, a, col);
            }
            for (const auto& cp : b.capsules) {
                if (cp.joint >= 0 && b.joints[cp.joint].broken) continue;
                r.line(b.nodes[cp.a].p, b.nodes[cp.b].p, Renderer::rgba(1, 0.3f, 1));
            }
        }
        if (debug.islands) {
            uint32_t col = b.sleeping ? Renderer::rgba(0.4f, 0.4f, 1, 0.6f) : Renderer::rgba(0.2f, 1, 0.3f, 0.8f);
            vec3 c = b.aabb.center(), e = b.aabb.extent() * 0.5f;
            r.box_wire(mat4::translate(c), e, col);
        }
        if (debug.wheels) {
            for (const auto& wh : b.wheels) {
                vec3 a0 = b.nodes[wh.axle0].p, a1 = b.nodes[wh.axle1].p;
                r.line(a0, a1, Renderer::rgba(1, 1, 0));
                for (size_t k = 0; k < wh.nodes.size(); k++) {
                    uint32_t ni = wh.nodes[k];
                    r.line(k & 1 ? a1 : a0, b.nodes[ni].p, Renderer::rgba(0.4f, 0.6f, 1, 0.6f));
                }
            }
        }
    }
    // (the most loaded, nearest first among equals: the UI hides a label behind a nearer one)
    std::sort(loaded.begin(), loaded.end(), [](const Loaded& a, const Loaded& b) { return a.util > b.util; });
    for (size_t k = 0; k < loaded.size() && k < 30; k++) {
        const float u = clampf(loaded[k].util, 0, 1);
        debug_labels.push_back({loaded[k].at, loaded[k].text, vec4(0.4f + 0.6f * u, 1.0f - 0.6f * u, 0.35f, 1)});
    }
    if (debug.collision) {
        for (const auto& bx : world.statics.boxes) {
            mat4 m = mat4::from_mat3(bx.rot, bx.center);
            r.box_wire(m, bx.half, Renderer::rgba(1, 0.6f, 0.1f, 0.7f));
        }
    }
    for (auto it = m_vol_hits.begin(); it != m_vol_hits.end();) { // (the bodies gone)
        const bool live = std::any_of(world.bodies().begin(), world.bodies().end(), [&](const auto& bp) { return bp.get() == it->first; });
        it = live ? std::next(it) : m_vol_hits.erase(it);
    }
}

void Game::grab_begin(vec3 o, vec3 d) {
    phys::RayHit h = world.pick_node(o, d, 300.0f, 0.35f);
    if (!h.body) return;
    grab_end();
    grab_active = true;
    grab_body = h.body;
    grab_node = h.node;
    grab_depth = h.t;
    const vec3 c = h.body->nodes[h.node].p;
    // a body taken: its pull from its mass (RoR: (m/3000)^0.75), node its nearest (the line drawn from it)
    auto take = [&](phys::SoftBody& b, int node) {
        b.grab_node = node;
        b.grab_target = c;
        b.grab_k = std::max(2000.0f, 60000.0f * std::pow(std::max(b.total_mass(), 50.0f) / 3000.0f, 0.75f));
        b.grab_scale = grab_strength;
        b.wake();
        grab_bodies.push_back(&b);
    };
    if (grab_radius <= 0.01f) {
        take(*h.body, h.node);
        return;
    }
    // the grab's sphere round the picked node: the nodes of every body in it, each pulled to the target and its offset
    // from the centre (the region keeps its shape), weighted (1 - (r/R)^2)^2 - all of it at the centre, none at the edge
    const float R = grab_radius, R2 = R * R;
    for (const auto& bp : world.bodies()) {
        phys::SoftBody& b = *bp;
        if (c.x < b.aabb.mn.x - R || c.x > b.aabb.mx.x + R || c.y < b.aabb.mn.y - R || c.y > b.aabb.mx.y + R || c.z < b.aabb.mn.z - R || c.z > b.aabb.mx.z + R) continue;
        b.grab_nodes.clear(), b.grab_offsets.clear(), b.grab_w.clear();
        int nearest = -1;
        float nd = 1e30f;
        for (uint32_t i = 0; i < (uint32_t)b.nodes.size(); i++) {
            const phys::Node& x = b.nodes[i];
            const float r2 = length2(x.p - c);
            if (x.inv_mass <= 0 || (b.info[i].flags & phys::NF_NO_MOUSE) || r2 > R2) continue;
            const float q = 1.0f - r2 / R2;
            b.grab_nodes.push_back(i), b.grab_offsets.push_back(x.p - c), b.grab_w.push_back(q * q);
            if (r2 < nd) nd = r2, nearest = (int)i;
        }
        if (&b == h.body) nearest = h.node;
        if (nearest >= 0) take(b, nearest);
    }
}

void Game::grab_update(vec3 o, vec3 d) {
    if (!grab_active || grab_bodies.empty()) return;
    for (phys::SoftBody* b : grab_bodies) {
        b->grab_target = o + d * grab_depth;
        b->grab_scale = grab_strength; // (changed while pulling: at once)
        b->wake();
    }
}

void Game::crane_vehicle(Vehicle* v, float lift, float roll_deg, float pitch_deg) {
    crane_release();
    if (!v) return;
    phys::SoftBody& b = *v->body;
    // the top of the frame as it stands (its highest frame nodes, else its highest nodes), then the car tilted about
    // its centre of mass (roll about its length, pitch about its width) and lifted
    const bool frame = !b.fem.empty();
    float top = -1e9f;
    for (uint32_t i = 0; i < (uint32_t)b.nodes.size(); i++)
        if (!frame || b.fem.slot(i) >= 0) top = std::max(top, b.nodes[i].p.y);
    for (uint32_t i = 0; i < (uint32_t)b.nodes.size(); i++) {
        const phys::Node& n = b.nodes[i];
        if ((frame && b.fem.slot(i) < 0) || n.p.y < top - 0.05f || n.inv_mass <= 0) continue;
        crane_nodes.push_back(i);
    }
    const vec3 fwd = v->forward(), side = normalize(cross(vec3(0, 1, 0), fwd));
    const quat r = quat::axis_angle(side, pitch_deg * kDeg2Rad) * quat::axis_angle(fwd, roll_deg * kDeg2Rad);
    b.transform(r, b.center_of_mass(), vec3(0, lift, 0));
    for (uint32_t i : crane_nodes) {
        b.nodes[i].inv_mass = 0;
        b.nodes[i].v = vec3(0);
        b.info[i].flags |= phys::NF_FIXED;
    }
    crane_car = v;
    b.wake();
}

void Game::crane_release() {
    bool alive = false;
    for (auto& v : vehicles) alive |= v.get() == crane_car;
    if (alive && crane_car) {
        phys::SoftBody& b = *crane_car->body;
        for (uint32_t i : crane_nodes)
            if (i < b.nodes.size()) {
                b.nodes[i].inv_mass = b.nodes[i].mass > 0 ? 1.0f / b.nodes[i].mass : 0.0f;
                b.info[i].flags &= (uint16_t)~phys::NF_FIXED;
            }
        b.wake();
    }
    crane_nodes.clear();
    crane_car = nullptr;
}

void Game::grab_end() {
    for (phys::SoftBody* b : grab_bodies) b->grab_node = -1, b->grab_nodes.clear(), b->grab_offsets.clear(), b->grab_w.clear();
    grab_bodies.clear();
    grab_active = false;
    grab_body = nullptr;
    grab_node = -1;
}

const char* Game::projectile_name(int kind) {
    static const char* n[] = {"Steel ball", "Rubber ball", "Crate", "Cannonball", "Plank"};
    return n[std::clamp(kind, 0, kProjectileKinds - 1)];
}

const char* CrashConfig::layout_name(int i) {
    static const char* n[] = {"Head-on", "Offset head-on (50%)", "T-bone (B hits A's side)", "Angled 45 deg", "Rear-end"};
    return n[std::clamp(i, 0, 4)];
}

void Game::remove_object(DynamicObject* o) {
    if (!o) return;
    if (grab_holds(o->body)) grab_end();
    for (size_t i = 0; i < objects.size(); i++)
        if (objects[i].get() == o) {
            world.remove_body(o->body);
            objects.erase(objects.begin() + i);
            break;
        }
    projectiles.erase(std::remove(projectiles.begin(), projectiles.end(), o), projectiles.end());
}

void Game::clear_projectiles() {
    std::vector<DynamicObject*> list = projectiles;
    for (DynamicObject* o : list) remove_object(o);
}

bool Game::update_cursor(vec3 ro, vec3 rd) {
    cursor_valid = false;
    rd = normalize(rd);
    // surfaces and beams first (the point between nodes), then nodes near the ray (ropes, thin trees)
    phys::RayHit h = world.raycast_bodies(ro, rd, 500.0f, 0.08f);
    phys::RayHit hn = world.pick_node(ro, rd, 500.0f, 0.35f);
    if (hn.body && (!h.body || hn.t < h.t - 0.3f)) {
        h = hn;
        h.t = length(hn.body->nodes[hn.node].p - ro);
    }
    float t_static = world.statics.raycast(ro, rd, 500.0f);
    if (h.body && (t_static < 0 || h.t < t_static + 0.5f)) {
        cursor_point = ro + rd * h.t;
        cursor_valid = true;
    } else if (t_static >= 0) {
        cursor_point = ro + rd * t_static;
        cursor_valid = true;
    }
    return cursor_valid;
}

void Game::laser_sweep(vec3 origin, vec3 dir) {
    // the sector between the last ray and this one (a still cursor sweeps nothing)
    if (laser_active) laser_total += world.laser_cut(origin, laser_dir, dir, laser_range);
    laser_active = true;
    laser_origin = origin;
    laser_dir = dir;
}

void Game::destroy_under_cursor() {
    if (!cursor_valid) return;
    destroyed_total += world.destroy_at(cursor_point, destroy_radius, destroy_blast ? 1.5f : 0.0f);
}

void Game::shoot(vec3 origin, vec3 dir) {
    auto& A = SharedAssets::get();
    dir = normalize(dir);
    vec3 at = origin + dir * 2.0f;
    std::string name = format("shot%d", ++m_object_counter);
    std::unique_ptr<DynamicObject> o;
    Rng rng((uint64_t)m_object_counter * 7919 + 3);
    switch (projectile_kind) {
    case 0: { // steel ball: one sphere, heavy
        BallDesc d;
        d.center = at;
        d.radius = 0.22f;
        d.mass = 40;
        d.bounce = 0.25f;
        d.mat = A.metal;
        o = build_ball(world, d, name);
        break;
    }
    case 1: { // rubber ball: bouncy
        BallDesc d;
        d.center = at;
        d.radius = 0.35f;
        d.mass = 8;
        d.bounce = 0.75f;
        d.friction = 0.9f;
        d.mat = A.red;
        o = build_ball(world, d, name);
        break;
    }
    case 2: { // crate
        SoftBoxDesc d;
        d.center = at;
        d.size = vec3(0.8f);
        d.mass = 50;
        d.rot = quat::axis_angle(rng.unit_vector(), rng.range(0, 3));
        d.mat = std::make_shared<Material>(*A.wood);
        d.mat->diffuse = A.tex_crate;
        d.beams = {1e6f, 1000, 3e4f, 1.5e5f, 0};
        o = build_soft_box(world, d, name);
        break;
    }
    case 3: { // cannonball: very heavy
        BallDesc d;
        d.center = at;
        d.radius = 0.3f;
        d.mass = 400;
        d.bounce = 0.1f;
        d.mat = std::make_shared<Material>(*A.metal);
        d.mat->color = vec4(0.18f, 0.18f, 0.2f, 1);
        o = build_ball(world, d, name);
        break;
    }
    default: { // plank, flies lengthwise
        SoftBoxDesc d;
        d.center = at;
        d.size = vec3(0.2f, 0.2f, 2.0f);
        d.nx = 2;
        d.ny = 2;
        d.nz = 6;
        d.mass = 30;
        d.rot = quat_from_to(vec3(0, 0, 1), dir);
        d.mat = A.wood;
        d.beams = {2e6f, 1000, 4e4f, 1.2e5f, 0};
        o = build_soft_box(world, d, name);
        break;
    }
    }
    // thicker contact band for fast projectiles so they don't tunnel through thin sheets (a ball: its sphere reaches)
    if (o->body->capsules.empty()) {
        o->body->collision_radius = std::max(o->body->collision_radius, projectile_speed * world.settings.dt * 1.6f);
        o->body->collision_radius = std::min(o->body->collision_radius, 0.12f);
    }
    o->body->set_velocity(dir * projectile_speed);
    o->body->max_speed = projectile_speed;
    DynamicObject* ptr = add_object(std::move(o));
    ptr->visuals_dirty = true;
    projectiles.push_back(ptr);
    while ((int)projectiles.size() > max_projectiles) remove_object(projectiles.front());
}

void Game::next_vehicle() {
    if (vehicles.empty()) return;
    int n = (int)vehicles.size();
    int next = (player + 1 + n) % n;
    for (auto& v : vehicles) v->is_player = false;
    player = next;
    vehicles[next]->is_player = true;
    vehicles[next]->ai = false;
    cam.mode = CameraController::CHASE;
    cam.snap_behind(vehicles[next]->forward());
}

void Game::run_crash() {
    for (int i = (int)vehicles.size() - 1; i >= 0; i--) remove_vehicle(vehicles[i].get());
    clear_projectiles();
    if (crash.vehicle_a.empty() || crash.vehicle_b.empty()) return;
    // both vehicles reach the impact point (origin) after `t_hit` seconds
    const float t_hit = 1.5f;
    float va = crash.speed_a / 3.6f, vb = crash.speed_b / 3.6f;
    auto place = [&](const std::string& id, vec3 dir, float speed, vec3 lateral, float extra) -> Vehicle* {
        vec3 start = -dir * (speed * t_hit + extra) + lateral;
        start.y = world.statics.has_terrain ? world.statics.terrain.height(start.x, start.z) : 0.0f;
        float yaw = std::atan2(dir.x, dir.z) * kRad2Deg;
        Vehicle* v = spawn_vehicle(id, start, yaw, false);
        return v;
    };
    Vehicle *a = nullptr, *b = nullptr;
    switch (crash.layout) {
    default:
    case 0: // head-on
        a = place(crash.vehicle_a, vec3(1, 0, 0), va, vec3(0), 2.5f);
        b = place(crash.vehicle_b, vec3(-1, 0, 0), vb, vec3(0), 2.5f);
        break;
    case 1: // offset head-on: B shifted by ~half a car width
        a = place(crash.vehicle_a, vec3(1, 0, 0), va, vec3(0, 0, 0.45f), 2.5f);
        b = place(crash.vehicle_b, vec3(-1, 0, 0), vb, vec3(0, 0, -0.45f), 2.5f);
        break;
    case 2: // T-bone: A parked across the path, B drives into its side
        a = place(crash.vehicle_a, vec3(1, 0, 0), 0.0f, vec3(0), 0.0f);
        b = place(crash.vehicle_b, vec3(0, 0, 1), vb, vec3(0), 3.5f);
        va = 0;
        break;
    case 3: // angled
        a = place(crash.vehicle_a, vec3(1, 0, 0), va, vec3(0), 2.5f);
        b = place(crash.vehicle_b, normalize(vec3(-1, 0, 1)), vb, vec3(0), 2.5f);
        break;
    case 4: // rear-end: B catches up with A
        a = place(crash.vehicle_a, vec3(1, 0, 0), va * 0.3f, vec3(0), -3.0f);
        b = place(crash.vehicle_b, vec3(1, 0, 0), vb, vec3(0), 3.5f);
        va *= 0.3f;
        break;
    }
    // let the suspensions settle for a moment, then launch (done in the scene update)
    m_crash_phase = 0;
    m_crash_va = va;
    m_crash_vb = vb;
    m_crash_a = a;
    m_crash_b = b;
    world.settings.time_scale = 1.0f;
    cam.look_free(vec3(-7.5f, 3.2f, -9.0f), vec3(0, 0.7f, 0));
}

void Game::crash_update(float dt) {
    // timeline (simulation time): settle -> launch + tow -> release just before contact -> slow motion
    if (!m_crash_a && !m_crash_b) return;
    float t = (float)world.time();
    const float t_launch = 0.4f, t_hit = t_launch + 1.5f;
    auto tow = [&](Vehicle* v, float speed, bool on) {
        if (!v) return;
        if (on) {
            v->assist_speed = speed;
            v->assist_dir = normalize(vec3(v->forward().x, 0, v->forward().z));
        } else {
            v->assist_speed = -1;
        }
    };
    if (m_crash_phase == 0 && t >= t_launch) {
        if (m_crash_a) m_crash_a->launch(m_crash_a->forward() * m_crash_va);
        if (m_crash_b) m_crash_b->launch(m_crash_b->forward() * m_crash_vb);
        tow(m_crash_a, m_crash_va, true);
        tow(m_crash_b, m_crash_vb, true);
        m_crash_phase = 1;
    }
    if (m_crash_phase == 1 && t >= t_hit - 0.35f) {
        if (crash.slow_motion) world.settings.time_scale = 0.2f;
        m_crash_phase = 2;
    }
    if (m_crash_phase == 2 && t >= t_hit - 0.12f) {
        tow(m_crash_a, 0, false);
        tow(m_crash_b, 0, false);
        m_crash_phase = 3;
    }
    (void)dt;
}

std::string Game::crash_report() const {
    std::string s;
    for (auto& v : vehicles)
        s += format("%s%s: %d beams broken, peak %.0f g\n", v.get() == m_crash_a ? "A " : (v.get() == m_crash_b ? "B " : ""), v->name.c_str(),
                    v->broken_beams(), v->peak_g);
    return s;
}

void Game::drop_primitive(int kind, vec3 at) {
    auto& A = SharedAssets::get();
    Rng rng((uint64_t)(world.time() * 1000) + m_object_counter * 31 + 7);
    std::string name = format("obj%d", ++m_object_counter);
    std::unique_ptr<DynamicObject> o;
    switch (kind) {
    case 0: { // crate
        SoftBoxDesc d;
        d.center = at;
        d.size = vec3(1.0f);
        d.nx = d.ny = d.nz = 3;
        d.mass = 80;
        d.rot = quat::axis_angle(rng.unit_vector(), rng.range(0, 1));
        d.mat = std::make_shared<Material>(*A.wood);
        d.mat->diffuse = A.tex_crate;
        d.beams = {1.2e6f, 1500, 3.0e4f, 1.5e5f, 0.0f};
        o = build_soft_box(world, d, name);
        break;
    }
    case 1: { // jelly sphere
        SoftSphereDesc d;
        d.center = at;
        d.radius = 0.7f;
        d.mass = 60;
        d.beams = {6e4f, 120, 1e9f, 1e9f, 0};
        o = build_soft_sphere(world, d, name);
        break;
    }
    case 2: { // heavy metal box (plastic)
        SoftBoxDesc d;
        d.center = at;
        d.size = vec3(1.4f, 0.8f, 1.0f);
        d.nx = 4;
        d.ny = 3;
        d.nz = 3;
        d.mass = 600;
        d.mat = A.metal;
        d.beams = {6e6f, 3000, 8e4f, 1e7f, 0.0f};
        o = build_soft_box(world, d, name);
        break;
    }
    case 3: { // long plank
        SoftBoxDesc d;
        d.center = at;
        d.size = vec3(4.0f, 0.15f, 0.4f);
        d.nx = 9;
        d.ny = 2;
        d.nz = 2;
        d.mass = 60;
        d.mat = A.wood;
        d.rot = quat::axis_angle(vec3(0, 1, 0), rng.range(0, 3));
        d.beams = {2e6f, 1500, 5e4f, 1.2e5f, 0.0f};
        o = build_soft_box(world, d, name);
        break;
    }
    default: { // barrel-ish sphere (stiff)
        SoftSphereDesc d;
        d.center = at;
        d.radius = 0.45f;
        d.subdiv = 1;
        d.mass = 120;
        d.mat = A.red;
        d.beams = {1.5e6f, 800, 1e5f, 1e7f, 0};
        o = build_soft_sphere(world, d, name);
        break;
    }
    }
    add_object(std::move(o));
}

} // namespace bl
