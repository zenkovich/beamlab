#include "game/app.h"
#include "world/static_model.h"
#include "phys/cache_sim.h"
#include "core/jobs.h"
#include "core/profiler.h"
#include "core/util.h"
#include "vehicle/vehicle.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <map>
#include <cstdio>
#include <numeric>
#include <thread>

#if defined(__APPLE__)
#include <sys/sysctl.h>
#endif

namespace bl {

static App* g_app = nullptr;

void scroll_cb(GLFWwindow* w, double x, double y) {
    ImGui_ImplGlfw_ScrollCallback(w, x, y);
    if (g_app && !ImGui::GetIO().WantCaptureMouse) g_app->m_wheel += (float)y;
}

bool App::init(const AppOptions& opt) {
    m_opt = opt;
    m_hide_ui = opt.no_ui;
    g_app = this;
    if (!glfwInit()) {
        log_error("glfwInit failed");
        return false;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);
    if (opt.hidden) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    if (!opt.hidden && !opt.size_given) glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE); // a maximized window (not fullscreen)
    m_win = glfwCreateWindow(opt.width, opt.height, "BeamLab - soft-body vehicle physics", nullptr, nullptr);
    if (!m_win) {
        log_error("window creation failed");
        return false;
    }
    glfwMakeContextCurrent(m_win);
    glfwSwapInterval(opt.no_vsync || opt.bench_frames > 0 || opt.hidden ? 0 : 1);
    log_info("OpenGL %s | %s", (const char*)glGetString(GL_VERSION), (const char*)glGetString(GL_RENDERER));

    int workers = opt.threads > 0 ? opt.threads - 1 : JobSystem::performance_cores() - 1;
    JobSystem::get().init(std::max(0, workers));
    log_info("job system: %d threads", JobSystem::get().num_threads());

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    std::string font = asset_path("fonts/Roboto-Medium.ttf");
    ImFontConfig cfg;
    cfg.OversampleH = 2;
    if (file_exists(font)) {
        io.Fonts->AddFontFromFileTTF(font.c_str(), 15.0f, &cfg);
        m_font_small = io.Fonts->AddFontFromFileTTF(font.c_str(), 12.0f, &cfg);
        m_font_big = io.Fonts->AddFontFromFileTTF(font.c_str(), 22.0f, &cfg);
        m_font_huge = io.Fonts->AddFontFromFileTTF(font.c_str(), 44.0f, &cfg);
    }
    setup_style();
    ImGui_ImplGlfw_InitForOpenGL(m_win, true);
    glfwSetScrollCallback(m_win, scroll_cb);
    ImGui_ImplOpenGL3_Init("#version 410 core");

    if (!m_renderer.init()) return false;
    if (!m_game.init()) return false;
    // (the terrain's ground materials: assets/textures, tools/fetch_assets.py; without them the shader's noise)
    m_renderer.terrain_grass = pbr_material("grass");
    m_renderer.terrain_dirt = pbr_material("dirt");
    m_renderer.terrain_paved = pbr_material("asphalt_fine");
    register_metrics();

    // initial scene / vehicle
    const auto& vreg = vehicle_registry();
    log_info("vehicle registry: %d definitions", (int)vreg.size());
    m_game.selected_vehicle = opt.vehicle;
    if (m_game.selected_vehicle.empty()) {
        for (auto& e : vreg)
            if (e.id == "audi_quattro/1988AudiQuattro") {
                m_game.selected_vehicle = e.id;
                break;
            }
        if (m_game.selected_vehicle.empty() && !vreg.empty()) m_game.selected_vehicle = vreg[0].id;
    }
    int scene = 0;
    const auto& sreg = scene_registry();
    for (int i = 0; i < (int)sreg.size(); i++)
        if (sreg[i].id == opt.scene) scene = i;
    if (!opt.scene.empty() && isdigit((unsigned char)opt.scene[0])) scene = std::clamp(atoi(opt.scene.c_str()), 0, (int)sreg.size() - 1);
    m_game.world.settings.time_scale = opt.time_scale;
    m_show_perf = opt.show_perf;
    if (getenv("BL_NOSLEEP")) m_game.world.settings.sleeping = false;
    if (const char* c = getenv("BL_COARSEN")) m_game.world.settings.coarsen_per_frame = atoi(c); // (diagnostics: 0 = off)
    if (getenv("BL_NORIGID")) m_game.world.settings.rigid_pieces = false;
    if (const char* t = getenv("BL_FRAME_THETA")) m_game.world.settings.frame_theta = (float)atof(t); // (diagnostics: the frames' step)
    if (const char* d = getenv("BL_FRAME_DISS")) m_game.world.settings.frame_dissipation = (float)atof(d);
    if (const char* e = getenv("BL_FRAME_EVERY")) m_game.world.settings.frame_every = std::max(1, atoi(e)); // (diagnostics: the rates)
    if (const char* c = getenv("BL_COLLISION_HZ")) m_game.world.settings.collision_hz = (float)atof(c);
    if (!opt.spawn.empty()) {
        float x = 0, z = 0, yaw = 0;
        if (sscanf(opt.spawn.c_str(), "%f,%f,%f", &x, &z, &yaw) >= 2) {
            m_game.spawn_override = true;
            m_game.spawn_override_pos = vec3(x, 0, z);
            m_game.spawn_override_yaw = yaw;
        }
    }
    if (opt.debug_beams) m_game.debug.beams = true;
    if (!opt.shots.empty()) {
        std::string text;
        if (!read_text_file(opt.shots, text)) log_warn("cannot read shot list %s", opt.shots.c_str());
        size_t pos = 0;
        while (pos < text.size()) {
            size_t end = text.find('\n', pos);
            if (end == std::string::npos) end = text.size();
            std::string line = text.substr(pos, end - pos);
            pos = end + 1;
            char cam[256], path[1024];
            Shot sh;
            if (line.empty() || line[0] == '#' || sscanf(line.c_str(), "%d %255s %1023s", &sh.frame, cam, path) != 3) continue;
            sh.camera = cam;
            sh.path = path;
            sh.beams = line.find(" beams") != std::string::npos;
            sh.noui = line.find(" noui") != std::string::npos;
            sh.pause = line.find(" pause") != std::string::npos;
            m_shots.push_back(sh);
        }
    }
    if (getenv("BL_COLLISION")) m_game.debug.collision = m_game.debug.volumes = true;
    if (getenv("BL_VOLUMES")) m_game.debug.volumes = true;   // (the volumes view alone, F7)
    if (getenv("BL_DEFORM")) m_game.debug.stress = false;    // (the beam view coloured by deformation, F8)
    if (getenv("BL_HIDEMESHES")) m_game.debug.hide_meshes = true; // (screenshots: the collision view alone)
    if (const char* x = getenv("BL_XRAY")) m_game.debug.xray = true, m_game.debug.xray_alpha = std::clamp((float)atof(x), 0.02f, 1.0f); // (the x-ray view, F6)
    if (getenv("BL_SKELETON")) {
        m_game.debug.beams = m_game.debug.nodes = m_game.debug.hide_meshes = true;
    }
    select_scene(scene);
    m_last_time = glfwGetTime();
    return true;
}

void App::register_metrics() {
    using namespace Perfmon;
    auto metric = [&](const char* name, std::function<double()> f, double good, double bad, std::vector<double> targets) {
        PerfMetricSettings s;
        s.goodValue = good;
        s.badValue = bad;
        m_perf.RegisterMetric(Metric(name, std::move(f), s, std::move(targets)));
    };
    metric("FPS", [this] { return (double)m_fps; }, 58, 30, {30, 60, 120, 240});
    metric("Frame ms", [this] { return m_frame_ms; }, 17, 33, {10, 20, 40, 80});
    metric("Physics ms", [this] { return (double)m_game.frame_physics_ms; }, 8, 16, {5, 10, 20, 40, 80});
    metric("GPU ms", [this] { return m_renderer.gpu_ms(); }, 8, 16, {5, 10, 20, 40});
    metric("Sim speed %", [this] { return (double)m_game.world.stats().realtime_factor * 100.0; }, 98, 80, {50, 100, 150});
    metric("Active nodes k", [this] { return m_game.world.stats().active_nodes / 1000.0; }, 1e9, 2e9, {1, 5, 10, 20, 50, 100});
}

void App::select_scene(int idx) {
    double t0 = time_seconds();
    // reloading the same scene keeps the camera (mode, free position, orbit / chase angles)
    const bool reload = idx == m_game.scene_index;
    const CameraController cam = m_game.cam;
    m_game.load_scene(idx);
    if (reload) m_game.cam = cam;
    toast(format("%s loaded (%.1f s)", scene_registry()[idx].name.c_str(), time_seconds() - t0));
}

void App::select_vehicle(const std::string& id, bool respawn) {
    m_game.selected_vehicle = id;
    if (!respawn || m_game.no_player_vehicle) return; // (scene without a player vehicle: just remember the choice)
    vec3 pos = m_game.spawn_pos;
    float yaw = m_game.spawn_yaw;
    if (Vehicle* pv = m_game.player_vehicle()) {
        // respawn at the current place, facing the same way
        pos = pv->position();
        vec3 f = pv->forward();
        yaw = std::atan2(f.x, f.z) * kRad2Deg;
        pos.y = m_game.world.statics.has_terrain ? m_game.world.statics.terrain.height(pos.x, pos.z) : 0;
        m_game.remove_vehicle(pv);
    }
    double t0 = time_seconds();
    Vehicle* v = m_game.spawn_vehicle(id, pos, yaw, true);
    if (v) toast(format("%s  (%d nodes, %d beams, %.0f kg, %.0f ms)", v->name.c_str(), v->body->node_count(), (int)v->body->beams.size(), v->mass(),
                        (time_seconds() - t0) * 1000.0));
    else toast("Failed to load vehicle - see log");
}

void App::toast(const std::string& t) {
    m_toast = t;
    m_toast_time = 4.0f;
    log_info("%s", t.c_str());
}

void App::gather_input(float dt, VehicleInput& vin, CameraInput& cin) {
    ImGuiIO& io = ImGui::GetIO();
    bool kb = !io.WantCaptureKeyboard;
    auto key = [&](int k) { return kb && glfwGetKey(m_win, k) == GLFW_PRESS; };
    double mx, my;
    glfwGetCursorPos(m_win, &mx, &my);
    vec2 md((float)(mx - m_prev_mx), (float)(my - m_prev_my));
    m_prev_mx = mx;
    m_prev_my = my;
    bool mouse_free = !io.WantCaptureMouse;
    bool rmb = glfwGetMouseButton(m_win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    bool lmb = glfwGetMouseButton(m_win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    static const bool scripted = getenv("BL_INPUT_SCRIPT") != nullptr;
    if (scripted) { // (a scripted run: the mouse the script moves, as the UI's - the tools' headless checks)
        mx = io.MousePos.x, my = io.MousePos.y;
        lmb = io.MouseDown[0], rmb = io.MouseDown[1];
    }
    if (rmb && !m_rmb && !mouse_free) rmb = false;
    m_rmb = rmb;
    cin.mouse_delta = md;
    cin.rotate = rmb;
    cin.wheel = m_wheel;
    m_wheel = 0;
    cin.fast = key(GLFW_KEY_LEFT_SHIFT);

    bool free_cam = m_game.cam.mode == CameraController::FREE || !m_game.player_vehicle();
    if (free_cam) {
        cin.move = vec3((key(GLFW_KEY_D) ? 1.f : 0.f) - (key(GLFW_KEY_A) ? 1.f : 0.f), (key(GLFW_KEY_E) ? 1.f : 0.f) - (key(GLFW_KEY_Q) ? 1.f : 0.f),
                        (key(GLFW_KEY_W) ? 1.f : 0.f) - (key(GLFW_KEY_S) ? 1.f : 0.f));
    }
    if (!free_cam) {
        float thr = (key(GLFW_KEY_W) || key(GLFW_KEY_UP)) ? 1.0f : 0.0f;
        float brk = (key(GLFW_KEY_S) || key(GLFW_KEY_DOWN)) ? 1.0f : 0.0f;
        float st = ((key(GLFW_KEY_D) || key(GLFW_KEY_RIGHT)) ? 1.0f : 0.0f) - ((key(GLFW_KEY_A) || key(GLFW_KEY_LEFT)) ? 1.0f : 0.0f);
        // gamepad
        GLFWgamepadstate gp;
        if (glfwJoystickIsGamepad(GLFW_JOYSTICK_1) && glfwGetGamepadState(GLFW_JOYSTICK_1, &gp)) {
            float ax = gp.axes[GLFW_GAMEPAD_AXIS_LEFT_X];
            if (std::fabs(ax) > 0.08f) st = ax;
            float rt = (gp.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER] + 1) * 0.5f, lt = (gp.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER] + 1) * 0.5f;
            if (rt > 0.05f) thr = rt;
            if (lt > 0.05f) brk = lt;
            if (gp.buttons[GLFW_GAMEPAD_BUTTON_A]) vin.handbrake = true;
        }
        vin.throttle = thr;
        vin.brake = brk;
        vin.steer = st;
        // The mouse steers as in Operation Flashpoint: it moves a cursor that is a heading in the world (a pixel of the
        // mouse a pixel of the cursor), not the wheels; the vehicle steers towards it in proportion to the angle left,
        // so it turns to where the cursor points and, the camera turning with it, the cursor comes back to the middle.
        // The cursor is not held to the screen: it may point anywhere round the vehicle. The steering keys take over
        // and put the cursor back ahead; the right button looks around as ever.
        Vehicle* pv = m_game.player_vehicle();
        // (BL_MOUSESTEER=<degrees>: a scripted check - the mode on, the cursor put that far to the right at t = 1 s)
        static const char* ms_test = getenv("BL_MOUSESTEER");
        if (ms_test) m_mouse_steer = true;
        const bool take = m_mouse_steer && pv && kb && ((!m_hide_ui && glfwGetWindowAttrib(m_win, GLFW_FOCUSED)) || ms_test);
        if (take != m_mouse_captured) {
            glfwSetInputMode(m_win, GLFW_CURSOR, take ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
            m_mouse_captured = take;
            m_aim_valid = false;
            md = vec2(0); // (the cursor jumps as it is taken or let go)
            glfwGetCursorPos(m_win, &m_prev_mx, &m_prev_my);
        }
        if (take) {
            vec3 f = pv->forward();
            f.y = 0;
            f = normalize_or(f, vec3(0, 0, 1));
            const float head = std::atan2(f.x, f.z);
            int ww0, wh0;
            glfwGetWindowSize(m_win, &ww0, &wh0);
            const float aspect = (float)std::max(1, ww0) / (float)std::max(1, wh0);
            const float hfov = 2.0f * std::atan(std::tan(0.5f * m_game.cam.fov * kDeg2Rad) * aspect);
            const float per_px = hfov / (float)std::max(1, ww0);
            if (!m_aim_valid || st != 0) m_aim_yaw = head, m_aim_pitch = 0, m_aim_valid = true;
            if (static bool put = false; ms_test && !put && m_game.world.time() > 1.0) put = true, m_aim_yaw -= (float)atof(ms_test) * kDeg2Rad;
            if (ms_test) md = vec2(0);
            if (st == 0) {
                // (the cursor is free: any heading round the vehicle, any height, as fast as the mouse moves)
                if (!rmb) m_aim_yaw -= md.x * per_px, m_aim_pitch = clampf(m_aim_pitch - md.y * per_px, -1.4f, 1.4f);
                const float off = std::remainder(m_aim_yaw - head, 2.0f * kPi);
                if (ms_test && (m_frame_index % 30) == 0) fprintf(stderr, "mouse steer t=%.2f heading %.1f cursor %.1f off %.1f deg\n", m_game.world.time(), head / kDeg2Rad, m_aim_yaw / kDeg2Rad, off / kDeg2Rad);
                m_aim_yaw = head + off;
                float turn = clampf(-off / (m_mouse_steer_lock * kDeg2Rad), -1, 1);
                if (dot(pv->velocity(), f) < -0.5f) turn = -turn; // (backing up: the wheels the other way turn it the same way)
                vin.steer = turn;
                vin.steer_direct = true;
            }
        } else {
            m_aim_valid = false;
        }
        vin.handbrake |= key(GLFW_KEY_SPACE);
        // RoR commands (cranes, doors, tippers): number keys 1..9, 0 = command 1..10
        static const int keys[10] = {GLFW_KEY_1, GLFW_KEY_2, GLFW_KEY_3, GLFW_KEY_4, GLFW_KEY_5, GLFW_KEY_6, GLFW_KEY_7, GLFW_KEY_8, GLFW_KEY_9, GLFW_KEY_0};
        for (int k = 0; k < 10; k++)
            if (key(keys[k])) vin.command_key = k + 1;
    }
    if (m_mouse_captured) lmb = false, mouse_free = false; // (the mouse steers: no tool under it)
    // mouse grab (LMB on a node)
    Camera c = m_game.last_camera();
    int ww, wh;
    glfwGetWindowSize(m_win, &ww, &wh);
    vec2 ndc((float)(mx / std::max(1, ww)) * 2 - 1, 1 - (float)(my / std::max(1, wh)) * 2);
    mat4 inv = inverse(c.viewproj);
    vec4 a = inv * vec4(ndc.x, ndc.y, -1, 1), b = inv * vec4(ndc.x, ndc.y, 1, 1);
    vec3 ro = a.xyz() / a.w, rd = normalize(b.xyz() / b.w - ro);
    // cursor hit point for the tool indicator (the UI hidden - a screenshot, a recording - none but while a tool is held)
    if ((mouse_free && !m_hide_ui) || m_lmb_held) m_game.update_cursor(ro, rd);
    else m_game.cursor_valid = false;
    switch (m_game.tool) {
    case Tool::Grab:
        if (lmb && mouse_free && !m_lmb_grab) {
            m_game.grab_begin(ro, rd);
            m_lmb_grab = true;
        } else if (lmb && m_lmb_grab) {
            m_game.grab_update(ro, rd);
        } else if (!lmb && m_lmb_grab) {
            m_game.grab_end();
            m_lmb_grab = false;
        }
        break;
    case Tool::Destroy:
        // hold the button and sweep: everything under the cursor breaks
        if (lmb && (mouse_free || m_lmb_held)) {
            m_lmb_held = true;
            m_game.destroy_under_cursor();
        } else if (!lmb) {
            m_lmb_held = false;
        }
        break;
    case Tool::Laser:
        // hold the button and sweep: the laser cuts along the cursor's path
        if (lmb && (mouse_free || m_lmb_held)) {
            m_lmb_held = true;
            m_game.laser_sweep(c.pos, rd);
        } else if (!lmb) {
            m_lmb_held = false;
            m_game.laser_release();
        }
        break;
    case Tool::Shoot:
        if (lmb && mouse_free && !m_lmb_held) {
            m_lmb_held = true;
            m_game.shoot(c.pos, rd);
            m_fire_timer = 1.0f / std::max(0.5f, m_game.fire_rate);
        } else if (lmb && m_lmb_held) {
            m_fire_timer -= dt;
            if (m_fire_timer <= 0) {
                m_game.shoot(c.pos, rd);
                m_fire_timer += 1.0f / std::max(0.5f, m_game.fire_rate);
            }
        } else if (!lmb) {
            m_lmb_held = false;
        }
        break;
    }
    if (m_game.tool != Tool::Grab && m_lmb_grab) {
        m_game.grab_end();
        m_lmb_grab = false;
    }
    if (m_game.tool != Tool::Laser) m_game.laser_release(); // (the next sweep starts from where it is used again)
}

void App::apply_camera_preset(const std::string& p) {
    if (p.empty()) return;
    auto& cam = m_game.cam;
    if (p.rfind("look:", 0) == 0) {
        vec3 e, t;
        if (sscanf(p.c_str() + 5, "%f,%f,%f,%f,%f,%f", &e.x, &e.y, &e.z, &t.x, &t.y, &t.z) == 6) cam.look_free(e, t);
        return;
    }
    if (p.rfind("glook:", 0) == 0) { // heights above the ground: glook:ex,ez,eh,tx,tz,th
        vec3 e, t;
        if (sscanf(p.c_str() + 6, "%f,%f,%f,%f,%f,%f", &e.x, &e.z, &e.y, &t.x, &t.z, &t.y) == 6) {
            const auto& st = m_game.world.statics;
            auto gh = [&](float x, float z) {
                return st.ground_height(x, z);
            };
            e.y += gh(e.x, e.z);
            t.y += gh(t.x, t.z);
            cam.look_free(e, t);
        }
        return;
    }
    Vehicle* v = m_game.player_vehicle();
    if (!v) return;
    vec3 f = v->forward();
    float heading = std::atan2(-f.x, -f.z);
    cam.mode = CameraController::ORBIT;
    if (p == "chase") { cam.yaw = heading; cam.pitch = 0.25f; cam.dist = 9; cam.mode = CameraController::CHASE; }
    else if (p == "front") { cam.yaw = heading + kPi * 0.8f; cam.pitch = 0.2f; }
    else if (p == "side") { cam.yaw = heading + kPi * 0.5f; cam.pitch = 0.1f; }
    else if (p == "rear") { cam.yaw = heading + kPi * 0.2f; cam.pitch = 0.3f; }
    else if (p == "top") { cam.yaw = heading; cam.pitch = 1.35f; }
    else if (p.rfind("orbit:", 0) == 0) {
        float y = 0, pi = 0.3f, d = 9;
        sscanf(p.c_str() + 6, "%f,%f,%f", &y, &pi, &d);
        cam.yaw = heading + y * kDeg2Rad;
        cam.pitch = pi * kDeg2Rad;
        cam.dist = d;
        if (d > 0) return;
    }
    // fit distance to the vehicle size
    AABB b = v->body->aabb;
    cam.dist = std::max(5.0f, length(b.extent()) * 1.25f);
}

// Scripted mouse and keys for headless checks of the UI (the model editor's tools): BL_INPUT_SCRIPT is a list of
// "frame:action" separated by ';' with actions "m x y" (move), "d x y" / "u x y" (left button down / up there),
// "r x y" / "R x y" (right button down / up), "w x y n" (wheel n), "k NAME" (press and release a key: A-Z, 0-9,
// Enter, Escape, Delete, Space, Up, Down, Left, Right), "c TEXT" (typed characters), "K NAME" / "U NAME" (hold / let go).
void App::input_script() {
    static std::vector<std::pair<int, std::string>> steps;
    static bool parsed = false;
    static std::vector<ImGuiKey> release;
    ImGuiIO& io = ImGui::GetIO();
    for (ImGuiKey k : release) io.AddKeyEvent(k, false);
    release.clear();
    if (!parsed) {
        parsed = true;
        if (const char* e = getenv("BL_INPUT_SCRIPT"))
            for (const std::string& part : split_any(e, ";")) {
                const size_t c = part.find(':');
                if (c != std::string::npos) steps.push_back({atoi(part.substr(0, c).c_str()), trim(part.substr(c + 1))});
            }
    }
    auto key_of = [](const std::string& n) -> ImGuiKey {
        if (n.size() == 1 && n[0] >= 'A' && n[0] <= 'Z') return (ImGuiKey)(ImGuiKey_A + (n[0] - 'A'));
        if (n.size() == 1 && n[0] >= '0' && n[0] <= '9') return (ImGuiKey)(ImGuiKey_0 + (n[0] - '0'));
        if (n == "Enter") return ImGuiKey_Enter;
        if (n == "Escape") return ImGuiKey_Escape;
        if (n == "Delete") return ImGuiKey_Delete;
        if (n == "Space") return ImGuiKey_Space;
        if (n == "Up") return ImGuiKey_UpArrow;
        if (n == "Down") return ImGuiKey_DownArrow;
        if (n == "Left") return ImGuiKey_LeftArrow;
        if (n == "Right") return ImGuiKey_RightArrow;
        if (n == "-") return ImGuiKey_Minus;
        if (n == "=") return ImGuiKey_Equal;
        if (n == "[") return ImGuiKey_LeftBracket;
        if (n == "]") return ImGuiKey_RightBracket;
        if (n == "Ctrl") return ImGuiMod_Ctrl;
        if (n == "Shift") return ImGuiMod_Shift;
        if (n == "Alt") return ImGuiMod_Alt;
        return ImGuiKey_None;
    };
    for (const auto& [f, a] : steps) {
        if (f != m_frame_index || a.size() < 1) continue;
        const char op = a[0];
        float x = 0, y = 0, n = 0;
        sscanf(a.c_str() + 1, "%f %f %f", &x, &y, &n);
        if (op == 'm' || op == 'd' || op == 'u' || op == 'r' || op == 'R' || op == 'w') io.AddMousePosEvent(x, y);
        if (op == 'd' || op == 'u') io.AddMouseButtonEvent(0, op == 'd');
        if (op == 'r' || op == 'R') io.AddMouseButtonEvent(1, op == 'r');
        if (op == 'w') io.AddMouseWheelEvent(0, n);
        if (op == 'k' || op == 'K' || op == 'U') {
            const ImGuiKey k = key_of(trim(a.substr(1)));
            if (k != ImGuiKey_None) {
                io.AddKeyEvent(k, op != 'U');
                if (op == 'k') release.push_back(k);
            }
        }
        if (op == 'c') io.AddInputCharactersUTF8(trim(a.substr(1)).c_str());
    }
}

void App::frame(float dt) {
    PROFILE_ZONE("Frame");
    uint64_t t_frame = prof::now();
    {
        PROFILE_ZONE("Events"); // (the window system's event loop and the UI frame start)
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        input_script();
        ImGui::NewFrame();
    }

    VehicleInput vin;
    CameraInput cin;
    {
        PROFILE_ZONE("Input");
        // the model editor takes the mouse and keyboard while it is open (a test drive is the game as usual)
        if (m_editor.active() && !m_editor.driving()) {
            m_editor.frame(m_win, dt);
            m_wheel = 0;
            m_lmb_held = false;
            m_game.cursor_valid = false;
        } else {
            m_editor.frame(m_win, dt);
            gather_input(dt, vin, cin);
        }
    }
    if (!m_opt.drive.empty()) {
        float thr = 0, st = 0, br = 0, period = 0;
        int cmd = 0;
        sscanf(m_opt.drive.c_str(), "%f,%f,%f,%d,%f", &thr, &st, &br, &cmd, &period);
        vin.throttle = thr;
        if (!m_mouse_captured) vin.steer = st;
        // (a period s: the steering swept from side to side, a sine of that period - "in all directions")
        if (period > 0) vin.steer = st * std::sin(2.0f * kPi * (float)m_frame_index / (60.0f * period));
        vin.brake = br;
        vin.command_key = cmd;
        // BL_ORBIT=cx,cz,r[,period s[,top km/h]]: the test driver drives round a circle of radius r about (cx, cz),
        // turning the other way round every period, the throttle off above the top speed (steering to a point on the
        // circle 8 m ahead)
        if (const char* orb = getenv("BL_ORBIT"); orb && m_game.player_vehicle()) {
            float cx = 0, cz = 0, r = 15, per = 0, top = 1e9f;
            sscanf(orb, "%f,%f,%f,%f,%f", &cx, &cz, &r, &per, &top);
            Vehicle* v = m_game.player_vehicle();
            const float t = (float)m_frame_index / 60.0f;
            const float dir = per > 0 && ((int)(t / per) & 1) ? -1.0f : 1.0f;
            const vec3 p = v->position(), f = v->forward();
            const float th = std::atan2(p.z - cz, p.x - cx) + dir * 8.0f / std::max(r, 1.0f);
            const vec3 tgt(cx + r * std::cos(th), p.y, cz + r * std::sin(th));
            const vec3 to = normalize_or(vec3(tgt.x - p.x, 0, tgt.z - p.z), f);
            const vec3 fh = normalize_or(vec3(f.x, 0, f.z), to);
            vin.steer = clampf(-2.0f * std::asin(clampf(cross(fh, to).y, -1, 1)), -1, 1);
            if (v->speed_kmh() > top) vin.throttle = 0;
        }
        // steer 0 = test driver holds the initial heading (asymmetric models and open diffs pull to one side)
        else if (Vehicle* v = m_game.player_vehicle(); v && st == 0 && !m_mouse_captured) {
            static vec3 hold_dir(0), hold_pos(0);
            vec3 f = v->forward();
            f.y = 0;
            f = normalize_or(f, vec3(0, 0, 1));
            if (m_frame_index <= 6 || length2(hold_dir) == 0) {
                hold_dir = f;
                hold_pos = v->position();
            }
            vec3 side = cross(vec3(0, 1, 0), hold_dir); // left
            float off = dot(v->position() - hold_pos, side);
            float head = std::asin(clampf(dot(cross(hold_dir, f), vec3(0, 1, 0)), -1, 1));
            vin.steer = clampf(0.08f * off + 1.5f * head, -1, 1);
        }
        if (Vehicle* v = m_game.player_vehicle(); v && m_frame_index % 30 == 0)
            printf("t=%5.2fs  speed %6.1f km/h  rpm %6.0f  gear %d  pos (%.1f %.1f %.1f)  broken %d\n", m_game.world.time(), v->speed_kmh(), v->rpm(), v->gear(),
                   v->position().x, v->position().y, v->position().z, v->broken_beams());
        if (Vehicle* v = m_game.player_vehicle(); v && getenv("BL_STEERDBG") && m_frame_index % 3 == 0) {
            // front wheel yaw relative to the body (the physical result of the steering hydros)
            const auto& b = *v->body;
            vec3 f = v->forward(), l = v->left();
            float best = -1e9f, yaw = 0;
            for (const auto& w : b.wheels) {
                vec3 c = (b.nodes[w.axle0].p + b.nodes[w.axle1].p) * 0.5f;
                if (dot(c, f) > best) {
                    best = dot(c, f);
                    vec3 ax = b.nodes[w.axle1].p - b.nodes[w.axle0].p;
                    yaw = std::atan2(dot(ax, f), std::fabs(dot(ax, l)) > 1e-6f ? dot(ax, l) : 1e-6f) * kRad2Deg;
                }
            }
            printf("steer t=%.3f input %.2f state %.3f wheel %.1f deg speed %.1f\n", m_game.world.time(), vin.steer, v->steer_state(), yaw,
                   v->speed_kmh());
        }
        if (Vehicle* v = m_game.player_vehicle(); v && getenv("BL_NODEDBG")) { // (diagnostics: nodes "a,b,c" of the player's body each frame)
            const auto& b = *v->body;
            std::string ids = getenv("BL_NODEDBG");
            for (size_t p = 0; p < ids.size();) {
                size_t e = ids.find(',', p);
                if (e == std::string::npos) e = ids.size();
                const int i = atoi(ids.substr(p, e - p).c_str());
                p = e + 1;
                if (i < 0 || i >= (int)b.nodes.size()) continue;
                const phys::Node& n = b.nodes[i];
                printf("f %d node %d m %.3f p (%.3f %.3f %.3f) v (%.2f %.2f %.2f) |v| %.2f\n", m_frame_index, i, n.mass, n.p.x, n.p.y, n.p.z, n.v.x, n.v.y, n.v.z, length(n.v));
            }
        }
        if (Vehicle* v = m_game.player_vehicle(); v && m_frame_index % (getenv("BL_WHEELEVERY") ? atoi(getenv("BL_WHEELEVERY")) : 30) == 0 && getenv("BL_WHEELDBG")) {
            const auto& b = *v->body;
            for (const auto& w : b.wheels) {
                vec3 a0 = b.nodes[w.axle0].p, a1 = b.nodes[w.axle1].p, ax = normalize(a1 - a0);
                float rs = 0, rmin = 1e9f, rmax = 0, ymin = 1e9f;
                for (uint32_t ni : w.nodes) ymin = std::min(ymin, b.nodes[ni].p.y);
                for (uint32_t ni : w.nodes) {
                    vec3 r = b.nodes[ni].p - a0;
                    r -= ax * dot(r, ax);
                    float l = length(r);
                    rs += l;
                    rmin = std::min(rmin, l);
                    rmax = std::max(rmax, l);
                }
                // toe (the axle turned about the vertical against the body's side axis) and camber (its tilt)
                const vec3 f = v->forward(), l = v->left();
                const float side = dot(ax, l) >= 0 ? 1.0f : -1.0f;
                // (camber against the body's up, not the world's: the body's roll is not the wheel's)
                const float toe = std::atan2(dot(ax, f) * side, std::fabs(dot(ax, l))) * kRad2Deg,
                            camber = std::asin(clampf(dot(ax, v->up()) * side, -1, 1)) * kRad2Deg;
                if (w.ring) { // (a ring tyre: its load, how far it is pressed in - each side - its spin, its wheel on its bearings)
                    float sq = 0, side[2] = {0, 0}, fo[2] = {0, 0};
                    for (float q : w.squash) sq = std::max(sq, q);
                    for (size_t k = 0; k < w.side_sq.size(); k++) side[k & 1] = std::max(side[k & 1], w.side_sq[k]);
                    for (size_t k = 0; k < w.fold.size(); k++) fo[k & 1] = std::max(fo[k & 1], w.fold[k]);
                    rs = w.radius - sq, rmin = rs, rmax = w.radius, ymin = w.pos.y - rs;
                    printf("    ring load %.0f N pressed in %.3f (left %.3f%s, right %.3f%s, %d crushed) shifted across %.1f mm spin %.2f rad/s, wheel %.1f mm off its axle\n", w.load, sq, side[0],
                           fo[0] > 0.5f ? " folded" : "", side[1], fo[1] > 0.5f ? " folded" : "", w.pinches, w.lat_most * 1e3f, w.spin,
                           length(w.pos - 0.5f * (a0 + a1)) * 1e3f);
                } else if (!w.nodes.empty()) {
                    rs /= (float)w.nodes.size();
                }
                printf("    wheel r=%.3f (min %.3f max %.3f, def %.3f) axle %.3f speed %.1f torque %.0f brake %.0f hub y %.2f tyre ymin %.3f toe %.2f camber %.2f deg\n", rs, rmin, rmax,
                       w.radius, length(a1 - a0), w.speed, w.last_torque, w.brake, a0.y, ymin, toe, camber);
            }
            {
                // the lowest node that is not a tyre's (does anything drag?)
                float ylow = 1e9f;
                int nlow = -1;
                for (size_t i = 0; i < b.nodes.size(); i++)
                    if (!(b.info[i].flags & (phys::NF_TYRE | phys::NF_RIM)) && b.nodes[i].p.y < ylow) ylow = b.nodes[i].p.y, nlow = (int)i;
                printf("    lowest body node %d at y %.3f\n", nlow, ylow);
            }
        }
    }
    if (getenv("BL_NODEFORM") && m_frame_index == 1)
        for (auto& b : m_game.world.bodies()) b->allow_deform = false;
    if (getenv("BL_NOBREAK") && m_frame_index == 1)
        for (auto& b : m_game.world.bodies()) b->allow_break = false;
    if (static const char* cr = getenv("BL_CACHEREPORT"); cr && m_frame_index == atoi(cr)) {
        // cache model of the sheet kernel on the biggest sheet (stdout)
        if (measure_cache()) printf("%s: %s", m_cache_body.c_str(), m_cache_text.c_str());
    }
    if (static const char* sd = getenv("BL_SHELLDBG"); sd && m_frame_index % std::max(1, atoi(sd)) == 0) {
        // triangle-element sheets: mesh size per level, refinements, cracks, loose pieces
        size_t pieces = 0, piece_shells = 0, awake_pieces = 0;
        for (const auto& bp : m_game.world.bodies())
            if (bp->name.find(" piece") != std::string::npos) {
                pieces++;
                piece_shells += bp->shells.size();
                awake_pieces += !bp->sleeping;
            }
        printf("t=%6.2fs physics %.1f ms, bodies %zu; pieces %zu (%zu triangles, %zu awake)\n", m_game.world.time(), m_game.frame_physics_ms,
               m_game.world.bodies().size(), pieces, piece_shells, awake_pieces);
        for (const auto& bp : m_game.world.bodies()) { // (a shot ball: how near the sheets' nodes come to it)
            if (bp->sphere_ball <= 0 && bp->name.rfind("shot", 0) != 0) continue;
            vec3 c(0);
            float m = 0, near = 1e9f, rad = bp->sphere_ball;
            for (const auto& n : bp->nodes) c += n.p * n.mass, m += n.mass;
            c = c / std::max(m, 1e-6f);
            if (rad <= 0) // (its nodes' mean distance from the centre)
                for (const auto& n : bp->nodes) rad += length(n.p - c) / (float)bp->nodes.size();
            for (const auto& q : m_game.world.bodies())
                if (q.get() != bp.get() && !q->shells.empty())
                    for (const auto& n : q->nodes) near = std::min(near, length(n.p - c));
            printf("  ball %s: centre (%.3f %.3f %.3f), radius %.3f, the nearest sheet node %.3f m away\n", bp->name.c_str(), c.x, c.y, c.z, rad, near);
        }
        for (const auto& bp : m_game.world.bodies()) {
            const auto& b = *bp;
            if (b.shells.empty() || b.name.find(" piece") != std::string::npos) continue;
            int hist[5] = {0};
            for (const auto& sh : b.shells) hist[std::min(4, (int)sh.level)]++;
            if (getenv("BL_SHELLWHERE")) // (where the sheet was refined: the refined triangles' centres, in the body's frame)
                for (const auto& sh : b.shells)
                    if (sh.level > 0) {
                        const vec3 c = (b.nodes[sh.n[0]].p + b.nodes[sh.n[1]].p + b.nodes[sh.n[2]].p) / 3.0f;
                        printf("    refined L%d at (%.2f %.2f %.2f) nodes %u %u %u\n", sh.level, c.x, c.y, c.z, sh.n[0], sh.n[1], sh.n[2]);
                    }
            size_t np = 0, npt = 0, nps = 0; // pieces cracked off this sheet (bodies named after it)
            double area = 0;
            for (const auto& sh : b.shells) area += sh.area0;
            for (const auto& q : m_game.world.bodies())
                if (q->name.rfind(b.name + " piece", 0) == 0) {
                    np++;
                    npt += q->shells.size();
                    nps = std::max(nps, q->shells.size());
                    for (const auto& sh : q->shells) area += sh.area0;
                }
            // a closed sheet (every edge between two triangles: a drum) also its volume, the dents in one number
            std::map<std::pair<uint32_t, uint32_t>, int> edges;
            double vol = 0;
            for (const auto& sh : b.shells) {
                for (int e = 0; e < 3; e++) edges[std::minmax(sh.n[e], sh.n[e == 2 ? 0 : e + 1])]++;
                vol += dot(b.nodes[sh.n[0]].p, cross(b.nodes[sh.n[1]].p, b.nodes[sh.n[2]].p)) / 6.0;
            }
            const bool closed = std::all_of(edges.begin(), edges.end(), [](const auto& kv) { return kv.second == 2; });
            vec3 c(0);
            for (const auto& n : b.nodes) c += n.p / (float)b.nodes.size();
            if (!b.fem.empty())
                printf("    frame: %zu members, %d splits, %d torn, %d failed solves, %d clamps, %d energy cuts, %d of %zu welds broken\n", b.fem.elems.size(), b.fem.splits,
                       b.fem.broken, b.fem.solve_failures, b.fem.clamps, b.guard_cuts, b.stats.broken_welds, b.welds.size());
            if (closed) printf("    area %.5f m2 (the sheet and its pieces), volume %.5f m3, centre (%.3f %.3f %.3f)\n", area, std::abs(vol), c.x, c.y, c.z);
            else printf("    area %.5f m2 (the sheet and its pieces)\n", area);
            printf("  %-18s shells %5zu (levels %d %d %d %d %d) nodes %5zu refined %5d cracks %5d pieces %zu (%zu triangles, biggest %zu)%s | k %.3g bend %.3g | impacts %zu\n",
                   b.name.c_str(), b.shells.size(), hist[0], hist[1], hist[2], hist[3], hist[4], b.nodes.size(), b.shell_stats.refined,
                   b.shell_stats.cracks, np, npt, nps, b.sleeping ? " asleep" : "", b.shell_mat.k, b.shell_mat.bend, b.shell_impacts.size());
            if (!b.sleeping) {
                float worst = 0;
                uint32_t wsi = 0;
                for (uint32_t si = 0; si < b.shells.size(); si++)
                    for (int e = 0; e < 3; e++) {
                        const phys::Shell& sh = b.shells[si];
                        const float st = length(b.nodes[sh.n[e]].p - b.nodes[sh.n[e == 2 ? 0 : e + 1]].p) / sh.L0[e] - 1.0f;
                        if (st > worst) worst = st, wsi = si;
                    }
                printf("      worst edge stretch %.3f at shell %u (nodes %u %u %u)\n", worst, wsi, b.shells[wsi].n[0], b.shells[wsi].n[1], b.shells[wsi].n[2]);
                // (what keeps it awake: the fastest node, the mass-weighted RMS speed the cracked-sheet rule uses)
                float vmax = 0;
                size_t imax = 0;
                double mv2 = 0, m = 0;
                for (size_t i = 0; i < b.nodes.size(); i++) {
                    const float v = length(b.nodes[i].v);
                    if (v > vmax) vmax = v, imax = i;
                    mv2 += (double)b.nodes[i].mass * v * v;
                    m += b.nodes[i].mass;
                }
                const vec3 p = b.nodes[imax].p;
                printf("      awake: max speed %.3f m/s at node %zu (%.2f %.2f %.2f, %zu shells round it), rms %.4f, sleep timer %.2f, static contacts %d\n", vmax, imax, p.x, p.y, p.z,
                       imax < b.node_shells.size() ? b.node_shells[imax].size() : 0, m > 0 ? std::sqrt(mv2 / m) : 0.0, b.sleep_timer, b.static_contacts);
                if (imax < b.node_shells.size()) {
                    // its stability: the springs and hinges it carries against its mass at the body's short step
                    float ksum = 0;
                    const float h = phys::kDefaultDt / (float)(1 << b.dt_shift());
                    for (uint32_t si : b.node_shells[imax]) {
                        const phys::Shell& sh = b.shells[si];
                        for (int e = 0; e < 3; e++)
                            if (sh.n[e] == imax || sh.n[e == 2 ? 0 : e + 1] == imax) ksum += sh.k[e];
                        const float q = 6.9282f * sh.area0 / (sh.L0[0] * sh.L0[0] + sh.L0[1] * sh.L0[1] + sh.L0[2] * sh.L0[2]);
                        printf("        shell %u: level %d, area %.2e (nominal %.2e), quality %.2f, L0 %.3f %.3f %.3f, nb %d %d %d, th0 %.2f %.2f %.2f, strain %.2f\n", si, sh.level, sh.area0,
                               sh.area_nom, q, sh.L0[0], sh.L0[1], sh.L0[2], sh.nb[0], sh.nb[1], sh.nb[2], sh.th0[0], sh.th0[1], sh.th0[2], si < b.shk.aux.size() ? b.shk.aux[si].strain : -1.0f);
                    }
                    printf("        node mass %.3e kg, sum k %.3e: k dt^2 / m = %.3f (budget 0.45), fixed %d\n", b.nodes[imax].mass, ksum, ksum * h * h * std::max(0.0f, b.nodes[imax].inv_mass),
                           (b.info[imax].flags & phys::NF_FIXED) ? 1 : 0);
                }
            }
        }
    }
    static int dbg_every = getenv("BL_DBGEVERY") ? atoi(getenv("BL_DBGEVERY")) : 15;
    if (const char* dbg = getenv("BL_BODYDBG"); dbg && m_frame_index % dbg_every == 0) {
        for (auto& b : m_game.world.bodies())
            if (b->name.find(dbg) != std::string::npos) {
                vec3 c = b->aabb.center(), e = b->aabb.extent();
                float ys = 0, bs = 0;
                for (const auto& bm : b->beams) {
                    if (bm.flags & phys::BF_BROKEN) continue;
                    float s = std::fabs(bm.stress);
                    ys = std::max(ys, s / std::max(1.0f, bm.stress > 0 ? bm.max_pos : -bm.max_neg));
                    bs = std::max(bs, s / std::max(1.0f, bm.strength));
                }
                printf("      beams: max stress/yield %.2f  stress/break %.2f\n", ys, bs);
                printf("f%3d %-16s sleep %d  center (%.2f %.2f %.2f) ext (%.2f %.2f %.2f) vmax %.2f broken j%d b%d\n", m_frame_index, b->name.c_str(), (int)b->sleeping,
                       c.x, c.y, c.z, e.x, e.y, e.z, b->max_speed, b->stats.broken_joints, b->stats.broken_beams);
            }
    }
    if (const char* fd = getenv("BL_FEMDBG"); fd && m_frame_index % std::max(1, atoi(fd)) == 0) // (the bodies of FEM triangles)
        for (auto& b : m_game.world.bodies()) {
            const phys::FemFrame& f = b->fem;
            if (f.tris.empty()) continue;
            int dented = 0;
            float peak = 0, low = 1e9f, fast = 0;
            for (const phys::FrameTri& t : f.tris)
                if (!t.broken) dented += t.dmg > 0, peak = std::max(peak, t.util);
            vec3 c(0), fp(0);
            for (const auto& n : b->nodes) {
                c += n.p / (float)b->nodes.size(), low = std::min(low, n.p.y);
                if (length(n.v) > fast) fast = length(n.v), fp = n.p;
            }
            float p0, p1;
            b->largest_parts(p0, p1);
            printf("fem f%d %-24s tris %4zu torn %3d dented %4d peak %.2f failed %d sleep %d fastest %.3f centre (%.3f %.3f %.3f) lowest %.3f parts %.3f %.3f\n",
                   m_frame_index, b->name.c_str(), f.tris.size(), f.tris_torn, dented, peak, f.solve_failures, (int)b->sleeping, fast, c.x, c.y, c.z, low, p0, p1);
            if (getenv("BL_FEMFAST")) printf("    fastest node at (%.3f %.3f %.3f)\n", fp.x - c.x, fp.y - c.y, fp.z - c.z);
            if (f.tris_refined || !f.impacts.empty()) printf("    %d bisections, %zu fracture patterns\n", f.tris_refined, f.impacts.size());
            if (getenv("BL_FEMDENT")) // (the dented triangles: their index in the truck's fem_tris, the plastic stretch)
                for (const phys::FrameTri& t : f.tris)
                    if (t.broken || t.dmg > 0) printf("    dent %d %.4f%s\n", t.tag, t.dmg, t.broken ? " torn" : "");
        }
    if (const char* en = getenv("BL_ENERGYDBG")) // (a body's energy every frame: its motion as a whole, the rest, its height)
        for (auto& b : m_game.world.bodies())
            if (b->name == en) {
                double m = 0, ke = 0;
                vec3 p(0), c(0);
                for (const auto& n : b->nodes) m += n.mass, p += n.v * n.mass, c += n.p * n.mass, ke += 0.5 * n.mass * length2(n.v);
                const vec3 v = p / (float)m;
                c = c / (float)m;
                const double ket = 0.5 * m * length2(v);
                vec3 L(0);
                float I = 0;
                for (const auto& n : b->nodes) L += cross(n.p - c, n.v - v) * n.mass, I += n.mass * length2(n.p - c);
                const vec3 w = L / std::max(1e-6f, I * 0.5f); // (roughly: I about an axis ~ half the polar sum)
                printf("energy f%d %s: KE %.4f J (whole %.4f, rest %.4f) PE %.4f J, centre (%.4f %.4f %.4f) v (%.4f %.4f %.4f) L (%.3f %.3f %.3f) w~(%.2f %.2f %.2f)%s\n",
                       m_frame_index, en, ke, ket, ke - ket, m * 9.81 * c.y, c.x, c.y, c.z, v.x, v.y, v.z, L.x, L.y, L.z, w.x, w.y, w.z, b->sleeping ? " asleep" : "");
                printf("   resting %d max speed %.3f static %d, bodies %d, rigid %.3f (%.3f on average), sleep timer %.2f, rest timer %.2f%s\n", (int)b->resting,
                       b->max_speed, b->static_contacts, b->body_contacts, b->rigid_speed, b->rigid_avg, b->sleep_timer, b->rest_timer, b->sleeping ? ", asleep" : "");
            }
    if (const char* sd = getenv("BL_SHOCKDBG"); sd && m_frame_index == atoi(sd))
        if (Vehicle* v = m_game.player_vehicle()) {
            // the wheels in the body's frame: the axle's middle from the centre of mass (forward, up, left), camber
            // (+: the top leans out) and toe (+: in) from the axle turned outwards
            const phys::SoftBody& b = *v->body;
            const vec3 f = v->forward(), l = v->left(), u = normalize(cross(f, l)), c = b.center_of_mass();
            for (const auto& w : b.wheels) {
                const vec3 a0 = b.nodes[w.axle0].p, a1 = b.nodes[w.axle1].p, m = (a0 + a1) * 0.5f - c;
                vec3 ax = normalize(a1 - a0);
                if (dot(ax, m - f * dot(m, f) - u * dot(m, u)) < 0) ax = -ax;
                printf("wheel at (%+.3f %+.3f %+.3f) camber %+.1f toe %+.1f deg\n", dot(m, f), dot(m, u), dot(m, l), -std::asin(clampf(dot(ax, u), -1, 1)) * kRad2Deg,
                       -std::atan2(dot(ax, f), std::fabs(dot(ax, l))) * kRad2Deg);
            }
        }
    if (getenv("BL_SHOCKLOG")) // (diagnostics: the player's springs each frame, % of the stroke to the short bound, - past it)
        if (Vehicle* v = m_game.player_vehicle()) {
            const phys::SoftBody& b = *v->body;
            printf("shocklog f%d t %.3f:", m_frame_index, m_game.world.time());
            for (const phys::Shock& sh : b.shocks) {
                const phys::Beam& bm = b.beams[sh.beam];
                if (sh.spring < 1000 || (bm.flags & phys::BF_BROKEN) || ((b.info[bm.a].flags | b.info[bm.b].flags) & (phys::NF_TYRE | phys::NF_RIM))) continue; // (not the tyres')
                const float len = length(b.nodes[bm.a].p - b.nodes[bm.b].p), lo = bm.L * (1 - sh.short_bound), hi = bm.L * (1 + sh.long_bound);
                printf(" %.0f", 100.0f * (len - lo) / (hi - lo));
            }
            printf("\n");
        }
    if (const char* sd = getenv("BL_SHOCKDBG"); sd && m_frame_index == atoi(sd)) {
        for (const auto& bp : m_game.world.bodies()) {
            const phys::SoftBody& b = *bp;
            for (size_t si = 0; si < b.shocks.size(); si++) {
                const phys::Shock& sh = b.shocks[si];
                const phys::Beam& bm = b.beams[sh.beam];
                const float len = length(b.nodes[bm.a].p - b.nodes[bm.b].p);
                if (sh.spring < 1000 && sh.spring_in < 1000) continue; // (the wheels' bounded beams)
                printf("shock %zu of %s: %u-%u L %.3f len %.3f (%+.1f%%, bounds -%.0f%% +%.0f%%) k %.0f bound k %.0f%s\n", si, b.name.c_str(), bm.a, bm.b, bm.L, len,
                       100 * (len - bm.L) / bm.L, 100 * sh.short_bound, 100 * sh.long_bound, sh.spring, sh.bound_spring, (bm.flags & phys::BF_BROKEN) ? " BROKEN" : "");
            }
        }
    }
    if (!m_opt.crane.empty()) {
        float lift = 1.0f, roll = 0, pitch = 0;
        int release = -1;
        sscanf(m_opt.crane.c_str(), "%f,%d,%f,%f", &lift, &release, &roll, &pitch);
        if (m_frame_index == 8) m_game.crane_vehicle(m_game.player_vehicle(), lift, roll, pitch);
        if (m_frame_index == release) m_game.crane_release();
    }
    if (!m_opt.autoshoot.empty() && m_frame_index > 10) {
        int kind = 0;
        float speed = 40, interval = 0.5f;
        sscanf(m_opt.autoshoot.c_str(), "%d,%f,%f", &kind, &speed, &interval);
        static float t_acc = 0;
        t_acc += dt;
        if (t_acc >= interval) {
            t_acc = 0;
            const Camera& c = m_game.last_camera();
            m_game.projectile_kind = kind;
            m_game.projectile_speed = speed;
            m_game.shoot(c.pos, c.forward());
        }
    }
    // (the panels' visible shake: how far the welded nodes' centre moves against the frame's point from one frame to the
    // next, rms and most over the status line's frames)
    static std::vector<vec3> shake_prev;
    static double shake_s2 = 0, shake_t = -1;
    static float shake_most = 0;
    static int shake_n = 0;
    if (!m_opt.action.empty() || !m_opt.autoshoot.empty() || !m_opt.drive.empty() || !m_opt.crane.empty())
        if (Vehicle* v = m_game.player_vehicle(); v && !v->body->welds.empty()) {
            const phys::SoftBody& b = *v->body;
            const double t = m_game.world.time();
            const bool have = shake_prev.size() == b.welds.size() && t > shake_t;
            shake_prev.resize(b.welds.size());
            for (size_t k = 0; k < b.welds.size(); k++) {
                const phys::Weld& wd = b.welds[k];
                vec3 cp(0);
                for (uint32_t i = wd.first; i < wd.first + wd.count; i++) cp += b.nodes[b.weld_nodes[i]].p * b.weld_w[i];
                const vec3 off = cp - (b.nodes[wd.anchor].p * (1 - wd.t) + b.nodes[wd.anchor2].p * wd.t);
                if (have && !wd.broken) {
                    const float r = length(off - shake_prev[k]) / (float)(t - shake_t);
                    shake_s2 += (double)r * r, shake_most = std::max(shake_most, r), shake_n++;
                    if (getenv("BL_MOVEDBG") && r > (float)atof(getenv("BL_MOVEDBG"))) // (diagnostics: which welds move)
                        printf("  frame %d weld %zu moves %.0f mm/s: node %u (%.2f %.2f %.2f) anchors %u %u\n", m_frame_index, k, r * 1000.0f,
                               b.weld_nodes[wd.first], cp.x, cp.y, cp.z, wd.anchor, wd.anchor2);
                }
                shake_prev[k] = off;
            }
            shake_t = t;
        }
    if ((!m_opt.action.empty() || !m_opt.autoshoot.empty() || !m_opt.drive.empty() || !m_opt.crane.empty()) && m_frame_index % (getenv("BL_STATUS_EVERY") ? atoi(getenv("BL_STATUS_EVERY")) : 120) == 0) {
        if (Vehicle* v = m_game.player_vehicle()) {
            vec3 p = v->position();
            printf("t=%6.1fs  %5.1f km/h  pos (%.0f %.0f %.0f)  broken %4d  | %s | %s", m_game.world.time(), v->speed_kmh(), p.x, p.y, p.z,
                   v->broken_beams(), m_game.scene_banner.c_str(), m_game.scene_status.c_str());
            if (!v->body->fem.empty())
                printf(" | frame: %zu members, %d splits, %d torn, %d failed solves, %d clamps", v->body->fem.elems.size(), v->body->fem.splits, v->body->fem.broken,
                       v->body->fem.solve_failures, v->body->fem.clamps);
            if (const char* wn = getenv("BL_WHEELNODES")) { // (diagnostics: wheel n's axle nodes and the frame members at them)
                const phys::SoftBody& b = *v->body;
                const int k = atoi(wn);
                if (k >= 0 && k < (int)b.wheels.size()) {
                    const auto& w = b.wheels[k];
                    const vec3 fw = v->forward(), upv = v->up(), lf = normalize_or(cross(upv, fw), vec3(1, 0, 0)), o = v->position();
                    auto loc = [&](uint32_t n) { const vec3 d = b.nodes[n].p - o; return vec3(dot(d, fw), dot(d, upv), dot(d, lf)); };
                    const vec3 a0 = loc(w.axle0), a1 = loc(w.axle1);
                    printf("\n  wheel %d axle0 %u (%.2f %.2f %.2f) axle1 %u (%.2f %.2f %.2f)", k, w.axle0, a0.x, a0.y, a0.z, w.axle1, a1.x, a1.y, a1.z);
                    for (size_t ei = 0; ei < b.fem.elems.size(); ei++) {
                        const phys::FrameElement& e = b.fem.elems[ei];
                        const uint32_t na = b.fem.node[e.a], nb = b.fem.node[e.b];
                        if (na != w.axle0 && nb != w.axle0 && na != w.axle1 && nb != w.axle1) continue;
                        const vec3 pa = loc(na), pb = loc(nb);
                        printf("\n    member %zu %u-%u (%.2f %.2f %.2f)-(%.2f %.2f %.2f) Mp %.0f ends %d/%d level %d torn %d dmg %.3f broken %d", ei, na, nb, pa.x, pa.y, pa.z, pb.x,
                               pb.y, pb.z, b.fem.sections[e.section].Mp, e.end_a, e.end_b, e.level, e.torn, e.damage, (int)e.broken);
                    }
                }
            }
            if (const char* cf = getenv("BL_CELLDBG")) { // (diagnostics: each car's deformation at the points of a file, "x y z" lines in
                // the definition's space - its safety cell's: their rms and largest distance off the best rigid fit, mm)
                static std::vector<vec3> pts;
                static std::map<const Vehicle*, std::vector<int>> ids;
                if (pts.empty())
                    if (FILE* f = fopen(cf, "r")) {
                        float x, y, z;
                        while (fscanf(f, "%f %f %f", &x, &y, &z) == 3) pts.push_back(vec3(x, y, z));
                        fclose(f);
                    }
                for (size_t vi = 0; vi < m_game.vehicles.size(); vi++) {
                    const Vehicle* w = m_game.vehicles[vi].get();
                    const auto& sp = w->spawn_nodes();
                    auto& id = ids[w];
                    if (id.empty())
                        for (const vec3& q : pts) {
                            int best = -1;
                            float bd = 2e-3f * 2e-3f;
                            for (size_t i = 0; i < sp.size(); i++)
                                if (const float d = length2(sp[i].p - q); d < bd) bd = d, best = (int)i;
                            if (best >= 0) id.push_back(best);
                        }
                    const auto& nd = w->body->nodes;
                    const size_t n = id.size();
                    if (n < 3) continue;
                    vec3 c0(0), c1(0);
                    for (int i : id) c0 += sp[i].p, c1 += nd[i].p;
                    c0 = c0 / (float)n, c1 = c1 / (float)n;
                    mat3 A(vec3(0), vec3(0), vec3(0));
                    for (int i : id) A = A + outer(nd[i].p - c1, sp[i].p - c0);
                    // (from the car's heading: the definition's -x forward, y up, +z left; then Mueller's iteration)
                    quat q = from_mat3(mat3(-w->forward(), w->up(), w->left()));
                    for (int it = 0; it < 40; it++) {
                        const mat3 R = to_mat3(q);
                        const vec3 om = cross(R.c[0], A.c[0]) + cross(R.c[1], A.c[1]) + cross(R.c[2], A.c[2]);
                        const float den = std::fabs(dot(R.c[0], A.c[0]) + dot(R.c[1], A.c[1]) + dot(R.c[2], A.c[2])) + 1e-9f;
                        const vec3 wv = om / den;
                        const float wl = length(wv);
                        if (!(wl > 1e-8f)) break;
                        q = normalize(quat::axis_angle(wv / wl, wl) * q);
                    }
                    const mat3 R = to_mat3(q);
                    float r2 = 0, mx = 0;
                    for (int i : id) {
                        const float d = length(nd[i].p - c1 - R * (sp[i].p - c0));
                        r2 += d * d, mx = std::max(mx, d);
                    }
                    printf(" | cell%zu n %zu rms %.1f max %.1f mm", vi, n, 1000.0f * std::sqrt(r2 / (float)n), 1000.0f * mx);
                }
            }
            if (getenv("BL_SUSPDBG")) { // (diagnostics: the wheels' beams bent for good, the frame's members bent for good)
                const phys::SoftBody& b = *v->body;
                std::vector<char> in(b.nodes.size(), 0);
                float wmax = 0;
                int wbent = 0;
                for (size_t k = 0; k < b.wheels.size(); k++) {
                    const auto& w = b.wheels[k];
                    std::fill(in.begin(), in.end(), 0);
                    for (uint32_t n : w.nodes) in[n] = 1;
                    for (uint32_t n : w.rim) in[n] = 1;
                    in[w.axle0] = in[w.axle1] = 1;
                    for (const phys::Beam& bm : b.beams)
                        if (in[bm.a] && in[bm.b] && bm.L0 > 0 && !(bm.flags & phys::BF_BROKEN)) {
                            const float pl = std::fabs(bm.L / bm.L0 - 1.0f);
                            wmax = std::max(wmax, pl), wbent += pl > 0.01f;
                        }
                }
                int bent = 0;
                float dmax = 0;
                std::vector<std::pair<float, size_t>> worst;
                for (size_t ei = 0; ei < b.fem.elems.size(); ei++) {
                    const phys::FrameElement& e = b.fem.elems[ei];
                    if (e.broken) continue;
                    bent += e.damage > 2e-3f, dmax = std::max(dmax, e.damage);
                    if (e.damage > 2e-3f) worst.push_back({e.damage, ei});
                }
                printf(" | up %.2f | wheels: %d beams bent >1%%, most %.1f%% | frame: %d members bent (most %.3f rad)", v->up().y, wbent, wmax * 100.0f, bent, dmax);
                // (the suspension's members: those at the wheels' axle nodes or one member from them - the uprights,
                // the wishbones, the tie rods and toe links - bent for good or torn)
                {
                    std::vector<char> near(b.fem.node.size(), 0);
                    for (const auto& w : b.wheels)
                        for (uint32_t an : {w.axle0, w.axle1})
                            if (const int sl = b.fem.slot(an); sl >= 0) near[sl] = 2;
                    for (const phys::FrameElement& e : b.fem.elems)
                        if (near[e.a] == 2 || near[e.b] == 2) near[e.a] = std::max<char>(near[e.a], 1), near[e.b] = std::max<char>(near[e.b], 1);
                    int sb = 0, st = 0;
                    float sm = 0;
                    size_t si = 0;
                    for (size_t ei = 0; ei < b.fem.elems.size(); ei++) {
                        const phys::FrameElement& e = b.fem.elems[ei];
                        if (!near[e.a] && !near[e.b]) continue;
                        if (e.broken) {
                            st++;
                            continue;
                        }
                        sb += e.damage > 2e-3f;
                        if (e.damage > sm) sm = e.damage, si = ei;
                    }
                    printf(" | susp: %d bent (most %.3f rad), %d torn", sb, sm, st);
                    if (sm > 2e-3f) { // (the worst: its plastic moment, where in the car's axes: forward, up, left)
                        const phys::FrameElement& e = b.fem.elems[si];
                        const vec3 fw = v->forward(), upv = v->up(), lf = normalize_or(cross(upv, fw), vec3(1, 0, 0)), o = v->position();
                        const vec3 m = (b.nodes[b.fem.node[e.a]].p + b.nodes[b.fem.node[e.b]].p) * 0.5f - o;
                        printf(" [Mp %.0f at %.2f %.2f %.2f]", b.fem.sections[e.section].Mp, dot(m, fw), dot(m, upv), dot(m, lf));
                    }
                }
                // (the worst: their section's plastic moment and where, in the car's axes: forward, up, left)
                std::sort(worst.rbegin(), worst.rend());
                const vec3 fw = v->forward(), upv = v->up(), lf = normalize_or(cross(upv, fw), vec3(1, 0, 0)), o = v->position();
                for (size_t k = 0; k < worst.size() && k < 4; k++) {
                    const phys::FrameElement& e = b.fem.elems[worst[k].second];
                    const vec3 m = (b.nodes[b.fem.node[e.a]].p + b.nodes[b.fem.node[e.b]].p) * 0.5f - o;
                    printf(" [%.3f Mp %.0f at %.2f %.2f %.2f]", worst[k].first, b.fem.sections[e.section].Mp, dot(m, fw), dot(m, upv), dot(m, lf));
                }
            }
            if (const phys::FemFrame& f = v->body->fem; !f.tris.empty()) { // (its FEM triangles: how many torn out, dented for good)
                int dented = 0;
                for (const phys::FrameTri& t : f.tris) dented += !t.broken && t.dmg > 0;
                printf(" | tris %zu, %d torn, %d dented", f.tris.size(), f.tris_torn, dented);
                if (!f.mounts.empty()) printf(", mounts %d of %zu let go", f.mounts_broken, f.mounts.size());
                printf(", %d loose, %d left on a corner, %d bisections, %zu patterns", f.loose_count, std::max(0, f.vertex_hinges() - f.authored_hinges),
                       f.tris_refined, f.impacts.size());
            }
            if (!v->body->fem.empty()) { // (its parts held together by anything: the two largest' share of its mass - a car cut in two, two halves)
                float p0, p1;
                v->body->largest_parts(p0, p1);
                printf(" | parts %.0f%% %.0f%%", 100.0f * p0, 100.0f * p1);
            }
            if (!v->body->volumes.empty()) { // (its collision volumes: their contacts so far, the largest force, off)
                printf(" | volumes:");
                for (const phys::CollisionVolume& cv : v->body->volumes)
                    printf(" %s %d (%.0f kN)%s", cv.name.c_str(), cv.hits, cv.peak / 1000.0f, cv.broken ? " off" : "");
            }
            if (!v->body->welds.empty()) {
                // (and how the panels shake on their welds: the welded nodes' centre against the frame's point, rms and most)
                const phys::SoftBody& b = *v->body;
                double s2 = 0;
                float most = 0;
                int n = 0;
                for (const phys::Weld& wd : b.welds) {
                    if (wd.broken) continue;
                    vec3 cv(0);
                    for (uint32_t i = wd.first; i < wd.first + wd.count; i++) cv += b.nodes[b.weld_nodes[i]].v * b.weld_w[i];
                    const vec3 av = b.nodes[wd.anchor].v * (1 - wd.t) + b.nodes[wd.anchor2].v * wd.t;
                    const float r = length(cv - av);
                    s2 += (double)r * r, most = std::max(most, r), n++;
                    if (getenv("BL_JITTERDBG") && r > 0.25f) // (diagnostics: which welds shake)
                        printf("\n  weld %d shakes %.0f mm/s: node %u (%.2f %.2f %.2f), k %.0f, c %.1f, %u nodes", (int)(&wd - b.welds.data()), r * 1000.0f,
                               b.weld_nodes[wd.first], b.nodes[b.weld_nodes[wd.first]].p.x, b.nodes[b.weld_nodes[wd.first]].p.y, b.nodes[b.weld_nodes[wd.first]].p.z, wd.k,
                               wd.c, wd.count);
                }
                printf(" | welds: %d of %zu broken, panels shake %.0f mm/s rms, %.0f most, move %.1f mm/s rms, %.0f most", b.stats.broken_welds, b.welds.size(),
                       n ? 1000.0 * std::sqrt(s2 / n) : 0.0, most * 1000.0f, shake_n ? 1000.0 * std::sqrt(shake_s2 / shake_n) : 0.0, shake_most * 1000.0f);
                shake_s2 = 0, shake_most = 0, shake_n = 0;
                if (getenv("BL_JITTERDBG")) printf(" | membrane: %d edges out of band, %d moved", b.mem_edges_out, b.mem_moved);
            }
            printf("\n");
        } else if (!m_game.scene_status.empty()) { // (a scene without a car: its own line)
            printf("t=%6.1fs  | %s\n", m_game.world.time(), m_game.scene_status.c_str());
        }
    }
    if (m_opt.launch_kmh > 0 && m_frame_index == 5)
        if (Vehicle* v = m_game.player_vehicle()) v->launch(v->forward() * (m_opt.launch_kmh / 3.6f));
    if (!m_opt.action.empty()) // (the scene's actions by their labels' parts: "label[@frame]; ..." - at frame 5 by default)
        for (const std::string& item : split_any(m_opt.action, ";")) {
            const size_t at = item.rfind('@');
            const std::string label = trim(item.substr(0, at));
            const int frame = at == std::string::npos ? 5 : atoi(item.c_str() + at + 1);
            if (m_frame_index != frame || label.empty()) continue;
            for (const SceneAction& a : m_game.scene_actions)
                if (a.label.find(label) != std::string::npos) {
                    log_info("scene action: %s", a.label.c_str());
                    a.run(m_game);
                    break;
                }
        }
    if (!m_opt.laser.empty()) {
        // test: a laser sweep across the screen: "x0,y0,x1,y1[,first frame,frames]" in screen fractions (y down)
        float x0 = 0.3f, y0 = 0.5f, x1 = 0.7f, y1 = 0.5f, f0 = 30, nf = 30;
        sscanf(m_opt.laser.c_str(), "%f,%f,%f,%f,%f,%f", &x0, &y0, &x1, &y1, &f0, &nf);
        const int k = m_frame_index - (int)f0;
        if (k >= 0 && k <= (int)nf) {
            const Camera& c = m_game.last_camera();
            const float t = nf > 0 ? (float)k / nf : 1.0f;
            const vec2 ndc((x0 + (x1 - x0) * t) * 2 - 1, 1 - (y0 + (y1 - y0) * t) * 2);
            const mat4 inv = inverse(c.viewproj);
            const vec4 a = inv * vec4(ndc.x, ndc.y, -1, 1), b = inv * vec4(ndc.x, ndc.y, 1, 1);
            const vec3 ro = a.xyz() / a.w, rd = normalize(b.xyz() / b.w - ro);
            // (the mouse input releases the laser every frame without a button: the sweep keeps its own last ray)
            static vec3 prev_dir;
            m_game.tool = Tool::Laser;
            m_game.update_cursor(ro, rd);
            if (k > 0) m_game.laser_total += m_game.world.laser_cut(c.pos, prev_dir, rd, m_game.laser_range);
            prev_dir = rd;
            m_game.laser_active = k < (int)nf; // (the beam is drawn while it sweeps)
            m_game.laser_origin = c.pos;
            m_game.laser_dir = rd;
            if (k == (int)nf) printf("laser: %d cuts\n", m_game.laser_total);
        }
    }
    if (m_opt.autodestroy > 0 && m_frame_index > 10) {
        const Camera& c = m_game.last_camera();
        m_game.destroy_radius = m_opt.autodestroy;
        if (m_game.update_cursor(c.pos, c.forward())) m_game.destroy_under_cursor();
        if (m_frame_index % 20 == 0)
            printf("destroy: cursor %d (%.2f %.2f %.2f) total %d\n", (int)m_game.cursor_valid, m_game.cursor_point.x, m_game.cursor_point.y,
                   m_game.cursor_point.z, m_game.destroyed_total);
    }
    m_game.update(dt, vin, cin);
    if (m_frame_index == 3 && !m_opt.camera.empty()) apply_camera_preset(m_opt.camera);
    if (m_frame_index == 3 && m_opt.editor) m_editor.open();
    if (m_frame_index == 6 && m_opt.editor_test) m_editor.test_drive();
    {
        PROFILE_ZONE("UI");
        uint64_t t0 = prof::now();
        ui_main_menu();
        if (m_show_hud && !(m_editor.active() && !m_editor.driving())) ui_hud(); // (the editor has its own overlay)
        ui_mouse_steer();
        if (m_show_perf) ui_perf();
        if (m_show_help) ui_help();
        if (m_show_vehicle_info) ui_vehicle_info();
        if (m_show_log) ui_log();
        if (m_show_demo) ImGui::ShowDemoWindow(&m_show_demo);
        m_editor.ui(m_font_small);
        ui_toasts();
        m_ui_ms = prof::ticks_to_ms(prof::now() - t0);
    }
    glfwGetFramebufferSize(m_win, &m_fb_w, &m_fb_h);
    const Shot* shot = nullptr;
    for (const Shot& sh : m_shots)
        if (sh.frame == m_frame_index) shot = &sh;
    if (shot) {
        apply_camera_preset(shot->camera);
        m_game.cam.update(0, CameraInput(), false, vec3(0), vec3(0, 0, 1), vec3(0), vec3(0), vec3(0, 1, 0));
        m_game.debug.beams = shot->beams;
        m_renderer.debug_depth_test = !shot->beams; // the wireframe over the (thick) sheets
        m_hide_ui = shot->noui;
        if (shot->pause) m_game.paused = true;
    }
    {
        uint64_t t0 = prof::now();
        m_editor.render(m_renderer, m_fb_w, m_fb_h); // (the game's render, with the editor's views when it is open)
        {
            PROFILE_ZONE("ImGui render");
            ImGui::Render();
            if (!m_hide_ui) ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }
        m_render_ms = prof::ticks_to_ms(prof::now() - t0);
    }
    if (shot) {
        glFinish();
        if (m_renderer.screenshot(shot->path, m_fb_w, m_fb_h)) log_info("shot saved: %s", shot->path.c_str());
        if (shot == &m_shots.back()) glfwSetWindowShouldClose(m_win, 1);
    }
    m_cpu_ms = prof::ticks_to_ms(prof::now() - t_frame);
    if (!m_opt.record.empty()) {
        // video frames: every n-th frame from the first one asked for, numbered from 0
        static std::string dir;
        static int every = 1, first = 0, count = 0;
        if (dir.empty()) {
            const size_t c = m_opt.record.find(',');
            dir = m_opt.record.substr(0, c);
            if (c != std::string::npos) sscanf(m_opt.record.c_str() + c + 1, "%d,%d", &every, &first);
            every = std::max(1, every);
        }
        if (m_frame_index >= first && (m_frame_index - first) % every == 0) {
            glFinish();
            char path[1024];
            snprintf(path, sizeof path, "%s/frame_%05d.png", dir.c_str(), count++);
            m_renderer.screenshot(path, m_fb_w, m_fb_h);
        }
    }
    if (!m_opt.screenshot.empty() && m_frame_index == m_opt.screenshot_frames) {
        glFinish();
        if (m_renderer.screenshot(m_opt.screenshot, m_fb_w, m_fb_h)) log_info("screenshot saved: %s", m_opt.screenshot.c_str());
        glfwSetWindowShouldClose(m_win, 1);
    }
    {
        PROFILE_ZONE("Swap");
        glfwSwapBuffers(m_win);
    }
    m_frame_index++;
}

int App::run(const AppOptions& opt) {
    if (!init(opt)) {
        shutdown();
        return 1;
    }
    const bool bench = opt.bench_frames > 0;
    std::vector<double> phys_ms, frame_ms;
    while (!glfwWindowShouldClose(m_win)) {
        double now = glfwGetTime();
        float dt = (float)(now - m_last_time);
        m_last_time = now;
        static const bool live = getenv("BL_LIVE") != nullptr; // (diagnostics: scripted runs stepped like the interactive app)
        const bool fixed = (bench || !opt.screenshot.empty() || !opt.shots.empty()) && !opt.realtime && !live;
        if (fixed) dt = 1.0f / 60.0f; // deterministic stepping
        dt = std::min(dt, 0.1f);
        m_frame_ms = dt * 1000.0;
        m_fps = m_fps * 0.9f + 0.1f * (dt > 0 ? 1.0f / dt : 0);
        // (a frame the physics could not keep up with is not caught up on: twice the steps next frame take twice as long, and
        // the rate falls to the clamp - the simulation slows down under 30 frames a second instead)
        static const float max_step = getenv("BL_MAXSTEP") ? (float)atof(getenv("BL_MAXSTEP")) : 0.034f;
        if (!fixed) dt = std::min(dt, max_step);
        if (!fixed && !opt.realtime) dt = pace(dt);
        static const bool prof_env = getenv("BL_PROFCSV") != nullptr;
        m_game.world.settings.element_stats = m_show_perf || prof_env || !m_rec_path.empty(); // (counting costs a pass)
        frame(dt);
        prof::end_frame();
        m_perf.Update(dt);
        write_prof_csv(dt);
        if (getenv("BL_HASHDBG")) { // (diagnostics: a hash of every node's position and velocity, each frame - two builds compared)
            uint64_t hsh = 1469598103934665603ull;
            for (const auto& bp : m_game.world.bodies())
                for (const phys::Node& x : bp->nodes) {
                    const float v[6] = {x.p.x, x.p.y, x.p.z, x.v.x, x.v.y, x.v.z};
                    const unsigned char* c = (const unsigned char*)v;
                    for (size_t i = 0; i < sizeof(v); i++) hsh = (hsh ^ c[i]) * 1099511628211ull;
                }
            printf("hash %d %016llx\n", m_frame_index, (unsigned long long)hsh);
        }
        record_elements();
        if (opt.realtime) // like vsync at 60 Hz: a frame takes at least 1/60 s
            while (glfwGetTime() - now < 1.0 / 60.0) std::this_thread::sleep_for(std::chrono::microseconds(200));
        if (m_toast_time > 0) m_toast_time -= dt;
        if (bench) {
            if (m_frame_index > 30) { // skip warm-up
                phys_ms.push_back(m_game.frame_physics_ms);
                frame_ms.push_back(m_cpu_ms);
            }
            if (m_frame_index >= opt.bench_frames + 30) break;
        }
    }
    if (bench && !phys_ms.empty()) {
        auto stats = [](std::vector<double> v, const char* name) {
            std::sort(v.begin(), v.end());
            double avg = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
            printf("%-14s avg %7.2f  p50 %7.2f  p95 %7.2f  max %7.2f ms\n", name, avg, v[v.size() / 2], v[v.size() * 95 / 100], v.back());
        };
        const auto& st = m_game.world.stats();
        printf("\n=== BENCH: %s | %d frames | threads %d ===\n", scene_registry()[m_game.scene_index].name.c_str(), (int)phys_ms.size(),
               JobSystem::get().num_threads());
        printf("bodies %d (active %d, sleeping %d), islands %d, active nodes %d, beams %d, contacts %d, pairs %d\n", (int)m_game.world.bodies().size(),
               st.active_bodies, st.sleeping_bodies, st.islands, st.active_nodes, st.active_beams, st.contacts, st.contact_pairs);
        stats(phys_ms, "physics/frame");
        stats(frame_ms, "cpu frame");
        printf("zones (last frame, summed over threads):\n");
        for (auto* z : prof::zones())
            if (z->avg_ms > 0.01) printf("  %-22s %8.3f ms  (%u calls)\n", z->name, z->avg_ms, z->last_calls);
    }
    shutdown();
    return 0;
}

// Frame pacing. With vsync the swap returns at uneven times (the compositor lets one frame through at once and blocks
// the next: measured frame times alternate ~3 and ~13 ms on a 120 Hz display), while the display still shows frames at
// its own even rate. Stepping the simulation by the measured times moved the scene unevenly from one shown frame to the
// next. The simulation steps by the average frame time instead, pulled towards the real clock (no drift: whatever it
// lags or leads is made up over a few tenths of a second); after a real hitch (> 0.1 s behind) it resynchronises.
float App::pace(float raw) {
    m_pace_avg = m_pace_avg <= 0 ? raw : m_pace_avg * 0.95 + raw * 0.05;
    m_pace_debt += raw;
    if (m_pace_debt > 0.1 || m_pace_debt < -0.1) {
        const float step = (float)std::clamp(m_pace_debt, 0.0, 0.1);
        m_pace_debt = 0;
        return step;
    }
    const double step = std::clamp(m_pace_avg + 0.04 * (m_pace_debt - m_pace_avg), 0.25 * m_pace_avg, 2.0 * m_pace_avg);
    m_pace_debt -= step;
    return (float)step;
}

bool App::measure_cache() {
    phys::SoftBody* best = nullptr;
    for (const auto& b : m_game.world.bodies()) // (the biggest sheet: the layout matters, not whether it moves now)
        if (!b->shells.empty() && (!best || b->shells.size() > best->shells.size())) best = b.get();
    if (!best) return false;
    const phys::KernelCacheReport r = phys::measure_kernel_cache(*best);
    m_cache_text = r.text();
    m_cache_body = best->name;
    m_cache_tris = r.shells;
    m_cache_cur = {r.cur.l1_hit_rate() * 100.0, r.cur_l2_bytes_per_tri, r.cur.line_utilisation(128) * 100.0, r.cur_bytes_per_tri, r.working_set_cur};
    m_cache_ref = {r.ref.l1_hit_rate() * 100.0, r.ref_l2_bytes_per_tri, r.ref.line_utilisation(128) * 100.0, r.ref_bytes_per_tri, r.working_set_ref};
    return true;
}

// Profiling log (BL_PROFCSV=<file>): per frame the physics zones (ms, summed over threads), the heaviest island and
// the sheet counters, for offline analysis of a run.
void App::write_prof_csv(float dt) {
    static const char* env = getenv("BL_PROFCSV");
    const char* path = env ? env : (m_rec_path.empty() ? nullptr : m_rec_path.c_str());
    static const char* zone_names[] = {"Frame", "Physics", "Islands build", "Islands", "Island", "Forces", "Collisions", "Integration", "Beam forces",
                                       "Sheet elements", "Sheet gather", "Static collisions", "Integrate", "Broadphase", "Broadphase query", "Fast pairs",
                                       "Fast pair query", "Fast pair refresh", "Narrow phase", "Contacts", "Pair inheritance", "Sheet topology",
                                       "Sheet refine", "Sheet cracks", "Sheet detach", "Sheet reorder", "Visual update", "Sheet mesh", "Sheet mesh rebuild", "Sheet upload",
                                       "Sheet pattern", "Sheet pattern codes", "Sheet settle", "Sheet split point", "Sheet coarsen", "Sheet fx", "Sheet budget", "Sheet sync",
                                       "Render", "Shadows", "Main pass", "UI", "ImGui render", "Draw submit", "Swap", "Input", "Events",
                                       "Frame elements", "Frame solve", "Frame assemble", "Frame factor", "Sheet membrane", "Sheet spheres",
                                       "Volumes", "Near pairs", "Frame held", "Frame sync", "Sheet events", "FEM assemble", "FEM factor", "FEM finish", "FEM repass", "Volume mids", "Volume find", "Volume push", "Frame events", "Short step first", "Short steps later"};
    if (!path) return;
    if (!m_prof_csv) {
        m_prof_csv = fopen(path, "w");
        if (!m_prof_csv) return;
        fprintf(m_prof_csv, "frame,t,dt_ms,substeps,physics_ms,cpu_ms,islands,islands_cpu_ms,heavy_ms,heavy_bodies,heavy_nodes,heavy_beams,"
                            "heavy_shells,heavy_sub,heavy_wide,heavy_body,active_bodies,active_nodes,awake_shells,shell_steps,beam_steps,"
                            "refines,cracks,pieces_created,narrow_tests,contacts,pair_rebuilds,fast_refreshes,team_wait_ms,team_stalls,speed_kmh,veh_z,"
                            "hp_forces,hp_gather,hp_collide,hp_integrate,hp_serial,hp_topology,hp_rebuild,hp_refresh,hp_contacts,raw_dt_ms");
        for (const char* z : zone_names) fprintf(m_prof_csv, ",%s", z);
        fprintf(m_prof_csv, ",bodies,bodies_awake,pieces,pieces_awake,pieces_rigid,nodes,nodes_awake,beams,beams_awake,beams_broken,shells,shells_awake,"
                            "shells_L0,shells_L1,shells_L2,shells_L3,shells_L4,shells_x1,shells_x2,shells_x4,hinges,hinges_awake,edges_border,"
                            "edges_crack,edges_cut,tris,tris_awake,impacts,node_steps,shell_evals,hinge_evals,beam_evals,contact_pairs,gpu_ms,fb_w,fb_h");
        fprintf(m_prof_csv, "\n");
    }
    const auto& st = m_game.world.stats();
    Vehicle* v = m_game.player_vehicle();
    fprintf(m_prof_csv, "%d,%.4f,%.3f,%d,%.3f,%.3f,%d,%.3f,%.3f,%d,%d,%d,%d,%d,%d,\"%s\",%d,%d,%d,%lld,%lld,%d,%d,%d,%lld,%d,%d,%d,%.3f,%d,%.2f,%.2f",
            m_frame_index, m_game.world.time(), dt * 1000.0f, st.substeps, m_game.frame_physics_ms, m_cpu_ms, st.islands, st.islands_cpu_ms,
            st.heavy_island_ms, st.heavy_island_bodies, st.heavy_island_nodes, st.heavy_island_beams, st.heavy_island_shells, st.heavy_island_sub,
            st.heavy_island_wide ? 1 : 0, st.heavy_island_body.c_str(), st.active_bodies, st.active_nodes, st.awake_shells, st.shell_steps,
            st.beam_steps, st.shell_refines, st.shell_cracks, st.pieces_created, st.narrow_tests, st.contacts, st.pair_rebuilds,
            st.fast_refreshes, st.team_wait_ms, st.team_stalls, v ? v->speed_kmh() : 0.0f, v ? v->position().z : 0.0f);
    for (double x : st.heavy_phase_ms) fprintf(m_prof_csv, ",%.4f", x);
    fprintf(m_prof_csv, ",%.3f", m_frame_ms);
    for (const char* z : zone_names) {
        const prof::Zone* zz = prof::find_zone(z);
        fprintf(m_prof_csv, ",%.4f", zz ? zz->last_ms : 0.0);
    }
    const auto& e = st.el;
    fprintf(m_prof_csv, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lld,%lld,%lld,%lld,%d", e.bodies,
            e.bodies_awake, e.pieces, e.pieces_awake, e.pieces_rigid, e.nodes, e.nodes_awake, e.beams, e.beams_awake, e.beams_broken, e.shells, e.shells_awake,
            e.shells_level[0], e.shells_level[1], e.shells_level[2], e.shells_level[3], e.shells_level[4], e.shells_rate[0], e.shells_rate[1],
            e.shells_rate[2], e.hinges, e.hinges_awake, e.edges_border, e.edges_crack, e.edges_cut, e.tris, e.tris_awake, e.impacts, e.node_steps,
            e.shell_evals, e.hinge_evals, e.beam_evals, st.contact_pairs);
    fprintf(m_prof_csv, ",%.3f,%d,%d\n", m_renderer.gpu_ms(), m_fb_w, m_fb_h);
}

// One sample of the Elements graphs per frame (while the performance widget is open and not frozen).
void App::record_elements() {
    if (!m_show_perf || m_elem_freeze) return;
    constexpr int kHist = 600;
    static double last_t = 0;
    const double t = m_game.world.time();
    if ((int)m_elem_hist.size() != kHist || t < last_t) { // (a new scene: a new history)
        m_elem_hist.assign(kHist, ElemSample{});
        m_elem_head = m_elem_count = 0;
    }
    last_t = t;
    if (t < 0.1) return; // (the loading frames: their times are not the simulation's)
    const auto& st = m_game.world.stats();
    const auto& e = st.el;
    auto zms = [](const char* n) {
        const prof::Zone* z = prof::find_zone(n);
        return z ? (float)z->last_ms : 0.0f;
    };
    ElemSample& x = m_elem_hist[m_elem_head];
    x.v[0] = (float)e.shells;
    x.v[1] = (float)e.shells_awake;
    x.v[2] = (float)(e.shells_level[3] + e.shells_level[4]);
    x.v[3] = (float)e.hinges_awake;
    x.v[4] = (float)e.beams_awake;
    x.v[5] = (float)st.contacts;
    x.v[6] = (float)e.shell_evals;
    x.v[7] = (float)e.node_steps;
    x.v[8] = (float)e.hinge_evals;
    x.v[9] = (float)e.beam_evals;
    x.v[10] = (float)m_game.frame_physics_ms;
    x.v[11] = zms("Sheet elements") + zms("Sheet gather");
    x.v[12] = zms("Broadphase") + zms("Fast pair refresh") + zms("Narrow phase") + zms("Contacts") + zms("Static collisions") + zms("Pair inheritance");
    x.v[13] = zms("Sheet topology") + zms("Sheet detach");
    x.v[14] = (float)e.nodes_awake;
    x.v[15] = (float)e.pieces_awake;
    m_elem_head = (m_elem_head + 1) % kHist;
    m_elem_count = std::min(m_elem_count + 1, kHist);
}

void App::shutdown() {
    if (m_prof_csv) fclose(m_prof_csv);
    m_prof_csv = nullptr;
    m_game.clear_vehicles();
    m_game.objects.clear();
    m_game.world.clear();
    if (ImGui::GetCurrentContext()) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }
    JobSystem::get().shutdown();
    if (m_win) glfwDestroyWindow(m_win);
    glfwTerminate();
}

} // namespace bl
