// ImGui user interface: main menu bar, HUD, performance widget, help and info windows.
#include "core/jobs.h"
#include "core/profiler.h"
#include "core/util.h"
#include "game/app.h"
#include "vehicle/drivetrain.h"
#include "vehicle/vehicle.h"
#include "world/ai.h"

#include "imgui.h"
#include "imgui_internal.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <functional>
#include <cmath>
#include <ctime>
#include <unistd.h>

namespace bl {

namespace {
const ImVec4 kAccent(0.96f, 0.62f, 0.16f, 1.0f);

ImU32 col32(float r, float g, float b, float a = 1.0f) { return ImGui::ColorConvertFloat4ToU32(ImVec4(r, g, b, a)); }

void draw_gauge(ImDrawList* dl, ImVec2 c, float radius, float value, float max_value, const char* label, const char* text, ImFont* big, ImFont* small,
                float redline = -1) {
    const float a0 = kPi * 0.75f, a1 = kPi * 2.25f;
    dl->AddCircleFilled(c, radius + 6, col32(0.05f, 0.06f, 0.08f, 0.72f), 48);
    dl->PathArcTo(c, radius - 4, a0, a1, 48);
    dl->PathStroke(col32(1, 1, 1, 0.12f), 0, 6.0f);
    if (redline > 0) {
        float ar = a0 + (a1 - a0) * clampf(redline / max_value, 0, 1);
        dl->PathArcTo(c, radius - 4, ar, a1, 24);
        dl->PathStroke(col32(0.9f, 0.2f, 0.15f, 0.6f), 0, 6.0f);
    }
    float t = clampf(value / std::max(1e-3f, max_value), 0, 1);
    float av = a0 + (a1 - a0) * t;
    dl->PathArcTo(c, radius - 4, a0, av, 48);
    dl->PathStroke(ImGui::ColorConvertFloat4ToU32(kAccent), 0, 6.0f);
    // ticks
    for (int i = 0; i <= 10; i++) {
        float a = a0 + (a1 - a0) * i / 10.0f;
        ImVec2 d(std::cos(a), std::sin(a));
        dl->AddLine(ImVec2(c.x + d.x * (radius - 14), c.y + d.y * (radius - 14)), ImVec2(c.x + d.x * (radius - 9), c.y + d.y * (radius - 9)),
                    col32(1, 1, 1, 0.45f), 1.5f);
    }
    ImVec2 nd(std::cos(av), std::sin(av));
    dl->AddLine(c, ImVec2(c.x + nd.x * (radius - 12), c.y + nd.y * (radius - 12)), col32(1, 0.35f, 0.2f), 2.5f);
    dl->AddCircleFilled(c, 5, col32(0.9f, 0.9f, 0.9f));
    ImGui::PushFont(big, 0.0f);
    ImVec2 ts = ImGui::CalcTextSize(text);
    dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y + radius * 0.28f), col32(1, 1, 1), text);
    ImGui::PopFont();
    ImGui::PushFont(small, 0.0f);
    ImVec2 ls = ImGui::CalcTextSize(label);
    dl->AddText(ImVec2(c.x - ls.x * 0.5f, c.y + radius * 0.28f + ts.y + 1), col32(1, 1, 1, 0.55f), label);
    ImGui::PopFont();
}

void bar(ImDrawList* dl, ImVec2 p, ImVec2 size, float v, ImU32 col, const char* label, ImFont* small) {
    dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), col32(0.05f, 0.06f, 0.08f, 0.72f), 4);
    float h = size.y * clampf(v, 0, 1);
    dl->AddRectFilled(ImVec2(p.x + 2, p.y + size.y - h), ImVec2(p.x + size.x - 2, p.y + size.y - 2), col, 3);
    ImGui::PushFont(small, 0.0f);
    ImVec2 ls = ImGui::CalcTextSize(label);
    dl->AddText(ImVec2(p.x + (size.x - ls.x) * 0.5f, p.y + size.y + 2), col32(1, 1, 1, 0.55f), label);
    ImGui::PopFont();
}
} // namespace

void App::setup_style() {
    ImGuiStyle& s = ImGui::GetStyle();
    ImGui::StyleColorsDark();
    s.WindowRounding = 8;
    s.FrameRounding = 5;
    s.PopupRounding = 6;
    s.GrabRounding = 4;
    s.TabRounding = 5;
    s.ScrollbarRounding = 6;
    s.WindowBorderSize = 0;
    s.PopupBorderSize = 1;
    s.FramePadding = ImVec2(8, 4);
    s.ItemSpacing = ImVec2(8, 6);
    s.WindowPadding = ImVec2(10, 10);
    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.09f, 0.11f, 0.92f);
    c[ImGuiCol_PopupBg] = ImVec4(0.09f, 0.10f, 0.12f, 0.97f);
    c[ImGuiCol_MenuBarBg] = ImVec4(0.07f, 0.08f, 0.10f, 0.96f);
    c[ImGuiCol_FrameBg] = ImVec4(0.16f, 0.17f, 0.20f, 1.0f);
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.22f, 0.23f, 0.27f, 1.0f);
    c[ImGuiCol_FrameBgActive] = ImVec4(0.26f, 0.27f, 0.31f, 1.0f);
    c[ImGuiCol_Header] = ImVec4(0.96f, 0.62f, 0.16f, 0.30f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.96f, 0.62f, 0.16f, 0.45f);
    c[ImGuiCol_HeaderActive] = ImVec4(0.96f, 0.62f, 0.16f, 0.60f);
    c[ImGuiCol_Button] = ImVec4(0.18f, 0.19f, 0.23f, 1.0f);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.96f, 0.62f, 0.16f, 0.55f);
    c[ImGuiCol_ButtonActive] = ImVec4(0.96f, 0.62f, 0.16f, 0.80f);
    c[ImGuiCol_CheckMark] = kAccent;
    c[ImGuiCol_SliderGrab] = kAccent;
    c[ImGuiCol_SliderGrabActive] = ImVec4(1.0f, 0.72f, 0.3f, 1.0f);
    c[ImGuiCol_TitleBg] = ImVec4(0.08f, 0.09f, 0.11f, 1.0f);
    c[ImGuiCol_TitleBgActive] = ImVec4(0.11f, 0.12f, 0.15f, 1.0f);
    c[ImGuiCol_Separator] = ImVec4(1, 1, 1, 0.10f);
    c[ImGuiCol_Text] = ImVec4(0.92f, 0.93f, 0.95f, 1.0f);
    c[ImGuiCol_TextDisabled] = ImVec4(0.55f, 0.57f, 0.62f, 1.0f);
}

void App::ui_main_menu() {
    // global hotkeys
    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureKeyboard && (io.KeyCtrl || io.KeySuper) && ImGui::IsKeyPressed(ImGuiKey_E)) {
        if (m_editor.active()) m_editor.close();
        else m_editor.open();
    }
    if (!io.WantCaptureKeyboard) {
        if (ImGui::IsKeyPressed(ImGuiKey_F1)) m_show_help = !m_show_help;
        if (ImGui::IsKeyPressed(ImGuiKey_F2)) m_show_perf = !m_show_perf;
        if (ImGui::IsKeyPressed(ImGuiKey_F3)) m_game.debug.beams = !m_game.debug.beams;
        if (ImGui::IsKeyPressed(ImGuiKey_F4)) m_game.debug.hide_meshes = !m_game.debug.hide_meshes;
        if (ImGui::IsKeyPressed(ImGuiKey_F6)) m_game.debug.xray = !m_game.debug.xray;
        if (ImGui::IsKeyPressed(ImGuiKey_F7)) m_game.debug.volumes = !m_game.debug.volumes;
        if (ImGui::IsKeyPressed(ImGuiKey_F8)) m_game.debug.stress = !m_game.debug.stress;
    }
    if (!io.WantCaptureKeyboard && (!m_editor.active() || m_editor.driving())) { // (the editor has its own keys)
        if (ImGui::IsKeyPressed(ImGuiKey_P)) m_game.paused = !m_game.paused;
        if (ImGui::IsKeyPressed(ImGuiKey_N)) m_game.step_once = true;
        if (ImGui::IsKeyPressed(ImGuiKey_C)) {
            auto next = (CameraController::Mode)(((int)m_game.cam.mode + 1) % 4);
            if (next == CameraController::FREE) m_game.cam.enter_free();
            else m_game.cam.mode = next;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_H)) m_show_hud = !m_show_hud;
        if (ImGui::IsKeyPressed(ImGuiKey_R) && m_game.player_vehicle()) {
            if (io.KeyShift) m_game.player_vehicle()->reset(m_game.spawn_pos, m_game.spawn_yaw);
            else m_game.player_vehicle()->recover();
        }
        // ---- time control
        auto& ts = m_game.world.settings.time_scale;
        const int cur = time_scale_index(ts);
        if (ImGui::IsKeyPressed(ImGuiKey_LeftBracket) || ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract)) ts = kTimeScales[std::max(0, cur - 1)];
        if (ImGui::IsKeyPressed(ImGuiKey_RightBracket) || ImGui::IsKeyPressed(ImGuiKey_KeypadAdd)) ts = kTimeScales[std::min(kTimeScaleCount - 1, cur + 1)];
        if (ImGui::IsKeyPressed(ImGuiKey_T)) ts = ts < 0.99f ? 1.0f : 0.2f;
        if (ImGui::IsKeyPressed(ImGuiKey_Backspace)) ts = 1.0f;
        // ---- cameras / vehicles / scene
        if (ImGui::IsKeyPressed(ImGuiKey_F)) {
            if (m_game.cam.mode != CameraController::FREE) {
                m_game.cam.enter_free();
            } else if (m_game.player_vehicle()) {
                m_game.cam.mode = CameraController::CHASE;
            }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Tab)) m_game.next_vehicle();
        if (ImGui::IsKeyPressed(ImGuiKey_F5)) select_scene(m_game.scene_index);
        // ---- tools
        if (ImGui::IsKeyPressed(ImGuiKey_G)) m_game.tool = Tool::Grab;
        if (ImGui::IsKeyPressed(ImGuiKey_X)) m_game.tool = Tool::Destroy;
        if (ImGui::IsKeyPressed(ImGuiKey_B)) m_game.tool = Tool::Shoot;
        if (ImGui::IsKeyPressed(ImGuiKey_L)) m_game.tool = Tool::Laser;
        if (ImGui::IsKeyPressed(ImGuiKey_Z)) m_game.projectile_kind = (m_game.projectile_kind + 1) % Game::kProjectileKinds;
        static const float speeds[] = {5, 10, 20, 40, 70, 100, 150};
        int si = 0;
        for (int i = 0; i < 7; i++)
            if (std::fabs(speeds[i] - m_game.projectile_speed) < std::fabs(speeds[si] - m_game.projectile_speed)) si = i;
        if (ImGui::IsKeyPressed(ImGuiKey_Comma)) m_game.projectile_speed = speeds[std::max(0, si - 1)];
        if (ImGui::IsKeyPressed(ImGuiKey_Period)) m_game.projectile_speed = speeds[std::min(6, si + 1)];
        if (ImGui::IsKeyPressed(ImGuiKey_Delete)) m_game.clear_projectiles();
    }
    if (!ImGui::BeginMainMenuBar()) return;
    ImGui::PushStyleColor(ImGuiCol_Text, kAccent);
    bool app_menu = ImGui::BeginMenu("BeamLab");
    ImGui::PopStyleColor();
    if (app_menu) {
        ImGui::TextDisabled("Soft-body vehicle physics prototype");
        ImGui::Separator();
        ImGui::MenuItem("Help", "F1", &m_show_help);
        ImGui::MenuItem("Performance", "F2", &m_show_perf);
        ImGui::MenuItem("Vehicle info", nullptr, &m_show_vehicle_info);
        ImGui::MenuItem("Log", nullptr, &m_show_log);
        ImGui::MenuItem("ImGui demo", nullptr, &m_show_demo);
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Cmd+Q")) glfwSetWindowShouldClose(m_win, 1);
        ImGui::EndMenu();
    }
    ImGui::Separator();

    // (a dropdown's arrow at the right end of a menu bar's selector: the menu item's rect kept from a frame it was closed
    // - an open one's BeginMenu has made its popup the current window - drawn into the menu bar's draw list)
    struct ArrowAt {
        ImVec2 mn, mx;
    };
    auto dropdown_arrow = [](ImDrawList* bar, bool open, ArrowAt& at) {
        if (!open) at.mn = ImGui::GetItemRectMin(), at.mx = ImGui::GetItemRectMax();
        const float h = at.mx.y - at.mn.y, sz = ImGui::GetFontSize();
        if (h > 0)
            ImGui::RenderArrow(bar, ImVec2(at.mx.x - sz * 1.15f, at.mn.y + (h - sz) * 0.5f + sz * 0.05f), ImGui::GetColorU32(ImGuiCol_Text), ImGuiDir_Down, 0.7f);
    };
    // ---- scene selector: a menu of the categories (a category "A/B" is a submenu B of A), the current one's name on it
    const auto& scenes = scene_registry();
    ImGui::TextDisabled("World");
    const char* cur_scene = m_game.scene_index >= 0 ? scenes[m_game.scene_index].name.c_str() : "-";
    {
        char label[160];
        snprintf(label, sizeof label, "%s     ###scenemenu", cur_scene); // (room for the dropdown's arrow)
        static ArrowAt at;
        ImDrawList* bar = ImGui::GetWindowDrawList();
        const bool open = ImGui::BeginMenu(label);
        dropdown_arrow(bar, open, at);
        if (open) {
            // the items of one menu: the scenes of `path`, then its submenus in the order they first come
            std::function<void(const std::string&)> items = [&](const std::string& path) {
                std::vector<std::string> subs;
                for (int i = 0; i < (int)scenes.size(); i++) {
                    const std::string& c = scenes[i].category;
                    if (c == path) {
                        // (the submenu's name off the item's: "Frame Car: Head-on" in Frame Car is "Head-on")
                        std::string name = scenes[i].name;
                        const std::string leaf = path.substr(path.rfind('/') == std::string::npos ? 0 : path.rfind('/') + 1);
                        if (name.rfind(leaf + ": ", 0) == 0) name = name.substr(leaf.size() + 2);
                        if (ImGui::MenuItem(name.c_str(), nullptr, i == m_game.scene_index)) select_scene(i);
                        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", scenes[i].description.c_str());
                    } else if (c.size() > path.size() + 1 && c.compare(0, path.size() + 1, path + "/") == 0) {
                        const std::string sub = c.substr(0, c.find('/', path.size() + 1));
                        if (std::find(subs.begin(), subs.end(), sub) == subs.end()) subs.push_back(sub);
                    }
                }
                for (const std::string& sub : subs)
                    if (ImGui::BeginMenu(sub.substr(path.size() + 1).c_str())) {
                        items(sub);
                        ImGui::EndMenu();
                    }
            };
            std::vector<std::string> tops;
            for (const SceneInfo& sc : scenes) {
                const std::string t = sc.category.substr(0, sc.category.find('/'));
                if (std::find(tops.begin(), tops.end(), t) == tops.end()) tops.push_back(t);
            }
            for (const std::string& t : tops)
                if (ImGui::BeginMenu(t.c_str())) {
                    items(t);
                    ImGui::EndMenu();
                }
            ImGui::EndMenu();
        }
    }
    // ---- vehicle selector: a menu of the kinds of vehicle, a vehicle of several variants a submenu of them
    const auto& vehs = vehicle_registry();
    ImGui::TextDisabled("Vehicle");
    const VehicleEntry* cur = find_vehicle(m_game.selected_vehicle);
    // (scenes without a player vehicle: free camera only)
    ImGui::BeginDisabled(m_game.no_player_vehicle);
    const char* shown = m_game.no_player_vehicle ? "- (free camera)" : cur ? cur->title.c_str() : "-";
    {
        char label[200];
        snprintf(label, sizeof label, "%s     ###vehiclemenu", shown);
        static ArrowAt at;
        ImDrawList* bar = ImGui::GetWindowDrawList();
        const bool open = ImGui::BeginMenu(label);
        dropdown_arrow(bar, open, at);
        if (open) {
            static const char* kKinds[] = {"Cars", "SUVs, vans, pickups", "Off-road", "Trucks", "Buses", "BeamLab test cars", "Editor models", "Trailers and loads",
                                           "Other"};
            auto kind_of = [](const VehicleEntry& e) {
                static const std::pair<const char*, int> folders[] = {
                    {"audi_80", 0}, {"audi_quattro", 0}, {"bmw_e36", 0}, {"bmw_e39_m5", 0}, {"dodge_viper", 0}, {"mercedes_clk", 0}, {"seat_ibiza", 0},
                    {"toyota_ae86", 0}, {"ford_f250_2014", 1}, {"ford_f_1999", 1}, {"mercedes_vito", 1}, {"mercedes_w460", 1}, {"mitsubishi_pajero", 1},
                    {"trophy_truck_v2", 2}, {"autocar_xpeditor", 3}, {"freightliner_fla", 3}, {"kenworth_wrecker", 3}, {"kme_predator", 3}, {"lcf_trucks", 3},
                    {"tatra_815_6x6", 3}, {"thomas_hdx_bus", 4}, {"man_caetano_enigma", 4}, {"frame_car", 5}, {"buggy", 5}, {"shell_car", 5}, {"sheet_car", 5},
                    {"yaris_biw", 5}, {"editor", 6}};
                if (!e.drivable) return 7;
                for (const auto& [f, k] : folders)
                    if (e.folder == f) return k;
                return 8;
            };
            for (int k = 0; k < (int)(sizeof kKinds / sizeof kKinds[0]); k++) {
                bool any = false;
                for (const auto& e : vehs) any |= kind_of(e) == k;
                if (!any || !ImGui::BeginMenu(kKinds[k])) continue;
                std::vector<std::string> done; // (the folders listed)
                for (const auto& e : vehs) {
                    if (kind_of(e) != k || std::find(done.begin(), done.end(), e.folder) != done.end()) continue;
                    done.push_back(e.folder);
                    std::vector<const VehicleEntry*> vs;
                    for (const auto& x : vehs)
                        if (x.folder == e.folder && kind_of(x) == k) vs.push_back(&x);
                    auto item = [&](const VehicleEntry& x) {
                        ImGui::PushID(x.id.c_str());
                        if (ImGui::MenuItem(x.title.c_str(), nullptr, cur && cur->id == x.id)) select_vehicle(x.id, true);
                        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s\n%s", x.type.c_str(), path_filename(x.file).c_str());
                        ImGui::PopID();
                    };
                    if (vs.size() == 1 || k == 6) { // (one variant, or the editor's own models: each on its own)
                        for (const VehicleEntry* x : vs) item(*x);
                    } else {
                        // (the folder's name without its authors, who are in the tooltip)
                        const size_t par = e.group.rfind(" (");
                        const std::string name = par != std::string::npos && e.group.back() == ')' ? e.group.substr(0, par) : e.group;
                        const bool open_sub = ImGui::BeginMenu((name + "##" + e.folder).c_str());
                        if (!open_sub && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", e.group.c_str());
                        if (open_sub) {
                            for (const VehicleEntry* x : vs) item(*x);
                            ImGui::EndMenu();
                        }
                    }
                }
                ImGui::EndMenu();
            }
            ImGui::EndMenu();
        }
    }
    ImGui::EndDisabled();
    if (ImGui::Button("Reset")) {
        if (m_game.no_player_vehicle) select_scene(m_game.scene_index); // (scenes without a vehicle: reload)
        else if (Vehicle* v = m_game.player_vehicle()) v->recover();
        else select_vehicle(m_game.selected_vehicle, true);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(m_game.no_player_vehicle ? "Reload the scene (F5); the camera stays where it is."
                                                   : "Put the vehicle back on its wheels (R).\nShift+R: back to the spawn point.");
    ImGui::Separator();

    // ---- simulation controls
    if (ImGui::Button(m_game.paused ? " Play " : "Pause")) m_game.paused = !m_game.paused;
    ImGui::BeginDisabled(!m_game.paused);
    if (ImGui::Button("Step")) m_game.step_once = true;
    ImGui::EndDisabled();
    {
        char cur[16];
        snprintf(cur, sizeof cur, "%gx", m_game.world.settings.time_scale);
        ImGui::SetNextItemWidth(70);
        if (ImGui::BeginCombo("##speed", cur)) {
            for (int i = 0; i < kTimeScaleCount; i++) {
                char name[16];
                snprintf(name, sizeof name, "%gx", kTimeScales[i]);
                if (ImGui::Selectable(name, std::fabs(m_game.world.settings.time_scale - kTimeScales[i]) < 1e-5f)) m_game.world.settings.time_scale = kTimeScales[i];
            }
            ImGui::EndCombo();
        }
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Simulation speed (slow motion)");
    ImGui::Separator();

    // ---- menus
    if (ImGui::BeginMenu("View")) {
        ImGui::SeparatorText("Camera (C)");
        int m = (int)m_game.cam.mode;
        ImGui::RadioButton("Chase", &m, 1);
        ImGui::SameLine();
        ImGui::RadioButton("Orbit", &m, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Cockpit", &m, 3);
        ImGui::SameLine();
        ImGui::RadioButton("Free", &m, 2);
        if (m != (int)m_game.cam.mode) {
            if (m == CameraController::FREE) m_game.cam.enter_free();
            else m_game.cam.mode = (CameraController::Mode)m;
        }
        ImGui::SliderFloat("FOV", &m_game.cam.fov, 35, 100, "%.0f");
        ImGui::SeparatorText("Debug views");
        ImGui::MenuItem("Beams, elements, wheels", "F3", &m_game.debug.beams);
        ImGui::MenuItem("  colored by load (off: by deformation)", "F8", &m_game.debug.stress, m_game.debug.beams);
        ImGui::MenuItem("  forces, stresses, names (labels)", nullptr, &m_game.debug.labels, m_game.debug.beams || m_game.debug.wheels || m_game.debug.volumes);
        ImGui::MenuItem("Collision volumes", "F7", &m_game.debug.volumes);
        ImGui::MenuItem("Nodes & frames", nullptr, &m_game.debug.nodes);
        ImGui::MenuItem("Collision geometry", nullptr, &m_game.debug.collision);
        ImGui::MenuItem("Wheels", nullptr, &m_game.debug.wheels);
        ImGui::MenuItem("Bodies (awake/asleep)", nullptr, &m_game.debug.islands);
        ImGui::MenuItem("Hide meshes (skeleton only)", "F4", &m_game.debug.hide_meshes);
        ImGui::MenuItem("X-ray: plates and sheets see-through", "F6", &m_game.debug.xray);
        if (m_game.debug.xray) ImGui::SliderFloat("  opacity", &m_game.debug.xray_alpha, 0.02f, 0.8f, "%.2f");
        bool on_top = !m_renderer.debug_depth_test;
        if (ImGui::MenuItem("Debug lines on top", nullptr, &on_top)) m_renderer.debug_depth_test = !on_top;
        ImGui::SeparatorText("Rendering");
        ImGui::MenuItem("Shadows", nullptr, &m_game.light.shadows);
        ImGui::MenuItem("HUD", "H", &m_show_hud);
        ImGui::SliderFloat("Fog", &m_game.light.fog_density, 0.0f, 0.006f, "%.4f");
        ImGui::SliderFloat("Exposure", &m_game.light.exposure, 0.4f, 2.0f);
        float sun_az = std::atan2(m_game.light.sun_dir.z, m_game.light.sun_dir.x), sun_el = std::asin(clampf(m_game.light.sun_dir.y, -1, 1));
        bool ch = ImGui::SliderAngle("Sun azimuth", &sun_az, -180, 180);
        ch |= ImGui::SliderAngle("Sun elevation", &sun_el, 5, 89);
        if (ch) m_game.light.sun_dir = vec3(std::cos(sun_el) * std::cos(sun_az), std::sin(sun_el), std::cos(sun_el) * std::sin(sun_az));
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Tools")) {
        ImGui::SeparatorText("Left mouse button");
        int t = (int)m_game.tool;
        ImGui::RadioButton("Grab and pull nodes  (G)", &t, 0);
        ImGui::RadioButton("Destroy under cursor, hold  (X)", &t, 1);
        ImGui::RadioButton("Shoot projectiles  (B)", &t, 2);
        ImGui::RadioButton("Laser: cut along the cursor, hold  (L)", &t, 3);
        m_game.tool = (Tool)t;
        ImGui::SeparatorText("Grab");
        ImGui::SliderFloat("Strength", &m_game.grab_strength, 0.05f, 50.0f, "%.2gx", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("Grab radius", &m_game.grab_radius, 0.0f, 3.0f, "%.2f m");
        ImGui::SeparatorText("Destroy");
        ImGui::SliderFloat("Radius", &m_game.destroy_radius, 0.1f, 3.0f, "%.2f m");
        ImGui::Checkbox("Blast impulse", &m_game.destroy_blast);
        ImGui::SeparatorText("Shoot");
        if (ImGui::BeginCombo("Projectile (Z)", Game::projectile_name(m_game.projectile_kind))) {
            for (int i = 0; i < Game::kProjectileKinds; i++)
                if (ImGui::Selectable(Game::projectile_name(i), i == m_game.projectile_kind)) m_game.projectile_kind = i;
            ImGui::EndCombo();
        }
        ImGui::SliderFloat("Speed (, .)", &m_game.projectile_speed, 5, 150, "%.0f m/s");
        ImGui::SliderFloat("Fire rate", &m_game.fire_rate, 1, 20, "%.0f /s");
        if (ImGui::MenuItem("Remove projectiles", "Del")) m_game.clear_projectiles();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Editor")) {
        m_editor.menu();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Physics")) {
        auto& s = m_game.world.settings;
        ImGui::SeparatorText("World");
        float g = -s.gravity.y;
        if (ImGui::SliderFloat("Gravity", &g, 0.0f, 30.0f, "%.2f m/s2")) s.gravity.y = -g;
        float ws = length(s.wind);
        float wdir = std::atan2(s.wind.z, s.wind.x);
        bool wch = ImGui::SliderFloat("Wind", &ws, 0, 30, "%.1f m/s");
        wch |= ImGui::SliderAngle("Wind direction", &wdir, -180, 180);
        if (wch) {
            s.wind = vec3(std::cos(wdir), 0, std::sin(wdir)) * ws;
            m_game.world.wake_all();
        }
        ImGui::SliderFloat("Gusts", &s.wind_gusts, 0, 1.5f);
        ImGui::SeparatorText("Tyres");
        ImGui::SliderFloat("Tyre grip", &s.tyre_grip, 0.3f, 2.0f, "x%.2f");
        ImGui::Checkbox("Traction assist (vehicles without TC)", &Drivetrain::traction_assist);
        ImGui::SeparatorText("Steering");
        ImGui::SliderFloat("Steering speed", &Drivetrain::steer_speed, 0.5f, 5.0f, "x%.1f");
        ImGui::SetItemTooltip("How fast the wheels (and the steering wheel) turn in. x1 = Rigs of Rods: full lock in 0.33 s at a standstill.");
        ImGui::SliderFloat("Return to centre", &Drivetrain::steer_return, 0.5f, 5.0f, "x%.1f");
        ImGui::SetItemTooltip("How fast the wheels straighten when the key is released (and when counter-steering). x1 = Rigs of Rods.");
        ImGui::SliderFloat("Slower at speed", &Drivetrain::steer_speed_sens, 0.0f, 1.5f, "%.2f");
        ImGui::SetItemTooltip("How much the steering slows down with speed (1 = Rigs of Rods, 0 = the same speed at any velocity).");
        if (ImGui::SmallButton("Rigs of Rods")) Drivetrain::steer_speed = Drivetrain::steer_return = Drivetrain::steer_speed_sens = 1.0f;
        ImGui::SameLine();
        if (ImGui::SmallButton("Sharp")) {
            Drivetrain::steer_speed = Drivetrain::steer_return = 2.0f;
            Drivetrain::steer_speed_sens = 1.0f;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Very sharp")) {
            Drivetrain::steer_speed = Drivetrain::steer_return = 3.5f;
            Drivetrain::steer_speed_sens = 0.5f;
        }
        ImGui::SeparatorText("Solver");
        ImGui::Checkbox("Inter-body collisions", &s.inter_body_collisions);
        ImGui::Checkbox("Sleeping (islands at rest)", &s.sleeping);
        ImGui::Checkbox("Multithreading", &s.multithreaded);
        ImGui::SliderInt("FEM step every n substeps", &s.frame_every, 1, 4);
        ImGui::SliderFloat("Collision detection (Hz)", &s.collision_hz, 30.0f, 2000.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderInt("Max substeps/frame", &s.max_substeps_per_frame, 10, 200);
        ImGui::TextDisabled("Fixed step: %.1f kHz", 1.0f / s.dt / 1000.0f);
        ImGui::SeparatorText("Sheets (triangle elements)");
        {
            // overrides of the materials, for experiments: off = every sheet keeps its material's value
            bool ov_level = s.sheet_max_level >= 0, ov_edge = s.sheet_min_edge >= 0, ov_piece = s.sheet_min_piece >= 0;
            if (ImGui::Checkbox("##ovl", &ov_level)) s.sheet_max_level = ov_level ? 4 : -1;
            ImGui::SameLine();
            ImGui::BeginDisabled(!ov_level);
            int lvl = std::max(0, s.sheet_max_level);
            if (ImGui::SliderInt("Refinement depth", &lvl, 0, 6, "%d levels")) s.sheet_max_level = lvl;
            ImGui::EndDisabled();
            ImGui::SetItemTooltip("Bisections of an authored triangle before a crack opens instead (4: edges a quarter, area 1/16; the materials' own value). Each 2 levels double the steps a sheet takes.");
            if (ImGui::Checkbox("##ove", &ov_edge)) s.sheet_min_edge = ov_edge ? 0.02f : -1.0f;
            ImGui::SameLine();
            ImGui::BeginDisabled(!ov_edge);
            float cm = std::max(0.0f, s.sheet_min_edge) * 100.0f;
            if (ImGui::SliderFloat("Min triangle edge", &cm, 0.5f, 20.0f, "%.1f cm")) s.sheet_min_edge = cm / 100.0f;
            ImGui::EndDisabled();
            ImGui::SetItemTooltip("No triangle edge shorter than this: the refinement stops earlier on a fine sheet (the materials: 2 cm).");
            if (ImGui::Checkbox("##ovp", &ov_piece)) s.sheet_min_piece = ov_piece ? 10 : -1;
            ImGui::SameLine();
            ImGui::BeginDisabled(!ov_piece);
            int mp = std::max(0, s.sheet_min_piece);
            if (ImGui::SliderInt("Min piece", &mp, 1, 60, "%d triangles")) s.sheet_min_piece = mp;
            ImGui::EndDisabled();
            ImGui::SetItemTooltip("No crack may cut off a piece of fewer finest triangles (the materials: 10 brittle, 20 ductile): fewer, bigger pieces.");
            ImGui::SliderInt("Refine quota / frame", &s.refine_per_frame, 0, 2000, s.refine_per_frame ? "%d splits" : "no limit");
            ImGui::SetItemTooltip("Bisections per sheet per frame from the kernel's overloads; the rest queue again next substep (an impact's burst spread over frames).");
            bool coarsen = s.coarsen_per_frame > 0;
            if (ImGui::Checkbox("Coarsening (merge settled triangles back)", &coarsen)) s.coarsen_per_frame = coarsen ? 4000 : 0;
            ImGui::BeginDisabled(!coarsen);
            ImGui::SliderFloat("  after quiet for", &s.coarsen_delay, 0.05f, 3.0f, "%.2f s");
            ImGui::SetItemTooltip("Time since the sheet's last refinement or crack before its settled triangles are merged.");
            ImGui::SliderFloat("  strain below", &s.coarsen_quiet, 0.05f, 1.0f, "%.2f of refine");
            ImGui::SetItemTooltip("A triangle is merged only while its strain is below this fraction of its refine threshold (a loaded zone keeps its detail).");
            int merges = s.coarsen_per_frame;
            if (ImGui::SliderInt("  merges / frame", &merges, 100, 20000)) s.coarsen_per_frame = merges;
            ImGui::EndDisabled();
            ImGui::Checkbox("Rigid pieces (a shard moves as one body)", &s.rigid_pieces);
            ImGui::SetItemTooltip("A piece cracked off becomes a rigid body: no bending, no further cracks, its contacts only (fabric, rubber and cardboard stay soft).");
            ImGui::Checkbox("Fracture patterns (glass web, metal ring, wood grain)", &s.fracture_patterns);
        }
        if (Vehicle* v = m_game.player_vehicle()) {
            ImGui::SeparatorText("Player vehicle");
            ImGui::Checkbox("Deformation", &v->body->allow_deform);
            ImGui::Checkbox("Breaking", &v->body->allow_break);
            ImGui::SliderFloat("Collision range", &v->body->collision_radius, 0.01f, 0.15f, "%.2f m");
        }
        if (ImGui::MenuItem("Wake all bodies")) m_game.world.wake_all();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Scene")) {
        const auto& sc = scene_registry()[std::max(0, m_game.scene_index)];
        ImGui::SeparatorText(sc.name.c_str());
        if (ImGui::MenuItem("Reload scene", "F5")) select_scene(m_game.scene_index);
        // (a label "Group/Item": the items of a group under a submenu of its name)
        for (size_t i = 0; i < m_game.scene_actions.size();) {
            const SceneAction& a = m_game.scene_actions[i];
            const size_t slash = a.label.find('/');
            if (slash == std::string::npos) {
                if (ImGui::MenuItem(a.label.c_str())) a.run(m_game);
                i++;
                continue;
            }
            const std::string group = a.label.substr(0, slash);
            size_t j = i;
            while (j < m_game.scene_actions.size() && m_game.scene_actions[j].label.compare(0, slash + 1, group + "/") == 0) j++;
            if (ImGui::BeginMenu(group.c_str())) {
                for (size_t k = i; k < j; k++)
                    if (ImGui::MenuItem(m_game.scene_actions[k].label.c_str() + slash + 1)) m_game.scene_actions[k].run(m_game);
                ImGui::EndMenu();
            }
            i = j;
        }
        if (sc.id == "vehicle_crash") {
            ImGui::SeparatorText("Crash setup");
            auto vehicle_combo = [&](const char* label, std::string& id) {
                const VehicleEntry* e = find_vehicle(id);
                ImGui::SetNextItemWidth(260);
                if (ImGui::BeginCombo(label, e ? e->title.c_str() : "-", ImGuiComboFlags_HeightLarge)) {
                    std::string last;
                    for (const auto& v : vehicle_registry()) {
                        if (!v.drivable) continue;
                        if (v.group != last) {
                            ImGui::SeparatorText(v.group.c_str());
                            last = v.group;
                        }
                        ImGui::PushID(v.id.c_str());
                        if (ImGui::Selectable(v.title.c_str(), v.id == id)) id = v.id;
                        ImGui::PopID();
                    }
                    ImGui::EndCombo();
                }
            };
            vehicle_combo("Vehicle A", m_game.crash.vehicle_a);
            ImGui::SetNextItemWidth(260);
            ImGui::SliderFloat("Speed A", &m_game.crash.speed_a, 0, 160, "%.0f km/h");
            vehicle_combo("Vehicle B", m_game.crash.vehicle_b);
            ImGui::SetNextItemWidth(260);
            ImGui::SliderFloat("Speed B", &m_game.crash.speed_b, 0, 160, "%.0f km/h");
            ImGui::SetNextItemWidth(260);
            if (ImGui::BeginCombo("Layout", CrashConfig::layout_name(m_game.crash.layout))) {
                for (int i = 0; i < 5; i++)
                    if (ImGui::Selectable(CrashConfig::layout_name(i), i == m_game.crash.layout)) m_game.crash.layout = i;
                ImGui::EndCombo();
            }
            ImGui::Checkbox("Slow motion at impact", &m_game.crash.slow_motion);
            if (ImGui::Button("Run crash  (F5)", ImVec2(260, 0))) select_scene(m_game.scene_index);
        }
        vec3 f = m_game.camera_focus() + vec3(0, 6, 0);
        ImGui::SeparatorText("Drop objects (at camera focus)");
        if (ImGui::MenuItem("Crate")) m_game.drop_primitive(0, f);
        if (ImGui::MenuItem("Jelly ball")) m_game.drop_primitive(1, f);
        if (ImGui::MenuItem("Metal block (plastic)")) m_game.drop_primitive(2, f);
        if (ImGui::MenuItem("Wooden plank (breakable)")) m_game.drop_primitive(3, f);
        if (ImGui::MenuItem("Heavy ball")) m_game.drop_primitive(4, f);
        if (ImGui::MenuItem("Rain of 30 crates")) {
            Rng rng((uint64_t)(m_game.world.time() * 1000) + 3);
            for (int i = 0; i < 30; i++) m_game.drop_primitive(i % 5 == 4 ? 1 : 0, f + vec3(rng.range(-5, 5), 4 + i * 0.9f, rng.range(-5, 5)));
        }
        ImGui::SeparatorText("Vehicles");
        if (ImGui::MenuItem("Add AI vehicle (selected model)")) {
            vec3 p = m_game.camera_focus() + vec3(8, 0, 8);
            p.y = m_game.world.statics.has_terrain ? m_game.world.statics.terrain.height(p.x, p.z) : 0;
            if (Vehicle* v = m_game.spawn_vehicle(m_game.selected_vehicle, p, 0, false)) v->ai = true;
            if (!m_game.scene_update) m_game.scene_update = [](Game& gg, float dt) { ai_update_all(gg, dt); };
        }
        if (ImGui::MenuItem("Remove AI vehicles")) {
            for (int i = (int)m_game.vehicles.size() - 1; i >= 0; i--)
                if (!m_game.vehicles[i]->is_player) m_game.remove_vehicle(m_game.vehicles[i].get());
        }
        if (Vehicle* v = m_game.player_vehicle()) {
            if (ImGui::MenuItem("Launch at 50 km/h")) v->body->set_velocity(v->forward() * (50 / 3.6f));
            if (ImGui::MenuItem("Launch at 80 km/h")) v->body->set_velocity(v->forward() * (80 / 3.6f));
            if (ImGui::MenuItem("Drop from 15 m")) {
                v->body->translate(vec3(0, 15, 0));
                v->body->set_velocity(vec3(0));
            }
        }
        if (sc.id == "canyon" || sc.id == "stress_bridge") {
            if (ImGui::MenuItem("Send heavy truck over the truss bridge")) {
                for (auto& e : vehicle_registry())
                    if (e.folder == "tatra_815_6x6") {
                        if (Vehicle* v = m_game.spawn_vehicle(e.id, vec3(-60, 0, 0), 90, false)) {
                            v->ai = true;
                            ai_set_route(*v, {vec3(200, 0, 0)}, 12.0f);
                        }
                        break;
                    }
            }
            if (ImGui::MenuItem("Send bus over the wooden bridge")) {
                for (auto& e : vehicle_registry())
                    if (e.folder == "thomas_hdx_bus") {
                        if (Vehicle* v = m_game.spawn_vehicle(e.id, vec3(-60, 0, 45), 90, false)) {
                            v->ai = true;
                            ai_set_route(*v, {vec3(200, 0, 45)}, 8.0f);
                        }
                        break;
                    }
            }
        }
        ImGui::EndMenu();
    }

    // ---- right side
    float right = ImGui::GetWindowWidth();
    const char* perf_label = m_show_perf ? "Perf ON" : "Perf";
    float w = ImGui::CalcTextSize(perf_label).x + ImGui::CalcTextSize("Help").x + 48;
    ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX() + 8.0f, right - w)); // never overlap the menus on narrow windows
    if (ImGui::SmallButton("Help")) m_show_help = !m_show_help;
    ImGui::PushStyleColor(ImGuiCol_Button, m_show_perf ? ImVec4(0.96f, 0.62f, 0.16f, 0.6f) : ImGui::GetStyle().Colors[ImGuiCol_Button]);
    if (ImGui::SmallButton(perf_label)) m_show_perf = !m_show_perf;
    ImGui::PopStyleColor();
    ImGui::EndMainMenuBar();
}

void App::ui_tools() {
    // compact tool bar under the menu bar (top-left)
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 12, vp->WorkPos.y + 10), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.6f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6, 6));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4, 4));
    ImGui::Begin("##tools", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                                         ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove);
    ImGui::PushFont(m_font_small, 0.0f);
    const char* names[] = {"Grab  G", "Destroy  X", "Shoot  B", "Laser  L"};
    for (int i = 0; i < 4; i++) {
        bool active = (int)m_game.tool == i;
        ImGui::PushStyleColor(ImGuiCol_Button, active ? ImVec4(0.96f, 0.62f, 0.16f, 0.85f) : ImVec4(0.14f, 0.15f, 0.18f, 0.9f));
        if (i) ImGui::SameLine();
        if (ImGui::Button(names[i], ImVec2(82, 0))) m_game.tool = (Tool)i;
        ImGui::PopStyleColor();
    }
    ImGui::PushItemWidth(254);
    if (m_game.tool == Tool::Grab) {
        ImGui::SliderFloat("##gstr", &m_game.grab_strength, 0.05f, 50.0f, "strength %.2gx  (hold LMB on a node)", ImGuiSliderFlags_Logarithmic);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("How hard the grab pulls (1x: from the body's mass); it changes at once, also while pulling");
        ImGui::SliderFloat("##grad", &m_game.grab_radius, 0.0f, 3.0f, "radius %.2f m  (0: one node)");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("A sphere round the picked node: every body's nodes in it are pulled together, keeping their places, the pull falling off to its edge");
    } else if (m_game.tool == Tool::Destroy) {
        ImGui::SliderFloat("##rad", &m_game.destroy_radius, 0.1f, 3.0f, "radius %.2f m  (hold LMB)");
        ImGui::Checkbox("blast", &m_game.destroy_blast);
        ImGui::SameLine();
        ImGui::TextDisabled("broken: %d", m_game.destroyed_total);
    } else if (m_game.tool == Tool::Shoot) {
        if (ImGui::BeginCombo("##proj", Game::projectile_name(m_game.projectile_kind))) {
            for (int i = 0; i < Game::kProjectileKinds; i++)
                if (ImGui::Selectable(Game::projectile_name(i), i == m_game.projectile_kind)) m_game.projectile_kind = i;
            ImGui::EndCombo();
        }
        ImGui::SliderFloat("##spd", &m_game.projectile_speed, 5, 150, "speed %.0f m/s  (, .)");
        ImGui::SliderFloat("##rate", &m_game.fire_rate, 1, 20, "rate %.0f shots/s");
    } else if (m_game.tool == Tool::Laser) {
        ImGui::SliderFloat("##range", &m_game.laser_range, 5, 300, "range %.0f m  (hold LMB, sweep)");
        ImGui::TextDisabled("cuts along the cursor's path, no blast: %d links cut", m_game.laser_total);
    }
    ImGui::PopItemWidth();
    ImGui::PopFont();
    ImGui::End();
    ImGui::PopStyleVar(2);
}

void App::ui_hud() {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    Vehicle* v = m_game.player_vehicle();
    ui_tools();
    // time scale indicator
    if (!m_game.paused && std::fabs(m_game.world.settings.time_scale - 1.0f) > 1e-3f) {
        ImGui::PushFont(m_font_big, 0.0f);
        char t[64];
        snprintf(t, sizeof(t), "%s %.2gx", m_game.world.settings.time_scale < 1 ? "SLOW MOTION" : "FAST", m_game.world.settings.time_scale);
        ImVec2 ts = ImGui::CalcTextSize(t);
        ImVec2 p(vp->WorkPos.x + (vp->WorkSize.x - ts.x) * 0.5f, vp->WorkPos.y + 40);
        dl->AddRectFilled(ImVec2(p.x - 12, p.y - 6), ImVec2(p.x + ts.x + 12, p.y + ts.y + 6), col32(0, 0, 0, 0.45f), 6);
        dl->AddText(p, col32(0.6f, 0.85f, 1.0f), t);
        ImGui::PopFont();
    }
    // world labels (sample names; the debug view's, close by only): behind the windows, faded with distance
    if (!m_game.labels.empty() || !m_game.debug_labels.empty()) {
        const Camera& cam = m_game.last_camera();
        ImDrawList* bg = ImGui::GetBackgroundDrawList();
        const ImVec2 ds = ImGui::GetIO().DisplaySize;
        // nearest first; a label hidden behind a nearer one is skipped
        struct Vis {
            float key;
            const WorldLabel* l;
            float a;
        };
        std::vector<Vis> vis;
        for (const WorldLabel& l : m_game.labels) {
            float dist = length(l.pos - cam.pos);
            if (dist <= 70.0f) vis.push_back({dist, &l, clampf((70.0f - dist) / 20.0f, 0, 1)});
        }
        for (const WorldLabel& l : m_game.debug_labels) { // (after the scene's; faded out past 10 m to 15)
            float dist = length(l.pos - cam.pos);
            if (dist <= 15.0f) vis.push_back({dist + 100.0f, &l, clampf((15.0f - dist) / 5.0f, 0, 1)});
        }
        std::sort(vis.begin(), vis.end(), [](const Vis& a, const Vis& b) { return a.key < b.key; });
        std::vector<ImVec4> drawn;
        for (const Vis& v : vis) {
            const WorldLabel& l = *v.l;
            const float a = v.a;
            vec4 c = cam.viewproj * vec4(l.pos, 1.0f);
            if (c.w <= 0.1f) continue;
            float sx = (c.x / c.w * 0.5f + 0.5f) * ds.x, sy = (0.5f - c.y / c.w * 0.5f) * ds.y;
            ImVec2 ts = ImGui::CalcTextSize(l.text.c_str());
            ImVec2 p(sx - ts.x * 0.5f, sy - ts.y * 0.5f);
            ImVec4 r(p.x - 6, p.y - 3, p.x + ts.x + 6, p.y + ts.y + 3);
            bool hidden = false;
            for (const ImVec4& o : drawn) hidden |= r.x < o.z && r.z > o.x && r.y < o.w && r.w > o.y;
            if (hidden) continue;
            drawn.push_back(r);
            bg->AddRectFilled(ImVec2(r.x, r.y), ImVec2(r.z, r.w), col32(0, 0, 0, 0.55f * a), 4);
            bg->AddText(p, col32(l.color.x, l.color.y, l.color.z, l.color.w * a), l.text.c_str());
        }
    }
    // the beam view's load colours: their legend (top right, under the performance button)
    if (m_game.debug.beams && m_game.debug.stress) {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImDrawList* dl = ImGui::GetBackgroundDrawList();
        struct Key {
            ImU32 lo, hi;
            const char* text;
        };
        const ImU32 idle = col32(0.52f, 0.55f, 0.60f);
        const Key keys[] = {{idle, col32(0.15f, 0.45f, 1.0f), "tension"},
                            {idle, col32(1.0f, 0.15f, 0.1f), "compression"},
                            {idle, col32(1.0f, 0.68f, 0.1f), "bending"},
                            {col32(1.0f, 0.85f, 0.15f), col32(1.0f, 0.12f, 0.08f), "plates, sheets: stress"}};
        const float lh = ImGui::GetTextLineHeight() + 4, bw = 46, w = 240;
        ImVec2 p(vp->WorkPos.x + vp->WorkSize.x - w - 12, vp->WorkPos.y + 64);
        dl->AddRectFilled(ImVec2(p.x - 8, p.y - 6), ImVec2(p.x + w, p.y + lh * 5 + 4), col32(0, 0, 0, 0.45f), 6);
        dl->AddText(p, col32(0.85f, 0.87f, 0.9f), "load: 0 ... its limit");
        for (int k = 0; k < 4; k++) {
            const ImVec2 q(p.x, p.y + lh * (k + 1));
            if (k == 3) { // (grey, yellow, red)
                dl->AddRectFilledMultiColor(q, ImVec2(q.x + bw * 0.5f, q.y + lh - 6), idle, keys[k].lo, keys[k].lo, idle);
                dl->AddRectFilledMultiColor(ImVec2(q.x + bw * 0.5f, q.y), ImVec2(q.x + bw, q.y + lh - 6), keys[k].lo, keys[k].hi, keys[k].hi, keys[k].lo);
            } else {
                dl->AddRectFilledMultiColor(q, ImVec2(q.x + bw, q.y + lh - 6), keys[k].lo, keys[k].hi, keys[k].hi, keys[k].lo);
            }
            dl->AddText(ImVec2(q.x + bw + 8, q.y - 2), col32(0.85f, 0.87f, 0.9f), keys[k].text);
        }
    }
    // scene banner (stage timer), below the time scale indicator
    if (!m_game.scene_banner.empty()) {
        ImGui::PushFont(m_font_big, 0.0f);
        ImVec2 ts = ImGui::CalcTextSize(m_game.scene_banner.c_str());
        ImVec2 p(vp->WorkPos.x + (vp->WorkSize.x - ts.x) * 0.5f, vp->WorkPos.y + 92);
        dl->AddRectFilled(ImVec2(p.x - 14, p.y - 6), ImVec2(p.x + ts.x + 14, p.y + ts.y + 6), col32(0, 0, 0, 0.5f), 6);
        dl->AddText(p, ImGui::ColorConvertFloat4ToU32(kAccent), m_game.scene_banner.c_str());
        ImGui::PopFont();
    }
    // perf toggle chip (HUD button), top-right under the menu bar
    {
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - 12, vp->WorkPos.y + 10), ImGuiCond_Always, ImVec2(1, 0));
        ImGui::SetNextWindowBgAlpha(0.0f);
        ImGui::Begin("##hudbtn", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                                              ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove);
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 12);
        ImGui::PushStyleColor(ImGuiCol_Button, m_show_perf ? ImVec4(0.96f, 0.62f, 0.16f, 0.85f) : ImVec4(0.08f, 0.09f, 0.11f, 0.75f));
        if (ImGui::Button(m_show_perf ? "  Performance  x " : "  Performance  ")) m_show_perf = !m_show_perf;
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
        ImGui::End();
    }
    // scene hint (bottom-left)
    if (!m_game.scene_hint.empty()) {
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 12, vp->WorkPos.y + vp->WorkSize.y - 12), ImGuiCond_Always, ImVec2(0, 1));
        ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(430, 400));
        ImGui::SetNextWindowBgAlpha(0.6f);
        ImGui::Begin("##hint", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                                            ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove);
        ImGui::PushFont(m_font_small, 0.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, kAccent);
        ImGui::TextUnformatted(scene_registry()[m_game.scene_index].name.c_str());
        ImGui::PopStyleColor();
        ImGui::PushTextWrapPos(410);
        ImGui::TextDisabled("%s", m_game.scene_hint.c_str());
        if (!m_game.scene_status.empty()) ImGui::TextColored(ImVec4(1, 0.8f, 0.5f, 1), "%s", m_game.scene_status.c_str());
        if (scene_registry()[m_game.scene_index].id == "vehicle_crash") {
            std::string rep = m_game.crash_report();
            if (!rep.empty()) ImGui::TextColored(ImVec4(1, 0.8f, 0.5f, 1), "%s", rep.c_str());
        }
        if (m_game.player_vehicle())
            ImGui::TextDisabled("WASD drive, Space handbrake, R reset, C/F camera, [ ] T time, G/X/B tools, F1 help");
        else
            ImGui::TextDisabled("Free camera: WASD / Q E fly, right mouse look, Shift fast; [ ] T time, G/X/B tools, F5 reload, F1 help");
        ImGui::PopTextWrapPos();
        ImGui::PopFont();
        ImGui::End();
    }
    if (m_game.paused) {
        ImGui::PushFont(m_font_big, 0.0f);
        const char* t = "PAUSED  (P to resume, N to step)";
        ImVec2 ts = ImGui::CalcTextSize(t);
        ImVec2 p(vp->WorkPos.x + (vp->WorkSize.x - ts.x) * 0.5f, vp->WorkPos.y + 40);
        dl->AddRectFilled(ImVec2(p.x - 12, p.y - 6), ImVec2(p.x + ts.x + 12, p.y + ts.y + 6), col32(0, 0, 0, 0.5f), 6);
        dl->AddText(p, ImGui::ColorConvertFloat4ToU32(kAccent), t);
        ImGui::PopFont();
    }
    if (!v || m_game.cam.mode == CameraController::FREE) return;
    // gauges (bottom-right)
    float r = 70;
    ImVec2 base(vp->WorkPos.x + vp->WorkSize.x - 30 - r, vp->WorkPos.y + vp->WorkSize.y - 30 - r);
    float kmh = std::fabs(v->speed_kmh());
    char sp[32];
    snprintf(sp, sizeof(sp), "%.0f", kmh);
    float vmax = std::max(60.0f, v->def().gui.speedo_max);
    draw_gauge(dl, base, r, kmh, vmax, "km/h", sp, m_font_big, m_font_small);
    if (v->has_engine()) {
        ImVec2 rc(base.x - 2 * r - 30, base.y + 10);
        char gear[16];
        int g = v->gear();
        snprintf(gear, sizeof(gear), "%s", g < 0 ? "R" : (g == 0 ? "N" : format("%d", g).c_str()));
        draw_gauge(dl, rc, r * 0.8f, v->rpm(), v->max_rpm() * 1.25f, "rpm", gear, m_font_big, m_font_small, v->max_rpm());
        ImVec2 bp(rc.x - r * 0.8f - 46, rc.y - 35);
        bar(dl, bp, ImVec2(14, 70), v->throttle(), col32(0.3f, 0.85f, 0.35f, 0.9f), "thr", m_font_small);
        bar(dl, ImVec2(bp.x + 20, bp.y), ImVec2(14, 70), v->brake(), col32(0.95f, 0.3f, 0.2f, 0.9f), "brk", m_font_small);
    }
    // damage indicator
    int broken = v->broken_beams();
    if (broken > 0) {
        char t[64];
        snprintf(t, sizeof(t), "damage: %d beams", broken);
        ImGui::PushFont(m_font_small, 0.0f);
        ImVec2 ts = ImGui::CalcTextSize(t);
        dl->AddText(ImVec2(base.x - ts.x * 0.5f, base.y - r - 22), col32(1, 0.45f, 0.3f, 0.9f), t);
        ImGui::PopFont();
    }
}

void App::ui_perf() {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - 12, vp->WorkPos.y + 46), ImGuiCond_Always, ImVec2(1, 0));
    ImGui::SetNextWindowBgAlpha(0.82f);
    // the profiler draws its labels to the right of the graph: reserve the width explicitly
    ImGui::SetNextWindowSizeConstraints(ImVec2(345, 0), ImVec2(345, 10000));
    ImGui::PushFont(m_font_small, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 6));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4, 2));
    if (ImGui::Begin("Performance", &m_show_perf, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                                                     ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav)) {
        m_perf.DrawGUI();
        // physics breakdown: CPU time summed over the worker threads (smoothed), grouped by what it is spent on
        ImGui::Separator();
        const auto& st = m_game.world.stats();
        auto zms = [](const char* n) {
            const prof::Zone* z = prof::find_zone(n);
            return z ? z->avg_ms : 0.0;
        };
        const prof::Zone* iz = prof::find_zone("Islands");
        // the key numbers: the physics wall time and its critical path, the CPU by what it is spent on
        ImGui::Text("Physics %.1f ms, heaviest island %.1f ms%s", iz ? iz->last_ms : 0.0, st.heavy_island_ms, st.heavy_island_wide ? " (team)" : "");
        ImGui::SameLine();
        ImGui::TextDisabled("%.0f ms CPU", st.islands_cpu_ms);
        struct Cat {
            const char* name;
            double ms;
            ImU32 col;
        };
        const Cat cats[] = {
            {"Beams", zms("Beam forces"), IM_COL32(245, 158, 40, 255)},
            {"Sheets", zms("Sheet elements") + zms("Sheet gather"), IM_COL32(250, 210, 70, 255)},
            {"Collisions", zms("Broadphase") + zms("Fast pair refresh") + zms("Narrow phase") + zms("Contacts") + zms("Static collisions") +
                               zms("Pair inheritance"),
             IM_COL32(200, 120, 230, 255)},
            {"Integrate", zms("Integrate"), IM_COL32(90, 170, 250, 255)},
            {"Cracks", zms("Sheet topology"), IM_COL32(240, 90, 80, 255)},
        };
        double total = 0;
        for (const Cat& c : cats) total += c.ms;
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 p = ImGui::GetCursorScreenPos();
        const float W = 325, H = 10;
        float x = p.x;
        for (const Cat& c : cats) {
            float w = total > 0 ? (float)(c.ms / total) * W : 0;
            dl->AddRectFilled(ImVec2(x, p.y), ImVec2(x + w, p.y + H), c.col);
            x += w;
        }
        ImGui::Dummy(ImVec2(W, H + 2));
        for (int i = 0; i < 5; i++) {
            ImGui::ColorButton(cats[i].name, ImGui::ColorConvertU32ToFloat4(cats[i].col), ImGuiColorEditFlags_NoTooltip, ImVec2(8, 8));
            ImGui::SameLine();
            ImGui::TextDisabled("%s", cats[i].name);
            if (i != 4) ImGui::SameLine();
        }
        if (ImGui::CollapsingHeader("Physics details")) {
            ImGui::TextDisabled("CPU ms per frame, all threads, smoothed");
            if (ImGui::BeginTable("physzones", 3, ImGuiTableFlags_SizingFixedFit)) {
                ImGui::TableSetupColumn("zone", ImGuiTableColumnFlags_WidthFixed, 118);
                ImGui::TableSetupColumn("ms", ImGuiTableColumnFlags_WidthFixed, 46);
                ImGui::TableSetupColumn("info", ImGuiTableColumnFlags_WidthStretch);
                auto row = [](const char* name, double ms, const std::string& info) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(name);
                    ImGui::TableNextColumn();
                    ImGui::Text("%6.2f", ms);
                    ImGui::TableNextColumn();
                    if (!info.empty()) ImGui::TextDisabled("%s", info.c_str());
                };
                row("Beam forces", zms("Beam forces"), format("%lld k beam evals", st.beam_steps / 1000));
                row("Sheet elements", zms("Sheet elements"), format("%lld k tri evals, %d awake", st.shell_steps / 1000, st.awake_shells));
                row("  gather", zms("Sheet gather"), "forces summed per node");
                row("Narrow phase", zms("Narrow phase"), format("%lld k pair tests", st.narrow_tests / 1000));
                row("Contacts", zms("Contacts"), "response, in order");
                row("Broadphase", zms("Broadphase"), format("%d rebuilds", st.pair_rebuilds));
                row("  fast pairs", zms("Fast pairs"), format("%d refreshes", st.fast_refreshes));
                row("  inherited", zms("Pair inheritance"), "after cracks");
                row("Static colls", zms("Static collisions"), "");
                row("Integrate", zms("Integrate"), "");
                row("Sheet refine", zms("Sheet refine"), format("%d splits", st.shell_refines));
                row("Sheet cracks", zms("Sheet cracks"), format("%d cracks", st.shell_cracks));
                row("Sheet detach", zms("Sheet detach"), format("%d new pieces", st.pieces_created));
                row("Sheet mesh", zms("Sheet mesh"), format("rebuild %.2f, upload %.2f", zms("Sheet mesh rebuild"), zms("Sheet upload")));
                ImGui::EndTable();
            }
            ImGui::TextDisabled("Heaviest island %.1f ms%s: %d bodies, %d nodes", st.heavy_island_ms, st.heavy_island_wide ? " (team)" : "",
                                st.heavy_island_bodies, st.heavy_island_nodes);
            ImGui::TextDisabled("  phases ms: forces %.2f, gather %.2f, collide %.2f", st.heavy_phase_ms[0], st.heavy_phase_ms[1], st.heavy_phase_ms[2]);
            ImGui::TextDisabled("  integrate %.2f, serial %.2f, cracks %.2f", st.heavy_phase_ms[3], st.heavy_phase_ms[4], st.heavy_phase_ms[5]);
            ImGui::TextDisabled("  %d beams, %d triangles x%d steps", st.heavy_island_beams, st.heavy_island_shells, st.heavy_island_sub);
            ImGui::TextDisabled("  most expensive: %.30s", st.heavy_island_body.c_str());
            ImGui::TextDisabled("  team: waited %.2f ms for helpers (%d > 0.2 ms)", st.team_wait_ms, st.team_stalls);
            ImGui::Separator();
            static bool cache_live = false;
            static double cache_t = -1;
            if (ImGui::SmallButton("Measure cache")) measure_cache();
            ImGui::SameLine();
            ImGui::Checkbox("every 2 s", &cache_live);
            if (cache_live && ImGui::GetTime() - cache_t > 2.0) {
                cache_t = ImGui::GetTime();
                measure_cache();
            }
            if (!m_cache_body.empty() && ImGui::BeginTable("cache", 5, ImGuiTableFlags_SizingFixedFit)) {
                ImGui::TableSetupColumn("layout", ImGuiTableColumnFlags_WidthFixed, 70);
                ImGui::TableSetupColumn("L1 hits", ImGuiTableColumnFlags_WidthFixed, 52);
                ImGui::TableSetupColumn("from L2", ImGuiTableColumnFlags_WidthFixed, 64);
                ImGui::TableSetupColumn("line use", ImGuiTableColumnFlags_WidthFixed, 56);
                ImGui::TableSetupColumn("set", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableHeadersRow();
                auto crow = [](const char* name, const CacheRow& c) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(name);
                    ImGui::TableNextColumn();
                    ImGui::Text("%.1f%%", c.l1);
                    ImGui::TableNextColumn();
                    ImGui::Text("%.0f B/tri", c.l2_bytes);
                    ImGui::TableNextColumn();
                    ImGui::Text("%.0f%%", c.util);
                    ImGui::TableNextColumn();
                    ImGui::Text("%.2f MB", c.ws / 1048576.0);
                };
                crow("kernel", m_cache_cur);
                crow("old layout", m_cache_ref);
                ImGui::EndTable();
                ImGui::TextDisabled("  %.24s, %zu tris; model L1 128K, L2 16M", m_cache_body.c_str(), m_cache_tris);
            }
        }
        }
        ImGui::Separator();
        ui_elements();
        // cache behaviour of the sheet force kernel (software model of the caches: the hardware counters need root)
    ImGui::End();
    ImGui::PopStyleVar(2);
    ImGui::PopFont();
}

// Elements of the world: how many triangles, links (between triangles, beams), nodes and bodies there are, how many
// are awake (the rest sleep: no cost), the work each kind took this frame; graphs of the last 600 frames on one time
// axis (the mouse over a graph shows every series at that frame in all of them).
void App::ui_elements() {
    const auto& st = m_game.world.stats();
    const auto& e = st.el;
    ImGui::SeparatorText("Elements");
    ImGui::Checkbox("freeze graphs", &m_elem_freeze);
    ImGui::SameLine();
    static const char* env = getenv("BL_PROFCSV");
    if (env) {
        ImGui::TextDisabled("CSV: %.28s", env);
    } else if (m_rec_path.empty()) {
        if (ImGui::SmallButton("Record CSV")) {
            char name[64];
            const std::time_t now = std::time(nullptr);
            std::strftime(name, sizeof name, "beamlab_stats_%Y%m%d_%H%M%S.csv", std::localtime(&now));
            char cwd[1024];
            std::string dir = getcwd(cwd, sizeof cwd) ? cwd : "";
            if (dir.empty() || dir == "/" || access(dir.c_str(), W_OK) != 0) dir = getenv("HOME") ? getenv("HOME") : ".";
            m_rec_path = dir + "/" + name;
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("One line per frame: the physics zones, the heaviest island and all these counts");
    } else {
        if (ImGui::SmallButton("Stop CSV")) {
            if (m_prof_csv) fclose(m_prof_csv);
            m_prof_csv = nullptr;
            m_rec_path.clear();
        } else {
            ImGui::SameLine();
            ImGui::TextDisabled("rec");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", m_rec_path.c_str());
        }
    }
    auto num = [](double v) {
        char b[32];
        if (v >= 1e7) snprintf(b, sizeof b, "%.0fM", v / 1e6);
        else if (v >= 1e6) snprintf(b, sizeof b, "%.1fM", v / 1e6);
        else if (v >= 1e4) snprintf(b, sizeof b, "%.0fk", v / 1e3);
        else snprintf(b, sizeof b, "%.0f", v);
        return std::string(b);
    };
    if (ImGui::BeginTable("elements", 5, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 74);
        ImGui::TableSetupColumn("total", ImGuiTableColumnFlags_WidthFixed, 50);
        ImGui::TableSetupColumn("awake", ImGuiTableColumnFlags_WidthFixed, 50);
        ImGui::TableSetupColumn("asleep", ImGuiTableColumnFlags_WidthFixed, 50);
        ImGui::TableSetupColumn("/ frame", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        auto row = [&](const char* name, const char* tip, long long total, long long awake, long long per_frame) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(name);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(num((double)total).c_str());
            ImGui::TableNextColumn();
            if (awake >= 0) ImGui::TextUnformatted(num((double)awake).c_str());
            ImGui::TableNextColumn();
            if (awake >= 0) ImGui::TextDisabled("%s", num((double)(total - awake)).c_str());
            ImGui::TableNextColumn();
            if (per_frame >= 0) ImGui::TextUnformatted(num((double)per_frame).c_str());
        };
        row("bodies", "rigid parts of the scene are no bodies", e.bodies, e.bodies_awake, -1);
        row("  pieces", "cracked off the sheets: bodies of their own (rigid: moved as one, no springs)", e.pieces, e.pieces_awake, -1);
        row("  rigid", "pieces that are rigid bodies", e.pieces_rigid, -1, -1);
        row("nodes", "/ frame: node integrations (awake nodes x short steps x substeps)", e.nodes, e.nodes_awake, e.node_steps);
        row("triangles", "sheet elements; / frame: evaluated by the force kernel (the coarse ones of a refined sheet less often)",
            e.shells, e.shells_awake, e.shell_evals);
        row("tri links", "shared edges between triangles (a bending hinge each; the springs are the triangles' own edges); / frame: hinges "
                         "evaluated",
            e.hinges, e.hinges_awake, e.hinge_evals);
        row("beams", "node-beam bodies (vehicles, trees, crates); / frame: beam evaluations", e.beams, e.beams_awake, e.beam_evals);
        row("coll. tris", "collision triangles (the sheets' triangles are theirs too)", e.tris, e.tris_awake, -1);
        ImGui::EndTable();
    }
    if (ImGui::CollapsingHeader("Element details")) {
        ImGui::TextDisabled("awake triangles by level  0: %s  1: %s  2: %s  3: %s  4: %s", num(e.shells_level[0]).c_str(), num(e.shells_level[1]).c_str(),
                            num(e.shells_level[2]).c_str(), num(e.shells_level[3]).c_str(), num(e.shells_level[4]).c_str());
        ImGui::TextDisabled("  in bodies stepping x1: %s  x2: %s  x4 per substep: %s", num(e.shells_rate[0]).c_str(), num(e.shells_rate[1]).c_str(),
                            num(e.shells_rate[2]).c_str());
        ImGui::TextDisabled("sheet edges: border %s, cracks %s, laser %s; patterns %d", num(e.edges_border).c_str(), num(e.edges_crack).c_str(),
                            num(e.edges_cut).c_str(), e.impacts);
        ImGui::TextDisabled("contacts %s (last substep), candidate pairs %s", num(st.contacts).c_str(), num(st.contact_pairs).c_str());
        ImGui::TextDisabled("  pair tests %s / frame", num((double)st.narrow_tests).c_str());
        ImGui::TextDisabled("frame: %d substeps, %d splits, %d cracks, %d pieces", st.substeps, st.shell_refines, st.shell_cracks, st.pieces_created);
        ImGui::TextDisabled("  beams broken: %s", num(e.beams_broken).c_str());
    }
    // graphs: the series of one graph share its scale; the oldest frame at the left
    const int n = m_elem_count, N = (int)m_elem_hist.size();
    if (n < 2 || N == 0) return;
    struct Series {
        const char* name;
        ImU32 col;
        int idx;
    };
    int new_hover = -1;
    auto sample = [&](int i) -> const ElemSample& { return m_elem_hist[(m_elem_head - n + i + N) % N]; };
    auto graph = [&](const char* title, const Series* ser, int ns, bool ms) {
        float mx = 1e-6f;
        for (int i = 0; i < n; i++)
            for (int k = 0; k < ns; k++) mx = std::max(mx, sample(i).v[ser[k].idx]);
        const int at = m_elem_hover >= 0 && m_elem_hover < n ? m_elem_hover : n - 1;
        ImGui::TextDisabled("%s  (top %s%s)%s", title, ms ? format("%.1f", mx).c_str() : num(mx).c_str(), ms ? " ms" : "",
                            m_elem_hover >= 0 ? format(",  %d frames ago", n - 1 - at).c_str() : "");
        const float W = ImGui::GetContentRegionAvail().x, H = 46;
        const ImVec2 p = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton(title, ImVec2(W, H));
        if (ImGui::IsItemHovered()) new_hover = std::clamp((int)((ImGui::GetIO().MousePos.x - p.x) / W * (n - 1) + 0.5f), 0, n - 1);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p, ImVec2(p.x + W, p.y + H), IM_COL32(20, 22, 26, 200));
        for (int k = 0; k < ns; k++) {
            ImVec2 prev;
            for (int i = 0; i < n; i++) {
                const ImVec2 q(p.x + W * (float)i / (float)(n - 1), p.y + H - 1 - (H - 2) * std::min(1.0f, sample(i).v[ser[k].idx] / mx));
                if (i) dl->AddLine(prev, q, ser[k].col, 1.2f);
                prev = q;
            }
        }
        if (m_elem_hover >= 0 && m_elem_hover < n) {
            const float x = p.x + W * (float)m_elem_hover / (float)(n - 1);
            dl->AddLine(ImVec2(x, p.y), ImVec2(x, p.y + H), IM_COL32(255, 255, 255, 110));
        }
        for (int k = 0; k < ns; k++) {
            if (k) ImGui::SameLine(0, 8);
            ImGui::ColorButton(ser[k].name, ImGui::ColorConvertU32ToFloat4(ser[k].col), ImGuiColorEditFlags_NoTooltip, ImVec2(7, 7));
            ImGui::SameLine(0, 3);
            const float v = sample(at).v[ser[k].idx];
            ImGui::TextDisabled("%s %s", ser[k].name, ms ? format("%.1f", v).c_str() : num(v).c_str());
        }
    };
    const ImU32 yel = IM_COL32(250, 210, 70, 255), ora = IM_COL32(245, 140, 40, 255), blu = IM_COL32(90, 170, 250, 255),
                gry = IM_COL32(150, 150, 150, 255), pur = IM_COL32(200, 120, 230, 255), red = IM_COL32(240, 90, 80, 255),
                wht = IM_COL32(235, 235, 235, 255), brn = IM_COL32(170, 110, 60, 255);
    const Series g1[] = {{"all", gry, 0}, {"awake", yel, 1}, {"fine", ora, 2}, {"nodes", blu, 14}};
    const Series g2[] = {{"tri links", yel, 3}, {"beams", ora, 4}, {"contacts", pur, 5}};
    const Series g3[] = {{"tri", yel, 6}, {"nodes", blu, 7}, {"hinges", ora, 8}, {"beams", brn, 9}};
    const Series g4[] = {{"phys", wht, 10}, {"sheets", yel, 11}, {"coll", pur, 12}, {"topo", red, 13}};
    graph("triangles (awake: fine = level 3+), awake nodes", g1, 4, false);
    graph("links, awake", g2, 3, false);
    graph("work per frame", g3, 4, false);
    graph("ms: physics wall, CPU of parts", g4, 4, true);
    m_elem_hover = new_hover;
}

void App::ui_help() {
    ImGui::SetNextWindowSize(ImVec2(520, 0), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::Begin("Help", &m_show_help, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse)) {
        auto row = [](const char* k, const char* d) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextColored(kAccent, "%s", k);
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(d);
        };
        if (ImGui::BeginTable("keys", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            row("W / S", "Throttle / brake (hold S when stopped to reverse)");
            row("A / D", "Steer");
            row("Space", "Handbrake");
            row("1 .. 0", "Vehicle commands (cranes, doors, tippers: odd/even key pairs)");
            row("R / Shift+R", "Recover vehicle / back to spawn");
            row("C", "Camera: chase, orbit, cockpit, free");
            row("RMB drag, wheel", "Look around, zoom");
            row("LMB drag", "Grab and pull a node (any body)");
            row("Free camera", "WASD move, Q/E down/up, Shift faster");
            row("P / N", "Pause / single step");
            row("[ / ]  T  Backspace", "Slower / faster time, slow motion toggle, normal speed");
            row("F / Tab", "Free camera on/off / control the next vehicle");
            row("G / X / B / L", "Tool: grab nodes / destroy under cursor (hold LMB) / shoot (LMB) / laser: cut along the cursor (hold LMB)");
            row("Z  , .  Del", "Projectile type, projectile speed -/+, remove projectiles");
            row("F5", "Reload scene / rerun the crash test");
            row("F1 / F2", "Help / performance widget");
            row("F3 / F4 / F6", "Beam view (with the ring tyres, the loaded elements' forces and stresses) / hide meshes / x-ray: plates and sheets see-through");
            row("F7 / F8", "Collision volumes / the beam view coloured by the elements' loads, or by their deformation");
            row("H", "Toggle HUD");
            row("Gamepad", "Left stick steer, triggers throttle/brake, A handbrake");
            ImGui::EndTable();
        }
        ImGui::SeparatorText("About");
        ImGui::PushTextWrapPos(500);
        ImGui::TextDisabled(
            "Rigs of Rods style node/beam soft-body physics at 2 kHz: plastic deformation, breaking, soft tyres, "
            "drivetrain, node-triangle collisions between bodies, orientation preserving joints (trees). "
            "Vehicles are original Rigs of Rods mods loaded from assets/vehicles.");
        ImGui::PopTextWrapPos();
    }
    ImGui::End();
}

void App::ui_vehicle_info() {
    Vehicle* v = m_game.player_vehicle();
    ImGui::SetNextWindowSize(ImVec2(380, 0), ImGuiCond_Appearing);
    if (ImGui::Begin("Vehicle info", &m_show_vehicle_info, ImGuiWindowFlags_NoSavedSettings)) {
        if (!v) {
            ImGui::TextDisabled("No vehicle");
        } else {
            const auto& d = v->def();
            ImGui::TextColored(kAccent, "%s", v->name.c_str());
            for (auto& a : d.authors) ImGui::TextDisabled("%s", a.c_str());
            ImGui::Separator();
            if (ImGui::BeginTable("vi", 2)) {
                auto row = [](const char* k, const std::string& val) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextDisabled("%s", k);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(val.c_str());
                };
                row("Mass", format("%.0f kg", v->mass()));
                row("Nodes / beams", format("%d / %d", v->body->node_count(), (int)v->body->beams.size()));
                row("Shocks", format("%d", (int)v->body->shocks.size()));
                row("Wheels", format("%d", (int)v->body->wheels.size()));
                row("Collision tris", format("%d", (int)v->body->tris.size()));
                if (d.engine.present) row("Engine", format("%.0f N m, %d gears", d.engine.torque, (int)d.engine.gears.size()));
                row("Flexbodies / props", format("%d / %d", (int)d.flexbodies.size(), (int)d.props.size()));
                row("Broken beams", format("%d", v->broken_beams()));
                ImGui::EndTable();
            }
            if (!v->load_warnings().empty() && ImGui::CollapsingHeader(format("Load warnings (%d)", (int)v->load_warnings().size()).c_str())) {
                ImGui::BeginChild("warn", ImVec2(0, 150));
                for (auto& w : v->load_warnings()) ImGui::TextDisabled("%s", w.c_str());
                ImGui::EndChild();
            }
        }
    }
    ImGui::End();
}

void App::ui_log() {
    ImGui::SetNextWindowSize(ImVec2(640, 300), ImGuiCond_Appearing);
    if (ImGui::Begin("Log", &m_show_log, ImGuiWindowFlags_NoSavedSettings)) {
        auto lines = log_recent(400);
        for (auto& l : lines) {
            ImVec4 c = l.level == 2 ? ImVec4(1, 0.4f, 0.35f, 1) : l.level == 1 ? ImVec4(1, 0.8f, 0.3f, 1) : ImVec4(0.8f, 0.82f, 0.86f, 1);
            ImGui::TextColored(c, "%s", l.text.c_str());
        }
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) ImGui::SetScrollHereY(1.0f);
    }
    ImGui::End();
}

void App::ui_toasts() {
    if (m_toast_time <= 0 || m_toast.empty()) return;
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float a = std::min(1.0f, m_toast_time);
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    ImVec2 ts = ImGui::CalcTextSize(m_toast.c_str());
    ImVec2 p(vp->WorkPos.x + (vp->WorkSize.x - ts.x) * 0.5f, vp->WorkPos.y + vp->WorkSize.y - 60);
    dl->AddRectFilled(ImVec2(p.x - 14, p.y - 8), ImVec2(p.x + ts.x + 14, p.y + ts.y + 8), col32(0.08f, 0.09f, 0.11f, 0.85f * a), 8);
    dl->AddText(p, col32(1, 1, 1, a), m_toast.c_str());
}

} // namespace bl
