// Application shell: window, main loop, input, UI.
#pragma once

#include "game/editor.h"
#include "game/game.h"
#include "gfx/renderer.h"

#include "PerfmonWidget.h"

#include <string>

struct GLFWwindow;
struct ImFont;

namespace bl {

struct AppOptions {
    std::string scene;          // scene id or index
    std::string vehicle;        // vehicle id
    int bench_frames = 0;       // >0: benchmark mode (fixed dt, prints stats, exits)
    std::string screenshot;     // path: render N frames then save a screenshot and exit
    int screenshot_frames = 120;
    std::string camera;         // screenshot camera preset: chase | side | front | top | orbit:<yaw>,<pitch>,<dist>
    int width = 1600, height = 900;    // (the window opens maximized unless --size is given)
    bool size_given = false;
    bool editor = false;        // open the model editor after the load (screenshots of it)
    bool editor_test = false;   // ... and start a test drive of its model (the scene test: save, registry, spawn, drive)
    bool hidden = false;
    bool no_vsync = false;
    int threads = -1;
    bool debug_beams = false;
    float time_scale = 1.0f;
    std::string spawn;          // override spawn point: "x,z,yaw_deg"
    bool show_perf = false;     // open the performance widget at start
    std::string autoshoot;      // test: "kind,speed,interval" fire from the camera at the screen center
    std::string crane;          // test: "lift[,release frame]" hang the player vehicle by the top of its frame (Game::crane_vehicle)
    std::string action;
    std::string record;         // video frames: "dir[,every[,first]]" writes dir/frame_00000.png ... (every n-th frame from `first`)
    bool no_ui = false;         // no UI drawn (clean screenshots and recordings)
    float launch_kmh = 0;       // test: launch the player vehicle forward at this speed after loading         // run the scene action whose label contains this text (after loading)
    float autodestroy = 0;      // test: destroy radius swept along the screen center ray every frame
    std::string laser;          // test: laser sweep "x0,y0,x1,y1[,first frame,frames]" across the screen (fractions, y down)
    std::string drive;          // autodrive test: "<throttle>,<steer>[,<brake>]" applied to the player vehicle, logs telemetry
    // Screenshot series: a text file, one shot per line: "<frame> <camera preset> <png path> [beams] [noui] [pause]"
    // (beams: triangle / beam wireframe, noui: no menus or labels, pause: physics stops from this frame on). The
    // program exits after the last shot.
    std::string shots;
    bool realtime = false;      // screenshots / benchmarks with the real frame time (the default steps a fixed 1/60 s)
};

class App {
public:
    int run(const AppOptions& opt);

private:
    bool init(const AppOptions& opt);
    void shutdown();
    void frame(float dt);
    void input_script(); // BL_INPUT_SCRIPT: scripted mouse and keys (headless UI checks)
    void gather_input(float dt, VehicleInput& vin, CameraInput& cin);
    void apply_camera_preset(const std::string& preset);
    // UI
    void setup_style();
    void ui_main_menu();
    void ui_hud();
    void ui_perf();
    // software cache model of the sheet force kernel (phys::measure_kernel_cache) on the biggest sheet
    bool measure_cache();
    float pace(float raw);          // frame time the simulation steps by (see app.cpp)
    double m_pace_avg = 0, m_pace_debt = 0;
    std::string m_cache_text;
    struct CacheRow {
        double l1 = 0, l2_bytes = 0, util = 0, touched = 0, ws = 0;
    } m_cache_cur, m_cache_ref;
    size_t m_cache_tris = 0;
    std::string m_cache_body;
    void ui_help();
    void ui_vehicle_info();
    void ui_log();
    void ui_toasts();
    void register_metrics();
    void toast(const std::string& text);
    void select_scene(int idx);
    void select_vehicle(const std::string& id, bool respawn);

    GLFWwindow* m_win = nullptr;
    Renderer m_renderer;
    Game m_game;
    ModelEditor m_editor{m_game}; // the model editor (Editor menu, Ctrl+E)
    Perfmon::PerfmonWidget m_perf;
    AppOptions m_opt;
    ImFont* m_font_small = nullptr;
    ImFont* m_font_big = nullptr;
    ImFont* m_font_huge = nullptr;

    bool m_show_perf = false;
    bool m_show_help = false;
    bool m_show_vehicle_info = false;
    bool m_show_log = false;
    bool m_show_demo = false;
    bool m_show_hud = true;
    int m_fb_w = 1, m_fb_h = 1;
    double m_last_time = 0;
    float m_fps = 60;
    double m_frame_ms = 0, m_cpu_ms = 0, m_render_ms = 0, m_ui_ms = 0;
    double m_prev_mx = 0, m_prev_my = 0;
    // Steering by the mouse as in Operation Flashpoint (M): the cursor is a heading in the world, the vehicle steers to
    // it; as it turns the cursor comes back to the screen's middle (gather_input, ui_mouse_steer).
    bool m_mouse_steer = false;    // (the mode: View menu, M)
    bool m_mouse_captured = false; // (the mouse taken now: the mode on, a vehicle driven, no menu)
    bool m_aim_valid = false;
    float m_aim_yaw = 0, m_aim_pitch = 0;    // (the cursor's heading in the world, its height over the horizon: radians)
    float m_mouse_steer_lock = 20.0f;        // (the angle off the cursor at which the wheels are at full lock, degrees)
    void ui_mouse_steer();
    bool m_rmb = false, m_lmb_grab = false, m_lmb_held = false;
    float m_fire_timer = 0;
    void ui_tools();
    float m_wheel = 0;
    float m_steer_state = 0;
    int m_frame_index = 0;
    struct Shot {
        int frame = 0;
        std::string camera, path;
        bool beams = false, noui = false, pause = false;
    };
    std::vector<Shot> m_shots;  // --shots
    bool m_hide_ui = false;
    FILE* m_prof_csv = nullptr; // BL_PROFCSV: one line per frame (zones, islands, sheet counters)
    std::string m_rec_path;     // the same log started from the performance widget (Elements: Record CSV)
    void write_prof_csv(float dt);
    // Elements section of the performance widget: the counts of the world per frame (triangles, links, awake / asleep,
    // the work done) and their history for the graphs
    struct ElemSample {
        float v[16];
    };
    std::vector<ElemSample> m_elem_hist; // ring
    int m_elem_head = 0, m_elem_count = 0;
    bool m_elem_freeze = false;
    int m_elem_hover = -1;               // frame under the mouse in the graphs (-1: none)
    void record_elements();
    void ui_elements();
    std::string m_toast;
    float m_toast_time = 0;
    friend void scroll_cb(GLFWwindow*, double, double);
};

} // namespace bl
