// Game state: physics world, scene content, vehicles, camera, interaction.
#pragma once

#include "game/camera.h"
#include "gfx/renderer.h"
#include "phys/world.h"
#include "world/objects.h"
#include "world/grass.h"
#include "world/stage.h"
#include "world/terrain.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace bl {

class Vehicle;
struct VehicleEntry;

struct VehicleInput {
    float throttle = 0, brake = 0, steer = 0; // steer: -1 left .. +1 right
    bool steer_direct = false; // (the mouse's steering: the wheels follow `steer` at once, not at the keys' rate that slows with speed)
    bool handbrake = false;
    bool shift_up = false, shift_down = false;
    int command_key = 0; // RoR command key (F-keys), 0 = none
    int command_dir = 0;
};

struct DebugView {
    bool beams = false;
    bool nodes = false;
    bool stress = true;        // (F8) the beam view coloured by the elements' loads (off: by their deformation)
    bool collision = false;    // collision triangles / capsules
    bool hide_meshes = false;  // skeleton only
    bool contacts = false;
    bool islands = false;
    bool wheels = false;
    bool frames = true;        // with the nodes: the joints' frames as small axes
    float beam_px = 1.0f;      // the beams' width on screen (points; the model editor's display setting)
    bool hide_terrain = false; // the ground is not drawn (it still collides): the model editor's floor switch
    bool xray = false;         // (F6) the FEM plates and the sheets see-through: the body's inside (its members, engine, wheels)
    float xray_alpha = 0.2f;
    bool labels = false;       // (off: View menu) with the beams: the ring tyres' load and slip, the most loaded elements' forces and stresses;
                               // with the volumes their names (close to the camera)
    bool volumes = false;      // (F7) the collision volumes (a car's engine, its zones, the ring tyres' drums)
};

enum class Tool { Grab = 0, Destroy = 1, Shoot = 2, Laser = 3 };

// The simulation speeds of the time control (the menu, [ and ], the model editor's physics test): slow motion down to
// a hundredth of real time
inline constexpr float kTimeScales[] = {0.01f, 0.02f, 0.05f, 0.1f, 0.25f, 0.5f, 1.0f, 2.0f};
inline constexpr int kTimeScaleCount = (int)(sizeof(kTimeScales) / sizeof(kTimeScales[0]));
inline int time_scale_index(float ts) { // (the nearest step, by ratio)
    int best = 0;
    for (int i = 1; i < kTimeScaleCount; i++)
        if (std::fabs(std::log(kTimeScales[i] / ts)) < std::fabs(std::log(kTimeScales[best] / ts))) best = i;
    return best;
}

// Vehicle vs vehicle crash test configuration (Crash scene).
struct CrashConfig {
    std::string vehicle_a, vehicle_b;
    float speed_a = 60.0f, speed_b = 60.0f; // km/h
    int layout = 0;                         // 0 head-on, 1 offset head-on, 2 T-bone, 3 angled 45, 4 rear-end
    bool slow_motion = true;
    static const char* layout_name(int i);
};

// Text floating in the world (names of test samples), drawn by the UI when the camera is close enough.
struct WorldLabel {
    vec3 pos;
    std::string text;
    vec4 color{1, 1, 1, 1};
};

// Extra entry of the Scene menu, defined by the current scene.
struct SceneAction {
    std::string label;
    std::function<void(class Game&)> run;
};

struct SceneInfo {
    std::string id;
    std::string name;
    std::string category;
    std::string description;
    std::function<void(class Game&)> build;
};
const std::vector<SceneInfo>& scene_registry();
// A vehicle whose `globals` cab material is `sheet/<Material>[/kg/m2[/thickness]]` gets its cab triangles turned into
// triangle elements at spawn (Vehicle::make_sheet_body); Game::spawn_vehicle calls this for every vehicle.
void apply_vehicle_sheet_body(class Vehicle* v, bool player);

class Game {
public:
    Game();
    ~Game();
    bool init();
    void load_scene(int index);
    void update(float dt, const VehicleInput& in, const CameraInput& cam_in);
    // cam: another camera than the controller's (the model editor's views); visuals: skin the vehicles and objects
    // (off for the second and later views of one frame)
    void render(Renderer& r, int w, int h, const Camera* cam = nullptr, bool visuals = true);
    void draw_debug(Renderer& r);

    // ---- scene building API
    phys::World world;
    TerrainRender terrain_render;
    std::unique_ptr<RoadRender> road_render;
    std::unique_ptr<GrassField> grass;
    std::unique_ptr<StageScenery> scenery; // imported stage: replaces the terrain mesh
    float ground_height(float x, float z) const; // terrain + road detail
    LightSettings light;
    void create_terrain(int nx, int nz, float cell, vec2 origin);
    void finish_terrain();
    void add_static_box(vec3 center, vec3 half, const quat& rot, uint8_t surface, MaterialPtr mat, bool render = true);
    void add_static_cylinder(vec3 base, float radius, float height, uint8_t surface, MaterialPtr mat);
    // a mesh of the scene's own, in world space (drawn, nothing to collide with: a road's ribbon over the terrain, a
    // building's walls round its static box)
    void add_static_mesh(const std::vector<Vertex>& v, const std::vector<uint32_t>& idx, MaterialPtr mat);
    // a mesh kept by someone else (a model of assets/models) drawn at `model`
    void add_static_visual(const GpuMesh* mesh, MaterialPtr mat, const mat4& model) { m_static_visuals.push_back({mesh, mat, model}); }
    // (before finish_terrain: the terrain's render mesh lowered per heightfield vertex - under a road's ribbon)
    std::vector<float> terrain_drop;
    DynamicObject* add_object(std::unique_ptr<DynamicObject> o);
    void set_spawn(vec3 pos, float yaw_deg) {
        spawn_pos = pos;
        spawn_yaw = yaw_deg;
    }
    // Place a vehicle definition in the world (returns nullptr on failure).
    Vehicle* spawn_vehicle(const std::string& vehicle_id, vec3 pos, float yaw_deg, bool make_player);
    void remove_vehicle(Vehicle* v);
    void clear_vehicles();
    void presettle_static_objects();
    // Scene-defined per-frame logic (AI drivers, launchers...)
    std::function<void(Game&, float)> scene_update;
    std::string scene_hint;
    std::string scene_status;               // live text under the hint (stage timer, results)
    std::string scene_banner;               // big text at the top centre (running stage time)
    std::vector<SceneAction> scene_actions; // scene-specific Scene menu entries
    std::vector<WorldLabel> labels;         // scene-specific labels in the world
    std::vector<WorldLabel> debug_labels;   // (the debug view's, remade each frame: volumes' names, ring tyres' load)
    bool no_player_vehicle = false; // scene manages its own vehicles (crash test)

    // ---- state
    int scene_index = -1;
    vec3 spawn_pos{0, 0, 0};
    float spawn_yaw = 0;
    std::vector<std::unique_ptr<DynamicObject>> objects;
    std::vector<std::unique_ptr<Vehicle>> vehicles;
    int player = -1;
    Vehicle* player_vehicle() const;
    CameraController cam;
    DebugView debug;
    bool paused = false;
    bool step_once = false;
    std::string selected_vehicle; // id from the vehicle registry
    bool spawn_override = false;  // use spawn_override_pos/yaw instead of the scene spawn point
    vec3 spawn_override_pos;
    float spawn_override_yaw = 0;
    float frame_physics_ms = 0;

    // mouse grab
    bool grab_active = false;
    phys::SoftBody* grab_body = nullptr;
    int grab_node = -1;
    float grab_depth = 0;
    float grab_strength = 1.0f;   // the grab tool's pull: a multiplier of the automatic strength (from the body's mass)
    float grab_radius = 0.3f;     // the grab tool's sphere: the nodes of every body within it are pulled, less towards its edge (0: one node)
    std::vector<phys::SoftBody*> grab_bodies; // (the bodies it holds: grab_body, the picked one, and the others in its sphere)
    bool grab_holds(const phys::SoftBody* b) const { return std::find(grab_bodies.begin(), grab_bodies.end(), b) != grab_bodies.end(); }
    void grab_begin(vec3 ray_o, vec3 ray_d);
    void grab_update(vec3 ray_o, vec3 ray_d);
    void grab_end();
    // a crane: the vehicle hung by the top of its frame (its highest frame nodes, else its highest nodes) `lift` above
    // where it is, tilted by roll and pitch (degrees), the wheels hanging in the suspension; crane_release lets go
    void crane_vehicle(Vehicle* v, float lift, float roll_deg = 0, float pitch_deg = 0);
    void crane_release();
    std::vector<uint32_t> crane_nodes;
    Vehicle* crane_car = nullptr;

    // spawnable primitives for the lab / menu
    void drop_primitive(int kind, vec3 at);
    void remove_object(DynamicObject* o);

    // ---- cursor tools
    Tool tool = Tool::Grab;
    float destroy_radius = 0.5f;
    bool destroy_blast = true;
    int projectile_kind = 0;        // see projectile_name()
    float projectile_speed = 40.0f; // m/s
    float fire_rate = 6.0f;         // shots per second while held
    int max_projectiles = 60;
    std::vector<DynamicObject*> projectiles;
    bool cursor_valid = false;
    vec3 cursor_point;
    int destroyed_total = 0;
    // laser: while held, everything the swept ray passes through is cut along it (World::laser_cut)
    bool laser_active = false;
    vec3 laser_origin, laser_dir;
    int laser_total = 0;
    float laser_range = 150.0f;
    void laser_sweep(vec3 origin, vec3 dir);
    void laser_release() { laser_active = false; }
    static const char* projectile_name(int kind);
    static constexpr int kProjectileKinds = 5;
    // Updates the cursor hit point (tools indicator). Returns true if something was hit.
    bool update_cursor(vec3 ray_o, vec3 ray_d);
    void destroy_under_cursor();
    void shoot(vec3 origin, vec3 dir);
    void clear_projectiles();
    void next_vehicle();

    // ---- crash scene
    CrashConfig crash;
    void run_crash();
    std::string crash_report() const;
    vec3 camera_focus() const;
    const Camera& last_camera() const { return m_last_cam; }

private:
    struct StaticVisual {
        const GpuMesh* mesh;
        MaterialPtr mat;
        mat4 model;
    };
    std::vector<StaticVisual> m_static_visuals;
    std::vector<std::unique_ptr<GpuMesh>> m_static_meshes;
    InstanceCollector m_instances;
    Camera m_last_cam;
    std::unordered_map<const phys::SoftBody*, std::vector<int>> m_vol_hits; // (the debug view: each volume's contacts at the last frame)
    int m_object_counter = 0;
    Vehicle* m_crash_a = nullptr;
    Vehicle* m_crash_b = nullptr;
    int m_crash_phase = 0;
    float m_crash_va = 0, m_crash_vb = 0;

public:
    void crash_update(float dt);
};

} // namespace bl
