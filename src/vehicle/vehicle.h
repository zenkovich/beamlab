// Runtime vehicle: soft body built from a RoR definition + drivetrain + controls + visuals.
#pragma once

#include "game/game.h"
#include "phys/softbody.h"
#include "vehicle/ror_def.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace bl {

struct VehicleEntry {
    std::string id;       // unique id (folder/file stem)
    std::string folder;   // assets/vehicles/<folder>
    std::string file;     // full path of the definition file
    std::string title;    // name from the definition
    std::string type;     // truck / car / trailer / load
    std::string group;    // folder display name
    std::string kind;     // the menu's section (vehicle_kinds): from SOURCE.txt's "kind: " line, else by the folder
    bool drivable = true; // has an engine (false: trailers, caravans, loads)
    std::string text;     // a definition in memory (the model editor's previews): `file` then only names its folder
    bool split_parts = false; // the visual part by part (the model editor: VehicleVisual::split_parts)
};
const std::vector<VehicleEntry>& vehicle_registry();
const std::vector<std::string>& vehicle_kinds(); // the sections, in the menu's order
void refresh_vehicle_registry(); // rescans assets/vehicles (the model editor saves new files there); entries move
const VehicleEntry* find_vehicle(const std::string& id);

class VehicleVisual;
struct Drivetrain;
class Vehicle;

struct AIState {
    std::vector<vec3> route;     // waypoints (xz used); empty = wander / derby
    std::vector<float> route_speed; // race line: speed limit per waypoint (m/s) -> look-ahead follower with braking
    std::vector<float> route_radius; // turn radius at each race line point (m)
    bool route_loop = true;
    bool autopilot = false;      // drive the player's vehicle too
    int wp = 0;
    float target_speed = 12.0f;  // m/s
    float stuck_timer = 0;
    float reverse_timer = 0;
    float retarget_timer = 0;
    vec3 wander_target;
    Vehicle* target = nullptr;
};

class Vehicle {
public:
    Vehicle() = default;
    ~Vehicle();
    static std::unique_ptr<Vehicle> create(const VehicleEntry& e, phys::World& world, vec3 pos, float yaw_deg, std::string* err);

    phys::SoftBody* body = nullptr;
    std::string id, name;
    bool is_player = false;
    bool ai = false;
    AIState ai_state;

    void set_input(const VehicleInput& in);
    void update_frame(float dt);   // main thread after physics
    void update_visuals();         // may run on a worker thread
    void draw(Renderer& r, InstanceCollector& ic, const DebugView& dbg);
    void reset(vec3 pos, float yaw_deg);
    void recover();                // put back on wheels at the current location
    // Sets the whole vehicle moving with `velocity`, wheels rolling without slip (crash tests).
    void launch(vec3 velocity);
    // The body panels (the collision cab triangles) become a sheet of triangle elements on the node-beam frame: they
    // dent, tear and crack like the Materials Lab sheets, drawn as such (the cab visual, if any, is not). `kg_m2`: the
    // sheet's areal mass on top of the nodes' own; `thickness`: drawn.
    // only: these cab triangles (by their nodes, any order) become elements, the others stay collision triangles
    // only_mat: each listed triangle's material (0: mat, k: extra[k - 1]: its own physics, areal density and look)
    struct SheetMaterial {
        phys::ShellMaterial mat;     // (mat.kg_m2: its areal density)
        MaterialPtr visual;
    };
    void make_sheet_body(const phys::ShellMaterial& mat, float kg_m2, MaterialPtr visual, float thickness, const std::vector<std::array<int, 3>>* only = nullptr,
                         const std::vector<int>* only_mat = nullptr, const std::vector<SheetMaterial>* extra = nullptr);
    bool sheet_body() const { return m_sheet != nullptr; }
    float peak_g = 0;              // peak deceleration (g) seen since the last reset
    float assist_speed = -1;       // crash tests: invisible "tow" holding this speed along assist_dir (<0 = off)
    vec3 assist_dir;
    vec3 last_com_vel;

    vec3 position() const;
    vec3 forward() const;
    vec3 left() const;
    vec3 up() const;
    vec3 velocity() const;
    vec3 cockpit_pos() const;
    float speed_kmh() const;
    float rpm() const;
    float max_rpm() const;
    int gear() const;
    int num_gears() const;
    float throttle() const;
    float brake() const;
    float steer_state() const; // -1 .. 1: where the steering is (the input rate limited)
    bool has_engine() const;
    // The brake lines cut (the axe cutting the car in two): no brake from then on, nor the hold at a standstill - held
    // by their brakes, the halves leaned on each other at the cut, level (until reset)
    void cut_brakes();
    float mass() const { return m_mass; }
    int broken_beams() const;
    const ror::Document& def() const { return m_def; }
    // the model editor: the body put back in definition space at `origin` (no rotation, no lift) and held there,
    // the visual drawn see-through (alpha < 1) or hidden (alpha 0)
    void place_definition(vec3 origin);
    void set_node_position(int i, vec3 p);
    float ghost = 1.0f;
    VehicleVisual* visual() { return m_visual.get(); }
    const FrameVisual* frame_visual() const { return m_frame.get(); }
    const std::vector<std::string>& load_warnings() const { return m_warnings; }
    const std::vector<phys::Node>& spawn_nodes() const { return m_spawn_nodes; } // (as built, in the definition's space)

private:
    friend class VehicleBuilder;
    friend class VehicleVisual;
    ror::Document m_def;
    std::unique_ptr<VehicleVisual> m_visual;
    std::unique_ptr<Drivetrain> m_drive;
    std::vector<phys::Node> m_spawn_nodes;  // pristine node state (definition space) for resets
    std::vector<phys::Beam> m_spawn_beams;
    std::vector<phys::Shock> m_spawn_shocks;
    std::vector<phys::Frame> m_spawn_frames;
    std::vector<phys::Joint> m_spawn_joints;
    std::vector<phys::Weld> m_spawn_welds;  // (the sheet's welds on the frame as built: make_sheet_body)
    phys::FemFrame m_spawn_fem;             // the frame elements as built (rest orientations: definition space)
    std::vector<phys::CollisionVolume> m_spawn_volumes; // the collision volumes as built (a reset puts them back whole)
    std::unique_ptr<ShellVisual> m_sheet;   // sheet body (make_sheet_body): its visual, and the pristine sheet for resets
    std::unique_ptr<FrameVisual> m_frame;   // the frame elements' tubes
    bool m_sheet_first = true;
    std::vector<phys::Shell> m_spawn_shells;
    std::vector<phys::Triangle> m_spawn_tris;
    std::vector<phys::NodeInfo> m_spawn_info;
    std::vector<float> m_spawn_base_mass;
    phys::World* m_world = nullptr;
    int m_cam_center = 0, m_cam_back = 0, m_cam_left = 0;
    int m_cinecam = -1;
    int m_cinecam_broken = 0; // (the body's broken beams when its cinecam's were last looked at)
    vec3 m_local_fwd{-1, 0, 0}, m_local_left{0, 0, 1};
    float m_mass = 0;
    VehicleInput m_input;
    std::vector<std::string> m_warnings;
    void substep(phys::SoftBody& b, float dt);
};

} // namespace bl
