// Parsed Rigs of Rods vehicle definition (.truck/.car/.trailer/.load ...).
// Produced by parse_truck_file(); consumed by the vehicle builder.
// Node references are resolved to file-order node indices (see NodeSlot / RoR numbering rules):
// explicit nodes, cinecam nodes and wheel-generated nodes all share one index space.
#pragma once

#include "core/math.h"

#include <string>
#include <vector>

namespace bl::ror {

// Snapshot of set_beam_defaults / set_beam_defaults_scale state at the time an element was parsed.
struct BeamDefaults {
    float spring = 9000000.0f;
    float damp = 12000.0f;
    float deform = 400000.0f;
    float brk = 1000000.0f;
    float diameter = 0.05f;
    std::string material = "tracks/beam";
    float plastic = 0.0f;
    bool plastic_given = false;     // plastic coef given (>=0) on this or an earlier set_beam_defaults line
    bool user_defined = false;      // a set_beam_defaults line was seen
    bool adv_deform = false;        // enable_advanced_deformation appeared before the set_beam_defaults line
    float scale_spring = 1, scale_damp = 1, scale_deform = 1, scale_break = 1;

    float k() const { return spring * scale_spring; }
    float d() const { return damp * scale_damp; }
    float break_scaled() const { return brk * scale_break; }
    // RoR deformation threshold rule (AS:5718)
    float deform_threshold() const {
        float D = user_defined ? deform : 400000.0f;
        if (user_defined && !adv_deform && D < 400000.0f) D = 400000.0f;
        float creak = (user_defined && plastic_given && plastic >= 0) ? 0.0f : 100000.0f;
        return std::max(D, creak) * scale_deform;
    }
};

struct NodeDefaults {
    float load_weight = -1.0f;
    float friction = 1.0f;
    float volume = 1.0f;
    float surface = 1.0f;
    std::string options;
};

struct NodeDef {
    vec3 pos;
    std::string options;        // own options OR-ed with node defaults options
    float load_weight = -1.0f;  // explicit weight ('l' with a weight, or set_node_defaults load_weight >= 0)
    bool loaded = false;        // 'l' option or node-default load weight >= 0
    NodeDefaults defaults;      // snapshot
    float minimass = -1.0f;     // set_default_minimass snapshot (-1 = use global minimass)
    int detacher_group = 0;
    std::string name;           // nodes2 name (empty for numbered nodes)
    int line = 0;
};

// One slot per created node in file order.
struct NodeSlot {
    enum Kind { EXPLICIT, CINECAM, WHEEL } kind = EXPLICIT;
    int ref = -1;   // EXPLICIT: index into Document::nodes_explicit; CINECAM: into cinecams; WHEEL: into wheels
    int sub = 0;    // WHEEL: index of the generated node within the wheel (0 .. 2*rays or 4*rays - 1)
};

struct BeamDef {
    int n1 = 0, n2 = 0;
    std::string options;        // v i r s, F (BeamLab: a frame element)
    int frame = -1;             // (BeamLab) option F: Document::frame_sections index of its section (-1: a plain beam)
    int end_a = -1, end_b = -1; // (BeamLab) option F: its own joints (phys::FrameJoint; -1: the section's)
    float support_limit = 4.0f; // for 's' beams (<= 0 or omitted -> 4.0)
    BeamDefaults bd;
    int detacher_group = 0;
    int line = 0;
};

struct ShockDef {
    int type = 1; // 1 shocks, 2 shocks2, 3 shocks3
    int n1 = 0, n2 = 0;
    // shocks
    float spring = 0, damp = 0;
    // shocks2 / shocks3
    float spring_in = 0, damp_in = 0, prog_spring_in = 0, prog_damp_in = 0;
    float spring_out = 0, damp_out = 0, prog_spring_out = 0, prog_damp_out = 0;
    float damp_in_slow = 0, split_vel_in = 0, damp_in_fast = 0;
    float damp_out_slow = 0, split_vel_out = 0, damp_out_fast = 0;
    float shortbound = 0, longbound = 0, precompression = 1;
    std::string options;
    BeamDefaults bd;
    int detacher_group = 0;
    int line = 0;
};

struct HydroDef {
    int n1 = 0, n2 = 0;
    float factor = 0;
    std::string options;
    BeamDefaults bd;
    int detacher_group = 0;
    int line = 0;
};

struct CommandDef {
    int n1 = 0, n2 = 0;
    float rate_short = 0, rate_long = 0;     // commands: both equal
    float shortbound = 1, longbound = 1;     // ratios of spawn length
    int key_contract = 0, key_extend = 0;
    std::string options;                     // n i r f c p o
    std::string description;
    float affect_engine = 1.0f;
    bool needs_engine = true;
    BeamDefaults bd;
    int detacher_group = 0;
    int line = 0;
};

struct CinecamDef {
    vec3 pos;
    int nodes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float spring = 8000.0f, damp = 800.0f, node_mass = 20.0f;
    BeamDefaults bd;
    NodeDefaults nd;
};

struct CameraDef {
    int center = 0, back = 0, left = 0;
};

struct WheelDef {
    enum Type { WHEELS, WHEELS2, MESHWHEELS, MESHWHEELS2, FLEXBODYWHEELS, RINGWHEELS } type = WHEELS;
    float radius = 0.5f;        // tyre radius (physics ring)
    float rim_radius = 0.0f;
    float width = 0.2f;
    int rays = 12;
    int n1 = 0, n2 = 0;         // axis nodes as written in the file (builder orders them by z)
    int rigidity = -1;          // -1 = none (9999 in the file)
    int braking = 0;
    int propulsion = 0;
    int arm = 0;
    float mass = 50.0f;
    float spring = 0, damp = 0;         // wheels/meshwheels*: tyre (and rim) spring/damp; wheels2/flexbody: tyre
    float rim_spring = 0, rim_damp = 0; // wheels2 / flexbodywheels
    float grip = 1.0f;                  // (BeamLab) ringwheels: the tyre's grip, times the ground's friction
    char side = 'l';                    // meshwheels*/flexbodywheels
    std::string face_material, band_material; // wheels / wheels2
    std::string rim_mesh;               // meshwheels*/flexbodywheels
    std::string tyre_material;          // meshwheels*: tyre material; flexbodywheels: tyre mesh
    NodeDefaults nd;                    // snapshot (friction etc. for generated nodes)
    BeamDefaults bd;                    // snapshot
    int first_node = 0;                 // file-order index of the first generated node
    int detacher_group = 0;
    int line = 0;
    int node_count() const { return type == RINGWHEELS ? 0 : (type == WHEELS2 || type == FLEXBODYWHEELS) ? 4 * rays : 2 * rays; }
};

struct EngineDef {
    bool present = false;
    float shift_down_rpm = 800, shift_up_rpm = 2000, torque = 1000;
    float diff_ratio = 1, rev_ratio = 1, neutral_ratio = 1;
    std::vector<float> gears;   // forward gears
};

struct EngoptionDef {
    bool present = false;
    float inertia = 10.0f;
    char type = 't';
    float clutch_force = -1, f4 = -1, f5 = -1, post_shift_time = -1, stall_rpm = -1, idle_rpm = -1;
    float max_idle_mix = -1, min_idle_mix = -1, braking_torque = -1;
};

struct TorqueCurveDef {
    std::string model;              // predefined model name (empty if custom points)
    std::vector<vec2> points;       // (rpm, fraction)
};

struct BrakesDef {
    float force = 30000.0f;
    float parking = -1.0f; // <0: 2*force
};

struct AxleDef {
    int w1a = -1, w1b = -1, w2a = -1, w2b = -1;
    std::string modes; // chars o l s v
};

struct TexcoordDef {
    int node = 0;
    float u = 0, v = 0;
};
struct CabDef {
    int n1 = 0, n2 = 0, n3 = 0;
    std::string options;
};
struct SubmeshDef {
    std::vector<TexcoordDef> texcoords;
    std::vector<CabDef> cabs;
    bool backmesh = false;
};

struct FlexbodyDef {
    int ref = 0, x = 0, y = 0;
    vec3 offset, rot; // rot in degrees
    std::string mesh;
    std::vector<int> forset;
    bool has_forset = false;
    int camera_mode = -2;
    int line = 0;
};

struct PropDef {
    enum Special { NONE, MIRROR_LEFT, MIRROR_RIGHT, DASHBOARD, DASHBOARD_RH, SPINPROP, PALE, SEAT, BEACON, REDBEACON, LIGHTBAR };
    int ref = 0, x = 0, y = 0;
    vec3 offset, rot; // rot in degrees
    std::string mesh;
    Special special = NONE;
    std::string wheel_mesh;      // dashboard steering wheel mesh (default dirwheel.mesh)
    vec3 wheel_offset;           // dashboard
    bool has_wheel_offset = false;
    float wheel_angle = 160.0f;
    std::string beacon_material;
    vec3 beacon_color{1, 0.5f, 0};
    int camera_mode = -2;
    int line = 0;
};

struct ManagedMaterialDef {
    std::string name, type, diffuse, damaged_diffuse, specular;
    bool double_sided = false;
};

struct RopeDef {
    int root = 0, end = 0;
    std::string options;
    BeamDefaults bd;
};

struct TieDef {
    int root = 0;
    float max_reach = 0, auto_shorten_rate = 0, min_length = 0, max_length = 0;
    std::string options;
    float max_stress = 100000.0f;
};

// TractionControl / AntiLockBrakes directives (regulation force = exponent ratio, pulses per second).
struct TractionControlDef {
    bool present = false;
    float regulation = 1.0f;  // clamped 1..20
    float wheelslip = 0.25f;
    float fade = 0.0f;
    float pulse = 2000.0f;
    bool on = true;
};
struct AntiLockBrakesDef {
    bool present = false;
    float regulation = 1.0f;
    float min_speed_kmh = 0.0f;
    float pulse = 2000.0f;
    bool on = true;
};

// slidenodes: a node constrained to slide along a rail (chain of nodes)
struct SlideNodeDef {
    int node = 0;
    std::vector<int> rail;       // rail nodes (consecutive pairs form segments)
    float spring = -1, break_force = -1, tolerance = -1, attach_rate = -1, attach_dist = -1;
    int railgroup = -1;
};
struct RailGroupDef {
    int id = 0;
    std::vector<int> nodes;
};

struct GuiSettingsDef {
    float speedo_max = 140.0f;
    bool use_max_rpm = false;
};

struct Document {
    std::string title;
    std::string path;            // full path of the definition file
    std::string dir;             // folder of the definition file (resources are looked up here)
    std::string guid;
    std::vector<std::string> authors;
    std::string description;
    int file_format_version = 0;

    float dry_mass = 0, cargo_mass = 0;
    std::string cab_material;    // globals material (empty = no cab visual)
    float minimass = 50.0f;
    bool minimass_skip_loaded = false;
    float collision_range = -1.0f;
    bool rollon = false;
    bool has_axles_section = false;

    std::vector<NodeSlot> nodes;          // all created nodes, file order (index = node reference)
    std::vector<NodeDef> nodes_explicit;  // nodes / nodes2 definitions
    std::vector<BeamDef> beams;
    std::vector<ShockDef> shocks;
    std::vector<HydroDef> hydros;
    std::vector<CommandDef> commands;
    std::vector<CinecamDef> cinecams;
    std::vector<CameraDef> cameras;
    std::vector<WheelDef> wheels;
    EngineDef engine;
    EngoptionDef engoption;
    TorqueCurveDef torquecurve;
    BrakesDef brakes;
    bool has_brakes = false;
    std::vector<AxleDef> axles;
    std::vector<SubmeshDef> submeshes;
    std::vector<int> contacters;
    std::vector<FlexbodyDef> flexbodies;
    std::vector<PropDef> props;
    std::vector<ManagedMaterialDef> managed_materials;
    std::vector<RopeDef> ropes;
    // BeamLab extensions of the format (the model editor writes them): orientation joints (the child node's frame is
    // held at its rest offset and orientation in the parent's: a bending-stiff link) and the cab triangles that are
    // triangle elements of the sheet body (globals `sheet/...`; none listed: all the cab triangles)
    struct JointDef {
        int parent = 0, child = 0;
        float k = 20000.0f;         // stiffness N/m of the hold (capped by the nodes' stability)
        float brk = 0;              // force N at which it yields and tears, 0: never
        int ref_x = -1, ref_y = -1; // the parent's frame: x towards ref_x, y towards ref_y (-1: picked among its beams)
        int line = 0;
    };
    struct ShellDef {
        int n1 = 0, n2 = 0, n3 = 0;
        int mat = 0;                // 0: the globals' sheet material, k: shell_materials[k - 1]
    };
    // (BeamLab) `set_shell_material name, Material, kg/m2, drawn thickness[, r, g, b[, refinement depth]]` in the
    // shells section: the material of the shells that follow (a colour below 0: the material's own look)
    struct ShellMaterialDef {
        std::string name, material = "Steel";
        float kg_m2 = 15.7f, thickness = 0.006f;
        vec3 color{-1, -1, -1};
        int max_level = -1;
    };
    std::vector<ShellMaterialDef> shell_materials;
    // (BeamLab) `set_frame_section material, shape, outer size m, wall m[, joints[, joint stiffness N m/rad[, break
    // force N[, joint damping N m s/rad]]]]`: the section of the frame elements that follow (beams with the option F:
    // FEM beams). shape: tube, box, rod, bar; joints (both ends, or `a/b`): rigid, ball (or pinned), hinge_v, hinge_h,
    // swivel, elastic; pinned1 / pinned2: a ball at one end. A frame element's line may name its own:
    // `n1, n2, F[, joint a[, joint b]]`. Break force: the member tears off its end a when the force at its ends passes
    // it (a bolted mount, a hinge that lets a door go); joint damping: a released end resists turning (a ball joint's
    // friction, a hinge's)
    struct FrameSectionDef {
        std::string material = "Steel", shape = "tube";
        float outer = 0.04f, wall = 0.002f;
        int end_a = 0, end_b = 0;   // phys::FrameJoint
        float joint_k = 2.0e4f;     // elastic joints (N m/rad)
        float brk = 0;              // break force (N), 0: none
        float joint_damp = 0;       // released joints' damping (N m s/rad)
    };
    std::vector<FrameSectionDef> frame_sections;
    // (BeamLab) `welds`: anchor, sheet node, radius m, break force N[, stiffness N/m[, anchor2, t]]: the sheet held on a frame node (or the point t of the way to anchor2) at
    // a point, the pull spread over the sheet's nodes round the weld's node (their weights falling off to zero at the
    // radius) instead of on the one node (phys::SoftBody::Weld); no stiffness: the most the step allows
    struct WeldDef {
        int anchor = 0, node = 0;
        float radius = 0.2f, brk = 0, k = 0;
        int anchor2 = -1;           // (`..., stiffness, anchor2, t`: held at the point t of the way to anchor2)
        float t = 0;
    };
    std::vector<WeldDef> welds;
    // (BeamLab) `mounts`: node a, node b, break force N[, stiffness N/m[, turning damping N m s/rad]]: b held at the
    // point of a's frame where it stands (a part on the body at a distance: no member between them; phys::FrameMount),
    // both frame nodes; no stiffness: the most the nodes' masses take at the step. A part held by mounts alone is a
    // component of the frame solved on its own
    // `..., turning damping, kind[, parameter]` - kind p (a point, the default), c (a clamp: b and the part's nodes round
    // it held; the parameter its break moment N m), h (a hinge: the parameter the part's second node on its line), s (a
    // stop: pushes b off a only), r (a strap: pulls b back past the parameter times its distance); see phys::FrameMount
    struct MountDef {
        int a = 0, b = 0;
        float brk = 0, k = 0, damp = 0;
        char kind = 'p';
        float param = 0;
        int b2 = -1;                // (a hinge's second node)
    };
    std::vector<MountDef> mounts;
    // (BeamLab) `collision_volumes`: convex hulls (phys::CollisionVolume) riding on frame nodes - "volume name[, break
    // rms]", its "anchors n1, n2, ..." and its "vertex x, y, z" lines (the hull's points, in the definition's space)
    struct VolumeDef {
        std::string name;
        float break_rms = 0.12f;
        float break_force = 0.0f;   // (the volume line's third value, N: its crush force; 0 none)
        std::vector<int> anchors;
        std::vector<vec3> verts;
        vec3 color{-1, -1, -1};     // `color r, g, b`: drawn as a solid (an engine block, a gearbox); negative: not drawn
    };
    std::vector<VolumeDef> volumes;
    // (BeamLab) `fem_tris`: n1, n2, n3 - triangle elements of the FEM frame (a shell: membrane and bending in the frame's
    // implicit step; phys::FrameTri), of the `set_fem_shell material, thickness m[, r, g, b[, refinement depth]]` in effect
    // before them (none: 1 mm steel). Their nodes are frame nodes (shared with the frame elements); their mass goes on them
    struct FemShellDef {
        std::string material = "Steel";
        float thickness = 0.001f;
        vec3 color{-1, -1, -1};     // (below 0: the vehicle's paint)
        int depth = -1;             // bisections of its triangles where they yield (ShellSection::max_level; -1: its default)
    };
    std::vector<FemShellDef> fem_shells;
    struct FemTriDef {
        int n1 = 0, n2 = 0, n3 = 0;
        int shell = 0;              // fem_shells index
    };
    std::vector<FemTriDef> fem_tris;
    std::vector<JointDef> joints;
    std::vector<ShellDef> shells;
    std::vector<TieDef> ties;
    std::vector<int> fixes;
    std::vector<SlideNodeDef> slidenodes;
    std::vector<RailGroupDef> railgroups;
    GuiSettingsDef gui;
    TractionControlDef traction;
    AntiLockBrakesDef antilock;
    float speed_limit = -1.0f; // speedlimiter (m/s), <0 = none

    std::vector<std::string> configs;     // section names (selectable configurations)
    std::string selected_config;
    std::vector<std::string> warnings;
};

// Parses a RoR vehicle file. `config` selects a `section` module ("" = first configuration if any).
// Returns false only if the file can't be read or has no nodes.
bool parse_truck_file(const std::string& path, Document& doc, const std::string& config = "");
// the same from text in memory; `path` names the file it stands for (its folder is where meshes and materials are found)
bool parse_truck_text(const std::string& text, const std::string& path, Document& doc, const std::string& config = "");

// Reads just the title (first non-comment line) - fast, for the vehicle list UI.
std::string read_truck_title(const std::string& path);

} // namespace bl::ror
