// The model editor's document: a node / beam / triangle / wheel model in the Rigs of Rods sense (definition space:
// -x forward, y up, +z left, the ground at y = 0), with beam presets (set_beam_defaults groups: the beam type and
// whether the ends hold their angle), shocks, steering hydros, wheels, a sheet body, the drivetrain and the graphics
// (flexbodies and props bound to nodes, the cab submeshes with their texture coordinates, managed materials, mesh
// wheels). It is written as a .truck file (write_truck) and read back by the game's own parser, so a model spawns
// like any other vehicle; an existing vehicle is imported from its parsed document, graphics included (its meshes
// stay in its folder, `Model::home`, where the model is saved too).
// No UI here: editor.cpp holds the interaction, this file is also built into the parser test.
#pragma once

#include "core/math.h"
#include "vehicle/ror_def.h"

#include <array>
#include <string>
#include <vector>

namespace bl::edit {

enum BeamType : int { BEAM_NORMAL = 0, BEAM_ROPE = 1, BEAM_SUPPORT = 2, BEAM_FRAME = 3 };

struct BeamGroup {                  // a set_beam_defaults preset
    std::string name = "Frame";
    float spring = 3.0e6f, damp = 400.0f, deform = 8.0e4f, brk = 7.0e5f, plastic = 0.0f;
    int type = BEAM_NORMAL;         // normal / rope (pulls only) / support (pushes only)
    bool invisible = false;
    // degrees of freedom of the ends: free (a pin: the beam turns freely about its nodes) or held (each end keeps its
    // angle to the rest of the structure at that node: written as a pair of `joints`, a bending-stiff connection)
    bool hold_rotation = false;
    float joint_k = 20000.0f;       // stiffness N/m of the hold
    vec4 color{0.85f, 0.85f, 0.85f, 1.0f};
    // BEAM_FRAME: frame elements (FEM beams, phys/frame_fem.h) of this section; the spring numbers and hold_rotation do
    // not apply. The joints at the ends (phys::FrameJoint: rigid, ball, hinge_v, hinge_h, swivel, elastic); a beam
    // can have its own (Beam::end_a / end_b)
    std::string frame_material = "Steel";
    int frame_shape = 0;            // phys::FrameShape: tube, box, rod, bar
    float frame_outer = 0.04f, frame_wall = 0.002f; // outer diameter or side, wall (m)
    int frame_end_a = 0, frame_end_b = 0;
    float frame_joint_k = 2.0e4f;   // elastic joints (N m/rad)
    float frame_break = 0;          // break force (N) past which a member tears off its end a (a mount, a hinge), 0: none
    float frame_joint_damp = 0;     // released joints' damping (N m s/rad)
    bool is_frame() const { return type == BEAM_FRAME; }
};

// Layers hold any element (a node, beam, triangle, wheel...): shown or hidden, editable or locked. Groups are named
// sets of nodes (Editorizer's `;grp:` comments of the truck file: a node is in one group), shown or hidden with their
// elements. Both are written into the file as comments and read back (the game's parser ignores them).
// a material of the shell triangles: what they are made of (their physics: stiffness, ductility, strength), their
// areal mass and drawn thickness, their look (-1: the material's own; the metals: the vehicle's paint)
struct ShellPreset {
    std::string name = "Panel";
    std::string material = "Steel";
    float kg_m2 = 15.7f, thickness = 0.006f;
    vec3 color{-1, -1, -1};
    int max_level = -1;
};

// a shell of FEM triangle elements (the truck's `fem_tris`, phys::FrameTri): a sheet of one of the frame members'
// materials of a thickness - the body is the structure (membrane and bending in the frame's implicit step), no frame
// under it needed; its look (below 0: the vehicle's paint)
struct FemPreset {
    std::string name = "Body steel 1 mm";
    std::string material = "Steel";
    float thickness = 0.001f;
    vec3 color{0.78f, 0.14f, 0.10f};
};

struct Layer {
    std::string name = "Default";
    bool visible = true, locked = false;
};

struct Group {
    std::string name;
    bool visible = true;
};

struct Node {
    vec3 p;
    float load = -1.0f;             // load weight (kg) of an 'l' node, -1: the beams' share only
    bool load_bearing = false;      // 'l': takes a share of the cargo mass and the load weight
    bool no_ground = false;         // 'c': no ground contact
    bool contacter = true;          // collides with other bodies
    bool fixed = false;             // fixes: nailed to the world
    float minimass = -1.0f;         // its own least mass (set_default_minimass), -1: the model's minimass
    int layer = 0, group = -1;
};

struct Beam {
    int a = 0, b = 0;
    int group = 0;                  // beam preset (BeamGroup)
    int layer = 0;
    int end_a = -1, end_b = -1;     // a frame element's own joints at its ends (phys::FrameJoint; -1: the preset's)
};

// the set_beam_defaults in effect for a shock, a hydro or a wheel: the builder takes its strength and set from them, a
// shock's bound spring and damping (its stops), a hydro's spring and damping, the rim beams of meshwheels2 (written
// before the element whenever they change; an import keeps the source's)
struct ElemDefaults {
    float spring = 9.0e6f, damp = 12000.0f, deform = 400000.0f, brk = 1.0e30f, plastic = 0.0f;
};

struct Shock {
    int a = 0, b = 0;
    float spring = 60000.0f, damp = 4000.0f, short_bound = 0.3f, long_bound = 0.3f, precomp = 1.0f;
    bool invisible = false;
    ElemDefaults bd{9.0e6f, 12000.0f, 400000.0f, 1.0e6f, 0.0f};
    int layer = 0;
};

struct Hydro {                      // steering rod: its length follows the steering input
    int a = 0, b = 0;
    float factor = 0.3f;
    bool invisible = true, speed_dep = false;
    ElemDefaults bd{1.5e7f, 500.0f, 1.0e30f, 1.0e30f, 0.0f};
    int layer = 0;
};

struct Tri {                        // cab triangle: a collision surface, or a triangle element of the sheet body
    int a = 0, b = 0, c = 0;
    bool collision = true;          // (a collision hull's triangle: `options` has 'h', one-sided, solid behind)
    bool shell = false;             // a triangle element (Model::sheet_material): it bends, dents and cracks
    int layer = 0;
    int submesh = -1;               // an imported submesh (its texture coordinates, material), -1: the editor's own
    int shell_preset = 0;           // a shell's material (Model::shell_preset): 0 the model's sheet material
    bool fem = false;               // a triangle element of the FEM frame (Model::fem_presets): not a cab triangle (it
    int fem_preset = 0;             // makes its own collision surface), neither `collision` nor `shell`
    std::string options;            // the cab options of an imported triangle (collision letters follow `collision`)
    bool hull() const { return !shell && options.find('h') != std::string::npos; }
};

// (BeamLab) a sheet's node welded to a frame node (the truck's `welds`, phys::SoftBody::Weld): the pull spread over the
// sheet's nodes round it within the radius; anchor2 >= 0: held at the point t of the way from anchor to anchor2
struct Weld {
    int anchor = 0, node = 0;
    float radius = 0.2f, brk = 0, k = 0;
    int anchor2 = -1;
    float t = 0;
};

// (BeamLab) a part's frame node b held at a distance on node a's frame (the truck's `mounts`, phys::FrameMount): a
// bolt, a hinge; it lets go past its break force
struct Mount {
    int a = 0, b = 0;
    float brk = 0, k = 0, damp = 0;
    char kind = 'p';                // p point, c clamp, h hinge, s stop, r strap (see ror::Document::MountDef)
    float param = 0;                // a clamp's break moment, a strap's length
    int b2 = -1;                    // a hinge's second node
};

// (BeamLab) a collision volume (the truck's `collision_volumes`, phys::CollisionVolume): a convex hull of points riding
// on anchor nodes of the frame - a car's engine, its cabin's seats, its trunk's load
struct Volume {
    std::string name;
    float break_rms = 0.12f;
    float break_force = 0.0f;       // its crush force (N; 0: none)
    std::vector<int> anchors;
    std::vector<vec3> verts;        // (the hull's points, in the model's space)
    vec3 color{-1, -1, -1};         // drawn as a solid of this colour in the game (an engine block); negative: not drawn
};

// a node that slides along a rail of nodes (the truck's `slidenodes`: a steering rack's ends in its housing)
struct SlideNode {
    int node = 0;
    std::vector<int> rail;
    float spring = -1, brk = -1, tolerance = -1, attach_rate = -1, attach_dist = -1;
};

struct Joint {                      // a beam with orientation: the child node is held at its rest offset in the
    int parent = 0, child = 0;      // parent node's frame (which turns with the parent's beams): a cantilever
    float k = 20000.0f;             // stiffness N/m of the hold (capped by the nodes' stability at spawn)
    float brk = 0;                  // force N at which it yields and tears, 0: never
    int layer = 0;
};

struct Wheel {
    int type = 0;                   // ror::WheelDef::Type: wheels, wheels2, meshwheels, meshwheels2, flexbodywheels, ringwheels
    float radius = 0.35f, rim_radius = 0.2f, width = 0.2f;
    int rays = 12;
    int n1 = -1, n2 = -1;           // axle nodes
    int rigidity = -1;              // -1: none (9999)
    int braking = 1, propulsion = 0, arm = -1;
    float mass = 30.0f, spring = 100000.0f, damp = 800.0f;
    float rim_spring = 400000.0f, rim_damp = 2000.0f; // wheels2 / flexbodywheels
    char side = 'l';                // mesh wheels: the side the rim faces
    std::string face_material = "tracks/wheelface", band_material = "tracks/wheelband"; // wheels / wheels2
    std::string rim_mesh, tyre_material; // mesh wheels (flexbodywheels: the tyre mesh)
    ElemDefaults bd;                // (its beams' strength and set; meshwheels2: the rim beams' spring and damping)
    float friction = 1.0f;          // its nodes' friction (set_node_defaults)
    float grip = 1.0f;              // ringwheels: the tyre's grip (times the ground's friction)
    bool gfx_hidden = false;        // its tyre and rim not drawn in the editor (not saved)
    int layer = 0;
};

// ---- graphics: meshes bound to nodes (the file's flexbodies and props), the cab's submeshes. A mesh may be bound to
// a node a wheel makes (a tyre mesh on the wheel's own nodes): such a reference is kWheelRef + wheel * 4096 + the
// node's index in the wheel, resolved to the file's numbering when written (the wheels' nodes follow the explicit ones)
constexpr int kWheelRef = 1 << 24;
inline bool is_wheel_ref(int n) { return n >= kWheelRef; }
inline int wheel_ref(int wheel, int k) { return kWheelRef + wheel * 4096 + k; }
inline int wheel_of_ref(int n) { return (n - kWheelRef) / 4096; }
inline int wheel_node_of_ref(int n) { return (n - kWheelRef) % 4096; }

struct Flexbody {                   // a mesh skinned to nodes: placed in the frame of ref (origin), x and y nodes, its
    int ref = 0, x = 0, y = 0;      // vertices bound to the nearest nodes of the forset (so it deforms with them)
    vec3 offset{0, 0, 0}, rot{0, 0, 0}; // in that frame; rotation in degrees
    std::string mesh;
    std::vector<int> forset;
    int layer = 0;
    bool hidden = false;            // not drawn in the editor (a view setting, kept in the file as a comment)
    bool enabled = true;            // part of the vehicle; off: kept in the file as a comment only
};

struct Prop {                       // a rigid mesh moving with the frame of its ref, x and y nodes
    int ref = 0, x = 0, y = 0;
    vec3 offset{0, 0, 0}, rot{0, 0, 0};
    std::string mesh;
    std::string extra;              // what follows the mesh name (a dashboard's wheel mesh, a beacon's colour...)
    int layer = 0;
    bool hidden = false, enabled = true; // (as a flexbody's)
};

struct Submesh {                    // an imported cab submesh: its texture coordinates (node, u, v)
    std::vector<std::pair<int, vec2>> texcoords;
    bool backmesh = false;
    bool hidden = false;            // not drawn in the editor (not saved)
};

struct Model {
    std::string title = "New model";
    std::string home = "editor";    // the folder under assets/vehicles it is saved in (an import: the source's)
    float dry_mass = 800.0f, cargo_mass = 0.0f, minimass = 10.0f;
    bool adv_deform = false;        // enable_advanced_deformation (an import keeps the source's)
    // the shell triangles' material (Vehicle::make_sheet_body through the globals cab material `sheet/...`)
    std::string sheet_material = "Steel";
    float sheet_kg_m2 = 15.7f, sheet_thickness = 0.006f;
    int sheet_max_level = -1;       // refinement depth of the shells, -1: the body default (3)
    // more materials for the shells (Tri::shell_preset k >= 1: shell_presets[k - 1]; 0 is the sheet_* above): a glass
    // window, a plastic bumper, an aluminium bonnet on a steel body. Written as `set_shell_material` in the shells section
    std::vector<struct ShellPreset> shell_presets;
    // the FEM triangles' shells (Tri::fem_preset; the first made when the first such triangle is): written as
    // `set_fem_shell` in the fem_tris section
    std::vector<FemPreset> fem_presets;
    // the skin: the cab triangles drawn in a flat colour (globals cab material `color/r,g,b`; a model with shell
    // triangles is drawn by its sheet instead); off: the imported cab material, if any
    bool skin = true;
    vec3 skin_color{0.75f, 0.2f, 0.15f};
    bool skin_hidden = false;       // the editor's own cab triangles not drawn in the editor (not saved)
    std::string cab_material;       // an imported model's own cab material
    std::vector<ror::ManagedMaterialDef> managed_materials;
    bool mm_double_sided = false;
    // a reference mesh of the editor (an .obj file drawn under the model, not part of the vehicle)
    std::string ref_path;
    vec3 ref_offset{0, 0, 0};
    float ref_scale = 1.0f, ref_yaw = 0.0f, ref_alpha = 0.35f;
    bool ref_mockup = true;
    // drivetrain
    bool engine = true;
    float min_rpm = 1000.0f, max_rpm = 6000.0f, torque = 300.0f, diff = 4.0f, reverse = 3.0f, neutral = 1.0f;
    std::vector<float> gears{3.2f, 2.0f, 1.4f, 1.0f, 0.8f};
    bool engine_car = true;         // engoption type c (car) / t (truck)
    // (a road car's: the engine inertia is in the drivetrain's rpm units, 0.4 made it rev ~5x too slowly and a clutch of
    // 200 slipped - the car crawled; RoR mods use 0.05-0.1 and 300-1000)
    float clutch_force = 1000.0f, inertia = 0.08f;
    float brake_force = 3000.0f;
    int cam_center = -1, cam_back = -1, cam_left = -1; // -1: chosen automatically (write_truck)
    // a cockpit camera node (cinecam) above the centre on eight beams to the nearest nodes: a node and beams of its own
    // that take a share of the dry mass, so only when asked (a copy of a vehicle that has one keeps it)
    bool cinecam = false;

    std::vector<BeamGroup> groups;  // beam presets
    std::vector<Layer> layers{Layer()};
    std::vector<Group> node_groups;
    std::vector<Node> nodes;
    std::vector<Beam> beams;
    std::vector<Shock> shocks;
    std::vector<Hydro> hydros;
    std::vector<Tri> tris;
    std::vector<Wheel> wheels;
    std::vector<Joint> joints;
    std::vector<Weld> welds;
    std::vector<Mount> mounts;
    std::vector<Volume> volumes;
    std::vector<SlideNode> slidenodes;
    std::vector<Flexbody> flexbodies;
    std::vector<Prop> props;
    std::vector<Submesh> submeshes;
    int shell_count() const;
    int fem_count() const;
    FemPreset fem_preset(int i) const;               // (none yet: the default)
    int ensure_fem_preset();                         // a preset to make triangles of: the first (made if none)
    void remove_fem_preset(int i);                   // (its triangles take the first)
    int wheel_node_count(int w) const;       // the nodes wheel w makes
    int file_node(int ref) const;            // a node reference (explicit or a wheel's) in the written file's numbering
    int ref_of_file_node(int n) const;       // and back (-1: none)
    vec3 ref_position(int ref) const;        // where it is (a wheel's node: on its tyre ring, as the builder puts it)
    bool ref_valid(int ref) const;
    bool has_graphics() const { return !flexbodies.empty() || !props.empty() || !submeshes.empty(); }
    int shell_preset_count() const { return 1 + (int)shell_presets.size(); }
    ShellPreset shell_preset(int i) const;           // 0: the sheet_* fields as a preset
    void set_shell_preset(int i, const ShellPreset& p);
    void remove_shell_preset(int i);                 // (its triangles take the first)

    Model();                        // the default groups
    // element counts referring to a node, the nearest node to a point (-1 if none within r), the mirror twin (z -> -z,
    // -1 if none), the centroid, a slug for the file name
    int node_uses(int n) const;
    int nearest_node(vec3 p, float r) const;
    int twin(int n) const;
    std::vector<int> twins() const; // every node's twin at once
    vec3 centroid() const;
    std::string slug() const;
    // editing: a node's removal takes its beams, shocks, hydros, triangles, wheels and meshes bound to it with it
    // (a forset or the texture coordinates only lose the node); every id is remapped
    void remove_nodes(const std::vector<int>& ids);
    // each group of nodes becomes one node at its position: the elements on them go to it, the ones that fold up (a
    // beam from the node to itself, a triangle with a node twice, a joint on itself) and the doubles go; a merged node
    // is fixed or load bearing if one of them was. Returns the number of nodes that went.
    int merge_nodes(const std::vector<std::vector<int>>& groups, const std::vector<vec3>& at);
    void remove_beam(int i);
    void remove_shock(int i);
    void remove_hydro(int i);
    void remove_tri(int i);
    void remove_wheel(int i);
    void remove_joint(int i);
    bool has_beam(int a, int b) const { return find_beam(a, b) >= 0; }
    int find_beam(int a, int b) const;
    int add_beam(int a, int b, int group);   // -1 if it exists or a == b
    // mirror copies of the nodes (and the beams, shocks, hydros and triangles among them) across z = 0; nodes on the
    // plane are shared. Returns the number of nodes made.
    int mirror(const std::vector<int>& ids);
    void auto_cameras(int& center, int& back, int& left) const;
    // visibility: a node shows when its layer and its group do; an element when its layer and its nodes do
    bool layer_visible(int l) const { return l < 0 || l >= (int)layers.size() || layers[l].visible; }
    bool layer_locked(int l) const { return l >= 0 && l < (int)layers.size() && layers[l].locked; }
    bool node_visible(int n) const;
    bool node_locked(int n) const { return n >= 0 && n < (int)nodes.size() && layer_locked(nodes[n].layer); }
    bool beam_visible(int i) const { const Beam& b = beams[i]; return layer_visible(b.layer) && node_visible(b.a) && node_visible(b.b); }
    bool shock_visible(int i) const { const Shock& s = shocks[i]; return layer_visible(s.layer) && node_visible(s.a) && node_visible(s.b); }
    bool hydro_visible(int i) const { const Hydro& h = hydros[i]; return layer_visible(h.layer) && node_visible(h.a) && node_visible(h.b); }
    bool tri_visible(int i) const { const Tri& t = tris[i]; return layer_visible(t.layer) && node_visible(t.a) && node_visible(t.b) && node_visible(t.c); }
    bool wheel_visible(int i) const { const Wheel& w = wheels[i]; return layer_visible(w.layer) && node_visible(w.n1) && node_visible(w.n2); }
    bool joint_visible(int i) const { const Joint& j = joints[i]; return layer_visible(j.layer) && node_visible(j.parent) && node_visible(j.child); }
    void remove_layer(int l);       // its elements go to layer 0
    void remove_group(int g);
    void set_layer(const std::vector<int>& node_ids, int layer); // the nodes and every element among them
};

// ---- templates
Model make_empty();
Model make_box(int nx, int ny, int nz, vec3 size, float mass);             // braced box of nodes
Model make_plate(int nx, int nz, vec2 size, float mass, bool sheet);      // a flat sheet of triangles (with a frame)
Model make_cylinder(int segments, int rings, float radius, float length, float mass);
Model make_cart(float length, float width, float height, float mass);     // a braced box body on four wheels
Model make_fem_plate(int nx, int nz, vec2 size, float thickness);         // a steel plate of FEM triangles
Model make_fem_box(int n, vec3 size, float thickness);                    // a hollow box of FEM triangles, n x n a face

// ---- tools
// Delaunay triangulation of the nodes projected on their flattest plane (the axis of least extent): the triangles
// (as node ids, wound to face away from `outside_from`) for a cover of beams, cab triangles or shell triangles
std::vector<std::array<int, 3>> triangulate(const Model& m, const std::vector<int>& ids, vec3 outside_from);
// a Wavefront .obj (positions and faces, polygons fanned): the reference mesh
bool load_obj(const std::string& path, std::vector<vec3>& verts, std::vector<uint32_t>& idx);
// "1-5, 7, 9-12": a forset's node list compressed into ranges, and back
std::string forset_string(const std::vector<int>& ids);
std::vector<int> parse_forset(const std::string& s);

// ---- files
// preview: the meshes switched off are written as parts of the vehicle too (the editor's preview draws them faint)
std::string write_truck(const Model& m, bool preview = false);
bool import_document(const ror::Document& d, Model& m, std::vector<std::string>& warnings);
// the editor's comments of a truck file (layers, groups) applied to a model imported from its parsed document
void read_markers(const std::string& text, Model& m);
std::vector<std::string> validate(const Model& m);                        // problems that would break the spawn

} // namespace bl::edit
