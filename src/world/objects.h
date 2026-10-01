// Dynamic world objects built from nodes/beams (boxes, ropes, bridges, trees ...) and their visuals.
#pragma once

#include <functional>
#include "phys/sheet_builder.h"

#include "gfx/renderer.h"
#include "phys/world.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace bl::phys {
class ShellMesher;
}

namespace bl {

// Triangles skinned directly to body nodes (positions follow the nodes).
struct SurfaceVisual {
    MaterialPtr mat;
    std::vector<uint32_t> corner_node; // 3 per triangle
    std::vector<vec2> corner_uv;
    bool smooth = false;
    GpuMesh mesh;
    std::vector<Vertex> verts;
    std::vector<uint32_t> idx;
    // smooth shading: unique node list
    std::vector<uint32_t> unique_nodes;
    std::vector<uint32_t> corner_vertex;
    std::vector<uint32_t> normal_group; // per vertex: first vertex of the same node (normals are shared per node)
    // torn triangles (stretched past phys::kTornStretch2) are hidden for good
    std::vector<float> rest_edge2;
    std::vector<uint8_t> torn;
    std::vector<int32_t> body_tri; // matching collision triangle (-1 = visual only)
    bool indices_dirty = false;
    void update(const phys::SoftBody& b, bool first); // CPU only (thread safe)
    void upload(bool first);                          // GL
};

// Sheet of triangle elements (phys::Shell), drawn 1:1 from the body's shells and rebuilt whenever its topology
// changes (refinement, cracks). Both faces are offset by half the thickness and the free edges (the border and
// the cracks) are closed with side walls, so every crack shows the material's cut edge.
// The frame elements of a body (phys::FemFrame) drawn as tubes: every member an 8-sided prism of its section's outer
// radius between its nodes, a little past them so that the tubes meet at a joint. Split members bend along their new
// nodes; the pieces of torn ones go with them. The vertices follow the nodes every frame, the indices only when the
// members change.
// Its triangle elements (FemFrame::tris) are plates of their section's thickness: both faces offset by half of it,
// shaded smooth across edges that bend less than 35 degrees, in `plate_mat` or their section's own material (a torn-out
// triangle is gone).
struct FrameVisual {
    MaterialPtr mat;
    MaterialPtr plate_mat;
    std::vector<MaterialPtr> section_mats;              // per shell section (none / null: plate_mat): a section's own colour
    std::vector<std::pair<int, int>> plate_ranges;      // per shell section: its plates' indices (first, count)
    GpuMesh mesh, plate_mesh;
    std::vector<Vertex> verts, plate_verts;
    std::vector<uint32_t> idx, plate_idx;
    size_t built = ~size_t(0), plate_built = ~size_t(0);
    bool rebuilt = false, plate_rebuilt = false;
    std::vector<vec3> node_normal;   // (scratch: the plates' smooth normals per frame node)
    mutable std::unordered_map<const Material*, std::unique_ptr<Material>> ghosts; // see-through copies of the materials
    void update(const phys::SoftBody& b);
    void upload();
    // (alpha below 1: see-through, as the editor's preview draws a vehicle; plate_alpha the plates' own, the x-ray view's
    // - negative: alpha)
    void draw(Renderer& r, float alpha = 1.0f, float plate_alpha = -1.0f) const;
};
MaterialPtr frame_plate_material(); // (bare steel plate, shared)
MaterialPtr frame_tube_material(); // (painted steel tube, shared)

struct ShellVisual {
    MaterialPtr mat;
    std::vector<MaterialPtr> mats;                    // per Shell::mat (none / null: mat): a sheet of several materials
    std::vector<std::pair<int, int>> ranges;          // per Shell::mat: its indices (first, count) after a rebuild
    void draw(Renderer& r, float alpha = 1.0f) const; // the sheet (by material) and its artifacts (alpha below 1: see-through)
    mutable std::unordered_map<const Material*, std::unique_ptr<Material>> ghosts; // (its see-through copies of the materials)
    float thickness = 0.0f;
    float crease_cos = -2.0f;                         // shading normals split where faces meet at more than acos(this)
                                                      // (a drum's ends and wall: 45 degrees); below -1: smooth everywhere
    GpuMesh mesh;
    std::vector<vec3> face_normal;
    std::vector<Vertex> verts;
    std::vector<uint32_t> idx;
    std::vector<vec3> node_normal;
    struct Wall {
        uint32_t shell;
        uint8_t edge;
    };
    std::vector<Wall> walls;
    uint32_t topo = UINT32_MAX;
    bool rebuilt = false;
    // Artifacts over the sheet (update_fx): the crack lines of a fracture pattern on the still intact material (glass:
    // a white web; metal: the crease of the ring), rims along the cracks (a glint on glass, fresh metal, fresh wood),
    // the scorched edge of a laser cut, splinters of wood along the grain and burrs of torn metal, the mark of an
    // impact (crushed glass, a scuff on metal, a bruise on wood). Each kind is a mesh of its own material.
    enum FxKind { FX_LINE, FX_RIM, FX_CHAR, FX_SLIVER, FX_MARK, FX_COUNT };
    struct Fx {
        GpuMesh mesh;
        std::vector<Vertex> v;
        std::vector<uint32_t> i;
        MaterialPtr mat;
        bool rebuilt = false;
    } fx[FX_COUNT];
    struct FxItem {
        uint32_t shell;
        uint8_t edge, kind;
        uint16_t impact;
        uint32_t seed;
    };
    std::vector<FxItem> fx_items;
    struct WebPt {                                    // a point of a pattern line on the sheet (FX_LINE): shell, barycentric
        int32_t shell;
        float u, v;
    };
    std::vector<WebPt> web;
    int fx_style = 0;                                 // phys::ShellPattern of the body (the artifacts' materials)
    void update_fx(const phys::SoftBody& b, bool rebuild);
    void update(const phys::SoftBody& b, bool first); // CPU only (thread safe)
    void upload(bool first);                          // GL
};
// Material of a sheet artifact (ShellVisual::FxKind) for a fracture pattern style (GL thread).
MaterialPtr make_sheet_fx_material(int kind, int style);

struct StickVisual {
    uint32_t beam;
    float radius;
    int mat; // index into DynamicObject::stick_mats
};

// Visual for trees: bark segments from joints and leaf clusters on frames (CPU-built mesh).
struct TreeVisual {
    struct Segment {
        uint32_t joint;
        float r0, r1; // radius at parent / child
    };
    struct Leaf {
        uint32_t frame;
        vec3 offset;   // in frame space
        quat rot;      // in frame space
        float size;
        vec4 tint;
    };
    std::vector<Segment> segments;
    std::vector<Leaf> leaves;
    MaterialPtr bark, leaf;
    GpuMesh bark_mesh, leaf_mesh;
    std::vector<Vertex> bv, lv;
    std::vector<uint32_t> bi, li;
    void update(const phys::SoftBody& b, bool first); // CPU only (thread safe)
    void upload(bool first);                          // GL
};

// Rigid mesh carried by the frame of a (box) body: imported props keep their own meshes, the body is the collision
// hull. The frame comes from an origin node and the two longest box edges.
struct RigidVisual {
    struct Part {
        const GpuMesh* mesh;
        const Material* mat;
    };
    std::vector<Part> parts;
    uint32_t n0 = 0, na = 1, nb = 2;
    mat4 local; // mesh space -> body frame
    mat4 model; // mesh space -> world, updated with the visuals
    mat4 frame(const phys::SoftBody& b) const;
};

class DynamicObject {
public:
    std::string name;
    phys::SoftBody* body = nullptr;
    std::unique_ptr<RigidVisual> rigid;
    std::vector<std::unique_ptr<SurfaceVisual>> surfaces;
    std::vector<StickVisual> sticks;
    std::vector<MaterialPtr> stick_mats;
    std::unique_ptr<TreeVisual> tree;
    std::unique_ptr<ShellVisual> sheet;
    std::unique_ptr<FrameVisual> frame;     // frame elements (debris of a torn frame)
    bool visuals_dirty = true;
    std::vector<InstanceData> stick_instances; // cached per stick material: flattened, see update
    std::vector<int> stick_instance_mat;

    bool needs_visual_update() const { return visuals_dirty || !body->sleeping || (sheet && sheet->topo != body->topo_version); }
    bool prepare_visuals();  // CPU part, returns true if an upload is needed (thread safe per object)
    void upload_visuals();   // GL part (main thread)
    void update_visuals();   // both
    void draw(Renderer& r, struct InstanceCollector& ic, float xray = 0.0f); // (xray > 0: its plates and sheet see-through, that alpha)

private:
    bool m_upload_first = false;

public:
};

// Collects instanced primitives from many objects into shared batches (one draw per material).
struct InstanceCollector {
    struct Batch {
        const GpuMesh* mesh;
        const Material* mat;
        std::vector<InstanceData> data;
        std::unique_ptr<InstanceBatch> gpu;
    };
    std::vector<Batch> batches;
    void begin();
    void add(const GpuMesh* mesh, const Material* mat, const InstanceData& d);
    void flush(Renderer& r);
};

// Shared procedural meshes/materials used by builders & scenes.
struct SharedAssets {
    GpuMesh box, cylinder, sphere, cone, stick;
    MaterialPtr birch_bark;
    MaterialPtr tape, straw, bale_side, bale_end, stake, banner;
    MaterialPtr wood, dark_wood, metal, rust, concrete, rubber, orange, white, red, blue, green, yellow, glass, bark, leaves, leaves2, needles, bush, cone_mat, stone, jelly;
    TexturePtr tex_checker, tex_fabric, tex_tape, tex_wood, tex_bark, tex_leaf, tex_leaf2, tex_needle, tex_concrete, tex_crate;
    void init();
    static SharedAssets& get();
};

// Gravel road material for RoadRender (texture laid out across the full ribbon width).
MaterialPtr make_road_material(float half_width, float edge);
MaterialPtr make_pothole_material(); // dark wet mud, frayed round edge

// ------------------------------------------------------------------ builders
struct BeamParams {
    float k = 2.0e6f, d = 4000.0f, deform = 4.0e5f, strength = 1.0e6f, plastic = 0.0f;
};

struct SoftBoxDesc {
    vec3 center;
    vec3 size{1, 1, 1};
    quat rot;
    int nx = 2, ny = 2, nz = 2; // nodes per axis
    float mass = 100.0f;
    BeamParams beams;
    MaterialPtr mat;
    bool fixed_bottom = false;
    float uv_scale = 1.0f;
};
std::unique_ptr<DynamicObject> build_soft_box(phys::World& w, const SoftBoxDesc& d, const std::string& name);

// A giant axe on a pendulum: a handle hung from its pivot (two fixed nodes on the axis, which runs along z) and a
// blade at its foot, released at `angle` from hanging straight down (towards -x): it swings down under its weight.
// Its cutting edge is the blade's leading side (the one it swings towards): edge_node its middle.
struct AxeDesc {
    vec3 pivot;
    float length = 8.2f;                                // pivot to the blade's edge
    float blade_w = 1.8f, blade_h = 1.5f, thick = 0.12f; // the blade: across the swing, along the handle, thickness (z)
    float handle = 0.24f;                               // the handle's section
    float mass = 2000.0f;
    float angle = 1.3f;
    MaterialPtr mat;
};
std::unique_ptr<DynamicObject> build_axe(phys::World& w, const AxeDesc& d, const std::string& name, uint32_t* edge_node = nullptr);

struct SoftSphereDesc {
    vec3 center;
    float radius = 0.6f;
    int subdiv = 2;
    float mass = 60.0f;
    BeamParams beams;
    MaterialPtr mat;
};
std::unique_ptr<DynamicObject> build_soft_sphere(phys::World& w, const SoftSphereDesc& d, const std::string& name);

// A ball as one collision shape (the Shoot tool's projectiles): a single node of the ball's mass, its sphere against
// the ground and the static world (the node's radius), against other bodies' triangles and nodes (a capsule of no
// length round it) and the sheets' sphere contacts (SoftBody::sphere_ball); no beams. It slides rather than rolls
// (no spin); bounce: the restitution of its static contacts.
struct BallDesc {
    vec3 center;
    float radius = 0.22f, mass = 40.0f, bounce = 0.2f, friction = 0.5f;
    MaterialPtr mat;
};
std::unique_ptr<DynamicObject> build_ball(phys::World& w, const BallDesc& d, const std::string& name);

struct RopeDesc {
    vec3 a, b;
    int segments = 16;
    float mass = 20.0f;
    float radius = 0.04f;
    bool fix_a = true, fix_b = false;
    BeamParams beams;
    MaterialPtr mat;
    float end_mass = 0.0f; // extra mass on the free end (pendulum)
    float end_size = 0.0f; // add a soft box weight at the end
};
std::unique_ptr<DynamicObject> build_rope(phys::World& w, const RopeDesc& d, const std::string& name);

struct BridgeDesc {
    vec3 start, end;        // deck centerline endpoints (anchored)
    float width = 4.5f;
    int segments = 12;
    float truss_height = 2.5f;
    bool truss = true;       // side trusses (else suspension-like simple deck)
    float girder_depth = 0;  // >0: Warren girder under the deck (wooden bridge)
    float mass_per_m = 350.0f;
    BeamParams deck, truss_beams;
    MaterialPtr deck_mat, truss_mat;
    uint8_t surface = phys::SURF_WOOD;
};
std::unique_ptr<DynamicObject> build_bridge(phys::World& w, const BridgeDesc& d, const std::string& name);

enum class TreeKind { Deciduous, Pine, Birch, Bush, Dead };
struct TreeDesc {
    vec3 base;
    TreeKind kind = TreeKind::Deciduous;
    float height = 8.0f;
    float trunk_radius = 0.18f;
    uint32_t seed = 1;
    float stiffness = 1.0f;   // scales joint stiffness
    float strength = 1.0f;    // scales break thresholds
    bool leaves = true;
};
std::unique_ptr<DynamicObject> build_tree(phys::World& w, const TreeDesc& d, const std::string& name);

// Cantilever / pole of oriented segments (demonstrates orientation preserving joints).
struct PoleDesc {
    vec3 base;
    vec3 dir{0, 1, 0};
    int segments = 8;
    float length = 4.0f;
    float radius = 0.08f;
    float mass = 40.0f;
    float k_ang = 4.0e4f;
    float yield_deg = 0.0f;   // plastic bending (0 = elastic)
    float break_torque = 0.0f;
    MaterialPtr mat;
    float tip_mass = 0.0f;
};
std::unique_ptr<DynamicObject> build_pole(phys::World& w, const PoleDesc& d, const std::string& name);

struct ConeDesc {
    vec3 base;
    float height = 0.7f;
    float radius = 0.2f;
    float mass = 3.0f;
};
std::unique_ptr<DynamicObject> build_traffic_cone(phys::World& w, const ConeDesc& d, const std::string& name);

// Truss tower / scaffold made of beams (breakable), anchored at the base.
struct TowerDesc {
    vec3 base;
    float width = 2.0f;
    float height = 10.0f;
    int levels = 5;
    float mass = 800.0f;
    BeamParams beams;
    MaterialPtr mat;
};
std::unique_ptr<DynamicObject> build_tower(phys::World& w, const TowerDesc& d, const std::string& name);

// Net / trampoline membrane anchored on the border.
struct NetDesc {
    vec3 center;
    float size = 6.0f;
    int n = 12;
    float mass = 60.0f;
    BeamParams beams;
    MaterialPtr mat;
};
std::unique_ptr<DynamicObject> build_net(phys::World& w, const NetDesc& d, const std::string& name);

// Cloth: grid of nodes with stretch / shear / bend links (tension only), can tear.
struct ClothDesc {
    vec3 origin;             // top-left corner
    vec3 u{1, 0, 0};         // width direction
    vec3 v{0, -1, 0};        // height direction (hanging down)
    float width = 3.0f, height = 2.5f;
    int nu = 32, nv = 26;
    float mass = 6.0f;
    float stretch_k = 4e4f;
    float tear_strain = 0.25f;  // links tear when stretched by this fraction
    int pin = 0;             // 0 = top edge pinned, 1 = top corners only, 2 = all four corners, 3 = all edges (a drum),
                             // 4 = top and side edges (a curtain in a gate)
    float damping = 0.05f;   // link damping as a fraction of critical (woven fabric is lossy: ~0.5)
    MaterialPtr mat;
};
std::unique_ptr<DynamicObject> build_cloth(phys::World& w, const ClothDesc& d, const std::string& name);

// Sheet of triangle elements (phys::Shell): one node layer, a grid of right isosceles triangles with alternating
// diagonals. Bending comes from hinges between neighbouring triangles. Overloaded triangles are bisected into
// smaller ones of the same shape down to mat.max_level / mat.min_edge, then cracks open between them (nodes are
// duplicated): damage gives finer pieces, never holes.
struct SheetDesc {
    vec3 center;
    vec3 u{1, 0, 0}, v{0, 1, 0};  // sheet axes (front normal = u x v)
    float width = 2.0f, height = 2.0f;
    int nu = 14, nv = 14;          // nodes per side (square cells keep the triangles right isosceles)
    float mass = 50.0f;
    float kg_m2 = 0;               // > 0: mass per area instead of `mass` (a shape's area is not width x height)
    float thickness = 0.02f;       // drawn thickness
    int clamp = 0;                 // 0 free, 1 all edges fixed, 2 top + sides (a panel in a gate), 3 top edge only, 4 two top corners
    phys::SheetShape shape = phys::SheetShape::Rect; // (see phys::SheetMeshDesc)
    float hole = 0.45f;            // ring: inner / outer size
    float curve = 0;               // > 0: bent into a cylinder of this radius around v
    float dome = 0;                // > 0: a spherical cap of this radius
    phys::ShellMaterial mat;
    MaterialPtr visual;
    float uv_scale = 1.0f;         // texture repeats over the sheet
    uint32_t seed = 1;             // brittle scatter
};
std::unique_ptr<DynamicObject> build_sheet(phys::World& w, const SheetDesc& d, const std::string& name);

// A steel drum (a 200 l oil barrel: 572 mm across, 880 mm tall, 1 mm wall): its wall and its two ends are one sheet of
// triangle elements, the rolling hoops pressed out of the wall; optionally (frame_rings) the chimes (the rolled seams where
// the ends meet the wall) and the hoops also as rings of frame elements (FEM) the sheet hangs on. Axis along rot * y,
// centred on `center`.
struct BarrelDesc {
    vec3 center;
    quat rot;
    float radius = 0.286f, height = 0.88f;
    int segments = 20;                 // round the wall (even; 20 x 9: 480 triangles in all; 24 x 12 dents finer and costs more)
    int rows = 9;                      // along it (a multiple of 3: the hoops at a third and two thirds)
    float kg_m2 = 7.85f;               // 1.0 mm steel (16.4 kg)
    bool hoops = true;                 // the two rolling hoops: rings of the wall pressed out (hoop_depth)
    float hoop_depth = 0.008f;
    bool frame_rings = false;          // the chimes and hoops also as rings of frame elements (FEM; see build_barrel)
    float chime = 0.008f, hoop = 0.008f; // those rings' sections (m): the chime a solid rod, the hoop a tube (a tenth of it the wall)
    phys::ShellMaterial mat;
    MaterialPtr visual;
    vec3 velocity{0, 0, 0}, spin{0, 0, 0}; // initial motion (spin about the centre, rad/s)
    int min_shift = 2;                 // the sheet's short steps per substep, 2^this (its hinges' stiffness: see SoftBody)
    // imperfections (m): a flat end and a true cylinder hold any load in their plane until the steel yields (a flat
    // disc cannot start to buckle), 60 times what a real drum takes; the ends dished in, the nodes off the surface
    float lid_dish = 0.02f, imperfection = 0.003f;
};
std::unique_ptr<DynamicObject> build_barrel(phys::World& w, const BarrelDesc& d, const std::string& name);

// Shells of FEM triangle elements (phys::FemFrame::tris; phys::ShellMesher): a plate, a closed box. The material is
// one of the frame members' (Steel, Aluminium, ...); nodes where `fixed` says so are clamped (position and rotation).
struct FemPlateDesc {
    vec3 origin, du{1, 0, 0}, dv{0, 0, 1}; // the corner and the cells' edges
    int nu = 10, nv = 10;
    std::string material = "Steel";
    float thickness = 0.002f;
    float damping = -1;                    // the section's Rayleigh damping (phys::ShellSection), below 0: the default
    std::function<bool(vec3)> fixed;       // (none: free)
    MaterialPtr visual;
    vec3 velocity{0, 0, 0};
    // more of it before the nodes' masses are set (a box welded on, a load on its nodes): the body, its mesher, the section
    std::function<void(phys::SoftBody&, phys::ShellMesher&, uint16_t)> more;
};
std::unique_ptr<DynamicObject> build_fem_plate(phys::World& w, const FemPlateDesc& d, const std::string& name);
struct FemBoxDesc {
    vec3 center, size{1, 1, 1};
    quat rot;
    int n = 5;                              // cells along each edge of a face
    std::string material = "Steel";
    float thickness = 0.002f;
    MaterialPtr visual;
    vec3 velocity{0, 0, 0}, spin{0, 0, 0};
    float lid_load = 0;                     // kg on the top face's nodes (a load on its lid, as part of it)
};
std::unique_ptr<DynamicObject> build_fem_box(phys::World& w, const FemBoxDesc& d, const std::string& name);

// Thin plate (metal / plastic): two node layers + diagonals give it bending stiffness.
struct PlateDesc {
    vec3 center;
    vec3 u{1, 0, 0}, v{0, 1, 0}; // plate axes (normal = u x v)
    float width = 2.4f, height = 1.8f, thickness = 0.04f;
    int nu = 26, nv = 20;
    float mass = 160.0f;
    float k = 4e6f, damp = 60.0f;  // target stiffness (clamped by the explicit stability budget)
    float yield_strain = 0.01f;    // plastic yield (metal ~1%)
    float break_strain = 0.5f;     // fracture
    bool clamp_edges = true;       // border nodes fixed (plate held in a frame)
    bool hang_top = false;         // only the top edge (last row along v) fixed: a panel hanging from a bar
    bool clamp_bottom = true;      // with clamp_edges: false leaves the bottom edge (first row along v) free (a gate)
    bool ductile = true;           // metal: stretches instead of tearing
    MaterialPtr mat;
};
std::unique_ptr<DynamicObject> build_plate(phys::World& w, const PlateDesc& d, const std::string& name);

// Rally barrier tape: wooden stakes (orientation joints: bend, then snap) with a red/white tape strip tied to
// their tops. The tape is a two-row strip of tension-only links: it sags, flutters in the wind and tears.
struct TapeLineDesc {
    std::vector<vec3> posts;      // stake foot points on the ground, in order
    float height = 0.95f;         // top edge of the tape
    float tape_width = 0.075f;
    float node_spacing = 0.4f;    // tape columns along the line
    float sag = 0.05f;            // mid-span sag of the authored tape
    float stake_radius = 0.018f;
    float stake_mass = 0.35f;
    float stake_k_ang = 80.0f;    // N m / rad: a thin stake pushed into the soil (it is the soil that gives)
    float stake_yield_deg = 3.0f;
    float stake_break_torque = 5.0f; // a nudge knocks it over
    float tape_node_mass = 0.006f;
    float tear_strain = 0.18f;    // thin tape: tears long before it could hold a car
    MaterialPtr tape_mat, stake_mat;
};
std::unique_ptr<DynamicObject> build_tape_line(phys::World& w, const TapeLineDesc& d, const std::string& name);

// Solid of revolution made of node rings (hay bales, haystacks). profile: (position along the axis, radius);
// a radius of 0 is a single apex node. Plastic, heavily damped beams: dents stay, the body does not bounce.
struct LatheDesc {
    vec3 base;                    // point on the axis where profile position 0 lies
    vec3 axis{0, 1, 0};
    std::vector<vec2> profile;
    int segments = 12;
    float mass = 300.0f;
    BeamParams beams{3e5f, 200.0f, 8000.0f, 1e5f, 0.3f};
    MaterialPtr side_mat, cap_mat; // cap_mat: flat end discs (radius > 0 at the first / last ring)
    float u_repeat = 2.0f;        // side texture repeats around
    float v_scale = 1.0f;         // side texture repeats per metre along the axis
    float friction = 1.0f;
};
std::unique_ptr<DynamicObject> build_lathe(phys::World& w, const LatheDesc& d, const std::string& name);
// round straw bale (lying on its side: horizontal axis), radius / width in metres
std::unique_ptr<DynamicObject> build_round_bale(phys::World& w, vec3 center, float yaw_deg, const std::string& name, float radius = 0.75f,
                                                float width = 1.2f, float mass = 280.0f);
// round bale standing on one flat end
std::unique_ptr<DynamicObject> build_standing_bale(phys::World& w, vec3 base, float yaw_deg, float radius, float height, const std::string& name,
                                                   float mass = 280.0f);
// traditional haystack (dome) standing on the ground
std::unique_ptr<DynamicObject> build_haystack(phys::World& w, vec3 base, float height, float radius, const std::string& name, float mass = 700.0f);

// Prop with its own rigid meshes (signs, banners, boards of an imported stage) on a light box body.
struct MeshPropDesc {
    mat4 xform;              // mesh space -> world
    vec3 hull_min, hull_max; // collision box in mesh space
    float mass = 40.0f;
    std::vector<RigidVisual::Part> parts;
};
std::unique_ptr<DynamicObject> build_mesh_prop(phys::World& w, const MeshPropDesc& d, const std::string& name);

// Pre-settles anchored scenery (trees, bridges) so that the rest pose under gravity equals the
// authored pose, then puts the body to sleep.
void presettle(phys::SoftBody& b, phys::World& w, float seconds, bool compensate_sag);

} // namespace bl
