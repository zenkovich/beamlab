// Vehicle rendering: cab (submesh) triangles, flexbodies skinned to nodes, props, wheels.
// Everything is merged into one dynamic vertex buffer per vehicle with one draw per material.
#pragma once

#include "gfx/renderer.h"
#include "phys/softbody.h"
#include "vehicle/ogre_material.h"
#include "vehicle/ogre_mesh.h"
#include "vehicle/ror_def.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace bl {

class VehicleVisual {
public:
    // ---- the model editor: the graphics part by part. With split_parts (set before build) every part is drawn in
    // batches of its own: a cab submesh, a flexbody, a prop (with its steering wheel), a wheel (tyre and rim), each
    // shown as usual, hidden, highlighted (selected / under the mouse) or faint (not part of the vehicle)
    enum PartKind { PART_CAB = 0, PART_FLEX = 1, PART_PROP = 2, PART_WHEEL = 3 };
    static int part_code(int kind, int index) { return kind << 20 | index; }
    static int part_kind(int code) { return code >> 20; }
    static int part_index(int code) { return code & 0xfffff; }
    enum PartState : uint8_t { PS_NORMAL = 0, PS_HIDDEN, PS_SELECTED, PS_HOVER, PS_FAINT };
    bool split_parts = false;
    float others_alpha = 1.0f;       // the parts in PS_NORMAL: their opacity (times the draw's alpha)
    void set_part_states(std::unordered_map<int, uint8_t> states) { m_state = std::move(states); }
    // the part whose triangle a ray hits first (-1: none; hidden parts are not hit), and the distance
    int pick(vec3 ro, vec3 rd, float* t = nullptr) const;

    // Build from the definition; `b` must still be in definition space.
    void build(const ror::Document& d, const phys::SoftBody& b, std::vector<std::string>& warnings);
    // CPU skinning (thread safe per vehicle).
    void update(const phys::SoftBody& b, float dir_state);
    // GPU upload + draw submission (main thread). alpha < 1: see-through (the model editor's mockup of the graphics)
    void draw(Renderer& r, float alpha = 1.0f);
    void mark_dirty() { m_need_update = true; }
    int vertex_count() const { return (int)m_verts.size(); }
    int batch_count() const { return (int)m_batches.size(); }

private:
    struct Locator {
        uint32_t ref, nx, ny;
        vec3 c;  // position coords
        vec3 n;  // normal coords
        float lx = 0, ly = 0; // (its two edges' lengths as built: Flex::stretch)
    };
    // A vertex skinned to the frames of FEM nodes (Flex::skin): its place and its normal carried by each of the kSkinK
    // nodes of its forset nearest it - the node's position and its orientation (FemFrame::q) turning the vertex's offset
    // from it as built (its place as built: m_vert_rest, the node's: m_node_rest) - blended by weights that fall smoothly to nothing at the next nearest node's distance (an
    // embedded deformation: the mesh bends as a smooth surface between the nodes; on a locator's three nodes each vertex
    // lay in one triangle's plane, and the mesh folded in facets along the triangles' edges)
    static constexpr int kSkinK = 8;   // (the nodes that carry a vertex: with four the folds still showed as ridges)
    struct Skin {
        uint32_t n[kSkinK];
        float w[kSkinK];
        vec3 nrm;
    };
    struct Flex {
        uint32_t first, count;
        std::vector<Locator> loc;
        std::vector<Skin> skin;   // (not empty: the vertices skinned to node frames instead of the locators)
        // (a skinned mesh's normals follow its surface as it bends between the nodes: its triangles (vertices from
        // `first`), each vertex's group - the vertices at its place with its normal: a seam's two sides shade as one -
        // and the group's normal off the triangles as built; each frame the normal off the triangles now against that one
        // carried by the nodes turns the vertex's own normal. Carried by the nodes alone the light showed every node's
        // patch as a stripe)
        std::vector<uint32_t> tri, group;
        std::vector<uint32_t> gptr, gtri;   // (per group's first vertex: the triangles at the group, for a gather in parallel)
        std::vector<vec3> face0;
        // (its nodes and their places as built: while they stand to each other as built - a part not bent, the whole car
        // before its first crash - the mesh moves as one rigid piece with the first of them, a vertex a matrix's product)
        std::vector<uint32_t> fnode;
        std::vector<vec3> frest;
        // (above 1: a locator's edges taken no longer than this times their length as built, no shorter than 1 over it -
        // a mesh over FEM parts that tear: a vertex whose node went off with a shard stays by its other nodes, the mesh
        // was pulled out in spikes after it. 0: as RoR, the edges as they are)
        float stretch = 0;
    };
    struct Batch {
        const Material* mat;
        int first, count;
        int part = -1;      // split_parts: the part it draws
    };
    // Rigid parts (props, steering wheels, rims) are static GPU meshes drawn with a model matrix: no per-frame
    // vertex transform or upload (detailed mods have 50k+ triangle rims).
    struct RigidMesh {
        GpuMesh gpu;
        std::vector<Vertex> verts;
        std::vector<uint32_t> idx;
        std::vector<Batch> batches; // opaque first
        bool ready = false;
    };
    struct Rigid {
        int mesh = -1;      // index into m_rmeshes
        mat4 model;
        enum Kind { PROP, STEERING_WHEEL, RIM } kind = PROP;
        int ref = 0, x = 0, y = 0;
        vec3 off;
        quat rot;           // Rz*Ry*Rx from the definition
        vec3 wheel_offset;  // steering wheel
        float wheel_angle = 160;
        int wheel = -1;     // RIM: index into body wheels
        char side = 'l';
        int part = -1;      // split_parts: the part it is
    };
    struct Tyre {
        uint32_t first;
        int wheel;          // index into body wheels
        int type;           // 0 = FlexMesh (wheels/wheels2), 1 = meshwheels generated tyre
        int rays;
        float rim_radius;
        bool wheels2;
    };
    struct CabTri {
        uint32_t a, b, c;
    };
    // a collision volume drawn as a solid (phys::CollisionVolume::color: an engine block, a gearbox): its hull's faces
    // fanned, flat shaded, placed where the volume is (or fitted to its anchors while it is not placed)
    struct Solid {
        int vol;                     // index into the body's volumes
        uint32_t first;
        std::vector<vec3> pos, nrm;  // (from the anchors' centre at rest, as the volume's points)
    };
    std::vector<Solid> m_solids;
    // a ring tyre (phys::Wheel::ring): the tyre's section swept round (kProfile points across it, two segments per
    // point of the ring), its tread pressed in and sheared as the ring's points are; the rim's two faces, their spokes
    struct RingTyre {
        int wheel;
        uint32_t tyre, rim;          // first vertices: the tyre's ((segs + 1) x kProfile), the rims' (2 x (1 + segs + 1))
        int segs;
    };
    static constexpr int kProfile = 10;
    std::vector<RingTyre> m_rings;

    MaterialPtr material(const std::string& name, bool double_sided = false);
    const OgreMesh* mesh(const std::string& file);
    void add_tris(const MaterialPtr& m, const std::vector<uint32_t>& idx);
    int rigid_mesh(const OgreMesh* om, const std::string& key, const std::string& material_override);
    uint32_t alloc(uint32_t n);

    std::string m_dir;
    MaterialLibrary m_lib;
    std::unordered_map<std::string, MaterialPtr> m_mats;
    std::unordered_map<std::string, std::unique_ptr<OgreMesh>> m_meshes;
    std::vector<std::string>* m_warn = nullptr;

    // cab
    uint32_t m_cab_first = 0, m_cab_count = 0;
    std::vector<uint32_t> m_cab_node;
    std::vector<CabTri> m_cab_tris;
    std::vector<Flex> m_flex;
    std::vector<Rigid> m_rigid;
    std::vector<Tyre> m_tyres;

    std::vector<Vertex> m_verts;
    struct TriList {
        const Material* mat;
        int part;
        std::vector<uint32_t> idx;
    };
    std::vector<TriList> m_tri_lists;
    int m_cur_part = -1;             // (build: the part add_tris files triangles under)
    std::unordered_map<int, uint8_t> m_state;
    std::unordered_map<const Material*, std::unique_ptr<Material>> m_variant[3]; // selected, hover, faint copies
    const Material* variant(const Material* m, int mode, float alpha);
    const Material* part_material(const Material* m, int part, float alpha);
    std::vector<MaterialPtr> m_keep;
    std::unordered_map<const Material*, std::unique_ptr<Material>> m_ghost; // see-through copies of the materials
    const Material* ghost_of(const Material* m, float alpha);
    std::vector<Batch> m_batches;
    std::vector<std::unique_ptr<RigidMesh>> m_rmeshes;
    std::unordered_map<std::string, int> m_rmesh_index;
    std::vector<uint32_t> m_indices;
    // A mesh over FEM parts that tear (Flex::stretch): each of its vertices' node, and the indices as drawn - a triangle
    // whose vertices' nodes are no longer of one part (SoftBody::part_labels: a shard torn off, a panel in two) is left
    // out (degenerate), the mesh tears where its part did; made again when the body's parts change
    // (and one stretched to three times its size as built and 0.25 m: a panel hanging on by a corner)
    std::vector<uint32_t> m_vert_node, m_draw_indices;
    std::vector<vec3> m_vert_rest;
    std::vector<vec3> m_skin_carried, m_skin_now;   // (update's scratch: a skinned mesh's normals off its triangles)
    std::vector<mat3> m_node_rot, m_node_R;         // (the FEM nodes' orientations: per frame node, per body node)
    std::vector<vec3> m_node_rest, m_node_t;        // (the body's nodes as built; each one's offset now: p - R rest)
    std::vector<uint8_t> m_node_has;                // (a body node in the frame now)
    uint64_t m_tear_key = ~0ull;
    int m_tear_again = 0;
    bool m_idx_dirty = false;
    GpuMesh m_gpu;
    AABB m_bounds;
    bool m_gpu_ready = false;
    bool m_dirty = true;
    bool m_uploaded = false;
    bool m_need_update = true;
};

} // namespace bl
