// Node/beam soft body (Rigs of Rods style mass-spring network) + oriented "frame" nodes.
//
// A SoftBody is a set of point masses (nodes) connected by:
//   * beams      - spring/damper with plastic deformation and breaking (RoR model)
//   * shocks     - beams with separate spring/damper/bounds (suspension)
//   * joints     - 6-DOF "orientation preserving" connections: a child node keeps its full pose
//                  (position + orientation) relative to a parent frame node. Used for thin
//                  structures like trees where each segment hangs off its parent segment.
// Collision primitives: contact nodes, triangles (node triplets) and capsules (node pairs).
#pragma once

#include "core/math.h"
#include "phys/frame_fem.h"
#include "phys/shell_pattern.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace bl::phys {

constexpr float kDefaultDt = 0.0005f; // 2000 Hz like RoR
constexpr int kFrameEvery = 1;        // the FEM frames' implicit step every this many substeps: 2000 Hz (WorldSettings::frame_every)

enum NodeFlag : uint16_t {
    NF_NONE = 0,
    NF_GROUND = 1 << 0,     // collides with terrain / static world
    NF_FIXED = 1 << 1,      // immovable anchor
    NF_TYRE = 1 << 2,       // wheel tread node (tyre ground model)
    NF_RIM = 1 << 3,        // wheel rim node
    NF_CONTACTER = 1 << 4,  // collides with other bodies' triangles
    NF_FRAME = 1 << 5,      // has an orientation (frame index valid)
    NF_LOAD = 1 << 6,       // RoR 'l' load node
    NF_NO_MOUSE = 1 << 7,   // cannot be grabbed
    NF_SELF_COLL = 1 << 8,  // takes part in self collision
};

// Hot node state (32 bytes, contiguous).
struct alignas(16) Node {
    vec3 p;
    float inv_mass;
    vec3 v;
    float mass;
};

struct NodeInfo {
    uint16_t flags = NF_GROUND | NF_CONTACTER;
    uint8_t surface = 0;        // index into ground models for the node's own "material" (unused for now)
    uint8_t pad = 0;
    float friction = 1.0f;      // friction multiplier
    float radius = 0.0f;        // extra collision radius (e.g. tyre nodes use 0)
    int32_t frame = -1;         // index into frames if NF_FRAME
    int32_t ror_id = -1;        // original node id from the definition (debug)
};

enum BeamType : uint8_t { BT_NORMAL = 0, BT_ROPE = 1, BT_SUPPORT = 2 };
enum BeamFlag : uint8_t {
    BF_BROKEN = 1 << 0,
    BF_INVISIBLE = 1 << 1,
    BF_NO_DEFORM = 1 << 2,   // no plastic deformation (hydros, wheels ...)
    BF_NO_BREAK = 1 << 3,
    BF_WHEEL = 1 << 4,
    BF_HYDRO = 1 << 5,
    BF_SHOCK = 1 << 6,
};

struct Beam {
    uint32_t a, b;
    float L;            // current rest length (changes with plastic deformation / hydros)
    float k, d;         // spring (N/m), damping (N s/m)
    float max_pos;      // compressive yield force (> 0)
    float max_neg;      // tensile yield force (< 0)
    float strength;     // break force
    float plastic;      // plastic coefficient (0 = default RoR behaviour)
    float stress;       // last force (positive = compression), for visualization
    float L0;           // initial rest length (reference for deformation visualization / hydros)
    float support_limit;// support beams break when stretched beyond L * support_limit
    uint8_t type;
    uint8_t flags;
    uint16_t group;     // user group id (e.g. detacher / visual group)
};

// Suspension element with its own force model (RoR SHOCK1 / shocks2 / shocks3).
struct Shock {
    uint32_t beam = 0;          // index into beams (flagged BF_SHOCK, skipped by the plain beam loop)
    uint8_t type = 1;           // 1 = shocks / bounded wheel beams, 2 = shocks2, 3 = shocks3
    bool soft_bump = false;     // shocks2 's'
    float spring = 0, damp = 0; // type 1 (in-bounds)
    float spring_in = 0, damp_in = 0, prog_spring_in = 0, prog_damp_in = 0;
    float spring_out = 0, damp_out = 0, prog_spring_out = 0, prog_damp_out = 0;
    float damp_in_slow = 1, split_in = 1, damp_in_fast = 1, damp_out_slow = 1, split_out = 1, damp_out_fast = 1;
    float short_bound = 0, long_bound = 0; // fractions of L
    float bound_spring = 9e6f, bound_damp = 12000; // target stiffness at the bump stops
    // the members holding each end's frame node when the frame first tore (0: not a frame node, 255: not yet): torn
    // down to one or none, the shock lets go (FemFrame::process_events)
    uint8_t seat[2] = {255, 255};
};

// Oriented node: orientation + angular velocity for frame-based joints.
struct Frame {
    uint32_t node;
    quat q;
    vec3 w;             // angular velocity (world)
    float inv_inertia;  // isotropic
    vec3 torque;
    // a kinematic frame follows the triad of its node and two reference nodes (x towards ref_x, y towards ref_y)
    // instead of integrating torques: the orientation of a node of a beam structure (the truck format's joints)
    int32_t ref_x = -1, ref_y = -1;
};

// Orientation preserving joint (6-DOF spring) from a parent frame to a child node (optionally a frame).
struct Joint {
    uint32_t parent_frame;
    uint32_t child_node;
    int32_t child_frame;    // -1 if the child has no orientation
    vec3 local_offset;      // child position in parent frame (rest)
    quat rel_rot;           // child orientation relative to parent (rest)
    float k_lin, d_lin;     // linear spring/damper
    float k_ang, d_ang;     // angular spring/damper (N m / rad)
    // Failure model (like RoR beams: yield first, then break after enough plastic strain, so that
    // almost no elastic energy is released when a joint fails):
    float break_force;      // linear yield force (N); 0 = unbreakable
    float break_torque;     // bending yield torque (N m); 0 = unbreakable
    float yield_angle;      // extra plastic bend limit (rad); 0 = none
    float max_stretch = 0.12f; // plastic stretch that tears the joint (m)
    float max_bend = 0.8f;     // plastic bend that snaps the joint (rad)
    float plastic_lin = 0;     // accumulated plastic strain
    float plastic_ang = 0;
    float radius;           // visual/collision radius of the segment parent->child
    float stress;           // normalized load (0..1) for visualization
    bool broken = false;
};

struct Triangle {
    uint32_t a, b, c;
    uint8_t surface = 0;
    // false: a hull triangle (cab option `h`), one-sided and solid: its winding faces out, a node up to the body's
    // hull_depth behind it over its face is pushed back out along its normal (a thin two-sided surface pushes a node
    // that got past its middle on through: two cars' frames went through each other)
    bool two_sided = true;
    bool torn = false;     // stretched far beyond its rest size (material torn apart): no collision
    float rest_edge2 = 0;  // longest rest edge, squared
};

// A surface triangle counts as torn once an edge exceeds this multiple of its longest rest edge.
// Speed limit of a node (m/s): nothing in the simulation moves faster (projectiles top out at 150 m/s, a light node
// they hit bounces off at up to twice that); a node beyond it is a numerical runaway and is clamped, the body is then
// repaired locally (World::repair_nodes).
constexpr float kMaxNodeSpeed = 400.0f;
constexpr float kTornStretch2 = 2.5f * 2.5f;
inline float max_edge2(vec3 a, vec3 b, vec3 c) { return std::max(length2(b - a), std::max(length2(c - b), length2(a - c))); }
// the point of triangle a b c nearest p, and its barycentric weights for a, b, c (Ericson, Real-Time Collision
// Detection 5.1.5)
inline vec3 closest_on_triangle(vec3 p, vec3 a, vec3 b, vec3 c, vec3& bary) {
    vec3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) { bary = {1, 0, 0}; return a; }
    vec3 bp = p - b;
    float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) { bary = {0, 1, 0}; return b; }
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        float v = d1 / (d1 - d3);
        bary = {1 - v, v, 0};
        return a + ab * v;
    }
    vec3 cp = p - c;
    float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) { bary = {0, 0, 1}; return c; }
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        float w = d2 / (d2 - d6);
        bary = {1 - w, 0, w};
        return a + ac * w;
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        bary = {0, 1 - w, w};
        return b + (c - b) * w;
    }
    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom, w = vc * denom;
    bary = {1 - v - w, v, w};
    return a + ab * v + ac * w;
}

struct Capsule {
    uint32_t a, b;
    float radius;
    int32_t joint = -1; // joint that forms this capsule (broken joint => capsule disabled)
};

// A collision volume (SoftBody::volumes): a heuristic for what fills a car - the engine and gearbox in the engine bay,
// the seats and the occupants in the cabin, the load in the trunk. A convex hull (its vertices given in the vehicle's
// space) riding on anchor nodes of the frame: placed every substep by their best rigid fit (the rotation by Mueller's
// iteration from the last one). Other bodies' nodes, balls and volumes inside it are pushed out, so are the nodes of
// the body's own parts on mounts (the hood, doors, lid, fenders, bumpers: `parts`) but not the frame it rides on, and it
// out of the static world (its vertices; a pole through its faces); what it takes goes to its anchors as the force and
// the moment it makes (spread over them as over a rigid body). Once its anchors are crushed or torn out of their shape
// past break_rms (their fit's residual) it is off for good: a cut car's halves are left to their own shells.
struct CollisionVolume {
    std::string name;
    std::vector<uint32_t> anchors;
    std::vector<vec3> rest;                  // the anchors from their centre at rest
    std::vector<vec3> verts;                 // the hull's vertices from the anchors' centre at rest
    std::vector<vec4> planes;                // its faces at rest: n (outwards), w = d; inside n . x <= d
    std::vector<std::vector<uint8_t>> faces; // (each face's vertices in order round it: the debug view)
    float break_rms = 0.12f;
    // a crush force (N; 0: none): the sum of its contacts' forces past it for kCrushTime and it is off - a bumper's
    // reinforcement is crushed and gives way to the beam under it (a rigid bar met the other car's at 28 m/s: 22 MN at a
    // point, all of it on its six anchors)
    float break_force = 0;
    static constexpr float kCrushTime = 0.003f;
    float load = 0, crush = 0;               // (its contacts' forces this substep; the load past its crush force over time)
    bool broken = false;
    std::vector<uint32_t> parts;             // the body's own nodes it holds off (SoftBody::find_volume_parts)
    vec3 color{-1, -1, -1};                  // (display) drawn as a solid of this colour (an engine block); negative: not drawn
    bool tyre = false;                       // a ring tyre's (Wheel::ring): other bodies only - its tyre meets the static world
    // placed (SoftBody::place_volumes)
    bool placed = false;
    quat q;
    vec3 c, v, w;                            // centre, its velocity, the angular velocity (world)
    float mass = 0, rms = 0;
    int hits = 0;                            // (contacts so far, substeps x points: the diagnostics)
    float peak = 0;                          // (the largest force it took at a point, N)
    mat3 Jinv;                               // (the anchors' spread about the centre, inverted: the moment's share)
    std::vector<vec3> cur;                   // the anchors from the centre now
    std::vector<vec3> wverts;
    std::vector<vec4> wplanes;
    vec3 mn, mx;
    vec3 vel_at(vec3 p) const { return v + cross(w, p - c); }
    // how far p is outside it: the largest n . p - d over its faces (negative inside), face its index
    float depth(vec3 p, int& face) const {
        float best = -1e30f;
        face = 0;
        for (size_t k = 0; k < wplanes.size(); k++) {
            const float s = wplanes[k].x * p.x + wplanes[k].y * p.y + wplanes[k].z * p.z - wplanes[k].w;
            if (s > best) best = s, face = (int)k;
        }
        return best;
    }
    // ... for the inside test: done at the first face it is outside of (then a positive value, not the largest) - a
    // fitted zone has up to 60 faces, most points tested are outside
    float depth_in(vec3 p, int& face) const {
        float best = -1e30f;
        face = 0;
        for (size_t k = 0; k < wplanes.size(); k++) {
            const float s = wplanes[k].x * p.x + wplanes[k].y * p.y + wplanes[k].z * p.z - wplanes[k].w;
            if (s > best) {
                best = s, face = (int)k;
                if (s > 0) return s;
            }
        }
        return best;
    }
};

// The convex hull of up to 255 points (a collision volume's; the model editor draws it): its faces' planes (n outwards,
// w = d: inside n . x <= d) and each face's points in order round it. False if they span no volume (under 4 faces).
bool convex_hull(const std::vector<vec3>& points, std::vector<vec4>& planes, std::vector<std::vector<uint8_t>>& faces);

// Triangle element (sheets): three nodes held by the triangle's own three edge springs (solved like beams, with
// plastic yield), bending hinges to the neighbours across its edges, drawn and collided as one face. It never
// breaks as a whole:
//   * overloaded, it is bisected through its longest edge into two halves of the same shape (right isosceles
//     triangles of a grid stay right isosceles: two bisections give the 4-triangle grid of the parent); the
//     neighbour across that edge is split at the same midpoint, so the mesh stays conforming. New nodes get the
//     interpolated position, velocity and mass; rest lengths, plastic state and texture follow the parent.
//   * at the smallest size (ShellMaterial::max_level / min_edge) a crack opens instead: a node shared by several
//     triangles is duplicated along the crack line, both copies inherit its position and velocity.
// Damage therefore produces finer segments and cracks between them, not holes.
struct Shell {
    uint32_t n[3];          // corners, counter-clockwise seen from the front
    int32_t nb[3];          // neighbour across edge e (n[e] -> n[e+1]); -1: free edge (border or crack)
    float L[3];             // rest lengths of the edges (plastic)
    float L0[3];            // original rest lengths (fracture strain reference)
    float th0[3];           // rest dihedral angle across each edge (plastic bends), mirrored in the neighbour
    float k[3], d[3];       // edge springs
    float kb;               // hinge stiffness at this shell's size (N m / rad)
    float kscale;           // softening for the explicit step (enforce_node_budget): a light node of few small triangles
                            // at a crack edge would otherwise carry more spring than its mass takes at the short step
    float mass;             // a third of it sits on each corner
    float area0;            // rest area (hinge stiffness, degenerate-shape guard)
    float area_nom;         // the authored triangle's area / 2^level: the size its rate class steps for (no triangle
                            // may get much smaller: its lighter nodes would need a shorter step)
    vec3 n0;                // rest normal: an edge between two fixed nodes is clamped against it (th0 = its rest angle)
    float flaw;             // strength factor (brittle scatter)
    float harden;           // the bend yield it gained flowing (ShellMaterial::bend_harden), rad
    float strain;           // (reference kernel of the tests only: the strain lives in ShellKernel::Aux, SoftBody::shell_strain)
    vec2 uv[3];
    uint32_t tri;           // collision triangle (same corners)
    int32_t host = -1;      // a sheet laid over a frame's triangle elements: the element it lies on (FemFrame::tris; its
                            // halves after a refinement too), -1 none. It parts with the element (SoftBody::sheet_follow)
                            // and does not crack on its own
    uint8_t level;          // bisections since the authored triangle
    uint8_t pending;        // an event is queued for this shell
    uint8_t cool;           // substeps before it may try to crack again (after a crack that could not open)
    uint8_t es[3];          // fracture pattern: strength of edge e's spring (x / 64 of the fracture strain), 64 = 1
    uint8_t hs[3];          // the same for the fold across edge e (hinge)
    uint8_t line;           // bit e: edge e lies on a pattern line (cracks follow it)
    uint8_t edges;          // bit e: edge e is the authored border; bit 3 + e: a laser cut
    uint8_t mat;            // its material: 0 SoftBody::shell_mat, k SoftBody::shell_mat_extra[k - 1] (inherited by its pieces)
    uint8_t half[8];        // the bisection history for coarsening: half[l] = the edge that is a half of the edge the
                            // level-l bisection split, bits 0-1 its index, bit 2 which of its ends is the node that
                            // bisection put in (0: n[e], 1: n[e + 1]); 255 unknown
};
constexpr uint8_t kNoHalf = 255;
inline uint8_t half_code(int edge, bool second) { return (uint8_t)(edge | (second ? 4 : 0)); }

struct alignas(16) F4 {
    float x, y, z, w;
};
struct F3 {
    float x, y, z;
};

// Overload of a shell found by the force kernel, handled by SoftBody::process_shell_events.
struct ShellEvent {
    uint32_t shell;
    uint8_t kind;   // 0 refine, 1 tension crack across edge `edge`, 2 bending crack along edge `edge`
    uint8_t edge;
};

// Hot copy of the shells for the force kernel (shell_kernel.cpp), in the order of SoftBody::shells and updated
// incrementally after every topology change (only the triangles and nodes the change touched).
//   * per triangle 72 bytes with everything an evaluation reads (in blocks of 4 triangles, field by field), 8 bytes of
//     state and the records of the bending hinges it evaluates: each hinge is evaluated by one of its two triangles, the
//     one in the faster rate class (the hinge keeps the rate the finer triangle needs); the records are in triangle
//     order (appended after topology changes, packed again by a full rebuild);
//   * no triangle writes to the nodes: it writes the forces on its corners and on the far corners of its hinges to
//     slots of its own (6 x 12 bytes), held until it is evaluated again (multi-rate). Every node then sums its slots
//     (a gather list per node): the evaluation has no write conflicts, any number of threads can share it, and the
//     sums do not depend on which thread did what (bitwise reproducible);
//   * the plastic state (rest lengths, rest angles) is written through to the Shell records, which the topology code
//     uses; the strain lives here only.
struct ShellKernel {
    struct Hot {            // 4 triangles field by field (288 B: the kernel loads each field of the 4 triangles as one
                            // vector); triangle si is lane si & 3 of block si >> 2
        uint32_t n[3][4];
        float k[3][4];
        float L[3][4];      // plastic rest lengths (mirrored in Shell::L)
        float L0[3][4];
        float brk[4];       // fracture strain of this triangle (material x flaw)
        float dk[4];        // edge damping per stiffness (d = k dk)
        float mass[4];
        uint32_t es[4];     // Shell::es of the three edges, a byte each (fracture strain factors x 64)
        uint32_t hinge[3][4]; // record of the hinge across edge e it evaluates, kNoHinge: none
        // its material's (SoftBody::shell_material): yield strain, refine at plastic strain, plastic bend, refine at a
        // fraction of the fracture strain, tension only (~0u)
        float yl[4], ry[4], by[4], rf[4];
        uint32_t tonly[4];
    };
    static constexpr uint32_t kNoHinge = 0xffffffffu;
    struct Hinge {          // 32 B (two 16-byte halves: the kernel loads 4 records and transposes them)
        uint32_t wing;      // the other triangle's corner opposite the edge
        uint32_t other;     // the other triangle | its edge << 30
        float th0;          // rest angle (mirrored in Shell::th0 of both triangles)
        float kh;           // stiffness: min(kb) le0^2 / (A1 + A2)
        float c1, c2;       // fade of squashed triangles: hinge_fade_c(le0, area0) of the lower / higher index triangle
        float lim;          // brittle fold limit: bend_break x the weaker flaw
        uint32_t meta;      // this triangle's edge (bits 0-1), swap (2), rev (3), level of the finer triangle (4-7)
    };
    struct Aux {            // 8 B
        float strain;       // largest edge strain relative to the fracture strain, last evaluation (visualisation, crack order)
        uint8_t level, cool, pending, flags;
    };
    enum : uint8_t {
        kClamp = 1,         // << e: free edge between two fixed nodes, clamped against the rest plane
        kOwn = 8,           // << e: evaluates the hinge across edge e
        kRefinable = 64,    // level and edge length allow a bisection
    };
    enum : uint32_t {
        kSwap = 4,          // Hinge::meta: the hinge's lower-index triangle (x1) is the other one
        kRev = 8,           // Hinge::meta: the hinge's edge (x3 -> x4) runs against this triangle's edge
    };
    struct HingeEvent {     // overload of a hinge, decided in shell_end (it depends on the events of both triangles)
        uint32_t key;       // lower-index triangle * 3 + its edge: the reference order
        uint32_t hi;        // the other triangle
        uint8_t fold;       // 1: folded beyond the brittle limit, 0: sharper than refine_angle
    };
    std::vector<Hot> hot;           // (shells + 3) / 4 blocks
    std::vector<Hinge> hinge;       // (Hot::hinge)
    std::vector<Aux> aux;
    std::vector<F3> slot;           // 6 per shell (+1 padding): forces on its corners, then on the far wing of the hinge of each edge
    std::vector<uint32_t> gather;   // per node `stride` slot indices (the node's forces), count in gcount
    std::vector<uint8_t> gcount;
    int stride = 16;
    uint32_t version = ~0u;         // SoftBody::topo_version the arrays are for (else rebuilt from the shells)
    size_t built_shells = 0, built_nodes = 0;
    std::vector<uint32_t> dirty_shells, dirty_nodes; // touched by topology operations since the last update
    // shells and the hinges they evaluate per rate class (counted again after a topology change): the work counters
    int class_shells[3] = {0, 0, 0}, class_hinges[3] = {0, 0, 0};
    uint32_t class_version = ~0u;
    size_t class_n = 0;
    // evaluation in progress (shell_begin .. shell_end)
    struct Pass {
        bool due[3] = {false, false, false};
        float dt[3] = {0, 0, 0};
        int maxc = 0;
        int cmin = 0;       // the lowest rate class this body evaluates (SoftBody::shell_min_shift)
        vec3 vmean;
        bool budget_left = false, deform = false, brk = false;
        int chunks = 0;
    } pass;
    std::vector<std::vector<ShellEvent>> ev_edge;           // per chunk
    std::vector<std::vector<HingeEvent>> ev_hinge;          // per chunk
};

struct ShellMaterial {
    float k = 1e5f;             // edge stiffness (N/m) of the shortest edge of a triangle (longer edges k * Lmin / L); the
                                // same at every size, so refinement keeps the sheet's stiffness
    float damp = 50.0f;         // edge damping (N s/m) at the authored size
    float yield = 1e9f;         // plastic yield strain, in plane
    float brk = 1e9f;           // fracture strain, total (from the original length)
    float refine = 0.5f;        // refine above this fraction of the fracture strain ...
    float refine_yield = 1e9f;  // ... or above this plastic strain (ductile sheets: the dented zone gets detail)
    float flaw = 0;             // brittle: random strength spread (+-)
    bool tension_only = false;  // fabric: edges go slack in compression
    float bend = 0;             // hinge stiffness (N m / rad), capped by the stability budget
    float bend_damp = 2.0f;     // hinge damping in substeps of stiffness
    float kg_m2 = -1.0f;        // areal density (kg / m2) of its triangles; -1: finalize_shells' argument
    float aero = 1.0f;          // air drag on the faces (multiplier of 0.5 rho Cd A v^2, Cd 1.2): flaps and membranes
                                // settle, light sheets flutter down
    float flex_damp = 2.0f;     // damping of the faces' motion across the sheet relative to the body (1/s): the internal
                                // friction of bending; hinge damping only reaches the fast modes, a swinging flap or a
                                // drumming membrane would go on for minutes
    float bend_yield = 1e9f;    // plastic bend (rad): beyond it the rest angle follows
    // Hardening: the bend yield grows by this much of every plastic bend, up to bend_harden_max times itself. Metal
    // work-hardens; perfectly plastic, a dented drum's hinges flowed back and forth for good against its membrane (the
    // rest angles they left could not all be met in its plane), and it trembled and turned on the ground
    float bend_harden = 0, bend_harden_max = 3.0f;
    float bend_break = 1e9f;    // brittle: a sharper bend cracks the sheet along that edge (rad)
    float refine_angle = 1e9f;  // refine where the sheet folds sharper than this (rad)
    int crack_rate = 2;         // cracks per substep: the strongest overloads, crack tips first (a crack runs instead of
                                // the whole loaded zone crumbling at once)
    int min_piece = 3;          // no crack may cut off a piece of fewer triangles (no dust; ductile sheets tear in flaps)
    float refine_budget = 0;    // at most this many times the authored triangle count (then cracks open at that size);
                                // 0: 2^max_level, as many as the finest level everywhere (a budget spent by one impact
                                // left the next impacts on the sheet without refinement)
    int max_level = 4;          // bisections (4: edges a quarter, area 1/16 of the authored triangles)
    float min_edge = 0.0f;      // no triangle edge shorter than this (m)
    // fracture pattern (shell_pattern.h): where the cracks run
    ShellPattern pattern = ShellPattern::None;
    float pattern_speed = 4.0f;   // a contact this fast (m/s, along the normal) lays the pattern round its point
    float pattern_size = 0.4f;    // zone radius of a 10 m/s impact (m; ~ sqrt of the speed)
    float pattern_weak = 0.35f;   // strength of the edges across a line (tension) and along it (fold)
    float pattern_strong = 2.0f;  // strength of the other edges in the zone: the pieces between the lines hold
    float grain_angle = 0.0f;     // Grain: direction of the fibres in the sheet's uv plane (rad from u)
    float grain_ratio = 3.0f;     // Grain: strength along the fibres / across them
    float vein_spacing = 0.07f;   // Grain: mean distance between the veins (m)
    bool rigid_pieces = true;     // a piece cracked off becomes a rigid body (SoftBody::make_rigid): it neither bends nor
                                  // cracks further, and costs its nodes' contacts only (a rag of cloth stays soft)
    // A metal sheet's own in-plane stiffness (E t: 2e8 N/m for 1 mm of steel) is thousands of times what the explicit
    // step lets the edge springs have on grams of node: such a sheet is a paper bag (a drum dropped a metre lay flat).
    // > 0: the membrane's yield force per unit width (N/m, sigma_y t): after each short step every edge is put back
    // within the yield strain of its rest length (a projection: stiff at any step), by at most the correction this
    // force makes in the step; what is left the edge flows plastically (SoftBody::project_membrane). Bending stays
    // with the hinges: a sheet folds without stretching, as thin metal dents.
    float membrane = 0;
};

// Soft-body wheel (RoR): torque is applied as tangential forces on the tyre ring nodes.
struct Wheel {
    uint32_t axle0 = 0, axle1 = 0;  // hub nodes (axle0.z <= axle1.z at spawn like RoR)
    std::vector<uint32_t> nodes;    // tyre contact nodes; even = outer ring (axle0 side), odd = inner ring
    std::vector<uint32_t> rim;      // rim nodes (wheels2 / flexbodywheels)
    float radius = 0.4f;
    float rim_radius = 0.0f;
    float width = 0.2f;
    int braked = 0;                 // 0 none, 1 foot+hand, 2 +skid-left, 3 +skid-right, 4 foot only
    int propulsed = 0;              // 0 none, 1 forward, 2 backward
    int arm = -1;                   // reference arm node (reaction torque)
    int near_attach = -1;           // axle node closest to the arm node
    float mass = 0;                 // sum of tyre node masses
    // per-substep inputs (set by the vehicle controller)
    float torque = 0;               // drive torque from the differentials (N m)
    float drive_torque = 0;         // (the drivetrain's torque for the whole substep: held for its later short steps)
    float brake = 0;                // available brake torque (N m)
    // state
    float speed = 0;                // tangential speed of the tread (m/s)
    float avg_speed = 0;
    float last_torque = 0;
    float last_retorque = 0;
    bool detached = false;
    int tag = -1;                   // user tag (vehicle builder: index of the wheel definition)
    // visual: tyre model type
    enum Type : uint8_t { W_WHEELS, W_WHEELS2, W_MESHWHEELS, W_MESHWHEELS2, W_FLEXBODY, W_RING } type = W_WHEELS;
    // (BeamLab) a ring tyre (the truck's `ringwheels`, World::ring_tyres): the rim a rigid round disc turning on the
    // axle - its spin and its angle its own state, no rim or tread nodes (`nodes` empty; the wheel's mass on the axle
    // nodes) - and the tyre a flexible ring over it: points round its tread (kRingRows across it) and its sidewalls,
    // pressed in by the static world (a stiffness and a damping per area of the tyre), the tread's points in contact
    // sheared along the ground (a brush: each held by its grip since it came into the patch, sliding past it). Other
    // bodies meet it through a collision volume round it (CollisionVolume::tyre)
    static constexpr int kRingRows = 3;
    bool ring = false;
    float spin = 0, angle = 0;       // rad/s about the axle (axle0 -> axle1), rad
    vec3 ref{0, 1, 0};               // (the rim's reference across the axle, carried along as the axle turns)
    float inertia = 1.0f;            // kg m2
    float k_area = 6.0e6f, c_area = 6.0e3f;   // the tyre pressed in: N/m3 (per area, per metre), N s/m3
    float k_shear = 9.0e6f, c_shear = 1.5e4f; // the tread sheared: N/m3, N s/m3
    float grip = 1.0f;               // (times the ground's friction)
    float crr = 0.012f;              // rolling resistance: its moment load x crr x radius
    int ring_n = 40;                 // points round it
    std::vector<vec3> shear;         // (each tread point's shear on the ground: ring_n x kRingRows, world)
    std::vector<float> squash;       // (per point round it: how far the tread is pressed in - the visual)
    std::vector<vec3> shift;         // (per point round it: its mean shear - the visual)
    float load = 0;                  // (the ground's push on it at the last step, N)
};

// RoR slide node: a node pulled onto the nearest point of a rail (polyline of nodes) by a stiff spring.
struct SlideNode {
    uint32_t node = 0;
    std::vector<uint32_t> rail;   // consecutive pairs form segments
    int seg = -1;                 // current segment (index of its first rail node)
    float k = 9000000.0f;
    float threshold = 0.0f;       // free play before the spring engages (m)
    float break_force = 0.0f;     // 0 = unbreakable
    bool broken = false;
};

class SoftBody;
using SubstepCallback = std::function<void(SoftBody& body, float dt)>;

// A sheet held on a frame at a point (a spot weld, a rivet, a bolt): the anchor (a frame node) holds the weighted
// centre of the sheet's nodes round the weld's node at its rest offset (turned with the anchor's frame node), a spring
// with damping; the weights fall off with the distance from the weld's node, to zero at the weld's radius, so the pull
// is spread over the sheet there instead of tearing one node out of it. Past `brk` it lets go for good.
struct Weld {
    uint32_t anchor = 0;
    uint32_t anchor2 = 0;           // a weld on a frame member between two nodes: the second, at `t` from `anchor`
    float t = 0;                    // (0: at `anchor` alone)
    int32_t slot = -1;              // the anchor's frame node (FemFrame::slot): its turn turns the offset; -1: none
    uint32_t first = 0, count = 0;  // the sheet's nodes: SoftBody::weld_nodes / weld_w [first, first + count)
    vec3 off{0, 0, 0};              // the anchor's rest offset from the nodes' weighted centre (the frame node's space)
    float k = 0, c = 0;             // stiffness N/m, damping N s/m
    float brk = 0;                  // force at which it lets go (0: never)
    bool broken = false;
};

struct BodyStats {
    int broken_beams = 0;
    int broken_joints = 0;
    int broken_welds = 0;
    float max_speed = 0;
};

class SoftBody {
public:
    std::string name;
    int id = -1;

    std::vector<Node> nodes;
    std::vector<vec3> force;
    std::vector<NodeInfo> info;
    std::vector<Beam> beams;
    std::vector<Shock> shocks;
    std::vector<Frame> frames;
    std::vector<Joint> joints;
    std::vector<Weld> welds;             // sheets held on the frame (see Weld)
    std::vector<uint32_t> weld_nodes;
    std::vector<float> weld_w;
    std::vector<Triangle> tris;
    std::vector<Capsule> capsules;
    std::vector<CollisionVolume> volumes; // (see CollisionVolume)
    bool volume_pass = false;             // (a cutting tool - the giant axe: through the volumes, it cuts)
    float contact_friction = 1.0f;        // its friction against other bodies, a factor on theirs (0.8): the giant axe's
                                          // blade 0.15
    // a volume on these anchors round the hull of these points (in the body's space as built); its index, -1: too few
    // anchors or points not spanning a volume
    int add_volume(const std::string& name, const std::vector<uint32_t>& anchors, const std::vector<vec3>& points, float break_rms);
    void place_volumes();                                  // (every substep: their fits, see CollisionVolume)
    void place_volume(size_t k);                           // (one of them: any order, side by side)
    void push_volume(CollisionVolume& cv, vec3 p, vec3 f); // a force f at p on it, onto its anchors (the force array)
    // Each volume's parts (after fem.finalize): the nodes of the frame's components held on by mounts (a part's side of
    // one) other than its anchors' - the hood, doors, lid, fenders, bumpers, not the body-in-white or the suspension -
    // and of the sheets welded to them (a door's skin on its frame), less the ones inside it as built. Returns those left
    // out (inside) over all volumes.
    int find_volume_parts();
    // a node copied (a crack's, a sheet's midpoint: id from like): the copy held off the volumes where the node is
    void copy_node_refs(uint32_t like, uint32_t id);
    // The body's nodes renumbered (map: old -> new, -1 gone): the volumes follow (one whose anchor is gone is off), and
    // the grab's nodes.
    void remap_node_refs(const std::function<int64_t(uint32_t)>& map);
    std::vector<Wheel> wheels;
    std::vector<float> wind_area;       // optional per-node drag area (m^2) for wind forces
    std::vector<SlideNode> slides;
    std::vector<Shell> shells;          // triangle elements (sheets), see Shell
    FemFrame fem;                       // frame elements (FEM beams with rigid joints), solved implicitly: frame_fem.h
    ShellMaterial shell_mat;
    ShellMaterial shell_mat_base;       // as finalized (the world applies its sheet overrides on top every frame)
    // more materials for its shells (Shell::mat k >= 1: shell_mat_extra[k - 1]); the body-wide behaviour (the fracture
    // pattern, crack rate, smallest piece, air drag, flex damping) stays shell_mat's
    std::vector<ShellMaterial> shell_mat_extra, shell_mat_extra_base;
    const ShellMaterial& shell_material(const Shell& s) const { return s.mat == 0 || s.mat > shell_mat_extra.size() ? shell_mat : shell_mat_extra[s.mat - 1]; }
    int shell_max_level() const {
        int l = shell_mat.max_level;
        for (const ShellMaterial& m : shell_mat_extra) l = std::max(l, m.max_level);
        return l;
    }

    // settings
    float collision_radius = 0.05f;     // node-vs-triangle thickness for inter-body collisions
    float ground_friction = 1.0f;
    float bounce = -1.0f;               // restitution of its static contacts (a ball's); -1: RoR's (80% of the approach taken)
    // The static contacts' push out of a penetration (RoR: a fifth of it a step as velocity, at most 2 m/s): up to
    // `contact_slop` deep none, at most `contact_push_max`. Each touch of a node kicks it off again; a stiff, barely
    // damped sheet (a steel drum) kept that up as a drumming that rocked a lying drum to and fro for good.
    float contact_push_max = 2.0f, contact_slop = 0.0f;
    // Rest damping (1/s): on the ground and slower than rest_speed, the body's motion as a whole (its centre's velocity
    // and its spin) and what its nodes move beside it are damped at this rate (fading out towards rest_speed). The
    // ground's rolling resistance and the metal's own damping, which the contacts and the hinges lack: a lying drum
    // rocked on its facets for good, a standing one crept along. 0: none.
    float rest_damp = 0, rest_speed = 0.5f;
    float rest_roll = 0;                // (m/s2) rolling resistance on the ground, at any speed
    bool fem_every_step = false;        // its frame solved in every short step (a sub-cycled sheet whose nodes the frame shares)
    int fem_left = 0;                   // substeps until its frame's next step (0: this one)
    int fem_period = 1;                 // the substeps its frame's last step spans (WorldSettings::frame_every[_lag])
    float fem_dissipation = -1.0f;      // its frame's implicit step's numerical damping (-1: the world's frame_dissipation)
    float rest_friction = 1.0f;         // the ground friction's factor at rest (a dented drum's skin trembled a little
                                        // and walked it across the ground on its friction)
    float rest_vib_damp = 0;            // (1/s) the vibration about the motion as a whole, at rest: a drum's ringing
                                        // dies away; the hinges and the projection kept it going at a few cm/s
    void apply_rest_damping(float dt, float substep);
    // its centre of mass `c` over the hull of its nodes on the ground (ground_touch), `spare` outside it counting (not
    // known, or not on the ground: true)
    bool over_support(vec3 c, float spare) const;
    // ... and a body so resting (on the ground or another, its motion as a whole slower than rest_rigid) does not flow
    // plastically unless a node moves faster than rest_plastic (an impact): its weight on a few nodes of contact, and the
    // trembling about it, nudged the hinges past their yield over and over (a ratchet: the lying drum's flat walked
    // round and it rolled along; a dented one trembled for good, its hinges flowing to and fro against its membrane)
    float rest_plastic = 3.0f, rest_rigid = 0.3f;
    bool resting = false;               // (the last frame's)
    float rigid_speed = 1e9f;           // (the last frame's speed as a whole, on the ground: the rest damping's)
    std::vector<uint32_t> part_of_;     // (part_labels' cache: each node's part, their count, the topology it was made for,
    int part_count_ = 0;                // the calls since)
    uint64_t part_key_ = ~0ull;
    int part_age_ = 0;
    float air_drag = 0.0f;              // RoR per-node drag coefficient (0.05), applied to the velocity relative to its part's
    float aero_cda = 0.0f;              // whole-body aerodynamic drag Cd*A (m^2) on the mean velocity
    bool self_collision = false;
    // Sheet contacts as spheres (phys/sphere_contacts.cpp): at the nodes, sphere_div - 1 along each edge and at the
    // triangles' centres, sphere_scale / sphere_div x the triangle's size across; with the other such bodies (instead of
    // node against triangle) and with itself where it is deformed
    bool sphere_contacts = false;
    bool sphere_target = false;         // a sheet the balls meet as spheres (with the other sheets: node against triangle)
    float sphere_ball = 0;              // > 0: a ball (a projectile) meets those sheets as one sphere of this radius at
                                        // its centre of mass, not node against triangle (a sheet's nodes caught between
                                        // its faces held a 40 kg ball stuck in a drum's wall)
    float sphere_scale = 0.4f;
    int sphere_div = 3;
    struct Sphere {
        uint32_t n[3];      // its nodes (k of them: 1 a node, 2 a point on an edge, 3 a triangle's centre)
        float w[3];         // their weights (the centre: sum w p)
        uint8_t k;
        float r;
    };
    std::vector<Sphere> spheres;         // in the order of the triangles that own them:
    std::vector<uint32_t> sphere_first;  // triangle i's are [sphere_first[i], sphere_first[i + 1])
    float sphere_rmax = 0;
    uint32_t spheres_version = ~0u;
    size_t spheres_shells = 0;
    bool can_sleep = true;
    bool allow_break = true;
    bool allow_deform = true;
    bool ductile = false;               // tensile yield doesn't weaken beams (metal sheets); RoR default = false
    bool is_static_like = false;        // anchored scenery (trees, bridges) -> sleeps aggressively
    int collision_group = 0;            // bodies with the same non-zero group don't collide
    float hull_depth = 0.3f;            // how deep behind a hull triangle (Triangle::two_sided false) a node is still pushed out (m)
    bool faces_only = false;            // its hull triangles touch nodes over their faces only, not off them by their edges (a
                                        // blade's wedge: the edges' contact at its 2 mm edge swept the nodes its cut left
                                        // beside it along ahead of it, at its speed), within face_skin of them (not the
                                        // node's radius: a cut's node 2 mm off the blade was thrown out to a centimetre)
    float face_skin = 0.002f;
    // (BeamLab) the FEM plates' mid points: each of the frame's triangle elements (FemFrame::tris) has one at its
    // centroid, collided beside its corners - against the static world, the other bodies' triangles and the collision
    // volumes - its force onto its corners, a third each; no node of its own. Between its nodes a plate let a curb's
    // edge, a pole, another car's corner or a volume's vertex through. Only the depth its corners' own contacts leave at
    // the middle counts (their share of it there): a plate flat on the ground is its corners'. BL_NOMIDS=1: none
    bool tri_mids = true;
    // (collide_volumes: the plates' mid points - a FEM triangle's or a sheet triangle's middle p, how far its corners
    // stand off it, the volumes whose parts hold all its corners, a bit each - sorted along the body's longest axis, kept
    // from substep to substep. The body's own: kept per thread, a body's island taken by another thread found another
    // thread's copy of other days, and a crash ran differently from run to run)
    struct MidPoint {
        vec3 p;
        float rad;
        uint32_t n[3];
        uint32_t idx, parts;
        bool fem;
    };
    struct MidCache {
        size_t ntris = ~size_t(0), nshells = 0, nparts = 0, nnodes = 0;
        int axis = 0;
        float max_rad = 0;
        std::vector<MidPoint> pts;
    };
    MidCache mid_cache;
    std::vector<uint8_t> mid_touch;     // (per FemFrame::tris: its mid point's contacts in the last frame - bit 0 the static
                                        // world, 1 another body's triangle, 2 a collision volume: the debug view)
    std::vector<vec3> static_pen;       // (collide_static's, for them: each node's static contact this step, its normal x depth)
    int mid_contacts = 0;               // (the mid points' contacts in the last frame: stats)
    // how far past its box the body reaches for other bodies' nodes: its hull triangles' depth, if it has any
    float hull_reach() const {
        if (hull_count < 0) {
            int n = 0;
            for (const Triangle& t : tris) n += !t.two_sided;
            hull_count = n;
        }
        return hull_count > 0 ? hull_depth : 0.0f;
    }
    mutable int hull_count = -1;        // (cached: the hull triangles in tris, -1 unknown)
    // which of `tris` are the sheet's own (Shell::tri), 1 each
    std::vector<char> shell_tri_mask() const {
        std::vector<char> m(tris.size(), 0);
        for (const Shell& sh : shells)
            if (sh.tri < tris.size()) m[sh.tri] = 1;
        return m;
    }

    // runtime
    AABB aabb;
    bool sleeping = false;
    bool wake_request = false;
    int contacter_count = -1;          // cached number of NF_CONTACTER nodes (-1 = unknown)
    int static_contacts = 0;           // static contacts during the last frame (stats)
    float rest_timer = 0, rigid_avg = 10; // (how long its motion as a whole has been slow on average: World::update_sleep)
    bool sleep_ready = false;          // (calm long enough to sleep, waiting for its neighbours: World::simulate_island)
    void fall_asleep() {
        sleeping = true, sleep_ready = false;
        for (auto& n : nodes) n.v = vec3(0);
        for (auto& f : frames) f.w = vec3(0);
        rb.v = rb.w = vec3(0);
    }
    int sphere_touches = 0;            // sphere contacts in overlap during the last frame (resting on another sheet)
    int body_contacts = 0;             // contacts with other bodies (node against triangle, capsules) during the last frame
    std::vector<const SoftBody*> touched; // (those bodies)
    void touch(const SoftBody& o) {
        body_contacts++;
        if (std::find(touched.begin(), touched.end(), &o) == touched.end()) touched.push_back(&o);
    }
    // A passive body left to itself cannot gain energy: with energy_guard, a frame in which it met no other body (nor in
    // the 15 before), was not grabbed and its motion and height (kinetic and potential energy) gained more than 2 J and a twentieth of its
    // kinetic energy has its velocities scaled back to that (the sheet and its frame rings on the same nodes, explicit
    // and implicit, a membrane projected between: now and then a drum with rings at rest went unstable within a few
    // frames and threw itself about, 100 J and more; the sheet alone gains at most 1 J in a frame, as its dents spring back)
    bool energy_guard = false;
    int guard_quiet = 0;                // (frames since it last met another body: the guard waits 15, a contact's spring
                                        // gives back what it stored for a few frames after)
    double guard_ke = 0, guard_pe = 0;  // (at the frame's start)
    int guard_cuts = 0;                 // (frames scaled back so far)
    void motion_energy(vec3 g, double& ke, double& pe) const;
    void guard_energy(vec3 g);
    float sleep_timer = 0;
    float sleep_speed = 0;
    std::vector<int16_t> node_wheel;    // wheel index of each tread node (-1: none), built on first contact              // > 0: own sleep threshold (soft bodies that keep rocking on the ground)
    float max_speed = 0;               // max node speed during the last frame
    BodyStats stats;
    SubstepCallback pre_substep;       // controller hook (vehicles), called each substep before forces
    SubstepCallback post_frame;        // called once after all substeps (per frame)

    // mouse grab: grab_node pulled to grab_target; with grab_nodes the nodes in the grab tool's sphere instead, each to
    // the target and its offset from the sphere's centre then (the region keeps its shape), the pull (grab_k) shared by
    // weight x mass - grab_w, falling off from the centre to the sphere's edge
    int grab_node = -1;
    vec3 grab_target;
    float grab_k = 0;
    float grab_scale = 1.0f;         // the pull's strength: a multiplier of grab_k and of its force cap (the grab tool's)
    std::vector<uint32_t> grab_nodes;
    std::vector<vec3> grab_offsets;
    std::vector<float> grab_w;

    // triangle elements: topology changes during the simulation
    uint32_t topo_version = 0;          // bumped whenever shells / nodes are added or re-linked (visuals rebuild)
    int shell_level = 0;                // finest shell level in the body
    bool topo_changed = false;          // set by process_shell_events (the island rebuilds its contact pairs)
    bool pieces_check = false;          // cracked this frame: loose pieces may have to become bodies (detach_pieces)
    // (the body's mean velocity, its moving nodes' momentum over their mass, at World time vcm_time: a fast blow's
    // speed on a frame's plates - World::body_velocity)
    vec3 vcm{0, 0, 0};
    double vcm_time = -1;
    size_t shell_cap = 0;               // refinement budget (number of shells)
    // Steps of its own per substep needed by the finest shells: the stable step shrinks with the triangle size.
    int dt_shift() const { return rigid ? 0 : std::max(shell_min_shift, (shell_level + 1) / 2); }
    // A sheet whose hinges must be stiffer than the substep lets them (thin metal that holds its shape: a drum): its
    // short steps are at least 2^this per substep and every triangle is evaluated at least that often (0..2)
    int shell_min_shift = 0;
    // A rigid body (a piece cracked off a sheet, ShellMaterial::rigid_pieces): the nodes move as one, placed from the
    // centre of mass and the orientation every step; the forces on them (contacts, gravity) add up to a force and a
    // torque. No springs, hinges or short steps: it costs its contacts only. make_soft returns it to the node model.
    bool rigid = false;
    struct Rigid {
        vec3 com, v, w;                 // centre of mass, its velocity, the angular velocity (world)
        quat q;                         // orientation (body -> world)
        mat3 inv_inertia;               // inverse inertia tensor in the body frame
        float mass = 0;
        std::vector<vec3> local;        // node offsets from the centre in the body frame
    } rb;
    void make_rigid();
    void make_soft();
    void rigid_step(float dt, bool touching, vec3& mn_out, vec3& mx_out, float& max_v2_out); // (touching: a static contact)
    std::vector<std::vector<uint32_t>> node_shells; // shells around each node
    std::vector<vec3> ext_force;        // scratch: contact forces held over the short steps of a sub-cycled body
    bool shell_acc_stale = true;        // topology changed: evaluate every shell at the next short step
    using ShellEvent = phys::ShellEvent;
    std::vector<ShellEvent> shell_events;
    ShellKernel shk;                    // hot copy of the shells for the force kernel
    // What the topology operations derived from what (the world's contact pairs are inherited instead of searched
    // again: a half of a triangle lies within it, a new node within its sources' reach). Cleared by the world.
    struct TopoLog {
        std::vector<std::pair<uint32_t, uint32_t>> tris;  // (new or shrunk collision triangle, the triangle it came from)
        std::vector<std::pair<uint32_t, uint32_t>> nodes; // (new node, a node it lies between / was split from)
        std::vector<std::pair<uint32_t, uint32_t>> mids;  // (new triangle element, the one it was halved from: its mid point)
        std::vector<int32_t> mid_remap;                   // (the triangle elements compacted after: old index -> new, -1 gone)
        bool overflow = false;                            // too much to inherit: search again
        void clear() {
            tris.clear();
            nodes.clear();
            mids.clear();
            mid_remap.clear();
            overflow = false;
        }
    } topo_log;
    struct ShellStats {
        int refined = 0, cracks = 0;
    } shell_stats;
    long long evals_shell = 0, evals_hinge = 0; // triangles / hinges evaluated since the world last read them
    bool is_piece = false;              // cracked off a sheet (detach_pieces)
    // topology counts of the sheet (counted again after a topology change): links between triangles, free edges by kind,
    // triangles by level
    struct TopoCounts {
        uint32_t version = ~0u;
        size_t n = 0;
        int hinges = 0, border = 0, cracks = 0, cuts = 0;
        int level[5] = {0, 0, 0, 0, 0};
    };
    const TopoCounts& topo_counts() const;
    mutable TopoCounts topo_counts_cache;
    // fracture patterns (shell_pattern.cpp)
    vec2 shell_uvm{0, 0};               // metres per uv unit along u and v: the material plane (0: no patterns)
    std::vector<ShellImpact> shell_impacts;
    struct ShellHit {                   // the fastest contact of the substep at pattern_speed or more (World)
        vec2 x;                         // where, in the material plane
        float speed = 0;
        float size = 0;                 // radius of the body that hit (0: unknown)
        double time = 0;                // World::time
    } shell_hit;
    bool pattern_on() const { return shell_mat.pattern != ShellPattern::None && shell_uvm.x > 0; }
    // material-plane position of corner c of shell si / of a node (its uv in metres)
    vec2 shell_x(uint32_t si, int c) const { return shells[si].uv[c] * shell_uvm; }
    vec2 node_x(uint32_t v) const;
    // the nearest pattern line to x within max_d (which: 1 the impacts' lines, 2 the veins of wood, 3 both)
    PatternLine pattern_nearest(vec2 x, float max_d, int which = 3) const;
    // where segment xa -> xb crosses a line, as its parameter in [lo, hi] closest to 0.5; -1 if nowhere
    float pattern_cross(vec2 xa, vec2 xb, float lo, float hi, int which = 3) const;
    // where on edge xa -> xb a bisection puts its node (0.5: the midpoint): on a line the edge crosses, else a little
    // off the midpoint inside a pattern zone (h: random seed; at most `dev` off the middle)
    float pattern_split(vec2 xa, vec2 xb, uint32_t h, float dev = 0.2f) const;
    // inside the zone of an impact (or anywhere on wood): the mesh there is made irregular
    bool pattern_zone(vec2 x) const;
    uint32_t pattern_seed = 1;
    int pattern_passes = 0;             // substeps left in which the impacts' lines are refined (a level each)
    void pattern_init(uint32_t seed);   // (finalize_shells) the material plane and the codes of wood
    // es / hs / line of shell si from the pattern, and its springs
    void pattern_codes(uint32_t si);
    // records a contact on the sheet (World, serial): the fastest of the substep lays a pattern; `size`: radius of the
    // body that hit it
    void pattern_contact(uint32_t a, uint32_t b, uint32_t c, vec3 bary, float speed, float size, double time);

    // Builders ---------------------------------------------------------------
    uint32_t add_node(vec3 p, float mass, uint16_t flags = NF_GROUND | NF_CONTACTER);
    uint32_t add_beam(uint32_t a, uint32_t b, float k, float d, float strength = 1e6f, float deform = 4e5f, uint8_t type = BT_NORMAL, uint8_t flags = 0);
    uint32_t add_frame(uint32_t node, quat q, float inertia);
    quat triad_orientation(uint32_t node, int32_t ref_x, int32_t ref_y) const; // (kinematic frames, see Frame)
    uint32_t add_joint(uint32_t parent_frame, uint32_t child_node, int32_t child_frame, float k_lin, float d_lin, float k_ang, float d_ang);
    void add_triangle(uint32_t a, uint32_t b, uint32_t c, bool two_sided = true) { tris.push_back({a, b, c, 0, two_sided}); }
    // Triangle element (counter-clockwise a, b, c seen from the front); the corners' masses come from the shells
    // (finalize_shells).
    uint32_t add_shell(uint32_t a, uint32_t b, uint32_t c, vec2 ua, vec2 ub, vec2 uc);
    // After the shells are added: node masses from `areal_density` (kg/m2), rest state, neighbours, collision
    // triangles and stiffness capped by the stability budget at step `dt`. `seed` drives the brittle scatter.
    // `keep_mass`: the nodes keep the mass they have (a sheet skin on a node-beam frame: the vehicle's masses) and
    // the sheet's share is added on top; a node the sheet loses keeps its own.
    void finalize_shells(float areal_density, float dt, uint32_t seed = 1, bool keep_mass = false);
    std::vector<float> node_base_mass;  // per node, the mass not from the shells (keep_mass; empty: none)
    float base_mass(uint32_t v) const { return v < node_base_mass.size() ? node_base_mass[v] : 0.0f; }
    // a body of shells only (renumbering its nodes breaks nothing outside: pieces, coarsening, reordering)
    bool shells_only() const { return !shells.empty() && beams.empty() && joints.empty() && frames.empty() && fem.empty() && wheels.empty() && slides.empty() && capsules.empty() && node_base_mass.empty(); }

    // Recompute beam rest lengths from current positions (L = L0 = |pb - pa|).
    void finalize();
    // Explicit-integration stability: scales beam stiffness/damping so that for every node
    // sum(k)*dt^2/m <= k_budget and sum(d)*dt/m <= d_budget. Returns number of beams scaled.
    int stabilize(float dt, float k_budget = 0.6f, float d_budget = 0.5f);
    int node_count() const { return (int)nodes.size(); }
    float total_mass() const;
    vec3 center_of_mass() const;
    vec3 average_velocity() const;
    // Its two largest parts held together by anything (beams, members, triangle elements, mounts, sheets, joints, welds,
    // slide nodes' rails): their shares of its mass (0..1; a car cut in two: two halves)
    void largest_parts(float& first, float& second) const;
    // Each node's part (as largest_parts': numbered 0.., `count` of them), kept until its topology changes - looked at
    // again no more often than every 8 calls once the node count holds (a crash's tears each substep)
    const std::vector<uint32_t>& part_labels(int* count = nullptr);
    void compute_aabb();
    void translate(vec3 d);
    void set_velocity(vec3 v);
    // Rigid transform of the whole body around `pivot`.
    void transform(const quat& r, vec3 pivot, vec3 translation);
    void wake() { sleeping = false; sleep_timer = 0; rest_timer = 0, rigid_avg = 10; }

    // Simulation pieces (called by the World) ---------------------------------
    void clear_forces(vec3 gravity);
    void compute_beam_forces();
    void compute_shock_forces();
    void compute_joint_forces();
    // A weld of the sheet's node `node` on the frame node `anchor` (the nodes where they are now: its rest state): the
    // sheet's nodes within `radius` of it, reached across its triangles, weighted (1 - d / radius)^2; stiffness `k`
    // (0: the most the short step `h` allows on the nodes' effective mass, as does any given), damped to a third of
    // critical. With anchor2: the point `t` of the way from the anchor to it on the member between them holds it (the
    // pull shared in that proportion). Returns false if the node is on no triangle.
    bool add_weld(uint32_t anchor, uint32_t node, float radius, float brk, float k, float h, int anchor2 = -1, float t = 0);
    // fem_torque: the step the frame's forces are computed on (its solve takes the torques then): a weld held off its
    // anchor (a panel standing off its tube, a lamp) puts the moment of its pull on the anchor's rotation, else the
    // offset turning with the anchor pushed the sheet without the anchor feeling it (not conservative: the frame's
    // nodes spun up to the solver's clamps)
    void compute_weld_forces(bool fem_torque = false);
    // first: the substep's first short step (the drivetrain set the wheels' torque just now); a sub-cycled body (a
    // refined sheet: 2 or 4 short steps) keeps that torque for the later ones - it was zeroed after the first, and a
    // car whose panel had one dent drove at half power
    void compute_wheel_forces(float dt, bool first = true);
    void compute_slide_forces();
    // Edge springs + bending hinges of the shells; overloads are queued as events.
    // Short step `step` of `sub` (of length h) in the current substep: each shell is evaluated at the rate of its
    // level and its force held in between (see dt_shift).
    void compute_shell_forces(float h, int step = 0, int sub = 1);
    // The same in parts, for the island's threads: shell_begin (serial) returns the number of chunks of
    // kShellChunk triangles; shell_eval(chunk) for each of them (any order, any thread); shell_end (serial) queues
    // the overload events; shell_gather adds the forces held by the triangles to F for nodes [n0, n1) (any split).
    static constexpr int kShellChunk = 128;
    int shell_begin(float h, int step, int sub);
    void shell_eval(int chunk);
    void shell_end();
    void shell_gather(size_t n0, size_t n1, vec3* F) const;
    // Brings the kernel's arrays up to date with the shells (after topology changes; cheap when nothing changed).
    void shell_sync();
    // Largest edge strain of a shell at its last evaluation, relative to its fracture strain.
    float& shell_strain(uint32_t si);
    // Drops queued overload events (and their pending marks).
    void clear_shell_events();
    // Refinement and cracks queued by compute_shell_forces (after the integration). Returns true if the topology
    // changed.
    bool process_shell_events();
    // Destroy tool: shells near `p` are refined to the finest size and shattered into loose triangles.
    int shatter_shells(vec3 p, float radius);
    // The reverse of the refinement where the sheet has settled: a node put in by a bisection whose triangles are at rest
    // (little strain, nothing queued) is taken out again and the triangles around it sewn back into their parents, up
    // to `max_merges` of them; the body's arrays are compacted (the kernel rebuilds). Returns the merges made.
    int coarsen_shells(int max_merges, float quiet = 0.3f); // (quiet: strain limit as a fraction of the refine threshold)
    // The explicit stability budget of every node (sum of the springs it carries x short step^2 / mass) enforced by
    // softening the triangles round the nodes over it (after topology changes: the crack edges' light nodes).
    void enforce_node_budget(const std::vector<uint32_t>* subset = nullptr); // (subset: these nodes only, else all)
    // The membrane projection of the sheets whose material has `membrane` (after the integration of a short step h):
    // returns the edges it moved.
    int project_membrane(float h);
    int mem_moved = 0, mem_edges_out = 0; // (the last projection: edges it moved, edges it found out of their band; diagnostics)
    // in the projection's first sweep the rate of stretch of every edge that left its band in the last 64 short steps
    // taken down by this part (0..1): the membrane's damping. The soft springs inside the band and its hard ends made a
    // car's welded panels rattle for good (the floors, the door glass half a millimetre a frame as it stood, the nodes'
    // speeds up to a metre a second between); a velocity projection, stable at any step, it leaves the sheet's turning
    // and moving whole alone
    float membrane_damp = 0;
    std::vector<uint8_t> ground_touch;  // (a projected sheet's nodes on the ground, collide_static: bit 0 in this short
                                        // step, bit 1 in this frame)
    struct MemEdge {                    // (project_membrane's list: each edge once, its one or two triangles' slots)
        uint32_t a, b, shell;
        uint32_t e : 24;
        uint32_t hot : 8;                    // (short steps left to damp it: it left its band lately)
        uint32_t shell2 = UINT32_MAX, e2 = 0; // (the triangle across it, if any: its copy of L follows)
        float L, band, force;           // plastic rest length, yield strain x L0, the strip's yield force (N)
    };
    std::vector<MemEdge> mem_edges;
    uint32_t mem_version = ~0u;
    size_t mem_shells = 0;
    // The explicit step's budget of a node, shared between its edge springs and its hinges (sum k dt^2 / m): a sheet
    // whose membrane is projected needs little of its springs and takes the rest for its hinges (a metal that must
    // hold its shape in bending)
    bool membrane_projected() const { return shell_mat.membrane > 0; }
    float edge_budget() const { return membrane_projected() ? 0.15f : 0.45f; }
    float hinge_budget() const { return membrane_projected() ? 0.5f : 0.2f; }
    double refine_time = -1e9;          // World::time of the last refinement or crack (coarsening waits after it)
    int topo_seen = 0;                  // shell_stats.refined + cracks the world last saw (to set refine_time)
    int refine_left = 1 << 30;          // bisections this body may still make this frame (World::refine_per_frame)
    // A pattern round the point of impact x (material plane) at `speed` by a body of radius `size` (0: unknown); false
    // if one is close already.
    bool add_impact(vec2 x, float speed, float size = 0, double time = -1e9);
    // Laser: cuts the sheet where the sector swept by a ray from `o` turning from direction d0 to d1 (within `range`)
    // passes through it. The triangles it crosses are refined to the finest size, the links between triangles on the two
    // sides are cut there and the nodes on the cut are split (no impulse: the parts only come apart). Returns the number
    // of links cut.
    int cut_shells(vec3 o, vec3 d0, vec3 d1, float range);
    // A shell on a live triangle element of the frame (Shell::host): it parts with it, it does not crack on its own
    bool on_plate(const Shell& s) const { return s.host >= 0 && (size_t)s.host < fem.tris.size() && !fem.tris[s.host].broken; }
    // The sheet follows a tear of the triangle elements under it (FemFrame::detach_tris: node v copied to c for the
    // elements `moved`): their shells take c, the links between shells on elements no longer joined along their edge
    // are cut, and the sheet's own nodes on such an edge (a refinement's midpoints) get a copy for each side. The masses
    // of v and c: their own (node_base_mass) and their shells' thirds.
    void sheet_follow(uint32_t v, uint32_t c, const std::vector<uint32_t>& moved);
    // ... and their bisection (FemFrame::refine_tri: element `host`'s edge (a, c) halved at node m, t from a, the half at
    // a now element ha, the one at c hc): its shells there halved the same way, each on its half, linked to the halves
    // across the edge once those are split too. A shell on an element is refined only so (no refinement of its own).
    void sheet_bisect(int32_t host, uint32_t a, uint32_t c, uint32_t m, float t, int32_t ha, int32_t hc);
    // Pieces of a cracked sheet that hang together with nothing else become bodies of their own (islands, sleep and
    // bounds of their own): every anchored piece stays (a free sheet keeps its biggest one). Only for bodies made of
    // shells alone. Returns the number of new bodies appended to `out`.
    int detach_pieces(std::vector<std::unique_ptr<SoftBody>>& out);
    // Renumbers the shells along a Morton curve of their centroids and the nodes in the order the shells first touch
    // them: neighbouring triangles and their corners end up next to each other in memory (the force kernel's loads and
    // its per-node gather hit the same cache lines). Topology changes append new triangles and nodes at the end; the
    // world renumbers a body again once a fifth of it is new (between frames: node and triangle indices change). Only
    // for bodies made of shells alone. Returns false if nothing was done.
    bool reorder_shells();
    size_t ordered_shells = 0;          // shell count at the last reorder_shells
    bool keep_node_order = false;       // others hold its node numbers (a vehicle's definition): never reordered
    // Sum of tangential tread speeds of propelled wheels / count (m/s), updated by compute_wheel_forces.
    float wheel_speed = 0;
    float wheel_spin = 0; // rad/s average of propelled wheels
    void integrate(float dt);
    // integrate() in parts: nodes [n0, n1) (their bounds and top speed squared are merged into mn, mx, max_v2), then
    // once the frames.
    void integrate_nodes(size_t n0, size_t n1, float dt, vec3& mn, vec3& mx, float& max_v2);
    void integrate_frames(float dt);
};

} // namespace bl::phys
