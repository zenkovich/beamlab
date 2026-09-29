// Frame elements (FEM): the stiff skeleton of a structure - a car's space frame, a roll cage, a truss - as 3D beam
// finite elements between oriented nodes.
//
// A frame node has a position (the soft body's node) and an orientation with its angular velocity. A member between
// two frame nodes is a Timoshenko beam (axial, torsion, bending in two planes with shear deformation) whose ends are
// welded to the nodes' orientations: every member meeting at a node keeps its angle to the others (a rigid joint)
// unless its end is released (a pin). The formulation is co-rotational: the member's frame follows its chord and the
// mean of its ends' orientations, and the small deformations measured in that frame (elongation, twist, the end
// rotations against the chord) meet the linear element stiffness; so large motions of the structure are exact and
// the member's own strain is small, as in a real frame.
//
// Members are far too stiff for the explicit step of the rest of the body (a 40 x 2 mm steel tube 0.5 m long is a
// 1e8 N/m spring, its end rotations a 7e4 N m/rad one on a few grams of rotational inertia). The frame is therefore
// integrated implicitly: every short step assembles the members' tangent stiffness (B^T D B in the current frames plus
// the geometric stiffness of tension) into one sparse system over the frame nodes' 6 degrees of freedom and solves it
// with a block Cholesky factorization (6 x 6 blocks, minimum degree ordering, double precision):
//
//     (M + (theta h^2 + beta h) K + theta h^2 Kg) dv = h f - (theta_d h^2 + beta h) K v
//
// f holds every force on the frame nodes at the start of the step (the members' own, the beams, shells, contacts and
// gravity of the explicit part), v the nodes' velocities (linear and angular), Kg the geometric stiffness of tension.
// The positions then move as all the body's nodes do (x += h v'), so the explicit elements on frame nodes stay as
// stable as elsewhere. For a linear oscillator the step's amplification has the determinant
// (1 + (theta - theta_d) a) / (1 + theta a), a = (w h)^2: with theta_d = 0 it keeps the energy exactly (only the
// frequencies far above 1 / (h sqrt(theta)) are lowered), theta_d > 0 damps what rings at the step's own rate; it is
// unconditionally stable for theta >= 1/4 + theta_d / 2. beta is the members' stiffness-proportional (Rayleigh)
// material damping. The new velocities go back to the body's integrator as the force m dv / h; the orientations are
// advanced here.
//
// A member's end joins its node in one of several ways (FrameJoint): welded, a ball joint, a hinge swinging in the
// member's vertical or horizontal plane, a swivel turning about the member's axis, or an elastic joint (a rotational
// spring of the section's joint stiffness). The freed rotations are condensed out of the element's stiffness (its
// end flexibilities added to the member's own and inverted back), so a released end takes no moment about them.
//
// Past its yield a member forms plastic hinges at its ends (the moment held at the plastic moment, the rotation kept
// when unloaded), yields in tension and buckles (Euler) or yields in compression. The tangent of a yielding hinge is
// that of a pin, so the implicit step does not fight the plastic flow. Nothing is removed when it fails, as with the
// sheets' triangles: a member where a hinge forms is split in two (a node put on its bent shape, the Hermite cubic of
// its end rotations, so its elastic state carries over exactly) down to a smallest length, to bend there along a
// curve; once a hinge's plastic strain reaches the material's elongation the member is torn off that joint (its end
// gets a copy of the node: the rest of the joint keeps the original), and axial or torsional failure splits it and
// tears it at the middle. The pieces go on as members of their own.
//
// Triangle elements (FrameTri): a flat shell between three frame nodes - a car's body panel, a plate, a box's wall -
// in the same implicit system as the members. The element is co-rotational as the members are: its frame follows
// the triangle (the normal, and the in-plane turning that best fits its rest shape onto the current one), and in that
// frame the small deformations meet the linear stiffness of a thin shell: the membrane (a constant strain triangle),
// the bending (the Discrete Kirchhoff Triangle of Batoz: exact for constant curvature, no shear locking) and a weak
// drilling stiffness tying each corner's turning about the normal to the element's own in-plane turning (the corners
// have six degrees of freedom; without it that one is free). Past the yield the membrane flows (its rest shape
// follows the stretch, radially back to the yield stress) and the bending forms a plastic fold (its corners' rest
// rotations follow, back to the plastic moment, baked into their rest frames past a few hundredths of a radian as
// the members' hinges are); a triangle whose plastic strain reaches the material's elongation tears out of the
// shell (eroded: its corners stay, with their mass). Its mass is lumped on the corners.
#pragma once

#include "core/math.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace bl::phys {

class SoftBody;

// A material of frame members: linear elastic, then perfectly plastic past its yield stress.
struct FrameMaterial {
    const char* name;
    float E, G;        // Young's and shear modulus (Pa)
    float rho;         // density (kg/m3)
    float yield;       // yield stress (Pa)
    float elongation;  // plastic strain at fracture
};
const FrameMaterial* frame_materials(int& count);
const FrameMaterial& frame_material(const std::string& name); // (an unknown name: steel)

// Section shapes: a round tube (outer diameter, wall), a square box section (side, wall), a solid round rod
// (diameter), a solid square bar (side).
enum class FrameShape : uint8_t { Tube = 0, Box = 1, Rod = 2, Bar = 3 };
FrameShape frame_shape(const std::string& name);
const char* frame_shape_name(FrameShape s);

// The cross-section and material of frame members.
struct FrameSection {
    float E = 2.1e11f, G = 8.1e10f, rho = 7850.0f;
    float A = 0;                    // area (m2)
    float Iy = 0, Iz = 0, J = 0;    // second moments of area about the local y and z axes, torsion constant (m4)
    float As_y = 0, As_z = 0;       // shear areas along y and z (Timoshenko)
    float Np = 0, Mp = 0, Tp = 0;   // plastic axial force (N), bending moment and torque (N m); 0: stays elastic
    float max_strain = 0.2f;        // plastic stretch (strain) at which a member tears apart
    float hinge_capacity = 2.0f;    // plastic rotation a joint takes before it tears (rad, the largest reached): a ductile
                                    // weld, or a folded sheet the member stands for, bends far before it cracks
    float hardening = 0.25f;        // the plastic moment grows by this share over the first radian of a hinge's
                                    // rotation (the bend spreads to the members around instead of tearing one joint)
    float half = 0.02f;             // half the outer size (m): the drawn tube
    float damping = 2e-5f;          // stiffness-proportional damping (s)
    float axial = 1.0f;             // a factor on the axial stiffness (a ring on a projected membrane: see build_barrel)
    float joint_k = 2.0e4f;         // FJ_ELASTIC joints: rotational stiffness (N m/rad)
    float break_force = 0;          // the force at its ends (axial and shear) past which the member tears off its end a
                                    // (N; 0: never): a bolted mount, a hinge letting a door go - weaker than the frame
    float joint_damp = 0;           // a released end (ball, hinge, swivel) resists turning against its node at this
                                    // (N m s/rad): a joint's friction, a bushing's (without it a broken-off wheel swung free)
    int max_level = 1;              // a yielding member is split once (its halves are not split again)
    float min_len() const { return std::max(0.15f, 8.0f * half); } // (nor into pieces shorter than this)
    float mass_per_m() const { return rho * A; }
};
// A section of `shape` with the outer size `outer` (diameter or side, m) and the wall `wall` (m; tubes and boxes).
FrameSection make_frame_section(const std::string& material, FrameShape shape, float outer, float wall);

// The section of triangle elements: a sheet of a material and thickness (isotropic, elastic-perfectly plastic).
struct ShellSection {
    float E = 2.1e11f, nu = 0.3f, rho = 7850.0f;
    float t = 0.001f;               // thickness (m)
    float yield = 0;                // yield stress (Pa; 0: stays elastic)
    float elongation = 0.2f;        // equivalent plastic strain at which a triangle tears out
    float damping = 3e-4f;          // stiffness-proportional damping (s: 1% of critical at 10 Hz, 9% at 100; a thin plate rang
                                    // for seconds at 0.3%: a sheet's free edges flapped, a slab rocked a box off)
    float drill = 0.05f;            // the corners' turning about the normal against the element's in-plane turning: a
                                    // spring of this share of G t A per corner (weak: only so the rotation is not free)
    float mass_per_m2() const { return rho * t; }
    float D() const { return E * t * t * t / (12.0f * (1.0f - nu * nu)); } // bending stiffness (N m)
};
// A shell section of `material` (the members' materials) and thickness (m).
ShellSection make_shell_section(const std::string& material, float thickness);

struct FrameTri {
    uint32_t n[3] = {0, 0, 0};      // frame nodes (FemFrame::node), counter-clockwise about the rest normal
    uint16_t section = 0;
    bool broken = false;            // torn out (or degenerate)
    int32_t tag = -1;               // the definition's triangle (vehicles), or the caller's own
    int32_t coll = -1;              // its collision triangle in the body (SoftBody::tris; torn with it), -1: none
    float X[3][2] = {{0, 0}, {0, 0}, {0, 0}}; // rest shape: the corners in the element's plane, from the centroid
    float X0[3][2] = {{0, 0}, {0, 0}, {0, 0}}; // the authored one (the plastic stretch is the rest shape's against it)
    float area0 = 0;                // the authored area (m2)
    float mass = 0;                 // mass it put on its corners, a third on each
    quat r0[3];                     // the element's rest frame in each corner node's frame
    vec3 th0[3] = {vec3(0), vec3(0), vec3(0)}; // plastic rest rotations of the corners (element axes, not yet baked)
    float dmg = 0;                  // plastic stretch reached (the rest shape's largest principal stretch against the
                                    // authored one, less one: the state, not the path - an element shaking at its yield
                                    // after an impact does not wear through)
    float util = 0;                 // stress against the yield (membrane or bending, the larger), for display
};

// How a member's end is joined to its node. The member's local axes: x along it, y in the vertical plane through it
// (towards the world's up; a vertical member: towards x), z across (horizontal).
enum FrameJoint : uint8_t {
    FJ_RIGID = 0,    // welded: it keeps its angle to everything at the node
    FJ_BALL = 1,     // ball joint: turns freely every way, takes no moment
    FJ_HINGE_V = 2,  // hinge swinging in the member's vertical plane (free about its z axis): a suspension arm
    FJ_HINGE_H = 3,  // hinge swinging in its horizontal plane (free about its y axis)
    FJ_SWIVEL = 4,   // turns freely about its own axis, bending held: a bearing, a steering column
    FJ_ELASTIC = 5,  // every rotation held by a spring (FrameSection::joint_k): a bushing, a bolted joint
    FJ_COUNT
};
const char* frame_joint_name(FrameJoint j);        // "rigid", "ball", "hinge_v", "hinge_h", "swivel", "elastic"
FrameJoint frame_joint(const std::string& name);   // (and "pinned": ball; unknown: rigid)

struct FrameElement {
    uint32_t a = 0, b = 0;          // frame nodes (FemFrame::node)
    uint16_t section = 0;
    uint8_t end_a = FJ_RIGID, end_b = FJ_RIGID; // FrameJoint of each end
    uint8_t level = 0;              // times split since authored
    uint8_t torn = 0;               // bit 0: end a torn off its joint, bit 1: end b (its node is the member's own copy)
    bool broken = false;            // degenerate (no length): out of the frame
    int32_t tag = -1;               // the definition's beam (vehicles), or the caller's own
    float L0 = 0;                   // rest length
    float mass = 0;                 // mass the member put on its nodes, half at each (moved along when split or torn)
    quat qa, qb;                    // the element's rest frame in the frames of its end nodes
    // plastic state: elongation, twist, rotations of the hinges at a and b about the element's y and z axes
    float up = 0, tp = 0;
    vec2 pa{0, 0}, pb{0, 0};
    // the plastic rotations turned into the rest frames qa, qb so far (a hinge's rotation past a few hundredths of a
    // radian goes there: the end rotations the element measures then stay the small elastic ones its co-rotated
    // formulation is exact for; bent through half a radian and measured as such, its forces were not the gradient of
    // its energy and pumped its fast modes: a crashed frame rang)
    vec2 ba{0, 0}, bb{0, 0};
    float bt = 0;
    float damage = 0;               // accumulated axial and torsional plastic strain
    float dmg_a = 0, dmg_b = 0;     // accumulated plastic strain of the hinges at a and b
    float overload = 0;             // a mount's overload so far: the time integral of its force over the break force, less
                                    // one (s); it lets go at FemFrame::kOverloadTime
    float util = 0;                 // load against the yield or buckling limit (1: yielding), for display
    float N = 0;                    // axial force (tension > 0), for display
};

// A part held on the frame at a distance (FemFrame::mounts): the point of node a's frame where node b stood when it was
// made (a's frame turns it) holds b there on a spring with damping (explicit), b free to turn about it: a ball joint -
// two on a line are a hinge, three or more a bolted part. It lets go when its force stands past the break force for
// FemFrame::kOverloadTime. The two nodes' turning against each other can be damped (a hinge's friction; implicit on each
// side against the other's turning at the step's start). A mount is no member: a part held by mounts alone is a
// component of the frame's own, solved apart from the rest (and beside it, in parallel).
// Its kinds (MountKind): a point (b alone, as above); a clamp - b and up to three of the part's nodes round it each held
// at its point of the anchor, so the part turns on it only as far as the springs give (a bolted flange: a part left on
// one bolt does not swing about it; with a break moment it gives FemFrame::kClampGive at it) and it lets go past the
// break force or the break moment about the nodes' middle; a
// hinge - b and a second node of the part on the hinge's line (b2) held: the part turns about that line only; a stop -
// b kept from coming nearer to a than it stood (it pushes only: a lid resting on its buffers); a strap - b kept within
// `len` times that distance of a (it pulls only: a lid's opening stay).
enum class MountKind : uint8_t { Point, Clamp, Hinge, Stop, Strap };

struct FrameMount {
    uint32_t a = 0, b = 0;          // body nodes (a: the anchor's side)
    // the point b is held at: carried by a and three frame nodes near it (their positions' affine combination, a point
    // of the body's where b stood when it was made, however far off: no turning of a node carries it - a bumper's
    // bracket 16 cm off a light node's frame spun the node and tore off at 14 kN as the car stood); a with no three
    // around it off one plane: the point of a's frame (off)
    uint32_t an[4] = {0, 0, 0, 0};
    float aw[4] = {1, 0, 0, 0};
    int na = 0;
    uint8_t members[5] = {0, 0, 0, 0, 0};   // (the members at the anchor's nodes and at b when made: a member torn off one of
                                            // them tore the mount's seat - the spring on a node left dangling flung it)
    vec3 off{0};                    // (in a's frame: from a to where b is held, when na is 0)
    float k = 0, c = 0;             // spring (N/m; 0 at the start: from the nodes' masses), damping (N s/m)
    float m_eff = 0;                // (the two sides' effective mass: the spring at most m_eff / h^2 at a frame step h)
    float brk = 0;                  // break force (N; 0: never)
    float damp = 0;                 // the turning's damping (N m s/rad)
    float overload = 0;             // (the force over the break force, less one, over time)
    float f = 0;                    // its force at the last step (N), for display
    bool made = false, broken = false;
    MountKind kind = MountKind::Point;
    uint32_t b2 = 0;                // a hinge: the part's second node on its line
    float brk_m = 0;                // a clamp: the break moment (N m; 0: never)
    float len = 1;                  // a strap: how far it lets b go, times the rest distance
    float L0 = 0;                   // (a stop's, a strap's rest distance of a and b)
    // (a clamp's and a hinge's other held nodes: bn[0] is b; their points' weights on the anchor's nodes, or their
    // offsets in a's frame; each one's spring and damping)
    int nb = 1;
    uint32_t bn[4] = {0, 0, 0, 0};
    float bw[4][4] = {};
    vec3 boff[4];
    float bk[4] = {0, 0, 0, 0}, bc[4] = {0, 0, 0, 0}, bm[4] = {0, 0, 0, 0};
    float m = 0;                    // its moment about the held nodes' middle at the last step (N m), for display
};

class FemFrame {
public:
    // a mount lets go when its force has stood this long over its break force (s: at twice it for 5 ms, at 1.5 times
    // for 10), not on a peak
    static constexpr float kOverloadTime = 0.005f;
    // a clamp mount's give: it turns the part this far (rad) at its break moment before it lets go
    static constexpr float kClampGive = 0.2f;
    // frame nodes
    std::vector<uint32_t> node;     // the body's node of each frame node
    std::vector<quat> q;            // orientation (node -> world); identity at rest in the definition's space
    std::vector<vec3> w;            // angular velocity (world, rad/s)
    std::vector<float> inertia;     // rotational inertia (kg m2, isotropic)
    std::vector<vec3> torque;       // torque on the node (N m): its members' (compute_forces) and any added from outside;
                                    // used and cleared by solve
    std::vector<vec3> member_f;     // the members' part of the node's force in b.force (compute_forces): it acts for the
                                    // whole implicit step h, the rest of b.force for the short step it was computed for
    // The frame nodes' positions in double precision: advanced with the nodes' new velocities (solve), the body's
    // float positions then set from them (sync_positions), or taken from those when something else moved a node (a
    // reset, a repair). A member is so stiff along its axis that the float's rounding would be kilonewtons of noise far
    // from the origin (500 m out a float steps 60 um: 6 kN on a 40 x 2 mm tube 0.5 m long), and the rounding of every
    // step would add up as a random walk of the nodes.
    std::vector<double> xd;         // 3 per frame node
    // Ground contacts of this step (the world's static collisions, zero: none): the contact already put the force on
    // the node that gives it the normal velocity it wants; the implicit step holds that (a stiff constraint along the
    // normal) so the members' response cannot drive the node back into the ground (light nodes chattered)
    std::vector<vec3> contact_n;
    std::vector<vec3> contact_f;    // those contacts' forces (in b.force too): not carried over the short steps
    std::vector<FrameSection> sections;
    std::vector<FrameElement> elems;
    std::vector<FrameMount> mounts;
    std::vector<ShellSection> shell_sections;
    std::vector<FrameTri> tris;
    int tris_torn = 0;              // triangles torn out so far
    int broken = 0;                 // tears so far (ends torn off their joints)
    int splits = 0;                 // members split so far
    int solve_failures = 0;         // steps whose factorization failed (the members' forces then act explicitly)
    int clamps = 0;                 // velocity changes cut to the safety limits (see solve)
    long long passes_ = 0;          // (statistics: assembled and factored systems)

    bool empty() const { return elems.empty() && tris.empty(); }
    int slot(uint32_t body_node) const { return body_node < slot_.size() ? slot_[body_node] : -1; }
    uint32_t add_node(uint32_t body_node);
    uint16_t add_section(const FrameSection& s);
    uint16_t add_shell_section(const ShellSection& s);
    // A triangle element between three body nodes (frame nodes made as needed; the mass is the caller's to put on the
    // nodes: tri_mass), `coll` its collision triangle in the body if any.
    uint32_t add_tri(uint32_t body_a, uint32_t body_b, uint32_t body_c, uint16_t section, int32_t tag = -1, int32_t coll = -1);
    float tri_mass(const FrameTri& t) const { return shell_sections[t.section].mass_per_m2() * t.area0; }
    // A member between two body nodes (their frame nodes are made as needed), with the joints at its ends.
    uint32_t add_element(uint32_t body_a, uint32_t body_b, uint16_t section, uint8_t end_a = FJ_RIGID, uint8_t end_b = FJ_RIGID, int32_t tag = -1);
    // A mount (see FrameMount) of body node b on body node a, where b is now; k 0: the most the nodes' masses take at
    // the step (a quarter of the explicit limit); damp: the turning's damping; kind and its parameter: a clamp's break
    // moment (N m), a hinge's second node, a strap's length (times the rest distance).
    uint32_t add_mount(uint32_t body_a, uint32_t body_b, float brk, float k = 0, float damp = 0, MountKind kind = MountKind::Point, float param = 0,
                       uint32_t body_b2 = 0);
    int mounts_broken = 0;          // mounts let go so far
    // components: parts of the frame joined by members (and the body's springs between their nodes), each solved on
    // its own (solve_component); mounts and springs between two of them act explicitly
    int components() const { return (int)comp_range_.size(); }
    int component_nodes(int c) const { return comp_range_[c].second - comp_range_[c].first; }
    // The component a body node's frame node is in (-1: none, or the pattern not made yet).
    int component_of(uint32_t body_node) const;
    // The latches let go: the point mounts of the parts that have hinges (a hood, a lid, a door swings free on them).
    // Returns how many.
    int release_latches(SoftBody& b);
    int component_members(int c) const { return (int)comp_elems_[c].size(); }
    int component_tris(int c) const { return (int)comp_tris_[c].size(); }
    float element_mass(const FrameElement& e) const { return sections[e.section].mass_per_m() * e.L0; }
    // With the body at rest: the members' rest lengths and frames, the nodes' rotational inertia (from the members'
    // masses) and the solver's ordering. Called by the first step if the caller does not (after adding members).
    void finalize(const SoftBody& b);
    bool ready() const { return ready_; }
    // The members' forces at the current state: added to b.force, their torques to `torque`. Plastic yield and
    // breaking happen here. In parts, for a team: begin_forces (returns the chunks of members), eval_forces for each
    // chunk (any order, in parallel: each member's own state and results), end_forces (onto the nodes, in order).
    void compute_forces(SoftBody& b);
    int begin_forces(SoftBody& b);
    void eval_forces(SoftBody& b, int chunk);
    void end_forces(SoftBody& b);
    static constexpr int kElemChunk = 64;
    static constexpr int kTriChunk = 32;
    // The implicit step of length h (after every other force on the nodes is in b.force), plus the impulses held since
    // the last one: the frame nodes' force is replaced by m dv / step, `step` being the body integrator's (theta,
    // dissipation: see above). A body stepping its sheets in short steps solves its frame once per substep (h the
    // substep): in the short steps between, hold() keeps the frame nodes moving at their new velocities and keeps the
    // forces on them for the next solve (the sheets' forces on the heavy frame nodes are stable at the substep). The
    // members' forces act for h; the other smooth forces (gravity, the sheets, the body's beams) are taken as they are
    // now for the whole of h too, and what they turn out to be in the later short steps corrects the next step (held
    // for those steps alone, one step late, they pushed on the sheets out of phase: a crashed car kept ringing, and
    // counted for h as well as held, twice over); contacts act for `step`, the later ones held.
    void solve(SoftBody& b, float h, float step, float theta, float dissipation);
    // The same in parts, for a team: solve_begin (returns the components), solve_component for each (any order, in
    // parallel), solve_end.
    int solve_begin(SoftBody& b, float h, float step, float theta, float dissipation);
    void solve_component(SoftBody& b, int c);
    void solve_end(SoftBody& b);
    void hold(SoftBody& b, float step);
    // A step between the frame's steps (WorldSettings::frame_every): the frame's response to this step's forces beyond
    // what the last step took them to be (its prediction), through that step's factorization again - the elements not
    // evaluated, assembled or factored, the contacts, the welds and the beams on the frame nodes answered by the
    // structure at once (held and taken one step late, the sheet's welds let go and a thin shell's light nodes, pushed
    // alone by a contact, tore). held_begin returns the components (0: no factorization to use: hold instead).
    int held_begin(SoftBody& b, float step);
    void held_component(SoftBody& b, int c);
    void held_end(SoftBody& b);
    std::vector<vec3> impulse;      // per frame node: the forces held over the short steps less what solve took them
                                    // to be (N s)
    // After the body's integration (a step of length h): the frame nodes' double positions and orientations advanced,
    // the float positions set from them (see xd).
    void sync_positions(SoftBody& b, float h);
    // rigid motions of the whole body
    void rotate(const quat& r);
    void set_orientation(const quat& r);  // every node (a respawn: rest orientations are the identity)
    void stop();                          // angular velocities to zero
    // Destroy tool: members passing within r of p are torn there (at a joint, off it; else split and torn at that
    // point). Returns the tears.
    int break_near(SoftBody& b, vec3 p, float r);
    // A triangle element torn out (a cut through it: the laser, the axe); finish_cuts after the last. Returns 1, 0 if
    // it was already.
    int tear_tri(SoftBody& b, uint32_t ti);
    // A member cut at t along it (0 at a): torn off the joint when that is close, else split there and torn. Then
    // finish_cuts once (the solver's pattern again, the body told of its new nodes). Returns the tears.
    int cut(SoftBody& b, uint32_t elem, float t);
    void finish_cuts(SoftBody& b, int tears);
    // Topology (after the integration): the splits and tears compute_forces queued. True if nodes were added.
    bool pending() const { return !events_.empty(); }
    bool process_events(SoftBody& b);
    // Debris: after tears, a piece of the frame that hangs on nothing else of the body (only frame members, lighter than
    // max_mass) leaves it as a body of its own, made rigid (the world's call, between frames: it costs its contacts only
    // and does not shiver at the frame's step on its light nodes); in the body its nodes are switched off and the frame
    // compacted. The large parts (a car torn in two) stay frames. Returns the new bodies.
    bool debris_check = false;      // set by a tear
    int detach_debris(SoftBody& b, std::vector<std::unique_ptr<SoftBody>>& out, float max_mass);
    // drops the broken members and the frame nodes left without members (renumbering the frame nodes)
    void compact(SoftBody& b);
    // A member split at t (0..1 from a): the new frame node, on the member's bent shape (-1: too short). A member's end
    // torn off its joint (end 0: a, 1: b): its node copied for it alone (false: nothing else at that node).
    int split(SoftBody& b, uint32_t elem, float t);
    bool tear(SoftBody& b, uint32_t elem, int end);
    int members_at(uint32_t frame_node) const;
    // the factor's size: off-diagonal blocks, block updates per factorization
    size_t factor_blocks() const { return row_.size(); }
    size_t factor_updates() const { return upd_.size(); }
    // The body renumbered its nodes (a sheet's reordering: nidx old -> new, all kept): the frame follows.
    void renumber(const SoftBody& b, const std::vector<uint32_t>& nidx);
    // The body split into parts (a sheet's pieces, SoftBody::detach_pieces: its nodes already moved, part_of[old node]
    // -1 kept, else the piece, nidx[old node] its index there): the members of a part go with it (a frame of its own,
    // their plastic state kept), the kept frame follows the renumbering. The parts are joined by the members too, so
    // no member spans two of them.
    void split_off(SoftBody& b, const std::vector<int>& part_of, const std::vector<uint32_t>& nidx, const std::vector<SoftBody*>& pieces);
    // Resets orientations or angular velocities that are not finite (numerical trouble): returns how many.
    int repair();
    // Elastic energy stored in the members (J), for checks; parts: axial, torsion, bending.
    double strain_energy(const SoftBody& b, double* parts = nullptr) const;
    // ... and in the triangles (J); parts: membrane, bending, drilling.
    double tri_energy(double* parts = nullptr) const;
    // a triangle's local stiffness (18 x 18: per corner u, v, w along the element's axes, then its rotations about them)
    // and its deformation at the current state (the same order) with the element's axes, for checks
    void tri_stiffness(uint32_t ti, double K[18][18]) const;
    bool tri_state(uint32_t ti, float d[18], vec3 axes[3]) const;

private:
    std::vector<int32_t> slot_;
    std::vector<char> welded_;      // (compute_forces: the frame nodes some member is welded to, for the joints' damping)
    float last_h_ = 0;              // (the last step's length: a mount's overload accumulates over time)
    // (the members' forces in parallel, begin_forces / eval_forces / end_forces: each member's end force and the two
    // torques on its nodes, and each chunk's events, then gathered onto the nodes in the members' order)
    struct ElemOut {
        vec3 fa, ta, tb;
        bool on = false;
    };
    std::vector<ElemOut> out_;
    struct TriOut {
        vec3 f[3], t[3];
        bool on = false;
    };
    std::vector<TriOut> tri_out_;
    bool ready_ = false;
    struct Event {
        uint32_t elem;
        uint8_t kind;   // 0 split at t, 1 tear end a, 2 tear end b, 3 split at t and tear there, 4 a triangle torn out
        float t;
    };
    std::vector<Event> events_;
    std::vector<std::vector<Event>> chunk_events_;
    void node_inertia(uint32_t fn, const SoftBody& b);
    // The body's own springs on frame nodes (beams, shocks, hydros, the wheels' and cinecam's beams): their stiffness
    // and damping join the implicit step on the frame's side (a bump stop of 9e6 N/m on a light frame node is far past
    // the explicit budget at the frame's step); a block of the pattern where both ends are frame nodes
    struct Link {
        uint32_t beam;
        int32_t fa, fb;                           // frame nodes of its ends (-1: not a frame node)
        int32_t shock;                            // its shock (-1: a plain beam)
        int block;                                // off-diagonal block of the pattern (both ends frame nodes), else -1
        uint8_t swap;
    };
    std::vector<Link> links_;
    const SoftBody* body_ = nullptr;              // (the body the links were found in)
    // tangent of the last force evaluation, per element: the axes, the length and the generalized stiffness
    struct Tangent {
        vec3 e1, e2, e3;
        float L = 1;
        float ka = 0, kt = 0;       // axial, torsion (elastic)
        float cy[3] = {0, 0, 0};    // bending about y: [aa, ab, bb] (elastic, with the joints)
        float cz[3] = {0, 0, 0};    // bending about z
        float ngeo = 0;             // tension / L: geometric stiffness across the member
        bool on = false;
        // yielding at the start of the step (bits: 1 the hinge at a, 2 at b, 4 axial, 8 torsion): the plastic tangent
        // (a pin, a soft bar) unless the step turns out to unload it (then the elastic one, solved again)
        uint8_t yield = 0, unload = 0;
        // steps left to start from the elastic tangent: a hinge that unloaded mid-flow (shaking at the yield surface after
        // an impact, a damaged frame resting at its yield load) would otherwise be solved twice every step; the elastic
        // tangent on a flowing hinge only stiffens the change of its rate a little
        uint8_t elastic_hold = 0;
        vec2 ma{0, 0}, mb{0, 0};    // the moments at the hinges and the axial force and torque: the direction of loading
        float N = 0, T = 0;
    };
    Tangent effective(const Tangent& t) const;
    std::vector<Tangent> tan_;
    // the triangles: their local stiffness (18 x 18, packed upper triangle; rebuilt when the rest shape flowed) and the
    // tangent of the last evaluation (the element's axes, its membrane tension for the geometric stiffness)
    static constexpr int kTriK = 171, kTriB = 54;
    std::vector<float> tri_k_;
    std::vector<float> tri_b_;                    // per triangle: the bending's curvatures from the corners' rotations at the
                                                  // three points (3 x 3 x 6), with the stiffness
    std::vector<uint8_t> tri_k_ok_;
    struct TriTan {
        vec3 e1, e2, e3;
        float g[3][3];              // geometric stiffness of the membrane's tension between the corners' translations
        bool on = false;
    };
    std::vector<TriTan> tri_tan_;
    void tri_build_k(uint32_t ti);
    std::vector<std::array<int, 3>> tri_block_;   // per triangle: the off-diagonal blocks of its corner pairs (01, 12, 20)
    std::vector<std::array<uint8_t, 3>> tri_swap_;
    std::vector<std::vector<uint32_t>> comp_tris_;
    void eval_tris(SoftBody& b, int chunk, std::vector<Event>& evs);
    // the sparse factorization: permuted order of the nodes, the lower block pattern of L column by column
    std::vector<int> perm_, iperm_;
    std::vector<int> col_ptr_, row_;             // off-diagonal blocks of column k: rows row_[col_ptr_[k] .. col_ptr_[k+1])
    struct Update {                               // right-looking update of column k: target -= L(pj) * L(pi)^T
        int pi, pj;                               // the two off-diagonal blocks of column k
        int target;                               // >= 0: off-diagonal block; < 0: diagonal block -(target + 1)
    };
    std::vector<int> upd_ptr_;                    // updates of column k: upd_[upd_ptr_[k] .. upd_ptr_[k + 1])
    std::vector<Update> upd_;
    std::vector<int> elem_block_;                 // per element: its off-diagonal block (-1: none)
    std::vector<uint8_t> elem_swap_;              // 1: the block's row node is the element's a
    std::vector<double> diag_, off_, rhs_;        // numeric blocks (36 doubles each), right-hand side (6 per node)
    std::vector<char> fixed_;                     // (solve's scratch)
    std::vector<vec3> pred_, tpred_;              // (the smooth force and torque a step took for its whole length, per node)
    std::vector<vec3> member_t_;                  // (the members' part of the torque, as member_f)
    std::vector<char> comp_factored_;             // (a component's factorization in diag_ / off_ / dinv_ is its last step's)
    float held_step_ = 0;
    std::vector<double> dinv_;                    // the factor's inverse diagonals
    std::vector<double> diagA_, offA_, rhsA_;     // the system as assembled, kept while members yield (active-set passes)
    struct Changed {
        uint32_t elem;
        uint8_t unload;                           // its unload bits as assembled
    };
    // the components (analyse): their range of the permuted order, members, the body's springs on their nodes (and
    // not across), the damped mounts' ends in them (frame node, the other end's, damping); per component in the
    // solve: members to assemble again, statistics
    std::vector<std::pair<int, int>> comp_range_;
    std::vector<std::vector<uint32_t>> comp_elems_, comp_links_;
    struct MountDamp {
        int32_t self, other;
        float damp;
        uint32_t mount;
    };
    std::vector<std::vector<MountDamp>> comp_mdamp_;
    std::vector<std::vector<Changed>> comp_changed_;
    struct CompStats {
        int failures = 0, clamps = 0;
        long long passes = 0;
    };
    std::vector<CompStats> comp_stats_;
    struct SolveArgs {
        float h = 0, step = 0, theta = 0, dissipation = 0;
        const vec3* ext = nullptr;
    } sv_;
    std::vector<vec3> w0_;                        // (the angular velocities at the solve's start: the mounts' damping)
    void mount_forces(SoftBody& b);
    void analyse();
    void element_matrix(const Tangent& t, double K[12][12]) const;
};

} // namespace bl::phys
