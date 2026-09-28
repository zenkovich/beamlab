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
#pragma once

#include "core/math.h"

#include <algorithm>
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
    int max_level = 1;              // a yielding member is split once (its halves are not split again)
    float min_len() const { return std::max(0.15f, 8.0f * half); } // (nor into pieces shorter than this)
    float mass_per_m() const { return rho * A; }
};
// A section of `shape` with the outer size `outer` (diameter or side, m) and the wall `wall` (m; tubes and boxes).
FrameSection make_frame_section(const std::string& material, FrameShape shape, float outer, float wall);

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
    float util = 0;                 // load against the yield or buckling limit (1: yielding), for display
    float N = 0;                    // axial force (tension > 0), for display
};

class FemFrame {
public:
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
    int broken = 0;                 // tears so far (ends torn off their joints)
    int splits = 0;                 // members split so far
    int solve_failures = 0;         // steps whose factorization failed (the members' forces then act explicitly)
    int clamps = 0;                 // velocity changes cut to the safety limits (see solve)
    long long passes_ = 0;          // (statistics: assembled and factored systems)

    bool empty() const { return elems.empty(); }
    int slot(uint32_t body_node) const { return body_node < slot_.size() ? slot_[body_node] : -1; }
    uint32_t add_node(uint32_t body_node);
    uint16_t add_section(const FrameSection& s);
    // A member between two body nodes (their frame nodes are made as needed), with the joints at its ends.
    uint32_t add_element(uint32_t body_a, uint32_t body_b, uint16_t section, uint8_t end_a = FJ_RIGID, uint8_t end_b = FJ_RIGID, int32_t tag = -1);
    float element_mass(const FrameElement& e) const { return sections[e.section].mass_per_m() * e.L0; }
    // With the body at rest: the members' rest lengths and frames, the nodes' rotational inertia (from the members'
    // masses) and the solver's ordering. Called by the first step if the caller does not (after adding members).
    void finalize(const SoftBody& b);
    bool ready() const { return ready_; }
    // The members' forces at the current state: added to b.force, their torques to `torque`. Plastic yield and
    // breaking happen here.
    void compute_forces(SoftBody& b);
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
    void hold(SoftBody& b, float step);
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

private:
    std::vector<int32_t> slot_;
    bool ready_ = false;
    struct Event {
        uint32_t elem;
        uint8_t kind;   // 0 split at t, 1 tear end a, 2 tear end b, 3 split at t and tear there
        float t;
    };
    std::vector<Event> events_;
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
    std::vector<double> dinv_;                    // the factor's inverse diagonals
    std::vector<double> diagA_, offA_, rhsA_;     // the system as assembled, kept while members yield (active-set passes)
    struct Changed {
        uint32_t elem;
        uint8_t unload;                           // its unload bits as assembled
    };
    std::vector<Changed> changed_;
    void analyse();
    void element_matrix(const Tangent& t, double K[12][12]) const;
};

} // namespace bl::phys
