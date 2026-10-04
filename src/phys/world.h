// Physics world: owns soft bodies, steps them with fixed 2 kHz substeps.
//
// Scheduling: every frame bodies are grouped into simulation islands (bodies whose swept
// AABBs overlap). Each island is simulated for the whole frame (all substeps) as one task on
// the job system - no per-substep barriers between independent islands. Islands whose bodies
// are all at rest are put to sleep and skipped.
//
// Inter-body contacts: node-vs-triangle, capsule-vs-triangle and node-vs-capsule, found via a
// spatial hash (Teschner et al. 2003) that is rebuilt every few substeps with a velocity based
// margin (cached candidate pairs, "Verlet list" style); narrow phase + response run every substep.
#pragma once

#include "phys/softbody.h"
#include "phys/static_world.h"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace bl::phys {

struct WorldSettings {
    vec3 gravity{0, -9.81f, 0};
    float dt = kDefaultDt;
    // frame elements (FemFrame::solve): how implicit the step is and its numerical dissipation (0: none, the members'
    // material damping only)
    float frame_theta = 0.5f;
    float frame_dissipation = 0.02f;
    // a piece of a frame torn off, of members alone and lighter than this (or a tenth of its body), leaves as a
    // rigid body (FemFrame::detach_debris)
    float frame_debris_mass = 40.0f;
    int max_substeps_per_frame = 100;   // slow-motion instead of spiral of death
    // A frame's physics budget (ms, 0: none): a frame steps at most the substeps its last frames' cost per substep fits
    // in it (but at least a quarter of its time), the rest dropped - the simulation slows for the heavy moments (an
    // impact's frames) instead of the frame rate (the app sets it while it runs in real time: App::run)
    float frame_budget_ms = 0;
    // (the spiral guard's threshold: the physics' share of an average frame past which a frame steps no more than an
    // average one - half in line with the frame's other work, more when the physics runs beside it: App)
    float overload_share = 0.5f;
    // the rates: the beams, shocks, wheels and contact forces every substep (2 kHz); the frames' implicit step
    // (FemFrame: members and triangles) every frame_every substeps (1: 2 kHz; between, the frame answers the substep's
    // forces through its factorization: FemFrame::held_begin), in every frame alike; collision detection (the candidate
    // pairs and the pairs within reach, see rebuild_pairs / near_pairs) at most collision_hz times a second
    int frame_every = kFrameEvery;
    // (a body of plates at rest - still as a whole, nothing moving has touched it, nothing of it torn, for
    // SoftBody::kPlatesQuiet: a wreck lying still - its frame's step every this many substeps; App sets 2 while it holds the frame rate in real time,
    // the fixed stepping of the tests keeps 1)
    int frame_every_quiet = 1;
    float collision_hz = 120.0f;
    bool inter_body_collisions = true;
    bool sleeping = true;
    float sleep_speed = 0.06f;          // m/s
    float tyre_grip = 1.0f;             // multiplier of the tyre friction (tyre_contact)
    vec3 wind_focus{0};                 // wind keeps bodies awake only within wind_radius of this point (the camera)
    float wind_radius = 0;              // 0: everywhere
    float sleep_time = 1.0f;            // s
    float time_scale = 1.0f;
    bool multithreaded = true;
    double team_cost = 12000;           // islands above this cost share their phases with idle threads (World::Island::teamed)
    bool pair_search_on_topology = false; // search the contact pairs again after a topology change instead of inheriting them
    vec3 wind{0, 0, 0};
    float wind_gusts = 0.5f;
    bool element_stats = false;         // count the elements every frame (WorldStats::el: the performance widget, the CSV)
    // sheets (triangle elements), for experiments: overrides of every sheet's material (-1: the material's own value)
    int sheet_max_level = -1;           // bisections (4: edges a quarter)
    float sheet_min_edge = -1;          // no triangle edge shorter than this (m)
    int sheet_min_piece = -1;           // no crack may cut off fewer finest triangles
    bool fracture_patterns = true;      // the materials' crack patterns round an impact
    float coarsen_quiet = 0.3f;         // coarsening merges triangles strained below this fraction of the refine threshold
    float coarsen_delay = 0.25f;        // s after the last refinement or crack of a sheet before it is coarsened again
    int refine_per_frame = 200;         // bisections per sheet per frame (0: no limit): an impact's burst of detail is spread
                                        // over a few frames instead of one long one (the overloads queue again)
    int coarsen_per_frame = 4000;       // merges per sheet per frame (0: no coarsening)
    bool rigid_pieces = true;           // pieces cracked off sheets become rigid bodies (with ShellMaterial::rigid_pieces)
};

struct WorldStats {
    int substeps = 0;
    int islands = 0;
    int active_bodies = 0;
    int sleeping_bodies = 0;
    int active_nodes = 0;
    int active_beams = 0;
    int contact_pairs = 0;      // candidate pairs (last rebuild, summed)
    int contacts = 0;           // actual contacts in last substep (summed)
    double step_ms = 0;
    double cost_per_substep_ms = 0;     // (the frames' step over their substeps, smoothed: the budget's estimate)
    int budget_cut_substeps = 0;        // (the last frame's substeps dropped by the frame budget)
    double sim_time = 0;
    float realtime_factor = 1;
    // profiling detail, per frame (the performance widget, BL_PROFCSV)
    double islands_cpu_ms = 0;          // all islands, summed over threads
    double heavy_island_ms = 0;         // the slowest island: the frame's critical path (one island runs on one thread)
    int heavy_island_bodies = 0, heavy_island_nodes = 0, heavy_island_beams = 0, heavy_island_shells = 0;
    int heavy_island_sub = 1;           // short steps per substep of its finest sheet
    double heavy_sub_ms[4] = {};        // its integration's parts (wall): static contacts of the nodes, tyres and plates on the
                                        // ground, the FEM steps begun (assembly), the FEM steps (factor, solve)
    double heavy_phase_ms[9] = {};      // its phases (wall): forces, gather, collisions, integration, serial, topology;
                                        // collisions split: rebuilds, fast pair refreshes, narrow phase + response
    bool heavy_island_wide = false;
    std::string heavy_island_body;      // its most expensive body
    int awake_shells = 0;               // triangle elements of awake bodies
    long long shell_steps = 0;          // triangle evaluations this frame (shells x short steps)
    long long beam_steps = 0;           // beam evaluations this frame
    int shell_refines = 0, shell_cracks = 0, pieces_created = 0;
    int shell_merges = 0;               // coarsening merges this frame
    long long narrow_tests = 0;         // inter-body candidate pairs tested (all substeps)
    int pair_rebuilds = 0, fast_refreshes = 0;
    int dropped_substeps = 0;           // frames that hit the substep limit (running count; slow motion)
    // The elements of the world after the frame (settings.element_stats): how many, how many awake (the rest sleep),
    // and the work each kind took in the frame
    struct Elements {
        int bodies = 0, bodies_awake = 0, pieces = 0, pieces_awake = 0, pieces_rigid = 0; // (pieces: cracked off a sheet)
        int nodes = 0, nodes_awake = 0;
        int beams = 0, beams_awake = 0, beams_broken = 0;
        int shells = 0, shells_awake = 0;       // triangle elements
        int shells_level[5] = {};               // awake, by level (the last: 4 and finer)
        int shells_rate[3] = {};                // awake, in bodies taking 1 / 2 / 4 short steps per substep
        int hinges = 0, hinges_awake = 0;       // links between triangles: a shared edge, a bending hinge each
        int edges_border = 0, edges_crack = 0, edges_cut = 0; // free edges of the sheets: authored border, cracks, laser
        int tris = 0, tris_awake = 0;           // collision triangles
        int impacts = 0;                        // fracture patterns laid
        long long node_steps = 0;               // node integrations (awake nodes x short steps)
        long long shell_evals = 0, hinge_evals = 0; // triangles / hinges the force kernel evaluated (multi-rate: the
                                                    // coarse ones less often)
        long long beam_evals = 0;
    } el;
    double team_wait_ms = 0;            // island owners waiting at phase ends for helpers still in a chunk
    int team_stalls = 0;                // such waits over 0.2 ms (a helper preempted by the OS)
};

// The sphere contacts' pairs between bodies (phys/sphere_contacts.cpp), found once a substep with a margin for the
// bodies' travel over it and kept for its short steps (one per island)
struct SpherePairs {
    struct Pair {
        uint32_t ba, sa, bb, sb; // bodies (in `bodies`) and spheres
        float r;                 // the sum of the radii
    };
    std::vector<Pair> pairs;
    std::vector<SoftBody*> bodies; // (the bodies they were found among: another set, found again)
};
int sphere_contacts(std::vector<SoftBody*>& bodies, float h, std::vector<uint32_t>& group_of, float wake_speed, SpherePairs& cache,
                    bool detect, float span);

struct RayHit {
    SoftBody* body = nullptr;
    int node = -1;
    float t = -1;
};

class World {
public:
    World();
    ~World();
    WorldSettings settings;
    StaticWorld statics;

    SoftBody* add_body(std::unique_ptr<SoftBody> b);
    void remove_body(SoftBody* b);
    void clear();
    const std::vector<std::unique_ptr<SoftBody>>& bodies() const { return m_bodies; }

    // Advance simulation by real frame time (substeps are derived from settings).
    void step_frame(float frame_dt);
    // Run exactly `n` substeps (benchmarks / single stepping).
    void step_substeps(int n);

    const WorldStats& stats() const { return m_stats; }
    double time() const { return m_time; }
    // Closest node to the ray within `radius` (for mouse grab).
    RayHit pick_node(vec3 o, vec3 d, float max_t, float radius) const;
    // First hit of a ray on body surfaces (triangles) and beams (thin cylinders of beam_radius).
    // Returns the hit distance and the body's node closest to the hit point.
    RayHit raycast_bodies(vec3 o, vec3 d, float max_t, float beam_radius) const;
    // Breaks every beam / joint / slide node passing within `radius` of `p` (cursor destruction).
    // Optionally pushes nearby nodes outwards with `impulse` m/s. Returns the number of broken elements.
    int destroy_at(vec3 p, float radius, float impulse);
    // Laser cut: everything the sector swept by a ray from `o` turning from d0 to d1 passes through (within `range`) is cut
    // along it: sheets are cut (SoftBody::cut_shells), beams, joints and surface triangles crossing it break. No impulse.
    // Returns the number of links / beams cut.
    // A collision volume it runs through is off. `skip`: a body left whole (the blade that does the cutting). `kerf`: a
    // blade's half thickness - a frame member lying along the plane within it, in the sector, is torn off at both ends
    // (the blade meets it lengthwise: an axe's wedge stopped on a tube it could not cut across), and the frame nodes the
    // cut leaves on the plane stand off it 2 mm on their own side (each face of the blade's wedge meets its own half's)
    int laser_cut(vec3 o, vec3 d0, vec3 d1, float range, const SoftBody* skip = nullptr, float kerf = 0);
    void wake_all();
    // A sheet cracked a loose piece off: it is a new body now (the game gives it a visual).
    std::function<void(SoftBody* parent, SoftBody* piece)> on_piece;

private:
    struct Island;
    void build_islands(float frame_time);
    void simulate_island(Island& isl, int substeps);
    void rebuild_pairs(Island& isl);
    void fast_pairs(Island& isl);
    void refresh_fast_pairs(Island& isl);
    void near_pairs(Island& isl);
    // (detected: the narrow phase's geometry done already, pair_detect / mid_detect in the forces' team stage)
    static constexpr uint32_t kPairChunkSize = 256;
    static constexpr uint32_t kFemChunk = 256;       // (a frame's body's static contacts in chunks of nodes)
    int pair_chunks(Island& isl);
    void pair_detect(Island& isl, int c);
    void collide_pairs(Island& isl, bool detected = false);
    int mid_chunks(Island& isl);
    void mid_detect(Island& isl, int c);
    void collide_mid_pairs(Island& isl, bool detected = false); // (the FEM plates' mid points against the other bodies' triangles: SoftBody::tri_mids)
    // (the bodies' collision volumes: SoftBody::volumes; stage 0 the whole of it, 1 their places and the look's parts -
    // their count: volume_find for each, beside the forces - 2 the contacts' forces)
    int collide_volumes(Island& isl, float dt, bool bodies, int stage = 0);
    void volume_find(Island& isl, int sq);
    void inherit_pairs(Island& isl, int body);
    // the body's ring tyres (Wheel::ring) against the static world, their spin with the drive and the brakes
    void ring_tyres(SoftBody& b, const std::vector<int>& box_ids, const std::vector<int>& cyl_ids, float terrain_max_h, float dt);
    // the FEM plates' mid points against the static world (SoftBody::tri_mids), after its nodes' (collide_static)
    void collide_static_mids(SoftBody& b, const std::vector<int>& box_ids, const std::vector<int>& cyl_ids, float terrain_max_h, float dt, int& contacts);
    // (collide_static_mids in two: the static world's contacts of the plates [k0, k1) - any thread, a range each -, then
    // their forces in the plates' order)
    struct StaticMid {
        uint32_t k, n[3];
        vec3 p;
        ContactInfo c;
    };
    void static_mids_find(const SoftBody& b, size_t k0, size_t k1, const std::vector<int>& box_ids, const std::vector<int>& cyl_ids, float terrain_max_h,
                          std::vector<StaticMid>& out) const;
    void static_mids_apply(SoftBody& b, const std::vector<StaticMid>& hits, float dt, int& contacts);
    void collide_static(SoftBody& b, size_t n0, size_t n1, const std::vector<int>& box_ids, const std::vector<int>& cyl_ids, float terrain_max_h, float dt,
                        int& contacts);
    int repair_nodes(SoftBody& b); // resets non-finite / runaway nodes; -1 when the body cannot be repaired
    void count_elements(); // WorldStats::el (settings.element_stats)
    void update_sleep(SoftBody& b, float frame_time);
    bool in_wind(const SoftBody& b) const;
    void apply_wind(SoftBody& b);

    std::vector<std::unique_ptr<SoftBody>> m_bodies;
    std::vector<std::unique_ptr<Island>> m_islands;
    int m_num_islands = 0;
    double m_accum = 0;
    double m_avg_frame = 0;         // average frame time (s), for the substep limit of step_frame
    double m_time = 0;
    WorldStats m_stats;
    int m_next_id = 1;
    std::atomic<long long> m_narrow{0};
    std::atomic<int> m_rebuilds{0}, m_fast_refreshes{0};
    long long m_refines_total = 0, m_cracks_total = 0; // for per-frame differences
};

} // namespace bl::phys
