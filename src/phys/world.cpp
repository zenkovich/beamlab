#include "phys/world.h"
#include "core/jobs.h"
#include "core/profiler.h"
#include "core/util.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <numeric>

namespace bl::phys {

namespace {

// Primitive reference inside an island.
struct PrimRef {
    uint16_t body;   // index in island body list
    uint8_t kind;    // 0 triangle, 1 capsule
    uint32_t idx;
};

inline uint32_t cell_hash(int x, int y, int z) {
    return (uint32_t)(x * 73856093) ^ (uint32_t)(y * 19349663) ^ (uint32_t)(z * 83492791);
}

inline float closest_on_segment(vec3 p, vec3 a, vec3 b) {
    vec3 ab = b - a;
    float l2 = dot(ab, ab);
    if (l2 < 1e-12f) return 0;
    return clampf(dot(p - a, ab) / l2, 0, 1);
}

// Closest points between segments p1q1 and p2q2 (Ericson 5.1.9). Returns params s,t.
inline void closest_seg_seg(vec3 p1, vec3 q1, vec3 p2, vec3 q2, float& s, float& t) {
    vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    float a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    if (a <= 1e-12f && e <= 1e-12f) { s = t = 0; return; }
    if (a <= 1e-12f) { s = 0; t = clampf(f / e, 0, 1); return; }
    float c = dot(d1, r);
    if (e <= 1e-12f) { t = 0; s = clampf(-c / a, 0, 1); return; }
    float b = dot(d1, d2), denom = a * e - b * b;
    s = denom != 0 ? clampf((b * f - c * e) / denom, 0, 1) : 0;
    t = (b * s + f) / e;
    if (t < 0) { t = 0; s = clampf(-c / a, 0, 1); }
    else if (t > 1) { t = 1; s = clampf((b - c) / a, 0, 1); }
}

// RoR-style contact between two weighted node sets (side A is pushed along +n).
// Effective mass uses exact barycentric weights: 1/m = sum(wa^2/ma) + sum(wb^2/mb).
// The force primitive_collision() cancels is the effective force on the relative motion at the contact point,
// m * (a_A - a_B), from everything accumulated on BOTH sides so far in this substep. (With side A's force only, a
// light triangle touched by several nodes of a heavy body at once got the full correction from each of them: a
// cloth under a steel ball flipped its velocity every substep and blew up.)
// A body that takes several short steps per substep (a refined sheet) holds the contact force over them while its
// own forces change: cancelling its own (internal) force of the first short step would then fling its light nodes.
// For such a side SA / SB point at the snapshot of its internal forces (SoftBody::ext_force), and only the contact
// forces added on top of it (by other contacts in this substep) are cancelled.
template <int NA, int NB>
inline bool contact_force(Node* const* A, vec3* const* FA, const float* wa, Node* const* B, vec3* const* FB, const float* wb,
                          bool a_movable, bool b_movable, vec3 n, float pen, float dt, const GroundModel& gm, float fric,
                          const vec3* const* SA = nullptr, const vec3* const* SB = nullptr) {
    float W = 0;
    if (a_movable)
        for (int i = 0; i < NA; i++) W += wa[i] * wa[i] * A[i]->inv_mass;
    if (b_movable)
        for (int i = 0; i < NB; i++) W += wb[i] * wb[i] * B[i]->inv_mass;
    if (W <= 1e-12f) return false;
    float m = 1.0f / W;
    vec3 va(0), vb(0), acc(0);
    for (int i = 0; i < NA; i++) {
        va += A[i]->v * wa[i];
        if (a_movable) acc += (SA ? *FA[i] - *SA[i] : *FA[i]) * (wa[i] * A[i]->inv_mass);
    }
    for (int i = 0; i < NB; i++) {
        vb += B[i]->v * wb[i];
        if (b_movable) acc -= (SB ? *FB[i] - *SB[i] : *FB[i]) * (wb[i] * B[i]->inv_mass);
    }
    const vec3 f_rel = acc * m; // effective force on (A - B) at the contact point
    if (!a_movable) {
        // A is immovable (sleeping/fixed): flip roles so the force is computed for B
        vec3 f = primitive_collision(-f_rel, vb - va, m, -n, dt, gm, pen, fric);
        for (int i = 0; i < NB; i++) *FB[i] += f * wb[i];
        return true;
    }
    vec3 f = primitive_collision(f_rel, va - vb, m, n, dt, gm, pen, fric);
    for (int i = 0; i < NA; i++) *FA[i] += f * wa[i];
    if (b_movable)
        for (int i = 0; i < NB; i++) *FB[i] -= f * wb[i];
    return true;
}

// A node behind a hull triangle's face (Triangle::two_sided false, deeper than the contact radius): the force along the
// face's normal that makes its velocity relative to the face after the step the push-out speed (a fifth of the depth a
// step, at most 2 m/s), counting the forces on both sides so far (the car behind the node pushes it in), and only when
// it would come out slower: no friction, nothing cancelled beyond that. (RoR's contact cancels every inward force on a
// node in contact; a node held deep in a hull for many steps was pumped out by its own springs: a ball started 15 cm
// behind a face left at 6.5 m/s.) SA / SB: see contact_force.
template <int NB>
inline bool hull_push_out(Node* A, vec3* FA, Node* const* B, vec3* const* FB, const float* wb, bool a_movable, bool b_movable, vec3 n, float pen, float dt,
                          const vec3* SA = nullptr, const vec3* const* SB = nullptr) {
    float W = a_movable ? A->inv_mass : 0.0f;
    if (b_movable)
        for (int i = 0; i < NB; i++) W += wb[i] * wb[i] * B[i]->inv_mass;
    if (W <= 1e-12f) return false;
    vec3 vb(0), acc(0);
    if (a_movable) acc += (SA ? *FA - *SA : *FA) * A->inv_mass;
    for (int i = 0; i < NB; i++) {
        vb += B[i]->v * wb[i];
        if (b_movable) acc -= (SB ? *FB[i] - *SB[i] : *FB[i]) * (wb[i] * B[i]->inv_mass);
    }
    const float m = 1.0f / W, vr = dot(A->v - vb, n), target = std::min(0.2f * pen / dt, 2.0f);
    const float fn = (target - vr) * m / dt - dot(acc, n) * m; // (the normal force it needs besides what it has)
    if (fn <= 0) return true;
    const vec3 f = n * fn;
    if (a_movable) *FA += f;
    if (b_movable)
        for (int i = 0; i < NB; i++) *FB[i] -= f * wb[i];
    return true;
}

} // namespace



struct World::Island {
    std::vector<SoftBody*> bodies;
    std::vector<std::vector<int>> box_ids, cyl_ids;
    std::vector<float> terrain_max;
    bool needs_pairs = false;
    bool teamed = false;        // big island: its phases are shared with the idle threads (Team)
    Team* team = nullptr;       // (during simulate_island)
    float dt = kDefaultDt;      // substep
    int rebuild_interval = 1, next_rebuild = 0;
    double cost = 0;
    double max_body_cost = 0;   // the most expensive body
    double team_cost = 0;       // cost for the team decision (with the sleeping sheets it may wake)
    int contacts = 0;
    int pair_count = 0;

    struct NT { uint16_t bn, bt; uint32_t node, tri; };
    struct NC { uint16_t bn, bc; uint32_t node, cap; };
    struct CT { uint16_t bc, bt; uint32_t cap, tri; };
    struct MT { uint16_t bm, bt; uint32_t mid, tri; }; // (a FEM plate's mid point, SoftBody::tri_mids: its FemFrame::tris)
    std::vector<NT> nt;
    std::vector<NC> nc;
    std::vector<CT> ct;
    std::vector<MT> mt;
    std::vector<std::vector<MT>> q_mt;       // (their broadphase's, per chunk of triangles)
    std::vector<SoftBody*> sph;              // the sheet bodies with sphere contacts stepping now (scratch)
    std::vector<int> mem_bodies;             // the sheets whose membranes are projected this short step (scratch)
    std::vector<float> mem_speed;            // their top speed after it (-1: not projected)
    std::vector<uint32_t> sph_group;
    SpherePairs sph_pairs;

    // spatial hash scratch
    struct Pt {
        uint16_t body;
        uint32_t node;
        int x, y, z;
        vec3 p;                              // (its place: a query's box test before the body's arrays)
    };
    std::vector<Pt> pts;
    std::vector<Pt> mpts;                    // (the plates' mid points in the hash: node = its FemFrame::tris)
    std::vector<vec3> mpos;                  // (their positions)
    std::vector<uint32_t> fill;
    std::vector<PrimRef> prims;
    std::vector<std::pair<uint32_t, uint32_t>> entries; // (bucket, prim)
    std::vector<uint32_t> bucket_start;
    std::vector<uint32_t> bucket_items;
    std::vector<uint32_t> run_end;           // per bucket item: end of the run of the same body's nodes
    std::vector<float> extra;                // per body: extra pair margin of a fast body
    std::vector<float> bulk;                 // per body: bulk speed of a fast body (margins, refresh schedule)
    float slow_speed = 0;                    // top speed of the slow bodies (last rebuild)
    float margin = 0;                        // pair margin between slow bodies (last rebuild)
    float inv_cell = 1.0f;                   // node hash cells (last rebuild)
    int fast_interval = 0, next_fast = 0;    // refresh schedule of the fast bodies' own pairs (0: none)
    bool fast_dense = false;                 // the fast bodies' pairs need the short schedule
    // profiling of the last frame
    double ms = 0;                           // wall time of simulate_island
    double ph_ms[9] = {};                    // its phases (wall): forces, gather, collisions, integration, serial merges, topology,
                                             // and of the collisions: rebuilds, fast pair refreshes, narrow phase + response
    int max_sub = 1;                         // most short steps per substep of a body
    long long shell_steps = 0, beam_steps = 0;
    long long node_steps = 0, hinge_evals = 0;  // (this frame: node integrations, hinges evaluated)
    size_t nt_fast = 0, nc_fast = 0;         // where the fast bodies' pairs start in nt / nc
    std::vector<uint8_t> near_keep;          // (near_pairs: scratch)
    std::vector<AABB> boxes;                 // per body, expanded by the contact reach
    std::vector<std::vector<int>> partners;  // per body: bodies whose boxes overlap

    // ---- parallel phases (simulate_island): work items, their results; every result has a slot of its own and is
    // merged in item order, so the outcome does not depend on the threads
    struct Work {
        uint32_t body;
        uint32_t kind;  // 0: internal forces of the body (beams, shocks, wheels ...), 1: chunk `a` of its triangles, 2: nodes [a, b),
                        // 5, 6: chunk `a` of the narrow phase (pair_detect, mid_detect), 7: of the volumes' (volume_find),
                        // 3: a rigid body's step, 4: chunk `a` of its frame's members
        uint32_t a, b;
    };
    std::vector<Work> work;
    std::vector<char> fem_parts;    // (bodies whose frame members are evaluated in chunks of their own this short step)
    std::vector<char> fem_defer;    // (integration items whose frame solves this short step, apart: then they integrate)
    std::vector<std::pair<uint32_t, int>> fem_work;   // (those frames' components: the item, the component)
    std::vector<std::pair<uint32_t, int>> fem_par;    // (their large components, factored in the team's stages)
    std::vector<std::pair<uint32_t, int>> fem_small;  // (the rest: solved whole, or held)
    std::vector<std::pair<uint32_t, int>> fem_hpar;   // (the large ones held, in the team's stages: FemFrame::held_par)
    std::vector<std::pair<int, int>> fem_htasks;      // (their groups: one of fem_hpar, the group)
    std::vector<char> fem_chunked;                    // (per body: a frame's body integrated in chunks of nodes)
    std::vector<int> vmid_bodies;                     // (collide_volumes: the bodies with plates' mid points)
    std::vector<int> ev_bodies;                       // (the bodies with frame events this short step)
    std::vector<char> ev_done;                        // (per body: its events done beside the others', 2 changed)
    std::vector<char> fem_pre;                        // (per body: its frame's step begun beside the contacts)
    std::vector<int> fem_pre_nc;                      // (... its components)
    struct PreTask { int k, c, chunk; };              // (... their blocks: body, component, chunk or -1 the whole of it)
    std::vector<PreTask> pre_tasks;
    std::vector<std::pair<int, uint32_t>> smid_tasks; // (their plates' mid points' static contacts: body, range of plates)
    std::vector<int> smid_first;                      // (per body: its first of those, -1 none)
    std::vector<std::vector<World::StaticMid>> smid;  // (each one's contacts found)
    // the collision volumes' look (collide_volumes): the volumes, their parts' chunks, the hits of each
    struct VTask {
        int ka;
        CollisionVolume* V;
    };
    struct VHit {
        uint8_t kind;       // 0 own part's node, 1 own part's plate, 2 node, 3 plate, 4 ball, 5 volume's vertex, 6 static vertex, 7 pole
        uint8_t moves, fem, pad;
        int kb;             // (the other body)
        uint32_t i;         // node, capsule's node, volume, the plate's triangle, the surface
        uint32_t n3[3];
        float w[3];
        vec3 p, n;
        float pen, extra;   // (a volume's vertex: its mass share; static: the force's cap)
    };
    struct VSub {
        int t, kb;
        uint8_t part;   // 0 own parts, 1 nodes of kb, 2 plates of kb, 3 balls and volumes of kb, 4 static
        uint32_t i0, i1; // (plates: a range of kb's mid points in their order)
    };
    std::vector<VTask> vtasks;
    std::vector<VSub> vsubs;
    std::vector<int> vsub_ptr;
    std::vector<std::vector<VHit>> vhits;
    std::vector<const SoftBody::MidCache*> vmids;
    bool vbodies = false;
    std::vector<std::pair<int, int>> fem_tasks;       // (a stage's chunks: the large component, its group)
    struct NodePart {
        vec3 mn, mx;
        float max_v2;
        int contacts;
    };
    std::vector<NodePart> parts;
    std::vector<int> subs;                   // short steps per substep of each body (0: asleep)
    struct Hit {                             // narrow phase result of one candidate pair
        uint32_t pair;
        float s;                             // segment parameter (capsules)
        vec3 bary, nrm;
        float pen;
    };
    std::vector<std::vector<Hit>> hits;      // per chunk of candidate pairs
    std::vector<std::vector<Hit>> mhits;     // (the plates' mid points' pairs: per chunk of them)
    struct QChunk {                          // broadphase query: triangles [t0, t1) of `body` (and its capsules when caps)
        uint16_t body, other;
        uint32_t t0, t1;
        bool caps;
    };
    std::vector<QChunk> qchunks;
    std::vector<std::vector<NT>> q_nt;       // per query chunk
    std::vector<std::vector<NC>> q_nc;
    struct Grid {                            // contacter nodes of a fast body: a dense grid over its bounds (CSR)
        vec3 origin;
        float inv_cell = 1;
        int dim[3] = {1, 1, 1};
        std::vector<uint32_t> start, items;  // items: node ids by cell, ascending within a cell
    };
    std::vector<Grid> grids;                 // per body
};

World::World() = default;
World::~World() = default;

SoftBody* World::add_body(std::unique_ptr<SoftBody> b) {
    b->id = m_next_id++;
    b->force.resize(b->nodes.size());
    b->compute_aabb();
    m_bodies.push_back(std::move(b));
    return m_bodies.back().get();
}

void World::remove_body(SoftBody* b) {
    for (size_t i = 0; i < m_bodies.size(); i++)
        if (m_bodies[i].get() == b) {
            m_bodies.erase(m_bodies.begin() + i);
            return;
        }
}

void World::clear() {
    m_refines_total = m_cracks_total = 0;
    m_bodies.clear();
    m_islands.clear();
    m_num_islands = 0;
    m_accum = 0;
    m_time = 0;
}

void World::wake_all() {
    for (auto& b : m_bodies) b->wake();
}

void World::step_frame(float frame_dt) {
    // Spiral of death guard, only while the physics itself makes the frames long (its last frame took more than half an
    // average frame): then a frame may take at most the substeps of an average frame (+1), and after a hitch the
    // simulation runs slow for a moment instead of doubling the next frame's work (which makes it slow too ...; with
    // 1.25x of them a crash's frames of 35 ms took 45 and more the next frames, 60 ms in the app). Frames that are long
    // for other reasons (uneven presentation, the window system) are always simulated in full: dropping their time made
    // the motion jerk.
    const double want = (double)frame_dt * settings.time_scale;
    m_avg_frame = m_avg_frame <= 0 ? std::clamp(want, 1.0 / 240.0, 1.0 / 15.0) : m_avg_frame * 0.95 + 0.05 * std::clamp(want, 1.0 / 240.0, 1.0 / 15.0);
    m_accum += want;
    int n = (int)(m_accum / settings.dt);
    const bool overloaded = m_stats.step_ms > 0.5 * m_avg_frame * 1000.0;
    const int maxn = overloaded ? std::min(settings.max_substeps_per_frame, (int)std::ceil(m_avg_frame / settings.dt) + 1)
                                : settings.max_substeps_per_frame;
    if (n > maxn) {
        n = maxn;
        m_accum = 0; // drop the rest (slow motion under heavy load)
        m_stats.dropped_substeps++;
    } else {
        m_accum -= n * (double)settings.dt;
    }
    if (n > 0) step_substeps(n);
    else m_stats.substeps = 0;
    float got = n * settings.dt;
    m_stats.realtime_factor = m_stats.realtime_factor * 0.9f + 0.1f * (want > 0 ? std::min(1.5f, got / frame_dt) : 1.0f);
}

// (diagnostics, BL_TRACE=<frame>,<first substep>,<substeps>[,<file>]: those substeps' every profiled scope on its thread,
// as JSON - prof::trace_dump)
namespace {
struct TraceArm {
    int frame = -1, s0 = 0, count = 0;
    std::string path = "trace.json";
    TraceArm() {
        if (const char* e = getenv("BL_TRACE")) {
            char buf[512] = {};
            if (sscanf(e, "%d,%d,%d,%511s", &frame, &s0, &count, buf) >= 3 && buf[0]) path = buf;
        }
    }
};
const TraceArm& trace_arm() {
    static TraceArm a;
    return a;
}
int g_trace_frame = 0;
} // namespace

void World::step_substeps(int n) {
    PROFILE_ZONE("Physics");
    g_trace_frame++;
    struct DumpAtEnd {
        uint64_t t0 = prof::now();
        ~DumpAtEnd() {
            if (trace_arm().frame == g_trace_frame) prof::g_trace = false, prof::trace_dump(trace_arm().path.c_str());
            static const bool helpdbg = getenv("BL_HELPDBG") != nullptr; // (diagnostics: the team's chunks, the owners' and the helpers')
            if (helpdbg) {
                long long own, helped;
                JobSystem::get().take_chunk_counts(own, helped);
                printf("help %d: %.1f ms, chunks own %lld helped %lld (%.0f%%)\n", g_trace_frame, prof::ticks_to_ms(prof::now() - t0), own, helped, own + helped ? 100.0 * helped / (own + helped) : 0.0);
            }
        }
    } dump_at_end;
    uint64_t t0 = prof::now();
    float frame_time = n * settings.dt;
    {
        // sheets that grew by a fifth since their last renumbering (refinement and cracks append at the end): back to a
        // Morton order, for the force kernel's cache locality (between frames: the contact pairs are searched anew)
        PROFILE_ZONE("Sheet reorder");
        for (auto& b : m_bodies)
            if (!b->shells.empty() && !b->rigid && !b->sleeping && b->shells.size() >= 64 && b->shells.size() * 5 > b->ordered_shells * 6 + 64) b->reorder_shells();
    }
    build_islands(frame_time);
    for (auto& b : m_bodies) {
        b->refine_left = settings.refine_per_frame > 0 ? settings.refine_per_frame : 1 << 30;
        if (b->shells.empty()) continue;
        // the sheet overrides (experiments) on top of the material as finalized
        auto over = [&](ShellMaterial m) {
            if (settings.sheet_max_level >= 0) m.max_level = settings.sheet_max_level;
            if (settings.sheet_min_edge >= 0) m.min_edge = settings.sheet_min_edge;
            if (settings.sheet_min_piece >= 0) m.min_piece = settings.sheet_min_piece;
            if (!settings.fracture_patterns) m.pattern = ShellPattern::None;
            return m;
        };
        b->shell_mat = over(b->shell_mat_base);
        for (size_t k = 0; k < b->shell_mat_extra.size() && k < b->shell_mat_extra_base.size(); k++) b->shell_mat_extra[k] = over(b->shell_mat_extra_base[k]);
        if (settings.sheet_max_level >= 0) b->shell_cap = std::max(b->shell_cap, (size_t)(b->shells.size() * 2) + 16);
    }
    m_stats.substeps = n;
    m_stats.contact_pairs = 0;
    m_stats.contacts = 0;
    {
        PROFILE_ZONE("Islands");
        // biggest islands first (LPT scheduling), one task each; an island big enough to dominate the frame shares its
        // phases with the threads that run out of islands (Team)
        std::vector<int> order(m_num_islands);
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int a, int b) { return m_islands[a]->cost > m_islands[b]->cost; });
        const int nthreads = JobSystem::get().num_threads();
        for (int i : order) {
            Island& isl = *m_islands[i];
            bool sheets = false, frames = false;
            for (const SoftBody* b : isl.bodies) {
                sheets |= !b->shells.empty();
                // (a frame of some size: its members' chunks and its components' solves go to the team)
                frames |= !b->fem.empty() && (b->fem.components() > 1 || b->fem.elems.size() + 3 * b->fem.tris.size() >= 200);
            }
            // (a sheet about to be hit refines and cracks within the frame: its island is worth a team long before its cost
            // says so; phases of a single work item run inline, a team costs next to nothing when it is not needed)
            isl.teamed = settings.multithreaded && nthreads > 1 &&
                         ((sheets && isl.team_cost > settings.team_cost * 0.2) || frames || (isl.bodies.size() >= 16 && isl.team_cost > settings.team_cost));
        }
        auto run = [&](int i, int) { simulate_island(*m_islands[order[i]], n); };
        if (settings.multithreaded) JobSystem::get().parallel_items((int)order.size(), run);
        else
            for (int i = 0; i < (int)order.size(); i++) run(i, 0);
    }
    for (int i = 0; i < m_num_islands; i++) {
        m_stats.contact_pairs += m_islands[i]->pair_count;
        m_stats.contacts += m_islands[i]->contacts;
    }
    {
        // profiling: the heaviest island (the critical path of the frame) and the work done
        WorldStats& st = m_stats;
        st.islands_cpu_ms = st.heavy_island_ms = 0;
        st.shell_steps = st.beam_steps = 0;
        st.el.node_steps = st.el.hinge_evals = 0;
        const Island* heavy = nullptr;
        for (int i = 0; i < m_num_islands; i++) {
            const Island& isl = *m_islands[i];
            st.islands_cpu_ms += isl.ms;
            st.shell_steps += isl.shell_steps;
            st.beam_steps += isl.beam_steps;
            st.el.node_steps += isl.node_steps;
            st.el.hinge_evals += isl.hinge_evals;
            if (!heavy || isl.ms > heavy->ms) heavy = &isl;
        }
        st.heavy_island_bodies = st.heavy_island_nodes = st.heavy_island_beams = st.heavy_island_shells = 0;
        st.heavy_island_sub = 1;
        st.heavy_island_wide = false;
        st.heavy_island_body.clear();
        if (heavy) {
            st.heavy_island_ms = heavy->ms;
            for (int k = 0; k < 9; k++) st.heavy_phase_ms[k] = heavy->ph_ms[k];
            st.heavy_island_bodies = (int)heavy->bodies.size();
            st.heavy_island_sub = heavy->max_sub;
            st.heavy_island_wide = heavy->teamed;
            double best = -1;
            for (const SoftBody* b : heavy->bodies) {
                st.heavy_island_nodes += (int)b->nodes.size();
                st.heavy_island_beams += (int)b->beams.size();
                st.heavy_island_shells += (int)b->shells.size();
                const double c = (double)(b->nodes.size() * 2 + b->beams.size() + b->shells.size() * 4) * (1 << b->dt_shift());
                if (c > best) {
                    best = c;
                    st.heavy_island_body = b->name;
                }
            }
        }
        long long refines = 0, cracks = 0;
        st.awake_shells = 0;
        for (const auto& b : m_bodies) {
            refines += b->shell_stats.refined;
            cracks += b->shell_stats.cracks;
            if (!b->sleeping) st.awake_shells += (int)b->shells.size();
        }
        st.shell_refines = (int)std::max(0LL, refines - m_refines_total);
        st.shell_cracks = (int)std::max(0LL, cracks - m_cracks_total);
        m_refines_total = refines;
        m_cracks_total = cracks;
        st.narrow_tests = m_narrow.exchange(0);
        st.pair_rebuilds = m_rebuilds.exchange(0);
        st.fast_refreshes = m_fast_refreshes.exchange(0);
        JobSystem::get().take_board_waits(st.team_wait_ms, st.team_stalls);
        st.pieces_created = 0;
    }
    {
        // sheets that cracked apart: loose pieces become bodies of their own (islands, bounds and sleep of their own)
        PROFILE_ZONE("Sheet detach");
        // (the sheets are independent: their pieces are cut off in parallel, then added in body order)
        std::vector<SoftBody*> cand;
        for (auto& bp : m_bodies)
            if (bp->pieces_check) {
                bp->pieces_check = false;
                cand.push_back(bp.get());
            }
        std::vector<std::vector<std::unique_ptr<SoftBody>>> pieces(cand.size());
        auto cut = [&](int i, int) {
            cand[i]->detach_pieces(pieces[i]);
            for (auto& p : pieces[i])
                if (settings.rigid_pieces && p->shell_mat.rigid_pieces) p->make_rigid();
        };
        if (settings.multithreaded && cand.size() > 1) JobSystem::get().parallel_items((int)cand.size(), cut);
        else
            for (int i = 0; i < (int)cand.size(); i++) cut(i, 0);
        for (size_t i = 0; i < cand.size(); i++)
            for (auto& piece : pieces[i]) {
                SoftBody* p = add_body(std::move(piece));
                if (on_piece) on_piece(cand[i], p);
                m_stats.pieces_created++;
            }
        // frames torn apart: the small pieces of members alone leave as rigid bodies (FemFrame::detach_debris)
        for (size_t bi = 0, nb0 = m_bodies.size(); bi < nb0; bi++) {
            SoftBody* bp = m_bodies[bi].get();
            if (!bp->fem.debris_check || bp->rigid) continue;
            std::vector<std::unique_ptr<SoftBody>> debris;
            bp->fem.detach_debris(*bp, debris, std::max(settings.frame_debris_mass, 0.1f * bp->total_mass()));
            for (auto& d : debris) {
                if (settings.rigid_pieces) d->make_rigid();
                SoftBody* p = add_body(std::move(d));
                if (on_piece) on_piece(bp, p);
                m_stats.pieces_created++;
            }
        }
    }
    {
        // sheets that have settled since their last refinement or crack go back towards their authored triangles
        // (SoftBody::coarsen_shells): a hit sheet stops paying for its detail once it is at rest (the fine triangles
        // made the whole sheet take short steps); the bodies are independent
        PROFILE_ZONE("Sheet coarsen");
        std::vector<SoftBody*> cand;
        for (auto& bp : m_bodies) {
            SoftBody& b = *bp;
            if (b.shells.empty() || b.shell_level == 0 || b.rigid) continue;
            const int seen = b.shell_stats.refined + b.shell_stats.cracks;
            if (seen != b.topo_seen) {
                b.topo_seen = seen;
                b.refine_time = m_time;
            }
            if (settings.coarsen_per_frame > 0 && m_time - b.refine_time > settings.coarsen_delay && b.grab_node < 0) cand.push_back(&b);
        }
        int merges = 0;
        std::atomic<int> total{0};
        auto run = [&](int i, int) {
            const int k = cand[i]->coarsen_shells(settings.coarsen_per_frame, settings.coarsen_quiet);
            if (k) total.fetch_add(k, std::memory_order_relaxed);
        };
        if (settings.multithreaded && cand.size() > 1) JobSystem::get().parallel_items((int)cand.size(), run);
        else
            for (int i = 0; i < (int)cand.size(); i++) run(i, 0);
        merges = total.load();
        m_stats.shell_merges = merges;
    }
    m_stats.el.shell_evals = m_stats.shell_steps;
    m_stats.el.beam_evals = m_stats.beam_steps;
    if (settings.element_stats) count_elements();
    m_time += frame_time;
    m_stats.sim_time = m_time;
    m_stats.step_ms = prof::ticks_to_ms(prof::now() - t0);
}

void World::count_elements() {
    WorldStats::Elements& e = m_stats.el;
    const long long ns = e.node_steps, se = e.shell_evals, he = e.hinge_evals, be = e.beam_evals;
    e = WorldStats::Elements{};
    e.node_steps = ns, e.shell_evals = se, e.hinge_evals = he, e.beam_evals = be;
    for (const auto& bp : m_bodies) {
        const SoftBody& b = *bp;
        const bool awake = !b.sleeping;
        e.bodies++;
        e.bodies_awake += awake;
        if (b.is_piece) {
            e.pieces++;
            e.pieces_awake += awake;
            e.pieces_rigid += b.rigid;
        }
        e.nodes += (int)b.nodes.size();
        e.beams += (int)b.beams.size();
        e.tris += (int)b.tris.size();
        for (const Beam& bm : b.beams) e.beams_broken += (bm.flags & BF_BROKEN) ? 1 : 0;
        if (awake) {
            e.nodes_awake += (int)b.nodes.size();
            e.beams_awake += (int)b.beams.size();
            e.tris_awake += (int)b.tris.size();
        }
        if (b.shells.empty()) continue;
        const SoftBody::TopoCounts& tc = b.topo_counts();
        const int n = (int)b.shells.size();
        e.shells += n;
        e.hinges += tc.hinges;
        e.edges_border += tc.border;
        e.edges_crack += tc.cracks;
        e.edges_cut += tc.cuts;
        if (!b.is_piece) e.impacts += (int)b.shell_impacts.size(); // (the pieces carry copies)
        if (awake) {
            e.shells_awake += n;
            e.hinges_awake += tc.hinges;
            for (int l = 0; l < 5; l++) e.shells_level[l] += tc.level[l];
            e.shells_rate[std::min(2, b.dt_shift())] += n;
        }
    }
}

bool World::in_wind(const SoftBody& b) const {
    if (settings.wind_radius <= 0) return true;
    vec3 c = b.aabb.center();
    float r = settings.wind_radius + length(b.aabb.extent()) * 0.5f;
    return length2(vec3(c.x - settings.wind_focus.x, 0, c.z - settings.wind_focus.z)) < r * r;
}

void World::build_islands(float frame_time) {
    PROFILE_ZONE("Islands build");
    const int nb = (int)m_bodies.size();
    std::vector<int> parent(nb);
    std::iota(parent.begin(), parent.end(), 0);
    auto find = [&](int x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    std::vector<AABB> boxes(nb);
    const bool windy = length2(settings.wind) > 0.01f;
    for (int i = 0; i < nb; i++) {
        SoftBody& b = *m_bodies[i];
        if (windy && b.sleeping && !b.wind_area.empty() && in_wind(b)) b.wake();
        AABB a = b.aabb;
        float m = b.sleeping ? 0.05f : std::min(b.max_speed, 200.0f) * frame_time * 2.0f + 0.3f;
        a.expand(m + b.hull_reach());
        boxes[i] = a;
    }
    if (settings.inter_body_collisions) {
        std::vector<int> idx(nb);
        std::iota(idx.begin(), idx.end(), 0);
        std::sort(idx.begin(), idx.end(), [&](int a, int b) { return boxes[a].mn.x < boxes[b].mn.x; });
        for (int ii = 0; ii < nb; ii++) {
            int i = idx[ii];
            SoftBody& bi = *m_bodies[i];
            for (int jj = ii + 1; jj < nb; jj++) {
                int j = idx[jj];
                if (boxes[j].mn.x > boxes[i].mx.x) break;
                if (!boxes[i].overlaps(boxes[j])) continue;
                SoftBody& bj = *m_bodies[j];
                if (bi.sleeping && bj.sleeping) continue;
                if (bi.collision_group != 0 && bi.collision_group == bj.collision_group) continue;
                // only bodies that can actually touch share an island: contacter nodes vs the other's
                // triangles/capsules, or capsules vs triangles (trees don't interact with each other)
                auto has_contacters = [](const SoftBody& b) {
                    if (b.contacter_count < 0) {
                        int n = 0;
                        for (const auto& inf : b.info) n += (inf.flags & NF_CONTACTER) ? 1 : 0;
                        const_cast<SoftBody&>(b).contacter_count = n;
                    }
                    return b.contacter_count > 0;
                };
                bool ti = !bi.tris.empty(), tj = !bj.tris.empty(), ci = !bi.capsules.empty(), cj = !bj.capsules.empty();
                bool can = (has_contacters(bi) && (tj || cj)) || (has_contacters(bj) && (ti || ci)) || (ci && tj) || (cj && ti);
                // (a collision volume meets the other's contacter nodes, balls and volumes)
                const bool vi = !bi.volumes.empty() && !bi.volume_pass && !bj.volume_pass, vj = !bj.volumes.empty() && !bj.volume_pass && !bi.volume_pass;
                can = can || (vi && (has_contacters(bj) || cj || vj)) || (vj && (has_contacters(bi) || ci));
                if (!can) continue;
                int a = find(i), b = find(j);
                if (a != b) parent[a] = b;
            }
        }
    }
    // group
    std::vector<int> island_of(nb, -1);
    m_num_islands = 0;
    for (int i = 0; i < nb; i++) {
        int r = find(i);
        if (island_of[r] < 0) {
            island_of[r] = m_num_islands++;
            if ((int)m_islands.size() < m_num_islands) m_islands.push_back(std::make_unique<Island>());
            Island& isl = *m_islands[island_of[r]];
            isl.bodies.clear();
        }
        m_islands[island_of[r]]->bodies.push_back(m_bodies[i].get());
    }
    // drop fully sleeping islands, compute costs and static candidates
    int w = 0;
    m_stats.active_bodies = m_stats.sleeping_bodies = m_stats.active_nodes = m_stats.active_beams = 0;
    for (int i = 0; i < m_num_islands; i++) {
        Island& isl = *m_islands[i];
        bool awake = false;
        for (SoftBody* b : isl.bodies) awake |= !b->sleeping;
        if (!awake || !settings.sleeping) {
            if (!settings.sleeping)
                for (SoftBody* b : isl.bodies) b->sleeping = false;
            if (!awake && settings.sleeping) {
                m_stats.sleeping_bodies += (int)isl.bodies.size();
                continue;
            }
        }
        isl.cost = 0;
        isl.max_body_cost = 0;
        isl.team_cost = 0;
        isl.needs_pairs = settings.inter_body_collisions && (isl.bodies.size() > 1 || isl.bodies[0]->self_collision);
        for (SoftBody* b : isl.bodies) {
            if (b->sleeping) {
                m_stats.sleeping_bodies++;
                // a sleeping sheet in an island is about to be hit (it shares the island with something moving): the
                // frame it wakes up in is the most expensive one (refinement, cracks), so it counts for the team
                if (!b->shells.empty())
                    isl.team_cost += (double)(b->nodes.size() * 2 + b->shells.size() * 4) * (1 << b->dt_shift()) * 2;
                continue;
            }
            m_stats.active_bodies++;
            m_stats.active_nodes += b->node_count();
            m_stats.active_beams += (int)b->beams.size();
            // (a refined sheet takes 2^dt_shift short steps per substep)
            const double c = (double)(b->nodes.size() * 2 + b->beams.size() + b->joints.size() * 3 + b->fem.elems.size() * 40 + b->fem.tris.size() * 150 + b->shells.size() * 4) * (1 << b->dt_shift());
            isl.cost += c;
            isl.team_cost += c;
            isl.max_body_cost = std::max(isl.max_body_cost, c);
        }
        isl.box_ids.resize(isl.bodies.size());
        isl.cyl_ids.resize(isl.bodies.size());
        isl.terrain_max.resize(isl.bodies.size());
        for (size_t k = 0; k < isl.bodies.size(); k++) {
            SoftBody& b = *isl.bodies[k];
            AABB a = b.aabb;
            a.expand(std::min(b.max_speed, 200.0f) * frame_time * 2.0f + 0.5f);
            isl.box_ids[k].clear();
            isl.cyl_ids[k].clear();
            statics.query(a, isl.box_ids[k], isl.cyl_ids[k]);
            isl.terrain_max[k] = statics.has_terrain ? statics.terrain.max_height(a.mn.x, a.mn.z, a.mx.x, a.mx.z) +
                                                           (statics.road ? statics.road->max_raise : 0.0f)
                                                     : -1e30f;
        }
        if (w != i) std::swap(m_islands[w], m_islands[i]);
        w++;
    }
    m_num_islands = w;
    m_stats.islands = w;
}

void World::apply_wind(SoftBody& b) {
    // quadratic drag towards the (gusty) wind velocity on nodes that have a drag area (trees)
    vec3 wind = settings.wind;
    if (b.wind_area.empty() || length2(wind) < 1e-6f) return;
    float t = (float)m_time;
    const int n = (int)b.nodes.size();
    for (int i = 0; i < n; i++) {
        float area = b.wind_area[i];
        if (area <= 0) continue;
        const Node& nd = b.nodes[i];
        float gust = 1.0f + settings.wind_gusts * (0.6f * std::sin(t * 1.3f + nd.p.x * 0.11f + nd.p.z * 0.07f) +
                                                   0.4f * std::sin(t * 3.7f + nd.p.z * 0.23f + nd.p.y * 0.3f));
        vec3 rel = wind * gust - nd.v;
        b.force[i] += rel * (0.6f * area * length(rel));
    }
}

void World::collide_static(SoftBody& b, size_t n0, size_t n1, const std::vector<int>& box_ids, const std::vector<int>& cyl_ids, float terrain_max_h, float dt,
                           int& contacts) {
    const auto& gms = ground_models();
    const int nbox = (int)box_ids.size(), ncyl = (int)cyl_ids.size();
    const float kill_y = statics.kill_y;
    Node* nd = b.nodes.data();
    vec3* F = b.force.data();
    const NodeInfo* inf = b.info.data();
    const bool test_terrain = statics.has_terrain;
    // (a body at rest grips the ground harder: SoftBody::rest_friction)
    const float fric_body = b.ground_friction * (b.resting ? b.rest_friction : 1.0f);
    thread_local std::vector<std::pair<int, TyreContact>> tyre;
    tyre.clear();
    uint8_t* touch = b.ground_touch.size() == b.nodes.size() ? b.ground_touch.data() : nullptr;
    if (touch) // (bit 0: in this short step, the membrane's; bit 1: in this frame, the rest damping's support)
        for (size_t i = n0; i < n1; i++) touch[i] &= 2;
    vec3* pen_out = nullptr; // (the plates' mid points take what their corners leave: SoftBody::tri_mids)
    if (b.tri_mids && !b.fem.tris.empty()) {
        if (b.static_pen.size() != b.nodes.size()) b.static_pen.assign(b.nodes.size(), vec3(0));
        pen_out = b.static_pen.data();
        for (size_t i = n0; i < n1; i++) pen_out[i] = vec3(0);
    }
    for (int i = (int)n0; i < (int)n1; i++) {
        Node& x = nd[i];
        if (!(inf[i].flags & NF_GROUND) || x.inv_mass <= 0) continue;
        if (x.p.y < kill_y) { // fell off the world (also past the edge of the terrain)
            x.v = vec3(0);
            x.inv_mass = 0;
            continue;
        }
        float r = inf[i].radius;
        bool near_terrain = test_terrain && x.p.y - r <= terrain_max_h + 0.05f;
        if (!near_terrain && nbox == 0 && ncyl == 0) continue;
        // (the obstacles whose bounds the node is within reach of: most nodes of a sheet hung in a gate are near none)
        int bids[16], cids[16];
        const int* bp = box_ids.data();
        const int* cp = cyl_ids.data();
        int nb = nbox, nc = ncyl;
        if (nbox <= 16 && ncyl <= 16) {
            nb = nc = 0;
            const float rr = r + 0.01f;
            for (int k = 0; k < nbox; k++) {
                const AABB& a = statics.boxes[box_ids[k]].aabb;
                if (x.p.x >= a.mn.x - rr && x.p.x <= a.mx.x + rr && x.p.y >= a.mn.y - rr && x.p.y <= a.mx.y + rr && x.p.z >= a.mn.z - rr &&
                    x.p.z <= a.mx.z + rr)
                    bids[nb++] = box_ids[k];
            }
            for (int k = 0; k < ncyl; k++) {
                const StaticCylinder& cy = statics.cylinders[cyl_ids[k]];
                const float cr = cy.radius + rr;
                if (std::fabs(x.p.x - cy.base.x) <= cr && std::fabs(x.p.z - cy.base.z) <= cr && x.p.y >= cy.base.y - rr &&
                    x.p.y <= cy.base.y + cy.height + rr)
                    cids[nc++] = cyl_ids[k];
            }
            bp = bids;
            cp = cids;
            if (!near_terrain && nb == 0 && nc == 0) continue;
        }
        ContactInfo c;
        if (!statics.collide_point(x.p, r, bp, nb, cp, nc, near_terrain, c)) continue;
        contacts++;
        if (touch) touch[i] = 3;
        if (pen_out) pen_out[i] = c.normal * std::max(0.0f, c.depth);
        const GroundModel& gm = gms[c.surface < gms.size() ? c.surface : 0];
        if (c.max_force > 0) { // yielding obstacle (bush, bendable tree): it pushes back up to a limit
            vec3 f = primitive_collision(F[i], x.v, x.mass, c.normal, dt, gm, c.depth, inf[i].friction * fric_body, b.contact_push_max, b.contact_slop);
            float fl = length(f);
            if (fl > c.max_force) f *= c.max_force / fl;
            F[i] += f;
            if (!b.fem.empty())
                if (const int sl = b.fem.slot((uint32_t)i); sl >= 0 && sl < (int)b.fem.contact_f.size()) b.fem.contact_f[sl] += f;
        } else if (inf[i].flags & NF_TYRE) {
            TyreContact t = tyre_contact(F[i], x.v, x.mass, c.normal, dt, gm, c.depth, inf[i].friction * fric_body * settings.tyre_grip);
            F[i] += t.normal_force;
            if (t.touching) tyre.push_back({i, t});
        } else {
            const vec3 f = primitive_collision(F[i], x.v, x.mass, c.normal, dt, gm, c.depth, inf[i].friction * fric_body, b.contact_push_max, b.contact_slop, b.bounce);
            F[i] += f;
            // a frame node pressed on: its normal motion is held in the frame's implicit step (FemFrame::contact_n)
            if (!b.fem.empty())
                if (const int sl = b.fem.slot((uint32_t)i); sl >= 0 && sl < (int)b.fem.contact_n.size()) {
                    if (sl < (int)b.fem.contact_f.size()) b.fem.contact_f[sl] += f;
                    // (sticking: slower than 5 cm/s along the ground, the friction holding what pushes it along: twice
                    // the normal, and its sliding held too)
                    const float fn = dot(f, c.normal);
                    const vec3 ft = F[i] - c.normal * dot(F[i], c.normal), vt = x.v - c.normal * dot(x.v, c.normal);
                    if (fn > 0) b.fem.contact_n[sl] = c.normal * (length2(vt) < 0.05f * 0.05f && length(ft) < 0.3f * fn ? 2.0f : 1.0f);
                }
        }
    }
    if (tyre.empty()) return;
    // each wheel's contact patch shares its grip: stick if the patch as a whole can hold, else slide
    if (b.node_wheel.size() != b.nodes.size()) {
        b.node_wheel.assign(b.nodes.size(), -1);
        for (size_t k = 0; k < b.wheels.size(); k++)
            for (uint32_t ni : b.wheels[k].nodes) b.node_wheel[ni] = (int16_t)k;
    }
    const size_t nw = b.wheels.size();
    thread_local std::vector<vec3> need_sum;
    thread_local std::vector<float> cap_sum;
    need_sum.assign(nw, vec3(0));
    cap_sum.assign(nw, 0.0f);
    for (const auto& [i, t] : tyre)
        if (int w = b.node_wheel[i]; w >= 0) {
            need_sum[w] += t.need;
            cap_sum[w] += t.cap;
        }
    for (const auto& [i, t] : tyre) {
        int w = b.node_wheel[i];
        bool stick = w >= 0 ? length(need_sum[w]) <= cap_sum[w] : length(t.need) <= t.cap;
        F[i] += stick ? t.need : t.slide;
    }
}

// A FEM plate's mid point (SoftBody::tri_mids): its triangle's three body nodes; false if it is torn out, a corner is
// not `flag`'s or none can move
// The size of the plug body x's blow punches out of body y's plates (FemFrame::note_hit): x's least half extent, at
// most 25 cm (a ball its radius; a car's corner, not the car), and the two bodies' collision radii (the hole it goes
// through: smaller, it tore the hole wider again on its way)
static inline float plug(const SoftBody& x, const SoftBody& y) {
    const vec3 e = x.aabb.mx - x.aabb.mn;
    return 0.85f * (std::clamp(0.5f * std::min(e.x, std::min(e.y, e.z)), 0.02f, 0.25f) + x.collision_radius + y.collision_radius); // (the
    // punch's ring grows a fifth with the speed: about the contact's at a crash's)
}

// A body's mean velocity this substep (cached: the contacts' serial pass asks for it at a fast contact on a frame's
// plates - the blow lays a pattern by the bodies' speeds, not by a light node's shaking)
static vec3 body_velocity(SoftBody& b, double time) {
    if (b.vcm_time != time) {
        vec3 p(0);
        float m = 0;
        for (const Node& n : b.nodes)
            if (n.inv_mass > 0) p += n.v * n.mass, m += n.mass;
        b.vcm = m > 0 ? p / m : vec3(0);
        b.vcm_time = time;
    }
    return b.vcm;
}
// (the speed of a blow on body y's plates: its contact's approach speed, at most the two bodies' own)
static inline float blow_speed(SoftBody& x, SoftBody* y, vec3 nrm, float vn, double time) {
    if (vn < y->fem.pattern_speed()) return 0;
    return std::min(vn, std::fabs(dot(body_velocity(x, time) - body_velocity(*y, time), nrm)));
}

static inline bool tri_mid(const SoftBody& b, size_t k, uint32_t* n, uint16_t flag) {
    const FrameTri& t = b.fem.tris[k];
    if (t.broken) return false;
    float im = 0;
    for (int j = 0; j < 3; j++) {
        if (t.n[j] >= b.fem.node.size()) return false;
        n[j] = b.fem.node[t.n[j]];
        if (n[j] >= b.nodes.size() || !(b.info[n[j]].flags & flag)) return false;
        im += b.nodes[n[j]].inv_mass;
    }
    return im > 0;
}
static inline vec3 tri_mid_p(const SoftBody& b, const uint32_t* n) { return (b.nodes[n[0]].p + b.nodes[n[1]].p + b.nodes[n[2]].p) * (1.0f / 3.0f); }
// a plate against the collision volumes (collide_volumes): a FEM triangle (idx its FemFrame::tris) or a sheet
// triangle (its Shell), its middle p and how far its corners stand off it (rad); parts: the body's volumes whose parts
// hold all its corners, a bit each
using MidPt = SoftBody::MidPoint;
// ... its effective mass at its point of barycentric weights w (1 / sum w^2 / m), the point's velocity and the force
// on it so far (that mass times its acceleration: what a contact there cancels)
static inline float tri_state(const SoftBody& b, const uint32_t* n, const float* w, vec3& v, vec3& F) {
    float W = 0;
    vec3 acc(0);
    v = vec3(0);
    for (int j = 0; j < 3; j++) {
        const Node& x = b.nodes[n[j]];
        W += w[j] * w[j] * x.inv_mass;
        v += x.v * w[j];
        acc += b.force[n[j]] * (w[j] * x.inv_mass);
    }
    const float m = 1.0f / std::max(W, 1e-12f);
    F = acc * m;
    return m;
}
// A triangle against a collision volume's hull: the part of it inside (the triangle clipped by the hull's faces - none:
// they do not meet), and the way out for it: through the face that part is nearest behind (its deepest point's way out
// through each, the least) or across the triangle's plane (the hull's corners through it on the fewer side: a
// volume's corner or edge through the middle of a plate). The contact at the inside part's middle: its barycentric
// weights w, the normal (the triangle's way out) and the depth
static bool tri_hull(const CollisionVolume& V, const vec3* T, float* w, vec3& nrm, float& depth) {
    const size_t np = V.wplanes.size();
    for (size_t k = 0; k < np; k++) { // (all three corners out past one face: apart)
        const vec4& pl = V.wplanes[k];
        const vec3 n = pl.xyz();
        if (dot(n, T[0]) > pl.w && dot(n, T[1]) > pl.w && dot(n, T[2]) > pl.w) return false;
    }
    vec3 buf[2][24];
    int cnt = 3;
    buf[0][0] = T[0], buf[0][1] = T[1], buf[0][2] = T[2];
    int cur = 0;
    for (size_t k = 0; k < np && cnt > 0; k++) { // (Sutherland-Hodgman: kept where n . x <= d)
        const vec3 n = V.wplanes[k].xyz();
        const float d = V.wplanes[k].w;
        const vec3* in = buf[cur];
        vec3* out = buf[cur ^ 1];
        int m = 0;
        for (int i = 0; i < cnt && m < 22; i++) {
            const vec3 a = in[i], b = in[(i + 1) % cnt];
            const float sa = dot(n, a) - d, sb = dot(n, b) - d;
            if (sa <= 0) out[m++] = a;
            if ((sa < 0) != (sb < 0) && std::fabs(sa - sb) > 1e-12f) out[m++] = a + (b - a) * (sa / (sa - sb));
        }
        cnt = m, cur ^= 1;
    }
    if (cnt == 0) return false;
    const vec3* poly = buf[cur];
    vec3 c(0);
    for (int i = 0; i < cnt; i++) c += poly[i] / (float)cnt;
    float best = 1e30f;
    for (size_t k = 0; k < np; k++) {
        const vec3 n = V.wplanes[k].xyz();
        float e = 0;
        for (int i = 0; i < cnt; i++) e = std::max(e, V.wplanes[k].w - dot(n, poly[i]));
        if (e < best) best = e, nrm = n;
    }
    const vec3 tn = cross(T[1] - T[0], T[2] - T[0]);
    const float tl = length(tn);
    if (tl > 1e-12f) {
        const vec3 nt = tn / tl;
        float hmin = 1e30f, hmax = -1e30f;
        for (const vec3& h : V.wverts) {
            const float s = dot(nt, h - T[0]);
            hmin = std::min(hmin, s), hmax = std::max(hmax, s);
        }
        if (hmax > 0 && hmin < 0) {
            if (hmax < best) best = hmax, nrm = nt;   // (the plate moved along its normal past the hull's corners)
            if (-hmin < best) best = -hmin, nrm = -nt;
        }
        // (the middle's weights on the corners)
        const vec3 e0 = T[1] - T[0], e1 = T[2] - T[0], q = c - T[0];
        const float d00 = dot(e0, e0), d01 = dot(e0, e1), d11 = dot(e1, e1), d20 = dot(q, e0), d21 = dot(q, e1);
        const float den = d00 * d11 - d01 * d01;
        float v = den != 0 ? (d11 * d20 - d01 * d21) / den : 1.0f / 3, u = den != 0 ? (d00 * d21 - d01 * d20) / den : 1.0f / 3;
        v = clampf(v, 0, 1), u = clampf(u, 0, 1);
        if (v + u > 1) v /= v + u, u = 1 - v;
        w[0] = 1 - v - u, w[1] = v, w[2] = u;
    } else {
        w[0] = w[1] = w[2] = 1.0f / 3;
    }
    depth = best;
    return best > 0 && best < 1e29f;
}
// ... its effective mass at the middle (1 / sum w^2 / m), its velocity and the force on it so far (that mass times its
// acceleration: what a contact there cancels)
static inline float tri_mid_state(const SoftBody& b, const uint32_t* n, vec3& v, vec3& F) {
    constexpr float kW = 1.0f / 3.0f;
    float W = 0;
    vec3 acc(0);
    v = vec3(0);
    for (int j = 0; j < 3; j++) {
        const Node& x = b.nodes[n[j]];
        W += kW * kW * x.inv_mass;
        v += x.v * kW;
        acc += b.force[n[j]] * (kW * x.inv_mass);
    }
    const float m = 1.0f / std::max(W, 1e-12f);
    F = acc * m;
    return m;
}

void World::collide_static_mids(SoftBody& b, const std::vector<int>& box_ids, const std::vector<int>& cyl_ids, float terrain_max_h, float dt, int& contacts) {
    std::vector<StaticMid> hits;
    static_mids_find(b, 0, b.fem.tris.size(), box_ids, cyl_ids, terrain_max_h, hits);
    static_mids_apply(b, hits, dt, contacts);
}

void World::static_mids_find(const SoftBody& b, size_t k0, size_t k1, const std::vector<int>& box_ids, const std::vector<int>& cyl_ids, float terrain_max_h,
                             std::vector<StaticMid>& out) const {
    out.clear();
    static const bool off = getenv("BL_NOMIDS") != nullptr;
    if (off || !b.tri_mids || b.fem.tris.empty() || b.rigid || b.static_pen.size() != b.nodes.size() || b.force.size() != b.nodes.size()) return;
    const int nbox = (int)box_ids.size(), ncyl = (int)cyl_ids.size();
    const bool test_terrain = statics.has_terrain;
    if (!test_terrain && nbox == 0 && ncyl == 0) return;
    for (size_t k = k0; k < k1 && k < b.fem.tris.size(); k++) {
        uint32_t n[3];
        if (!tri_mid(b, k, n, NF_GROUND)) continue;
        const vec3 p = tri_mid_p(b, n);
        const bool near_terrain = test_terrain && p.y <= terrain_max_h + 0.05f;
        if (!near_terrain && nbox == 0 && ncyl == 0) continue;
        // (the obstacles it is within: most plates are near none)
        int bids[16], cids[16];
        const int* bp = box_ids.data();
        const int* cp = cyl_ids.data();
        int nb = nbox, nc = ncyl;
        if (nbox <= 16 && ncyl <= 16) {
            nb = nc = 0;
            for (int q = 0; q < nbox; q++) {
                const AABB& a = statics.boxes[box_ids[q]].aabb;
                if (p.x >= a.mn.x - 0.01f && p.x <= a.mx.x + 0.01f && p.y >= a.mn.y - 0.01f && p.y <= a.mx.y + 0.01f && p.z >= a.mn.z - 0.01f && p.z <= a.mx.z + 0.01f)
                    bids[nb++] = box_ids[q];
            }
            for (int q = 0; q < ncyl; q++) {
                const StaticCylinder& cy = statics.cylinders[cyl_ids[q]];
                const float cr = cy.radius + 0.01f;
                if (std::fabs(p.x - cy.base.x) <= cr && std::fabs(p.z - cy.base.z) <= cr && p.y >= cy.base.y - 0.01f && p.y <= cy.base.y + cy.height + 0.01f)
                    cids[nc++] = cyl_ids[q];
            }
            bp = bids;
            cp = cids;
            if (!near_terrain && nb == 0 && nc == 0) continue;
        }
        ContactInfo c;
        if (!statics.collide_point(p, 0.0f, bp, nb, cp, nc, near_terrain, c) || !(c.depth > 0)) continue;
        out.push_back({(uint32_t)k, {n[0], n[1], n[2]}, p, c});
    }
}

void World::static_mids_apply(SoftBody& b, const std::vector<StaticMid>& hits, float dt, int& contacts) {
    if (hits.empty()) return;
    const auto& gms = ground_models();
    const NodeInfo* inf = b.info.data();
    const vec3* pen = b.static_pen.data();
    vec3* F = b.force.data();
    const float fric_body = b.ground_friction * (b.resting ? b.rest_friction : 1.0f);
    if (b.mid_touch.size() != b.fem.tris.size()) b.mid_touch.assign(b.fem.tris.size(), 0);
    constexpr float kW = 1.0f / 3.0f;
    for (const StaticMid& h : hits) {
        const uint32_t* n = h.n;
        const ContactInfo& c = h.c;
        // (of its depth, what its corners' own contacts leave at the middle)
        const float depth = c.depth - kW * (std::max(0.0f, dot(pen[n[0]], c.normal)) + std::max(0.0f, dot(pen[n[1]], c.normal)) + std::max(0.0f, dot(pen[n[2]], c.normal)));
        if (depth <= 0) continue;
        vec3 v, Fm;
        const float m = tri_mid_state(b, n, v, Fm);
        const float fr = kW * (inf[n[0]].friction + inf[n[1]].friction + inf[n[2]].friction);
        const GroundModel& gm = gms[c.surface < gms.size() ? c.surface : 0];
        vec3 f = primitive_collision(Fm, v, m, c.normal, dt, gm, depth, fr * fric_body, b.contact_push_max, b.contact_slop, b.bounce);
        if (c.max_force > 0 && length(f) > c.max_force) f *= c.max_force / length(f);
        // (its middle's normal motion held in the frame's implicit step, FemFrame::tri_press: a plate on a beam under its
        // middle, pushed off by its mid point and back by its members, shook at 0.3 m/s)
        for (int j = 0; j < 3; j++) {
            F[n[j]] += f * kW;
            if (const int sl = b.fem.slot(n[j]); sl >= 0 && sl < (int)b.fem.contact_f.size()) b.fem.contact_f[sl] += f * kW;
        }
        if (dot(f, c.normal) > 0) {
            if (b.fem.tri_press.size() != b.fem.tris.size()) b.fem.tri_press.assign(b.fem.tris.size(), FemFrame::TriPress());
            b.fem.tri_press[h.k] = {c.normal, 200.0f * m, dot(Fm + f, c.normal) / m};
        }
        contacts++;
        b.mid_contacts++;
        b.mid_touch[h.k] |= 1;
        // (a fast blow on a plate lays its fracture pattern there: a pole's, a wall's)
        if (b.fem.patterned() && -dot(v, c.normal) >= b.fem.pattern_speed()) // (the body's own speed: not its light node's shaking)
            b.fem.note_hit(n[0], n[1], n[2], vec3(kW), std::min(-dot(v, c.normal), std::fabs(dot(body_velocity(b, m_time), c.normal))), 0.0f, m_time);
    }
}

// A candidate pair of a node and a collision triangle (the broadphase): within r of it; a hull triangle (one-sided,
// see Triangle::two_sided) also takes a node up to `depth` behind it over its face. tu: the triangle's unit normal.
static inline bool near_triangle(vec3 p, vec3 a, vec3 b, vec3 c, vec3 tu, float r, bool two_sided, float depth) {
    const float s = dot(p - a, tu);
    vec3 bary;
    if (two_sided) return std::fabs(s) <= r && length2(closest_on_triangle(p, a, b, c, bary) - p) <= r * r;
    if (s > r || s < -r - depth) return false;
    const vec3 q = p - tu * s; // (over the face, within r of it)
    return length2(closest_on_triangle(q, a, b, c, bary) - q) <= r * r;
}

// The collision volumes (CollisionVolume): placed on their anchors, then each against its body's parts (their nodes
// inside it), the other bodies of the island - their nodes inside it, their balls (a capsule of no length) reaching
// into it, their volumes' vertices inside it - and against the static world (its vertices; a pole through a face).
// Each contact as the ground's (primitive_collision) on the effective mass of the pair, the volume's side onto its
// anchors. Serial: a few volumes, a few contacts.
int World::collide_volumes(Island& isl, float dt, bool bodies, int stage) {
    PROFILE_ACCUM("Volumes");
    const GroundModel& gm = ground_models()[SURF_METAL];
    constexpr float kFric = 0.4f;
    if (stage != 2) {
    isl.vtasks.clear();
    bool any = false;
    for (SoftBody* b : isl.bodies) any |= !b->volumes.empty() && !b->volume_pass;
    if (!any) return 0;
    {
        // (their fits side by side: a crash's crushed volumes, a few iterations each)
        thread_local std::vector<std::pair<SoftBody*, uint32_t>> place;
        place.clear();
        for (SoftBody* b : isl.bodies)
            for (uint32_t k = 0; k < (uint32_t)b->volumes.size(); k++) place.push_back({b, k});
        std::vector<std::pair<SoftBody*, uint32_t>>& pl = place;
        if (isl.team) isl.team->run((int)pl.size(), [&](int i) { pl[i].first->place_volume(pl[i].second); });
        else
            for (auto& x : pl) x.first->place_volume(x.second);
    }
    static const bool mids_off = getenv("BL_NOMIDS") != nullptr;
    const int nb = (int)isl.bodies.size();
    // the bodies' plates' mid points this substep (SoftBody::tri_mids): each body's once, sorted along its longest axis
    // (two cars side by side share their x) - its FEM triangles' and its sheet's (a sheet triangle on FEM nodes is its FEM
    // triangle's), and of which of its volumes' parts all three corners are (the bit of each: a part's plate held off its
    // body's volume). A volume looks at those in its range (each volume over every triangle of the other car cost 8 ms a
    // frame in a head-on). Kept from substep to substep: their places moved and their order mended (insertion: nearly in
    // order), made again when the body's triangles, sheet or volumes' parts change
    using MidCache = SoftBody::MidCache;
    // (each body's on a thread of its own: below)
    auto mids_of = [&](int kb) -> MidCache& {
        SoftBody& B = *isl.bodies[kb];
        MidCache& mc = B.mid_cache;
        size_t nparts = 0;
        for (const CollisionVolume& cv : B.volumes) nparts += cv.parts.size();
        if (mc.ntris != B.fem.tris.size() || mc.nshells != B.shells.size() || mc.nparts != nparts || mc.nnodes != B.nodes.size()) {
            mc.ntris = B.fem.tris.size(), mc.nshells = B.shells.size(), mc.nparts = nparts, mc.nnodes = B.nodes.size();
            mc.pts.clear();
            std::vector<uint32_t> part_bits(B.nodes.size(), 0u);
            for (size_t k = 0; k < B.volumes.size() && k < 32; k++)
                for (uint32_t i : B.volumes[k].parts)
                    if (i < part_bits.size()) part_bits[i] |= 1u << k;
            mc.max_rad = 0;
            auto add = [&](const uint32_t* n, uint32_t idx, bool fem) {
                MidPt m;
                m.p = tri_mid_p(B, n);
                m.rad = 0;
                for (int j = 0; j < 3; j++) m.rad = std::max(m.rad, length(B.nodes[n[j]].p - m.p));
                m.rad += 0.05f; // (it stretches a little before it is made again)
                mc.max_rad = std::max(mc.max_rad, m.rad);
                m.n[0] = n[0], m.n[1] = n[1], m.n[2] = n[2];
                m.idx = idx, m.fem = fem;
                m.parts = part_bits[n[0]] & part_bits[n[1]] & part_bits[n[2]];
                mc.pts.push_back(m);
            };
            for (uint32_t k = 0; k < (uint32_t)B.fem.tris.size(); k++) {
                uint32_t n[3];
                if (tri_mid(B, k, n, NF_CONTACTER)) add(n, k, true);
            }
            const bool fem_tris = !B.fem.tris.empty();
            for (uint32_t si = 0; si < (uint32_t)B.shells.size(); si++) {
                const Shell& sh = B.shells[si];
                bool ok = true, on_fem = fem_tris;
                float im = 0;
                for (int j = 0; j < 3 && ok; j++) {
                    ok = sh.n[j] < B.nodes.size() && (B.info[sh.n[j]].flags & NF_CONTACTER);
                    if (ok) im += B.nodes[sh.n[j]].inv_mass, on_fem = on_fem && B.fem.slot(sh.n[j]) >= 0;
                }
                if (ok && !on_fem && im > 0) add(sh.n, si, false);
            }
            const vec3 ext = B.aabb.mx - B.aabb.mn;
            mc.axis = ext.x >= ext.y && ext.x >= ext.z ? 0 : ext.y >= ext.z ? 1 : 2;
            const int ax = mc.axis;
            std::sort(mc.pts.begin(), mc.pts.end(), [ax](const MidPt& a, const MidPt& b) { return a.p[ax] < b.p[ax]; });
            return mc;
        }
        // (their places now; a torn one out of reach; the order mended)
        const int ax = mc.axis;
        for (MidPt& m : mc.pts) {
            const bool gone = m.fem ? (m.idx >= B.fem.tris.size() || B.fem.tris[m.idx].broken)
                                    : (m.idx >= B.shells.size() || (B.shells[m.idx].tri < B.tris.size() && B.tris[B.shells[m.idx].tri].torn));
            m.p = gone ? vec3(1e30f) : tri_mid_p(B, m.n);
        }
        for (size_t i = 1; i < mc.pts.size(); i++)
            for (size_t j = i; j > 0 && mc.pts[j].p[ax] < mc.pts[j - 1].p[ax]; j--) std::swap(mc.pts[j], mc.pts[j - 1]);
        return mc;
    };
    auto overlap = [](vec3 amn, vec3 amx, vec3 bmn, vec3 bmx, float r) {
        return amn.x - r <= bmx.x && bmn.x - r <= amx.x && amn.y - r <= bmx.y && bmn.y - r <= amx.y && amn.z - r <= bmx.z && bmn.z - r <= amx.z;
    };
    using VTask = Island::VTask;
    using VHit = Island::VHit;
    std::vector<VTask>& vtasks = isl.vtasks;
    std::vector<std::vector<VHit>>& vhits = isl.vhits;
    vtasks.clear();
    for (int ka = 0; ka < nb; ka++) {
        SoftBody& A = *isl.bodies[ka];
        if (A.volumes.empty() || A.volume_pass || A.force.size() != A.nodes.size()) continue;
        for (CollisionVolume& V : A.volumes) {
            if (!V.placed || !(V.mass > 0)) continue;
            if (V.break_force > 0) { // (its contacts of the last substep against its crush force)
                V.crush = std::max(0.0f, V.crush + (V.load / V.break_force - 1.0f) * dt);
                V.load = 0;
                if (V.crush > CollisionVolume::kCrushTime) {
                    V.broken = true, V.placed = false;
                    continue;
                }
            }
            vtasks.push_back({ka, &V});
        }
    }
    if (vtasks.empty()) return 0;
    // (the plates' mid points of every body a volume may look at, made here: the chunks only read them)
    std::vector<const MidCache*>& mids = isl.vmids;
    mids.assign(nb, nullptr);
    isl.vbodies = bodies;
    if (!mids_off) {
        PROFILE_ACCUM("Volume mids");
        std::vector<int>& with = isl.vmid_bodies;
        with.clear();
        for (int kb = 0; kb < nb; kb++)
            if (isl.bodies[kb]->tri_mids) with.push_back(kb);
        auto one = [&](int q) { mids[with[q]] = &mids_of(with[q]); };
        if (isl.team) isl.team->run((int)with.size(), one);
        else
            for (int q = 0; q < (int)with.size(); q++) one(q);
    }
    // (each volume's look in parts, each a chunk of the team: its own body's parts, then per other body its nodes, its
    // plates, the rest of it, then the static world - the hits in that order; one chunk a volume, the bumpers' against
    // the other car were the whole stage)
    using VSub = Island::VSub;
    std::vector<VSub>& vsubs = isl.vsubs;
    std::vector<int>& vsub_ptr = isl.vsub_ptr;
    vsubs.clear(), vsub_ptr.assign(vtasks.size() + 1, 0);
    for (size_t t = 0; t < vtasks.size(); t++) {
        const int ka = vtasks[t].ka;
        const SoftBody& A = *isl.bodies[ka];
        const CollisionVolume& V = *vtasks[t].V;
        vsubs.push_back({(int)t, ka, 0, 0, 0});
        if (bodies)
            for (int kb = 0; kb < nb; kb++) {
                if (kb == ka) continue;
                const SoftBody& B = *isl.bodies[kb];
                if (B.volume_pass || (A.sleeping && B.sleeping) || B.force.size() != B.nodes.size()) continue;
                if (!overlap(V.mn, V.mx, B.aabb.mn, B.aabb.mx, 0.5f)) continue;
                vsubs.push_back({(int)t, kb, 1, 0, 0});
                // (the plates in the volume's range along their axis, in chunks: a bumper against the other car's
                // crushed front was one long chunk)
                if (B.tri_mids && mids[kb]) {
                    const MidCache& mc = *mids[kb];
                    const int ax = mc.axis;
                    const float lo = V.mn[ax] - mc.max_rad, hi = V.mx[ax] + mc.max_rad;
                    const uint32_t a = (uint32_t)(std::lower_bound(mc.pts.begin(), mc.pts.end(), lo, [ax](const MidPt& m, float x) { return m.p[ax] < x; }) - mc.pts.begin());
                    const uint32_t e = (uint32_t)(std::upper_bound(mc.pts.begin(), mc.pts.end(), hi, [ax](float x, const MidPt& m) { return x < m.p[ax]; }) - mc.pts.begin());
                    constexpr uint32_t kMidChunk = 48;
                    for (uint32_t c = a; c < e; c += kMidChunk) vsubs.push_back({(int)t, kb, 2, c, std::min(e, c + kMidChunk)});
                }
                vsubs.push_back({(int)t, kb, 3, 0, 0});
            }
        vsubs.push_back({(int)t, ka, 4, 0, 0});
        vsub_ptr[t + 1] = (int)vsubs.size();
    }
    if (vhits.size() < vsubs.size()) vhits.resize(vsubs.size());
    if (stage == 1) return (int)vsubs.size(); // (volume_find in the forces' team stage, then stage 2)
    {
        PROFILE_ACCUM("Volume find");
        if (isl.team) isl.team->run((int)vsubs.size(), [&](int sq) { volume_find(isl, sq); });
        else
            for (int t = 0; t < (int)vsubs.size(); t++) volume_find(isl, t);
    }
    }
    // the contacts' forces, in order
    PROFILE_ACCUM("Volume push");
    using VHit = Island::VHit;
    const std::vector<Island::VTask>& vtasks = isl.vtasks;
    const std::vector<std::vector<VHit>>& vhits = isl.vhits;
    const std::vector<int>& vsub_ptr = isl.vsub_ptr;
    for (size_t t = 0; t < vtasks.size(); t++) {
        const int ka = vtasks[t].ka;
        SoftBody& A = *isl.bodies[ka];
        CollisionVolume& V = *vtasks[t].V;
        const bool a_moves = !A.sleeping && !A.rigid;
        // a contact at p along n (out of the volume) pen deep, of a side of mass m moving at vel with force F on it;
        // returns the force on that side (the volume takes it back)
        auto contact = [&](vec3 p, vec3 n, float pen, float m, vec3 vel, vec3 F, bool other_moves) -> vec3 {
            const float mv = a_moves ? V.mass : 1e30f, mo = other_moves ? m : 1e30f;
            if (mv > 1e29f && mo > 1e29f) return vec3(0);
            const float me = 1.0f / (1.0f / mv + 1.0f / mo);
            const vec3 f = primitive_collision(other_moves ? F * (me / m) : vec3(0), vel - V.vel_at(p), me, n, dt, gm, pen, kFric);
            if (a_moves) A.push_volume(V, p, -f);
            V.load += length(f);
            return other_moves ? f : vec3(0);
        };
        for (int sq = vsub_ptr[t]; sq < vsub_ptr[t + 1]; sq++)
        for (const VHit& h : vhits[sq]) {
            SoftBody& B = *isl.bodies[h.kb];
            switch (h.kind) {
            case 0:
            case 2:
            case 4: {
                const Node& x = B.nodes[h.i];
                B.force[h.i] += contact(h.p, h.n, h.pen, x.mass, x.v, B.force[h.i], h.moves);
                if (h.kind != 0) B.body_contacts++;
                break;
            }
            case 1:
            case 3: {
                // (on the plate's effective mass there; the force onto the corners by their weights)
                vec3 v, Fm;
                const float me = tri_state(B, h.n3, h.w, v, Fm);
                const vec3 f = contact(h.p, h.n, h.pen, me, v, Fm, h.moves);
                for (int j = 0; j < 3; j++) B.force[h.n3[j]] += f * h.w[j];
                if (h.fem) {
                    if (B.mid_touch.size() != B.fem.tris.size()) B.mid_touch.assign(B.fem.tris.size(), 0);
                    if (h.i < B.mid_touch.size()) B.mid_touch[h.i] |= 4;
                }
                B.mid_contacts++;
                if (h.kind == 3) B.body_contacts++;
                break;
            }
            case 5: {
                CollisionVolume& W = B.volumes[h.i];
                const vec3 f = contact(h.p, h.n, h.pen, h.extra, W.vel_at(h.p), vec3(0), h.moves);
                if (h.moves) B.push_volume(W, h.p, f);
                break;
            }
            case 6: {
                const GroundModel& g = ground_models()[h.i < SURF_COUNT ? h.i : 0];
                vec3 f = primitive_collision(vec3(0), V.vel_at(h.p), V.mass / (float)h.n3[0], h.n, dt, g, h.pen, kFric);
                if (h.extra > 0 && length(f) > h.extra) f *= h.extra / length(f);
                A.push_volume(V, h.p, f);
                V.load += length(f);
                break;
            }
            default: {
                const GroundModel& g = ground_models()[h.i < SURF_COUNT ? h.i : 0];
                const vec3 f = primitive_collision(vec3(0), V.vel_at(h.p), V.mass, -h.n, dt, g, h.pen, kFric);
                A.push_volume(V, h.p - h.n * h.extra, f);
                V.load += length(f);
                break;
            }
            }
        }
    }
    return 0;
}

// One part of one volume's look (Island::VSub): the geometry alone, any order and thread (collide_volumes, stage 1:
// beside the forces); its hits in its own list
void World::volume_find(Island& isl, int sq) {
    PROFILE_ACCUM("Volume task");
    static prof::Zone* const part_zone[5] = {prof::zone("Volume own"), prof::zone("Volume nodes"), prof::zone("Volume plates"), prof::zone("Volume balls"), prof::zone("Volume static")};
    prof::AccumScope part_scope(part_zone[std::min<int>(4, isl.vsubs[sq].part)]);
    using VHit = Island::VHit;
    const std::vector<Island::VTask>& tasks = isl.vtasks;
    const std::vector<Island::VSub>& subs = isl.vsubs;
    std::vector<std::vector<VHit>>& hits = isl.vhits;
    const std::vector<const SoftBody::MidCache*>& mid_of = isl.vmids;
    using MidCache = SoftBody::MidCache;
    auto overlap = [](vec3 amn, vec3 amx, vec3 bmn, vec3 bmx, float r) {
        return amn.x - r <= bmx.x && bmn.x - r <= amx.x && amn.y - r <= bmx.y && bmn.y - r <= amx.y && amn.z - r <= bmx.z && bmn.z - r <= amx.z;
    };
    {
        PROFILE_ACCUM("Volume task");
        std::vector<VHit>& out = hits[sq];
        out.clear();
        const int t = subs[sq].t, part = subs[sq].part;
        const int ka = tasks[t].ka;
        SoftBody& A = *isl.bodies[ka];
        CollisionVolume& V = *tasks[t].V;
        const bool a_moves = !A.sleeping && !A.rigid;
        int face;
        const uint32_t vbit = (size_t)(&V - A.volumes.data()) < 32 ? 1u << (uint32_t)(&V - A.volumes.data()) : 0u;
        // a plate in it (a FEM triangle or a sheet's, SoftBody::tri_mids): the triangle against the hull (tri_hull), the
        // contact at the middle of its part inside, of the depth what its corners in it (their own contacts) leave at
        // that point
        auto mid = [&](const SoftBody& B, int kb, const MidPt& m, bool moves, uint8_t kind) {
            const vec3 T[3] = {B.nodes[m.n[0]].p, B.nodes[m.n[1]].p, B.nodes[m.n[2]].p};
            const vec3 lo = vmin(T[0], vmin(T[1], T[2])), hi = vmax(T[0], vmax(T[1], T[2]));
            if (hi.x < V.mn.x || lo.x > V.mx.x || hi.y < V.mn.y || lo.y > V.mx.y || hi.z < V.mn.z || lo.z > V.mx.z) return;
            float w[3], depth;
            vec3 nrm;
            if (!tri_hull(V, T, w, nrm, depth)) return;
            float held = 0; // (its corners inside: their own contacts, their share at the point)
            for (int j = 0; j < 3; j++) {
                int f2;
                const float s2 = V.depth(T[j], f2);
                if (s2 < 0) held -= w[j] * s2;
            }
            // (3 mm of it let be: a panel skin resting a hair into a volume flush with it shook, 14 mm/s rms against 9)
            constexpr float kSlop = 0.003f;
            if (depth - held <= kSlop) return;
            VHit h{};
            h.kind = kind, h.moves = moves, h.fem = m.fem, h.kb = kb, h.i = m.idx;
            for (int j = 0; j < 3; j++) h.n3[j] = m.n[j], h.w[j] = w[j];
            h.p = T[0] * w[0] + T[1] * w[1] + T[2] * w[2];
            h.n = nrm, h.pen = depth - held - kSlop;
            out.push_back(h);
        };
        // (the mid points of a body within the volume's range along the axis they are sorted by)
        auto in_x = [&](int kb, auto&& fn) {
            const MidCache& mc = *mid_of[kb];
            const int ax = mc.axis;
            const float lo = V.mn[ax] - mc.max_rad, hi = V.mx[ax] + mc.max_rad; // (a triangle reaching in from outside)
            auto it = std::lower_bound(mc.pts.begin(), mc.pts.end(), lo, [ax](const MidPt& m, float x) { return m.p[ax] < x; });
            for (; it != mc.pts.end() && it->p[ax] <= hi; ++it)
                if (it->p.x >= V.mn.x - it->rad && it->p.x <= V.mx.x + it->rad) fn(*it);
        };
        auto node_hit = [&](uint8_t kind, int kb, uint32_t i, vec3 p, vec3 n, float pen, bool moves) {
            VHit h{};
            h.kind = kind, h.moves = moves, h.kb = kb, h.i = i, h.p = p, h.n = n, h.pen = pen;
            out.push_back(h);
        };
        // its body's parts (a door pushed in, the hood folded back): their nodes out of it, the reaction on its anchors
        if (part == 0 && a_moves) {
            static prof::Zone* const zn = prof::zone("Volume own nodes");
            static prof::Zone* const zm = prof::zone("Volume own plates");
            prof::AccumScope sn(zn);
            for (uint32_t i : V.parts) {
                if (i >= A.nodes.size()) continue;
                const Node& x = A.nodes[i];
                if (x.inv_mass <= 0) continue;
                if (x.p.x < V.mn.x || x.p.x > V.mx.x || x.p.y < V.mn.y || x.p.y > V.mx.y || x.p.z < V.mn.z || x.p.z > V.mx.z) continue;
                const float s2 = V.depth_in(x.p, face);
                if (s2 >= 0) continue;
                node_hit(0, ka, i, x.p, V.wplanes[face].xyz(), -s2, true);
            }
            prof::AccumScope sm(zm);
            if (A.tri_mids && mid_of[ka] && vbit)
                in_x(ka, [&](const MidPt& m) { if (m.parts & vbit) mid(A, ka, m, true, 1); });
        }
        if (part >= 1 && part <= 3) {
            const int kb = subs[sq].kb;
            {
                const SoftBody& B = *isl.bodies[kb];
                const bool b_moves = !B.sleeping && !B.rigid;
                // its nodes
                for (size_t i = 0; part == 1 && i < B.nodes.size(); i++) {
                    const Node& x = B.nodes[i];
                    if (x.inv_mass <= 0 || !(B.info[i].flags & NF_CONTACTER)) continue;
                    if (x.p.x < V.mn.x || x.p.x > V.mx.x || x.p.y < V.mn.y || x.p.y > V.mx.y || x.p.z < V.mn.z || x.p.z > V.mx.z) continue;
                    const float s2 = V.depth_in(x.p, face);
                    if (s2 >= 0) continue;
                    node_hit(2, kb, (uint32_t)i, x.p, V.wplanes[face].xyz(), -s2, b_moves);
                }
                // its plates' mid points (a range of them)
                if (part == 2) {
                    const MidCache& mc = *mid_of[kb];
                    for (uint32_t q = subs[sq].i0; q < subs[sq].i1 && q < mc.pts.size(); q++) {
                        const MidPt& m = mc.pts[q];
                        if (m.p.x >= V.mn.x - m.rad && m.p.x <= V.mx.x + m.rad) mid(B, kb, m, b_moves, 3);
                    }
                }
                if (part != 3) return;
                // its balls
                for (const Capsule& cp : B.capsules) {
                    if (cp.a != cp.b || cp.a >= B.nodes.size()) continue;
                    const Node& x = B.nodes[cp.a];
                    if (x.inv_mass <= 0) continue;
                    const float s2 = V.depth(x.p, face);
                    if (s2 >= cp.radius) continue;
                    node_hit(4, kb, cp.a, x.p - V.wplanes[face].xyz() * s2, V.wplanes[face].xyz(), cp.radius - s2, b_moves);
                }
                // its volumes' vertices
                for (size_t wi = 0; wi < B.volumes.size(); wi++) {
                    const CollisionVolume& W = B.volumes[wi];
                    if (!W.placed || !(W.mass > 0) || !overlap(V.mn, V.mx, W.mn, W.mx, 0.0f)) continue;
                    const float mw = W.mass / (float)W.wverts.size();
                    for (const vec3& p : W.wverts) {
                        const float s2 = V.depth_in(p, face);
                        if (s2 >= 0) continue;
                        VHit h{};
                        h.kind = 5, h.moves = b_moves, h.kb = kb, h.i = (uint32_t)wi, h.p = p, h.n = V.wplanes[face].xyz(), h.pen = -s2, h.extra = mw;
                        out.push_back(h);
                    }
                }
            }
            return;
        }
        if (part != 4 || !a_moves || V.tyre) return; // (a ring tyre's: its tyre meets the static world)
        // the static world: its vertices (the effective mass shared by the ones touching), a pole through a face
        const std::vector<int>& bids = isl.box_ids[ka];
        const std::vector<int>& cids = isl.cyl_ids[ka];
        const bool near_terrain = statics.has_terrain && V.mn.y <= isl.terrain_max[ka] + 0.05f;
        const size_t first = out.size();
        for (int k = 0; k < (int)V.wverts.size(); k++) {
            ContactInfo ci;
            if (statics.collide_point(V.wverts[k], 0.0f, bids.data(), (int)bids.size(), cids.data(), (int)cids.size(), near_terrain, ci) && ci.depth > 0) {
                VHit h{};
                h.kind = 6, h.kb = ka, h.i = ci.surface, h.p = V.wverts[k], h.n = ci.normal, h.pen = ci.depth, h.extra = ci.max_force;
                out.push_back(h);
            }
        }
        const uint32_t nst = (uint32_t)(out.size() - first);
        for (size_t q = first; q < out.size(); q++) out[q].n3[0] = nst; // (how many share the mass)
        for (int id : cids) {
            if (id < 0 || id >= (int)statics.cylinders.size()) continue;
            const StaticCylinder& cy = statics.cylinders[id];
            if (cy.base.x + cy.radius < V.mn.x || cy.base.x - cy.radius > V.mx.x || cy.base.z + cy.radius < V.mn.z || cy.base.z - cy.radius > V.mx.z) continue;
            const float y0 = std::max(V.mn.y, cy.base.y), y1 = std::min(V.mx.y, cy.base.y + cy.height);
            if (y1 <= y0) continue;
            // (its axis at a few heights through the hull: the deepest one)
            float best = 1e30f;
            vec3 bp(0), bn(0);
            for (int m = 0; m < 5; m++) {
                const vec3 q(cy.base.x, y0 + (y1 - y0) * (m + 0.5f) / 5.0f, cy.base.z);
                const float s2 = V.depth(q, face);
                vec3 nh = V.wplanes[face].xyz();
                nh.y = 0;
                if (s2 < best && length2(nh) > 0.25f) best = s2, bp = q, bn = normalize(nh);
            }
            if (best >= cy.radius) continue;
            VHit h{};
            h.kind = 7, h.kb = ka, h.i = cy.surface, h.p = bp, h.n = bn, h.pen = cy.radius - best, h.extra = best;
            out.push_back(h);
        }
    }
}

// The ring tyres (Wheel::ring): each a wheel of its own - a rigid body (the rim and the tyre) on a bearing at each of its
// axle's nodes - its tyre a ring of tread strips round the rim, each on two sidewalls of its own. A strip's rows are
// pressed into the static world (the ground, a curb, a wall), each sidewall pushing back on its two rows with a
// stiffness and a damping per area of the tyre (folding over, crushed - Wheel::fold_at); the rows in contact held on the
// ground by their shear (a brush: each point's own since it came into the patch, as the rim turns it through) up to the
// ground's grip, sliding past it (the rubber keeps 85% of it), let go as they leave; the tread at the patch - the belt -
// shifted on the rim along the tread and across it by what the brush passes on (the sidewalls bending: a spring and a
// damper each way, in series with the brush - solved implicitly over the patch's rows, stuck and then the sliding ones
// at their grip; the strips round the patch shifted with it, less the further off it they are). Onto
// the wheel: the ground's pushes at the rows, its weight, the bearings' (implicit springs - their reaction onto the
// axle nodes); the drive's moment about the axle, the rolling resistance and the brake (it holds the spin up to its
// torque) - the drive's and the brake's reaction onto the wheel's arm.
void World::ring_tyres(SoftBody& b, const std::vector<int>& box_ids, const std::vector<int>& cyl_ids, float terrain_max_h, float dt) {
    const auto& gms = ground_models();
    Node* nd = b.nodes.data();
    vec3* F = b.force.data();
    const float fric_body = b.ground_friction * (b.resting ? b.rest_friction : 1.0f) * settings.tyre_grip;
    constexpr int kRows = Wheel::kRingRows;
    for (Wheel& w : b.wheels) {
        if (!w.ring || w.detached || w.axle0 >= b.nodes.size() || w.axle1 >= b.nodes.size()) continue;
        Node& n0 = nd[w.axle0];
        Node& n1 = nd[w.axle1];
        const vec3 hd = n1.p - n0.p;
        const float L = length(hd);
        if (!(L > 1e-4f) || !(w.mass > 0) || !(w.inertia > 0) || !(w.inertia_t > 0)) continue;
        const vec3 ah = hd / L;
        // (not placed yet, or its nodes moved away from it - a body set somewhere else: on its hub again)
        if (!w.seated || length2((n0.p + n1.p) * 0.5f - w.pos) > 0.25f * 0.25f) {
            w.seated = false;
            b.seat_wheel(w);
        }
        const mat3 Rw = to_mat3(w.rot);
        const vec3 c = w.pos, a = Rw.c[0], e1 = Rw.c[1], e2 = Rw.c[2];
        w.ref = e1, w.angle = 0;
        const vec3 om = w.omega(), om_t = om - a * dot(om, a); // (its turn across the axle: the sidewalls' damping)
        const int N = std::max(8, w.ring_n);
        if ((int)w.shear.size() != N * kRows) w.shear.assign(N * kRows, vec3(0));
        if ((int)w.squash.size() != N || (int)w.carcass.size() != N || (int)w.fold.size() != 2 * N || (int)w.side_sq.size() != 2 * N) {
            w.squash.assign(N, 0.0f), w.shift.assign(N, vec3(0)), w.carcass.assign(N, vec2(0));
            w.side_sq.assign(2 * N, 0.0f), w.fold.assign(2 * N, 0.0f);
        }
        const float R = w.radius, W = w.width, rim = std::min(w.rim_radius, 0.95f * R);
        const float wall = std::max(0.01f, R - rim);
        const float arc = 2.0f * kPi * R / (float)N;
        const bool terrain = statics.has_terrain && c.y - R <= terrain_max_h + 0.05f;
        vec3 Ft(0), Mt(0);
        float load = 0, lat_most = 0;
        bool crushed = false;
        if (terrain || !box_ids.empty() || !cyl_ids.empty()) {
            const float dA = arc * W / (float)kRows;
            const float dAs = 2.0f * kPi * (R - 0.5f * wall) / (float)(N / 2) * 0.5f * wall;
            const float kb = w.k_shear * dA, cb = w.c_shear * dA;
            const float sb = w.fold_at * wall, sc = w.crush_at * wall;
            auto touch = [&](vec3 q, ContactInfo& ci) {
                return statics.collide_point(q, 0.0f, box_ids.data(), (int)box_ids.size(), cyl_ids.data(), (int)cyl_ids.size(), terrain && q.y <= terrain_max_h + 0.05f, ci) &&
                       ci.depth > 0;
            };
            // a sidewall's push per area pressed in by s: up to its fold linear, beyond it (folded) softer, crushed stiff
            auto law = [&](float s, float folded) {
                float sig = w.k_area * (std::min(s, sb) + (1.0f - (1.0f - w.fold_soft) * folded) * std::max(0.0f, s - sb));
                if (s > sc) sig += w.crush_k * w.k_area * (s - sc);
                return sig;
            };
            struct Row {
                vec3 q, n, v, u, u1, f; // (its point, the ground's normal, its base's velocity along the ground, its shear and
                                        // the new one, its push along the ground)
                float cap, wt;          // (its grip, its strip's share of the belt's shift)
                int at;
                bool slide;
            };
            thread_local std::vector<Row> rw;
            rw.clear();
            // the belt: the tread's shift on the rim at the patch, its strips the more of it the nearer they are to it
            const vec3 dp = normalize_or(w.belt_dir - a * dot(w.belt_dir, a), -e1);
            vec3 nsum(0), dsum(0);
            float fold_most = 0;
            for (int i = 0; i < N; i++) {
                const float th = 2.0f * kPi * (float)i / (float)N;
                const vec3 d = e1 * std::cos(th) + e2 * std::sin(th), t = cross(a, d);
                const float wt = clampf((dot(d, dp) - 0.5f) * 2.0f, 0.0f, 1.0f);
                const vec3 Sw = w.belt * wt;
                const float lat = dot(Sw, a);
                float* fo = &w.fold[2 * i];
                float* ss = &w.side_sq[2 * i];
                float side[2] = {0.0f, 0.0f};
                for (int j = 0; j < kRows; j++) {
                    vec3& u = w.shear[i * kRows + j];
                    const float y = ((float)j - 0.5f * (kRows - 1)) * W / (float)kRows;
                    const vec3 q = c + a * y + d * R + Sw;
                    ContactInfo ci;
                    if (!touch(q, ci)) {
                        u = vec3(0);
                        continue;
                    }
                    // (the tread shifted towards axle1 leans on the left sidewall: its outer row the most)
                    const int sd = j < kRows / 2 ? 0 : 1;
                    const float s = std::min(ci.depth, 1.2f * wall) - w.lean * lat * (y / (0.375f * W));
                    if (!(s > 0)) {
                        u = vec3(0);
                        continue;
                    }
                    side[sd] = std::max(side[sd], s);
                    const vec3 n = ci.normal;
                    // (damped as the rim comes down on the ground there, not as its tread turns into the patch: that is
                    // the rolling resistance, taken apart below; a folded sidewall's rubber the more, a crushed one -
                    // the rim's flange through it - as it is stiff)
                    const float damp = w.c_area * (1.0f + 2.0f * fo[sd] + (s > sc ? w.crush_k : 0.0f));
                    const float fn = (law(s, fo[sd]) - damp * dot(w.vel + cross(om_t, q - c), n)) * dA;
                    if (!(fn > 0)) {
                        u = vec3(0);
                        continue;
                    }
                    const GroundModel& gm = gms[ci.surface < gms.size() ? ci.surface : 0];
                    const vec3 vq = w.vel + cross(om, q - c);
                    Row r;
                    r.q = q, r.n = n, r.v = vq - n * dot(vq, n), r.u = u - n * dot(u, n), r.u1 = vec3(0), r.f = vec3(0);
                    r.cap = fn * gm.ms * gm.strength * w.grip * fric_body, r.wt = wt, r.at = i * kRows + j, r.slide = false;
                    rw.push_back(r);
                    nsum += n * fn, dsum += d * fn;
                    if (wt > 0.5f) fold_most = std::max(fold_most, std::max(fo[0], fo[1]));
                    Ft += n * fn;
                    Mt += cross(q - c, n * fn);
                    load += fn;
                }
                // each sidewall's fold: coming on past 1.1 of the fold's pressing-in, off below 0.8 of it, over ~3 ms
                for (int sd = 0; sd < 2; sd++) {
                    const float target = side[sd] > 1.1f * sb ? 1.0f : side[sd] < 0.8f * sb ? 0.0f : fo[sd];
                    fo[sd] += (target - fo[sd]) * std::min(1.0f, dt / 0.003f);
                    crushed |= side[sd] > sc;
                    ss[sd] = side[sd];
                }
                w.squash[i] = std::max(side[0], side[1]);
                w.carcass[i] = vec2(dot(Sw, t), lat);
                w.shift[i] = Sw; // (and its rows' mean shear: below)
                // the sidewalls' faces (every other point round them): pushed by a wall, a curb's face; their rubber's grip
                if (i % 2) continue;
                for (int j = 0; j < 2; j++) {
                    const vec3 q = c + a * ((j ? 0.5f : -0.5f) * W) + d * (R - 0.5f * wall);
                    ContactInfo ci;
                    if (!touch(q, ci)) continue;
                    const vec3 vq = w.vel + cross(om, q - c);
                    const float fn = (w.k_area * std::min(ci.depth, 0.5f * W) - w.c_area * dot(w.vel + cross(om_t, q - c), ci.normal)) * dAs;
                    if (!(fn > 0)) continue;
                    const vec3 vt = vq - ci.normal * dot(vq, ci.normal);
                    const float vl = length(vt);
                    const vec3 f = ci.normal * fn - (vl > 1e-4f ? vt * (std::min(0.8f * fn, vl * w.c_shear * dAs) / vl) : vec3(0));
                    Ft += f;
                    Mt += cross(q - c, f);
                    load += fn;
                }
            }
            // the belt's shift along the tread and across it: the rows stuck (the sliding ones at their grip, fixed), the
            // sidewalls' springs and dampers against the brush's - implicit; in steady rolling it holds still and the
            // brush alone takes the slip (a cornering tyre's stiffness as the brush's), as it changes it lags (its
            // relaxation length)
            const float ry = w.c_lat / (w.c_lat + w.k_lat * dt), rx = w.c_long / (w.c_long + w.k_long * dt);
            const vec3 ns = length2(nsum) > 0 ? normalize(nsum) : vec3(0);
            vec3 ag = a - ns * dot(a, ns);
            if (!rw.empty() && length2(ns) > 0 && length2(ag) > 0.01f) {
                ag = normalize(ag);
                vec3 tg = cross(ag, ns);
                if (dot(tg, cross(a, normalize_or(dsum, dp))) < 0) tg = -tg;
                const float soft = 1.0f - 0.3f * fold_most; // (a folded sidewall holds less across)
                const float kx = w.k_long, cx = w.c_long, ky = w.k_lat * soft, cy = w.c_lat * soft;
                const vec2 B(dot(w.belt, tg), dot(w.belt, ag));
                auto solve = [&]() {
                    vec3 base(0);
                    float m = 0;
                    for (const Row& r : rw) {
                        if (r.slide) base += r.f * r.wt;
                        else base += ((r.u - r.v * dt) * kb - r.v * cb) * r.wt, m += r.wt * r.wt * (kb + cb / dt);
                    }
                    return vec2((dot(base, tg) + (m + cx / dt) * B.x) / (m + kx + cx / dt), (dot(base, ag) + (m + cy / dt) * B.y) / (m + ky + cy / dt));
                };
                auto rows = [&](vec2 B2, bool mark) {
                    const vec3 Bd = (tg * (B2.x - B.x) + ag * (B2.y - B.y)) / dt;
                    bool any = false;
                    for (Row& r : rw) {
                        if (r.slide) continue;
                        const vec3 vb = r.v + Bd * r.wt;
                        r.u1 = r.u - vb * dt;
                        vec3 f = r.u1 * kb - vb * cb;
                        const float fl = length(f);
                        if (fl > r.cap) { // (sliding: the rubber keeps 85% of the grip, the shear what that holds)
                            f *= 0.85f * r.cap / fl;
                            r.u1 = f / kb;
                            if (mark) r.slide = any = true;
                        }
                        r.f = f;
                    }
                    return any;
                };
                vec2 B1 = solve();
                if (rows(B1, true)) B1 = solve(), rows(B1, false);
                w.belt = std::isfinite(B1.x) && std::isfinite(B1.y) ? tg * B1.x + ag * B1.y : vec3(0);
                lat_most = std::fabs(B1.y);
            } else
                w.belt = w.belt * std::min(rx, ry); // (in the air: let back)
            for (const Row& r : rw) {
                Ft += r.f;
                Mt += cross(r.q - c, r.f);
                w.shear[r.at] = r.u1;
                w.shift[r.at / kRows] += r.u1 / (float)kRows;
            }
            if (length2(dsum) > 0) w.belt_dir = normalize(dsum);
        } else {
            std::fill(w.shear.begin(), w.shear.end(), vec3(0));
            std::fill(w.squash.begin(), w.squash.end(), 0.0f);
            std::fill(w.side_sq.begin(), w.side_sq.end(), 0.0f);
            std::fill(w.fold.begin(), w.fold.end(), 0.0f);
            w.belt = w.belt * std::min(w.c_long / (w.c_long + w.k_long * dt), w.c_lat / (w.c_lat + w.k_lat * dt)); // (let back)
            const vec3 dp = normalize_or(w.belt_dir - a * dot(w.belt_dir, a), -e1);
            for (int i = 0; i < N; i++) {
                const float th = 2.0f * kPi * (float)i / (float)N;
                const vec3 d = e1 * std::cos(th) + e2 * std::sin(th);
                const vec3 Sw = w.belt * clampf((dot(d, dp) - 0.5f) * 2.0f, 0.0f, 1.0f);
                w.carcass[i] = vec2(dot(Sw, cross(a, d)), dot(Sw, a));
                w.shift[i] = Sw;
            }
        }
        // the bearings: each axle node holds the wheel's axle at its place along it (a spring and a damper stepped
        // implicitly on the pair's mass - the node's, half the wheel's with its turn across - so stiff at any step)
        vec3 Fb(0), Mb(0);
        for (int k = 0; k < 2; k++) {
            Node& hn = k ? n1 : n0;
            const vec3 r = a * (k ? w.bearing_half : -w.bearing_half), x = hn.p - (c + r), xv = hn.v - (w.vel + cross(om, r));
            const float mu = 1.0f / (hn.inv_mass + 2.0f / w.mass + 2.0f * w.bearing_half * w.bearing_half / w.inertia_t);
            const vec3 v1 = (xv - x * (dt * w.bearing_k / mu)) / (1.0f + dt * w.bearing_c / mu + dt * dt * w.bearing_k / mu);
            const vec3 f = (x + v1 * dt) * w.bearing_k + v1 * w.bearing_c;
            F[k ? w.axle1 : w.axle0] -= f;
            Fb += f;
            Mb += cross(r, f);
        }
        // its motion: the ground's push, the bearings', its weight; its moments and the drive's
        const float Td = w.propulsed == 2 ? -w.torque : w.torque;
        w.vel += (Ft + Fb) * (dt / w.mass) + settings.gravity * dt;
        w.pos += w.vel * dt;
        w.mom += (Mt + Mb + a * Td) * dt;
        // the spin about the axle (the world's: the differential is on the body - read against the hub's turn, the hubs'
        // wind-up under the drive's reaction, +-2.4 rad/s, fed the differential's coupling and the driven wheels ran at
        // 25% slip; read off the wheel's arm, the suspension's travel - the arm on the body - pumped 285 kW into a car of
        // 158): the rolling resistance (its load times Crr at its radius, no more than stops it); the brake - its caliper
        // on the hub - holding the spin on the hub (the hub's frame node's turn about the axle, an axle of frame
        // elements; else the world's) up to its torque
        float spin = dot(w.omega(), a), Tb = 0;
        const float s0 = spin, rr = dt * w.crr * load * R / w.inertia;
        spin = std::fabs(spin) <= rr ? 0.0f : spin - std::copysign(rr, spin);
        if (w.brake > 0) {
            float hub = 0;
            if (const int sl = b.fem.slot(w.axle0); sl >= 0 && sl < (int)b.fem.w.size()) hub = dot(b.fem.w[sl], a);
            const float rel = spin - hub, lim = dt * w.brake / w.inertia;
            if (std::fabs(rel) <= lim) Tb = -rel * w.inertia / dt, spin = hub;
            else Tb = -std::copysign(w.brake, rel), spin -= std::copysign(lim, rel);
        }
        w.mom += a * ((spin - s0) * w.inertia);
        if (!std::isfinite(spin) || !std::isfinite(dot(w.mom, w.mom)) || !std::isfinite(dot(w.vel, w.pos))) { // (on its hub again)
            w.spin = 0, w.seated = false;
            b.seat_wheel(w);
        } else
            w.spin = spin;
        // its turn over the step
        const vec3 om1 = w.omega();
        if (const float wl = length(om1); wl * dt > 1e-9f) w.rot = normalize(quat::axis_angle(om1 / wl, wl * dt) * w.rot);
        w.load = load;
        w.lat_most = lat_most;
        if (crushed && !w.crushed) w.pinches++; // (a sidewall crushed: the rim's flange on the ground through it - once a blow)
        w.crushed = crushed;
        // the drive's and the brake's reaction onto the wheel's support (as Wheel's arm takes the node wheels')
        const float T = Td + Tb;
        if (w.arm >= 0 && w.near_attach >= 0 && std::fabs(T) > 0.01f) {
            const vec3 rr2 = nd[w.arm].p - nd[w.near_attach].p, r = rr2 - ah * dot(rr2, ah);
            const float off = length(rr2 - r), rl = length(r);
            if (rl > 0.01f && 2 * off < rl) {
                const vec3 cf = cross(ah, r / rl) * (0.5f * T / rl) * (1.0f - 2.0f * off / rl);
                F[w.arm] -= cf;
                F[w.near_attach] += cf;
            }
        }
        w.torque = 0;
    }
}

void World::rebuild_pairs(Island& isl) {
    PROFILE_ACCUM("Broadphase");
    m_rebuilds.fetch_add(1, std::memory_order_relaxed);
    static const bool bp_dbg = getenv("BL_BPDBG") != nullptr; // slow rebuilds -> stderr
    const double bp_t0 = bp_dbg ? time_seconds() : 0.0;
    double bp_t[8] = {};
    int bp_k = 0;
    int bp_mark = 0;
    auto bp_lap = [&]() {
        if (bp_dbg && bp_k < 8) bp_t[bp_k++] = time_seconds();
        if (prof::g_trace) prof::trace_mark("bp", bp_mark);
        bp_mark++;
    };
    isl.nt.clear();
    isl.nc.clear();
    isl.ct.clear();
    const int nb = (int)isl.bodies.size();
    float top_speed = 0, rmax = 0;
    bool any_caps = false, any_tris = false;
    for (SoftBody* b : isl.bodies) {
        top_speed = std::max(top_speed, b->max_speed);
        rmax = std::max(rmax, b->collision_radius);
        any_caps |= !b->capsules.empty();
        any_tris |= !b->tris.empty();
        if (!b->sleeping)
            for (Triangle& t : b->tris)
                if (!t.torn && t.rest_edge2 > 0 && max_edge2(b->nodes[t.a].p, b->nodes[t.b].p, b->nodes[t.c].p) > kTornStretch2 * t.rest_edge2) t.torn = true;
    }
    // The rebuild interval is the collision detection's (WorldSettings::collision_hz: 17 substeps at 120 Hz), Verlet-list
    // style: the candidate margin covers the relative motion until the next rebuild. The margin follows the slow bodies'
    // speed; a fast body (projectile, a car at speed) gets its own extra margin and is paired separately, so one bullet
    // flying into a pile does not widen the whole pile's margins. (near_pairs then keeps the pairs within reach.)
    constexpr float kFastSpeed = 8.0f;
    float slow_speed = 0;
    for (SoftBody* b : isl.bodies)
        if (b->max_speed <= kFastSpeed) slow_speed = std::max(slow_speed, b->max_speed);
    (void)top_speed;
    const float speed = slow_speed + 0.5f;
    isl.slow_speed = speed;
    const int interval = std::max(1, (int)std::ceil(1.0 / ((double)std::max(1.0f, settings.collision_hz) * isl.dt) - 1e-6));
    // A fast body's schedule follows its bulk speed (mass-weighted RMS of its nodes, +25%), not its fastest node: the top
    // of a spinning wheel moves at twice the car's speed. Its nodes get margins of their own speed (fast_pairs).
    isl.bulk.assign(nb, 0.0f);
    float fast_speed = 0;
    for (int bi = 0; bi < nb; bi++) {
        SoftBody& b = *isl.bodies[bi];
        if (b.sleeping || b.max_speed <= kFastSpeed) continue;
        double mv2 = 0, m = 0;
        for (const Node& x : b.nodes) {
            mv2 += (double)x.mass * length2(x.v);
            m += x.mass;
        }
        isl.bulk[bi] = clampf(m > 0 ? 1.25f * (float)std::sqrt(mv2 / m) : b.max_speed, kFastSpeed, 400.0f);
        fast_speed = std::max(fast_speed, isl.bulk[bi]);
    }
    isl.rebuild_interval = interval;
    const float margin = 2.0f * speed * interval * isl.dt + 0.02f; // between slow bodies
    isl.margin = margin;
    isl.extra.assign(nb, 0.0f);
    bool any_fast = false;
    for (int bi = 0; bi < nb; bi++)
        if (isl.bodies[bi]->max_speed > kFastSpeed && !isl.bodies[bi]->sleeping) {
            isl.extra[bi] = isl.bulk[bi] * interval * isl.dt;
            any_fast = true;
        }
    // Cells: 1 m, finer where a dense mesh (a refined sheet) meets small queries: a query scans every node of the
    // cells its box touches, a 1 m cell of a finely cracked sheet holds hundreds of them.
    {
        double tri_size = 0;
        int samples = 0;
        for (SoftBody* b : isl.bodies)
            for (size_t t = 0; t < b->tris.size(); t += 7) {
                const Triangle& tr = b->tris[t];
                tri_size += std::sqrt(max_edge2(b->nodes[tr.a].p, b->nodes[tr.b].p, b->nodes[tr.c].p));
                samples++;
            }
        const float mean = samples ? (float)(tri_size / samples) : 1.0f;
        isl.inv_cell = 1.0f / clampf(mean + 2.0f * (rmax + margin), 0.25f, 1.0f);
    }
    bp_lap(); // 0: speeds, margins, cell size
    const float inv_cell = isl.inv_cell;
    auto cell_of = [&](float v) { return (int)std::floor(v * inv_cell); };
    auto same_group = [&](int a, int b) {
        int ga = isl.bodies[a]->collision_group, gb = isl.bodies[b]->collision_group;
        return ga != 0 && ga == gb;
    };

    // ---- 0) partners: the bodies whose (reach expanded) boxes overlap. Only nodes inside a partner's box go into
    // the hash and only primitives near a partner query it: a dense sheet would otherwise scan hundreds of its own
    // nodes per triangle on every rebuild (and a projectile next to it makes that every other substep).
    const float reach = rmax + margin;
    isl.boxes.resize(nb);
    isl.partners.resize(nb);
    for (int bi = 0; bi < nb; bi++) {
        isl.boxes[bi] = isl.bodies[bi]->aabb;
        isl.boxes[bi].expand(reach + isl.extra[bi] + isl.bodies[bi]->hull_reach());
        isl.partners[bi].clear();
    }
    for (int a = 0; a < nb; a++)
        for (int b = a + 1; b < nb; b++) {
            if (same_group(a, b) || (isl.bodies[a]->sleeping && isl.bodies[b]->sleeping) || !isl.boxes[a].overlaps(isl.boxes[b])) continue;
            // (their spheres meet: two sheets with sphere_contacts; a ball and such a sheet or a sphere_target)
            const SoftBody &ba = *isl.bodies[a], &bb = *isl.bodies[b];
            if ((ba.sphere_contacts && bb.sphere_contacts) || (ba.sphere_ball > 0 && bb.sphere_ball <= 0 && (bb.sphere_contacts || bb.sphere_target)) ||
                (bb.sphere_ball > 0 && ba.sphere_ball <= 0 && (ba.sphere_contacts || ba.sphere_target)))
                continue;
            isl.partners[a].push_back(b);
            isl.partners[b].push_back(a);
        }
    auto near_partner = [&](int bi, vec3 mn, vec3 mx) {
        if (isl.bodies[bi]->self_collision) return true;
        for (int o : isl.partners[bi]) {
            const AABB& q = isl.boxes[o];
            if (mn.x <= q.mx.x && mx.x >= q.mn.x && mn.y <= q.mx.y && mx.y >= q.mn.y && mn.z <= q.mx.z && mx.z >= q.mn.z) return true;
        }
        return false;
    };

    bp_lap(); // 1: partners
    // ---- 1) spatial hash of contacter nodes (one cell per node, Teschner et al. 2003)
    isl.pts.clear();
    isl.entries.clear();
    for (int bi = 0; bi < nb; bi++) {
        SoftBody& b = *isl.bodies[bi];
        if ((isl.partners[bi].empty() && !b.self_collision) || isl.extra[bi] > 0) continue; // (fast bodies: step 2b)
        for (uint32_t ni = 0; ni < b.nodes.size(); ni++) {
            if (!(b.info[ni].flags & NF_CONTACTER)) continue;
            vec3 p = b.nodes[ni].p;
            if (!near_partner(bi, p, p)) continue;
            Island::Pt pt{(uint16_t)bi, ni, cell_of(p.x), cell_of(p.y), cell_of(p.z), p};
            isl.entries.push_back({cell_hash(pt.x, pt.y, pt.z), (uint32_t)isl.pts.size()});
            isl.pts.push_back(pt);
        }
    }
    auto build_buckets = [&](uint32_t& mask) {
        uint32_t nbuckets = 64;
        while (nbuckets < isl.entries.size() * 2) nbuckets <<= 1;
        mask = nbuckets - 1;
        isl.bucket_start.assign(nbuckets + 1, 0);
        for (auto& e : isl.entries) isl.bucket_start[(e.first & mask) + 1]++;
        for (uint32_t i = 0; i < nbuckets; i++) isl.bucket_start[i + 1] += isl.bucket_start[i];
        isl.bucket_items.resize(isl.entries.size());
        isl.fill.assign(isl.bucket_start.begin(), isl.bucket_start.end() - 1);
        for (auto& e : isl.entries) isl.bucket_items[isl.fill[e.first & mask]++] = e.second;
    };
    bp_lap(); // 2: node list
    if (!isl.pts.empty() && (any_tris || any_caps)) {
        uint32_t mask;
        build_buckets(mask);
        // nodes went in body by body and the bucket fill is stable: in every bucket the nodes of one body form a run.
        // run_end lets a primitive jump over its own body's nodes in one step.
        isl.run_end.resize(isl.bucket_items.size());
        for (uint32_t bk = 0; bk + 1 < (uint32_t)isl.bucket_start.size(); bk++) {
            uint32_t e = isl.bucket_start[bk + 1];
            for (uint32_t k = e; k-- > isl.bucket_start[bk];) {
                bool same = k + 1 < e && isl.pts[isl.bucket_items[k + 1]].body == isl.pts[isl.bucket_items[k]].body;
                isl.run_end[k] = same ? isl.run_end[k + 1] : k + 1;
            }
        }
        // ---- 2) triangles and capsules query the node hash, in chunks of triangles (shared by the team)
        constexpr uint32_t kTriChunk = 16; // (a crash's queries: 512 a chunk were a few chunks of 0.3 ms)
        isl.qchunks.clear();
        for (int bt = 0; bt < nb; bt++) {
            SoftBody& o = *isl.bodies[bt];
            if (isl.partners[bt].empty() && !o.self_collision) continue;
            const uint32_t ntri = (uint32_t)o.tris.size();
            for (uint32_t t0 = 0;; t0 += kTriChunk) {
                const uint32_t t1 = std::min(ntri, t0 + kTriChunk);
                isl.qchunks.push_back({(uint16_t)bt, 0, t0, t1, t1 == ntri});
                if (t1 == ntri) break;
            }
        }
        bp_lap(); // 3: buckets
        const int nq = (int)isl.qchunks.size();
        if ((int)isl.q_nt.size() < nq) {
            isl.q_nt.resize(nq);
            isl.q_nc.resize(nq);
        }
        auto query = [&](int qc) {
            PROFILE_ACCUM("Broadphase query");
            const Island::QChunk& q = isl.qchunks[qc];
            auto& out_nt = isl.q_nt[qc];
            auto& out_nc = isl.q_nc[qc];
            out_nt.clear();
            out_nc.clear();
            const int bt = q.body;
            SoftBody& o = *isl.bodies[bt];
            const float rq0 = rmax + margin + isl.extra[bt];
            for (uint32_t t = q.t0; t < q.t1; t++) {
                const Triangle& tr = o.tris[t];
                if (tr.torn) continue;
                const float depth = tr.two_sided ? 0.0f : o.hull_depth, rq = rq0 + depth; // (a hull triangle: its depth behind too)
                vec3 a = o.nodes[tr.a].p, bb = o.nodes[tr.b].p, c = o.nodes[tr.c].p;
                vec3 tmn = vmin(a, vmin(bb, c)), tmx = vmax(a, vmax(bb, c));
                if (!near_partner(bt, tmn - vec3(rq), tmx + vec3(rq))) continue;
                const vec3 tn = cross(bb - a, c - a);
                const float tn2 = dot(tn, tn);
                const vec3 tu = tn2 > 1e-20f ? tn * (1.0f / std::sqrt(tn2)) : vec3(0);
                int x0 = cell_of(tmn.x - rq), x1 = cell_of(tmx.x + rq);
                int y0 = cell_of(tmn.y - rq), y1 = cell_of(tmx.y + rq);
                int z0 = cell_of(tmn.z - rq), z1 = cell_of(tmx.z + rq);
                if ((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > 512) continue; // degenerate/exploded
                for (int z = z0; z <= z1; z++)
                    for (int y = y0; y <= y1; y++)
                        for (int x = x0; x <= x1; x++) {
                            uint32_t bk = cell_hash(x, y, z) & mask;
                            for (uint32_t k = isl.bucket_start[bk]; k < isl.bucket_start[bk + 1]; k++) {
                                const Island::Pt& pt = isl.pts[isl.bucket_items[k]];
                                if (pt.body == bt && !o.self_collision) { // own nodes: skip the whole run
                                    k = isl.run_end[k] - 1;
                                    continue;
                                }
                                if (pt.x != x || pt.y != y || pt.z != z) continue; // hash collision
                                // (out of the widest reach of the triangle's box: most of a cell's nodes)
                                if (pt.p.x < tmn.x - rq || pt.p.y < tmn.y - rq || pt.p.z < tmn.z - rq || pt.p.x > tmx.x + rq || pt.p.y > tmx.y + rq || pt.p.z > tmx.z + rq) continue;
                                const int bn = pt.body;
                                SoftBody& b = *isl.bodies[bn];
                                bool self = bn == bt;
                                if (self && (!b.self_collision || tr.a == pt.node || tr.b == pt.node || tr.c == pt.node)) continue;
                                if (!self && same_group(bn, bt)) continue;
                                if (b.sleeping && o.sleeping) continue;
                                vec3 p = b.nodes[pt.node].p;
                                float r = b.collision_radius + margin + isl.extra[bt];
                                const float rb = r + depth;
                                if (p.x < tmn.x - rb || p.y < tmn.y - rb || p.z < tmn.z - rb || p.x > tmx.x + rb || p.y > tmx.y + rb || p.z > tmx.z + rb) continue;
                                if (!near_triangle(p, a, bb, c, tu, r, tr.two_sided, depth)) continue;
                                out_nt.push_back({(uint16_t)bn, (uint16_t)bt, pt.node, t});
                            }
                        }
            }
            if (!q.caps) return;
            for (uint32_t ci = 0; ci < o.capsules.size(); ci++) {
                const Capsule& cp = o.capsules[ci];
                if (cp.joint >= 0 && o.joints[cp.joint].broken) continue;
                vec3 a = o.nodes[cp.a].p, bb = o.nodes[cp.b].p;
                float rq2 = cp.radius + rmax + margin + isl.extra[bt];
                vec3 mn = vmin(a, bb) - vec3(rq2), mx = vmax(a, bb) + vec3(rq2);
                if (!near_partner(bt, mn, mx)) continue;
                int x0 = cell_of(mn.x), x1 = cell_of(mx.x), y0 = cell_of(mn.y), y1 = cell_of(mx.y), z0 = cell_of(mn.z), z1 = cell_of(mx.z);
                if ((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > 512) continue;
                for (int z = z0; z <= z1; z++)
                    for (int y = y0; y <= y1; y++)
                        for (int x = x0; x <= x1; x++) {
                            uint32_t bk = cell_hash(x, y, z) & mask;
                            for (uint32_t k = isl.bucket_start[bk]; k < isl.bucket_start[bk + 1]; k++) {
                                const Island::Pt& pt = isl.pts[isl.bucket_items[k]];
                                if (pt.body == bt) {
                                    k = isl.run_end[k] - 1;
                                    continue;
                                }
                                if (pt.x != x || pt.y != y || pt.z != z) continue;
                                const int bn = pt.body;
                                if (same_group(bn, bt)) continue;
                                SoftBody& b = *isl.bodies[bn];
                                if (b.sleeping && o.sleeping) continue;
                                vec3 p = b.nodes[pt.node].p;
                                float r = cp.radius + b.collision_radius + margin + isl.extra[bt];
                                float s = closest_on_segment(p, a, bb);
                                if (length2(a + (bb - a) * s - p) > r * r) continue;
                                out_nc.push_back({(uint16_t)bn, (uint16_t)bt, pt.node, ci});
                            }
                        }
            }
        };
        isl.team->run(nq, query);
        for (int q = 0; q < nq; q++) {
            isl.nt.insert(isl.nt.end(), isl.q_nt[q].begin(), isl.q_nt[q].end());
            isl.nc.insert(isl.nc.end(), isl.q_nc[q].begin(), isl.q_nc[q].end());
        }
    }
    // ---- 2b) nodes of the fast bodies (not in the hash) against the primitives of their partners
    // The fast bodies' nodes are paired with their partners' primitives with a margin for their travel until the next
    // rebuild. Next to a dense mesh (a finely cracked sheet) that pairs a cannonball's nodes with every triangle within
    // half a metre (150k pairs per substep): then they get a schedule of their own with a few centimetres of margin.
    // (their own schedule: a refresh every ~6 cm of bulk travel, margins of a few centimetres; the pairs of a car in a
    // finely cracked sheet with the slow schedule's margin would be tens of thousands per substep)
    bp_lap(); // 4: queries
    isl.nt_fast = isl.nt.size();
    isl.nc_fast = isl.nc.size();
    isl.fast_interval = interval; // (refreshed with the rest: collision_hz; their margins cover their travel until then)
    isl.fast_dense = false;
    (void)fast_speed;
    if (any_fast) fast_pairs(isl);
    bp_lap(); // 5: fast pairs
    // ---- 3) capsules vs triangles (only when the island has both, e.g. trees vs vehicles)
    if (any_caps && any_tris) {
        isl.prims.clear();
        isl.entries.clear();
        for (int bi = 0; bi < nb; bi++) {
            SoftBody& b = *isl.bodies[bi];
            if (isl.partners[bi].empty() && !b.self_collision) continue;
            const float r = b.collision_radius + margin + isl.extra[bi];
            for (uint32_t t = 0; t < b.tris.size(); t++) {
                const Triangle& tr = b.tris[t];
                if (tr.torn) continue;
                vec3 a = b.nodes[tr.a].p, bb = b.nodes[tr.b].p, c = b.nodes[tr.c].p;
                vec3 mn = vmin(a, vmin(bb, c)) - vec3(r), mx = vmax(a, vmax(bb, c)) + vec3(r);
                if (!near_partner(bi, mn, mx)) continue;
                int x0 = cell_of(mn.x), x1 = cell_of(mx.x), y0 = cell_of(mn.y), y1 = cell_of(mx.y), z0 = cell_of(mn.z), z1 = cell_of(mx.z);
                if ((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > 512) continue;
                uint32_t pid = (uint32_t)isl.prims.size();
                isl.prims.push_back({(uint16_t)bi, 0, t});
                for (int z = z0; z <= z1; z++)
                    for (int y = y0; y <= y1; y++)
                        for (int x = x0; x <= x1; x++) isl.entries.push_back({cell_hash(x, y, z), pid});
            }
        }
        uint32_t mask;
        build_buckets(mask);
        for (int bc = 0; bc < nb; bc++) {
            SoftBody& b = *isl.bodies[bc];
            for (uint32_t ci = 0; ci < b.capsules.size(); ci++) {
                const Capsule& cp = b.capsules[ci];
                if (cp.joint >= 0 && b.joints[cp.joint].broken) continue;
                vec3 a = b.nodes[cp.a].p, bb = b.nodes[cp.b].p;
                float rr = cp.radius + b.collision_radius + margin + isl.extra[bc];
                vec3 mn = vmin(a, bb) - vec3(rr), mx = vmax(a, bb) + vec3(rr);
                int x0 = cell_of(mn.x), x1 = cell_of(mx.x), y0 = cell_of(mn.y), y1 = cell_of(mx.y), z0 = cell_of(mn.z), z1 = cell_of(mx.z);
                if ((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > 512) continue;
                size_t start = isl.ct.size();
                for (int z = z0; z <= z1; z++)
                    for (int y = y0; y <= y1; y++)
                        for (int x = x0; x <= x1; x++) {
                            uint32_t bk = cell_hash(x, y, z) & mask;
                            for (uint32_t k = isl.bucket_start[bk]; k < isl.bucket_start[bk + 1]; k++) {
                                const PrimRef& pr = isl.prims[isl.bucket_items[k]];
                                if (pr.body == bc || same_group(bc, pr.body)) continue;
                                SoftBody& o = *isl.bodies[pr.body];
                                if (b.sleeping && o.sleeping) continue;
                                const Triangle& t = o.tris[pr.idx];
                                vec3 ta = o.nodes[t.a].p, tb = o.nodes[t.b].p, tc = o.nodes[t.c].p;
                                vec3 tmn = vmin(ta, vmin(tb, tc)), tmx = vmax(ta, vmax(tb, tc));
                                if (tmx.x < mn.x || tmx.y < mn.y || tmx.z < mn.z || tmn.x > mx.x || tmn.y > mx.y || tmn.z > mx.z) continue;
                                isl.ct.push_back({(uint16_t)bc, pr.body, ci, pr.idx});
                            }
                        }
                // dedupe (a capsule can see the same triangle through several cells)
                std::sort(isl.ct.begin() + start, isl.ct.end(), [](const Island::CT& x, const Island::CT& y) {
                    return x.bt != y.bt ? x.bt < y.bt : x.tri < y.tri;
                });
                isl.ct.erase(std::unique(isl.ct.begin() + start, isl.ct.end(), [](const Island::CT& x, const Island::CT& y) {
                    return x.bt == y.bt && x.tri == y.tri;
                }), isl.ct.end());
            }
        }
    }
    // ---- 4) the FEM plates' mid points (SoftBody::tri_mids) against the other bodies' triangles: the mid points of the
    // bodies with partners in a hash of their own, the partners' triangles query it (the margins as the nodes': the
    // triangle's body's and the mid point's, fast or slow; each mid point in one cell, a triangle scans each cell once)
    if (prof::g_trace) prof::trace_mark("bp", 10);
    isl.mt.clear();
    static const bool no_mids = getenv("BL_NOMIDS") != nullptr;
    if (!no_mids && any_tris) {
        isl.mpts.clear();
        isl.mpos.clear();
        isl.entries.clear();
        float mid_extra = 0;
        for (int bi = 0; bi < nb; bi++) {
            SoftBody& b = *isl.bodies[bi];
            if (!b.tri_mids || b.fem.tris.empty() || b.rigid || isl.partners[bi].empty()) continue;
            for (uint32_t k = 0; k < (uint32_t)b.fem.tris.size(); k++) {
                uint32_t n[3];
                if (!tri_mid(b, k, n, NF_CONTACTER)) continue;
                const vec3 p = tri_mid_p(b, n);
                if (!near_partner(bi, p, p)) continue;
                Island::Pt pt{(uint16_t)bi, k, cell_of(p.x), cell_of(p.y), cell_of(p.z), p};
                isl.entries.push_back({cell_hash(pt.x, pt.y, pt.z), (uint32_t)isl.mpts.size()});
                isl.mpts.push_back(pt);
                isl.mpos.push_back(p);
                mid_extra = std::max(mid_extra, isl.extra[bi]);
            }
        }
        if (prof::g_trace) prof::trace_mark("bp", 11);
        if (!isl.mpts.empty()) {
            uint32_t mask;
            build_buckets(mask);
            if (prof::g_trace) prof::trace_mark("bp", 12);
            // (the partners' triangles in chunks shared by the team, each chunk's pairs of its own, then in order)
            struct MChunk {
                int body;
                uint32_t t0, t1;
            };
            std::vector<MChunk> mch;
            for (int bt = 0; bt < nb; bt++) {
                SoftBody& o = *isl.bodies[bt];
                if (isl.partners[bt].empty() || o.tris.empty()) continue;
                bool mids_near = false; // (a partner with mid points)
                for (int q : isl.partners[bt]) mids_near |= isl.bodies[q]->tri_mids && !isl.bodies[q]->fem.tris.empty();
                if (!mids_near) continue;
                for (uint32_t t0 = 0; t0 < (uint32_t)o.tris.size(); t0 += 8) mch.push_back({bt, t0, std::min((uint32_t)o.tris.size(), t0 + 8)});
            }
            if ((int)isl.q_mt.size() < (int)mch.size()) isl.q_mt.resize(mch.size());
            isl.team->run((int)mch.size(), [&](int qc) {
                PROFILE_ACCUM("Broadphase query");
                auto& out = isl.q_mt[qc];
                out.clear();
                const int bt = mch[qc].body;
                const SoftBody& o = *isl.bodies[bt];
                const float rq = rmax + margin + isl.extra[bt] + mid_extra;
                for (uint32_t t = mch[qc].t0; t < mch[qc].t1; t++) {
                    const Triangle& tr = o.tris[t];
                    if (tr.torn) continue;
                    const vec3 a = o.nodes[tr.a].p, bb = o.nodes[tr.b].p, c = o.nodes[tr.c].p;
                    const vec3 tmn = vmin(a, vmin(bb, c)), tmx = vmax(a, vmax(bb, c));
                    if (!near_partner(bt, tmn - vec3(rq), tmx + vec3(rq))) continue;
                    const vec3 tn = cross(bb - a, c - a);
                    const float tn2 = dot(tn, tn);
                    const vec3 tu = tn2 > 1e-20f ? tn * (1.0f / std::sqrt(tn2)) : vec3(0);
                    const int x0 = cell_of(tmn.x - rq), x1 = cell_of(tmx.x + rq), y0 = cell_of(tmn.y - rq), y1 = cell_of(tmx.y + rq), z0 = cell_of(tmn.z - rq),
                              z1 = cell_of(tmx.z + rq);
                    if ((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > 512) continue;
                    for (int z = z0; z <= z1; z++)
                        for (int y = y0; y <= y1; y++)
                            for (int x = x0; x <= x1; x++) {
                                const uint32_t bk = cell_hash(x, y, z) & mask;
                                for (uint32_t kk = isl.bucket_start[bk]; kk < isl.bucket_start[bk + 1]; kk++) {
                                    const uint32_t mi = isl.bucket_items[kk];
                                    const Island::Pt& pt = isl.mpts[mi];
                                    if (pt.x != x || pt.y != y || pt.z != z || pt.body == bt) continue;
                                    if (pt.p.x < tmn.x - rq || pt.p.y < tmn.y - rq || pt.p.z < tmn.z - rq || pt.p.x > tmx.x + rq || pt.p.y > tmx.y + rq || pt.p.z > tmx.z + rq) continue;
                                    if (same_group(pt.body, bt)) continue;
                                    const SoftBody& b = *isl.bodies[pt.body];
                                    if (b.sleeping && o.sleeping) continue;
                                    const vec3 p = isl.mpos[mi];
                                    const float r = b.collision_radius + margin + isl.extra[bt] + isl.extra[pt.body];
                                    if (p.x < tmn.x - r || p.y < tmn.y - r || p.z < tmn.z - r || p.x > tmx.x + r || p.y > tmx.y + r || p.z > tmx.z + r) continue;
                                    if (!near_triangle(p, a, bb, c, tu, r, true, 0.0f)) continue;
                                    out.push_back({pt.body, (uint16_t)bt, pt.node, t});
                                }
                            }
                }
            });
            if (prof::g_trace) prof::trace_mark("bp", 13);
            for (size_t qc = 0; qc < mch.size(); qc++) isl.mt.insert(isl.mt.end(), isl.q_mt[qc].begin(), isl.q_mt[qc].end());
        }
    }
    if (prof::g_trace) prof::trace_mark("bp", 14);
    isl.pair_count = (int)(isl.nt.size() + isl.nc.size() + isl.ct.size() + isl.mt.size());
    if (bp_t0 > 0) {
        double ms = (time_seconds() - bp_t0) * 1000.0;
        if (ms > 0.1) {
            std::string parts;
            double prev = bp_t0;
            for (int k = 0; k < bp_k; k++) {
                parts += format("%.3f ", (bp_t[k] - prev) * 1000.0);
                prev = bp_t[k];
            }
            fprintf(stderr, "BP parts: %s\n", parts.c_str());
            size_t ntri = 0, ncap = 0;
            for (SoftBody* b : isl.bodies) ntri += b->tris.size(), ncap += b->capsules.size();
            fprintf(stderr, "BP %.2f ms: bodies %zu pts %zu tris %zu caps %zu entries %zu -> nt %zu nc %zu ct %zu mt %zu teamed %d interval %d margin %.3f rmax %.3f cell %.2f\n", ms,
                    isl.bodies.size(), isl.pts.size(), ntri, ncap, isl.entries.size(), isl.nt.size(), isl.nc.size(), isl.ct.size(), isl.mt.size(), (int)isl.teamed,
                    isl.rebuild_interval, margin, rmax, 1.0f / isl.inv_cell);
        }
    }
}

// Pairs of the fast bodies' nodes with their partners' triangles and capsules (margin: the fast body's travel until
// the next refresh, see fast_interval). The fast body's contacter nodes go into a small hash grid of their own
// (cells about the reach); each partner triangle scans only the cells its reach box touches instead of every node
// of the fast body. Pairs come out per triangle in node order, as a full scan would give them.
void World::fast_pairs(Island& isl) {
    PROFILE_ACCUM("Fast pairs");
    const int nb = (int)isl.bodies.size();
    // the same fast bodies as at the last full rebuild (their nodes are not in the hash), margins for this schedule
    auto fast_extra = [&](int b) {
        if (isl.extra[b] <= 0) return 0.0f;
        return isl.bulk[b] * isl.fast_interval * isl.dt;
    };
    const float travel = 1.25f * isl.fast_interval * isl.dt; // a node's own margin: its speed x this
    // (the fast pairs are refreshed every fast_interval: each side needs a margin for its own motion until then, a slow
    // partner at most the slow bodies' top speed; not the slow schedule's margin)
    const float slow_reach = 2.0f * isl.slow_speed * isl.fast_interval * isl.dt + 0.01f;
    const double t_g0 = getenv("BL_FASTDBG") ? time_seconds() : 0.0;
    if (isl.grids.size() < (size_t)nb) isl.grids.resize(nb);
    constexpr uint32_t kTriChunk = 16;
    isl.qchunks.clear();
    for (int bf = 0; bf < nb; bf++) {
        if (fast_extra(bf) <= 0) continue;
        SoftBody& F = *isl.bodies[bf];
        float rmax = 0;
        for (int bo : isl.partners[bf]) rmax = std::max(rmax, F.collision_radius + slow_reach + fast_extra(bo));
        Island::Grid& G = isl.grids[bf];
        // cells about the reach, at most 48 per axis (a long body gets longer cells)
        const vec3 ext = F.aabb.extent() + vec3(2e-3f);
        float cellsz = clampf(2.0f * rmax, 0.15f, 2.0f);
        cellsz = std::max(cellsz, maxc(ext) / 48.0f);
        G.inv_cell = 1.0f / cellsz;
        G.origin = F.aabb.mn - vec3(1e-3f);
        for (int a = 0; a < 3; a++) G.dim[a] = std::max(1, std::min(48, (int)(ext[a] * G.inv_cell) + 1));
        const uint32_t ncell = (uint32_t)(G.dim[0] * G.dim[1] * G.dim[2]);
        auto cell_index = [&](vec3 p) {
            const int x = std::clamp((int)((p.x - G.origin.x) * G.inv_cell), 0, G.dim[0] - 1);
            const int y = std::clamp((int)((p.y - G.origin.y) * G.inv_cell), 0, G.dim[1] - 1);
            const int z = std::clamp((int)((p.z - G.origin.z) * G.inv_cell), 0, G.dim[2] - 1);
            return (uint32_t)((z * G.dim[1] + y) * G.dim[0] + x);
        };
        const uint32_t nn = (uint32_t)F.nodes.size();
        G.start.assign(ncell + 1, 0);
        // every node goes into the cells within its own travel until the next refresh (a few are in two or more)
        auto cells_of = [&](uint32_t ni, auto&& fn) {
            const vec3 p = F.nodes[ni].p;
            const float m = length(F.nodes[ni].v) * travel;
            const uint32_t lo = cell_index(p - vec3(m)), hi = cell_index(p + vec3(m));
            if (lo == hi) {
                fn(lo);
                return;
            }
            const int x0 = (int)(lo % G.dim[0]), y0 = (int)(lo / G.dim[0] % G.dim[1]), z0 = (int)(lo / (G.dim[0] * G.dim[1]));
            const int x1 = (int)(hi % G.dim[0]), y1 = (int)(hi / G.dim[0] % G.dim[1]), z1 = (int)(hi / (G.dim[0] * G.dim[1]));
            for (int z = z0; z <= z1; z++)
                for (int y = y0; y <= y1; y++)
                    for (int x = x0; x <= x1; x++) fn((uint32_t)((z * G.dim[1] + y) * G.dim[0] + x));
        };
        uint32_t count = 0;
        for (uint32_t ni = 0; ni < nn; ni++)
            if (F.info[ni].flags & NF_CONTACTER) cells_of(ni, [&](uint32_t c) {
                G.start[c + 1]++;
                count++;
            });
        for (uint32_t i = 0; i < ncell; i++) G.start[i + 1] += G.start[i];
        G.items.resize(count);
        isl.fill.assign(G.start.begin(), G.start.end() - 1);
        for (uint32_t ni = 0; ni < nn; ni++) // (in node order: every cell lists its nodes ascending)
            if (F.info[ni].flags & NF_CONTACTER) cells_of(ni, [&](uint32_t c) { G.items[isl.fill[c]++] = ni; });
        for (int bo : isl.partners[bf]) {
            const uint32_t ntri = (uint32_t)isl.bodies[bo]->tris.size();
            for (uint32_t t0 = 0;; t0 += kTriChunk) {
                const uint32_t t1 = std::min(ntri, t0 + kTriChunk);
                isl.qchunks.push_back({(uint16_t)bf, (uint16_t)bo, t0, t1, t1 == ntri});
                if (t1 == ntri) break;
            }
        }
    }
    const int nq = (int)isl.qchunks.size();
    if ((int)isl.q_nt.size() < nq) {
        isl.q_nt.resize(nq);
        isl.q_nc.resize(nq);
    }
    static const bool dbg = getenv("BL_FASTDBG") != nullptr; // per call: work and time -> stderr
    static std::atomic<long long> c_tris{0}, c_tests{0}, c_cells{0};
    const double t_q0 = dbg ? time_seconds() : 0.0;
    const double grid_ms = (t_q0 - t_g0) * 1000.0;
    auto query = [&](int qc) {
        PROFILE_ACCUM("Fast pair query");
        const Island::QChunk& q = isl.qchunks[qc];
        auto& out_nt = isl.q_nt[qc];
        auto& out_nc = isl.q_nc[qc];
        out_nt.clear();
        out_nc.clear();
        const int bf = q.body, bo = q.other;
        SoftBody& F = *isl.bodies[bf];
        SoftBody& o = *isl.bodies[bo];
        const Island::Grid& G = isl.grids[bf];
        const bool o_fast = isl.extra[bo] > 0;
        const float r0 = F.collision_radius + (o_fast ? fast_extra(bo) + 0.01f : slow_reach); // (+ each node's own travel)
        // (bound for the body's box: the fastest node of either side)
        const float r = F.collision_radius + slow_reach + travel * (F.max_speed + (o_fast ? o.max_speed : 0.0f)) + fast_extra(bo);
        AABB fb = F.aabb;
        fb.expand(r);
        static thread_local std::vector<uint32_t> found;
        // nodes of F in the cells of the box [mn, mx] (each once, ascending) that pass `test`
        auto scan = [&](vec3 mn, vec3 mx, auto&& test, int& n) {
            const int x0 = std::max(0, (int)std::floor((mn.x - G.origin.x) * G.inv_cell)), x1 = std::min(G.dim[0] - 1, (int)((mx.x - G.origin.x) * G.inv_cell));
            const int y0 = std::max(0, (int)std::floor((mn.y - G.origin.y) * G.inv_cell)), y1 = std::min(G.dim[1] - 1, (int)((mx.y - G.origin.y) * G.inv_cell));
            const int z0 = std::max(0, (int)std::floor((mn.z - G.origin.z) * G.inv_cell)), z1 = std::min(G.dim[2] - 1, (int)((mx.z - G.origin.z) * G.inv_cell));
            found.clear();
            n = 0;
            if (x0 > x1 || y0 > y1 || z0 > z1) return true; // (the box misses the grid)
            if (dbg) c_cells.fetch_add((long long)std::max(0, x1 - x0 + 1) * std::max(0, y1 - y0 + 1) * std::max(0, z1 - z0 + 1), std::memory_order_relaxed);
            for (int z = z0; z <= z1; z++)
                for (int y = y0; y <= y1; y++) {
                    const uint32_t row = (uint32_t)((z * G.dim[1] + y) * G.dim[0]);
                    const uint32_t k0 = G.start[row + x0], k1 = G.start[row + x1 + 1]; // (a row of cells is contiguous)
                    found.insert(found.end(), G.items.begin() + k0, G.items.begin() + k1);
                }
            // (a node reaching into several cells is listed once per cell: test each once)
            std::sort(found.begin(), found.end());
            int m = 0;
            const int nf = (int)found.size();
            for (int i = 0; i < nf; i++) {
                if (i > 0 && found[i] == found[i - 1]) continue;
                if (test(found[i])) found[m++] = found[i];
            }
            if (dbg) c_tests.fetch_add(nf, std::memory_order_relaxed);
            n = m;
            return true;
        };
        const float r00 = F.collision_radius + (o_fast ? 0.01f : slow_reach); // (+ the triangle's travel if its body is fast, + the node's)
        for (uint32_t t = q.t0; t < q.t1; t++) {
            const Triangle& tr = o.tris[t];
            if (tr.torn) continue;
            vec3 a = o.nodes[tr.a].p, bb = o.nodes[tr.b].p, c = o.nodes[tr.c].p;
            vec3 tmn = vmin(a, vmin(bb, c)), tmx = vmax(a, vmax(bb, c));
            if (tmx.x < fb.mn.x || tmx.y < fb.mn.y || tmx.z < fb.mn.z || tmn.x > fb.mx.x || tmn.y > fb.mx.y || tmn.z > fb.mx.z) continue;
            if (dbg) c_tris.fetch_add(1, std::memory_order_relaxed);
            const float rt = o_fast ? r00 + travel * std::sqrt(std::max(length2(o.nodes[tr.a].v), std::max(length2(o.nodes[tr.b].v), length2(o.nodes[tr.c].v))))
                                    : r00;
            const float depth = tr.two_sided ? 0.0f : o.hull_depth;
            const vec3 tn = cross(bb - a, c - a);
            const float tn2 = dot(tn, tn);
            const vec3 tu = tn2 > 1e-20f ? tn * (1.0f / std::sqrt(tn2)) : vec3(0);
            auto test = [&](uint32_t ni) {
                const Node& x = F.nodes[ni];
                const vec3 p = x.p;
                const float rn = rt + length(x.v) * travel, rb = rn + depth;
                if (p.x < tmn.x - rb || p.y < tmn.y - rb || p.z < tmn.z - rb || p.x > tmx.x + rb || p.y > tmx.y + rb || p.z > tmx.z + rb) return false;
                return near_triangle(p, a, bb, c, tu, rn, tr.two_sided, depth);
            };
            int n;
            if (!scan(tmn - vec3(rt + depth), tmx + vec3(rt + depth), test, n)) {
                n = 0; // (more than 512 nodes near one triangle: all of them, in order)
                for (uint32_t ni = 0; ni < F.nodes.size(); ni++)
                    if ((F.info[ni].flags & NF_CONTACTER) && test(ni)) out_nt.push_back({(uint16_t)bf, (uint16_t)bo, ni, t});
                continue;
            }
            for (int i = 0; i < n; i++) out_nt.push_back({(uint16_t)bf, (uint16_t)bo, found[i], t});
        }
        if (!q.caps) return;
        for (uint32_t ci = 0; ci < o.capsules.size(); ci++) {
            const Capsule& cp = o.capsules[ci];
            if (cp.joint >= 0 && o.joints[cp.joint].broken) continue;
            vec3 a = o.nodes[cp.a].p, bb = o.nodes[cp.b].p;
            const float rc = cp.radius + r;
            vec3 cmn = vmin(a, bb) - vec3(rc), cmx = vmax(a, bb) + vec3(rc);
            if (cmx.x < F.aabb.mn.x || cmx.y < F.aabb.mn.y || cmx.z < F.aabb.mn.z || cmn.x > F.aabb.mx.x || cmn.y > F.aabb.mx.y ||
                cmn.z > F.aabb.mx.z)
                continue;
            const float rc0 = cp.radius + r0;
            cmn = vmin(a, bb) - vec3(rc0);
            cmx = vmax(a, bb) + vec3(rc0);
            auto test = [&](uint32_t ni) {
                const Node& x = F.nodes[ni];
                const float rn = rc0 + length(x.v) * travel;
                float sseg = closest_on_segment(x.p, a, bb);
                return length2(a + (bb - a) * sseg - x.p) <= rn * rn;
            };
            int n;
            if (!scan(cmn, cmx, test, n)) {
                for (uint32_t ni = 0; ni < F.nodes.size(); ni++)
                    if ((F.info[ni].flags & NF_CONTACTER) && test(ni)) out_nc.push_back({(uint16_t)bf, (uint16_t)bo, ni, ci});
                continue;
            }
            for (int i = 0; i < n; i++) out_nc.push_back({(uint16_t)bf, (uint16_t)bo, found[i], ci});
        }
    };
    isl.team->run(nq, query);
    const double query_ms = dbg ? (time_seconds() - t_q0) * 1000.0 : 0.0;
    size_t before = isl.nt.size() + isl.nc.size();
    for (int q = 0; q < nq; q++) {
        isl.nt.insert(isl.nt.end(), isl.q_nt[q].begin(), isl.q_nt[q].end());
        isl.nc.insert(isl.nc.end(), isl.q_nc[q].begin(), isl.q_nc[q].end());
    }
    if (dbg) {
        int fast = 0;
        size_t tris = 0;
        std::string names;
        for (int bf = 0; bf < nb; bf++)
            if (fast_extra(bf) > 0) {
                fast++;
                names += isl.bodies[bf]->name + format("(%.0f m/s, %zu partners) ", isl.bodies[bf]->max_speed, isl.partners[bf].size());
                for (int bo : isl.partners[bf]) tris += isl.bodies[bo]->tris.size();
            }
        fprintf(stderr, "FAST %.3f ms (grid %.3f query %.3f, %d chunks, team %d) interval %d margin %.3f: %d fast bodies, %zu partner tris (%lld in reach, %lld cells, %lld node tests), %zu pairs: %s\n",
                (time_seconds() - t_g0) * 1000.0, grid_ms, query_ms, nq, (int)isl.team->parallel(), isl.fast_interval, isl.margin, fast, tris, c_tris.exchange(0), c_cells.exchange(0), c_tests.exchange(0),
                isl.nt.size() + isl.nc.size() - before, names.c_str());
    }
}

// The collision detection's second half, at its rate: of the candidate pairs (node, triangle) the ones within reach
// until the next detection - the contact radius (a hull's depth behind it) and the travel of both sides at 1.25 times
// their speeds, 2 cm more; the exact contact of those is found every substep (collide_pairs), the rest wait.
void World::near_pairs(Island& isl) {
    PROFILE_ACCUM("Near pairs");
    const size_t n = isl.nt.size();
    if (n == 0) return;
    const float T = (float)std::max(isl.rebuild_interval, 1) * isl.dt;
    isl.near_keep.assign(n, 1);
    constexpr size_t kChunk = 512;
    const int nch = (int)((n + kChunk - 1) / kChunk);
    auto pass = [&](int c) {
        const size_t i0 = (size_t)c * kChunk, i1 = std::min(n, i0 + kChunk);
        for (size_t i = i0; i < i1; i++) {
            const Island::NT& pr = isl.nt[i];
            const SoftBody& bn = *isl.bodies[pr.bn];
            const SoftBody& bt = *isl.bodies[pr.bt];
            const Node& x = bn.nodes[pr.node];
            const Triangle& t = bt.tris[pr.tri];
            if (t.torn) {
                isl.near_keep[i] = 0;
                continue;
            }
            const Node &a = bt.nodes[t.a], &b = bt.nodes[t.b], &c3 = bt.nodes[t.c];
            vec3 bary;
            const vec3 q = closest_on_triangle(x.p, a.p, b.p, c3.p, bary);
            const float vt = std::sqrt(std::max(length2(a.v), std::max(length2(b.v), length2(c3.v))));
            const float reach = bn.collision_radius + (t.two_sided ? 0.0f : bt.hull_depth) + 1.25f * (length(x.v) + vt) * T + 0.02f;
            isl.near_keep[i] = length2(x.p - q) < reach * reach;
        }
    };
    if (isl.team) isl.team->run(nch, pass);
    else
        for (int c = 0; c < nch; c++) pass(c);
    // (compacted in order: the fast bodies' pairs stay after nt_fast)
    size_t w = 0, fast = isl.nt_fast >= n ? std::string::npos : 0;
    for (size_t i = 0; i < n; i++) {
        if (i == isl.nt_fast) fast = w;
        if (isl.near_keep[i]) isl.nt[w++] = isl.nt[i];
    }
    isl.nt_fast = fast == std::string::npos ? w : fast;
    isl.nt.resize(w);
}

void World::refresh_fast_pairs(Island& isl) {
    PROFILE_ACCUM("Fast pair refresh");
    m_fast_refreshes.fetch_add(1, std::memory_order_relaxed);
    isl.nt.resize(isl.nt_fast);
    isl.nc.resize(isl.nc_fast);
    fast_pairs(isl);
}

// A sheet cracked or refined during the last substep: its new triangles and nodes take over the contact pairs of the
// ones they came from (SoftBody::TopoLog) instead of a new search. Exact as candidate pairs: a half of a triangle lies
// within it, a new node lies on an edge between two nodes (within reach of whatever one of them reaches, the reach
// of a convex shape being convex) or on the node it was split from. The margins stay valid until the next rebuild.
void World::inherit_pairs(Island& isl, int k) {
    SoftBody& X = *isl.bodies[k];
    const SoftBody::TopoLog& L = X.topo_log;
    if (L.tris.empty() && L.nodes.empty() && L.mids.empty() && L.mid_remap.empty()) return;
    PROFILE_ACCUM("Pair inheritance");
    // children per source (sorted), expanded transitively (a child split again in the same substep)
    static thread_local std::vector<std::pair<uint32_t, uint32_t>> tsrc, nsrc;
    tsrc.clear();
    nsrc.clear();
    for (auto& [child, from] : L.tris) tsrc.push_back({from, child});
    for (auto& [child, from] : L.nodes) nsrc.push_back({from, child});
    std::sort(tsrc.begin(), tsrc.end());
    std::sort(nsrc.begin(), nsrc.end());
    auto children = [](const std::vector<std::pair<uint32_t, uint32_t>>& src, uint32_t x, auto&& emit) {
        uint32_t stack[64];
        int n = 0, guard = 0;
        stack[n++] = x;
        while (n > 0 && ++guard < 4096) { // (a cycle in the log would never end)
            const uint32_t y = stack[--n];
            auto it = std::lower_bound(src.begin(), src.end(), std::make_pair(y, 0u));
            for (; it != src.end() && it->first == y; ++it) {
                emit(it->second);
                if (n < 64) stack[n++] = it->second;
            }
        }
    };
    const uint32_t tmin = tsrc.empty() ? UINT32_MAX : tsrc.front().first, tmax = tsrc.empty() ? 0 : tsrc.back().first;
    const uint32_t nmin = nsrc.empty() ? UINT32_MAX : nsrc.front().first, nmax = nsrc.empty() ? 0 : nsrc.back().first;
    // (the sources marked: a pair looks its node and triangle up at once - their range alone had most pairs in it, each
    // a search, a crash's topology change a millisecond)
    static thread_local std::vector<uint8_t> nmark, tmark;
    if (nmark.size() < (size_t)nmax + 1 && !nsrc.empty()) nmark.resize((size_t)nmax + 1, 0);
    if (tmark.size() < (size_t)tmax + 1 && !tsrc.empty()) tmark.resize((size_t)tmax + 1, 0);
    for (auto& x : nsrc) nmark[x.first] = 1;
    for (auto& x : tsrc) tmark[x.first] = 1;
    std::vector<uint8_t>& nm = nmark;
    std::vector<uint8_t>& tm = tmark;
    struct Unmark {
        std::vector<std::pair<uint32_t, uint32_t>>& ns;
        std::vector<std::pair<uint32_t, uint32_t>>& ts;
        std::vector<uint8_t>& nm;
        std::vector<uint8_t>& tm;
        ~Unmark() {
            for (auto& x : ns) nm[x.first] = 0;
            for (auto& x : ts) tm[x.first] = 0;
        }
    } unmark{nsrc, tsrc, nm, tm};
    static thread_local std::vector<Island::NT> nt_new;
    static thread_local std::vector<Island::NC> nc_new;
    static thread_local std::vector<Island::CT> ct_new;
    auto nt_pass = [&](size_t b0, size_t b1) {
        nt_new.clear();
        for (size_t i = b0; i < b1; i++) {
            const Island::NT p = isl.nt[i];
            if (p.bn == k && p.node >= nmin && p.node <= nmax && nm[p.node])
                children(nsrc, p.node, [&](uint32_t c) { nt_new.push_back({p.bn, p.bt, c, p.tri}); });
            if (p.bt == k && p.tri >= tmin && p.tri <= tmax && tm[p.tri])
                children(tsrc, p.tri, [&](uint32_t c) { nt_new.push_back({p.bn, p.bt, p.node, c}); });
        }
        std::sort(nt_new.begin(), nt_new.end(), [](const Island::NT& a, const Island::NT& b) {
            return std::tie(a.bt, a.tri, a.bn, a.node) < std::tie(b.bt, b.tri, b.bn, b.node);
        });
        nt_new.erase(std::unique(nt_new.begin(), nt_new.end(), [](const Island::NT& a, const Island::NT& b) {
            return a.bn == b.bn && a.bt == b.bt && a.node == b.node && a.tri == b.tri;
        }), nt_new.end());
    };
    // slow pairs (before nt_fast) stay in the slow section; the fast bodies' pairs are searched again at every refresh
    nt_pass(0, isl.nt_fast);
    isl.nt.insert(isl.nt.begin() + (ptrdiff_t)isl.nt_fast, nt_new.begin(), nt_new.end());
    isl.nt_fast += nt_new.size();
    nt_pass(isl.nt_fast, isl.nt.size());
    isl.nt.insert(isl.nt.end(), nt_new.begin(), nt_new.end());
    auto nc_pass = [&](size_t b0, size_t b1) {
        nc_new.clear();
        for (size_t i = b0; i < b1; i++) {
            const Island::NC p = isl.nc[i];
            if (p.bn == k && p.node >= nmin && p.node <= nmax && nm[p.node])
                children(nsrc, p.node, [&](uint32_t c) { nc_new.push_back({p.bn, p.bc, c, p.cap}); });
        }
        std::sort(nc_new.begin(), nc_new.end(), [](const Island::NC& a, const Island::NC& b) {
            return std::tie(a.bc, a.cap, a.bn, a.node) < std::tie(b.bc, b.cap, b.bn, b.node);
        });
        nc_new.erase(std::unique(nc_new.begin(), nc_new.end(), [](const Island::NC& a, const Island::NC& b) {
            return a.bn == b.bn && a.bc == b.bc && a.node == b.node && a.cap == b.cap;
        }), nc_new.end());
    };
    nc_pass(0, isl.nc_fast);
    isl.nc.insert(isl.nc.begin() + (ptrdiff_t)isl.nc_fast, nc_new.begin(), nc_new.end());
    isl.nc_fast += nc_new.size();
    nc_pass(isl.nc_fast, isl.nc.size());
    isl.nc.insert(isl.nc.end(), nc_new.begin(), nc_new.end());
    ct_new.clear();
    for (const Island::CT& p : isl.ct)
        if (p.bt == k && p.tri >= tmin && p.tri <= tmax && tm[p.tri]) children(tsrc, p.tri, [&](uint32_t c) { ct_new.push_back({p.bc, p.bt, p.cap, c}); });
    isl.ct.insert(isl.ct.end(), ct_new.begin(), ct_new.end());
    // the plates' mid points (Island::MT): a halved triangle element's new half takes its parent's pairs, a split
    // collision triangle's children theirs
    if (!isl.mt.empty() && (!L.mids.empty() || !tsrc.empty())) {
        static thread_local std::vector<std::pair<uint32_t, uint32_t>> msrc;
        static thread_local std::vector<Island::MT> mt_new;
        msrc.clear(), mt_new.clear();
        for (auto& [child, from] : L.mids) msrc.push_back({from, child});
        std::sort(msrc.begin(), msrc.end());
        for (const Island::MT& p : isl.mt) {
            if (p.bm == k && !msrc.empty()) children(msrc, p.mid, [&](uint32_t c) { mt_new.push_back({p.bm, p.bt, c, p.tri}); });
            if (p.bt == k && p.tri >= tmin && p.tri <= tmax && tm[p.tri]) children(tsrc, p.tri, [&](uint32_t c) { mt_new.push_back({p.bm, p.bt, p.mid, c}); });
        }
        isl.mt.insert(isl.mt.end(), mt_new.begin(), mt_new.end());
    }
    // ... then renumbered as the body compacted them (gone: the pair too)
    if (!L.mid_remap.empty()) {
        size_t w = 0;
        for (size_t i = 0; i < isl.mt.size(); i++) {
            Island::MT p = isl.mt[i];
            if (p.bm == k) {
                const int32_t m = p.mid < L.mid_remap.size() ? L.mid_remap[p.mid] : -1;
                if (m < 0) continue;
                p.mid = (uint32_t)m;
            }
            isl.mt[w++] = p;
        }
        isl.mt.resize(w);
    }
    isl.pair_count = (int)(isl.nt.size() + isl.nc.size() + isl.ct.size() + isl.mt.size());
}

// Contacts of the candidate pairs: the narrow phase (closest points of the current positions: independent per pair,
// in chunks shared by the team), then the response in pair order (serial: each contact cancels the forces of the
// ones before it, see contact_force).
// The narrow phase's chunks of the candidate pairs (node-triangle, node-capsule, capsule-triangle): the geometry alone
// (pair_detect, any order, in parallel: beside the forces when the pairs were not made again this substep), then the
// response in order (collide_pairs)
int World::pair_chunks(Island& isl) {
    const size_t nnt = isl.nt.size(), nnc = isl.nc.size(), nct = isl.ct.size();
    constexpr uint32_t kPairChunk = kPairChunkSize;
    const int cnt = (int)((nnt + kPairChunk - 1) / kPairChunk), cnc = (int)((nnc + kPairChunk - 1) / kPairChunk),
              cct = (int)((nct + kPairChunk - 1) / kPairChunk);
    const int nch = cnt + cnc + cct;
    if ((int)isl.hits.size() < nch) isl.hits.resize(nch);
    return nch;
}

void World::pair_detect(Island& isl, int c) {
    const size_t nnt = isl.nt.size(), nnc = isl.nc.size(), nct = isl.ct.size();
    constexpr uint32_t kPairChunk = kPairChunkSize;
    const int cnt = (int)((nnt + kPairChunk - 1) / kPairChunk), cnc = (int)((nnc + kPairChunk - 1) / kPairChunk),
              cct = (int)((nct + kPairChunk - 1) / kPairChunk);
    (void)cct;
    {
        {
            PROFILE_ACCUM("Narrow phase");
            auto& out = isl.hits[c];
            out.clear();
            if (c < cnt) {
                const size_t i0 = (size_t)c * kPairChunk, i1 = std::min(nnt, i0 + kPairChunk);
                for (size_t i = i0; i < i1; i++) {
                    const Island::NT& pr = isl.nt[i];
                    const SoftBody& bn = *isl.bodies[pr.bn];
                    const SoftBody& bt = *isl.bodies[pr.bt];
                    if (bn.sleeping && bt.sleeping) continue;
                    const Node& n = bn.nodes[pr.node];
                    const Triangle& t = bt.tris[pr.tri];
                    if (t.torn) continue; // (torn since the pairs were found: a cut, a stretch)
                    static const bool no_hull = getenv("BL_HULL") && atoi(getenv("BL_HULL")) == 0; // (diagnostics: hulls measured, not colliding)
                    if (no_hull && !t.two_sided) continue;
                    const Node& a = bt.nodes[t.a];
                    const Node& b = bt.nodes[t.b];
                    const Node& cc = bt.nodes[t.c];
                    vec3 bary;
                    const float r = bt.faces_only && !t.two_sided ? bt.face_skin : bn.collision_radius;
                    if (!t.two_sided) {
                        // a hull triangle: over its face the node is pushed out along the face's normal, from as deep
                        // as the hull's depth behind it; off the face, outside, the ordinary contact with its edges
                        const vec3 fn = normalize(cross(b.p - a.p, cc.p - a.p));
                        const float s = dot(n.p - a.p, fn);
                        if (s >= r || s < -bt.hull_depth) continue;
                        const vec3 q = n.p - fn * s;
                        const vec3 cq = closest_on_triangle(q, a.p, b.p, cc.p, bary);
                        if (length2(cq - q) <= 1e-6f) {
                            out.push_back({(uint32_t)i, 0.0f, bary, fn, r - s});
                            continue;
                        }
                        if (s < 0 || bt.faces_only) continue; // (behind, not over this face: another face's; a blade's off its
                        // faces: nothing - its edge's contact swept the nodes its cut left beside it along ahead of it)
                    } else {
                        // (off the triangle's box by more than its reach: most of the candidates)
                        const vec3 lo = vmin(a.p, vmin(b.p, cc.p)) - vec3(r), hi = vmax(a.p, vmax(b.p, cc.p)) + vec3(r);
                        if (n.p.x < lo.x || n.p.y < lo.y || n.p.z < lo.z || n.p.x > hi.x || n.p.y > hi.y || n.p.z > hi.z) continue;
                    }
                    vec3 cp = closest_on_triangle(n.p, a.p, b.p, cc.p, bary);
                    vec3 d = n.p - cp;
                    float dist2 = dot(d, d);
                    if (dist2 >= r * r) continue;
                    float dist = std::sqrt(dist2);
                    vec3 nrm;
                    if (dist > 1e-5f) nrm = d / dist;
                    else {
                        nrm = normalize(cross(b.p - a.p, cc.p - a.p));
                        if (dot(n.v - (a.v * bary.x + b.v * bary.y + cc.v * bary.z), nrm) > 0) nrm = -nrm;
                    }
                    out.push_back({(uint32_t)i, 0.0f, bary, nrm, r - dist});
                }
            } else if (c < cnt + cnc) {
                const size_t i0 = (size_t)(c - cnt) * kPairChunk, i1 = std::min(nnc, i0 + kPairChunk);
                for (size_t i = i0; i < i1; i++) {
                    const Island::NC& pr = isl.nc[i];
                    const SoftBody& bn = *isl.bodies[pr.bn];
                    const SoftBody& bc = *isl.bodies[pr.bc];
                    if (bn.sleeping && bc.sleeping) continue;
                    const Capsule& cap = bc.capsules[pr.cap];
                    if (cap.joint >= 0 && bc.joints[cap.joint].broken) continue;
                    const Node& n = bn.nodes[pr.node];
                    const Node& a = bc.nodes[cap.a];
                    const Node& b = bc.nodes[cap.b];
                    float t = closest_on_segment(n.p, a.p, b.p);
                    vec3 cp = a.p + (b.p - a.p) * t;
                    vec3 d = n.p - cp;
                    float r = bn.collision_radius + cap.radius;
                    float dist2 = dot(d, d);
                    if (dist2 >= r * r) continue;
                    float dist = std::sqrt(dist2);
                    vec3 nrm = dist > 1e-5f ? d / dist : vec3(0, 1, 0);
                    out.push_back({(uint32_t)i, t, vec3(0), nrm, r - dist});
                }
            } else {
                const size_t i0 = (size_t)(c - cnt - cnc) * kPairChunk, i1 = std::min(nct, i0 + kPairChunk);
                for (size_t i = i0; i < i1; i++) {
                    const Island::CT& pr = isl.ct[i];
                    const SoftBody& bc = *isl.bodies[pr.bc];
                    const SoftBody& bt = *isl.bodies[pr.bt];
                    if (bc.sleeping && bt.sleeping) continue;
                    const Capsule& cap = bc.capsules[pr.cap];
                    if (cap.joint >= 0 && bc.joints[cap.joint].broken) continue;
                    const Node& p0 = bc.nodes[cap.a];
                    const Node& p1 = bc.nodes[cap.b];
                    const Triangle& tri = bt.tris[pr.tri];
                    const Node& a = bt.nodes[tri.a];
                    const Node& b = bt.nodes[tri.b];
                    const Node& cc = bt.nodes[tri.c];
                    float best = 1e30f, best_s = 0;
                    vec3 best_bary, best_seg, best_tri;
                    auto consider = [&](float s, vec3 sp, vec3 tp, vec3 bary) {
                        float dd = length2(sp - tp);
                        if (dd < best) { best = dd; best_s = s; best_seg = sp; best_tri = tp; best_bary = bary; }
                    };
                    vec3 bary;
                    vec3 q = closest_on_triangle(p0.p, a.p, b.p, cc.p, bary);
                    consider(0, p0.p, q, bary);
                    q = closest_on_triangle(p1.p, a.p, b.p, cc.p, bary);
                    consider(1, p1.p, q, bary);
                    const vec3 tv[3] = {a.p, b.p, cc.p};
                    for (int e = 0; e < 3; e++) {
                        float s, u;
                        closest_seg_seg(p0.p, p1.p, tv[e], tv[(e + 1) % 3], s, u);
                        vec3 sp = p0.p + (p1.p - p0.p) * s;
                        vec3 tp = tv[e] + (tv[(e + 1) % 3] - tv[e]) * u;
                        vec3 bb(0);
                        bb[e] = 1 - u;
                        bb[(e + 1) % 3] = u;
                        consider(s, sp, tp, bb);
                    }
                    vec3 tn = cross(b.p - a.p, cc.p - a.p);
                    float d0 = dot(p0.p - a.p, tn), d1 = dot(p1.p - a.p, tn);
                    bool crossing = false;
                    if ((d0 < 0) != (d1 < 0) && std::fabs(d0 - d1) > 1e-12f) {
                        float s = d0 / (d0 - d1);
                        vec3 ip = p0.p + (p1.p - p0.p) * s;
                        q = closest_on_triangle(ip, a.p, b.p, cc.p, bary);
                        if (length2(q - ip) < 1e-8f) {
                            crossing = true;
                            best = 0;
                            best_s = s;
                            best_seg = ip;
                            best_tri = q;
                            best_bary = bary;
                        }
                    }
                    float r = cap.radius + bc.collision_radius;
                    if (best >= r * r) continue;
                    float dist = std::sqrt(best);
                    vec3 nrm;
                    float pen;
                    if (!crossing && dist > 1e-5f) {
                        nrm = (best_seg - best_tri) / dist;
                        pen = r - dist;
                    } else {
                        nrm = normalize(tn);
                        vec3 mid = (p0.p + p1.p) * 0.5f;
                        if (dot(mid - a.p, nrm) < 0) nrm = -nrm;
                        float dm = std::min(std::fabs(d0), std::fabs(d1)) / std::max(1e-6f, length(tn));
                        pen = r + std::min(dm, 0.2f);
                    }
                    out.push_back({(uint32_t)i, best_s, best_bary, nrm, pen});
                }
            }
        }
    }
}

void World::collide_pairs(Island& isl, bool detected) {
    const size_t nnt = isl.nt.size(), nnc = isl.nc.size(), nct = isl.ct.size();
    constexpr uint32_t kPairChunk = kPairChunkSize;
    const int cnt = (int)((nnt + kPairChunk - 1) / kPairChunk), cnc = (int)((nnc + kPairChunk - 1) / kPairChunk),
              cct = (int)((nct + kPairChunk - 1) / kPairChunk);
    const int nch = cnt + cnc + cct;
    m_narrow.fetch_add((long long)(nnt + nnc + nct), std::memory_order_relaxed);
    if (nch == 0) return;
    if (!detected) {
        pair_chunks(isl);
        isl.team->run(nch, [&](int c) { pair_detect(isl, c); });
    }
    PROFILE_ACCUM("Contacts");
    int contacts = 0;
    const float dt = isl.dt;
    const GroundModel& gm = ground_models()[SURF_CONCRETE];
    // sleeping bodies act as immovable obstacles and get woken up for the next substep -- but only by a partner
    // that actually moves: two resting bodies in contact (a stack of bales) never fall asleep in the same frame,
    // so waking on any contact would keep them awake forever
    const float wake_speed = settings.sleep_speed;
    auto movable = [wake_speed](SoftBody& b, const SoftBody& other) {
        if (b.sleeping) {
            // (the partner's own rest threshold: a soft bale keeps rocking a little below its sleep speed, a stack of
            // them would otherwise wake each other in turn forever and creep apart)
            // (a sheet with rest damping trembles at its skin while still as a whole: three times it, as the sphere
            // contacts; a drum on another woke the one below as it fell asleep, and it the one above, for good)
            if (other.max_speed > std::max(wake_speed, other.sleep_speed) * (other.rest_damp > 0 ? 3.0f : 1.0f)) b.wake_request = true;
            return false;
        }
        return true;
    };
    // snapshot of a sub-cycled body's internal force (see contact_force), nullptr for everything else
    auto snap = [](const SoftBody& b, uint32_t i) -> const vec3* {
        return b.dt_shift() > 0 && i < b.ext_force.size() ? &b.ext_force[i] : nullptr;
    };
    for (int c = 0; c < nch; c++) {
        for (const Island::Hit& h : isl.hits[c]) {
            if (c < cnt) {
                const Island::NT& pr = isl.nt[h.pair];
                SoftBody& bn = *isl.bodies[pr.bn];
                SoftBody& bt = *isl.bodies[pr.bt];
                const Triangle& t = bt.tris[pr.tri];
                bool ma = movable(bn, bt), mb = movable(bt, bn);
                Node* A[1] = {&bn.nodes[pr.node]};
                vec3* FA[1] = {&bn.force[pr.node]};
                float wa[1] = {1.0f};
                Node* B[3] = {&bt.nodes[t.a], &bt.nodes[t.b], &bt.nodes[t.c]};
                vec3* FB[3] = {&bt.force[t.a], &bt.force[t.b], &bt.force[t.c]};
                float wb[3] = {h.bary.x, h.bary.y, h.bary.z};
                float fric = (bn.info[pr.node].flags & NF_TYRE) ? bn.info[pr.node].friction : 0.8f * std::min(bn.contact_friction, bt.contact_friction);
                const vec3* SA[1] = {snap(bn, pr.node)};
                const vec3* SB[3] = {snap(bt, t.a), snap(bt, t.b), snap(bt, t.c)};
                if (!t.two_sided && h.pen > bn.collision_radius) { // (behind a hull's face)
                    if (hull_push_out<3>(A[0], FA[0], B, FB, wb, ma, mb, h.nrm, h.pen, dt, SA[0], SB[0] ? SB : nullptr)) contacts++, bn.touch(bt), bt.touch(bn);
                    continue;
                }
                if (contact_force<1, 3>(A, FA, wa, B, FB, wb, ma, mb, h.nrm, h.pen, dt, gm, fric, SA[0] ? SA : nullptr, SB[0] ? SB : nullptr)) {
                    contacts++, bn.touch(bt), bt.touch(bn);
                    // a fast contact on a sheet with a fracture pattern, or on a frame's plates: where it lays one (the
                    // approach speed; the body's size - a car's, its node's plates take a plug of a fifth of it)
                    if (bt.shell_mat.pattern != ShellPattern::None || bn.shell_mat.pattern != ShellPattern::None || bt.fem.patterned() || bn.fem.patterned()) {
                        const float vn = std::fabs(dot(A[0]->v - (B[0]->v * wb[0] + B[1]->v * wb[1] + B[2]->v * wb[2]), h.nrm));
                        auto radius = [](const SoftBody& x) { return 0.5f * maxc(x.aabb.mx - x.aabb.mn); };
                        bt.pattern_contact(t.a, t.b, t.c, h.bary, vn, radius(bn), m_time);
                        bn.pattern_contact(pr.node, pr.node, pr.node, vec3(1, 0, 0), vn, radius(bt), m_time);
                        if (bt.fem.patterned()) bt.fem.note_hit(t.a, t.b, t.c, h.bary, blow_speed(bn, &bt, h.nrm, vn, m_time), plug(bn, bt), m_time);
                        if (bn.fem.patterned()) bn.fem.note_hit(pr.node, pr.node, pr.node, vec3(1, 0, 0), blow_speed(bt, &bn, h.nrm, vn, m_time), plug(bt, bn), m_time);
                    }
                }
            } else if (c < cnt + cnc) {
                const Island::NC& pr = isl.nc[h.pair];
                SoftBody& bn = *isl.bodies[pr.bn];
                SoftBody& bc = *isl.bodies[pr.bc];
                const Capsule& cap = bc.capsules[pr.cap];
                bool ma = movable(bn, bc), mb = movable(bc, bn);
                Node* A[1] = {&bn.nodes[pr.node]};
                vec3* FA[1] = {&bn.force[pr.node]};
                float wa[1] = {1.0f};
                Node* B[2] = {&bc.nodes[cap.a], &bc.nodes[cap.b]};
                vec3* FB[2] = {&bc.force[cap.a], &bc.force[cap.b]};
                float wb[2] = {1 - h.s, h.s};
                const vec3* SA[1] = {snap(bn, pr.node)};
                const vec3* SB[2] = {snap(bc, cap.a), snap(bc, cap.b)};
                if (contact_force<1, 2>(A, FA, wa, B, FB, wb, ma, mb, h.nrm, h.pen, dt, gm, 0.8f * std::min(bn.contact_friction, bc.contact_friction), SA[0] ? SA : nullptr,
                                        SB[0] ? SB : nullptr))
                    contacts++, bn.touch(bc), bc.touch(bn);
            } else {
                const Island::CT& pr = isl.ct[h.pair];
                SoftBody& bc = *isl.bodies[pr.bc];
                SoftBody& bt = *isl.bodies[pr.bt];
                const Capsule& cap = bc.capsules[pr.cap];
                const Triangle& tri = bt.tris[pr.tri];
                bool ma = movable(bc, bt), mb = movable(bt, bc);
                Node* A[2] = {&bc.nodes[cap.a], &bc.nodes[cap.b]};
                vec3* FA[2] = {&bc.force[cap.a], &bc.force[cap.b]};
                float wa[2] = {1 - h.s, h.s};
                Node* B[3] = {&bt.nodes[tri.a], &bt.nodes[tri.b], &bt.nodes[tri.c]};
                vec3* FB[3] = {&bt.force[tri.a], &bt.force[tri.b], &bt.force[tri.c]};
                float wb[3] = {h.bary.x, h.bary.y, h.bary.z};
                const vec3* SA[2] = {snap(bc, cap.a), snap(bc, cap.b)};
                const vec3* SB[3] = {snap(bt, tri.a), snap(bt, tri.b), snap(bt, tri.c)};
                if (contact_force<2, 3>(A, FA, wa, B, FB, wb, ma, mb, h.nrm, h.pen, dt, gm, 0.8f * std::min(bc.contact_friction, bt.contact_friction), SA[0] ? SA : nullptr,
                                        SB[0] ? SB : nullptr)) {
                    contacts++, bc.touch(bt), bt.touch(bc);
                    if (bt.shell_mat.pattern != ShellPattern::None || bt.fem.patterned()) {
                        const vec3 va = A[0]->v * wa[0] + A[1]->v * wa[1];
                        const float vn = std::fabs(dot(va - (B[0]->v * wb[0] + B[1]->v * wb[1] + B[2]->v * wb[2]), h.nrm));
                        bt.pattern_contact(tri.a, tri.b, tri.c, h.bary, vn, 0.5f * maxc(bc.aabb.mx - bc.aabb.mn), m_time);
                        if (bt.fem.patterned()) bt.fem.note_hit(tri.a, tri.b, tri.c, h.bary, blow_speed(bc, &bt, h.nrm, vn, m_time), plug(bc, bt), m_time);
                    }
                }
            }
        }
    }
    isl.contacts += contacts;
}

// The FEM plates' mid points (SoftBody::tri_mids) against the other bodies' triangles (rebuild_pairs 4): a node's contact
// (contact_force), the mid point's side its triangle's three corners a third each; over a hull triangle's face only in
// front of it. What the corners' own contacts with that triangle leave of its depth at the middle counts.
int World::mid_chunks(Island& isl) {
    const int nch = (int)((isl.mt.size() + kPairChunkSize - 1) / kPairChunkSize);
    if ((int)isl.mhits.size() < nch) isl.mhits.resize(nch);
    return nch;
}

void World::mid_detect(Island& isl, int ch) {
    const float dt = isl.dt;
    (void)dt;
    constexpr float kW = 1.0f / 3.0f;
    constexpr size_t kChunk = kPairChunkSize;
    {
        PROFILE_ACCUM("Narrow phase");
        auto& out = isl.mhits[ch];
        out.clear();
        const size_t i1 = std::min(isl.mt.size(), (size_t)(ch + 1) * kChunk);
        for (size_t i = (size_t)ch * kChunk; i < i1; i++) {
            const Island::MT& pr = isl.mt[i];
            const SoftBody& bm = *isl.bodies[pr.bm];
            const SoftBody& bt = *isl.bodies[pr.bt];
            if ((bm.sleeping && bt.sleeping) || pr.tri >= bt.tris.size() || pr.mid >= bm.fem.tris.size()) continue;
            const Triangle& t = bt.tris[pr.tri];
            if (t.torn) continue;
            uint32_t n[3];
            if (!tri_mid(bm, pr.mid, n, NF_CONTACTER)) continue;
            const vec3 p = tri_mid_p(bm, n);
            const Node &a = bt.nodes[t.a], &b = bt.nodes[t.b], &c = bt.nodes[t.c];
            const bool faces = bt.faces_only && !t.two_sided; // (a blade's face: over it alone, within its skin - see faces_only)
            const float r = faces ? bt.face_skin : bm.collision_radius;
            // (off the triangle's box by more than its reach)
            const vec3 lo = vmin(a.p, vmin(b.p, c.p)) - vec3(r), hi = vmax(a.p, vmax(b.p, c.p)) + vec3(r);
            if (p.x < lo.x || p.y < lo.y || p.z < lo.z || p.x > hi.x || p.y > hi.y || p.z > hi.z) continue;
            vec3 bary;
            const vec3 cp = closest_on_triangle(p, a.p, b.p, c.p, bary);
            const vec3 d = p - cp;
            const float dist2 = dot(d, d);
            if (dist2 >= r * r) continue;
            const vec3 fn = normalize_or(cross(b.p - a.p, c.p - a.p), vec3(0, 1, 0));
            if (!t.two_sided && dot(p - a.p, fn) < 0) continue; // (behind a hull's face: its nodes' business)
            // (a blade's edge: not pressed into the plates beside its cut - pushed along their normals, the sheet's halves
            // were dragged down with it and flew apart)
            if (faces && std::min(bary.x, std::min(bary.y, bary.z)) < 1e-4f) continue;
            const float dist = std::sqrt(dist2);
            vec3 nrm = dist > 1e-5f ? d / dist : fn;
            // (on the triangle's edge or corner: an edge pressed into the plate's face, along the plate's normal - the
            // point's own off the edge tipped over and pushed a plate lying on two blades off them sideways)
            if (std::min(bary.x, std::min(bary.y, bary.z)) < 1e-4f) {
                const vec3 pn = normalize_or(cross(bm.nodes[n[1]].p - bm.nodes[n[0]].p, bm.nodes[n[2]].p - bm.nodes[n[0]].p), nrm);
                nrm = dist > 1e-5f ? (dot(pn, d) >= 0 ? pn : -pn) : pn;
            }
            if (dist <= 1e-5f) {
                vec3 vm(0);
                for (int j = 0; j < 3; j++) vm += bm.nodes[n[j]].v * kW;
                if (dot(vm - (a.v * bary.x + b.v * bary.y + c.v * bary.z), nrm) > 0) nrm = -nrm;
            }
            // (what its corners reach of the triangle themselves)
            float held = 0;
            for (int j = 0; j < 3; j++) {
                vec3 bj;
                const float dj = length(bm.nodes[n[j]].p - closest_on_triangle(bm.nodes[n[j]].p, a.p, b.p, c.p, bj));
                held += std::max(0.0f, r - dj) * kW;
            }
            const float pen = r - dist - held;
            if (pen > 0) out.push_back({(uint32_t)i, 0.0f, bary, nrm, pen});
        }
    }
}

void World::collide_mid_pairs(Island& isl, bool detected) {
    if (isl.mt.empty()) return;
    const float dt = isl.dt;
    constexpr float kW = 1.0f / 3.0f;
    // the narrow phase in chunks shared by the island's team (most pairs are off their triangle's reach: their margins
    // take the travel until the next rebuild; beside the forces when the pairs were not made again: mid_detect), the
    // response in order
    const int nch = mid_chunks(isl);
    if (!detected) isl.team->run(nch, [&](int ch) { mid_detect(isl, ch); });
    PROFILE_ACCUM("Contacts");
    int contacts = 0;
    const GroundModel& gm = ground_models()[SURF_CONCRETE];
    const float wake_speed = settings.sleep_speed;
    auto movable = [wake_speed](SoftBody& b, const SoftBody& other) { // (as collide_pairs')
        if (b.sleeping) {
            if (other.max_speed > std::max(wake_speed, other.sleep_speed) * (other.rest_damp > 0 ? 3.0f : 1.0f)) b.wake_request = true;
            return false;
        }
        return true;
    };
    auto snap = [](const SoftBody& b, uint32_t i) -> const vec3* { return b.dt_shift() > 0 && i < b.ext_force.size() ? &b.ext_force[i] : nullptr; };
    for (int ch = 0; ch < nch; ch++)
        for (const Island::Hit& h : isl.mhits[ch]) {
            const Island::MT& pr = isl.mt[h.pair];
            SoftBody& bm = *isl.bodies[pr.bm];
            SoftBody& bt = *isl.bodies[pr.bt];
            const Triangle& t = bt.tris[pr.tri];
            uint32_t n[3];
            if (!tri_mid(bm, pr.mid, n, NF_CONTACTER)) continue;
            const bool ma = movable(bm, bt), mb = movable(bt, bm);
            Node* A[3] = {&bm.nodes[n[0]], &bm.nodes[n[1]], &bm.nodes[n[2]]};
            vec3* FA[3] = {&bm.force[n[0]], &bm.force[n[1]], &bm.force[n[2]]};
            const float wa[3] = {kW, kW, kW};
            Node* B[3] = {&bt.nodes[t.a], &bt.nodes[t.b], &bt.nodes[t.c]};
            vec3* FB[3] = {&bt.force[t.a], &bt.force[t.b], &bt.force[t.c]};
            const float wb[3] = {h.bary.x, h.bary.y, h.bary.z};
            const vec3* SA[3] = {snap(bm, n[0]), snap(bm, n[1]), snap(bm, n[2])};
            const vec3* SB[3] = {snap(bt, t.a), snap(bt, t.b), snap(bt, t.c)};
            if (contact_force<3, 3>(A, FA, wa, B, FB, wb, ma, mb, h.nrm, h.pen, dt, gm, 0.8f * std::min(bm.contact_friction, bt.contact_friction), SA[0] ? SA : nullptr,
                                    SB[0] ? SB : nullptr)) {
                contacts++, bm.touch(bt), bt.touch(bm);
                bm.mid_contacts++;
                if (bm.mid_touch.size() != bm.fem.tris.size()) bm.mid_touch.assign(bm.fem.tris.size(), 0);
                bm.mid_touch[pr.mid] |= 2;
                if (bm.fem.patterned()) { // (a fast blow on the plate: its fracture pattern)
                    const vec3 va = (A[0]->v + A[1]->v + A[2]->v) * kW, vb = B[0]->v * wb[0] + B[1]->v * wb[1] + B[2]->v * wb[2];
                    bm.fem.note_hit(n[0], n[1], n[2], vec3(kW), blow_speed(bt, &bm, h.nrm, std::fabs(dot(va - vb, h.nrm)), m_time), plug(bt, bm), m_time);
                }
            }
        }
    isl.contacts += contacts;
}

// One island for a whole frame. Every short step is a sequence of phases, each split into work items that any thread
// of the island's team may take (Team): internal forces (per body, and the triangles of a sheet in chunks), the
// sheets' forces gathered per node, the contacts (narrow phase in chunks, response in order), static contacts and
// integration (nodes in ranges); the topology changes and the merging of results are serial, in a fixed order.
// Bodies that take several short steps per substep (refined sheets, see SoftBody::dt_shift) continue with their own
// steps after the contacts, holding the contact forces of the substep.
void World::simulate_island(Island& isl, int substeps) {
    PROFILE_ZONE("Island");
    const uint64_t isl_t0 = prof::now();
    const float dt = settings.dt;
    const vec3 g = settings.gravity;
    // max speed is needed by the broadphase margin before the first substep
    for (SoftBody* b : isl.bodies) {
        float m2 = 0;
        if (!b->sleeping)
            for (auto& n : b->nodes) m2 = std::max(m2, length2(n.v));
        b->max_speed = std::sqrt(m2);
    }
    isl.contacts = 0;
    const int nb = (int)isl.bodies.size();
    Team team(isl.teamed);
    Team aside(isl.teamed); // (a phase posted beside this thread's serial parts: the frames' blocks beside the contacts)
    isl.team = &team;
    for (SoftBody* b : isl.bodies) {
        b->static_contacts = 0;
        b->sphere_touches = 0;
        b->body_contacts = 0;
        b->mid_contacts = 0;
        if (!b->fem.tris.empty()) b->mid_touch.assign(b->fem.tris.size(), 0);
        b->touched.clear();
        if (b->energy_guard && !b->rigid) b->motion_energy(settings.gravity, b->guard_ke, b->guard_pe);
        b->topo_log.clear();
    }
    isl.dt = dt;
    isl.subs.assign(nb, 0);
    constexpr uint32_t kNodeChunk = 1024;
    // a frame's implicit step this substep (every frame_every substeps; a sub-cycled sheet's own frame every short step)
    const int fe = std::max(1, settings.frame_every);
    auto fem_due = [](const SoftBody& b) { return b.fem_left <= 0; };
    // (the frame's step begun: its length - the numerical dissipation per step over the steps it spans, the same per
    // second)
    auto fem_begin = [&](int k) {
        SoftBody& b = *isl.bodies[k];
        const float bdt = dt / (float)isl.subs[k];
        const float hs = b.fem_every_step ? 1.0f : (float)b.fem_period;
        return b.fem.solve_begin(b, b.fem_every_step ? bdt : dt * (float)b.fem_period, bdt, settings.frame_theta,
                                 (b.fem_dissipation >= 0 ? b.fem_dissipation : settings.frame_dissipation) / hs);
    };
    // internal forces of one body other than its triangles (a sub-cycled body recomputes them for every short step)
    auto base_forces = [&](SoftBody& b, float bdt, bool controller, bool fem_parts) {
        PROFILE_ACCUM("Beam forces"); // (gravity, drag, wind, controllers, beams, shocks, joints, wheels)
        b.clear_forces(g);
        if (!b.wind_area.empty()) apply_wind(b);
        // (the controllers once a substep, for the whole of it: given the short step's length, a refined sheet's car
        // ran its engine, gearbox and steering 2-4 times slower than the clock)
        if (controller && b.pre_substep) b.pre_substep(b, isl.dt);
        b.compute_beam_forces();
        if (!b.shocks.empty()) b.compute_shock_forces();
        if (!b.joints.empty()) b.compute_joint_forces();
        const bool fem_step = !b.fem.empty() && (b.fem_every_step || (controller && fem_due(b))) && !b.rigid;
        if (!b.welds.empty()) b.compute_weld_forces(!b.fem.empty() && !b.rigid); // (their moment on the frame nodes: its steps and the held ones)
        if (fem_step && !fem_parts) { // (once per substep: the frame's step, FemFrame::solve; fem_parts: in chunks of their own)
            PROFILE_ACCUM("Frame elements");
            b.fem.compute_forces(b);
        }
        if (!b.wheels.empty()) b.compute_wheel_forces(bdt, controller);
        if (!b.slides.empty()) b.compute_slide_forces();
    };
    // node ranges of the bodies stepping at short step j (a body with wheels stays whole: its tyre patches share grip)
    // (a frame's body without tyre nodes - a tyre patch shares its grip, collide_static - in chunks of nodes for its
    // static contacts: the ring tyres and the plates' mid points after, a task each; FemFrame's step for the body)
    isl.fem_chunked.assign(nb, 0);
    for (int k = 0; k < nb; k++) {
        const SoftBody& b = *isl.bodies[k];
        if (b.fem.empty() || b.rigid || b.nodes.size() <= kFemChunk) continue;
        bool tyre = false;
        for (const NodeInfo& in : b.info) tyre |= (in.flags & NF_TYRE) != 0;
        isl.fem_chunked[k] = !tyre;
    }
    auto node_work = [&](int j, bool only_sheets) {
        isl.work.clear();
        for (int k = 0; k < nb; k++) {
            if (isl.subs[k] <= j) continue;
            SoftBody& b = *isl.bodies[k];
            if (only_sheets && (b.shells.empty() || b.rigid) && isl.subs[k] == 1) continue;
            const uint32_t n = (uint32_t)b.nodes.size();
            if (b.rigid) { // (one step for the whole body: its forces add up to a force and a torque)
                isl.work.push_back({(uint32_t)k, 3, 0, n});
                continue;
            }
            if (isl.fem_chunked[k]) {
                for (uint32_t a = 0; a < n; a += kFemChunk) isl.work.push_back({(uint32_t)k, 2, a, std::min(n, a + kFemChunk)});
                continue;
            }
            if (!b.wheels.empty() || !b.fem.empty() || n <= kNodeChunk) { // (the frame solves all its nodes at once)
                isl.work.push_back({(uint32_t)k, 2, 0, n});
                continue;
            }
            for (uint32_t a = 0; a < n; a += kNodeChunk) isl.work.push_back({(uint32_t)k, 2, a, std::min(n, a + kNodeChunk)});
        }
        isl.parts.resize(isl.work.size());
    };
    bool rebuild_now = false;
    for (double& x : isl.ph_ms) x = 0;
    uint64_t tph = prof::now();
    auto lap = [&](int k) {
        const uint64_t t = prof::now();
        isl.ph_ms[k] += prof::ticks_to_ms(t - tph);
        tph = t;
    };
    isl.shell_steps = isl.beam_steps = isl.node_steps = isl.hinge_evals = 0;
    static prof::Zone* first_zone = prof::zone("Short step first");
    static prof::Zone* short_zone = prof::zone("Short steps later");
    for (int s = 0; s < substeps; s++) {
        if (trace_arm().frame == g_trace_frame) {
            prof::g_trace = s >= trace_arm().s0 && s < trace_arm().s0 + trace_arm().count;
            if (prof::g_trace) prof::trace_mark("substep", s);
        }
        int maxsub = 0;
        for (int k = 0; k < nb; k++) {
            SoftBody& b = *isl.bodies[k];
            if (b.wake_request) {
                b.wake_request = false;
                b.wake();
            }
            isl.subs[k] = b.sleeping ? 0 : 1 << b.dt_shift();
            if (!b.fem.empty() && b.fem_left <= 0) b.fem_period = fe; // (its frame's step now: over this many substeps)
            maxsub = std::max(maxsub, isl.subs[k]);
            isl.node_steps += (long long)b.nodes.size() * isl.subs[k];
            isl.beam_steps += (long long)b.beams.size() * isl.subs[k];
        }
        for (int j = 0; j < maxsub; j++) {
            prof::AccumScope short_scope(j > 0 ? short_zone : first_zone); // (diagnostics: the short steps' cost, the first and the later)
            if (prof::g_trace) prof::trace_mark("short step", j);
            // 1) internal forces (a frame's members in chunks beside the body's other forces and its sheet: one after
            // another they were the longest item of the step)
            isl.work.clear();
            isl.fem_parts.assign(nb, 0);
            for (int k = 0; k < nb; k++) {
                if (isl.subs[k] <= j) continue;
                SoftBody& b = *isl.bodies[k];
                isl.work.push_back({(uint32_t)k, 0, 0, 0});
                if (!b.fem.empty() && (b.fem_every_step || (j == 0 && fem_due(b))) && !b.rigid) {
                    const int fc = b.fem.begin_forces(b);
                    for (int c = 0; c < fc; c++) isl.work.push_back({(uint32_t)k, 4, (uint32_t)c, 0});
                    isl.fem_parts[k] = 1;
                }
                if (!b.shells.empty() && !b.rigid) {
                    const int ch = b.shell_begin(dt / (float)isl.subs[k], j, isl.subs[k]);
                    for (int c = 0; c < ch; c++) isl.work.push_back({(uint32_t)k, 1, (uint32_t)c, 0});
                }
            }
            // (the narrow phase's geometry beside the forces when the pairs are not made again this substep: it needs
            // the positions alone)
            bool detected = false;
            if (j == 0 && isl.needs_pairs && !(s == 0 || s >= isl.next_rebuild || rebuild_now) &&
                !(isl.fast_interval > 0 && isl.fast_interval < isl.rebuild_interval && s >= isl.next_fast)) {
                detected = true;
                for (int c = 0, n = pair_chunks(isl); c < n; c++) isl.work.push_back({0u, 5, (uint32_t)c, 0});
                for (int c = 0, n = isl.mt.empty() ? 0 : mid_chunks(isl); c < n; c++) isl.work.push_back({0u, 6, (uint32_t)c, 0});
            }
            // (and the collision volumes' look: their places now, the look beside the forces, their forces below - but
            // not when the pairs are made again: a triangle stretched past tearing is torn there)
            const bool vol_staged = j == 0 && (detected || !isl.needs_pairs);
            if (vol_staged)
                for (int c = 0, n = collide_volumes(isl, dt, isl.bodies.size() > 1, 1); c < n; c++) isl.work.push_back({0u, 7, (uint32_t)c, 0});
            {
                PROFILE_ACCUM("Forces");
                team.run((int)isl.work.size(), [&](int i) {
                    const Island::Work& w = isl.work[i];
                    if (w.kind >= 5) return w.kind == 5 ? pair_detect(isl, (int)w.a) : w.kind == 6 ? mid_detect(isl, (int)w.a) : volume_find(isl, (int)w.a);
                    SoftBody& b = *isl.bodies[w.body];
                    if (w.kind == 0) {
                        base_forces(b, dt / (float)isl.subs[w.body], j == 0, isl.fem_parts[w.body] != 0);
                    } else if (w.kind == 4) {
                        PROFILE_ACCUM("Frame elements");
                        b.fem.eval_forces(b, (int)w.a);
                    } else {
                        PROFILE_ACCUM("Sheet elements");
                        b.shell_eval((int)w.a);
                    }
                });
                lap(0);
                // the frames' forces onto their nodes, in chunks of nodes (each node's in the elements' order), then
                // their mounts and events
                {
                    constexpr size_t kGatherChunk = 128;
                    isl.fem_tasks.clear();
                    for (int k = 0; k < nb; k++)
                        if (isl.fem_parts[k])
                            for (size_t c = 0; c * kGatherChunk < isl.bodies[k]->fem.node.size(); c++) isl.fem_tasks.push_back({k, (int)c});
                    team.run((int)isl.fem_tasks.size(), [&](int t) {
                        PROFILE_ACCUM("Frame elements");
                        SoftBody& b = *isl.bodies[isl.fem_tasks[t].first];
                        const size_t c = (size_t)isl.fem_tasks[t].second;
                        b.fem.gather_forces(b, c * kGatherChunk, (c + 1) * kGatherChunk);
                    });
                }
                for (int k = 0; k < nb; k++)
                    if (isl.fem_parts[k]) isl.bodies[k]->fem.end_forces_rest(*isl.bodies[k]);
                for (int k = 0; k < nb; k++)
                    if (isl.subs[k] > j && isl.bodies[k]->shk.pass.chunks > 0) isl.bodies[k]->shell_end();
                lap(4);
            }
            isl.fem_pre.assign(nb, 0);
            if (j == 0) {
                // 2) the sheets' forces on their nodes before the contacts (a contact cancels the forces accumulated so
                // far), and the snapshot of the internal forces of the sub-cycled bodies
                node_work(0, true);
                for (int k = 0; k < nb; k++)
                    if (isl.subs[k] > 1) isl.bodies[k]->ext_force.resize(isl.bodies[k]->nodes.size());
                if (!isl.work.empty()) {
                    PROFILE_ACCUM("Forces");
                    team.run((int)isl.work.size(), [&](int i) {
                        const Island::Work& w = isl.work[i];
                        SoftBody& b = *isl.bodies[w.body];
                        vec3* F = b.force.data();
                        if (!b.shells.empty() && !b.rigid) {
                            PROFILE_ACCUM("Sheet gather");
                            b.shell_gather(w.a, w.b, F);
                        }
                        if (isl.subs[w.body] > 1) std::copy(F + w.a, F + w.b, b.ext_force.data() + w.a);
                    });
                }
                lap(1);
                if (isl.teamed && isl.needs_pairs) {
                    // (the frames stepping now - as the integration below will find them: fem_begin -, their step begun
                    // and their matrices' blocks gathered on the team's other board while this thread takes the contacts
                    // in order: the elements' tangents are in, the contacts do not change them; FemFrame::gather_blocks.
                    // After the sub-cycled bodies' arrays of the contacts' forces are made, above: solve_begin keeps them.
                    // Without contacts between bodies there is nothing to hide them behind: gathered whole below)
                    isl.fem_pre_nc.assign(nb, 0);
                    isl.pre_tasks.clear();
                    for (int k = 0; k < nb; k++) {
                        SoftBody& b = *isl.bodies[k];
                        if (isl.subs[k] <= j || b.rigid || b.fem.empty() || !(b.fem_every_step || (j == 0 && fem_due(b)))) continue;
                        const int nc = fem_begin(k);
                        isl.fem_pre[k] = 1, isl.fem_pre_nc[k] = nc;
                        for (int c = 0; c < nc; c++)
                            for (int g = 0, n = b.fem.par_component(c) ? b.fem.asm_chunks(c) : 1; g < n; g++) isl.pre_tasks.push_back({k, c, b.fem.par_component(c) ? g : -1});
                    }
                    aside.post((int)isl.pre_tasks.size(), [&isl](int t) {
                        PROFILE_ACCUM("Frame blocks");
                        const Island::PreTask& pt = isl.pre_tasks[t];
                        SoftBody& b = *isl.bodies[pt.k];
                        if (pt.chunk < 0) b.fem.gather_blocks(b, pt.c);
                        else b.fem.gather_blocks(b, pt.c, pt.chunk);
                    });
                }
                // 3) inter-body contacts (forces)
                if (isl.needs_pairs) {
                    PROFILE_ACCUM("Collisions");
                    const uint64_t c0 = prof::now();
                    if (s == 0 || s >= isl.next_rebuild || rebuild_now) {
                        rebuild_pairs(isl);
                        near_pairs(isl);
                        isl.next_rebuild = s + isl.rebuild_interval;
                        isl.next_fast = s + isl.fast_interval;
                        rebuild_now = false;
                        isl.ph_ms[6] += prof::ticks_to_ms(prof::now() - c0);
                    } else if (isl.fast_interval > 0 && isl.fast_interval < isl.rebuild_interval && s >= isl.next_fast) {
                        refresh_fast_pairs(isl);
                        near_pairs(isl);
                        isl.next_fast = s + isl.fast_interval;
                        isl.ph_ms[7] += prof::ticks_to_ms(prof::now() - c0);
                    }
                    const uint64_t c1 = prof::now();
                    collide_pairs(isl, detected);
                    collide_mid_pairs(isl, detected);
                    isl.ph_ms[8] += prof::ticks_to_ms(prof::now() - c1);
                }
                collide_volumes(isl, dt, isl.bodies.size() > 1, vol_staged ? 2 : 0); // (the collision volumes: against the other bodies and the static world)
                lap(2);
            }
            // 4) static contacts + integration (the sheets' forces of the later short steps are gathered here)
            node_work(j, false);
            isl.fem_defer.assign(isl.work.size(), 0);
            for (int k = 0; k < nb; k++) { // (the chunked bodies' arrays collide_static fills, made here: the chunks share them)
                SoftBody& b = *isl.bodies[k];
                if (isl.fem_chunked[k] && b.tri_mids && !b.fem.tris.empty() && b.static_pen.size() != b.nodes.size()) b.static_pen.assign(b.nodes.size(), vec3(0));
            }
            // (and their plates' mid points' static contacts found beside those, in ranges: one after another they were
            // the step's wait before the frames' steps; their forces in order after: below)
            constexpr uint32_t kMidChunk = 64;
            isl.smid_tasks.clear();
            isl.smid_first.assign(nb, -1);
            for (int k = 0; k < nb; k++) {
                const SoftBody& b = *isl.bodies[k];
                if (!isl.fem_chunked[k] || b.fem.tris.empty()) continue;
                bool any = false;
                for (size_t i = 0; i < isl.work.size() && !any; i++) any = (int)isl.work[i].body == k && isl.work[i].a == 0;
                if (!any) continue;
                isl.smid_first[k] = (int)isl.smid_tasks.size();
                for (uint32_t c = 0; (size_t)c * kMidChunk < b.fem.tris.size(); c++) isl.smid_tasks.push_back({k, c});
            }
            if (isl.smid.size() < isl.smid_tasks.size()) isl.smid.resize(isl.smid_tasks.size());
            {
                PROFILE_ACCUM("Integration");
                const int nwork = (int)isl.work.size();
                team.run(nwork + (int)isl.smid_tasks.size(), [&](int i) {
                    if (i >= nwork) {
                        PROFILE_ACCUM("Static collisions");
                        const auto [k, c] = isl.smid_tasks[i - nwork];
                        static_mids_find(*isl.bodies[k], (size_t)c * kMidChunk, (size_t)(c + 1) * kMidChunk, isl.box_ids[k], isl.cyl_ids[k], isl.terrain_max[k], isl.smid[i - nwork]);
                        return;
                    }
                    const Island::Work& w = isl.work[i];
                    const int k = (int)w.body;
                    SoftBody& b = *isl.bodies[k];
                    const int sub = isl.subs[k];
                    const float bdt = dt / (float)sub;
                    vec3* F = b.force.data();
                    if (j > 0 && !b.shells.empty() && !b.rigid) {
                        PROFILE_ACCUM("Sheet gather");
                        b.shell_gather(w.a, w.b, F);
                    }
                    if (sub > 1) {
                        // the contact forces of the substep, held over the short steps
                        vec3* E = b.ext_force.data();
                        const size_t ne = std::min((size_t)w.b, b.ext_force.size());
                        if (j == 0)
                            for (size_t i2 = w.a; i2 < ne; i2++) E[i2] = F[i2] - E[i2];
                        else
                            for (size_t i2 = w.a; i2 < ne; i2++) F[i2] += E[i2];
                    }
                    Island::NodePart& out = isl.parts[i];
                    out.contacts = 0;
                    {
                        PROFILE_ACCUM("Static collisions");
                        collide_static(b, w.a, w.b, isl.box_ids[k], isl.cyl_ids[k], isl.terrain_max[k], bdt, out.contacts);
                        if (!isl.fem_chunked[k]) {
                            if (w.a == 0 && !b.wheels.empty() && !b.rigid) ring_tyres(b, isl.box_ids[k], isl.cyl_ids[k], isl.terrain_max[k], bdt);
                            if (w.a == 0 && w.b == b.nodes.size() && !b.fem.tris.empty()) collide_static_mids(b, isl.box_ids[k], isl.cyl_ids[k], isl.terrain_max[k], bdt, out.contacts);
                        }
                    }
                    PROFILE_ACCUM("Integrate");
                    out.mn = vec3(1e30f);
                    out.mx = vec3(-1e30f);
                    out.max_v2 = 0;
                    isl.fem_defer[i] = 0;
                    if (isl.fem_chunked[k]) { // (the rest of it once the body's chunks are in: below)
                        isl.fem_defer[i] = 5;
                        return;
                    }
                    if (w.kind == 3) {
                        b.rigid_step(bdt, out.contacts > 0, out.mn, out.mx, out.max_v2);
                    } else {
                        if (!b.fem.empty()) {
                            // every force on the frame nodes is in: their implicit step (the new velocities as forces),
                            // once per substep, next, a component each; the short steps of a sub-cycled body hold them
                            // at those velocities
                            if (b.fem_every_step || (j == 0 && fem_due(b))) {
                                isl.fem_defer[i] = 1;
                                return;
                            }
                            static const bool no_held = getenv("BL_NOHELD") != nullptr; // (diagnostics: held as before)
                            if (!b.rigid && !no_held) { // (between the frame's steps: its factorization answers this step's forces)
                                isl.fem_defer[i] = 2;
                                return;
                            }
                            b.fem.hold(b, bdt);
                        }
                        b.integrate_nodes(w.a, w.b, bdt, out.mn, out.mx, out.max_v2);
                    }
                });
                // the chunked frames' bodies: their ring tyres and plates' mid points against the static world, then the
                // frame's step (on the body's first chunk; the others integrate after it: 4)
                {
                    isl.fem_tasks.clear();
                    for (int i = 0; i < (int)isl.work.size(); i++)
                        if (isl.fem_defer[i] == 5 && isl.work[i].a == 0) isl.fem_tasks.push_back({i, 0});
                    team.run((int)isl.fem_tasks.size(), [&](int t) {
                        const int i0 = isl.fem_tasks[t].first;
                        const int k = (int)isl.work[i0].body;
                        SoftBody& b = *isl.bodies[k];
                        const float bdt = dt / (float)isl.subs[k];
                        {
                            PROFILE_ACCUM("Static collisions");
                            if (!b.wheels.empty() && !b.rigid) ring_tyres(b, isl.box_ids[k], isl.cyl_ids[k], isl.terrain_max[k], bdt);
                            if (const int t0 = isl.smid_first[k]; t0 >= 0)
                                for (int q = t0; q < (int)isl.smid_tasks.size() && isl.smid_tasks[q].first == k; q++) static_mids_apply(b, isl.smid[q], bdt, isl.parts[i0].contacts);
                        }
                        int first = 4;
                        static const bool no_held = getenv("BL_NOHELD") != nullptr;
                        if (b.fem_every_step || (j == 0 && fem_due(b))) first = 1;
                        else if (!b.rigid && !no_held) first = 2;
                        else b.fem.hold(b, bdt);
                        for (int i = i0; i < (int)isl.work.size() && (int)isl.work[i].body == k; i++) isl.fem_defer[i] = i == i0 ? first : 4;
                    });
                }
                // the frames' implicit steps: each frame's components (parts held by mounts, solved apart) beside each
                // other and the other frames'; then those bodies' integration
                {
                    PROFILE_ACCUM("Frame blocks wait");
                    aside.wait();
                }
                for (const Island::PreTask& pt : isl.pre_tasks) isl.bodies[pt.k]->fem.mark_blocks(pt.c);
                isl.pre_tasks.clear();
                isl.fem_work.clear();
                for (int i = 0; i < (int)isl.work.size(); i++) {
                    if (!isl.fem_defer[i] || isl.fem_defer[i] == 4) continue;
                    const int k = (int)isl.work[i].body;
                    SoftBody& b = *isl.bodies[k];
                    const float bdt = dt / (float)isl.subs[k];
                    int nc = 0;
                    if (isl.fem_defer[i] == 2) {
                        nc = b.fem.held_begin(b, bdt);
                        if (nc == 0) isl.fem_defer[i] = 3, b.fem.hold(b, bdt); // (no factorization yet: held)
                    } else {
                        nc = isl.fem_pre[k] ? isl.fem_pre_nc[k] : fem_begin(k);
                        isl.fem_pre[k] = 2;
                    }
                    for (int c = 0; c < nc; c++) isl.fem_work.push_back({(uint32_t)i, c});
                }
                for (int k = 0; k < nb; k++) // (begun beside the contacts and not stepping now: never - see above)
                    if (isl.fem_pre[k] == 1) fprintf(stderr, "frame of body %d begun beside the contacts, not stepped\n", k);
                if (std::find_if(isl.fem_defer.begin(), isl.fem_defer.end(), [](char c) { return c != 0; }) != isl.fem_defer.end()) {
                    // the large components (FemFrame::par_component) in the team's stages: their assembly in chunks
                    // of columns, the rest of it, the factor's subtrees, the updates above those, then the rest of the
                    // step beside the small components' whole steps and the held ones
                    isl.fem_par.clear(), isl.fem_small.clear(), isl.fem_hpar.clear();
                    for (const auto& fw : isl.fem_work) {
                        const FemFrame& f = isl.bodies[isl.work[fw.first].body]->fem;
                        if (isl.fem_defer[fw.first] == 1 && f.par_component(fw.second)) isl.fem_par.push_back(fw);
                        else if (isl.fem_defer[fw.first] == 2 && f.held_par(fw.second)) isl.fem_hpar.push_back(fw);
                        else isl.fem_small.push_back(fw);
                    }
                    auto small = [&](int q) {
                        const auto& fw = isl.fem_small[q];
                        SoftBody& b = *isl.bodies[isl.work[fw.first].body];
                        if (isl.fem_defer[fw.first] == 2) {
                            b.fem.held_component(b, fw.second);
                        } else {
                            PROFILE_ACCUM("Frame solve");
                            b.fem.solve_component(b, fw.second);
                        }
                    };
                    // (the large components held: their groups' right side and forward substitution beside the rest,
                    // then the columns above, then the groups' backward substitution - held_component one after
                    // another was a car's body's 24 us on one thread at every later short step)
                    auto hpar_body = [&](int q) -> SoftBody& { return *isl.bodies[isl.work[isl.fem_hpar[q].first].body]; };
                    isl.fem_htasks.clear();
                    for (int q = 0; q < (int)isl.fem_hpar.size(); q++)
                        for (int g = 0, n = hpar_body(q).fem.par_groups(isl.fem_hpar[q].second); g < n; g++) isl.fem_htasks.push_back({q, g});
                    const int nsmall = (int)isl.fem_small.size(), nhf = (int)isl.fem_htasks.size();
                    auto small_or_front = [&](int q) {
                        if (q < nsmall) return small(q);
                        const auto [h, g] = isl.fem_htasks[q - nsmall];
                        hpar_body(h).fem.held_front(hpar_body(h), isl.fem_hpar[h].second, g);
                    };
                    auto held_rest = [&]() {
                        if (isl.fem_hpar.empty()) return;
                        team.run((int)isl.fem_hpar.size(), [&](int q) { hpar_body(q).fem.held_top(hpar_body(q), isl.fem_hpar[q].second); });
                        team.run(nhf, [&](int t) {
                            const auto [h, g] = isl.fem_htasks[t];
                            hpar_body(h).fem.held_back(hpar_body(h), isl.fem_hpar[h].second, g);
                        });
                    };
                    if (isl.fem_par.empty() && !isl.fem_hpar.empty() && team.parallel()) {
                        // (the small ones taken by whoever is free in the large ones' three stages - the stages' own
                        // chunks first, then the small ones till those are done, the last stage till none is left: all
                        // of them in the first stage made it wait for them while two threads had the columns above)
                        std::atomic<int> next_small{0}, stage_done{0};
                        const int nfill = JobSystem::get().num_threads(), nhp = (int)isl.fem_hpar.size();
                        auto fill = [&](int need, bool drain) {
                            for (;;) {
                                if (!drain && stage_done.load(std::memory_order_acquire) >= need) return;
                                const int q = next_small.fetch_add(1, std::memory_order_acq_rel);
                                if (q >= nsmall) return;
                                small(q);
                            }
                        };
                        auto stage = [&](int n, bool drain, auto&& own) {
                            stage_done.store(0, std::memory_order_relaxed);
                            team.run(n + nfill, [&](int t) {
                                if (t >= n) return fill(n, drain);
                                own(t);
                                stage_done.fetch_add(1, std::memory_order_acq_rel);
                            });
                        };
                        stage(nhf, false, [&](int t) {
                            const auto [h, g] = isl.fem_htasks[t];
                            hpar_body(h).fem.held_front(hpar_body(h), isl.fem_hpar[h].second, g);
                        });
                        stage(nhp, false, [&](int q) { hpar_body(q).fem.held_top(hpar_body(q), isl.fem_hpar[q].second); });
                        stage(nhf, true, [&](int t) {
                            const auto [h, g] = isl.fem_htasks[t];
                            hpar_body(h).fem.held_back(hpar_body(h), isl.fem_hpar[h].second, g);
                        });
                        for (int q = next_small.load(); q < nsmall; q++) small(q); // (none left: the last stage drained them)
                    } else if (isl.fem_par.empty()) {
                        team.run(nsmall + nhf, small_or_front);
                        held_rest();
                    } else {
                        auto par_body = [&](int q) -> SoftBody& { return *isl.bodies[isl.work[isl.fem_par[q].first].body]; };
                        std::vector<int> all_par(isl.fem_par.size()), repass, back;
                        for (int q = 0; q < (int)isl.fem_par.size(); q++) all_par[q] = q;
                        const std::vector<int>* sel = &all_par;
                        // (0: the assembly's chunks, 1: the subtrees' groups, 2: the deferred updates' groups, 3: the
                        // subtrees' backward substitution)
                        auto tasks = [&](int which) {
                            isl.fem_tasks.clear();
                            for (int q : *sel) {
                                const FemFrame& f = par_body(q).fem;
                                const int c = isl.fem_par[q].second;
                                const int n = which == 0 ? f.asm_chunks(c) : which == 2 ? f.par_defers(c) : f.par_groups(c);
                                for (int g = 0; g < n; g++) isl.fem_tasks.push_back({q, g});
                            }
                            team.run((int)isl.fem_tasks.size(), [&](int t) {
                                const int q = isl.fem_tasks[t].first, g = isl.fem_tasks[t].second, c = isl.fem_par[q].second;
                                SoftBody& b = par_body(q);
                                PROFILE_ACCUM("Frame solve");
                                if (which == 0) b.fem.solve_gather(b, c, g);
                                else if (which == 1) b.fem.solve_factor(c, g);
                                else if (which == 2) b.fem.solve_defer(c, g);
                                else b.fem.solve_back(c, g);
                            });
                        };
                        {
                            PROFILE_ACCUM("FEM assemble");
                            tasks(0);
                            team.run((int)isl.fem_par.size(), [&](int q) {
                                PROFILE_ACCUM("Frame solve");
                                par_body(q).fem.solve_assemble(par_body(q), isl.fem_par[q].second);
                            });
                        }
                        {
                            PROFILE_ACCUM("FEM factor");
                            tasks(1);
                            tasks(2);
                        }
                        PROFILE_ACCUM("FEM finish");
                        const int np = (int)isl.fem_par.size();
                        // (the components that factored: their subtrees' backward substitution, then the rest of the pass)
                        auto back_end = [&](const std::vector<int>& from) {
                            back.clear();
                            for (int q : from)
                                if (par_body(q).fem.wants_back(isl.fem_par[q].second)) back.push_back(q);
                            if (back.empty()) return;
                            sel = &back;
                            tasks(3);
                            team.run((int)back.size(), [&](int i) {
                                PROFILE_ACCUM("Frame solve");
                                par_body(back[i]).fem.solve_finish_end(par_body(back[i]), isl.fem_par[back[i]].second);
                            });
                        };
                        // (the finish of each - the small ones beside it)
                        auto finish_run = [&](const std::vector<int>& list, bool with_small) {
                            const int nl = (int)list.size();
                            team.run(nl + (with_small ? nsmall + nhf : 0), [&](int t) {
                                if (t >= nl) return small_or_front(t - nl);
                                SoftBody& b = par_body(list[t]);
                                PROFILE_ACCUM("Frame solve");
                                b.fem.solve_finish(b, isl.fem_par[list[t]].second);
                            });
                        };
                        finish_run(all_par, true);
                        back_end(all_par);
                        // (a hinge unloaded: that component's pass again, in the stages too - one after another it was a
                        // whole factorization of a car's body on one thread, twice in a crash's substep)
                        for (int round = 0; round < 2; round++) {
                            repass.clear();
                            for (int q = 0; q < np; q++)
                                if (par_body(q).fem.wants_repass(isl.fem_par[q].second)) repass.push_back(q);
                            if (repass.empty()) break;
                            PROFILE_ACCUM("FEM repass");
                            sel = &repass;
                            team.run((int)repass.size(), [&](int i) {
                                PROFILE_ACCUM("Frame solve");
                                par_body(repass[i]).fem.solve_repass(par_body(repass[i]), isl.fem_par[repass[i]].second);
                            });
                            tasks(1);
                            tasks(2);
                            finish_run(repass, false);
                            back_end(repass);
                        }
                        held_rest();
                    }
                    for (int i = 0; i < (int)isl.work.size(); i++) {
                        if (isl.fem_defer[i] == 1) isl.bodies[isl.work[i].body]->fem.solve_end(*isl.bodies[isl.work[i].body]);
                        if (isl.fem_defer[i] == 2) isl.bodies[isl.work[i].body]->fem.held_end(*isl.bodies[isl.work[i].body]);
                    }
                    team.run((int)isl.work.size(), [&](int i) {
                        if (!isl.fem_defer[i]) return;
                        PROFILE_ACCUM("Integrate");
                        const Island::Work& w = isl.work[i];
                        SoftBody& b = *isl.bodies[w.body];
                        Island::NodePart& out = isl.parts[i];
                        b.integrate_nodes(w.a, w.b, dt / (float)isl.subs[w.body], out.mn, out.mx, out.max_v2);
                    });
                }
            }
            lap(3);
            // 4b) the sheets' sphere contacts (between them, and each deformed one with itself), after the integration
            isl.sph.clear();
            isl.sph_group.clear();
            bool sph_awake = false, any_ball = false;
            for (int k = 0; k < nb; k++) any_ball |= isl.bodies[k]->sphere_ball > 0 && !isl.bodies[k]->sleeping;
            for (int k = 0; k < nb; k++) {
                SoftBody& b = *isl.bodies[k];
                // (a ball takes one step a substep, but meets the sheets at their every short step: its position and
                // velocity pushed between its steps; left out, a sheet's nodes sank 5 cm into a ball resting on it)
                const bool sheet = !b.shells.empty() && (b.sphere_contacts || (b.sphere_target && any_ball));
                if ((isl.subs[k] > j || b.sleeping || b.sphere_ball > 0) && !b.rigid && (sheet || b.sphere_ball > 0)) {
                    isl.sph.push_back(&b), isl.sph_group.push_back(b.collision_group);
                    sph_awake |= !b.sleeping;
                }
            }
            // (between the bodies every short step, as the membranes that carry the push, the pairs found at the first;
            // a sleeping one as an obstacle)
            if (sph_awake) sphere_contacts(isl.sph, dt, isl.sph_group, settings.sleep_speed, isl.sph_pairs, j == 0, dt * (float)maxsub);
            // 5a) the sheets' membranes, the bodies in parallel (each one's projection is its own; one after another
            // they were a third of a pile of barrels' step)
            isl.mem_bodies.clear();
            for (size_t i = 0; i < isl.work.size();) {
                const int k = (int)isl.work[i].body;
                if (!isl.bodies[k]->shells.empty() && !isl.bodies[k]->rigid) isl.mem_bodies.push_back(k);
                while (i < isl.work.size() && (int)isl.work[i].body == k) i++;
            }
            isl.mem_speed.assign(nb, -1.0f);
            team.run((int)isl.mem_bodies.size(), [&](int q) {
                PROFILE_ACCUM("Sheet membrane");
                const int k = isl.mem_bodies[q];
                SoftBody& b = *isl.bodies[k];
                // (metal sheets: the in-plane stiffness the step cannot give; the speed that counts is the one left
                // after it: a dented sheet at rest has its hinges pull a little every step and the projection take
                // it back, and would never sleep on the speed before)
                const int moved_now = b.project_membrane(dt / (float)isl.subs[k]);
                if (moved_now > 0 || b.membrane_projected()) {
                    float m2 = 0;
                    for (const Node& x : b.nodes) m2 = std::max(m2, length2(x.v));
                    isl.mem_speed[k] = std::sqrt(m2);
                }
            });
            // 5) per body: bounds, top speed, frames; then the sheets' refinement and cracks. (The frames' double
            // positions first, in chunks: one after another they were a millisecond of a crash's frame)
            {
                PROFILE_ACCUM("Frame sync");
                constexpr size_t kSyncChunk = 64; // (a node's turn ~45 ns: 256 of them were a car's 11 us on one thread)
                isl.fem_tasks.clear();
                for (size_t i = 0; i < isl.work.size();) {
                    const int k = (int)isl.work[i].body;
                    const SoftBody& b = *isl.bodies[k];
                    if (!b.fem.empty() && !b.rigid)
                        for (size_t c = 0; c * kSyncChunk < b.fem.node.size(); c++) isl.fem_tasks.push_back({k, (int)c});
                    while (i < isl.work.size() && (int)isl.work[i].body == k) i++;
                }
                team.run((int)isl.fem_tasks.size(), [&](int t) {
                    SoftBody& b = *isl.bodies[isl.fem_tasks[t].first];
                    const size_t c = (size_t)isl.fem_tasks[t].second;
                    b.fem.sync_positions(b, dt / (float)isl.subs[isl.fem_tasks[t].first], c * kSyncChunk, (c + 1) * kSyncChunk);
                });
            }
            // (the frames' splits and tears of the bodies that have some, beside each other: each one's pattern analysed
            // again, a third of a millisecond, both cars' one after the other in a crash's substeps)
            isl.ev_bodies.clear();
            isl.ev_done.assign(nb, 0);
            for (size_t i = 0; i < isl.work.size();) {
                const int k = (int)isl.work[i].body;
                const SoftBody& b = *isl.bodies[k];
                if (!b.fem.empty() && !b.rigid && b.fem.pending()) isl.ev_bodies.push_back(k);
                while (i < isl.work.size() && (int)isl.work[i].body == k) i++;
            }
            if (isl.ev_bodies.size() >= 2)
                team.run((int)isl.ev_bodies.size(), [&](int q) {
                    PROFILE_ACCUM("Frame events");
                    const int k = isl.ev_bodies[q];
                    SoftBody& b = *isl.bodies[k];
                    isl.ev_done[k] = b.fem.process_events(b) ? 2 : 1;
                });
            for (size_t i = 0; i < isl.work.size();) {
                const int k = (int)isl.work[i].body;
                SoftBody& b = *isl.bodies[k];
                vec3 mn(1e30f), mx(-1e30f);
                float v2 = 0;
                for (; i < isl.work.size() && (int)isl.work[i].body == k; i++) {
                    const Island::NodePart& p = isl.parts[i];
                    mn = vmin(mn, p.mn);
                    mx = vmax(mx, p.mx);
                    v2 = std::max(v2, p.max_v2);
                    b.static_contacts += p.contacts;
                }
                b.aabb.mn = mn;
                b.aabb.mx = mx;
                float ms = std::sqrt(v2);
                if (isl.mem_speed[k] >= 0) ms = isl.mem_speed[k]; // (after its membrane's projection: 5a)
                if (ms > b.max_speed) b.max_speed = ms;
                if (!b.fem.empty() && !b.rigid) {
                    PROFILE_ACCUM("Frame events");
                    // the frame's splits and tears (new nodes: their contact pairs are inherited below)
                    const bool changed = isl.ev_done[k] ? isl.ev_done[k] == 2 : b.fem.pending() && b.fem.process_events(b);
                    if (changed) b.topo_changed = true, b.topo_version++, b.shk.version++;
                }
                b.integrate_frames(dt / (float)isl.subs[k]);
                b.fem.cap_loose(b);
                if (!b.rigid && (!b.shell_events.empty() || b.shell_hit.speed > 0 || b.pattern_passes > 0)) {
                    lap(4);
                    PROFILE_ACCUM("Sheet events");
                    b.process_shell_events();
                    lap(5);
                }
            }
            lap(4);
        }
        for (int k = 0; k < nb; k++) {
            SoftBody& b = *isl.bodies[k];
            if (isl.subs[k] > 0 && !b.fem.empty()) b.fem_left = b.fem_left <= 0 ? b.fem_period - 1 : b.fem_left - 1;
        }
        // new nodes and triangles: they take over the contact pairs of what they came from (or a new search)
        for (int k = 0; k < nb; k++) {
            SoftBody& b = *isl.bodies[k];
            if (!b.topo_changed) continue;
            b.topo_changed = false;
            if (isl.needs_pairs && !rebuild_now) {
                if (b.topo_log.overflow || b.self_collision || settings.pair_search_on_topology) rebuild_now = true;
                else inherit_pairs(isl, k);
            }
            b.topo_log.clear();
        }
        lap(2);
    }
    isl.team = nullptr;
    int static_contacts = 0;
    for (SoftBody* b : isl.bodies) static_contacts += b->static_contacts;
    isl.contacts = (isl.contacts + static_contacts) / std::max(1, substeps);
    const float frame_time = substeps * dt;
    for (SoftBody* b : isl.bodies) {
        if (b->sleeping) continue;
        if (!std::isfinite(b->aabb.mn.x) || !std::isfinite(b->aabb.mx.y) || maxc(b->aabb.extent()) > 800.0f || b->max_speed > kMaxNodeSpeed * 0.9f) {
            // numerical trouble in a few nodes: put them back among their neighbours instead of freezing the body
            int fixed = repair_nodes(*b);
            if (fixed > 0) log_warn("body '%s': %d unstable node(s) reset", b->name.c_str(), fixed);
            if (fixed < 0) {
                log_warn("body '%s' exploded - freezing", b->name.c_str());
                for (auto& n : b->nodes) {
                    if (!std::isfinite(n.p.x) || !std::isfinite(n.p.y) || !std::isfinite(n.p.z)) n.p = vec3(0, -1000, 0);
                    n.v = vec3(0);
                    n.inv_mass = 0;
                }
                b->compute_aabb();
                b->sleeping = true;
                continue;
            }
        }
        if (b->post_frame) b->post_frame(*b, frame_time);
        b->guard_quiet = b->body_contacts > 0 || b->sphere_touches > 0 ? 0 : b->guard_quiet + 1;
        if (b->energy_guard && !b->rigid && b->guard_quiet > 15 && b->grab_node < 0 && b->wind_area.empty()) b->guard_energy(settings.gravity);
        b->resting = false, b->rigid_speed = 1e9f; // (set by the rest damping: on the ground or another, slow as a whole)
        if (b->rest_damp > 0 && (b->static_contacts > 0 || b->sphere_touches > 0 || b->body_contacts > 0) && !b->rigid) b->apply_rest_damping(frame_time, dt);
        for (uint8_t& t : b->ground_touch) t &= 1;
        update_sleep(*b, frame_time);
    }
    // a sheet with rest damping ready to sleep waits for the bodies next to it: they fall asleep together (one asleep is
    // an immovable obstacle, its neighbour's contacts changed at once, and the jolt woke it again: a drum on another
    // and the one below took turns for good)
    for (SoftBody* b : isl.bodies) {
        if (!b->sleep_ready) continue;
        bool all = true;
        for (const SoftBody* o : b->touched) all &= o->sleeping || o->sleep_ready;
        if (all) b->fall_asleep();
    }
    // profiling: wall time and work of this island (the triangles and hinges the force kernel evaluated: counted by it)
    isl.max_sub = 1;
    for (SoftBody* b : isl.bodies) {
        isl.max_sub = std::max(isl.max_sub, 1 << b->dt_shift());
        isl.shell_steps += b->evals_shell;
        isl.hinge_evals += b->evals_hinge;
        b->evals_shell = b->evals_hinge = 0;
    }
    isl.ms = prof::ticks_to_ms(prof::now() - isl_t0);
}

int World::repair_nodes(SoftBody& b) {
    // reference: the body's median position; a node is bad when it is not finite, too fast or flung far away
    const size_t n = b.nodes.size();
    std::vector<float> xs, ys, zs;
    for (const Node& x : b.nodes)
        if (std::isfinite(x.p.x) && std::isfinite(x.p.y) && std::isfinite(x.p.z)) {
            xs.push_back(x.p.x);
            ys.push_back(x.p.y);
            zs.push_back(x.p.z);
        }
    if (xs.empty()) return -1;
    auto median = [](std::vector<float>& v) {
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
    };
    const vec3 med(median(xs), median(ys), median(zs));
    const float reach = 300.0f;
    b.fem.repair(); // (orientations of frame nodes that went bad)
    std::vector<uint8_t> bad(n, 0);
    int nbad = 0;
    for (size_t i = 0; i < n; i++) {
        const Node& x = b.nodes[i];
        bool finite = std::isfinite(x.p.x) && std::isfinite(x.p.y) && std::isfinite(x.p.z) && std::isfinite(x.v.x) && std::isfinite(x.v.y) &&
                      std::isfinite(x.v.z);
        if (!finite || length2(x.v) > sqr(0.9f * kMaxNodeSpeed) || length(x.p - med) > reach) {
            bad[i] = 1;
            nbad++;
            static const bool dbg = getenv("BL_REPAIRDBG") != nullptr;
            if (dbg) printf("repair: body %s node %zu p (%.2f %.2f %.2f) |v| %.1f\n", b.name.c_str(), i, x.p.x, x.p.y, x.p.z, length(x.v));
            if (dbg && i < b.node_shells.size()) {
                // (diagnostics: the node's triangles and its stability budget at the body's step)
                const float h = settings.dt / (float)(1 << b.dt_shift());
                float ks = 0;
                fprintf(stderr, "repair: body '%s' node %zu speed %.0f m/s finite %d, mass %.2e, level %d, %zu shells:", b.name.c_str(), i, length(x.v), (int)finite, x.mass,
                        b.shell_level, b.node_shells[i].size());
                for (uint32_t si : b.node_shells[i]) {
                    const Shell& s = b.shells[si];
                    for (int e = 0; e < 3; e++)
                        if (s.n[e] == i || s.n[e == 2 ? 0 : e + 1] == i) ks += s.k[e];
                    fprintf(stderr, " [L%d area %.1e/%.1e kscale %.2f nb %d %d %d]", s.level, s.area0, s.area_nom, s.kscale, s.nb[0], s.nb[1], s.nb[2]);
                }
                fprintf(stderr, " k dt^2/m %.3f", x.inv_mass > 0 ? ks * h * h * x.inv_mass : 0.0f);
                const vec3 F = i < b.force.size() ? b.force[i] : vec3(0), E = i < b.ext_force.size() ? b.ext_force[i] : vec3(0);
                fprintf(stderr, " t %.4f p (%.2f %.2f %.2f) v (%.0f %.0f %.0f) F (%.0f %.0f %.0f) E (%.0f %.0f %.0f) static contacts %d\n", time(), x.p.x, x.p.y, x.p.z, x.v.x,
                        x.v.y, x.v.z, F.x, F.y, F.z, E.x, E.y, E.z, b.static_contacts);
            }
        }
    }
    if (nbad == 0) return 0;
    if (nbad * 2 > (int)n) return -1; // most of the body is gone: nothing sane to rebuild from
    const int reset = nbad;
    // move bad nodes onto the average of their good neighbours (a few passes reach nodes inside bad clusters)
    for (int pass = 0; pass < 8 && nbad > 0; pass++) {
        std::vector<vec3> sp(n, vec3(0)), sv(n, vec3(0));
        std::vector<int> cnt(n, 0);
        auto link = [&](uint32_t x, uint32_t y) {
            if (bad[x] && !bad[y]) {
                sp[x] += b.nodes[y].p;
                sv[x] += b.nodes[y].v;
                cnt[x]++;
            } else if (bad[y] && !bad[x]) {
                sp[y] += b.nodes[x].p;
                sv[y] += b.nodes[x].v;
                cnt[y]++;
            }
        };
        for (const Beam& bm : b.beams)
            if (!(bm.flags & BF_BROKEN)) link(bm.a, bm.b);
        for (const Shell& sh : b.shells)
            for (int e = 0; e < 3; e++) link(sh.n[e], sh.n[e == 2 ? 0 : e + 1]);
        for (size_t i = 0; i < n; i++)
            if (bad[i] && cnt[i] > 0) {
                b.nodes[i].p = sp[i] / (float)cnt[i];
                b.nodes[i].v = sv[i] / (float)cnt[i];
                bad[i] = 0;
                nbad--;
            }
    }
    // isolated leftovers: park them at the median, at rest
    for (size_t i = 0; i < n; i++)
        if (bad[i]) {
            b.nodes[i].p = med;
            b.nodes[i].v = vec3(0);
        }
    for (Frame& fr : b.frames) fr.w = vec3(0);
    b.compute_aabb();
    b.max_speed = 0;
    for (const Node& x : b.nodes) b.max_speed = std::max(b.max_speed, length(x.v));
    return reset;
}

void World::update_sleep(SoftBody& b, float frame_time) {
    b.sleep_ready = false;
    const bool windy = !b.wind_area.empty() && length2(settings.wind) > 0.01f && in_wind(b);
    if (!settings.sleeping || !b.can_sleep || b.grab_node >= 0 || windy) {
        b.sleep_timer = 0;
        return;
    }
    const float rest = std::max(settings.sleep_speed, b.sleep_speed);
    bool calm = b.max_speed < rest;
    if (!calm && !b.shells.empty() && b.max_speed < 4.0f * rest) {
        // a cracked sheet: the tip of some flap may still sway a little while the sheet as a whole is at rest
        double mv2 = 0, m = 0;
        for (const Node& n : b.nodes) {
            mv2 += (double)n.mass * length2(n.v);
            m += n.mass;
        }
        calm = m > 0 && mv2 / m < sqr(0.5 * rest);
    }
    // (and still as a whole but for a wobble, three times that on average over half a second, for 3 s: a drum crushed
    // flat by a fall kept its skin trembling at a few tenths of a joule and wandered about at a few cm/s)
    if (b.rest_damp > 0 && b.rigid_speed < 1e8f) { // (not measured: no contact this frame, a flicker in a pile)
        b.rigid_avg += (std::min(b.rigid_speed, 10.0f) - b.rigid_avg) * std::min(1.0f, frame_time / 0.5f);
        b.rest_timer = b.rigid_avg < 3.0f * rest ? b.rest_timer + frame_time : 0.0f;
    }
    if (!calm && b.rest_damp > 0 && b.rest_timer > 3.0f) calm = true;
    if (!calm && b.rest_damp > 0 && b.rigid_speed < rest) {
        // a body with rest damping on the ground, still as a whole: it sleeps though its skin trembles (a dented drum's
        // buckled end kept a few millimetres of trembling going, and it never slept but crept on)
        double mv2 = 0, m = 0;
        for (const Node& n : b.nodes) mv2 += (double)n.mass * length2(n.v), m += n.mass;
        calm = m > 0 && mv2 / m < sqr(0.5f);
    }
    if (calm) {
        b.sleep_timer += frame_time;
        if (b.sleep_timer > settings.sleep_time) {
            if (b.rest_damp > 0) b.sleep_ready = true; // (with its neighbours: simulate_island)
            else b.fall_asleep();
        }
    } else {
        b.sleep_timer = 0;
    }
}

int World::destroy_at(vec3 p, float radius, float impulse) {
    int broken = 0;
    const float r2 = radius * radius;
    auto seg_d2 = [&](vec3 a, vec3 b) {
        vec3 ab = b - a;
        float l2 = dot(ab, ab);
        float t = l2 > 1e-12f ? clampf(dot(p - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
        return length2(a + ab * t - p);
    };
    for (auto& bp : m_bodies) {
        SoftBody& b = *bp;
        AABB a = b.aabb;
        a.expand(radius);
        if (!a.contains(p)) continue;
        int before = broken;
        for (Beam& bm : b.beams) {
            if (bm.flags & BF_BROKEN) continue;
            if (seg_d2(b.nodes[bm.a].p, b.nodes[bm.b].p) > r2) continue;
            bm.flags |= BF_BROKEN;
            b.stats.broken_beams++;
            broken++;
        }
        for (Joint& j : b.joints) {
            if (j.broken) continue;
            if (seg_d2(b.nodes[b.frames[j.parent_frame].node].p, b.nodes[j.child_node].p) > r2) continue;
            j.broken = true;
            b.stats.broken_joints++;
            broken++;
        }
        if (!b.fem.empty()) {
            const int n = b.fem.break_near(b, p, radius);
            b.stats.broken_beams += n; // (the frame's members count with the beams, as when they break under load)
            broken += n;
        }
        if (!b.shells.empty()) broken += b.shatter_shells(p, radius);
        const std::vector<char> of_shell = b.shell_tri_mask(); // (the sheet's own triangles go with its shells)
        for (size_t ti = 0; ti < b.tris.size(); ti++) {
            Triangle& t = b.tris[ti];
            if (t.torn || of_shell[ti]) continue;
            vec3 bary;
            if (length2(closest_on_triangle(p, b.nodes[t.a].p, b.nodes[t.b].p, b.nodes[t.c].p, bary) - p) > r2) continue;
            t.torn = true;
            broken++;
        }
        for (SlideNode& s : b.slides)
            if (!s.broken && length2(b.nodes[s.node].p - p) < r2) {
                s.broken = true;
                broken++;
            }
        if (impulse > 0)
            for (Node& n : b.nodes) {
                vec3 d = n.p - p;
                float l2 = dot(d, d);
                if (l2 > r2 * 4.0f || n.inv_mass <= 0) continue;
                float l = std::sqrt(l2);
                n.v += (l > 1e-4f ? d / l : vec3(0, 1, 0)) * (impulse * (1.0f - l / (2.0f * radius)));
            }
        if (broken != before || impulse > 0) b.wake();
    }
    return broken;
}

int World::laser_cut(vec3 o, vec3 d0, vec3 d1, float range, const SoftBody* skip, float kerf) {
    vec3 m = cross(d0, d1);
    if (length2(m) < 1e-14f) return 0; // (the ray did not move: nothing swept)
    m = normalize(m);
    auto side = [&](vec3 p) { return dot(p - o, m); };
    auto in_wedge = [&](vec3 x) {
        const vec3 w = x - o;
        return dot(cross(d0, w), m) >= 0 && dot(cross(w, d1), m) >= 0 && dot(w, d0 + d1) > 0 && dot(w, w) < range * range;
    };
    // a segment crossing the swept sector
    auto crosses = [&](vec3 a, vec3 b) {
        const float sa = side(a), sb = side(b);
        if ((sa < 0) == (sb < 0)) return false;
        return in_wedge(a + (b - a) * (sa / (sa - sb)));
    };
    int cut = 0;
    for (auto& bp : m_bodies) {
        SoftBody& b = *bp;
        if (&b == skip) continue;
        int pos = 0, neg = 0;
        for (int k = 0; k < 8; k++) {
            const vec3 c((k & 1) ? b.aabb.mx.x : b.aabb.mn.x, (k & 2) ? b.aabb.mx.y : b.aabb.mn.y, (k & 4) ? b.aabb.mx.z : b.aabb.mn.z);
            (side(c) >= 0 ? pos : neg)++;
        }
        if (!pos || !neg) continue;
        const int before = cut;
        // (a node on the plane, within a centimetre of it, belongs to neither side: a beam to it is cut, its members on the
        // two sides part there - one shaking across the plane was on the far side as the sector passed it, and held on)
        auto on_plane = [&](vec3 p) { return std::fabs(side(p)) < 0.01f && in_wedge(p - m * side(p)); };
        for (Beam& bm : b.beams) {
            if ((bm.flags & BF_BROKEN) || !(crosses(b.nodes[bm.a].p, b.nodes[bm.b].p) || on_plane(b.nodes[bm.a].p) || on_plane(b.nodes[bm.b].p))) continue;
            bm.flags |= BF_BROKEN;
            b.stats.broken_beams++;
            cut++;
        }
        for (Joint& j : b.joints) {
            if (j.broken || !crosses(b.nodes[b.frames[j.parent_frame].node].p, b.nodes[j.child_node].p)) continue;
            j.broken = true;
            b.stats.broken_joints++;
            cut++;
        }
        for (Weld& wd : b.welds) { // (a weld whose sheet reaches across the cut: it holds neither side)
            if (wd.broken) continue;
            const vec3 ap = b.nodes[wd.anchor].p * (1 - wd.t) + b.nodes[wd.anchor2].p * wd.t;
            for (uint32_t i = wd.first; i < wd.first + wd.count && !wd.broken; i++)
                if (crosses(ap, b.nodes[b.weld_nodes[i]].p)) wd.broken = true, b.stats.broken_welds++, cut++;
        }
        for (CollisionVolume& cv : b.volumes) { // (a collision volume the cut runs through: the lump it stands for in two, off -
            // whole, a seat's stopped the axe's blade in the car)
            if (cv.broken || !cv.placed) continue;
            bool through = false;
            for (size_t i = 0; i < cv.wverts.size() && !through; i++)
                for (size_t j = i + 1; j < cv.wverts.size() && !through; j++) through = crosses(cv.wverts[i], cv.wverts[j]);
            if (through) cv.broken = true, cv.placed = false, cut++;
        }
        if (!b.fem.empty() && !b.rigid) {
            // frame members: cut where they cross the swept plane (the ones hit first, then the cuts: a split adds members)
            std::vector<std::pair<uint32_t, float>> hits;
            std::vector<uint32_t> along;
            for (size_t ei = 0; ei < b.fem.elems.size(); ei++) {
                const FrameElement& e = b.fem.elems[ei];
                if (e.broken) continue;
                const vec3 pa = b.nodes[b.fem.node[e.a]].p, pc = b.nodes[b.fem.node[e.b]].p;
                const float sa = side(pa), sc = side(pc);
                if (kerf > 0 && std::fabs(sa) < kerf && std::fabs(sc) < kerf && (e.torn & 3) != 3) { // (along the plane, in the blade's way)
                    const vec3 qa = pa - m * sa, qc = pc - m * sc;
                    if (in_wedge(qa) || in_wedge(qc) || in_wedge((qa + qc) * 0.5f)) along.push_back((uint32_t)ei);
                    continue;
                }
                if ((sa < 0) == (sc < 0)) continue;
                const float t = sa / (sa - sc);
                static const bool ldbg = getenv("BL_LASERDBG") != nullptr;
                if (ldbg) {
                    const vec3 x = pa + (pc - pa) * t, w = x - o;
                    auto ang = [&](vec3 v) { return std::atan2(dot(cross(vec3(0, -1, 0), v), m), dot(vec3(0, -1, 0), v)) * 57.2958f; };
                    printf("  member %zu crosses at (%.2f %.2f %.2f) r %.2f angle %.2f, wedge %.2f..%.2f -> %d\n", ei, x.x, x.y, x.z, length(w), ang(w), ang(d0), ang(d1),
                           (int)in_wedge(x));
                }
                if (in_wedge(pa + (pc - pa) * t)) hits.push_back({(uint32_t)ei, t});
            }
            int tears = 0;
            for (const auto& [ei, t] : hits) tears += b.fem.cut(b, ei, t);
            for (uint32_t ei : along)
                for (int end = 0; end < 2; end++)
                    if (ei < b.fem.elems.size() && !b.fem.elems[ei].broken && !(b.fem.elems[ei].torn & (1 << end))) tears += b.fem.tear(b, ei, end);
            // triangle elements across the cut: the shell parts along its edges nearest the cut (FemFrame::part_tris: the
            // nodes there duplicated, nothing removed)
            // (a triangle crossing it - or with a corner on its plane, within a centimetre: the corner joins the two sides'
            // triangles, none of which crosses)
            auto tri_crosses = [&](uint32_t ti) {
                const FrameTri& t = b.fem.tris[ti];
                const vec3 pa = b.nodes[b.fem.node[t.n[0]]].p, pb = b.nodes[b.fem.node[t.n[1]]].p, pc = b.nodes[b.fem.node[t.n[2]]].p;
                if (crosses(pa, pb) || crosses(pb, pc) || crosses(pc, pa)) return true;
                for (const vec3& q : {pa, pb, pc})
                    if (std::fabs(side(q)) < 0.01f && in_wedge(q - m * side(q))) return true;
                return false;
            };
            std::vector<uint32_t> crossed;
            for (uint32_t ti = 0; ti < (uint32_t)b.fem.tris.size(); ti++)
                if (!b.fem.tris[ti].broken && tri_crosses(ti)) crossed.push_back(ti);
            // (refined there first, their new nodes on the cut's plane: it parts along a fine, straight line)
            if (!crossed.empty()) {
                const std::function<float(vec3)> side_fn = side;
                // (two levels past the sections' depth; one under a sheet - halved with the plates, the sheet's finest
                // level is the body's step: at the third, the Shell Car's whole skin stepped four times a substep, its
                // physics 5.7 -> 10.3 ms a frame after the axe; at the second twice, as after any bisection)
                crossed = b.fem.refine_cut(b, crossed, side_fn, tri_crosses, b.shells.empty() ? 2 : 1);
            }
            if (!crossed.empty()) tears += b.fem.part_tris(b, crossed, side);
            // the members meeting at a frame node on the plane from its two sides: those on its positive side torn off it
            for (uint32_t fn = 0; fn < (uint32_t)b.fem.node.size(); fn++) {
                const vec3 p = b.nodes[b.fem.node[fn]].p;
                if (!on_plane(p)) continue;
                std::vector<std::pair<uint32_t, int>> pos;
                bool neg = false;
                for (uint32_t ei = 0; ei < (uint32_t)b.fem.elems.size(); ei++) {
                    const FrameElement& e = b.fem.elems[ei];
                    if (e.broken || (e.a != fn && e.b != fn) || e.a == e.b) continue;
                    const float sm = side((b.nodes[b.fem.node[e.a]].p + b.nodes[b.fem.node[e.b]].p) * 0.5f);
                    if (sm >= 0) pos.push_back({ei, e.a == fn ? 0 : 1});
                    else neg = true;
                }
                if (neg)
                    for (const auto& [ei, end] : pos) tears += b.fem.tear(b, ei, end) ? 1 : 0;
            }
            // the mounts reaching across the cut (a part held on the other side): let go
            for (FrameMount& mt : b.fem.mounts) {
                if (mt.broken) continue;
                bool across = crosses(b.nodes[mt.a].p, b.nodes[mt.b].p);
                for (int k = 0; k < mt.nb && !across; k++) across = crosses(b.nodes[mt.a].p, b.nodes[mt.bn[k]].p);
                if (across) mt.broken = true, b.fem.mounts_broken++, b.stats.broken_beams++, tears++;
            }
            // the kerf (a blade's): the frame nodes the cut left on its plane (its new ones there and their copies) stand
            // off it on their own side - their elements' - by 2 mm, so that each face of the blade's wedge (its edge 2 mm
            // thick) meets its own half's: on the plane, inside the edge, the two faces pushed them back and forth in turn,
            // a sheet's cut edge flew apart at 150 m/s
            if (kerf > 0 && tears > 0) {
                const float off = std::min(kerf, 0.002f);
                std::vector<float> lean(b.fem.node.size(), 0.0f);
                for (const FrameTri& t : b.fem.tris) {
                    if (t.broken) continue;
                    const float sc = side((b.nodes[b.fem.node[t.n[0]]].p + b.nodes[b.fem.node[t.n[1]]].p + b.nodes[b.fem.node[t.n[2]]].p) * (1.0f / 3.0f));
                    for (int q = 0; q < 3; q++) lean[t.n[q]] += sc;
                }
                for (const FrameElement& e : b.fem.elems) {
                    if (e.broken) continue;
                    const float sm = side((b.nodes[b.fem.node[e.a]].p + b.nodes[b.fem.node[e.b]].p) * 0.5f);
                    lean[e.a] += sm, lean[e.b] += sm;
                }
                for (uint32_t fn = 0; fn < (uint32_t)b.fem.node.size(); fn++) {
                    Node& x = b.nodes[b.fem.node[fn]];
                    const float s = side(x.p);
                    if (std::fabs(s) >= off || lean[fn] == 0 || x.inv_mass <= 0 || !in_wedge(x.p - m * s)) continue;
                    const vec3 d = m * ((lean[fn] > 0 ? off : -off) - s);
                    x.p += d;
                    if (b.fem.xd.size() >= 3 * (size_t)fn + 3) b.fem.xd[3 * fn] += d.x, b.fem.xd[3 * fn + 1] += d.y, b.fem.xd[3 * fn + 2] += d.z;
                }
            }
            b.fem.finish_cuts(b, tears);
            cut += tears;
            if (getenv("BL_FRAMEDBG")) printf("laser: %s: %zu members crossed, %d torn\n", b.name.c_str(), hits.size(), tears);
        }
        // the collision triangles across the cut: the sheet's go with its shells, the others (a frame's hull) are torn
        const std::vector<char> of_shell = b.shell_tri_mask();
        for (size_t ti = 0; ti < b.tris.size(); ti++) {
            Triangle& t = b.tris[ti];
            if (t.torn || of_shell[ti]) continue;
            const vec3 pa = b.nodes[t.a].p, pb = b.nodes[t.b].p, pc = b.nodes[t.c].p;
            if (!crosses(pa, pb) && !crosses(pb, pc) && !crosses(pc, pa)) continue;
            t.torn = true;
            cut++;
        }
        if (!b.shells.empty()) cut += b.cut_shells(o, d0, d1, range);
        // the welds near the cut again, the frame and the sheet cut: a weld whose sheet's nodes went with one side and its
        // anchor with the other (a node on the cut split, the weld kept the other side's copy, both where they were) holds
        // the halves together - the Frame Car stayed whole on its panels' welds. Each end's side: its elements' (the
        // anchor's members and triangles, the sheet node's shells), not its place on the plane.
        if (cut != before && !b.welds.empty() && !b.fem.empty() && b.node_shells.size() == b.nodes.size()) {
            std::vector<float> lean(b.fem.node.size(), 0.0f);
            for (const FrameTri& t : b.fem.tris) {
                if (t.broken) continue;
                const float sc = side((b.nodes[b.fem.node[t.n[0]]].p + b.nodes[b.fem.node[t.n[1]]].p + b.nodes[b.fem.node[t.n[2]]].p) * (1.0f / 3.0f));
                for (int q = 0; q < 3; q++) lean[t.n[q]] += sc;
            }
            for (const FrameElement& e : b.fem.elems)
                if (!e.broken) {
                    const float sm = side((b.nodes[b.fem.node[e.a]].p + b.nodes[b.fem.node[e.b]].p) * 0.5f);
                    lean[e.a] += sm, lean[e.b] += sm;
                }
            auto frame_lean = [&](uint32_t n) {
                const int sl = b.fem.slot(n);
                return sl >= 0 && sl < (int)lean.size() ? lean[sl] : side(b.nodes[n].p);
            };
            auto sheet_lean = [&](uint32_t n) {
                float l = 0;
                for (uint32_t si : b.node_shells[n]) {
                    const Shell& sh = b.shells[si];
                    l += side((b.nodes[sh.n[0]].p + b.nodes[sh.n[1]].p + b.nodes[sh.n[2]].p) * (1.0f / 3.0f));
                }
                return l;
            };
            for (Weld& wd : b.welds) {
                if (wd.broken) continue;
                const vec3 ap = b.nodes[wd.anchor].p * (1 - wd.t) + b.nodes[wd.anchor2].p * wd.t;
                if (std::fabs(side(ap)) > 0.3f || !in_wedge(ap - m * side(ap))) continue; // (a weld reaches a grid's step)
                const float la = frame_lean(wd.anchor) + (wd.t > 0 ? frame_lean(wd.anchor2) : 0.0f);
                for (uint32_t i = wd.first; i < wd.first + wd.count && !wd.broken; i++)
                    if (la * sheet_lean(b.weld_nodes[i]) < 0) wd.broken = true, b.stats.broken_welds++, cut++;
            }
        }
        if (cut != before) b.wake();
    }
    return cut;
}

static bool ray_hits_aabb(vec3 o, vec3 d, const AABB& a, float max_t) {
    float t0 = 0, t1 = max_t;
    for (int k = 0; k < 3; k++) {
        if (std::fabs(d[k]) < 1e-9f) {
            if (o[k] < a.mn[k] || o[k] > a.mx[k]) return false;
            continue;
        }
        float ta = (a.mn[k] - o[k]) / d[k], tb = (a.mx[k] - o[k]) / d[k];
        if (ta > tb) std::swap(ta, tb);
        t0 = std::max(t0, ta);
        t1 = std::min(t1, tb);
        if (t0 > t1) return false;
    }
    return true;
}

RayHit World::raycast_bodies(vec3 o, vec3 d, float max_t, float beam_radius) const {
    RayHit best;
    float best_t = max_t;
    for (auto& bp : m_bodies) {
        const SoftBody& b = *bp;
        AABB box = b.aabb;
        box.expand(beam_radius);
        if (!ray_hits_aabb(o, d, box, best_t)) continue;
        float hit_t = best_t;
        // triangles (Moller-Trumbore, two-sided)
        for (const Triangle& tr : b.tris) {
            if (tr.torn) continue;
            vec3 p0 = b.nodes[tr.a].p, e1 = b.nodes[tr.b].p - p0, e2 = b.nodes[tr.c].p - p0;
            vec3 pv = cross(d, e2);
            float det = dot(e1, pv);
            if (std::fabs(det) < 1e-12f) continue;
            float inv = 1.0f / det;
            vec3 tv = o - p0;
            float u = dot(tv, pv) * inv;
            if (u < 0 || u > 1) continue;
            vec3 qv = cross(tv, e1);
            float v = dot(d, qv) * inv;
            if (v < 0 || u + v > 1) continue;
            float t = dot(e2, qv) * inv;
            if (t > 0 && t < hit_t) hit_t = t;
        }
        // beams: closest approach of the ray to the segment
        for (const Beam& bm : b.beams) {
            if (bm.flags & BF_BROKEN) continue;
            vec3 a = b.nodes[bm.a].p, ab = b.nodes[bm.b].p - a;
            vec3 w = o - a;
            float bb = dot(d, ab), cc = dot(ab, ab), dd = dot(d, w), ee = dot(ab, w);
            float den = cc - bb * bb; // |d| = 1
            if (cc < 1e-8f) continue;
            float s = den > 1e-9f ? clampf((ee - bb * dd) / den, 0.0f, 1.0f) : 0.0f; // along the beam
            float t = std::max(0.0f, dot(a + ab * s - o, d));
            if (t >= hit_t) continue;
            if (length2(o + d * t - (a + ab * s)) < beam_radius * beam_radius) hit_t = t;
        }
        if (hit_t < best_t) {
            best_t = hit_t;
            best.body = const_cast<SoftBody*>(&b);
            best.t = hit_t;
        }
    }
    if (best.body) {
        vec3 hp = o + d * best.t;
        float bd = 1e30f;
        for (int i = 0; i < (int)best.body->nodes.size(); i++) {
            float l2 = length2(best.body->nodes[i].p - hp);
            if (l2 < bd) {
                bd = l2;
                best.node = i;
            }
        }
    }
    return best;
}

RayHit World::pick_node(vec3 o, vec3 d, float max_t, float radius) const {
    RayHit best;
    float best_score = 1e30f;
    for (auto& bp : m_bodies) {
        const SoftBody& b = *bp;
        // quick AABB reject
        AABB a = b.aabb;
        a.expand(radius);
        float t0 = 0, t1 = max_t;
        bool miss = false;
        for (int k = 0; k < 3 && !miss; k++) {
            if (std::fabs(d[k]) < 1e-9f) {
                if (o[k] < a.mn[k] || o[k] > a.mx[k]) miss = true;
                continue;
            }
            float ta = (a.mn[k] - o[k]) / d[k], tb = (a.mx[k] - o[k]) / d[k];
            if (ta > tb) std::swap(ta, tb);
            t0 = std::max(t0, ta);
            t1 = std::min(t1, tb);
            if (t0 > t1) miss = true;
        }
        if (miss) continue;
        for (int i = 0; i < (int)b.nodes.size(); i++) {
            if (b.info[i].flags & NF_NO_MOUSE) continue;
            if (b.nodes[i].inv_mass <= 0) continue;
            vec3 rel = b.nodes[i].p - o;
            float t = dot(rel, d);
            if (t < 0 || t > max_t) continue;
            float dist = length(rel - d * t);
            float rr = radius * std::max(1.0f, t * 0.02f);
            if (dist > rr) continue;
            float score = t + dist * 20.0f;
            if (score < best_score) {
                best_score = score;
                best.body = const_cast<SoftBody*>(&b);
                best.node = i;
                best.t = t;
            }
        }
    }
    return best;
}

} // namespace bl::phys
