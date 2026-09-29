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
    std::vector<NT> nt;
    std::vector<NC> nc;
    std::vector<CT> ct;
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
    };
    std::vector<Pt> pts;
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
    std::vector<AABB> boxes;                 // per body, expanded by the contact reach
    std::vector<std::vector<int>> partners;  // per body: bodies whose boxes overlap

    // ---- parallel phases (simulate_island): work items, their results; every result has a slot of its own and is
    // merged in item order, so the outcome does not depend on the threads
    struct Work {
        uint32_t body;
        uint32_t kind;  // 0: internal forces of the body (beams, shocks, wheels ...), 1: chunk `a` of its triangles, 2: nodes [a, b),
                        // 3: a rigid body's step, 4: chunk `a` of its frame's members
        uint32_t a, b;
    };
    std::vector<Work> work;
    std::vector<char> fem_parts;    // (bodies whose frame members are evaluated in chunks of their own this short step)
    std::vector<char> fem_defer;    // (integration items whose frame solves this short step, apart: then they integrate)
    std::vector<std::pair<uint32_t, int>> fem_work;   // (those frames' components: the item, the component)
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
    // average frame): then a frame may take at most 1.25x the substeps of an average frame (+1), and after a hitch the
    // simulation runs slow for a moment instead of doubling the next frame's work (which makes it slow too ...). Frames
    // that are long for other reasons (uneven presentation, the window system) are always simulated in full: dropping
    // their time made the motion jerk.
    const double want = (double)frame_dt * settings.time_scale;
    m_avg_frame = m_avg_frame <= 0 ? std::clamp(want, 1.0 / 240.0, 1.0 / 15.0) : m_avg_frame * 0.95 + 0.05 * std::clamp(want, 1.0 / 240.0, 1.0 / 15.0);
    m_accum += want;
    int n = (int)(m_accum / settings.dt);
    const bool overloaded = m_stats.step_ms > 0.5 * m_avg_frame * 1000.0;
    const int maxn = overloaded ? std::min(settings.max_substeps_per_frame, (int)std::ceil(1.25 * m_avg_frame / settings.dt) + 1)
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

void World::step_substeps(int n) {
    PROFILE_ZONE("Physics");
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
            const vec3 f = primitive_collision(F[i], x.v, x.mass, c.normal, dt, gm, c.depth, inf[i].friction * fric_body, b.contact_push_max, b.contact_slop);
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

void World::rebuild_pairs(Island& isl) {
    PROFILE_ACCUM("Broadphase");
    m_rebuilds.fetch_add(1, std::memory_order_relaxed);
    static const bool bp_dbg = getenv("BL_BPDBG") != nullptr; // slow rebuilds -> stderr
    const double bp_t0 = bp_dbg ? time_seconds() : 0.0;
    double bp_t[8] = {};
    int bp_k = 0;
    auto bp_lap = [&]() {
        if (bp_dbg && bp_k < 8) bp_t[bp_k++] = time_seconds();
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
    // Adaptive rebuild interval (Verlet-list style): the candidate margin must cover the relative motion
    // until the next rebuild. The interval follows the slow bodies (piles at rest rebuild rarely); a fast body
    // (projectile, a car at speed) gets its own extra margin and is paired separately, so one bullet flying into a
    // pile does not make the whole pile rebuild every substep.
    constexpr float kFastSpeed = 8.0f;
    float slow_speed = 0;
    for (SoftBody* b : isl.bodies)
        if (b->max_speed <= kFastSpeed) slow_speed = std::max(slow_speed, b->max_speed);
    (void)top_speed;
    const float speed = slow_speed + 0.5f;
    isl.slow_speed = speed;
    const float margin_budget = 0.08f;
    int interval = (int)(margin_budget / (2.0f * speed * isl.dt));
    interval = std::max(1, std::min(settings.pair_rebuild_interval, interval));
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
            Island::Pt pt{(uint16_t)bi, ni, cell_of(p.x), cell_of(p.y), cell_of(p.z)};
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
        constexpr uint32_t kTriChunk = 512;
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
    isl.fast_interval = interval;
    isl.fast_dense = false;
    if (any_fast) {
        isl.fast_interval = std::max(1, std::min(interval, (int)(0.06f / (fast_speed * isl.dt))));
        isl.fast_dense = isl.fast_interval < interval;
        fast_pairs(isl);
    }
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
    isl.pair_count = (int)(isl.nt.size() + isl.nc.size() + isl.ct.size());
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
            fprintf(stderr, "BP %.2f ms: bodies %zu pts %zu tris %zu caps %zu entries %zu -> nt %zu nc %zu ct %zu teamed %d interval %d\n", ms,
                    isl.bodies.size(), isl.pts.size(), ntri, ncap, isl.entries.size(), isl.nt.size(), isl.nc.size(), isl.ct.size(), (int)isl.teamed,
                    isl.rebuild_interval);
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
    constexpr uint32_t kTriChunk = 64;
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
    if (L.tris.empty() && L.nodes.empty()) return;
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
    static thread_local std::vector<Island::NT> nt_new;
    static thread_local std::vector<Island::NC> nc_new;
    static thread_local std::vector<Island::CT> ct_new;
    auto nt_pass = [&](size_t b0, size_t b1) {
        nt_new.clear();
        for (size_t i = b0; i < b1; i++) {
            const Island::NT p = isl.nt[i];
            if (p.bn == k && p.node >= nmin && p.node <= nmax)
                children(nsrc, p.node, [&](uint32_t c) { nt_new.push_back({p.bn, p.bt, c, p.tri}); });
            if (p.bt == k && p.tri >= tmin && p.tri <= tmax)
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
            if (p.bn == k && p.node >= nmin && p.node <= nmax)
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
        if (p.bt == k && p.tri >= tmin && p.tri <= tmax) children(tsrc, p.tri, [&](uint32_t c) { ct_new.push_back({p.bc, p.bt, p.cap, c}); });
    isl.ct.insert(isl.ct.end(), ct_new.begin(), ct_new.end());
    isl.pair_count = (int)(isl.nt.size() + isl.nc.size() + isl.ct.size());
}

// Contacts of the candidate pairs: the narrow phase (closest points of the current positions: independent per pair,
// in chunks shared by the team), then the response in pair order (serial: each contact cancels the forces of the
// ones before it, see contact_force).
void World::collide_pairs(Island& isl) {
    const size_t nnt = isl.nt.size(), nnc = isl.nc.size(), nct = isl.ct.size();
    m_narrow.fetch_add((long long)(nnt + nnc + nct), std::memory_order_relaxed);
    constexpr uint32_t kPairChunk = 1024;
    const int cnt = (int)((nnt + kPairChunk - 1) / kPairChunk), cnc = (int)((nnc + kPairChunk - 1) / kPairChunk),
              cct = (int)((nct + kPairChunk - 1) / kPairChunk);
    const int nch = cnt + cnc + cct;
    if (nch == 0) return;
    if ((int)isl.hits.size() < nch) isl.hits.resize(nch);
    {
        isl.team->run(nch, [&](int c) {
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
                    const float r = bn.collision_radius;
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
                        if (s < 0) continue; // (behind, not over this face: another face's)
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
        });
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
                float fric = (bn.info[pr.node].flags & NF_TYRE) ? bn.info[pr.node].friction : 0.8f;
                const vec3* SA[1] = {snap(bn, pr.node)};
                const vec3* SB[3] = {snap(bt, t.a), snap(bt, t.b), snap(bt, t.c)};
                if (!t.two_sided && h.pen > bn.collision_radius) { // (behind a hull's face)
                    if (hull_push_out<3>(A[0], FA[0], B, FB, wb, ma, mb, h.nrm, h.pen, dt, SA[0], SB[0] ? SB : nullptr)) contacts++, bn.touch(bt), bt.touch(bn);
                    continue;
                }
                if (contact_force<1, 3>(A, FA, wa, B, FB, wb, ma, mb, h.nrm, h.pen, dt, gm, fric, SA[0] ? SA : nullptr, SB[0] ? SB : nullptr)) {
                    contacts++, bn.touch(bt), bt.touch(bn);
                    // a fast contact on a sheet with a fracture pattern: where it lays one (the approach speed)
                    if (bt.shell_mat.pattern != ShellPattern::None || bn.shell_mat.pattern != ShellPattern::None) {
                        const float vn = std::fabs(dot(A[0]->v - (B[0]->v * wb[0] + B[1]->v * wb[1] + B[2]->v * wb[2]), h.nrm));
                        auto radius = [](const SoftBody& x) { return 0.5f * maxc(x.aabb.mx - x.aabb.mn); };
                        bt.pattern_contact(t.a, t.b, t.c, h.bary, vn, radius(bn), m_time);
                        bn.pattern_contact(pr.node, pr.node, pr.node, vec3(1, 0, 0), vn, radius(bt), m_time);
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
                if (contact_force<1, 2>(A, FA, wa, B, FB, wb, ma, mb, h.nrm, h.pen, dt, gm, 0.8f, SA[0] ? SA : nullptr, SB[0] ? SB : nullptr))
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
                if (contact_force<2, 3>(A, FA, wa, B, FB, wb, ma, mb, h.nrm, h.pen, dt, gm, 0.8f, SA[0] ? SA : nullptr, SB[0] ? SB : nullptr)) {
                    contacts++, bc.touch(bt), bt.touch(bc);
                    if (bt.shell_mat.pattern != ShellPattern::None) {
                        const vec3 va = A[0]->v * wa[0] + A[1]->v * wa[1];
                        const float vn = std::fabs(dot(va - (B[0]->v * wb[0] + B[1]->v * wb[1] + B[2]->v * wb[2]), h.nrm));
                        bt.pattern_contact(tri.a, tri.b, tri.c, h.bary, vn, 0.5f * maxc(bc.aabb.mx - bc.aabb.mn), m_time);
                    }
                }
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
    isl.team = &team;
    for (SoftBody* b : isl.bodies) {
        b->static_contacts = 0;
        b->sphere_touches = 0;
        b->body_contacts = 0;
        b->touched.clear();
        if (b->energy_guard && !b->rigid) b->motion_energy(settings.gravity, b->guard_ke, b->guard_pe);
        b->topo_log.clear();
    }
    isl.dt = dt;
    isl.subs.assign(nb, 0);
    constexpr uint32_t kNodeChunk = 1024;
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
        const bool fem_step = !b.fem.empty() && (controller || b.fem_every_step) && !b.rigid;
        if (!b.welds.empty()) b.compute_weld_forces(fem_step);
        if (fem_step && !fem_parts) { // (once per substep: the frame's step, FemFrame::solve; fem_parts: in chunks of their own)
            PROFILE_ACCUM("Frame elements");
            b.fem.compute_forces(b);
        }
        if (!b.wheels.empty()) b.compute_wheel_forces(bdt, controller);
        if (!b.slides.empty()) b.compute_slide_forces();
    };
    // node ranges of the bodies stepping at short step j (a body with wheels stays whole: its tyre patches share grip)
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
    for (int s = 0; s < substeps; s++) {
        int maxsub = 0;
        for (int k = 0; k < nb; k++) {
            SoftBody& b = *isl.bodies[k];
            if (b.wake_request) {
                b.wake_request = false;
                b.wake();
            }
            isl.subs[k] = b.sleeping ? 0 : 1 << b.dt_shift();
            maxsub = std::max(maxsub, isl.subs[k]);
            isl.node_steps += (long long)b.nodes.size() * isl.subs[k];
            isl.beam_steps += (long long)b.beams.size() * isl.subs[k];
        }
        for (int j = 0; j < maxsub; j++) {
            // 1) internal forces (a frame's members in chunks beside the body's other forces and its sheet: one after
            // another they were the longest item of the step)
            isl.work.clear();
            isl.fem_parts.assign(nb, 0);
            for (int k = 0; k < nb; k++) {
                if (isl.subs[k] <= j) continue;
                SoftBody& b = *isl.bodies[k];
                isl.work.push_back({(uint32_t)k, 0, 0, 0});
                if (!b.fem.empty() && (j == 0 || b.fem_every_step) && !b.rigid) {
                    const int fc = b.fem.begin_forces(b);
                    for (int c = 0; c < fc; c++) isl.work.push_back({(uint32_t)k, 4, (uint32_t)c, 0});
                    isl.fem_parts[k] = 1;
                }
                if (!b.shells.empty() && !b.rigid) {
                    const int ch = b.shell_begin(dt / (float)isl.subs[k], j, isl.subs[k]);
                    for (int c = 0; c < ch; c++) isl.work.push_back({(uint32_t)k, 1, (uint32_t)c, 0});
                }
            }
            {
                PROFILE_ACCUM("Forces");
                team.run((int)isl.work.size(), [&](int i) {
                    const Island::Work& w = isl.work[i];
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
                for (int k = 0; k < nb; k++)
                    if (isl.fem_parts[k]) isl.bodies[k]->fem.end_forces(*isl.bodies[k]);
                for (int k = 0; k < nb; k++)
                    if (isl.subs[k] > j && isl.bodies[k]->shk.pass.chunks > 0) isl.bodies[k]->shell_end();
                lap(4);
            }
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
                // 3) inter-body contacts (forces)
                if (isl.needs_pairs) {
                    PROFILE_ACCUM("Collisions");
                    const uint64_t c0 = prof::now();
                    if (s == 0 || s >= isl.next_rebuild || rebuild_now) {
                        rebuild_pairs(isl);
                        isl.next_rebuild = s + isl.rebuild_interval;
                        isl.next_fast = s + isl.fast_interval;
                        rebuild_now = false;
                        isl.ph_ms[6] += prof::ticks_to_ms(prof::now() - c0);
                    } else if (isl.fast_interval > 0 && isl.fast_interval < isl.rebuild_interval && s >= isl.next_fast) {
                        refresh_fast_pairs(isl);
                        isl.next_fast = s + isl.fast_interval;
                        isl.ph_ms[7] += prof::ticks_to_ms(prof::now() - c0);
                    }
                    const uint64_t c1 = prof::now();
                    collide_pairs(isl);
                    isl.ph_ms[8] += prof::ticks_to_ms(prof::now() - c1);
                }
                lap(2);
            }
            // 4) static contacts + integration (the sheets' forces of the later short steps are gathered here)
            node_work(j, false);
            isl.fem_defer.assign(isl.work.size(), 0);
            {
                PROFILE_ACCUM("Integration");
                team.run((int)isl.work.size(), [&](int i) {
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
                    }
                    PROFILE_ACCUM("Integrate");
                    out.mn = vec3(1e30f);
                    out.mx = vec3(-1e30f);
                    out.max_v2 = 0;
                    isl.fem_defer[i] = 0;
                    if (w.kind == 3) {
                        b.rigid_step(bdt, out.contacts > 0, out.mn, out.mx, out.max_v2);
                    } else {
                        if (!b.fem.empty()) {
                            // every force on the frame nodes is in: their implicit step (the new velocities as forces),
                            // once per substep, next, a component each; the short steps of a sub-cycled body hold them
                            // at those velocities
                            if (b.fem_every_step || j == 0) {
                                isl.fem_defer[i] = 1;
                                return;
                            }
                            b.fem.hold(b, bdt);
                        }
                        b.integrate_nodes(w.a, w.b, bdt, out.mn, out.mx, out.max_v2);
                    }
                });
                // the frames' implicit steps: each frame's components (parts held by mounts, solved apart) beside each
                // other and the other frames'; then those bodies' integration
                isl.fem_work.clear();
                for (int i = 0; i < (int)isl.work.size(); i++) {
                    if (!isl.fem_defer[i]) continue;
                    SoftBody& b = *isl.bodies[isl.work[i].body];
                    const float bdt = dt / (float)isl.subs[isl.work[i].body];
                    const int nc = b.fem.solve_begin(b, b.fem_every_step ? bdt : dt, bdt, settings.frame_theta,
                                                     b.fem_dissipation >= 0 ? b.fem_dissipation : settings.frame_dissipation);
                    for (int c = 0; c < nc; c++) isl.fem_work.push_back({(uint32_t)i, c});
                }
                if (!isl.fem_work.empty()) {
                    team.run((int)isl.fem_work.size(), [&](int q) {
                        PROFILE_ACCUM("Frame solve");
                        SoftBody& b = *isl.bodies[isl.work[isl.fem_work[q].first].body];
                        b.fem.solve_component(b, isl.fem_work[q].second);
                    });
                    for (int i = 0; i < (int)isl.work.size(); i++)
                        if (isl.fem_defer[i]) isl.bodies[isl.work[i].body]->fem.solve_end(*isl.bodies[isl.work[i].body]);
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
            // 5) per body: bounds, top speed, frames; then the sheets' refinement and cracks
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
                    b.fem.sync_positions(b, dt / (float)isl.subs[k]);
                    // the frame's splits and tears (new nodes: their contact pairs are inherited below)
                    if (b.fem.pending() && b.fem.process_events(b)) b.topo_changed = true, b.topo_version++, b.shk.version++;
                }
                b.integrate_frames(dt / (float)isl.subs[k]);
                if (!b.rigid && (!b.shell_events.empty() || b.shell_hit.speed > 0 || b.pattern_passes > 0)) {
                    lap(4);
                    b.process_shell_events();
                    lap(5);
                }
            }
            lap(4);
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

int World::laser_cut(vec3 o, vec3 d0, vec3 d1, float range, const SoftBody* skip) {
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
        for (Beam& bm : b.beams) {
            if ((bm.flags & BF_BROKEN) || !crosses(b.nodes[bm.a].p, b.nodes[bm.b].p)) continue;
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
        if (!b.fem.empty() && !b.rigid) {
            // frame members: cut where they cross the swept plane (the ones hit first, then the cuts: a split adds members)
            std::vector<std::pair<uint32_t, float>> hits;
            for (size_t ei = 0; ei < b.fem.elems.size(); ei++) {
                const FrameElement& e = b.fem.elems[ei];
                if (e.broken) continue;
                const vec3 pa = b.nodes[b.fem.node[e.a]].p, pc = b.nodes[b.fem.node[e.b]].p;
                const float sa = side(pa), sc = side(pc);
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
