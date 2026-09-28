#include "phys/cache_sim.h"
#include "core/util.h"
#include "phys/shell_util.h"

#include <algorithm>
#include <bit>

namespace bl::phys {

using namespace shell_detail;

CacheSim::CacheSim(Level l1, Level l2, int line) : m_line(line) {
    m_shift = 0;
    while ((1 << m_shift) < line) m_shift++;
    m_l1.ways = l1.ways;
    m_l1.sets = (int)(l1.size / line / l1.ways);
    m_l1.w.assign((size_t)m_l1.sets * m_l1.ways, Way{});
    m_l2.ways = l2.ways;
    m_l2.sets = (int)(l2.size / line / l2.ways);
    m_l2.w.assign((size_t)m_l2.sets * m_l2.ways, Way{});
}

void CacheSim::mark(Way& w, uint32_t off, uint32_t len) {
    for (uint32_t b = off; b < off + len && b < 128; b++) w.used[b >> 6] |= 1ull << (b & 63);
}

bool CacheSim::lookup(Cache& c, uint64_t la, bool l1, uint32_t off, uint32_t len) {
    Way* set = &c.w[(size_t)(la % (uint64_t)c.sets) * c.ways];
    m_clock++;
    for (int i = 0; i < c.ways; i++)
        if (set[i].tag == la) {
            set[i].lru = m_clock;
            if (l1) mark(set[i], off, len);
            return true;
        }
    // miss: replace the least recently used way
    Way* v = set;
    for (int i = 1; i < c.ways; i++)
        if (set[i].lru < v->lru) v = &set[i];
    if (l1 && v->tag != ~0ull) {
        m_stats.lines_evicted++;
        m_stats.bytes_used_evicted += (uint64_t)(std::popcount(v->used[0]) + std::popcount(v->used[1]));
    }
    v->tag = la;
    v->lru = m_clock;
    v->used[0] = v->used[1] = 0;
    if (l1) mark(*v, off, len);
    return false;
}

void CacheSim::access(const void* p, size_t bytes) {
    if (!bytes) return;
    m_stats.accesses++;
    m_stats.bytes_touched += bytes;
    uint64_t a = (uint64_t)(uintptr_t)p;
    const uint64_t end = a + bytes;
    while (a < end) {
        const uint64_t la = a >> m_shift;
        const uint32_t off = (uint32_t)(a & (uint64_t)(m_line - 1));
        const uint32_t len = (uint32_t)std::min<uint64_t>(end - a, (uint64_t)m_line - off);
        if (lookup(m_l1, la, true, off, len)) {
            m_stats.l1_hits++;
        } else {
            m_stats.l1_misses++;
            if (lookup(m_l2, la, false, 0, 0)) m_stats.l2_hits++;
            else m_stats.l2_misses++;
        }
        a += len;
    }
}

void CacheSim::flush() {
    for (Way& w : m_l1.w)
        if (w.tag != ~0ull) {
            m_stats.lines_evicted++;
            m_stats.bytes_used_evicted += (uint64_t)(std::popcount(w.used[0]) + std::popcount(w.used[1]));
            w.used[0] = w.used[1] = 0;
        }
}

namespace {

// ---- access stream of the pre-optimisation kernel (tests/reference_shell.cpp): Shell records, force arrays per rate
// class, an edge loop and a hinge loop over the shells, then the class arrays added to the forces
void trace_reference(const SoftBody& b, CacheSim& C, std::vector<vec3>& acc, std::vector<vec3>& force) {
    const Node* nd = b.nodes.data();
    const size_t nn = b.nodes.size();
    const Shell* SH = b.shells.data();
    const int ns = (int)b.shells.size();
    for (size_t i = 0; i < nn; i++) C.access(&nd[i].v, 16); // mean velocity (v, mass)
    for (size_t i = 0; i < nn; i++) C.access(&acc[i], 12);  // acc.assign(nn, 0)
    for (int si = 0; si < ns; si++) {
        const Shell& s = SH[si];
        C.access(&s.level, 1);
        for (int e = 0; e < 3; e++) {
            const uint32_t a = s.n[e], bb = s.n[nx(e)];
            C.access(&s.n[e], 4);
            C.access(&s.n[nx(e)], 4);
            C.access(&nd[a].p, 12);
            C.access(&nd[bb].p, 12);
            C.access(&s.k[e], 4);
            C.access(&s.d[e], 4);
            C.access(&s.L0[e], 4);
            C.access(&s.flaw, 4);
            C.access(&s.L[e], 4);
            C.access(&nd[a].v, 12);
            C.access(&nd[bb].v, 12);
            C.access(&acc[a], 12);
            C.access(&acc[bb], 12);
        }
        C.access(&s.strain, 4);
        for (int c = 0; c < 3; c++) C.access(&nd[s.n[c]].p, 12);
        for (int c = 0; c < 3; c++) C.access(&nd[s.n[c]].v, 12);
        C.access(&s.mass, 4);
        for (int c = 0; c < 3; c++) C.access(&acc[s.n[c]], 12);
        for (int e = 0; e < 3; e++) {
            C.access(&s.nb[e], 4);
            if (s.nb[e] < 0) {
                C.access(&b.info[s.n[e]].flags, 2);
                C.access(&b.info[s.n[nx(e)]].flags, 2);
            }
        }
        C.access(&s.cool, 1);
        C.access(&s.pending, 1);
    }
    for (int si = 0; si < ns; si++) {
        const Shell& s = SH[si];
        for (int e = 0; e < 3; e++) {
            C.access(&s.nb[e], 4);
            const int j = s.nb[e];
            if (j <= si) continue;
            const Shell& t = SH[j];
            C.access(&s.level, 1);
            C.access(&t.level, 1);
            C.access(&t.n[0], 12); // edge_of
            const int fe = edge_of(t, s.n[e], s.n[nx(e)]);
            if (fe < 0) continue;
            const uint32_t id[4] = {s.n[pv(e)], t.n[pv(fe)], s.n[e], s.n[nx(e)]};
            for (uint32_t k : id) C.access(&nd[k].p, 12);
            C.access(&s.th0[e], 4);
            C.access(&s.pending, 1);
            C.access(&t.pending, 1);
            C.access(&s.flaw, 4);
            C.access(&t.flaw, 4);
            C.access(&s.L0[e], 4);
            C.access(&s.area0, 4);
            C.access(&t.area0, 4);
            C.access(&s.kb, 4);
            C.access(&t.kb, 4);
            for (uint32_t k : id) C.access(&nd[k].v, 12);
            for (uint32_t k : id) C.access(&acc[k], 12);
        }
    }
    for (size_t i = 0; i < nn; i++) {
        C.access(&acc[i], 12);
        C.access(&force[i], 12);
    }
}

// ---- access stream of the current kernel (shell_kernel.cpp): per triangle its state and hot records, its corners,
// the hinges it evaluates (record + far wing), its slots; then per node its gather list, its slots and its force
void trace_current(const SoftBody& b, CacheSim& C, std::vector<vec3>& force) {
    const ShellKernel& K = b.shk;
    const Node* nd = b.nodes.data();
    const size_t nn = b.nodes.size();
    const int ns = (int)b.shells.size();
    for (size_t i = 0; i < nn; i++) C.access(&nd[i].v, 16); // mean velocity (v, mass)
    for (int bi = 0; bi < (ns + 3) / 4; bi++) {
        const ShellKernel::Hot& h = K.hot[bi];
        for (int l = 0; l < 4 && bi * 4 + l < ns; l++) C.access(&K.aux[bi * 4 + l], sizeof(ShellKernel::Aux));
        C.access(&h, sizeof(h));
        for (int l = 0; l < 4 && bi * 4 + l < ns; l++) {
            const int si = bi * 4 + l;
            const ShellKernel::Aux& ax = K.aux[si];
            for (int c = 0; c < 3; c++) {
                C.access(&nd[h.n[c][l]].p, 16); // (vector loads: 16 bytes)
                C.access(&nd[h.n[c][l]].v, 16);
            }
            if (ax.flags & (ShellKernel::kClamp * 7)) C.access(&b.shells[si], sizeof(Shell));
            for (int e = 0; e < 3; e++) {
                if (h.hinge[e][l] == ShellKernel::kNoHinge) continue;
                const ShellKernel::Hinge& hg = K.hinge[h.hinge[e][l]];
                C.access(&hg, sizeof(hg));
                C.access(&nd[hg.wing].p, 16);
                C.access(&nd[hg.wing].v, 16);
                C.access(&K.slot[(size_t)si * 6 + 3 + e], sizeof(F3));
            }
            C.access(&K.slot[(size_t)si * 6], 3 * sizeof(F3));
        }
    }
    for (size_t v = 0; v < nn && v < K.gcount.size(); v++) {
        const int n = K.gcount[v];
        C.access(&K.gcount[v], 1);
        C.access(&K.gather[v * K.stride], (size_t)n * 4);
        for (int k = 0; k < n; k++) C.access(&K.slot[K.gather[v * K.stride + k]], sizeof(F3));
        C.access(&force[v], 12);
    }
}

} // namespace

KernelCacheReport measure_kernel_cache(SoftBody& b) {
    KernelCacheReport r;
    b.shell_sync();
    r.shells = b.shells.size();
    r.nodes = b.nodes.size();
    for (uint32_t si = 0; si < b.shells.size(); si++)
        for (int e = 0; e < 3; e++) r.hinges += b.shk.hot[si >> 2].hinge[e][si & 3] != ShellKernel::kNoHinge ? 1 : 0;
    std::vector<vec3> acc(b.nodes.size()), force(b.nodes.size());
    {
        CacheSim C;
        trace_current(b, C, force); // (warm-up: the steady state of a simulation)
        C.reset_stats();
        trace_current(b, C, force);
        C.flush();
        r.cur = C.stats();
    }
    {
        CacheSim C;
        trace_reference(b, C, acc, force);
        C.reset_stats();
        trace_reference(b, C, acc, force);
        C.flush();
        r.ref = C.stats();
    }
    const double ns = (double)std::max<size_t>(1, r.shells);
    r.cur_bytes_per_tri = (double)r.cur.bytes_touched / ns;
    r.ref_bytes_per_tri = (double)r.ref.bytes_touched / ns;
    r.cur_l2_bytes_per_tri = (double)r.cur.l1_misses * 128 / ns;
    r.ref_l2_bytes_per_tri = (double)r.ref.l1_misses * 128 / ns;
    r.cur_mem_bytes_per_tri = (double)r.cur.l2_misses * 128 / ns;
    r.ref_mem_bytes_per_tri = (double)r.ref.l2_misses * 128 / ns;
    const ShellKernel& K = b.shk;
    r.working_set_cur = (double)(K.hot.size() * sizeof(ShellKernel::Hot) + K.hinge.size() * sizeof(ShellKernel::Hinge) +
                                 K.aux.size() * sizeof(ShellKernel::Aux) + K.slot.size() * sizeof(F3) + K.gather.size() * 4 + K.gcount.size() +
                                 b.nodes.size() * sizeof(Node) + b.nodes.size() * sizeof(vec3));
    r.working_set_ref = (double)(b.shells.size() * sizeof(Shell) + b.nodes.size() * (sizeof(Node) + 2 * sizeof(vec3)));
    return r;
}

std::string KernelCacheReport::text() const {
    auto row = [&](const char* name, const CacheStats& s, double bpt, double l2bpt, double membpt, double ws) {
        return format("  %-10s L1 hits %5.1f%%, lines from L2 %6.1f B/tri (touched %5.1f B/tri), line utilisation %5.1f%%, L2 hits %5.1f%%, "
                      "from memory %.1f B/tri, working set %.2f MB\n",
                      name, s.l1_hit_rate() * 100.0, l2bpt, bpt, s.line_utilisation(128) * 100.0, s.l2_hit_rate() * 100.0, membpt, ws / 1048576.0);
    };
    return format("cache model (L1D 128 KB 8-way, L2 16 MB 16-way, 128 B lines): %zu triangles, %zu hinges, %zu nodes\n", shells, hinges, nodes) +
           row("current", cur, cur_bytes_per_tri, cur_l2_bytes_per_tri, cur_mem_bytes_per_tri, working_set_cur) +
           row("reference", ref, ref_bytes_per_tri, ref_l2_bytes_per_tri, ref_mem_bytes_per_tri, working_set_ref);
}

} // namespace bl::phys
