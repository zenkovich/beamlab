// Force kernel of the triangle elements (phys::Shell): edge springs with plastic yield, air drag and flex damping on
// the faces, clamped borders and the bending hinges; overloads become events for process_shell_events (shell.cpp).
//
// Data layout (ShellKernel, see softbody.h): blocks of 4 triangles field by field (their corners, springs and rest
// lengths load as vectors), 8 bytes of state per triangle, a 32-byte record per bending hinge; the nodes are read,
// never written: every triangle writes the forces on its corners and on the far wings of its hinges to slots of its
// own and each node gathers its slots (no write conflicts: any number of threads, results independent of them).
// Sheets are kept in a Morton order (SoftBody::reorder_shells): the corners of a block and its neighbours share cache
// lines. The arrays follow the topology incrementally (the records of the triangles and nodes a change touched).
//
// Evaluation (NEON, 4 triangles per iteration): the corners are loaded and transposed once; springs, face and the
// hinges the triangles evaluate (per edge, the lanes owning a hinge across it: three corners are in registers already,
// only the far wing is loaded) accumulate into the same corner vectors; rare work (plastic bends, overload checks,
// clamped borders) runs per lane only where a vector test flags it.
// Hinges: Bridson et al. 2003 (dihedral angle and its gradient); the angle comes from a 256-interval table
// (lut_atan2, linear interpolation), square roots from the reciprocal square root estimate and Newton steps, the
// fourth gradient from the other three (their sum is zero: translation invariance).
#include "core/profiler.h"
#include "phys/shell_util.h"

#include <algorithm>
#include <cmath>

namespace bl::phys {

using namespace shell_detail;
using SK = ShellKernel;

namespace {

inline F3 f3(vec3 v) { return F3{v.x, v.y, v.z}; }

#if defined(__aarch64__)
// ---- 4-lane vectors (NEON): one lane per hinge
struct V3 {
    float32x4_t x, y, z;
};
inline V3 operator-(V3 a, V3 b) { return {vsubq_f32(a.x, b.x), vsubq_f32(a.y, b.y), vsubq_f32(a.z, b.z)}; }
inline V3 operator+(V3 a, V3 b) { return {vaddq_f32(a.x, b.x), vaddq_f32(a.y, b.y), vaddq_f32(a.z, b.z)}; }
inline V3 operator-(V3 a) { return {vnegq_f32(a.x), vnegq_f32(a.y), vnegq_f32(a.z)}; }
inline V3 operator*(V3 a, float32x4_t s) { return {vmulq_f32(a.x, s), vmulq_f32(a.y, s), vmulq_f32(a.z, s)}; }
inline float32x4_t dot4(V3 a, V3 b) { return vfmaq_f32(vfmaq_f32(vmulq_f32(a.x, b.x), a.y, b.y), a.z, b.z); }
inline V3 cross4(V3 a, V3 b) {
    return {vfmsq_f32(vmulq_f32(a.y, b.z), a.z, b.y), vfmsq_f32(vmulq_f32(a.z, b.x), a.x, b.z), vfmsq_f32(vmulq_f32(a.x, b.y), a.y, b.x)};
}
inline float32x4_t rsqrt4(float32x4_t x) {
    float32x4_t e = vrsqrteq_f32(x);
    e = vmulq_f32(e, vrsqrtsq_f32(vmulq_f32(x, e), e));
    e = vmulq_f32(e, vrsqrtsq_f32(vmulq_f32(x, e), e));
    return e;
}
// 4 x (x, y, z, w) rows at 16-byte aligned addresses -> x, y, z columns
inline V3 load4(const float* a, const float* b, const float* c, const float* d) {
    const float32x4x2_t t01 = vtrnq_f32(vld1q_f32(a), vld1q_f32(b)); // a.x b.x a.z b.z | a.y b.y a.w b.w
    const float32x4x2_t t23 = vtrnq_f32(vld1q_f32(c), vld1q_f32(d));
    return {vcombine_f32(vget_low_f32(t01.val[0]), vget_low_f32(t23.val[0])), vcombine_f32(vget_low_f32(t01.val[1]), vget_low_f32(t23.val[1])),
            vcombine_f32(vget_high_f32(t01.val[0]), vget_high_f32(t23.val[0]))};
}
// lut_atan2 per lane (the table lookups are four scalar loads; the rest in vectors)
inline float32x4_t atan2_4(float32x4_t y, float32x4_t x) {
    const float32x4_t ax = vabsq_f32(x), ay = vabsq_f32(y);
    const float32x4_t mx = vmaxq_f32(ax, ay), mn = vminq_f32(ax, ay);
    const float32x4_t a = vdivq_f32(mn, vmaxq_f32(mx, vdupq_n_f32(1e-30f)));
    const float32x4_t q = vmulq_f32(a, vdupq_n_f32(256.0f));
    const int32x4_t i = vcvtq_s32_f32(q);
    const float32x4_t fr = vsubq_f32(q, vcvtq_f32_s32(i));
    const auto& T = atan_table().v;
    const float32x2_t e0 = vld1_f32(T[vgetq_lane_s32(i, 0)]), e1 = vld1_f32(T[vgetq_lane_s32(i, 1)]), e2 = vld1_f32(T[vgetq_lane_s32(i, 2)]),
                      e3 = vld1_f32(T[vgetq_lane_s32(i, 3)]);
    const float32x2x2_t z01 = vzip_f32(e0, e1), z23 = vzip_f32(e2, e3); // (v0 v1 | d0 d1)
    const float32x4_t v = vcombine_f32(z01.val[0], z23.val[0]), dv = vcombine_f32(z01.val[1], z23.val[1]);
    float32x4_t r = vfmaq_f32(v, fr, dv);
    r = vbslq_f32(vcgtq_f32(ay, ax), vsubq_f32(vdupq_n_f32(1.57079637f), r), r);
    r = vbslq_f32(vcltq_f32(x, vdupq_n_f32(0.0f)), vsubq_f32(vdupq_n_f32(3.14159274f), r), r);
    return vbslq_f32(vcltq_f32(y, vdupq_n_f32(0.0f)), vnegq_f32(r), r);
}
inline bool any4(uint32x4_t m) { return vmaxvq_u32(m) != 0; }
#endif

// hot and state records of one shell from the Shell (the plastic state is copied from there, the strain and the
// cooling time stay where they are unless the shell is new); the hinge block is left to sync_hinges
void sync_record(SoftBody& b, uint32_t si, bool fresh) {
    const Shell& s = b.shells[si];
    const ShellMaterial& M = b.shell_material(s);
    SK::Hot& h = b.shk.hot[si >> 2];
    const uint32_t l = si & 3;
    SK::Aux& a = b.shk.aux[si];
    for (int c = 0; c < 3; c++) {
        h.n[c][l] = s.n[c];
        h.k[c][l] = s.k[c];
        h.L[c][l] = s.L[c];
        h.L0[c][l] = s.L0[c];
    }
    h.brk[l] = M.brk * s.flaw;
    h.yl[l] = M.membrane > 0 ? 1e9f : M.yield; // (a projected membrane flows in the projection, at its strength: the springs
                                                // left past the band by the last unconverged sweep would creep it)
    h.ry[l] = M.refine_yield;
    h.by[l] = M.bend_yield + s.harden;
    h.rf[l] = M.refine;
    h.tonly[l] = M.tension_only ? ~0u : 0u;
    h.es[l] = (uint32_t)s.es[0] | (uint32_t)s.es[1] << 8 | (uint32_t)s.es[2] << 16;
    h.dk[l] = s.k[0] != 0.0f ? s.d[0] / s.k[0] : 0.0f; // (the same for the three edges: see shell_springs)
    h.mass[l] = s.mass;
    if (fresh) {
        a.strain = 0;
        for (int e = 0; e < 3; e++) h.hinge[e][l] = SK::kNoHinge;
    }
    a.level = s.level;
    a.cool = s.cool;
    a.pending = s.pending;
    uint8_t fl = 0;
    const float lmax = std::max(s.L0[0], std::max(s.L0[1], s.L0[2]));
    if (s.level < M.max_level && lmax * 0.70710678f >= M.min_edge) fl |= SK::kRefinable;
    for (int e = 0; e < 3; e++) {
        const uint32_t p = s.n[e], q = s.n[nx(e)], c = s.n[pv(e)];
        if (s.nb[e] < 0 && (b.info[p].flags & b.info[q].flags & NF_FIXED) && !(b.info[c].flags & NF_FIXED)) fl |= (uint8_t)(SK::kClamp << e);
    }
    a.flags = (uint8_t)((a.flags & (SK::kOwn * 7)) | fl);
}

// The hinges si evaluates (the rate classes must be up to date): a record per edge, rewritten in place, new ones
// appended (a full rebuild packs them in triangle order again).
void sync_hinges(SoftBody& b, uint32_t si, bool packed) {
    const Shell& s = b.shells[si];
    SK& K = b.shk;
    SK::Aux& a = K.aux[si];
    SK::Hot& H = K.hot[si >> 2];
    const uint32_t ln = si & 3;
    a.flags &= (uint8_t)~(SK::kOwn * 7);
    for (int e = 0; e < 3; e++) {
        uint32_t& slot = H.hinge[e][ln];
        const int j = s.nb[e];
        int fe = -1;
        if (j >= 0 && (size_t)j < b.shells.size()) fe = edge_of(b.shells[j], s.n[e], s.n[nx(e)]);
        const Shell* tp = fe >= 0 ? &b.shells[j] : nullptr;
        const int cs = rate_class(s.level), ct = tp ? rate_class(tp->level) : 0;
        if (!tp || !(cs > ct || (cs == ct && si < (uint32_t)j))) {
            slot = SK::kNoHinge; // (a record left behind is dropped by the next full rebuild)
            continue;
        }
        const Shell& t = *tp;
        const bool lo_is_s = si < (uint32_t)j;
        const Shell& L = lo_is_s ? s : t;
        const Shell& Hs = lo_is_s ? t : s;
        const int elo = lo_is_s ? e : fe;
        const float le0 = L.L0[elo];
        SK::Hinge hg;
        hg.wing = t.n[pv(fe)];
        hg.other = (uint32_t)j | ((uint32_t)fe << 30);
        hg.th0 = L.th0[elo];
        hg.kh = std::min(L.kb, Hs.kb) * (le0 * le0 / (L.area0 + Hs.area0));
        hg.c1 = hinge_fade_c(le0, L.area0);
        hg.c2 = hinge_fade_c(le0, Hs.area0);
        hg.lim = std::min(b.shell_material(s).bend_break, b.shell_material(t).bend_break) * std::min(s.flaw, t.flaw) * ((float)std::min(s.hs[e], t.hs[fe]) * (1.0f / 64.0f));
        hg.meta = (uint32_t)e | (lo_is_s ? 0u : SK::kSwap) | (s.n[e] != L.n[elo] ? SK::kRev : 0u) | ((uint32_t)std::min(15, (int)std::max(s.level, t.level)) << 4);
        if (packed || slot == SK::kNoHinge || slot >= K.hinge.size()) {
            slot = (uint32_t)K.hinge.size();
            K.hinge.push_back(hg);
        } else {
            K.hinge[slot] = hg;
        }
        a.flags |= (uint8_t)(SK::kOwn << e);
    }
}

// gather list of node v: the corner slot of every shell around it, and the wing slot of every hinge across the edge
// opposite to it that the other triangle evaluates. Returns the count (may exceed the stride: then nothing is written).
int sync_gather(SoftBody& b, uint32_t v) {
    SK& K = b.shk;
    // (up to the longest stride, 255: the centre of a disc of 48 triangles has 96 entries, and a buffer of 64 was
    // sorted and copied past its end)
    uint32_t tmp[256];
    int n = 0;
    for (uint32_t si : b.node_shells[v]) {
        const Shell& s = b.shells[si];
        const int c = corner_of(s, v);
        if (c < 0) continue;
        if (n < 256) tmp[n] = si * 6 + (uint32_t)c;
        n++;
        const int E = nx(c);
        const int j = s.nb[E];
        if (j < 0 || (K.aux[si].flags & (SK::kOwn << E))) continue;
        const int fe = edge_of(b.shells[j], s.n[E], s.n[nx(E)]);
        if (fe < 0 || !(K.aux[j].flags & (SK::kOwn << fe))) continue;
        if (n < 256) tmp[n] = (uint32_t)j * 6 + 3 + (uint32_t)fe;
        n++;
    }
    if (n <= K.stride && n <= 256) {
        std::sort(tmp, tmp + n); // (ascending slots: the gather reads forward)
        std::copy(tmp, tmp + n, K.gather.begin() + (size_t)v * K.stride);
        K.gcount[v] = (uint8_t)n;
    }
    return n;
}

void rebuild_all(SoftBody& b) {
    SK& K = b.shk;
    const size_t ns = b.shells.size(), nn = b.nodes.size();
    K.hot.assign((ns + 3) / 4, SK::Hot{});
    for (SK::Hot& h : K.hot)
        for (auto& row : h.hinge)
            for (uint32_t& x : row) x = SK::kNoHinge;
    K.aux.assign(ns, SK::Aux{});
    K.hinge.clear();
    K.slot.assign(ns * 6 + 1, F3{0, 0, 0});
    for (uint32_t si = 0; si < ns; si++) sync_record(b, si, true);
    for (uint32_t si = 0; si < ns; si++) sync_hinges(b, si, true);
    if (b.node_shells.size() < nn) b.node_shells.resize(nn);
    // stride of the gather lists: the longest list now and two spare entries (a longer one later packs them again)
    int need = 0;
    K.stride = 0;
    K.gather.clear();
    K.gcount.assign(nn, 0);
    for (uint32_t v = 0; v < nn; v++) need = std::max(need, sync_gather(b, v)); // (counts only: stride 0)
    K.stride = std::min(255, std::max(8, (need + 2 + 3) & ~3));
    K.gather.assign(nn * K.stride, 0);
    for (uint32_t v = 0; v < nn; v++) sync_gather(b, v);
    K.built_shells = ns;
    K.built_nodes = nn;
    K.dirty_shells.clear();
    K.dirty_nodes.clear();
    K.version = b.topo_version;
    b.shell_acc_stale = true;
}

// after topology operations that recorded what they touched (ShellOps)
void update_dirty(SoftBody& b) {
    SK& K = b.shk;
    const size_t ns = b.shells.size(), nn = b.nodes.size();
    if (K.hot.size() != (K.built_shells + 3) / 4 || K.gcount.size() != K.built_nodes || K.built_shells > ns || K.built_nodes > nn ||
        K.hinge.size() > ns * 3 + 256) { // (too many records left behind: pack them again)
        rebuild_all(b);
        return;
    }
    {
        SK::Hot empty{};
        for (auto& row : empty.hinge)
            for (uint32_t& x : row) x = SK::kNoHinge;
        K.hot.resize((ns + 3) / 4, empty);
    }
    K.aux.resize(ns, SK::Aux{});
    K.slot.resize(ns * 6 + 1, F3{0, 0, 0});
    for (size_t si = K.built_shells; si < ns; si++) K.dirty_shells.push_back((uint32_t)si);
    for (size_t v = K.built_nodes; v < nn; v++) K.dirty_nodes.push_back((uint32_t)v);
    // the touched shells and their neighbours (hinge ownership follows the rate classes of both)
    std::vector<uint32_t>& D = K.dirty_shells;
    std::sort(D.begin(), D.end());
    D.erase(std::unique(D.begin(), D.end()), D.end());
    std::vector<uint32_t> E = D;
    for (uint32_t si : D)
        if (si < ns)
            for (int e = 0; e < 3; e++)
                if (b.shells[si].nb[e] >= 0) E.push_back((uint32_t)b.shells[si].nb[e]);
    std::sort(E.begin(), E.end());
    E.erase(std::unique(E.begin(), E.end()), E.end());
    while (!E.empty() && E.back() >= ns) E.pop_back();
    for (uint32_t si : E) sync_record(b, si, si >= K.built_shells);
    for (uint32_t si : E) sync_hinges(b, si, false);
    // nodes: the ones whose fans changed, the corners of every re-synced shell (their wings are among them)
    std::vector<uint32_t>& N = K.dirty_nodes;
    for (uint32_t si : E)
        for (int c = 0; c < 3; c++) N.push_back(b.shells[si].n[c]);
    std::sort(N.begin(), N.end());
    N.erase(std::unique(N.begin(), N.end()), N.end());
    K.gather.resize(nn * K.stride, 0);
    K.gcount.resize(nn, 0);
    if (b.node_shells.size() < nn) b.node_shells.resize(nn);
    for (uint32_t v : N) {
        if (v >= nn) continue;
        if (sync_gather(b, v) > K.stride) {
            rebuild_all(b); // (a fan beyond the stride: re-laid out wider)
            return;
        }
    }
    K.built_shells = ns;
    K.built_nodes = nn;
    D.clear();
    N.clear();
    K.version = b.topo_version;
}

} // namespace

void SoftBody::shell_sync() {
    if (shells.empty()) return;
    SK& K = shk;
    if (K.version == topo_version && K.built_shells == shells.size() && K.built_nodes == nodes.size() && K.dirty_shells.empty() &&
        K.dirty_nodes.empty())
        return;
    // topology operations record what they touched and bump the version by one; anything else is rebuilt
    if (!K.dirty_shells.empty() || !K.dirty_nodes.empty() || K.version == topo_version) update_dirty(*this);
    else rebuild_all(*this);
}

float& SoftBody::shell_strain(uint32_t si) {
    shell_sync();
    return shk.aux[si].strain;
}

void SoftBody::clear_shell_events() {
    shell_events.clear();
    for (auto& s : shells) s.pending = 0;
    for (auto& a : shk.aux) a.pending = 0;
}

int SoftBody::shell_begin(float h, int step, int sub) {
    if (shells.empty() || rigid) return 0;
    shell_sync();
    SK& K = shk;
    SK::Pass& P = K.pass;
    // Multi-rate: a shell is evaluated at the rate its size needs (class c = ceil(level / 2): every sub >> c short
    // steps) and its forces are held in between; the nodes all move with the short step. A damaged sheet pays the
    // short step only for its fine triangles. After a topology change everything is evaluated again (a held force
    // would push a split node with the force of triangles it no longer carries).
    int maxc = 0;
    while ((2 << maxc) <= sub && maxc < 2) maxc++;
    P.maxc = maxc;
    P.cmin = std::clamp(shell_min_shift, 0, 2);
    bool any = false;
    for (int c = 0; c < 3; c++) {
        P.due[c] = c <= maxc && (step % (sub >> c) == 0 || shell_acc_stale);
        P.dt[c] = c <= maxc ? h * (float)(sub >> c) : h;
        any |= P.due[c];
    }
    shell_acc_stale = false;
    // work counters: the shells (and the hinges they own) of the rate classes due at this short step
    if (K.class_version != topo_version || K.class_n != shells.size()) {
        for (int c = 0; c < 3; c++) K.class_shells[c] = K.class_hinges[c] = 0;
        for (size_t si = 0; si < K.aux.size() && si < shells.size(); si++) {
            const SK::Aux& a = K.aux[si];
            const int c = std::max(rate_class(a.level), std::clamp(shell_min_shift, 0, 2));
            K.class_shells[c]++;
            K.class_hinges[c] += ((a.flags >> 3) & 1) + ((a.flags >> 4) & 1) + ((a.flags >> 5) & 1); // (kOwn << e)
        }
        K.class_version = topo_version;
        K.class_n = shells.size();
    }
    for (int c = 0; c < 3; c++)
        if (P.due[std::min(c, maxc)]) {
            evals_shell += K.class_shells[c];
            evals_hinge += K.class_hinges[c];
        }
    P.vmean = vec3(0);
    if (shell_mat.flex_damp > 0 && any) {
        vec3 mv(0);
        float m = 0;
        const Node* nd = nodes.data();
        for (size_t i = 0, n = nodes.size(); i < n; i++) {
            mv += nd[i].v * nd[i].mass;
            m += nd[i].mass;
        }
        P.vmean = m > 0 ? mv / m : vec3(0);
    }
    P.budget_left = shells.size() + 4 <= shell_cap;
    P.deform = allow_deform && !(resting && max_speed < rest_plastic);
    P.brk = allow_break;
    P.chunks = any ? (int)((shells.size() + kShellChunk - 1) / kShellChunk) : 0;
    if ((int)K.ev_edge.size() < P.chunks) {
        K.ev_edge.resize(P.chunks);
        K.ev_hinge.resize(P.chunks);
    }
    return P.chunks;
}

#if defined(__aarch64__)
namespace {
const SK::Hinge kZeroHinge{}; // (lanes without a hinge on an edge load this record; their results are masked out)
}
#endif

void SoftBody::shell_eval(int chunk) {
    SK& K = shk;
    const SK::Pass& P = K.pass;
    const ShellMaterial& M = shell_mat;
    const uint32_t ns = (uint32_t)shells.size();
    const uint32_t s0 = (uint32_t)chunk * kShellChunk, s1 = std::min(ns, s0 + (uint32_t)kShellChunk);
    std::vector<ShellEvent>& evE = K.ev_edge[chunk];
    std::vector<SK::HingeEvent>& evH = K.ev_hinge[chunk];
    evE.clear();
    evH.clear();
    Node* const nd = nodes.data();
    SK::Hot* const HOT = K.hot.data();
    SK::Aux* const AUX = K.aux.data();
    SK::Hinge* const HG = K.hinge.data();
    F3* const SL = K.slot.data();
    Shell* const SH = shells.data();
    const float aero_k = -0.5f * 1.225f * 1.2f * M.aero * 0.5f;
    const float flex_k = -M.flex_damp;
    const float third = 1.0f / 3.0f;
    const bool deform = P.deform, brk = P.brk;
    const float bend_damp = M.bend_damp;
    // (the yield, the plastic bend, the refinement thresholds and tension-only are the triangle's material's: Hot)
    const vec3 vmean = P.vmean;

    // overload of a hinge: decided in shell_end (it depends on the events of both triangles)
    auto hinge_events = [&](const SK::Hinge& hg, uint32_t si, float dth, float theta, float eq) {
        if (!brk || AUX[si].pending) return;
        const uint32_t oth = hg.other & 0x3fffffffu;
        if (AUX[oth].pending) return;
        const bool swap = hg.meta & SK::kSwap;
        const uint32_t lo = swap ? oth : si, hi = swap ? si : oth;
        const uint32_t elo = swap ? (hg.other >> 30) : (hg.meta & 3u);
        if (std::fabs(dth) * eq > hg.lim) {
            evH.push_back({lo * 3 + elo, hi, 1});
        } else if (std::fabs(dth) * eq > M.refine_angle && P.budget_left &&
                   (AUX[lo].level < shell_material(SH[lo]).max_level || AUX[hi].level < shell_material(SH[hi]).max_level)) {
            // (the fold relative to the rest angle: a body's authored edges are folds that ask for nothing)
            evH.push_back({lo * 3 + elo, hi, 0});
        }
    };
    // plastic bend: the rest angle follows (written through to both triangles); returns the reduced angle
    auto hinge_yield = [&](SK::Hinge& hg, uint32_t si, float dth, float eq) {
        const float bend_yield = HOT[si >> 2].by[si & 3];
        const float ex = dth - std::copysign(bend_yield / eq, dth);
        const float th0 = wrap_pi(hg.th0 + ex);
        const uint32_t oth = hg.other & 0x3fffffffu;
        hg.th0 = th0;
        SH[si].th0[hg.meta & 3u] = th0;
        SH[oth].th0[hg.other >> 30] = th0;
        // (the triangle that evaluates the hinge hardens: its yield is the hinge's; the other one's is another thread's)
        if (const ShellMaterial& TM = shell_material(SH[si]); TM.bend_harden > 0) {
            SH[si].harden = std::min((TM.bend_harden_max - 1.0f) * TM.bend_yield, SH[si].harden + TM.bend_harden * std::fabs(ex) * eq);
            HOT[si >> 2].by[si & 3] = TM.bend_yield + SH[si].harden;
        }
        return dth - ex;
    };
    // the rest of a triangle after its springs, face and hinges: clamped borders (rare), overload events of the edges,
    // its corner slots
    auto finish = [&](uint32_t si, vec3* f, float worst, int worst_e, bool plastic, bool face, vec3 nrm, float area2) {
        SK::Aux& ax = AUX[si];
        ax.strain = worst;
        if (face && (ax.flags & (SK::kClamp * 7))) {
            Shell& s = SH[si];
            const SK::Hot& h = HOT[si >> 2];
            const uint32_t l = si & 3;
            const vec3 p[3] = {nd[h.n[0][l]].p, nd[h.n[1][l]].p, nd[h.n[2][l]].p};
            const vec3 v[3] = {nd[h.n[0][l]].v, nd[h.n[1][l]].v, nd[h.n[2][l]].v};
            const float dt = P.dt[std::min(std::max(rate_class(ax.level), P.cmin), P.maxc)];
            for (int e = 0; e < 3; e++) {
                // clamped edge (both ends fixed, nothing beyond): a hinge against the rest plane, acting on the
                // opposite corner (a pinned border would let torn flaps swing forever)
                if (!(ax.flags & (SK::kClamp << e))) continue;
                const int c = pv(e);
                const vec3 ev = p[nx(e)] - p[e];
                const float le = length(ev);
                if (le < 1e-6f) continue;
                const vec3 eh = ev / le;
                const float h1 = area2 / le;
                const float th = lut_atan2(dot(cross(s.n0, nrm), eh), dot(s.n0, nrm));
                float dth = wrap_pi(th - s.th0[e]);
                const float eq = bend_equivalent(s.level);
                const ShellMaterial& SM = shell_material(s);
                const float lim = std::min(SM.bend_yield, SM.bend_break * s.flaw);
                if (deform && std::fabs(dth) * eq > lim) {
                    const float ex = dth - std::copysign(lim / eq, dth);
                    s.th0[e] = wrap_pi(s.th0[e] + ex);
                    dth -= ex;
                }
                const float kh = s.kb * (s.L0[e] * s.L0[e] / (2.0f * s.area0)) * clampf((kPi - std::fabs(th)) / 0.4f, 0.0f, 1.0f);
                const float rate = dot(v[c], nrm) / h1;
                f[c] += nrm * (-kh * (dth + M.bend_damp * dt * rate) / h1);
            }
        }
        if (ax.cool) SH[si].cool = --ax.cool;
        if (brk && !ax.pending) {
            const bool can_refine = P.budget_left && (ax.flags & SK::kRefinable);
            if (worst > 1.0f) {
                if (can_refine) evE.push_back({si, 0, 0});
                else if (!ax.cool) evE.push_back({si, 1, (uint8_t)worst_e});
            } else if ((worst > HOT[si >> 2].rf[si & 3] || plastic) && can_refine) {
                evE.push_back({si, 0, 0});
            }
        }
        F3* const out = SL + (size_t)si * 6;
        out[0] = f3(f[0]);
        out[1] = f3(f[1]);
        out[2] = f3(f[2]);
    };

#if defined(__aarch64__)
    // Four triangles at a time: the lanes are the triangles of a hot block. Springs, face and the hinges the triangles
    // evaluate (per edge: the lanes owning a hinge across it; its three own corners are in registers already, only
    // the far wing is loaded) all accumulate into the same corner vectors.
    const float32x4_t vzero = vdupq_n_f32(0.0f), vone = vdupq_n_f32(1.0f), pi = vdupq_n_f32(kPi), two_pi = vdupq_n_f32(2 * kPi);
    const V3 vmean4 = {vdupq_n_f32(vmean.x), vdupq_n_f32(vmean.y), vdupq_n_f32(vmean.z)};
    for (uint32_t bi = s0 >> 2; bi < (s1 + 3) >> 2; bi++) {
        uint32_t due_l[4];
        float dt_l[4];
        bool any_due = false;
        for (uint32_t l = 0; l < 4; l++) {
            const uint32_t si = bi * 4 + l;
            const int cl = si < s1 ? std::min(std::max(rate_class(AUX[si].level), P.cmin), P.maxc) : 0;
            due_l[l] = si >= s0 && si < s1 && P.due[cl] ? ~0u : 0u;
            dt_l[l] = P.dt[cl];
            any_due |= due_l[l] != 0;
        }
        if (!any_due) continue;
        SK::Hot& H = HOT[bi];
        const uint32x4_t due = vld1q_u32(due_l);
        V3 Pc[3], Vc[3];
        for (int c = 0; c < 3; c++) {
            const Node *q0 = &nd[H.n[c][0]], *q1 = &nd[H.n[c][1]], *q2 = &nd[H.n[c][2]], *q3 = &nd[H.n[c][3]];
            Pc[c] = load4(&q0->p.x, &q1->p.x, &q2->p.x, &q3->p.x);
            Vc[c] = load4(&q0->v.x, &q1->v.x, &q2->v.x, &q3->v.x);
        }
        V3 F[3] = {{vzero, vzero, vzero}, {vzero, vzero, vzero}, {vzero, vzero, vzero}};
        // ---- edge springs (plastic yield up to the fracture strain)
        float32x4_t wx = vzero, wy = vone;
        uint32x4_t we = vdupq_n_u32(0), plastic = vdupq_n_u32(0);
        const float32x4_t brk4 = vld1q_f32(H.brk), dk4 = vld1q_f32(H.dk), yl4 = vld1q_f32(H.yl), ry4 = vld1q_f32(H.ry);
        const uint32x4_t es4 = vld1q_u32(H.es), tonly4 = vld1q_u32(H.tonly);
        for (int e = 0; e < 3; e++) {
            const int a = e, bb = nx(e);
            const V3 dis = Pc[a] - Pc[bb];
            const float32x4_t len2 = dot4(dis, dis);
            const uint32x4_t ok = vcgeq_f32(len2, vdupq_n_f32(1e-14f));
            const float32x4_t len2s = vbslq_f32(ok, len2, vone);
            const float32x4_t inv = rsqrt4(len2s);
            const float32x4_t len = vmulq_f32(len2s, inv);
            float32x4_t k = vld1q_f32(H.k[e]);
            float32x4_t d = vmulq_f32(k, dk4);
            float32x4_t L = vld1q_f32(H.L[e]);
            const float32x4_t L0 = vld1q_f32(H.L0[e]);
            if (any4(tonly4)) {
                const uint32x4_t slack = vandq_u32(tonly4, vcltq_f32(len, L));
                k = vbslq_f32(slack, vzero, k);
                d = vbslq_f32(slack, vmulq_f32(d, vdupq_n_f32(0.1f)), d);
            }
            // elongation at fracture (the edge's pattern code: a byte per edge, x / 64)
            const float32x4_t fe = vmulq_n_f32(vcvtq_f32_u32(vandq_u32(vshlq_u32(es4, vdupq_n_s32(-8 * e)), vdupq_n_u32(255))), 1.0f / 64.0f);
            const float32x4_t lim = vmulq_f32(L0, vmulq_f32(brk4, fe));
            const float32x4_t stretch = vsubq_f32(len, L0);
            if (deform) {
                // beyond the yield strain the rest length follows (not past the fracture strain)
                const float32x4_t ylen = vmulq_f32(yl4, L0);
                const float32x4_t diff = vsubq_f32(len, L);
                const uint32x4_t up = vcgtq_f32(diff, ylen), dn = vcltq_f32(diff, vnegq_f32(ylen));
                const uint32x4_t chg = vandq_u32(vandq_u32(vandq_u32(ok, due), vandq_u32(vcgtq_f32(k, vzero), vcltq_f32(stretch, lim))), vorrq_u32(up, dn));
                if (any4(chg)) {
                    L = vbslq_f32(chg, vbslq_f32(up, vsubq_f32(len, ylen), vaddq_f32(len, ylen)), L);
                    vst1q_f32(H.L[e], L);
                    uint32_t c4[4];
                    vst1q_u32(c4, chg);
                    for (uint32_t l = 0; l < 4; l++)
                        if (c4[l]) SH[bi * 4 + l].L[e] = H.L[e][l];
                }
            }
            const float32x4_t slen = vfmsq_f32(vmulq_f32(vnegq_f32(k), vsubq_f32(len, L)), vmulq_f32(d, dot4(Vc[a] - Vc[bb], dis)), inv);
            const V3 fv = dis * vbslq_f32(ok, vmulq_f32(slen, inv), vzero);
            F[a] = F[a] + fv;
            F[bb] = F[bb] - fv;
            // (only while it is being pulled: a relaxed, stretched flap is no crack)
            const uint32x4_t better = vandq_u32(vandq_u32(ok, vcgtq_f32(len, L)), vcgtq_f32(vmulq_f32(stretch, wy), vmulq_f32(wx, lim)));
            wx = vbslq_f32(better, stretch, wx);
            wy = vbslq_f32(better, lim, wy);
            we = vbslq_u32(better, vdupq_n_u32((uint32_t)e), we);
            plastic = vorrq_u32(plastic, vandq_u32(ok, vcgtq_f32(vabsq_f32(vsubq_f32(L, L0)), vmulq_f32(ry4, L0))));
        }
        const float32x4_t worst = vdivq_f32(wx, wy);
        // ---- face: air drag and the internal friction of bending, a third on every corner
        const V3 N = cross4(Pc[1] - Pc[0], Pc[2] - Pc[0]);
        const float32x4_t n2 = dot4(N, N);
        const uint32x4_t okf = vcgtq_f32(n2, vdupq_n_f32(1e-16f));
        const float32x4_t n2s = vbslq_f32(okf, n2, vone);
        const float32x4_t rn = rsqrt4(n2s);
        const V3 nrm = N * rn;
        const float32x4_t area2 = vmulq_f32(n2s, rn);
        const float32x4_t vn = vmulq_f32(dot4(Vc[0] + Vc[1] + Vc[2], nrm), vdupq_n_f32(third));
        const float32x4_t drag = vmulq_f32(vmulq_f32(vdupq_n_f32(aero_k), area2), vmulq_f32(vabsq_f32(vn), vn));
        const float32x4_t flex = vmulq_f32(vmulq_f32(vdupq_n_f32(flex_k), vld1q_f32(H.mass)), vsubq_f32(vn, dot4(vmean4, nrm)));
        const V3 fa = nrm * vbslq_f32(okf, vmulq_f32(vaddq_f32(drag, flex), vdupq_n_f32(third)), vzero);
        F[0] = F[0] + fa;
        F[1] = F[1] + fa;
        F[2] = F[2] + fa;
        // ---- bending hinges (Bridson et al. 2003) across each edge, for the lanes that evaluate one there
        const float32x4_t dt4 = vld1q_f32(dt_l);
        for (int e = 0; e < 3; e++) {
            uint32_t own_l[4];
            const SK::Hinge* R[4];
            bool any_own = false;
            for (uint32_t l = 0; l < 4; l++) {
                const uint32_t hi = H.hinge[e][l];
                own_l[l] = due_l[l] && hi != SK::kNoHinge ? ~0u : 0u;
                R[l] = own_l[l] ? &HG[hi] : &kZeroHinge;
                any_own |= own_l[l] != 0;
            }
            if (!any_own) continue;
            const uint32x4_t own = vld1q_u32(own_l);
            // the four records, transposed: (wing, other, th0, kh), (c1, c2, lim, meta)
            const float32x4x2_t ta = vtrnq_f32(vld1q_f32((const float*)R[0]), vld1q_f32((const float*)R[1]));
            const float32x4x2_t tb = vtrnq_f32(vld1q_f32((const float*)R[2]), vld1q_f32((const float*)R[3]));
            const float32x4_t th0v = vcombine_f32(vget_high_f32(ta.val[0]), vget_high_f32(tb.val[0]));
            const float32x4_t khv = vcombine_f32(vget_high_f32(ta.val[1]), vget_high_f32(tb.val[1]));
            const float32x4x2_t tc = vtrnq_f32(vld1q_f32((const float*)R[0] + 4), vld1q_f32((const float*)R[1] + 4));
            const float32x4x2_t td = vtrnq_f32(vld1q_f32((const float*)R[2] + 4), vld1q_f32((const float*)R[3] + 4));
            const float32x4_t c1v = vcombine_f32(vget_low_f32(tc.val[0]), vget_low_f32(td.val[0]));
            const float32x4_t c2v = vcombine_f32(vget_low_f32(tc.val[1]), vget_low_f32(td.val[1]));
            const float32x4_t limv = vcombine_f32(vget_high_f32(tc.val[0]), vget_high_f32(td.val[0]));
            const uint32x4_t metav = vreinterpretq_u32_f32(vcombine_f32(vget_high_f32(tc.val[1]), vget_high_f32(td.val[1])));
            const uint32x4_t swapm = vtstq_u32(metav, vdupq_n_u32(SK::kSwap)), revm = vtstq_u32(metav, vdupq_n_u32(SK::kRev));
            const float eqs[4] = {bend_equivalent((int)(R[0]->meta >> 4)), bend_equivalent((int)(R[1]->meta >> 4)), bend_equivalent((int)(R[2]->meta >> 4)),
                                  bend_equivalent((int)(R[3]->meta >> 4))};
            const float32x4_t eq = vld1q_f32(eqs);
            const Node *w0 = &nd[R[0]->wing], *w1n = &nd[R[1]->wing], *w2n = &nd[R[2]->wing], *w3n = &nd[R[3]->wing];
            const V3 W = load4(&w0->p.x, &w1n->p.x, &w2n->p.x, &w3n->p.x), WV = load4(&w0->v.x, &w1n->v.x, &w2n->v.x, &w3n->v.x);
            const int cw = pv(e), en = nx(e);
            auto sel = [](uint32x4_t m, V3 a, V3 b) { return V3{vbslq_f32(m, a.x, b.x), vbslq_f32(m, a.y, b.y), vbslq_f32(m, a.z, b.z)}; };
            const V3 x1 = sel(swapm, W, Pc[cw]), x2 = sel(swapm, Pc[cw], W);
            const V3 u1 = sel(swapm, WV, Vc[cw]), u2 = sel(swapm, Vc[cw], WV);
            const V3 x3 = sel(revm, Pc[en], Pc[e]), x4 = sel(revm, Pc[e], Pc[en]);
            const V3 u3 = sel(revm, Vc[en], Vc[e]), u4 = sel(revm, Vc[e], Vc[en]);
            const V3 E = x4 - x3;
            const float32x4_t le2 = dot4(E, E);
            const V3 N1 = cross4(E, x1 - x3), N2 = cross4(x3 - x4, x2 - x4);
            const float32x4_t a1 = dot4(N1, N1), a2 = dot4(N2, N2);
            const uint32x4_t valid =
                vandq_u32(own, vandq_u32(vcgeq_f32(le2, vdupq_n_f32(1e-12f)), vandq_u32(vcgeq_f32(a1, vdupq_n_f32(1e-16f)), vcgeq_f32(a2, vdupq_n_f32(1e-16f)))));
            const float32x4_t le2s = vbslq_f32(valid, le2, vone), a1s = vbslq_f32(valid, a1, vone), a2s = vbslq_f32(valid, a2, vone);
            const float32x4_t ile = rsqrt4(le2s), il1 = rsqrt4(a1s), il2 = rsqrt4(a2s);
            const float32x4_t le = vmulq_f32(le2s, ile);
            // (atan2 is scale free: the unnormalised normals give the angle)
            const float32x4_t theta = atan2_4(vmulq_f32(dot4(cross4(N1, N2), E), ile), dot4(N1, N2));
            const float32x4_t h1 = vmulq_f32(vmulq_f32(a1s, il1), ile), h2 = vmulq_f32(vmulq_f32(a2s, il2), ile);
            const V3 w1 = N1 * vmulq_f32(il1, il1), w2 = N2 * vmulq_f32(il2, il2);
            const float32x4_t nle = vnegq_f32(le);
            const V3 g1 = w1 * nle, g2 = w2 * nle;
            const V3 g3 = -(w1 * vmulq_f32(dot4(x1 - x4, E), ile) + w2 * vmulq_f32(dot4(x2 - x4, E), ile));
            const V3 g4 = -(g1 + g2 + g3); // (the gradients sum to zero: translation invariance)
            float32x4_t dth = vsubq_f32(theta, th0v);
            dth = vbslq_f32(vcgtq_f32(dth, pi), vsubq_f32(dth, two_pi), dth);
            dth = vbslq_f32(vcltq_f32(dth, vnegq_f32(pi)), vaddq_f32(dth, two_pi), dth);
            const float32x4_t adth_eq = vmulq_f32(vabsq_f32(dth), eq);
            // rare lanes, scalar in lane order: plastic bends, overloads (a fold beyond the brittle limit, or sharper
            // than refine_angle with refinement left)
            const uint32x4_t yielding = vandq_u32(valid, vcgtq_f32(adth_eq, deform ? vld1q_f32(H.by) : vdupq_n_f32(3.4e38f)));
            const bool events =
                brk && any4(vandq_u32(valid, vorrq_u32(vcgtq_f32(adth_eq, limv),
                                                       vcgtq_f32(adth_eq, vdupq_n_f32(P.budget_left ? M.refine_angle : 3.4e38f)))));
            if (events || any4(yielding)) {
                float dd[4], tt[4];
                uint32_t yy[4], vv[4];
                vst1q_f32(dd, dth);
                vst1q_f32(tt, theta);
                vst1q_u32(yy, yielding);
                vst1q_u32(vv, valid);
                for (uint32_t l = 0; l < 4; l++) {
                    if (!vv[l]) continue;
                    SK::Hinge& hg = HG[H.hinge[e][l]];
                    if (yy[l]) dd[l] = hinge_yield(hg, bi * 4 + l, dd[l], eqs[l]);
                    hinge_events(hg, bi * 4 + l, dd[l], tt[l], eqs[l]);
                }
                dth = vld1q_f32(dd);
            }
            // stiffness from the rest shape, faded out for a triangle squashed towards a line (its angle gradient
            // ~ 1/height blows up) and near a full fold (the angle wraps at +-pi)
            float32x4_t w = vminq_f32(vmaxq_f32(vmulq_f32(vsubq_f32(pi, vabsq_f32(theta)), vdupq_n_f32(1.0f / 0.4f)), vzero), vone);
            const float32x4_t hr = vminq_f32(vmaxq_f32(vminq_f32(vmulq_f32(h1, c1v), vmulq_f32(h2, c2v)), vzero), vone);
            w = vmulq_f32(w, vmulq_f32(hr, hr));
            const float32x4_t khw = vmulq_f32(khv, w);
            const float32x4_t rate = vaddq_f32(vaddq_f32(dot4(g1, u1), dot4(g2, u2)), vaddq_f32(dot4(g3, u3), dot4(g4, u4)));
            float32x4_t fm = vmulq_f32(vnegq_f32(khw), vfmaq_f32(dth, vmulq_f32(vdupq_n_f32(bend_damp), dt4), rate));
            fm = vbslq_f32(vandq_u32(valid, vcgtq_f32(khw, vzero)), fm, vzero);
            const V3 G1 = g1 * fm, G2 = g2 * fm, G3 = g3 * fm, G4 = g4 * fm;
            F[cw] = F[cw] + sel(swapm, G2, G1);
            F[e] = F[e] + sel(revm, G4, G3);
            F[en] = F[en] + sel(revm, G3, G4);
            const V3 Wf = sel(swapm, G1, G2); // on the far wing: to the slot of this edge
            float wx4[4], wy4[4], wz4[4];
            vst1q_f32(wx4, Wf.x);
            vst1q_f32(wy4, Wf.y);
            vst1q_f32(wz4, Wf.z);
            for (uint32_t l = 0; l < 4; l++)
                if (own_l[l]) SL[(size_t)(bi * 4 + l) * 6 + 3 + e] = F3{wx4[l], wy4[l], wz4[l]};
        }
        // ---- per triangle: the rest
        float fx[3][4], fy[3][4], fz[3][4], nx4[4], ny4[4], nz4[4], ar[4], wo[4];
        uint32_t we4[4], pl4[4], okf4[4];
        for (int c = 0; c < 3; c++) {
            vst1q_f32(fx[c], F[c].x);
            vst1q_f32(fy[c], F[c].y);
            vst1q_f32(fz[c], F[c].z);
        }
        vst1q_f32(nx4, nrm.x);
        vst1q_f32(ny4, nrm.y);
        vst1q_f32(nz4, nrm.z);
        vst1q_f32(ar, area2);
        vst1q_f32(wo, worst);
        vst1q_u32(we4, we);
        vst1q_u32(pl4, plastic);
        vst1q_u32(okf4, okf);
        for (uint32_t l = 0; l < 4; l++) {
            if (!due_l[l]) continue;
            vec3 f[3] = {vec3(fx[0][l], fy[0][l], fz[0][l]), vec3(fx[1][l], fy[1][l], fz[1][l]), vec3(fx[2][l], fy[2][l], fz[2][l])};
            finish(bi * 4 + l, f, wo[l], (int)we4[l], pl4[l] != 0, okf4[l] != 0, vec3(nx4[l], ny4[l], nz4[l]), ar[l]);
        }
    }
#else
    for (uint32_t si = s0; si < s1; si++) {
        const int cl = std::min(std::max(rate_class(AUX[si].level), P.cmin), P.maxc);
        if (!P.due[cl]) continue;
        const float dt = P.dt[cl];
        SK::Hot& H = HOT[si >> 2];
        const uint32_t l = si & 3;
        const vec3 p[3] = {nd[H.n[0][l]].p, nd[H.n[1][l]].p, nd[H.n[2][l]].p};
        const vec3 v[3] = {nd[H.n[0][l]].v, nd[H.n[1][l]].v, nd[H.n[2][l]].v};
        vec3 f[3] = {vec3(0), vec3(0), vec3(0)};
        // ---- edge springs (plastic yield up to the fracture strain)
        float wx = 0, wy = 1; // worst strain = wx / wy
        int worst_e = 0;
        bool plastic = false;
        for (int e = 0; e < 3; e++) {
            const int a = e, bb = nx(e);
            const vec3 dis = p[a] - p[bb];
            const float len2 = dot(dis, dis);
            if (len2 < 1e-14f) continue;
            const float inv = rsqrt(len2);
            const float len = len2 * inv;
            float k = H.k[e][l], d = k * H.dk[l];
            float L = H.L[e][l];
            const float L0 = H.L0[e][l];
            if (H.tonly[l] && len < L) {
                k = 0;
                d *= 0.1f;
            }
            const float lim = L0 * (H.brk[l] * ((float)((H.es[l] >> (8 * e)) & 255u) * (1.0f / 64.0f))); // elongation at fracture
            const float stretch = len - L0;
            if (deform && k > 0 && stretch < lim) {
                // plastic yield: beyond the yield strain the rest length follows (not past the fracture strain: a
                // material that has used up its ductility only loads its neighbours further, it does not flow forever)
                const float ylen = H.yl[l] * L0;
                const float diff = len - L;
                if (diff > ylen || diff < -ylen) {
                    L = diff > ylen ? len - ylen : len + ylen;
                    H.L[e][l] = L;
                    SH[si].L[e] = L;
                }
            }
            const float slen = -k * (len - L) - d * dot(v[a] - v[bb], dis) * inv;
            const vec3 fv = dis * (slen * inv);
            f[a] += fv;
            f[bb] -= fv;
            // (only while it is being pulled: a relaxed, stretched flap is no crack)
            const bool better = len > L && stretch * wy > wx * lim;
            wx = better ? stretch : wx;
            wy = better ? lim : wy;
            worst_e = better ? e : worst_e;
            plastic |= std::fabs(L - L0) > H.ry[l] * L0;
        }
        const float worst = wx / wy;
        // ---- face: air drag and the internal friction of bending
        const vec3 N = cross(p[1] - p[0], p[2] - p[0]);
        const float n2 = dot(N, N);
        vec3 nrm(0);
        float area2 = 0;
        if (n2 > 1e-16f) {
            const float rn = rsqrt(n2);
            nrm = N * rn;
            area2 = n2 * rn;
            const float vn = dot(v[0] + v[1] + v[2], nrm) * third;
            const float drag = aero_k * area2 * std::fabs(vn) * vn;
            const float flex = flex_k * H.mass[l] * (vn - dot(vmean, nrm));
            const vec3 fa = nrm * ((drag + flex) * third);
            f[0] += fa;
            f[1] += fa;
            f[2] += fa;
        }
        // ---- bending hinges (Bridson et al. 2003)
        for (int e = 0; e < 3; e++) {
            if (H.hinge[e][l] == SK::kNoHinge) continue;
            SK::Hinge& hg = HG[H.hinge[e][l]];
            const Node& W = nd[hg.wing];
            const int cw = pv(e);
            const bool swap = hg.meta & SK::kSwap, rev = hg.meta & SK::kRev;
            const vec3 x1 = swap ? W.p : p[cw], x2 = swap ? p[cw] : W.p;
            const vec3 u1 = swap ? W.v : v[cw], u2 = swap ? v[cw] : W.v;
            const int i3 = rev ? nx(e) : e, i4 = rev ? e : nx(e);
            const vec3 x3 = p[i3], x4 = p[i4];
            const vec3 E = x4 - x3;
            const float le2 = dot(E, E);
            const vec3 N1 = cross(E, x1 - x3), N2 = cross(x3 - x4, x2 - x4);
            const float a1 = dot(N1, N1), a2 = dot(N2, N2);
            F3& wslot = SL[(size_t)si * 6 + 3 + e];
            if (le2 < 1e-12f || a1 < 1e-16f || a2 < 1e-16f) {
                wslot = F3{0, 0, 0};
                continue;
            }
            const float ile = rsqrt(le2), il1 = rsqrt(a1), il2 = rsqrt(a2);
            const float le = le2 * ile;
            const float theta = lut_atan2(dot(cross(N1, N2), E) * ile, dot(N1, N2));
            const float h1 = a1 * il1 * ile, h2 = a2 * il2 * ile;
            const vec3 w1 = N1 * (il1 * il1), w2 = N2 * (il2 * il2);
            const vec3 g1 = w1 * (-le), g2 = w2 * (-le);
            const vec3 g3 = -(w1 * (dot(x1 - x4, E) * ile) + w2 * (dot(x2 - x4, E) * ile));
            const vec3 g4 = -(g1 + g2 + g3);
            float dth = theta - hg.th0;
            if (dth > kPi) dth -= 2 * kPi;
            else if (dth < -kPi) dth += 2 * kPi;
            const float eq = bend_equivalent((int)(hg.meta >> 4));
            if (deform && std::fabs(dth) * eq > H.by[l]) dth = hinge_yield(hg, si, dth, eq);
            hinge_events(hg, si, dth, theta, eq);
            float w = clampf((kPi - std::fabs(theta)) / 0.4f, 0.0f, 1.0f);
            w *= sqr(clampf(std::min(h1 * hg.c1, h2 * hg.c2), 0.0f, 1.0f));
            const float kh = hg.kh * w;
            if (kh <= 0) {
                wslot = F3{0, 0, 0};
                continue;
            }
            const float rate = dot(g1, u1) + dot(g2, u2) + dot(g3, v[i3]) + dot(g4, v[i4]);
            const float fm = -kh * (dth + bend_damp * dt * rate);
            vec3 fw;
            if (swap) {
                fw = g1 * fm;
                f[cw] += g2 * fm;
            } else {
                f[cw] += g1 * fm;
                fw = g2 * fm;
            }
            f[i3] += g3 * fm;
            f[i4] += g4 * fm;
            wslot = f3(fw);
        }
        finish(si, f, worst, worst_e, plastic, n2 > 1e-16f, nrm, area2);
    }
#endif
}

void SoftBody::shell_end() {
    SK& K = shk;
    const SK::Pass& P = K.pass;
    auto queue = [&](uint32_t si, uint8_t kind, int e) {
        shell_events.push_back({si, kind, (uint8_t)e});
        shells[si].pending = 1;
        K.aux[si].pending = 1;
    };
    // edge overloads in shell order, then the hinges in the order of their lower-index triangle (as a serial pass
    // over the shells would find them: a hinge is skipped when either triangle already has an event)
    static thread_local std::vector<SK::HingeEvent> hev;
    hev.clear();
    for (int c = 0; c < P.chunks; c++) {
        for (const ShellEvent& e : K.ev_edge[c]) queue(e.shell, e.kind, e.edge);
        hev.insert(hev.end(), K.ev_hinge[c].begin(), K.ev_hinge[c].end());
        K.ev_edge[c].clear();
        K.ev_hinge[c].clear();
    }
    if (hev.empty()) return;
    std::sort(hev.begin(), hev.end(), [](const SK::HingeEvent& a, const SK::HingeEvent& b) { return a.key < b.key; });
    for (const SK::HingeEvent& he : hev) {
        const uint32_t lo = he.key / 3, elo = he.key % 3, hi = he.hi;
        if (K.aux[lo].pending || K.aux[hi].pending) continue;
        const int llo = K.aux[lo].level, lhi = K.aux[hi].level;
        const bool rlo = P.budget_left && llo < shell_material(shells[lo]).max_level, rhi = P.budget_left && lhi < shell_material(shells[hi]).max_level;
        if (he.fold) {
            // a brittle sheet folded too sharply: refine first, crack along this edge at the finest size
            if (rlo || rhi) queue(rlo && (!rhi || llo <= lhi) ? lo : hi, 0, 0);
            else if (!K.aux[lo].cool) queue(lo, 2, (int)elo);
        } else if (rlo || rhi) {
            queue(rlo && (!rhi || llo <= lhi) ? lo : hi, 0, 0);
        }
    }
}

void SoftBody::shell_gather(size_t n0, size_t n1, vec3* F) const {
    const SK& K = shk;
    n1 = std::min(n1, K.gcount.size());
    const F3* const SL = K.slot.data();
    const int stride = K.stride;
    for (size_t v = n0; v < n1; v++) {
        const uint32_t* g = K.gather.data() + v * stride;
        const int n = K.gcount[v];
#if defined(__aarch64__)
        // (16-byte loads of 12-byte slots: the fourth lane is the next slot's x, never used; the array has a spare slot)
        float32x4_t acc = vdupq_n_f32(0.0f);
        for (int k = 0; k < n; k++) acc = vaddq_f32(acc, vld1q_f32(&SL[g[k]].x));
        F[v].x += vgetq_lane_f32(acc, 0);
        F[v].y += vgetq_lane_f32(acc, 1);
        F[v].z += vgetq_lane_f32(acc, 2);
#else
        float x = 0, y = 0, z = 0;
        for (int k = 0; k < n; k++) {
            x += SL[g[k]].x;
            y += SL[g[k]].y;
            z += SL[g[k]].z;
        }
        F[v] += vec3(x, y, z);
#endif
    }
}

void SoftBody::compute_shell_forces(float h, int step, int sub) {
    if (shells.empty() || rigid) return;
    const int chunks = shell_begin(h, step, sub);
    {
        PROFILE_ACCUM("Sheet elements");
        for (int c = 0; c < chunks; c++) shell_eval(c);
    }
    if (chunks) shell_end();
    PROFILE_ACCUM("Sheet gather");
    shell_gather(0, nodes.size(), force.data());
}

} // namespace bl::phys
