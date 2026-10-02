// Fracture patterns of the triangle-element sheets (see shell_pattern.h): the lines, the queries on them (a grid per
// impact), the strength codes of the edges and the contacts that lay a pattern.
#include "core/profiler.h"
#include "phys/shell_util.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace bl::phys {

using namespace shell_detail;

namespace {

constexpr float kTwoPi = 6.28318531f;
constexpr uint32_t kNone = 0xffffffffu;

inline float r01(uint32_t h) { return (float)(hash32(h) & 0xffffff) * (1.0f / 16777216.0f); }
inline uint32_t mix(uint32_t a, uint32_t b) { return hash32(a * 0x9e3779b9u + hash32(b + 0x632be5abu)); }
inline float cross2(vec2 a, vec2 b) { return a.x * b.y - a.y * b.x; }
inline uint32_t bits(float f) {
    uint32_t u;
    std::memcpy(&u, &f, 4);
    return u;
}
inline vec2 vmin2(vec2 a, vec2 b) { return vec2(std::min(a.x, b.x), std::min(a.y, b.y)); }
inline vec2 vmax2(vec2 a, vec2 b) { return vec2(std::max(a.x, b.x), std::max(a.y, b.y)); }

inline float seg_dist(vec2 x, vec2 a, vec2 b, vec2* foot) {
    const vec2 ab = b - a;
    const float l2 = dot(ab, ab);
    const float t = l2 > 1e-14f ? std::clamp(dot(x - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
    const vec2 f = a + ab * t;
    *foot = f;
    return length(x - f);
}

// the segments in the grid cells overlapping box [lo, hi]
template <class F>
void visit(const ShellImpact& im, vec2 lo, vec2 hi, F&& f) {
    if (im.gn <= 0) return;
    const float ic = 1.0f / im.cell;
    const int x0 = (int)std::floor((lo.x - im.org.x) * ic), x1 = (int)std::floor((hi.x - im.org.x) * ic);
    const int y0 = (int)std::floor((lo.y - im.org.y) * ic), y1 = (int)std::floor((hi.y - im.org.y) * ic);
    if (x1 < 0 || y1 < 0 || x0 >= im.gn || y0 >= im.gn) return;
    for (int y = std::max(0, y0); y <= std::min(im.gn - 1, y1); y++)
        for (int x = std::max(0, x0); x <= std::min(im.gn - 1, x1); x++) {
            const int c = y * im.gn + x;
            for (uint32_t k = im.start[c]; k < im.start[c + 1]; k++) f(im.segs[im.items[k]]);
        }
}

void build_grid(ShellImpact& im) {
    vec2 mn(1e30f), mx(-1e30f);
    for (const auto& s : im.segs) {
        mn = vmin2(mn, vmin2(s.a, s.b));
        mx = vmax2(mx, vmax2(s.a, s.b));
    }
    if (im.segs.empty()) return;
    const float ext = std::max(mx.x - mn.x, mx.y - mn.y) + 1e-3f;
    im.gn = std::clamp((int)std::sqrt((float)im.segs.size()) * 2, 4, 64);
    im.cell = ext / (float)im.gn;
    im.org = mn;
    const int nc = im.gn * im.gn;
    std::vector<std::pair<uint32_t, uint32_t>> ent; // (cell, segment)
    const float ic = 1.0f / im.cell;
    for (uint32_t k = 0; k < im.segs.size(); k++) {
        const auto& s = im.segs[k];
        const vec2 lo = vmin2(s.a, s.b), hi = vmax2(s.a, s.b);
        const int x0 = std::clamp((int)std::floor((lo.x - mn.x) * ic), 0, im.gn - 1), x1 = std::clamp((int)std::floor((hi.x - mn.x) * ic), 0, im.gn - 1);
        const int y0 = std::clamp((int)std::floor((lo.y - mn.y) * ic), 0, im.gn - 1), y1 = std::clamp((int)std::floor((hi.y - mn.y) * ic), 0, im.gn - 1);
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++) ent.push_back({(uint32_t)(y * im.gn + x), k});
    }
    std::sort(ent.begin(), ent.end());
    im.start.assign(nc + 1, 0);
    im.items.resize(ent.size());
    for (size_t i = 0; i < ent.size(); i++) {
        im.start[ent[i].first + 1]++;
        im.items[i] = ent[i].second;
    }
    for (int c = 0; c < nc; c++) im.start[c + 1] += im.start[c];
}

// A wood vein k: its offset across the fibres at `a` along them (and the slope), wavy and randomly spaced.
float vein_y(uint32_t seed, int k, float sp, float a, float* slope) {
    const uint32_t h = mix(seed, (uint32_t)k * 2654435761u);
    const float A = sp * (0.1f + 0.3f * r01(h ^ 0x11u)), w = kTwoPi / (0.25f + 1.1f * r01(h ^ 0x22u));
    const float ph = kTwoPi * r01(h ^ 0x33u), ph2 = kTwoPi * r01(h ^ 0x44u);
    if (slope) *slope = A * w * (std::cos(a * w + ph) + 0.35f * 2.7f * std::cos(a * 2.7f * w + ph2));
    return ((float)k + 0.35f * (2.0f * r01(h ^ 0x55u) - 1.0f)) * sp + A * (std::sin(a * w + ph) + 0.35f * std::sin(a * 2.7f * w + ph2));
}

inline vec2 grain_dir(const ShellMaterial& m) { return vec2(std::cos(m.grain_angle), std::sin(m.grain_angle)); }

float zone_radius(const ShellImpact& im) {
    switch ((ShellPattern)im.kind) {
    case ShellPattern::Radial: return im.reach;
    case ShellPattern::Punch: return im.r * 1.6f;
    default: return im.r * 1.2f;
    }
}

} // namespace

// The lines of an impact at point c.
ShellImpact make_pattern(ShellPattern kind, float pattern_size, vec2 grain, vec2 c, float speed, float size, uint32_t seed) {
    struct {
        ShellPattern pattern;
        float pattern_size;
    } m{kind, pattern_size};
    ShellImpact im;
    im.c = c;
    im.speed = speed;
    im.seed = seed;
    im.kind = (uint8_t)m.pattern;
    // the zone grows with the speed and with the body that hit (a metal plug is about its size)
    const float s = std::clamp(std::sqrt(speed / 10.0f), 0.5f, 2.2f);
    if (m.pattern == ShellPattern::Punch && size > 0) im.r = std::clamp(size * (0.8f + 0.2f * s), 0.05f, 1.5f * m.pattern_size * s + 0.3f);
    else im.r = m.pattern_size * s * (size > 0 ? std::clamp(std::sqrt(size / 0.2f), 0.6f, 2.0f) : 1.0f);
    auto R = [&](uint32_t k) { return r01(mix(seed, k)); };
    uint32_t line = 0;
    auto poly = [&](const std::vector<vec2>& p, bool closed = false) {
        for (size_t i = 0; i + 1 < p.size(); i++) im.segs.push_back({p[i], p[i + 1], line});
        if (closed && p.size() > 2) im.segs.push_back({p.back(), p.front(), line});
        line++;
    };
    auto note = [&](vec2 p) { im.reach = std::max(im.reach, length(p - c)); };
    if (m.pattern == ShellPattern::Radial) {
        // a web: 7 .. 12 wavy radial cracks from a small crushed spot, 3 .. 5 polygonal rings between them (a
        // straight chord per sector, each corner at its own radius; the outer rings are not complete)
        const int nr = 7 + (int)(R(1) * 6.0f);
        const float base = R(2) * kTwoPi;
        float ang[12], amp[12], w[12], ph[12], len[12];
        for (int i = 0; i < nr; i++) {
            ang[i] = base + kTwoPi * ((float)i + 0.7f * (R(10 + i) - 0.5f)) / (float)nr;
            amp[i] = 0.03f + 0.07f * R(30 + i);
            w[i] = kTwoPi / (im.r * (0.4f + R(50 + i)));
            ph[i] = kTwoPi * R(70 + i);
            len[i] = im.r * (1.2f + 1.4f * R(90 + i));
        }
        auto ray = [&](int i, float rr) {
            const float th = ang[i] + amp[i] * std::sin(rr * w[i] + ph[i]);
            return c + vec2(std::cos(th), std::sin(th)) * rr;
        };
        const float r0 = im.r * 0.05f;
        for (int i = 0; i < nr; i++) {
            std::vector<vec2> p;
            for (int k = 0; k <= 12; k++) p.push_back(ray(i, r0 + (len[i] - r0) * (float)k / 12.0f));
            note(p.back());
            poly(p);
        }
        const int ng = 3 + (int)(R(3) * 3.0f);
        for (int j = 0; j < ng; j++) {
            const float rad = im.r * std::pow((float)(j + 1) / (float)ng, 1.4f) * (0.9f + 0.2f * R(150 + j));
            std::vector<vec2> v(nr);
            for (int i = 0; i < nr; i++) v[i] = ray(i, std::min(len[i], rad * (1.0f + 0.16f * (2.0f * R(200 + j * 16 + i) - 1.0f))));
            for (int i = 0; i < nr; i++) {
                if (j > 0 && R(400 + j * 16 + i) < 0.1f * (float)j) continue; // (a missing chord)
                im.segs.push_back({v[i], v[(i + 1) % nr], line});
            }
            line++;
        }
    } else if (m.pattern == ShellPattern::Punch) {
        // a ring round the point of contact (the plug), short radial tears out of it, sometimes petals inside
        const int nv = 22;
        const float wob = R(5) * kTwoPi;
        std::vector<vec2> ring(nv);
        for (int i = 0; i < nv; i++) {
            const float th = kTwoPi * (float)i / (float)nv;
            const float rr = im.r * (1.0f + 0.05f * (2.0f * R(20 + i) - 1.0f) + 0.06f * std::sin(2.0f * th + wob));
            ring[i] = c + vec2(std::cos(th), std::sin(th)) * rr;
            note(ring[i]);
        }
        poly(ring, true);
        const int nt = 3 + (int)(R(6) * 3.0f);
        for (int i = 0; i < nt; i++) {
            const float th = kTwoPi * ((float)i + 0.8f * (R(60 + i) - 0.5f)) / (float)nt;
            const float l = im.r * (1.5f + 0.9f * R(80 + i));
            std::vector<vec2> p;
            for (int k = 0; k <= 4; k++) {
                const float rr = im.r * 0.97f + (l - im.r * 0.97f) * (float)k / 4.0f;
                const float t2 = th + 0.12f * (2.0f * R(100 + i * 8 + k) - 1.0f);
                p.push_back(c + vec2(std::cos(t2), std::sin(t2)) * rr);
            }
            note(p.back());
            poly(p);
        }
        if (R(7) < 0.6f) {
            const int np = 4 + (int)(R(8) * 3.0f);
            const float b0 = R(9) * kTwoPi;
            for (int i = 0; i < np; i++) {
                const float th = b0 + kTwoPi * (float)i / (float)np;
                poly({c + vec2(std::cos(th), std::sin(th)) * (im.r * 0.12f), c + vec2(std::cos(th), std::sin(th)) * (im.r * 0.98f)});
            }
        }
    } else if (m.pattern == ShellPattern::Grain) {
        // the board splits along the fibres through the point of contact (and beside it), and breaks across there
        const vec2 g = grain, n(-g.y, g.x);
        auto split = [&](float off, float l0, float l1, uint32_t k) {
            std::vector<vec2> p;
            const float A = im.r * (0.04f + 0.06f * R(k)), w = kTwoPi / (im.r * (0.6f + R(k + 1))), ph = kTwoPi * R(k + 2);
            for (int i = 0; i <= 14; i++) {
                const float a = -l0 + (l0 + l1) * (float)i / 14.0f;
                p.push_back(c + g * a + n * (off + A * std::sin(a * w + ph)));
            }
            note(p.front());
            note(p.back());
            poly(p);
        };
        split(0.0f, im.r * (1.5f + 1.5f * R(11)), im.r * (1.5f + 1.5f * R(12)), 20);
        for (int side : {-1, 1})
            if (R(13 + side) < 0.7f) split((float)side * im.r * (0.3f + 0.3f * R(15 + side)), im.r * (0.5f + R(17 + side)), im.r * (0.5f + R(19 + side)), 30 + side * 5);
        std::vector<vec2> p; // across: a jagged break
        const float l = im.r * (0.8f + 0.6f * R(40));
        for (int i = 0; i <= 8; i++) p.push_back(c + n * (-l * 0.5f + l * (float)i / 8.0f) + g * (im.r * 0.12f * (2.0f * R(41 + i) - 1.0f)));
        note(p.front());
        note(p.back());
        poly(p);
    }
    build_grid(im);
    return im;
}

PatternLine pattern_nearest(const ShellImpact& im, vec2 x, float max_d) {
    PatternLine best;
    best.d = max_d;
    if (length(x - im.c) <= im.reach + max_d)
        visit(im, x - vec2(best.d), x + vec2(best.d), [&](const ShellImpact::Seg& s) {
            vec2 f;
            const float d = seg_dist(x, s.a, s.b, &f);
            if (d < best.d) {
                best.d = d;
                best.foot = f;
                best.tangent = normalize(s.b - s.a);
                best.id = s.line;
            }
        });
    if (best.id == kNone) best.d = 1e30f;
    return best;
}

float pattern_cross(const ShellImpact& im, vec2 xa, vec2 xb, float lo, float hi) {
    float best = -1, bd = 1e9f;
    const vec2 d = xb - xa;
    if (length((xa + xb) * 0.5f - im.c) > im.reach + length(d) * 0.5f) return -1;
    visit(im, vmin2(xa, xb), vmax2(xa, xb), [&](const ShellImpact::Seg& s) {
        const vec2 e = s.b - s.a;
        const float den = cross2(d, e);
        if (std::fabs(den) < 1e-14f) return;
        const vec2 w = s.a - xa;
        const float t = cross2(w, e) / den, u = cross2(w, d) / den;
        if (u >= 0.0f && u <= 1.0f && t >= lo && t <= hi && std::fabs(t - 0.5f) < bd) bd = std::fabs(t - 0.5f), best = t;
    });
    return best;
}

float pattern_zone_radius(const ShellImpact& im) { return zone_radius(im); }

namespace {
ShellImpact make_impact(const ShellMaterial& m, vec2 c, float speed, float size, uint32_t seed) {
    return make_pattern(m.pattern, m.pattern_size, grain_dir(m), c, speed, size, seed);
}
} // namespace

// ------------------------------------------------------------------------------------------------ queries
vec2 SoftBody::node_x(uint32_t v) const {
    if (v >= node_shells.size() || node_shells[v].empty()) return vec2(0);
    const uint32_t si = node_shells[v][0];
    const int c = corner_of(shells[si], v);
    return c >= 0 ? shell_x(si, c) : vec2(0);
}

PatternLine SoftBody::pattern_nearest(vec2 x, float max_d, int which) const {
    PatternLine best;
    best.d = max_d;
    if (which & 1)
        for (size_t i = 0; i < shell_impacts.size(); i++) {
            const ShellImpact& im = shell_impacts[i];
            if (length(x - im.c) > im.reach + max_d) continue;
            visit(im, x - vec2(best.d), x + vec2(best.d), [&](const ShellImpact::Seg& s) {
                vec2 f;
                const float d = seg_dist(x, s.a, s.b, &f);
                if (d < best.d) {
                    best.d = d;
                    best.foot = f;
                    best.tangent = normalize(s.b - s.a);
                    best.id = (uint32_t)(i + 1) << 20 | s.line;
                    best.vein = false;
                }
            });
        }
    if ((which & 2) && shell_mat.pattern == ShellPattern::Grain) {
        const vec2 g = grain_dir(shell_mat), n(-g.y, g.x);
        const float sp = std::max(0.005f, shell_mat.vein_spacing);
        const float a = dot(x, g), sx = dot(x, n);
        const int k0 = (int)std::floor(sx / sp);
        for (int k = k0 - 1; k <= k0 + 2; k++) {
            float sl;
            const float y = vein_y(pattern_seed, k, sp, a, &sl);
            const float q = std::sqrt(1.0f + sl * sl);
            const float d = std::fabs(sx - y) / q;
            if (d < best.d) {
                const vec2 N = (n - g * sl) * (1.0f / q);
                best.d = d;
                best.foot = x - N * ((sx - y) / q);
                best.tangent = normalize(g + n * sl);
                best.id = 0x80000000u | ((uint32_t)k & 0x7fffffffu);
                best.vein = true;
            }
        }
    }
    if (best.id == kNone) best.d = 1e30f;
    return best;
}

float SoftBody::pattern_cross(vec2 xa, vec2 xb, float lo, float hi, int which) const {
    float best = -1, bd = 1e9f;
    auto consider = [&](float t) {
        if (t >= lo && t <= hi && std::fabs(t - 0.5f) < bd) {
            bd = std::fabs(t - 0.5f);
            best = t;
        }
    };
    const vec2 d = xb - xa, mid = (xa + xb) * 0.5f;
    const float half = length(d) * 0.5f;
    if (which & 1)
        for (const ShellImpact& im : shell_impacts) {
            if (length(mid - im.c) > im.reach + half) continue;
            visit(im, vmin2(xa, xb), vmax2(xa, xb), [&](const ShellImpact::Seg& s) {
                const vec2 e = s.b - s.a;
                const float den = cross2(d, e);
                if (std::fabs(den) < 1e-14f) return;
                const vec2 w = s.a - xa;
                const float t = cross2(w, e) / den, u = cross2(w, d) / den;
                if (u >= 0.0f && u <= 1.0f) consider(t);
            });
        }
    if ((which & 2) && shell_mat.pattern == ShellPattern::Grain) {
        const vec2 g = grain_dir(shell_mat), n(-g.y, g.x);
        const float sp = std::max(0.005f, shell_mat.vein_spacing);
        const float sa = dot(xa, n), sb = dot(xb, n), aa = dot(xa, g), ab = dot(xb, g);
        const int k0 = (int)std::floor(std::min(sa, sb) / sp) - 1, k1 = std::min(k0 + 6, (int)std::floor(std::max(sa, sb) / sp) + 1);
        for (int k = k0; k <= k1; k++) {
            const float fa = sa - vein_y(pattern_seed, k, sp, aa, nullptr), fb = sb - vein_y(pattern_seed, k, sp, ab, nullptr);
            if ((fa < 0) == (fb < 0)) continue;
            float t = fa / (fa - fb);
            // (one secant step on the wavy vein)
            const float fm = (sa + (sb - sa) * t) - vein_y(pattern_seed, k, sp, aa + (ab - aa) * t, nullptr);
            if ((fa < 0) != (fm < 0)) t = std::fabs(fa - fm) > 1e-12f ? t * fa / (fa - fm) : t;
            else t = std::fabs(fm - fb) > 1e-12f ? t + (1.0f - t) * fm / (fm - fb) : t;
            consider(t);
        }
    }
    return best;
}

bool SoftBody::pattern_zone(vec2 x) const {
    if (shell_mat.pattern == ShellPattern::Grain) return true;
    for (const ShellImpact& im : shell_impacts)
        if (length(x - im.c) < zone_radius(im)) return true;
    return false;
}

float SoftBody::pattern_split(vec2 xa, vec2 xb, uint32_t h, float dev) const {
    dev = std::min(dev, 0.2f);
    if (!pattern_on() || dev <= 0.005f) return 0.5f;
    const float t = pattern_cross(xa, xb, 0.5f - dev, 0.5f + dev);
    if (t >= 0) return t;
    return pattern_zone((xa + xb) * 0.5f) ? 0.5f + std::min(0.12f, dev) * rand_pm1(h) : 0.5f;
}

// ------------------------------------------------------------------------------------------------ edge codes
void SoftBody::pattern_codes(uint32_t si) {
    PROFILE_ACCUM("Sheet pattern codes");
    Shell& s = shells[si];
    const ShellMaterial& M = shell_mat;
    s.line = 0;
    for (int e = 0; e < 3; e++) {
        s.es[e] = s.hs[e] = 64;
        if (!pattern_on()) continue;
        // (symmetric in the two ends: the neighbour across the edge gets the same codes)
        const vec2 xa = shell_x(si, e), xb = shell_x(si, nx(e));
        const vec2 d = xb - xa, mid = (xa + xb) * 0.5f;
        const float len = length(d);
        if (len < 1e-7f) continue;
        const vec2 u = d * (1.0f / len);
        float ft = 1, fh = 1;
        bool on = false;
        // the impacts' lines: edges along a line fold there, edges across it (or leaving it at an angle) are pulled
        // apart there, the other edges of the zone hold the pieces together
        bool near = false, zone = false;
        for (const ShellImpact& im : shell_impacts) {
            const float dc = length(mid - im.c);
            if (dc > im.reach + len) continue;
            near = true;
            zone |= dc < zone_radius(im);
        }
        if (near) {
            const float tol = 0.06f * len + 1e-4f;
            const PatternLine la = pattern_nearest(xa, tol, 1), lb = pattern_nearest(xb, tol, 1);
            const bool ona = la.d < tol, onb = lb.d < tol;
            if (ona && onb && la.id == lb.id && pattern_nearest(mid, 1.5f * tol, 1).d < 1.5f * tol) {
                fh = M.pattern_weak;
                on = true;
            } else if (pattern_cross(xa, xb, 0.02f, 0.98f, 1) >= 0 || (ona && std::fabs(dot(u, la.tangent)) < 0.8f) ||
                       (onb && std::fabs(dot(u, lb.tangent)) < 0.8f)) {
                ft = M.pattern_weak;
            } else if (zone) {
                ft = fh = M.pattern_strong;
            }
        }
        if (M.pattern == ShellPattern::Grain) {
            // fibres: pulled along them the edge holds grain_ratio times more than across them; a fold along them splits
            // them apart. Veins: weaker lines across the board
            const float c2 = dot(u, grain_dir(M)) * dot(u, grain_dir(M));
            const float lo = 2.0f / (1.0f + std::max(1.0f, M.grain_ratio)), hi = std::max(1.0f, M.grain_ratio) * lo;
            ft *= lo + (hi - lo) * c2;
            fh *= hi + (lo - hi) * c2;
            const float vt = 0.08f * len + 1e-4f;
            const PatternLine va = pattern_nearest(xa, vt, 2), vb = pattern_nearest(xb, vt, 2);
            if (va.d < vt && vb.d < vt && va.id == vb.id) {
                fh *= 0.7f;
                on = true;
            } else if (pattern_cross(xa, xb, 0.02f, 0.98f, 2) >= 0) {
                ft *= 0.75f;
            }
        }
        s.es[e] = (uint8_t)std::clamp((int)std::lround(ft * 64.0f), 4, 255);
        s.hs[e] = (uint8_t)std::clamp((int)std::lround(fh * 64.0f), 4, 255);
        if (on) s.line |= (uint8_t)(1u << e);
    }
    shell_springs(s, shell_material(s));
}

// ------------------------------------------------------------------------------------------------ impacts
void SoftBody::pattern_init(uint32_t seed) {
    pattern_seed = hash32(seed * 7919u + 17u);
    // metres per uv unit: least squares of (du sx)^2 + (dv sy)^2 = L^2 over the edges (the uv of a sheet is its
    // rest plane scaled)
    double a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
    for (const Shell& s : shells)
        for (int e = 0; e < 3; e++) {
            const vec2 du = s.uv[nx(e)] - s.uv[e];
            const double X = (double)du.x * du.x, Y = (double)du.y * du.y, L2 = (double)s.L0[e] * s.L0[e];
            a11 += X * X;
            a12 += X * Y;
            a22 += Y * Y;
            b1 += X * L2;
            b2 += Y * L2;
        }
    const double det = a11 * a22 - a12 * a12;
    shell_uvm = vec2(0);
    if (det > 1e-18 * std::max(1.0, a11 * a22)) {
        const double p = (b1 * a22 - b2 * a12) / det, q = (a11 * b2 - a12 * b1) / det;
        if (p > 0 && q > 0) shell_uvm = vec2((float)std::sqrt(p), (float)std::sqrt(q));
    }
    if (pattern_on())
        for (uint32_t si = 0; si < shells.size(); si++) pattern_codes(si);
}

void SoftBody::pattern_contact(uint32_t a, uint32_t b, uint32_t c, vec3 bary, float speed, float size, double time) {
    if (speed < shell_mat.pattern_speed || speed <= shell_hit.speed || !pattern_on()) return;
    shell_hit.x = node_x(a) * bary.x + node_x(b) * bary.y + node_x(c) * bary.z;
    shell_hit.speed = speed;
    shell_hit.size = size;
    shell_hit.time = time;
}

bool SoftBody::add_impact(vec2 x, float speed, float size, double time) {
    if (!pattern_on() || !allow_break || rigid || shell_mat.pattern == ShellPattern::None) return false;
    for (const ShellImpact& im : shell_impacts) {
        const float d = length(x - im.c);
        if (d < std::max(im.r, 0.1f)) return false; // (the same spot again)
        if (d < 3.0f * im.r && time - im.time < 0.25) return false; // (the same body still going through: its other points)
    }
    if (shell_impacts.size() >= 12) return false;
    const uint32_t seed = mix(pattern_seed, (uint32_t)shell_impacts.size() * 7919u ^ bits(x.x) ^ hash32(bits(x.y)));
    shell_impacts.push_back(make_impact(shell_mat, x, speed, size, seed));
    shell_impacts.back().time = time;
    const ShellImpact& im = shell_impacts.back();
    for (uint32_t si = 0; si < shells.size(); si++) {
        const Shell& s = shells[si];
        const vec2 cx = (shell_x(si, 0) + shell_x(si, 1) + shell_x(si, 2)) * (1.0f / 3.0f);
        const float lmax = std::max(s.L0[0], std::max(s.L0[1], s.L0[2]));
        if (length(cx - im.c) > im.reach + lmax) continue;
        pattern_codes(si);
        shk.dirty_shells.push_back(si);
    }
    pattern_passes = std::max(pattern_passes, shell_mat.max_level + 1);
    return true;
}

} // namespace bl::phys
