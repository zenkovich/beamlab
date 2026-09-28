#include "phys/road.h"
#include "core/jobs.h"

#include <algorithm>
#include <cmath>

namespace bl::phys {

void RoadSurface::build(const std::vector<vec2>& centre, float hw, float edge_w, uint8_t surf, uint32_t seed,
                        const std::vector<vec2>& washboard) {
    p = centre;
    half_width = hw;
    edge = edge_w;
    surface = surf;
    m_seed = seed;
    m_washboard = washboard;
    s.assign(p.size(), 0.0f);
    for (size_t i = 1; i < p.size(); i++) s[i] = s[i - 1] + bl::length(p[i] - p[i - 1]);
    // segment grid
    vec2 mn(1e30f), mx(-1e30f);
    for (vec2 q : p) {
        mn = vec2(std::min(mn.x, q.x), std::min(mn.y, q.y));
        mx = vec2(std::max(mx.x, q.x), std::max(mx.y, q.y));
    }
    const float reach = hw + edge + 1.0f;
    m_origin = mn - vec2(reach);
    m_gx = (int)((mx.x - mn.x + 2 * reach) / m_cell) + 1;
    m_gz = (int)((mx.y - mn.y + 2 * reach) / m_cell) + 1;
    m_grid.assign((size_t)m_gx * m_gz, {});
    for (int i = 0; i + 1 < (int)p.size(); i++) {
        vec2 a = p[i], b = p[i + 1];
        int x0 = (int)((std::min(a.x, b.x) - reach - m_origin.x) / m_cell), x1 = (int)((std::max(a.x, b.x) + reach - m_origin.x) / m_cell);
        int z0 = (int)((std::min(a.y, b.y) - reach - m_origin.y) / m_cell), z1 = (int)((std::max(a.y, b.y) + reach - m_origin.y) / m_cell);
        for (int z = std::max(0, z0); z <= std::min(m_gz - 1, z1); z++)
            for (int x = std::max(0, x0); x <= std::min(m_gx - 1, x1); x++) m_grid[(size_t)z * m_gx + x].push_back(i);
    }
    // potholes: plenty, often in clusters, mostly in and between the wheel tracks
    Rng rng(seed * 7919u + 13);
    m_holes.clear();
    m_hole_buckets.assign((size_t)(length() / 10.0f) + 2, {});
    auto add_hole = [&](RoadPothole h) {
        if (h.s < 2.0f || h.s > length() - 2.0f) return;
        m_hole_buckets[(size_t)(h.s / 10.0f)].push_back((int)m_holes.size());
        m_holes.push_back(h);
    };
    for (float d = 12.0f; d < length() - 12.0f; d += rng.range(1.5f, 9.0f)) {
        RoadPothole h;
        h.s = d;
        float side = rng.uniform() < 0.5f ? -1.0f : 1.0f;
        h.lat = rng.uniform() < 0.55f ? side * rng.range(0.45f, 1.15f) : rng.range(-(hw - 1.4f), hw - 1.4f);
        h.radius = rng.range(0.25f, 0.9f);
        h.depth = rng.range(0.05f, 0.18f) * (0.6f + 0.8f * h.radius);
        add_hole(h);
        int extra = rng.uniform() < 0.45f ? (int)rng.range(1.0f, 4.0f) : 0; // clusters
        for (int k = 0; k < extra; k++) {
            RoadPothole h2 = h;
            h2.s += rng.range(-1.8f, 1.8f);
            h2.lat = clampf(h2.lat + rng.range(-0.9f, 0.9f), -(hw - 1.0f), hw - 1.0f);
            h2.radius *= rng.range(0.4f, 0.9f);
            h2.depth *= rng.range(0.6f, 1.1f);
            add_hole(h2);
        }
    }
    // bumps: stones and humps sticking out of the gravel
    m_bumps.clear();
    m_bump_buckets.assign(m_hole_buckets.size(), {});
    for (float d = 8.0f; d < length() - 8.0f; d += rng.range(2.0f, 12.0f)) {
        RoadPothole b{d, rng.range(-(hw - 0.9f), hw - 0.9f), rng.range(0.15f, 0.45f), rng.range(0.03f, 0.08f)};
        m_bump_buckets[(size_t)(b.s / 10.0f)].push_back((int)m_bumps.size());
        m_bumps.push_back(b);
    }
    // extra washboard stretches anywhere on the road
    for (float d = 40.0f; d < length() - 40.0f; d += rng.range(60.0f, 160.0f))
        if (rng.uniform() < 0.5f) m_washboard.push_back({d, d + rng.range(15.0f, 35.0f)});
    // bake the shape (contacts sample it several thousand times per frame; the render mesh reads the same grid)
    const float W = hw + edge;
    m_ns = (int)(length() / m_step) + 2;
    m_nl = (int)(2 * W / m_step) + 2;
    m_grid_h.clear();
    std::vector<float> baked((size_t)m_ns * m_nl);
    JobSystem::get().parallel_for(m_ns, 64, [&](int r0, int r1, int) {
        for (int r = r0; r < r1; r++)
            for (int k = 0; k < m_nl; k++) baked[(size_t)r * m_nl + k] = detail_exact(r * m_step, -W + k * m_step);
    });
    m_grid_h.swap(baked);
}

int RoadSurface::segment_at(float d) const {
    int i = (int)(std::upper_bound(s.begin(), s.end(), d) - s.begin()) - 1;
    return std::clamp(i, 0, (int)p.size() - 2);
}

vec2 RoadSurface::point(float d) const {
    int i = segment_at(d);
    float t = clampf((d - s[i]) / std::max(1e-4f, s[i + 1] - s[i]), 0, 1);
    return p[i] + (p[i + 1] - p[i]) * t;
}

vec2 RoadSurface::tangent(float d) const {
    int i = segment_at(d);
    return normalize(p[i + 1] - p[i]);
}

bool RoadSurface::locate(float x, float z, float& out_s, float& out_lat) const {
    if (p.size() < 2) return false;
    int cx = (int)((x - m_origin.x) / m_cell), cz = (int)((z - m_origin.y) / m_cell);
    if (cx < 0 || cz < 0 || cx >= m_gx || cz >= m_gz) return false;
    const auto& cell = m_grid[(size_t)cz * m_gx + cx];
    if (cell.empty()) return false;
    vec2 q(x, z);
    float best = 1e30f;
    for (int i : cell) {
        vec2 a = p[i], ab = p[i + 1] - p[i];
        float l2 = std::max(1e-8f, dot(ab, ab));
        float t = clampf(dot(q - a, ab) / l2, 0, 1);
        vec2 d = q - (a + ab * t);
        float dd = dot(d, d);
        if (dd < best) {
            best = dd;
            out_s = s[i] + (s[i + 1] - s[i]) * t;
            vec2 tn = ab * (1.0f / std::sqrt(l2));
            out_lat = dot(d, vec2(tn.y, -tn.x));
        }
    }
    return std::fabs(out_lat) < half_width + edge;
}

float RoadSurface::bilinear(float d, float lat, float* dds, float* ddl) const {
    const float W = half_width + edge;
    float fs = clampf(d / m_step, 0.0f, (float)m_ns - 1.001f), fl = (lat + W) / m_step;
    if (fl < 0 || fl >= m_nl - 1) {
        if (dds) *dds = *ddl = 0;
        return 0.0f;
    }
    int is = (int)fs, il = (int)fl;
    float ts = fs - is, tl = fl - il;
    float h00 = grid(is, il), h01 = grid(is, il + 1), h10 = grid(is + 1, il), h11 = grid(is + 1, il + 1);
    float a = h00 + (h01 - h00) * tl, b = h10 + (h11 - h10) * tl;
    if (dds) {
        *dds = (b - a) / m_step;
        *ddl = ((h01 - h00) * (1 - ts) + (h11 - h10) * ts) / m_step;
    }
    return a + (b - a) * ts;
}

float RoadSurface::detail(float d, float lat) const {
    if (m_grid_h.empty()) return detail_exact(d, lat);
    return bilinear(d, lat, nullptr, nullptr);
}

float RoadSurface::rut_center(float d, float side) const {
    const uint32_t rs = m_seed + (side < 0 ? 11u : 17u);
    return 0.8f + 0.18f * value_noise2(d * 0.045f, side, rs) + 0.06f * value_noise2(d * 0.3f, side, rs + 1);
}

float RoadSurface::detail_exact(float d, float lat) const {
    const float a = std::fabs(lat), w = half_width;
    if (a >= w + edge) return 0.0f;
    // camber: crown in the middle, the verges lower
    float h = 0.045f * (1.0f - sqr(std::min(a / w, 1.0f))) - 0.02f;
    // two wheel ruts: each wanders across the road, deepens and fades on its own (0-9 cm), width varies
    const float side = lat < 0 ? -1.0f : 1.0f;
    const uint32_t rs = m_seed + (side < 0 ? 11u : 17u);
    float rut_c = rut_center(d, side);
    float rut_w = 0.2f + 0.07f * value_noise2(d * 0.08f, side, rs + 2);
    float rut_d = std::max(0.0f, 0.05f + 0.04f * value_noise2(d * 0.03f, side, rs + 3) + 0.015f * value_noise2(d * 0.25f, side, rs + 4));
    h -= rut_d * std::exp(-sqr((a - rut_c) / rut_w));
    // loose gravel: a ridge between the ruts, ridges along the outside of the ruts, a berm at the verge
    h += 0.018f * std::exp(-sqr(lat / 0.3f)) + 0.012f * std::exp(-sqr((a - rut_c - rut_w * 1.6f) / 0.15f)) +
         0.03f * std::exp(-sqr((a - (w - 0.15f)) / 0.3f));
    // undulations: long waves, bumps, and chatter
    h += 0.06f * value_noise2(d * 0.06f, lat * 0.1f, m_seed + 5) + 0.04f * value_noise2(d * 0.18f, lat * 0.25f, m_seed + 2) +
         0.015f * value_noise2(d * 0.9f, lat * 1.1f, m_seed + 3);
    // washboard (corrugations) in the braking zones before corners
    for (vec2 r : m_washboard)
        if (d > r.x && d < r.y) {
            float env = smoothstepf(r.x, r.x + 6.0f, d) * (1.0f - smoothstepf(r.y - 6.0f, r.y, d));
            h += 0.016f * env * std::sin(d * (2.0f * kPi / 0.72f) + 0.3f * std::sin(lat * 1.3f));
        }
    // potholes: smooth bowls with a small lip
    int b = (int)(d / 10.0f);
    for (int k = std::max(0, b - 1); k <= std::min((int)m_hole_buckets.size() - 1, b + 1); k++)
        for (int hi : m_hole_buckets[(size_t)k]) {
            const RoadPothole& ph = m_holes[(size_t)hi];
            float r2 = sqr(d - ph.s) + sqr(lat - ph.lat);
            float R = ph.radius * 1.25f;
            if (r2 >= R * R) continue;
            float q = std::sqrt(r2) / ph.radius;
            if (q < 1.0f) h -= ph.depth * sqr(1.0f - q * q);
            else h += ph.depth * 0.15f * std::sin((q - 1.0f) / 0.25f * kPi); // lip
        }
    for (int k = std::max(0, b - 1); k <= std::min((int)m_bump_buckets.size() - 1, b + 1); k++)
        for (int bi : m_bump_buckets[(size_t)k]) {
            const RoadPothole& bp = m_bumps[(size_t)bi];
            float r2 = sqr(d - bp.s) + sqr(lat - bp.lat);
            if (r2 < bp.radius * bp.radius) h += bp.depth * sqr(1.0f - r2 / (bp.radius * bp.radius));
        }
    // the verge never dips below the bed (the terrain mesh is only lowered under the inner road), then fades out
    h = lerpf(h, std::max(h, 0.0f), smoothstepf(w - 1.1f, w - 0.5f, a));
    return h * (1.0f - smoothstepf(w - 0.25f, w + edge, a));
}

bool RoadSurface::sample(float x, float z, float& dh, vec2& grad, float& lat_out) const {
    float d, lat;
    if (!locate(x, z, d, lat)) return false;
    float ds, dl;
    dh = bilinear(d, lat, &ds, &dl);
    vec2 t = tangent(d), l(t.y, -t.x);
    grad = t * ds + l * dl;
    lat_out = lat;
    return true;
}

} // namespace bl::phys
