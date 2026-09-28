// Detailed road surface over the terrain heightfield: the heightfield carries the smooth road bed, this adds the
// small-scale shape (camber, wheel ruts, bumps, washboard, potholes) analytically, so it has any resolution for
// the physics and the render mesh samples exactly the same function.
#pragma once

#include "core/math.h"

#include <cstdint>
#include <vector>

namespace bl::phys {

struct RoadPothole {
    float s, lat, radius, depth;
};

class RoadSurface {
public:
    // centre: polyline (xz) sampled densely (~1 m). washboard: (s0, s1) ranges with corrugations (braking zones).
    void build(const std::vector<vec2>& centre, float half_width, float edge, uint8_t surface, uint32_t seed,
               const std::vector<vec2>& washboard);
    // Nearest point on the centre line: arc length s and signed lateral offset (+ = left of the driving direction).
    bool locate(float x, float z, float& s, float& lat) const;
    // Height offset over the road bed at (s, lat); 0 beyond the verge. Baked into a 0.1 m grid at build time.
    float detail(float s, float lat) const;
    // the analytic shape the grid is baked from
    float detail_exact(float s, float lat) const;
    // Height offset + its world-space slope (d/dx, d/dz) at (x, z). Returns false off the road.
    bool sample(float x, float z, float& dh, vec2& grad, float& lat_out) const;
    // centre of the wheel rut on one side (side = -1 right, +1 left): the ruts wander across the road
    float rut_center(float s, float side) const;
    const std::vector<RoadPothole>& holes() const { return m_holes; }
    vec2 point(float s) const;
    vec2 tangent(float s) const;
    vec2 left(float s) const {
        vec2 t = tangent(s);
        return {t.y, -t.x};
    }

    std::vector<vec2> p;
    std::vector<float> s;
    float half_width = 3.5f;
    float edge = 1.2f;       // verge: detail fades out, the render mesh fades into the grass
    uint8_t surface = 0;
    float max_raise = 0.2f; // upper bound of detail() above the bed (contact culling margin)
    float length() const { return s.empty() ? 0.0f : s.back(); }

private:
    int segment_at(float d) const;
    std::vector<RoadPothole> m_holes;
    std::vector<std::vector<int>> m_hole_buckets; // per 10 m of s
    std::vector<RoadPothole> m_bumps;             // stones / humps (depth = height)
    std::vector<std::vector<int>> m_bump_buckets;
    std::vector<vec2> m_washboard;
    // baked detail: rows along s, columns across (lat from -(half_width + edge))
    float m_step = 0.1f;
    int m_ns = 0, m_nl = 0;
    std::vector<float> m_grid_h;
    float grid(int is, int il) const { return m_grid_h[(size_t)is * m_nl + il]; }
    float bilinear(float s, float lat, float* ds, float* dl) const;
    float m_cell = 10.0f;
    vec2 m_origin;
    int m_gx = 0, m_gz = 0;
    std::vector<std::vector<int>> m_grid; // segment indices per cell
    uint32_t m_seed = 1;
};

} // namespace bl::phys
