// Fracture patterns of the triangle-element sheets (shell_pattern.cpp): the lines a material's cracks follow.
//
// Glass (Radial): a web around the point of impact, wavy radial cracks and polygonal rings between them. Metals
// (Punch): a ring round the point of impact (the plug) with short radial tears out of it. Wood (Grain): the fibres
// run one way, the edges across them are weak (it splits along the grain) and random wavy veins inside are weaker
// still; an impact splits it along the grain through the point of contact. The lines live in the sheet's material
// plane (its uv in metres, SoftBody::shell_uvm), so they stay on the material however the sheet bends.
//
// The lines are laid when a contact faster than ShellMaterial::pattern_speed hits the sheet. Their triangles are
// refined (a level per substep), each bisection puts its new node on a line the split edge crosses (or, in the
// zone, a little off the midpoint: the crack edges are not straight), the edges across a line are weakened and the
// other edges of the zone strengthened, so the cracks run along the lines and the pieces between them hold together.
#pragma once

#include "core/math.h"

#include <cstdint>
#include <vector>

namespace bl::phys {

enum class ShellPattern : uint8_t { None, Radial, Punch, Grain };

struct ShellImpact {
    vec2 c;                 // point of impact in the material plane (m)
    float r = 0;            // zone radius: the pieces between the lines inside it are strengthened
    float reach = 0;        // every line is within this distance of c
    float speed = 0;        // impact speed (m/s)
    double time = 0;        // when (World::time)
    uint32_t seed = 0;
    uint8_t kind = 0;       // ShellPattern
    uint8_t level = 0;      // the lines are refined up to this level so far
    struct Seg {
        vec2 a, b;
        uint32_t line;      // polyline id (segments of one ray, ring or tear)
    };
    std::vector<Seg> segs;
    // grid over the segments (queries look at the cells near the point only)
    vec2 org;
    float cell = 1.0f;
    int gn = 0;
    std::vector<uint32_t> start, items; // cell -> items[start[cell] .. start[cell + 1])
};

// the nearest line to a point
struct PatternLine {
    float d = 1e30f;        // distance (m)
    vec2 foot;              // nearest point on the line
    vec2 tangent{1, 0};
    uint32_t id = 0xffffffffu;
    bool vein = false;      // a wood vein (a weaker line than an impact's)
};

// The lines of an impact at c (m/s `speed`, by a body of radius `size`, 0: unknown) of a pattern `kind` whose zone is
// `pattern_size` at 10 m/s (Grain: the fibres along `grain`) - the sheets' (SoftBody::add_impact) and the triangle
// elements' (FemFrame) generator; the queries on one impact's lines: the nearest (within max_d, else d 1e30), the
// crossing of segment xa-xb nearest its middle (its t within [lo, hi], -1: none), the zone between the lines
ShellImpact make_pattern(ShellPattern kind, float pattern_size, vec2 grain, vec2 c, float speed, float size, uint32_t seed);
PatternLine pattern_nearest(const ShellImpact& im, vec2 x, float max_d);
float pattern_cross(const ShellImpact& im, vec2 xa, vec2 xb, float lo, float hi);
float pattern_zone_radius(const ShellImpact& im);

} // namespace bl::phys
