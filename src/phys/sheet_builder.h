// Sheets of triangle elements (phys::Shell) of different shapes: the 4-8 grid of the rectangle (the pattern
// bisection keeps), clipped to a shape, its border nodes moved onto the outline, then optionally bent into a
// cylinder (a half-pipe, a tube section) or a spherical cap (a dome, a bowl).
#pragma once

#include "phys/softbody.h"

namespace bl::phys {

enum class SheetShape : uint8_t { Rect, Disc, Ring, Triangle, LShape };

struct SheetMeshDesc {
    vec3 center;
    vec3 u{1, 0, 0}, v{0, 1, 0};  // sheet axes (front normal = u x v)
    float width = 2.0f, height = 2.0f;
    int nu = 14, nv = 14;          // grid nodes per side (square cells keep the triangles right isosceles)
    SheetShape shape = SheetShape::Rect; // (ellipse, ring, triangle inscribed in width x height; L = without the top right quarter)
    float hole = 0.45f;            // Ring: inner size / outer size
    float curve = 0;               // > 0: bent around an axis along v at this radius (the sheet's u runs round it)
    float dome = 0;                // > 0: a spherical cap of this radius (the rim bends back along -normal)
    int clamp = 0;                 // 0 free, 1 the outer border fixed, 2 a gate (top + sides), 3 the top only, 4 two top corners
    float uv_scale = 1.0f;
};

// Adds the nodes and shells to `b` (masses are left to SoftBody::finalize_shells). Returns the area of the sheet.
float add_sheet_mesh(SoftBody& b, const SheetMeshDesc& d);

} // namespace bl::phys
