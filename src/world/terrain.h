// Terrain generation helpers + render data (heightfield mesh, splat texture, road decals).
#pragma once

#include "gfx/mesh.h"
#include "gfx/renderer.h"
#include "phys/static_world.h"

#include <memory>
#include <vector>

namespace bl {

class TerrainRender {
public:
    ~TerrainRender();
    // render_drop: optional per-vertex lowering of the render mesh (hidden under a road mesh)
    void build(const phys::Heightfield& hf, const std::vector<float>* render_drop = nullptr);
    void draw(Renderer& r) const;

private:
    GpuMesh m_mesh;
    GLuint m_splat = 0;
};

// Render mesh of a phys::RoadSurface: a dense ribbon (0.3 m x 0.12 m) that samples the same bed + detail
// function as the contacts, in chunks for culling; the texture's alpha edge dissolves into the grass.
class RoadRender {
public:
    void build(const phys::RoadSurface& road, const phys::Heightfield& hf, MaterialPtr mat);
    void draw(Renderer& r) const;
    // lowering of the terrain render mesh under the opaque part of the road
    static std::vector<float> terrain_drop(const phys::RoadSurface& road, const phys::Heightfield& hf);

private:
    void build_potholes(const phys::RoadSurface& road, const phys::Heightfield& hf);
    std::vector<std::unique_ptr<GpuMesh>> m_chunks;
    MaterialPtr m_mat;
    GpuMesh m_holes;          // mud decals in the potholes
    MaterialPtr m_hole_mat;
};

// Heightfield editing utilities used by scene builders.
namespace terrain_edit {
// fbm hills; `amp` metres, `scale` feature size in metres.
void add_noise(phys::Heightfield& hf, float amp, float scale, int octaves, uint32_t seed);
// Flatten a rectangle (world xz, rotated by yaw) to height `h` with a smooth border of `falloff` metres.
void flatten_rect(phys::Heightfield& hf, vec2 center, vec2 half, float yaw, float h, float falloff, int surface = -1);
void flatten_circle(phys::Heightfield& hf, vec2 center, float radius, float h, float falloff, int surface = -1);
// Carve a straight channel (chasm / river) along a segment: depth below current, half width, wall falloff.
void carve_channel(phys::Heightfield& hf, vec2 a, vec2 b, float bottom_h, float half_width, float falloff);
// Paint surface along a polyline with given half width (roads, tracks). Also smooths height along the road.
void paint_road(phys::Heightfield& hf, const std::vector<vec2>& pts, float half_width, int surface, float smooth = 0.6f);
void paint_circle(phys::Heightfield& hf, vec2 c, float r, int surface);
// Surface by slope/height rules (rock on slopes etc).
void auto_surfaces(phys::Heightfield& hf);
} // namespace terrain_edit

} // namespace bl
