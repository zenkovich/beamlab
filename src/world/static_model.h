// Open assets of the scenes: PBR materials of assets/textures and models of assets/models (tools/fetch_assets.py).
#pragma once

#include "gfx/mesh.h"
#include "gfx/renderer.h"

#include <string>
#include <vector>

namespace bl {

// A material of assets/textures/<name>_color.jpg with its _normal, _rough and _ao maps (those there are); cached.
// Without the colour map: `fallback`.
MaterialPtr pbr_material(const std::string& name, MaterialPtr fallback = nullptr);

// A model of assets/models/<id>: its mesh (<id>.blm), its material (diffuse, normal, ARM), its bounds and its hull's
// planes in its own space (n . p <= d inside: <id>.hull). Cached for the program's life; nullptr when it is not there.
struct StaticModel {
    GpuMesh mesh;
    MaterialPtr mat;
    AABB bounds;
    std::vector<vec4> planes;
    int triangles = 0;
};
const StaticModel* static_model(const std::string& id);

} // namespace bl
