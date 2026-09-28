// Loader for OGRE binary meshes (.mesh, MeshSerializer v1.10 .. v1.100) used by RoR content.
#pragma once

#include "core/math.h"

#include <string>
#include <vector>

namespace bl {

struct MeshVertex { // same layout as gfx Vertex
    vec3 pos;
    vec3 normal;
    vec2 uv;
};

struct OgreSubmesh {
    std::string material;              // material name referenced by the submesh
    std::vector<MeshVertex> vertices;  // expanded: shared geometry is copied into each submesh that uses it
    std::vector<uint32_t> indices;     // triangle list
    bool has_normals = false;
    bool has_uvs = false;
};

struct OgreMesh {
    std::vector<OgreSubmesh> submeshes;
    AABB bounds;
    std::string version;
};

// Returns false on failure (error message in *error). Normals are generated when missing.
bool load_ogre_mesh(const std::string& path, OgreMesh& out, std::string* error = nullptr);

} // namespace bl
