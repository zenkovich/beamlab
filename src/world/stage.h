// Imported rally stage (assets/stages/<name>/stage.bin, written by tools/fetch_rbr_stage.py from a Richard Burns
// Rally community stage): static render meshes with their textures, the ground as a collision heightfield,
// props (their own meshes on light bodies), round bales and the driveline.
#pragma once

#include "gfx/mesh.h"
#include "gfx/renderer.h"

#include <memory>
#include <string>
#include <vector>

namespace bl {

struct StageBundle {
    enum Kind : uint8_t { GROUND = 0, OBJECT = 1, ALPHA = 2 };
    enum TexFlag : uint8_t { TEX_ALPHA = 1, TEX_FOLIAGE = 2, TEX_GROUND = 4 };
    struct Batch {
        uint8_t kind = GROUND, flags = 0; // flags & 1: no backface culling
        int tex1 = -1, tex2 = -1;
        std::vector<VertexC> v;
        std::vector<uint32_t> idx;
    };
    struct Texture {
        std::string file;
        uint8_t flags = 0;
    };
    struct Template {
        std::string name;
        uint32_t kind = 0; // RBR object kind (sign, banner, ...)
        float mass = 40;
        vec3 hull_min, hull_max;
        std::vector<Batch> meshes;
    };
    struct Prop {
        int tmpl;
        mat3 rot;
        vec3 pos;
    };
    struct Bale {
        vec3 pos;
        float yaw, radius, height;
    };
    struct StaticCollider { // tree trunks, stumps, walls, bushes of the stage
        vec3 centre, half;
        mat3 rot;           // columns: box axes
        bool yielding;      // bendable trees and bushes
        bool deck;          // bridge deck over another part of the road (the heightfield holds the lower level)
    };

    std::string title, credits;
    std::vector<Texture> textures;
    // ground collision: the top surface of the stage's collision mesh in tiles along the road (see
    // phys::Heightfield); small gaps are filled, where the collision mesh ends a 3 m step walls the stage off
    int nx = 0, nz = 0;
    float cell = 0.25f;
    vec2 origin;
    int tile = 64;
    std::vector<std::pair<int, int>> tiles;
    std::vector<float> tile_height;   // tile * tile per tile
    std::vector<uint8_t> tile_surface;
    std::vector<Batch> batches;
    std::vector<Template> templates;
    std::vector<Prop> props;
    std::vector<Bale> bales;
    std::vector<StaticCollider> statics;
    std::vector<vec3> line; // driveline, ~1 m spacing
    float line_length = 0;
    vec3 start;
    float heading = 0;
    float clock_start = -1, clock_finish = -1; // driveline locations of the start / finish (< 0: unknown)

    bool load(const std::string& path, std::string& error);
};

// GPU side of a bundle: one mesh per batch (the batches are spatial blocks, culled by their bounds).
class StageScenery {
public:
    bool build(const StageBundle& b, const std::string& dir);
    void draw(Renderer& r) const;
    // meshes of a prop template (for RigidVisual parts)
    struct Part {
        std::unique_ptr<GpuMesh> mesh;
        MaterialPtr mat;
    };
    std::vector<std::vector<Part>> templates;
    size_t triangles = 0;

private:
    MaterialPtr material(const StageBundle& b, const StageBundle::Batch& batch);
    std::vector<Part> m_items;
    std::vector<TexturePtr> m_tex;
    std::vector<std::pair<uint64_t, MaterialPtr>> m_mats;
};

} // namespace bl
