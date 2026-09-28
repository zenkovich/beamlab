// Decorative grass: instanced tufts in chunks, flattened by vehicles (wheels and a low body) in their direction of
// travel; flattened tufts slowly stand up again. Purely visual, no physics cost besides the crush queries.
#pragma once

#include "core/math.h"
#include "gfx/mesh.h"
#include "gfx/renderer.h"

#include <functional>
#include <memory>
#include <vector>

namespace bl {

class Vehicle;

class GrassField {
public:
    GrassField();
    ~GrassField();
    // position on the ground, scale ~1 (0.45 m tall), tint 0..1 (green .. dry)
    void add(vec3 p, float scale, float tint, uint32_t seed);
    void finalize();
    // flatten under the vehicles, recover over time
    void update(const std::vector<std::unique_ptr<Vehicle>>& vehicles, float dt, const std::function<float(float, float)>& ground);
    void draw(Renderer& r, vec3 cam_pos);
    size_t count() const { return m_count; }
    float view_distance = 90.0f;

private:
    struct Tuft {
        vec3 p;
        float yaw, scale;
        float bend = 0;     // 0 upright .. 1 flat
        float dir = 0;      // flatten direction (yaw of the push)
        float hold = 0;     // stays down this long before it starts to recover
        vec4 tint;
    };
    struct Chunk {
        std::vector<Tuft> tufts;
        AABB box;
        std::unique_ptr<InstanceBatch> batch;
        std::vector<InstanceData> inst;
        bool dirty = true;
        bool recovering = false;
    };
    void rebuild(Chunk& c);
    void crush(vec2 c, float radius, float dir, float hold);

    float m_cell = 16.0f;
    std::vector<std::unique_ptr<Chunk>> m_chunks;
    std::vector<std::pair<int64_t, int>> m_index; // cell key -> chunk (sorted after finalize)
    GpuMesh m_mesh;
    MaterialPtr m_mat;
    size_t m_count = 0;
    float m_recover_clock = 0;
};

} // namespace bl
