// GPU meshes: static / dynamic vertex+index buffers and instance buffers.
#pragma once

#include "core/math.h"
#include "gfx/gl.h"

#include <cstdint>
#include <vector>

namespace bl {

struct Vertex {
    vec3 pos;
    vec3 normal;
    vec2 uv;
};

// Vertex of imported static scenery: baked vertex colour (RGBA8) and a second uv set for a second texture layer.
struct VertexC {
    vec3 pos;
    vec3 normal;
    vec2 uv;
    vec2 uv2;
    uint32_t color;
};

// Per-instance data for instanced draws: affine transform rows + color.
struct InstanceData {
    vec4 row0, row1, row2; // model matrix rows (3x4)
    vec4 color;
    static InstanceData from(const mat4& m, vec4 color) {
        return {{m.c[0].x, m.c[1].x, m.c[2].x, m.c[3].x},
                {m.c[0].y, m.c[1].y, m.c[2].y, m.c[3].y},
                {m.c[0].z, m.c[1].z, m.c[2].z, m.c[3].z},
                color};
    }
};

// Attribute locations shared by all shaders.
enum : GLuint { ATTR_POS = 0, ATTR_NORMAL = 1, ATTR_UV = 2, ATTR_COLOR = 3, ATTR_I_ROW0 = 4, ATTR_I_ROW1 = 5, ATTR_I_ROW2 = 6, ATTR_I_COLOR = 7, ATTR_UV2 = 8 };

class GpuMesh {
public:
    GpuMesh() = default;
    ~GpuMesh();
    GpuMesh(const GpuMesh&) = delete;
    GpuMesh& operator=(const GpuMesh&) = delete;

    void create(const Vertex* v, int nv, const uint32_t* idx, int ni, bool dynamic = false);
    void create(const std::vector<Vertex>& v, const std::vector<uint32_t>& idx, bool dynamic = false) {
        create(v.data(), (int)v.size(), idx.data(), (int)idx.size(), dynamic);
    }
    // static mesh with vertex colours + two uv sets (draw with a Material that has vertex_color set)
    void create(const VertexC* v, int nv, const uint32_t* idx, int ni);
    // Replace vertex data (buffer grows when needed).
    void update_vertices(const Vertex* v, int nv);
    void update_vertices(const Vertex* v, int nv, const AABB& known_bounds); // skips the bounds pass
    void update_indices(const uint32_t* idx, int ni);
    void destroy();

    void draw() const { draw_range(0, m_index_count); }
    void draw_range(int first_index, int count) const;
    void draw_instanced(GLuint instance_vao, int instances) const;

    bool valid() const { return m_vao != 0; }
    int index_count() const { return m_index_count; }
    int vertex_count() const { return m_vertex_count; }
    GLuint vao() const { return m_vao; }
    GLuint vbo() const { return m_vbo; }
    GLuint ibo() const { return m_ibo; }
    AABB bounds;

private:
    GLuint m_vao = 0, m_vbo = 0, m_ibo = 0;
    int m_index_count = 0, m_vertex_count = 0;
    int m_vbo_capacity = 0, m_ibo_capacity = 0;
    bool m_dynamic = false;
};

// Instance buffer bound together with a mesh into its own VAO.
class InstanceBatch {
public:
    ~InstanceBatch();
    void init(const GpuMesh* mesh);
    void upload(const InstanceData* inst, int count);
    void upload(const std::vector<InstanceData>& v) { upload(v.data(), (int)v.size()); }
    void draw() const;
    int count() const { return m_count; }
    const GpuMesh* mesh() const { return m_mesh; }

private:
    const GpuMesh* m_mesh = nullptr;
    GLuint m_vao = 0, m_buf = 0;
    int m_count = 0, m_capacity = 0;
};

// Procedural primitives (unit sized, centered).
void make_box(std::vector<Vertex>& v, std::vector<uint32_t>& i, vec3 half_extent = vec3(0.5f));
void make_sphere(std::vector<Vertex>& v, std::vector<uint32_t>& i, float radius, int seg = 24, int rings = 16);
// Cylinder along +Y from y=0 to y=1, radius 1 at bottom, `top_radius` at top.
void make_cylinder(std::vector<Vertex>& v, std::vector<uint32_t>& i, int seg, float top_radius = 1.0f, bool caps = true);
void make_cone(std::vector<Vertex>& v, std::vector<uint32_t>& i, int seg);
void compute_normals(std::vector<Vertex>& v, const std::vector<uint32_t>& idx);

} // namespace bl
