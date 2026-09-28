#include "gfx/mesh.h"

#include <cstddef>

namespace bl {

GpuMesh::~GpuMesh() { destroy(); }

void GpuMesh::destroy() {
    if (m_vao) glDeleteVertexArrays(1, &m_vao);
    if (m_vbo) glDeleteBuffers(1, &m_vbo);
    if (m_ibo) glDeleteBuffers(1, &m_ibo);
    m_vao = m_vbo = m_ibo = 0;
    m_index_count = m_vertex_count = m_vbo_capacity = m_ibo_capacity = 0;
}

static void setup_vertex_attribs() {
    glEnableVertexAttribArray(ATTR_POS);
    glVertexAttribPointer(ATTR_POS, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, pos));
    glEnableVertexAttribArray(ATTR_NORMAL);
    glVertexAttribPointer(ATTR_NORMAL, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(ATTR_UV);
    glVertexAttribPointer(ATTR_UV, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, uv));
}

void GpuMesh::create(const Vertex* v, int nv, const uint32_t* idx, int ni, bool dynamic) {
    destroy();
    m_dynamic = dynamic;
    glGenVertexArrays(1, &m_vao);
    glBindVertexArray(m_vao);
    glGenBuffers(1, &m_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(std::max(nv, 1) * sizeof(Vertex)), v, dynamic ? GL_STREAM_DRAW : GL_STATIC_DRAW);
    m_vbo_capacity = std::max(nv, 1);
    glGenBuffers(1, &m_ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(std::max(ni, 1) * sizeof(uint32_t)), idx, GL_STATIC_DRAW);
    m_ibo_capacity = std::max(ni, 1);
    setup_vertex_attribs();
    glBindVertexArray(0);
    m_vertex_count = nv;
    m_index_count = ni;
    bounds = AABB();
    if (v)
        for (int i = 0; i < nv; i++) bounds.add(v[i].pos);
}

void GpuMesh::create(const VertexC* v, int nv, const uint32_t* idx, int ni) {
    destroy();
    glGenVertexArrays(1, &m_vao);
    glBindVertexArray(m_vao);
    glGenBuffers(1, &m_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(std::max(nv, 1) * sizeof(VertexC)), v, GL_STATIC_DRAW);
    m_vbo_capacity = std::max(nv, 1);
    glGenBuffers(1, &m_ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(std::max(ni, 1) * sizeof(uint32_t)), idx, GL_STATIC_DRAW);
    m_ibo_capacity = std::max(ni, 1);
    glEnableVertexAttribArray(ATTR_POS);
    glVertexAttribPointer(ATTR_POS, 3, GL_FLOAT, GL_FALSE, sizeof(VertexC), (void*)offsetof(VertexC, pos));
    glEnableVertexAttribArray(ATTR_NORMAL);
    glVertexAttribPointer(ATTR_NORMAL, 3, GL_FLOAT, GL_FALSE, sizeof(VertexC), (void*)offsetof(VertexC, normal));
    glEnableVertexAttribArray(ATTR_UV);
    glVertexAttribPointer(ATTR_UV, 2, GL_FLOAT, GL_FALSE, sizeof(VertexC), (void*)offsetof(VertexC, uv));
    glEnableVertexAttribArray(ATTR_UV2);
    glVertexAttribPointer(ATTR_UV2, 2, GL_FLOAT, GL_FALSE, sizeof(VertexC), (void*)offsetof(VertexC, uv2));
    glEnableVertexAttribArray(ATTR_COLOR);
    glVertexAttribPointer(ATTR_COLOR, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(VertexC), (void*)offsetof(VertexC, color));
    glBindVertexArray(0);
    m_vertex_count = nv;
    m_index_count = ni;
    bounds = AABB();
    for (int i = 0; i < nv; i++) bounds.add(v[i].pos);
}

void GpuMesh::update_vertices(const Vertex* v, int nv, const AABB& known_bounds) {
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    if (nv > m_vbo_capacity) m_vbo_capacity = nv + nv / 4;
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(m_vbo_capacity * sizeof(Vertex)), nullptr, GL_STREAM_DRAW); // orphan
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(nv * sizeof(Vertex)), v);
    m_vertex_count = nv;
    bounds = known_bounds;
}

void GpuMesh::update_vertices(const Vertex* v, int nv) {
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    if (nv > m_vbo_capacity) {
        m_vbo_capacity = nv + nv / 4;
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(m_vbo_capacity * sizeof(Vertex)), nullptr, GL_STREAM_DRAW);
    } else {
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(m_vbo_capacity * sizeof(Vertex)), nullptr, GL_STREAM_DRAW); // orphan
    }
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(nv * sizeof(Vertex)), v);
    m_vertex_count = nv;
    // keep bounds current for culling (dynamic meshes are in world space)
    AABB b;
    for (int i = 0; i < nv; i++) b.add(v[i].pos);
    bounds = b;
}

void GpuMesh::update_indices(const uint32_t* idx, int ni) {
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
    if (ni > m_ibo_capacity) m_ibo_capacity = ni + ni / 4;
    // (orphaned: the previous frame may still be drawing from the old storage; writing into it would wait for the GPU)
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(m_ibo_capacity * sizeof(uint32_t)), nullptr, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, (GLsizeiptr)(ni * sizeof(uint32_t)), idx);
    glBindVertexArray(0);
    m_index_count = ni;
}

void GpuMesh::draw_range(int first_index, int count) const {
    if (!m_vao || count <= 0) return;
    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, (void*)(size_t)(first_index * sizeof(uint32_t)));
}

void GpuMesh::draw_instanced(GLuint instance_vao, int instances) const {
    if (!instance_vao || instances <= 0 || m_index_count <= 0) return;
    glBindVertexArray(instance_vao);
    glDrawElementsInstanced(GL_TRIANGLES, m_index_count, GL_UNSIGNED_INT, nullptr, instances);
}

// ------------------------------------------------------------------ InstanceBatch
InstanceBatch::~InstanceBatch() {
    if (m_vao) glDeleteVertexArrays(1, &m_vao);
    if (m_buf) glDeleteBuffers(1, &m_buf);
}

void InstanceBatch::init(const GpuMesh* mesh) {
    m_mesh = mesh;
    if (!m_vao) glGenVertexArrays(1, &m_vao);
    if (!m_buf) glGenBuffers(1, &m_buf);
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, mesh->vbo());
    setup_vertex_attribs();
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->ibo());
    glBindBuffer(GL_ARRAY_BUFFER, m_buf);
    if (m_capacity == 0) {
        m_capacity = 64;
        glBufferData(GL_ARRAY_BUFFER, m_capacity * sizeof(InstanceData), nullptr, GL_STREAM_DRAW);
    }
    for (int k = 0; k < 4; k++) {
        GLuint a = ATTR_I_ROW0 + k;
        glEnableVertexAttribArray(a);
        glVertexAttribPointer(a, 4, GL_FLOAT, GL_FALSE, sizeof(InstanceData), (void*)(size_t)(k * sizeof(vec4)));
        glVertexAttribDivisor(a, 1);
    }
    glBindVertexArray(0);
}

void InstanceBatch::upload(const InstanceData* inst, int count) {
    m_count = count;
    if (count <= 0) return;
    glBindBuffer(GL_ARRAY_BUFFER, m_buf);
    if (count > m_capacity) {
        m_capacity = count + count / 2;
        glBufferData(GL_ARRAY_BUFFER, m_capacity * sizeof(InstanceData), nullptr, GL_STREAM_DRAW);
    } else {
        glBufferData(GL_ARRAY_BUFFER, m_capacity * sizeof(InstanceData), nullptr, GL_STREAM_DRAW);
    }
    glBufferSubData(GL_ARRAY_BUFFER, 0, count * sizeof(InstanceData), inst);
}

void InstanceBatch::draw() const {
    if (m_mesh && m_count > 0) m_mesh->draw_instanced(m_vao, m_count);
}

// ------------------------------------------------------------------ primitives
void compute_normals(std::vector<Vertex>& v, const std::vector<uint32_t>& idx) {
    for (auto& x : v) x.normal = vec3(0);
    for (size_t t = 0; t + 2 < idx.size(); t += 3) {
        Vertex &a = v[idx[t]], &b = v[idx[t + 1]], &c = v[idx[t + 2]];
        vec3 n = cross(b.pos - a.pos, c.pos - a.pos);
        a.normal += n;
        b.normal += n;
        c.normal += n;
    }
    for (auto& x : v) x.normal = normalize_or(x.normal, vec3(0, 1, 0));
}

void make_box(std::vector<Vertex>& v, std::vector<uint32_t>& idx, vec3 h) {
    v.clear();
    idx.clear();
    const vec3 n[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (int f = 0; f < 6; f++) {
        vec3 nn = n[f];
        vec3 u = std::fabs(nn.y) > 0.5f ? vec3(1, 0, 0) : vec3(0, 1, 0);
        vec3 t = cross(u, nn);
        u = cross(nn, t);
        uint32_t base = (uint32_t)v.size();
        const float s[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
        for (int k = 0; k < 4; k++) {
            vec3 p = (nn + t * s[k][0] + u * s[k][1]) * h;
            v.push_back({p, nn, {s[k][0] * 0.5f + 0.5f, s[k][1] * 0.5f + 0.5f}});
        }
        idx.insert(idx.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
}

void make_sphere(std::vector<Vertex>& v, std::vector<uint32_t>& idx, float r, int seg, int rings) {
    v.clear();
    idx.clear();
    for (int y = 0; y <= rings; y++) {
        float th = kPi * y / rings;
        for (int x = 0; x <= seg; x++) {
            float ph = 2 * kPi * x / seg;
            vec3 n(std::sin(th) * std::cos(ph), std::cos(th), std::sin(th) * std::sin(ph));
            v.push_back({n * r, n, {(float)x / seg, (float)y / rings}});
        }
    }
    for (int y = 0; y < rings; y++)
        for (int x = 0; x < seg; x++) {
            uint32_t a = y * (seg + 1) + x, b = a + seg + 1;
            idx.insert(idx.end(), {a, a + 1, b, b, a + 1, b + 1});
        }
}

void make_cylinder(std::vector<Vertex>& v, std::vector<uint32_t>& idx, int seg, float top_r, bool caps) {
    v.clear();
    idx.clear();
    float slope = 1.0f - top_r;
    for (int y = 0; y <= 1; y++) {
        float r = y ? top_r : 1.0f;
        for (int x = 0; x <= seg; x++) {
            float a = 2 * kPi * x / seg;
            vec3 d(std::cos(a), 0, std::sin(a));
            vec3 n = normalize(vec3(d.x, slope, d.z));
            v.push_back({d * r + vec3(0, (float)y, 0), n, {(float)x / seg, (float)y}});
        }
    }
    for (int x = 0; x < seg; x++) {
        uint32_t a = x, b = x + seg + 1;
        idx.insert(idx.end(), {a, b, a + 1, a + 1, b, b + 1});
    }
    if (caps) {
        for (int y = 0; y <= 1; y++) {
            float r = y ? top_r : 1.0f;
            vec3 n(0, y ? 1.0f : -1.0f, 0);
            uint32_t c = (uint32_t)v.size();
            v.push_back({vec3(0, (float)y, 0), n, {0.5f, 0.5f}});
            for (int x = 0; x <= seg; x++) {
                float a = 2 * kPi * x / seg;
                v.push_back({vec3(std::cos(a) * r, (float)y, std::sin(a) * r), n, {std::cos(a) * 0.5f + 0.5f, std::sin(a) * 0.5f + 0.5f}});
            }
            for (int x = 0; x < seg; x++) {
                if (y) idx.insert(idx.end(), {c, c + 2 + x, c + 1 + x});
                else idx.insert(idx.end(), {c, c + 1 + x, c + 2 + x});
            }
        }
    }
}

void make_cone(std::vector<Vertex>& v, std::vector<uint32_t>& idx, int seg) { make_cylinder(v, idx, seg, 0.0f, true); }

} // namespace bl
