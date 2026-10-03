#include "gfx/renderer.h"
#include "core/profiler.h"
#include "core/util.h"

#include <algorithm>
#include <cstring>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace bl {

static constexpr int kShadowSize = 2048;

bool Renderer::init() {
    bool ok = true;
    ok &= m_mesh.load("mesh");
    ok &= m_mesh_vc.load("mesh", "#define VCOLOR 1");
    ok &= m_mesh_inst.load("mesh", "#define INSTANCED 1");
    ok &= m_mesh_wind.load("mesh", "#define INSTANCED 1\n#define WIND 1");
    ok &= m_shadow.load("shadow");
    ok &= m_shadow_inst.load("shadow", "#define INSTANCED 1");
    ok &= m_sky.load("sky");
    ok &= m_terrain.load("terrain");
    ok &= m_lines.load("lines");
    if (!ok) return false;

    glGenTextures(1, &m_shadow_tex);
    glBindTexture(GL_TEXTURE_2D_ARRAY, m_shadow_tex);
    glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_DEPTH_COMPONENT32F, kShadowSize, kShadowSize, 3, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glGenFramebuffers(1, &m_shadow_fbo);

    glGenVertexArrays(1, &m_line_vao);
    glGenBuffers(1, &m_line_vbo);
    glBindVertexArray(m_line_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_line_vbo);
    glEnableVertexAttribArray(ATTR_POS);
    glVertexAttribPointer(ATTR_POS, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex), (void*)0);
    glEnableVertexAttribArray(ATTR_COLOR);
    glVertexAttribPointer(ATTR_COLOR, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(LineVertex), (void*)12);
    glBindVertexArray(0);
    glGenVertexArrays(1, &m_empty_vao);
    glGenQueries(2, m_gpu_query);
    return true;
}

void Renderer::begin_frame(int w, int h, const Camera& cam, const LightSettings& light, float time) {
    m_w = std::max(1, w);
    m_h = std::max(1, h);
    m_cam = cam;
    m_cam.update(m_vx >= 0 ? (float)std::max(1, m_vw) / (float)std::max(1, m_vh) : (float)m_w / (float)m_h);
    m_light = light;
    m_time = time;
    m_items.clear();
    m_inst.clear();
    m_terrain_mesh = nullptr;
    m_lines_buf.clear();
    m_points_buf.clear();
    m_strips_buf.clear();
}

void Renderer::draw_mesh(const GpuMesh* mesh, const Material* mat, const mat4& model, int first, int count) {
    if (!mesh || !mesh->valid() || !mat) return;
    if (count < 0) count = mesh->index_count() - first;
    if (count <= 0) return;
    float key = 0;
    vec3 center(0);
    float radius = -1;
    if (mesh->bounds.valid()) {
        center = model.transform_point(mesh->bounds.center());
        float s = std::max(length(model.c[0].xyz()), std::max(length(model.c[1].xyz()), length(model.c[2].xyz())));
        radius = length(mesh->bounds.extent()) * 0.5f * s;
    }
    if (mat->blend) key = -length2(center - m_cam.pos);
    m_items.push_back({mesh, mat, model, first, count, key, center, radius});
}

void Renderer::draw_instanced(const InstanceBatch* batch, const Material* mat, bool wind) {
    if (!batch || batch->count() <= 0) return;
    m_inst.push_back({batch, mat, wind});
}

void Renderer::draw_terrain(const GpuMesh* mesh, GLuint splat) {
    m_terrain_mesh = mesh;
    m_terrain_splat = splat;
}

void Renderer::line(vec3 a, vec3 b, uint32_t c) {
    m_lines_buf.push_back({a, c});
    m_lines_buf.push_back({b, c});
}
void Renderer::line(vec3 a, vec3 b, vec4 c) { line(a, b, rgba(c.x, c.y, c.z, c.w)); }
void Renderer::point(vec3 p, uint32_t c) { m_points_buf.push_back({p, c}); }

void Renderer::thick_line(vec3 a, vec3 b, uint32_t c, float px) {
    const float w = px * px_scale;
    if (w <= 1.05f) {
        line(a, b, c);
        return;
    }
    // a quad across the view direction, as wide on screen as asked at both ends
    const float vh = (float)std::max(1, m_vx >= 0 ? m_vh : m_h);
    const vec3 fwd = m_cam.forward();
    const bool ortho = m_cam.ortho_half > 0;
    auto half = [&](vec3 p) {
        const float per_px = ortho ? 2.0f * m_cam.ortho_half / vh : 2.0f * std::tan(m_cam.fov_deg * kDeg2Rad * 0.5f) * std::max(m_cam.znear, dot(p - m_cam.pos, fwd)) / vh;
        return per_px * w * 0.5f;
    };
    const vec3 d = b - a;
    vec3 side = cross(d, ortho ? fwd : (a + b) * 0.5f - m_cam.pos);
    if (length2(side) < 1e-14f) side = cross(fwd, m_cam.up);
    side = normalize_or(side, vec3(1, 0, 0));
    const vec3 sa = side * half(a), sb = side * half(b);
    const LineVertex q[6] = {{a - sa, c}, {a + sa, c}, {b + sb, c}, {a - sa, c}, {b + sb, c}, {b - sb, c}};
    m_strips_buf.insert(m_strips_buf.end(), q, q + 6);
}

void Renderer::box_wire(const mat4& m, vec3 h, uint32_t color) {
    vec3 c[8];
    for (int i = 0; i < 8; i++) c[i] = m.transform_point(vec3(i & 1 ? h.x : -h.x, i & 2 ? h.y : -h.y, i & 4 ? h.z : -h.z));
    const int e[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (auto& k : e) line(c[k[0]], c[k[1]], color);
}

void Renderer::compute_cascades() {
    float aspect = (float)m_w / (float)m_h;
    float tan_h = std::tan(m_cam.fov_deg * kDeg2Rad * 0.5f);
    vec3 fwd = m_cam.forward();
    float near_d = m_cam.znear;
    for (int c = 0; c < 3; c++) {
        float far_d = m_cascade_far[c];
        float mid = (near_d + far_d) * 0.5f;
        vec3 center = m_cam.pos + fwd * mid;
        float half_depth = (far_d - near_d) * 0.5f;
        float fh = far_d * tan_h, fw = fh * aspect;
        float radius = std::sqrt(half_depth * half_depth + fh * fh + fw * fw);
        radius = std::ceil(radius * 16.0f) / 16.0f;
        vec3 L = m_light.sun_dir;
        vec3 up = std::fabs(L.y) > 0.95f ? vec3(1, 0, 0) : vec3(0, 1, 0);
        // Stabilize: snap the center to shadow texel increments in light space.
        mat4 lview0 = look_at(vec3(0), -L, up);
        vec3 ls = lview0.transform_point(center);
        float texel = 2.0f * radius / kShadowSize;
        ls.x = std::floor(ls.x / texel) * texel;
        ls.y = std::floor(ls.y / texel) * texel;
        mat4 inv = inverse(lview0);
        center = inv.transform_point(ls);
        float back = 400.0f;
        mat4 lview = look_at(center + L * (radius + back), center, up);
        mat4 lproj = ortho(-radius, radius, -radius, radius, 0.1f, 2.0f * radius + back * 2.0f);
        m_shadow_mat[c] = lproj * lview;
        m_cascade_radius[c] = radius;
        near_d = far_d * 0.9f;
    }
}

void Renderer::set_common_uniforms(Shader& s) {
    s.set("u_cam_pos", m_cam.pos);
    s.set("u_sun_dir", m_light.sun_dir);
    s.set("u_sun_color", m_light.sun_color);
    s.set("u_sky_color", m_light.sky_color);
    s.set("u_ground_color", m_light.ground_color);
    s.set("u_fog_color", m_light.fog_color);
    s.set("u_fog_density", m_light.fog_density);
    s.set("u_exposure", m_light.exposure);
    s.set_array("u_shadow_mat", m_shadow_mat, 3);
    s.set("u_cascade_far", vec3(m_cascade_far[0], m_cascade_far[1], m_cascade_far[2]));
    s.set("u_shadow_enabled", m_light.shadows ? 1.0f : 0.0f);
    s.set("u_shadow_map", 1);
    s.set("u_viewproj", m_cam.viewproj);
    s.set("u_time", m_time);
    s.set("u_wind", vec3(1, 0, 0.5f));
}

void Renderer::bind_material(Shader& s, const Material* m) {
    glActiveTexture(GL_TEXTURE0);
    Texture* t = m->diffuse ? m->diffuse.get() : TextureCache::get().white().get();
    glBindTexture(GL_TEXTURE_2D, t->id);
    s.set("u_tex", 0);
    s.set("u_color", m->color);
    s.set("u_alpha_ref", m->alpha_ref);
    s.set("u_spec", m->specular);
    s.set("u_gloss", m->gloss);
    s.set("u_reflect", m->reflect);
    s.set("u_blend", m->blend ? 1.0f : 0.0f);
    s.set("u_emissive", m->emissive);
    s.set("u_unlit", m->unlit ? 1.0f : 0.0f);
    const bool pbr = !m->vertex_color && (m->normal_map || m->rough_map || m->ao_map);
    s.set("u_pbr", pbr ? 1.0f : 0.0f);
    if (pbr) {
        TextureCache& tc = TextureCache::get();
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, (m->normal_map ? m->normal_map : tc.flat_normal())->id);
        glActiveTexture(GL_TEXTURE3);
        glBindTexture(GL_TEXTURE_2D, (m->rough_map ? m->rough_map : tc.white())->id);
        glActiveTexture(GL_TEXTURE4);
        glBindTexture(GL_TEXTURE_2D, (m->ao_map ? m->ao_map : tc.white())->id);
        glActiveTexture(GL_TEXTURE0);
        s.set("u_normal_map", 2);
        s.set("u_rough_map", 3);
        s.set("u_ao_map", 4);
        s.set("u_roughness", m->roughness);
        s.set("u_arm", m->arm ? 1.0f : 0.0f);
        s.set("u_normal_strength", m->normal_strength);
    }
    if (m->vertex_color) {
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, m->diffuse2 ? m->diffuse2->id : t->id);
        glActiveTexture(GL_TEXTURE0);
        s.set("u_tex2", 2);
        s.set("u_dual", m->diffuse2 ? 1.0f : 0.0f);
    }
    if (m->double_sided) glDisable(GL_CULL_FACE);
    else glEnable(GL_CULL_FACE);
    // alpha tested foliage/decals: alpha-to-coverage gives smooth MSAA edges instead of shimmering cut-outs
    if (m->alpha_ref > 0 && !m->blend) glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    else glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
}

void Renderer::shadow_pass() {
    PROFILE_ZONE("Shadows");
    glBindFramebuffer(GL_FRAMEBUFFER, m_shadow_fbo);
    glViewport(0, 0, kShadowSize, kShadowSize);
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.5f, 2.0f);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    for (int c = 0; c < 3; c++) {
        glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, m_shadow_tex, 0, c);
        glClear(GL_DEPTH_BUFFER_BIT);
        // Cull by distance: cascade c only needs objects in its sphere-ish range.
        m_shadow.use();
        m_shadow.set("u_viewproj", m_shadow_mat[c]);
        m_shadow.set("u_tex", 0);
        glActiveTexture(GL_TEXTURE0);
        if (m_terrain_mesh) {
            m_shadow.set("u_model", mat4());
            m_shadow.set("u_alpha_ref", 0.0f);
            m_terrain_mesh->draw();
            m_draw_calls++;
        }
        for (auto& it : m_items) {
            if (!it.mat->cast_shadow || it.mat->blend) continue;
            if (it.radius >= 0) {
                vec4 sp = m_shadow_mat[c] * vec4(it.center, 1.0f);
                float rn = it.radius / m_cascade_radius[c];
                if (std::fabs(sp.x) > 1.0f + rn || std::fabs(sp.y) > 1.0f + rn) continue;
            }
            m_shadow.set("u_model", it.model);
            float aref = it.mat->alpha_ref;
            m_shadow.set("u_alpha_ref", aref);
            if (aref > 0) glBindTexture(GL_TEXTURE_2D, it.mat->diffuse ? it.mat->diffuse->id : TextureCache::get().white()->id);
            it.mesh->draw_range(it.first, it.count);
            m_draw_calls++;
        }
        m_shadow_inst.use();
        m_shadow_inst.set("u_viewproj", m_shadow_mat[c]);
        m_shadow_inst.set("u_tex", 0);
        for (auto& it : m_inst) {
            if (!it.mat->cast_shadow) continue;
            float aref = it.mat->alpha_ref;
            m_shadow_inst.set("u_alpha_ref", aref);
            if (aref > 0) glBindTexture(GL_TEXTURE_2D, it.mat->diffuse ? it.mat->diffuse->id : TextureCache::get().white()->id);
            it.batch->draw();
            m_draw_calls++;
        }
    }
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer::draw_items(bool blended) {
    Shader* cur = nullptr;
    const Material* last = nullptr;
    for (auto& it : m_items) {
        if (it.mat->blend != blended) continue;
        if (it.radius >= 0) {
            bool out = false;
            for (int p = 0; p < 6 && !out; p++) out = dot(m_frustum[p].xyz(), it.center) + m_frustum[p].w < -it.radius;
            if (out) {
                m_culled++;
                continue;
            }
        }
        Shader* s = it.mat->vertex_color ? &m_mesh_vc : &m_mesh;
        if (s != cur) {
            s->use();
            set_common_uniforms(*s);
            cur = s;
            last = nullptr;
        }
        if (it.mat != last) {
            bind_material(*cur, it.mat);
            last = it.mat;
        }
        cur->set("u_model", it.model);
        it.mesh->draw_range(it.first, it.count);
        m_draw_calls++;
        m_tris += it.count / 3;
    }
}

void Renderer::flush_debug() {
    if (m_lines_buf.empty() && m_points_buf.empty() && m_strips_buf.empty()) return;
    PROFILE_ZONE("Debug draw");
    m_lines.use();
    m_lines.set("u_viewproj", m_cam.viewproj);
    glBindVertexArray(m_line_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_line_vbo);
    size_t total = m_lines_buf.size() + m_points_buf.size() + m_strips_buf.size();
    if ((int)total > m_line_capacity) {
        m_line_capacity = (int)(total + total / 2);
    }
    const size_t n_lines = m_lines_buf.size(), n_points = m_points_buf.size();
    glBufferData(GL_ARRAY_BUFFER, m_line_capacity * sizeof(LineVertex), nullptr, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, n_lines * sizeof(LineVertex), m_lines_buf.data());
    glBufferSubData(GL_ARRAY_BUFFER, n_lines * sizeof(LineVertex), n_points * sizeof(LineVertex), m_points_buf.data());
    glBufferSubData(GL_ARRAY_BUFFER, (n_lines + n_points) * sizeof(LineVertex), m_strips_buf.size() * sizeof(LineVertex), m_strips_buf.data());
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (debug_depth_test) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    m_lines.set("u_round", 0.0f);
    m_lines.set("u_point_size", 1.0f);
    if (!m_strips_buf.empty()) glDrawArrays(GL_TRIANGLES, (GLint)(n_lines + n_points), (GLsizei)m_strips_buf.size());
    glDrawArrays(GL_LINES, 0, (GLsizei)m_lines_buf.size());
    if (!m_points_buf.empty()) {
        glEnable(GL_PROGRAM_POINT_SIZE);
        m_lines.set("u_round", 1.0f);
        m_lines.set("u_point_size", point_size);
        glDrawArrays(GL_POINTS, (GLint)m_lines_buf.size(), (GLsizei)m_points_buf.size());
    }
    m_draw_calls += m_strips_buf.empty() ? 2 : 3;
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
}

void Renderer::render() {
    PROFILE_ZONE("Render");
    m_draw_calls = 0;
    m_tris = 0;
    // GPU timing (read the query from two frames ago)
    int qi = m_query_idx;
    if (m_query_pending[qi]) {
        GLint avail = 0;
        glGetQueryObjectiv(m_gpu_query[qi], GL_QUERY_RESULT_AVAILABLE, &avail);
        if (avail) {
            GLuint64 ns = 0;
            glGetQueryObjectui64v(m_gpu_query[qi], GL_QUERY_RESULT, &ns);
            m_gpu_ms = ns * 1e-6;
        }
        m_query_pending[qi] = false;
    }
    glBeginQuery(GL_TIME_ELAPSED, m_gpu_query[qi]);

    compute_cascades();
    {
        // frustum planes (Gribb/Hartmann) from the view-projection matrix, normalized
        const mat4 m = transpose(m_cam.viewproj);
        const vec4 r0 = m.c[0], r1 = m.c[1], r2 = m.c[2], r3 = m.c[3];
        vec4 pl[6] = {r3 + r0, r3 - r0, r3 + r1, r3 - r1, r3 + r2, r3 - r2};
        for (int i = 0; i < 6; i++) {
            float l = length(pl[i].xyz());
            m_frustum[i] = l > 0 ? pl[i] * (1.0f / l) : pl[i];
        }
        m_culled = 0;
    }
    if (m_light.shadows) shadow_pass();

    {
        PROFILE_ZONE("Main pass");
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        if (m_vx >= 0) {
            glViewport(m_vx, m_vy, m_vw, m_vh);
            glEnable(GL_SCISSOR_TEST);
            glScissor(m_vx, m_vy, m_vw, m_vh);
        } else {
            glViewport(0, 0, m_w, m_h);
        }
        glClearColor(m_light.fog_color.x, m_light.fog_color.y, m_light.fog_color.z, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D_ARRAY, m_shadow_tex);

        if (m_terrain_mesh) {
            m_terrain.use();
            set_common_uniforms(m_terrain);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, m_terrain_splat);
            m_terrain.set("u_splat", 0);
            const bool detail = terrain_grass && terrain_dirt && terrain_grass->diffuse && terrain_dirt->diffuse;
            if (detail) {
                TextureCache& tc = TextureCache::get();
                const GLuint ids[4] = {terrain_grass->diffuse->id, (terrain_grass->normal_map ? terrain_grass->normal_map : tc.flat_normal())->id, terrain_dirt->diffuse->id,
                                       (terrain_dirt->normal_map ? terrain_dirt->normal_map : tc.flat_normal())->id};
                const char* names[4] = {"u_grass", "u_grass_normal", "u_dirt", "u_dirt_normal"};
                for (int k = 0; k < 4; k++) {
                    glActiveTexture(GL_TEXTURE6 + k);
                    glBindTexture(GL_TEXTURE_2D, ids[k]);
                    m_terrain.set(names[k], 6 + k);
                }
                glActiveTexture(GL_TEXTURE0);
            }
            m_terrain.set("u_has_detail", detail ? 1 : 0);
            m_terrain_mesh->draw();
            m_draw_calls++;
            m_tris += m_terrain_mesh->index_count() / 3;
        }
        draw_items(false);
        for (int pass = 0; pass < 2; pass++) {
            Shader& s = pass == 0 ? m_mesh_inst : m_mesh_wind;
            bool any = false;
            for (auto& it : m_inst)
                if (it.wind == (pass == 1)) any = true;
            if (!any) continue;
            s.use();
            set_common_uniforms(s);
            for (auto& it : m_inst) {
                if (it.wind != (pass == 1)) continue;
                bind_material(s, it.mat);
                it.batch->draw();
                m_draw_calls++;
                m_tris += it.batch->count() * it.batch->mesh()->index_count() / 3;
            }
        }
        // sky last among opaques (fills only empty pixels)
        glDisable(GL_CULL_FACE);
        if (draw_sky) {
            m_sky.use();
            set_common_uniforms(m_sky);
            m_sky.set("u_inv_viewproj", inverse(m_cam.viewproj));
            m_sky.set("u_clouds", sky_clouds ? 1.0f : 0.0f);
            m_sky.set("u_panorama_on", m_light.sky_panorama ? 1.0f : 0.0f);
            if (m_light.sky_panorama) {
                glActiveTexture(GL_TEXTURE5);
                glBindTexture(GL_TEXTURE_2D, m_light.sky_panorama->id);
                glActiveTexture(GL_TEXTURE0);
                m_sky.set("u_panorama", 5);
                m_sky.set("u_sky_yaw", m_light.sky_yaw);
            }
            glBindVertexArray(m_empty_vao);
            glDrawArrays(GL_TRIANGLES, 0, 3);
            m_draw_calls++;
        }

        // blended, back to front
        std::stable_sort(m_items.begin(), m_items.end(), [](const Item& a, const Item& b) { return a.sort_key < b.sort_key; });
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        draw_items(true);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
        flush_debug();
        glDisable(GL_SCISSOR_TEST);
    }
    glEndQuery(GL_TIME_ELAPSED);
    m_query_pending[qi] = true;
    m_query_idx ^= 1;
    glBindVertexArray(0);
    glEnable(GL_CULL_FACE);
}

void Renderer::clear_screen(int w, int h, vec3 color) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_SCISSOR_TEST);
    glViewport(0, 0, w, h);
    glClearColor(color.x, color.y, color.z, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

bool Renderer::screenshot(const std::string& path, int w, int h) {
    std::vector<uint8_t> px((size_t)w * h * 4), flipped((size_t)w * h * 4);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
    for (int y = 0; y < h; y++) std::memcpy(&flipped[(size_t)y * w * 4], &px[(size_t)(h - 1 - y) * w * 4], (size_t)w * 4);
    for (size_t i = 3; i < flipped.size(); i += 4) flipped[i] = 255;
    return stbi_write_png(path.c_str(), w, h, 4, flipped.data(), w * 4) != 0;
}

} // namespace bl
