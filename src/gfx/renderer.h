// Forward renderer: cascaded shadow maps, sky, lit materials, instancing, debug lines.
#pragma once

#include "core/math.h"
#include "gfx/mesh.h"
#include "gfx/shader.h"
#include "gfx/texture.h"

#include <memory>
#include <vector>

namespace bl {

struct Material {
    TexturePtr diffuse;
    // (PBR maps, any of them: a tangent-space normal map (OpenGL's: +y up), a roughness map (its red), an ambient
    // occlusion map; with one of them the surface is lit by its roughness - GGX and the sky's reflection - instead of
    // specular and gloss; `roughness` multiplies the map, `detail_scale` > 0 tiles the normal map that many times a uv)
    TexturePtr normal_map, rough_map, ao_map;
    float roughness = 1.0f;
    float normal_strength = 1.0f;
    bool arm = false;          // (rough_map is an ARM texture: occlusion in its red, roughness in its green)
    TexturePtr diffuse2;       // vertex_color only: second layer (uv2), blended over the first by its alpha
    bool vertex_color = false; // mesh built from VertexC: albedo *= vertex colour (baked lighting)
    vec4 color{1, 1, 1, 1};
    float alpha_ref = 0.0f; // >0 enables alpha test
    bool blend = false;
    bool double_sided = false;
    bool cast_shadow = true;
    bool unlit = false;
    // (leaves: lit from both sides, the sun's light through them, no highlight; the normal's length is the vertex's
    // occlusion - how deep in the crown it is)
    bool foliage = false;
    // (a car's surfaces: its paint under a clear coat, glass, polished metal - each lit its own way, the sky in them)
    enum Surface : uint8_t { Plain = 0, Paint = 1, Glass = 2, Chrome = 3, Lens = 4 };
    uint8_t surface = Plain;
    float specular = 0.25f;
    float gloss = 24.0f;
    float reflect = 0.0f;
    vec3 emissive{0, 0, 0};
    std::string name;
};
using MaterialPtr = std::shared_ptr<Material>;

struct Camera {
    vec3 pos{0, 2, 10};
    vec3 target{0, 0, 0};
    vec3 up{0, 1, 0};
    float fov_deg = 60.0f;
    float znear = 0.1f;
    float zfar = 2500.0f;
    float ortho_half = 0.0f; // > 0: orthographic projection, half the view height (m)
    mat4 view, proj, viewproj;
    void update(float aspect) {
        view = look_at(pos, target, up);
        proj = ortho_half > 0 ? ortho(-ortho_half * aspect, ortho_half * aspect, -ortho_half, ortho_half, znear, zfar) : perspective(fov_deg * kDeg2Rad, aspect, znear, zfar);
        viewproj = proj * view;
    }
    vec3 forward() const { return normalize(target - pos); }
};

struct LightSettings {
    vec3 sun_dir = normalize(vec3(0.45f, 0.75f, 0.35f));
    vec3 sun_color{2.55f, 2.4f, 2.2f};
    vec3 sky_color{0.36f, 0.5f, 0.75f};
    vec3 ground_color{0.22f, 0.2f, 0.17f};
    vec3 fog_color{0.66f, 0.74f, 0.84f};
    float fog_density = 0.0011f;
    float exposure = 0.9f;
    bool shadows = true;
    // the sky as a picture: an equirectangular panorama (tonemapped, sRGB) turned by sky_yaw about the vertical (rad);
    // none: the gradient and the procedural clouds
    TexturePtr sky_panorama;
    float sky_yaw = 0;
};

class Renderer {
public:
    bool init();
    void begin_frame(int fb_w, int fb_h, const Camera& cam, const LightSettings& light, float time);
    // Draw submission (valid until end of frame).
    void draw_mesh(const GpuMesh* mesh, const Material* mat, const mat4& model, int first = 0, int count = -1);
    void draw_instanced(const InstanceBatch* batch, const Material* mat, bool wind = false);
    void draw_terrain(const GpuMesh* mesh, GLuint splat_tex);
    // the terrain's ground: tiled grass and earth materials (colour, normal map), the splat their tint; none: the
    // shader's noise over the splat as before
    MaterialPtr terrain_grass, terrain_dirt;
    // (and its paving - asphalt, concrete: the splat's colour in an asphalt's grain, its normal and roughness maps)
    MaterialPtr terrain_paved;

    // Debug primitives (world space).
    void line(vec3 a, vec3 b, uint32_t color);
    void line(vec3 a, vec3 b, vec4 color);
    // a line px points wide (px_scale framebuffer pixels per point): a strip facing the camera; 1 or less: a line
    void thick_line(vec3 a, vec3 b, uint32_t color, float px);
    void point(vec3 p, uint32_t color);
    void box_wire(const mat4& m, vec3 half, uint32_t color);

    void render();
    bool screenshot(const std::string& path, int w, int h);
    // a part of the framebuffer to render into (pixels, y up; x < 0: all of it): the editor's split views render the
    // frame once per view. The sky can be left out (an orthographic view has no horizon).
    void set_viewport(int x, int y, int w, int h) { m_vx = x, m_vy = y, m_vw = w, m_vh = h; }
    void clear_screen(int w, int h, vec3 color);
    bool draw_sky = true;
    // The post-process (a full view alone, not the editor's split views): the scene drawn into a buffer of linear light,
    // then over it the ambient occlusion off its depth, the bloom of its brights, the film curve and the grade
    // (BL_NOPOST=1: straight to the screen as before)
    bool post_fx = true;
    float ao_strength = 0.9f, ao_radius = 0.7f, bloom_strength = 0.07f;
    bool sky_clouds = true;  // off: the sky is a plain gradient with the horizon (the model editor)

    const Camera& camera() const { return m_cam; }
    int draw_calls() const { return m_draw_calls; }
    int culled() const { return m_culled; }
    int triangles() const { return m_tris; }
    double gpu_ms() const { return m_gpu_ms; }
    bool debug_depth_test = true;
    float point_size = 6.0f;  // framebuffer pixels
    float px_scale = 1.0f;    // framebuffer pixels per UI point (thick lines' widths are in points)

    static uint32_t rgba(float r, float g, float b, float a = 1.0f) {
        auto c = [](float v) { return (uint32_t)(clampf(v, 0, 1) * 255.0f + 0.5f); };
        return c(r) | (c(g) << 8) | (c(b) << 16) | (c(a) << 24);
    }

private:
    struct Item {
        const GpuMesh* mesh;
        const Material* mat;
        mat4 model;
        int first, count;
        float sort_key;
        vec3 center;   // world bounding sphere (radius < 0: never culled)
        float radius;
    };
    struct InstItem {
        const InstanceBatch* batch;
        const Material* mat;
        bool wind;
    };
    struct LineVertex {
        vec3 pos;
        uint32_t color;
    };

    void shadow_pass();
    void set_common_uniforms(Shader& s);
    void bind_material(Shader& s, const Material* m);
    void draw_items(bool blended);
    void flush_debug();
    void compute_cascades();

    Shader m_mesh, m_mesh_vc, m_mesh_inst, m_mesh_wind, m_shadow, m_shadow_inst, m_sky, m_terrain, m_lines;
    Shader m_post_ao, m_post_blur, m_post_final;
    // (the post-process's buffers: the scene multisampled, resolved - colour and depth as textures -, the occlusion at
    // half size and its blur, the bloom's levels)
    static constexpr int kBloomLevels = 5;
    GLuint m_ms_fbo = 0, m_ms_color = 0, m_ms_depth = 0, m_rs_fbo = 0, m_rs_color = 0, m_rs_depth = 0;
    GLuint m_ao_fbo[2] = {0, 0}, m_ao_tex[2] = {0, 0}, m_bloom_fbo[kBloomLevels] = {}, m_bloom_tex[kBloomLevels] = {};
    int m_pw = 0, m_ph = 0;
    bool m_hdr = false;   // (this render goes into the post-process's buffer)
    bool post_targets(int w, int h);
    void post_process();
    GLuint m_shadow_tex = 0, m_shadow_fbo = 0;
    GLuint m_line_vao = 0, m_line_vbo = 0;
    GLuint m_empty_vao = 0;
    GLuint m_gpu_query[2] = {0, 0};
    int m_query_idx = 0;
    bool m_query_pending[2] = {false, false};
    double m_gpu_ms = 0.0;

    Camera m_cam;
    LightSettings m_light;
    float m_time = 0;
    int m_w = 1, m_h = 1;
    int m_vx = -1, m_vy = 0, m_vw = 1, m_vh = 1;
    mat4 m_shadow_mat[3];
    float m_cascade_far[3] = {18.0f, 60.0f, 220.0f};
    float m_cascade_radius[3] = {1, 1, 1};
    vec4 m_frustum[6];
    int m_culled = 0;

    std::vector<Item> m_items;
    std::vector<InstItem> m_inst;
    const GpuMesh* m_terrain_mesh = nullptr;
    GLuint m_terrain_splat = 0;
    std::vector<LineVertex> m_lines_buf, m_points_buf, m_strips_buf; // (strips: the thick lines' triangles)
    int m_draw_calls = 0, m_tris = 0;
    int m_line_capacity = 0;
};

} // namespace bl
