#include "world/terrain.h"
#include "world/objects.h"
#include "core/profiler.h"

#include <algorithm>
#include <cmath>

namespace bl {

TerrainRender::~TerrainRender() {
    if (m_splat) glDeleteTextures(1, &m_splat);
}

void TerrainRender::build(const phys::Heightfield& hf, const std::vector<float>* render_drop) {
    PROFILE_ZONE("Terrain mesh");
    const int nx = hf.nx(), nz = hf.nz();
    std::vector<Vertex> v((size_t)nx * nz);
    const vec2 o = hf.origin();
    const float c = hf.cell();
    for (int z = 0; z < nz; z++)
        for (int x = 0; x < nx; x++) {
            Vertex& vx = v[(size_t)z * nx + x];
            vx.pos = vec3(o.x + x * c, hf.h(x, z), o.y + z * c);
            if (render_drop) vx.pos.y -= (*render_drop)[(size_t)z * nx + x];
            vx.normal = hf.vertex_normal(x, z);
            vx.uv = vec2((x + 0.5f) / nx, (z + 0.5f) / nz);
        }
    std::vector<uint32_t> idx;
    idx.reserve((size_t)(nx - 1) * (nz - 1) * 6);
    for (int z = 0; z < nz - 1; z++)
        for (int x = 0; x < nx - 1; x++) {
            uint32_t i00 = z * nx + x, i10 = i00 + 1, i01 = i00 + nx, i11 = i01 + 1;
            // split along 00-11 like the physics heightfield (CCW seen from above, +Y up)
            idx.insert(idx.end(), {i00, i11, i10, i00, i01, i11});
        }
    m_mesh.create(v, idx, false);

    // splat texture: sRGB albedo + material id
    const auto& gms = phys::ground_models();
    std::vector<uint8_t> px((size_t)nx * nz * 4);
    for (int z = 0; z < nz; z++)
        for (int x = 0; x < nx; x++) {
            int s = hf.surf(x, z);
            vec3 col = gms[s].color;
            float var = 0.85f + 0.3f * (fbm2(x * 0.05f, z * 0.05f, 3, 2.0f, 0.5f, 99) * 0.5f + 0.5f);
            if (s == phys::SURF_GRASS) {
                float t = fbm2(x * 0.02f, z * 0.02f, 3, 2.0f, 0.5f, 7) * 0.5f + 0.5f;
                col = lerp(vec3(0.23f, 0.33f, 0.13f), vec3(0.34f, 0.40f, 0.17f), t);
            }
            if (s == phys::SURF_ASPHALT || s == phys::SURF_CONCRETE) var = 0.95f + 0.1f * (var - 0.85f);
            col = col * var;
            uint8_t* p = &px[((size_t)z * nx + x) * 4];
            p[0] = (uint8_t)(clampf(col.x, 0, 1) * 255);
            p[1] = (uint8_t)(clampf(col.y, 0, 1) * 255);
            p[2] = (uint8_t)(clampf(col.z, 0, 1) * 255);
            p[3] = (s == phys::SURF_ASPHALT || s == phys::SURF_CONCRETE) ? 255 : 0; // paved mask
        }
    // soften surface borders (surfaces are painted per heightfield vertex): separable 5-tap blur of the colour
    {
        std::vector<uint8_t> tmp(px.size());
        const int R = 2;
        for (int pass = 0; pass < 2; pass++) {
            const std::vector<uint8_t>& src = pass == 0 ? px : tmp;
            std::vector<uint8_t>& dst = pass == 0 ? tmp : px;
            for (int z = 0; z < nz; z++)
                for (int x = 0; x < nx; x++) {
                    int acc[4] = {0, 0, 0, 0}, n = 0;
                    for (int k = -R; k <= R; k++) {
                        int xx = pass == 0 ? std::clamp(x + k, 0, nx - 1) : x;
                        int zz = pass == 1 ? std::clamp(z + k, 0, nz - 1) : z;
                        const uint8_t* s = &src[((size_t)zz * nx + xx) * 4];
                        acc[0] += s[0];
                        acc[1] += s[1];
                        acc[2] += s[2];
                        acc[3] += s[3];
                        n++;
                    }
                    uint8_t* d = &dst[((size_t)z * nx + x) * 4];
                    for (int c2 = 0; c2 < 4; c2++) d[c2] = (uint8_t)(acc[c2] / n);
                }
        }
    }
    if (!m_splat) glGenTextures(1, &m_splat);
    glBindTexture(GL_TEXTURE_2D, m_splat);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, nx, nz, 0, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void TerrainRender::draw(Renderer& r) const { r.draw_terrain(&m_mesh, m_splat); }

// ------------------------------------------------------------------ road
void RoadRender::build(const phys::RoadSurface& road, const phys::Heightfield& hf, MaterialPtr mat) {
    PROFILE_ZONE("Road mesh");
    m_mat = std::move(mat);
    m_chunks.clear();
    const float W = road.half_width + road.edge;
    const float ds = 0.3f, dl = 0.12f;
    const int cols = (int)std::ceil(2 * W / dl) + 1;
    const float chunk_len = 24.0f;
    const float L = road.length();
    for (float s0 = 0; s0 < L - 0.01f; s0 += chunk_len) {
        float s1 = std::min(L, s0 + chunk_len);
        int rows = std::max(2, (int)std::ceil((s1 - s0) / ds) + 1);
        std::vector<Vertex> v((size_t)rows * cols);
        for (int r = 0; r < rows; r++) {
            float s = s0 + (s1 - s0) * r / (rows - 1);
            vec2 c = road.point(s), l = road.left(s);
            for (int k = 0; k < cols; k++) {
                float lat = -W + 2 * W * k / (cols - 1);
                vec2 q = c + l * lat;
                float h = hf.height(q.x, q.y);
                Vertex& vx = v[(size_t)r * cols + k];
                vx.pos = vec3(q.x, h + road.detail(s, lat) + 0.008f, q.y);
                // warp the texture across so its rut band (0.8 m out) runs along this side's wandering rut
                float a = std::fabs(lat), side = lat < 0 ? -1.0f : 1.0f, rc = road.rut_center(s, side);
                float ta = a < rc ? a * 0.8f / rc : 0.8f + (a - rc) * (W - 0.8f) / std::max(0.1f, W - rc);
                vx.uv = vec2((side * ta + W) / (2 * W), s / 16.0f);
            }
        }
        // normals from the grid (central differences)
        for (int r = 0; r < rows; r++)
            for (int k = 0; k < cols; k++) {
                const vec3& a = v[(size_t)r * cols + std::max(0, k - 1)].pos;
                const vec3& b = v[(size_t)r * cols + std::min(cols - 1, k + 1)].pos;
                const vec3& e = v[(size_t)std::max(0, r - 1) * cols + k].pos;
                const vec3& f = v[(size_t)std::min(rows - 1, r + 1) * cols + k].pos;
                vec3 n = cross(b - a, f - e); // lateral (left) x forward
                if (n.y < 0) n = -n;
                v[(size_t)r * cols + k].normal = normalize_or(n, vec3(0, 1, 0));
            }
        std::vector<uint32_t> idx;
        idx.reserve((size_t)(rows - 1) * (cols - 1) * 6);
        for (int r = 0; r + 1 < rows; r++)
            for (int k = 0; k + 1 < cols; k++) {
                uint32_t a = r * cols + k, b = a + 1, c2 = a + cols, d = c2 + 1;
                // CCW seen from above: lat increases to the left of the driving direction
                idx.insert(idx.end(), {a, c2, b, b, c2, d});
            }
        // pick the winding that faces up
        {
            vec3 p0 = v[idx[0]].pos, p1 = v[idx[1]].pos, p2 = v[idx[2]].pos;
            if (cross(p1 - p0, p2 - p0).y < 0)
                for (size_t i = 0; i + 2 < idx.size(); i += 3) std::swap(idx[i + 1], idx[i + 2]);
        }
        auto m = std::make_unique<GpuMesh>();
        m->create(v, idx, false);
        m_chunks.push_back(std::move(m));
    }
    build_potholes(road, hf);
}

void RoadRender::build_potholes(const phys::RoadSurface& road, const phys::Heightfield& hf) {
    const int S = 14, RINGS = 3;
    std::vector<Vertex> v;
    std::vector<uint32_t> idx;
    for (const phys::RoadPothole& ph : road.holes()) {
        vec2 c = road.point(ph.s), l = road.left(ph.s), t = road.tangent(ph.s);
        float R = ph.radius * 1.15f;
        uint32_t base = (uint32_t)v.size();
        auto vert = [&](float ds, float dl, vec2 uv) {
            vec2 q = c + l * (ph.lat + dl) + t * ds;
            float h = hf.height(q.x, q.y) + road.detail(ph.s + ds, ph.lat + dl) + 0.014f;
            v.push_back({vec3(q.x, h, q.y), vec3(0, 1, 0), uv});
        };
        vert(0, 0, {0.5f, 0.5f});
        for (int rg = 1; rg <= RINGS; rg++)
            for (int i = 0; i < S; i++) {
                float a = 2 * kPi * i / S, rr = R * rg / RINGS;
                vert(std::cos(a) * rr, std::sin(a) * rr, {0.5f + 0.5f * std::cos(a) * rg / RINGS, 0.5f + 0.5f * std::sin(a) * rg / RINGS});
            }
        for (int i = 0; i < S; i++) {
            int j = (i + 1) % S;
            idx.insert(idx.end(), {base, base + 1 + (uint32_t)j, base + 1 + (uint32_t)i});
            for (int rg = 1; rg < RINGS; rg++) {
                uint32_t a0 = base + 1 + (rg - 1) * S + i, a1 = base + 1 + (rg - 1) * S + j;
                uint32_t b0 = base + 1 + rg * S + i, b1 = base + 1 + rg * S + j;
                idx.insert(idx.end(), {a0, a1, b1, a0, b1, b0});
            }
        }
    }
    if (idx.empty()) return;
    // orient every triangle upwards, then smooth normals
    for (size_t i = 0; i + 2 < idx.size(); i += 3)
        if (cross(v[idx[i + 1]].pos - v[idx[i]].pos, v[idx[i + 2]].pos - v[idx[i]].pos).y < 0) std::swap(idx[i + 1], idx[i + 2]);
    compute_normals(v, idx);
    m_holes.create(v, idx, false);
    m_hole_mat = make_pothole_material();
}

void RoadRender::draw(Renderer& r) const {
    for (const auto& c : m_chunks) r.draw_mesh(c.get(), m_mat.get(), mat4());
    if (m_hole_mat) r.draw_mesh(&m_holes, m_hole_mat.get(), mat4());
}

std::vector<float> RoadRender::terrain_drop(const phys::RoadSurface& road, const phys::Heightfield& hf) {
    std::vector<float> drop((size_t)hf.nx() * hf.nz(), 0.0f);
    const vec2 o = hf.origin();
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) {
            float s, lat;
            if (!road.locate(o.x + x * hf.cell(), o.y + z * hf.cell(), s, lat)) continue;
            // lowered under the inner road (where ruts and potholes dip below the bed); the verge detail is >= 0
            if (std::fabs(lat) < road.half_width - 0.45f) drop[(size_t)z * hf.nx() + x] = 0.6f;
        }
    return drop;
}

namespace terrain_edit {

void add_noise(phys::Heightfield& hf, float amp, float scale, int octaves, uint32_t seed) {
    const vec2 o = hf.origin();
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) {
            float wx = o.x + x * hf.cell(), wz = o.y + z * hf.cell();
            hf.h(x, z) += fbm2(wx / scale, wz / scale, octaves, 2.0f, 0.5f, seed) * amp;
        }
}

void flatten_rect(phys::Heightfield& hf, vec2 center, vec2 half, float yaw, float h, float falloff, int surface) {
    const vec2 o = hf.origin();
    float cs = std::cos(yaw), sn = std::sin(yaw);
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) {
            vec2 w(o.x + x * hf.cell() - center.x, o.y + z * hf.cell() - center.y);
            vec2 l(w.x * cs + w.y * sn, -w.x * sn + w.y * cs);
            float dx = std::max(0.0f, std::fabs(l.x) - half.x), dz = std::max(0.0f, std::fabs(l.y) - half.y);
            float d = std::sqrt(dx * dx + dz * dz);
            if (d >= falloff) continue;
            float t = falloff > 0 ? smoothstepf(0, 1, 1.0f - d / falloff) : 1.0f;
            if (d <= 0) t = 1.0f;
            hf.h(x, z) = lerpf(hf.h(x, z), h, t);
            if (surface >= 0 && d <= 0.01f) hf.surf(x, z) = (uint8_t)surface;
        }
}

void flatten_circle(phys::Heightfield& hf, vec2 c, float r, float h, float falloff, int surface) {
    const vec2 o = hf.origin();
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) {
            float d = length(vec2(o.x + x * hf.cell(), o.y + z * hf.cell()) - c) - r;
            if (d >= falloff) continue;
            float t = d <= 0 ? 1.0f : smoothstepf(0, 1, 1.0f - d / falloff);
            hf.h(x, z) = lerpf(hf.h(x, z), h, t);
            if (surface >= 0 && d <= 0) hf.surf(x, z) = (uint8_t)surface;
        }
}

void carve_channel(phys::Heightfield& hf, vec2 a, vec2 b, float bottom_h, float half_width, float falloff) {
    const vec2 o = hf.origin();
    vec2 ab = b - a;
    float l2 = dot(ab, ab);
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) {
            vec2 p(o.x + x * hf.cell(), o.y + z * hf.cell());
            float t = clampf(dot(p - a, ab) / l2, 0, 1);
            float d = length(p - (a + ab * t)) - half_width;
            if (d >= falloff) continue;
            float k = d <= 0 ? 1.0f : 1.0f - d / falloff;
            k = std::pow(k, 0.35f); // steep walls
            float target = bottom_h + fbm2(p.x * 0.1f, p.y * 0.1f, 2, 2.0f, 0.5f, 5) * 0.8f;
            if (target < hf.h(x, z)) hf.h(x, z) = lerpf(hf.h(x, z), target, k);
            if (k > 0.3f) hf.surf(x, z) = phys::SURF_ROCK;
        }
}

void paint_road(phys::Heightfield& hf, const std::vector<vec2>& pts, float half_width, int surface, float smooth) {
    const vec2 o = hf.origin();
    const float c = hf.cell();
    // smooth heights along the road cross-section (average across width)
    std::vector<float> orig(hf.nx() * (size_t)hf.nz());
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) orig[(size_t)z * hf.nx() + x] = hf.h(x, z);
    for (size_t s = 0; s + 1 < pts.size(); s++) {
        vec2 a = pts[s], b = pts[s + 1], ab = b - a;
        float l2 = std::max(1e-6f, dot(ab, ab));
        float reach = half_width + 3.0f;
        int x0 = std::max(0, (int)((std::min(a.x, b.x) - reach - o.x) / c)), x1 = std::min(hf.nx() - 1, (int)((std::max(a.x, b.x) + reach - o.x) / c) + 1);
        int z0 = std::max(0, (int)((std::min(a.y, b.y) - reach - o.y) / c)), z1 = std::min(hf.nz() - 1, (int)((std::max(a.y, b.y) + reach - o.y) / c) + 1);
        for (int z = z0; z <= z1; z++)
            for (int x = x0; x <= x1; x++) {
                vec2 p(o.x + x * c, o.y + z * c);
                float t = clampf(dot(p - a, ab) / l2, 0, 1);
                vec2 q = a + ab * t;
                float d = length(p - q);
                if (d > reach) continue;
                // height at the road center line: bilinear sample of the original terrain
                float fx = clampf((q.x - o.x) / c, 0.0f, (float)(hf.nx() - 1.001f));
                float fz = clampf((q.y - o.y) / c, 0.0f, (float)(hf.nz() - 1.001f));
                int ix = (int)fx, iz = (int)fz;
                float tx = fx - ix, tz = fz - iz;
                auto H = [&](int xx, int zz) { return orig[(size_t)zz * hf.nx() + xx]; };
                float hc = lerpf(lerpf(H(ix, iz), H(ix + 1, iz), tx), lerpf(H(ix, iz + 1), H(ix + 1, iz + 1), tx), tz);
                float k = d <= half_width ? smooth : smooth * smoothstepf(0.0f, 1.0f, 1.0f - (d - half_width) / 3.0f);
                hf.h(x, z) = lerpf(hf.h(x, z), hc, clampf(k, 0, 1));
                if (d <= half_width) hf.surf(x, z) = (uint8_t)surface;
            }
    }
}

void paint_circle(phys::Heightfield& hf, vec2 cc, float r, int surface) {
    const vec2 o = hf.origin();
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++)
            if (length(vec2(o.x + x * hf.cell(), o.y + z * hf.cell()) - cc) <= r) hf.surf(x, z) = (uint8_t)surface;
}

void auto_surfaces(phys::Heightfield& hf) {
    for (int z = 0; z < hf.nz(); z++)
        for (int x = 0; x < hf.nx(); x++) {
            uint8_t s = hf.surf(x, z);
            if (s != phys::SURF_GRASS) continue;
            vec3 n = hf.vertex_normal(x, z);
            if (n.y < 0.62f) hf.surf(x, z) = phys::SURF_ROCK;
            else if (n.y < 0.8f) hf.surf(x, z) = phys::SURF_DIRT;
        }
}

} // namespace terrain_edit
} // namespace bl
