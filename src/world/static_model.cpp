#include "world/static_model.h"

#include "core/util.h"

#include <cstdio>
#include <cstring>
#include <map>
#include <memory>

namespace bl {

MaterialPtr pbr_material(const std::string& name, MaterialPtr fallback) {
    static std::map<std::string, MaterialPtr> cache;
    if (auto it = cache.find(name); it != cache.end()) return it->second ? it->second : fallback;
    TextureCache& tc = TextureCache::get();
    const std::string base = asset_path("textures/" + name);
    MaterialPtr m;
    if (TexturePtr c = tc.load(base + "_color.jpg")) {
        m = std::make_shared<Material>();
        m->name = name, m->diffuse = c;
        m->normal_map = tc.load(base + "_normal.jpg", false);
        m->rough_map = tc.load(base + "_rough.jpg", false);
        if (FILE* f = fopen((base + "_ao.jpg").c_str(), "rb")) { // (not every set has one)
            fclose(f);
            m->ao_map = tc.load(base + "_ao.jpg", false);
        }
    }
    cache[name] = m;
    return m ? m : fallback;
}

const StaticModel* static_model(const std::string& id) {
    static std::map<std::string, std::unique_ptr<StaticModel>> cache;
    if (auto it = cache.find(id); it != cache.end()) return it->second.get();
    std::unique_ptr<StaticModel> out;
    const std::string dir = asset_path("models/" + id + "/" + id);
    if (FILE* f = fopen((dir + ".blm").c_str(), "rb")) {
        char magic[4];
        uint32_t nv = 0, ni = 0;
        if (fread(magic, 1, 4, f) == 4 && std::memcmp(magic, "BLM1", 4) == 0 && fread(&nv, 4, 1, f) == 1 && fread(&ni, 4, 1, f) == 1 && nv > 0 && ni > 0 && nv < (1u << 26) &&
            ni < (1u << 28)) {
            std::vector<Vertex> v(nv);
            std::vector<uint32_t> idx(ni);
            if (fread(v.data(), sizeof(Vertex), nv, f) == nv && fread(idx.data(), 4, ni, f) == ni) {
                out = std::make_unique<StaticModel>();
                for (const Vertex& x : v) out->bounds.add(x.pos);
                out->mesh.create(v, idx, false);
                out->triangles = (int)(ni / 3);
                TextureCache& tc = TextureCache::get();
                auto m = std::make_shared<Material>();
                m->name = id;
                m->diffuse = tc.load(dir + "_diff.jpg");
                m->normal_map = tc.load(dir + "_nor_gl.jpg", false);
                m->rough_map = tc.load(dir + "_arm.jpg", false);
                m->arm = m->rough_map != nullptr;
                out->mat = m;
            }
        }
        fclose(f);
    }
    if (out)
        if (FILE* f = fopen((dir + ".hull").c_str(), "r")) {
            char line[256];
            while (fgets(line, sizeof line, f)) {
                vec4 p;
                if (sscanf(line, "plane %f %f %f %f", &p.x, &p.y, &p.z, &p.w) == 4) out->planes.push_back(p);
            }
            fclose(f);
        }
    const StaticModel* r = out.get();
    cache[id] = std::move(out);
    return r;
}

bool panorama_sky(LightSettings& light) {
    TexturePtr sky = TextureCache::get().load(asset_path("textures/sky.jpg"));
    if (!sky) return false;
    light.sky_panorama = sky;
    std::string txt;
    vec3 sun(0.54f, 0.77f, 0.35f);
    if (read_text_file(asset_path("textures/sky_sun.txt"), txt)) sscanf(txt.c_str(), "%f %f %f", &sun.x, &sun.y, &sun.z);
    light.sun_dir = normalize(sun);
    light.sun_color = vec3(3.1f, 2.85f, 2.45f), light.sky_color = vec3(0.30f, 0.43f, 0.70f), light.fog_color = vec3(0.70f, 0.78f, 0.88f);
    light.ground_color = vec3(0.20f, 0.19f, 0.16f);
    light.fog_density = 0.0006f;
    return true;
}

} // namespace bl
