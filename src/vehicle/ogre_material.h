// Minimal OGRE material script (.material) reader + RoR managed materials.
// Extracts what a simple forward renderer needs from the first technique/pass(es).
#pragma once

#include "core/math.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace bl {

namespace ror {
struct ManagedMaterialDef;
}

struct MaterialDesc {
    std::string name;
    std::string diffuse_tex;     // resolved absolute path of the base color texture ("" = none)
    std::string emissive_tex;    // optional second-pass/lights texture ("" = none)
    vec4 diffuse{1, 1, 1, 1};    // diffuse colour (multiplies texture)
    vec3 ambient{1, 1, 1};
    vec3 specular{0, 0, 0};
    float shininess = 0.0f;
    vec3 emissive{0, 0, 0};
    bool alpha_test = false;     // alpha_rejection / alpha-tested pass
    float alpha_ref = 0.5f;      // 0..1
    bool blend = false;          // scene_blend alpha_blend / transparent
    bool double_sided = false;   // cull_hardware none / cull_software none
    bool depth_write = true;
    bool lighting = true;
    bool invisible = false;      // colour_write off, tracks/trans, etc.
    bool reflective = false;     // env_map / cubic reflection present
};

class MaterialLibrary {
public:
    // Parses every *.material file under `dir` (recursive). Texture names are resolved (case-insensitively)
    // against files under `dir`; unresolvable textures leave diffuse_tex empty.
    void load_dir(const std::string& dir);
    // Registers RoR built-in materials that vehicles reference (tracks/trans, tracks/beam, tracks/transred ...).
    void add_ror_builtins();
    // Registers RoR 'managedmaterials' entries (textures resolved inside `dir`).
    void add_managed(const ror::ManagedMaterialDef& def, const std::string& dir);
    const MaterialDesc* find(const std::string& name) const;
    size_t size() const { return m_materials.size(); }
    std::vector<std::string> warnings;

private:
    std::unordered_map<std::string, MaterialDesc> m_materials;
};

} // namespace bl
