#pragma once

#include "core/math.h"
#include "gfx/gl.h"

#include <string>
#include <unordered_map>

namespace bl {

class Shader {
public:
    Shader() = default;
    ~Shader();
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    // Loads assets/shaders/<name>.vert and .frag; `defines` is inserted after #version.
    bool load(const std::string& name, const std::string& defines = "");
    bool compile(const std::string& vs, const std::string& fs, const std::string& debug_name);
    void use() const;
    GLuint id() const { return m_prog; }

    GLint loc(const char* name);
    void set(const char* n, float v) { glUniform1f(loc(n), v); }
    void set(const char* n, int v) { glUniform1i(loc(n), v); }
    void set(const char* n, vec2 v) { glUniform2f(loc(n), v.x, v.y); }
    void set(const char* n, vec3 v) { glUniform3f(loc(n), v.x, v.y, v.z); }
    void set(const char* n, vec4 v) { glUniform4f(loc(n), v.x, v.y, v.z, v.w); }
    void set(const char* n, const mat4& m) { glUniformMatrix4fv(loc(n), 1, GL_FALSE, m.data()); }
    void set_array(const char* n, const mat4* m, int count) { glUniformMatrix4fv(loc(n), count, GL_FALSE, m[0].data()); }
    void set_array(const char* n, const float* v, int count) { glUniform1fv(loc(n), count, v); }

private:
    GLuint m_prog = 0;
    std::unordered_map<std::string, GLint> m_locs;
};

} // namespace bl
