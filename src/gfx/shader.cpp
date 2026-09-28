#include "gfx/shader.h"
#include "core/util.h"

#include <cstring>
#include <vector>

namespace bl {

namespace gl {
bool has_extension(const char* name) {
    GLint n = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &n);
    for (GLint i = 0; i < n; i++) {
        const char* e = (const char*)glGetStringi(GL_EXTENSIONS, i);
        if (e && std::strcmp(e, name) == 0) return true;
    }
    return false;
}
void check_errors(const char* where) {
    for (GLenum e = glGetError(); e != GL_NO_ERROR; e = glGetError()) log_error("GL error 0x%x at %s", e, where);
}
} // namespace gl

Shader::~Shader() {
    if (m_prog) glDeleteProgram(m_prog);
}

static GLuint compile_stage(GLenum type, const std::string& src, const std::string& name) {
    GLuint s = glCreateShader(type);
    const char* p = src.c_str();
    glShaderSource(s, 1, &p, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[8192];
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        log_error("shader '%s' (%s) compile failed:\n%s", name.c_str(), type == GL_VERTEX_SHADER ? "vs" : "fs", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

// Resolves `#include "file"` lines relative to assets/shaders.
static std::string preprocess(const std::string& src, int depth = 0) {
    if (depth > 8) return src;
    std::string out;
    size_t pos = 0;
    while (pos < src.size()) {
        size_t eol = src.find('\n', pos);
        if (eol == std::string::npos) eol = src.size();
        std::string line = src.substr(pos, eol - pos);
        std::string t = trim(line);
        if (t.rfind("#include", 0) == 0) {
            size_t a = t.find('"'), b = t.rfind('"');
            std::string inc;
            if (a != std::string::npos && b > a && read_text_file(asset_path("shaders/" + t.substr(a + 1, b - a - 1)), inc))
                out += preprocess(inc, depth + 1) + "\n";
            else
                log_error("shader include failed: %s", t.c_str());
        } else {
            out += line + "\n";
        }
        pos = eol + 1;
    }
    return out;
}

bool Shader::load(const std::string& name, const std::string& defines) {
    std::string vs, fs;
    if (!read_text_file(asset_path("shaders/" + name + ".vert"), vs) || !read_text_file(asset_path("shaders/" + name + ".frag"), fs)) {
        log_error("shader '%s' not found", name.c_str());
        return false;
    }
    std::string header = "#version 410 core\n" + defines + "\n";
    return compile(header + preprocess(vs), header + preprocess(fs), name);
}

bool Shader::compile(const std::string& vs_src, const std::string& fs_src, const std::string& name) {
    GLuint vs = compile_stage(GL_VERTEX_SHADER, vs_src, name);
    GLuint fs = compile_stage(GL_FRAGMENT_SHADER, fs_src, name);
    if (!vs || !fs) return false;
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[8192];
        glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        log_error("shader '%s' link failed:\n%s", name.c_str(), log);
        glDeleteProgram(p);
        return false;
    }
    if (m_prog) glDeleteProgram(m_prog);
    m_prog = p;
    m_locs.clear();
    return true;
}

void Shader::use() const { glUseProgram(m_prog); }

GLint Shader::loc(const char* name) {
    auto it = m_locs.find(name);
    if (it != m_locs.end()) return it->second;
    GLint l = glGetUniformLocation(m_prog, name);
    m_locs.emplace(name, l);
    return l;
}

} // namespace bl
