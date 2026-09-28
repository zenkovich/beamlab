#pragma once

#include "gfx/gl.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace bl {

struct Texture {
    GLuint id = 0;
    int width = 0, height = 0;
    bool has_alpha = false; // at least some texels are not fully opaque
    std::string path;
    ~Texture();
};
using TexturePtr = std::shared_ptr<Texture>;

// Decoded image in RGBA8 (top row first).
struct Image {
    int w = 0, h = 0;
    std::vector<uint8_t> rgba;
};

bool load_image_file(const std::string& path, Image& out);
bool decode_dds(const uint8_t* data, size_t size, Image& out);

class TextureCache {
public:
    static TextureCache& get();
    // Loads (or returns cached) texture; returns nullptr on failure.
    TexturePtr load(const std::string& path, bool srgb_hint = true);
    TexturePtr from_image(const Image& img, const std::string& name, bool mipmaps = true, bool clamp = false);
    TexturePtr white();
    TexturePtr flat_normal();
    void clear();
    size_t count() const { return m_cache.size(); }

private:
    std::vector<std::pair<std::string, TexturePtr>> m_cache;
    TexturePtr m_white, m_normal;
    float m_max_aniso = -1.0f;
    int m_s3tc = -1;
    TexturePtr upload_dds_compressed(const std::vector<uint8_t>& file, const std::string& path);
};

} // namespace bl
