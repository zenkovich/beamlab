#include "gfx/texture.h"
#include "core/profiler.h"
#include "core/util.h"

#include <algorithm>
#include <cstring>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_HDR
#define STBI_NO_PIC
#define STBI_NO_PNM
#include "stb_image.h"

namespace bl {

Texture::~Texture() {
    if (id) glDeleteTextures(1, &id);
}

// ------------------------------------------------------------------------------------ DDS
namespace {

#pragma pack(push, 1)
struct DDSPixelFormat {
    uint32_t size, flags, fourcc, rgb_bits, r_mask, g_mask, b_mask, a_mask;
};
struct DDSHeader {
    uint32_t size, flags, height, width, pitch, depth, mip_count;
    uint32_t reserved1[11];
    DDSPixelFormat pf;
    uint32_t caps, caps2, caps3, caps4, reserved2;
};
#pragma pack(pop)

constexpr uint32_t fourcc(char a, char b, char c, char d) {
    return (uint32_t)(uint8_t)a | ((uint32_t)(uint8_t)b << 8) | ((uint32_t)(uint8_t)c << 16) | ((uint32_t)(uint8_t)d << 24);
}

inline void rgb565(uint16_t c, uint8_t* o) {
    uint8_t r = (c >> 11) & 31, g = (c >> 5) & 63, b = c & 31;
    o[0] = (uint8_t)((r << 3) | (r >> 2));
    o[1] = (uint8_t)((g << 2) | (g >> 4));
    o[2] = (uint8_t)((b << 3) | (b >> 2));
    o[3] = 255;
}

// Decodes one DXT color block into 16 RGBA texels; alpha untouched when `force4` (DXT3/5).
void decode_color_block(const uint8_t* b, uint8_t out[16][4], bool force4) {
    uint16_t c0 = (uint16_t)(b[0] | (b[1] << 8)), c1 = (uint16_t)(b[2] | (b[3] << 8));
    uint8_t pal[4][4];
    rgb565(c0, pal[0]);
    rgb565(c1, pal[1]);
    if (c0 > c1 || force4) {
        for (int k = 0; k < 3; k++) {
            pal[2][k] = (uint8_t)((2 * pal[0][k] + pal[1][k]) / 3);
            pal[3][k] = (uint8_t)((pal[0][k] + 2 * pal[1][k]) / 3);
        }
        pal[2][3] = pal[3][3] = 255;
    } else {
        for (int k = 0; k < 3; k++) pal[2][k] = (uint8_t)((pal[0][k] + pal[1][k]) / 2);
        pal[2][3] = 255;
        pal[3][0] = pal[3][1] = pal[3][2] = 0;
        pal[3][3] = 0;
    }
    uint32_t idx = (uint32_t)b[4] | ((uint32_t)b[5] << 8) | ((uint32_t)b[6] << 16) | ((uint32_t)b[7] << 24);
    for (int i = 0; i < 16; i++) {
        int s = (idx >> (2 * i)) & 3;
        out[i][0] = pal[s][0];
        out[i][1] = pal[s][1];
        out[i][2] = pal[s][2];
        if (!force4) out[i][3] = pal[s][3];
    }
}

void decode_dxt5_alpha(const uint8_t* b, uint8_t out[16][4]) {
    uint8_t a[8];
    a[0] = b[0];
    a[1] = b[1];
    if (a[0] > a[1]) {
        for (int i = 1; i < 7; i++) a[i + 1] = (uint8_t)(((7 - i) * a[0] + i * a[1]) / 7);
    } else {
        for (int i = 1; i < 5; i++) a[i + 1] = (uint8_t)(((5 - i) * a[0] + i * a[1]) / 5);
        a[6] = 0;
        a[7] = 255;
    }
    uint64_t bits = 0;
    for (int i = 0; i < 6; i++) bits |= (uint64_t)b[2 + i] << (8 * i);
    for (int i = 0; i < 16; i++) out[i][3] = a[(bits >> (3 * i)) & 7];
}

int mask_shift(uint32_t m) {
    if (!m) return 0;
    int s = 0;
    while (!(m & 1)) { m >>= 1; s++; }
    return s;
}
int mask_bits(uint32_t m) {
    int n = 0;
    while (m) { n += m & 1; m >>= 1; }
    return n;
}
uint8_t extract(uint32_t v, uint32_t mask) {
    if (!mask) return 255;
    int sh = mask_shift(mask), bits = mask_bits(mask);
    uint32_t x = (v & mask) >> sh;
    if (bits >= 8) return (uint8_t)(x >> (bits - 8));
    return (uint8_t)((x * 255) / ((1u << bits) - 1));
}

} // namespace

bool decode_dds(const uint8_t* data, size_t size, Image& out) {
    if (size < 128 || std::memcmp(data, "DDS ", 4) != 0) return false;
    DDSHeader h;
    std::memcpy(&h, data + 4, sizeof(h));
    const uint8_t* p = data + 128;
    size_t avail = size - 128;
    int w = (int)h.width, ht = (int)h.height;
    if (w <= 0 || ht <= 0 || w > 16384 || ht > 16384) return false;
    out.w = w;
    out.h = ht;
    out.rgba.assign((size_t)w * ht * 4, 255);
    uint32_t fcc = h.pf.fourcc;
    bool compressed = (h.pf.flags & 0x4) != 0;
    if (compressed && (fcc == fourcc('D', 'X', 'T', '1') || fcc == fourcc('D', 'X', 'T', '3') || fcc == fourcc('D', 'X', 'T', '5'))) {
        int bw = (w + 3) / 4, bh = (ht + 3) / 4;
        size_t bsize = fcc == fourcc('D', 'X', 'T', '1') ? 8 : 16;
        if (avail < (size_t)bw * bh * bsize) return false;
        uint8_t block[16][4];
        for (int by = 0; by < bh; by++)
            for (int bx = 0; bx < bw; bx++) {
                const uint8_t* b = p + ((size_t)by * bw + bx) * bsize;
                if (fcc == fourcc('D', 'X', 'T', '1')) {
                    decode_color_block(b, block, false);
                } else if (fcc == fourcc('D', 'X', 'T', '3')) {
                    for (int i = 0; i < 16; i++) {
                        int nib = (b[i / 2] >> ((i & 1) * 4)) & 15;
                        block[i][3] = (uint8_t)(nib * 17);
                    }
                    decode_color_block(b + 8, block, true);
                } else {
                    decode_dxt5_alpha(b, block);
                    decode_color_block(b + 8, block, true);
                }
                for (int i = 0; i < 16; i++) {
                    int x = bx * 4 + (i & 3), y = by * 4 + (i >> 2);
                    if (x < w && y < ht) std::memcpy(&out.rgba[((size_t)y * w + x) * 4], block[i], 4);
                }
            }
        return true;
    }
    if (compressed) {
        log_warn("DDS: unsupported fourcc %.4s", (const char*)&fcc);
        return false;
    }
    int bpp = (int)h.pf.rgb_bits / 8;
    if (bpp < 1 || bpp > 4 || avail < (size_t)w * ht * bpp) return false;
    bool lum = (h.pf.flags & 0x20000) != 0;
    bool has_a = (h.pf.flags & 0x1) != 0;
    for (int i = 0; i < w * ht; i++) {
        uint32_t v = 0;
        std::memcpy(&v, p + (size_t)i * bpp, bpp);
        uint8_t* o = &out.rgba[(size_t)i * 4];
        if (lum) {
            o[0] = o[1] = o[2] = extract(v, h.pf.r_mask ? h.pf.r_mask : 0xFF);
            o[3] = has_a ? extract(v, h.pf.a_mask) : 255;
        } else {
            o[0] = extract(v, h.pf.r_mask);
            o[1] = extract(v, h.pf.g_mask);
            o[2] = extract(v, h.pf.b_mask);
            o[3] = has_a ? extract(v, h.pf.a_mask) : 255;
        }
    }
    return true;
}

bool load_image_file(const std::string& path, Image& out) {
    std::vector<uint8_t> file;
    if (!read_file(path, file)) return false;
    if (file.size() >= 4 && std::memcmp(file.data(), "DDS ", 4) == 0) return decode_dds(file.data(), file.size(), out);
    int w, h, c;
    uint8_t* px = stbi_load_from_memory(file.data(), (int)file.size(), &w, &h, &c, 4);
    if (!px) return false;
    out.w = w;
    out.h = h;
    out.rgba.assign(px, px + (size_t)w * h * 4);
    stbi_image_free(px);
    return true;
}

// ------------------------------------------------------------------------------------ cache
TextureCache& TextureCache::get() {
    static TextureCache tc;
    return tc;
}

void TextureCache::clear() { m_cache.clear(); }

static bool image_has_alpha(const Image& img) {
    for (size_t i = 3; i < img.rgba.size(); i += 4)
        if (img.rgba[i] < 250) return true;
    return false;
}

TexturePtr TextureCache::from_image(const Image& img, const std::string& name, bool mipmaps, bool clamp) {
    if (m_max_aniso < 0) {
        m_max_aniso = 1.0f;
        if (gl::has_extension("GL_EXT_texture_filter_anisotropic")) glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &m_max_aniso);
    }
    auto t = std::make_shared<Texture>();
    t->width = img.w;
    t->height = img.h;
    t->path = name;
    t->has_alpha = image_has_alpha(img);
    glGenTextures(1, &t->id);
    glBindTexture(GL_TEXTURE_2D, t->id);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, img.w, img.h, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.rgba.data());
    if (mipmaps) {
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, std::min(8.0f, m_max_aniso));
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    GLint wrap = clamp ? GL_CLAMP_TO_EDGE : GL_REPEAT;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
    return t;
}

TexturePtr TextureCache::upload_dds_compressed(const std::vector<uint8_t>& file, const std::string& path) {
    if (m_s3tc < 0) m_s3tc = gl::has_extension("GL_EXT_texture_compression_s3tc") ? 1 : 0;
    if (!m_s3tc || file.size() < 128) return nullptr;
    DDSHeader h;
    std::memcpy(&h, file.data() + 4, sizeof(h));
    if (!(h.pf.flags & 0x4)) return nullptr;
    GLenum fmt;
    size_t bsize = 16;
    if (h.pf.fourcc == fourcc('D', 'X', 'T', '1')) { fmt = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT; bsize = 8; }
    else if (h.pf.fourcc == fourcc('D', 'X', 'T', '3')) fmt = GL_COMPRESSED_RGBA_S3TC_DXT3_EXT;
    else if (h.pf.fourcc == fourcc('D', 'X', 'T', '5')) fmt = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
    else return nullptr;
    int levels = std::max(1u, (h.flags & 0x20000) ? h.mip_count : 1u);
    int w = (int)h.width, ht = (int)h.height;
    // Require a mip chain for good minification; otherwise fall back to CPU decode + glGenerateMipmap.
    int full = 1;
    for (int m = std::max(w, ht); m > 1; m >>= 1) full++;
    if (levels < std::min(full, 4)) return nullptr;
    if ((w & 3) || (ht & 3)) return nullptr;

    auto t = std::make_shared<Texture>();
    t->width = w;
    t->height = ht;
    t->path = path;
    // cheap alpha probe
    {
        const uint8_t* p = file.data() + 128;
        size_t nblocks = (size_t)((w + 3) / 4) * ((ht + 3) / 4);
        if (file.size() >= 128 + nblocks * bsize) {
            for (size_t i = 0; i < nblocks && !t->has_alpha; i += 7) {
                const uint8_t* b = p + i * bsize;
                if (bsize == 8) {
                    uint16_t c0 = (uint16_t)(b[0] | (b[1] << 8)), c1 = (uint16_t)(b[2] | (b[3] << 8));
                    if (c0 <= c1) {
                        uint32_t idx = (uint32_t)b[4] | ((uint32_t)b[5] << 8) | ((uint32_t)b[6] << 16) | ((uint32_t)b[7] << 24);
                        for (int k = 0; k < 16; k++)
                            if (((idx >> (2 * k)) & 3) == 3) { t->has_alpha = true; break; }
                    }
                } else if (h.pf.fourcc == fourcc('D', 'X', 'T', '5')) {
                    if (std::min(b[0], b[1]) < 245) t->has_alpha = true;
                } else {
                    for (int k = 0; k < 8; k++)
                        if (b[k] != 0xFF) { t->has_alpha = true; break; }
                }
            }
        }
    }
    glGenTextures(1, &t->id);
    glBindTexture(GL_TEXTURE_2D, t->id);
    size_t off = 128;
    int uploaded = 0;
    for (int l = 0; l < levels; l++) {
        int lw = std::max(1, w >> l), lh = std::max(1, ht >> l);
        size_t sz = (size_t)((lw + 3) / 4) * ((lh + 3) / 4) * bsize;
        if (off + sz > file.size()) break;
        glCompressedTexImage2D(GL_TEXTURE_2D, l, fmt, lw, lh, 0, (GLsizei)sz, file.data() + off);
        off += sz;
        uploaded++;
    }
    if (uploaded == 0) return nullptr;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, uploaded - 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, uploaded > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    if (m_max_aniso < 0) {
        m_max_aniso = 1.0f;
        if (gl::has_extension("GL_EXT_texture_filter_anisotropic")) glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &m_max_aniso);
    }
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, std::min(8.0f, m_max_aniso));
    if (glGetError() != GL_NO_ERROR) return nullptr;
    return t;
}

TexturePtr TextureCache::load(const std::string& path, bool) {
    for (auto& e : m_cache)
        if (e.first == path) return e.second;
    PROFILE_ZONE("Texture load");
    std::vector<uint8_t> file;
    TexturePtr t;
    if (read_file(path, file)) {
        if (file.size() >= 4 && std::memcmp(file.data(), "DDS ", 4) == 0) t = upload_dds_compressed(file, path);
        if (!t) {
            Image img;
            bool ok = false;
            if (file.size() >= 4 && std::memcmp(file.data(), "DDS ", 4) == 0)
                ok = decode_dds(file.data(), file.size(), img);
            else {
                int w, h, c;
                uint8_t* px = stbi_load_from_memory(file.data(), (int)file.size(), &w, &h, &c, 4);
                if (px) {
                    img.w = w;
                    img.h = h;
                    img.rgba.assign(px, px + (size_t)w * h * 4);
                    stbi_image_free(px);
                    ok = true;
                }
            }
            if (ok) t = from_image(img, path);
        }
    }
    if (!t) log_warn("texture load failed: %s", path.c_str());
    m_cache.emplace_back(path, t);
    return t;
}

TexturePtr TextureCache::white() {
    if (!m_white) {
        Image img;
        img.w = img.h = 4;
        img.rgba.assign(4 * 4 * 4, 255);
        m_white = from_image(img, "<white>", false);
    }
    return m_white;
}

TexturePtr TextureCache::flat_normal() {
    if (!m_normal) {
        Image img;
        img.w = img.h = 4;
        img.rgba.resize(4 * 4 * 4);
        for (int i = 0; i < 16; i++) {
            img.rgba[i * 4 + 0] = 128;
            img.rgba[i * 4 + 1] = 128;
            img.rgba[i * 4 + 2] = 255;
            img.rgba[i * 4 + 3] = 255;
        }
        m_normal = from_image(img, "<normal>", false);
    }
    return m_normal;
}

} // namespace bl
