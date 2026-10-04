// Procedural textures, meshes and materials shared by world objects.
#include "world/objects.h"

#include <cmath>

namespace bl {

namespace {

struct Canvas {
    int w, h;
    std::vector<vec4> px;
    Canvas(int w_, int h_, vec4 c) : w(w_), h(h_), px((size_t)w_ * h_, c) {}
    vec4& at(int x, int y) { return px[(size_t)((y % h + h) % h) * w + ((x % w + w) % w)]; }
    Image to_image() const {
        Image img;
        img.w = w;
        img.h = h;
        img.rgba.resize((size_t)w * h * 4);
        for (size_t i = 0; i < px.size(); i++)
            for (int k = 0; k < 4; k++) img.rgba[i * 4 + k] = (uint8_t)(clampf(px[i][k], 0, 1) * 255.0f + 0.5f);
        return img;
    }
};

TexturePtr make_tex(const Canvas& c, const char* name, bool clamp = false) {
    return TextureCache::get().from_image(c.to_image(), name, true, clamp);
}

TexturePtr gen_checker() {
    Canvas c(256, 256, vec4(1));
    for (int y = 0; y < 256; y++)
        for (int x = 0; x < 256; x++) {
            bool k = ((x / 32) + (y / 32)) & 1;
            float n = 0.92f + 0.08f * value_noise2(x * 0.2f, y * 0.2f, 3);
            vec3 col = k ? vec3(0.95f, 0.72f, 0.18f) : vec3(0.18f, 0.18f, 0.2f);
            c.at(x, y) = vec4(col * n, 1);
        }
    return make_tex(c, "<checker>");
}

// tartan: coloured warp/weft stripes interleaved by a 2/2 twill weave (thread = 2 px)
TexturePtr gen_fabric() {
    const int N = 512;
    Canvas c(N, N, vec4(1));
    auto stripe = [](int t) -> vec3 {
        int s = t % 128;
        if (s < 36) return vec3(0.62f, 0.07f, 0.07f);   // red ground
        if (s < 40) return vec3(0.95f, 0.85f, 0.45f);   // yellow line
        if (s < 64) return vec3(0.55f, 0.06f, 0.06f);
        if (s < 92) return vec3(0.07f, 0.16f, 0.12f);   // dark green band
        if (s < 96) return vec3(0.62f, 0.07f, 0.07f);
        if (s < 104) return vec3(0.05f, 0.07f, 0.18f);  // navy
        return vec3(0.60f, 0.07f, 0.07f);
    };
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
            int tx = x / 2, ty = y / 2;               // thread indices
            bool warp_on_top = ((tx + ty) / 2) & 1;   // twill diagonal
            vec3 col = warp_on_top ? stripe(tx) : stripe(ty);
            // thread shading: rounded yarn + slub noise
            float across = warp_on_top ? (x & 1) : (y & 1);
            float yarn = 0.82f + 0.18f * across;
            float slub = 0.9f + 0.1f * value_noise2(x * 0.07f, y * 0.9f, 31) * (warp_on_top ? 1.0f : 0.0f) +
                         0.1f * value_noise2(x * 0.9f, y * 0.07f, 37) * (warp_on_top ? 0.0f : 1.0f);
            c.at(x, y) = vec4(col * yarn * slub, 1);
        }
    return make_tex(c, "<fabric>");
}

// start / finish banner: black and white checks, 30 x 2 (a 12 m x 0.8 m cross bar)
TexturePtr gen_banner() {
    const int W = 960, H = 64;
    Canvas c(W, H, vec4(1));
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            bool k = ((x / 32) + (y / 32)) & 1;
            c.at(x, y) = vec4(k ? vec3(0.06f) : vec3(0.95f), 1);
        }
    return make_tex(c, "<banner>");
}

// rally barrier tape: diagonal red / white stripes (u along the tape, one repeat = 4 stripes)
TexturePtr gen_tape() {
    const int W = 256, H = 32;
    Canvas c(W, H, vec4(1));
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            bool red = ((x + y) / 32) & 1;
            vec3 col = red ? vec3(0.86f, 0.08f, 0.06f) : vec3(0.96f, 0.96f, 0.94f);
            float edge = (y < 2 || y >= H - 2) ? 0.85f : 1.0f;
            float n = 0.96f + 0.04f * value_noise2(x * 0.3f, y * 0.3f, 41);
            c.at(x, y) = vec4(col * (edge * n), 1);
        }
    return make_tex(c, "<tape>");
}

// straw: fine fibres along x; `net`: round-bale net wrap (thin light lines)
TexturePtr gen_straw(bool net) {
    const int N = 256;
    Canvas c(N, N, vec4(1));
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
            float fx = x / (float)N, fy = y / (float)N;
            // streaks: stretched noise (long along x, thin across y)
            float s1 = value_noise2(fx * 6.0f, fy * 90.0f, 51) * 0.5f + 0.5f;
            float s2 = value_noise2(fx * 14.0f, fy * 160.0f, 52) * 0.5f + 0.5f;
            float blotch = fbm2(fx * 5.0f, fy * 5.0f, 3, 2.0f, 0.5f, 53) * 0.5f + 0.5f;
            vec3 light(0.86f, 0.74f, 0.44f), dark(0.55f, 0.43f, 0.2f), grey(0.62f, 0.58f, 0.44f);
            vec3 col = lerp(dark, light, clampf(0.35f + 0.5f * s1 + 0.3f * (s2 - 0.5f), 0, 1));
            col = lerp(col, grey, clampf((blotch - 0.6f) * 1.5f, 0, 0.35f)); // weathered patches
            if (net) {
                // diamond net wrap
                float a = std::fabs(std::fmod((float)(x + y) + 1000.0f, 21.0f) - 10.5f);
                float b = std::fabs(std::fmod((float)(x - y) + 1000.0f, 21.0f) - 10.5f);
                if (a > 9.8f || b > 9.8f) col = lerp(col, vec3(0.92f, 0.95f, 0.9f), 0.55f);
            }
            c.at(x, y) = vec4(col, 1);
        }
    return make_tex(c, net ? "<straw_net>" : "<straw>");
}

// round bale end: rolled spiral + fibre noise
TexturePtr gen_bale_end() {
    const int N = 256;
    Canvas c(N, N, vec4(1));
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
            float dx = (x + 0.5f) / N - 0.5f, dy = (y + 0.5f) / N - 0.5f;
            float r = std::sqrt(dx * dx + dy * dy) * 2.0f, a = std::atan2(dy, dx);
            float spiral = std::sin((r * 26.0f + a / (2 * kPi)) * 2 * kPi) * 0.5f + 0.5f;
            float fib = value_noise2(a * 40.0f, r * 8.0f, 61) * 0.5f + 0.5f;
            vec3 light(0.88f, 0.76f, 0.46f), dark(0.5f, 0.39f, 0.18f);
            vec3 col = lerp(dark, light, clampf(0.25f + 0.45f * spiral + 0.35f * fib, 0, 1));
            col *= 0.85f + 0.15f * clampf(1.0f - r, 0, 1); // darker core shadow at the rim
            c.at(x, y) = vec4(col, 1);
        }
    return make_tex(c, "<bale_end>");
}

TexturePtr gen_wood(bool planks) {
    Canvas c(256, 256, vec4(1));
    for (int y = 0; y < 256; y++)
        for (int x = 0; x < 256; x++) {
            float gx = x / 256.0f, gy = y / 256.0f;
            float grain = std::sin((gy * 40.0f + fbm2(gx * 4, gy * 16, 4, 2.0f, 0.5f, 11) * 6.0f)) * 0.5f + 0.5f;
            vec3 a(0.55f, 0.38f, 0.22f), b(0.40f, 0.26f, 0.14f);
            vec3 col = lerp(a, b, grain * 0.6f + 0.2f * value_noise2(gx * 60, gy * 4, 5));
            if (planks) {
                int plank = y / 32;
                col *= 0.85f + 0.15f * (value_noise2((float)plank * 3.1f, 0.5f, 17) * 0.5f + 0.5f);
                if (y % 32 < 2) col *= 0.35f; // gaps
                if ((x + plank * 97) % 128 < 2) col *= 0.5f;
            }
            c.at(x, y) = vec4(col, 1);
        }
    return make_tex(c, planks ? "<planks>" : "<wood>");
}

TexturePtr gen_bark(bool birch) {
    Canvas c(128, 256, vec4(1));
    for (int y = 0; y < 256; y++)
        for (int x = 0; x < 128; x++) {
            float gx = x / 128.0f, gy = y / 256.0f;
            float n = fbm2(gx * 12.0f, gy * 3.0f, 4, 2.0f, 0.55f, 23);
            vec3 col;
            if (birch) {
                col = vec3(0.86f, 0.85f, 0.80f) * (0.9f + 0.1f * n);
                float mark = fbm2(gx * 5.0f, gy * 30.0f, 3, 2.0f, 0.5f, 41);
                if (mark > 0.35f) col = vec3(0.15f, 0.14f, 0.13f);
            } else {
                float ridge = std::fabs(std::sin(gx * 6.2831f * 5.0f + n * 3.0f));
                col = lerp(vec3(0.22f, 0.16f, 0.11f), vec3(0.40f, 0.31f, 0.22f), ridge * 0.7f + 0.3f * (n * 0.5f + 0.5f));
            }
            c.at(x, y) = vec4(col, 1);
        }
    return make_tex(c, birch ? "<birch>" : "<bark>");
}

// Leaf cluster card: several leaf shapes on a transparent background.
TexturePtr gen_leaves(vec3 base, vec3 var, uint32_t seed, bool small) {
    // a spray of leaves on twigs: pointed blades with a midrib, each lit along its length, the ones drawn first (under the
    // others) darker - a card of it reads as a bough with depth, not as a blob of flat discs
    const int S = 256;
    Canvas c(S, S, vec4(base.x * 0.6f, base.y * 0.6f, base.z * 0.6f, 0));
    Rng rng(seed);
    const int twigs = small ? 9 : 7;
    const int per_twig = small ? 16 : 11;
    for (int t = 0; t < twigs; t++) {
        // a twig from near the card's middle outwards
        float ta = 2 * kPi * (t + rng.range(-0.3f, 0.3f)) / twigs;
        float tx = S * 0.5f + std::cos(ta) * S * 0.06f, ty = S * 0.5f + std::sin(ta) * S * 0.06f;
        float tl = S * rng.range(0.30f, 0.44f);
        vec3 wood(0.22f, 0.16f, 0.10f);
        for (int q = 0; q < (int)tl; q++) {
            int x = (int)(tx + std::cos(ta) * q), y = (int)(ty + std::sin(ta) * q);
            if (x > 1 && y > 1 && x < S - 2 && y < S - 2) c.at(x, y) = vec4(wood, 1), c.at(x + 1, y) = vec4(wood, 1);
        }
        for (int i = 0; i < per_twig; i++) {
            float along = rng.range(0.15f, 1.0f) * tl;
            float cx = tx + std::cos(ta) * along, cy = ty + std::sin(ta) * along;
            float ang = ta + (rng.uniform() < 0.5f ? 1.0f : -1.0f) * rng.range(0.5f, 1.2f);
            float len = (small ? rng.range(9, 14) : rng.range(15, 24)), wid = len * rng.range(0.30f, 0.42f);
            // (depth: the first leaves of a twig lie under the later ones)
            float depth = 0.55f + 0.45f * (float)i / per_twig;
            vec3 col = (base + var * rng.range(-1, 1)) * depth * rng.range(0.85f, 1.15f);
            if (rng.uniform() < 0.08f) col = col * vec3(1.5f, 1.25f, 0.6f); // (a yellowing one)
            float ca = std::cos(ang), sa = std::sin(ang);
            float ox = cx + ca * len, oy = cy + sa * len; // (the blade's middle: it grows from the twig)
            for (int y = (int)(oy - len - 2); y <= (int)(oy + len + 2); y++)
                for (int x = (int)(ox - len - 2); x <= (int)(ox + len + 2); x++) {
                    if (x < 1 || y < 1 || x >= S - 1 || y >= S - 1) continue;
                    float lx = (x - ox) * ca + (y - oy) * sa, ly = -(x - ox) * sa + (y - oy) * ca;
                    float u = lx / len; // -1 at the stalk, 1 at the tip
                    if (u < -1 || u > 1) continue;
                    float half = wid * std::pow(std::max(0.0f, 1.0f - u * u), 0.75f) * (1.0f - 0.25f * u); // (broader near the stalk, a point at the tip)
                    if (std::fabs(ly) > half) continue;
                    float rib = std::fabs(ly) < 0.7f ? 1.18f : 1.0f;
                    float vein = 0.94f + 0.06f * std::cos((lx * 0.9f + std::fabs(ly) * 1.4f));
                    float edge = 0.82f + 0.18f * (1.0f - std::fabs(ly) / std::max(half, 0.5f));
                    float side = ly > 0 ? 1.06f : 0.94f; // (the blade folded a little along its rib)
                    c.at(x, y) = vec4(col * rib * vein * edge * side, 1);
                }
        }
    }
    return make_tex(c, small ? "<leaves_small>" : "<leaves>", true);
}

TexturePtr gen_needles() {
    // a conifer's bough: twigs fanning out from the card's foot, each a brush of needles along it - the ones under the
    // others darker, their tips paler
    const int S = 256;
    Canvas c(S, S, vec4(0.07f, 0.16f, 0.08f, 0));
    Rng rng(77);
    auto line = [&](float x0, float y0, float ang, float len, vec3 col0, vec3 col1, int thick) {
        for (int q = 0; q < (int)len; q++) {
            int x = (int)(x0 + std::cos(ang) * q), y = (int)(y0 + std::sin(ang) * q);
            vec3 col = col0 + (col1 - col0) * ((float)q / len);
            for (int t = 0; t < thick; t++)
                if (x + t > 0 && y > 0 && x + t < S - 1 && y < S - 1) c.at(x + t, y) = vec4(col, 1);
        }
    };
    const int twigs = 13;
    for (int t = 0; t < twigs; t++) {
        // (the card's v runs up the bough: twigs from its middle line outwards and up)
        float along = (t + 0.5f) / twigs;
        float side = t % 2 ? 1.0f : -1.0f;
        float x0 = S * 0.5f, y0 = S * (0.96f - 0.86f * along);
        float ang = -kPi * 0.5f + side * rng.range(0.75f, 1.25f);
        float len = S * rng.range(0.26f, 0.44f) * (1.0f - 0.45f * along);
        float depth = 0.6f + 0.4f * rng.uniform();
        line(x0, y0, ang, len, vec3(0.20f, 0.14f, 0.09f), vec3(0.17f, 0.13f, 0.08f), 2);
        for (int q = 4; q < (int)len; q += 2) {
            float px = x0 + std::cos(ang) * q, py = y0 + std::sin(ang) * q;
            for (int sd = -1; sd <= 1; sd += 2) {
                float na = ang + sd * rng.range(0.55f, 1.0f);
                vec3 g = vec3(0.07f, 0.20f, 0.09f) * depth * rng.range(0.75f, 1.25f);
                line(px, py, na, rng.range(9, 16), g, g * vec3(1.5f, 1.45f, 1.2f), 1);
            }
        }
    }
    line(S * 0.5f, S * 0.98f, -kPi * 0.5f, S * 0.9f, vec3(0.21f, 0.15f, 0.10f), vec3(0.18f, 0.13f, 0.08f), 3);
    return make_tex(c, "<needles>", true);
}

TexturePtr gen_concrete() {
    Canvas c(256, 256, vec4(1));
    for (int y = 0; y < 256; y++)
        for (int x = 0; x < 256; x++) {
            float n = fbm2(x * 0.05f, y * 0.05f, 5, 2.0f, 0.55f, 31) * 0.5f + 0.5f;
            float spk = value_noise2(x * 0.9f, y * 0.9f, 9) * 0.5f + 0.5f;
            vec3 col = vec3(0.62f, 0.61f, 0.58f) * (0.8f + 0.25f * n + 0.08f * spk);
            if (x % 128 < 2 || y % 128 < 2) col *= 0.7f;
            c.at(x, y) = vec4(col, 1);
        }
    return make_tex(c, "<concrete>");
}

TexturePtr gen_crate() {
    Canvas c(256, 256, vec4(1));
    for (int y = 0; y < 256; y++)
        for (int x = 0; x < 256; x++) {
            float gx = x / 256.0f, gy = y / 256.0f;
            float grain = std::sin(gy * 50.0f + fbm2(gx * 3, gy * 20, 3, 2.0f, 0.5f, 5) * 5.0f) * 0.5f + 0.5f;
            vec3 col = lerp(vec3(0.72f, 0.55f, 0.32f), vec3(0.58f, 0.42f, 0.23f), grain * 0.7f);
            bool frame = x < 22 || x > 233 || y < 22 || y > 233;
            float d1 = std::fabs((float)x - (float)y), d2 = std::fabs((float)x - (255.0f - y));
            if (frame || d1 < 14) col *= 0.82f;
            if (frame && (x % 64 < 2)) col *= 0.6f;
            (void)d2;
            if (x == 21 || x == 234 || y == 21 || y == 234) col *= 0.4f;
            c.at(x, y) = vec4(col, 1);
        }
    return make_tex(c, "<crate>");
}

MaterialPtr mat(TexturePtr t, vec4 color, float spec = 0.2f, float gloss = 16.0f) {
    auto m = std::make_shared<Material>();
    m->diffuse = t;
    m->color = color;
    m->specular = spec;
    m->gloss = gloss;
    return m;
}

} // namespace

SharedAssets& SharedAssets::get() {
    static SharedAssets a;
    return a;
}

void SharedAssets::init() {
    std::vector<Vertex> v;
    std::vector<uint32_t> i;
    make_box(v, i, vec3(0.5f));
    box.create(v, i);
    make_cylinder(v, i, 12, 1.0f, true);
    cylinder.create(v, i);
    make_sphere(v, i, 1.0f, 20, 12);
    sphere.create(v, i);
    make_cone(v, i, 16);
    cone.create(v, i);
    // stick: box from y=0..1 with unit cross-section centered on the axis
    make_box(v, i, vec3(0.5f));
    for (auto& x : v) x.pos.y += 0.5f;
    stick.create(v, i);

    tex_checker = gen_checker();
    tex_fabric = gen_fabric();
    tex_tape = gen_tape();
    tex_wood = gen_wood(true);
    tex_bark = gen_bark(false);
    auto tex_birch = gen_bark(true);
    tex_leaf = gen_leaves(vec3(0.20f, 0.38f, 0.10f), vec3(0.05f, 0.08f, 0.03f), 5, false);
    tex_leaf2 = gen_leaves(vec3(0.38f, 0.50f, 0.14f), vec3(0.08f, 0.08f, 0.03f), 9, true);
    tex_needle = gen_needles();
    tex_concrete = gen_concrete();
    tex_crate = gen_crate();
    auto white_tex = TextureCache::get().white();

    wood = mat(tex_wood, vec4(1), 0.1f, 8);
    dark_wood = mat(gen_wood(false), vec4(0.8f, 0.75f, 0.7f, 1), 0.1f, 8);
    metal = mat(white_tex, vec4(0.55f, 0.57f, 0.6f, 1), 0.8f, 64);
    metal->reflect = 0.3f;
    rust = mat(white_tex, vec4(0.52f, 0.28f, 0.16f, 1), 0.3f, 16);
    concrete = mat(tex_concrete, vec4(1), 0.1f, 8);
    stone = mat(tex_concrete, vec4(0.8f, 0.78f, 0.74f, 1), 0.1f, 8);
    rubber = mat(white_tex, vec4(0.08f, 0.08f, 0.08f, 1), 0.2f, 8);
    orange = mat(tex_checker, vec4(1), 0.4f, 32);
    white = mat(white_tex, vec4(0.9f, 0.9f, 0.9f, 1), 0.3f, 32);
    red = mat(white_tex, vec4(0.75f, 0.12f, 0.1f, 1), 0.6f, 48);
    red->reflect = 0.15f;
    blue = mat(white_tex, vec4(0.12f, 0.3f, 0.75f, 1), 0.6f, 48);
    blue->reflect = 0.15f;
    green = mat(white_tex, vec4(0.2f, 0.6f, 0.25f, 1), 0.4f, 32);
    yellow = mat(white_tex, vec4(0.9f, 0.75f, 0.1f, 1), 0.5f, 32);
    glass = mat(white_tex, vec4(0.6f, 0.8f, 0.9f, 0.35f), 1.0f, 128);
    glass->blend = true;
    glass->reflect = 0.5f;
    jelly = mat(white_tex, vec4(0.2f, 0.85f, 0.35f, 0.8f), 0.9f, 96);
    jelly->blend = true;
    jelly->reflect = 0.3f;
    bark = mat(tex_bark, vec4(1), 0.05f, 8);
    birch_bark = mat(tex_birch, vec4(1), 0.1f, 8);
    leaves = mat(tex_leaf, vec4(1), 0.15f, 12);
    leaves->alpha_ref = 0.5f;
    leaves->double_sided = true;
    leaves->foliage = true;
    leaves2 = mat(tex_leaf2, vec4(1), 0.15f, 12);
    leaves2->alpha_ref = 0.5f;
    leaves2->double_sided = true;
    leaves2->foliage = true;
    needles = mat(tex_needle, vec4(1), 0.1f, 8);
    needles->alpha_ref = 0.45f;
    needles->double_sided = true;
    needles->foliage = true;
    bush = mat(tex_leaf2, vec4(0.8f, 0.9f, 0.7f, 1), 0.1f, 8);
    bush->alpha_ref = 0.5f;
    bush->double_sided = true;
    bush->foliage = true;
    cone_mat = mat(white_tex, vec4(0.95f, 0.35f, 0.05f, 1), 0.4f, 24);
    banner = mat(gen_banner(), vec4(1), 0.2f, 16);
    tape = mat(tex_tape, vec4(1), 0.25f, 24);
    tape->double_sided = true;
    tape->cast_shadow = false; // a 7 cm strip; its shadow only adds cascade flicker
    straw = mat(gen_straw(false), vec4(1), 0.05f, 6);
    bale_side = mat(gen_straw(true), vec4(1), 0.05f, 6);
    bale_end = mat(gen_bale_end(), vec4(1), 0.05f, 6);
    stake = mat(white_tex, vec4(0.52f, 0.38f, 0.24f, 1), 0.05f, 8);
}

// Gravel road across the whole ribbon (u: -W..W across, v: 16 m along): compacted wheel ruts, loose gravel on
// the crown and pushed to the verges, the edge breaks up into dirt and its alpha frays into the terrain's grass.
MaterialPtr make_road_material(float hw, float edge) {
    const int NU = 1024, NV = 2048;
    const float W = hw + edge, L = 16.0f; // metres across / along
    Canvas c(NU, NV, vec4(1));
    // isotropic noise in metres, tileable along the road (blend of two periods)
    auto nz = [&](float along, float lat, float per_m, uint32_t seed) {
        float a = value_noise2(along * per_m, lat * per_m, seed), b = value_noise2((along - L) * per_m, lat * per_m, seed);
        return lerpf(a, b, along / L);
    };
    for (int y = 0; y < NV; y++)
        for (int x = 0; x < NU; x++) {
            float lat = -W + 2 * W * (x + 0.5f) / NU, a = std::fabs(lat);
            float along = L * (y + 0.5f) / NV;
            float grain = nz(along, lat, 90.0f, 11);  // ~1 cm
            float pebble = nz(along, lat, 28.0f, 12); // ~3-4 cm stones
            float patch = nz(along, lat, 2.5f, 13);   // colour variation over half a metre
            vec3 base = lerp(vec3(0.53f, 0.50f, 0.45f), vec3(0.60f, 0.57f, 0.51f), patch * 0.5f + 0.5f);
            base *= 0.93f + 0.07f * grain;
            if (pebble > 0.5f) base = lerp(base, vec3(0.70f, 0.68f, 0.63f), clampf((pebble - 0.5f) * 5, 0, 1) * 0.8f);
            if (pebble < -0.6f) base = lerp(base, vec3(0.40f, 0.37f, 0.33f), clampf((-pebble - 0.6f) * 5, 0, 1) * 0.7f);
            // ruts: compacted, darker, smooth (no loose pebbles)
            float rut = std::exp(-sqr((a - 0.8f) / 0.26f));
            base = lerp(base, vec3(0.43f, 0.40f, 0.35f) * (0.97f + 0.05f * grain), rut * 0.75f);
            // loose gravel on the crown and pushed to the verges
            float loose = std::exp(-sqr(lat / 0.3f)) + std::exp(-sqr((a - (hw - 0.15f)) / 0.3f));
            base = lerp(base, vec3(0.64f, 0.61f, 0.55f) * (0.92f + 0.12f * pebble), clampf(loose, 0, 1) * 0.45f);
            // verge: dark soil and grass blades, the alpha frays into the terrain's grass
            float frayed = a - hw + 0.3f * nz(along, lat, 0.8f, 14) + 0.12f * nz(along, lat, 4.0f, 15);
            float dirt = smoothstepf(-0.25f, 0.3f, frayed);
            base = lerp(base, vec3(0.33f, 0.28f, 0.21f) * (0.9f + 0.15f * grain), dirt * 0.8f);
            float blades = nz(along * 6.0f, lat, 30.0f, 16); // streaky tufts
            float grassy = smoothstepf(0.0f, 0.55f, frayed) * clampf(0.5f + blades, 0, 1);
            base = lerp(base, vec3(0.24f, 0.32f, 0.12f) * (0.85f + 0.3f * grain), clampf(grassy, 0, 1) * 0.7f);
            float alpha = 1.0f - smoothstepf(0.3f, 0.7f, frayed + 0.12f * grain);
            c.at(x, y) = vec4(base, alpha);
        }
    auto m = std::make_shared<Material>();
    m->diffuse = make_tex(c, "<road>");
    m->alpha_ref = 0.5f;
    m->cast_shadow = false;
    m->specular = 0.06f;
    m->gloss = 12.0f;
    m->name = "road";
    return m;
}

MaterialPtr make_sheet_fx_material(int kind, int style) {
    const bool glass = style == (int)phys::ShellPattern::Radial, metal = style == (int)phys::ShellPattern::Punch,
               wood = style == (int)phys::ShellPattern::Grain;
    auto m = std::make_shared<Material>();
    m->cast_shadow = false;
    m->double_sided = true;
    switch (kind) {
    case ShellVisual::FX_LINE: // crack lines on the intact material
        m->unlit = false;
        m->blend = true;
        m->color = glass ? vec4(0.22f, 0.3f, 0.34f, 0.6f) : vec4(0.1f, 0.1f, 0.11f, 0.45f); // (a crack in glass refracts: dark)
        m->specular = 0.0f;
        break;
    case ShellVisual::FX_RIM: // along the cracks: the glint of a glass edge, fresh metal, fresh wood
        if (glass) {
            m->unlit = true;
            m->blend = true;
            m->color = vec4(0.85f, 1.0f, 0.97f, 0.8f);
        } else if (metal) {
            m->color = vec4(0.62f, 0.63f, 0.66f, 1.0f);
            m->specular = 1.0f;
            m->gloss = 80.0f;
        } else if (wood) {
            m->color = vec4(0.9f, 0.76f, 0.52f, 1.0f);
            m->specular = 0.05f;
        } else {
            m->color = vec4(0.7f, 0.7f, 0.72f, 1.0f);
            m->specular = 0.4f;
        }
        break;
    case ShellVisual::FX_CHAR: // scorched edge of a laser cut
        m->color = vec4(0.07f, 0.05f, 0.04f, 1.0f);
        m->specular = 0.08f;
        m->gloss = 8.0f;
        break;
    case ShellVisual::FX_SLIVER: // splinters of wood, burrs of metal
        m->color = wood ? vec4(0.88f, 0.74f, 0.5f, 1.0f) : vec4(0.45f, 0.46f, 0.49f, 1.0f);
        m->specular = wood ? 0.05f : 0.9f;
        m->gloss = wood ? 6.0f : 70.0f;
        break;
    default: { // FX_MARK: the mark round a point of impact (a radial texture; uv 0.5 at the point)
        const int N = 128;
        Canvas c(N, N, vec4(0));
        for (int y = 0; y < N; y++)
            for (int x = 0; x < N; x++) {
                const float dx = (x + 0.5f) / N * 2 - 1, dy = (y + 0.5f) / N * 2 - 1;
                const float r = std::sqrt(dx * dx + dy * dy), a = std::atan2(dy, dx);
                const float n = value_noise2(x * 0.3f, y * 0.3f, 91), star = value_noise2(std::cos(a) * 9.0f, std::sin(a) * 9.0f, 92);
                float alpha;
                vec3 col;
                if (glass) { // crushed glass: white powder, a starry edge
                    const float edge = 0.55f + 0.4f * star;
                    alpha = (1.0f - smoothstepf(edge * 0.4f, edge, r)) * (0.65f + 0.35f * n);
                    col = vec3(0.97f, 0.99f, 1.0f);
                } else if (metal) { // scuffed paint and a dark ring round the hole
                    const float ring = 1.0f - smoothstepf(0.0f, 0.18f, std::fabs(r - 0.72f - 0.06f * star));
                    alpha = clampf(0.28f * (1.0f - smoothstepf(0.5f, 1.0f, r)) * (0.6f + 0.4f * n) + 0.45f * ring, 0, 1);
                    col = vec3(0.1f, 0.09f, 0.085f);
                } else { // a bruise
                    alpha = 0.45f * (1.0f - smoothstepf(0.2f, 1.0f, r + 0.2f * (n - 0.5f)));
                    col = vec3(0.25f, 0.16f, 0.08f);
                }
                c.at(x, y) = vec4(col, r < 1.0f ? alpha : 0.0f);
            }
        m->diffuse = make_tex(c, glass ? "<fx crush>" : metal ? "<fx scuff>" : "<fx bruise>", true);
        m->blend = true;
        m->unlit = glass;
        m->specular = 0.1f;
        break;
    }
    }
    m->name = "sheet fx";
    return m;
}

MaterialPtr make_pothole_material() {
    const int N = 128;
    Canvas c(N, N, vec4(0));
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
            float dx = (x + 0.5f) / N * 2 - 1, dy = (y + 0.5f) / N * 2 - 1;
            float r = std::sqrt(dx * dx + dy * dy), a = std::atan2(dy, dx);
            float edge = 0.78f + 0.12f * value_noise2(std::cos(a) * 2.5f, std::sin(a) * 2.5f, 71);
            float grain = value_noise2(x * 0.35f, y * 0.35f, 72);
            vec3 mud = lerp(vec3(0.20f, 0.17f, 0.13f), vec3(0.28f, 0.24f, 0.19f), 0.5f + 0.5f * grain);
            mud = lerp(mud, vec3(0.13f, 0.12f, 0.11f), clampf(1.0f - r / (edge * 0.6f), 0, 1) * 0.6f); // wet middle
            float alpha = 1.0f - smoothstepf(edge - 0.12f, edge + 0.04f, r + 0.05f * grain);
            c.at(x, y) = vec4(mud, alpha);
        }
    auto m = std::make_shared<Material>();
    m->diffuse = make_tex(c, "<pothole>", true);
    m->alpha_ref = 0.5f;
    m->cast_shadow = false;
    m->specular = 0.35f; // wet
    m->gloss = 40.0f;
    m->name = "pothole";
    return m;
}

} // namespace bl
