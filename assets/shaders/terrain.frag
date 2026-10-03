#include "common.glsl"
in vec3 v_wpos;
in vec3 v_normal;
in vec2 v_uv;
uniform sampler2D u_splat;   // rgb: albedo (sRGB), a: paved (asphalt/concrete) mask
uniform sampler2D u_grass;         // tiled grass: colour, normal
uniform sampler2D u_grass_normal;
uniform sampler2D u_dirt;          // tiled earth: colour, normal
uniform sampler2D u_dirt_normal;
uniform int u_has_detail;
out vec4 frag;

float hash2(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float vnoise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash2(i), hash2(i + vec2(1, 0)), f.x), mix(hash2(i + vec2(0, 1)), hash2(i + vec2(1, 1)), f.x), f.y);
}
float fbm(vec2 p) {
    float s = 0.0, a = 0.5;
    for (int i = 0; i < 4; i++) { s += a * vnoise(p); p *= 2.07; a *= 0.5; }
    return s;
}

void main() {
    vec4 sp = texture(u_splat, v_uv);
    vec3 albedo = srgb_to_linear(sp.rgb);
    vec3 N = normalize(v_normal);
    float d = distance(v_wpos, u_cam_pos);
    // multi-scale detail noise, faded with distance to avoid shimmering
    float n1 = fbm(v_wpos.xz * 0.35);
    float n2 = vnoise(v_wpos.xz * 3.0);
    float n3 = vnoise(v_wpos.xz * 17.0);
    float fine = mix(n3, 0.5, smoothstep(10.0, 40.0, d));
    float detail = 0.75 + 0.35 * n1 + 0.15 * (n2 - 0.5) + 0.2 * (fine - 0.5);
    // paved materials (asphalt=2, concrete=3) get less large-scale variation
    float paved = sp.a;
    detail = mix(detail, 0.92 + 0.08 * n1 + 0.1 * (fine - 0.5), paved);
    if (u_has_detail != 0) {
        // Grass and earth as textures, the splat their tint: grass where the splat is green, earth elsewhere (dirt, mud,
        // gravel: the splat's own colour over the earth's grain). Two scales of each against the tiling's repeat, the
        // near one fading out with distance; their normal maps on the ground's plane.
        vec2 uv = v_wpos.xz;
        float near = 1.0 - smoothstep(25.0, 90.0, d);
        vec3 g1 = srgb_to_linear(texture(u_grass, uv * 0.31).rgb), g2 = srgb_to_linear(texture(u_grass, uv.yx * 0.043).rgb);
        vec3 e1 = srgb_to_linear(texture(u_dirt, uv * 0.27).rgb), e2 = srgb_to_linear(texture(u_dirt, uv.yx * 0.051).rgb);
        vec3 grass = mix(g2, g1, 0.35 + 0.4 * near) * (0.7 + 0.35 * n1) * vec3(0.30, 0.46, 0.24);
        vec3 earth = mix(e2, e1, 0.35 + 0.4 * near) * (0.24 + 0.1 * n1) * vec3(0.95, 0.84, 0.70);
        float is_grass = smoothstep(0.02, 0.10, sp.g - max(sp.r, sp.b) * 0.98) * (1.0 - paved);
        // (other ground - mud, sand, gravel, ice, painted paving: the splat's colour with the earth's grain; plain dirt
        // the earth itself)
        float el = dot(earth, vec3(0.3, 0.5, 0.2)) / 0.035;
        float is_dirt = (1.0 - smoothstep(0.07, 0.15, distance(sp.rgb, vec3(0.45, 0.36, 0.26)))) * (1.0 - paved);
        vec3 other = mix(albedo * mix(el, 1.0, 0.5), earth, is_dirt);
        // (worn bare patches in the grass)
        float bare = smoothstep(0.62, 0.78, fbm(uv * 0.06 + 13.0)) * 0.25;
        vec3 ground = mix(other, mix(grass, earth, bare), is_grass);
        albedo = mix(ground, ground * 0.9 + albedo * 0.1, smoothstep(250.0, 600.0, d));
        vec3 gn = texture(u_grass_normal, uv * 0.31).xyz * 2.0 - 1.0, en = texture(u_dirt_normal, uv * 0.27).xyz * 2.0 - 1.0;
        vec3 tn = mix(en, gn, is_grass * (1.0 - bare));
        N = normalize(N + (vec3(tn.x, 0.0, -tn.y) * (0.9 * near)));
    } else
        albedo *= detail;
    // steep slopes become rock
    float rock = smoothstep(0.7, 0.5, N.y) * (1.0 - paved);
    vec3 rock_col = srgb_to_linear(vec3(0.34, 0.32, 0.30)) * (0.7 + 0.5 * n1 + 0.2 * fine);
    albedo = mix(albedo, rock_col, rock);
    float spec = mix(0.05, 0.15, paved);
    vec3 col = shade(albedo, N, v_wpos, spec, 16.0, 0.0, 1.0);
    col = apply_fog(col, v_wpos);
    frag = vec4(tonemap(col), 1.0);
}
