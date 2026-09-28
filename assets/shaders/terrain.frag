#include "common.glsl"
in vec3 v_wpos;
in vec3 v_normal;
in vec2 v_uv;
uniform sampler2D u_splat;   // rgb: albedo (sRGB), a: paved (asphalt/concrete) mask
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
