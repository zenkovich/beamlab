#include "common.glsl"
in vec2 v_ndc;
uniform mat4 u_inv_viewproj;
uniform float u_clouds; // 0: a plain gradient (the model editor)
out vec4 frag;
void main() {
    vec4 a = u_inv_viewproj * vec4(v_ndc, -1.0, 1.0);
    vec4 b = u_inv_viewproj * vec4(v_ndc, 1.0, 1.0);
    vec3 dir = normalize(b.xyz / b.w - a.xyz / a.w);
    vec3 col = sky_radiance(dir);
    // soft procedural clouds
    if (dir.y > 0.0 && u_clouds > 0.5) {
        vec2 uv = dir.xz / (dir.y + 0.15) * 1.5;
        float n = 0.0, amp = 0.5;
        for (int i = 0; i < 5; i++) {
            vec2 f = fract(uv), c = floor(uv);
            f = f * f * (3.0 - 2.0 * f);
            float h00 = fract(sin(dot(c, vec2(127.1, 311.7))) * 43758.5453);
            float h10 = fract(sin(dot(c + vec2(1, 0), vec2(127.1, 311.7))) * 43758.5453);
            float h01 = fract(sin(dot(c + vec2(0, 1), vec2(127.1, 311.7))) * 43758.5453);
            float h11 = fract(sin(dot(c + vec2(1, 1), vec2(127.1, 311.7))) * 43758.5453);
            n += amp * mix(mix(h00, h10, f.x), mix(h01, h11, f.x), f.y);
            uv *= 2.03;
            amp *= 0.5;
        }
        float cl = smoothstep(0.55, 0.85, n) * smoothstep(0.0, 0.25, dir.y);
        col = mix(col, vec3(1.0) * (u_sun_color * 0.25 + u_sky_color * 0.6), cl * 0.75);
    }
    frag = vec4(tonemap(col), 1.0);
}
