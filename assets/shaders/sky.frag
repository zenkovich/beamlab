#include "common.glsl"
in vec2 v_ndc;
uniform mat4 u_inv_viewproj;
uniform float u_clouds; // 0: a plain gradient (the model editor)
uniform float u_panorama_on;
uniform sampler2D u_panorama; // an equirectangular sky (tonemapped sRGB)
uniform float u_sky_yaw;
out vec4 frag;
void main() {
    vec4 a = u_inv_viewproj * vec4(v_ndc, -1.0, 1.0);
    vec4 b = u_inv_viewproj * vec4(v_ndc, 1.0, 1.0);
    vec3 dir = normalize(b.xyz / b.w - a.xyz / a.w);
    if (u_panorama_on > 0.5) {
        float az = atan(dir.z, dir.x) + u_sky_yaw;
        vec2 uv = vec2(az / 6.2831853 + 0.5, acos(clamp(dir.y, -1.0, 1.0)) / 3.14159265);
        // (the seam at +-pi: the derivatives taken off the continuous direction)
        vec3 c = textureGrad(u_panorama, uv, vec2(length(dFdx(dir)) / 6.2831853, 0.0), vec2(0.0, length(dFdy(dir)) / 3.14159265)).rgb;
        // (below the horizon the ground's colour: the terrain's far edge meets it)
        c = mix(c, pow(u_fog_color, vec3(1.0 / 2.2)), smoothstep(0.0, -0.08, dir.y));
        if (u_hdr > 0.5) {   // (the picture's light again: linear, its brights stretched back out - the sun's glow blooms)
            vec3 l = srgb_to_linear(c);
            c = l * (1.0 + 5.0 * l * l * l) * 1.12 / max(u_exposure, 0.05) * 0.9;
        }
        frag = vec4(c, 1.0);
        return;
    }
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
