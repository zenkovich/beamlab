// Shared lighting / shadow / fog code.
uniform vec3 u_cam_pos;
uniform vec3 u_sun_dir;      // direction TOWARDS the sun
uniform vec3 u_sun_color;
uniform vec3 u_sky_color;
uniform vec3 u_ground_color;
uniform vec3 u_fog_color;
uniform float u_fog_density;
uniform float u_exposure;

uniform sampler2DArrayShadow u_shadow_map;
uniform mat4 u_shadow_mat[3];
uniform vec3 u_cascade_far;   // view-distance where each cascade ends
uniform float u_shadow_enabled;

vec3 srgb_to_linear(vec3 c) { return pow(c, vec3(2.2)); }

float shadow_cascade(int c, vec3 wpos, vec3 N) {
    float texel = 1.0 / 2048.0;
    float nbias = (c == 0 ? 0.02 : (c == 1 ? 0.05 : 0.15));
    vec4 sp = u_shadow_mat[c] * vec4(wpos + N * nbias, 1.0);
    vec3 p = sp.xyz / sp.w * 0.5 + 0.5;
    if (p.x <= 0.0 || p.x >= 1.0 || p.y <= 0.0 || p.y >= 1.0 || p.z >= 1.0) return 1.0;
    float bias = 0.0005 * float(c + 1);
    float sum = 0.0;
    for (int y = -1; y <= 1; y++)
        for (int x = -1; x <= 1; x++)
            sum += texture(u_shadow_map, vec4(p.xy + vec2(x, y) * texel * 1.2, float(c), p.z - bias));
    return sum / 9.0;
}

float shadow_factor(vec3 wpos, vec3 N) {
    if (u_shadow_enabled < 0.5) return 1.0;
    float d = distance(wpos, u_cam_pos);
    if (d < u_cascade_far.x) {
        float s = shadow_cascade(0, wpos, N);
        float t = smoothstep(u_cascade_far.x * 0.85, u_cascade_far.x, d);
        if (t > 0.0) s = mix(s, shadow_cascade(1, wpos, N), t);
        return s;
    }
    if (d < u_cascade_far.y) {
        float s = shadow_cascade(1, wpos, N);
        float t = smoothstep(u_cascade_far.y * 0.85, u_cascade_far.y, d);
        if (t > 0.0) s = mix(s, shadow_cascade(2, wpos, N), t);
        return s;
    }
    if (d < u_cascade_far.z) return mix(shadow_cascade(2, wpos, N), 1.0, smoothstep(u_cascade_far.z * 0.8, u_cascade_far.z, d));
    return 1.0;
}

vec3 sky_radiance(vec3 dir) {
    float h = clamp(dir.y, -1.0, 1.0);
    vec3 zenith = u_sky_color * 0.9;
    vec3 horizon = mix(u_fog_color, vec3(1.0), 0.15);
    vec3 col = mix(horizon, zenith, pow(max(h, 0.0), 0.45));
    col = mix(col, u_ground_color * 0.6, smoothstep(0.0, -0.25, h));
    float sd = max(dot(dir, u_sun_dir), 0.0);
    col += u_sun_color * (pow(sd, 900.0) * 6.0 + pow(sd, 12.0) * 0.08);
    return col;
}

// Blinn-Phong + hemispheric ambient + fake sky reflection.
vec3 shade(vec3 albedo, vec3 N, vec3 wpos, float spec, float gloss, float reflectivity, float ao) {
    vec3 V = normalize(u_cam_pos - wpos);
    vec3 L = u_sun_dir;
    float ndl = max(dot(N, L), 0.0);
    float sh = ndl > 0.0 ? shadow_factor(wpos, N) : 0.0;
    vec3 H = normalize(L + V);
    float ndh = max(dot(N, H), 0.0);
    float fres = pow(1.0 - max(dot(N, V), 0.0), 5.0);
    vec3 ambient = mix(u_ground_color, u_sky_color, N.y * 0.5 + 0.5) * ao;
    vec3 col = albedo * (ambient * 0.9 + u_sun_color * ndl * sh);
    float sp = pow(ndh, gloss) * spec * (gloss + 8.0) / 25.0;
    col += u_sun_color * sp * sh * ndl;
    if (reflectivity > 0.0) {
        vec3 R = reflect(-V, N);
        float r = reflectivity * (0.04 + 0.96 * fres);
        col = mix(col, sky_radiance(R) * ao, r);
    }
    return col;
}

vec3 apply_fog(vec3 col, vec3 wpos) {
    float d = distance(wpos, u_cam_pos);
    float f = 1.0 - exp(-d * u_fog_density);
    vec3 dir = normalize(wpos - u_cam_pos);
    vec3 fc = u_fog_color + u_sun_color * pow(max(dot(dir, u_sun_dir), 0.0), 8.0) * 0.15;
    return mix(col, fc, clamp(f, 0.0, 1.0));
}

vec3 tonemap(vec3 x) {
    x *= u_exposure;
    // ACES fitted (Narkowicz)
    x = clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
    return pow(x, vec3(1.0 / 2.2));
}
