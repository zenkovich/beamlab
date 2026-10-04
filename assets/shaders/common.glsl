// Shared lighting / shadow / fog code.
uniform vec3 u_cam_pos;
uniform vec3 u_sun_dir;      // direction TOWARDS the sun
uniform vec3 u_sun_color;
uniform vec3 u_sky_color;
uniform vec3 u_ground_color;
uniform vec3 u_fog_color;
uniform float u_fog_density;
uniform float u_exposure;
uniform float u_hdr;          // 1: the scene is drawn into the post-process's buffer (linear light)

uniform sampler2DArrayShadow u_shadow_map;
uniform mat4 u_shadow_mat[3];
uniform vec3 u_cascade_far;   // view-distance where each cascade ends
uniform float u_shadow_enabled;

uniform sampler2D u_env;     // the sky's panorama (equirectangular, tonemapped sRGB), for reflections
uniform float u_env_on;
uniform float u_env_yaw;

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

// What a surface mirrors from `dir`, blurred by its roughness: the sky's panorama (its picture linear again and its
// brights stretched back out: it was tonemapped), the ground's colour under the horizon; no panorama: the gradient sky.
vec3 env_radiance(vec3 dir, float rough) {
    if (u_env_on < 0.5) return sky_radiance(dir);
    float az = atan(dir.z, dir.x) + u_env_yaw;
    vec2 uv = vec2(az / 6.2831853 + 0.5, acos(clamp(dir.y, -1.0, 1.0)) / 3.14159265);
    vec3 c = srgb_to_linear(textureLod(u_env, uv, rough * 7.0).rgb);
    c *= 1.0 + 3.0 * c * c;
    vec3 ground = u_ground_color * 0.9 + u_sun_color * 0.03;
    return mix(c, ground, smoothstep(0.02, -0.12, dir.y));
}

// A sun's highlight on a surface of roughness `rough` (GGX, Schlick's F with F0 f0): what of the sun's colour it mirrors.
float ggx_spec(vec3 N, vec3 V, vec3 L, float rough, float f0) {
    float ndl = max(dot(N, L), 0.0), ndv = max(dot(N, V), 1e-3);
    vec3 H = normalize(L + V);
    float ndh = max(dot(N, H), 0.0), vdh = max(dot(V, H), 0.0);
    float a = max(rough * rough, 0.004), a2 = a * a;
    float d = ndh * ndh * (a2 - 1.0) + 1.0;
    float D = a2 / (3.14159 * d * d);
    float k = (rough + 1.0) * (rough + 1.0) / 8.0;
    float G = (ndl / (ndl * (1.0 - k) + k)) * (ndv / (ndv * (1.0 - k) + k));
    float F = f0 + (1.0 - f0) * pow(1.0 - vdh, 5.0);
    return min(D * G * F / max(4.0 * ndv, 1e-3), 40.0);
}

// A car's paint: the base coat (a paint is no brighter than 0.6 of white: a texture's pure red is toned down to it, a dark one left) lit
// as a slightly metallic lacquer, under a clear coat that mirrors the sky by its Fresnel (5% head-on, all of it at a
// glance) and the sun in a small sharp highlight.
vec3 shade_paint(vec3 albedo, vec3 N, vec3 wpos) {
    vec3 V = normalize(u_cam_pos - wpos);
    vec3 L = u_sun_dir;
    float ndl = max(dot(N, L), 0.0), ndv = max(dot(N, V), 1e-3);
    float sh = ndl > 0.0 ? shadow_factor(wpos, N) : 0.0;
    albedo /= 1.0 + 0.6 * max(albedo.r, max(albedo.g, albedo.b));
    vec3 ambient = mix(u_ground_color, u_sky_color, N.y * 0.5 + 0.5);
    vec3 base = albedo * (ambient * 0.9 + u_sun_color * ndl * sh);
    base += albedo * u_sun_color * (ggx_spec(N, V, L, 0.34, 0.6) * 0.5 * sh);     // (the base coat's broad sheen, in its colour)
    float F = 0.05 + 0.95 * pow(1.0 - ndv, 5.0);
    vec3 R = reflect(-V, N);
    vec3 env = env_radiance(R, 0.03);
    // (what the clear coat mirrors below the horizon is the road under the car: darker under it than the open ground)
    env *= mix(0.55, 1.0, smoothstep(-0.5, 0.05, R.y));
    vec3 col = base * (1.0 - F) + env * F;
    col += u_sun_color * (ggx_spec(N, V, L, 0.08, 0.05) * sh);
    return col;
}

// A lamp (its lens, its reflector behind it, a tail light's red plastic): the texture lit, brighter where it faces the
// eye (the reflector throws the light back), a clear cover mirroring the sky and the sun; a blended lens stays seen
// through where it mirrors nothing.
vec3 shade_lens(vec3 albedo, float alpha_in, vec3 N, vec3 wpos, out float alpha) {
    vec3 V = normalize(u_cam_pos - wpos);
    vec3 L = u_sun_dir;
    float ndl = max(dot(N, L), 0.0), ndv = max(dot(N, V), 1e-3);
    float sh = ndl > 0.0 ? shadow_factor(wpos, N) : 0.0;
    vec3 ambient = mix(u_ground_color, u_sky_color, N.y * 0.5 + 0.5);
    vec3 base = albedo * (ambient * 1.0 + u_sun_color * (ndl * sh)) * (0.75 + 0.6 * ndv * ndv);
    base += albedo * u_sun_color * (ggx_spec(N, V, L, 0.3, 0.8) * 0.6 * sh);   // (the reflector's sparkle)
    float F = 0.06 + 0.94 * pow(1.0 - ndv, 5.0);
    vec3 refl = env_radiance(reflect(-V, N), 0.02) * F + u_sun_color * (ggx_spec(N, V, L, 0.06, 0.06) * sh);
    float cover = clamp(dot(refl, vec3(0.3, 0.5, 0.2)) * 1.4, 0.0, 1.0);
    alpha = alpha_in + (1.0 - alpha_in) * cover;
    return (base * alpha_in * (1.0 - F) + refl) / max(alpha, 1e-3);
}

// Polished metal (chrome, a lamp's reflector): the sky mirrored in its colour, a sharp sun.
vec3 shade_chrome(vec3 albedo, vec3 N, vec3 wpos) {
    vec3 V = normalize(u_cam_pos - wpos);
    float ndv = max(dot(N, V), 1e-3);
    float sh = dot(N, u_sun_dir) > 0.0 ? shadow_factor(wpos, N) : 0.0;
    vec3 tint = mix(albedo, vec3(1.0), 0.6);
    vec3 F = tint * (0.75 + 0.25 * pow(1.0 - ndv, 5.0));
    return env_radiance(reflect(-V, N), 0.10) * F + u_sun_color * tint * (ggx_spec(N, V, u_sun_dir, 0.14, 0.9) * sh);
}

// Glass: its own dark tint by its alpha, the sky mirrored by its Fresnel and the sun's glint on top of what is seen
// through it. Returns the colour to blend and writes the alpha: opaque where it mirrors.
vec3 shade_glass(vec3 albedo, float alpha_in, vec3 N, vec3 wpos, out float alpha) {
    vec3 V = normalize(u_cam_pos - wpos);
    float ndv = max(dot(N, V), 1e-3);
    float sh = dot(N, u_sun_dir) > 0.0 ? shadow_factor(wpos, N) : 0.0;
    float F = 0.08 + 0.92 * pow(1.0 - ndv, 5.0);
    alpha_in = max(alpha_in, 0.22);   // (a pane is seen: at least its tint)
    vec3 refl = env_radiance(reflect(-V, N), 0.0) * F + u_sun_color * (ggx_spec(N, V, u_sun_dir, 0.05, 0.05) * sh);
    vec3 ambient = mix(u_ground_color, u_sky_color, N.y * 0.5 + 0.5);
    vec3 body = albedo * vec3(0.55, 0.62, 0.58) * ambient * 0.5;
    float cover = clamp(dot(refl, vec3(0.3, 0.5, 0.2)) * 1.4, 0.0, 1.0);
    alpha = alpha_in + (1.0 - alpha_in) * cover;
    return (body * alpha_in + refl) / max(alpha, 1e-3);
}

// Leaves: lit from either side with the light wrapped round them, the sun through them when it is behind (a yellow-green
// glow), no highlight; `ao` how open the leaf is to the sky (deep in the crown: dark).
vec3 shade_foliage(vec3 albedo, vec3 N, vec3 wpos, float ao) {
    vec3 V = normalize(u_cam_pos - wpos);
    vec3 L = u_sun_dir;
    float wrap = clamp((dot(N, L) + 0.6) / 1.6, 0.0, 1.0);
    float sh = mix(1.0, shadow_factor(wpos, N), 0.6);
    float through = pow(max(dot(V, -L), 0.0), 3.0) * 0.7 + 0.25;
    float open = mix(0.45, 1.0, ao);
    vec3 ambient = mix(u_ground_color * vec3(0.8, 1.0, 0.7), u_sky_color, N.y * 0.35 + 0.65) * open;
    vec3 col = albedo * (ambient * 1.25 + u_sun_color * (wrap * sh * mix(0.6, 1.0, ao)));
    col += albedo * vec3(1.35, 1.25, 0.45) * u_sun_color * (through * sh * open * (1.0 - wrap) * 0.9);
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
        col = mix(col, env_radiance(R, 0.25) * ao, r);
    }
    return col;
}

// A surface lit by its roughness: Lambert, GGX for the sun, the sky's light off it by its Fresnel (dielectric, F0 0.04).
vec3 shade_pbr(vec3 albedo, vec3 N, vec3 wpos, float rough, float ao) {
    vec3 V = normalize(u_cam_pos - wpos);
    vec3 L = u_sun_dir;
    float ndl = max(dot(N, L), 0.0), ndv = max(dot(N, V), 1e-3);
    float sh = ndl > 0.0 ? shadow_factor(wpos, N) : 0.0;
    vec3 H = normalize(L + V);
    float ndh = max(dot(N, H), 0.0), vdh = max(dot(V, H), 0.0);
    float a = max(rough * rough, 0.02), a2 = a * a;
    float d = ndh * ndh * (a2 - 1.0) + 1.0;
    float D = a2 / (3.14159 * d * d);
    float k = (rough + 1.0) * (rough + 1.0) / 8.0;
    float G = (ndl / (ndl * (1.0 - k) + k)) * (ndv / (ndv * (1.0 - k) + k));
    float F = 0.04 + 0.96 * pow(1.0 - vdh, 5.0);
    float spec = min(D * G * F / max(4.0 * ndl * ndv, 1e-3), 8.0);
    vec3 ambient = mix(u_ground_color, u_sky_color, N.y * 0.5 + 0.5) * ao;
    vec3 col = albedo * (ambient * 0.9 + u_sun_color * ndl * sh) + u_sun_color * (spec * ndl * sh);
    // the sky in it: sharper and stronger the smoother it is, at grazing angles
    float fres = 0.04 + (max(1.0 - rough, 0.04) - 0.04) * pow(1.0 - ndv, 5.0);
    col = mix(col, env_radiance(reflect(-V, N), rough) * ao, fres * (1.0 - rough) * (1.0 - rough * 0.5));   // (a rough road is no mirror)
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
    if (u_hdr > 0.5) return x;   // (into the post-process's buffer: its light as it is, the film curve there)
    // ACES fitted (Narkowicz)
    x = clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
    return pow(x, vec3(1.0 / 2.2));
}
