#include "common.glsl"
in vec3 v_wpos;
in vec3 v_normal;
in vec2 v_uv;
in vec4 v_color;

uniform sampler2D u_tex;
#ifdef VCOLOR
in vec2 v_uv2;
uniform sampler2D u_tex2;
uniform float u_dual;
#endif
uniform vec4 u_color;
uniform float u_alpha_ref;
uniform float u_blend;
uniform float u_spec;
uniform float u_gloss;
uniform float u_reflect;
uniform vec3 u_emissive;
uniform float u_unlit;
uniform float u_foliage;
uniform float u_surface;   // 0 as it is, 1 a car's paint, 2 glass, 3 polished metal, 4 a lamp
uniform float u_pbr;
uniform sampler2D u_normal_map;
uniform sampler2D u_rough_map;
uniform sampler2D u_ao_map;
uniform float u_roughness;
uniform float u_arm;
uniform float u_normal_strength;

out vec4 frag;

void main() {
#ifdef VCOLOR
    // imported scenery (RBR style): the vertex colour is baked light, a second layer is blended over the first
    vec4 tex = texture(u_tex, v_uv);
    if (u_dual > 0.5) {
        vec4 t2 = texture(u_tex2, v_uv2);
        tex = vec4(mix(tex.rgb, t2.rgb, t2.a * v_color.a), 1.0);
    } else {
        tex.a *= v_color.a;
    }
    vec4 base = tex * u_color * vec4(v_color.rgb, 1.0);
#else
    vec4 tex = texture(u_tex, v_uv);
    vec4 base = tex * u_color * v_color;
#endif
    if (u_alpha_ref > 0.0) {
        if (u_blend > 0.5) {
            // blended (glass): plain alpha rejection, keep the translucency
            if (base.a < u_alpha_ref) discard;
        } else {
            // sharpen alpha around the reference so alpha-to-coverage produces crisp but antialiased edges
            base.a = clamp((base.a - u_alpha_ref) / max(fwidth(base.a), 1e-4) + 0.5, 0.0, 1.0);
            if (base.a <= 0.0) discard;
        }
    }
    vec3 albedo = srgb_to_linear(base.rgb);
    vec3 N = normalize(v_normal);
    if (!gl_FrontFacing && u_foliage < 0.5) N = -N;
    vec3 col;
    float alpha = base.a;
    if (u_unlit > 0.5) col = albedo;
    else if (u_foliage > 0.5) col = shade_foliage(albedo, N, v_wpos, clamp(length(v_normal), 0.0, 1.0));
    else if (u_surface > 3.5) col = shade_lens(albedo, base.a, N, v_wpos, alpha);
    else if (u_surface > 2.5) col = shade_chrome(albedo, N, v_wpos);
    else if (u_surface > 1.5) col = shade_glass(albedo, base.a, N, v_wpos, alpha);
    else if (u_surface > 0.5) col = shade_paint(albedo, N, v_wpos);
#ifndef VCOLOR
    else if (u_pbr > 0.5) {
        // the normal map in the surface's own frame, from the position's and the uv's screen derivatives
        vec3 dp1 = dFdx(v_wpos), dp2 = dFdy(v_wpos);
        vec2 du1 = dFdx(v_uv), du2 = dFdy(v_uv);
        vec3 dp2p = cross(dp2, N), dp1p = cross(N, dp1);
        vec3 T = dp2p * du1.x + dp1p * du2.x, B = dp2p * du1.y + dp1p * du2.y;
        float inv = inversesqrt(max(max(dot(T, T), dot(B, B)), 1e-20));
        vec3 tn = texture(u_normal_map, v_uv).xyz * 2.0 - 1.0;
        tn.xy *= u_normal_strength;
        vec3 Np = normalize(T * (tn.x * inv) - B * (tn.y * inv) + N * tn.z);
        vec3 rm = texture(u_rough_map, v_uv).rgb;
        float rough = clamp((u_arm > 0.5 ? rm.g : rm.r) * u_roughness, 0.04, 1.0);
        col = shade_pbr(albedo, Np, v_wpos, rough, u_arm > 0.5 ? rm.r : texture(u_ao_map, v_uv).r);
    }
#endif
    else col = shade(albedo, N, v_wpos, u_spec, u_gloss, u_reflect, 1.0);
    col += u_emissive;
    col = apply_fog(col, v_wpos);
    frag = vec4(tonemap(col), alpha);
}
