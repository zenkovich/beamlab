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
    if (!gl_FrontFacing) N = -N;
    vec3 col;
    if (u_unlit > 0.5) col = albedo;
    else col = shade(albedo, N, v_wpos, u_spec, u_gloss, u_reflect, 1.0);
    col += u_emissive;
    col = apply_fog(col, v_wpos);
    frag = vec4(tonemap(col), base.a);
}
