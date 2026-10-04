// The picture from the scene's light: the occlusion over it, the bloom of its brights added, the film curve (ACES),
// a little more contrast and colour, a faint vignette.
in vec2 v_ndc;
uniform sampler2D u_color;   // the scene, linear, exposed
uniform sampler2D u_ao;
uniform sampler2D u_bloom;
uniform float u_ao_strength;
uniform float u_bloom_strength;
uniform float u_debug;   // 1: the occlusion alone
out vec4 frag;
void main() {
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec3 c = texture(u_color, uv).rgb;
    float ao = texture(u_ao, uv).r;
    c *= mix(1.0, ao, u_ao_strength);
    c += texture(u_bloom, uv).rgb * u_bloom_strength;
    float lum = dot(c, vec3(0.2126, 0.7152, 0.0722));
    c = mix(c, vec3(lum), 0.35 * smoothstep(0.6, 6.0, max(c.r, max(c.g, c.b))));   // (film: the brightest colours wash toward white)
    c = clamp((c * (2.51 * c + 0.03)) / (c * (2.43 * c + 0.59) + 0.14), 0.0, 1.0);   // (ACES fitted, Narkowicz)
    c = pow(c, vec3(1.0 / 2.2));
    float l = dot(c, vec3(0.2126, 0.7152, 0.0722));
    c = mix(vec3(l), c, 1.14);
    c = (c - 0.5) * 1.07 + 0.5;
    c *= 1.0 - 0.16 * pow(length(v_ndc) * 0.72, 3.0);
    frag = vec4(clamp(c, 0.0, 1.0), 1.0);
    if (u_debug > 0.5) frag = vec4(vec3(ao), 1.0);
}
