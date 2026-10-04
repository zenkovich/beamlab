// The post-process's small filters. u_mode 0: the bright part of the picture at half size (bloom's start); 1: half size
// again (a box of four bilinear taps); 2: up to the larger level (a tent of nine), added to it; 3: a 4 x 4 box (the
// occlusion's noise smoothed).
in vec2 v_ndc;
uniform sampler2D u_src;
uniform vec2 u_texel;   // the source's texel
uniform int u_mode;
out vec4 frag;
void main() {
    vec2 uv = v_ndc * 0.5 + 0.5;
    if (u_mode == 3) {
        vec3 s = vec3(0.0);
        for (int y = -2; y < 2; y++)
            for (int x = -2; x < 2; x++) s += texture(u_src, uv + (vec2(x, y) + 0.5) * u_texel).rgb;
        frag = vec4(s / 16.0, 1.0);
        return;
    }
    if (u_mode == 2) {
        vec3 s = texture(u_src, uv).rgb * 4.0;
        s += (texture(u_src, uv + vec2(u_texel.x, 0.0)).rgb + texture(u_src, uv - vec2(u_texel.x, 0.0)).rgb + texture(u_src, uv + vec2(0.0, u_texel.y)).rgb +
              texture(u_src, uv - vec2(0.0, u_texel.y)).rgb) * 2.0;
        s += texture(u_src, uv + u_texel).rgb + texture(u_src, uv - u_texel).rgb + texture(u_src, uv + vec2(u_texel.x, -u_texel.y)).rgb +
             texture(u_src, uv + vec2(-u_texel.x, u_texel.y)).rgb;
        frag = vec4(s / 16.0, 1.0);
        return;
    }
    vec3 s = (texture(u_src, uv + u_texel * vec2(-1.0, -1.0)).rgb + texture(u_src, uv + u_texel * vec2(1.0, -1.0)).rgb +
              texture(u_src, uv + u_texel * vec2(-1.0, 1.0)).rgb + texture(u_src, uv + u_texel * vec2(1.0, 1.0)).rgb) * 0.25;
    if (u_mode == 0) {
        s = min(s, vec3(40.0));
        float l = dot(s, vec3(0.2126, 0.7152, 0.0722));
        s *= smoothstep(0.9, 2.2, l);   // (what is brighter than white: the sun's glints, the sky by the sun, lamps)
    }
    frag = vec4(s, 1.0);
}
