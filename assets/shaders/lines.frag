in vec4 v_color;
uniform float u_round;
uniform float u_hdr;
out vec4 frag;
void main() {
    if (u_round > 0.5) {
        vec2 d = gl_PointCoord * 2.0 - 1.0;
        if (dot(d, d) > 1.0) discard;
    }
    frag = v_color;
    if (u_hdr > 0.5) frag.rgb = pow(frag.rgb, vec3(2.2)) * 1.6;   // (the post-process's curve gives it back about as it was)
}
