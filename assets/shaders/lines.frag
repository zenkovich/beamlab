in vec4 v_color;
uniform float u_round;
out vec4 frag;
void main() {
    if (u_round > 0.5) {
        vec2 d = gl_PointCoord * 2.0 - 1.0;
        if (dot(d, d) > 1.0) discard;
    }
    frag = v_color;
}
