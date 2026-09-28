in vec2 v_uv;
uniform sampler2D u_tex;
uniform float u_alpha_ref;
void main() {
    if (u_alpha_ref > 0.0 && texture(u_tex, v_uv).a < u_alpha_ref) discard;
}
