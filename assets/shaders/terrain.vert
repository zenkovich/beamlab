layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_uv;
uniform mat4 u_viewproj;
out vec3 v_wpos;
out vec3 v_normal;
out vec2 v_uv;
void main() {
    v_wpos = a_pos;
    v_normal = a_normal;
    v_uv = a_uv;
    gl_Position = u_viewproj * vec4(a_pos, 1.0);
}
