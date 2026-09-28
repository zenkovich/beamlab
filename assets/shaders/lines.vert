layout(location = 0) in vec3 a_pos;
layout(location = 3) in vec4 a_color;
uniform mat4 u_viewproj;
uniform float u_point_size;
out vec4 v_color;
void main() {
    v_color = a_color;
    gl_Position = u_viewproj * vec4(a_pos, 1.0);
    gl_PointSize = u_point_size;
}
