layout(location = 0) in vec3 a_pos;
layout(location = 2) in vec2 a_uv;
#ifdef INSTANCED
layout(location = 4) in vec4 i_row0;
layout(location = 5) in vec4 i_row1;
layout(location = 6) in vec4 i_row2;
#endif
uniform mat4 u_model;
uniform mat4 u_viewproj;
out vec2 v_uv;
void main() {
#ifdef INSTANCED
    mat4 M = transpose(mat4(i_row0, i_row1, i_row2, vec4(0.0, 0.0, 0.0, 1.0)));
#else
    mat4 M = u_model;
#endif
    v_uv = a_uv;
    gl_Position = u_viewproj * (M * vec4(a_pos, 1.0));
}
