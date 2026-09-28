layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_uv;
#ifdef INSTANCED
layout(location = 4) in vec4 i_row0;
layout(location = 5) in vec4 i_row1;
layout(location = 6) in vec4 i_row2;
layout(location = 7) in vec4 i_color;
#endif
#ifdef VCOLOR
layout(location = 3) in vec4 a_color;
layout(location = 8) in vec2 a_uv2;
out vec2 v_uv2;
#endif
uniform mat4 u_model;
uniform mat4 u_viewproj;
uniform float u_time;
uniform vec3 u_wind;

out vec3 v_wpos;
out vec3 v_normal;
out vec2 v_uv;
out vec4 v_color;

void main() {
#ifdef INSTANCED
    mat4 M = transpose(mat4(i_row0, i_row1, i_row2, vec4(0.0, 0.0, 0.0, 1.0)));
    v_color = i_color;
#else
    mat4 M = u_model;
    v_color = vec4(1.0);
#endif
#ifdef VCOLOR
    v_color = a_color;
    v_uv2 = a_uv2;
#endif
    vec4 wp = M * vec4(a_pos, 1.0);
    mat3 nm = mat3(M);
    v_normal = nm * a_normal;
#ifdef WIND
    // leaf flutter: small high-frequency motion scaled by distance from the card center
    float ph = dot(wp.xyz, vec3(0.7, 0.3, 0.5)) + u_time * 3.0;
    float amp = length(u_wind) * 0.02 + 0.01;
    wp.xyz += vec3(sin(ph * 1.3), sin(ph * 2.1) * 0.5, cos(ph * 1.7)) * amp * (a_uv.y);
#endif
    v_wpos = wp.xyz;
    v_uv = a_uv;
    gl_Position = u_viewproj * wp;
}
