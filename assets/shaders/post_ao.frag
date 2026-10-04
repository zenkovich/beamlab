// Ambient occlusion off the depth buffer (at half size): how much of the hemisphere over each pixel's surface, within
// u_radius metres, lies behind what the depth buffer shows there - corners, the gap under a car, a panel's seams go dark.
in vec2 v_ndc;
uniform sampler2D u_depth;
uniform mat4 u_proj, u_inv_proj;
uniform float u_radius;
uniform vec2 u_texel;
out vec4 frag;

vec3 view_pos(vec2 uv) {
    float d = texture(u_depth, uv).r;
    vec4 p = u_inv_proj * vec4(uv * 2.0 - 1.0, d * 2.0 - 1.0, 1.0);
    return p.xyz / p.w;
}

void main() {
    vec2 uv = v_ndc * 0.5 + 0.5;
    if (texture(u_depth, uv).r >= 0.999999) {
        frag = vec4(1.0);
        return;
    }
    // the surface's normal off its neighbours' positions, each way from the nearer one in depth (an edge: the side the
    // pixel is on); a pixel that stands off both its neighbours - a hairline gap between two meshes, a wire - is left lit
    vec3 P = view_pos(uv);
    vec3 Pl = view_pos(uv - vec2(u_texel.x, 0.0)), Pr = view_pos(uv + vec2(u_texel.x, 0.0));
    vec3 Pd = view_pos(uv - vec2(0.0, u_texel.y)), Pu = view_pos(uv + vec2(0.0, u_texel.y));
    float gap = 0.03 + 0.05 * -P.z;
    if ((abs(Pl.z - P.z) > gap && abs(Pr.z - P.z) > gap) || (abs(Pd.z - P.z) > gap && abs(Pu.z - P.z) > gap)) {
        frag = vec4(1.0);
        return;
    }
    vec3 dx = abs(Pl.z - P.z) < abs(Pr.z - P.z) ? P - Pl : Pr - P;
    vec3 dy = abs(Pd.z - P.z) < abs(Pu.z - P.z) ? P - Pd : Pu - P;
    vec3 N = normalize(cross(dx, dy));
    if (N.z < 0.0) N = -N;
    vec3 T = normalize(cross(N, abs(N.y) < 0.9 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0))), B = cross(N, T);
    float turn = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715)))) * 6.2831853;
    float radius = u_radius * clamp(-P.z * 0.35, 0.35, 1.0);   // (near the eye a smaller reach: fine detail)
    float slack = 0.02 + 0.012 * -P.z;   // (the depth buffer's grain grows with the distance: no bands on a far flat road)
    float occ = 0.0;
    const int n = 14;
    for (int i = 0; i < n; i++) {
        float a = turn + float(i) * 2.399963, r = sqrt((float(i) + 0.5) / float(n));
        vec3 dir = T * (cos(a) * r) + B * (sin(a) * r) + N * sqrt(max(1.0 - r * r, 0.0));
        float reach = mix(0.2, 1.0, (float(i) + 1.0) / float(n));
        vec3 S = P + dir * (radius * reach) + N * slack;
        vec4 q = u_proj * vec4(S, 1.0);
        vec2 suv = q.xy / q.w * 0.5 + 0.5;
        float sz = view_pos(suv).z;
        float near = smoothstep(0.0, 1.0, radius / max(abs(P.z - sz), 1e-4));   // (what is far in front does not shade it)
        occ += (sz >= S.z + slack ? 1.0 : 0.0) * near;
    }
    float ao = 1.0 - occ / float(n);
    ao = mix(ao, 1.0, smoothstep(60.0, 120.0, -P.z));
    frag = vec4(vec3(ao), 1.0);
}
