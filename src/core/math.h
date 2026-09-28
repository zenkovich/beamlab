// Small self-contained 3D math library (vectors, quaternions, matrices).
// Conventions: right-handed, Y-up (same as Rigs of Rods / OGRE), column-major matrices.
#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>

namespace bl {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kRad2Deg = 180.0f / kPi;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float saturate(float v) { return clampf(v, 0.0f, 1.0f); }
inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
inline float smoothstepf(float e0, float e1, float x) {
    float t = saturate((x - e0) / (e1 - e0));
    return t * t * (3.0f - 2.0f * t);
}
inline float signf(float v) { return v < 0.0f ? -1.0f : 1.0f; }
inline float sqr(float v) { return v * v; }

// ------------------------------------------------------------------ vec2
struct vec2 {
    float x = 0, y = 0;
    constexpr vec2() = default;
    constexpr vec2(float x_, float y_) : x(x_), y(y_) {}
    constexpr explicit vec2(float s) : x(s), y(s) {}
    vec2 operator+(vec2 b) const { return {x + b.x, y + b.y}; }
    vec2 operator-(vec2 b) const { return {x - b.x, y - b.y}; }
    vec2 operator*(float s) const { return {x * s, y * s}; }
    vec2 operator*(vec2 b) const { return {x * b.x, y * b.y}; }
    vec2 operator/(float s) const { return {x / s, y / s}; }
    vec2 operator-() const { return {-x, -y}; }
    vec2& operator+=(vec2 b) { x += b.x; y += b.y; return *this; }
    vec2& operator-=(vec2 b) { x -= b.x; y -= b.y; return *this; }
    vec2& operator*=(float s) { x *= s; y *= s; return *this; }
};
inline float dot(vec2 a, vec2 b) { return a.x * b.x + a.y * b.y; }
inline float length(vec2 a) { return std::sqrt(dot(a, a)); }
inline vec2 normalize(vec2 a) { float l = length(a); return l > 1e-12f ? a * (1.0f / l) : vec2(0, 0); }

// ------------------------------------------------------------------ vec3
struct vec3 {
    float x = 0, y = 0, z = 0;
    constexpr vec3() = default;
    constexpr vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    constexpr explicit vec3(float s) : x(s), y(s), z(s) {}
    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
    vec3 operator+(vec3 b) const { return {x + b.x, y + b.y, z + b.z}; }
    vec3 operator-(vec3 b) const { return {x - b.x, y - b.y, z - b.z}; }
    vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    vec3 operator*(vec3 b) const { return {x * b.x, y * b.y, z * b.z}; }
    vec3 operator/(float s) const { float i = 1.0f / s; return {x * i, y * i, z * i}; }
    vec3 operator/(vec3 b) const { return {x / b.x, y / b.y, z / b.z}; }
    vec3 operator-() const { return {-x, -y, -z}; }
    vec3& operator+=(vec3 b) { x += b.x; y += b.y; z += b.z; return *this; }
    vec3& operator-=(vec3 b) { x -= b.x; y -= b.y; z -= b.z; return *this; }
    vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    vec3& operator/=(float s) { float i = 1.0f / s; x *= i; y *= i; z *= i; return *this; }
    bool operator==(vec3 b) const { return x == b.x && y == b.y && z == b.z; }
    bool operator!=(vec3 b) const { return !(*this == b); }
};
inline vec3 operator*(float s, vec3 v) { return v * s; }
inline float dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline vec3 cross(vec3 a, vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float length2(vec3 a) { return dot(a, a); }
inline float length(vec3 a) { return std::sqrt(dot(a, a)); }
inline float distance(vec3 a, vec3 b) { return length(a - b); }
inline vec3 normalize(vec3 a) { float l = length(a); return l > 1e-12f ? a * (1.0f / l) : vec3(0, 0, 0); }
inline vec3 normalize_or(vec3 a, vec3 fallback) { float l = length(a); return l > 1e-12f ? a * (1.0f / l) : fallback; }
inline vec3 vmin(vec3 a, vec3 b) { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline vec3 vmax(vec3 a, vec3 b) { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }
inline vec3 vabs(vec3 a) { return {std::fabs(a.x), std::fabs(a.y), std::fabs(a.z)}; }
inline vec3 lerp(vec3 a, vec3 b, float t) { return a + (b - a) * t; }
inline float maxc(vec3 a) { return std::max(a.x, std::max(a.y, a.z)); }
inline float minc(vec3 a) { return std::min(a.x, std::min(a.y, a.z)); }
// Any unit vector perpendicular to n (n must be unit length).
inline vec3 any_perpendicular(vec3 n) {
    vec3 t = std::fabs(n.x) < 0.7f ? vec3(1, 0, 0) : vec3(0, 1, 0);
    return normalize(cross(n, t));
}

// ------------------------------------------------------------------ vec4
struct vec4 {
    float x = 0, y = 0, z = 0, w = 0;
    constexpr vec4() = default;
    constexpr vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    constexpr vec4(vec3 v, float w_) : x(v.x), y(v.y), z(v.z), w(w_) {}
    constexpr explicit vec4(float s) : x(s), y(s), z(s), w(s) {}
    vec3 xyz() const { return {x, y, z}; }
    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
    vec4 operator+(vec4 b) const { return {x + b.x, y + b.y, z + b.z, w + b.w}; }
    vec4 operator-(vec4 b) const { return {x - b.x, y - b.y, z - b.z, w - b.w}; }
    vec4 operator*(float s) const { return {x * s, y * s, z * s, w * s}; }
    vec4 operator*(vec4 b) const { return {x * b.x, y * b.y, z * b.z, w * b.w}; }
};
inline float dot(vec4 a, vec4 b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

// ------------------------------------------------------------------ mat3 (column-major)
struct mat3 {
    vec3 c[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    mat3() = default;
    mat3(vec3 c0, vec3 c1, vec3 c2) { c[0] = c0; c[1] = c1; c[2] = c2; }
    static mat3 identity() { return mat3(); }
    static mat3 diag(vec3 d) { return mat3({d.x, 0, 0}, {0, d.y, 0}, {0, 0, d.z}); }
    vec3 operator*(vec3 v) const { return c[0] * v.x + c[1] * v.y + c[2] * v.z; }
    mat3 operator*(const mat3& b) const { return mat3((*this) * b.c[0], (*this) * b.c[1], (*this) * b.c[2]); }
    mat3 operator*(float s) const { return mat3(c[0] * s, c[1] * s, c[2] * s); }
    mat3 operator+(const mat3& b) const { return mat3(c[0] + b.c[0], c[1] + b.c[1], c[2] + b.c[2]); }
    vec3 row(int i) const { return {c[0][i], c[1][i], c[2][i]}; }
};
inline mat3 transpose(const mat3& m) { return mat3(m.row(0), m.row(1), m.row(2)); }
inline float determinant(const mat3& m) { return dot(m.c[0], cross(m.c[1], m.c[2])); }
inline mat3 inverse(const mat3& m) {
    vec3 r0 = cross(m.c[1], m.c[2]);
    vec3 r1 = cross(m.c[2], m.c[0]);
    vec3 r2 = cross(m.c[0], m.c[1]);
    float det = dot(m.c[0], r0);
    float inv = std::fabs(det) > 1e-30f ? 1.0f / det : 0.0f;
    // rows of inverse are r0,r1,r2 scaled
    return transpose(mat3(r0 * inv, r1 * inv, r2 * inv));
}
inline mat3 outer(vec3 a, vec3 b) { return mat3(a * b.x, a * b.y, a * b.z); }

// ------------------------------------------------------------------ quaternion
struct quat {
    float x = 0, y = 0, z = 0, w = 1;
    constexpr quat() = default;
    constexpr quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    static quat identity() { return {}; }
    static quat axis_angle(vec3 axis, float angle) {
        float h = angle * 0.5f, s = std::sin(h);
        vec3 a = normalize(axis);
        return {a.x * s, a.y * s, a.z * s, std::cos(h)};
    }
    vec3 v() const { return {x, y, z}; }
    quat operator*(const quat& b) const {
        return {w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y - x * b.z + y * b.w + z * b.x,
                w * b.z + x * b.y - y * b.x + z * b.w,
                w * b.w - x * b.x - y * b.y - z * b.z};
    }
    quat operator*(float s) const { return {x * s, y * s, z * s, w * s}; }
    quat operator+(const quat& b) const { return {x + b.x, y + b.y, z + b.z, w + b.w}; }
    vec3 rotate(vec3 p) const {
        vec3 u{x, y, z};
        vec3 t = cross(u, p) * 2.0f;
        return p + t * w + cross(u, t);
    }
    vec3 operator*(vec3 p) const { return rotate(p); }
};
inline quat conj(const quat& q) { return {-q.x, -q.y, -q.z, q.w}; }
inline float dot(const quat& a, const quat& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
inline quat normalize(const quat& q) {
    float l = std::sqrt(dot(q, q));
    return l > 1e-12f ? q * (1.0f / l) : quat();
}
inline mat3 to_mat3(const quat& q) {
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    return mat3({1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy)},
                {2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx)},
                {2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy)});
}
inline quat from_mat3(const mat3& m) {
    float m00 = m.c[0].x, m11 = m.c[1].y, m22 = m.c[2].z;
    float tr = m00 + m11 + m22;
    quat q;
    if (tr > 0) {
        float s = std::sqrt(tr + 1.0f) * 2;
        q.w = 0.25f * s;
        q.x = (m.c[1].z - m.c[2].y) / s;
        q.y = (m.c[2].x - m.c[0].z) / s;
        q.z = (m.c[0].y - m.c[1].x) / s;
    } else if (m00 > m11 && m00 > m22) {
        float s = std::sqrt(1.0f + m00 - m11 - m22) * 2;
        q.w = (m.c[1].z - m.c[2].y) / s;
        q.x = 0.25f * s;
        q.y = (m.c[1].x + m.c[0].y) / s;
        q.z = (m.c[2].x + m.c[0].z) / s;
    } else if (m11 > m22) {
        float s = std::sqrt(1.0f + m11 - m00 - m22) * 2;
        q.w = (m.c[2].x - m.c[0].z) / s;
        q.x = (m.c[1].x + m.c[0].y) / s;
        q.y = 0.25f * s;
        q.z = (m.c[2].y + m.c[1].z) / s;
    } else {
        float s = std::sqrt(1.0f + m22 - m00 - m11) * 2;
        q.w = (m.c[0].y - m.c[1].x) / s;
        q.x = (m.c[2].x + m.c[0].z) / s;
        q.y = (m.c[2].y + m.c[1].z) / s;
        q.z = 0.25f * s;
    }
    return normalize(q);
}
// Minimal rotation taking unit vector a to unit vector b.
inline quat quat_from_to(vec3 a, vec3 b) {
    float d = dot(a, b);
    if (d < -0.999999f) {
        vec3 axis = any_perpendicular(a);
        return quat::axis_angle(axis, kPi);
    }
    vec3 c = cross(a, b);
    quat q{c.x, c.y, c.z, 1.0f + d};
    return normalize(q);
}
// Rotation vector (axis * angle) of a quaternion, shortest arc.
inline vec3 quat_log(const quat& qin) {
    quat q = qin.w < 0 ? qin * -1.0f : qin;
    vec3 v = q.v();
    float s = length(v);
    if (s < 1e-8f) return v * 2.0f;
    float angle = 2.0f * std::atan2(s, q.w);
    return v * (angle / s);
}
inline quat quat_exp(vec3 rv) {
    float a = length(rv);
    if (a < 1e-8f) return normalize(quat{rv.x * 0.5f, rv.y * 0.5f, rv.z * 0.5f, 1.0f});
    return quat::axis_angle(rv / a, a);
}
inline quat slerp(quat a, quat b, float t) {
    float d = dot(a, b);
    if (d < 0) { b = b * -1.0f; d = -d; }
    if (d > 0.9995f) return normalize(a + (b + a * -1.0f) * t);
    float th = std::acos(d), s = std::sin(th);
    return a * (std::sin((1 - t) * th) / s) + b * (std::sin(t * th) / s);
}
// Euler angles (degrees) in RoR/OGRE prop convention: rotate X, then Y, then Z (q = qz*qy*qx? see usage)
inline quat quat_euler_xyz_deg(float rx, float ry, float rz) {
    return quat::axis_angle({0, 0, 1}, rz * kDeg2Rad) * quat::axis_angle({0, 1, 0}, ry * kDeg2Rad) *
           quat::axis_angle({1, 0, 0}, rx * kDeg2Rad);
}

// ------------------------------------------------------------------ mat4 (column-major)
struct mat4 {
    vec4 c[4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    mat4() = default;
    mat4(vec4 c0, vec4 c1, vec4 c2, vec4 c3) { c[0] = c0; c[1] = c1; c[2] = c2; c[3] = c3; }
    static mat4 identity() { return mat4(); }
    const float* data() const { return &c[0].x; }
    vec4 operator*(vec4 v) const { return c[0] * v.x + c[1] * v.y + c[2] * v.z + c[3] * v.w; }
    mat4 operator*(const mat4& b) const { return mat4((*this) * b.c[0], (*this) * b.c[1], (*this) * b.c[2], (*this) * b.c[3]); }
    vec3 transform_point(vec3 p) const { return (c[0] * p.x + c[1] * p.y + c[2] * p.z + c[3]).xyz(); }
    vec3 transform_dir(vec3 d) const { return (c[0] * d.x + c[1] * d.y + c[2] * d.z).xyz(); }
    vec3 translation() const { return c[3].xyz(); }
    static mat4 translate(vec3 t) { mat4 m; m.c[3] = vec4(t, 1); return m; }
    static mat4 scale(vec3 s) { mat4 m; m.c[0].x = s.x; m.c[1].y = s.y; m.c[2].z = s.z; return m; }
    static mat4 from_mat3(const mat3& r, vec3 t = {}) {
        return mat4(vec4(r.c[0], 0), vec4(r.c[1], 0), vec4(r.c[2], 0), vec4(t, 1));
    }
    static mat4 from_trs(vec3 t, const quat& r, vec3 s = vec3(1)) {
        mat3 m = to_mat3(r);
        return mat4(vec4(m.c[0] * s.x, 0), vec4(m.c[1] * s.y, 0), vec4(m.c[2] * s.z, 0), vec4(t, 1));
    }
    mat3 upper3() const { return mat3(c[0].xyz(), c[1].xyz(), c[2].xyz()); }
};

inline mat4 perspective(float fovy_rad, float aspect, float znear, float zfar) {
    float f = 1.0f / std::tan(fovy_rad * 0.5f);
    mat4 m;
    m.c[0] = {f / aspect, 0, 0, 0};
    m.c[1] = {0, f, 0, 0};
    m.c[2] = {0, 0, (zfar + znear) / (znear - zfar), -1};
    m.c[3] = {0, 0, 2 * zfar * znear / (znear - zfar), 0};
    return m;
}
inline mat4 ortho(float l, float r, float b, float t, float n, float f) {
    mat4 m;
    m.c[0] = {2 / (r - l), 0, 0, 0};
    m.c[1] = {0, 2 / (t - b), 0, 0};
    m.c[2] = {0, 0, -2 / (f - n), 0};
    m.c[3] = {-(r + l) / (r - l), -(t + b) / (t - b), -(f + n) / (f - n), 1};
    return m;
}
inline mat4 look_at(vec3 eye, vec3 target, vec3 up) {
    vec3 f = normalize(target - eye);
    vec3 s = normalize(cross(f, up));
    if (length2(s) < 1e-12f) s = any_perpendicular(f);
    vec3 u = cross(s, f);
    mat4 m;
    m.c[0] = {s.x, u.x, -f.x, 0};
    m.c[1] = {s.y, u.y, -f.y, 0};
    m.c[2] = {s.z, u.z, -f.z, 0};
    m.c[3] = {-dot(s, eye), -dot(u, eye), dot(f, eye), 1};
    return m;
}
inline mat4 transpose(const mat4& m) {
    mat4 r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) r.c[i][j] = m.c[j][i];
    return r;
}
mat4 inverse(const mat4& m);

// ------------------------------------------------------------------ AABB
struct AABB {
    vec3 mn{1e30f, 1e30f, 1e30f};
    vec3 mx{-1e30f, -1e30f, -1e30f};
    void add(vec3 p) { mn = vmin(mn, p); mx = vmax(mx, p); }
    void add(const AABB& b) { mn = vmin(mn, b.mn); mx = vmax(mx, b.mx); }
    void expand(float r) { mn -= vec3(r); mx += vec3(r); }
    bool valid() const { return mn.x <= mx.x; }
    bool overlaps(const AABB& b) const {
        return mn.x <= b.mx.x && mx.x >= b.mn.x && mn.y <= b.mx.y && mx.y >= b.mn.y && mn.z <= b.mx.z && mx.z >= b.mn.z;
    }
    bool contains(vec3 p) const {
        return p.x >= mn.x && p.x <= mx.x && p.y >= mn.y && p.y <= mx.y && p.z >= mn.z && p.z <= mx.z;
    }
    vec3 center() const { return (mn + mx) * 0.5f; }
    vec3 extent() const { return mx - mn; }
};

// ------------------------------------------------------------------ random
struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed = 0x9E3779B97F4A7C15ull) : s(seed ? seed : 1) {}
    uint64_t next() {
        s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
        return s * 0x2545F4914F6CDD1Dull;
    }
    float uniform() { return (float)((next() >> 40) * (1.0 / 16777216.0)); }
    float range(float a, float b) { return a + (b - a) * uniform(); }
    int irange(int a, int b) { return a + (int)(next() % (uint64_t)(b - a + 1)); }
    vec3 unit_vector() {
        float z = range(-1, 1), a = range(0, 2 * kPi), r = std::sqrt(std::max(0.0f, 1 - z * z));
        return {r * std::cos(a), r * std::sin(a), z};
    }
};

// Hash-based value noise, used for procedural terrain.
float value_noise2(float x, float y, uint32_t seed = 0);
float fbm2(float x, float y, int octaves, float lacunarity = 2.0f, float gain = 0.5f, uint32_t seed = 0);

} // namespace bl
