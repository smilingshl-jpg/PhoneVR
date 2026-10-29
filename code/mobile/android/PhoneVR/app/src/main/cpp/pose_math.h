// Small, dependency-free pose maths shared by reprojection and 6DoF fusion (PhoneVR 6DoF fork).
// Kept header-only and free of Android/ALVR types so it can be unit tested on a PC (app/src/test/cpp).
//
// Conventions (same as ALVR / OpenXR): right-handed, +X right, +Y up, -Z forward, metres.
// A pose's orientation maps the device's own axes into the world ("world from device").
#pragma once

#include <cmath>

namespace pvr6 {

constexpr float kPi = 3.14159265358979f;

struct Vec3 {
    float x = 0, y = 0, z = 0;
};

struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
};

inline Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 scale(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline Quat conj(Quat q) { return {-q.x, -q.y, -q.z, q.w}; }

inline Quat mul(Quat a, Quat b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline Quat normalize(Quat q) {
    float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n < 1e-9f) return {};
    return {q.x / n, q.y / n, q.z / n, q.w / n};
}

inline Vec3 rotate(Quat q, Vec3 v) {
    // v' = v + 2w(r x v) + 2 r x (r x v), r = vector part
    Vec3 r{q.x, q.y, q.z};
    Vec3 t = scale(cross(r, v), 2.0f);
    return add(add(v, scale(t, q.w)), cross(r, t));
}

inline Quat fromAxisAngle(Vec3 axis, float angle) {
    float n = length(axis);
    if (n < 1e-9f) return {};
    float s = std::sin(angle * 0.5f) / n;
    return {axis.x * s, axis.y * s, axis.z * s, std::cos(angle * 0.5f)};
}

// Rotation that takes `from` to `to` (both world-from-device): to = delta * from.
inline Quat worldDelta(Quat from, Quat to) { return normalize(mul(to, conj(from))); }

// Angular velocity (rad/s, world frame) that turns q0 into q1 over dt seconds.
inline Vec3 angularVelocity(Quat q0, Quat q1, float dt) {
    if (dt <= 0) return {};
    Quat d = worldDelta(q0, q1);
    if (d.w < 0) d = {-d.x, -d.y, -d.z, -d.w};   // shortest way round
    float s = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (s < 1e-7f) return {};
    float angle = 2.0f * std::atan2(s, d.w);
    return scale(Vec3{d.x / s, d.y / s, d.z / s}, angle / dt);
}

// Heading: rotation about +Y of the device's forward (-Z) axis, radians.
inline float yawOf(Quat q) {
    Vec3 f = rotate(q, Vec3{0, 0, -1});
    return std::atan2(-f.x, -f.z);
}

inline Quat yawOnly(float yaw) { return fromAxisAngle(Vec3{0, 1, 0}, yaw); }

inline float wrapAngle(float a) {
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
}

// Column-major 3x3 rotation matrix (what glUniformMatrix3fv expects with transpose = GL_FALSE).
inline void toMat3ColumnMajor(Quat q, float out[9]) {
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    // column 0
    out[0] = 1 - 2 * (yy + zz);
    out[1] = 2 * (xy + wz);
    out[2] = 2 * (xz - wy);
    // column 1
    out[3] = 2 * (xy - wz);
    out[4] = 1 - 2 * (xx + zz);
    out[5] = 2 * (yz + wx);
    // column 2
    out[6] = 2 * (xz + wy);
    out[7] = 2 * (yz - wx);
    out[8] = 1 - 2 * (xx + yy);
}

// ---- reprojection ----
// The stream frame was rendered with the eye facing `frame`; the eye now faces `now`.
// For a view direction d in the *current* eye space, the same world direction in the *frame's* eye space
// is (frame^-1 * now) d. The shader samples the frame there.
inline Quat reprojectionRotation(Quat frame, Quat now) { return normalize(mul(conj(frame), now)); }

// Tangent-space field of view, radians in ALVR's convention (left/down negative).
struct Fov {
    float left = -0.8f, right = 0.8f, up = 0.8f, down = -0.8f;
};

// Where (u, v in [0,1], v up) a current-view pixel at (u, v) samples the frame. Returns false if the
// direction points behind the frame's eye (shown black). Mirrors the GLSL shader exactly (for tests).
inline bool reprojectUV(Quat rot, Fov cur, Fov src, float u, float v, float *outU, float *outV) {
    float tl = std::tan(cur.left), tr = std::tan(cur.right), td = std::tan(cur.down), tu = std::tan(cur.up);
    Vec3 d{tl + u * (tr - tl), td + v * (tu - td), -1.0f};
    Vec3 s = rotate(rot, d);
    if (s.z > -1e-4f) return false;
    float x = s.x / -s.z, y = s.y / -s.z;
    float sl = std::tan(src.left), sr = std::tan(src.right), sd = std::tan(src.down), su = std::tan(src.up);
    *outU = (x - sl) / (sr - sl);
    *outV = (y - sd) / (su - sd);
    return true;
}

}   // namespace pvr6
