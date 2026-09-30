/*
 * Copyright (c) 2026 Chris Giles
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Chris Giles makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 */

#pragma once

#include <cmath>
#include <cstdint>

#define PI 3.14159265358979f

// Math types

struct float3
{
    float x, y, z;

    float &operator[](int i) { return ((float *)this)[i]; }
    const float &operator[](int i) const { return ((float *)this)[i]; }
};

struct float4
{
    float x, y, z, w;
};

struct quat
{
    float x, y, z, w;
};

// Row-major 3x3 matrix
struct float3x3
{
    float3 row[3];

    float3 &operator[](int i) { return row[i]; }
    const float3 &operator[](int i) const { return row[i]; }

    float3 col(int i) const { return {row[0][i], row[1][i], row[2][i]}; }
};

// Column-major 4x4 matrix (OpenGL convention), only used for rendering
struct float4x4
{
    float m[16];
};

// Axis aligned bounding box
struct AABB
{
    float3 min, max;
};

// float3 operators

inline float3 operator+(float3 a, float3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline float3 operator-(float3 a, float3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline float3 operator*(float3 a, float3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline float3 operator/(float3 a, float3 b) { return {a.x / b.x, a.y / b.y, a.z / b.z}; }
inline float3 operator*(float3 a, float b) { return {a.x * b, a.y * b, a.z * b}; }
inline float3 operator*(float a, float3 b) { return {a * b.x, a * b.y, a * b.z}; }
inline float3 operator/(float3 a, float b) { return {a.x / b, a.y / b, a.z / b}; }
inline float3 operator-(float3 v) { return {-v.x, -v.y, -v.z}; }
inline float3 &operator+=(float3 &a, float3 b) { return a = a + b; }
inline float3 &operator-=(float3 &a, float3 b) { return a = a - b; }
inline float3 &operator*=(float3 &a, float b) { return a = a * b; }

// Scalar and vector functions

inline float min(float a, float b) { return a < b ? a : b; }
inline float max(float a, float b) { return a > b ? a : b; }
inline float clamp(float x, float a, float b) { return max(a, min(b, x)); }
inline float sign(float x) { return x < 0 ? -1.0f : 1.0f; }
inline float rad(float deg) { return deg * PI / 180.0f; }

inline float dot(float3 a, float3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float lengthSq(float3 v) { return dot(v, v); }
inline float length(float3 v) { return sqrtf(dot(v, v)); }
inline float3 cross(float3 a, float3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float3 min(float3 a, float3 b) { return {min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)}; }
inline float3 max(float3 a, float3 b) { return {max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)}; }
inline float3 abs(float3 v) { return {fabsf(v.x), fabsf(v.y), fabsf(v.z)}; }
inline float3 clamp(float3 v, float3 a, float3 b) { return min(max(v, a), b); }
inline float maxComponent(float3 v) { return max(v.x, max(v.y, v.z)); }

inline float3 normalize(float3 v)
{
    float len = length(v);
    return len > 1e-12f ? v / len : float3{0, 1, 0};
}

// Returns two unit vectors orthogonal to n (and each other), used as the contact friction basis.
inline void tangents(float3 n, float3 &t1, float3 &t2)
{
    t1 = fabsf(n.x) > 0.57735f ? float3{n.y, -n.x, 0} : float3{0, n.z, -n.y};
    t1 = normalize(t1);
    t2 = cross(n, t1);
}

// float3x3 functions

inline float3 operator*(const float3x3 &a, float3 b) { return {dot(a[0], b), dot(a[1], b), dot(a[2], b)}; }
inline float3x3 operator+(const float3x3 &a, const float3x3 &b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
inline float3x3 operator-(const float3x3 &a, const float3x3 &b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline float3x3 operator*(const float3x3 &a, float b) { return {a[0] * b, a[1] * b, a[2] * b}; }

inline float3x3 operator*(const float3x3 &a, const float3x3 &b)
{
    float3x3 r;
    for (int i = 0; i < 3; i++)
        r[i] = {dot(a[i], b.col(0)), dot(a[i], b.col(1)), dot(a[i], b.col(2))};
    return r;
}

inline float3x3 transpose(const float3x3 &a) { return {a.col(0), a.col(1), a.col(2)}; }
inline float3x3 skew(float3 v) { return {float3{0, -v.z, v.y}, float3{v.z, 0, -v.x}, float3{-v.y, v.x, 0}}; }
inline float3x3 diagonal(float3 d) { return {float3{d.x, 0, 0}, float3{0, d.y, 0}, float3{0, 0, d.z}}; }
inline float3x3 identity3() { return diagonal({1, 1, 1}); }
inline float3x3 outer(float3 a, float3 b) { return {b * a.x, b * a.y, b * a.z}; }

inline float3x3 inverse(const float3x3 &a)
{
    float3 c0 = cross(a[1], a[2]);
    float3 c1 = cross(a[2], a[0]);
    float3 c2 = cross(a[0], a[1]);
    float det = dot(a[0], c0);
    return transpose(float3x3{c0, c1, c2}) * (1.0f / det);
}

// Quaternion functions

inline quat operator*(quat a, quat b)
{
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline quat conjugate(quat q) { return {-q.x, -q.y, -q.z, q.w}; }

inline quat normalize(quat q)
{
    float len = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return {q.x / len, q.y / len, q.z / len, q.w / len};
}

inline quat axisAngle(float3 axis, float angle)
{
    float3 a = normalize(axis) * sinf(0.5f * angle);
    return {a.x, a.y, a.z, cosf(0.5f * angle)};
}

inline float3 rotate(quat q, float3 v)
{
    float3 u = {q.x, q.y, q.z};
    float3 t = cross(u, v) * 2.0f;
    return v + t * q.w + cross(u, t);
}

// Rotation matrix R such that R * v == rotate(q, v)
inline float3x3 rotation(quat q)
{
    return {
        float3{1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.w * q.z), 2 * (q.x * q.z + q.w * q.y)},
        float3{2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.w * q.x)},
        float3{2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y)}};
}

// Integrates an orientation by angular velocity w over dt (first order)
inline quat integrate(quat q, float3 w, float dt)
{
    quat dq = quat{w.x, w.y, w.z, 0} * q;
    float h = 0.5f * dt;
    return normalize(quat{q.x + dq.x * h, q.y + dq.y * h, q.z + dq.z * h, q.w + dq.w * h});
}

// AABB functions

inline AABB emptyAABB() { return {{INFINITY, INFINITY, INFINITY}, {-INFINITY, -INFINITY, -INFINITY}}; }
inline void grow(AABB &box, float3 p) { box.min = min(box.min, p), box.max = max(box.max, p); }
inline AABB expand(const AABB &box, float r) { return {box.min - float3{r, r, r}, box.max + float3{r, r, r}}; }
inline AABB intersect(const AABB &a, const AABB &b) { return {max(a.min, b.min), min(a.max, b.max)}; }
inline float3 center(const AABB &box) { return (box.min + box.max) * 0.5f; }
inline float3 size(const AABB &box) { return box.max - box.min; }

inline bool overlaps(const AABB &a, const AABB &b)
{
    return a.min.x <= b.max.x && a.max.x >= b.min.x &&
           a.min.y <= b.max.y && a.max.y >= b.min.y &&
           a.min.z <= b.max.z && a.max.z >= b.min.z;
}

inline bool contains(const AABB &box, float3 p)
{
    return p.x >= box.min.x && p.y >= box.min.y && p.z >= box.min.z &&
           p.x <= box.max.x && p.y <= box.max.y && p.z <= box.max.z;
}

// Small, fast random number generator (PCG32), which gives the same sequence on every platform
struct Random
{
    uint64_t state = 0x853c49e6748fea9bULL;

    uint32_t next()
    {
        uint64_t old = state;
        state = old * 6364136223846793005ULL + 1442695040888963407ULL;
        uint32_t xorshifted = (uint32_t)(((old >> 18u) ^ old) >> 27u);
        uint32_t rot = (uint32_t)(old >> 59u);
        return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31));
    }

    float uniform() { return (next() >> 8) * (1.0f / 16777216.0f); }
    float uniform(float a, float b) { return a + (b - a) * uniform(); }
    int integer(int n) { return (int)(next() % (uint32_t)n); }
    float3 inBox(const AABB &box) { return box.min + size(box) * float3{uniform(), uniform(), uniform()}; }

    float3 onSphere()
    {
        float z = uniform(-1, 1), a = uniform(0, 2 * PI), r = sqrtf(max(0.0f, 1 - z * z));
        return {r * cosf(a), r * sinf(a), z};
    }

    quat orientation()
    {
        float u1 = uniform(), u2 = uniform(0, 2 * PI), u3 = uniform(0, 2 * PI);
        float a = sqrtf(1 - u1), b = sqrtf(u1);
        return {a * sinf(u2), a * cosf(u2), b * sinf(u3), b * cosf(u3)};
    }
};

// float4x4 functions (rendering only)

inline float4x4 operator*(const float4x4 &a, const float4x4 &b)
{
    float4x4 r;
    for (int c = 0; c < 4; c++)
        for (int rr = 0; rr < 4; rr++)
            r.m[c * 4 + rr] = a.m[0 * 4 + rr] * b.m[c * 4 + 0] + a.m[1 * 4 + rr] * b.m[c * 4 + 1] +
                              a.m[2 * 4 + rr] * b.m[c * 4 + 2] + a.m[3 * 4 + rr] * b.m[c * 4 + 3];
    return r;
}

// Builds the affine transform x -> R * (s * x) + t
inline float4x4 affine(const float3x3 &R, float s, float3 t)
{
    return {R[0][0] * s, R[1][0] * s, R[2][0] * s, 0,
            R[0][1] * s, R[1][1] * s, R[2][1] * s, 0,
            R[0][2] * s, R[1][2] * s, R[2][2] * s, 0,
            t.x, t.y, t.z, 1};
}

inline float4x4 perspective(float fovY, float aspect, float zNear, float zFar)
{
    float f = 1.0f / tanf(0.5f * fovY);
    return {f / aspect, 0, 0, 0,
            0, f, 0, 0,
            0, 0, (zFar + zNear) / (zNear - zFar), -1,
            0, 0, 2 * zFar * zNear / (zNear - zFar), 0};
}

inline float4x4 orthographic(float l, float r, float b, float t, float n, float f)
{
    return {2 / (r - l), 0, 0, 0,
            0, 2 / (t - b), 0, 0,
            0, 0, -2 / (f - n), 0,
            -(r + l) / (r - l), -(t + b) / (t - b), -(f + n) / (f - n), 1};
}

inline float4x4 lookAt(float3 eye, float3 target, float3 up)
{
    float3 f = normalize(target - eye);
    float3 s = normalize(cross(f, up));
    float3 u = cross(s, f);
    return {s.x, u.x, -f.x, 0,
            s.y, u.y, -f.y, 0,
            s.z, u.z, -f.z, 0,
            -dot(s, eye), -dot(u, eye), dot(f, eye), 1};
}
