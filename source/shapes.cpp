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

#include <algorithm>

#include "shapes.h"

static float3 signs(float3 p) { return {sign(p.x), sign(p.y), sign(p.z)}; }

// Unit radial direction in the xz plane
static float3 radialDirection(float3 p, float radial)
{
    return radial > 1e-9f ? float3{p.x / radial, 0, p.z / radial} : float3{1, 0, 0};
}

float sdBox(float3 p, float3 halfExtents, float3 *grad)
{
    float3 q = abs(p) - halfExtents;
    float3 outside = max(q, {0, 0, 0});
    float outsideLength = length(outside);
    if (grad)
    {
        if (outsideLength > 0)
            *grad = outside / outsideLength * signs(p);
        else
        {
            int axis = q.x > q.y ? (q.x > q.z ? 0 : 2) : (q.y > q.z ? 1 : 2);
            *grad = {0, 0, 0};
            (*grad)[axis] = sign(p[axis]);
        }
    }
    return outsideLength + min(maxComponent(q), 0.0f);
}

float sdSphere(float3 p, float radius, float3 *grad)
{
    float len = length(p);
    if (grad)
        *grad = len > 1e-9f ? p / len : float3{0, 1, 0};
    return len - radius;
}

float sdCapsule(float3 p, float radius, float halfLength, float3 *grad)
{
    return sdSphere({p.x, p.y - clamp(p.y, -halfLength, halfLength), p.z}, radius, grad);
}

float sdCylinder(float3 p, float radius, float halfHeight, float3 *grad)
{
    float radial = sqrtf(p.x * p.x + p.z * p.z);
    float dx = radial - radius, dy = fabsf(p.y) - halfHeight;
    float ox = max(dx, 0.0f), oy = max(dy, 0.0f);
    float outside = sqrtf(ox * ox + oy * oy);
    if (grad)
    {
        float3 e = radialDirection(p, radial);
        if (outside > 0)
            *grad = e * (ox / outside) + float3{0, sign(p.y) * oy / outside, 0};
        else
            *grad = dx > dy ? e : float3{0, sign(p.y), 0};
    }
    return outside + min(max(dx, dy), 0.0f);
}

float sdCone(float3 p, float radius, float halfHeight, float3 *grad)
{
    // Capped cone from Quilez [Qui] with the top radius set to zero. The problem is solved in the
    // 2D half plane q = (radial distance, height) against the base segment and the slanted side.
    float qx = sqrtf(p.x * p.x + p.z * p.z), qy = p.y;
    float cax = qx - min(qx, qy < 0 ? radius : 0.0f), cay = fabsf(qy) - halfHeight;
    float kx = -radius, ky = 2 * halfHeight;
    float t = clamp((qx * radius + (halfHeight - qy) * ky) / (kx * kx + ky * ky), 0.0f, 1.0f);
    float cbx = qx + kx * t, cby = qy - halfHeight + ky * t;
    float s = (cbx < 0 && cay < 0) ? -1.0f : 1.0f;
    float da = cax * cax + cay * cay, db = cbx * cbx + cby * cby;
    if (grad)
    {
        // The gradient points away from the closest point on the profile
        float gx = da < db ? cax : cbx, gy = da < db ? cay * sign(qy) : cby;
        float len = sqrtf(gx * gx + gy * gy);
        if (len > 1e-9f)
            gx *= s / len, gy *= s / len;
        else if (da < db)
            gx = 0, gy = sign(qy);
        else
            gx = ky, gy = radius, len = sqrtf(gx * gx + gy * gy), gx /= len, gy /= len;
        *grad = radialDirection(p, qx) * gx + float3{0, gy, 0};
    }
    return s * sqrtf(min(da, db));
}

float sdTorus(float3 p, float majorRadius, float minorRadius, float3 *grad)
{
    float radial = sqrtf(p.x * p.x + p.z * p.z);
    float qx = radial - majorRadius, qy = p.y;
    float len = sqrtf(qx * qx + qy * qy);
    if (grad)
    {
        float3 e = radialDirection(p, radial);
        *grad = len > 1e-9f ? e * (qx / len) + float3{0, qy / len, 0} : e;
    }
    return len - minorRadius;
}

// Primitive constructors

Box::Box(float3 halfExtents) : halfExtents(halfExtents)
{
    convex = true;
    bounds = {-halfExtents, halfExtents};
    mesh = makeBoxMesh(halfExtents);
}

Sphere::Sphere(float radius) : radius(radius)
{
    convex = true;
    bounds = {float3{-radius, -radius, -radius}, float3{radius, radius, radius}};
    mesh = makeSphereMesh(radius);
}

Capsule::Capsule(float radius, float halfLength) : radius(radius), halfLength(halfLength)
{
    convex = true;
    bounds = {float3{-radius, -radius - halfLength, -radius}, float3{radius, radius + halfLength, radius}};
    mesh = makeCapsuleMesh(radius, halfLength);
}

Cylinder::Cylinder(float radius, float halfHeight) : radius(radius), halfHeight(halfHeight)
{
    convex = true;
    bounds = {float3{-radius, -halfHeight, -radius}, float3{radius, halfHeight, radius}};
    mesh = makeCylinderMesh(radius, halfHeight);
}

Cone::Cone(float radius, float halfHeight) : radius(radius), halfHeight(halfHeight)
{
    convex = true;
    bounds = {float3{-radius, -halfHeight, -radius}, float3{radius, halfHeight, radius}};
    mesh = makeConeMesh(radius, halfHeight);
}

Torus::Torus(float majorRadius, float minorRadius) : majorRadius(majorRadius), minorRadius(minorRadius)
{
    float r = majorRadius + minorRadius;
    bounds = {float3{-r, -minorRadius, -r}, float3{r, minorRadius, r}};
    mesh = makeTorusMesh(majorRadius, minorRadius);
}

// Spike plate

static float hash01(uint32_t x)
{
    x ^= x >> 16, x *= 0x7feb352dU, x ^= x >> 15, x *= 0x846ca68bU, x ^= x >> 16;
    return (x & 0xffffff) / 16777216.0f;
}

SpikePlate::SpikePlate(float3 halfExtents, int count, float spikeRadius, float minHeight, float maxHeight)
    : halfExtents(halfExtents), count(count), spikeRadius(spikeRadius), minHeight(minHeight), maxHeight(maxHeight)
{
    bounds = {-halfExtents, halfExtents + float3{0, maxHeight, 0}};
    mesh = makeBoxMesh(halfExtents);
    for (int j = 0; j < count; j++)
        for (int i = 0; i < count; i++)
            mesh.append(makeConeMesh(spikeRadius, 0.5f * spikeHeight(i, j), 16), identity3(), spikeCenter(i, j));
}

float SpikePlate::spikeHeight(int i, int j) const
{
    return minHeight + (maxHeight - minHeight) * hash01(17u * (uint32_t)(i + j * count) + 11u);
}

float3 SpikePlate::spikeCenter(int i, int j) const
{
    float3 pitch = halfExtents * (2.0f / count);
    return {-halfExtents.x + (i + 0.5f) * pitch.x, halfExtents.y + 0.5f * spikeHeight(i, j), -halfExtents.z + (j + 0.5f) * pitch.z};
}

float SpikePlate::sdf(float3 p, float3 *grad) const
{
    float d = sdBox(p, halfExtents, grad);

    // Spikes are thinner than the grid pitch, so only the 3x3 neighborhood of the cell below p
    // can contain the closest one.
    float3 pitch = halfExtents * (2.0f / count);
    int ci = std::clamp((int)floorf((p.x + halfExtents.x) / pitch.x), 0, count - 1);
    int cj = std::clamp((int)floorf((p.z + halfExtents.z) / pitch.z), 0, count - 1);
    for (int j = std::max(cj - 1, 0); j <= std::min(cj + 1, count - 1); j++)
    {
        for (int i = std::max(ci - 1, 0); i <= std::min(ci + 1, count - 1); i++)
        {
            // The cones continue into the plate by a spike radius. If their bases were on its surface,
            // the base would be the closest surface inside a spike, and the gradient there would point
            // down into the plate.
            float h = spikeHeight(i, j), e = spikeRadius;
            float3 g, c = spikeCenter(i, j) - float3{0, 0.5f * e, 0};
            float dc = sdCone(p - c, spikeRadius * (h + e) / h, 0.5f * (h + e), grad ? &g : nullptr);
            if (dc < d)
            {
                d = dc;
                if (grad)
                    *grad = g;
            }
        }
    }
    return d;
}

// Spiky wheel

SpikyWheel::SpikyWheel(float hubRadius, float hubHalfWidth, float spikeRadius, float spikeLength, int spikes)
    : hubRadius(hubRadius), hubHalfWidth(hubHalfWidth), spikeRadius(spikeRadius), spikeHalfHeight(0.5f * spikeLength), spikes(spikes)
{
    float r = hubRadius + spikeLength;
    bounds = {float3{-r, -hubHalfWidth, -r}, float3{r, hubHalfWidth, r}};
    mesh = makeCylinderMesh(hubRadius, hubHalfWidth);
    Mesh cone = makeConeMesh(spikeRadius, spikeHalfHeight, 16);
    float ringY = hubHalfWidth - spikeRadius;
    for (int k = 0; k < spikes; k++)
    {
        // Frame with the cone axis (+y) mapped to the radial direction
        float a = 2 * PI * k / spikes;
        float3 side = {-sinf(a), 0, cosf(a)}, dir = {cosf(a), 0, sinf(a)};
        float3x3 R = transpose(float3x3{side, dir, float3{0, 1, 0}});
        for (float y : {-ringY, ringY})
            mesh.append(cone, R, dir * (hubRadius + spikeHalfHeight) + float3{0, y, 0});
    }
}

float SpikyWheel::sdf(float3 p, float3 *grad) const
{
    float d = sdCylinder(p, hubRadius, hubHalfWidth, grad);

    // Only the spikes nearest in angle to p are evaluated
    float step = 2 * PI / spikes, ringY = hubHalfWidth - spikeRadius;
    int nearest = (int)floorf(atan2f(p.z, p.x) / step + 0.5f);
    for (int k = nearest - 1; k <= nearest + 1; k++)
    {
        float a = k * step;
        float3 side = {-sinf(a), 0, cosf(a)}, dir = {cosf(a), 0, sinf(a)};
        for (float y : {-ringY, ringY})
        {
            float3 r = p - dir * (hubRadius + spikeHalfHeight) - float3{0, y, 0};
            float3 g;
            float dc = sdCone({dot(r, side), dot(r, dir), r.y}, spikeRadius, spikeHalfHeight, grad ? &g : nullptr);
            if (dc < d)
            {
                d = dc;
                if (grad)
                    *grad = side * g.x + dir * g.y + float3{0, g.z, 0};
            }
        }
    }
    return d;
}

// Operator distorted boxes (Sec. 4.5). The distorted fields are phi_box(q(x)), so by the chain
// rule their gradients are J^T grad phi_box(q), where J = dq/dx.

static const float DISPLACE_FREQUENCY = 7.0f;

WarpedBox::WarpedBox(Type type, float3 halfExtents, float k) : type(type), halfExtents(halfExtents), k(k)
{
    float3 h = halfExtents;
    if (type == Bend)
    {
        float r = sqrtf(h.x * h.x + h.y * h.y);
        bounds = {float3{-r, -r, -h.z}, float3{r, r, h.z}};
    }
    else if (type == Twist)
    {
        float r = sqrtf(h.x * h.x + h.y * h.y);
        bounds = {float3{-r, -h.z, -r}, float3{r, h.z, r}};
    }
    else
    {
        bounds = expand({-h, h}, fabsf(k));
    }

    // Render mesh: a finely subdivided box mapped through the inverse warp. The twist permutes
    // two axes, which mirrors the mesh, so its triangles are flipped to keep them front facing.
    mesh = makeBoxMesh(halfExtents, 32);
    for (float3 &p : mesh.positions)
        p = inverse(p);
    if (type == Twist)
        for (size_t i = 0; i < mesh.indices.size(); i += 3)
            std::swap(mesh.indices[i + 1], mesh.indices[i + 2]);
    mesh.computeNormals();
}

float WarpedBox::sdf(float3 p, float3 *grad) const
{
    if (type == Displace)
    {
        // DISPLACE(x, k) = phi(x) + k sin(w x) sin(w y) sin(w z), with k as the amplitude (see README)
        float w = DISPLACE_FREQUENCY;
        float3 s = {sinf(w * p.x), sinf(w * p.y), sinf(w * p.z)};
        float3 c = {cosf(w * p.x), cosf(w * p.y), cosf(w * p.z)};
        float d = sdBox(p, halfExtents, grad) + k * s.x * s.y * s.z;
        if (grad)
            *grad += float3{c.x * s.y * s.z, s.x * c.y * s.z, s.x * s.y * c.z} * (k * w);
        return d;
    }

    float3 q;
    float3x3 J;
    if (type == Bend)
    {
        // BEND(x, k) = phi(q), q = Rz(k x) x
        float c = cosf(k * p.x), s = sinf(k * p.x);
        q = {c * p.x - s * p.y, s * p.x + c * p.y, p.z};
        J = {float3{c - k * q.y, -s, 0}, float3{s + k * q.x, c, 0}, float3{0, 0, 1}};
    }
    else
    {
        // TWIST(x, k) = phi(q), q = (c x - s z, s x + c z, y) with the angle k y
        float c = cosf(k * p.y), s = sinf(k * p.y);
        q = {c * p.x - s * p.z, s * p.x + c * p.z, p.y};
        J = {float3{c, -k * q.y, -s}, float3{s, k * q.x, c}, float3{0, 1, 0}};
    }

    float3 g;
    float d = sdBox(q, halfExtents, grad ? &g : nullptr);
    if (grad)
        *grad = transpose(J) * g;
    return d;
}

// Maps a point on the undistorted box to the corresponding point on the distorted surface
float3 WarpedBox::inverse(float3 q) const
{
    if (type == Twist)
    {
        float c = cosf(k * q.z), s = sinf(k * q.z);
        return {c * q.x + s * q.y, q.z, -s * q.x + c * q.y};
    }
    if (type == Displace)
        return project(q);

    // Bend has no closed form inverse, solve q = Rz(k x) x with Newton's method
    float3 p = q;
    for (int it = 0; it < 20; it++)
    {
        float c = cosf(k * p.x), s = sinf(k * p.x);
        float3 b = {c * p.x - s * p.y, s * p.x + c * p.y, p.z};
        float rx = b.x - q.x, ry = b.y - q.y;
        float a00 = c - k * b.y, a01 = -s, a10 = s + k * b.x, a11 = c;
        float det = a00 * a11 - a01 * a10;
        if (fabsf(det) < 1e-9f)
            break;
        p.x -= (a11 * rx - a01 * ry) / det;
        p.y -= (-a10 * rx + a00 * ry) / det;
    }
    return p;
}

// Shape

float Shape::laplacian(float3 p, float delta) const
{
    // Eq. 7: Laplacian of phi (the mean curvature near the surface) by central differences of the gradient
    float sum = 0;
    for (int i = 0; i < 3; i++)
    {
        float3 e = {0, 0, 0}, g0, g1;
        e[i] = delta;
        sdf(p + e, &g1);
        sdf(p - e, &g0);
        sum += (g1[i] - g0[i]) / (2 * delta);
    }
    return sum;
}

float3 Shape::project(float3 p) const
{
    float tolerance = 1e-5f * maxComponent(size(bounds));
    for (int it = 0; it < 32; it++)
    {
        float3 g;
        float d = sdf(p, &g);
        if (fabsf(d) < tolerance)
            break;
        p -= g * (d / max(lengthSq(g), 1e-8f));
    }
    return p;
}

void Shape::surfaceCandidates(std::vector<float3> &points, int count) const
{
    // Random points in the bounding box, projected onto the surface using the gradient (Sec. 3.5.2)
    Random rng;
    AABB box = expand(bounds, 0.1f * maxComponent(size(bounds)));
    float tolerance = 1e-3f * maxComponent(size(bounds));
    for (int i = 0; i < count; i++)
    {
        float3 p = project(rng.inBox(box));
        if (fabsf(sdf(p)) < tolerance)
            points.push_back(p);
    }
}

void Shape::finalize()
{
    // Mass properties for unit density by integrating over a voxelization of the shape
    float3 extent = size(bounds);
    float h = maxComponent(extent) / 48;
    int n[3];
    for (int i = 0; i < 3; i++)
        n[i] = std::max(1, (int)ceilf(extent[i] / h));
    float3 cellSize = extent / float3{(float)n[0], (float)n[1], (float)n[2]};
    float dv = cellSize.x * cellSize.y * cellSize.z;

    double mass = 0, first[3] = {0, 0, 0}, second[3][3] = {};
    for (int z = 0; z < n[2]; z++)
    {
        for (int y = 0; y < n[1]; y++)
        {
            for (int x = 0; x < n[0]; x++)
            {
                float3 c = bounds.min + cellSize * float3{x + 0.5f, y + 0.5f, z + 0.5f};
                if (sdf(c) >= 0)
                    continue;
                mass += dv;
                for (int i = 0; i < 3; i++)
                {
                    first[i] += c[i] * dv;
                    for (int j = 0; j < 3; j++)
                        second[i][j] += c[i] * c[j] * dv + (i == j ? cellSize[i] * cellSize[i] / 12 * dv : 0);
                }
            }
        }
    }

    volume = max((float)mass, 1e-6f);
    for (int i = 0; i < 3; i++)
        centerOfMass[i] = mass > 0 ? (float)(first[i] / mass) : center(bounds)[i];

    // Second moment about the center of mass S, then inertia I = tr(S) Id - S
    float3x3 S;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = (float)(second[i][j] - mass * centerOfMass[i] * centerOfMass[j]);
    float trace = S[0][0] + S[1][1] + S[2][2];
    inertia = identity3() * trace - S;

    // High curvature seed points (Sec. 3.5.2). Our models are normalized to fit in a unit cube with
    // kappa = 4 and delta = 0.001 (the Laplacian scales with 1 / size, the step with size).
    float L = maxComponent(extent);
    float kappa = 4.0f / L, delta = 0.001f * L;
    float spacing = 0.08f * L;

    std::vector<float3> candidates;
    surfaceCandidates(candidates, 8000);
    seeds.clear();
    for (float3 p : candidates)
    {
        if (fabsf(laplacian(p, delta)) <= kappa)
            continue;

        // Poisson disk rejection keeps the seeds spread out over the high curvature regions
        bool farEnough = true;
        for (float3 s : seeds)
            if (lengthSq(s - p) < spacing * spacing)
                farEnough = false;
        if (farEnough)
            seeds.push_back(p);
    }
}
