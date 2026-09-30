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

#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <tuple>

#include "mesh.h"

uint32_t Mesh::addVertex(float3 p, float3 n)
{
    positions.push_back(p);
    normals.push_back(n);
    return (uint32_t)positions.size() - 1;
}

void Mesh::addTriangle(uint32_t a, uint32_t b, uint32_t c)
{
    indices.push_back(a);
    indices.push_back(b);
    indices.push_back(c);
}

void Mesh::append(const Mesh &other, const float3x3 &R, float3 t)
{
    uint32_t base = (uint32_t)positions.size();
    for (size_t i = 0; i < other.positions.size(); i++)
        addVertex(R * other.positions[i] + t, R * other.normals[i]);
    for (uint32_t index : other.indices)
        indices.push_back(base + index);
}

AABB Mesh::bounds() const
{
    AABB box = emptyAABB();
    for (float3 p : positions)
        grow(box, p);
    return box;
}

void Mesh::computeNormals(float creaseAngle)
{
    int count = triangleCount();
    std::vector<float3> faceNormals(count);
    std::vector<std::vector<int>> incident(positions.size());
    for (int t = 0; t < count; t++)
    {
        faceNormals[t] = cross(vertex(t, 1) - vertex(t, 0), vertex(t, 2) - vertex(t, 0));
        for (int c = 0; c < 3; c++)
            incident[indices[t * 3 + c]].push_back(t);
    }

    // Each triangle corner averages the (area weighted) normals of the incident faces that are
    // within the crease angle of its own face. Corners with matching normals are then re-welded.
    Mesh result;
    std::vector<std::vector<uint32_t>> welded(positions.size());
    float cosCrease = cosf(creaseAngle);
    for (int t = 0; t < count; t++)
    {
        float3 faceNormal = normalize(faceNormals[t]);
        uint32_t corners[3];
        for (int c = 0; c < 3; c++)
        {
            uint32_t v = indices[t * 3 + c];
            float3 n = {0, 0, 0};
            for (int other : incident[v])
                if (dot(normalize(faceNormals[other]), faceNormal) >= cosCrease)
                    n += faceNormals[other];
            n = normalize(n);

            corners[c] = UINT32_MAX;
            for (uint32_t candidate : welded[v])
                if (dot(result.normals[candidate], n) > 0.9999f)
                    corners[c] = candidate;
            if (corners[c] == UINT32_MAX)
            {
                corners[c] = result.addVertex(positions[v], n);
                welded[v].push_back(corners[c]);
            }
        }
        result.addTriangle(corners[0], corners[1], corners[2]);
    }
    *this = result;
}

bool loadObj(const std::string &path, Mesh &mesh, std::string &error)
{
    std::ifstream file(path);
    if (!file)
    {
        error = "Could not open " + path;
        return false;
    }

    std::vector<float3> positions;
    std::vector<uint32_t> indices;
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream stream(line);
        std::string tag;
        stream >> tag;
        if (tag == "v")
        {
            float3 p = {0, 0, 0};
            stream >> p.x >> p.y >> p.z;
            positions.push_back(p);
        }
        else if (tag == "f")
        {
            // Face corners look like "v", "v/vt", "v//vn" or "v/vt/vn", only the position index is used
            std::vector<uint32_t> face;
            std::string corner;
            while (stream >> corner)
            {
                int index = atoi(corner.c_str());
                index = index < 0 ? (int)positions.size() + index : index - 1;
                if (index < 0 || index >= (int)positions.size())
                {
                    error = "Invalid face index in " + path;
                    return false;
                }
                face.push_back((uint32_t)index);
            }
            for (size_t i = 2; i < face.size(); i++)
            {
                indices.push_back(face[0]);
                indices.push_back(face[i - 1]);
                indices.push_back(face[i]);
            }
        }
    }

    if (indices.empty())
    {
        error = "No triangles found in " + path;
        return false;
    }

    // Weld duplicated positions so that normals and the watertightness check see a connected surface
    AABB box = emptyAABB();
    for (float3 p : positions)
        grow(box, p);
    float cell = max(maxComponent(size(box)) * 1e-5f, 1e-9f);
    std::map<std::tuple<int, int, int>, uint32_t> unique;
    std::vector<uint32_t> remap(positions.size());
    mesh = Mesh();
    for (size_t i = 0; i < positions.size(); i++)
    {
        float3 q = (positions[i] - box.min) / cell;
        auto key = std::make_tuple((int)lroundf(q.x), (int)lroundf(q.y), (int)lroundf(q.z));
        auto it = unique.find(key);
        if (it == unique.end())
            it = unique.emplace(key, mesh.addVertex(positions[i], {0, 1, 0})).first;
        remap[i] = it->second;
    }
    for (size_t i = 0; i + 2 < indices.size(); i += 3)
    {
        uint32_t a = remap[indices[i]], b = remap[indices[i + 1]], c = remap[indices[i + 2]];
        if (a != b && b != c && c != a)
            mesh.addTriangle(a, b, c);
    }
    if (mesh.indices.empty())
    {
        error = "Only degenerate triangles found in " + path;
        return false;
    }
    return true;
}

// Adds a (u x v) grid spanning the box face with outward normal n = u x v
static void addBoxFace(Mesh &mesh, float3 n, float3 u, float3 v, float3 h, float3 cells)
{
    int nu = (int)fmaxf(1.0f, roundf(dot(abs(u), cells)));
    int nv = (int)fmaxf(1.0f, roundf(dot(abs(v), cells)));
    float3 origin = n * dot(abs(n), h) - u * dot(abs(u), h) - v * dot(abs(v), h);
    uint32_t base = (uint32_t)mesh.positions.size();
    for (int j = 0; j <= nv; j++)
        for (int i = 0; i <= nu; i++)
            mesh.addVertex(origin + u * (2 * dot(abs(u), h) * i / nu) + v * (2 * dot(abs(v), h) * j / nv), n);
    for (int j = 0; j < nv; j++)
    {
        for (int i = 0; i < nu; i++)
        {
            uint32_t a = base + j * (nu + 1) + i;
            mesh.addTriangle(a, a + 1, a + nu + 2);
            mesh.addTriangle(a, a + nu + 2, a + nu + 1);
        }
    }
}

Mesh makeBoxMesh(float3 h, int segments)
{
    float3 cells = h * ((float)segments / maxComponent(h));
    float3 X = {1, 0, 0}, Y = {0, 1, 0}, Z = {0, 0, 1};
    Mesh mesh;
    addBoxFace(mesh, X, Y, Z, h, cells);
    addBoxFace(mesh, -X, Z, Y, h, cells);
    addBoxFace(mesh, Y, Z, X, h, cells);
    addBoxFace(mesh, -Y, X, Z, h, cells);
    addBoxFace(mesh, Z, X, Y, h, cells);
    addBoxFace(mesh, -Z, Y, X, h, cells);
    return mesh;
}

// Latitude/longitude grid. Rings with index < split are shifted down by offset, the rest up,
// which turns a sphere into a capsule when a duplicated equator ring is inserted.
static Mesh makeLatLongMesh(float radius, int slices, int stacks, float offset)
{
    Mesh mesh;
    int rings = offset > 0 ? stacks + 2 : stacks + 1;
    for (int i = 0; i < rings; i++)
    {
        int ring = (offset > 0 && i > stacks / 2) ? i - 1 : i;
        float phi = -0.5f * PI + PI * ring / stacks;
        float shift = (offset > 0 && i <= stacks / 2) ? -offset : offset;
        for (int j = 0; j <= slices; j++)
        {
            float theta = 2 * PI * j / slices;
            float3 n = {cosf(phi) * cosf(theta), sinf(phi), cosf(phi) * sinf(theta)};
            mesh.addVertex(n * radius + float3{0, shift, 0}, n);
        }
    }
    for (int i = 0; i + 1 < rings; i++)
    {
        for (int j = 0; j < slices; j++)
        {
            uint32_t a = i * (slices + 1) + j, b = a + slices + 1;
            mesh.addTriangle(a, b, b + 1);
            mesh.addTriangle(a, b + 1, a + 1);
        }
    }
    return mesh;
}

Mesh makeSphereMesh(float radius, int slices, int stacks)
{
    return makeLatLongMesh(radius, slices, stacks, 0.0f);
}

Mesh makeCapsuleMesh(float radius, float halfLength, int slices, int stacks)
{
    return makeLatLongMesh(radius, slices, 2 * (stacks / 2), halfLength);
}

Mesh makeCylinderMesh(float radius, float halfHeight, int slices)
{
    Mesh mesh;
    uint32_t top = mesh.addVertex({0, halfHeight, 0}, {0, 1, 0});
    uint32_t bottom = mesh.addVertex({0, -halfHeight, 0}, {0, -1, 0});
    for (int j = 0; j < slices; j++)
    {
        float t0 = 2 * PI * j / slices, t1 = 2 * PI * (j + 1) / slices;
        float3 n0 = {cosf(t0), 0, sinf(t0)}, n1 = {cosf(t1), 0, sinf(t1)};
        float3 up = {0, halfHeight, 0};
        uint32_t a = mesh.addVertex(n0 * radius - up, n0), b = mesh.addVertex(n0 * radius + up, n0);
        uint32_t c = mesh.addVertex(n1 * radius + up, n1), d = mesh.addVertex(n1 * radius - up, n1);
        mesh.addTriangle(a, b, c);
        mesh.addTriangle(a, c, d);
        uint32_t e = mesh.addVertex(n0 * radius + up, {0, 1, 0}), f = mesh.addVertex(n1 * radius + up, {0, 1, 0});
        uint32_t g = mesh.addVertex(n0 * radius - up, {0, -1, 0}), h = mesh.addVertex(n1 * radius - up, {0, -1, 0});
        mesh.addTriangle(top, f, e);
        mesh.addTriangle(bottom, g, h);
    }
    return mesh;
}

Mesh makeConeMesh(float radius, float halfHeight, int slices)
{
    // Apex at +halfHeight, base disk at -halfHeight
    Mesh mesh;
    uint32_t bottom = mesh.addVertex({0, -halfHeight, 0}, {0, -1, 0});
    auto sideNormal = [&](float t) { return normalize(float3{2 * halfHeight * cosf(t), radius, 2 * halfHeight * sinf(t)}); };
    for (int j = 0; j < slices; j++)
    {
        float t0 = 2 * PI * j / slices, t1 = 2 * PI * (j + 1) / slices;
        float3 r0 = {radius * cosf(t0), -halfHeight, radius * sinf(t0)};
        float3 r1 = {radius * cosf(t1), -halfHeight, radius * sinf(t1)};
        uint32_t a = mesh.addVertex(r0, sideNormal(t0));
        uint32_t apex = mesh.addVertex({0, halfHeight, 0}, sideNormal(0.5f * (t0 + t1)));
        uint32_t b = mesh.addVertex(r1, sideNormal(t1));
        mesh.addTriangle(a, apex, b);
        mesh.addTriangle(bottom, mesh.addVertex(r0, {0, -1, 0}), mesh.addVertex(r1, {0, -1, 0}));
    }
    return mesh;
}

Mesh makeTorusMesh(float majorRadius, float minorRadius, int majorSegments, int minorSegments)
{
    Mesh mesh;
    for (int i = 0; i <= majorSegments; i++)
    {
        float u = 2 * PI * i / majorSegments;
        for (int j = 0; j <= minorSegments; j++)
        {
            float v = 2 * PI * j / minorSegments;
            float3 n = {cosf(v) * cosf(u), sinf(v), cosf(v) * sinf(u)};
            float3 c = {majorRadius * cosf(u), 0, majorRadius * sinf(u)};
            mesh.addVertex(c + n * minorRadius, n);
        }
    }
    for (int i = 0; i < majorSegments; i++)
    {
        for (int j = 0; j < minorSegments; j++)
        {
            uint32_t a = i * (minorSegments + 1) + j, b = a + minorSegments + 1;
            mesh.addTriangle(a, a + 1, b + 1);
            mesh.addTriangle(a, b + 1, b);
        }
    }
    return mesh;
}
