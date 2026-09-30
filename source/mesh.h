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

#include <string>
#include <vector>

#include "maths.h"

// Indexed triangle mesh. Used for rendering every shape, and as the input from which
// voxel signed distance fields are baked.
struct Mesh
{
    std::vector<float3> positions;
    std::vector<float3> normals;
    std::vector<uint32_t> indices;

    int triangleCount() const { return (int)indices.size() / 3; }
    float3 vertex(int tri, int corner) const { return positions[indices[tri * 3 + corner]]; }

    uint32_t addVertex(float3 p, float3 n);
    void addTriangle(uint32_t a, uint32_t b, uint32_t c);
    void append(const Mesh &other, const float3x3 &R = identity3(), float3 t = {0, 0, 0});
    AABB bounds() const;

    // Replaces the normals with area weighted face normals, averaged only across edges whose
    // dihedral angle is below creaseAngle so that sharp features stay sharp.
    void computeNormals(float creaseAngle = rad(40.0f));
};

// Loads a Wavefront OBJ file (polygons are fan triangulated, duplicated vertices welded, normals
// left unset). Returns false and sets error on failure.
bool loadObj(const std::string &path, Mesh &mesh, std::string &error);

// Primitive render meshes, all centered at the origin with y as the axis of symmetry
Mesh makeBoxMesh(float3 halfExtents, int segments = 1);
Mesh makeSphereMesh(float radius, int slices = 32, int stacks = 16);
Mesh makeCapsuleMesh(float radius, float halfLength, int slices = 32, int stacks = 8);
Mesh makeCylinderMesh(float radius, float halfHeight, int slices = 32);
Mesh makeConeMesh(float radius, float halfHeight, int slices = 32);
Mesh makeTorusMesh(float majorRadius, float minorRadius, int majorSegments = 48, int minorSegments = 24);
