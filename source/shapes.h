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

#include <memory>
#include <string>
#include <vector>

#include "maths.h"
#include "mesh.h"

// A signed distance field (SDF) phi defined in the shape's local frame, with phi < 0 inside.
// The collision framework only ever queries shapes through sdf(), so any representation can be
// plugged in (Sec. 3.6). Two families are provided: analytical primitives / composites below,
// and voxel grids baked from triangle meshes (VoxelSdf).
struct Shape
{
    AABB bounds;               // Local bounds of the zero level set
    Mesh mesh;                 // Render mesh (local frame)
    std::vector<float3> seeds; // High curvature seed points (Sec. 3.5.2), local frame
    bool convex = false;       // Two convex shapes intersect in a single convex region

    // Mass properties for unit density, inertia is about the center of mass
    float volume = 0;
    float3 centerOfMass = {0, 0, 0};
    float3x3 inertia = identity3();

    virtual ~Shape() {}

    // Signed distance at p. When grad is non-null the gradient of phi is also written.
    virtual float sdf(float3 p, float3 *grad = nullptr) const = 0;

    // Computes the mass properties and the curvature seed points. Must be called once after
    // construction, since both are derived from sdf().
    void finalize();

    // Laplacian of phi by central differences of the gradient (Eq. 7)
    float laplacian(float3 p, float delta) const;

    // Moves p onto the zero level set by following the gradient
    float3 project(float3 p) const;

protected:
    virtual void surfaceCandidates(std::vector<float3> &points, int count) const;
};

using ShapePtr = std::shared_ptr<Shape>;

// Signed distance functions of the analytical primitives (with gradients), axis of symmetry is y

float sdBox(float3 p, float3 halfExtents, float3 *grad);
float sdSphere(float3 p, float radius, float3 *grad);
float sdCapsule(float3 p, float radius, float halfLength, float3 *grad);
float sdCylinder(float3 p, float radius, float halfHeight, float3 *grad);
float sdCone(float3 p, float radius, float halfHeight, float3 *grad); // Apex at +halfHeight
float sdTorus(float3 p, float majorRadius, float minorRadius, float3 *grad);

// Analytical primitives

struct Box : Shape
{
    float3 halfExtents;
    Box(float3 halfExtents);
    float sdf(float3 p, float3 *grad) const override { return sdBox(p, halfExtents, grad); }
};

struct Sphere : Shape
{
    float radius;
    Sphere(float radius);
    float sdf(float3 p, float3 *grad) const override { return sdSphere(p, radius, grad); }
};

struct Capsule : Shape
{
    float radius, halfLength;
    Capsule(float radius, float halfLength);
    float sdf(float3 p, float3 *grad) const override { return sdCapsule(p, radius, halfLength, grad); }
};

struct Cylinder : Shape
{
    float radius, halfHeight;
    Cylinder(float radius, float halfHeight);
    float sdf(float3 p, float3 *grad) const override { return sdCylinder(p, radius, halfHeight, grad); }
};

struct Cone : Shape
{
    float radius, halfHeight;
    Cone(float radius, float halfHeight);
    float sdf(float3 p, float3 *grad) const override { return sdCone(p, radius, halfHeight, grad); }
};

struct Torus : Shape
{
    float majorRadius, minorRadius;
    Torus(float majorRadius, float minorRadius);
    float sdf(float3 p, float3 *grad) const override { return sdTorus(p, majorRadius, minorRadius, grad); }
};

// Analytical composites (unions of primitives), e.g. the spike plates of Figs. 1 and 4

// Box with a grid of cone shaped spikes of varying height on its top face
struct SpikePlate : Shape
{
    float3 halfExtents;
    int count;
    float spikeRadius, minHeight, maxHeight;
    SpikePlate(float3 halfExtents, int count, float spikeRadius, float minHeight, float maxHeight);
    float sdf(float3 p, float3 *grad) const override;
    float spikeHeight(int i, int j) const;
    float3 spikeCenter(int i, int j) const;
};

// Cylindrical hub with two rings of radial cone shaped spikes (Fig. 9, bottom right)
struct SpikyWheel : Shape
{
    float hubRadius, hubHalfWidth, spikeRadius, spikeHalfHeight;
    int spikes;
    SpikyWheel(float hubRadius, float hubHalfWidth, float spikeRadius, float spikeLength, int spikes);
    float sdf(float3 p, float3 *grad) const override;
};

// Operator distorted box (Sec. 4.5). The warps break the Eikonal property |grad phi| = 1,
// which makes the field quasi-exact.
struct WarpedBox : Shape
{
    enum Type
    {
        Bend,
        Twist,
        Displace
    };

    Type type;
    float3 halfExtents;
    float k;
    WarpedBox(Type type, float3 halfExtents, float k);
    float sdf(float3 p, float3 *grad) const override;
    float3 inverse(float3 q) const;
};

// Voxel discretization (Sec. 3.6): the signed distance and its gradient are baked from a
// triangle mesh onto a regular grid and trilinearly interpolated.
struct VoxelSdf : Shape
{
    std::string name; // Taken from the file name of the mesh
    int nx = 0, ny = 0, nz = 0;
    float3 origin = {0, 0, 0};
    float cell = 0;
    std::vector<float4> grid; // (phi, grad phi) at every grid node
    bool watertight = false;

    // The mesh is normalized to fit in a unit cube (Sec. 3.5.2), the grid has resolution^3
    // cells over that cube (100^3 in the paper).
    VoxelSdf(const Mesh &mesh, const std::string &name, int resolution = 100);
    float sdf(float3 p, float3 *grad) const override;

protected:
    void surfaceCandidates(std::vector<float3> &points, int count) const override;
};

// Loads an OBJ file and bakes it into a voxel SDF. Returns null and sets error on failure.
std::shared_ptr<VoxelSdf> loadVoxelSdf(const std::string &path, std::string &error, int resolution = 100);
