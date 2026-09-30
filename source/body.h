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

#include "shapes.h"

// Rigid body whose geometry is a (uniformly scaled) signed distance field
struct Body
{
    ShapePtr shape;
    float scale;
    float3 position;                            // World position of the center of mass
    quat orientation;
    float3 velocity = {0, 0, 0};                // Linear and angular velocity (world space)
    float3 angularVelocity = {0, 0, 0};
    float3 pseudoVelocity = {0, 0, 0};          // Position correction of a substep (split impulses)
    float3 pseudoAngularVelocity = {0, 0, 0};
    float mass = 0, invMass = 0;                // Bodies have unit density
    float3x3 inertia = identity3();             // Body frame, about the center of mass
    float3x3 invInertiaWorld = identity3() * 0;
    float friction = 0.5f;
    float3 color = {0.8f, 0.8f, 0.8f};
    bool isStatic = false;
    bool visible = true;                        // The ground body is drawn as an infinite floor instead
    AABB bounds;                                // World bounds, updated every step

    // origin is the world position of the shape's local origin
    Body(ShapePtr shape, float3 origin, quat orientation = {0, 0, 0, 1}, float scale = 1.0f, bool isStatic = false)
        : shape(shape), scale(scale), orientation(normalize(orientation)), isStatic(isStatic)
    {
        position = origin + rotate(this->orientation, shape->centerOfMass * scale);
        if (!isStatic)
        {
            mass = shape->volume * scale * scale * scale;
            inertia = shape->inertia * powf(scale, 5);
        }
        updateInertia();
        updateBounds();
    }

    // Transforms between the shape's local frame and world space
    float3 toWorld(float3 p) const { return position + rotate(orientation, (p - shape->centerOfMass) * scale); }
    float3 toLocal(float3 x) const { return rotate(conjugate(orientation), x - position) / scale + shape->centerOfMass; }

    // World space signed distance (and gradient) of the body
    float sdf(float3 x, float3 *grad = nullptr) const
    {
        float3 g;
        float d = shape->sdf(toLocal(x), grad ? &g : nullptr) * scale;
        if (grad)
            *grad = rotate(orientation, g);
        return d;
    }

    // World space velocity of the material point at x (VEL in Eq. 6)
    float3 pointVelocity(float3 x) const { return velocity + cross(angularVelocity, x - position); }

    // Inverse mass and world space inverse inertia, zero for static bodies
    void updateInertia()
    {
        if (isStatic)
            return;
        float3x3 R = rotation(orientation);
        invMass = 1.0f / mass;
        invInertiaWorld = R * inverse(inertia) * transpose(R);
    }

    void updateBounds()
    {
        bounds = emptyAABB();
        const AABB &b = shape->bounds;
        for (int i = 0; i < 8; i++)
            grow(bounds, toWorld({i & 1 ? b.max.x : b.min.x, i & 2 ? b.max.y : b.min.y, i & 4 ? b.max.z : b.min.z}));
    }

    // Bounds of a world space box in the shape's local frame
    AABB localBounds(const AABB &world) const
    {
        AABB local = emptyAABB();
        for (int i = 0; i < 8; i++)
            grow(local, toLocal({i & 1 ? world.max.x : world.min.x, i & 2 ? world.max.y : world.min.y, i & 4 ? world.max.z : world.min.z}));
        return local;
    }
};
