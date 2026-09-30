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

#include <map>
#include <memory>
#include <vector>

#include "collide.h"

// Rigid body simulator used by the CPU experiments of the paper (Sec. 4): a velocity level,
// constraint based formulation [AEF22] solved with sequential impulses, with Baumgarte
// stabilization (applied with split impulses) and warm started impulses. Collision detection
// runs Algorithm 1 for every pair.
struct Solver
{
    std::vector<std::unique_ptr<Body>> bodies;
    std::map<std::pair<int, int>, Manifold> manifolds;
    CollisionParams params;
    CollisionStats stats;
    Random rng;

    float dt = 1.0f / 60.0f;           // Time step (16 ms in the paper)
    float3 gravity = {0, -9.81f, 0};
    int substeps = 4;                  // Substeps per time step, each with its own collision detection
    int iterations = 20;               // Solver iterations per substep (velocities and position correction)
    float baumgarte = 0.2f;            // Fraction of the penetration corrected per substep
    float maxCorrection = 2.0f;        // Limit on the separating velocity of the correction

    // Mouse dragging, the anchor is relative to the center of mass in the body frame
    Body *dragBody = nullptr;
    float3 dragAnchor = {0, 0, 0};
    float3 dragTarget = {0, 0, 0};

    Body *add(Body *body);
    void clear();
    void step();

    // Returns the closest dynamic body hit by the ray, and the hit point
    Body *pick(float3 origin, float3 direction, float3 &hit) const;

private:
    void collide(float h);
    void applyDrag(float h);
};
