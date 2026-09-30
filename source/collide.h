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

#include <vector>

#include "body.h"

// SDF-SDF discrete collision detection by caching and importance sampling. The paper's
// Algorithm 1 is implemented by Manifold::update, section and equation numbers refer to the paper.

enum Optimizer
{
    OPTIMIZER_NELDER_MEAD,      // Derivative free simplex method [NM65], our default (Sec. 3.3)
    OPTIMIZER_ELLIPSOID,        // Deep cut ellipsoid method [LM24]
    OPTIMIZER_GRADIENT_DESCENT, // Stochastic gradient descent [RM51]
    OPTIMIZER_COUNT
};

extern const char *optimizerNames[OPTIMIZER_COUNT];

// Parameters of the collision framework, defaults are the values used in Sec. 4
struct CollisionParams
{
    Optimizer optimizer = OPTIMIZER_NELDER_MEAD;
    int cacheSize = 4;                  // n_cache, points kept in the temporal cache (Sec. 3.5.1)
    int randomSamples = 1;              // n_rand, new stochastic samples per step (Sec. 3.5.2)
    bool warmStart = true;              // Use the temporal cache (Sec. 3.5.1), which also warm starts the solver impulses
    bool curvatureSampling = true;      // Draw stochastic samples from high curvature seeds (Sec. 3.5.2)
    bool skipConvexSamples = true;      // Skip random samples of convex pairs tracked by the cache (not in the paper)
    float epsilon = 0.1f;               // Softmax smoothing (Eq. 2), zero gives the hardmax of Eq. 1
    float alpha = 0.01f;                // Repulsion weight (Eq. 3)
    float replacementThreshold = 0.01f; // Improvement of g a new sample needs to replace a cached point (not in the paper)
    float tolerance = 0.01f;            // Termination tolerance tau (Sec. 3.3), also the duplicate distance (Alg. 1)
    int maxIterations = 50;             // Optimizer iteration limit (Table 1)
    float stepSize = 0.1f;              // Initial step size of gradient descent
    float margin = 0.02f;               // Contact skin, bodies closer than this get contacts (not in the paper)
    float allowedPenetration = 0.01f;   // Penetration of the skin left uncorrected, half the margin (not in the paper)
};

// Per step statistics, matching the quantities reported in Tables 1-5
struct CollisionStats
{
    int pairs = 0;           // Body pairs with overlapping bounds
    int contacts = 0;        // Active contact points
    int iterations = 0;      // Optimizer iterations
    int queries = 0;         // Objective evaluations (each queries both SDFs)
    double milliseconds = 0; // Narrow phase time t_coll
};

// One point of the contact manifold M, i.e. {x*, d, n} in Algorithm 1
struct ContactPoint
{
    float3 x = {0, 0, 0};             // Minimizer x* of the objective (world space)
    float3 normal = {0, 1, 0};        // Contact normal n, points from body A to body B (Eq. 5)
    float depth = 0;                  // Penetration depth d (Eq. 5), the solver uses the separation instead
    float separation = 0;             // Gap (> 0) or overlap (< 0) of the surfaces at x*
    float softmax = 0;                // Value of Eq. 2 at x*
    float rank = 0;                   // g(x*) (Eq. 3) with respect to the final manifold, used for sorting
    bool active = false;              // Classified as a contact (Sec. 3.4)
    bool cached = false;              // Warm started from the cache, as opposed to a new stochastic sample
    float3x3 ellipsoid = identity3(); // Final ellipsoid of the ellipsoid method, used to warm start it

    // Solver impulses, cached with the point to warm start the contact solver
    float normalImpulse = 0;
    float3 frictionImpulse = {0, 0, 0};
};

// The collision objective g(x) of Eq. 3: a smooth upper bound of max(phiA, phiB) (Eq. 2), which is
// smallest deep inside the intersection of the bodies, minus a repulsion from the points already
// in the manifold.
struct Objective
{
    const Body *bodyA, *bodyB;
    const std::vector<ContactPoint> *manifold; // Points repelling new ones (M in Eq. 3)
    float epsilon, alpha;
    int *queries;

    float operator()(float3 x, float3 *grad = nullptr) const;
};

// Softmax of two distance values (Eq. 2), or the hardmax of Eq. 1 when epsilon is zero
float softmax(float phiA, float phiB, float epsilon);

// Optimizers minimizing g over the search domain V from the initial point x (updated in place).
// Each returns the number of iterations and stops once iterates move less than the tolerance (Sec. 3.3).
int nelderMead(const Objective &g, float3 &x, const AABB &domain, float3 initialStep, float tolerance, int maxIterations);
int ellipsoidMethod(const Objective &g, float3 &x, const AABB &domain, float3x3 &E, float tolerance, int maxIterations);
int gradientDescent(const Objective &g, float3 &x, const AABB &domain, float stepSize, float tolerance, int maxIterations, Random &rng);

// Contact manifold between two bodies, which also holds their temporal cache across steps
struct Manifold
{
    Body *bodyA = nullptr, *bodyB = nullptr;
    AABB volume; // Search domain V, the overlap of the bounding boxes (Sec. 3.1)

    // The manifold M found in the last step, sorted by g. Its first n_cache points are the
    // cache CACHE(A, B) that warm starts the next step (Sec. 3.5.1).
    std::vector<ContactPoint> points;

    // Algorithm 1: warm start the cached points, add stochastic samples, remove duplicates and
    // update the cache. dt is the time step used to advect the cache (Eq. 6).
    void update(const CollisionParams &params, float dt, Random &rng, CollisionStats &stats);

private:
    void addPoint(ContactPoint point, float3 seed, const CollisionParams &params, Random &rng, CollisionStats &stats);
    float3 stochasticSample(const CollisionParams &params, Random &rng) const;
};
