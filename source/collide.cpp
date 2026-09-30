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

#include "collide.h"

const char *optimizerNames[OPTIMIZER_COUNT] = {"Nelder-Mead", "Ellipsoid", "Gradient Descent"};

float softmax(float phiA, float phiB, float epsilon)
{
    // Eq. 2, an upper bound of max(phiA, phiB) that is smooth where phiA = phiB (Eq. 1 when epsilon = 0)
    return 0.5f * (phiA + phiB + sqrtf((phiA - phiB) * (phiA - phiB) + epsilon));
}

float Objective::operator()(float3 x, float3 *grad) const
{
    (*queries)++;

    float3 gradA, gradB;
    float phiA = bodyA->sdf(x, grad ? &gradA : nullptr);
    float phiB = bodyB->sdf(x, grad ? &gradB : nullptr);
    float root = sqrtf((phiA - phiB) * (phiA - phiB) + epsilon);
    float value = 0.5f * (phiA + phiB + root);
    if (grad)
    {
        float t = root > 0 ? (phiA - phiB) / root : 0.0f;
        *grad = gradA * (0.5f * (1 + t)) + gradB * (0.5f * (1 - t));
    }

    // Eq. 3: repulsion pushing x away from the points already in the manifold
    for (const ContactPoint &point : *manifold)
    {
        float3 dx = x - point.x;
        float len = length(dx);
        value -= alpha * len;
        if (grad && len > 1e-9f)
            *grad -= dx * (alpha / len);
    }
    return value;
}

void Manifold::update(const CollisionParams &params, float dt, Random &rng, CollisionStats &stats)
{
    stats.pairs++;

    // Sec. 3.1: points are searched for in the overlap V of the bounding boxes, which are grown
    // by half the margin so that contacts are found as soon as the bodies are within the margin
    float r = 0.5f * params.margin;
    volume = intersect(expand(bodyA->bounds, r), expand(bodyB->bounds, r));

    // The best n_cache points of the previous step form CACHE(A, B) (Alg. 1, line 29). They are
    // taken at the start of the step so that they also carry the solver impulses of that step.
    int cached = std::min((int)points.size(), params.warmStart ? params.cacheSize : 0);
    std::vector<ContactPoint> cache(points.begin(), points.begin() + cached);
    points.clear();

    // Alg. 1, lines 4-10: warm start from the cached points
    for (ContactPoint point : cache)
    {
        // Eq. 6: advect with the average velocity of the two bodies (only the moving one if the other is static)
        float3 v = bodyA->isStatic   ? bodyB->pointVelocity(point.x)
                   : bodyB->isStatic ? bodyA->pointVelocity(point.x)
                                     : (bodyA->pointVelocity(point.x) + bodyB->pointVelocity(point.x)) * 0.5f;
        point.cached = true;
        addPoint(point, point.x + v * dt, params, rng, stats);
    }

    // Alg. 1, lines 12-18: stochastic samples, which also top the cache back up to n_cache points.
    // The random samples find new contact regions (Sec. 3.5.2), but two convex shapes intersect in
    // a single convex region, which Nelder-Mead and the ellipsoid method reach from the cached
    // points. So they are skipped while every cached point is a contact. Gradient descent stalls
    // too easily (Sec. 3.3) and keeps them.
    int contacts = (int)std::count_if(cache.begin(), cache.end(), [](const ContactPoint &p) { return p.active; });
    bool skip = params.skipConvexSamples && params.optimizer != OPTIMIZER_GRADIENT_DESCENT && contacts > 0 &&
                contacts == params.cacheSize && bodyA->shape->convex && bodyB->shape->convex;
    int samples = (skip ? 0 : params.randomSamples) + std::max(0, params.cacheSize - (int)cache.size());
    for (int i = 0; i < samples; i++)
        addPoint(ContactPoint(), stochasticSample(params, rng), params, rng, stats);

    // Alg. 1, lines 20-27 (Sec. 3.7): remove points that converged within tau of an earlier point.
    // Cached points come first, so they survive over new samples.
    std::vector<ContactPoint> unique;
    for (const ContactPoint &p : points)
    {
        bool duplicate = false;
        for (const ContactPoint &q : unique)
            duplicate |= lengthSq(p.x - q.x) < params.tolerance * params.tolerance;
        if (!duplicate)
            unique.push_back(p);
    }
    points.swap(unique);

    // Alg. 1, line 28: sort by g(x_i) (Eq. 3) evaluated against the final manifold, which prefers
    // deep points that are far from the others. The first n_cache points become the next cache.
    // A new sample has to improve on a cached point by a threshold to replace it (not in the
    // paper). Otherwise points that are about equally deep, such as on flat contact faces, keep
    // replacing each other and the contacts (and the solver's warm start) never settle.
    for (ContactPoint &p : points)
    {
        p.rank = p.softmax + (p.cached ? 0.0f : params.replacementThreshold);
        for (const ContactPoint &q : points)
            p.rank -= params.alpha * length(p.x - q.x);
    }
    std::stable_sort(points.begin(), points.end(), [](const ContactPoint &a, const ContactPoint &b) { return a.rank < b.rank; });

    for (const ContactPoint &p : points)
        stats.contacts += p.active;
}

void Manifold::addPoint(ContactPoint point, float3 seed, const CollisionParams &params, Random &rng, CollisionStats &stats)
{
    // Eq. 4: x* = argmin g(x) starting from the seed, repelled by the points found so far (Eq. 3)
    Objective g = {bodyA, bodyB, &points, params.epsilon, params.alpha, &stats.queries};
    float3 x = seed;
    if (params.optimizer == OPTIMIZER_NELDER_MEAD)
    {
        // Sec. 3.7: the initial simplex spans an axis aligned ellipsoid 10% of the size of V. A
        // cached point only has to follow the bodies, so its simplex just covers how far it was
        // advected (not in the paper). Otherwise it would wander on flat contacts, where all points
        // are about equally deep.
        float3 step = max(size(volume) * 0.1f, float3{1, 1, 1} * (0.5f * params.tolerance));
        if (point.cached)
            step = min(step, float3{1, 1, 1} * max(length(seed - point.x), 5 * params.tolerance));
        stats.iterations += nelderMead(g, x, volume, step, params.tolerance, params.maxIterations);
    }
    else if (params.optimizer == OPTIMIZER_ELLIPSOID)
    {
        // Start from an ellipsoid centered at x that encloses V. Cached points are warm started from
        // their previous ellipsoid (Sec. 3.7, as in [LM24]), blended with this one so that the
        // minimizer stays inside after the bodies have moved.
        float3 extent = size(volume) * 0.5f + abs(x - center(volume));
        float3x3 E = diagonal(extent * extent * 3.0f);
        if (point.cached)
            E = point.ellipsoid * 0.75f + E * 0.25f;
        stats.iterations += ellipsoidMethod(g, x, volume, E, params.tolerance, params.maxIterations);
        point.ellipsoid = E;
    }
    else
    {
        stats.iterations += gradientDescent(g, x, volume, params.stepSize, params.tolerance, params.maxIterations, rng);
    }

    // Sec. 3.4, Eq. 5: depth and normal from the more deeply penetrating field. The normal of B is
    // negated so that n always points from A towards B.
    float3 gradA, gradB;
    float phiA = bodyA->sdf(x, &gradA), phiB = bodyB->sdf(x, &gradB);
    stats.queries++;
    float3 nA = normalize(gradA), nB = -normalize(gradB), previous = point.normal;
    point.x = x;
    point.depth = min(phiA, phiB);
    point.normal = phiA < phiB ? nA : nB;
    point.separation = phiA + phiB;
    point.softmax = softmax(phiA, phiB, params.epsilon);

    // The optimizer converges where phiA and phiB are about equal, so either field can be the deeper
    // one. Where the surfaces face the same way, e.g. near the axis of a thin spike inside another
    // body, nA and nB are opposite, and the normal would flip from step to step and push the bodies
    // back and forth. So cached points take whichever of the two is closer to their previous normal.
    float3 coherent = dot(nA, previous) > dot(nB, previous) ? nA : nB;
    if (point.cached)
        point.normal = coherent;

    // Unless x* is inside both bodies, the separation is the gap between the closest points of A
    // and B. Outside both, the normal points across that gap, unless that reverses the normal of a
    // cached point. The closest points are found with a Newton step, which also holds for fields
    // that are not exact distances.
    if (phiA > 0 || phiB > 0)
    {
        float3 gap = gradA * (phiA / max(lengthSq(gradA), 1e-8f)) - gradB * (phiB / max(lengthSq(gradB), 1e-8f));
        point.separation = length(gap);
        if (phiA > 0 && phiB > 0 && point.separation > 1e-6f && !(point.cached && dot(gap, previous) < 0))
            point.normal = gap / point.separation;
    }

    // x* is a contact when it lies inside both bodies (phiA < 0 and phiB < 0, Sec. 3.4). Bodies
    // closer than the contact margin are also in contact (not in the paper): the margin is a skin
    // around them, which the solver keeps apart.
    point.active = point.separation < params.margin;
    points.push_back(point);
}

float3 Manifold::stochasticSample(const CollisionParams &params, Random &rng) const
{
    // STOCHASTICSAMPLE (Sec. 3.5.2): a uniformly chosen high curvature seed of either body inside V,
    // falling back to a uniform sample of V. Seeds are culled in the local frame of each body first.
    float3 sample = rng.inBox(volume);
    if (!params.curvatureSampling)
        return sample;

    int count = 0;
    for (const Body *body : {bodyA, bodyB})
    {
        AABB local = body->localBounds(volume);
        for (float3 seed : body->shape->seeds)
        {
            if (!contains(local, seed))
                continue;
            float3 x = body->toWorld(seed);
            if (contains(volume, x) && rng.integer(++count) == 0) // Reservoir sampling
                sample = x;
        }
    }
    return sample;
}
