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

#include <utility>

#include "collide.h"

// The three optimizers compared in the paper (Sec. 3.3 and 4). All of them minimize the collision
// objective g (Eq. 3) from an initial point inside the search domain V (Sec. 3.1), and terminate
// once the iterates change by less than tau, ||x(k+1) - x(k)|| < tau (Sec. 3.3).

int nelderMead(const Objective &g, float3 &x, const AABB &domain, float3 initialStep, float tolerance, int maxIterations)
{
    // Derivative free simplex method [NM65] with the standard coefficients: reflection 1,
    // expansion 2, contraction 1/2 and shrink 1/2. Only distance queries are needed. Trial points
    // are projected onto V.
    auto project = [&](float3 p) { return clamp(p, domain.min, domain.max); };
    x = project(x);
    float3 v[4] = {x, x, x, x};
    for (int i = 0; i < 3; i++)
    {
        // Initial simplex along the axes, pointing into V
        v[i + 1][i] += x[i] + initialStep[i] <= domain.max[i] ? initialStep[i] : -initialStep[i];
        v[i + 1] = project(v[i + 1]);
    }
    float f[4];
    for (int i = 0; i < 4; i++)
        f[i] = g(v[i]);

    int it = 0;
    for (; it < maxIterations; it++)
    {
        // Order the vertices from best to worst
        for (int i = 1; i < 4; i++)
            for (int j = i; j > 0 && f[j] < f[j - 1]; j--)
                std::swap(f[j], f[j - 1]), std::swap(v[j], v[j - 1]);

        // Converged when the whole simplex lies within tau of the best vertex, or when the objective
        // is flat over the simplex (the value based test of [NM65])
        float extent = max(length(v[1] - v[0]), max(length(v[2] - v[0]), length(v[3] - v[0])));
        if (extent < tolerance || f[3] - f[0] < 0.05f * tolerance)
            break;

        float3 centroid = (v[0] + v[1] + v[2]) / 3.0f;
        float3 reflected = project(centroid + (centroid - v[3]));
        float fr = g(reflected);
        if (fr < f[0])
        {
            float3 expanded = project(centroid + (centroid - v[3]) * 2.0f);
            float fe = g(expanded);
            v[3] = fe < fr ? expanded : reflected;
            f[3] = min(fe, fr);
        }
        else if (fr < f[2])
        {
            v[3] = reflected;
            f[3] = fr;
        }
        else
        {
            // Contract towards the better of the reflected and worst points, shrink if that fails
            bool outside = fr < f[3];
            float3 contracted = centroid + ((outside ? reflected : v[3]) - centroid) * 0.5f;
            float fc = g(contracted);
            if (fc < min(fr, f[3]))
            {
                v[3] = contracted;
                f[3] = fc;
            }
            else
            {
                for (int i = 1; i < 4; i++)
                {
                    v[i] = v[0] + (v[i] - v[0]) * 0.5f;
                    f[i] = g(v[i]);
                }
            }
        }
    }

    int best = 0;
    for (int i = 1; i < 4; i++)
        if (f[i] < f[best])
            best = i;
    x = v[best];
    return it;
}

int ellipsoidMethod(const Objective &g, float3 &x, const AABB &domain, float3x3 &E, float tolerance, int maxIterations)
{
    // Deep cut ellipsoid method [LM24]. The ellipsoid {y : (y - x)^T E^-1 (y - x) <= 1} contains the
    // minimizer for convex problems, every cut removes (more than) half of it.
    const float n = 3.0f;
    float3 best = clamp(x, domain.min, domain.max);
    float bestValue = INFINITY;

    int it = 0;
    for (; it < maxIterations; it++)
    {
        // A center outside V is cut by the face of V it violates the most (a feasibility cut),
        // otherwise by the gradient of g, deeper the more its value exceeds the best one so far
        float3 grad = {0, 0, 0};
        float value = 0, violation = 0;
        for (int i = 0; i < 3; i++)
        {
            float below = domain.min[i] - x[i], above = x[i] - domain.max[i];
            if (max(below, above) > violation)
            {
                violation = max(below, above);
                grad = {0, 0, 0};
                grad[i] = above > below ? 1.0f : -1.0f;
            }
        }
        if (violation == 0)
        {
            value = g(x, &grad);
            if (value < bestValue)
                bestValue = value, best = x;
            if (length(grad) < tolerance)
                break;
        }

        float3 Eg = E * grad;
        float gEg = dot(grad, Eg);
        if (!(gEg > 1e-12f))
            break;

        float s = sqrtf(gEg);
        float a = min((violation > 0 ? violation : value - bestValue) / s, 0.5f);
        float3 step = Eg * ((1 + n * a) / ((n + 1) * s));
        x -= step;
        E = (E - outer(Eg, Eg) * (2 * (1 + n * a) / ((n + 1) * (1 + a) * gEg))) * (n * n * (1 - a * a) / (n * n - 1));
        E = (E + transpose(E)) * 0.5f;

        if (length(step) < tolerance)
            break;
    }

    x = best;
    return it;
}

int gradientDescent(const Objective &g, float3 &x, const AABB &domain, float stepSize, float tolerance, int maxIterations, Random &rng)
{
    // Stochastic (projected) gradient descent [RM51] with a decaying step size and a decaying random
    // perturbation of the gradient. Iterates are kept inside the search domain V.
    x = clamp(x, domain.min, domain.max);
    float3 best = x;
    float bestValue = INFINITY;

    int it = 0;
    for (; it < maxIterations; it++)
    {
        float3 grad;
        float value = g(x, &grad);
        if (value < bestValue)
            bestValue = value, best = x;
        float gradLength = length(grad);
        if (gradLength < tolerance)
            break;

        float decay = 1.0f / sqrtf((float)(it + 1));
        float3 noise = rng.onSphere() * (0.25f * decay * max(gradLength, 1.0f));
        float3 next = clamp(x - (grad + noise) * (stepSize * decay), domain.min, domain.max);
        bool converged = length(next - x) < tolerance;
        x = next;
        if (converged)
            break;
    }

    x = best;
    return it;
}
