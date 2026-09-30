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
#include <chrono>
#include <numeric>

#include "solver.h"

// Frictional contact constraint built from an active point of a manifold
struct Contact
{
    Body *bodyA, *bodyB;
    ContactPoint *point;
    float3 rA, rB, normal, tangent[2];
    float normalMass, tangentMass[2];
    float bias, friction;
    float normalImpulse, tangentImpulse[2], pseudoImpulse;
};

static void applyImpulse(Body *body, float3 r, float3 impulse)
{
    body->velocity += impulse * body->invMass;
    body->angularVelocity += body->invInertiaWorld * cross(r, impulse);
}

static void applyPseudoImpulse(Body *body, float3 r, float3 impulse)
{
    body->pseudoVelocity += impulse * body->invMass;
    body->pseudoAngularVelocity += body->invInertiaWorld * cross(r, impulse);
}

static float3 pseudoPointVelocity(const Body *body, float3 r) { return body->pseudoVelocity + cross(body->pseudoAngularVelocity, r); }

static float effectiveMass(const Body *a, const Body *b, float3 rA, float3 rB, float3 dir)
{
    // At least one of the bodies is dynamic, so the denominator is positive
    float3 ra = cross(rA, dir), rb = cross(rB, dir);
    return 1.0f / (a->invMass + b->invMass + dot(ra, a->invInertiaWorld * ra) + dot(rb, b->invInertiaWorld * rb));
}

Body *Solver::add(Body *body)
{
    bodies.emplace_back(body);
    return body;
}

void Solver::clear()
{
    bodies.clear();
    manifolds.clear();
    dragBody = nullptr;
    rng = Random();
}

void Solver::step()
{
    stats = CollisionStats();
    float h = dt / substeps;
    for (int substep = 0; substep < substeps; substep++)
    {
        // Collision detection at the current positions. The work and time add up over the
        // substeps, pairs and contacts are those of the last substep.
        for (auto &body : bodies)
            body->updateBounds();
        stats.pairs = stats.contacts = 0;
        collide(h);

        // External forces
        for (auto &body : bodies)
            if (body->invMass > 0)
                body->velocity += gravity * h;
        applyDrag(h);

        // Build the contact constraints and warm start them with the impulses cached in the manifold
        std::vector<Contact> contacts;
        for (auto &entry : manifolds)
        {
            Manifold &m = entry.second;
            for (ContactPoint &p : m.points)
            {
                if (!p.active)
                {
                    p.normalImpulse = 0, p.frictionImpulse = {0, 0, 0};
                    continue;
                }

                Contact c;
                c.bodyA = m.bodyA, c.bodyB = m.bodyB, c.point = &p;
                c.rA = p.x - m.bodyA->position;
                c.rB = p.x - m.bodyB->position;
                c.normal = p.normal;
                tangents(c.normal, c.tangent[0], c.tangent[1]);
                c.normalMass = effectiveMass(c.bodyA, c.bodyB, c.rA, c.rB, c.normal);
                for (int i = 0; i < 2; i++)
                    c.tangentMass[i] = effectiveMass(c.bodyA, c.bodyB, c.rA, c.rB, c.tangent[i]);
                c.friction = sqrtf(m.bodyA->friction * m.bodyB->friction);

                // The contact margin is a skin that keeps the bodies apart. Its penetration is
                // measured with the separation, the full gap (or overlap) of the surfaces. The depth
                // d of Eq. 5 is only about half of it, since the optimizer balances phiA and phiB.
                // Within the allowed penetration a contact only stops the bodies from approaching,
                // which keeps it in the cache without jittering. Deeper ones are pushed apart by
                // Baumgarte stabilization (below).
                float penetration = params.margin - p.separation;
                c.bias = max(-baumgarte * max(penetration - params.allowedPenetration, 0.0f) / h, -maxCorrection);

                c.pseudoImpulse = 0;
                c.normalImpulse = p.normalImpulse;
                c.tangentImpulse[0] = dot(p.frictionImpulse, c.tangent[0]);
                c.tangentImpulse[1] = dot(p.frictionImpulse, c.tangent[1]);
                float3 impulse = c.normal * c.normalImpulse + c.tangent[0] * c.tangentImpulse[0] + c.tangent[1] * c.tangentImpulse[1];
                applyImpulse(c.bodyA, c.rA, -impulse);
                applyImpulse(c.bodyB, c.rB, impulse);
                contacts.push_back(c);
            }
        }

        // Sequential impulses (projected Gauss-Seidel)
        for (int it = 0; it < iterations; it++)
        {
            for (Contact &c : contacts)
            {
                // Non-penetration: v_n >= 0 with a non-negative accumulated impulse
                float3 dv = c.bodyB->pointVelocity(c.point->x) - c.bodyA->pointVelocity(c.point->x);
                float lambda = -dot(dv, c.normal) * c.normalMass;
                float previous = c.normalImpulse;
                c.normalImpulse = max(previous + lambda, 0.0f);
                float3 impulse = c.normal * (c.normalImpulse - previous);
                applyImpulse(c.bodyA, c.rA, -impulse);
                applyImpulse(c.bodyB, c.rB, impulse);

                // Coulomb friction, clamped to the friction cone
                dv = c.bodyB->pointVelocity(c.point->x) - c.bodyA->pointVelocity(c.point->x);
                float t0 = c.tangentImpulse[0] - dot(dv, c.tangent[0]) * c.tangentMass[0];
                float t1 = c.tangentImpulse[1] - dot(dv, c.tangent[1]) * c.tangentMass[1];
                float limit = c.friction * c.normalImpulse, len = sqrtf(t0 * t0 + t1 * t1);
                if (len > limit)
                    t0 *= limit / len, t1 *= limit / len;
                impulse = c.tangent[0] * (t0 - c.tangentImpulse[0]) + c.tangent[1] * (t1 - c.tangentImpulse[1]);
                c.tangentImpulse[0] = t0, c.tangentImpulse[1] = t1;
                applyImpulse(c.bodyA, c.rA, -impulse);
                applyImpulse(c.bodyB, c.rB, impulse);
            }
        }

        // Store the impulses in the manifold for warm starting the next step
        for (Contact &c : contacts)
        {
            c.point->normalImpulse = c.normalImpulse;
            c.point->frictionImpulse = c.tangent[0] * c.tangentImpulse[0] + c.tangent[1] * c.tangentImpulse[1];
        }

        // Baumgarte stabilization with split impulses: v_n + bias >= 0 is solved for pseudo velocities,
        // which move the bodies in this substep and are then discarded. Adding the correction to the
        // velocities instead lets contacts that cannot all be corrected at once, such as a spike wedged
        // between two legs, pump energy into the bodies and make them jump.
        for (auto &body : bodies)
            body->pseudoVelocity = body->pseudoAngularVelocity = {0, 0, 0};
        for (int it = 0; it < iterations; it++)
        {
            for (Contact &c : contacts)
            {
                float3 dv = pseudoPointVelocity(c.bodyB, c.rB) - pseudoPointVelocity(c.bodyA, c.rA);
                float lambda = -(dot(dv, c.normal) + c.bias) * c.normalMass;
                float previous = c.pseudoImpulse;
                c.pseudoImpulse = max(previous + lambda, 0.0f);
                float3 impulse = c.normal * (c.pseudoImpulse - previous);
                applyPseudoImpulse(c.bodyA, c.rA, -impulse);
                applyPseudoImpulse(c.bodyB, c.rB, impulse);
            }
        }

        // Integrate positions, with a little damping
        for (auto &body : bodies)
        {
            if (body->isStatic)
                continue;
            body->velocity *= expf(-0.05f * h);
            body->angularVelocity *= expf(-0.3f * h);
            body->position += (body->velocity + body->pseudoVelocity) * h;
            body->orientation = integrate(body->orientation, body->angularVelocity + body->pseudoAngularVelocity, h);
            body->updateInertia();
        }
    }
}

void Solver::collide(float h)
{
    // Broad phase: sort and sweep along x over the bounding boxes grown by half the margin. Pairs
    // that already had a manifold keep it, and with it their cache.
    float r = 0.5f * params.margin;
    std::vector<int> order(bodies.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return bodies[a]->bounds.min.x < bodies[b]->bounds.min.x; });

    std::map<std::pair<int, int>, Manifold> overlapping;
    for (size_t i = 0; i < order.size(); i++)
    {
        for (size_t j = i + 1; j < order.size(); j++)
        {
            if (bodies[order[j]]->bounds.min.x - r > bodies[order[i]]->bounds.max.x + r)
                break;

            std::pair<int, int> key = std::minmax(order[i], order[j]);
            Body *a = bodies[key.first].get(), *b = bodies[key.second].get();
            if ((a->isStatic && b->isStatic) || !overlaps(expand(a->bounds, r), expand(b->bounds, r)))
                continue;

            Manifold &manifold = overlapping[key];
            auto previous = manifolds.find(key);
            if (previous != manifolds.end())
                manifold = std::move(previous->second);
            manifold.bodyA = a, manifold.bodyB = b;
        }
    }
    manifolds.swap(overlapping);

    // Narrow phase: Algorithm 1 for every pair
    auto start = std::chrono::steady_clock::now();
    for (auto &entry : manifolds)
        entry.second.update(params, h, rng, stats);
    stats.milliseconds += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void Solver::applyDrag(float h)
{
    if (!dragBody)
        return;

    // Pull the grabbed point towards the target with a critically damped velocity target
    Body *body = dragBody;
    float3 r = rotate(body->orientation, dragAnchor);
    float3 anchor = body->position + r;
    float3 targetVelocity = (dragTarget - anchor) * (0.2f / h) - body->pointVelocity(anchor) * 0.85f;
    float3x3 K = identity3() * body->invMass - skew(r) * body->invInertiaWorld * skew(r);
    float3 impulse = inverse(K) * targetVelocity;
    float maxImpulse = body->mass * 50.0f * h;
    if (length(impulse) > maxImpulse)
        impulse = impulse * (maxImpulse / length(impulse));
    applyImpulse(body, r, impulse);
}

Body *Solver::pick(float3 origin, float3 direction, float3 &hit) const
{
    Body *best = nullptr;
    float bestT = INFINITY;
    for (auto &body : bodies)
    {
        if (body->isStatic)
            continue;

        // Clip the ray against the bounding box, then sphere trace the SDF
        float t0 = 0, t1 = bestT;
        for (int i = 0; i < 3; i++)
        {
            float inv = 1.0f / direction[i];
            float a = (body->bounds.min[i] - origin[i]) * inv, b = (body->bounds.max[i] - origin[i]) * inv;
            t0 = max(t0, min(a, b)), t1 = min(t1, max(a, b));
        }
        float t = t0;
        for (int i = 0; i < 256 && t < t1; i++)
        {
            float d = body->sdf(origin + direction * t);
            if (d < 1e-3f * body->scale)
            {
                best = body.get(), bestT = t;
                break;
            }
            t += max(0.9f * d, 1e-3f * body->scale); // Under-relaxed, since some fields are only quasi-exact
        }
    }
    hit = origin + direction * bestT;
    return best;
}
