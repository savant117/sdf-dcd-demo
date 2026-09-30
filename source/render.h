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
#include <vector>

#include "solver.h"

// Orbit camera around a target point
struct Camera
{
    float3 target = {0, 1, 0};
    float distance = 10.0f;
    float yaw = 0.6f, pitch = 0.35f;
    float fovY = rad(45.0f);
    float shift = 0.0f; // Horizontal offset of the view center in normalized device coordinates

    float3 eye() const
    {
        return target + float3{cosf(pitch) * sinf(yaw), sinf(pitch), cosf(pitch) * cosf(yaw)} * distance;
    }

    // Tangent of half the vertical field of view, widened on portrait screens to keep the horizontal one
    float tanHalfFov(float aspect) const
    {
        return tanf(0.5f * fovY) / min(aspect, 1.0f);
    }

    // Perspective projection, off center by the shift
    float4x4 projection(float aspect) const
    {
        float4x4 P = perspective(2 * atanf(tanHalfFov(aspect)), aspect, 0.05f, 500.0f);
        P.m[8] = -shift;
        return P;
    }

    // World space direction through the pixel (x, y), with y pointing down
    float3 ray(float x, float y, float width, float height) const
    {
        float3 forward = normalize(target - eye());
        float3 right = normalize(cross(forward, {0, 1, 0}));
        float3 up = cross(right, forward);
        float t = tanHalfFov(width / height);
        return normalize(forward + right * ((2 * x / width - 1 - shift) * t * width / height) + up * ((1 - 2 * y / height) * t));
    }
};

struct RenderOptions
{
    bool cachedContacts = false; // Contact points (and normals) warm started from the cache
    bool randomContacts = false; // Contact points (and normals) found from new stochastic samples
    bool seeds = false;          // High curvature seed points (Fig. 3)
    bool volumes = false;        // Search domains V of the manifolds
    float pointScale = 1.0f;     // Size of the points in pixels per UI unit

    // Screen space ambient occlusion with visibility bitmasks [Therrien et al. 2023]
    bool ambientOcclusion = true;
    float aoRadius = 1.0f;     // World space radius of the sampled hemisphere
    float aoThickness = 0.25f; // Thickness assumed behind every depth sample
};

struct Renderer
{
    bool init();
    void draw(const Solver &solver, const Camera &camera, int width, int height, const RenderOptions &options);

private:
    struct GpuMesh
    {
        unsigned int vao = 0, vbo = 0, ibo = 0;
        int count = 0;
    };

    unsigned int meshProgram = 0, depthProgram = 0, lineProgram = 0, skyProgram = 0;
    unsigned int normalProgram = 0, aoProgram = 0, denoiseProgram = 0;
    unsigned int shadowFramebuffer = 0, shadowTexture = 0;
    unsigned int lineVao = 0, lineVbo = 0, emptyVao = 0;
    std::map<const Shape *, GpuMesh> meshes;
    std::vector<float> lineVertices, pointVertices, seedVertices;

    // Ambient occlusion: view space normals and depth, then the noisy and denoised occlusion
    unsigned int normalFramebuffer = 0, normalTexture = 0, depthTexture = 0;
    unsigned int aoFramebuffer = 0, aoTexture = 0, denoiseFramebuffer = 0, denoiseTexture = 0;
    unsigned int whiteTexture = 0; // Used instead when ambient occlusion is off
    int aoWidth = 0, aoHeight = 0;

    const GpuMesh &gpuMesh(const Shape *shape);
    void drawFloor(unsigned int program);
    void drawBodies(const Solver &solver, unsigned int program);
    void drawVertices(const std::vector<float> &vertices, unsigned int mode, const float4x4 &viewProj, float pointSize);
    void ambientOcclusion(const Solver &solver, const float4x4 &view, const float4x4 &projection, float shift,
                          int width, int height, const RenderOptions &options);
};
