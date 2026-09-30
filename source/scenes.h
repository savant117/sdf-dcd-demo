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

#include "render.h"

// Preset scenes, most of them smaller versions of the experiments in the paper
struct Scene
{
    const char *name;
    const char *description;
    void (*build)(Solver &solver, Camera &camera);
};

extern const Scene scenes[];
extern const int sceneCount;

// Shared shapes, created on first use. Mesh shapes are baked into voxel SDFs from assets/*.obj.
ShapePtr getShape(const std::string &name);

// Names of all shapes that can be dropped into a scene (including imported meshes)
std::vector<std::string> shapeNames();

// Bakes an OBJ file into a voxel SDF and registers it under its file name, which is returned in
// name. Returns false and sets error on failure (error also holds warnings on success).
bool importShape(const std::string &path, std::string &name, std::string &error);

// Adds a body of the given shape to the solver
Body *spawn(Solver &solver, const std::string &shape, float3 position, quat orientation = {0, 0, 0, 1}, float scale = 1.0f);

// Drops a pile of the given shape onto the ground (used for imported meshes)
void buildPile(Solver &solver, Camera &camera, const std::string &shape);
