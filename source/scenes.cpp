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

#include <cctype>
#include <map>

#include <SDL.h>

#include "scenes.h"

static std::map<std::string, ShapePtr> library;
static std::vector<std::string> imported;
static Random sceneRandom;

static std::string assetPath(const std::string &file)
{
#ifdef __EMSCRIPTEN__
    return "/assets/" + file;
#else
    char *base = SDL_GetBasePath();
    std::string path = std::string(base ? base : "") + "assets/" + file;
    SDL_free(base);
    return path;
#endif
}

static ShapePtr finalized(Shape *shape)
{
    shape->finalize();
    return ShapePtr(shape);
}

ShapePtr getShape(const std::string &name)
{
    auto it = library.find(name);
    if (it != library.end())
        return it->second;

    ShapePtr shape;
    if (name == "Ground")
        shape = ShapePtr(new Box({60, 1, 60}));
    else if (name == "Box")
        shape = finalized(new Box({0.5f, 0.5f, 0.5f}));
    else if (name == "Plank")
        shape = finalized(new Box({1.15f, 0.28f, 0.55f}));
    else if (name == "Bent Plank")
        shape = finalized(new WarpedBox(WarpedBox::Bend, {1.15f, 0.28f, 0.55f}, 0.95f));
    else if (name == "Twisted Plank")
        shape = finalized(new WarpedBox(WarpedBox::Twist, {1.15f, 0.55f, 0.28f}, 1.25f));
    else if (name == "Displaced Plank")
        shape = finalized(new WarpedBox(WarpedBox::Displace, {1.15f, 0.28f, 0.55f}, 0.1f));
    else if (name == "Sphere")
        shape = finalized(new Sphere(0.5f));
    else if (name == "Capsule")
        shape = finalized(new Capsule(0.3f, 0.4f));
    else if (name == "Cylinder")
        shape = finalized(new Cylinder(0.4f, 0.4f));
    else if (name == "Cone")
        shape = finalized(new Cone(0.5f, 0.5f));
    else if (name == "Torus")
        shape = finalized(new Torus(0.5f, 0.2f));
    else if (name == "Spike Plate")
        shape = finalized(new SpikePlate({0.7f, 0.08f, 0.7f}, 10, 0.045f, 0.12f, 0.26f));
    else if (name == "Spiky Wheel")
        shape = finalized(new SpikyWheel(0.35f, 0.48f, 0.08f, 1.4f, 16));
    else
    {
        // Built in meshes, baked into 100^3 voxel SDFs
        std::string error;
        std::string lower = name;
        for (char &c : lower)
            c = (char)tolower(c);
        shape = loadVoxelSdf(assetPath(lower + ".obj"), error);
        if (!shape)
        {
            SDL_Log("%s", error.c_str());
            shape = getShape("Box");
        }
    }
    library[name] = shape;
    return shape;
}

std::vector<std::string> shapeNames()
{
    std::vector<std::string> names = {"Box", "Sphere", "Capsule", "Cylinder", "Cone", "Torus", "Plank", "Bent Plank",
                                      "Twisted Plank", "Displaced Plank", "Spike Plate", "Spiky Wheel", "Bunny", "Cow",
                                      "Chair", "Gear"};
    names.insert(names.end(), imported.begin(), imported.end());
    return names;
}

bool importShape(const std::string &path, std::string &name, std::string &error)
{
    std::shared_ptr<VoxelSdf> shape = loadVoxelSdf(path, error);
    if (!shape)
        return false;
    name = shape->name;
    while (library.count(name))
        name += "'";
    library[name] = shape;
    imported.push_back(name);
    if (!shape->watertight)
        error = "Warning: the mesh is not watertight, inside/outside may be ambiguous";
    return true;
}

// Muted pastel colors similar to the renderings in the paper
static float3 pastel()
{
    float h = sceneRandom.uniform() * 6.0f, s = sceneRandom.uniform(0.25f, 0.45f), v = sceneRandom.uniform(0.75f, 0.9f);
    float f = h - floorf(h), p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));
    switch ((int)h % 6)
    {
    case 0: return {v, t, p};
    case 1: return {q, v, p};
    case 2: return {p, v, t};
    case 3: return {p, q, v};
    case 4: return {t, p, v};
    default: return {v, p, q};
    }
}

Body *spawn(Solver &solver, const std::string &shape, float3 position, quat orientation, float scale)
{
    Body *body = solver.add(new Body(getShape(shape), position, orientation, scale));
    body->color = pastel();
    return body;
}

static Body *spawnStatic(Solver &solver, const std::string &shape, float3 position, quat orientation = {0, 0, 0, 1}, float scale = 1.0f)
{
    Body *body = solver.add(new Body(getShape(shape), position, orientation, scale, true));
    body->color = {0.72f, 0.72f, 0.76f};
    return body;
}

// Every scene starts with a static ground box whose top face is at y = 0
static void begin(Solver &solver, Camera &camera, float3 target, float distance)
{
    solver.clear();
    sceneRandom = Random();
    spawnStatic(solver, "Ground", {0, -1, 0})->visible = false;
    camera.target = target;
    camera.distance = distance;
    camera.yaw = 0.6f;
    camera.pitch = 0.35f;
}

static void boxStack(Solver &solver, Camera &camera)
{
    // Fig. 5 (left): pyramid of analytical boxes. They start a small gap apart, within the contact
    // margin but short of the allowed penetration, so that the stack is at rest from the start
    // instead of being pushed apart by the solver.
    begin(solver, camera, {0, 2.5f, 0}, 13);
    const int levels = 7;
    float gap = solver.params.margin - 0.5f * solver.params.allowedPenetration;
    for (int level = 0; level < levels; level++)
        for (int i = 0; i < levels - level; i++)
            spawn(solver, "Box", {(i - 0.5f * (levels - level - 1)) * 1.05f, 0.5f + gap + level * (1.0f + gap), 0});
}

static void fallingBoxes(Solver &solver, Camera &camera, const char *shape)
{
    // Fig. 8: pile of falling boxes, optionally distorted by the operators of Sec. 4.5
    begin(solver, camera, {0, 1.2f, 0}, 11);
    for (int layer = 0; layer < 2; layer++)
    {
        for (int z = 0; z < 2; z++)
        {
            for (int x = 0; x < 3; x++)
            {
                float3 p = {(x - 1.0f) * 1.85f + sceneRandom.uniform(-0.1f, 0.1f), 2.2f + layer * 3.0f + 0.35f * (x + z),
                            (z - 0.5f) * 1.55f + sceneRandom.uniform(-0.1f, 0.1f)};
                spawn(solver, shape, p, sceneRandom.orientation(), 0.9f);
            }
        }
    }
}

static void boxes(Solver &solver, Camera &camera) { fallingBoxes(solver, camera, "Plank"); }
static void bentBoxes(Solver &solver, Camera &camera) { fallingBoxes(solver, camera, "Bent Plank"); }
static void twistedBoxes(Solver &solver, Camera &camera) { fallingBoxes(solver, camera, "Twisted Plank"); }
static void displacedBoxes(Solver &solver, Camera &camera) { fallingBoxes(solver, camera, "Displaced Plank"); }

static void primitives(Solver &solver, Camera &camera)
{
    // Every analytical primitive, dropped into a pile
    begin(solver, camera, {0, 1.0f, 0}, 10);
    const char *names[] = {"Box", "Sphere", "Capsule", "Cylinder", "Cone", "Torus"};
    int n = 0;
    for (int layer = 0; layer < 3; layer++)
        for (int z = 0; z < 4; z++)
            for (int x = 0; x < 4; x++, n++)
                spawn(solver, names[n % 6], {(x - 1.5f) * 1.3f, 1.0f + layer * 1.4f, (z - 1.5f) * 1.3f}, sceneRandom.orientation());
}

static void gearStack(Solver &solver, Camera &camera)
{
    // Fig. 5 (right): tower of voxel gears
    begin(solver, camera, {0, 2.0f, 0}, 9);
    for (int i = 0; i < 10; i++)
        spawn(solver, "Gear", {sceneRandom.uniform(-0.05f, 0.05f), 0.2f + i * 0.45f, sceneRandom.uniform(-0.05f, 0.05f)},
              axisAngle({0, 1, 0}, sceneRandom.uniform(0, 2 * PI)), 1.5f);
}

static void chairs(Solver &solver, Camera &camera)
{
    // Fig. 9 (bottom left): pile of voxel chairs
    begin(solver, camera, {0, 1.0f, 0}, 7);
    for (int layer = 0; layer < 3; layer++)
        for (int z = 0; z < 3; z++)
            for (int x = 0; x < 3; x++)
                spawn(solver, "Chair", {(x - 1.0f) * 1.4f, 0.8f + layer * 1.6f, (z - 1.0f) * 1.3f}, sceneRandom.orientation(), 1.15f);
}

static void cows(Solver &solver, Camera &camera)
{
    // Fig. 1 (bottom right): cows dropped onto a bed of spikes
    begin(solver, camera, {0, 1.2f, 0}, 9);
    spawnStatic(solver, "Spike Plate", {0, 0.08f * 4.0f, 0}, {0, 0, 0, 1}, 4.0f);
    for (int layer = 0; layer < 2; layer++)
        for (int z = 0; z < 3; z++)
            for (int x = 0; x < 3; x++)
                spawn(solver, "Cow", {(x - 1.0f) * 1.3f, 2.2f + layer * 1.2f, (z - 1.0f) * 1.3f}, sceneRandom.orientation(), 1.3f);
}

static void bunnies(Solver &solver, Camera &camera)
{
    // Fig. 1 (top right): pile of voxel bunnies
    begin(solver, camera, {0, 0.8f, 0}, 7.5f);
    for (int layer = 0; layer < 3; layer++)
        for (int z = 0; z < 3; z++)
            for (int x = 0; x < 3; x++)
                spawn(solver, "Bunny", {(x - 1.0f) * 1.1f + 0.2f * layer, 0.8f + layer * 1.1f, (z - 1.0f) * 1.1f}, sceneRandom.orientation());
}

static void spikyWheel(Solver &solver, Camera &camera)
{
    // Fig. 9 (bottom right): a heavy spiky wheel rolling on its spikes into a set of pins
    begin(solver, camera, {1, 1.2f, 0}, 13);
    Body *wheel = spawn(solver, "Spiky Wheel", {-6, 2.0f, 0}, axisAngle({1, 0, 0}, 0.5f * PI), 1.2f);
    wheel->mass *= 4, wheel->inertia = wheel->inertia * 4;
    wheel->velocity = {6, 0, 0};
    wheel->angularVelocity = {0, 0, -4};
    wheel->updateInertia();
    for (int row = 0; row < 4; row++)
        for (int i = 0; i <= row; i++)
            spawn(solver, "Capsule", {3.0f + row * 0.8f, 0.7f, (i - 0.5f * row) * 0.9f}, {0, 0, 0, 1}, 1.0f);
}

void buildPile(Solver &solver, Camera &camera, const std::string &shape)
{
    begin(solver, camera, {0, 1.2f, 0}, 10);
    for (int layer = 0; layer < 3; layer++)
        for (int z = 0; z < 3; z++)
            for (int x = 0; x < 3; x++)
                spawn(solver, shape, {(x - 1.0f) * 1.3f, 1.0f + layer * 1.3f, (z - 1.0f) * 1.3f}, sceneRandom.orientation());
}

const Scene scenes[] = {
    {"Box Stack", "Pyramid of analytical boxes (Fig. 5, left)", boxStack},
    {"Gear Stack", "Tower of voxel gears (Fig. 5, right)", gearStack},
    {"Falling Boxes", "Undistorted pile of boxes (Fig. 8)", boxes},
    {"Falling Boxes: Bend", "BEND distorted boxes (Sec. 4.5, Fig. 8)", bentBoxes},
    {"Falling Boxes: Twist", "TWIST distorted boxes (Sec. 4.5, Fig. 8)", twistedBoxes},
    {"Falling Boxes: Displace", "DISPLACE distorted boxes (Sec. 4.5, Fig. 8)", displacedBoxes},
    {"Pile of Chairs", "Voxel chairs with thin, interlocking legs (Fig. 9)", chairs},
    {"Spiky Wheel", "Analytical composite rolling on its spikes (Fig. 9)", spikyWheel},
    {"Cows on Spikes", "Voxel cows on an analytical bed of spikes (Fig. 1)", cows},
    {"Bunny Pile", "Pile of voxel bunnies (Fig. 1)", bunnies},
    {"Primitives", "All analytical primitives", primitives},
};

const int sceneCount = sizeof(scenes) / sizeof(scenes[0]);
