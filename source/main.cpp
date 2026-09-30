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

#include <cfloat>
#include <cstdio>
#include <string>
#include <vector>

#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/html5.h>
#endif

#ifdef _WIN32
#include <windows.h>
#include <commdlg.h>
#endif

#include "gl.h"
#include "scenes.h"

SDL_Window *window = nullptr;
SDL_GLContext context = nullptr;
bool running = true;

Solver solver;
Renderer renderer;
Camera camera;
RenderOptions renderOptions;
int currentScene = 0;
bool paused = false;
std::string dropShape = "Bunny";
std::string pendingImport;
std::string status;
char importPath[512] = "";
bool showImportPath = false;
float uiScale = 1.0f;

// Interaction state
bool orbiting = false;
float dragDistance = 0;

// Timing and statistics, exponentially smoothed for display
Uint64 lastCounter = 0;
float accumulator = 0;
float frameMs = 0, stepMs = 0, collisionMs = 0, iterations = 0, queries = 0;
float history[120] = {};

void loadScene(int index)
{
    currentScene = index;
    scenes[index].build(solver, camera);
    solver.stats = CollisionStats();
    stepMs = collisionMs = iterations = queries = 0;
}

// OBJ import: a native file dialog on Windows, the browser's file picker on the web and a path
// field elsewhere. Dropping an OBJ file onto the window works everywhere.

#ifdef __EMSCRIPTEN__
extern "C" EMSCRIPTEN_KEEPALIVE void importObj(const char *path)
{
    pendingImport = path;
}

EM_JS(void, openFilePicker, (), {
    var input = document.createElement('input');
    input.type = 'file';
    input.accept = '.obj';
    input.onchange = function() {
        var file = input.files[0];
        if (!file)
            return;
        file.arrayBuffer().then(function(buffer) {
            var path = '/' + file.name.replace(/[^A-Za-z0-9_.-]/g, '_');
            FS.writeFile(path, new Uint8Array(buffer));
            Module.ccall('importObj', null, ['string'], [path]);
        });
    };
    input.click();
});
#endif

void openImportDialog()
{
#if defined(__EMSCRIPTEN__)
    openFilePicker();
#elif defined(_WIN32)
    char path[MAX_PATH] = "";
    OPENFILENAMEA dialog = {};
    dialog.lStructSize = sizeof(dialog);
    dialog.lpstrFilter = "Wavefront OBJ (*.obj)\0*.obj\0All files\0*.*\0";
    dialog.lpstrFile = path;
    dialog.nMaxFile = MAX_PATH;
    dialog.Flags = OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetOpenFileNameA(&dialog))
        pendingImport = path;
#else
    showImportPath = true;
#endif
}

void processImport()
{
    // Show a message for one frame first, since baking the voxel SDF blocks for a moment
    static bool announced = false;
    if (pendingImport.empty())
        return;
    if (!announced)
    {
        status = "Baking the voxel SDF of " + pendingImport.substr(pendingImport.find_last_of("/\\") + 1) + "...";
        announced = true;
        return;
    }
    announced = false;

    std::string name, error;
    Uint64 start = SDL_GetPerformanceCounter();
    if (importShape(pendingImport, name, error))
    {
        double seconds = (SDL_GetPerformanceCounter() - start) / (double)SDL_GetPerformanceFrequency();
        char text[256];
        snprintf(text, sizeof(text), "Imported %s (baked in %.2f s). %s", name.c_str(), seconds, error.c_str());
        status = text;
        dropShape = name;
        buildPile(solver, camera, name);
    }
    else
        status = "Import failed: " + error;
    pendingImport.clear();
}

void dropSelectedShape()
{
    float3 p = camera.target + float3{solver.rng.uniform(-0.5f, 0.5f), 4.0f, solver.rng.uniform(-0.5f, 0.5f)};
    spawn(solver, dropShape, p, solver.rng.orientation());
}

void input()
{
    ImGuiIO &io = ImGui::GetIO();
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    float3 ray = camera.ray(io.MousePos.x, io.MousePos.y, (float)w, (float)h);

    // Left mouse drags bodies, or orbits the camera when pressed over empty space
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse)
    {
        float3 hit;
        solver.dragBody = solver.pick(camera.eye(), ray, hit);
        if (solver.dragBody)
        {
            solver.dragAnchor = rotate(conjugate(solver.dragBody->orientation), hit - solver.dragBody->position);
            dragDistance = length(hit - camera.eye());
        }
        else
            orbiting = true;
    }
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        solver.dragBody = nullptr, orbiting = false;
    if (solver.dragBody)
        solver.dragTarget = camera.eye() + ray * dragDistance;

    // Right mouse orbits, middle mouse pans and the wheel zooms
    if (orbiting || (ImGui::IsMouseDown(ImGuiMouseButton_Right) && !io.WantCaptureMouse))
    {
        camera.yaw -= io.MouseDelta.x * 0.006f;
        camera.pitch = clamp(camera.pitch + io.MouseDelta.y * 0.006f, -0.2f, 1.5f);
    }
    if (ImGui::IsMouseDown(ImGuiMouseButton_Middle) && !io.WantCaptureMouse)
    {
        float3 forward = normalize(camera.target - camera.eye());
        float3 right = normalize(cross(forward, {0, 1, 0})), up = cross(right, forward);
        float speed = camera.distance * 0.0015f;
        camera.target += (up * io.MouseDelta.y - right * io.MouseDelta.x) * speed;
    }
    if (io.MouseWheel != 0 && !io.WantCaptureMouse)
        camera.distance = clamp(camera.distance * powf(0.9f, io.MouseWheel), 1.0f, 100.0f);

    if (!io.WantCaptureKeyboard)
    {
        if (ImGui::IsKeyPressed(ImGuiKey_Space))
            dropSelectedShape();
        if (ImGui::IsKeyPressed(ImGuiKey_P))
            paused = !paused;
        if (ImGui::IsKeyPressed(ImGuiKey_R))
            loadScene(currentScene);
    }
}

void ui()
{
    // The panel fits its height to the contents, and starts collapsed on narrow screens
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(min(360 * uiScale, w - 20.0f), 0), ImGuiCond_Always);
    ImGui::SetNextWindowCollapsed(w < 700, ImGuiCond_FirstUseEver);
    bool expanded = ImGui::Begin("SDF-SDF Collisions");

    // Center the view in the space right of the panel
    float right = expanded ? ImGui::GetWindowPos().x + ImGui::GetWindowSize().x : 0.0f;
    camera.shift = right < 0.5f * w ? right / w : 0.0f;
    if (!expanded)
    {
        ImGui::End();
        return;
    }

    // Leave room for the longest label next to the widgets
    ImGui::PushItemWidth(-ImGui::CalcTextSize("Allowed Penetration").x - 2 * ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::TextDisabled("Left: drag bodies / orbit   Right: orbit");
    ImGui::TextDisabled("Middle: pan   Wheel: zoom   Space: drop");

    if (ImGui::BeginCombo("Scene", scenes[currentScene].name))
    {
        for (int i = 0; i < sceneCount; i++)
            if (ImGui::Selectable(scenes[i].name, i == currentScene))
                loadScene(i);
        ImGui::EndCombo();
    }
    ImGui::TextWrapped("%s", scenes[currentScene].description);
    if (ImGui::Button("Reset"))
        loadScene(currentScene);
    ImGui::SameLine();
    ImGui::Checkbox("Pause", &paused);
    if (paused)
    {
        ImGui::SameLine();
        if (ImGui::Button("Step"))
            solver.step();
    }

    if (ImGui::BeginCombo("Shape", dropShape.c_str()))
    {
        for (const std::string &name : shapeNames())
            if (ImGui::Selectable(name.c_str(), name == dropShape))
                dropShape = name;
        ImGui::EndCombo();
    }
    if (ImGui::Button("Drop Shape"))
        dropSelectedShape();
    ImGui::SameLine();
    if (ImGui::Button("Import OBJ..."))
        openImportDialog();
    if (showImportPath)
    {
        ImGui::InputText("OBJ Path", importPath, sizeof(importPath));
        ImGui::SameLine();
        if (ImGui::Button("Load"))
            pendingImport = importPath;
    }
    if (!status.empty())
        ImGui::TextWrapped("%s", status.c_str());

    CollisionParams &p = solver.params;
    if (ImGui::CollapsingHeader("Collision Detection", ImGuiTreeNodeFlags_DefaultOpen))
    {
        int optimizer = p.optimizer;
        if (ImGui::Combo("Optimizer", &optimizer, optimizerNames, OPTIMIZER_COUNT))
            p.optimizer = (Optimizer)optimizer;
        if (p.optimizer == OPTIMIZER_GRADIENT_DESCENT)
            ImGui::SliderFloat("Step Size", &p.stepSize, 0.01f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderInt("Cached Samples", &p.cacheSize, 0, 16);
        ImGui::SetItemTooltip("n_cache: points warm started from the previous step (Sec. 3.5.1)");
        ImGui::SliderInt("Random Samples", &p.randomSamples, 0, 16);
        ImGui::SetItemTooltip("n_rand: new stochastic samples per body pair and step (Sec. 3.5.2)");
        ImGui::Checkbox("Temporal Cache", &p.warmStart);
        ImGui::SetItemTooltip("Warm start from the cached points of the previous step (Sec. 3.5.1). The contact\n"
                              "impulses are stored with the cached points, so this also warm starts the solver.");
        ImGui::SameLine();
        ImGui::Checkbox("Curvature Sampling", &p.curvatureSampling);
        ImGui::SetItemTooltip("Draw stochastic samples from high curvature seed points instead of uniformly (Sec. 3.5.2)");
        ImGui::Checkbox("Skip Random Samples of Convex Pairs", &p.skipConvexSamples);
        ImGui::SetItemTooltip("Two convex shapes intersect in a single region, so while every cached point is a\n"
                              "contact the random samples are skipped (an addition to Alg. 1, not used with\n"
                              "gradient descent)");
        ImGui::SliderFloat("Softmax Epsilon", &p.epsilon, 0.0f, 0.5f, "%.3f");
        ImGui::SetItemTooltip("Smoothing of the objective (Eq. 2), zero gives the hardmax of Eq. 1");
        ImGui::SliderFloat("Repulsion Alpha", &p.alpha, 0.0f, 0.1f, "%.3f");
        ImGui::SetItemTooltip("Repulsion between collision points (Eq. 3)");
        ImGui::SliderFloat("Replacement", &p.replacementThreshold, 0.0f, 0.05f, "%.3f");
        ImGui::SetItemTooltip("Improvement of g a new sample needs to replace a cached point. Keeps the cache from\n"
                              "churning on contacts where many points are about equally deep (not in the paper).");
        ImGui::SliderFloat("Tolerance", &p.tolerance, 0.001f, 0.1f, "%.3f", ImGuiSliderFlags_Logarithmic);
        ImGui::SetItemTooltip("Optimizer termination tolerance tau (Sec. 3.3), points closer than tau are removed as duplicates (Alg. 1)");
        ImGui::SliderInt("Max Iterations", &p.maxIterations, 1, 200);
        ImGui::SliderFloat("Contact Margin", &p.margin, 0.0f, 0.1f, "%.3f");
        ImGui::SetItemTooltip("Skin around the bodies: contacts are created within this distance and keep it open");
        ImGui::SliderFloat("Allowed Penetration", &p.allowedPenetration, 0.0f, 0.1f, "%.3f");
        ImGui::SetItemTooltip("Penetration of the skin that is not corrected (half the margin by default). Such contacts\n"
                              "stay in the cache and stop the bodies from approaching, which reduces jitter.");
        if (ImGui::Button("Paper Defaults"))
            p = CollisionParams();
    }

    if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Cached Contacts", &renderOptions.cachedContacts);
        ImGui::SetItemTooltip("Contact points warm started from the temporal cache (green) and their normals");
        ImGui::SameLine();
        ImGui::Checkbox("Random Contacts", &renderOptions.randomContacts);
        ImGui::SetItemTooltip("Contact points found from new stochastic samples (orange) and their normals");
        ImGui::Checkbox("Seeds", &renderOptions.seeds);
        ImGui::SetItemTooltip("High curvature seed points (Fig. 3)");
        ImGui::SameLine();
        ImGui::Checkbox("Search Volumes", &renderOptions.volumes);
        ImGui::SetItemTooltip("Overlap V of the bounding boxes of each pair (Sec. 3.1)");
    }

    ImGui::SetNextItemOpen(h >= 700, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader("Statistics"))
    {
        ImGui::Text("Bodies %d   Pairs %d   Contacts %d", (int)solver.bodies.size(), solver.stats.pairs, solver.stats.contacts);
        ImGui::Text("Iterations %.0f   SDF Queries %.0f", iterations, queries);
        ImGui::Text("t_coll %.2f ms   Step %.2f ms   Frame %.1f ms", collisionMs, stepMs, frameMs);
        ImGui::PlotLines("##tcoll", history, IM_ARRAYSIZE(history), 0, "Collision time (ms)", 0.0f, FLT_MAX, ImVec2(-FLT_MIN, 40 * uiScale));
    }
    ImGui::PopItemWidth();
    ImGui::End();
}

void mainLoop()
{
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT)
            running = false;
        else if (event.type == SDL_DROPFILE)
        {
            pendingImport = event.drop.file;
            SDL_free(event.drop.file);
        }
        else if (event.type == SDL_MULTIGESTURE && event.mgesture.numFingers == 2)
            camera.distance = clamp(camera.distance * (1 - 2 * event.mgesture.dDist), 1.0f, 100.0f);
    }
    processImport();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
    input();
    ui();

    // Step the simulation at a fixed rate of 1 / dt, at most once per frame
    Uint64 counter = SDL_GetPerformanceCounter();
    double elapsed = (counter - lastCounter) / (double)SDL_GetPerformanceFrequency();
    lastCounter = counter;
    frameMs = 0.95f * frameMs + 0.05f * (float)(elapsed * 1000);
    accumulator = min(accumulator + (float)elapsed, 0.1f);
    if (!paused && accumulator >= 0.9f * solver.dt)
    {
        accumulator = max(accumulator - solver.dt, 0.0f);
        Uint64 start = SDL_GetPerformanceCounter();
        solver.step();
        float ms = (float)((SDL_GetPerformanceCounter() - start) * 1000.0 / SDL_GetPerformanceFrequency());

        const float s = 0.1f;
        stepMs += s * (ms - stepMs);
        collisionMs += s * ((float)solver.stats.milliseconds - collisionMs);
        iterations += s * (solver.stats.iterations - iterations);
        queries += s * (solver.stats.queries - queries);
        for (int i = 0; i + 1 < IM_ARRAYSIZE(history); i++)
            history[i] = history[i + 1];
        history[IM_ARRAYSIZE(history) - 1] = (float)solver.stats.milliseconds;
    }

    int width, height, windowWidth, windowHeight;
    SDL_GL_GetDrawableSize(window, &width, &height);
    SDL_GetWindowSize(window, &windowWidth, &windowHeight);
    renderOptions.pointScale = uiScale * width / max((float)windowWidth, 1.0f);
    renderer.draw(solver, camera, width, height, renderOptions);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);

#ifdef __EMSCRIPTEN__
    // Remove the loading message of the page once the first frame is shown
    static bool firstFrame = true;
    if (firstFrame)
        EM_ASM({ var loading = document.getElementById('loading'); if (loading) loading.remove(); });
    firstFrame = false;
#endif
}

int main(int, char *[])
{
#ifdef _WIN32
    SDL_SetHint(SDL_HINT_WINDOWS_DPI_AWARENESS, "permonitorv2");
#endif
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        printf("Failed to initialize SDL: %s\n", SDL_GetError());
        return -1;
    }

    // OpenGL ES 3.0 (WebGL2) on the web, OpenGL 3.3 core on the desktop
#ifdef __EMSCRIPTEN__
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    const char *glslVersion = "#version 300 es";
#else
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    const char *glslVersion = "#version 330 core";
#endif
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    const char *title = "SDF-SDF Collisions via Caching and Importance Sampling";
    Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
    window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1600, 900, flags);
    if (!window)
    {
        // Retry without multisampling, which is not available everywhere
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);
        window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1600, 900, flags);
    }
    context = window ? SDL_GL_CreateContext(window) : nullptr;
    if (!context || !loadGL())
    {
        printf("Failed to create an OpenGL context: %s\n", SDL_GetError());
        return -1;
    }
    SDL_GL_MakeCurrent(window, context);
    SDL_GL_SetSwapInterval(1);

    // Dear ImGui, with the font rasterized at the display's pixel density
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = nullptr;
#ifdef __EMSCRIPTEN__
    if (emscripten_run_script_int("window.matchMedia('(pointer: coarse)').matches ? 1 : 0"))
        uiScale = 1.6f;
    float pixelRatio = (float)emscripten_get_device_pixel_ratio();
#else
    // Window coordinates are pixels on Windows and Linux, but points on macOS (2 pixels on Retina)
    int drawableWidth, windowWidth;
    SDL_GL_GetDrawableSize(window, &drawableWidth, nullptr);
    SDL_GetWindowSize(window, &windowWidth, nullptr);
    float pixelRatio = (float)drawableWidth / (float)max(windowWidth, 1);
    float dpi = 96.0f * pixelRatio;
    SDL_GetDisplayDPI(0, &dpi, nullptr, nullptr);
    uiScale = max(1.0f, dpi / (96.0f * pixelRatio));
#endif
    ImFontConfig font;
    font.SizePixels = 14.0f * uiScale * pixelRatio;
    io.Fonts->AddFontDefault(&font);
    io.FontGlobalScale = 1.0f / pixelRatio;
    ImGui::StyleColorsDark();
    ImGui::GetStyle().ScaleAllSizes(uiScale);
    ImGui::GetStyle().WindowRounding = 6.0f;
    ImGui::GetStyle().Colors[ImGuiCol_WindowBg].w = 0.9f;
    ImGui_ImplSDL2_InitForOpenGL(window, context);
    ImGui_ImplOpenGL3_Init(glslVersion);

    if (!renderer.init())
        return -1;
#ifdef __EMSCRIPTEN__
    int scene = emscripten_run_script_int("parseInt(new URLSearchParams(location.search).get('scene')) || 0");
    loadScene(scene >= 0 && scene < sceneCount ? scene : 0);
#else
    loadScene(0);
#endif
    lastCounter = SDL_GetPerformanceCounter();

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop(mainLoop, 0, 1);
#else
    while (running)
        mainLoop();
#endif

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
