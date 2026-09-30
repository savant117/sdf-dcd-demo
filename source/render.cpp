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

#include <SDL.h>

#include "gl.h"
#include "render.h"

// Shaders are written in the common subset of GLSL 3.30 and GLSL ES 3.00 (WebGL2)
#ifdef __EMSCRIPTEN__
static const char *shaderHeader =
    "#version 300 es\nprecision highp float;\nprecision highp int;\nprecision highp sampler2D;\nprecision highp sampler2DShadow;\n";
#else
static const char *shaderHeader = "#version 330 core\n";
#endif

static const char *meshVertexShader = R"(
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
uniform mat4 uModel;
uniform mat4 uViewProj;
uniform mat4 uLightViewProj;
out vec3 vPosition;
out vec3 vNormal;
out vec4 vLightPosition;
void main()
{
    vec4 world = uModel * vec4(aPosition, 1.0);
    vPosition = world.xyz;
    vNormal = mat3(uModel) * aNormal;
    vLightPosition = uLightViewProj * world;
    gl_Position = uViewProj * world;
}
)";

static const char *meshFragmentShader = R"(
in vec3 vPosition;
in vec3 vNormal;
in vec4 vLightPosition;
uniform vec3 uColor;
uniform vec3 uLightDirection;
uniform vec3 uEye;
uniform int uFloor;
uniform sampler2DShadow uShadowMap;
uniform sampler2D uAmbientOcclusion;
uniform vec2 uScreenSize;
out vec4 fragColor;

float shadow(vec3 n)
{
    vec3 p = vLightPosition.xyz / vLightPosition.w * 0.5 + 0.5;
    if (p.z >= 1.0 || p.x <= 0.0 || p.y <= 0.0 || p.x >= 1.0 || p.y >= 1.0)
        return 1.0;
    vec2 texel = 1.0 / vec2(textureSize(uShadowMap, 0));
    float bias = 0.0004 + 0.0012 * (1.0 - abs(dot(n, uLightDirection)));
    float sum = 0.0;
    for (int x = -1; x <= 1; x++)
        for (int y = -1; y <= 1; y++)
            sum += texture(uShadowMap, vec3(p.xy + vec2(x, y) * texel, p.z - bias));
    return sum / 9.0;
}

void main()
{
    vec3 n = normalize(gl_FrontFacing ? vNormal : -vNormal);
    vec3 albedo = uColor;
    if (uFloor == 1)
    {
        vec2 cell = floor(vPosition.xz);
        albedo = mix(vec3(0.60, 0.62, 0.66), vec3(0.80, 0.82, 0.86), mod(cell.x + cell.y, 2.0));
    }
    albedo = pow(albedo, vec3(2.2));

    vec3 v = normalize(uEye - vPosition);
    float lit = max(dot(n, uLightDirection), 0.0) * shadow(n);
    float specular = pow(max(dot(n, normalize(uLightDirection + v)), 0.0), 48.0) * 0.2 * lit;
    vec3 ambient = mix(vec3(0.20, 0.19, 0.18), vec3(0.42, 0.47, 0.58), 0.5 + 0.5 * n.y);
    ambient *= texture(uAmbientOcclusion, gl_FragCoord.xy / uScreenSize).r;
    vec3 color = pow(albedo * (ambient + vec3(1.0, 0.96, 0.9) * 1.2 * lit) + specular, vec3(1.0 / 2.2));
    if (uFloor == 1)
        color = mix(color, vec3(0.80, 0.86, 0.95), smoothstep(20.0, 80.0, length(vPosition - uEye)));
    fragColor = vec4(color, 1.0);
}
)";

static const char *depthVertexShader = R"(
layout(location = 0) in vec3 aPosition;
uniform mat4 uModel;
uniform mat4 uViewProj;
void main()
{
    gl_Position = uViewProj * uModel * vec4(aPosition, 1.0);
}
)";

static const char *depthFragmentShader = R"(
out vec4 fragColor;
void main()
{
    fragColor = vec4(1.0);
}
)";

static const char *lineVertexShader = R"(
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
uniform mat4 uViewProj;
uniform float uPointSize;
out vec3 vColor;
void main()
{
    vColor = aColor;
    gl_Position = uViewProj * vec4(aPosition, 1.0);
    gl_PointSize = uPointSize;
}
)";

static const char *lineFragmentShader = R"(
in vec3 vColor;
uniform int uRound;
out vec4 fragColor;
void main()
{
    if (uRound == 1 && length(gl_PointCoord - vec2(0.5)) > 0.5)
        discard;
    fragColor = vec4(vColor, 1.0);
}
)";

// Full screen triangle with a sky gradient along the view direction
static const char *skyVertexShader = R"(
uniform vec3 uForward;
uniform vec3 uRight;
uniform vec3 uUp;
out vec3 vDirection;
void main()
{
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2)) * 2.0 - 1.0;
    vDirection = uForward + uRight * p.x + uUp * p.y;
    gl_Position = vec4(p, 0.0, 1.0);
}
)";

static const char *skyFragmentShader = R"(
in vec3 vDirection;
out vec4 fragColor;
void main()
{
    float up = max(normalize(vDirection).y, 0.0);
    fragColor = vec4(mix(vec3(0.80, 0.86, 0.95), vec3(0.45, 0.62, 0.88), pow(up, 0.6)), 1.0);
}
)";

// View space normals (and depth) for the ambient occlusion
static const char *normalVertexShader = R"(
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uViewProj;
out vec3 vNormal;
void main()
{
    vNormal = mat3(uView) * (mat3(uModel) * aNormal);
    gl_Position = uViewProj * uModel * vec4(aPosition, 1.0);
}
)";

static const char *normalFragmentShader = R"(
in vec3 vNormal;
out vec4 fragColor;
void main()
{
    fragColor = vec4(normalize(gl_FrontFacing ? vNormal : -vNormal) * 0.5 + 0.5, 1.0);
}
)";

static const char *fullscreenVertexShader = R"(
void main()
{
    gl_Position = vec4(vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2)) * 2.0 - 1.0, 0.0, 1.0);
}
)";

// Ambient occlusion with visibility bitmasks, following Algorithm 1 of Therrien et al. 2023,
// "Screen Space Indirect Lighting with Visibility Bitmask" (arXiv:2301.11376). Instead of the two
// horizon angles of GTAO, every hemisphere slice is divided into 32 sectors (one bit each). Each
// depth sample is a slab of constant thickness that occludes the sectors between its front and
// back, so light can pass behind thin surfaces. The visibility of a slice is its fraction of
// unoccluded sectors.
static const char *aoFragmentShader = R"(
uniform sampler2D uDepth;
uniform sampler2D uNormal;
uniform vec3 uProjection; // x and y scale of the projection and the horizontal shift of its center
uniform vec2 uDepthRange; // Maps the depth buffer to view space z
uniform float uRadius;
uniform float uThickness;
out vec4 fragColor;

const int SLICES = 2;       // Slice directions per pixel N_d
const int STEPS = 8;        // Samples on each side of a slice N_s
const float SECTORS = 32.0; // Sectors per slice N_b
const float PI = 3.14159265;
const float BAYER[16] = float[16](0.0, 8.0, 2.0, 10.0, 12.0, 4.0, 14.0, 6.0, 3.0, 11.0, 1.0, 9.0, 15.0, 7.0, 13.0, 5.0);

vec3 viewPosition(vec2 uv)
{
    float z = -uDepthRange.y / (2.0 * texture(uDepth, uv).r - 1.0 + uDepthRange.x);
    vec2 ndc = uv * 2.0 - 1.0;
    return vec3((ndc.x - uProjection.z) / uProjection.x, ndc.y / uProjection.y, -1.0) * -z;
}

uint countBits(uint x)
{
    x -= (x >> 1u) & 0x55555555u;
    x = (x & 0x33333333u) + ((x >> 2u) & 0x33333333u);
    x = (x + (x >> 4u)) & 0x0F0F0F0Fu;
    return (x * 0x01010101u) >> 24u;
}

// Sectors at least half covered by the angles [a, b], with the hemisphere slice mapped to [0, 1]
// (Alg. 1, lines 18-19, with the round criterion of Sec. 3.1)
uint occludedSectors(float a, float b)
{
    uint first = uint(round(a * SECTORS)), count = uint(round(b * SECTORS)) - first;
    uint bits = count >= 32u ? 0xFFFFFFFFu : (1u << count) - 1u;
    return first >= 32u ? 0u : bits << first;
}

void main()
{
    vec2 size = vec2(textureSize(uDepth, 0));
    vec2 uv = gl_FragCoord.xy / size;
    if (texture(uDepth, uv).r >= 1.0)
    {
        fragColor = vec4(1.0); // Sky
        return;
    }
    vec3 p = viewPosition(uv);
    vec3 n = normalize(texture(uNormal, uv).xyz * 2.0 - 1.0);
    vec3 v = normalize(-p);

    // Radius of the hemisphere projected onto the image in pixels (Alg. 1, line 5), capped so
    // that samples stay close in memory when the camera is near a surface
    float radius = min(uRadius * uProjection.y / -p.z * 0.5 * size.y, 0.25 * size.y);

    // Every pixel of a 4x4 block uses different slice directions and sample offsets (Alg. 1,
    // line 7). The denoising pass averages the block.
    ivec2 cell = ivec2(gl_FragCoord.xy) & 3;
    float sliceNoise = (BAYER[cell.x + 4 * cell.y] + 0.5) / 16.0;
    float stepNoise = (BAYER[cell.y + 4 * ((cell.x + 2) & 3)] + 0.5) / 16.0;

    float visibility = 0.0;
    for (int i = 0; i < SLICES; i++)
    {
        // Slice through the view vector along the screen direction omega (Alg. 1, lines 10-11),
        // and the angle of the normal projected onto it, measured from v towards omega
        float phi = PI * (float(i) + sliceNoise) / float(SLICES);
        vec2 omega = vec2(cos(phi), sin(phi));
        vec3 direction = vec3(omega, 0.0);
        vec3 tangent = normalize(direction - dot(direction, v) * v);
        vec3 axis = normalize(cross(direction, v));
        vec3 projectedNormal = n - axis * dot(n, axis);
        float normalAngle = sign(dot(projectedNormal, tangent)) *
                            acos(clamp(dot(projectedNormal, v) / max(length(projectedNormal), 1e-5), -1.0, 1.0));

        uint occluded = 0u;
        for (int side = 0; side < 2; side++)
        {
            float s = side == 0 ? 1.0 : -1.0;
            for (int j = 0; j < STEPS; j++)
            {
                // Samples are denser near the pixel, where occluders matter the most. They are
                // snapped to pixel centers so that position and depth belong to the same pixel.
                float r = (float(j) + stepNoise) / float(STEPS);
                vec2 sampleUV = uv + s * omega * (1.0 + r * r * (radius - 1.0)) / size;
                if (sampleUV != clamp(sampleUV, 0.0, 1.0))
                    break;
                sampleUV = (floor(sampleUV * size) + 0.5) / size;

                // Front and back of the sample, a slab of thickness t behind the depth buffer, away
                // from the camera (Alg. 1, lines 14-16). Samples outside the hemisphere are ignored,
                // and so are samples (almost) in the tangent plane of p. Those cannot occlude it, but
                // after snapping they are slightly off the slice, which would darken flat surfaces.
                vec3 front = viewPosition(sampleUV) - p;
                float distanceSq = dot(front, front);
                if (distanceSq > uRadius * uRadius || distanceSq < 1e-8 || dot(front, n) < 0.035 * sqrt(distanceSq))
                    continue;
                vec3 back = front - v * uThickness;
                vec2 angles = acos(clamp(vec2(dot(front, v) * inversesqrt(distanceSq), dot(normalize(back), v)), -1.0, 1.0));

                // Signed angles in the slice relative to the projected normal, mapped to [0, 1]
                angles = clamp((s * angles - normalAngle) / PI + 0.5, 0.0, 1.0);
                occluded |= occludedSectors(min(angles.x, angles.y), max(angles.x, angles.y));
            }
        }
        visibility += 1.0 - float(countBits(occluded)) / SECTORS; // Alg. 1, line 26
    }
    fragColor = vec4(visibility / float(SLICES));
}
)";

// Depth aware 4x4 box filter that averages the 16 noise offsets of the ambient occlusion pass
static const char *denoiseFragmentShader = R"(
uniform sampler2D uAmbientOcclusion;
uniform sampler2D uDepth;
uniform vec2 uDepthRange;
out vec4 fragColor;

float viewDepth(ivec2 pixel)
{
    return uDepthRange.y / (2.0 * texelFetch(uDepth, pixel, 0).r - 1.0 + uDepthRange.x);
}

void main()
{
    ivec2 pixel = ivec2(gl_FragCoord.xy), last = textureSize(uDepth, 0) - 1;
    float depth = viewDepth(pixel), sum = 0.0, weight = 0.0;
    for (int y = -2; y < 2; y++)
    {
        for (int x = -2; x < 2; x++)
        {
            ivec2 q = clamp(pixel + ivec2(x, y), ivec2(0), last);
            float w = max(1.0 - abs(viewDepth(q) - depth) / (0.05 * depth), 0.0);
            sum += w * texelFetch(uAmbientOcclusion, q, 0).r;
            weight += w;
        }
    }
    fragColor = vec4(sum / weight);
}
)";

static const int shadowSize = 2048;
static const float3 lightDirection = normalize(float3{-0.55f, 1.0f, -0.2f}); // Towards the light

static unsigned int compileProgram(const char *vertexSource, const char *fragmentSource)
{
    auto compile = [](GLenum type, const char *source)
    {
        const char *sources[2] = {shaderHeader, source};
        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 2, sources, nullptr);
        glCompileShader(shader);
        GLint ok = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (!ok)
        {
            char log[2048];
            glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
            SDL_Log("Shader compilation failed:\n%s", log);
        }
        return shader;
    };

    GLuint program = glCreateProgram();
    GLuint vs = compile(GL_VERTEX_SHADER, vertexSource), fs = compile(GL_FRAGMENT_SHADER, fragmentSource);
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        char log[2048];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        SDL_Log("Program link failed:\n%s", log);
        return 0;
    }
    return program;
}

static void setMatrix(unsigned int program, const char *name, const float4x4 &m)
{
    glUniformMatrix4fv(glGetUniformLocation(program, name), 1, GL_FALSE, m.m);
}

static void setVector(unsigned int program, const char *name, float3 v)
{
    glUniform3f(glGetUniformLocation(program, name), v.x, v.y, v.z);
}

// Render target texture, its storage is allocated once the size is known
static GLuint createTexture(GLint filter)
{
    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    return texture;
}

static GLuint createFramebuffer(GLuint color, GLuint depth)
{
    GLuint framebuffer = 0;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color, 0);
    if (depth)
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth, 0);
    return framebuffer;
}

static float4x4 modelMatrix(const Body &body)
{
    float3x3 R = rotation(body.orientation);
    return affine(R, body.scale, body.position - R * (body.shape->centerOfMass * body.scale));
}

bool Renderer::init()
{
    meshProgram = compileProgram(meshVertexShader, meshFragmentShader);
    depthProgram = compileProgram(depthVertexShader, depthFragmentShader);
    lineProgram = compileProgram(lineVertexShader, lineFragmentShader);
    skyProgram = compileProgram(skyVertexShader, skyFragmentShader);
    normalProgram = compileProgram(normalVertexShader, normalFragmentShader);
    aoProgram = compileProgram(fullscreenVertexShader, aoFragmentShader);
    denoiseProgram = compileProgram(fullscreenVertexShader, denoiseFragmentShader);
    if (!meshProgram || !depthProgram || !lineProgram || !skyProgram || !normalProgram || !aoProgram || !denoiseProgram)
        return false;

    // Ambient occlusion targets, allocated in the first frame. A white texture replaces the
    // occlusion when it is turned off.
    normalTexture = createTexture(GL_NEAREST);
    depthTexture = createTexture(GL_NEAREST);
    aoTexture = createTexture(GL_NEAREST);
    denoiseTexture = createTexture(GL_LINEAR);
    normalFramebuffer = createFramebuffer(normalTexture, depthTexture);
    aoFramebuffer = createFramebuffer(aoTexture, 0);
    denoiseFramebuffer = createFramebuffer(denoiseTexture, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    const unsigned char white[4] = {255, 255, 255, 255};
    whiteTexture = createTexture(GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, white);

    // Depth only framebuffer for the shadow map, sampled with hardware depth comparison
    glGenTextures(1, &shadowTexture);
    glBindTexture(GL_TEXTURE_2D, shadowTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, shadowSize, shadowSize, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

    glGenFramebuffers(1, &shadowFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowTexture, 0);
    GLenum none = GL_NONE;
    glDrawBuffers(1, &none);
    glReadBuffer(GL_NONE);
    bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (!complete)
    {
        SDL_Log("Shadow framebuffer is incomplete");
        return false;
    }

    glGenVertexArrays(1, &lineVao);
    glGenBuffers(1, &lineVbo);
    glBindVertexArray(lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glGenVertexArrays(1, &emptyVao);
    glBindVertexArray(0);

#ifdef GL_PROGRAM_POINT_SIZE
    glEnable(GL_PROGRAM_POINT_SIZE);
#endif
    return true;
}

const Renderer::GpuMesh &Renderer::gpuMesh(const Shape *shape)
{
    auto it = meshes.find(shape);
    if (it != meshes.end())
        return it->second;

    // Upload interleaved positions and normals
    const Mesh &mesh = shape->mesh;
    std::vector<float> vertices;
    for (size_t i = 0; i < mesh.positions.size(); i++)
    {
        float3 p = mesh.positions[i], n = mesh.normals[i];
        vertices.insert(vertices.end(), {p.x, p.y, p.z, n.x, n.y, n.z});
    }

    GpuMesh &gpu = meshes[shape];
    gpu.count = (int)mesh.indices.size();
    glGenVertexArrays(1, &gpu.vao);
    glGenBuffers(1, &gpu.vbo);
    glGenBuffers(1, &gpu.ibo);
    glBindVertexArray(gpu.vao);
    glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(uint32_t), mesh.indices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glBindVertexArray(0);
    return gpu;
}

void Renderer::drawFloor(unsigned int program)
{
    // A thin box whose top face is the plane y = 0
    static Box floorShape({200, 0.001f, 200});
    const GpuMesh &floor = gpuMesh(&floorShape);
    setMatrix(program, "uModel", affine(identity3(), 1, {0, -0.001f, 0}));
    glBindVertexArray(floor.vao);
    glDrawElements(GL_TRIANGLES, floor.count, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Renderer::drawBodies(const Solver &solver, unsigned int program)
{
    int modelLocation = glGetUniformLocation(program, "uModel");
    int colorLocation = glGetUniformLocation(program, "uColor");
    for (auto &body : solver.bodies)
    {
        if (!body->visible)
            continue;
        const GpuMesh &mesh = gpuMesh(body->shape.get());
        glUniformMatrix4fv(modelLocation, 1, GL_FALSE, modelMatrix(*body).m);
        if (colorLocation >= 0)
            glUniform3f(colorLocation, body->color.x, body->color.y, body->color.z);
        glBindVertexArray(mesh.vao);
        glDrawElements(GL_TRIANGLES, mesh.count, GL_UNSIGNED_INT, nullptr);
    }
    glBindVertexArray(0);
}

void Renderer::drawVertices(const std::vector<float> &vertices, unsigned int mode, const float4x4 &viewProj, float pointSize)
{
    if (vertices.empty())
        return;
    glUseProgram(lineProgram);
    setMatrix(lineProgram, "uViewProj", viewProj);
    glUniform1f(glGetUniformLocation(lineProgram, "uPointSize"), pointSize);
    glUniform1i(glGetUniformLocation(lineProgram, "uRound"), mode == GL_POINTS);
    glBindVertexArray(lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STREAM_DRAW);
    glDrawArrays(mode, 0, (int)vertices.size() / 6);
    glBindVertexArray(0);
}

static void addVertex(std::vector<float> &vertices, float3 p, float3 color)
{
    vertices.insert(vertices.end(), {p.x, p.y, p.z, color.x, color.y, color.z});
}

void Renderer::ambientOcclusion(const Solver &solver, const float4x4 &view, const float4x4 &projection, float shift,
                                int width, int height, const RenderOptions &options)
{
    // Half resolution on large (high DPI) screens
    int scale = width * height > 1600000 ? 2 : 1;
    int w = max(width / scale, 1), h = max(height / scale, 1);
    if (w != aoWidth || h != aoHeight)
    {
        aoWidth = w, aoHeight = h;
        glBindTexture(GL_TEXTURE_2D, normalTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glBindTexture(GL_TEXTURE_2D, depthTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
        for (GLuint texture : {aoTexture, denoiseTexture})
        {
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, w, h, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);
        }
    }
    glViewport(0, 0, w, h);

    // View space normals and depth of the floor and the bodies
    glBindFramebuffer(GL_FRAMEBUFFER, normalFramebuffer);
    glClearColor(0.5f, 0.5f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glUseProgram(normalProgram);
    setMatrix(normalProgram, "uView", view);
    setMatrix(normalProgram, "uViewProj", projection * view);
    drawFloor(normalProgram);
    drawBodies(solver, normalProgram);
    glDisable(GL_DEPTH_TEST);

    // Visibility bitmask occlusion (Alg. 1 of Therrien et al.)
    glBindVertexArray(emptyVao);
    glBindFramebuffer(GL_FRAMEBUFFER, aoFramebuffer);
    glUseProgram(aoProgram);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, normalTexture);
    glUniform1i(glGetUniformLocation(aoProgram, "uDepth"), 0);
    glUniform1i(glGetUniformLocation(aoProgram, "uNormal"), 1);
    setVector(aoProgram, "uProjection", {projection.m[0], projection.m[5], shift});
    glUniform2f(glGetUniformLocation(aoProgram, "uDepthRange"), projection.m[10], projection.m[14]);
    glUniform1f(glGetUniformLocation(aoProgram, "uRadius"), options.aoRadius);
    glUniform1f(glGetUniformLocation(aoProgram, "uThickness"), options.aoThickness);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    // Denoising
    glBindFramebuffer(GL_FRAMEBUFFER, denoiseFramebuffer);
    glUseProgram(denoiseProgram);
    glBindTexture(GL_TEXTURE_2D, aoTexture);
    glUniform1i(glGetUniformLocation(denoiseProgram, "uDepth"), 0);
    glUniform1i(glGetUniformLocation(denoiseProgram, "uAmbientOcclusion"), 1);
    glUniform2f(glGetUniformLocation(denoiseProgram, "uDepthRange"), projection.m[10], projection.m[14]);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    glActiveTexture(GL_TEXTURE0);
}

void Renderer::draw(const Solver &solver, const Camera &camera, int width, int height, const RenderOptions &options)
{
    float3 eye = camera.eye();
    float4x4 view = lookAt(eye, camera.target, {0, 1, 0});
    float4x4 projection = camera.projection((float)width / height);
    float4x4 viewProj = projection * view;

    // Fit the shadow map to the visible bodies
    AABB box = emptyAABB();
    for (auto &body : solver.bodies)
        if (body->visible)
            grow(box, body->bounds.min), grow(box, body->bounds.max);
    float3 c = box.min.x <= box.max.x ? center(box) : float3{0, 0, 0};
    float radius = box.min.x <= box.max.x ? 0.5f * length(size(box)) + 1.0f : 10.0f;
    float4x4 lightViewProj = orthographic(-radius, radius, -radius, radius, 0.0f, 4 * radius) *
                             lookAt(c + lightDirection * (2 * radius), c, {0, 1, 0});

    // Shadow pass, rendering to the current framebuffer is resumed afterwards
    GLint target = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &target);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer);
    glViewport(0, 0, shadowSize, shadowSize);
    glDepthMask(GL_TRUE);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.0f, 4.0f);
    glUseProgram(depthProgram);
    setMatrix(depthProgram, "uViewProj", lightViewProj);
    drawBodies(solver, depthProgram);
    glDisable(GL_POLYGON_OFFSET_FILL);
    if (options.ambientOcclusion)
        ambientOcclusion(solver, view, projection, camera.shift, width, height, options);
    glBindFramebuffer(GL_FRAMEBUFFER, target);

    // Sky
    glViewport(0, 0, width, height);
    glClearColor(0.8f, 0.86f, 0.95f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    float3 forward = normalize(camera.target - eye);
    float3 right = normalize(cross(forward, {0, 1, 0}));
    float3 up = cross(right, forward);
    float t = camera.tanHalfFov((float)width / height);
    glDisable(GL_DEPTH_TEST);
    glUseProgram(skyProgram);
    setVector(skyProgram, "uForward", forward);
    setVector(skyProgram, "uRight", right * (t * width / height));
    setVector(skyProgram, "uUp", up * t);
    glBindVertexArray(emptyVao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glEnable(GL_DEPTH_TEST);

    // Floor and bodies
    glUseProgram(meshProgram);
    setMatrix(meshProgram, "uViewProj", viewProj);
    setMatrix(meshProgram, "uLightViewProj", lightViewProj);
    setVector(meshProgram, "uLightDirection", lightDirection);
    setVector(meshProgram, "uEye", eye);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, options.ambientOcclusion ? denoiseTexture : whiteTexture);
    glUniform1i(glGetUniformLocation(meshProgram, "uAmbientOcclusion"), 1);
    glUniform2f(glGetUniformLocation(meshProgram, "uScreenSize"), (float)width, (float)height);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowTexture);
    glUniform1i(glGetUniformLocation(meshProgram, "uShadowMap"), 0);

    glUniform1i(glGetUniformLocation(meshProgram, "uFloor"), 1);
    drawFloor(meshProgram);
    glUniform1i(glGetUniformLocation(meshProgram, "uFloor"), 0);
    drawBodies(solver, meshProgram);

    // Debug visualization
    float pointSize = 7.0f * options.pointScale;
    lineVertices.clear(), pointVertices.clear(), seedVertices.clear();
    for (auto &entry : solver.manifolds)
    {
        const Manifold &m = entry.second;
        if (options.volumes)
        {
            float3 a = m.volume.min, b = m.volume.max, color = {0.2f, 0.45f, 0.9f};
            for (int i = 0; i < 12; i++)
            {
                // The 12 edges of the box: 4 along each axis
                int axis = i / 4, u = (axis + 1) % 3, v = (axis + 2) % 3;
                float3 p = a, q = a;
                p[u] = q[u] = (i & 1) ? b[u] : a[u];
                p[v] = q[v] = (i & 2) ? b[v] : a[v];
                q[axis] = b[axis];
                addVertex(lineVertices, p, color);
                addVertex(lineVertices, q, color);
            }
        }
        for (const ContactPoint &p : m.points)
        {
            if (!p.active || !(p.cached ? options.cachedContacts : options.randomContacts))
                continue;
            // Green for points warm started from the cache, orange for new stochastic samples
            float3 color = p.cached ? float3{0.1f, 0.8f, 0.2f} : float3{1.0f, 0.55f, 0.0f};
            addVertex(pointVertices, p.x, color);
            addVertex(lineVertices, p.x, {0.9f, 0.1f, 0.1f});
            addVertex(lineVertices, p.x + p.normal * 0.3f, {0.9f, 0.1f, 0.1f});
        }
    }
    if (options.seeds)
    {
        for (auto &body : solver.bodies)
        {
            if (!body->visible)
                continue;
            for (float3 seed : body->shape->seeds)
            {
                // Nudged towards the camera so that seeds on the surface are not hidden by it
                float3 p = body->toWorld(seed);
                addVertex(seedVertices, p + normalize(eye - p) * 0.01f, {0.85f, 0.1f, 0.1f});
            }
        }
    }

    drawVertices(seedVertices, GL_POINTS, viewProj, 0.6f * pointSize);
    glDisable(GL_DEPTH_TEST);
    drawVertices(lineVertices, GL_LINES, viewProj, 1.0f);
    drawVertices(pointVertices, GL_POINTS, viewProj, pointSize);
    glEnable(GL_DEPTH_TEST);
}
