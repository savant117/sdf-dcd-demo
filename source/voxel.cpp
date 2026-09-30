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
#include <cctype>
#include <map>
#include <numeric>

#include "shapes.h"

namespace
{

// Closest point on triangle abc to p [Eri04]
float3 closestPointOnTriangle(float3 p, float3 a, float3 b, float3 c)
{
    float3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0)
        return a;

    float3 bp = p - b;
    float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3)
        return b;

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0)
        return a + ab * (d1 / (d1 - d3));

    float3 cp = p - c;
    float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6)
        return c;

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0)
        return a + ac * (d2 / (d2 - d6));

    float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

    float denom = 1.0f / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

float distanceSq(const AABB &box, float3 p)
{
    return lengthSq(p - clamp(p, box.min, box.max));
}

// Bounding volume hierarchy over the triangles, used for closest point queries while baking
struct TriangleBvh
{
    struct Node
    {
        AABB box;
        int left = -1, right = -1, first = 0, count = 0;
    };

    const Mesh &mesh;
    std::vector<Node> nodes;
    std::vector<int> order;
    std::vector<float3> centroids;

    TriangleBvh(const Mesh &mesh) : mesh(mesh)
    {
        order.resize(mesh.triangleCount());
        std::iota(order.begin(), order.end(), 0);
        for (int t = 0; t < mesh.triangleCount(); t++)
            centroids.push_back((mesh.vertex(t, 0) + mesh.vertex(t, 1) + mesh.vertex(t, 2)) / 3.0f);
        build(0, (int)order.size());
    }

    int build(int first, int count)
    {
        int index = (int)nodes.size();
        nodes.push_back({});
        AABB box = emptyAABB(), centers = emptyAABB();
        for (int i = first; i < first + count; i++)
        {
            for (int c = 0; c < 3; c++)
                grow(box, mesh.vertex(order[i], c));
            grow(centers, centroids[order[i]]);
        }
        nodes[index].box = box;
        if (count <= 4)
        {
            nodes[index].first = first;
            nodes[index].count = count;
            return index;
        }

        // Median split along the longest axis of the centroids
        float3 extent = size(centers);
        int axis = extent.x > extent.y ? (extent.x > extent.z ? 0 : 2) : (extent.y > extent.z ? 1 : 2);
        int mid = first + count / 2;
        std::nth_element(order.begin() + first, order.begin() + mid, order.begin() + first + count,
                         [&](int a, int b) { return centroids[a][axis] < centroids[b][axis]; });
        int left = build(first, mid - first);
        int right = build(mid, first + count - mid);
        nodes[index].left = left;
        nodes[index].right = right;
        return index;
    }

    // Finds the closest point on the mesh to p, only considering points closer than sqrt(bestSq)
    void closest(float3 p, float &bestSq, float3 &point, int &triangle) const
    {
        int stack[64], top = 0;
        stack[top++] = 0;
        while (top > 0)
        {
            const Node &node = nodes[stack[--top]];
            if (distanceSq(node.box, p) >= bestSq)
                continue;

            if (node.count > 0)
            {
                for (int i = node.first; i < node.first + node.count; i++)
                {
                    int t = order[i];
                    float3 q = closestPointOnTriangle(p, mesh.vertex(t, 0), mesh.vertex(t, 1), mesh.vertex(t, 2));
                    float d = lengthSq(q - p);
                    if (d < bestSq)
                        bestSq = d, point = q, triangle = t;
                }
                continue;
            }

            // Visit the nearer child first
            bool leftFirst = distanceSq(nodes[node.left].box, p) < distanceSq(nodes[node.right].box, p);
            stack[top++] = leftFirst ? node.right : node.left;
            stack[top++] = leftFirst ? node.left : node.right;
        }
    }
};

bool isWatertight(const Mesh &mesh)
{
    // Every edge of a closed manifold mesh is shared by exactly two triangles
    std::map<std::pair<uint32_t, uint32_t>, int> edges;
    for (int t = 0; t < mesh.triangleCount(); t++)
    {
        for (int c = 0; c < 3; c++)
        {
            uint32_t a = mesh.indices[t * 3 + c], b = mesh.indices[t * 3 + (c + 1) % 3];
            edges[{std::min(a, b), std::max(a, b)}]++;
        }
    }
    for (auto &edge : edges)
        if (edge.second != 2)
            return false;
    return true;
}

} // namespace

VoxelSdf::VoxelSdf(const Mesh &input, const std::string &name, int resolution)
{
    this->name = name;

    // Normalize the mesh to fit in a unit cube centered at the origin
    mesh = input;
    AABB box = mesh.bounds();
    float scale = 1.0f / maxComponent(size(box));
    for (float3 &p : mesh.positions)
        p = (p - center(box)) * scale;
    bounds = mesh.bounds();
    watertight = isWatertight(mesh);

    // The triangles must face outwards, flip them if the enclosed (signed) volume is negative
    float signedVolume = 0;
    for (int t = 0; t < mesh.triangleCount(); t++)
        signedVolume += dot(mesh.vertex(t, 0), cross(mesh.vertex(t, 1), mesh.vertex(t, 2))) / 6;
    if (signedVolume < 0)
        for (int t = 0; t < mesh.triangleCount(); t++)
            std::swap(mesh.indices[t * 3 + 1], mesh.indices[t * 3 + 2]);

    // Grid covering the bounds plus a few cells of padding
    const int padding = 3;
    cell = 1.0f / resolution;
    origin = bounds.min - float3{1, 1, 1} * (padding * cell);
    int n[3];
    for (int i = 0; i < 3; i++)
        n[i] = (int)ceilf(size(bounds)[i] / cell) + 1 + 2 * padding;
    nx = n[0], ny = n[1], nz = n[2];
    auto index = [&](int x, int y, int z) { return x + nx * (y + ny * z); };

    // Inside/outside classification by counting the signed surface crossings (+1 entering, -1
    // leaving) along grid lines in x, y and z, i.e. the winding number, which also handles meshes
    // made of overlapping parts. The majority vote of the three directions is robust to small holes.
    std::vector<uint8_t> insideVotes(nx * ny * nz, 0);
    for (int a = 0; a < 3; a++)
    {
        int b = (a + 1) % 3, c = (a + 2) % 3;
        float jitterB = 0.1234567f * cell, jitterC = 0.3456789f * cell; // Keeps the lines off vertices and edges
        std::vector<std::vector<std::pair<float, int>>> crossings(n[b] * n[c]);
        for (int t = 0; t < mesh.triangleCount(); t++)
        {
            float3 v0 = mesh.vertex(t, 0), v1 = mesh.vertex(t, 1), v2 = mesh.vertex(t, 2);
            float det = (v1[b] - v0[b]) * (v2[c] - v0[c]) - (v2[b] - v0[b]) * (v1[c] - v0[c]);
            if (fabsf(det) < 1e-12f)
                continue;

            float loB = min(v0[b], min(v1[b], v2[b])), hiB = max(v0[b], max(v1[b], v2[b]));
            float loC = min(v0[c], min(v1[c], v2[c])), hiC = max(v0[c], max(v1[c], v2[c]));
            int j0 = std::max(0, (int)ceilf((loB - origin[b] - jitterB) / cell));
            int j1 = std::min(n[b] - 1, (int)floorf((hiB - origin[b] - jitterB) / cell));
            int k0 = std::max(0, (int)ceilf((loC - origin[c] - jitterC) / cell));
            int k1 = std::min(n[c] - 1, (int)floorf((hiC - origin[c] - jitterC) / cell));
            for (int k = k0; k <= k1; k++)
            {
                for (int j = j0; j <= j1; j++)
                {
                    // Barycentric coordinates of the grid line in the projected triangle
                    float pb = origin[b] + j * cell + jitterB - v0[b], pc = origin[c] + k * cell + jitterC - v0[c];
                    float w1 = (pb * (v2[c] - v0[c]) - (v2[b] - v0[b]) * pc) / det;
                    float w2 = ((v1[b] - v0[b]) * pc - pb * (v1[c] - v0[c])) / det;
                    // The sign of det is the sign of the normal along the line, so det < 0 enters the mesh
                    if (w1 >= 0 && w2 >= 0 && w1 + w2 <= 1)
                        crossings[j + k * n[b]].push_back({v0[a] + w1 * (v1[a] - v0[a]) + w2 * (v2[a] - v0[a]), det < 0 ? 1 : -1});
                }
            }
        }

        for (int k = 0; k < n[c]; k++)
        {
            for (int j = 0; j < n[b]; j++)
            {
                auto &line = crossings[j + k * n[b]];
                std::sort(line.begin(), line.end());
                size_t passed = 0;
                int winding = 0;
                for (int i = 0; i < n[a]; i++)
                {
                    float position = origin[a] + i * cell;
                    while (passed < line.size() && line[passed].first < position)
                        winding += line[passed++].second;
                    int node[3];
                    node[a] = i, node[b] = j, node[c] = k;
                    if (winding > 0)
                        insideVotes[index(node[0], node[1], node[2])]++;
                }
            }
        }
    }

    // Exact distance and gradient from the closest point on the mesh. The closest triangle of the
    // previous node along x is usually still the closest, and bounds the search radius of the query.
    TriangleBvh bvh(mesh);
    grid.resize(nx * ny * nz);
    for (int z = 0; z < nz; z++)
    {
        for (int y = 0; y < ny; y++)
        {
            int triangle = -1;
            for (int x = 0; x < nx; x++)
            {
                float3 p = origin + float3{(float)x, (float)y, (float)z} * cell;
                float bestSq = INFINITY;
                float3 q = p;
                if (triangle >= 0)
                {
                    q = closestPointOnTriangle(p, mesh.vertex(triangle, 0), mesh.vertex(triangle, 1), mesh.vertex(triangle, 2));
                    bestSq = lengthSq(q - p) * 1.0001f + 1e-12f;
                }
                bvh.closest(p, bestSq, q, triangle);

                float d = length(p - q);
                bool inside = insideVotes[index(x, y, z)] >= 2;
                float3 g = d > 1e-6f ? (p - q) / d : normalize(cross(mesh.vertex(triangle, 1) - mesh.vertex(triangle, 0),
                                                                     mesh.vertex(triangle, 2) - mesh.vertex(triangle, 0)));
                if (inside && d > 1e-6f)
                    g = -g;
                grid[index(x, y, z)] = {inside ? -d : d, g.x, g.y, g.z};
            }
        }
    }

    // Render normals
    mesh.computeNormals();
}

float VoxelSdf::sdf(float3 p, float3 *grad) const
{
    // Trilinear interpolation of the grid, points outside it add their distance to the grid
    float3 hi = origin + float3{(float)(nx - 1), (float)(ny - 1), (float)(nz - 1)} * cell;
    float3 q = clamp(p, origin, hi);
    float3 f = (q - origin) / cell;
    int x = std::min((int)f.x, nx - 2), y = std::min((int)f.y, ny - 2), z = std::min((int)f.z, nz - 2);
    float tx = f.x - x, ty = f.y - y, tz = f.z - z;

    const float4 *c = &grid[x + nx * (y + ny * z)];
    int sy = nx, sz = nx * ny;
    float w[8] = {(1 - tx) * (1 - ty) * (1 - tz), tx * (1 - ty) * (1 - tz), (1 - tx) * ty * (1 - tz), tx * ty * (1 - tz),
                  (1 - tx) * (1 - ty) * tz, tx * (1 - ty) * tz, (1 - tx) * ty * tz, tx * ty * tz};
    const float4 *corner[8] = {c, c + 1, c + sy, c + sy + 1, c + sz, c + sz + 1, c + sy + sz, c + sy + sz + 1};

    float d = 0;
    for (int i = 0; i < 8; i++)
        d += w[i] * corner[i]->x;

    float3 outside = p - q;
    float outsideLength = length(outside);
    d += outsideLength;

    if (grad)
    {
        float3 g = {0, 0, 0};
        for (int i = 0; i < 8; i++)
            g += float3{corner[i]->y, corner[i]->z, corner[i]->w} * w[i];
        *grad = outsideLength > 0 ? normalize(g + outside / outsideLength) : g;
    }
    return d;
}

void VoxelSdf::surfaceCandidates(std::vector<float3> &points, int count) const
{
    // Area weighted random points on the triangle mesh
    std::vector<float> cumulative;
    float total = 0;
    for (int t = 0; t < mesh.triangleCount(); t++)
    {
        total += 0.5f * length(cross(mesh.vertex(t, 1) - mesh.vertex(t, 0), mesh.vertex(t, 2) - mesh.vertex(t, 0)));
        cumulative.push_back(total);
    }

    Random rng;
    for (int i = 0; i < count; i++)
    {
        int t = (int)(std::lower_bound(cumulative.begin(), cumulative.end(), rng.uniform() * total) - cumulative.begin());
        t = std::min(t, mesh.triangleCount() - 1);
        float r1 = sqrtf(rng.uniform()), r2 = rng.uniform();
        points.push_back(mesh.vertex(t, 0) * (1 - r1) + mesh.vertex(t, 1) * (r1 * (1 - r2)) + mesh.vertex(t, 2) * (r1 * r2));
    }
}

std::shared_ptr<VoxelSdf> loadVoxelSdf(const std::string &path, std::string &error, int resolution)
{
    Mesh mesh;
    if (!loadObj(path, mesh, error))
        return nullptr;

    // Name the shape after the file, e.g. "assets/bunny.obj" -> "Bunny"
    std::string name = path.substr(path.find_last_of("/\\") + 1);
    name = name.substr(0, name.find_last_of('.'));
    if (!name.empty())
        name[0] = (char)toupper(name[0]);

    auto shape = std::make_shared<VoxelSdf>(mesh, name, resolution);
    shape->finalize();
    return shape;
}
