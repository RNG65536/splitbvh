/*
 * MIT License
 *
 * Copyright(c) 2019 Asif Ali
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <iostream>
#include <vector>
#include "rr_scene.h"

namespace rrbvh
{
    Scene::~Scene()
    {
        for (int i = 0; i < meshes.size(); i++)
            delete meshes[i];
        meshes.clear();

        if (sceneBvh)
            delete sceneBvh;
    };

    int Scene::AddMesh(const std::string& filename)
    {
        int id = -1;
        // Check if mesh was already loaded
        for (int i = 0; i < meshes.size(); i++)
            if (meshes[i]->name == filename)
                return i;

        id = meshes.size();
        RMesh* mesh = new RMesh;

        printf("Loading model %s\n", filename.c_str());
        if (mesh->LoadFromFile(filename))
            meshes.push_back(mesh);
        else
        {
            printf("Unable to load model %s\n", filename.c_str());
            delete mesh;
            id = -1;
        }
        return id;
    }

    int Scene::AddMeshInstance(const MeshInstance& meshInstance)
    {
        int id = meshInstances.size();
        meshInstances.push_back(meshInstance);
        return id;
    }

    void Scene::createTLAS()
    {
        // Loop through all the mesh Instances and build a Top Level BVH
        std::vector<bbox> bounds;
        bounds.resize(meshInstances.size());

        for (int i = 0; i < meshInstances.size(); i++)
        {
            bbox b_box = meshes[meshInstances[i].meshID]->bvh->Bounds();
            Mat4 matrix = meshInstances[i].transform;

            Vec3 minBound = b_box.pmin;
            Vec3 maxBound = b_box.pmax;

            Vec3 right       = Vec3(matrix[0][0], matrix[0][1], matrix[0][2]);
            Vec3 up          = Vec3(matrix[1][0], matrix[1][1], matrix[1][2]);
            Vec3 forward     = Vec3(matrix[2][0], matrix[2][1], matrix[2][2]);
            Vec3 translation = Vec3(matrix[3][0], matrix[3][1], matrix[3][2]);

            Vec3 xa = right * minBound.x;
            Vec3 xb = right * maxBound.x;

            Vec3 ya = up * minBound.y;
            Vec3 yb = up * maxBound.y;

            Vec3 za = forward * minBound.z;
            Vec3 zb = forward * maxBound.z;

            minBound = min(xa, xb) + min(ya, yb) + min(za, zb) + translation;
            maxBound = max(xa, xb) + max(ya, yb) + max(za, zb) + translation;

            bbox bound;
            bound.pmin = minBound;
            bound.pmax = maxBound;

            bounds[i] = bound;
        }
        sceneBvh->Build(&bounds[0], bounds.size());
        sceneBounds = sceneBvh->Bounds();
    }

    void Scene::createBLAS()
    {
        // Loop through all meshes and build BVHs
#pragma omp parallel for
        for (int i = 0; i < meshes.size(); i++)
        {
            printf("Building BVH for %s\n", meshes[i]->name.c_str());
            meshes[i]->BuildBVH();
        }
    }

    void Scene::RebuildInstances()
    {
        delete sceneBvh;
        sceneBvh = new Bvh(10.0f, 64, false);

        createTLAS();
        bvhTranslator.UpdateTLAS(sceneBvh, meshInstances);

        //Copy transforms
        for (int i = 0; i < meshInstances.size(); i++)
            transforms[i] = meshInstances[i].transform;

        instancesModified = true;
        dirty = true;
    }

    void Scene::ProcessScene()
    {
        if (initialized)
        {
            printf("Scene already processed\n");
            return;
        }

        printf("Processing scene data\n");
        createBLAS();

        printf("Building scene BVH\n");
        createTLAS();

        // Flatten BVH
        printf("Flattening BVH\n");
        bvhTranslator.Process(sceneBvh, meshes, meshInstances);

        // Copy mesh data
        int verticesCnt = 0;
        printf("Copying Mesh Data\n");
        for (int i = 0; i < meshes.size(); i++)
        {
            // Copy indices from BVH and not from Mesh. 
            // Required if splitBVH is used as a triangle can be shared by leaf nodes
            int numIndices = meshes[i]->bvh->GetNumIndices();
            const int* triIndices = meshes[i]->bvh->GetIndices();

            for (int j = 0; j < numIndices; j++)
            {
                int index = triIndices[j];
                int v1 = (index * 3 + 0) + verticesCnt;
                int v2 = (index * 3 + 1) + verticesCnt;
                int v3 = (index * 3 + 2) + verticesCnt;

                vertIndices.push_back(Indices{ v1, v2, v3 });
            }

            verticesUVX.insert(verticesUVX.end(), meshes[i]->verticesUVX.begin(), meshes[i]->verticesUVX.end());
            normalsUVY.insert(normalsUVY.end(), meshes[i]->normalsUVY.begin(), meshes[i]->normalsUVY.end());

            verticesCnt += meshes[i]->verticesUVX.size();
        }

        // Copy transforms
        printf("Copying transforms\n");
        transforms.resize(meshInstances.size());
        for (int i = 0; i < meshInstances.size(); i++)
            transforms[i] = meshInstances[i].transform;

        initialized = true;
    }
}