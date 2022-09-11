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

#pragma once

#include <string>
#include <vector>
#include <map>
#include <glm/glm.hpp>
#include "rr_bvh.h"
#include "rr_mesh.h"
#include "rr_bvh_translator.h"

namespace rrbvh
{
    struct Indices
    {
        int x, y, z;
    };

    class Scene
    {
    public:
        Scene() : initialized(false), dirty(true)
        {
            sceneBvh = new Bvh(10.0f, 64, false);
        }
        ~Scene();

        int AddMesh(const std::string& filename);
        int AddMeshInstance(const MeshInstance& meshInstance);

        void ProcessScene();
        void RebuildInstances();

        // Meshes
        std::vector<RMesh*> meshes;
        std::vector<std::tuple<std::string, glm::mat4, int>> obj_meshes;

        // Scene Mesh Data 
        std::vector<Indices> vertIndices;
        std::vector<Vec4> verticesUVX; // Vertex + texture Coord (u/s)
        std::vector<Vec4> normalsUVY; // Normal + texture Coord (v/t)
        std::vector<Mat4> transforms;

        // Instances
        std::vector<MeshInstance> meshInstances;

        // Bvh
        BvhTranslator bvhTranslator; // Produces a flat bvh array for GPU consumption
        bbox sceneBounds;

        bool initialized;
        bool dirty;
        // To check if scene elements need to be resent to GPU
        bool instancesModified = false;
        bool envMapModified = false;

    private:
        Bvh* sceneBvh = nullptr;
        void createBLAS();
        void createTLAS();
    };
}
