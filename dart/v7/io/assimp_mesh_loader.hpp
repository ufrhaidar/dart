/*
 * Copyright (c) The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <dart/v7/io/mesh_loader.hpp>

namespace dart::v7 {

/// MeshLoader implementation for Assimp
/// @tparam S Scalar type
template <typename S>
class AssimpMeshLoader : public MeshLoader<S>
{
public:
  using Scalar = S;
  using Mesh = typename MeshLoader<S>::Mesh;

  AssimpMeshLoader() = default;
  ~AssimpMeshLoader() override = default;

  [[nodiscard]] std::unique_ptr<Mesh> load(
      const std::string& filepath) override;
};

} // namespace dart::v7

//==============================================================================
//
// Implementation
//
//==============================================================================

#include <dart/v7/assert.hpp>
#include <dart/v7/io/assimp_utils.hpp>
#include <dart/v7/logging.hpp>

#include <assimp/scene.h>

namespace dart::v7 {

template <typename S>
std::unique_ptr<typename AssimpMeshLoader<S>::Mesh> AssimpMeshLoader<S>::load(
    const std::string& uri)
{
  // Load the scene and return nullptr if it fails.
  std::unique_ptr<const aiScene> assimpScene = loadAssimpScene(uri);
  if (!assimpScene)
    return nullptr;

  // Return nullptr if there are no meshes.
  if (assimpScene->mNumMeshes == 0)
    return nullptr;

  // TODO(JS): Support multiple meshes.
  auto assimpMesh = assimpScene->mMeshes[0];
  DART_ASSERT(assimpMesh);

  auto mesh = std::make_unique<TriMesh<S>>();

  // Parse faces
  mesh->reserveTriangles(assimpMesh->mNumFaces);
  for (auto i = 0u; i < assimpMesh->mNumFaces; ++i) {
    const aiFace& face = assimpMesh->mFaces[i];
    // TODO(JS): Support more than triangles.
    if (face.mNumIndices != 3)
      return nullptr;
    mesh->addTriangle(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
  }

  // Parse vertices
  mesh->reserveVertices(assimpMesh->mNumVertices);
  for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
    const aiVector3D& assimpVertex = assimpMesh->mVertices[i];
    mesh->addVertex(assimpVertex.x, assimpVertex.y, assimpVertex.z);
  }

  // Parse vertex normals
  if (assimpMesh->mNormals) {
    mesh->reserveVertexNormals(assimpMesh->mNumVertices);
    for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
      const aiVector3D& assimpNormal = assimpMesh->mNormals[i];
      mesh->addVertexNormal(assimpNormal.x, assimpNormal.y, assimpNormal.z);
    }
  } else {
    // Note: Faces should be set in prior.
    mesh->computeVertexNormals();
  }

  return mesh;
}

} // namespace dart::v7
