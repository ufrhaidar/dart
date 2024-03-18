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

#include "dart/v7/io/assimp_utils.hpp"

#include "dart/v7/assert.hpp"
#include "dart/v7/io/assimp_mesh_loader.hpp"
#include "dart/v7/logging.hpp"
#include "dart/v7/resource_loader.hpp"

#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#include <assimp/Importer.hpp>
#include <assimp/cfileio.h>
#include <assimp/cimport.h>
#include <assimp/mesh.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <filesystem>

namespace dart::v7 {

namespace {

inline Assimp::IOSystem* getIOSystem(aiFileIO* _io)
{
  return reinterpret_cast<Assimp::IOSystem*>(_io->UserData);
}

inline Assimp::IOStream* getIOStream(aiFile* _file)
{
  return reinterpret_cast<Assimp::IOStream*>(_file->UserData);
}

inline void fileFlushProc(aiFile* _file)
{
  getIOStream(_file)->Flush();
}

inline std::size_t fileReadProc(
    aiFile* _file, char* _buffer, std::size_t _size, std::size_t _count)
{
  return getIOStream(_file)->Read(_buffer, _size, _count);
}

inline aiReturn fileSeekProc(
    aiFile* _file, std::size_t _offset, aiOrigin _origin)
{
  return getIOStream(_file)->Seek(_offset, _origin);
}

inline std::size_t fileSizeProc(aiFile* _file)
{
  return getIOStream(_file)->FileSize();
}

inline std::size_t fileTellProc(aiFile* _file)
{
  return getIOStream(_file)->Tell();
}

inline std::size_t fileWriteProc(
    aiFile* _file, const char* _buffer, std::size_t _size, std::size_t _count)
{
  return getIOStream(_file)->Write(_buffer, _size, _count);
}

inline aiFile* fileOpenProc(aiFileIO* _io, const char* _path, const char* _mode)
{
  Assimp::IOStream* stream = getIOSystem(_io)->Open(_path, _mode);
  if (!stream)
    return nullptr;

  aiFile* out = new aiFile;
  out->FileSizeProc = &fileSizeProc;
  out->FlushProc = &fileFlushProc;
  out->ReadProc = &fileReadProc;
  out->SeekProc = &fileSeekProc;
  out->TellProc = &fileTellProc;
  out->WriteProc = &fileWriteProc;
  out->UserData = reinterpret_cast<char*>(stream);
  return out;
}

inline void fileCloseProc(aiFileIO* _io, aiFile* _file)
{
  getIOSystem(_io)->Close(getIOStream(_file));
  delete _file;
}

inline aiFileIO createFileIO(Assimp::IOSystem* _system)
{
  aiFileIO out;
  out.OpenProc = &fileOpenProc;
  out.CloseProc = &fileCloseProc;
  out.UserData = reinterpret_cast<char*>(_system);
  return out;
}

} // namespace

std::unique_ptr<const aiScene> loadAssimpScene(const std::string& uri)
{
  // Remove points and lines from the import.
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(
      propertyStore,
      AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT | aiPrimitiveType_LINE);

  // Wrap ResourceRetriever in an IOSystem from Assimp's C++ API.  Then wrap
  // the IOSystem in an aiFileIO from Assimp's C API. Yes, this API is
  // completely ridiculous...
  AssimpInputResourceRetrieverAdaptor systemIO();
  aiFileIO fileIO = createFileIO(&systemIO);

  // Import the file.
  const aiScene* scene = aiImportFileExWithProperties(
      uri.c_str(),
      aiProcess_GenNormals | aiProcess_Triangulate
          | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
          | aiProcess_OptimizeMeshes,
      &fileIO,
      propertyStore);

  // If succeeded, store the importer in the scene to keep it alive. This is
  // necessary because the importer owns the memory that it allocates.
  if (!scene) {
    DART_WARN("[MeshShape::loadMesh] Failed loading mesh '{}'.", uri);
    aiReleasePropertyStore(propertyStore);
    return nullptr;
  }

  // Assimp rotates Collada files such that the up-axis (specified in the
  // Collada file) aligns with Assimp's y-axis. Here we are reverting this
  // rotation. We are only catching files with the .dae file ending here. We
  // might miss files with an .xml file ending, which would need to be looked
  // into to figure out whether they are Collada files.
  std::string extension;
  const std::size_t extensionIndex = uri.find_last_of('.');
  if (extensionIndex != std::string::npos)
    extension = uri.substr(extensionIndex);

  std::transform(
      std::begin(extension),
      std::end(extension),
      std::begin(extension),
      ::tolower);

  if (extension == ".dae" || extension == ".zae")
    scene->mRootNode->mTransformation = aiMatrix4x4();

  // Finally, pre-transform the vertices. We can't do this as part of the
  // import process, because we may have changed mTransformation above.
  scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);
  if (!scene)
    DART_WARN("[MeshShape::loadMesh] Failed pre-transforming vertices.");

  aiReleasePropertyStore(propertyStore);

  return std::unique_ptr<const aiScene>(scene);
};

AssimpInputResourceRetrieverAdaptor::AssimpInputResourceRetrieverAdaptor(
    ResourceLoaderPtr loader)
{
  // do nothing
}

AssimpInputResourceRetrieverAdaptor::~AssimpInputResourceRetrieverAdaptor()
{
  // do nothing
}

bool AssimpInputResourceRetrieverAdaptor::Exists(const char* pFile) const
{
  return std::filesystem::exists(pFile);
}

char AssimpInputResourceRetrieverAdaptor::getOsSeparator() const
{
  // URIs always use forward slash as a delimeter.
  return '/';
}

Assimp::IOStream* AssimpInputResourceRetrieverAdaptor::Open(
    const char* pFile, const char* pMode)
{
  // TODO: How do we support text mode?
  if (pMode != std::string("r") && pMode != std::string("rb")
      && pMode != std::string("rt")) {
    DART_WARN(
        "[AssimpInputResourceRetrieverAdaptor::Open] Unsupported mode '{}'. "
        "Only 'r', 'rb', and 'rt' are supported.",
        pMode);
    return nullptr;
  }

  if (const ResourcePtr resource = mResourceRetriever->retrieve(pFile))
    return new AssimpInputResourceAdaptor(resource);
  else
    return nullptr;
}

void AssimpInputResourceRetrieverAdaptor::Close(Assimp::IOStream* pFile)
{
  if (pFile)
    delete pFile;
}

AssimpInputResourceAdaptor::AssimpInputResourceAdaptor(
    ResourcePtr _resource)
  : mResource(_resource)
{
  assert(_resource);
}

AssimpInputResourceAdaptor::~AssimpInputResourceAdaptor()
{
  // do nothing
}

std::size_t AssimpInputResourceAdaptor::Read(
    void* pvBuffer, std::size_t psize, std::size_t pCount)
{
  return mResource->read(pvBuffer, psize, pCount);
}

std::size_t AssimpInputResourceAdaptor::Write(
    const void* /*pvBuffer*/, std::size_t /*pSize*/, std::size_t /*pCount*/)
{
  DART_WARN(
      "[AssimpInputResourceAdaptor::Write] Write is not implemented. This is a "
      "read-only stream.");
  return 0;
}

aiReturn AssimpInputResourceAdaptor::Seek(std::size_t pOffset, aiOrigin pOrigin)
{
  Resource::SeekType origin;
  switch (pOrigin) {
    case aiOrigin_CUR:
      origin = Resource::SeekType::CUR;
      break;

    case aiOrigin_END:
      origin = Resource::SeekType::END;
      break;

    case aiOrigin_SET:
      origin = Resource::SeekType::SET;
      break;

    default:
      dtwarn << "[AssimpInputResourceAdaptor::Seek] Invalid origin. Expected"
                " aiOrigin_CUR, aiOrigin_END, or aiOrigin_SET.\n";
      return aiReturn_FAILURE;
  }

  if (mResource->seek(pOffset, origin))
    return aiReturn_SUCCESS;
  else
    return aiReturn_FAILURE;
}

std::size_t AssimpInputResourceAdaptor::Tell() const
{
  return mResource->tell();
}

std::size_t AssimpInputResourceAdaptor::FileSize() const
{
  return mResource->getSize();
}

void AssimpInputResourceAdaptor::Flush()
{
  // dtwarn << "[AssimpInputResourceAdaptor::Flush] Flush is not implemented."
  //           " This is a read-only stream.\n";
}

} // namespace dart::v7
