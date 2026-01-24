/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/lock_helper.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CAssimpModel.h>
#include <mrpt/viz/CSetOfTexturedTriangles.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/config.h>

#include <iostream>

#if MRPT_HAS_ASSIMP
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <assimp/Importer.hpp>
#endif

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt::poses;

IMPLEMENTS_SERIALIZABLE(CAssimpModel, CSetOfObjects, mrpt::viz)

// ============================================================================
// Pimpl wrapper for Assimp scene
// ============================================================================

struct CAssimpModel::AssimpSceneWrapper
{
#if MRPT_HAS_ASSIMP
  Assimp::Importer importer;
  const aiScene* scene = nullptr;
#endif
};

// ============================================================================
// Serialization
// ============================================================================

uint8_t CAssimpModel::serializeGetVersion() const { return 1; }

void CAssimpModel::serializeTo(mrpt::serialization::CArchive& out) const
{
  // Serialize base class (CSetOfObjects)
  CSetOfObjects::serializeTo(out);

  // v1: model info
  out << m_modelPath;
  out << m_modelLoadFlags;
  out << m_splitTrianglesRenderingBBox;
}

void CAssimpModel::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    {
      // Deserialize base class
      CSetOfObjects::serializeFrom(in, version);

      if (version >= 1)
      {
        in >> m_modelPath;
        in >> m_modelLoadFlags;
        in >> m_splitTrianglesRenderingBBox;
      }

      // Rebuild internal pointers to child objects
      m_texturedMeshes.clear();
      m_nonTexturedMesh.reset();

      for (auto& child : *this)
      {
        if (auto texMesh = std::dynamic_pointer_cast<CSetOfTexturedTriangles>(child))
        {
          m_texturedMeshes.push_back(texMesh);
        }
        else if (auto triMesh = std::dynamic_pointer_cast<CSetOfTriangles>(child))
        {
          m_nonTexturedMesh = triMesh;
        }
      }
    }
    break;

    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }

  CVisualObject::notifyChange();
}

// ============================================================================
// Clear
// ============================================================================

void CAssimpModel::clear()
{
  // Clear child objects (from CSetOfObjects)
  CSetOfObjects::clear();

  // Clear internal state
  m_modelPath.clear();
  m_modelDirectory.clear();
  m_modelLoadFlags = 0;
  m_texturedMeshes.clear();
  m_nonTexturedMesh.reset();
  m_textureCache.clear();
  m_cachedBBox.reset();
  m_bboxMin = {0, 0, 0};
  m_bboxMax = {0, 0, 0};

#if MRPT_HAS_ASSIMP
  if (m_assimpScene)
  {
    m_assimpScene->scene = nullptr;
  }
#endif

  CVisualObject::notifyChange();
}

// ============================================================================
// Load Scene
// ============================================================================

void CAssimpModel::loadScene(const std::string& file_name, int flags)
{
#if MRPT_HAS_ASSIMP
  MRPT_START

  // Clear previous content
  clear();

  const bool verbose = (flags & LoadFlags::Verbose) != 0;

  if (verbose)
  {
    std::cout << "[CAssimpModel] Loading: " << file_name << "\n";
  }

  // Store model info
  m_modelPath = file_name;
  m_modelLoadFlags = static_cast<uint32_t>(flags);
  m_modelDirectory = mrpt::system::extractFileDirectory(file_name);

  // Initialize pimpl if needed
  if (!m_assimpScene)
  {
    m_assimpScene = mrpt::pimpl<AssimpSceneWrapper>();
  }

  // Build Assimp import flags
  unsigned int assimpFlags = 0;

  if (flags & LoadFlags::RealTimeFast)
  {
    assimpFlags |= aiProcessPreset_TargetRealtime_Fast;
  }
  else if (flags & LoadFlags::RealTimeQuality)
  {
    assimpFlags |= aiProcessPreset_TargetRealtime_Quality;
  }
  else if (flags & LoadFlags::RealTimeMaxQuality)
  {
    assimpFlags |= aiProcessPreset_TargetRealtime_MaxQuality;
  }
  else
  {
    // Default: quality preset
    assimpFlags |= aiProcessPreset_TargetRealtime_Quality;
  }

  if (flags & LoadFlags::FlipUVs)
  {
    assimpFlags |= aiProcess_FlipUVs;
  }

  // Always triangulate and generate normals if missing
  assimpFlags |= aiProcess_Triangulate;
  assimpFlags |= aiProcess_GenSmoothNormals;
  assimpFlags |= aiProcess_JoinIdenticalVertices;

  // Load the scene
  m_assimpScene->scene = m_assimpScene->importer.ReadFile(file_name, assimpFlags);

  if (!m_assimpScene->scene || m_assimpScene->scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE ||
      !m_assimpScene->scene->mRootNode)
  {
    const char* errorStr = m_assimpScene->importer.GetErrorString();
    THROW_EXCEPTION_FMT(
        "Assimp failed to load '%s': %s", file_name.c_str(),
        errorStr ? errorStr : "(unknown error)");
  }

  if (verbose)
  {
    const aiScene* sc = m_assimpScene->scene;
    std::cout << "[CAssimpModel] Loaded scene with:\n"
              << "  - " << sc->mNumMeshes << " meshes\n"
              << "  - " << sc->mNumMaterials << " materials\n"
              << "  - " << sc->mNumTextures << " embedded textures\n";
  }

  // Process the scene
  processAssimpScene();

  // Apply splitting if enabled
  if (m_splitTrianglesRenderingBBox > 0.0f)
  {
    applySplitTrianglesRendering();
  }

  if (verbose)
  {
    std::cout << "[CAssimpModel] Created:\n"
              << "  - " << m_texturedMeshes.size() << " textured mesh groups\n"
              << "  - " << (m_nonTexturedMesh ? "1" : "0") << " non-textured mesh\n"
              << "  - BBox: [" << m_bboxMin << "] to [" << m_bboxMax << "]\n";
  }

  CVisualObject::notifyChange();

  MRPT_END
#else
  MRPT_UNUSED_PARAM(file_name);
  MRPT_UNUSED_PARAM(flags);
  THROW_EXCEPTION("MRPT was built without Assimp support. Cannot load 3D models.");
#endif
}

// ============================================================================
// Process Assimp Scene
// ============================================================================

void CAssimpModel::processAssimpScene()
{
#if MRPT_HAS_ASSIMP
  MRPT_START

  if (!m_assimpScene || !m_assimpScene->scene)
  {
    return;
  }

  const aiScene* scene = m_assimpScene->scene;

  // Initialize bounding box
  m_bboxMin = {
      std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max()};
  m_bboxMax = {
      std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::lowest()};

  // Create non-textured mesh container
  m_nonTexturedMesh = CSetOfTriangles::Create();
  m_nonTexturedMesh->setName("non_textured_mesh");

  // Process the scene hierarchy starting from root
  CPose3D identityPose;
  processNode(scene->mRootNode, scene, identityPose);

  // Add non-textured mesh to children if it has content
  if (m_nonTexturedMesh && m_nonTexturedMesh->getTrianglesCount() > 0)
  {
    insert(m_nonTexturedMesh);
  }
  else
  {
    m_nonTexturedMesh.reset();
  }

  MRPT_END
#endif
}

void CAssimpModel::processNode(
    const void* nodePtr, const void* scenePtr, const CPose3D& parentTransform)
{
#if MRPT_HAS_ASSIMP
  const aiNode* node = static_cast<const aiNode*>(nodePtr);
  const aiScene* scene = static_cast<const aiScene*>(scenePtr);

  if (!node)
  {
    return;
  }

  // Compute this node's transform
  const aiMatrix4x4& m = node->mTransformation;

  // Assimp uses row-major matrices, convert to CPose3D
  // aiMatrix4x4 is column-major when accessed via a[row][col]
  mrpt::math::CMatrixDouble44 mat;
  mat(0, 0) = m.a1;
  mat(0, 1) = m.a2;
  mat(0, 2) = m.a3;
  mat(0, 3) = m.a4;
  mat(1, 0) = m.b1;
  mat(1, 1) = m.b2;
  mat(1, 2) = m.b3;
  mat(1, 3) = m.b4;
  mat(2, 0) = m.c1;
  mat(2, 1) = m.c2;
  mat(2, 2) = m.c3;
  mat(2, 3) = m.c4;
  mat(3, 0) = m.d1;
  mat(3, 1) = m.d2;
  mat(3, 2) = m.d3;
  mat(3, 3) = m.d4;

  CPose3D nodeTransform(mat);
  CPose3D worldTransform = parentTransform + nodeTransform;

  // Process all meshes in this node
  for (unsigned int i = 0; i < node->mNumMeshes; i++)
  {
    const aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
    processMesh(mesh, scene, worldTransform);
  }

  // Recursively process children
  for (unsigned int i = 0; i < node->mNumChildren; i++)
  {
    processNode(node->mChildren[i], scene, worldTransform);
  }
#else
  MRPT_UNUSED_PARAM(nodePtr);
  MRPT_UNUSED_PARAM(scenePtr);
  MRPT_UNUSED_PARAM(parentTransform);
#endif
}

void CAssimpModel::processMesh(const void* meshPtr, const void* scenePtr, const CPose3D& transform)
{
#if MRPT_HAS_ASSIMP
  MRPT_START

  const aiMesh* mesh = static_cast<const aiMesh*>(meshPtr);
  const aiScene* scene = static_cast<const aiScene*>(scenePtr);

  if (!mesh || mesh->mNumFaces == 0)
  {
    return;
  }

  const bool verbose = (m_modelLoadFlags & LoadFlags::Verbose) != 0;
  const bool ignoreMaterialColor = (m_modelLoadFlags & LoadFlags::IgnoreMaterialColor) != 0;
  const bool ignoreTextures = (m_modelLoadFlags & LoadFlags::IgnoreTextures) != 0;

  // Get material
  const aiMaterial* material = nullptr;
  if (mesh->mMaterialIndex < scene->mNumMaterials)
  {
    material = scene->mMaterials[mesh->mMaterialIndex];
  }

  // Check for diffuse texture
  std::string texturePath;
  if (!ignoreTextures && material)
  {
    aiString aiTexPath;
    if (material->GetTexture(aiTextureType_DIFFUSE, 0, &aiTexPath) == AI_SUCCESS)
    {
      // Resolve texture path relative to model directory
      std::string texFile = aiTexPath.C_Str();

      // Handle embedded textures (start with '*')
      if (!texFile.empty() && texFile[0] == '*')
      {
        // Embedded texture - use index as identifier
        texturePath = "*embedded_" + texFile.substr(1);
      }
      else
      {
        // External texture file
        if (mrpt::system::fileExists(texFile))
        {
          texturePath = texFile;
        }
        else
        {
          // Try relative to model directory
          texturePath = m_modelDirectory + "/" + texFile;
          if (!mrpt::system::fileExists(texturePath))
          {
            // Try just the filename
            texturePath = m_modelDirectory + "/" + mrpt::system::extractFileName(texFile);
            if (!mrpt::system::fileExists(texturePath))
            {
              if (verbose)
              {
                std::cerr << "[CAssimpModel] Warning: texture not found: " << texFile << "\n";
              }
              texturePath.clear();
            }
          }
        }
      }
    }
  }

  // Get material color (if not ignoring)
  TColor materialColor(255, 255, 255, 255);
  if (!ignoreMaterialColor && material)
  {
    aiColor4D color;
    if (AI_SUCCESS == aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, &color))
    {
      materialColor.R = static_cast<uint8_t>(color.r * 255);
      materialColor.G = static_cast<uint8_t>(color.g * 255);
      materialColor.B = static_cast<uint8_t>(color.b * 255);
      materialColor.A = static_cast<uint8_t>(color.a * 255);
    }
  }
  else if (ignoreMaterialColor)
  {
    // Use the object's base color
    materialColor = getColor_u8();
  }

  // Determine target: textured or non-textured
  const bool hasTexture = !texturePath.empty();

  CSetOfTexturedTriangles::Ptr texturedMesh;
  if (hasTexture)
  {
    texturedMesh = getOrCreateTexturedMesh(texturePath);
  }

  // Process faces (triangles)
  for (unsigned int f = 0; f < mesh->mNumFaces; f++)
  {
    const aiFace& face = mesh->mFaces[f];

    // We only handle triangles (should be guaranteed by aiProcess_Triangulate)
    if (face.mNumIndices != 3)
    {
      continue;
    }

    TTriangle tri;

    for (unsigned int v = 0; v < 3; v++)
    {
      unsigned int idx = face.mIndices[v];

      // Position
      const aiVector3D& pos = mesh->mVertices[idx];
      TPoint3Df localPos(pos.x, pos.y, pos.z);

      // Transform to world coordinates
      TPoint3D worldPos;
      transform.composePoint(
          localPos.x, localPos.y, localPos.z, worldPos.x, worldPos.y, worldPos.z);

      tri.vertices[v].xyzrgba.pt = TPoint3Df(
          static_cast<float>(worldPos.x), static_cast<float>(worldPos.y),
          static_cast<float>(worldPos.z));

      // Update bounding box
      updateBoundingBox(tri.vertices[v].xyzrgba.pt);

      // Normal
      if (mesh->HasNormals())
      {
        const aiVector3D& norm = mesh->mNormals[idx];
        // Transform normal (rotation only)
        const TPoint3D worldNorm = transform.rotateVector(TPoint3D(norm.x, norm.y, norm.z));
        tri.vertices[v].normal = TVector3Df(
            static_cast<float>(worldNorm.x), static_cast<float>(worldNorm.y),
            static_cast<float>(worldNorm.z));
      }

      // Color
      if (mesh->HasVertexColors(0))
      {
        const aiColor4D& col = mesh->mColors[0][idx];
        tri.vertices[v].xyzrgba.r = static_cast<uint8_t>(col.r * 255);
        tri.vertices[v].xyzrgba.g = static_cast<uint8_t>(col.g * 255);
        tri.vertices[v].xyzrgba.b = static_cast<uint8_t>(col.b * 255);
        tri.vertices[v].xyzrgba.a = static_cast<uint8_t>(col.a * 255);
      }
      else
      {
        tri.vertices[v].xyzrgba.r = materialColor.R;
        tri.vertices[v].xyzrgba.g = materialColor.G;
        tri.vertices[v].xyzrgba.b = materialColor.B;
        tri.vertices[v].xyzrgba.a = materialColor.A;
      }

      // Texture coordinates
      if (mesh->HasTextureCoords(0))
      {
        const aiVector3D& uv = mesh->mTextureCoords[0][idx];
        tri.vertices[v].uv.x = uv.x;
        tri.vertices[v].uv.y = uv.y;
      }
      else
      {
        tri.vertices[v].uv.x = 0.0f;
        tri.vertices[v].uv.y = 0.0f;
      }
    }

    // Compute normals if not provided
    if (!mesh->HasNormals())
    {
      tri.computeNormals();
    }

    // Add to appropriate container
    if (hasTexture && texturedMesh)
    {
      texturedMesh->insertTriangle(tri);
    }
    else if (m_nonTexturedMesh)
    {
      m_nonTexturedMesh->insertTriangle(tri);
    }
  }

  MRPT_END
#else
  MRPT_UNUSED_PARAM(meshPtr);
  MRPT_UNUSED_PARAM(scenePtr);
  MRPT_UNUSED_PARAM(transform);
#endif
}

// ============================================================================
// Texture Management
// ============================================================================

const CAssimpModel::LoadedTexture* CAssimpModel::loadTexture(const std::string& texturePath)
{
#if MRPT_HAS_ASSIMP
  MRPT_START

  // Check cache
  auto it = m_textureCache.find(texturePath);
  if (it != m_textureCache.end())
  {
    return &it->second;
  }

  const bool verbose = (m_modelLoadFlags & LoadFlags::Verbose) != 0;

  LoadedTexture tex;

  // Handle embedded textures
  if (texturePath.substr(0, 10) == "*embedded_")
  {
    if (!m_assimpScene || !m_assimpScene->scene)
    {
      return nullptr;
    }

    int texIdx = std::stoi(texturePath.substr(10));
    if (texIdx < 0 || static_cast<unsigned>(texIdx) >= m_assimpScene->scene->mNumTextures)
    {
      if (verbose)
      {
        std::cerr << "[CAssimpModel] Invalid embedded texture index: " << texIdx << "\n";
      }
      return nullptr;
    }

    const aiTexture* aiTex = m_assimpScene->scene->mTextures[texIdx];

    if (aiTex->mHeight == 0)
    {
      // Compressed texture data
      // TODO: Load from memory using CImage
      if (verbose)
      {
        std::cerr << "[CAssimpModel] Compressed embedded textures not yet supported\n";
      }
      return nullptr;
    }

    {
      // Raw ARGB8888 data
      tex.rgb.resize(aiTex->mWidth, aiTex->mHeight, mrpt::img::CH_RGB);
      tex.alpha = mrpt::img::CImage(aiTex->mWidth, aiTex->mHeight, mrpt::img::CH_GRAY);

      for (unsigned int y = 0; y < aiTex->mHeight; y++)
      {
        for (unsigned int x = 0; x < aiTex->mWidth; x++)
        {
          const aiTexel& texel = aiTex->pcData[y * aiTex->mWidth + x];
          tex.rgb.setPixel({x, y}, TColor(texel.r, texel.g, texel.b));
          tex.alpha->setPixel({x, y}, TColor(texel.a, texel.a, texel.a));
        }
      }
    }
  }
  else
  {
    // Load from file
    if (!tex.rgb.loadFromFile(texturePath))
    {
      if (verbose)
      {
        std::cerr << "[CAssimpModel] Failed to load texture: " << texturePath << "\n";
      }
      return nullptr;
    }

    if (verbose)
    {
      std::cout << "[CAssimpModel] Loaded texture: " << texturePath << " (" << tex.rgb.getWidth()
                << "x" << tex.rgb.getHeight() << ")\n";
    }
  }

  // Store in cache
  auto [insertIt, inserted] = m_textureCache.emplace(texturePath, std::move(tex));
  return &insertIt->second;

  MRPT_END
#else
  MRPT_UNUSED_PARAM(texturePath);
  return nullptr;
#endif
}

CSetOfTexturedTriangles::Ptr CAssimpModel::getOrCreateTexturedMesh(const std::string& texturePath)
{
  // Check if we already have a mesh for this texture
  for (auto& mesh : m_texturedMeshes)
  {
    // Compare by name (we use texture path as name)
    if (mesh->getName() == texturePath)
    {
      return mesh;
    }
  }

  // Create new textured mesh
  auto mesh = CSetOfTexturedTriangles::Create();
  mesh->setName(texturePath);

  // Load and assign texture
  const LoadedTexture* tex = loadTexture(texturePath);
  if (tex != nullptr)
  {
    if (tex->alpha.has_value())
    {
      mesh->assignImage(tex->rgb, tex->alpha.value());
    }
    else
    {
      mesh->assignImage(tex->rgb);
    }
  }

  // Add to children and tracking list
  insert(mesh);
  m_texturedMeshes.push_back(mesh);

  return mesh;
}

// ============================================================================
// Bounding Box
// ============================================================================

void CAssimpModel::updateBoundingBox(const TPoint3Df& point)
{
  m_bboxMin.x = std::min(m_bboxMin.x, point.x);
  m_bboxMin.y = std::min(m_bboxMin.y, point.y);
  m_bboxMin.z = std::min(m_bboxMin.z, point.z);

  m_bboxMax.x = std::max(m_bboxMax.x, point.x);
  m_bboxMax.y = std::max(m_bboxMax.y, point.y);
  m_bboxMax.z = std::max(m_bboxMax.z, point.z);

  m_cachedBBox.reset();  // Invalidate cache
}

TBoundingBoxf CAssimpModel::internalBoundingBoxLocal() const
{
  if (m_cachedBBox.has_value())
  {
    return m_cachedBBox.value();
  }

  // If we have computed bounds during loading
  if (m_bboxMax.x >= m_bboxMin.x)
  {
    m_cachedBBox = TBoundingBoxf(m_bboxMin, m_bboxMax);
    return m_cachedBBox.value();
  }

  // Fall back to computing from children
  return CSetOfObjects::internalBoundingBoxLocal();
}

// ============================================================================
// Triangle Splitting
// ============================================================================

void CAssimpModel::setSplitTrianglesRenderingBBox(float bbox_size)
{
  if (m_splitTrianglesRenderingBBox == bbox_size)
  {
    return;
  }

  m_splitTrianglesRenderingBBox = bbox_size;

  // If we have loaded content, reapply splitting
  if (!m_modelPath.empty() && bbox_size > 0.0f)
  {
    applySplitTrianglesRendering();
    CVisualObject::notifyChange();
  }
}

void CAssimpModel::applySplitTrianglesRendering()
{
  // TODO: Implement spatial subdivision of textured meshes for correct
  // transparency sorting. This would involve:
  // 1. For each textured mesh, compute spatial grid based on bbox_size
  // 2. Assign triangles to grid cells
  // 3. Create separate child objects for each cell
  // 4. Enable depth sorting in the renderer

  // For now, this is a no-op placeholder
}

// ============================================================================
// Query Methods
// ============================================================================

size_t CAssimpModel::getNonTexturedTriangleCount() const
{
  if (m_nonTexturedMesh)
  {
    return m_nonTexturedMesh->getTrianglesCount();
  }
  return 0;
}

size_t CAssimpModel::getTotalVertexCount() const
{
  size_t count = 0;

  for (const auto& mesh : m_texturedMeshes)
  {
    count += mesh->getTrianglesCount() * 3;
  }

  if (m_nonTexturedMesh)
  {
    count += m_nonTexturedMesh->getTrianglesCount() * 3;
  }

  return count;
}

size_t CAssimpModel::getTotalTriangleCount() const
{
  size_t count = 0;

  for (const auto& mesh : m_texturedMeshes)
  {
    count += mesh->getTrianglesCount();
  }

  if (m_nonTexturedMesh)
  {
    count += m_nonTexturedMesh->getTrianglesCount();
  }

  return count;
}

std::vector<CAssimpModel::TextureInfo> CAssimpModel::getTextureInfo() const
{
  std::vector<TextureInfo> result;
  result.reserve(m_texturedMeshes.size());

  for (const auto& mesh : m_texturedMeshes)
  {
    TextureInfo info;
    info.filepath = mesh->getName();
    info.triangleCount = mesh->getTrianglesCount();

    const auto& img = mesh->getTextureImage();
    if (!img.isEmpty())
    {
      info.width = img.getWidth();
      info.height = img.getHeight();
      info.hasAlpha = !mesh->getTextureAlphaImage().isEmpty();
    }

    result.push_back(info);
  }

  return result;
}

// ============================================================================
// Ray Tracing
// ============================================================================

bool CAssimpModel::traceRay(const CPose3D& o, double& dist) const
{
  return CSetOfObjects::traceRay(o, dist);
}