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

#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/opengl/CompiledScene.h>
#include <mrpt/opengl/LinesProxy.h>
#include <mrpt/opengl/PointsProxy.h>
#include <mrpt/opengl/TexturedTrianglesProxy.h>
#include <mrpt/opengl/TrianglesProxy.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CText3D.h>

#include "gltext.h"

#include <Eigen/Dense>
#include <iostream>

using namespace mrpt::opengl;
using namespace mrpt::viz;

static const bool SCENE_VERBOSE = mrpt::get_env<bool>("MRPT_SCENE_VERBOSE", false);

// ============================================================================
// Text3DProxy: generates text geometry from CText3D using the gltext system
// ============================================================================
namespace
{
class Text3DProxy : public TrianglesProxyBase
{
 public:
  void compile(const mrpt::viz::CVisualObject* sourceObj) override
  {
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
    MRPT_START

    const auto* text3d = dynamic_cast<const mrpt::viz::CText3D*>(sourceObj);
    if (text3d == nullptr) { return; }

    m_lightEnabled = false;
    m_cullFace = mrpt::viz::TCullFace::NONE;

    // Generate text geometry using gltext
    std::vector<mrpt::viz::TTriangle> tris;
    std::vector<mrpt::math::TPoint3Df> lineVerts;
    std::vector<mrpt::img::TColor> lineColors;

    internal::glSetFont(text3d->getFont());
    internal::glDrawTextTransformed(
        text3d->getString(), tris, lineVerts, lineColors, mrpt::poses::CPose3D(),
        text3d->getScaleX(), text3d->getColor_u8(), text3d->getTextStyle(),
        text3d->setTextSpacing(), text3d->setTextKerning());

    m_triangleCount = tris.size();
    if (m_triangleCount == 0) { return; }

    const size_t vertexCount = m_triangleCount * 3;

    std::vector<mrpt::math::TPoint3Df> vertices;
    std::vector<mrpt::math::TVector3Df> normals;
    std::vector<mrpt::img::TColor> colors;
    vertices.reserve(vertexCount);
    normals.reserve(vertexCount);
    colors.reserve(vertexCount);

    for (const auto& tri : tris)
    {
      for (int i = 0; i < 3; ++i)
      {
        vertices.push_back(tri.vertices[i].xyzrgba.pt);
        normals.push_back(tri.vertices[i].normal);
        const auto& rgba = tri.vertices[i].xyzrgba;
        colors.emplace_back(rgba.r, rgba.g, rgba.b, rgba.a);
      }
    }

    // Create VAO and upload to GPU
    m_vao.createOnce();
    m_vao.bind();

    m_vertexBuffer.createOnce();
    m_vertexBuffer.bind();
    m_vertexBuffer.allocate(
        vertices.data(), static_cast<int>(sizeof(mrpt::math::TPoint3Df) * vertexCount));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(mrpt::math::TPoint3Df), nullptr);

    m_colorBuffer.createOnce();
    m_colorBuffer.bind();
    m_colorBuffer.allocate(
        colors.data(), static_cast<int>(sizeof(mrpt::img::TColor) * vertexCount));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(mrpt::img::TColor), nullptr);

    m_normalBuffer.createOnce();
    m_normalBuffer.bind();
    m_normalBuffer.allocate(
        normals.data(), static_cast<int>(sizeof(mrpt::math::TVector3Df) * vertexCount));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(mrpt::math::TVector3Df), nullptr);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    m_cachedBBox.reset();
    CHECK_OPENGL_ERROR_IN_DEBUG();

    MRPT_END
#endif
  }

  const char* typeName() const override { return "Text3DProxy"; }
};
}  // namespace

// ============================================================================
// CompiledScene Implementation
// ============================================================================

CompiledScene::CompiledScene()
{
  m_contextThread = std::this_thread::get_id();

  if (SCENE_VERBOSE)
  {
    std::cout << "[CompiledScene] Created\n";
  }
}

void CompiledScene::compile(const Scene& scene, CompilationStats* stats)
{
  MRPT_START

  checkContextThread();

  // Clear any existing compilation
  clear();

  // Store reference to source scene (as const)
  m_sourceScene = scene.shared_from_this();

  CompilationStats localStats;
  CompilationStats& s = stats ? *stats : localStats;
  s.reset();

  if (SCENE_VERBOSE)
  {
    std::cout << "[CompiledScene::compile] Starting compilation...\n";
  }

  // Iterate over all viewports in the scene
  for (const Viewport::Ptr& vizViewportPtr : scene.viewports())
  {
    ASSERT_(vizViewportPtr);
    const auto& vizViewport = *vizViewportPtr;
    const std::string& vpName = vizViewport.getName();

    // Create compiled viewport
    auto compiledVp = std::make_shared<CompiledViewport>(vpName);

    // Copy viewport configuration
    compiledVp->updateFromVizViewport(vizViewport);

    // Compile all objects in the viewport
    compileViewport(vizViewport, *compiledVp, s);

    // Store compiled viewport
    m_viewports[vpName] = compiledVp;

    if (SCENE_VERBOSE)
    {
      std::cout << "[CompiledScene::compile] Viewport '" << vpName
                << "': " << compiledVp->getProxyCount() << " proxies\n";
    }
  }

  m_isCompiled = true;
  m_lastStats = s;

  if (SCENE_VERBOSE)
  {
    std::cout << "[CompiledScene::compile] Done. Total objects: " << s.numObjectsTotal
              << ", compiled: " << s.numObjectsCompiled << ", proxies: " << s.numProxiesCreated
              << "\n";
  }

  MRPT_END
}

void CompiledScene::compileViewport(
    const Viewport& vizViewport, CompiledViewport& compiledViewport, CompilationStats& stats)
{
  MRPT_START

  // Handle cloned viewport mode (objects from another viewport — skip
  // compiling our own objects)
  if (vizViewport.isCloned())
  {
    compiledViewport.setCloneMode(vizViewport.isClonedCameraFrom(), vizViewport.isClonedCamera());
    return;
  }

  // Handle cloned camera only (use camera from another viewport, but
  // compile our own objects normally)
  if (vizViewport.isClonedCamera())
  {
    compiledViewport.setCloneMode(vizViewport.isClonedCameraFrom(), true /*cloneCamera*/);
    // Don't return — continue to compile objects below
  }

  // Handle image view mode - skip normal object compilation
  if (vizViewport.isImageViewMode())
  {
    return;
  }

  // Compile all objects in viewport
  for (const auto& obj : vizViewport)
  {
    if (!obj)
    {
      continue;
    }

    compileObject(obj, compiledViewport, stats);
  }

  MRPT_END
}

mrpt::math::CMatrixFloat44 CompiledScene::computeModelMatrix(
    const CVisualObject& obj, const mrpt::math::CMatrixFloat44& parentModelMatrix)
{
  mrpt::math::CMatrixFloat44 HM =
      obj.getCPose().getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>().cast_float();

  // Apply scaling if any axis differs from 1.0
  if (obj.getScaleX() != 1 || obj.getScaleY() != 1 || obj.getScaleZ() != 1)
  {
    auto scale = mrpt::math::CMatrixFloat44::Identity();
    scale(0, 0) = obj.getScaleX();
    scale(1, 1) = obj.getScaleY();
    scale(2, 2) = obj.getScaleZ();
    HM.asEigen() = HM.asEigen() * scale.asEigen();
  }

  // Compose with parent transform
  mrpt::math::CMatrixFloat44 result;
  result.asEigen() = parentModelMatrix.asEigen() * HM.asEigen();
  return result;
}

void CompiledScene::compileObject(
    const std::shared_ptr<CVisualObject>& obj,
    CompiledViewport& compiledViewport,
    CompilationStats& stats,
    const mrpt::math::CMatrixFloat44& parentModelMatrix)
{
  MRPT_START

  if (!obj)
  {
    return;
  }

  stats.numObjectsTotal++;

  // Skip invisible objects
  if (!obj->isVisible())
  {
    return;
  }

  // Compute model matrix for this object (pose + scale + parent)
  const auto modelMatrix = computeModelMatrix(*obj, parentModelMatrix);

  // Check if this is a container (CSetOfObjects)
  const auto* setOfObjects = dynamic_cast<const CSetOfObjects*>(obj.get());
  if (setOfObjects)
  {
    // Recursively compile children, passing this container's model matrix
    for (auto it = setOfObjects->begin(); it != setOfObjects->end(); ++it)
    {
      compileObject(*it, compiledViewport, stats, modelMatrix);
    }
    return;
  }

  // Skip if we already have a proxy for this object
  if (hasProxyFor(obj))
  {
    return;
  }

  // Create proxy(ies) for this object — one per mixin type
  auto proxies = createProxiesByType(obj);
  if (proxies.empty())
  {
    if (SCENE_VERBOSE)
    {
      std::cout << "[CompiledScene::compileObject] No proxy type for: "
                << obj->GetRuntimeClass()->className << "\n";
    }
    return;
  }

  obj->updateBuffers();  // Populate viz buffers first

  for (auto& proxy : proxies)
  {
    // Set the source object reference in the proxy
    proxy->setSourceObject(obj);

    // Set the model matrix (object local frame -> world frame)
    proxy->m_modelMatrix = modelMatrix;

    // Compile the proxy (upload data to GPU)
    proxy->compile(obj.get());

    // Add to viewport
    compiledViewport.addProxy(proxy, obj);

    stats.numProxiesCreated++;
  }

  // Clear dirty flag after successful compile
  obj->clearChangedFlag();

  // Register in our tracking map (store first proxy for tracking;
  // all proxies are tracked via the viewport's objectToProxy map)
  std::weak_ptr<CVisualObject> weakObj = obj;
  m_objectToProxy[weakObj] = proxies.front();

  stats.numObjectsCompiled++;

  MRPT_END
}

bool CompiledScene::hasProxyFor(const std::shared_ptr<CVisualObject>& obj) const
{
  if (!obj)
  {
    return false;
  }

  std::weak_ptr<CVisualObject> weakObj = obj;
  return m_objectToProxy.find(weakObj) != m_objectToProxy.end();
}

std::vector<RenderableProxy::Ptr> CompiledScene::createProxiesByType(
    const std::shared_ptr<CVisualObject>& obj)
{
  std::vector<RenderableProxy::Ptr> proxies;

  if (!obj)
  {
    return proxies;
  }

  // Check for CText3D first (special case: generates geometry in the proxy)
  if (dynamic_cast<const mrpt::viz::CText3D*>(obj.get()) != nullptr)
  {
    proxies.push_back(std::make_shared<Text3DProxy>());
    return proxies;
  }

  // Check for textured triangles (more specific than plain triangles)
  if (dynamic_cast<const VisualObjectParams_TexturedTriangles*>(obj.get()))
  {
    proxies.push_back(std::make_shared<TexturedTrianglesProxy>());
  }
  // Check for plain triangles (only if NOT textured, since textured already
  // handles the triangle data)
  else if (dynamic_cast<const VisualObjectParams_Triangles*>(obj.get()))
  {
    proxies.push_back(std::make_shared<TrianglesProxy>());
  }

  // Check for points (independent of triangles — an object can have both)
  if (dynamic_cast<const VisualObjectParams_Points*>(obj.get()))
  {
    proxies.push_back(std::make_shared<PointsProxy>());
  }

  // Check for lines (independent of triangles — an object can have both)
  if (dynamic_cast<const VisualObjectParams_Lines*>(obj.get()))
  {
    proxies.push_back(std::make_shared<LinesProxy>());
  }

  return proxies;
}

bool CompiledScene::updateIfNeeded(CompilationStats* stats)
{
  MRPT_START

  if (!m_isCompiled || !m_sourceScene)
  {
    return false;
  }

  checkContextThread();

  CompilationStats localStats;
  CompilationStats& s = stats ? *stats : localStats;
  s.reset();

  bool anyChanges = false;

  // Step 1: Cleanup orphaned proxies (deleted source objects)
  cleanupOrphanedProxies(s);
  if (s.numOrphanedProxies > 0)
  {
    anyChanges = true;
  }

  // Step 2: Scan for new objects in the source scene
  compileNewObjects(s);
  if (s.numNewObjects > 0)
  {
    anyChanges = true;
  }

  // Step 3: Update dirty objects
  updateDirtyObjects(s);
  if (s.numObjectsUpdated > 0)
  {
    anyChanges = true;
  }

  if (anyChanges)
  {
    m_lastStats = s;
  }

  return anyChanges;

  MRPT_END
}

void CompiledScene::cleanupOrphanedProxies(CompilationStats& stats)
{
  MRPT_START

  // Find and remove proxies whose source objects have been deleted
  for (auto it = m_objectToProxy.begin(); it != m_objectToProxy.end();)
  {
    if (it->first.expired())
    {
      // Object was deleted - remove proxy from all viewports
      auto& proxy = it->second;

      for (auto& [name, viewport] : m_viewports)
      {
        viewport->removeProxy(proxy);
      }

      it = m_objectToProxy.erase(it);
      stats.numOrphanedProxies++;
      stats.numProxiesDeleted++;

      if (SCENE_VERBOSE)
      {
        std::cout << "[CompiledScene::cleanupOrphanedProxies] Removed orphan\n";
      }
    }
    else
    {
      ++it;
    }
  }

  MRPT_END
}

void CompiledScene::compileNewObjects(CompilationStats& stats)
{
  MRPT_START

  if (!m_sourceScene)
  {
    return;
  }

  // Iterate all viewports and their objects to find new ones
  for (const auto& vpPtr : m_sourceScene->viewports())
  {
    ASSERT_(vpPtr);
    const Viewport& vizViewport = *vpPtr;
    const std::string& vpName = vizViewport.getName();

    // Get or create compiled viewport
    auto compiledVpIt = m_viewports.find(vpName);
    if (compiledVpIt == m_viewports.end())
    {
      // New viewport - create and compile it
      auto compiledVp = std::make_shared<CompiledViewport>(vpName);
      compiledVp->updateFromVizViewport(vizViewport);
      compileViewport(vizViewport, *compiledVp, stats);
      m_viewports[vpName] = compiledVp;
      continue;
    }

    CompiledViewport& compiledViewport = *compiledVpIt->second;

    // Update viewport configuration (camera, lights, etc.)
    compiledViewport.updateFromVizViewport(vizViewport);

    // Skip cloned/image viewports
    if (vizViewport.isCloned() || vizViewport.isImageViewMode())
    {
      continue;
    }

    // Check each object in the viewport
    for (const auto& obj : vizViewport)
    {
      if (!obj || !obj->isVisible())
      {
        continue;
      }

      // Handle containers recursively
      const auto* setOfObjects = dynamic_cast<const CSetOfObjects*>(obj.get());
      if (setOfObjects != nullptr)
      {
        // Use a stack to avoid recursion
        std::vector<const CSetOfObjects*> containers;
        containers.push_back(setOfObjects);

        while (!containers.empty())
        {
          const CSetOfObjects* container = containers.back();
          containers.pop_back();

          for (const auto& child : *container)
          {
            if (!child || !child->isVisible())
            {
              continue;
            }

            const auto* childContainer = dynamic_cast<const CSetOfObjects*>(child.get());
            if (childContainer != nullptr)
            {
              containers.push_back(childContainer);
            }
            else if (!hasProxyFor(child))
            {
              // New object found - compile it
              compileObject(child, compiledViewport, stats);
              stats.numNewObjects++;
            }
          }
        }
      }
      else if (!hasProxyFor(obj))
      {
        // New object found - compile it
        compileObject(obj, compiledViewport, stats);
        stats.numNewObjects++;
      }
    }
  }

  MRPT_END
}

void CompiledScene::updateDirtyObjects(CompilationStats& stats)
{
  MRPT_START

  for (auto& [weakObj, proxy] : m_objectToProxy)
  {
    auto obj = weakObj.lock();
    if (!obj)
    {
      continue;  // Will be cleaned up by cleanupOrphanedProxies
    }

    stats.numObjectsTotal++;

    // Check if object needs update via its dirty flag
    if (obj->hasToUpdateBuffers())
    {
      // First: let the viz object populate its internal buffers from geometry
      obj->updateBuffers();

      // Recompute model matrix from the object's current pose
      // (Use identity as parent since we don't track parent hierarchy here;
      //  a full recompile would be needed if parent transforms change)
      const auto modelMatrix =
          computeModelMatrix(*obj, mrpt::math::CMatrixFloat44::Identity());

      // Update ALL proxies for this object across all viewports
      for (auto& [vpName, viewport] : m_viewports)
      {
        viewport->updateProxiesForObject(weakObj, obj.get(), modelMatrix);
      }

      obj->clearChangedFlag();  // Clear dirty flag
      stats.numObjectsUpdated++;

      if (SCENE_VERBOSE)
      {
        std::cout << "[CompiledScene::updateDirtyObjects] Updated: " << obj->getName() << "\n";
      }
    }
  }

  MRPT_END
}

void CompiledScene::recompile()
{
  MRPT_START

  if (!m_sourceScene)
  {
    return;
  }

  compile(*m_sourceScene, &m_lastStats);

  MRPT_END
}

void CompiledScene::clear()
{
  MRPT_START

  m_viewports.clear();
  m_objectToProxy.clear();
  m_shaderManager.clear();
  m_sourceScene.reset();
  m_isCompiled = false;

  MRPT_END
}

bool CompiledScene::hasPendingUpdates() const
{
  // Check if any tracked object has dirty flag set
  for (const auto& [weakObj, proxy] : m_objectToProxy)
  {
    auto obj = weakObj.lock();
    if (obj && obj->hasToUpdateBuffers())
    {
      return true;
    }
  }

  // Also check if source scene has new objects we haven't compiled yet
  // (This is a quick check - actual detection happens in compileNewObjects)
  return false;
}

void CompiledScene::render(int renderWidth, int renderHeight, int renderOffsetX, int renderOffsetY)
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!m_isCompiled)
  {
    return;
  }

  checkContextThread();

  // Auto-update if enabled
  if (m_autoUpdate)
  {
    updateIfNeeded();
  }

  // Render all viewports in order
  for (auto& [name, viewport] : m_viewports)
  {
    viewport->render(renderWidth, renderHeight, renderOffsetX, renderOffsetY, m_shaderManager);
  }
#endif

  MRPT_END
}

void CompiledScene::renderViewport(
    const std::string& viewportName,
    int renderWidth,
    int renderHeight,
    int renderOffsetX,
    int renderOffsetY)
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  auto it = m_viewports.find(viewportName);
  if (it == m_viewports.end())
  {
    THROW_EXCEPTION_FMT("Viewport '%s' not found", viewportName.c_str());
  }

  checkContextThread();

  // Auto-update if enabled
  if (m_autoUpdate)
  {
    updateIfNeeded();
  }

  it->second->render(renderWidth, renderHeight, renderOffsetX, renderOffsetY, m_shaderManager);
#endif

  MRPT_END
}

void CompiledScene::checkContextThread() const
{
#ifndef NDEBUG
  if (std::this_thread::get_id() != m_contextThread)
  {
    THROW_EXCEPTION(
        "CompiledScene methods must be called from the same thread that created it "
        "(the OpenGL context thread)");
  }
#endif
}