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
#include <mrpt/viz/CText.h>
#include <mrpt/viz/CText3D.h>

#include <Eigen/Dense>
#include <iostream>

#include "gltext.h"

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
    if (text3d == nullptr)
    {
      return;
    }

    m_lightEnabled = false;
    m_cullFace = mrpt::viz::TCullFace::NONE;

    // Generate text geometry using gltext
    std::vector<mrpt::viz::TTriangle> tris;
    std::vector<mrpt::math::TPoint3Df> lineVerts;
    std::vector<mrpt::img::TColor> lineColors;

    internal::glSetFont(text3d->getFont());
    // Use scale=1.0 here; the actual scale is applied via the model matrix
    // (CVisualObject::setScale), so we don't want to bake it into geometry.
    internal::glDrawTextTransformed(
        text3d->getString(), tris, lineVerts, lineColors, mrpt::poses::CPose3D(), 1.0f,
        text3d->getColor_u8(), text3d->getTextStyle(), text3d->setTextSpacing(),
        text3d->setTextKerning());

    m_triangleCount = tris.size();
    if (m_triangleCount == 0)
    {
      return;
    }

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

// ============================================================================
// Text2DLabelProxy: generates text geometry from CText using the gltext system.
// Used for CText objects and for enableShowName() labels.
// ============================================================================
class Text2DLabelProxy : public TrianglesProxyBase
{
 public:
  void compile(const mrpt::viz::CVisualObject* sourceObj) override
  {
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
    MRPT_START

    const auto* textObj = dynamic_cast<const mrpt::viz::CText*>(sourceObj);
    if (textObj == nullptr)
    {
      return;
    }

    m_lightEnabled = false;
    m_cullFace = mrpt::viz::TCullFace::NONE;

    // Generate text geometry using gltext
    std::vector<mrpt::viz::TTriangle> tris;
    std::vector<mrpt::math::TPoint3Df> lineVerts;
    std::vector<mrpt::img::TColor> lineColors;

    internal::glSetFont(textObj->getFont());
    // CText uses NICE style by default, scale=1.0 (model matrix handles scaling)
    internal::glDrawTextTransformed(
        textObj->getString(), tris, lineVerts, lineColors, mrpt::poses::CPose3D(), 1.0f,
        textObj->getColor_u8(), mrpt::viz::NICE, 1.5, 0.1);

    m_triangleCount = tris.size();
    if (m_triangleCount == 0)
    {
      return;
    }

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

  const char* typeName() const override { return "Text2DLabelProxy"; }
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
  m_sourceScene = &scene;

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

    // Store compiled viewport (map for lookup, vector for render order)
    m_viewports[vpName] = compiledVp;
    m_viewportRenderOrder.push_back(vpName);

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
    compiledViewport.setCloneMode(
        vizViewport.getClonedViewportName(), vizViewport.isClonedCamera());
    return;
  }

  // Handle cloned camera only (use camera from another viewport, but
  // compile our own objects normally)
  if (vizViewport.isClonedCamera())
  {
    compiledViewport.setCloneMode(vizViewport.isClonedCameraFrom(), true /*cloneCamera*/);
    // Don't return — continue to compile objects below
  }

  // Handle image view mode: compile the CTexturedPlane and install it as the image proxy.
  if (vizViewport.isImageViewMode())
  {
    auto plane = vizViewport.getImageViewPlane();
    if (plane)
    {
      auto proxies = createProxiesByType(plane);
      if (!proxies.empty())
      {
        plane->updateBuffers();
        for (auto& proxy : proxies)
        {
          proxy->setSourceObject(plane);
          proxy->m_modelMatrix = mrpt::math::CMatrixFloat44::Identity();
          proxy->m_visible = true;
          proxy->compile(plane.get());
        }
        // Only the first (TexturedTriangles) proxy is used for image view
        compiledViewport.setImageViewMode(proxies.front());
        // Track in proxy maps for dirty-update support
        std::weak_ptr<CVisualObject> weakPlane = plane;
        m_objectToProxy[weakPlane].push_back(std::move(proxies));
        m_objectVersions[weakPlane] = plane->dataVersion();
      }
    }
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
    const CVisualObject::PoseAndScale& ps, const mrpt::math::CMatrixFloat44& parentModelMatrix)
{
  mrpt::math::CMatrixFloat44 HM =
      ps.pose.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>().cast_float();

  // Apply scaling if any axis differs from 1.0
  if (ps.scaleX != 1 || ps.scaleY != 1 || ps.scaleZ != 1)
  {
    auto scale = mrpt::math::CMatrixFloat44::Identity();
    scale(0, 0) = ps.scaleX;
    scale(1, 1) = ps.scaleY;
    scale(2, 2) = ps.scaleZ;
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
  const auto ps = obj->getPoseAndScale();
  const auto modelMatrix = computeModelMatrix(ps, parentModelMatrix);

  // Check if this is a container (CSetOfObjects)
  const auto* setOfObjects = dynamic_cast<const CSetOfObjects*>(obj.get());
  if (setOfObjects)
  {
    // Track version for containers (needed for multi-occurrence dirty detection)
    std::weak_ptr<CVisualObject> weakObj = obj;
    m_objectVersions[weakObj] = obj->dataVersion();

    // Recursively compile children, passing this container's model matrix
    for (auto it = setOfObjects->begin(); it != setOfObjects->end(); ++it)
    {
      compileObject(*it, compiledViewport, stats, modelMatrix);
    }
    return;
  }

  // NOTE: We intentionally do NOT skip objects that already have proxies.
  // The same CVisualObject can appear at multiple positions in the scene
  // graph (DAG structure, e.g. cached models shared between blocks).
  // Each occurrence gets its own proxy group with its own model matrix.

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

  // Register in our tracking map as a new occurrence.
  // m_objectToProxy[weakObj] is a vector of occurrence groups;
  // each group is a vector of proxies for one tree position.
  std::weak_ptr<CVisualObject> weakObj = obj;
  m_objectToProxy[weakObj].push_back(std::move(proxies));
  m_objectVersions[weakObj] = obj->dataVersion();

  stats.numObjectsCompiled++;

  // Handle composite objects (e.g. CAxis has internal CText3D children)
  if (obj->isCompositeObject())
  {
    const auto& children = obj->getInternalChildren();
    for (const auto& child : children)
    {
      if (!child)
      {
        continue;
      }
      compileObject(child, compiledViewport, stats, modelMatrix);
    }
  }

  // Handle enableShowName() label: compile the label CText at the same
  // position as the parent object
  if (obj->isShowNameEnabled())
  {
    auto labelPtr = obj->labelObjectPtr();
    if (labelPtr)
    {
      // Ensure the label text matches the object's current name
      labelPtr->setString(obj->getName());

      // Compile the label if not already compiled
      if (!hasProxyFor(labelPtr))
      {
        compileObject(labelPtr, compiledViewport, stats, modelMatrix);
      }
    }
  }

  MRPT_END
}

bool CompiledScene::hasProxyFor(const std::shared_ptr<CVisualObject>& obj) const
{
  if (!obj)
  {
    return false;
  }

  std::weak_ptr<CVisualObject> weakObj = obj;
  auto it = m_objectToProxy.find(weakObj);
  return it != m_objectToProxy.end() && !it->second.empty();
}

std::vector<RenderableProxy::Ptr> CompiledScene::createProxiesByType(
    const std::shared_ptr<CVisualObject>& obj)
{
  std::vector<RenderableProxy::Ptr> proxies;

  if (!obj)
  {
    return proxies;
  }

  // Check for CText first (special case: generates geometry in the proxy)
  if (dynamic_cast<const mrpt::viz::CText*>(obj.get()) != nullptr)
  {
    proxies.push_back(std::make_shared<Text2DLabelProxy>());
    return proxies;
  }

  // Check for CText3D (special case: generates geometry in the proxy)
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
      // Object was deleted - remove all its proxies (across all occurrences) from all viewports
      for (auto& occurrenceProxies : it->second)
      {
        for (auto& proxy : occurrenceProxies)
        {
          for (auto& [name, viewport] : m_viewports)
          {
            viewport->removeProxy(proxy);
          }
          stats.numProxiesDeleted++;
        }
      }

      m_objectVersions.erase(it->first);
      it = m_objectToProxy.erase(it);
      stats.numOrphanedProxies++;

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
      m_viewportRenderOrder.push_back(vpName);
      continue;
    }

    CompiledViewport& compiledViewport = *compiledVpIt->second;

    // Update viewport configuration (camera, lights, etc.)
    compiledViewport.updateFromVizViewport(vizViewport);

    // Skip cloned viewports
    if (vizViewport.isCloned())
    {
      continue;
    }

    // Handle image view mode: if the plane was created after initial compile, compile it now
    if (vizViewport.isImageViewMode())
    {
      auto plane = vizViewport.getImageViewPlane();
      if (plane && !compiledViewport.isImageViewMode())
      {
        // Viewport just transitioned to image-view mode — compile the plane proxy
        auto proxies = createProxiesByType(plane);
        if (!proxies.empty())
        {
          plane->updateBuffers();
          for (auto& proxy : proxies)
          {
            proxy->setSourceObject(plane);
            proxy->m_modelMatrix = mrpt::math::CMatrixFloat44::Identity();
            proxy->m_visible = true;
            proxy->compile(plane.get());
          }
          compiledViewport.setImageViewMode(proxies.front());
          std::weak_ptr<CVisualObject> weakPlane = plane;
          m_objectToProxy[weakPlane].push_back(std::move(proxies));
          m_objectVersions[weakPlane] = plane->dataVersion();
        }
      }
      continue;
    }

    // Check each object in the viewport
    for (const auto& obj : vizViewport)
    {
      if (!obj || !obj->isVisible())
      {
        continue;
      }

      // Helper: check if a CSetOfObjects is new (not yet tracked in m_objectVersions)
      auto isNewContainer = [this](const std::shared_ptr<CVisualObject>& o) -> bool
      {
        std::weak_ptr<CVisualObject> w = o;
        return m_objectVersions.find(w) == m_objectVersions.end();
      };

      // Handle containers recursively
      const auto* setOfObjects = dynamic_cast<const CSetOfObjects*>(obj.get());
      if (setOfObjects != nullptr)
      {
        if (isNewContainer(obj))
        {
          // New top-level container: compile fully via compileObject().
          // This recursively creates proxy groups for ALL children,
          // including shared objects that may already have proxies from
          // other occurrences (DAG support).
          compileObject(obj, compiledViewport, stats);
          stats.numNewObjects++;
        }
        else
        {
          // Existing container: walk into it to find new children
          std::vector<std::pair<const CSetOfObjects*, std::shared_ptr<CVisualObject>>> containers;
          containers.push_back({setOfObjects, obj});

          while (!containers.empty())
          {
            auto [container, containerPtr] = containers.back();
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
                if (isNewContainer(child))
                {
                  // New sub-container: compile fully (handles shared children)
                  compileObject(child, compiledViewport, stats);
                  stats.numNewObjects++;
                }
                else
                {
                  containers.push_back({childContainer, child});
                }
              }
              else if (!hasProxyFor(child))
              {
                // Genuinely new leaf object - compile it
                compileObject(child, compiledViewport, stats);
                stats.numNewObjects++;
              }
            }
          }
        }
      }
      else if (!hasProxyFor(obj))
      {
        // New leaf object found - compile it
        compileObject(obj, compiledViewport, stats);
        stats.numNewObjects++;
      }
      // Also check composite objects' internal children
      else if (obj->isCompositeObject())
      {
        for (const auto& child : obj->getInternalChildren())
        {
          if (child && !hasProxyFor(child))
          {
            compileObject(child, compiledViewport, stats);
            stats.numNewObjects++;
          }
        }
      }
    }
  }

  MRPT_END
}

void CompiledScene::updateDirtyObjects(CompilationStats& stats)
{
  MRPT_START

  if (!m_sourceScene) return;

  // Clear occurrence counter for this update pass
  m_updateOccurrenceCounter.clear();

  // Walk the entire scene tree (not just m_objectToProxy) so we also
  // detect dirty containers (CSetOfObjects) whose pose/visibility changed.
  for (const auto& vizViewport : m_sourceScene->viewports())
  {
    // For image-view viewports, update the image plane directly
    if (vizViewport->isImageViewMode())
    {
      auto plane = vizViewport->getImageViewPlane();
      if (plane)
      {
        updateDirtyObjectRecursive(
            plane, mrpt::math::CMatrixFloat44::Identity(), false, true, stats);
      }
      continue;
    }

    for (const auto& obj : *vizViewport)
    {
      if (!obj)
      {
        continue;
      }
      updateDirtyObjectRecursive(obj, mrpt::math::CMatrixFloat44::Identity(), false, true, stats);
    }
  }

  // Post-walk: update all version tracking to current values.
  // This is deferred from the walk to ensure multi-occurrence objects
  // (same CVisualObject at multiple tree positions) have consistent
  // dirty detection across all their occurrences within a single pass.
  for (auto& [weakObj, ver] : m_objectVersions)
  {
    auto obj = weakObj.lock();
    if (obj) ver = obj->dataVersion();
  }

  // Post-walk: hide proxies for occurrences that no longer exist in the tree
  // (e.g., a child was removed from a CSetOfObjects container).
  for (auto& [weakObj, occurrences] : m_objectToProxy)
  {
    auto counterIt = m_updateOccurrenceCounter.find(weakObj);
    size_t visitedOccs = (counterIt != m_updateOccurrenceCounter.end()) ? counterIt->second : 0;
    for (size_t i = visitedOccs; i < occurrences.size(); i++)
    {
      for (auto& proxy : occurrences[i])
      {
        proxy->m_visible = false;
      }
    }
  }

  MRPT_END
}

void CompiledScene::updateDirtyObjectRecursive(
    const std::shared_ptr<mrpt::viz::CVisualObject>& obj,
    const mrpt::math::CMatrixFloat44& parentModelMatrix,
    bool parentDirty,
    bool parentVisible,
    CompilationStats& stats)
{
  if (!obj) return;

  std::weak_ptr<mrpt::viz::CVisualObject> weakObj = obj;
  const uint64_t currentVersion = obj->dataVersion();
  auto versionIt = m_objectVersions.find(weakObj);
  const uint64_t lastVersion = (versionIt != m_objectVersions.end()) ? versionIt->second : 0;
  const bool selfDirty = (currentVersion != lastVersion);
  const bool dirty = selfDirty || parentDirty;

  // Read pose+scale+visible atomically to avoid tearing
  const auto ps = obj->getPoseAndScale();
  const bool effectiveVisible = parentVisible && ps.visible;

  // Compute this object's model matrix (always needed for children)
  const auto modelMatrix = computeModelMatrix(ps, parentModelMatrix);

  // Check if this is a container (CSetOfObjects)
  const auto* setOfObjects = dynamic_cast<const mrpt::viz::CSetOfObjects*>(obj.get());
  if (setOfObjects)
  {
    // NOTE: version is NOT updated here; it's deferred to post-walk
    // in updateDirtyObjects() for multi-occurrence consistency.

    // Recurse into children, propagating dirty and visibility
    for (const auto& childObj : *setOfObjects)
    {
      updateDirtyObjectRecursive(childObj, modelMatrix, dirty, effectiveVisible, stats);
    }
    return;
  }

  // Leaf object: update the proxy group for THIS occurrence.
  // The occurrence counter matches tree walk order to proxy groups.
  auto proxyIt = m_objectToProxy.find(weakObj);
  if (proxyIt != m_objectToProxy.end())
  {
    size_t& occIdx = m_updateOccurrenceCounter[weakObj];
    auto& allOccurrences = proxyIt->second;

    if (occIdx < allOccurrences.size())
    {
      auto& occProxies = allOccurrences[occIdx];

      if (dirty)
      {
        // Regenerate viz-side buffers only once per dirty object
        // (not per occurrence — the data is the same)
        if (selfDirty && occIdx == 0)
        {
          obj->updateBuffers();
        }

        for (auto& proxy : occProxies)
        {
          proxy->m_modelMatrix = modelMatrix;
          proxy->m_visible = effectiveVisible;
          if (selfDirty)
          {
            proxy->updateBuffers(obj.get());
          }
        }

        stats.numObjectsUpdated++;

        if (SCENE_VERBOSE)
        {
          std::cout << "[CompiledScene::updateDirtyObjects] Updated: " << obj->getName()
                    << " (occurrence " << occIdx << ")\n";
        }
      }
    }

    occIdx++;
  }

  // Handle composite objects' internal children (e.g. CAxis text labels)
  if (obj->isCompositeObject())
  {
    const auto& children = obj->getInternalChildren();
    for (const auto& child : children)
    {
      if (!child)
      {
        continue;
      }
      updateDirtyObjectRecursive(child, modelMatrix, dirty, effectiveVisible, stats);
    }
  }

  // Handle enableShowName() label
  if (obj->isShowNameEnabled())
  {
    auto labelPtr = obj->labelObjectPtr();
    if (labelPtr)
    {
      // Keep label text in sync with object name
      labelPtr->setString(obj->getName());

      // Update existing label proxy, or compile a new one
      if (hasProxyFor(labelPtr))
      {
        auto labelVersionIt =
            m_objectVersions.find(std::weak_ptr<mrpt::viz::CVisualObject>(labelPtr));
        const uint64_t labelLastVer =
            (labelVersionIt != m_objectVersions.end()) ? labelVersionIt->second : 0;
        if (dirty || labelPtr->dataVersion() != labelLastVer)
        {
          updateDirtyObjectRecursive(labelPtr, modelMatrix, true, effectiveVisible, stats);
        }
      }
      else
      {
        // Label not yet compiled — find the viewport and compile it
        for (auto& [vpName, viewport] : m_viewports)
        {
          compileObject(labelPtr, *viewport, stats, modelMatrix);
          break;  // labels only need one viewport
        }
      }
    }
  }
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
  m_viewportRenderOrder.clear();
  m_objectToProxy.clear();
  m_objectVersions.clear();
  m_updateOccurrenceCounter.clear();
  m_shaderManager.clear();
  m_sourceScene = nullptr;
  m_isCompiled = false;

  MRPT_END
}

bool CompiledScene::hasPendingUpdates() const
{
  // Check if any tracked object has changed since we last compiled it
  for (const auto& [weakObj, ver] : m_objectVersions)
  {
    auto obj = weakObj.lock();
    if (!obj) continue;
    if (obj->dataVersion() != ver)
    {
      return true;
    }
  }

  // Also check if source scene has new objects we haven't compiled yet
  // (This is a quick check - actual detection happens in compileNewObjects)
  return false;
}

size_t CompiledScene::getProxyCount() const
{
  size_t count = 0;
  for (const auto& [_, occurrences] : m_objectToProxy)
  {
    for (const auto& occ : occurrences)
    {
      count += occ.size();
    }
  }
  return count;
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

  // Render all viewports in insertion order (so overlay viewports
  // render after the main viewport, not alphabetically).
  for (const auto& name : m_viewportRenderOrder)
  {
    auto it = m_viewports.find(name);
    if (it == m_viewports.end()) continue;
    auto& viewport = it->second;

    // For cloned viewports, resolve the source viewport
    const CompiledViewport* sourceVp = nullptr;
    if (viewport->isCloningObjects())
    {
      auto srcIt = m_viewports.find(viewport->getClonedViewportName());
      if (srcIt != m_viewports.end())
      {
        sourceVp = srcIt->second.get();

        // If also cloning the camera, copy it from the source viewport.
        // The cloned viewport will use this camera with its own viewport
        // dimensions for matrix computation.
        if (viewport->isCloningCamera())
        {
          const auto& srcVizVpName = viewport->getClonedViewportName();
          auto srcVizVp = m_sourceScene->getViewport(srcVizVpName);
          if (srcVizVp)
          {
            viewport->updateCamera(srcVizVp->getCamera());
            viewport->forceMatrixUpdate();
          }
        }
      }
    }
    viewport->render(
        renderWidth, renderHeight, renderOffsetX, renderOffsetY, m_shaderManager, sourceVp);
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