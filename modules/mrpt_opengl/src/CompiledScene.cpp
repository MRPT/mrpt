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
#include <mrpt/opengl/CompiledScene.h>
#include <mrpt/opengl/LinesProxy.h>
#include <mrpt/opengl/PointsProxy.h>
#include <mrpt/opengl/TexturedTrianglesProxy.h>
#include <mrpt/opengl/TrianglesProxy.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <iostream>

using namespace mrpt::opengl;
using namespace mrpt::viz;

static const bool SCENE_VERBOSE = mrpt::get_env<bool>("MRPT_SCENE_VERBOSE", false);

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

  // Handle cloned viewport mode
  if (vizViewport.isClonedCamera())
  {
    compiledViewport.setCloneMode(vizViewport.isClonedCameraFrom(), vizViewport.isClonedCamera());
    return;
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

void CompiledScene::compileObject(
    const std::shared_ptr<CVisualObject>& obj,
    CompiledViewport& compiledViewport,
    CompilationStats& stats)
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

  // Check if this is a container (CSetOfObjects)
  const auto* setOfObjects = dynamic_cast<const CSetOfObjects*>(obj.get());
  if (setOfObjects)
  {
    // Recursively compile children
    for (auto it = setOfObjects->begin(); it != setOfObjects->end(); ++it)
    {
      compileObject(*it, compiledViewport, stats);
    }
    return;
  }

  // Skip if we already have a proxy for this object
  if (hasProxyFor(obj))
  {
    return;
  }

  // Create proxy for this object
  auto proxy = createProxyByType(obj);
  if (!proxy)
  {
    if (SCENE_VERBOSE)
    {
      std::cout << "[CompiledScene::compileObject] No proxy type for: "
                << obj->GetRuntimeClass()->className << "\n";
    }
    return;
  }

  obj->updateBuffers();  // Populate viz buffers first

  // Set the source object reference in the proxy
  proxy->setSourceObject(obj);

  // Compile the proxy (upload data to GPU)
  proxy->compile(obj.get());

  // Clear dirty flag after successful compile
  obj->clearChangedFlag();

  // Register in our tracking map
  std::weak_ptr<CVisualObject> weakObj = obj;
  m_objectToProxy[weakObj] = proxy;

  // Add to viewport
  compiledViewport.addProxy(proxy, obj);

  stats.numObjectsCompiled++;
  stats.numProxiesCreated++;

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

RenderableProxy::Ptr CompiledScene::createProxyByType(const std::shared_ptr<CVisualObject>& obj)
{
  if (!obj)
  {
    return nullptr;
  }

  // Check for textured triangles first (more specific than plain triangles)
  if (dynamic_cast<const VisualObjectParams_TexturedTriangles*>(obj.get()))
  {
    return std::make_shared<TexturedTrianglesProxy>();
  }

  // Check for triangles
  if (dynamic_cast<const VisualObjectParams_Triangles*>(obj.get()))
  {
    return std::make_shared<TrianglesProxy>();
  }

  // Check for points
  if (dynamic_cast<const VisualObjectParams_Points*>(obj.get()))
  {
    return std::make_shared<PointsProxy>();
  }

  // Check for lines
  if (dynamic_cast<const VisualObjectParams_Lines*>(obj.get()))
  {
    return std::make_shared<LinesProxy>();
  }

  // Unknown type - return null
  return nullptr;
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
      proxy->updateBuffers(obj.get());
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