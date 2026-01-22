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
#include <mrpt/opengl/TextProxy.h>
#include <mrpt/opengl/TexturedTrianglesProxy.h>
#include <mrpt/opengl/TrianglesProxy.h>
#include <mrpt/viz/CSetOfObjects.h>

#include <iostream>

using namespace mrpt::opengl;
using namespace mrpt::viz;

// Enable debug output
static const bool COMPILED_SCENE_VERBOSE =
    mrpt::get_env<bool>("MRPT_COMPILED_SCENE_VERBOSE", false);

CompiledScene::CompiledScene() { m_contextThread = std::this_thread::get_id(); }

CompiledScene::~CompiledScene()
{
  try
  {
    clear();
  }
  catch (const std::exception& e)
  {
    std::cerr << "[CompiledScene::~CompiledScene] Exception during cleanup: " << e.what() << "\n";
  }
}

void CompiledScene::checkContextThread() const
{
  if (std::this_thread::get_id() != m_contextThread)
  {
    THROW_EXCEPTION(
        "CompiledScene operations must be called from the same thread that "
        "created it (the OpenGL context thread).");
  }
}

void CompiledScene::compile(const mrpt::viz::Scene& scene, CompilationStats* stats)
{
  MRPT_START
  checkContextThread();

  CompilationStats localStats;
  localStats.reset();

  if (COMPILED_SCENE_VERBOSE)
    std::cout << "[CompiledScene::compile] Starting full compilation...\n";

  // Clear any previous compilation
  clear();

  // Store reference to source scene (strong reference to keep it alive)
  m_sourceScene = std::make_shared<mrpt::viz::Scene>(scene);

  // Compile each viewport
  for (const auto& vizViewport : m_sourceScene->getViewports())
  {
    auto compiledVp = std::make_shared<CompiledViewport>(vizViewport->getName());

    compileViewport(*vizViewport, *compiledVp, localStats);

    m_viewports[vizViewport->getName()] = compiledVp;
  }

  m_isCompiled = true;
  m_compilationVersion++;
  m_lastStats = localStats;

  if (stats) *stats = localStats;

  if (COMPILED_SCENE_VERBOSE)
  {
    std::cout << "[CompiledScene::compile] Compilation complete:\n"
              << "  - Viewports: " << m_viewports.size() << "\n"
              << "  - Total objects: " << localStats.numObjectsTotal << "\n"
              << "  - Proxies created: " << localStats.numProxiesCreated << "\n";
  }

  MRPT_END
}

void CompiledScene::compileViewport(
    const mrpt::viz::Viewport& vizViewport,
    CompiledViewport& compiledViewport,
    CompilationStats& stats)
{
  MRPT_START

  // Copy viewport configuration (camera, lighting, etc.)
  compiledViewport.updateFromVizViewport(vizViewport);

  // Compile all objects in this viewport
  for (const auto& obj : vizViewport)
  {
    if (!obj) continue;

    stats.numObjectsTotal++;
    compileObject(obj, compiledViewport, stats);
  }

  MRPT_END
}

void CompiledScene::compileObject(
    const std::shared_ptr<mrpt::viz::CVisualObject>& objPtr,
    CompiledViewport& compiledViewport,
    CompilationStats& stats)
{
  MRPT_START

  if (!objPtr) return;

  // Check if this object is a composite (has children)
  if (objPtr->isCompositeObject())
  {
    // For composite objects (like CSetOfObjects), recurse into children
    const auto* setOfObjs = dynamic_cast<const CSetOfObjects*>(objPtr.get());
    if (setOfObjs)
    {
      for (const auto& child : *setOfObjs)
      {
        if (child)
        {
          stats.numObjectsTotal++;
          compileObject(child, compiledViewport, stats);
        }
      }
    }
    return;  // Don't create proxy for the container itself
  }

  // Create proxy for this object
  auto proxy = createProxyByType(objPtr);

  if (!proxy)
  {
    // This object type doesn't need GPU resources (e.g., camera objects)
    return;
  }

  // Register the proxy in our tracking maps (using weak_ptr)
  registerProxy(objPtr, proxy);

  // Perform initial compilation (upload to GPU)
  proxy->compile(objPtr.get());

  // Add proxy to the viewport's rendering structures
  compiledViewport.addProxy(proxy, objPtr);

  stats.numObjectsCompiled++;
  stats.numProxiesCreated++;

  if (COMPILED_SCENE_VERBOSE)
  {
    std::cout << "[CompiledScene::compileObject] Created proxy for: "
              << objPtr->GetRuntimeClass()->className;
    if (!objPtr->getName().empty()) std::cout << " ('" << objPtr->getName() << "')";
    std::cout << "\n";
  }

  MRPT_END
}

RenderableProxy::Ptr CompiledScene::createProxyByType(
    const std::shared_ptr<mrpt::viz::CVisualObject>& objPtr)
{
  MRPT_START

  if (!objPtr) return nullptr;

  // Determine which shaders this object uses
  const auto requiredShaders = objPtr->requiredShaders();

  if (requiredShaders.empty() ||
      (requiredShaders.size() == 1 && requiredShaders[0] == DefaultShaderID::NONE))
  {
    // This object doesn't render anything (e.g., CCamera)
    return nullptr;
  }

  // Create appropriate proxy based on the primary shader type
  const auto primaryShader = requiredShaders[0];

  switch (primaryShader)
  {
    case DefaultShaderID::POINTS:
      return std::make_shared<PointsProxy>();

    case DefaultShaderID::WIREFRAME:
      return std::make_shared<LinesProxy>();

    case DefaultShaderID::TRIANGLES_LIGHT:
    case DefaultShaderID::TRIANGLES_NO_LIGHT:
    case DefaultShaderID::TRIANGLES_SHADOW_1ST:
    case DefaultShaderID::TRIANGLES_SHADOW_2ND:
      return std::make_shared<TrianglesProxy>();

    case DefaultShaderID::TEXTURED_TRIANGLES_LIGHT:
    case DefaultShaderID::TEXTURED_TRIANGLES_NO_LIGHT:
    case DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_1ST:
    case DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_2ND:
      return std::make_shared<TexturedTrianglesProxy>();

    case DefaultShaderID::TEXT:
      return std::make_shared<TextProxy>();

    default:
      std::cerr << "[CompiledScene::createProxyByType] Warning: Unknown shader type "
                << static_cast<int>(primaryShader) << " for object "
                << objPtr->GetRuntimeClass()->className << "\n";
      return nullptr;
  }

  MRPT_END
}

void CompiledScene::registerProxy(
    const std::shared_ptr<mrpt::viz::CVisualObject>& objPtr, RenderableProxy::Ptr proxy)
{
  MRPT_START

  ASSERT_(objPtr != nullptr);
  ASSERT_(proxy != nullptr);

  // Create weak_ptr for safe tracking
  std::weak_ptr<mrpt::viz::CVisualObject> objWeak = objPtr;

  // Add to both tracking maps
  m_objectToProxy[objWeak] = proxy;
  m_proxyToObject[proxy.get()] = objWeak;

  MRPT_END
}

void CompiledScene::unregisterProxy(const std::shared_ptr<mrpt::viz::CVisualObject>& objPtr)
{
  MRPT_START

  if (!objPtr) return;

  std::weak_ptr<mrpt::viz::CVisualObject> objWeak = objPtr;

  auto it = m_objectToProxy.find(objWeak);
  if (it != m_objectToProxy.end())
  {
    // Remove from reverse map
    m_proxyToObject.erase(it->second.get());

    // Remove from primary map (this destroys the proxy)
    m_objectToProxy.erase(it);
  }

  MRPT_END
}

RenderableProxy::Ptr CompiledScene::getProxyFor(
    const std::shared_ptr<mrpt::viz::CVisualObject>& objPtr) const
{
  if (!objPtr) return nullptr;

  std::weak_ptr<mrpt::viz::CVisualObject> objWeak = objPtr;
  auto it = m_objectToProxy.find(objWeak);
  return it != m_objectToProxy.end() ? it->second : nullptr;
}

bool CompiledScene::updateIfNeeded(CompilationStats* stats)
{
  MRPT_START
  checkContextThread();

  if (!m_isCompiled)
  {
    THROW_EXCEPTION("Cannot update: scene has not been compiled yet. Call compile() first.");
  }

  CompilationStats localStats;
  localStats.reset();

  bool anyUpdates = false;

  // First pass: Clean up orphaned proxies (objects that were deleted)
  cleanupOrphanedProxies(localStats);

  if (localStats.numOrphanedProxies > 0)
  {
    anyUpdates = true;
  }

  // Second pass: Check each tracked object for changes
  for (auto it = m_objectToProxy.begin(); it != m_objectToProxy.end(); ++it)
  {
    const auto& objWeak = it->first;
    auto& proxy = it->second;

    // Try to lock the weak_ptr to access the object
    if (auto objPtr = objWeak.lock())
    {
      if (needsUpdate(objWeak))
      {
        // Recompile this object's GPU data
        proxy->updateBuffers(objPtr.get());

        localStats.numObjectsRecompiled++;
        anyUpdates = true;

        if (COMPILED_SCENE_VERBOSE)
        {
          std::cout << "[CompiledScene::updateIfNeeded] Updated: "
                    << objPtr->GetRuntimeClass()->className;
          if (!objPtr->getName().empty()) std::cout << " ('" << objPtr->getName() << "')";
          std::cout << "\n";
        }
      }
    }
    // If lock() fails, the object was deleted - will be cleaned up in next
    // cleanupOrphanedProxies() call
  }

  // Update viewport cameras/lighting if needed
  for (auto& [name, compiledVp] : m_viewports)
  {
    if (compiledVp->updateIfNeeded())
    {
      anyUpdates = true;
    }
  }

  if (anyUpdates)
  {
    m_compilationVersion++;
  }

  m_lastStats = localStats;
  if (stats) *stats = localStats;

  return anyUpdates;

  MRPT_END
}

bool CompiledScene::needsUpdate(const std::weak_ptr<mrpt::viz::CVisualObject>& objWeak) const
{
  MRPT_START

  // Try to lock the weak_ptr
  auto objPtr = objWeak.lock();
  if (!objPtr)
  {
    // Object was deleted
    return false;
  }

  // Check the object's dirty flag
  return objPtr->hasToUpdateBuffers();

  MRPT_END
}

void CompiledScene::cleanupOrphanedProxies(CompilationStats& stats)
{
  MRPT_START

  // Find and remove proxies whose source objects have been deleted
  std::vector<std::weak_ptr<mrpt::viz::CVisualObject>> toRemove;

  for (auto it = m_objectToProxy.begin(); it != m_objectToProxy.end();)
  {
    const auto& objWeak = it->first;

    // Try to lock the weak_ptr
    if (objWeak.expired())
    {
      // Object was deleted - mark proxy for removal
      auto& proxy = it->second;

      // Remove from reverse map
      m_proxyToObject.erase(proxy.get());

      // Remove from all viewports
      for (auto& [name, compiledVp] : m_viewports)
      {
        compiledVp->removeProxy(proxy);
      }

      // Remove from primary map
      it = m_objectToProxy.erase(it);

      stats.numOrphanedProxies++;
      stats.numProxiesDeleted++;

      if (COMPILED_SCENE_VERBOSE)
      {
        std::cout << "[CompiledScene::cleanupOrphanedProxies] Removed orphaned proxy\n";
      }
    }
    else
    {
      ++it;
    }
  }

  MRPT_END
}

size_t CompiledScene::getOrphanedProxyCount() const
{
  MRPT_START

  size_t count = 0;

  for (const auto& [objWeak, proxy] : m_objectToProxy)
  {
    if (objWeak.expired())
    {
      count++;
    }
  }

  return count;

  MRPT_END
}

void CompiledScene::render(int renderWidth, int renderHeight, int renderOffsetX, int renderOffsetY)
{
  MRPT_START
  checkContextThread();

  if (!m_isCompiled)
  {
    THROW_EXCEPTION("Cannot render: scene has not been compiled yet. Call compile() first.");
  }

  // Auto-update if enabled (includes orphan cleanup)
  if (m_autoUpdate)
  {
    updateIfNeeded();
  }

  // Render each viewport
  for (auto& [name, compiledVp] : m_viewports)
  {
    compiledVp->render(renderWidth, renderHeight, renderOffsetX, renderOffsetY, m_shaderManager);
  }

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
  checkContextThread();

  auto it = m_viewports.find(viewportName);
  if (it == m_viewports.end())
  {
    THROW_EXCEPTION_FMT("Viewport '%s' not found in compiled scene.", viewportName.c_str());
  }

  if (m_autoUpdate)
  {
    updateIfNeeded();
  }

  it->second->render(renderWidth, renderHeight, renderOffsetX, renderOffsetY, m_shaderManager);

  MRPT_END
}

void CompiledScene::forceRecompile()
{
  MRPT_START

  if (!m_sourceScene)
  {
    THROW_EXCEPTION("Cannot recompile: no source scene available.");
  }

  compile(*m_sourceScene);

  MRPT_END
}

void CompiledScene::clear()
{
  MRPT_START
  checkContextThread();

  if (COMPILED_SCENE_VERBOSE && !m_objectToProxy.empty())
  {
    std::cout << "[CompiledScene::clear] Clearing " << m_objectToProxy.size() << " proxies...\n";
  }

  // Clear all viewports (this also releases their GPU resources)
  m_viewports.clear();
  // Clear proxy tracking maps (this destroys all proxies and releases weak_ptrs)
  m_objectToProxy.clear();
  m_proxyToObject.clear();
  // Release source scene reference
  m_sourceScene.reset();
  // Reset state
  m_isCompiled = false;
  m_lastStats.reset();
  if (COMPILED_SCENE_VERBOSE)
  {
    std::cout << "[CompiledScene::clear] Clear complete.\n";
  }
  MRPT_END
}
bool CompiledScene::hasPendingUpdates() const
{
  MRPT_START
  if (!m_isCompiled) return false;
  // Check if any tracked object is dirty (skip expired weak_ptrs)
  for (const auto& [objWeak, proxy] : m_objectToProxy)
  {
    if (needsUpdate(objWeak)) return true;
  }
  // Check for orphaned proxies
  if (getOrphanedProxyCount() > 0) return true;
  // Check viewports
  for (const auto& [name, compiledVp] : m_viewports)
  {
    if (compiledVp->hasPendingUpdates()) return true;
  }
  return false;
  MRPT_END
}
RenderableProxy::Ptr CompiledScene::createProxyFor(
    const std::shared_ptrmrpt::viz::CVisualObject& objPtr)
{
  MRPT_START
  checkContextThread();
  if (!objPtr) return nullptr;
  auto proxy = createProxyByType(objPtr);
  if (proxy)
  {
    registerProxy(objPtr, proxy);
    proxy->compile(objPtr.get());
  }
  return proxy;
  MRPT_END
}