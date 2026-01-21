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

  // Store reference to source scene (shallow copy to keep it alive)
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
    compileObject(obj.get(), compiledViewport, stats);
  }

  MRPT_END
}

void CompiledScene::compileObject(
    const mrpt::viz::CVisualObject* obj,
    CompiledViewport& compiledViewport,
    CompilationStats& stats)
{
  MRPT_START

  if (!obj) return;

  // Check if this object is a composite (has children)
  if (obj->isCompositeObject())
  {
    // For composite objects (like CSetOfObjects), recurse into children
    const auto* setOfObjs = dynamic_cast<const CSetOfObjects*>(obj);
    if (setOfObjs)
    {
      for (const auto& child : *setOfObjs)
      {
        if (child)
        {
          stats.numObjectsTotal++;
          compileObject(child.get(), compiledViewport, stats);
        }
      }
    }
    return;  // Don't create proxy for the container itself
  }

  // Create proxy for this object
  auto proxy = createProxyByType(obj);

  if (!proxy)
  {
    // This object type doesn't need GPU resources (e.g., camera objects)
    return;
  }

  // Register the proxy in our tracking maps
  registerProxy(obj, proxy);

  // Perform initial compilation (upload to GPU)
  proxy->compile(obj);

  // Add proxy to the viewport's rendering structures
  compiledViewport.addProxy(proxy, obj);

  stats.numObjectsCompiled++;
  stats.numProxiesCreated++;

  if (COMPILED_SCENE_VERBOSE)
  {
    std::cout << "[CompiledScene::compileObject] Created proxy for: "
              << obj->GetRuntimeClass()->className;
    if (!obj->getName().empty()) std::cout << " ('" << obj->getName() << "')";
    std::cout << "\n";
  }

  MRPT_END
}

RenderableProxy::Ptr CompiledScene::createProxyByType(const mrpt::viz::CVisualObject* obj)
{
  MRPT_START

  if (!obj) return nullptr;

  // Determine which shaders this object uses
  const auto requiredShaders = obj->requiredShaders();

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
                << obj->GetRuntimeClass()->className << "\n";
      return nullptr;
  }

  MRPT_END
}

void CompiledScene::registerProxy(const mrpt::viz::CVisualObject* obj, RenderableProxy::Ptr proxy)
{
  MRPT_START

  ASSERT_(obj != nullptr);
  ASSERT_(proxy != nullptr);

  // Add to both tracking maps
  m_objectToProxy[obj] = proxy;
  m_proxyToObject[proxy.get()] = obj;

  MRPT_END
}

void CompiledScene::unregisterProxy(const mrpt::viz::CVisualObject* obj)
{
  MRPT_START

  auto it = m_objectToProxy.find(obj);
  if (it != m_objectToProxy.end())
  {
    // Remove from reverse map
    m_proxyToObject.erase(it->second.get());

    // Remove from primary map (this destroys the proxy)
    m_objectToProxy.erase(it);
  }

  MRPT_END
}

RenderableProxy::Ptr CompiledScene::getProxyFor(const mrpt::viz::CVisualObject* obj) const
{
  auto it = m_objectToProxy.find(obj);
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

  // Check each tracked object for changes
  for (auto& [obj, proxy] : m_objectToProxy)
  {
    if (needsUpdate(obj))
    {
      // Recompile this object's GPU data
      proxy->updateBuffers(obj);

      localStats.numObjectsRecompiled++;
      anyUpdates = true;

      if (COMPILED_SCENE_VERBOSE)
      {
        std::cout << "[CompiledScene::updateIfNeeded] Updated: "
                  << obj->GetRuntimeClass()->className;
        if (!obj->getName().empty()) std::cout << " ('" << obj->getName() << "')";
        std::cout << "\n";
      }
    }
  }

  // Update viewport cameras/lighting if needed
  for (auto& [name, compiledVp] : m_viewports)
  {
    if (compiledVp->updateIfNeeded())
    {
      anyUpdates = true;
    }
  }

  // Clean up proxies for objects that have been removed from the scene
  cleanupOrphanedProxies(localStats);

  if (anyUpdates)
  {
    m_compilationVersion++;
  }

  m_lastStats = localStats;
  if (stats) *stats = localStats;

  return anyUpdates;

  MRPT_END
}

bool CompiledScene::needsUpdate(const mrpt::viz::CVisualObject* obj) const
{
  MRPT_START

  if (!obj) return false;

  // Check the object's dirty flag
  return obj->hasToUpdateBuffers();

  MRPT_END
}

void CompiledScene::cleanupOrphanedProxies(CompilationStats& stats)
{
  MRPT_START

  // This would require the Scene to notify us when objects are removed.
  // For now, we keep all proxies alive until clear() is called.
  //
  // Future enhancement: Scene could maintain a list of removed objects,
  // or we could periodically scan the scene to detect deletions.

  // TODO: Implement orphan detection and cleanup

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

  // Auto-update if enabled
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

  // Clear proxy tracking maps (this destroys all proxies)
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

  // Check if any tracked object is dirty
  for (const auto& [obj, proxy] : m_objectToProxy)
  {
    if (needsUpdate(obj)) return true;
  }

  // Check viewports
  for (const auto& [name, compiledVp] : m_viewports)
  {
    if (compiledVp->hasPendingUpdates()) return true;
  }

  return false;

  MRPT_END
}

RenderableProxy::Ptr CompiledScene::createProxyFor(const mrpt::viz::CVisualObject* obj)
{
  MRPT_START
  checkContextThread();

  auto proxy = createProxyByType(obj);

  if (proxy)
  {
    registerProxy(obj, proxy);
    proxy->compile(obj);
  }

  return proxy;

  MRPT_END
}