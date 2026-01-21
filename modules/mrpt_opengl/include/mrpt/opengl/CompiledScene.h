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
#pragma once

#include <mrpt/opengl/CompiledViewport.h>
#include <mrpt/opengl/RenderableProxy.h>
#include <mrpt/opengl/ShaderProgramManager.h>
#include <mrpt/viz/Scene.h>

#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

namespace mrpt::opengl
{
/** Statistics collected during scene compilation and rendering.
 * \ingroup mrpt_opengl_grp
 */
struct CompilationStats
{
  size_t numObjectsTotal = 0;
  size_t numObjectsCompiled = 0;
  size_t numObjectsRecompiled = 0;
  size_t numProxiesCreated = 0;
  size_t numProxiesDeleted = 0;

  void reset()
  {
    numObjectsTotal = 0;
    numObjectsCompiled = 0;
    numObjectsRecompiled = 0;
    numProxiesCreated = 0;
    numProxiesDeleted = 0;
  }
};

/** A compiled, GPU-ready representation of a mrpt::viz::Scene.
 *
 * This class bridges the gap between the abstract scene graph (mrpt::viz::Scene)
 * and the actual OpenGL rendering. It maintains the mapping between
 * CVisualObject instances and their corresponding GPU-side RenderableProxy
 * objects.
 *
 * Key responsibilities:
 * - Initial compilation: translates the entire Scene into GPU structures
 * - Incremental updates: detects changes via dirty flags and recompiles only
 *   what's needed
 * - Resource management: owns all RenderableProxy instances and shader programs
 * - Rendering orchestration: delegates to CompiledViewport instances
 *
 * Typical usage:
 * \code
 * viz::Scene scene;
 * // ... populate scene with objects ...
 *
 * opengl::CompiledScene compiled;
 * compiled.compile(scene);  // Initial compilation
 *
 * // Render loop:
 * while (running) {
 *   myObject->setColor(...);  // triggers dirty flag internally
 *   compiled.updateIfNeeded(); // only recompiles changed objects
 *   compiled.render();
 * }
 * \endcode
 *
 * Thread safety: This class is NOT thread-safe. All operations must be
 * called from the same thread that owns the OpenGL context.
 *
 * \sa CompiledViewport, RenderableProxy, mrpt::viz::Scene
 * \ingroup mrpt_opengl_grp
 */
class CompiledScene
{
 public:
  using Ptr = std::shared_ptr<CompiledScene>;

  CompiledScene();
  ~CompiledScene();

  // Disable copy/move for now (could implement later if needed)
  CompiledScene(const CompiledScene&) = delete;
  CompiledScene& operator=(const CompiledScene&) = delete;
  CompiledScene(CompiledScene&&) = delete;
  CompiledScene& operator=(CompiledScene&&) = delete;
}
  /** @name Compilation and Updates
   * @{ */

  /** Performs initial compilation of the entire scene.
   *
   * This creates RenderableProxy instances for all CVisualObject instances
   * in the scene, uploads data to GPU, and prepares all necessary OpenGL
   * state.
   *
   * \param scene The abstract scene to compile
   * \param stats Optional pointer to receive compilation statistics
   * \throws std::runtime_error if OpenGL context is not available
   *
   * \note This must be called from a thread with an active OpenGL context.
   * \note Calling compile() multiple times will clear previous compilation
   *       and start fresh.
   */
  void compile(const mrpt::viz::Scene& scene, CompilationStats* stats = nullptr);

/** Incrementally updates only the parts of the scene that have changed.
 *
 * This checks dirty flags on all tracked CVisualObject instances and
 * recompiles only those that have been modified since the last render.
 * Much more efficient than full recompilation for animated scenes.
 *
 * \param stats Optional pointer to receive update statistics
 * \return true if any updates were performed, false if nothing changed
 *
 * \note This is called automatically by render() if auto-update is enabled.
 */
bool updateIfNeeded(CompilationStats* stats = nullptr);

/** Forces a full recompilation of the entire scene.
 *
 * Equivalent to calling compile() again with the same scene.
 * Useful if you want to ensure everything is fresh, though normally
 * updateIfNeeded() is sufficient.
 */
void forceRecompile();

/** Clears all compiled data and GPU resources.
 *
 * After calling this, you must call compile() again before rendering.
 */
void clear();

/** @} */

/** @name Rendering
 * @{ */

/** Renders all viewports in the compiled scene.
 *
 * \param renderWidth Width of the render target in pixels
 * \param renderHeight Height of the render target in pixels
 * \param renderOffsetX X offset for viewport positioning
 * \param renderOffsetY Y offset for viewport positioning
 *
 * If auto-update is enabled (default), this automatically calls
 * updateIfNeeded() before rendering.
 */
void render(
    int renderWidth = 0, int renderHeight = 0, int renderOffsetX = 0, int renderOffsetY = 0);

/** Renders a specific viewport by name.
 *
 * \throws std::runtime_error if viewport name not found
 */
void renderViewport(
    const std::string& viewportName,
    int renderWidth = 0,
    int renderHeight = 0,
    int renderOffsetX = 0,
    int renderOffsetY = 0);

/** @} */

/** @name Configuration
 * @{ */

/** Enable/disable automatic update before each render().
 *
 * Default: true. When enabled, render() automatically calls updateIfNeeded()
 * to ensure GPU state matches the scene.
 *
 * Set to false if you want manual control over when updates happen.
 */
void setAutoUpdate(bool enable) { m_autoUpdate = enable; }

/** Returns current auto-update setting */
bool getAutoUpdate() const { return m_autoUpdate; }

/** Enable/disable asynchronous compilation (experimental).
 *
 * When enabled, updateIfNeeded() spawns a background thread to perform
 * GPU buffer updates. You must call waitForAsyncUpdate() before rendering.
 *
 * Default: false
 * \note Not yet implemented in this version
 */
void setAsyncUpdates(bool enable) { m_asyncUpdates = enable; }

/** @} */

/** @name Status Queries
 * @{ */

/** Returns true if the scene has been compiled at least once */
bool isCompiled() const { return m_isCompiled; }

/** Returns true if there are pending changes that need updating */
bool hasPendingUpdates() const;

/** Number of viewports in the compiled scene */
size_t getViewportCount() const { return m_viewports.size(); }

/** Number of total RenderableProxy objects */
size_t getProxyCount() const { return m_objectToProxy.size(); }

/** Returns the source Scene that was compiled.
 * \return nullptr if not yet compiled
 */
const mrpt::viz::Scene* getSourceScene() const { return m_sourceScene.get(); }

/** Access to compiled viewports */
const std::map<std::string, CompiledViewport::Ptr>& getViewports() const { return m_viewports; }

/** Access to a specific compiled viewport */
CompiledViewport::Ptr getViewport(const std::string& name) const
{
  auto it = m_viewports.find(name);
  return it != m_viewports.end() ? it->second : nullptr;
}

/** Access to the shader program manager */
ShaderProgramManager& shaderManager() { return m_shaderManager; }
const ShaderProgramManager& shaderManager() const { return m_shaderManager; }

/** Access to last compilation statistics */
const CompilationStats& lastStats() const { return m_lastStats; }

/** @} */

/** @name Advanced: Manual Proxy Management
 * @{ */

/** Creates a proxy for a specific visual object without full scene compilation.
 *
 * Advanced users can manually manage individual proxies. Normally you don't
 * need this as compile() handles everything.
 *
 * \return The created proxy, or nullptr if object type not supported
 */
RenderableProxy::Ptr createProxyFor(const mrpt::viz::CVisualObject* obj);

/** Manually register a proxy for tracking.
 *
 * Used internally by compile() and createProxyFor().
 */
void registerProxy(const mrpt::viz::CVisualObject* obj, RenderableProxy::Ptr proxy);

/** Manually unregister and delete a proxy. */
void unregisterProxy(const mrpt::viz::CVisualObject* obj);

/** Retrieve the proxy associated with a visual object.
 * \return nullptr if object has no proxy
 */
RenderableProxy::Ptr getProxyFor(const mrpt::viz::CVisualObject* obj) const;

/** @} */

private:
/** Reference to the source scene (shallow copy) */
std::shared_ptr<mrpt::viz::Scene> m_sourceScene;

/** Compiled viewports, indexed by name */
std::map<std::string, CompiledViewport::Ptr> m_viewports;

/** Mapping: CVisualObject* -> RenderableProxy
 * This is the core tracking structure for incremental updates.
 */
std::unordered_map<const mrpt::viz::CVisualObject*, RenderableProxy::Ptr> m_objectToProxy;

/** Reverse mapping: RenderableProxy* -> CVisualObject*
 * Used for cleanup and debugging.
 */
std::unordered_map<RenderableProxy*, const mrpt::viz::CVisualObject*> m_proxyToObject;

/** Centralized shader program management */
ShaderProgramManager m_shaderManager;

/** Compilation state flags */
bool m_isCompiled = false;
bool m_autoUpdate = true;
bool m_asyncUpdates = false;

/** Statistics from last compilation/update */
CompilationStats m_lastStats;

/** Version counter for detecting scene changes.
 * Incremented on each compile/update.
 */
uint64_t m_compilationVersion = 0;

/** Thread ID of the OpenGL context owner.
 * All operations must happen on this thread.
 */
std::thread::id m_contextThread;

/** @name Internal Compilation Helpers
 * @{ */

/** Recursively compiles a viewport and all its objects */
void compileViewport(
    const mrpt::viz::Viewport& vizViewport,
    CompiledViewport& compiledViewport,
    CompilationStats& stats);

/** Recursively compiles an object and its children */
void compileObject(
    const mrpt::viz::CVisualObject* obj,
    CompiledViewport& compiledViewport,
    CompilationStats& stats);

/** Determines if an object needs recompilation based on dirty flags */
bool needsUpdate(const mrpt::viz::CVisualObject* obj) const;

/** Creates appropriate proxy type based on object's shader requirements */
RenderableProxy::Ptr createProxyByType(const mrpt::viz::CVisualObject* obj);

/** Cleans up proxies for objects that no longer exist */
void cleanupOrphanedProxies(CompilationStats& stats);

/** Validates that we're being called from the correct thread */
void checkContextThread() const;

/** @} */

;

}  // namespace mrpt::opengl