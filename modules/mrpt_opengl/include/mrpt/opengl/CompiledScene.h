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
#include <thread>

namespace mrpt::opengl
{
/** Statistics collected during scene compilation and rendering.
 * \ingroup mrpt_opengl_grp
 */
struct CompilationStats
{
  size_t numObjectsTotal = 0;
  size_t numObjectsCompiled = 0;
  size_t numObjectsUpdated = 0;
  size_t numProxiesCreated = 0;
  size_t numProxiesDeleted = 0;
  size_t numOrphanedProxies = 0;  //!< Proxies removed due to deleted source objects
  size_t numNewObjects = 0;       //!< New objects added since last compile

  void reset()
  {
    numObjectsTotal = 0;
    numObjectsCompiled = 0;
    numObjectsUpdated = 0;
    numProxiesCreated = 0;
    numProxiesDeleted = 0;
    numOrphanedProxies = 0;
    numNewObjects = 0;
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
 * - Dynamic object support: detects newly added objects in the source Scene
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
 *   scene.insert(newObject);  // dynamically add objects
 *   compiled.updateIfNeeded(); // compiles new objects, updates changed ones
 *   compiled.render();
 * }
 * \endcode
 *
 * Thread safety:
 * - All CompiledScene methods must be called from the OpenGL context thread
 * - Source viz objects may be modified from other threads (they use shared_mutex)
 * - The proxy will read from viz objects during compile/update (acquires read lock)
 *
 * \sa CompiledViewport, RenderableProxy, mrpt::viz::Scene
 * \ingroup mrpt_opengl_grp
 */
class CompiledScene
{
 public:
  using Ptr = std::shared_ptr<CompiledScene>;

  CompiledScene();
  ~CompiledScene() = default;

  // Non-copyable, non-movable (owns GPU resources)
  CompiledScene(const CompiledScene&) = delete;
  CompiledScene& operator=(const CompiledScene&) = delete;
  CompiledScene(CompiledScene&&) = delete;
  CompiledScene& operator=(CompiledScene&&) = delete;

  /** @name Compilation and Updates
   * @{ */

  /** Performs initial compilation of the entire scene.
   *
   * This creates RenderableProxy instances for all CVisualObject instances
   * in the scene, uploads data to GPU, and prepares all necessary OpenGL
   * state.
   *
   * \param scene The abstract scene to compile (reference kept internally)
   * \param stats Optional pointer to receive compilation statistics
   * \throws std::runtime_error if OpenGL context is not available
   *
   * \note This must be called from a thread with an active OpenGL context.
   * \note Calling compile() multiple times will clear previous compilation
   *       and start fresh.
   */
  void compile(const mrpt::viz::Scene& scene, CompilationStats* stats = nullptr);

  /** Incrementally updates the compiled scene to match the source Scene.
   *
   * This method:
   * 1. Removes proxies for deleted source objects (via weak_ptr expiration)
   * 2. Creates proxies for newly added objects in the source Scene
   * 3. Updates GPU buffers for objects whose dirty flag is set
   *
   * Much more efficient than full recompilation for animated/dynamic scenes.
   *
   * \param stats Optional pointer to receive update statistics
   * \return true if any updates were performed, false if nothing changed
   *
   * \note This is called automatically by render() if auto-update is enabled.
   * \note The source Scene is re-queried each time to detect new objects.
   */
  bool updateIfNeeded(CompilationStats* stats = nullptr);

  /** Forces a full recompilation of the entire scene.
   *
   * Clears all existing proxies and recompiles from scratch.
   * Use sparingly - updateIfNeeded() is usually sufficient.
   */
  void recompile();

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
  [[nodiscard]] bool getAutoUpdate() const { return m_autoUpdate; }

  /** @} */

  /** @name Status Queries
   * @{ */

  /** Returns true if the scene has been compiled at least once */
  [[nodiscard]] bool isCompiled() const { return m_isCompiled; }

  /** Returns true if there are pending changes that need updating */
  [[nodiscard]] bool hasPendingUpdates() const;

  /** Number of viewports in the compiled scene */
  [[nodiscard]] size_t getViewportCount() const { return m_viewports.size(); }

  /** Number of total RenderableProxy objects */
  [[nodiscard]] size_t getProxyCount() const { return m_objectToProxy.size(); }

  /** Returns the source Scene that was compiled.
   * \return nullptr if not yet compiled
   */
  [[nodiscard]] const mrpt::viz::Scene* getSourceScene() const { return m_sourceScene.get(); }

  /** Access to compiled viewports */
  [[nodiscard]] const std::map<std::string, CompiledViewport::Ptr>& getViewports() const
  {
    return m_viewports;
  }

  /** Access to a specific compiled viewport */
  [[nodiscard]] CompiledViewport::Ptr getViewport(const std::string& name) const
  {
    auto it = m_viewports.find(name);
    return it != m_viewports.end() ? it->second : nullptr;
  }

  /** Access to the shader program manager */
  [[nodiscard]] ShaderProgramManager& shaderManager() { return m_shaderManager; }
  [[nodiscard]] const ShaderProgramManager& shaderManager() const { return m_shaderManager; }

  /** Access to last compilation statistics */
  [[nodiscard]] const CompilationStats& lastStats() const { return m_lastStats; }

  /** @} */

 private:
  /** Reference to the source scene (kept for incremental updates) */
  std::shared_ptr<const mrpt::viz::Scene> m_sourceScene;

  /** Compiled viewports, indexed by name */
  std::map<std::string, CompiledViewport::Ptr> m_viewports;

  /** Mapping: weak_ptr<CVisualObject> -> RenderableProxy
   * Using weak_ptr allows detection of deleted source objects.
   * This is the core tracking structure for incremental updates.
   */
  std::map<
      std::weak_ptr<mrpt::viz::CVisualObject>,
      RenderableProxy::Ptr,
      std::owner_less<std::weak_ptr<mrpt::viz::CVisualObject>>>
      m_objectToProxy;

  /** Centralized shader program management */
  ShaderProgramManager m_shaderManager;

  /** Compilation state flags */
  bool m_isCompiled = false;
  bool m_autoUpdate = true;

  /** Statistics from last compilation/update */
  CompilationStats m_lastStats;

  /** Thread ID of the OpenGL context owner.
   * All operations must happen on this thread.
   */
  std::thread::id m_contextThread;

  /** @name Internal Compilation Helpers
   * @{ */

  /** Compiles a viewport and all its objects */
  void compileViewport(
      const mrpt::viz::Viewport& vizViewport,
      CompiledViewport& compiledViewport,
      CompilationStats& stats);

  /** Compiles an object and its children (for CSetOfObjects) */
  void compileObject(
      const std::shared_ptr<mrpt::viz::CVisualObject>& obj,
      CompiledViewport& compiledViewport,
      CompilationStats& stats);

  /** Checks if we already have a proxy for this object */
  [[nodiscard]] bool hasProxyFor(const std::shared_ptr<mrpt::viz::CVisualObject>& obj) const;

  /** Creates appropriate proxy type based on object's parameter mixins */
  [[nodiscard]] RenderableProxy::Ptr createProxyByType(
      const std::shared_ptr<mrpt::viz::CVisualObject>& obj);

  /** Cleans up proxies for objects that no longer exist */
  void cleanupOrphanedProxies(CompilationStats& stats);

  /** Scans source scene for new objects not yet compiled */
  void compileNewObjects(CompilationStats& stats);

  /** Updates GPU buffers for objects with dirty flags */
  void updateDirtyObjects(CompilationStats& stats);

  /** Validates that we're being called from the correct thread */
  void checkContextThread() const;

  /** @} */
};

}  // namespace mrpt::opengl