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
#include <vector>

namespace mrpt::opengl
{
/** Statistics collected during scene compilation and rendering.
 * \ingroup mrpt_opengl_grp
 */
struct CompilationStats
{
  size_t numObjectsTotal = 0;     //!< Objects (positions in the scene graph) visited
  size_t numObjectsCompiled = 0;  //!< Objects whose proxies were created
  size_t numObjectsUpdated = 0;   //!< Objects whose buffers or transforms were updated
  size_t numProxiesCreated = 0;
  size_t numProxiesDeleted = 0;
  size_t numOrphanedProxies = 0;  //!< Objects removed from the scene since the last update
  size_t numNewObjects = 0;       //!< Objects added to the scene since the last update

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
 * and the actual OpenGL rendering. It keeps one node for each position of an
 * object in the scene graph, holding the RenderableProxy instances of that
 * object. The same object may appear at several positions (e.g. a model
 * shared by several groups), each drawn with its own model matrix.
 *
 * Key responsibilities:
 * - Initial compilation: translates the entire Scene into GPU structures
 * - Incremental updates: objects inserted or removed anywhere in the scene
 *   graph, buffers of objects whose data changed
 *   (mrpt::viz::CVisualObject::dataVersion()), and model matrices and
 *   visibility of objects whose pose, scale or visibility changed
 *   (mrpt::viz::CVisualObject::transformVersion()). If nothing changed in any
 *   scene (mrpt::viz::sceneChangeCount()), the scene graph is not traversed.
 * - Objects are compiled the first time they are visible.
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
 *   myObject->setColor(...);  // increments its data version
 *   scene.insert(newObject);  // dynamically add objects
 *   compiled.updateIfNeeded(); // compiles new objects, updates changed ones
 *   compiled.render();
 * }
 * \endcode
 *
 * Thread safety:
 * - All CompiledScene methods must be called from the OpenGL context thread
 * - Properties of individual viz objects (pose, color, geometry...) may be
 *   modified from other threads, since they are protected by their own mutexes.
 * - The structure of the scene graph (inserting or removing objects or
 *   viewports) must not change while compile(), updateIfNeeded() or render()
 *   run: callers must synchronize those changes with rendering (e.g. GUI
 *   windows provide a mutex for their scene).
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

  // Non-copyable, non-movable (owns GPU resources)
  CompiledScene(const CompiledScene&) = delete;
  CompiledScene& operator=(const CompiledScene&) = delete;
  CompiledScene(CompiledScene&&) = delete;
  CompiledScene& operator=(CompiledScene&&) = delete;

  /** @name Compilation and Updates
   * @{ */

  /** Performs initial compilation of the entire scene.
   *
   * This creates RenderableProxy instances for all visible CVisualObject
   * instances in the scene, uploads data to GPU, and prepares all necessary
   * OpenGL state.
   *
   * \param scene The abstract scene to compile (reference kept internally)
   * \param stats Optional pointer to receive compilation statistics
   *
   * \note This must be called from a thread with an active OpenGL context.
   * \note Calling compile() multiple times will clear previous compilation
   *       and start fresh.
   */
  void compile(const mrpt::viz::Scene& scene, CompilationStats* stats = nullptr);

  /** Incrementally updates the compiled scene to match the source Scene:
   * viewports added or removed, objects inserted or removed anywhere in the
   * scene graph, and objects whose data, pose, scale or visibility changed.
   *
   * \param stats Optional pointer to receive update statistics
   * \return true if any updates were performed, false if nothing changed
   *
   * \note This is called automatically by render() if auto-update is enabled.
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

  /** Returns true if this was compiled from that scene object: the same
   * instance, not another one that was later created at its memory address. */
  [[nodiscard]] bool isCompiledFrom(const mrpt::viz::Scene& scene) const;

  /** Returns true if any object of any scene changed since the last update,
   * so the next updateIfNeeded() will check this scene for changes. */
  [[nodiscard]] bool hasPendingUpdates() const;

  /** Number of viewports in the compiled scene */
  [[nodiscard]] size_t getViewportCount() const { return m_viewports.size(); }

  /** Number of total RenderableProxy objects (across all occurrences) */
  [[nodiscard]] size_t getProxyCount() const;

  /** Returns the source Scene that was compiled.
   * \return nullptr if not yet compiled
   */
  [[nodiscard]] const mrpt::viz::Scene* getSourceScene() const { return m_sourceScene; }

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
  /** A position of an object in the scene graph (defined in the .cpp file) */
  struct Node;
  /** A compiled viewport, with the nodes of its objects */
  struct ViewportEntry;

  /** Reference to the source scene (kept for incremental updates).
   * Raw pointer because the Scene may be stack-allocated (not managed by
   * shared_ptr). The caller must ensure the Scene outlives the
   * CompiledScene. */
  const mrpt::viz::Scene* m_sourceScene = nullptr;

  /** Compiled viewports, in the order of the source Scene (render order) */
  std::vector<std::unique_ptr<ViewportEntry>> m_entries;

  /** Compiled viewports, indexed by name */
  std::map<std::string, CompiledViewport::Ptr> m_viewports;

  /** mrpt::viz::sceneChangeCount() at the last traversal of the scene graph
   * (0: never) */
  uint64_t m_lastSceneChangeCount = 0;

  /** Texture::Options::shareScope of the textures of this scene: the EGL
   * context it was compiled in (one scope for all non-EGL contexts), so scenes
   * in the same context (e.g. a GUI and offscreen sensors rendered in it)
   * share their textures. */
  const void* m_textureShareScope = nullptr;

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

  /** State of the parent of a node, propagated down the scene graph. */
  struct ParentState
  {
    const mrpt::math::CMatrixFloat44* worldMatrix = nullptr;
    bool changed = false;  //!< Its world matrix, visibility or shadow casting changed
    bool visible = true;
    bool castShadows = true;
  };

  /** @name Internal Compilation Helpers
   * @{ */

  /** Brings the compiled viewports in line with those of the source scene.
   * \return true if any viewport was added, removed, or changed its mode */
  bool syncViewports(CompilationStats& stats);

  /** Updates the nodes of all objects of a viewport */
  void syncViewportObjects(ViewportEntry& vp, CompilationStats& stats);

  /** Matches the nodes of a list of objects (children of a container, of a
   * viewport, etc.) with the current objects, creating or removing nodes as
   * needed, then updates each node. */
  template <class OBJECTS>
  void syncNodes(
      std::vector<std::unique_ptr<Node>>& nodes,
      const OBJECTS& objects,
      const ParentState& parent,
      ViewportEntry& vp,
      CompilationStats& stats);

  /** Updates a node from its object: transform, visibility, buffers, and
   * children. */
  void updateNode(
      Node& node,
      const std::shared_ptr<const mrpt::viz::CVisualObject>& obj,
      const ParentState& parent,
      ViewportEntry& vp,
      CompilationStats& stats);

  /** Creates and compiles the proxies of a (non container) object. */
  void compileNodeProxies(
      Node& node, const mrpt::viz::CVisualObject& obj, ViewportEntry& vp, CompilationStats& stats);

  /** Removes the proxies of a node and all its descendants from the viewport. */
  static void releaseNode(Node& node, ViewportEntry& vp, CompilationStats& stats);

  /** Computes a model matrix from an object's pose and scale. */
  static mrpt::math::CMatrixFloat44 computeModelMatrix(
      const mrpt::viz::CVisualObject::PoseAndScale& ps,
      const mrpt::math::CMatrixFloat44& parentModelMatrix);

  /** Creates appropriate proxy types based on object's parameter mixins.
   * Returns one proxy per mixin type (e.g., CBox gets both a TrianglesProxy
   * and a LinesProxy). */
  [[nodiscard]] static std::vector<RenderableProxy::Ptr> createProxiesByType(
      const mrpt::viz::CVisualObject& obj);

  /** Validates that we're being called from the correct thread */
  void checkContextThread() const;

  /** @} */
};

}  // namespace mrpt::opengl
