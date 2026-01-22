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

#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/Shader.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mrpt::opengl
{
/** Manages the lifecycle and caching of OpenGL shader programs.
 *
 * This class centralizes shader program management for the entire rendering
 * system. It provides:
 * - **Lazy loading**: Shaders are compiled on first use
 * - **Caching**: Each shader is compiled once and reused
 * - **Automatic cleanup**: Programs are destroyed when manager is destroyed
 * - **Thread safety**: All operations must be on the OpenGL context thread
 * - **Custom shader support**: Load shaders from source code or files
 *
 * The manager handles both built-in default shaders (from DefaultShaders.h)
 * and user-defined custom shaders.
 *
 * Typical usage:
 * \code
 * ShaderProgramManager shaderMgr;
 *
 * // Get a built-in shader (compiled on first request):
 * auto pointsShader = shaderMgr.getProgram(DefaultShaderID::POINTS);
 * pointsShader->use();
 *
 * // Load a custom shader:
 * auto customShader = shaderMgr.loadCustomProgram(
 *     "myShader",
 *     vertexShaderSource,
 *     fragmentShaderSource);
 * \endcode
 *
 * Thread safety:
 * - This class is NOT thread-safe
 * - All operations must be called from the OpenGL context thread
 * - Violating this will cause OpenGL errors or crashes
 *
 * \sa Program, Shader, DefaultShaders.h, CompiledScene
 * \ingroup mrpt_opengl_grp
 */
class ShaderProgramManager
{
 public:
  /** Constructor.
   *
   * Does not compile any shaders yet - they are loaded lazily on first use.
   */
  ShaderProgramManager();

  /** Destructor.
   *
   * Automatically cleans up all shader programs. Must be called from
   * the same thread that created the shaders (OpenGL context thread).
   */
  ~ShaderProgramManager();

  /** @name Built-in Shader Access
   * @{ */

  /** Retrieves a built-in shader program by ID.
   *
   * If the shader hasn't been compiled yet, it is compiled on first access.
   * Subsequent calls return the cached program.
   *
   * \param id The shader ID (from DefaultShaderID enum)
   * \return Shared pointer to the program, or nullptr on compilation failure
   *
   * \throws std::runtime_error if shader compilation fails
   *
   * \note This must be called from the OpenGL context thread
   * \note The returned pointer remains valid as long as this manager exists
   */
  Program::Ptr getProgram(shader_id_t id);

  /** Checks if a shader program has been compiled and cached.
   *
   * \param id The shader ID to check
   * \return true if program exists in cache, false otherwise
   */
  bool hasProgram(shader_id_t id) const;

  /** Preloads all default shaders.
   *
   * By default, shaders are compiled lazily. Call this to compile
   * all built-in shaders at initialization time, which can help
   * avoid stuttering during first render.
   *
   * \param outErrors Optional vector to receive compilation errors
   * \return Number of shaders successfully compiled
   *
   * \note This can take 100-500ms depending on GPU/driver
   */
  size_t preloadAllDefaultShaders(std::vector<std::string>* outErrors = nullptr);

  /** @} */

  /** @name Custom Shader Management
   * @{ */

  /** Loads a custom shader program from source code.
   *
   * The shader is compiled and linked immediately. If successful,
   * it is cached and can be retrieved later by name.
   *
   * \param name Unique name for this custom shader
   * \param vertexShaderSource GLSL source code for vertex shader
   * \param fragmentShaderSource GLSL source code for fragment shader
   * \param geometryShaderSource Optional GLSL source for geometry shader
   * \param outErrorMsg Optional string to receive compilation/link errors
   * \return Shared pointer to program, or nullptr on failure
   *
   * \throws std::runtime_error if name already exists
   *
   * Example:
   * \code
   * const char* vertSrc = R"(
   *   #version 330 core
   *   layout(location = 0) in vec3 position;
   *   uniform mat4 pmv_matrix;
   *   void main() {
   *     gl_Position = pmv_matrix * vec4(position, 1.0);
   *   }
   * )";
   *
   * const char* fragSrc = R"(
   *   #version 330 core
   *   out vec4 color;
   *   void main() {
   *     color = vec4(1.0, 0.0, 0.0, 1.0);
   *   }
   * )";
   *
   * auto prog = shaderMgr.loadCustomProgram("redShader", vertSrc, fragSrc);
   * \endcode
   */
  Program::Ptr loadCustomProgram(
      const std::string& name,
      const std::string& vertexShaderSource,
      const std::string& fragmentShaderSource,
      const std::string& geometryShaderSource = "",
      std::string* outErrorMsg = nullptr);

  /** Loads a custom shader program from files.
   *
   * Convenience wrapper that reads shader source from files.
   *
   * \param name Unique name for this custom shader
   * \param vertexShaderFile Path to vertex shader (.vert, .vs, etc.)
   * \param fragmentShaderFile Path to fragment shader (.frag, .fs, etc.)
   * \param geometryShaderFile Optional path to geometry shader (.geom, .gs)
   * \param outErrorMsg Optional string to receive errors
   * \return Shared pointer to program, or nullptr on failure
   */
  Program::Ptr loadCustomProgramFromFiles(
      const std::string& name,
      const std::string& vertexShaderFile,
      const std::string& fragmentShaderFile,
      const std::string& geometryShaderFile = "",
      std::string* outErrorMsg = nullptr);

  /** Retrieves a custom shader by name.
   *
   * \param name The name given when loading the shader
   * \return Shared pointer to program, or nullptr if not found
   */
  Program::Ptr getCustomProgram(const std::string& name) const;

  /** Checks if a custom shader exists.
   *
   * \param name The shader name to check
   * \return true if custom shader exists, false otherwise
   */
  bool hasCustomProgram(const std::string& name) const;

  /** Removes a custom shader from the cache.
   *
   * The program is destroyed if no other references exist.
   *
   * \param name Name of shader to remove
   * \return true if shader was found and removed, false if not found
   */
  bool removeCustomProgram(const std::string& name);

  /** @} */

  /** @name Shader Variants (for Shadow Mapping)
   * @{ */

  /** Retrieves the appropriate shader variant for the current render pass.
   *
   * Shadow rendering requires different shaders for different passes:
   * - 1st pass (shadow map generation): depth-only shaders
   * - 2nd pass (normal rendering): shaders that sample shadow map
   *
   * This method automatically selects the right variant based on the
   * base shader ID and render pass.
   *
   * \param baseShaderID The base shader (e.g., TRIANGLES_LIGHT)
   * \param isShadowMapPass true for 1st pass, false for 2nd pass
   * \return The appropriate shader variant
   *
   * Example mappings:
   * - TRIANGLES_LIGHT + shadowMapPass=true  → TRIANGLES_SHADOW_1ST
   * - TRIANGLES_LIGHT + shadowMapPass=false → TRIANGLES_SHADOW_2ND
   * - POINTS + shadowMapPass=true           → NONE (points don't cast shadows)
   * - WIREFRAME + shadowMapPass=false       → WIREFRAME (no shadow variant)
   */
  shader_id_t getShaderVariant(shader_id_t baseShaderID, bool isShadowMapPass) const;

  /** @} */

  /** @name Utility Methods
   * @{ */

  /** Clears all cached programs (built-in and custom).
   *
   * Forces recompilation on next access. Useful for shader hot-reloading
   * during development.
   *
   * \warning All existing Program::Ptr references become invalid
   */
  void clear();

  /** Returns number of cached built-in programs */
  size_t getBuiltinProgramCount() const { return m_builtinPrograms.size(); }

  /** Returns number of cached custom programs */
  size_t getCustomProgramCount() const { return m_customPrograms.size(); }

  /** Returns total number of cached programs */
  size_t getTotalProgramCount() const { return m_builtinPrograms.size() + m_customPrograms.size(); }

  /** Lists all loaded shader IDs (built-in).
   * \return Vector of shader IDs currently in cache
   */
  std::vector<shader_id_t> getLoadedShaderIDs() const;

  /** Lists all loaded custom shader names.
   * \return Vector of custom shader names currently in cache
   */
  std::vector<std::string> getLoadedCustomShaderNames() const;

  /** Enables/disables verbose logging of shader compilation.
   *
   * When enabled, compilation success/failure is logged to stdout.
   * Default: false
   */
  void setVerbose(bool verbose) { m_verbose = verbose; }

  /** Returns current verbose setting */
  bool isVerbose() const { return m_verbose; }

  /** @} */

 private:
  /** @name Internal Compilation Helpers
   * @{ */

  /** Compiles a built-in shader by ID.
   *
   * \param id Shader ID from DefaultShaderID
   * \param outErrorMsg Optional error message output
   * \return Compiled program, or nullptr on failure
   */
  Program::Ptr compileBuiltinShader(shader_id_t id, std::string* outErrorMsg = nullptr);

  /** Compiles shader source into a Program.
   *
   * \param vertexSrc Vertex shader GLSL source
   * \param fragmentSrc Fragment shader GLSL source
   * \param geometrySrc Optional geometry shader source (empty = none)
   * \param shaderName Name for error messages
   * \param outErrorMsg Optional error message output
   * \return Compiled program, or nullptr on failure
   */
  Program::Ptr compileShaderProgram(
      const std::string& vertexSrc,
      const std::string& fragmentSrc,
      const std::string& geometrySrc,
      const std::string& shaderName,
      std::string* outErrorMsg = nullptr);

  /** Declares all common uniforms for a program.
   *
   * Most shaders use common uniforms like pmv_matrix, p_matrix, etc.
   * This helper declares them all to avoid repetition.
   *
   * \param program The program to populate with uniform declarations
   */
  void declareCommonUniforms(Program& program);

  /** Declares all common attributes for a program.
   *
   * Most shaders use common attributes like position, vertexColor, etc.
   *
   * \param program The program to populate with attribute declarations
   */
  void declareCommonAttributes(Program& program);

  /** @} */

  /** @name Shader Variant Mapping
   * @{ */

  /** Builds the internal mapping table for shader variants.
   *
   * Called once during construction to populate m_shaderVariants.
   */
  void buildShaderVariantMap();

  /** Mapping: (base_shader, is_shadow_pass) → variant_shader
   * Used by getShaderVariant()
   */
  std::map<std::pair<shader_id_t, bool>, shader_id_t> m_shaderVariants;

  /** @} */

  /** Cache of built-in shader programs, indexed by shader ID */
  std::map<shader_id_t, Program::Ptr> m_builtinPrograms;

  /** Cache of custom shader programs, indexed by name */
  std::map<std::string, Program::Ptr> m_customPrograms;

  /** Verbose logging flag */
  bool m_verbose = false;

  /** Thread ID where this manager was created (OpenGL context thread) */
  std::thread::id m_contextThread;

  /** Validates we're on the correct thread */
  void checkThread() const;

 public:
  // Disable copy/move (manages OpenGL resources)
  ShaderProgramManager(const ShaderProgramManager&) = delete;
  ShaderProgramManager& operator=(const ShaderProgramManager&) = delete;
  ShaderProgramManager(ShaderProgramManager&&) = delete;
  ShaderProgramManager& operator=(ShaderProgramManager&&) = delete;
};

/** Helper: Creates a shader program manager and preloads all default shaders.
 *
 * Convenience function for initialization. Logs any compilation errors to stderr.
 *
 * \return Unique pointer to initialized manager
 *
 * Example:
 * \code
 * auto shaderMgr = createAndPreloadShaders();
 * // All default shaders are now ready to use
 * \endcode
 *
 * \ingroup mrpt_opengl_grp
 */
std::unique_ptr<ShaderProgramManager> createAndPreloadShaders();

}  // namespace mrpt::opengl