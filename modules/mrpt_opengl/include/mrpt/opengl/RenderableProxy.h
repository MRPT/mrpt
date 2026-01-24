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

#include <mrpt/img/TColor.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/Buffer.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <mrpt/opengl/VertexArrayObject.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CVisualObject.h>

#include <memory>
#include <optional>
#include <vector>

namespace mrpt::opengl
{
// Forward declarations
class Program;
struct TLightParameters;

/** Rendering context passed to RenderableProxy::render().
 * Contains all state needed for rendering an object.
 * \ingroup mrpt_opengl_grp
 */
struct RenderContext
{
  /** The shader program to use for rendering */
  Program* shader = nullptr;

  /** ID of the shader being used */
  shader_id_t shader_id = DefaultShaderID::NONE;

  /** Projection/view/model matrices and camera state */
  const TRenderMatrices* state = nullptr;

  /** Lighting parameters */
  const TLightParameters* lights = nullptr;

  /** Is this a shadow map generation pass? */
  bool isShadowMapPass = false;
};

/** Base class for GPU-side representations of mrpt::viz::CVisualObject instances.
 *
 * This is the core abstraction that bridges the abstract scene graph
 * (mrpt::viz) with the actual OpenGL rendering (mrpt::opengl).
 *
 * Key responsibilities:
 * - **GPU Resource Management**: Owns OpenGL buffers (VBOs, VAOs, textures)
 * - **Compilation**: Translates abstract object data into GPU buffers
 * - **Incremental Updates**: Efficiently updates only changed data
 * - **Rendering**: Issues OpenGL draw calls using bound shaders
 *
 * Design pattern:
 * Each concrete CVisualObject type has a corresponding RenderableProxy:
 * - viz::VisualObjectParams_Points      → opengl::PointsProxy
 * - viz::VisualObjectParams_Lines       → opengl::LinesProxy
 * - viz::VisualObjectParams_Triangles   → opengl::TrianglesProxy
 * - viz::VisualObjectParams_TexturedTriangles → opengl::TexturedTrianglesProxy
 *
 * Lifecycle:
 * 1. Created by CompiledScene during compilation
 * 2. compile() called once to upload initial data to GPU
 * 3. updateBuffers() called when source object changes (dirty flag)
 * 4. render() called every frame to draw
 * 5. Destroyed when source object deleted or scene recompiled
 *
 * Thread safety:
 * - All methods must be called from the OpenGL context thread
 * - Source object access is read-only (via const pointers)
 * - Source objects are tracked via weak_ptr in CompiledScene
 *
 * \sa CompiledScene, CompiledViewport, mrpt::viz::CVisualObject
 * \ingroup mrpt_opengl_grp
 */
class RenderableProxy
{
 public:
  using Ptr = std::shared_ptr<RenderableProxy>;

  RenderableProxy() = default;
  virtual ~RenderableProxy() = default;

  /** @name Core Rendering Interface
   * @{ */

  /** Initial compilation: uploads object data to GPU.
   *
   * This is called once when the proxy is first created. It should:
   * - Create OpenGL buffers (VBOs, VAOs, textures)
   * - Upload initial vertex/color/normal/texture data
   * - Cache any frequently-used values
   *
   * \param sourceObj The abstract viz object (read-only access)
   *
   * \note Must be called from OpenGL context thread
   * \note After this call, the proxy should be ready to render
   */
  virtual void compile(const mrpt::viz::CVisualObject* sourceObj) = 0;

  /** Incremental update: refreshes GPU buffers with changed data.
   *
   * This is called when the source object's dirty flag is set
   * (hasToUpdateBuffers() returns true). It should:
   * - Re-upload only the changed data (vertices, colors, etc.)
   * - Be as efficient as possible (don't recompile everything)
   *
   * \param sourceObj The abstract viz object (read-only access)
   *
   * \note Must be called from OpenGL context thread
   * \note Default implementation calls compile() - override for efficiency
   */
  virtual void updateBuffers(const mrpt::viz::CVisualObject* sourceObj) { compile(sourceObj); }

  /** Renders this object using the provided context.
   *
   * This is called every frame for visible objects. It should:
   * - Bind appropriate buffers (VAO, VBO, textures)
   * - Set shader uniforms (model matrix, material properties, etc.)
   * - Issue draw calls (glDrawArrays, glDrawElements, etc.)
   *
   * \param rc Rendering context (shader, matrices, lights)
   *
   * \note Must be called from OpenGL context thread
   * \note The shader program is already bound when this is called
   * \note Common matrices (P, V, M) are already uploaded by CompiledViewport
   */
  virtual void render(const RenderContext& rc) const = 0;

  /** @} */

  /** @name Shader and Rendering Properties
   * @{ */

  /** Returns the list of shader programs this object needs.
   *
   * Most objects use a single shader, but some may use multiple
   * (e.g., different shaders for shadow map pass vs. normal rendering).
   *
   * \return Vector of shader IDs, typically with 1 element
   */
  [[nodiscard]] virtual std::vector<shader_id_t> requiredShaders() const = 0;

  /** Does this object cast shadows?
   *
   * Used to determine if the object should be rendered during the
   * shadow map generation pass (1st pass of shadow rendering).
   *
   * \return true if object casts shadows (default: true)
   */
  [[nodiscard]] virtual bool castsShadows() const { return true; }

  /** Should this object be checked for frustum culling?
   *
   * Some objects (like skyboxes) should never be culled even if
   * their bounding box is outside the view frustum.
   *
   * \return true if eligible for culling (default: true)
   */
  [[nodiscard]] virtual bool cullEligible() const { return true; }

  /** @} */

  /** @name Bounding Box (for Culling and Spatial Queries)
   * @{ */

  /** Returns the object's bounding box in local coordinates.
   *
   * Used for frustum culling and spatial queries. The bounding box
   * should be as tight as possible for efficient culling.
   *
   * \return Bounding box, or empty box if not applicable
   *
   * \note This is in the object's local frame, before applying pose transform
   */
  [[nodiscard]] virtual mrpt::math::TBoundingBoxf getBoundingBoxLocal() const
  {
    return mrpt::math::TBoundingBoxf();
  }

  /** Returns the object's bounding box in world coordinates.
   *
   * This applies the object's pose transformation to the local bbox.
   *
   * \param objPose The object's SE(3) pose in world frame
   * \return Transformed bounding box
   */
  [[nodiscard]] mrpt::math::TBoundingBoxf getBoundingBox(const mrpt::poses::CPose3D& objPose) const
  {
    return getBoundingBoxLocal().compose(objPose);
  }

  /** @} */

  /** @name Type Information
   * @{ */

  /** Returns a human-readable type name for this proxy.
   * Used for debugging and logging.
   */
  [[nodiscard]] virtual const char* typeName() const { return "RenderableProxy"; }

  /** @} */

 protected:
  /** Weak reference to the source viz object.
   * Used to:
   * - Check if source still exists (weak_ptr::expired())
   * - Access source data during updateBuffers() if needed
   * - Query dirty flags
   * Set by CompiledScene during proxy creation.
   */
  std::weak_ptr<mrpt::viz::CVisualObject> m_sourceObject;

 public:
  /** Sets the source object reference. Called by CompiledScene during compilation. */
  void setSourceObject(std::weak_ptr<mrpt::viz::CVisualObject> obj)
  {
    m_sourceObject = std::move(obj);
  }

  /** Returns the source object, or nullptr if it has been deleted. */
  [[nodiscard]] std::shared_ptr<mrpt::viz::CVisualObject> getSourceObject() const
  {
    return m_sourceObject.lock();
  }

  /** Check if source object still exists */
  [[nodiscard]] bool isSourceValid() const { return !m_sourceObject.expired(); }

  /** Check if source object has pending changes (dirty flag) */
  [[nodiscard]] bool sourceNeedsUpdate() const
  {
    auto src = m_sourceObject.lock();
    return src && src->hasToUpdateBuffers();
  }

 protected:
  /** @name Helper Methods for Derived Classes
   * @{ */

  /** Helper: uploads a 4x4 matrix as a uniform to the current shader.
   *
   * \param rc Render context with bound shader
   * \param uniformName Name of the uniform variable in the shader
   * \param matrix The 4x4 matrix to upload
   */
  static void uploadMatrix(
      const RenderContext& rc, const char* uniformName, const mrpt::math::CMatrixFloat44& matrix);

  /** Helper: uploads a 3-component vector as a uniform.
   *
   * \param rc Render context with bound shader
   * \param uniformName Name of the uniform variable
   * \param v The 3D vector
   */
  static void uploadVector3(
      const RenderContext& rc, const char* uniformName, const mrpt::math::TVector3Df& v);

  /** Helper: uploads a color (4 floats) as a uniform.
   *
   * \param rc Render context with bound shader
   * \param uniformName Name of the uniform variable
   * \param color The color (RGBA in [0,1])
   */
  static void uploadColor(
      const RenderContext& rc, const char* uniformName, const mrpt::img::TColorf& color);

  /** Helper: uploads a float scalar as a uniform.
   *
   * \param rc Render context with bound shader
   * \param uniformName Name of the uniform variable
   * \param value The float value
   */
  static void uploadFloat(const RenderContext& rc, const char* uniformName, float value);

  /** Helper: uploads an integer as a uniform.
   *
   * \param rc Render context with bound shader
   * \param uniformName Name of the uniform variable
   * \param value The integer value
   */
  static void uploadInt(const RenderContext& rc, const char* uniformName, int value);

  /** @} */

 public:
  // Prevent copying (these own GPU resources)
  RenderableProxy(const RenderableProxy&) = delete;
  RenderableProxy& operator=(const RenderableProxy&) = delete;

  // Allow moving
  RenderableProxy(RenderableProxy&&) = default;
  RenderableProxy& operator=(RenderableProxy&&) = default;
};

/** Specialization for proxies that render points.
 *
 * This provides common functionality for all point-based rendering,
 * including vertex and color buffer management.
 *
 * \sa mrpt::viz::VisualObjectParams_Points
 * \ingroup mrpt_opengl_grp
 */
class PointsProxyBase : public RenderableProxy
{
 public:
  PointsProxyBase() = default;

  void compile(const mrpt::viz::CVisualObject* sourceObj) override;
  void updateBuffers(const mrpt::viz::CVisualObject* sourceObj) override;
  void render(const RenderContext& rc) const override;

  std::vector<shader_id_t> requiredShaders() const override { return {DefaultShaderID::POINTS}; }

  mrpt::math::TBoundingBoxf getBoundingBoxLocal() const override;

  const char* typeName() const override { return "PointsProxyBase"; }

 protected:
  /** Vertex buffer (positions) */
  Buffer m_vertexBuffer{Buffer::Type::Vertex};

  /** Color buffer (per-vertex colors) */
  Buffer m_colorBuffer{Buffer::Type::Vertex};

  /** Vertex Array Object (caches attribute bindings) */
  VertexArrayObject m_vao;

  /** Number of points to render */
  size_t m_pointCount = 0;

  /** Cached bounding box */
  mutable std::optional<mrpt::math::TBoundingBoxf> m_cachedBBox;
};

/** Specialization for proxies that render lines/wireframes.
 *
 * This provides common functionality for all line-based rendering,
 * including vertex, color, and line parameter management.
 *
 * \sa mrpt::viz::VisualObjectParams_Lines
 * \ingroup mrpt_opengl_grp
 */
class LinesProxyBase : public RenderableProxy
{
 public:
  LinesProxyBase() = default;

  void compile(const mrpt::viz::CVisualObject* sourceObj) override;
  void updateBuffers(const mrpt::viz::CVisualObject* sourceObj) override;
  void render(const RenderContext& rc) const override;

  std::vector<shader_id_t> requiredShaders() const override { return {DefaultShaderID::WIREFRAME}; }

  mrpt::math::TBoundingBoxf getBoundingBoxLocal() const override;

  const char* typeName() const override { return "LinesProxyBase"; }

 protected:
  /** Vertex buffer (line endpoints) */
  Buffer m_vertexBuffer{Buffer::Type::Vertex};

  /** Color buffer (per-vertex colors) */
  Buffer m_colorBuffer{Buffer::Type::Vertex};

  /** Vertex Array Object */
  VertexArrayObject m_vao;

  /** Number of vertices (lines have 2 vertices each) */
  size_t m_vertexCount = 0;

  /** Line width */
  float m_lineWidth = 1.0f;

  /** Anti-aliasing enabled */
  bool m_antiAliasing = false;

  /** Cached bounding box */
  mutable std::optional<mrpt::math::TBoundingBoxf> m_cachedBBox;
};

/** Specialization for proxies that render triangles (with or without lighting).
 *
 * This provides common functionality for triangle mesh rendering.
 *
 * \sa mrpt::viz::VisualObjectParams_Triangles
 * \ingroup mrpt_opengl_grp
 */
class TrianglesProxyBase : public RenderableProxy
{
 public:
  TrianglesProxyBase() = default;

  void compile(const mrpt::viz::CVisualObject* sourceObj) override;
  void updateBuffers(const mrpt::viz::CVisualObject* sourceObj) override;
  void render(const RenderContext& rc) const override;

  std::vector<shader_id_t> requiredShaders() const override;

  mrpt::math::TBoundingBoxf getBoundingBoxLocal() const override;

  const char* typeName() const override { return "TrianglesProxyBase"; }

 protected:
  /** Vertex buffer (triangle vertices) */
  Buffer m_vertexBuffer{Buffer::Type::Vertex};

  /** Normal buffer (per-vertex normals) */
  Buffer m_normalBuffer{Buffer::Type::Vertex};

  /** Color buffer (per-vertex colors) */
  Buffer m_colorBuffer{Buffer::Type::Vertex};

  /** Vertex Array Object */
  VertexArrayObject m_vao;

  /** Number of triangles */
  size_t m_triangleCount = 0;

  /** Lighting enabled */
  bool m_lightEnabled = true;

  /** Face culling mode */
  mrpt::viz::TCullFace m_cullFace = mrpt::viz::TCullFace::NONE;

  /** Cached bounding box */
  mutable std::optional<mrpt::math::TBoundingBoxf> m_cachedBBox;

  /** Determines which shader to use based on lighting and shadow settings */
  shader_id_t selectShader(bool isShadowMapPass) const;
};

/** Specialization for proxies that render textured triangles.
 *
 * Extends TrianglesProxyBase with texture coordinate and texture management.
 *
 * \sa mrpt::viz::VisualObjectParams_TexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
class TexturedTrianglesProxyBase : public TrianglesProxyBase
{
 public:
  TexturedTrianglesProxyBase() = default;

  void compile(const mrpt::viz::CVisualObject* sourceObj) override;
  void updateBuffers(const mrpt::viz::CVisualObject* sourceObj) override;
  void render(const RenderContext& rc) const override;

  std::vector<shader_id_t> requiredShaders() const override;

  const char* typeName() const override { return "TexturedTrianglesProxyBase"; }

 protected:
  /** Texture coordinate buffer (UV coordinates) */
  Buffer m_texCoordBuffer{Buffer::Type::Vertex};

  /** Texture object (owned externally, managed by TextureCache) */
  class Texture* m_texture = nullptr;

  /** Texture interpolation (linear vs nearest) */
  bool m_textureInterpolate = false;

  /** Use mipmaps */
  bool m_textureMipMaps = true;
};

}  // namespace mrpt::opengl