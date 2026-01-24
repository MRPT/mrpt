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

#include <mrpt/opengl/RenderableProxy.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/opengl_api.h>

using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::img;

// ============================================================================
// RenderableProxy static helper implementations
// ============================================================================

void RenderableProxy::uploadMatrix(
    const RenderContext& rc, const char* uniformName, const CMatrixFloat44& matrix)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!rc.shader) return;

  const int loc = rc.shader->uniformId(uniformName);
  if (loc >= 0)
  {
    // OpenGL expects column-major, MRPT matrices are row-major, so transpose
    glUniformMatrix4fv(loc, 1, GL_TRUE, matrix.data());
  }
#endif
}

void RenderableProxy::uploadVector3(
    const RenderContext& rc, const char* uniformName, const TVector3Df& v)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!rc.shader) return;

  const int loc = rc.shader->uniformId(uniformName);
  if (loc >= 0)
  {
    glUniform3f(loc, v.x, v.y, v.z);
  }
#endif
}

void RenderableProxy::uploadColor(
    const RenderContext& rc, const char* uniformName, const TColorf& color)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!rc.shader) return;

  const int loc = rc.shader->uniformId(uniformName);
  if (loc >= 0)
  {
    glUniform4f(loc, color.R, color.G, color.B, color.A);
  }
#endif
}

void RenderableProxy::uploadFloat(const RenderContext& rc, const char* uniformName, float value)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!rc.shader) return;

  const int loc = rc.shader->uniformId(uniformName);
  if (loc >= 0)
  {
    glUniform1f(loc, value);
  }
#endif
}

void RenderableProxy::uploadInt(const RenderContext& rc, const char* uniformName, int value)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!rc.shader) return;

  const int loc = rc.shader->uniformId(uniformName);
  if (loc >= 0)
  {
    glUniform1i(loc, value);
  }
#endif
}

// ============================================================================
// PointsProxyBase implementation
// ============================================================================

void PointsProxyBase::compile(const mrpt::viz::CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj) return;

  // Try to cast to VisualObjectParams_Points
  const auto* pointsObj = dynamic_cast<const mrpt::viz::VisualObjectParams_Points*>(sourceObj);
  if (!pointsObj) return;

  // Lock the source object's buffer data
  std::shared_lock<std::shared_mutex> lck(pointsObj->shaderPointsBuffersMutex().data);

  const auto& vertices = pointsObj->shaderPointsVertexPointBuffer();
  const auto& colors = pointsObj->shaderPointsVertexColorBuffer();

  m_pointCount = vertices.size();
  if (m_pointCount == 0) return;

  // Create VAO
  m_vao.createOnce();
  m_vao.bind();

  // Upload vertex positions
  m_vertexBuffer.createOnce();
  m_vertexBuffer.bind();
  m_vertexBuffer.allocate(vertices.data(), sizeof(TPoint3Df) * m_pointCount);

  // Attribute 0: position (vec3)
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(TPoint3Df), nullptr);

  // Upload colors (if available)
  if (!colors.empty() && colors.size() == m_pointCount)
  {
    m_colorBuffer.createOnce();
    m_colorBuffer.bind();
    m_colorBuffer.allocate(colors.data(), sizeof(TColor) * m_pointCount);

    // Attribute 1: color (vec4 as unsigned bytes, normalized)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(TColor), nullptr);
  }
  else
  {
    // Use object's base color - disable attribute and use uniform
    glDisableVertexAttribArray(1);
  }

  // Unbind
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // Invalidate cached bounding box
  m_cachedBBox.reset();

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

void PointsProxyBase::updateBuffers(const mrpt::viz::CVisualObject* sourceObj)
{
  // For now, just recompile. Could optimize to only update changed data.
  compile(sourceObj);
}

void PointsProxyBase::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (m_pointCount == 0) return;
  if (!m_vao.isCreated()) return;

  m_vao.bind();

  glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_pointCount));

  glBindVertexArray(0);

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

TBoundingBoxf PointsProxyBase::getBoundingBoxLocal() const
{
  if (m_cachedBBox.has_value()) return m_cachedBBox.value();

  // Return empty bbox if no points
  return TBoundingBoxf();
}

// ============================================================================
// LinesProxyBase implementation
// ============================================================================

void LinesProxyBase::compile(const mrpt::viz::CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj) return;

  // Try to cast to VisualObjectParams_Lines
  const auto* linesObj = dynamic_cast<const mrpt::viz::VisualObjectParams_Lines*>(sourceObj);
  if (linesObj)
  {
    m_lineWidth = linesObj->getLineWidth();
    m_antiAliasing = linesObj->isAntiAliasingEnabled();
  }

  // Note: The actual line vertex data needs to come from the concrete object
  // (e.g., CSetOfLines). This base class sets up the infrastructure.
  // Derived classes should call this and then populate the buffers.

  m_vao.createOnce();

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

void LinesProxyBase::updateBuffers(const mrpt::viz::CVisualObject* sourceObj)
{
  compile(sourceObj);
}

void LinesProxyBase::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (m_vertexCount == 0) return;
  if (!m_vao.isCreated()) return;

  // Set line width
  glLineWidth(m_lineWidth);

  // Enable anti-aliasing if requested
  if (m_antiAliasing)
  {
    glEnable(GL_LINE_SMOOTH);
  }

  m_vao.bind();

  glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_vertexCount));

  glBindVertexArray(0);

  // Restore state
  if (m_antiAliasing)
  {
    glDisable(GL_LINE_SMOOTH);
  }
  glLineWidth(1.0f);

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

TBoundingBoxf LinesProxyBase::getBoundingBoxLocal() const
{
  if (m_cachedBBox.has_value()) return m_cachedBBox.value();

  return TBoundingBoxf();
}

// ============================================================================
// TrianglesProxyBase implementation
// ============================================================================

void TrianglesProxyBase::compile(const mrpt::viz::CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj) return;

  // Try to cast to VisualObjectParams_Triangles
  const auto* triObj = dynamic_cast<const mrpt::viz::VisualObjectParams_Triangles*>(sourceObj);
  if (!triObj) return;

  m_lightEnabled = triObj->isLightEnabled();
  m_cullFace = triObj->cullFaces();

  // Lock the source object's buffer data
  std::shared_lock<std::shared_mutex> lck(triObj->shaderTrianglesBufferMutex().data);

  const auto& triangles = triObj->shaderTrianglesBuffer();

  m_triangleCount = triangles.size();
  if (m_triangleCount == 0) return;

  // Extract flat arrays from TTriangle structures
  // Each TTriangle has 3 vertices, each with position, normal, and color
  const size_t vertexCount = m_triangleCount * 3;

  std::vector<TPoint3Df> vertices;
  std::vector<TVector3Df> normals;
  std::vector<TColor> colors;

  vertices.reserve(vertexCount);
  normals.reserve(vertexCount);
  colors.reserve(vertexCount);

  for (const auto& tri : triangles)
  {
    for (int i = 0; i < 3; ++i)
    {
      vertices.push_back(tri.vertices[i].xyzrgba.pt);
      normals.push_back(tri.vertices[i].normal);

      // Convert float RGBA to TColor (0-255)
      const auto& rgba = tri.vertices[i].xyzrgba;
      colors.emplace_back(
          static_cast<uint8_t>(rgba.r * 255), static_cast<uint8_t>(rgba.g * 255),
          static_cast<uint8_t>(rgba.b * 255), static_cast<uint8_t>(rgba.a * 255));
    }
  }

  // Create VAO
  m_vao.createOnce();
  m_vao.bind();

  // Upload vertex positions
  m_vertexBuffer.createOnce();
  m_vertexBuffer.bind();
  m_vertexBuffer.allocate(vertices.data(), sizeof(TPoint3Df) * vertexCount);

  // Attribute 0: position (vec3)
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(TPoint3Df), nullptr);

  // Upload normals
  m_normalBuffer.createOnce();
  m_normalBuffer.bind();
  m_normalBuffer.allocate(normals.data(), sizeof(TVector3Df) * vertexCount);

  // Attribute 1: normal (vec3)
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(TVector3Df), nullptr);

  // Upload colors
  m_colorBuffer.createOnce();
  m_colorBuffer.bind();
  m_colorBuffer.allocate(colors.data(), sizeof(TColor) * vertexCount);

  // Attribute 2: color (vec4 as unsigned bytes, normalized)
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(TColor), nullptr);

  // Unbind
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // Invalidate cached bounding box
  m_cachedBBox.reset();

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

void TrianglesProxyBase::updateBuffers(const mrpt::viz::CVisualObject* sourceObj)
{
  compile(sourceObj);
}

void TrianglesProxyBase::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (m_triangleCount == 0) return;
  if (!m_vao.isCreated()) return;

  // Setup face culling
  bool cullingWasEnabled = glIsEnabled(GL_CULL_FACE);
  GLint previousCullMode = GL_BACK;

  if (m_cullFace != mrpt::viz::TCullFace::NONE)
  {
    glEnable(GL_CULL_FACE);
    glGetIntegerv(GL_CULL_FACE_MODE, &previousCullMode);

    if (m_cullFace == mrpt::viz::TCullFace::BACK)
      glCullFace(GL_BACK);
    else if (m_cullFace == mrpt::viz::TCullFace::FRONT)
      glCullFace(GL_FRONT);
  }
  else
  {
    glDisable(GL_CULL_FACE);
  }

  m_vao.bind();

  const GLsizei vertexCount = static_cast<GLsizei>(m_triangleCount * 3);
  glDrawArrays(GL_TRIANGLES, 0, vertexCount);

  glBindVertexArray(0);

  // Restore culling state
  if (cullingWasEnabled)
    glEnable(GL_CULL_FACE);
  else
    glDisable(GL_CULL_FACE);
  glCullFace(previousCullMode);

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

std::vector<shader_id_t> TrianglesProxyBase::requiredShaders() const
{
  // Return appropriate shader based on lighting setting
  if (m_lightEnabled)
    return {DefaultShaderID::TRIANGLES_LIGHT};
  else
    return {DefaultShaderID::TRIANGLES_NO_LIGHT};
}

shader_id_t TrianglesProxyBase::selectShader(bool isShadowMapPass) const
{
  if (isShadowMapPass)
  {
    // Shadow map generation pass - depth only
    return DefaultShaderID::TRIANGLES_SHADOW_1ST;
  }
  else
  {
    // Normal rendering
    if (m_lightEnabled)
      return DefaultShaderID::TRIANGLES_LIGHT;
    else
      return DefaultShaderID::TRIANGLES_NO_LIGHT;
  }
}

TBoundingBoxf TrianglesProxyBase::getBoundingBoxLocal() const
{
  if (m_cachedBBox.has_value()) return m_cachedBBox.value();

  return TBoundingBoxf();
}

// ============================================================================
// TexturedTrianglesProxyBase implementation
// ============================================================================

void TexturedTrianglesProxyBase::compile(const mrpt::viz::CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj) return;

  // First compile base triangle data
  TrianglesProxyBase::compile(sourceObj);

  // Try to cast to VisualObjectParams_TexturedTriangles
  const auto* texTriObj =
      dynamic_cast<const mrpt::viz::VisualObjectParams_TexturedTriangles*>(sourceObj);
  if (!texTriObj) return;

  m_textureInterpolate = texTriObj->textureLinearInterpolation();
  m_textureMipMaps = texTriObj->textureMipMap();

  // Lock the source object's buffer data
  std::shared_lock<std::shared_mutex> lck(texTriObj->shaderTexturedTrianglesBufferMutex().data);

  const auto& triangles = texTriObj->shaderTexturedTrianglesBuffer();

  if (triangles.empty()) return;

  // Extract texture coordinates
  const size_t vertexCount = triangles.size() * 3;
  std::vector<mrpt::math::TPoint2Df> texCoords;
  texCoords.reserve(vertexCount);

  for (const auto& tri : triangles)
  {
    for (int i = 0; i < 3; ++i)
    {
      texCoords.emplace_back(tri.vertices[i].uv.x, tri.vertices[i].uv.y);
    }
  }

  // Bind VAO to add texture coordinate attribute
  m_vao.bind();

  // Upload texture coordinates
  m_texCoordBuffer.createOnce();
  m_texCoordBuffer.bind();
  m_texCoordBuffer.allocate(texCoords.data(), sizeof(mrpt::math::TPoint2Df) * vertexCount);

  // Attribute 3: texture coordinates (vec2)
  glEnableVertexAttribArray(3);
  glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(mrpt::math::TPoint2Df), nullptr);

  // Unbind
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // TODO: Handle texture upload via Texture class
  // This would need access to the image data from the source object
  // m_texture = TextureCache::getOrCreate(texTriObj->getTextureImage(), ...);

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

void TexturedTrianglesProxyBase::updateBuffers(const mrpt::viz::CVisualObject* sourceObj)
{
  compile(sourceObj);
}

void TexturedTrianglesProxyBase::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  // Bind texture if available
  if (m_texture)
  {
    glActiveTexture(GL_TEXTURE0 + MATERIAL_DIFFUSE_TEXTURE_UNIT);
    // m_texture->bind();  // Would need Texture class implementation
  }

  // Render triangles
  TrianglesProxyBase::render(rc);

  // Unbind texture
  if (m_texture)
  {
    glActiveTexture(GL_TEXTURE0 + MATERIAL_DIFFUSE_TEXTURE_UNIT);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  MRPT_END
#endif
}

std::vector<shader_id_t> TexturedTrianglesProxyBase::requiredShaders() const
{
  if (m_lightEnabled)
    return {DefaultShaderID::TEXTURED_TRIANGLES_LIGHT};
  else
    return {DefaultShaderID::TEXTURED_TRIANGLES_NO_LIGHT};
}
