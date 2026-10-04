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

#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/RenderableProxy.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/Texture.h>
#include <mrpt/opengl/opengl_api.h>

#include <cmath>

using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::img;

namespace
{
template <class POINT>
std::optional<TBoundingBoxf> boundingBoxOf(const std::vector<POINT>& pts)
{
  if (pts.empty())
  {
    return std::nullopt;
  }
  auto bb = TBoundingBoxf::PlusMinusInfinity();
  for (const auto& p : pts)
  {
    bb.updateWithPoint({p.x, p.y, p.z});
  }
  return bb;
}

bool anyTranslucent(const std::vector<TColor>& colors)
{
  for (const auto& c : colors)
  {
    if (c.A != 0xff)
    {
      return true;
    }
  }
  return false;
}

/** Points and lines without per-vertex colors use the object color: the
 * color attribute is disabled in their VAO, so its constant value is used. */
void setConstantVertexColor(const TColor& c)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  glVertexAttrib4f(1, c.R / 255.0f, c.G / 255.0f, c.B / 255.0f, c.A / 255.0f);
#endif
}
}  // namespace

// ============================================================================
// RenderableProxy static helper implementations
// ============================================================================

void RenderableProxy::uploadMatrix(
    const RenderContext& rc, const char* uniformName, const CMatrixFloat44& matrix)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!rc.shader)
  {
    return;
  }
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
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!rc.shader)
  {
    return;
  }
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
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!rc.shader)
  {
    return;
  }
  const int loc = rc.shader->uniformId(uniformName);
  if (loc >= 0)
  {
    // Use glUniform3f since light_color and similar are vec3 in shaders
    glUniform3f(loc, color.R, color.G, color.B);
  }
#endif
}

void RenderableProxy::uploadFloat(const RenderContext& rc, const char* uniformName, float value)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!rc.shader)
  {
    return;
  }
  const int loc = rc.shader->uniformId(uniformName);
  if (loc >= 0)
  {
    glUniform1f(loc, value);
  }
#endif
}

void RenderableProxy::uploadInt(const RenderContext& rc, const char* uniformName, int value)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!rc.shader)
  {
    return;
  }
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
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj)
  {
    return;
  }
  // Try to cast to VisualObjectParams_Points
  const auto* pointsObj = dynamic_cast<const mrpt::viz::VisualObjectParams_Points*>(sourceObj);
  if (!pointsObj)
  {
    return;
  }
  // Lock the source object's buffer data
  std::shared_lock<std::shared_mutex> lck(pointsObj->shaderPointsBuffersMutex().data);

  const auto& vertices = pointsObj->shaderPointsVertexPointBuffer();
  const auto& colors = pointsObj->shaderPointsVertexColorBuffer();

  m_pointCount = vertices.size();
  m_localBBox = boundingBoxOf(vertices);
  m_hasColorBuffer = !colors.empty() && colors.size() == m_pointCount;
  m_baseColor = sourceObj->getColor_u8();
  m_transparent = m_hasColorBuffer ? anyTranslucent(colors) : m_baseColor.A != 0xff;
  if (m_pointCount == 0)
  {
    return;
  }
  // Create VAO
  m_vao.createOnce();
  m_vao.bind();

  // Upload vertex positions
  m_vertexBuffer.createOnce();
  m_vertexBuffer.bind();
  m_vertexBuffer.allocate(vertices.data(), static_cast<int>(sizeof(TPoint3Df) * m_pointCount));

  // Attribute 0: position (vec3)
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(TPoint3Df), nullptr);

  // Upload colors (if available)
  if (m_hasColorBuffer)
  {
    m_colorBuffer.createOnce();
    m_colorBuffer.bind();
    m_colorBuffer.allocate(colors.data(), static_cast<int>(sizeof(TColor) * m_pointCount));

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

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

void PointsProxyBase::updateBuffers(const mrpt::viz::CVisualObject* sourceObj)
{
  // For now, just recompile. Could optimize to only update changed data.
  compile(sourceObj);
}

void PointsProxyBase::render([[maybe_unused]] const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  MRPT_START

  if (m_pointCount == 0)
  {
    return;
  }
  if (!m_vao.isCreated())
  {
    return;
  }
  m_vao.bind();

  if (!m_hasColorBuffer)
  {
    setConstantVertexColor(m_baseColor);
  }
  glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(m_pointCount));

  glBindVertexArray(0);

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

// ============================================================================
// LinesProxyBase implementation
// ============================================================================

// CHANGE TO:
void LinesProxyBase::compile(const mrpt::viz::CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj)
  {
    return;
  }

  // Try to cast to VisualObjectParams_Lines
  const auto* linesObj = dynamic_cast<const mrpt::viz::VisualObjectParams_Lines*>(sourceObj);
  if (!linesObj)
  {
    return;
  }

  // Extract line parameters
  m_lineWidth = linesObj->getLineWidth();
  m_antiAliasing = linesObj->isAntiAliasingEnabled();

  // Lock the source object's buffer data
  std::shared_lock<std::shared_mutex> lck(linesObj->shaderLinesBufferMutex().data);

  const auto& vertices = linesObj->shaderLinesVertexPointBuffer();
  const auto& colors = linesObj->shaderLinesVertexColorBuffer();

  m_vertexCount = vertices.size();
  m_localBBox = boundingBoxOf(vertices);
  m_hasColorBuffer = !colors.empty() && colors.size() == m_vertexCount;
  m_baseColor = sourceObj->getColor_u8();
  m_transparent = m_hasColorBuffer ? anyTranslucent(colors) : m_baseColor.A != 0xff;
  if (m_vertexCount == 0)
  {
    return;
  }

  // Create VAO
  m_vao.createOnce();
  m_vao.bind();

  // Upload vertex positions
  m_vertexBuffer.createOnce();
  m_vertexBuffer.bind();
  m_vertexBuffer.allocate(vertices.data(), static_cast<int>(sizeof(TPoint3Df) * m_vertexCount));

  // Attribute 0: position (vec3)
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(TPoint3Df), nullptr);

  // Upload colors
  if (m_hasColorBuffer)
  {
    m_colorBuffer.createOnce();
    m_colorBuffer.bind();
    m_colorBuffer.allocate(colors.data(), static_cast<int>(sizeof(TColor) * m_vertexCount));

    // Attribute 1: color (vec4 as unsigned bytes, normalized)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(TColor), nullptr);
  }
  else
  {
    glDisableVertexAttribArray(1);
  }

  // Unbind
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

void LinesProxyBase::updateBuffers(const mrpt::viz::CVisualObject* sourceObj)
{
  compile(sourceObj);
}

void LinesProxyBase::render([[maybe_unused]] const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  MRPT_START

  if (m_vertexCount == 0)
  {
    return;
  }
  if (!m_vao.isCreated())
  {
    return;
  }
  // Note: glLineWidth >1.0 is unsupported on most modern GL implementations.
  // Thick lines require a geometry shader approach (future enhancement).
  // For now, always use 1.0 to avoid GL_INVALID_VALUE.
  glLineWidth(1.0f);

  // Enable anti-aliasing if requested
  if (m_antiAliasing)
  {
    glEnable(GL_LINE_SMOOTH);
  }

  m_vao.bind();

  if (!m_hasColorBuffer)
  {
    setConstantVertexColor(m_baseColor);
  }
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

// ============================================================================
// TrianglesProxyBase implementation
// ============================================================================

void TrianglesProxyBase::compile(const mrpt::viz::CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj)
  {
    return;
  }

  // NOTE: Caller must have called sourceObj->updateBuffers() before this
  // to ensure m_triangles is populated from the object's geometry parameters.

  // Try to cast to VisualObjectParams_Triangles
  const auto* triObj = dynamic_cast<const mrpt::viz::VisualObjectParams_Triangles*>(sourceObj);
  if (!triObj)
  {
    return;
  }

  m_lightEnabled = triObj->isLightEnabled();
  m_cullFace = triObj->cullFaces();

  // Lock the source object's buffer data
  std::shared_lock<std::shared_mutex> lck(triObj->shaderTrianglesBufferMutex().data);

  const auto& triangles = triObj->shaderTrianglesBuffer();

  m_triangleCount = triangles.size();
  if (m_triangleCount == 0)
  {
    m_localBBox.reset();
    m_transparent = false;
    return;
  }

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

      // rgba.r/g/b/a are uint8_t (TPointXYZfRGBAu8):
      const auto& rgba = tri.vertices[i].xyzrgba;
      colors.emplace_back(rgba.r, rgba.g, rgba.b, rgba.a);
    }
  }

  // Create VAO
  m_vao.createOnce();
  m_vao.bind();

  // Upload vertex positions
  m_vertexBuffer.createOnce();
  m_vertexBuffer.bind();
  m_vertexBuffer.allocate(vertices.data(), static_cast<int>(sizeof(TPoint3Df) * vertexCount));

  // Attribute 0: position (vec3)
  // Shader attrib order: position(0), vertexColor(1), vertexNormal(2)
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(TPoint3Df), nullptr);

  // Upload colors
  m_colorBuffer.createOnce();
  m_colorBuffer.bind();
  m_colorBuffer.allocate(colors.data(), static_cast<int>(sizeof(TColor) * vertexCount));

  // Attribute 1: vertexColor (vec4 as unsigned bytes, normalized)
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(TColor), nullptr);

  // Upload normals
  m_normalBuffer.createOnce();
  m_normalBuffer.bind();
  m_normalBuffer.allocate(normals.data(), static_cast<int>(sizeof(TVector3Df) * vertexCount));

  // Attribute 2: vertexNormal (vec3)
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(TVector3Df), nullptr);

  // Unbind
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  m_localBBox = boundingBoxOf(vertices);
  m_transparent = anyTranslucent(colors);

  CHECK_OPENGL_ERROR_IN_DEBUG();

  MRPT_END
#endif
}

void TrianglesProxyBase::updateBuffers(const mrpt::viz::CVisualObject* sourceObj)
{
  compile(sourceObj);
}

void TrianglesProxyBase::render([[maybe_unused]] const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  MRPT_START

  if (m_triangleCount == 0)
  {
    return;
  }
  if (!m_vao.isCreated())
  {
    return;
  }

  // Setup face culling. Culling only affects what the camera sees: all faces
  // cast shadows, so it is disabled while rendering the shadow map.
  bool cullingWasEnabled = glIsEnabled(GL_CULL_FACE);
  GLint previousCullMode = GL_BACK;
  glGetIntegerv(GL_CULL_FACE_MODE, &previousCullMode);
  const bool isShadowMap1stPass = rc.shader_id == DefaultShaderID::TRIANGLES_SHADOW_1ST ||
                                  rc.shader_id == DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_1ST;
  const auto cullFace = isShadowMap1stPass ? mrpt::viz::TCullFace::NONE : m_cullFace;

  if (cullFace != mrpt::viz::TCullFace::NONE)
  {
    glEnable(GL_CULL_FACE);

    if (cullFace == mrpt::viz::TCullFace::BACK)
    {
      glCullFace(GL_BACK);
    }
    else if (cullFace == mrpt::viz::TCullFace::FRONT)
    {
      glCullFace(GL_FRONT);
    }
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

// ============================================================================
// TexturedTrianglesProxyBase implementation
// ============================================================================

void TexturedTrianglesProxyBase::compile(const mrpt::viz::CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj)
  {
    return;
  }

  // Try to cast to VisualObjectParams_TexturedTriangles
  const auto* texTriObj =
      dynamic_cast<const mrpt::viz::VisualObjectParams_TexturedTriangles*>(sourceObj);
  if (!texTriObj)
  {
    // Fallback: compile as plain triangles
    TrianglesProxyBase::compile(sourceObj);
    return;
  }

  m_textureInterpolate = texTriObj->textureLinearInterpolation();
  m_textureMipMaps = texTriObj->textureMipMap();
  m_lightEnabled = texTriObj->isLightEnabled();
  m_cullFace = texTriObj->cullFaces();

  // Lock the source object's buffer data
  std::shared_lock<std::shared_mutex> lck(texTriObj->shaderTexturedTrianglesBufferMutex().data);

  const auto& triangles = texTriObj->shaderTexturedTrianglesBuffer();

  m_triangleCount = triangles.size();
  if (m_triangleCount == 0)
  {
    m_localBBox.reset();
    m_transparent = false;
    return;
  }

  // Extract flat arrays from TTriangle structures
  const size_t vertexCount = m_triangleCount * 3;

  std::vector<TPoint3Df> vertices;
  std::vector<TVector3Df> normals;
  std::vector<TColor> colors;
  std::vector<mrpt::math::TPoint2Df> texCoords;
  // Tangents as (x,y,z,w), with w=+1/-1 the handedness of the UV mapping
  std::vector<float> tangents;

  vertices.reserve(vertexCount);
  normals.reserve(vertexCount);
  colors.reserve(vertexCount);
  texCoords.reserve(vertexCount);
  tangents.reserve(vertexCount * 4);

  for (const auto& tri : triangles)
  {
    // Tangent and bitangent from edge vectors and UV deltas. The tangent is a
    // fallback for vertices without one; the bitangent gives the handedness of
    // the UV mapping (mirrored mappings need the bitangent flipped).
    const auto& p0 = tri.vertices[0].xyzrgba.pt;
    const auto& p1 = tri.vertices[1].xyzrgba.pt;
    const auto& p2 = tri.vertices[2].xyzrgba.pt;
    const TVector3Df e1(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
    const TVector3Df e2(p2.x - p0.x, p2.y - p0.y, p2.z - p0.z);
    const float du1 = tri.vertices[1].uv.x - tri.vertices[0].uv.x;
    const float dv1 = tri.vertices[1].uv.y - tri.vertices[0].uv.y;
    const float du2 = tri.vertices[2].uv.x - tri.vertices[0].uv.x;
    const float dv2 = tri.vertices[2].uv.y - tri.vertices[0].uv.y;
    const float det = du1 * dv2 - du2 * dv1;

    TVector3Df triTangent(1.0f, 0.0f, 0.0f);  // degenerate UV: arbitrary tangent
    float handedness = 1.0f;
    if (std::abs(det) > 1e-12f)
    {
      const float r = 1.0f / det;
      triTangent = (e1 * dv2 - e2 * dv1) * r;
      const TVector3Df triBitangent = (e2 * du1 - e1 * du2) * r;
      const auto cross = [](const TVector3Df& a, const TVector3Df& b)
      { return TVector3Df(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); };
      const TVector3Df NxT = cross(cross(e1, e2), triTangent);
      const float dotB = NxT.x * triBitangent.x + NxT.y * triBitangent.y + NxT.z * triBitangent.z;
      handedness = dotB < 0 ? -1.0f : 1.0f;
    }

    for (int i = 0; i < 3; ++i)
    {
      vertices.push_back(tri.vertices[i].xyzrgba.pt);
      normals.push_back(tri.vertices[i].normal);
      texCoords.emplace_back(tri.vertices[i].uv.x, tri.vertices[i].uv.y);

      // Use per-vertex tangent if available, else the computed per-triangle tangent
      const auto& vt = tri.vertices[i].tangent;
      const bool hasVertexTangent = vt.x != 0 || vt.y != 0 || vt.z != 0;
      const TVector3Df& t = hasVertexTangent ? vt : triTangent;
      tangents.insert(tangents.end(), {t.x, t.y, t.z, handedness});

      // rgba.r/g/b/a are uint8_t (TPointXYZfRGBAu8), use directly:
      const auto& rgba = tri.vertices[i].xyzrgba;
      colors.emplace_back(rgba.r, rgba.g, rgba.b, rgba.a);
    }
  }

  // Create VAO
  m_vao.createOnce();
  m_vao.bind();

  // Attribute 0: position (vec3)
  m_vertexBuffer.createOnce();
  m_vertexBuffer.bind();
  m_vertexBuffer.allocate(vertices.data(), static_cast<int>(sizeof(TPoint3Df) * vertexCount));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(TPoint3Df), nullptr);

  // Attribute 1: vertexColor (vec4 as unsigned bytes, normalized)
  m_colorBuffer.createOnce();
  m_colorBuffer.bind();
  m_colorBuffer.allocate(colors.data(), static_cast<int>(sizeof(TColor) * vertexCount));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(TColor), nullptr);

  // Attribute 2: vertexNormal (vec3)
  m_normalBuffer.createOnce();
  m_normalBuffer.bind();
  m_normalBuffer.allocate(normals.data(), static_cast<int>(sizeof(TVector3Df) * vertexCount));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(TVector3Df), nullptr);

  // Attribute 3: texture coordinates (vec2)
  m_texCoordBuffer.createOnce();
  m_texCoordBuffer.bind();
  m_texCoordBuffer.allocate(
      texCoords.data(), static_cast<int>(sizeof(mrpt::math::TPoint2Df) * vertexCount));
  glEnableVertexAttribArray(3);
  glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(mrpt::math::TPoint2Df), nullptr);

  // Attribute 4: tangent (vec3) for normal mapping
  m_tangentBuffer.createOnce();
  m_tangentBuffer.bind();
  m_tangentBuffer.allocate(tangents.data(), static_cast<int>(sizeof(float) * tangents.size()));
  glEnableVertexAttribArray(4);
  glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);

  // Unbind
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  m_localBBox = boundingBoxOf(vertices);
  m_transparent = anyTranslucent(colors);

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
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  MRPT_START

  // Bind texture if available
  if (m_texture)
  {
    m_texture->bindAsTexture2D();
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
