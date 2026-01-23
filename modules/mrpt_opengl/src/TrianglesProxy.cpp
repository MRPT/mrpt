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

#include <mrpt/opengl/TrianglesProxy.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/TTriangle.h>

using namespace mrpt::opengl;
using namespace mrpt::viz;

void TrianglesProxy::compile(const mrpt::viz::CVisualObject* sourceObj)
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  ASSERT_(sourceObj != nullptr);

  // Dynamic cast to access triangle-specific interface
  const auto* trianglesObj = dynamic_cast<const VisualObjectParams_Triangles*>(sourceObj);
  if (!trianglesObj)
  {
    THROW_EXCEPTION_FMT(
        "TrianglesProxy::compile() called with incompatible object type: %s",
        sourceObj->GetRuntimeClass()->className);
  }

  // Extract rendering parameters
  extractTriangleParams(sourceObj);

  // Lock the source object's triangle data for reading
  std::shared_lock<std::shared_mutex> lock(trianglesObj->shaderTrianglesBufferMutex().data);

  const auto& triangles = trianglesObj->shaderTrianglesBuffer();

  m_triangleCount = triangles.size();

  // Invalidate cached bbox
  m_cachedBBox.reset();

  if (m_triangleCount == 0)
  {
    // Empty mesh - nothing to upload
    return;
  }

  // Convert triangle array to separate vertex/normal/color arrays
  std::vector<mrpt::math::TPoint3Df> vertices;
  std::vector<mrpt::math::TVector3Df> normals;
  std::vector<mrpt::img::TColor> colors;

  extractTriangleData(triangles, vertices, normals, colors);

  // Each triangle has 3 vertices
  const size_t vertexCount = m_triangleCount * 3;

  ASSERT_EQUAL_(vertices.size(), vertexCount);
  ASSERT_EQUAL_(normals.size(), vertexCount);
  ASSERT_EQUAL_(colors.size(), vertexCount);

  // Create VAO if needed
  if (!m_vao.created())
  {
    m_vao.createOnce();
  }
  m_vao.bind();

  // Upload vertex positions
  if (!m_vertexBuffer.created())
  {
    m_vertexBuffer.createOnce();
  }
  m_vertexBuffer.bind();
  m_vertexBuffer.allocate(vertices.data(), sizeof(vertices[0]) * vertices.size());
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Upload vertex normals
  if (!m_normalBuffer.created())
  {
    m_normalBuffer.createOnce();
  }
  m_normalBuffer.bind();
  m_normalBuffer.allocate(normals.data(), sizeof(normals[0]) * normals.size());
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Upload vertex colors
  if (!m_colorBuffer.created())
  {
    m_colorBuffer.createOnce();
  }
  m_colorBuffer.bind();
  m_colorBuffer.allocate(colors.data(), sizeof(colors[0]) * colors.size());
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Unbind VAO
  m_vao.release();

#endif

  MRPT_END
}

void TrianglesProxy::updateBuffers(const mrpt::viz::CVisualObject* sourceObj)
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  ASSERT_(sourceObj != nullptr);

  const auto* trianglesObj = dynamic_cast<const VisualObjectParams_Triangles*>(sourceObj);
  if (!trianglesObj)
  {
    THROW_EXCEPTION_FMT(
        "TrianglesProxy::updateBuffers() called with incompatible object type: %s",
        sourceObj->GetRuntimeClass()->className);
  }

  // Update rendering parameters
  extractTriangleParams(sourceObj);

  // Lock and read triangle data
  std::shared_lock<std::shared_mutex> lock(trianglesObj->shaderTrianglesBufferMutex().data);

  const auto& triangles = trianglesObj->shaderTrianglesBuffer();

  const size_t newTriangleCount = triangles.size();

  // Invalidate cached bbox
  m_cachedBBox.reset();

  if (newTriangleCount == 0)
  {
    m_triangleCount = 0;
    return;
  }

  // Extract triangle data
  std::vector<mrpt::math::TPoint3Df> vertices;
  std::vector<mrpt::math::TVector3Df> normals;
  std::vector<mrpt::img::TColor> colors;

  extractTriangleData(triangles, vertices, normals, colors);

  const size_t vertexCount = newTriangleCount * 3;

  // Check if we need to reallocate or just update
  const bool needsRealloc = (newTriangleCount != m_triangleCount);

  m_triangleCount = newTriangleCount;

  // Bind VAO
  m_vao.bind();

  // Update vertex buffer
  m_vertexBuffer.bind();
  if (needsRealloc)
  {
    m_vertexBuffer.allocate(vertices.data(), sizeof(vertices[0]) * vertices.size());
  }
  else
  {
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices[0]) * vertices.size(), vertices.data());
  }
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Update normal buffer
  m_normalBuffer.bind();
  if (needsRealloc)
  {
    m_normalBuffer.allocate(normals.data(), sizeof(normals[0]) * normals.size());
  }
  else
  {
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(normals[0]) * normals.size(), normals.data());
  }
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Update color buffer
  m_colorBuffer.bind();
  if (needsRealloc)
  {
    m_colorBuffer.allocate(colors.data(), sizeof(colors[0]) * colors.size());
  }
  else
  {
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(colors[0]) * colors.size(), colors.data());
  }
  CHECK_OPENGL_ERROR_IN_DEBUG();

  m_vao.release();

#endif

  MRPT_END
}

void TrianglesProxy::render(const RenderContext& rc) const
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  if (m_triangleCount == 0) return;

  ASSERT_(rc.shader != nullptr);
  ASSERT_(rc.state != nullptr);

  // Upload triangle-specific uniforms
  uploadTriangleUniforms(rc);

  // Setup face culling
  setupFaceCulling();

  // Bind VAO
  m_vao.bind();

  // Setup vertex attribute: position
  if (rc.shader->hasAttribute("position"))
  {
    const GLuint attrPosition = rc.shader->attributeId("position");
    glEnableVertexAttribArray(attrPosition);
    m_vertexBuffer.bind();
    glVertexAttribPointer(
        attrPosition,     // attribute location
        3,                // size (x, y, z)
        GL_FLOAT,         // type
        GL_FALSE,         // normalized?
        0,                // stride
        BUFFER_OFFSET(0)  // offset
    );
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }

  // Setup vertex attribute: normal (if lighting is enabled and not in shadow 1st pass)
  if (rc.shader->hasAttribute("vertexNormal") && !rc.state->is1stShadowMapPass)
  {
    const GLuint attrNormal = rc.shader->attributeId("vertexNormal");
    glEnableVertexAttribArray(attrNormal);
    m_normalBuffer.bind();
    glVertexAttribPointer(
        attrNormal,       // attribute location
        3,                // size (nx, ny, nz)
        GL_FLOAT,         // type
        GL_FALSE,         // normalized?
        0,                // stride
        BUFFER_OFFSET(0)  // offset
    );
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }

  // Setup vertex attribute: color
  if (rc.shader->hasAttribute("vertexColor"))
  {
    const GLuint attrColor = rc.shader->attributeId("vertexColor");
    glEnableVertexAttribArray(attrColor);
    m_colorBuffer.bind();
    glVertexAttribPointer(
        attrColor,         // attribute location
        4,                 // size (R, G, B, A)
        GL_UNSIGNED_BYTE,  // type
        GL_TRUE,           // normalized (converts 0-255 to 0.0-1.0)
        0,                 // stride
        BUFFER_OFFSET(0)   // offset
    );
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }

  // Draw the triangles
  const size_t vertexCount = m_triangleCount * 3;
  glDrawArrays(GL_TRIANGLES, 0, vertexCount);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Cleanup: disable vertex attributes
  if (rc.shader->hasAttribute("position"))
  {
    glDisableVertexAttribArray(rc.shader->attributeId("position"));
  }
  if (rc.shader->hasAttribute("vertexNormal"))
  {
    glDisableVertexAttribArray(rc.shader->attributeId("vertexNormal"));
  }
  if (rc.shader->hasAttribute("vertexColor"))
  {
    glDisableVertexAttribArray(rc.shader->attributeId("vertexColor"));
  }

  m_vao.release();

  // Restore face culling state
  restoreFaceCulling();

#endif

  MRPT_END
}

std::vector<shader_id_t> TrianglesProxy::requiredShaders() const
{
  // Return appropriate shader based on lighting settings
  if (m_params.lightEnabled)
  {
    return {DefaultShaderID::TRIANGLES_LIGHT};
  }
  else
  {
    return {DefaultShaderID::TRIANGLES_NO_LIGHT};
  }
}

void TrianglesProxy::extractTriangleParams(const mrpt::viz::CVisualObject* sourceObj)
{
  MRPT_START

  const auto* trianglesObj = dynamic_cast<const VisualObjectParams_Triangles*>(sourceObj);
  ASSERT_(trianglesObj != nullptr);

  // Extract triangle rendering parameters
  m_params.lightEnabled = trianglesObj->isLightEnabled();
  m_params.cullFace = trianglesObj->cullFaces();

  // Extract material properties from base CVisualObject
  m_params.materialShininess = sourceObj->materialShininess();

  MRPT_END
}

void TrianglesProxy::uploadTriangleUniforms(const RenderContext& rc) const
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  ASSERT_(rc.shader != nullptr);
  ASSERT_(rc.lights != nullptr);

  // Upload lighting parameters (if shader uses them)
  if (m_params.lightEnabled && !rc.state->is1stShadowMapPass)
  {
    if (rc.shader->hasUniform("light_diffuse"))
    {
      glUniform4f(
          rc.shader->uniformId("light_diffuse"), rc.lights->diffuse.R, rc.lights->diffuse.G,
          rc.lights->diffuse.B, rc.lights->diffuse.A);
    }

    if (rc.shader->hasUniform("light_ambient"))
    {
      glUniform4f(
          rc.shader->uniformId("light_ambient"), rc.lights->ambient.R, rc.lights->ambient.G,
          rc.lights->ambient.B, rc.lights->ambient.A);
    }

    if (rc.shader->hasUniform("light_specular"))
    {
      glUniform4f(
          rc.shader->uniformId("light_specular"), rc.lights->specular.R, rc.lights->specular.G,
          rc.lights->specular.B, rc.lights->specular.A);
    }

    if (rc.shader->hasUniform("light_direction"))
    {
      glUniform3f(
          rc.shader->uniformId("light_direction"), rc.lights->direction.x, rc.lights->direction.y,
          rc.lights->direction.z);
    }
  }

  // Material shininess is uploaded by CompiledViewport in common uniforms,
  // but we can override if needed

#endif

  MRPT_END
}

void TrianglesProxy::setupFaceCulling() const
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  switch (m_params.cullFace)
  {
    case mrpt::viz::TCullFace::NONE:
      glDisable(GL_CULL_FACE);
      break;

    case mrpt::viz::TCullFace::BACK:
      glEnable(GL_CULL_FACE);
      glCullFace(GL_BACK);
      break;

    case mrpt::viz::TCullFace::FRONT:
      glEnable(GL_CULL_FACE);
      glCullFace(GL_FRONT);
      break;
  }

  CHECK_OPENGL_ERROR_IN_DEBUG();

#endif

  MRPT_END
}

void TrianglesProxy::restoreFaceCulling() const
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  // Restore to default: no culling
  if (m_params.cullFace != mrpt::viz::TCullFace::NONE)
  {
    glDisable(GL_CULL_FACE);
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }

#endif

  MRPT_END
}

void TrianglesProxy::extractTriangleData(
    const std::vector<mrpt::viz::TTriangle>& triangles,
    std::vector<mrpt::math::TPoint3Df>& vertices,
    std::vector<mrpt::math::TVector3Df>& normals,
    std::vector<mrpt::img::TColor>& colors) const
{
  MRPT_START

  const size_t triangleCount = triangles.size();
  const size_t vertexCount = triangleCount * 3;

  // Reserve space
  vertices.reserve(vertexCount);
  normals.reserve(vertexCount);
  colors.reserve(vertexCount);

  // Extract data from TTriangle structs
  for (const auto& tri : triangles)
  {
    // Vertex 0
    vertices.push_back(tri.vertices[0].xyzrgba.pt);
    normals.push_back(tri.vertices[0].normal);
    colors.push_back(tri.vertices[0].xyzrgba.r);

    // Vertex 1
    vertices.push_back(tri.vertices[1].xyzrgba.pt);
    normals.push_back(tri.vertices[1].normal);
    colors.push_back(tri.vertices[1].xyzrgba.r);

    // Vertex 2
    vertices.push_back(tri.vertices[2].xyzrgba.pt);
    normals.push_back(tri.vertices[2].normal);
    colors.push_back(tri.vertices[2].xyzrgba.r);
  }

  MRPT_END
}

// ========== TrianglesProxyBase Implementation ==========

void TrianglesProxyBase::compile(const mrpt::viz::CVisualObject* sourceObj)
{
  // Default implementation - derived classes should override
  THROW_EXCEPTION("TrianglesProxyBase::compile() should be overridden in derived class");
}

void TrianglesProxyBase::updateBuffers(const mrpt::viz::CVisualObject* sourceObj)
{
  // Default: just recompile everything
  compile(sourceObj);
}

void TrianglesProxyBase::render(const RenderContext& rc) const
{
  // Default implementation - derived classes should override
  THROW_EXCEPTION("TrianglesProxyBase::render() should be overridden in derived class");
}

std::vector<shader_id_t> TrianglesProxyBase::requiredShaders() const
{
  // Default: use lit triangles
  return {DefaultShaderID::TRIANGLES_LIGHT};
}

mrpt::math::TBoundingBoxf TrianglesProxyBase::getBoundingBoxLocal() const
{
  MRPT_START

  // Return cached bbox if available
  if (m_cachedBBox.has_value())
  {
    return m_cachedBBox.value();
  }

  // Would need to compute from vertices - derived classes should cache during compile()
  return mrpt::math::TBoundingBoxf();

  MRPT_END
}

shader_id_t TrianglesProxyBase::selectShader(bool isShadowMapPass) const
{
  if (isShadowMapPass)
  {
    // 1st pass: depth-only shader
    return DefaultShaderID::TRIANGLES_SHADOW_1ST;
  }
  else
  {
    // 2nd pass: depends on lighting
    if (m_lightEnabled)
    {
      return DefaultShaderID::TRIANGLES_SHADOW_2ND;
    }
    else
    {
      return DefaultShaderID::TRIANGLES_NO_LIGHT;
    }
  }
}