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

#include <mrpt/opengl/PointsProxy.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CVisualObject.h>

using namespace mrpt::opengl;
using namespace mrpt::viz;

void PointsProxy::compile(const mrpt::viz::CVisualObject* sourceObj)
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  ASSERT_(sourceObj != nullptr);

  // Dynamic cast to access point-specific interface
  const auto* pointsObj = dynamic_cast<const VisualObjectParams_Points*>(sourceObj);
  if (!pointsObj)
  {
    THROW_EXCEPTION_FMT(
        "PointsProxy::compile() called with incompatible object type: %s",
        sourceObj->GetRuntimeClass()->className);
  }

  // Extract rendering parameters
  extractPointParams(sourceObj);

  // Lock the source object's point data for reading
  std::shared_lock<std::shared_mutex> lock(pointsObj->shaderPointsBuffersMutex().data);

  const auto& vertices = pointsObj->shaderPointsVertexPointBuffer();
  const auto& colors = pointsObj->shaderPointsVertexColorBuffer();

  m_pointCount = vertices.size();

  // Invalidate cached bbox
  m_cachedBBox.reset();

  if (m_pointCount == 0)
  {
    // Empty point cloud - nothing to upload
    return;
  }

  // Ensure vertex and color buffers have the same size
  ASSERT_EQUAL_(vertices.size(), colors.size());

  // Create VAO if needed
  if (!m_vao.created())
  {
    m_vao.createOnce();
  }
  m_vao.bind();

  // Upload vertex positions (3D points)
  if (!m_vertexBuffer.created())
  {
    m_vertexBuffer.createOnce();
  }
  m_vertexBuffer.bind();
  m_vertexBuffer.allocate(vertices.data(), sizeof(vertices[0]) * vertices.size());
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Upload vertex colors (RGBA)
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

void PointsProxy::updateBuffers(const mrpt::viz::CVisualObject* sourceObj)
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  ASSERT_(sourceObj != nullptr);

  const auto* pointsObj = dynamic_cast<const VisualObjectParams_Points*>(sourceObj);
  if (!pointsObj)
  {
    THROW_EXCEPTION_FMT(
        "PointsProxy::updateBuffers() called with incompatible object type: %s",
        sourceObj->GetRuntimeClass()->className);
  }

  // Update rendering parameters
  extractPointParams(sourceObj);

  // Lock and read point data
  std::shared_lock<std::shared_mutex> lock(pointsObj->shaderPointsBuffersMutex().data);

  const auto& vertices = pointsObj->shaderPointsVertexPointBuffer();
  const auto& colors = pointsObj->shaderPointsVertexColorBuffer();

  const size_t newPointCount = vertices.size();

  // Invalidate cached bbox
  m_cachedBBox.reset();

  if (newPointCount == 0)
  {
    m_pointCount = 0;
    return;
  }

  ASSERT_EQUAL_(vertices.size(), colors.size());

  // Check if we need to reallocate or just update
  const bool needsRealloc = (newPointCount != m_pointCount);

  m_pointCount = newPointCount;

  // Bind VAO
  m_vao.bind();

  // Update vertex buffer
  m_vertexBuffer.bind();
  if (needsRealloc)
  {
    // Reallocate with new size
    m_vertexBuffer.allocate(vertices.data(), sizeof(vertices[0]) * vertices.size());
  }
  else
  {
    // Update existing buffer
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices[0]) * vertices.size(), vertices.data());
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

void PointsProxy::render(const RenderContext& rc) const
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  if (m_pointCount == 0) return;

  ASSERT_(rc.shader != nullptr);
  ASSERT_(rc.state != nullptr);

  // Skip rendering points in shadow map pass (they don't cast shadows)
  if (rc.state->is1stShadowMapPass) return;

  // Upload point-specific uniforms
  uploadPointUniforms(rc);

  // Bind VAO (this restores all vertex attribute state)
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

  // Draw the points
  glDrawArrays(GL_POINTS, 0, m_pointCount);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Cleanup: disable vertex attributes
  if (rc.shader->hasAttribute("position"))
  {
    glDisableVertexAttribArray(rc.shader->attributeId("position"));
  }
  if (rc.shader->hasAttribute("vertexColor"))
  {
    glDisableVertexAttribArray(rc.shader->attributeId("vertexColor"));
  }

  m_vao.release();

#endif

  MRPT_END
}

void PointsProxy::extractPointParams(const mrpt::viz::CVisualObject* sourceObj)
{
  MRPT_START

  const auto* pointsObj = dynamic_cast<const VisualObjectParams_Points*>(sourceObj);
  ASSERT_(pointsObj != nullptr);

  // Extract point rendering parameters
  m_params.pointSize = pointsObj->getPointSize();
  m_params.variablePointSize = pointsObj->isEnabledVariablePointSize();
  m_params.variablePointSize_K = pointsObj->getVariablePointSize_k();
  m_params.variablePointSize_DepthScale = pointsObj->getVariablePointSize_DepthScale();

  MRPT_END
}

void PointsProxy::uploadPointUniforms(const RenderContext& rc) const
{
  MRPT_START

#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL

  ASSERT_(rc.shader != nullptr);

  // Upload point size
  if (rc.shader->hasUniform("vertexPointSize"))
  {
    glUniform1f(rc.shader->uniformId("vertexPointSize"), m_params.pointSize);
  }

  // Upload variable point size parameters
  if (rc.shader->hasUniform("enableVariablePointSize"))
  {
    glUniform1i(
        rc.shader->uniformId("enableVariablePointSize"), m_params.variablePointSize ? 1 : 0);
  }

  if (rc.shader->hasUniform("variablePointSize_K"))
  {
    glUniform1f(rc.shader->uniformId("variablePointSize_K"), m_params.variablePointSize_K);
  }

  if (rc.shader->hasUniform("variablePointSize_DepthScale"))
  {
    glUniform1f(
        rc.shader->uniformId("variablePointSize_DepthScale"),
        m_params.variablePointSize_DepthScale);
  }

#endif

  MRPT_END
}

// ========== PointsProxyBase Implementation ==========

void PointsProxyBase::compile(const mrpt::viz::CVisualObject* sourceObj)
{
  // Default implementation - derived classes should override
  THROW_EXCEPTION("PointsProxyBase::compile() should be overridden in derived class");
}

void PointsProxyBase::updateBuffers(const mrpt::viz::CVisualObject* sourceObj)
{
  // Default: just recompile everything
  compile(sourceObj);
}

void PointsProxyBase::render(const RenderContext& rc) const
{
  // Default implementation - derived classes should override
  THROW_EXCEPTION("PointsProxyBase::render() should be overridden in derived class");
}

mrpt::math::TBoundingBoxf PointsProxyBase::getBoundingBoxLocal() const
{
  MRPT_START

  // Return cached bbox if available
  if (m_cachedBBox.has_value())
  {
    return m_cachedBBox.value();
  }

  // Need to compute bbox from vertex buffer
  // This would require reading back from GPU or caching vertices CPU-side
  // For now, return empty bbox - derived classes should cache during compile()

  return mrpt::math::TBoundingBoxf();

  MRPT_END
}