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

#include <mrpt/opengl/LinesProxy.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CVisualObject.h>

using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt::viz;

void LinesProxy::compile(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj) return;

  // Extract line rendering parameters
  extractLineParams(sourceObj);

  // Call base class to setup VAO
  LinesProxyBase::compile(sourceObj);

  // Note: Actual line vertex data population is handled by derived classes
  // or by specific line object types (CSetOfLines, etc.)
  // This class provides the infrastructure and rendering.

  MRPT_END
#endif
}

void LinesProxy::updateBuffers(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj) return;

  // Update cached parameters
  extractLineParams(sourceObj);

  // Update buffers via base class
  LinesProxyBase::updateBuffers(sourceObj);

  MRPT_END
#endif
}

void LinesProxy::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (m_vertexCount == 0) return;

  // Setup OpenGL line state
  setupLineState();

  // Upload line-specific uniforms
  uploadLineUniforms(rc);

  // Call base class render
  LinesProxyBase::render(rc);

  // Restore OpenGL line state
  restoreLineState();

  MRPT_END
#endif
}

void LinesProxy::extractLineParams(const CVisualObject* sourceObj)
{
  const auto* linesObj = dynamic_cast<const VisualObjectParams_Lines*>(sourceObj);
  if (linesObj)
  {
    m_params.lineWidth = linesObj->getLineWidth();
    m_params.antiAliasing = linesObj->isAntiAliasingEnabled();
  }
  else
  {
    // Use defaults
    m_params.lineWidth = 1.0f;
    m_params.antiAliasing = false;
  }
}

void LinesProxy::uploadLineUniforms(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!rc.shader) return;

  // Line width might also be a uniform in some shader implementations
  if (rc.shader->hasUniform("lineWidth"))
  {
    uploadFloat(rc, "lineWidth", m_params.lineWidth);
  }
#endif
}

void LinesProxy::setupLineState() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  // Set line width
  glLineWidth(m_params.lineWidth);

  // Enable anti-aliasing if requested
  if (m_params.antiAliasing)
  {
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  }

  // Enable blending for smooth lines
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
#endif
}

void LinesProxy::restoreLineState() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  // Restore default line width
  glLineWidth(1.0f);

  // Disable anti-aliasing
  if (m_params.antiAliasing)
  {
    glDisable(GL_LINE_SMOOTH);
  }

  // Note: Don't disable blending here as other objects might need it
  // The rendering pipeline should manage global state
#endif
}
