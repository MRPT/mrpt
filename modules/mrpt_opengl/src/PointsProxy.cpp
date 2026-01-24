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

#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CVisualObject.h>

using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt::viz;

void PointsProxy::compile(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj) return;

  // Extract point rendering parameters
  extractPointParams(sourceObj);

  // Call base class to upload vertex/color data
  PointsProxyBase::compile(sourceObj);

  MRPT_END
#endif
}

void PointsProxy::updateBuffers(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj) return;

  // Update cached parameters
  extractPointParams(sourceObj);

  // Update buffers
  PointsProxyBase::updateBuffers(sourceObj);

  MRPT_END
#endif
}

void PointsProxy::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (m_pointCount == 0) return;

  // Upload point-specific uniforms
  uploadPointUniforms(rc);

  // Call base class render
  PointsProxyBase::render(rc);

  MRPT_END
#endif
}

void PointsProxy::extractPointParams(const CVisualObject* sourceObj)
{
  const auto* pointsObj = dynamic_cast<const VisualObjectParams_Points*>(sourceObj);
  if (!pointsObj) return;

  m_params.pointSize = pointsObj->getPointSize();
  m_params.variablePointSize = pointsObj->isEnabledVariablePointSize();
  m_params.variablePointSize_K = pointsObj->getVariablePointSize_k();
  m_params.variablePointSize_DepthScale = pointsObj->getVariablePointSize_DepthScale();
}

void PointsProxy::uploadPointUniforms(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!rc.shader) return;

  // Point size uniform
  if (rc.shader->hasUniform("vertexPointSize"))
  {
    uploadFloat(rc, "vertexPointSize", m_params.pointSize);
  }

  // Variable point size parameters
  if (rc.shader->hasUniform("enableVariablePointSize"))
  {
    uploadInt(rc, "enableVariablePointSize", m_params.variablePointSize ? 1 : 0);
  }

  if (m_params.variablePointSize)
  {
    if (rc.shader->hasUniform("variablePointSize_K"))
    {
      uploadFloat(rc, "variablePointSize_K", m_params.variablePointSize_K);
    }
    if (rc.shader->hasUniform("variablePointSize_DepthScale"))
    {
      uploadFloat(rc, "variablePointSize_DepthScale", m_params.variablePointSize_DepthScale);
    }
  }
#endif
}
