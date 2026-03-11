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

#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TrianglesProxy.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/TLightParameters.h>

using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt::viz;

void TrianglesProxy::compile(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj)
  {
    return;
  }
  // Extract triangle rendering parameters
  extractTriangleParams(sourceObj);

  // Call base class to upload vertex/normal/color data
  TrianglesProxyBase::compile(sourceObj);

  MRPT_END
#endif
}

void TrianglesProxy::updateBuffers(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj)
  {
    return;
  }
  // Update cached parameters
  extractTriangleParams(sourceObj);

  // Update buffers
  TrianglesProxyBase::updateBuffers(sourceObj);

  MRPT_END
#endif
}

void TrianglesProxy::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (m_triangleCount == 0)
  {
    return;
  }
  // Setup face culling
  setupFaceCulling();

  // Upload triangle-specific uniforms
  uploadTriangleUniforms(rc);

  // Call base class render (handles VAO binding and draw call)
  TrianglesProxyBase::render(rc);

  // Restore face culling state
  restoreFaceCulling();

  MRPT_END
#endif
}

std::vector<shader_id_t> TrianglesProxy::requiredShaders() const
{
  // Only return the base shader here. Shadow shader variants are selected
  // at render time based on the rendering pass (shadow map vs normal).
  if (m_params.lightEnabled)
    return {DefaultShaderID::TRIANGLES_LIGHT};
  else
    return {DefaultShaderID::TRIANGLES_NO_LIGHT};
}

void TrianglesProxy::extractTriangleParams(const CVisualObject* sourceObj)
{
  // Get material params from base object
  m_params.materialShininess = sourceObj->materialShininess();
  m_params.materialSpecularExponent = sourceObj->materialSpecularExponent();

  // Get triangle-specific params
  const auto* triObj = dynamic_cast<const VisualObjectParams_Triangles*>(sourceObj);
  if (triObj)
  {
    m_params.lightEnabled = triObj->isLightEnabled();
    m_params.cullFace = triObj->cullFaces();
  }
}

void TrianglesProxy::uploadTriangleUniforms(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!rc.shader)
  {
    return;
  }
  // Material specular intensity (shininess)
  if (rc.shader->hasUniform("materialSpecular"))
  {
    uploadFloat(rc, "materialSpecular", m_params.materialShininess);
  }

  // Blinn-Phong specular exponent
  if (rc.shader->hasUniform("materialSpecularExponent"))
  {
    uploadFloat(rc, "materialSpecularExponent", m_params.materialSpecularExponent);
  }

  // Camera position (needed for specular lighting)
  if (rc.shader->hasUniform("cam_position") && rc.state != nullptr)
  {
    const auto& e = rc.state->eye;
    uploadVector3(
        rc, "cam_position",
        mrpt::math::TVector3Df(
            static_cast<float>(e.x), static_cast<float>(e.y), static_cast<float>(e.z)));
  }

  // Light parameters (if lighting enabled)
  if (m_params.lightEnabled && rc.lights)
  {
    if (rc.shader->hasUniform("light_diffuse"))
    {
      uploadFloat(rc, "light_diffuse", rc.lights->diffuse);
    }
    if (rc.shader->hasUniform("light_ambient"))
    {
      uploadFloat(rc, "light_ambient", rc.lights->ambient);
    }
    if (rc.shader->hasUniform("light_specular"))
    {
      uploadFloat(rc, "light_specular", rc.lights->specular);
    }
    if (rc.shader->hasUniform("light_direction"))
    {
      uploadVector3(rc, "light_direction", rc.lights->direction);
    }
    if (rc.shader->hasUniform("light_color"))
    {
      uploadColor(rc, "light_color", rc.lights->color);
    }
  }

  // Shadow parameters (if in shadow pass)
  if (rc.isShadowMapPass && rc.lights)
  {
    if (rc.shader->hasUniform("shadow_bias"))
    {
      uploadFloat(rc, "shadow_bias", rc.lights->shadow_bias);
    }
    if (rc.shader->hasUniform("shadow_bias_cam2frag"))
    {
      uploadFloat(rc, "shadow_bias_cam2frag", rc.lights->shadow_bias_cam2frag);
    }
    if (rc.shader->hasUniform("shadow_bias_normal"))
    {
      uploadFloat(rc, "shadow_bias_normal", rc.lights->shadow_bias_normal);
    }
  }
#endif
}

void TrianglesProxy::setupFaceCulling() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  switch (m_params.cullFace)
  {
    case TCullFace::NONE:
      glDisable(GL_CULL_FACE);
      break;
    case TCullFace::BACK:
      glEnable(GL_CULL_FACE);
      glCullFace(GL_BACK);
      break;
    case TCullFace::FRONT:
      glEnable(GL_CULL_FACE);
      glCullFace(GL_FRONT);
      break;
  }
#endif
}

void TrianglesProxy::restoreFaceCulling() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  // Restore to default state (culling disabled)
  glDisable(GL_CULL_FACE);
#endif
}

void TrianglesProxy::extractTriangleData(
    const std::vector<TTriangle>& triangles,
    std::vector<TPoint3Df>& vertices,
    std::vector<TVector3Df>& normals,
    std::vector<TColor>& colors) const
{
  const size_t vertexCount = triangles.size() * 3;

  vertices.clear();
  normals.clear();
  colors.clear();

  vertices.reserve(vertexCount);
  normals.reserve(vertexCount);
  colors.reserve(vertexCount);

  for (const auto& tri : triangles)
  {
    for (int i = 0; i < 3; ++i)
    {
      vertices.push_back(tri.vertices[i].xyzrgba.pt);
      normals.push_back(tri.vertices[i].normal);

      // RGBA is already uint8_t (0-255)
      const auto& rgba = tri.vertices[i].xyzrgba;
      colors.emplace_back(rgba.r, rgba.g, rgba.b, rgba.a);
    }
  }
}
