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

#include <mrpt/core/bits_math.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TrianglesProxy.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/TLightParameters.h>

#include <algorithm>
#include <cmath>

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
  m_params.materialEmissive = sourceObj->materialEmissive();

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

  // Emissive color
  if (rc.shader->hasUniform("materialEmissive"))
  {
    uploadVector3(
        rc, "materialEmissive",
        mrpt::math::TVector3Df(
            m_params.materialEmissive.R, m_params.materialEmissive.G, m_params.materialEmissive.B));
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

  // Multi-light parameters (if lighting enabled)
  if (m_params.lightEnabled && rc.lights)
  {
    const auto& lights = rc.lights->lights;
    const int numLights = static_cast<int>(std::min<size_t>(lights.size(), mrpt::viz::MAX_LIGHTS));

    if (rc.shader->hasUniform("num_lights"))
    {
      uploadInt(rc, "num_lights", numLights);
    }
    if (rc.shader->hasUniform("light_ambient"))
    {
      uploadFloat(rc, "light_ambient", rc.lights->ambient);
    }
    if (rc.shader->hasUniform("ambient_sky_color"))
    {
      const auto& c = rc.lights->ambientSkyColor;
      uploadVector3(rc, "ambient_sky_color", mrpt::math::TVector3Df(c.R, c.G, c.B));
    }
    if (rc.shader->hasUniform("ambient_ground_color"))
    {
      const auto& c = rc.lights->ambientGroundColor;
      uploadVector3(rc, "ambient_ground_color", mrpt::math::TVector3Df(c.R, c.G, c.B));
    }

    // Build arrays and upload via raw GL calls
    if (numLights > 0 && rc.shader->hasUniform("light_type"))
    {
      int types[mrpt::viz::MAX_LIGHTS] = {};
      float colors[mrpt::viz::MAX_LIGHTS * 3] = {};
      float diffuses[mrpt::viz::MAX_LIGHTS] = {};
      float speculars[mrpt::viz::MAX_LIGHTS] = {};
      float directions[mrpt::viz::MAX_LIGHTS * 3] = {};
      float positions[mrpt::viz::MAX_LIGHTS * 3] = {};
      float attenuations[mrpt::viz::MAX_LIGHTS * 3] = {};
      float spotCutoffs[mrpt::viz::MAX_LIGHTS * 2] = {};

      for (int i = 0; i < numLights; i++)
      {
        const auto& l = lights[i];
        types[i] = static_cast<int>(l.type);
        colors[i * 3 + 0] = l.color.R;
        colors[i * 3 + 1] = l.color.G;
        colors[i * 3 + 2] = l.color.B;
        diffuses[i] = l.diffuse;
        speculars[i] = l.specular;
        directions[i * 3 + 0] = l.direction.x;
        directions[i * 3 + 1] = l.direction.y;
        directions[i * 3 + 2] = l.direction.z;
        positions[i * 3 + 0] = l.position.x;
        positions[i * 3 + 1] = l.position.y;
        positions[i * 3 + 2] = l.position.z;
        attenuations[i * 3 + 0] = l.attenuation_constant;
        attenuations[i * 3 + 1] = l.attenuation_linear;
        attenuations[i * 3 + 2] = l.attenuation_quadratic;
        spotCutoffs[i * 2 + 0] = std::cos(mrpt::DEG2RAD(l.spot_inner_cutoff_deg));
        spotCutoffs[i * 2 + 1] = std::cos(mrpt::DEG2RAD(l.spot_outer_cutoff_deg));
      }

      glUniform1iv(rc.shader->uniformId("light_type"), numLights, types);
      glUniform3fv(rc.shader->uniformId("light_color"), numLights, colors);
      glUniform1fv(rc.shader->uniformId("light_diffuse"), numLights, diffuses);
      glUniform1fv(rc.shader->uniformId("light_specular"), numLights, speculars);
      glUniform3fv(rc.shader->uniformId("light_direction"), numLights, directions);
      glUniform3fv(rc.shader->uniformId("light_position"), numLights, positions);
      glUniform3fv(rc.shader->uniformId("light_attenuation"), numLights, attenuations);
      glUniform2fv(rc.shader->uniformId("light_spot_cutoff"), numLights, spotCutoffs);
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
