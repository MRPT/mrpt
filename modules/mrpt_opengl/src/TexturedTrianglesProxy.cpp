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
#include <mrpt/img/CImage.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TexturedTrianglesProxy.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/TLightParameters.h>

#include <algorithm>
#include <cmath>

using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt::viz;

void TexturedTrianglesProxy::compile(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj)
  {
    return;
  }

  // Extract texture rendering parameters
  extractTextureParams(sourceObj);

  // Call base class to upload vertex/normal/color/texcoord data
  TexturedTrianglesProxyBase::compile(sourceObj);

  // Create/update texture from source image
  const auto* texTriObj = dynamic_cast<const VisualObjectParams_TexturedTriangles*>(sourceObj);
  if (texTriObj)
  {
    if (texTriObj->textureImageHasBeenAssigned())
    {
      updateTexture(texTriObj);
    }
    if (texTriObj->normalMapHasBeenAssigned())
    {
      updateNormalMapTexture(texTriObj);
    }
  }

  MRPT_END
#endif
}

void TexturedTrianglesProxy::updateBuffers(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (!sourceObj)
  {
    return;
  }

  // Update cached parameters
  extractTextureParams(sourceObj);

  // Update buffers via base class
  TexturedTrianglesProxyBase::updateBuffers(sourceObj);

  // Update texture if image changed
  const auto* texTriObj = dynamic_cast<const VisualObjectParams_TexturedTriangles*>(sourceObj);
  if (texTriObj)
  {
    if (texTriObj->textureImageHasBeenAssigned())
    {
      updateTexture(texTriObj);
    }
    if (texTriObj->normalMapHasBeenAssigned())
    {
      updateNormalMapTexture(texTriObj);
    }
  }

  MRPT_END
#endif
}

void TexturedTrianglesProxy::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  MRPT_START

  if (m_triangleCount == 0)
  {
    return;
  }

  // Bind texture
  bindTexture();

  // Upload texture-specific uniforms
  uploadTextureUniforms(rc);

  // Setup face culling
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

  // Enable alpha blending if texture has transparency
  if (m_params.hasTransparency)
  {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }

  // Call base class render
  TexturedTrianglesProxyBase::render(rc);

  // Restore state
  glDisable(GL_CULL_FACE);

  // Unbind texture
  unbindTexture();

  MRPT_END
#endif
}

[[nodiscard]] std::vector<shader_id_t> TexturedTrianglesProxy::requiredShaders() const
{
  // Only return the base shader here. Shadow shader variants are selected
  // at render time based on the rendering pass (shadow map vs normal).
  if (m_params.lightEnabled)
  {
    return {DefaultShaderID::TEXTURED_TRIANGLES_LIGHT};
  }
  else
  {
    return {DefaultShaderID::TEXTURED_TRIANGLES_NO_LIGHT};
  }
}

void TexturedTrianglesProxy::extractTextureParams(const CVisualObject* sourceObj)
{
  // Get material params from base object
  m_params.materialShininess = sourceObj->materialShininess();
  m_params.materialSpecularExponent = sourceObj->materialSpecularExponent();
  m_params.materialEmissive = sourceObj->materialEmissive();

  // Get textured triangle-specific params
  const auto* texTriObj = dynamic_cast<const VisualObjectParams_TexturedTriangles*>(sourceObj);
  if (texTriObj)
  {
    m_params.lightEnabled = texTriObj->isLightEnabled();
    m_params.cullFace = texTriObj->cullFaces();
    m_params.textureInterpolate = texTriObj->textureLinearInterpolation();
    m_params.textureMipMaps = texTriObj->textureMipMap();

    // Check if alpha image is assigned
    const auto& alphaImg = texTriObj->getTextureAlphaImage();
    m_params.hasTransparency = !alphaImg.isEmpty();

    m_params.hasNormalMap = texTriObj->normalMapHasBeenAssigned();
  }
}

void TexturedTrianglesProxy::uploadTextureUniforms(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!rc.shader)
  {
    return;
  }

  // Texture sampler uniform (bind to texture unit 0)
  if (rc.shader->hasUniform("textureSampler"))
  {
    uploadInt(rc, "textureSampler", MATERIAL_DIFFUSE_TEXTURE_UNIT);
  }

  // Normal map sampler uniform (bind to texture unit 2)
  if (rc.shader->hasUniform("normalMapSampler"))
  {
    uploadInt(rc, "normalMapSampler", NORMAL_MAP_TEXTURE_UNIT);
  }

  // Material specular intensity
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

  // Fog parameters
  if (rc.lights && rc.shader->hasUniform("fog_enabled"))
  {
    uploadInt(rc, "fog_enabled", rc.lights->fog_enabled ? 1 : 0);
    if (rc.lights->fog_enabled)
    {
      const auto& fc = rc.lights->fog_color;
      uploadVector3(rc, "fog_color", mrpt::math::TVector3Df(fc.R, fc.G, fc.B));
      uploadFloat(rc, "fog_near", rc.lights->fog_near);
      uploadFloat(rc, "fog_far", rc.lights->fog_far);
      uploadInt(rc, "fog_mode", static_cast<int>(rc.lights->fog_mode));
      uploadFloat(rc, "fog_density", rc.lights->fog_density);
    }
  }
#endif
}

void TexturedTrianglesProxy::updateTexture(const VisualObjectParams_TexturedTriangles* texTriObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!texTriObj)
  {
    return;
  }

  const auto& textureImage = texTriObj->getTextureImage();
  if (textureImage.isEmpty())
  {
    return;
  }

  // Create texture object if needed
  if (!m_ownedTexture)
  {
    m_ownedTexture = std::make_unique<Texture>();
  }

  // Configure texture options using Texture::Options
  Texture::Options options;
  options.generateMipMaps = m_params.textureMipMaps;
  options.magnifyLinearFilter = m_params.textureInterpolate;
  options.enableTransparency = m_params.hasTransparency;

  // Check for alpha texture
  const auto& alphaImage = texTriObj->getTextureAlphaImage();

  // Always unload the old texture before re-uploading, to bypass the
  // data-pointer cache in Texture::assignImage2D which would return the
  // stale first-frame texture for streaming images that reuse the same buffer.
  if (m_ownedTexture->initialized())
  {
    m_ownedTexture->unloadTexture();
  }

  if (!alphaImage.isEmpty())
    m_ownedTexture->assignImage2D(textureImage, alphaImage, options, MATERIAL_DIFFUSE_TEXTURE_UNIT);
  else
    m_ownedTexture->assignImage2D(textureImage, options, MATERIAL_DIFFUSE_TEXTURE_UNIT);

  // Update base class texture pointer
  m_texture = m_ownedTexture.get();
#endif
}

void TexturedTrianglesProxy::updateNormalMapTexture(
    const VisualObjectParams_TexturedTriangles* texTriObj)
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  if (!texTriObj || !texTriObj->normalMapHasBeenAssigned())
  {
    return;
  }

  const auto& normalMapImage = texTriObj->getNormalMapImage();
  if (normalMapImage.isEmpty())
  {
    return;
  }

  if (!m_ownedNormalMapTexture)
  {
    m_ownedNormalMapTexture = std::make_unique<Texture>();
  }

  Texture::Options options;
  options.generateMipMaps = m_params.textureMipMaps;
  options.magnifyLinearFilter = true;  // always interpolate normal maps
  options.enableTransparency = false;
  options.isColorData = false;  // normal maps are linear data, not sRGB

  if (m_ownedNormalMapTexture->initialized())
  {
    m_ownedNormalMapTexture->unloadTexture();
  }

  m_ownedNormalMapTexture->assignImage2D(normalMapImage, options, NORMAL_MAP_TEXTURE_UNIT);
#endif
}

void TexturedTrianglesProxy::bindTexture() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  // Bind diffuse texture (unit 0)
  glActiveTexture(GL_TEXTURE0 + MATERIAL_DIFFUSE_TEXTURE_UNIT);
  if (m_ownedTexture)
  {
    m_ownedTexture->bindAsTexture2D();
  }
  else
  {
    // No texture assigned: create a 1x1 white texture so vertex color passes
    // through (texColor=white, final=white*vertexColor=vertexColor)
    if (m_defaultWhiteGLTexId == 0)
    {
      glGenTextures(1, &m_defaultWhiteGLTexId);
      glBindTexture(GL_TEXTURE_2D, m_defaultWhiteGLTexId);
      const uint8_t white[3] = {255, 255, 255};
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, white);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    glBindTexture(GL_TEXTURE_2D, m_defaultWhiteGLTexId);
  }

  // Bind normal map (unit 2)
  glActiveTexture(GL_TEXTURE0 + NORMAL_MAP_TEXTURE_UNIT);
  if (m_ownedNormalMapTexture)
  {
    m_ownedNormalMapTexture->bindAsTexture2D();
  }
  else
  {
    // No normal map: create a 1x1 flat-blue texture encoding identity normal
    // (0.5, 0.5, 1.0) in tangent space → (0, 0, 1) after decode
    if (m_defaultFlatNormalMapGLTexId == 0)
    {
      glGenTextures(1, &m_defaultFlatNormalMapGLTexId);
      glBindTexture(GL_TEXTURE_2D, m_defaultFlatNormalMapGLTexId);
      const uint8_t flatNormal[3] = {128, 128, 255};
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, flatNormal);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    glBindTexture(GL_TEXTURE_2D, m_defaultFlatNormalMapGLTexId);
  }
#endif
}

void TexturedTrianglesProxy::unbindTexture() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  glActiveTexture(GL_TEXTURE0 + NORMAL_MAP_TEXTURE_UNIT);
  glBindTexture(GL_TEXTURE_2D, 0);

  glActiveTexture(GL_TEXTURE0 + MATERIAL_DIFFUSE_TEXTURE_UNIT);
  glBindTexture(GL_TEXTURE_2D, 0);
#endif
}