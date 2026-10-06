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

namespace
{
/** 1x1 white texture, used when none is assigned so vertex colors pass
 * through unchanged. */
const CImage& defaultDiffuseImage()
{
  static const CImage img = []()
  {
    CImage im(1, 1, CH_RGB);
    im.at<uint8_t>(0, 0, 0) = 0xff;
    im.at<uint8_t>(0, 0, 1) = 0xff;
    im.at<uint8_t>(0, 0, 2) = 0xff;
    return im;
  }();
  return img;
}

/** 1x1 normal map encoding the unperturbed normal (0,0,1) in tangent space. */
const CImage& defaultNormalMapImage()
{
  static const CImage img = []()
  {
    CImage im(1, 1, CH_RGB);
    im.at<uint8_t>(0, 0, 0) = 128;
    im.at<uint8_t>(0, 0, 1) = 128;
    im.at<uint8_t>(0, 0, 2) = 255;
    return im;
  }();
  return img;
}
}  // namespace

void TexturedTrianglesProxy::compile(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
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
    if (texTriObj->emissiveMapHasBeenAssigned())
    {
      updateEmissiveMapTexture(texTriObj);
    }
  }
  assignDefaultTexturesIfMissing();

  MRPT_END
#endif
}

void TexturedTrianglesProxy::updateBuffers(const CVisualObject* sourceObj)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
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
    if (texTriObj->emissiveMapHasBeenAssigned())
    {
      updateEmissiveMapTexture(texTriObj);
    }
  }
  assignDefaultTexturesIfMissing();

  MRPT_END
#endif
}

void TexturedTrianglesProxy::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
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

shader_id_t TexturedTrianglesProxy::shadowMapShader() const
{
  return m_params.alphaCutoff > 0.0f ? DefaultShaderID::TEXTURED_TRIANGLES_SHADOW_1ST
                                     : DefaultShaderID::TRIANGLES_SHADOW_1ST;
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
    m_params.alphaCutoff = texTriObj->effectiveAlphaCutoff();

    m_params.hasNormalMap = texTriObj->normalMapHasBeenAssigned();
  }
}

void TexturedTrianglesProxy::uploadTextureUniforms(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!rc.shader)
  {
    return;
  }

  // Texture sampler uniform (bind to texture unit 0)
  if (rc.shader->hasUniform("textureSampler"))
  {
    uploadInt(rc, "textureSampler", MATERIAL_DIFFUSE_TEXTURE_UNIT);
  }

  if (rc.shader->hasUniform("alphaCutoff"))
  {
    uploadFloat(rc, "alphaCutoff", m_params.alphaCutoff);
  }

  // Normal map sampler uniform (bind to texture unit 2)
  if (rc.shader->hasUniform("normalMapSampler"))
  {
    uploadInt(rc, "normalMapSampler", NORMAL_MAP_TEXTURE_UNIT);
  }
  if (rc.shader->hasUniform("emissiveMapSampler"))
  {
    uploadInt(rc, "emissiveMapSampler", EMISSIVE_MAP_TEXTURE_UNIT);
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

  // Camera position (view direction for specular lighting and back sides, and
  // fog distance)
  if (rc.shader->hasUniform("cam_position") && rc.state != nullptr)
  {
    const auto& e = rc.state->eye;
    uploadVector3(
        rc, "cam_position",
        mrpt::math::TVector3Df(
            static_cast<float>(e.x), static_cast<float>(e.y), static_cast<float>(e.z)));
  }

  // Multi-light parameters (if lighting enabled)
  if (m_params.lightEnabled)
  {
    uploadLights(rc);
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
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
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
  options.shareScope = m_resourceScope;

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
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
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
  options.shareScope = m_resourceScope;

  if (m_ownedNormalMapTexture->initialized())
  {
    m_ownedNormalMapTexture->unloadTexture();
  }

  m_ownedNormalMapTexture->assignImage2D(normalMapImage, options, NORMAL_MAP_TEXTURE_UNIT);
#endif
}

void TexturedTrianglesProxy::updateEmissiveMapTexture(
    const VisualObjectParams_TexturedTriangles* texTriObj)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (!texTriObj || !texTriObj->emissiveMapHasBeenAssigned())
  {
    return;
  }

  const auto& emissiveMapImage = texTriObj->getEmissiveMapImage();
  if (emissiveMapImage.isEmpty())
  {
    return;
  }

  if (!m_ownedEmissiveMapTexture)
  {
    m_ownedEmissiveMapTexture = std::make_unique<Texture>();
  }

  Texture::Options options;
  options.generateMipMaps = m_params.textureMipMaps;
  options.magnifyLinearFilter = m_params.textureInterpolate;
  options.enableTransparency = false;
  options.shareScope = m_resourceScope;

  if (m_ownedEmissiveMapTexture->initialized())
  {
    m_ownedEmissiveMapTexture->unloadTexture();
  }

  m_ownedEmissiveMapTexture->assignImage2D(emissiveMapImage, options, EMISSIVE_MAP_TEXTURE_UNIT);
#endif
}

void TexturedTrianglesProxy::assignDefaultTexturesIfMissing()
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  Texture::Options o;
  o.generateMipMaps = false;
  o.magnifyLinearFilter = false;
  o.shareScope = m_resourceScope;

  if (!m_ownedTexture || !m_ownedTexture->initialized())
  {
    m_ownedTexture = std::make_unique<Texture>();
    m_ownedTexture->assignImage2D(defaultDiffuseImage(), o, MATERIAL_DIFFUSE_TEXTURE_UNIT);
    m_texture = m_ownedTexture.get();
  }
  if (!m_ownedNormalMapTexture || !m_ownedNormalMapTexture->initialized())
  {
    o.isColorData = false;
    m_ownedNormalMapTexture = std::make_unique<Texture>();
    m_ownedNormalMapTexture->assignImage2D(defaultNormalMapImage(), o, NORMAL_MAP_TEXTURE_UNIT);
  }
  if (!m_ownedEmissiveMapTexture || !m_ownedEmissiveMapTexture->initialized())
  {
    o.isColorData = true;
    m_ownedEmissiveMapTexture = std::make_unique<Texture>();
    m_ownedEmissiveMapTexture->assignImage2D(defaultDiffuseImage(), o, EMISSIVE_MAP_TEXTURE_UNIT);
  }
#endif
}

void TexturedTrianglesProxy::bindTexture() const
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  if (m_ownedTexture)
  {
    m_ownedTexture->bindAsTexture2D();
  }
  if (m_ownedNormalMapTexture)
  {
    m_ownedNormalMapTexture->bindAsTexture2D();
  }
  if (m_ownedEmissiveMapTexture)
  {
    m_ownedEmissiveMapTexture->bindAsTexture2D();
  }
#endif
}

void TexturedTrianglesProxy::unbindTexture() const
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  glActiveTexture(GL_TEXTURE0 + EMISSIVE_MAP_TEXTURE_UNIT);
  glBindTexture(GL_TEXTURE_2D, 0);

  glActiveTexture(GL_TEXTURE0 + NORMAL_MAP_TEXTURE_UNIT);
  glBindTexture(GL_TEXTURE_2D, 0);

  glActiveTexture(GL_TEXTURE0 + MATERIAL_DIFFUSE_TEXTURE_UNIT);
  glBindTexture(GL_TEXTURE_2D, 0);
#endif
}