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

#include <mrpt/img/CImage.h>
#include <mrpt/opengl/DefaultShaders.h>
#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TexturedTrianglesProxy.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/TLightParameters.h>

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
  if (texTriObj && texTriObj->textureImageHasBeenAssigned())
  {
    updateTexture(texTriObj);
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
  if (texTriObj && texTriObj->textureImageHasBeenAssigned())
  {
    updateTexture(texTriObj);
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

void TexturedTrianglesProxy::bindTexture() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  glActiveTexture(GL_TEXTURE0 + MATERIAL_DIFFUSE_TEXTURE_UNIT);
  if (m_ownedTexture)
  {
    m_ownedTexture->bindAsTexture2D();
  }
  else
  {
    // No texture assigned: create a 1x1 white texture so vertex color passes
    // through (texColor=white, final=white*vertexColor=vertexColor)
    // Create a 1x1 white GL texture bypassing MRPT tracking to avoid ID conflicts
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
#endif
}

void TexturedTrianglesProxy::unbindTexture() const
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
  glActiveTexture(GL_TEXTURE0 + MATERIAL_DIFFUSE_TEXTURE_UNIT);
  glBindTexture(GL_TEXTURE_2D, 0);
#endif
}