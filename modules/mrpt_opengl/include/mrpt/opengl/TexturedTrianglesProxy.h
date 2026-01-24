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
#pragma once

#include <mrpt/opengl/RenderableProxy.h>
#include <mrpt/opengl/Texture.h>

namespace mrpt::opengl
{
/** GPU-side proxy for rendering textured triangle meshes.
 *
 * This proxy handles rendering of any CVisualObject that uses textured triangle
 * shaders, such as:
 * - CTexturedPlane
 * - CMesh (with texture)
 * - CMeshFast (with texture)
 * - CSetOfTexturedTriangles
 * - Any object derived from VisualObjectParams_TexturedTriangles
 *
 * The proxy manages:
 * - Vertex buffer (3D triangle vertices)
 * - Normal buffer (per-vertex normals for lighting)
 * - Color buffer (per-vertex RGBA colors)
 * - Texture coordinate buffer (UV coordinates)
 * - Texture object (diffuse map)
 * - VAO for efficient attribute binding
 *
 * Textured rendering features:
 * - Phong lighting with texture modulation
 * - Optional transparency via alpha channel or separate alpha texture
 * - Texture filtering (nearest/linear interpolation)
 * - Mipmapping support
 * - Face culling modes
 * - Shadow casting
 *
 * Shader selection:
 * - Normal rendering: TEXTURED_TRIANGLES_LIGHT or TEXTURED_TRIANGLES_NO_LIGHT
 * - Shadow 1st pass: TEXTURED_TRIANGLES_SHADOW_1ST
 * - Shadow 2nd pass: TEXTURED_TRIANGLES_SHADOW_2ND
 *
 * \sa TexturedTrianglesProxyBase, mrpt::viz::VisualObjectParams_TexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
class TexturedTrianglesProxy : public TexturedTrianglesProxyBase
{
 public:
  TexturedTrianglesProxy() = default;
  ~TexturedTrianglesProxy() override = default;

  /** @name RenderableProxy Interface Implementation
   * @{ */

  void compile(const mrpt::viz::CVisualObject* sourceObj) override;
  void updateBuffers(const mrpt::viz::CVisualObject* sourceObj) override;
  void render(const RenderContext& rc) const override;

  std::vector<shader_id_t> requiredShaders() const override;

  const char* typeName() const override { return "TexturedTrianglesProxy"; }

  /** @} */

 private:
  /** Cached texture rendering parameters */
  struct TextureParams
  {
    bool lightEnabled = true;
    mrpt::viz::TCullFace cullFace = mrpt::viz::TCullFace::NONE;
    float materialShininess = 0.2f;
    bool textureInterpolate = false;
    bool textureMipMaps = true;
    bool hasTransparency = false;
  };

  /** Cached parameters from last compile/update */
  mutable TextureParams m_params;

  /** Owned texture object (created from source image) */
  std::unique_ptr<Texture> m_ownedTexture;

  /** Helper: Extract texture rendering parameters from source object */
  void extractTextureParams(const mrpt::viz::CVisualObject* sourceObj);

  /** Helper: Upload texture-specific uniforms to shader */
  void uploadTextureUniforms(const RenderContext& rc) const;

  /** Helper: Create or update texture from source image */
  void updateTexture(const mrpt::viz::VisualObjectParams_TexturedTriangles* texTriObj);

  /** Helper: Setup texture state for rendering */
  void bindTexture() const;

  /** Helper: Cleanup texture state after rendering */
  void unbindTexture() const;
};

}  // namespace mrpt::opengl
