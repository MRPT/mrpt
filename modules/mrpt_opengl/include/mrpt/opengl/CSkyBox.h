/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/opengl/Buffer.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CUBE_TEXTURE_FACE.h>
#include <mrpt/opengl/TTriangle.h>
#include <mrpt/opengl/Texture.h>
#include <mrpt/opengl/VertexArrayObject.h>

namespace mrpt::opengl
{
/** A Sky Box: 6 textures that are always rendered at "infinity" to give the
 *  impression of the scene to be much larger.
 *
 * Refer to example \ref opengl_skybox_example
 *
 *  <img src="mrpt-skybox-demo.gif" />
 *
 * \sa opengl::Scene
 * \ingroup mrpt_opengl_grp
 */
class CSkyBox : public CRenderizable
{
  DEFINE_SERIALIZABLE(CSkyBox, mrpt::opengl)

 public:
  CSkyBox()
  {
    m_vbo.data = std::make_unique<Buffer>();
    m_vao.data = std::make_unique<VertexArrayObject>();
  }
  virtual ~CSkyBox() override;

  /** @name Renderizable shader API virtual methods
   * @{ */
  void render(const RenderContext& rc) const override;
  void renderUpdateBuffers() const override;
  virtual shader_list_t requiredShaders() const override { return {DefaultShaderID::SKYBOX}; }
  // Not needed, only for VAO and VBO
  void freeOpenGLResources() override {}
  void initializeTextures() const override;
  /** @} */

  /** Assigns a texture. It is mandatory to assign all 6 faces before
   * initializing/rendering the texture.
   *
   * \note Images are copied, the original ones can be deleted.
   */
  void assignImage(const CUBE_TEXTURE_FACE face, const mrpt::img::CImage& img);

  /// \overload with move semantics for the image.
  void assignImage(const CUBE_TEXTURE_FACE face, img::CImage&& img);

  auto internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf override;

  bool cullElegible() const override { return false; }

 private:
  /// The cube texture for the 6 faces
  mutable Texture m_cubeTexture;
  std::array<mrpt::img::CImage, 6> m_textureImages;

  mutable mrpt::containers::NonCopiableData<std::unique_ptr<Buffer>> m_vbo;
  mutable mrpt::containers::NonCopiableData<std::unique_ptr<VertexArrayObject>> m_vao;
};

}  // namespace mrpt::opengl
