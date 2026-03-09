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

#include <mrpt/opengl/Buffer.h>
#include <mrpt/opengl/RenderableProxy.h>
#include <mrpt/opengl/Texture.h>
#include <mrpt/opengl/VertexArrayObject.h>

#include <memory>

namespace mrpt::opengl
{
/** GPU-side proxy for rendering sky boxes (cube-mapped environment boxes).
 *
 * Renders a CSkyBox using the SKYBOX shader and a cube map texture.
 * The skybox always appears "at infinity" behind all scene objects.
 *
 * \sa mrpt::viz::CSkyBox
 * \ingroup mrpt_opengl_grp
 */
class SkyBoxProxy : public RenderableProxy
{
 public:
  SkyBoxProxy() = default;
  ~SkyBoxProxy() override = default;

  void compile(const mrpt::viz::CVisualObject* sourceObj) override;
  void updateBuffers(const mrpt::viz::CVisualObject* sourceObj) override;
  void render(const RenderContext& rc) const override;

  [[nodiscard]] std::vector<shader_id_t> requiredShaders() const override
  {
    return {DefaultShaderID::SKYBOX};
  }

  [[nodiscard]] bool castsShadows() const override { return false; }
  [[nodiscard]] bool cullEligible() const override { return false; }

  [[nodiscard]] const char* typeName() const override { return "SkyBoxProxy"; }

 private:
  /** Vertex buffer: unit cube (36 vertices × 3 floats) */
  Buffer m_vertexBuffer{Buffer::Type::Vertex};

  /** Vertex Array Object */
  VertexArrayObject m_vao;

  /** The cube map texture */
  std::unique_ptr<Texture> m_cubeTexture;

  /** true if the cube texture has been successfully loaded */
  bool m_textureLoaded = false;

  /** Upload static unit cube vertex data to GPU */
  void uploadCubeVertices();

  /** Create or re-upload the cube map texture from the source CSkyBox */
  void updateCubeTexture(const mrpt::viz::CVisualObject* sourceObj);
};

}  // namespace mrpt::opengl
