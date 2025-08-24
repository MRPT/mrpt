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

#include <mrpt/opengl/Buffer.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/VertexArrayObject.h>

#include <shared_mutex>

namespace mrpt::opengl
{
/** Renderizable generic renderer for objects using the wireframe shader.
 *
 *  \sa opengl::Scene
 *
 * \ingroup mrpt_opengl_grp
 */
class CRenderizableShaderWireFrame : public virtual CRenderizable
{
  DEFINE_VIRTUAL_SERIALIZABLE(CRenderizableShaderWireFrame, mrpt::opengl)

 public:
  CRenderizableShaderWireFrame() = default;
  virtual ~CRenderizableShaderWireFrame();

  virtual shader_list_t requiredShaders() const override { return {DefaultShaderID::WIREFRAME}; }
  void render(const RenderContext& rc) const override;
  void renderUpdateBuffers() const override;

  /** Must be implemented in derived classes to update the geometric entities
   * to be drawn in "m_*_buffer" fields. */
  virtual void onUpdateBuffers_Wireframe() = 0;

  // See base docs
  void freeOpenGLResources() override
  {
    m_vertexBuffer.destroy();
    m_colorBuffer.destroy();
    m_vao.destroy();
  }

  /** @name Raw access to wireframe shader buffer data
   * @{ */
  const auto& shaderWireframeVertexPointBuffer() const { return m_vertex_buffer_data; }
  const auto& shaderWireframeVertexColorBuffer() const { return m_color_buffer_data; }
  auto& shaderWireframeBuffersMutex() const { return m_wireframeMtx; }

  /** @} */

 protected:
  mutable std::vector<mrpt::math::TPoint3Df> m_vertex_buffer_data;
  mutable std::vector<mrpt::img::TColor> m_color_buffer_data;
  mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_wireframeMtx;

  /** Returns the bounding box of m_vertex_buffer_data, or (0,0,0)-(0,0,0) if
   * empty. */
  const mrpt::math::TBoundingBox wireframeVerticesBoundingBox() const;

 private:
  mutable Buffer m_vertexBuffer, m_colorBuffer;
  mutable VertexArrayObject m_vao;
};

}  // namespace mrpt::opengl
