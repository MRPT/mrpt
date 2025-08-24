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
#include <mrpt/opengl/TTriangle.h>
#include <mrpt/opengl/VertexArrayObject.h>

#include <shared_mutex>

namespace mrpt::opengl
{
/** Renderizable generic renderer for objects using the triangles shader.
 *
 *  \sa opengl::Scene
 *
 * \ingroup mrpt_opengl_grp
 */
class CRenderizableShaderTriangles : public virtual CRenderizable
{
  DEFINE_VIRTUAL_SERIALIZABLE(CRenderizableShaderTriangles, mrpt::opengl)

 public:
  CRenderizableShaderTriangles() = default;
  virtual ~CRenderizableShaderTriangles() override;

  virtual shader_list_t requiredShaders() const override
  {
    return {m_enableLight ? DefaultShaderID::TRIANGLES_LIGHT : DefaultShaderID::TRIANGLES_NO_LIGHT};
  }
  void render(const RenderContext& rc) const override;
  void renderUpdateBuffers() const override;

  /** Must be implemented in derived classes to update the geometric entities
   * to be drawn in "m_*_buffer" fields. */
  virtual void onUpdateBuffers_Triangles() = 0;

  // See base docs
  void freeOpenGLResources() override
  {
    m_trianglesBuffer.destroy();
    m_vao.destroy();
  }

  /** @name Raw access to triangle shader buffer data
   * @{ */
  const auto& shaderTrianglesBuffer() const { return m_triangles; }
  auto& shaderTrianglesBufferMutex() const { return m_trianglesMtx; }
  /** @} */

 protected:
  /** List of triangles  \sa TTriangle */
  mutable std::vector<mrpt::opengl::TTriangle> m_triangles;
  mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_trianglesMtx;

  /** Returns the bounding box of m_triangles, or (0,0,0)-(0,0,0) if empty. */
  const mrpt::math::TBoundingBoxf trianglesBoundingBox() const;

 private:
  mutable Buffer m_trianglesBuffer;
  mutable VertexArrayObject m_vao;
};

}  // namespace mrpt::opengl
