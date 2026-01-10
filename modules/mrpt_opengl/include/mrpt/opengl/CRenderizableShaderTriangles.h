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
  CRenderizableShaderTriangles();

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
    auto gh = gls();
    gh.state.trianglesBuffer->destroy();
    gh.state.vao->destroy();
  }

  bool isLightEnabled() const { return m_enableLight; }
  void enableLight(bool enable = true) { m_enableLight = enable; }

  /** Control whether to render the FRONT, BACK, or BOTH (default) set of
   * faces. Refer to docs for glCullFace().
   * Example: If set to `cullFaces(TCullFace::BACK);`, back faces will not be
   * drawn ("culled")
   */
  void cullFaces(const TCullFace& cf) { m_cullface = cf; }
  TCullFace cullFaces() const { return m_cullface; }

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

  void params_serialize(mrpt::serialization::CArchive& out) const;
  void params_deserialize(mrpt::serialization::CArchive& in);

 private:
  bool m_enableLight = true;
  TCullFace m_cullface = TCullFace::NONE;

  struct GlState
  {
    std::unique_ptr<Buffer> trianglesBuffer = std::make_unique<Buffer>();
    std::unique_ptr<VertexArrayObject> vao = std::make_unique<VertexArrayObject>();
  };
  mutable mrpt::containers::NonCopiableData<GlState> m_gls;
  mutable mrpt::containers::NonCopiableData<std::mutex> m_glsMtx;
  struct GlsHandle
  {
    GlState& state;
    std::unique_lock<std::mutex> lock;
  };

  [[nodiscard]] GlsHandle gls() const
  {
    std::unique_lock<std::mutex> lock(m_glsMtx.data);
    return {m_gls.data, std::move(lock)};
  }
};

}  // namespace mrpt::opengl
