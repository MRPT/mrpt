/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/COpenGLBuffer.h>
#include <mrpt/opengl/COpenGLVertexArrayObject.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/TTriangle.h>

namespace mrpt::opengl
{
/** Renderizable generic renderer for objects using the triangles shader.
 *
 *  \sa opengl::COpenGLScene
 *
 * \ingroup mrpt_opengl_grp
 */
class CRenderizableShaderTriangles : public virtual CRenderizable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CRenderizableShaderTriangles)

   public:
	CRenderizableShaderTriangles() = default;
	virtual ~CRenderizableShaderTriangles() override;

	virtual shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::TRIANGLES};
	}
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	/** Must be implemented in derived classes to update the geometric entities
	 * to be drawn in "m_*_buffer" fields. */
	virtual void onUpdateBuffers_Triangles() = 0;

   protected:
	/** List of triangles  \sa TTriangle */
	mutable std::vector<mrpt::opengl::TTriangle> m_triangles;

   private:
	mutable COpenGLBuffer m_trianglesBuffer, m_colorBuffer;
	mutable COpenGLVertexArrayObject m_vao;
};

}  // namespace mrpt::opengl
