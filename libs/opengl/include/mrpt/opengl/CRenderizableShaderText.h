/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/Buffer.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/TTriangle.h>
#include <mrpt/opengl/VertexArrayObject.h>

#include <shared_mutex>

namespace mrpt::opengl
{
/** Renderizable generic renderer for objects using the "text shader".
 *
 *  \sa opengl::Scene
 *
 * \ingroup mrpt_opengl_grp
 */
class CRenderizableShaderText : public virtual CRenderizable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CRenderizableShaderText)

   public:
	CRenderizableShaderText() = default;
	virtual ~CRenderizableShaderText() override;

	virtual shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::TEXT};
	}
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	/** Must be implemented in derived classes to update the geometric entities
	 * to be drawn in "m_*_buffer" fields. */
	virtual void onUpdateBuffers_Text() = 0;

	// See base docs
	void freeOpenGLResources() override
	{
		m_trianglesBuffer.destroy();
		m_trianglesColorBuffer.destroy();
		m_linesVertexBuffer.destroy();
		m_linesColorBuffer.destroy();
		m_vao.destroy();
	}

   protected:
	/** List of triangles  \sa TTriangle */
	mutable std::vector<mrpt::opengl::TTriangle> m_triangles;
	/** List of lines */
	mutable std::vector<mrpt::math::TPoint3Df> m_vertex_buffer_data;
	mutable std::vector<mrpt::img::TColor> m_color_buffer_data;

	mutable mrpt::containers::NonCopiableData<std::shared_mutex> m_textDataMtx;

   private:
	mutable Buffer m_trianglesBuffer, m_trianglesColorBuffer;
	mutable Buffer m_linesVertexBuffer, m_linesColorBuffer;
	mutable VertexArrayObject m_vao;
};

}  // namespace mrpt::opengl
