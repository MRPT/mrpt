/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/COpenGLBuffer.h>
#include <mrpt/opengl/COpenGLVertexArrayObject.h>
#include <mrpt/opengl/CRenderizable.h>

namespace mrpt::opengl
{
/** Renderizable generic renderer for objects using the wireframe shader.
 *
 *  \sa opengl::COpenGLScene
 *
 * \ingroup mrpt_opengl_grp
 */
class CRenderizableShaderWireFrame : public virtual CRenderizable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CRenderizableShaderWireFrame)

   public:
	CRenderizableShaderWireFrame() = default;
	virtual ~CRenderizableShaderWireFrame();

	virtual shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::WIREFRAME};
	}
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	/** Must be implemented in derived classes to update the geometric entities
	 * to be drawn in "m_*_buffer" fields. */
	virtual void onUpdateBuffers_Wireframe() = 0;

	void setLineWidth(float w)
	{
		m_lineWidth = w;
		CRenderizable::notifyChange();
	}
	float getLineWidth() const { return m_lineWidth; }
	void enableAntiAliasing(bool enable = true)
	{
		m_antiAliasing = enable;
		CRenderizable::notifyChange();
	}
	bool isAntiAliasingEnabled() const { return m_antiAliasing; }

	// See base docs
	void freeOpenGLResources() override
	{
		m_vertexBuffer.destroy();
		m_colorBuffer.destroy();
		m_vao.destroy();
	}

	/** @name Raw access to wireframe shader buffer data
	 * @{ */
	const auto& shaderWireframeVertexPointBuffer() const
	{
		return m_vertex_buffer_data;
	}
	const auto& shaderWireframeVertexColorBuffer() const
	{
		return m_color_buffer_data;
	}
	/** @} */

   protected:
	float m_lineWidth = 1.0f;
	bool m_antiAliasing = false;
	mutable std::vector<mrpt::math::TPoint3Df> m_vertex_buffer_data;
	mutable std::vector<mrpt::img::TColor> m_color_buffer_data;

	/** Returns the bounding box of m_vertex_buffer_data, or (0,0,0)-(0,0,0) if
	 * empty. */
	const mrpt::math::TBoundingBox wireframeVerticesBoundingBox() const;

   private:
	mutable COpenGLBuffer m_vertexBuffer, m_colorBuffer;
	mutable COpenGLVertexArrayObject m_vao;
};

}  // namespace mrpt::opengl
