/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/CRenderizable.h>

namespace mrpt::opengl
{
/** Renderizable generic renderer for objects using the wireframe shader.
 *
 *  \sa opengl::COpenGLScene
 *
 * \ingroup mrpt_opengl_grp
 */
class CRenderizableShaderWireFrame : public CRenderizable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CRenderizableShaderWireFrame)

   public:
	CRenderizableShaderWireFrame() = default;
	virtual ~CRenderizableShaderWireFrame();

	shader_id_t shaderType() const override
	{
		return DefaultShaderID::WIREFRAME;
	}
	void render(
		const mrpt::opengl::TRenderMatrices& state,
		mrpt::opengl::Program& shaders) const override;
	void renderUpdateBuffers() const override;

	/** Must be implemented in derived classes to update the geometric entities
	 * to be drawn in "m_*_buffer" fields. */
	virtual void onUpdateBuffers() = 0;

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

   protected:
	float m_lineWidth = 1.0f;
	bool m_antiAliasing = false;
	mutable unsigned int m_vertexBuffer = 0, m_vao = 0, m_colorBuffer = 0;
	mutable std::vector<mrpt::math::TPoint3Df> m_vertex_buffer_data;
	mutable std::vector<mrpt::img::TColor> m_color_buffer_data;
};
}  // namespace mrpt::opengl
