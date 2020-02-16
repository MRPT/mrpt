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
/** Renderizable generic renderer for objects using the points shader.
 *
 *  \sa opengl::COpenGLScene
 *
 * \ingroup mrpt_opengl_grp
 */
class CRenderizableShaderPoints : public virtual CRenderizable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CRenderizableShaderPoints)

   public:
	CRenderizableShaderPoints() = default;
	virtual ~CRenderizableShaderPoints();

	virtual shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::POINTS};
	}
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	/** Must be implemented in derived classes to update the geometric entities
	 * to be drawn in "m_*_buffer" fields. */
	virtual void onUpdateBuffers_Points() = 0;

	/** By default is 1.0 */
	inline void setPointSize(float p) { m_pointSize = p; }
	inline float getPointSize() const { return m_pointSize; }

   protected:
	mutable std::vector<mrpt::math::TPoint3Df> m_vertex_buffer_data;
	mutable std::vector<mrpt::img::TColor> m_color_buffer_data;

	float m_pointSize = 1.0f;

   private:
	mutable unsigned int m_vertexBuffer = 0, m_vao = 0, m_colorBuffer = 0;
};

}  // namespace mrpt::opengl
