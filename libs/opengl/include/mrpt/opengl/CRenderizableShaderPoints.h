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
 * All points may have the same point size (see setPointSize()) or a dynamic,
 * depth-dependent size to emulate the effect of larger points when looked
 * closely (see enableVariablePointSize()).
 *
 * In the latter case, point size is computed in the shader as:
 *
 *  gl_PointSize = vertexPointSize +
 *  variablePointSize_k/(variablePointSize_DepthScale*gl_Position.z + 0.01);
 *
 * where the paramters vertexPointSize, variablePointSize_k, and
 * variablePointSize_DepthScale can be set in this class via setPointSize(),
 * setVariablePointSize_k(), and setVariablePointSize_DepthScale(),
 * respectively.
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
	virtual ~CRenderizableShaderPoints() override;

	virtual shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::POINTS};
	}
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	/** Must be implemented in derived classes to update the geometric entities
	 * to be drawn in "m_*_buffer" fields. */
	virtual void onUpdateBuffers_Points() = 0;

	/** By default is 1.0. \sa enableVariablePointSize() */
	inline void setPointSize(float p) { m_pointSize = p; }
	inline float getPointSize() const { return m_pointSize; }

	/** Enable/disable variable eye distance-dependent point size (default=true)
	 */
	inline void enableVariablePointSize(bool enable = true)
	{
		m_variablePointSize = enable;
	}
	inline bool isEnabledVariablePointSize() const
	{
		return m_variablePointSize;
	}
	/** see CRenderizableShaderPoints for a discussion of this parameter. */
	void setVariablePointSize_k(float v) { m_variablePointSize_K = v; }
	float getVariablePointSize_k() const { return m_variablePointSize_K; }

	/** see CRenderizableShaderPoints for a discussion of this parameter. */
	void setVariablePointSize_DepthScale(float v)
	{
		m_variablePointSize_DepthScale = v;
	}
	float getVariablePointSize_DepthScale() const
	{
		return m_variablePointSize_DepthScale;
	}

   protected:
	mutable std::vector<mrpt::math::TPoint3Df> m_vertex_buffer_data;
	mutable std::vector<mrpt::img::TColor> m_color_buffer_data;

	float m_pointSize = 1.0f;
	bool m_variablePointSize = true;
	float m_variablePointSize_K = 1.0f;
	float m_variablePointSize_DepthScale = 0.03f;

	void params_serialize(mrpt::serialization::CArchive& out) const;
	void params_deserialize(mrpt::serialization::CArchive& in);

   private:
	mutable unsigned int m_vertexBuffer = 0, m_vao = 0, m_colorBuffer = 0;
};

}  // namespace mrpt::opengl
