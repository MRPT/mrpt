/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CRenderizableShaderWireFrame.h>

#include <array>

namespace mrpt::opengl
{
/** Draw a 3D world axis, with coordinate marks at some regular interval
 *  \sa opengl::COpenGLScene
 *
 *  ![mrpt::opengl::CAxis](preview_CAxis.png)
 *
 * \ingroup mrpt_opengl_grp
 */
class CAxis : public CRenderizableShaderWireFrame
{
	DEFINE_SERIALIZABLE(CAxis, mrpt::opengl)
   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void onUpdateBuffers_Wireframe() override;
	void render(const RenderContext& rc) const override;
	void enqueueForRenderRecursive(
		const mrpt::opengl::TRenderMatrices& state, RenderQueue& rq,
		bool wholeInView) const override;
	bool isCompositeObject() const override { return true; }
	/** @} */

	/** Constructor */
	CAxis(
		float xmin = -1.0f, float ymin = -1.0f, float zmin = -1.0f,
		float xmax = 1.0f, float ymax = 1.0f, float zmax = 1.0f,
		float frecuency = 1.f, float lineWidth = 3.0f, bool marks = true);

	virtual ~CAxis() override = default;

	void setAxisLimits(
		float xmin, float ymin, float zmin, float xmax, float ymax, float zmax);
	/** Changes the frequency of the "ticks" */
	void setFrequency(float f);
	float getFrequency() const;
	/** Changes the size of text labels (default:0.25) */
	void setTextScale(float f);
	float getTextScale() const;
	/** axis: {0,1,2}=>{X,Y,Z} */
	void setTextLabelOrientation(
		int axis, float yaw_deg, float pitch_deg, float roll_deg);
	/** axis: {0,1,2}=>{X,Y,Z} */
	void getTextLabelOrientation(
		int axis, float& yaw_deg, float& pitch_deg, float& roll_deg) const;

	void enableTickMarks(bool v = true);
	void enableTickMarks(bool show_x, bool show_y, bool show_z);
	/** As a ratio of "marks frequency" (default: 0.05) */
	void setTickMarksLength(float len);
	float getTickMarksLength(float len) { return m_markLen; }

	mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

   protected:
	float m_xmin, m_ymin, m_zmin;
	float m_xmax, m_ymax, m_zmax;
	float m_frequency;
	/** draw marks for X,Y,Z */
	std::array<bool, 3> m_marks = {false, false, false};
	float m_textScale{0.10f};
	float m_textRot[3][3];	// {x,y,z},{yaw,pitch,roll}
	float m_markLen{0.07f};

	mrpt::containers::PerThreadDataHolder<mrpt::opengl::CListOpenGLObjects>
		m_gl_labels;
};

}  // namespace mrpt::opengl
