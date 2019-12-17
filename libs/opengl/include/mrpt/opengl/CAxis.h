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
#include <array>

namespace mrpt::opengl
{
/** Draw a 3D world axis, with coordinate marks at some regular interval
 *  \sa opengl::COpenGLScene
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *  <tr> <td> mrpt::opengl::CAxis </td> <td> \image html preview_CAxis.png
 * </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CAxis : public CRenderizable
{
	DEFINE_SERIALIZABLE(CAxis, mrpt::opengl)
   protected:
	float m_xmin, m_ymin, m_zmin;
	float m_xmax, m_ymax, m_zmax;
	float m_frequency;
	float m_lineWidth;
	/** draw marks for X,Y,Z */
	std::array<bool, 3> m_marks = {false, false, false};
	float m_textScale{0.25f};
	float m_textRot[3][3];  // {x,y,z},{yaw,pitch,roll}
	float m_markLen{0.07f};

   public:
	/** Constructor */
	CAxis(
		float xmin = -1.0f, float ymin = -1.0f, float zmin = -1.0f,
		float xmax = 1.0f, float ymax = 1.0f, float zmax = 1.0f,
		float frecuency = 1.f, float lineWidth = 3.0f, bool marks = false);

	~CAxis() override = default;

	void setAxisLimits(
		float xmin, float ymin, float zmin, float xmax, float ymax, float zmax);
	/** Changes the frequency of the "ticks" */
	void setFrequency(float f);
	float getFrequency() const;
	void setLineWidth(float w);
	float getLineWidth() const;
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

	void render() const override;
	void renderUpdateBuffers() const override;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;
};

}  // namespace mrpt::opengl
