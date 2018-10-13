/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/CRenderizableDisplayList.h>

namespace mrpt::opengl
{
/** A grid of lines over the XY plane.
 *  \sa opengl::COpenGLScene
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CGridPlaneXY </td> <td> \image html
 * preview_CGridPlaneXY.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CGridPlaneXY : public CRenderizableDisplayList
{
	DEFINE_SERIALIZABLE(CGridPlaneXY)

   protected:
	float m_xMin, m_xMax;
	float m_yMin, m_yMax;
	float m_plane_z;
	float m_frequency;
	float m_lineWidth;
	bool m_antiAliasing;

   public:
	void setLineWidth(float w)
	{
		m_lineWidth = w;
		CRenderizableDisplayList::notifyChange();
	}
	float getLineWidth() const { return m_lineWidth; }
	void enableAntiAliasing(bool enable = true)
	{
		m_antiAliasing = enable;
		CRenderizableDisplayList::notifyChange();
	}
	bool isAntiAliasingEnabled() const { return m_antiAliasing; }
	void setPlaneLimits(float xmin, float xmax, float ymin, float ymax)
	{
		m_xMin = xmin;
		m_xMax = xmax;
		m_yMin = ymin;
		m_yMax = ymax;
		CRenderizableDisplayList::notifyChange();
	}

	void getPlaneLimits(
		float& xmin, float& xmax, float& ymin, float& ymax) const
	{
		xmin = m_xMin;
		xmax = m_xMax;
		ymin = m_yMin;
		ymax = m_yMax;
	}

	void setPlaneZcoord(float z)
	{
		CRenderizableDisplayList::notifyChange();
		m_plane_z = z;
	}
	float getPlaneZcoord() const { return m_plane_z; }
	void setGridFrequency(float freq)
	{
		ASSERT_(freq > 0);
		m_frequency = freq;
		CRenderizableDisplayList::notifyChange();
	}
	float getGridFrequency() const { return m_frequency; }
	/** Render */
	void render_dl() const override;

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Constructor  */
	CGridPlaneXY(
		float xMin = -10, float xMax = 10, float yMin = -10, float yMax = 10,
		float z = 0, float frequency = 1, float lineWidth = 1.3f,
		bool antiAliasing = true);

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CGridPlaneXY() override = default;
};

}  // namespace mrpt::opengl
