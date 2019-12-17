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
class CGridPlaneXY : public CRenderizable
{
	DEFINE_SERIALIZABLE(CGridPlaneXY, mrpt::opengl)

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
		CRenderizable::notifyChange();
	}
	float getLineWidth() const { return m_lineWidth; }
	void enableAntiAliasing(bool enable = true)
	{
		m_antiAliasing = enable;
		CRenderizable::notifyChange();
	}
	bool isAntiAliasingEnabled() const { return m_antiAliasing; }
	void setPlaneLimits(float xmin, float xmax, float ymin, float ymax)
	{
		m_xMin = xmin;
		m_xMax = xmax;
		m_yMin = ymin;
		m_yMax = ymax;
		CRenderizable::notifyChange();
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
		CRenderizable::notifyChange();
		m_plane_z = z;
	}
	float getPlaneZcoord() const { return m_plane_z; }
	void setGridFrequency(float freq)
	{
		ASSERT_(freq > 0);
		m_frequency = freq;
		CRenderizable::notifyChange();
	}
	float getGridFrequency() const { return m_frequency; }

	void render() const override;
	void renderUpdateBuffers() const override;
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
