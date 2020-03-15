/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/CRenderizableShaderWireFrame.h>

namespace mrpt::opengl
{
/** A grid of lines over the XZ plane.
 *  \sa opengl::COpenGLScene
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CGridPlaneXZ </td> <td> \image html
 * preview_CGridPlaneXZ.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CGridPlaneXZ : public CRenderizableShaderWireFrame
{
	DEFINE_SERIALIZABLE(CGridPlaneXZ, mrpt::opengl)

   protected:
	float m_xMin, m_xMax;
	float m_zMin, m_zMax;
	float m_plane_y;
	float m_frequency;

   public:
	void setPlaneLimits(float xmin, float xmax, float zmin, float zmax)
	{
		m_xMin = xmin;
		m_xMax = xmax;
		m_zMin = zmin;
		m_zMax = zmax;
		CRenderizable::notifyChange();
	}

	void getPlaneLimits(
		float& xmin, float& xmax, float& zmin, float& zmax) const
	{
		xmin = m_xMin;
		xmax = m_xMax;
		zmin = m_zMin;
		zmax = m_zMax;
	}

	void setPlaneYcoord(float y)
	{
		m_plane_y = y;
		CRenderizable::notifyChange();
	}
	float getPlaneYcoord() const { return m_plane_y; }
	void setGridFrequency(float freq)
	{
		ASSERT_(freq > 0);
		m_frequency = freq;
		CRenderizable::notifyChange();
	}
	float getGridFrequency() const { return m_frequency; }

	void onUpdateBuffers_Wireframe() override;

	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Constructor */
	CGridPlaneXZ(
		float xMin = -10, float xMax = 10, float zMin = -10, float zMax = 10,
		float y = 0, float frequency = 1, float lineWidth = 1.3f,
		bool antiAliasing = true);
	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CGridPlaneXZ() override = default;
};

}  // namespace mrpt::opengl
