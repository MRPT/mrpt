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
/** A solid or wire-frame sphere.
 *  \sa opengl::COpenGLScene
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CSphere </td> <td> \image html preview_CSphere.png
 * </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CSphere : public CRenderizableDisplayList
{
	DEFINE_SERIALIZABLE(CSphere)

   protected:
	float m_radius;
	int m_nDivsLongitude, m_nDivsLatitude;
	bool m_keepRadiusIndependentEyeDistance{false};

   public:
	void setRadius(float r)
	{
		m_radius = r;
		CRenderizableDisplayList::notifyChange();
	}
	float getRadius() const { return m_radius; }
	void setNumberDivsLongitude(int N)
	{
		m_nDivsLongitude = N;
		CRenderizableDisplayList::notifyChange();
	}
	void setNumberDivsLatitude(int N)
	{
		m_nDivsLatitude = N;
		CRenderizableDisplayList::notifyChange();
	}
	void enableRadiusIndependentOfEyeDistance(bool v = true)
	{
		m_keepRadiusIndependentEyeDistance = v;
		CRenderizableDisplayList::notifyChange();
	}

	/** \sa CRenderizableDisplayList */
	bool should_skip_display_list_cache() const override
	{
		return m_keepRadiusIndependentEyeDistance;
	}

	/** Render */
	void render_dl() const override;

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Ray tracing
	 */
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

	/** Constructor
	 */
	CSphere(
		float radius = 1.0f, int nDivsLongitude = 20, int nDivsLatitude = 20)
		: m_radius(radius),
		  m_nDivsLongitude(nDivsLongitude),
		  m_nDivsLatitude(nDivsLatitude)

	{
	}

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CSphere() override = default;
};

}  // namespace mrpt::opengl
