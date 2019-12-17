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
class CSphere : public CRenderizable
{
	DEFINE_SERIALIZABLE(CSphere, mrpt::opengl)

   protected:
	float m_radius;
	int m_nDivsLongitude, m_nDivsLatitude;
	bool m_keepRadiusIndependentEyeDistance{false};

   public:
	void setRadius(float r)
	{
		m_radius = r;
		CRenderizable::notifyChange();
	}
	float getRadius() const { return m_radius; }
	void setNumberDivsLongitude(int N)
	{
		m_nDivsLongitude = N;
		CRenderizable::notifyChange();
	}
	void setNumberDivsLatitude(int N)
	{
		m_nDivsLatitude = N;
		CRenderizable::notifyChange();
	}
	void enableRadiusIndependentOfEyeDistance(bool v = true)
	{
		m_keepRadiusIndependentEyeDistance = v;
		CRenderizable::notifyChange();
	}

	void render() const override;
	void renderUpdateBuffers() const override;
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
