/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CGeneralizedEllipsoidTemplate.h>

namespace mrpt::opengl
{
/** A solid or wire-frame sphere.
 *
 * ![mrpt::opengl::CSphere](preview_CSphere.png)
 *
 * \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */
class CSphere : public CGeneralizedEllipsoidTemplate<3>
{
	using BASE = CGeneralizedEllipsoidTemplate<3>;
	DEFINE_SERIALIZABLE(CSphere, mrpt::opengl)

   public:
	void renderUpdateBuffers() const override;

	void setRadius(float r)
	{
		m_radius = r;
		CRenderizable::notifyChange();
	}
	float getRadius() const { return m_radius; }
	void setNumberDivsLongitude(int N)
	{
		m_nDivsLongitude = N;
		BASE::setNumberOfSegments(m_nDivsLongitude);
	}
	void setNumberDivsLatitude(int N)
	{
		m_nDivsLatitude = N;
		CRenderizable::notifyChange();
	}

	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;
	virtual mrpt::math::TBoundingBox getBoundingBox() const override;

	/** Constructor */
	CSphere(
		float radius = 1.0f, int nDivsLongitude = 20, int nDivsLatitude = 20)
		: m_radius(radius),
		  m_nDivsLongitude(nDivsLongitude),
		  m_nDivsLatitude(nDivsLatitude)
	{
		regenerateBaseParams();
		BASE::enableDrawSolid3D(true);	// default
	}

	virtual ~CSphere() override = default;

   protected:
	float m_radius;
	int m_nDivsLongitude, m_nDivsLatitude;

	void regenerateBaseParams()
	{
		BASE::setCovMatrix(mrpt::math::CMatrixDouble33::Identity());
		BASE::setQuantiles(m_radius);
		BASE::setNumberOfSegments(m_nDivsLongitude);
	}

	void transformFromParameterSpace(
		const std::vector<BASE::array_parameter_t>& in_pts,
		std::vector<BASE::array_point_t>& out_pts) const override
	{
		out_pts = in_pts;
	}
};

}  // namespace mrpt::opengl
