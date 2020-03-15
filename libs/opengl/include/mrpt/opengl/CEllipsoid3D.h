/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/opengl/CGeneralizedEllipsoidTemplate.h>

namespace mrpt::opengl
{
/** A 3D ellipsoid, centered at zero with respect to this object pose.
 * The color is determined by the RGBA fields in the class "CRenderizable".
 * Note that a transparent ellipsoid can be drawn for "0<alpha<1" values.
 * If any of the eigen values of the covariance matrix of the ellipsoid is
 * zero, nothing will be rendered.
 *  \sa opengl::COpenGLScene
 *
 * Please read the documentation of
 * CGeneralizedEllipsoidTemplate::setQuantiles() for learning
 * the mathematical details about setting the desired confidence interval.
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 *border-style: solid;">
 *   <tr> <td> mrpt::opengl::CEllipsoid3D </td> <td> \image html
 *preview_CEllipsoid.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CEllipsoid3D : public CGeneralizedEllipsoidTemplate<3>
{
	using BASE = CGeneralizedEllipsoidTemplate<3>;

	DEFINE_SERIALIZABLE(CEllipsoid3D, mrpt::opengl)

   public:
	CEllipsoid3D() = default;
	virtual ~CEllipsoid3D() override = default;

	/** The number of segments of a 3D ellipse (in both "axes") (default=20) */
	void set3DsegmentsCount(unsigned int N) { BASE::setNumberOfSegments(N); }

	/** Ray tracing */
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

   protected:
	/** To be implemented by derived classes: maps, using some arbitrary space
	 * transformation, a list of points
	 *  defining an ellipsoid in parameter space into their corresponding
	 * points in 2D/3D space.
	 */
	void transformFromParameterSpace(
		const std::vector<BASE::array_parameter_t>& in_pts,
		std::vector<BASE::array_point_t>& out_pts) const override;
};

}  // namespace mrpt::opengl
