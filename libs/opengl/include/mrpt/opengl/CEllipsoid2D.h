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
/** A 2D ellipse on the XY plane, centered at the origin of this object pose.
 *
 * The color is determined by the RGBA fields in the class "CRenderizable".
 * Note that a transparent ellipse can be drawn for "0<alpha<1" values.
 * If any of the eigen value of the covariance matrix of the ellipsoid is
 * zero, it will not be rendered.
 *  \sa opengl::COpenGLScene
 *
 * Please read the documentation of
 * CGeneralizedEllipsoidTemplate::setQuantiles() for learning
 * the mathematical details about setting the desired confidence interval.
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 *border-style: solid;">
 *   <tr> <td> mrpt::opengl::CEllipsoid2D </td> <td> \image html
 *preview_CEllipsoid.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CEllipsoid2D : public CGeneralizedEllipsoidTemplate<2>
{
	using BASE = CGeneralizedEllipsoidTemplate<2>;

	DEFINE_SERIALIZABLE(CEllipsoid2D, mrpt::opengl)

   public:
	CEllipsoid2D() = default;
	virtual ~CEllipsoid2D() override = default;

	/** The number of segments of a 2D ellipse (default=20) */
	void set2DsegmentsCount(unsigned int N) { BASE::setNumberOfSegments(N); }

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
