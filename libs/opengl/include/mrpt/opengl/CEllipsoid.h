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
/** A 2D ellipse or 3D ellipsoid, depending on the size of the m_cov matrix (2x2
 *or 3x3).
 *  The center of the ellipsoid is the "m_x,m_y,m_z" object's coordinates. In
 *the case of
 *   a 2D ellipse it will be drawn in the XY plane, for z=0.
 *  The color is determined by the RGBA fields in the class "CRenderizable".
 *Note that a
 *   transparent ellipsoid can be drawn for "0<alpha<1" values.
 *	 If one of the eigen value of the covariance matrix of the ellipsoid is
 *null, ellipsoid will not be rendered.
 *  \sa opengl::COpenGLScene
 *
 *
 * Please read the documentation of
 *CGeneralizedEllipsoidTemplate::setQuantiles() for learning
 *  the mathematical details about setting the desired confidence interval.
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 *border-style: solid;">
 *   <tr> <td> mrpt::opengl::CEllipsoid </td> <td> \image html
 *preview_CEllipsoid.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CEllipsoid : public CGeneralizedEllipsoidTemplate<3>
{
	using BASE = CGeneralizedEllipsoidTemplate<3>;

	DEFINE_SERIALIZABLE(CEllipsoid, mrpt::opengl)

   public:
	CEllipsoid() = default;
	virtual ~CEllipsoid() override = default;

	/** Set the 2x2 or 3x3 covariance matrix that will determine the aspect of
	 * the ellipsoid  */
	void setCovMatrix(const mrpt::math::CMatrixDouble& m);

	/// \overload
	inline void setCovMatrix(const mrpt::math::CMatrixFloat& m)
	{
		setCovMatrix(m.cast_double());
	}

	/**  Set the 2x2 or 3x3 covariance matrix that will determine the aspect of
	 * the ellipsoid (if resizeToSize>0, the matrix will be cut to the square
	 * matrix of the given size)
	 */
	template <typename T>
	void setCovMatrix(
		const mrpt::math::CMatrixFixed<T, 3, 3>& m, int resizeToSize = -1)
	{
		setCovMatrix(mrpt::math::CMatrixDynamic<T>(m), resizeToSize);
	}

	/**  Set the 2x2 or 3x3 covariance matrix that will determine the aspect of
	 * the ellipsoid (if resizeToSize>0, the matrix will be cut to the square
	 * matrix of the given size)
	 */
	template <typename T>
	void setCovMatrix(const mrpt::math::CMatrixFixed<T, 2, 2>& m)
	{
		setCovMatrix(mrpt::math::CMatrixDynamic<T>(m));
	}

	mrpt::math::CMatrixDouble getCovMatrix() const
	{
		return mrpt::math::CMatrixDouble(m_cov);
	}

	/** The number of segments of a 2D ellipse (default=20) */
	void set2DsegmentsCount(unsigned int N) { BASE::setNumberOfSegments(N); }
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
