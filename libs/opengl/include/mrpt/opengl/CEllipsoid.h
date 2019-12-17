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
#include <mrpt/opengl/CRenderizable.h>

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
class CEllipsoid : public CRenderizable
{
	DEFINE_SERIALIZABLE(CEllipsoid, mrpt::opengl)

   protected:
	/** Used to store computed values the first time this is rendered, and to
	 * avoid recomputing them again.
	 */
	math::CMatrixD m_eigVal, m_eigVec, m_prevComputedCov;

	/** The 2x2 or 3x3 covariance matrix that will determine the aspect of the
	 * ellipsoid. */
	math::CMatrixD m_cov;
	/** If set to true (default), a whole ellipsoid surface will be drawn, or if
	 * set to "false" it will be drawn as a "wireframe". */
	bool m_drawSolid3D{true};
	/** The number of "sigmas" for drawing the ellipse/ellipsoid (default=3) */
	float m_quantiles{3};
	/** The number of segments of a 2D ellipse (default=20) */
	unsigned int m_2D_segments{20};
	/** The number of segments of a 3D ellipse (in both "axis") (default=20) */
	unsigned int m_3D_segments{20};
	/** The line width for 2D ellipses or 3D wireframe ellipsoids (default=1) */
	float m_lineWidth{1.0};
	mutable mrpt::math::TPoint3D m_bb_min, m_bb_max;

   public:
	/** Set the 2x2 or 3x3 covariance matrix that will determine the aspect of
	 * the ellipsoid (if resizeToSize>0, the matrix will be cut to the square
	 * matrix of the given size) */
	void setCovMatrix(
		const mrpt::math::CMatrixDouble& m, int resizeToSize = -1);
	/** Set the 2x2 or 3x3 covariance matrix that will determine the aspect of
	 * the ellipsoid (if resizeToSize>0, the matrix will be cut to the square
	 * matrix of the given size). */
	void setCovMatrix(const mrpt::math::CMatrixFloat& m, int resizeToSize = -1);

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

	/** If set to true (default), a whole ellipsoid surface will be drawn, or if
	 * set to "false" it will be drawn as a "wireframe". */
	void enableDrawSolid3D(bool v)
	{
		m_drawSolid3D = v;
		CRenderizable::notifyChange();
	}
	/** The number of "sigmas" for drawing the ellipse/ellipsoid (default=3) */
	void setQuantiles(float q)
	{
		m_quantiles = q;
		CRenderizable::notifyChange();
	}
	float getQuantiles() const { return m_quantiles; }
	/** The number of segments of a 2D ellipse (default=20) */
	void set2DsegmentsCount(unsigned int N)
	{
		m_2D_segments = N;
		CRenderizable::notifyChange();
	}
	/** The number of segments of a 3D ellipse (in both "axis") (default=20) */
	void set3DsegmentsCount(unsigned int N)
	{
		m_3D_segments = N;
		CRenderizable::notifyChange();
	}

	/** The line width for 2D ellipses or 3D wireframe ellipsoids (default=1) */
	void setLineWidth(float w)
	{
		m_lineWidth = w;
		CRenderizable::notifyChange();
	}
	float getLineWidth() const { return m_lineWidth; }

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
	CEllipsoid()
		: m_eigVal(),
		  m_eigVec(),
		  m_prevComputedCov(),
		  m_cov(2, 2),

		  m_bb_min(0, 0, 0),
		  m_bb_max(0, 0, 0)
	{
	}
	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CEllipsoid() override = default;
};

}  // namespace mrpt::opengl
