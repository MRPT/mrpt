/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/matrix_serialization.h>  // for >> ops
//#include <mrpt/math/types_math.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/serialization/CArchive.h>  // for >> ops

namespace mrpt
{
namespace opengl
{
namespace detail
{
template <int DIM>
void renderGeneralizedEllipsoidTemplate(
	const std::vector<mrpt::math::CMatrixFixed<float, DIM, 1>>& pts,
	const float lineWidth, const uint32_t slices, const uint32_t stacks);
template <>
void renderGeneralizedEllipsoidTemplate<2>(
	const std::vector<mrpt::math::CMatrixFixed<float, 2, 1>>& pts,
	const float lineWidth, const uint32_t slices, const uint32_t stacks);
template <>
void renderGeneralizedEllipsoidTemplate<3>(
	const std::vector<mrpt::math::CMatrixFixed<float, 3, 1>>& pts,
	const float lineWidth, const uint32_t slices, const uint32_t stacks);

template <int DIM>
void generalizedEllipsoidPoints(
	const mrpt::math::CMatrixFixed<double, DIM, DIM>& U,
	const mrpt::math::CMatrixFixed<double, DIM, 1>& mean,
	std::vector<mrpt::math::CMatrixFixed<float, DIM, 1>>& out_params_pts,
	const uint32_t slices, const uint32_t stacks);
template <>
void generalizedEllipsoidPoints<2>(
	const mrpt::math::CMatrixFixed<double, 2, 2>& U,
	const mrpt::math::CMatrixFixed<double, 2, 1>& mean,
	std::vector<mrpt::math::CMatrixFixed<float, 2, 1>>& out_params_pts,
	const uint32_t slices, const uint32_t stacks);
template <>
void generalizedEllipsoidPoints<3>(
	const mrpt::math::CMatrixFixed<double, 3, 3>& U,
	const mrpt::math::CMatrixFixed<double, 3, 1>& mean,
	std::vector<mrpt::math::CMatrixFixed<float, 3, 1>>& out_params_pts,
	const uint32_t slices, const uint32_t stacks);
}  // namespace detail

/** A class that generalizes the concept of an ellipsoid to arbitrary
 * parameterizations of
 *  uncertainty shapes in either 2D or 3D. See derived classes for examples.
 *
 * Please read the documentation of
 * CGeneralizedEllipsoidTemplate::setQuantiles() for learning
 *  the mathematical details about setting the desired confidence interval.
 *
 *  The main method to set the modeled uncertainty is \a setCovMatrixAndMean()
 *
 * \tparam DIM The dimensionality of the parameter space, which must coincide
 * with that of the rendering space (2 or 3)
 *
 * \ingroup mrpt_opengl_grp
 */
template <int DIM>
class CGeneralizedEllipsoidTemplate : public CRenderizable
{
   public:
	/** The type of fixed-size covariance matrices for this representation */
	using cov_matrix_t = mrpt::math::CMatrixFixed<double, DIM, DIM>;
	/** The type of fixed-size vector for this representation */
	using mean_vector_t = mrpt::math::CMatrixFixed<double, DIM, 1>;

	using array_parameter_t = mrpt::math::CMatrixFixed<float, DIM, 1>;
	using array_point_t = mrpt::math::CMatrixFixed<float, DIM, 1>;

	/**  Set the NxN covariance matrix that will determine the aspect of the
	 * ellipsoid - Notice that the
	 *  covariance determines the uncertainty in the parameter space, which
	 * would be transformed by derived function
	 */
	template <typename MATRIX, typename VECTOR>
	void setCovMatrixAndMean(const MATRIX& new_cov, const VECTOR& new_mean)
	{
		MRPT_START
		ASSERT_(new_cov.cols() == new_cov.rows() && new_cov.cols() == DIM);
		m_cov = new_cov;
		m_mean = new_mean;
		m_needToRecomputeEigenVals = true;
		CRenderizable::notifyChange();
		MRPT_END
	}

	/** Gets the current uncertainty covariance of parameter space */
	const cov_matrix_t& getCovMatrix() const { return m_cov; }

	/** Changes the scale of the "sigmas" for drawing the ellipse/ellipsoid
	 *(default=3, ~97 or ~98% CI); the exact mathematical meaning is:
	 *   This value of "quantiles" \a q should be set to the square root of the
	 *chi-squared inverse cdf corresponding to
	 *   the desired confidence interval.
	 *   <b>Note that this value depends on the dimensionality</b>.
	 *   Refer to the MATLAB functions \a chi2inv() and \a chi2cdf().
	 *
	 *  Some common values follow here for the convenience of users:
	 *		- Dimensionality=3 (3D ellipsoids):
	 *			- 19.8748% CI -> q=1
	 *			- 73.8536% CI -> q=2
	 *			- 97.0709% CI -> q=3
	 *			- 99.8866% CI -> q=4
	 *		- Dimensionality=2 (2D ellipses):
	 *			- 39.347% CI -> q=1
	 *			- 86.466% CI -> q=2
	 *			- 98.8891% CI -> q=3
	 *			- 99.9664% CI -> q=4
	 *		- Dimensionality=1 (Not aplicable to this class but provided for
	 *reference):
	 *			- 68.27% CI -> q=1
	 *			- 95.45% CI -> q=2
	 *			- 99.73% CI -> q=3
	 *			- 99.9937% CI -> q=4
	 *
	 */
	void setQuantiles(float q)
	{
		m_quantiles = q;
		CRenderizable::notifyChange();
	}
	/** Refer to documentation of \a setQuantiles() */
	float getQuantiles() const { return m_quantiles; }
	/** The line width for 2D ellipses or 3D wireframe ellipsoids (default=1) */
	void setLineWidth(float w)
	{
		m_lineWidth = w;
		CRenderizable::notifyChange();
	}
	float getLineWidth() const { return m_lineWidth; }
	/** Set the number of segments of the surface/curve (higher means with
	 * greater resolution) */
	void setNumberOfSegments(const uint32_t numSegments)
	{
		m_numSegments = numSegments;
		CRenderizable::notifyChange();
	}
	uint32_t getNumberOfSegments() { return m_numSegments; }

	/** Render. If one of the eigen value of the covariance matrix of the
	 * ellipsoid is null, ellipsoid will not be rendered to ensure stability in
	 * the rendering process.
	 */
	void renderUpdateBuffers() const override
	{
		MRPT_START
		// 1) Update eigenvectors/values:
		if (m_needToRecomputeEigenVals)
		{
			m_needToRecomputeEigenVals = false;
			// Handle the special case of an ellipsoid of volume = 0
			const double d = m_cov.det();
			if (fabs(d) < 1e-20 || d != d)  // Note: "d!=d" is a great test for
			// invalid numbers, don't remove!
			{
				// All zeros:
				m_U.setZero(DIM, DIM);
			}
			else
			{
				// Not null matrix:
				m_cov.chol(m_U);
			}
		}

		// Only if all the eigenvalues are !=0
		bool eig_ok = true;
		for (int i = 0; i < DIM; i++)
			if (m_U.coeff(i, i) == 0) eig_ok = false;

		if (eig_ok)
		{
			// 2) Generate "standard" ellipsoid:
			std::vector<array_parameter_t> params_pts;
			cov_matrix_t Uscaled = m_U;
			Uscaled *= static_cast<double>(m_quantiles);
			detail::generalizedEllipsoidPoints<DIM>(
				Uscaled, m_mean, params_pts, m_numSegments, m_numSegments);

			// 3) Transform into 2D/3D render space:

			this->transformFromParameterSpace(params_pts, m_render_pts);

			// 3.5) Save bounding box:
			m_bb_min = mrpt::math::TPoint3D(
				std::numeric_limits<double>::max(),
				std::numeric_limits<double>::max(), 0);
			m_bb_max = mrpt::math::TPoint3D(
				-std::numeric_limits<double>::max(),
				-std::numeric_limits<double>::max(), 0);
			for (size_t i = 0; i < m_render_pts.size(); i++)
				for (int k = 0; k < DIM; k++)
				{
					mrpt::keep_min(m_bb_min[k], m_render_pts[i][k]);
					mrpt::keep_max(m_bb_max[k], m_render_pts[i][k]);
				}
			// Convert to coordinates of my parent:
			m_pose.composePoint(m_bb_min, m_bb_min);
			m_pose.composePoint(m_bb_max, m_bb_max);
		}
		MRPT_END
	}

	// Render precomputed points in m_render_pts:
	void render() const override
	{
		mrpt::opengl::detail::renderGeneralizedEllipsoidTemplate<DIM>(
			m_render_pts, m_lineWidth, m_numSegments, m_numSegments);
	}

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override
	{
		bb_min = m_bb_min;
		bb_max = m_bb_max;
	}

	/** Ray tracing
	 */
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override
	{
		MRPT_UNUSED_PARAM(o);
		MRPT_UNUSED_PARAM(dist);
		THROW_EXCEPTION("Not implemented ");
	}

   protected:
	/** To be implemented by derived classes: maps, using some arbitrary space
	 * transformation, a list of points
	 *  defining an ellipsoid in parameter space into their corresponding
	 * points in 2D/3D space.
	 */
	virtual void transformFromParameterSpace(
		const std::vector<array_point_t>& params_pts,
		std::vector<array_point_t>& out_pts) const = 0;

	mutable cov_matrix_t m_cov;
	mean_vector_t m_mean;
	mutable bool m_needToRecomputeEigenVals{true};
	/** The number of "sigmas" for drawing the ellipse/ellipsoid (default=3) */
	float m_quantiles{3.f};
	/** The line width for 2D ellipses or 3D wireframe ellipsoids (default=1) */
	float m_lineWidth{1.f};
	/** Number of segments in 2D/3D ellipsoids (default=10) */
	uint32_t m_numSegments{50};
	mutable mrpt::math::TPoint3D m_bb_min, m_bb_max;

	/** Cholesky U triangular matrix cache. */
	mutable cov_matrix_t m_U;
	mutable std::vector<array_point_t> m_render_pts;

	void thisclass_writeToStream(mrpt::serialization::CArchive& out) const
	{
		using namespace mrpt::math;

		const uint8_t version = 0;
		out << version << m_cov << m_mean << m_quantiles << m_lineWidth
			<< m_numSegments;
	}
	void thisclass_readFromStream(mrpt::serialization::CArchive& in)
	{
		uint8_t version;
		in >> version;
		switch (version)
		{
			case 0:
			{
				in >> m_cov >> m_mean >> m_quantiles >> m_lineWidth >>
					m_numSegments;
				m_needToRecomputeEigenVals = true;
			}
			break;
			default:
				MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
		};
		CRenderizable::notifyChange();
	}

	CGeneralizedEllipsoidTemplate() : m_bb_min(0, 0, 0), m_bb_max(0, 0, 0) {}
	~CGeneralizedEllipsoidTemplate() override = default;
};

}  // namespace opengl

}  // namespace mrpt
