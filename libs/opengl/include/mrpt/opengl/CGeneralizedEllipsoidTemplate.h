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
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>
#include <mrpt/serialization/CArchive.h>  // for >> ops

namespace mrpt::opengl
{
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
class CGeneralizedEllipsoidTemplate
	: virtual public CRenderizableShaderTriangles,
	  virtual public CRenderizableShaderWireFrame
{
   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void render(const RenderContext& rc) const override
	{
		switch (rc.shader_id)
		{
			case DefaultShaderID::TRIANGLES:
				if (m_drawSolid3D) CRenderizableShaderTriangles::render(rc);
				break;
			case DefaultShaderID::WIREFRAME:
				if (!m_drawSolid3D) CRenderizableShaderWireFrame::render(rc);
				break;
		};
	}
	void renderUpdateBuffers() const override
	{
		// 1) Update eigenvectors/values:
		if (m_needToRecomputeEigenVals)
		{
			m_needToRecomputeEigenVals = false;
			// Handle the special case of an ellipsoid of volume = 0
			const double d = m_cov.det();
			// Note: "d!=d" is a great test for invalid numbers, don't remove!
			if (std::abs(d) < 1e-20 || d != d)
			{
				// All zeros:
				m_U.setZero(DIM, DIM);
			}
			else
			{
				// A valid matrix:
				m_cov.chol(m_U);
			}
		}

		// Only if all the eigenvalues are !=0
		bool eig_ok = true;
		for (int i = 0; i < DIM; i++)
			if (m_U.coeff(i, i) == 0) eig_ok = false;

		// 2) Generate "standard" ellipsoid:
		std::vector<array_parameter_t> params_pts;
		cov_matrix_t Uscaled = m_U;
		Uscaled *= static_cast<double>(m_quantiles);
		generatePoints(Uscaled, params_pts);

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

		if (!eig_ok)
		{
			CRenderizableShaderTriangles::m_triangles.clear();
			CRenderizableShaderWireFrame::m_vertex_buffer_data.clear();
			CRenderizableShaderWireFrame::m_color_buffer_data.clear();
		}
		else
		{
			CRenderizableShaderTriangles::renderUpdateBuffers();
			CRenderizableShaderWireFrame::renderUpdateBuffers();
		}
	}
	virtual shader_list_t requiredShaders() const override
	{
		// May use up to two shaders (triangles and lines):
		return {DefaultShaderID::WIREFRAME, DefaultShaderID::TRIANGLES};
	}
	// Render precomputed points in m_render_pts:
	void onUpdateBuffers_Wireframe() override { implUpdate_Wireframe(); }
	// Render precomputed points in m_render_pts:
	void onUpdateBuffers_Triangles() override { implUpdate_Triangles(); }
	/** @} */

	/** The type of fixed-size covariance matrices for this representation
	 */
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
		ASSERT_EQUAL_(new_cov.cols(), new_cov.rows());
		ASSERT_EQUAL_(new_cov.cols(), DIM);
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
	 *   This value of "quantiles" \a q should be set to the square root of
	 *the chi-squared inverse cdf corresponding to the desired confidence
	 *interval. <b>Note that this value depends on the dimensionality</b>.
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

	/** Set the number of segments of the surface/curve (higher means with
	 * greater resolution) */
	void setNumberOfSegments(const uint32_t numSegments)
	{
		m_numSegments = numSegments;
		CRenderizable::notifyChange();
	}
	uint32_t getNumberOfSegments() { return m_numSegments; }

	/** If set to "true", a whole ellipsoid surface will be drawn, or if
	 * set to "false" (default) it will be drawn as a "wireframe". */
	void enableDrawSolid3D(bool v)
	{
		m_drawSolid3D = v;
		CRenderizable::notifyChange();
	}

	/** Evaluates the bounding box of this object (including possible
	 * children) in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override
	{
		bb_min = m_bb_min;
		bb_max = m_bb_max;
	}

	/** Ray tracing  */
	virtual bool traceRay(
		const mrpt::poses::CPose3D& o, double& dist) const override
	{
		MRPT_UNUSED_PARAM(o);
		MRPT_UNUSED_PARAM(dist);
		THROW_EXCEPTION("Not implemented ");
	}

   protected:
	/** To be implemented by derived classes: maps, using some arbitrary
	 * space transformation, a list of points defining an ellipsoid in
	 * parameter space into their corresponding points in 2D/3D space.
	 */
	virtual void transformFromParameterSpace(
		const std::vector<array_point_t>& params_pts,
		std::vector<array_point_t>& out_pts) const = 0;

	mutable cov_matrix_t m_cov;
	mean_vector_t m_mean;
	mutable bool m_needToRecomputeEigenVals{true};
	/** The number of "sigmas" for drawing the ellipse/ellipsoid (default=3)
	 */
	float m_quantiles{3.f};
	/** Number of segments in 2D/3D ellipsoids (default=20) */
	uint32_t m_numSegments{20};
	mutable mrpt::math::TPoint3D m_bb_min{0, 0, 0}, m_bb_max{0, 0, 0};

	/** If set to "true", a whole ellipsoid surface will be drawn, or
	 * if set to "false" (default)it will be drawn as a "wireframe". */
	bool m_drawSolid3D{false};

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

	CGeneralizedEllipsoidTemplate() = default;
	virtual ~CGeneralizedEllipsoidTemplate() override = default;

	// Uscaled, m_mean, params_pts, m_numSegments, m_numSegments
	void generatePoints(
		const cov_matrix_t& U,
		std::vector<array_parameter_t>& out_params_pts) const;

	void implUpdate_Wireframe();
	void implUpdate_Triangles();

   public:
	// Solve virtual public inheritance ambiguity:
	virtual const mrpt::rtti::TRuntimeClassId* GetRuntimeClass() const override
	{
		return CRenderizableShaderWireFrame::GetRuntimeClass();
	}
};

}  // namespace mrpt::opengl
