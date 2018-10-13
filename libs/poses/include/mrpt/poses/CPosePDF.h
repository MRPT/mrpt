/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CProbabilityDensityFunction.h>

namespace mrpt::poses
{
class CPosePDFGaussian;  // frd decl.

/** Declares a class that represents a probability density function (pdf) of a
 * 2D pose (x,y,phi).
 *   This class is just the base class for unifying many diferent ways this pdf
 * can be implemented.
 *
 *  For convenience, a pose composition is also defined for any pdf derived
 * class,
 *   changeCoordinatesReference, in the form of a method rather than an
 * operator.
 *
 *
 *  See also the tutorial on <a
 * href="http://www.mrpt.org/Probability_Density_Distributions_Over_Spatial_Representations"
 * >probabilistic spatial representations in the MRPT</a>.
 *
 * \sa CPose2D, CPose3DPDF, CPoseRandomSampler
 * \ingroup poses_pdf_grp
 */
class CPosePDF : public mrpt::serialization::CSerializable,
				 public mrpt::math::CProbabilityDensityFunction<CPose2D, 3>
{
	DEFINE_VIRTUAL_SERIALIZABLE(CPosePDF)

   public:
	/** Copy operator, translating if necesary (for example, between particles
	 * and gaussian representations)
	 */
	virtual void copyFrom(const CPosePDF& o) = 0;

	/** Bayesian fusion of two pose distributions (product of two
	 * distributions->new distribution), then save the result in this object
	 * (WARNING: See implementing classes to see classes that can and cannot be
	 * mixtured!)
	 * \param p1 The first distribution to fuse
	 * \param p2 The second distribution to fuse
	 * \param minMahalanobisDistToDrop If set to different of 0, the result of
	 * very separate Gaussian modes (that will result in negligible components)
	 * in SOGs will be dropped to reduce the number of modes in the output.
	 */
	virtual void bayesianFusion(
		const CPosePDF& p1, const CPosePDF& p2,
		const double minMahalanobisDistToDrop = 0) = 0;

	/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
	 */
	virtual void inverse(CPosePDF& o) const = 0;

	virtual void changeCoordinatesReference(
		const CPose3D& newReferenceBase) = 0;

	/** This static method computes the pose composition Jacobians, with these
	   formulas:
		\code
			df_dx =
			[ 1, 0, -sin(phi_x)*x_u-cos(phi_x)*y_u ]
			[ 0, 1,  cos(phi_x)*x_u-sin(phi_x)*y_u ]
			[ 0, 0,                              1 ]

			df_du =
			[ cos(phi_x) , -sin(phi_x) ,  0  ]
			[ sin(phi_x) ,  cos(phi_x) ,  0  ]
			[         0  ,          0  ,  1  ]
		\endcode
	  */
	static void jacobiansPoseComposition(
		const CPose2D& x, const CPose2D& u, mrpt::math::CMatrixDouble33& df_dx,
		mrpt::math::CMatrixDouble33& df_du, const bool compute_df_dx = true,
		const bool compute_df_du = true);

	/** \overload */
	static void jacobiansPoseComposition(
		const CPosePDFGaussian& x, const CPosePDFGaussian& u,
		mrpt::math::CMatrixDouble33& df_dx, mrpt::math::CMatrixDouble33& df_du);

	enum
	{
		is_3D_val = 0
	};
	static constexpr bool is_3D() { return is_3D_val != 0; }
	enum
	{
		is_PDF_val = 1
	};
	static constexpr bool is_PDF() { return is_PDF_val != 0; }
	/** Returns a 3D representation of this PDF (it doesn't clear the current
	 * contents of out_obj, but append new OpenGL objects to that list)
	 * \note Needs the mrpt-opengl library, and using
	 * mrpt::opengl::CSetOfObjects::Ptr as template argument.
	 * \note By default, ellipsoids for the confidence intervals of  "q=3" are
	 * drawn; for more mathematical details, see
	 * CGeneralizedEllipsoidTemplate::setQuantiles()
	 */
	template <class OPENGL_SETOFOBJECTSPTR>
	inline void getAs3DObject(OPENGL_SETOFOBJECTSPTR& out_obj) const
	{
		using SETOFOBJECTS = typename OPENGL_SETOFOBJECTSPTR::element_type;
		out_obj->insertCollection(*SETOFOBJECTS::posePDF2opengl(*this));
	}

	/** Returns a 3D representation of this PDF.
	 * \note Needs the mrpt-opengl library, and using
	 * mrpt::opengl::CSetOfObjects::Ptr as template argument.
	 */
	template <class OPENGL_SETOFOBJECTSPTR>
	inline OPENGL_SETOFOBJECTSPTR getAs3DObject() const
	{
		using SETOFOBJECTS = typename OPENGL_SETOFOBJECTSPTR::element_type;
		return SETOFOBJECTS::posePDF2opengl(*this);
	}

   public:
	MRPT_MAKE_ALIGNED_OPERATOR_NEW
};  // End of class def.

}  // namespace mrpt::poses
