/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPoint2DPDF_H
#define CPoint2DPDF_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CProbabilityDensityFunction.h>
#include <mrpt/poses/CPoint2D.h>

namespace mrpt
{
namespace poses
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPoint2DPDF, mrpt::utils::CSerializable )

	/** Declares a class that represents a Probability Distribution function (PDF) of a 2D point (x,y).
	 *   This class is just the base class for unifying many diferent
	 *    ways this PDF can be implemented.
	 *
	 *  For convenience, a pose composition is also defined for any
	 *    PDF derived class, changeCoordinatesReference, in the form of a method rather than an operator.
	 *
	 *  For a similar class for 6D poses (a 3D point with attitude), see CPose3DPDF
	 *
	 *  See also the tutorial on <a href="http://www.mrpt.org/Probability_Density_Distributions_Over_Spatial_Representations" >probabilistic spatial representations in the MRPT</a>.
	 * \ingroup poses_pdf_grp
	 * \sa CPoint2D, CPointPDF
	 */
	class BASE_IMPEXP CPoint2DPDF : public mrpt::utils::CSerializable, public mrpt::utils::CProbabilityDensityFunction<CPoint2D,2>
	{
		DEFINE_VIRTUAL_SERIALIZABLE( CPoint2DPDF )

	 public:
		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		virtual void  copyFrom(const CPoint2DPDF &o) = 0;

		/** Bayesian fusion of two point distributions (product of two distributions->new distribution), then save the result in this object (WARNING: See implementing classes to see classes that can and cannot be mixtured!)
		  * \param p1 The first distribution to fuse
		  * \param p2 The second distribution to fuse
		  * \param minMahalanobisDistToDrop If set to different of 0, the result of very separate Gaussian modes (that will result in negligible components) in SOGs will be dropped to reduce the number of modes in the output.
		  */
		virtual void  bayesianFusion( const CPoint2DPDF &p1, const CPoint2DPDF &p2, const double &minMahalanobisDistToDrop = 0)  = 0 ;

		enum { is_3D_val = 0 };
		static inline bool is_3D() { return is_3D_val!=0; }
		enum { is_PDF_val = 1 };
		static inline bool is_PDF() { return is_PDF_val!=0; }

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPoint2DPDF, mrpt::utils::CSerializable )


	} // End of namespace
} // End of namespace

#endif
