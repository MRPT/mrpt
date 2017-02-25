/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPointPDF_H
#define CPointPDF_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CProbabilityDensityFunction.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPosePDF.h>

namespace mrpt
{
namespace poses
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPointPDF, mrpt::utils::CSerializable )

	/** Declares a class that represents a Probability Distribution
	 *    function (PDF) of a 3D point (x,y,z).
	 *   This class is just the base class for unifying many diferent
	 *    ways this PDF can be implemented.
	 *
	 *  For convenience, a pose composition is also defined for any
	 *    PDF derived class, changeCoordinatesReference, in the form of a method rather than an operator.
	 *
	 *  For a similar class for 6D poses (a 3D point with attitude), see CPose3DPDF
	 *
	 *  See also the tutorial on <a href="http://www.mrpt.org/Probability_Density_Distributions_Over_Spatial_Representations">probabilistic spatial representations in the MRPT</a>.
	 *
	 * \sa CPoint3D
	 * \ingroup poses_pdf_grp
	 */
	class BASE_IMPEXP CPointPDF : public mrpt::utils::CSerializable, public mrpt::utils::CProbabilityDensityFunction<CPoint3D,3>
	{
		DEFINE_VIRTUAL_SERIALIZABLE( CPointPDF )

	 public:
		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		virtual void  copyFrom(const CPointPDF &o) = 0;


		/** Bayesian fusion of two point distributions (product of two distributions->new distribution), then save the result in this object (WARNING: See implementing classes to see classes that can and cannot be mixtured!)
		  * \param p1 The first distribution to fuse
		  * \param p2 The second distribution to fuse
		  * \param minMahalanobisDistToDrop If set to different of 0, the result of very separate Gaussian modes (that will result in negligible components) in SOGs will be dropped to reduce the number of modes in the output.
		  */
		virtual void  bayesianFusion( const CPointPDF &p1, const CPointPDF &p2, const double &minMahalanobisDistToDrop = 0)  = 0 ;

		enum { is_3D_val = 1 };
		static inline bool is_3D() { return is_3D_val!=0; }
		enum { is_PDF_val = 1 };
		static inline bool is_PDF() { return is_PDF_val!=0; }

		/** Returns a 3D representation of this PDF (it doesn't clear the current contents of out_obj, but append new OpenGL objects to that list)
		  * \note Needs the mrpt-opengl library, and using mrpt::opengl::CSetOfObjectsPtr as template argument.
		  * \note By default, ellipsoids for the confidence intervals of  "q=3" are drawn; for more mathematical details, see  CGeneralizedEllipsoidTemplate::setQuantiles()
		  */
		template <class OPENGL_SETOFOBJECTSPTR>
		inline void getAs3DObject(OPENGL_SETOFOBJECTSPTR &out_obj) const {
			typedef typename OPENGL_SETOFOBJECTSPTR::value_type SETOFOBJECTS;
			out_obj->insertCollection( *SETOFOBJECTS::posePDF2opengl(*this) );
		}

		/** Returns a 3D representation of this PDF.
		  * \note Needs the mrpt-opengl library, and using mrpt::opengl::CSetOfObjectsPtr as template argument.
		  */
		template <class OPENGL_SETOFOBJECTSPTR,class OPENGL_SETOFOBJECTS>
		inline OPENGL_SETOFOBJECTSPTR getAs3DObject() const {
			typedef typename OPENGL_SETOFOBJECTSPTR::value_type SETOFOBJECTS;
			return SETOFOBJECTS::posePDF2opengl(*this);
		}

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPointPDF, mrpt::utils::CSerializable )


	} // End of namespace
} // End of namespace

#endif
