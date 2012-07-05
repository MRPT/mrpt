/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CPOSEPDF_H
#define CPOSEPDF_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/utils/CProbabilityDensityFunction.h>


namespace mrpt
{
namespace poses
{
	using namespace mrpt::math;

	class CPosePDFGaussian; // frd decl.

	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPosePDF, mrpt::utils::CSerializable )

	/** Declares a class that represents a probability density function (pdf) of a 2D pose (x,y,phi).
	 *   This class is just the base class for unifying many diferent ways this pdf can be implemented.
	 *
	 *  For convenience, a pose composition is also defined for any pdf derived class,
	 *   changeCoordinatesReference, in the form of a method rather than an operator.
	 *
	 *
	 *  See also the tutorial on <a href="http://www.mrpt.org/Probability_Density_Distributions_Over_Spatial_Representations" >probabilistic spatial representations in the MRPT</a>.
	 *
	 * \sa CPose2D, CPose3DPDF, CPoseRandomSampler
	 * \ingroup poses_pdf_grp
	 */
	class BASE_IMPEXP CPosePDF : public mrpt::utils::CSerializable, public mrpt::utils::CProbabilityDensityFunction<CPose2D,3>
	{
		DEFINE_VIRTUAL_SERIALIZABLE( CPosePDF )

	public:
		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		virtual void  copyFrom(const CPosePDF &o) = 0;


		/** Bayesian fusion of two pose distributions (product of two distributions->new distribution), then save the result in this object (WARNING: See implementing classes to see classes that can and cannot be mixtured!)
		  * \param p1 The first distribution to fuse
		  * \param p2 The second distribution to fuse
		  * \param minMahalanobisDistToDrop If set to different of 0, the result of very separate Gaussian modes (that will result in negligible components) in SOGs will be dropped to reduce the number of modes in the output.
		  */
		virtual void  bayesianFusion(const  CPosePDF &p1,const  CPosePDF &p2, const double&minMahalanobisDistToDrop = 0)  = 0 ;

		/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
		  */
		virtual void  inverse(CPosePDF &o) const = 0;


		/** This static method computes the pose composition Jacobians, with these formulas:
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
			const CPose2D &x,
			const CPose2D &u,
			CMatrixDouble33			 &df_dx,
			CMatrixDouble33			 &df_du);

		/** \overload */
		static void jacobiansPoseComposition(
			const CPosePDFGaussian &x,
			const CPosePDFGaussian &u,
			CMatrixDouble33			 &df_dx,
			CMatrixDouble33			 &df_du);



		enum { is_3D_val = 0 };
		static inline bool is_3D() { return is_3D_val!=0; }
		enum { is_PDF_val = 1 };
		static inline bool is_PDF() { return is_PDF_val!=0; }

		/** Returns a 3D representation of this PDF (it doesn't clear the current contents of out_obj, but append new OpenGL objects to that list)
		  * \note Needs the mrpt-opengl library, and using mrpt::opengl::CSetOfObjectsPtr as template argument.
		  */
		template <class OPENGL_SETOFOBJECTSPTR>
		inline void getAs3DObject(OPENGL_SETOFOBJECTSPTR &out_obj) const {
			typedef typename OPENGL_SETOFOBJECTSPTR::value_type SETOFOBJECTS;
			out_obj->insertCollection( *SETOFOBJECTS::posePDF2opengl(*this) );
		}

		/** Returns a 3D representation of this PDF.
		  * \note Needs the mrpt-opengl library, and using mrpt::opengl::CSetOfObjectsPtr as template argument.
		  */
		template <class OPENGL_SETOFOBJECTSPTR>
		inline OPENGL_SETOFOBJECTSPTR getAs3DObject() const {
			typedef typename OPENGL_SETOFOBJECTSPTR::value_type SETOFOBJECTS;
			return SETOFOBJECTS::posePDF2opengl(*this);
		}


	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
