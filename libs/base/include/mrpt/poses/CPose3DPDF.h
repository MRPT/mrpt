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
#ifndef CPose3DPDF_H
#define CPose3DPDF_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/utils/CProbabilityDensityFunction.h>

namespace mrpt
{
namespace poses
{
	using namespace mrpt::math;

	class CPosePDF;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPose3DPDF, mrpt::utils::CSerializable )

	/** Declares a class that represents a Probability Density Function (PDF) of a 3D pose (6D actually).
	 *   This class is just the base class for unifying many diferent
	 *    ways this PDF can be implemented.
	 *
	 *  For convenience, a pose composition is also defined for any
	 *    PDF derived class, changeCoordinatesReference, in the form of a method rather than an operator.
     *
	 *  For a similar class for 3D points (without attitude), see CPointPDF
	 *
	 *
	 *  See also the tutorial on <a href="http://www.mrpt.org/Probability_Density_Distributions_Over_Spatial_Representations">probabilistic spatial representations in the MRPT</a>.
	 *
	 * \sa CPose3D, CPosePDF, CPointPDF
	 * \ingroup poses_pdf_grp
	 */
	class BASE_IMPEXP CPose3DPDF : public mrpt::utils::CSerializable, public mrpt::utils::CProbabilityDensityFunction<CPose3D,6>
	{
		DEFINE_VIRTUAL_SERIALIZABLE( CPose3DPDF )

	 public:
		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  * \sa createFrom2D
		  */
		virtual void  copyFrom(const CPose3DPDF &o) = 0;

		/** This is a static transformation method from 2D poses to 3D PDFs, preserving the representation type (particles->particles, Gaussians->Gaussians,etc)
		  *  It returns a new object of any of the derived classes of CPose3DPDF. This object must be deleted by the user when not required anymore.
		  *  \sa copyFrom
		  */
		static CPose3DPDF* createFrom2D(const CPosePDF &o);

		/** Bayesian fusion of two pose distributions, then save the result in this object (WARNING: Currently only distributions of the same class can be fused! eg, gaussian with gaussian,etc) */
		virtual void  bayesianFusion( const CPose3DPDF &p1, const CPose3DPDF &p2 )  = 0 ;

		/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF */
		virtual void  inverse(CPose3DPDF &o) const = 0;

		/** This static method computes the pose composition Jacobians.
		*
		* See this techical report: http:///www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty
		*
		* Direct equations (for the covariances) in yaw-pitch-roll are too complex.
		*  Make a way around them and consider instead this path:
		* \code
		*      X(6D)       U(6D)
		*        |           |
		*        v           v
		*      X(7D)       U(7D)
		*        |           |
		*        +--- (+) ---+
		*              |
		*              v
		*            RES(7D)
		*              |
		*              v
		*            RES(6D)
		* \endcode
		*
		*/
		static void jacobiansPoseComposition(
			const CPose3D &x,
			const CPose3D &u,
			CMatrixDouble66  &df_dx,
			CMatrixDouble66	 &df_du);


		enum { is_3D_val = 1 };
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
