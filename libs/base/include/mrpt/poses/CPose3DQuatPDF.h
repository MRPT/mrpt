/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
#ifndef CPose3DQuatPDF_H
#define CPose3DQuatPDF_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/utils/CProbabilityDensityFunction.h>

namespace mrpt
{
	namespace poses
	{
		using namespace mrpt::math;

		class CPosePDF;
		class CPose3DPDF;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPose3DQuatPDF, mrpt::utils::CSerializable )

		/** Declares a class that represents a Probability Density Function (PDF) of a 3D pose (6D actually), by means of a 7-vector with a translation [x y z] and a quaternion [qr qx qy qz].
		 *   This class is just the base class for unifying many diferent ways this PDF can be implemented.
		 *
		 *  For convenience, a pose composition is also defined for any
		 *    PDF derived class, changeCoordinatesReference, in the form of a method rather than an operator.
		 *
		 *  - For a similar class for 3D points (without attitude), see CPointPDF.
		 *  - For a similar class for 3D poses  (with Euler angles instead of quaternions), see CPose3DPDF.
		 *
		 *
		 *  See also the tutorial on <a href="http://www.mrpt.org/Probability_Density_Distributions_Over_Spatial_Representations" >probabilistic spatial representations in the MRPT</a>.
		 *
		 * \sa CPose3DQuatPDF, CPose3DPDF
		 * \ingroup poses_pdf_grp
		 */
		class BASE_IMPEXP CPose3DQuatPDF :
			public mrpt::utils::CSerializable,
			public mrpt::utils::CProbabilityDensityFunction<CPose3DQuat,7>
		{
			DEFINE_VIRTUAL_SERIALIZABLE( CPose3DQuatPDF )

		 public:
			/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
			  * \sa createFrom2D
			  */
			virtual void  copyFrom(const CPose3DQuatPDF &o) = 0;

			/** This is a static transformation method from 2D poses to 3D PDFs, preserving the representation type (particles->particles, Gaussians->Gaussians,etc)
			  *  It returns a new object of any of the derived classes of CPose3DQuatPDF. This object must be deleted by the user when not required anymore.
			  *  \sa copyFrom
			  */
			static CPose3DQuatPDF* createFrom2D(const CPosePDF &o);

			/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF */
			virtual void  inverse(CPose3DQuatPDF &o) const = 0;

			/** This static method computes the two Jacobians of a pose composition operation $f(x,u)= x \oplus u$
			  *  \param out_x_oplus_u If set to !=NULL, the result of "x+u" will be stored here (it will be computed internally anyway).
			  *  To see the mathematical derivation of the formulas, refer to the technical report here:
			  *   - http://www.mrpt.org/Probability_Density_Distributions_Over_Spatial_Representations
			  */
			static void jacobiansPoseComposition(
				const CPose3DQuat &x,
				const CPose3DQuat &u,
				CMatrixDouble77	  &df_dx,
				CMatrixDouble77	  &df_du,
				CPose3DQuat       *out_x_oplus_u=NULL);


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
