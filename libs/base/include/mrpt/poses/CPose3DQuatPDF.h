/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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



			/** Returns a 3D representation of this PDF.
			  * \note Needs the mrpt-opengl library, and using mrpt::opengl::CSetOfObjectsPtr as template argument.
			  */
			template <class OPENGL_SETOFOBJECTSPTR>
			inline void getAs3DObject(OPENGL_SETOFOBJECTSPTR &out_obj) const {
				typedef typename OPENGL_SETOFOBJECTSPTR::value_type SETOFOBJECTS;
				*out_obj = *SETOFOBJECTS::posePDF2opengl(*this);
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
