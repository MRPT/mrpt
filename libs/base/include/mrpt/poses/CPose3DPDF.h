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
