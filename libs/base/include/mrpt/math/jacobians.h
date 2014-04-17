/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_math_jacobians_H
#define  mrpt_math_jacobians_H

#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPose3DPDF.h>

namespace mrpt
{
	namespace math
	{

		/** Estimate the Jacobian of a multi-dimensional function around a point "x", using finite differences of a given size in each input dimension.
			*  The template argument USERPARAM is for the data can be passed to the functor.
			*   If it is not required, set to "int" or any other basic type.
			*
			*  This is a generic template which works with:
			*    VECTORLIKE: vector_float, vector_double, CArrayNumeric<>, double [N], ...
			*    MATRIXLIKE: CMatrixTemplateNumeric, CMatrixFixedNumeric
			*/
		template <class VECTORLIKE,class VECTORLIKE2, class VECTORLIKE3, class MATRIXLIKE, class USERPARAM >
		void estimateJacobian(
			const VECTORLIKE 	&x,
			void 				(*functor) (const VECTORLIKE &x,const USERPARAM &y, VECTORLIKE3  &out),
			const VECTORLIKE2 	&increments,
			const USERPARAM		&userParam,
			MATRIXLIKE 			&out_Jacobian )
		{
			MRPT_START
			ASSERT_(x.size()>0 && increments.size() == x.size());

			size_t m = 0;		// will determine automatically on the first call to "f":
			const size_t n = x.size();

			for (size_t j=0;j<n;j++) { ASSERT_( increments[j]>0 ) }		// Who knows...

			VECTORLIKE3	f_minus, f_plus;
			VECTORLIKE	x_mod(x);

			// Evaluate the function "i" with increments in the "j" input x variable:
			for (size_t j=0;j<n;j++)
			{
				// Create the modified "x" vector:
				x_mod[j]=x[j]+increments[j];
				functor(x_mod,userParam,   f_plus);

				x_mod[j]=x[j]-increments[j];
				functor(x_mod,userParam,   f_minus);

				x_mod[j]=x[j]; // Leave as original
				const double Ax_2_inv = 0.5/increments[j];

				// The first time?
				if (j==0)
				{
					m = f_plus.size();
					out_Jacobian.setSize(m,n);
				}

				for (size_t i=0;i<m;i++)
					out_Jacobian.get_unsafe(i,j) = Ax_2_inv* (f_plus[i]-f_minus[i]);

			} // end for j

			MRPT_END
		}


		/** A collection of functions to compute jacobians of diverse transformations, etc (some functions are redirections to existing methods elsewhere, so this namespace is actually used with grouping purposes).
		  *  Since most functions in this namespace are inline, their use implies no execution time overload and the code may be more clear to read, so it's recommended to use them where needed.
		 * \ingroup mrpt_base_grp
		  */
		namespace jacobians
		{
			using mrpt::poses::CPose3D;
			using mrpt::math::CQuaternionDouble;

			/** Computes the 4x3 Jacobian of the transformation from a 3D pose angles (yaw pitch roll) into a Quaternion, that is, the Jacobian of:
			  * \f[ \mathbf{q} = \left( \begin{array}{c} \cos (\phi /2) \cos (\theta /2) \cos (\psi /2) +  \sin (\phi /2) \sin (\theta /2) \sin (\psi /2) \\ \sin (\phi /2) \cos (\theta /2) \cos (\psi /2) -  \cos (\phi /2) \sin (\theta /2) \sin (\psi /2) \\ \cos (\phi /2) \sin (\theta /2) \cos (\psi /2) +  \sin (\phi /2) \cos (\theta /2) \sin (\psi /2) \\ \cos (\phi /2) \cos (\theta /2) \sin (\psi /2) -  \sin (\phi /2) \sin (\theta /2) \cos (\psi /2) \\ \end{array}\right) \f]
			  * With : \f$ \phi = roll \f$,  \f$ \theta = pitch \f$ and \f$ \psi = yaw \f$.
			  * \sa jacob_yawpitchroll_from_quat, mrpt::poses::CPose3D::getAsQuaternion
			  */
			inline void jacob_quat_from_yawpitchroll(
				mrpt::math::CMatrixFixedNumeric<double,4,3>  &out_dq_dr,
				const double yaw,
				const double pitch,
				const double roll
				)
			{
				CQuaternionDouble q(UNINITIALIZED_QUATERNION);
				CPose3D  p(0,0,0,yaw,pitch,roll);
				p.getAsQuaternion(q,&out_dq_dr);
			}

			/** Computes the 4x3 Jacobian of the transformation from a 3D pose angles (yaw pitch roll) into a Quaternion, that is, the Jacobian of:
			  * \f[ \mathbf{q} = \left( \begin{array}{c} \cos (\phi /2) \cos (\theta /2) \cos (\psi /2) +  \sin (\phi /2) \sin (\theta /2) \sin (\psi /2) \\ \sin (\phi /2) \cos (\theta /2) \cos (\psi /2) -  \cos (\phi /2) \sin (\theta /2) \sin (\psi /2) \\ \cos (\phi /2) \sin (\theta /2) \cos (\psi /2) +  \sin (\phi /2) \cos (\theta /2) \sin (\psi /2) \\ \cos (\phi /2) \cos (\theta /2) \sin (\psi /2) -  \sin (\phi /2) \sin (\theta /2) \cos (\psi /2) \\ \end{array}\right) \f]
			  * With : \f$ \phi = roll \f$,  \f$ \theta = pitch \f$ and \f$ \psi = yaw \f$.
			  * \sa jacob_yawpitchroll_from_quat, mrpt::poses::CPose3D::getAsQuaternion
			  */
			inline void jacob_quat_from_yawpitchroll(
				mrpt::math::CMatrixFixedNumeric<double,4,3> &out_dq_dr,
				const CPose3D  &in_pose
				)
			{
				CQuaternionDouble q(UNINITIALIZED_QUATERNION);
				in_pose.getAsQuaternion(q,&out_dq_dr);
			}


			/** Computes the 3x4 Jacobian of the transformation from a quaternion (qr qx qy qz) to 3D pose angles (yaw pitch roll).
			  * \sa jacob_quat_from_yawpitchroll
			  */
			inline void jacob_yawpitchroll_from_quat(
				mrpt::math::CMatrixFixedNumeric<double,3,4>  &out_dr_dq
				)
			{
				THROW_EXCEPTION("TO DO")
			}

			/** Compute the Jacobian of the rotation composition operation \f$ p = f(\cdot) = q_{this} \times r \f$, that is the 4x4 matrix \f$ \frac{\partial f}{\partial q_{this} }  \f$.
			  *  The output matrix can be a dynamic or fixed size (4x4) matrix. The input quaternion can be mrpt::math::CQuaternionFloat or mrpt::math::CQuaternionDouble.
			  */
			template <class QUATERNION,class MATRIXLIKE>
			inline void jacob_quat_rotation(const QUATERNION& quaternion, MATRIXLIKE &out_mat4x4)
			{
				quaternion.rotationJacobian(out_mat4x4);
			}

			/** Given the 3D(6D) pose composition \f$ f(x,u) = x \oplus u \f$, compute the two 6x6 Jacobians \f$ \frac{\partial f}{\partial x} \f$ and \f$ \frac{\partial f}{\partial u} \f$.
			  * For the equations, see CPose3DPDF::jacobiansPoseComposition
			  */
			inline void jacobs_6D_pose_comp(
				const CPose3D	&x,
				const CPose3D	&u,
				CMatrixDouble66		&out_df_dx,
				CMatrixDouble66		&out_df_du)
			{
				mrpt::poses::CPose3DPDF::jacobiansPoseComposition(x,u,out_df_dx,out_df_du);
			}

			/** Given the 3D(6D) pose composition \f$ f(x,u) = x \oplus u \f$, compute the two 6x6 Jacobians \f$ \frac{\partial f}{\partial x} \f$ and \f$ \frac{\partial f}{\partial u} \f$.
			  * For the equations, see CPose3DQuatPDF::jacobiansPoseComposition
			  */
			inline void jacobs_6D_pose_comp(
				const mrpt::poses::CPose3DQuat	&x,
				const mrpt::poses::CPose3DQuat	&u,
				CMatrixDouble77		&out_df_dx,
				CMatrixDouble77		&out_df_du)
			{
				mrpt::poses::CPose3DQuatPDF::jacobiansPoseComposition(x,u,out_df_dx,out_df_du);
			}

			/** Given the 2D pose composition \f$ f(x,u) = x \oplus u \f$, compute the two 3x3 Jacobians \f$ \frac{\partial f}{\partial x} \f$ and \f$ \frac{\partial f}{\partial u} \f$.
			  * For the equations, see CPosePDF::jacobiansPoseComposition
			  */
			inline void jacobs_2D_pose_comp(
				const mrpt::poses::CPosePDFGaussian &x,
				const mrpt::poses::CPosePDFGaussian &u,
				CMatrixDouble33			 &out_df_dx,
				CMatrixDouble33			 &out_df_du)
			{
				mrpt::poses::CPosePDF::jacobiansPoseComposition(x,u,out_df_dx,out_df_du);
			}

			/** Numerical estimation of the Jacobian of a user-supplied function - this template redirects to mrpt::math::estimateJacobian, see that function for documentation. */
			template <class VECTORLIKE,class VECTORLIKE2, class VECTORLIKE3, class MATRIXLIKE, class USERPARAM >
			inline void jacob_numeric_estimate(
				const VECTORLIKE 	&x,
				void 				(*functor) (const VECTORLIKE &x,const USERPARAM &y, VECTORLIKE3  &out),
				const VECTORLIKE2 	&increments,
				const USERPARAM		&userParam,
				MATRIXLIKE 			&out_Jacobian )
			{
				mrpt::math::estimateJacobian(x,functor,increments,userParam,out_Jacobian);
			}


		} // End of jacobians namespace

	} // End of MATH namespace

} // End of namespace

#endif
