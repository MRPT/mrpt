/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#ifndef MRPT_SE3_TRAITS_H
#define MRPT_SE3_TRAITS_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

namespace mrpt
{
	namespace poses
	{
		/** \addtogroup poses_grp
		  *  @{ */

		/** A helper class for SE(2) and SE(3) geometry-related transformations, on-manifold optimization Jacobians, etc.
		  * \sa SE_traits<2>, SE_traits<3>, CPose3D, CPose2D
		  */
		template <size_t DOF> struct BASE_IMPEXP SE_traits;

		/** Specialization of SE for 3D poses \sa SE_traits */
		template <> struct BASE_IMPEXP SE_traits<3>
		{
			enum { VECTOR_SIZE = 6 };
			typedef CArrayDouble<VECTOR_SIZE> array_t;
			typedef CMatrixFixedNumeric<double,VECTOR_SIZE,VECTOR_SIZE> matrix_VxV_t;
			typedef CPose3D  pose_t;

			/** Exponential map in SE(3) */
			static inline void exp(const array_t &x, CPose3D &P) { P = CPose3D::exp(x); }

			/** Logarithm map in SE(3) */
			static inline void ln(const CPose3D &P, array_t &x) { P.ln(x); }

			/** A pseudo-Logarithm map in SE(3), where the output = [X,Y,Z, Ln(ROT)], that is, the normal
			  *  SO(3) logarithm is used for the rotation components, but the translation is left unmodified.
			  */
			static void pseudo_ln(const CPose3D &P, array_t &x);

			/** Return one or both of the following 6x6 Jacobians, useful in graph-slam problems:
			  *   \f[  \frac{\partial pseudoLn(P_1 D P_2^{-1}) }{\partial \epsilon_1}  \f]
			  *   \f[  \frac{\partial pseudoLn(P_1 D P_2^{-1}) }{\partial \epsilon_2}  \f]
			  *  With \f$ \epsilon_1 \f$ and \f$ \epsilon_2 \f$ being increments in the linearized manifold for P1 and P2.
			  */
			static void jacobian_dP1DP2inv_depsilon(
				const CPose3D &P1DP2inv,
				matrix_VxV_t *df_de1,
				matrix_VxV_t *df_de2);

		}; // end SE_traits

		/** Specialization of SE for 2D poses \sa SE_traits */
		template <> struct BASE_IMPEXP SE_traits<2>
		{
			enum { VECTOR_SIZE = 3 };
			typedef CArrayDouble<VECTOR_SIZE> array_t;
			typedef CMatrixFixedNumeric<double,VECTOR_SIZE,VECTOR_SIZE> matrix_VxV_t;
			typedef CPose2D  pose_t;

			/** Exponential map in SE(2) */
			static inline void exp(const array_t &x, CPose2D &P) { P.x(x[0]); P.y(x[1]); P.phi(x[2]); }

			/** Logarithm map in SE(2) */
			static inline void ln(const CPose2D &P, array_t &x) { x[0] = P.x(); x[1] = P.y(); x[2] = P.phi();  }

			/** A pseudo-Logarithm map in SE(2), where the output = [X,Y, Ln(ROT)], that is, the normal
			  *  SO(2) logarithm is used for the rotation components, but the translation is left unmodified.
			  */
			static inline void pseudo_ln(const CPose2D &P, array_t &x) { ln(P,x); }

			/** Return one or both of the following 3x3 Jacobians, useful in graph-slam problems:
			  *   \f[  \frac{\partial pseudoLn(P_1 D P_2^{-1}) }{\partial \epsilon_1}  \f]
			  *   \f[  \frac{\partial pseudoLn(P_1 D P_2^{-1}) }{\partial \epsilon_2}  \f]
			  *  With \f$ \epsilon_1 \f$ and \f$ \epsilon_2 \f$ being increments in the linearized manifold for P1 and P2.
			  */
			static void jacobian_dP1DP2inv_depsilon(
				const CPose2D &P1DP2inv,
				matrix_VxV_t *df_de1,
				matrix_VxV_t *df_de2);

		}; // end SE_traits

		/** @} */ // end of grouping

	} // End of namespace
} // End of namespace

#endif
