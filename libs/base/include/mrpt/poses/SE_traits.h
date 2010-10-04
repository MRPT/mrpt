/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef MRPT_SE3_TRAITS_H
#define MRPT_SE3_TRAITS_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

namespace mrpt
{
	namespace poses
	{

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
			static void exp(const array_t &x, CPose3D &P) { P = CPose3D::exp(x); } 

			/** Logarithm map in SE(3) */
			static void ln(const CPose3D &P, array_t &x) { P.ln(x); } 

			/** Return one or both of the following 6x6 Jacobians, useful in graph-slam problems:
			  *   \f[  \frac{\partial }{\partial \epsilon}  \f]
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
			static void exp(const array_t &x, CPose2D &P) { P.x(x[0]); P.y(x[1]); P.phi(x[2]); } 

			/** Logarithm map in SE(2) */
			static void ln(const CPose2D &P, array_t &x) { x[0] = P.x(); x[1] = P.y(); x[2] = P.phi();  } 

			/** Return one or both of the following 3x3 Jacobians, useful in graph-slam problems:
			  *   \f[  \frac{\partial }{\partial \epsilon}  \f]
			  */
			static void jacobian_dP1DP2inv_depsilon(
				const CPose2D &P1DP2inv,
				matrix_VxV_t *df_de1, 
				matrix_VxV_t *df_de2);

		}; // end SE_traits


	} // End of namespace
} // End of namespace

#endif
