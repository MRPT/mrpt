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

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/poses/SE_traits.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;

/** A pseudo-Logarithm map in SE(3), where the output = [X,Y,Z, Ln(ROT)], that is, the normal
  *  SO(3) logarithm is used for the rotation components, but the translation is left unmodified.
  */
void SE_traits<3>::pseudo_ln(const CPose3D &P, array_t &x)
{
	x[0] = P.m_coords[0];
	x[1] = P.m_coords[1];
	x[2] = P.m_coords[2];
	CArrayDouble<3> ln_rot = P.ln_rotation();
	x[3] = ln_rot[0];
	x[4] = ln_rot[1];
	x[5] = ln_rot[2];
}


/** Return one or both of the following 6x6 Jacobians, useful in graph-slam problems...
  */
void SE_traits<3>::jacobian_dP1DP2inv_depsilon(
	const CPose3D &P1DP2inv,
	matrix_VxV_t *df_de1,
	matrix_VxV_t *df_de2)
{
	const CMatrixDouble33 & R = P1DP2inv.getRotationMatrix(); // The rotation matrix.

	// Common part: d_Ln(R)_dR:
	CMatrixFixedNumeric<double,3,9> dLnRot_dRot(UNINITIALIZED_MATRIX);
	CPose3D::ln_rot_jacob(R, dLnRot_dRot);

	if (df_de1)
	{
		matrix_VxV_t & J1 = *df_de1;
		// This Jacobian has the structure:
		//           [   I_3    |      -[d_t]_x      ]
		//  Jacob1 = [ ---------+------------------- ]
		//           [   0_3x3  |   dLnR_dR * (...)  ]
		//
		J1.zeros();
		J1(0,0) = 1;
		J1(1,1) = 1;
		J1(2,2) = 1;

								  J1(0,4) = P1DP2inv.z();  J1(0,5) = -P1DP2inv.y();
		J1(1,3) = -P1DP2inv.z();                           J1(1,5) =  P1DP2inv.x();
		J1(2,3) =  P1DP2inv.y();  J1(2,4) =-P1DP2inv.x();

		EIGEN_ALIGN16 const double aux_vals[] = {
				  0,  R(2,0), -R(1,0),
			-R(2,0),       0,  R(0,0),
			 R(1,0), -R(0,0),       0,
			 // -----------------------
				  0,  R(2,1), -R(1,1),
			-R(2,1),       0,  R(0,1),
			 R(1,1), -R(0,1),       0,
			 // -----------------------
				  0,  R(2,2), -R(1,2),
			-R(2,2),       0,  R(0,2),
			 R(1,2), -R(0,2),       0
		};
		const CMatrixFixedNumeric<double,9,3> aux(aux_vals);

		// right-bottom part = dLnRot_dRot * aux
		J1.block(3,3,3,3) = (dLnRot_dRot * aux).eval();
		//CSubmatrixView<matrix_VxV_t,3,3>(J1,3,3).multiply_AB(dLnRot_dRot, aux);
	}
	if (df_de2)
	{
		// This Jacobian has the structure:
		//           [    -R    |      0_3x3         ]
		//  Jacob2 = [ ---------+------------------- ]
		//           [   0_3x3  |   dLnR_dR * (...)  ]
		//
		matrix_VxV_t & J2 = *df_de2;
		J2.zeros();

		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
				J2.set_unsafe(i,j, -R.get_unsafe(i,j));

		EIGEN_ALIGN16 const double aux_vals[] = {
				  0,  R(0,2), -R(0,1),
				  0,  R(1,2), -R(1,1),
				  0,  R(2,2), -R(2,1),
			 // -----------------------
			-R(0,2),       0,  R(0,0),
			-R(1,2),       0,  R(1,0),
			-R(2,2),       0,  R(2,0),
			 // -----------------------
			 R(0,1), -R(0,0),       0,
			 R(1,1), -R(1,0),       0,
			 R(2,1), -R(2,0),       0
		};
		const CMatrixFixedNumeric<double,9,3> aux(aux_vals);

		// right-bottom part = dLnRot_dRot * aux
		//CSubmatrixView<matrix_VxV_t,3,3>(J2,3,3).multiply_AB(dLnRot_dRot, aux);
		J2.block(3,3,3,3) = (dLnRot_dRot * aux).eval();
	}
}



/** Return one or both of the following 6x6 Jacobians, useful in graph-slam problems...
  */
void SE_traits<2>::jacobian_dP1DP2inv_depsilon(
	const CPose2D &P1DP2inv,
	matrix_VxV_t *df_de1,
	matrix_VxV_t *df_de2)
{
	if (df_de1)
	{
		matrix_VxV_t & J1 = *df_de1;
		// This Jacobian has the structure:
		//           [   I_2    |  -[d_t]_x      ]
		//  Jacob1 = [ ---------+--------------- ]
		//           [   0      |   1            ]
		//
		J1.unit(VECTOR_SIZE,1.0);
		J1(0,2) = -P1DP2inv.y();
		J1(1,2) =  P1DP2inv.x();
	}
	if (df_de2)
	{
		// This Jacobian has the structure:
		//           [    -R    |    0   ]
		//  Jacob2 = [ ---------+------- ]
		//           [     0    |    -1  ]
		//
		matrix_VxV_t & J2 = *df_de2;

		const double ccos = cos(P1DP2inv.phi());
		const double csin = sin(P1DP2inv.phi());

		const double vals[] =  {
			-ccos, csin, 0,
			-csin,-ccos, 0,
			    0,    0, -1
		};
		J2 = CMatrixFixedNumeric<double,3,3>(vals);
	}
}
