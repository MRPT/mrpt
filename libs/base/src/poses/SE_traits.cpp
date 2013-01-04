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
