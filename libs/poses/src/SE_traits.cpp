/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/SE_traits.h>

using namespace mrpt;
using namespace mrpt::math;

using namespace mrpt::poses;

/** A pseudo-Logarithm map in SE(3), where the output = [X,Y,Z, Ln(ROT)], that
 * is, the normal
 *  SO(3) logarithm is used for the rotation components, but the translation is
 * left unmodified.
 */
void SE_traits<3>::pseudo_ln(const CPose3D& P, array_t& x)
{
	x[0] = P.m_coords[0];
	x[1] = P.m_coords[1];
	x[2] = P.m_coords[2];
	CArrayDouble<3> ln_rot = P.ln_rotation();
	x[3] = ln_rot[0];
	x[4] = ln_rot[1];
	x[5] = ln_rot[2];
}

/** Return one or both of the following 6x6 Jacobians, useful in graph-slam
 * problems...
 */
void SE_traits<3>::jacobian_dP1DP2inv_depsilon(
	const CPose3D& P1DP2inv, matrix_VxV_t* df_de1, matrix_VxV_t* df_de2)
{
	const CMatrixDouble33& R =
		P1DP2inv.getRotationMatrix();  // The rotation matrix.

	// Common part: d_Ln(R)_dR:
	CMatrixFixedNumeric<double, 3, 9> dLnRot_dRot = CPose3D::ln_rot_jacob(R);

	if (df_de1)
	{
		matrix_VxV_t& J1 = *df_de1;
		// This Jacobian has the structure:
		//           [   I_3    |      -[d_t]_x      ]
		//  Jacob1 = [ ---------+------------------- ]
		//           [   0_3x3  |   dLnR_dR * (...)  ]
		//
		J1.zeros();
		J1(0, 0) = 1;
		J1(1, 1) = 1;
		J1(2, 2) = 1;

		J1(0, 4) = P1DP2inv.z();
		J1(0, 5) = -P1DP2inv.y();
		J1(1, 3) = -P1DP2inv.z();
		J1(1, 5) = P1DP2inv.x();
		J1(2, 3) = P1DP2inv.y();
		J1(2, 4) = -P1DP2inv.x();

		alignas(MRPT_MAX_ALIGN_BYTES) const double aux_vals[] = {
			0, R(2, 0), -R(1, 0), -R(2, 0), 0, R(0, 0), R(1, 0), -R(0, 0), 0,
			// -----------------------
			0, R(2, 1), -R(1, 1), -R(2, 1), 0, R(0, 1), R(1, 1), -R(0, 1), 0,
			// -----------------------
			0, R(2, 2), -R(1, 2), -R(2, 2), 0, R(0, 2), R(1, 2), -R(0, 2), 0};
		const CMatrixFixedNumeric<double, 9, 3> aux(aux_vals);

		// right-bottom part = dLnRot_dRot * aux
		J1.block(3, 3, 3, 3) = (dLnRot_dRot * aux).eval();
	}
	if (df_de2)
	{
		// This Jacobian has the structure:
		//           [    -R    |      0_3x3         ]
		//  Jacob2 = [ ---------+------------------- ]
		//           [   0_3x3  |   dLnR_dR * (...)  ]
		//
		matrix_VxV_t& J2 = *df_de2;
		J2.zeros();

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				J2.set_unsafe(i, j, -R.get_unsafe(i, j));

		alignas(MRPT_MAX_ALIGN_BYTES) const double aux_vals[] = {
			0, R(0, 2), -R(0, 1), 0, R(1, 2), -R(1, 1), 0, R(2, 2), -R(2, 1),
			// -----------------------
			-R(0, 2), 0, R(0, 0), -R(1, 2), 0, R(1, 0), -R(2, 2), 0, R(2, 0),
			// -----------------------
			R(0, 1), -R(0, 0), 0, R(1, 1), -R(1, 0), 0, R(2, 1), -R(2, 0), 0};
		const CMatrixFixedNumeric<double, 9, 3> aux(aux_vals);

		// right-bottom part = dLnRot_dRot * aux
		J2.block(3, 3, 3, 3) = (dLnRot_dRot * aux).eval();
	}
}

void SE_traits<3>::jacobian_dDinvP1invP2_depsilon(
	const CPose3D& Dinv, const CPose3D& P1, const CPose3D& P2,
    mrpt::optional_ref<matrix_VxV_t> df_de1,
    mrpt::optional_ref<matrix_VxV_t> df_de2)
{
	// The rotation matrix of the overall error expression:
	const CPose3D DinvP1invP2 = Dinv + (-P1) + P2;

	// Common part: d_PseudoLn(T)_dT:
	const auto dLnT_dT = SE_traits<3>::jacob_pseudo_ln(DinvP1invP2);

	// See section 10.3.10 of Tech. report:
	// "A tutorial on SE(3) transformation parameterizations and on-manifold
	// optimization"
	if (df_de1)
	{
		matrix_VxV_t& J1 = df_de1.value().get();

		const auto dP1e_de = CPose3D::jacob_dDexpe_de(P1);

		J1 = dLnT_dT * dP1e_de;
	}
	if (df_de2)
	{
		matrix_VxV_t& J2 = df_de2.value().get();
		const auto dAe_de = CPose3D::jacob_dDexpe_de(DinvP1invP2);
		J2 = dLnT_dT * dAe_de;
	}
}

/** Logarithm for pseudo_ln(). See section 10.3.11 of tech. report.
 * http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
 */
mrpt::math::CMatrixDouble6_12 SE_traits<3>::jacob_pseudo_ln(const CPose3D& P)
{
	mrpt::math::CMatrixDouble6_12 J;
	J.setZero();
	const CMatrixDouble33& R = P.getRotationMatrix();
	J.block<3, 9>(0, 0) = CPose3D::ln_rot_jacob(R);
	J(3, 9) = J(4, 10) = J(5, 11) = 1.0;
	return J;
}

void SE_traits<2>::jacobian_dP1DP2inv_depsilon(
	const CPose2D& P1DP2inv, matrix_VxV_t* df_de1, matrix_VxV_t* df_de2)
{
	if (df_de1)
	{
		matrix_VxV_t& J1 = *df_de1;
		// This Jacobian has the structure:
		//           [   I_2    |  -[d_t]_x      ]
		//  Jacob1 = [ ---------+--------------- ]
		//           [   0      |   1            ]
		//
		J1.unit(VECTOR_SIZE, 1.0);
		J1(0, 2) = -P1DP2inv.y();
		J1(1, 2) = P1DP2inv.x();
	}
	if (df_de2)
	{
		// This Jacobian has the structure:
		//           [    -R    |    0   ]
		//  Jacob2 = [ ---------+------- ]
		//           [     0    |    -1  ]
		//
		matrix_VxV_t& J2 = *df_de2;

		const double ccos = cos(P1DP2inv.phi());
		const double csin = sin(P1DP2inv.phi());

		const double vals[] = {-ccos, csin, 0, -csin, -ccos, 0, 0, 0, -1};
		J2 = CMatrixFixedNumeric<double, 3, 3>(vals);
	}
}

void SE_traits<2>::jacobian_dDinvP1invP2_depsilon(
	const CPose2D& Dinv, const CPose2D& P1, const CPose2D& P2,
    mrpt::optional_ref<matrix_VxV_t> df_de1,
    mrpt::optional_ref<matrix_VxV_t> df_de2)

{
	using mrpt::math::TPoint2D;
	const double phi1 = P1.phi();

	const TPoint2D dt(P2.x() - P1.x(), P2.y() - P1.y());
	const double si = std::sin(phi1), ci = std::cos(phi1);

	CMatrixDouble22 RotDinv;
	Dinv.getRotationMatrix(RotDinv);
	CMatrixDouble33 K;  // zeros
	K.block<2, 2>(0, 0) = RotDinv;
	K(2, 2) = 1.0;

	if (df_de1)
	{
		auto& J1 = df_de1.value().get();
		J1(0, 0) = -ci;
		J1(0, 1) = -si;
		J1(0, 2) = -si * dt.x + ci * dt.y;
		J1(1, 0) = si;
		J1(1, 1) = -ci;
		J1(1, 2) = -ci * dt.x - si * dt.y;
		J1(2, 0) = 0;
		J1(2, 1) = 0;
		J1(2, 2) = -1;
		J1 = K * J1;
	}

	if (df_de2)
	{
		auto& J2 = df_de2.value().get();
		J2(0, 0) = ci;
		J2(0, 1) = si;
		J2(0, 2) = 0;
		J2(1, 0) = -si;
		J2(1, 1) = ci;
		J2(1, 2) = 0;
		J2(2, 0) = 0;
		J2(2, 1) = 0;
		J2(2, 2) = 1;
		J2 = K * J2;
	}
}
