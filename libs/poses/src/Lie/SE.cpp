/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"  // Precompiled headers

#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/otherlibs/sophus/so3.hpp>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::poses::Lie;

// See .h for documentation

// ====== SE(3) ===========
SE<3>::type SE<3>::exp(const SE<3>::tangent_vector& x)
{
	CPose3D p(UNINITIALIZED_POSE);
	auto R = Sophus::SO3<double>::exp(x.tail<3>());
	p.setRotationMatrix(R.matrix());
	p.x(x[0]);
	p.y(x[1]);
	p.z(x[2]);
	return p;
}

SE<3>::tangent_vector SE<3>::log(const SE<3>::type& P)
{
	tangent_vector v;
	const auto log_R = mrpt::poses::Lie::SO<3>::log(P.getRotationMatrix());
	v[0] = P.x();
	v[1] = P.y();
	v[2] = P.z();
	v[3] = log_R[0];
	v[4] = log_R[1];
	v[5] = log_R[2];
	return v;
}

// See 10.3.1 in \cite blanco_se3_tutorial
SE<3>::tang2mat_jacob SE<3>::jacob_dexpe_de(const SE<3>::tangent_vector& x)
{
	// 12x6 Jacobian:
	tang2mat_jacob J = tang2mat_jacob::Zero();
	J.block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();
	const auto w = SO<3>::tangent_vector(x.tail<3>());
	J.block<9, 3>(0, 3) = SO<3>::jacob_dexpe_de(w);
	return J;
}

SE<3>::mat2tang_jacob SE<3>::jacob_dlogv_dv(const SE<3>::type& P)
{
	mrpt::math::CMatrixDouble6_12 J;
	J.setZero();
	const CMatrixDouble33& R = P.getRotationMatrix();
	J.block<3, 9>(3, 0) = SO<3>::jacob_dlogv_dv(R);
	J(0, 9) = J(1, 10) = J(2, 11) = 1.0;
	return J;
}

// Section 10.3.3 in tech report
// http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
mrpt::math::CMatrixDouble12_6 SE<3>::jacob_dexpeD_de(const CPose3D& D)
{
	mrpt::math::CMatrixDouble12_6 jacob;
	jacob.block<9, 3>(0, 0).setZero();
	jacob.block<3, 3>(9, 0).setIdentity();
	for (int i = 0; i < 3; i++)
	{
		auto trg_blc = jacob.block<3, 3>(3 * i, 3);
		mrpt::math::skew_symmetric3_neg(
			D.getRotationMatrix().block<3, 1>(0, i), trg_blc);
	}
	{
		auto trg_blc = jacob.block<3, 3>(9, 3);
		mrpt::math::skew_symmetric3_neg(D.m_coords, trg_blc);
	}
	return jacob;
}

// Section 10.3.4 in tech report
// http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
mrpt::math::CMatrixDouble12_6 SE<3>::jacob_dDexpe_de(const CPose3D& D)
{
	mrpt::math::CMatrixDouble12_6 jacob;
	const auto& dRot = D.getRotationMatrix();
	jacob.setZero();
	jacob.block<3, 3>(9, 0) = dRot;

	jacob.block<3, 1>(3, 5) = -dRot.col(0);
	jacob.block<3, 1>(6, 4) = dRot.col(0);

	jacob.block<3, 1>(0, 5) = dRot.col(1);
	jacob.block<3, 1>(6, 3) = -dRot.col(1);

	jacob.block<3, 1>(0, 4) = -dRot.col(2);
	jacob.block<3, 1>(3, 3) = dRot.col(2);
	return jacob;
}

// Eq. 10.3.7 in tech report
// http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
mrpt::math::CMatrixDouble12_6 SE<3>::jacob_dAexpeD_de(
	const CPose3D& A, const CPose3D& D)
{
	const auto& Arot = A.getRotationMatrix();

	mrpt::math::CMatrixDouble12_6 jacob;
	jacob.block<9, 3>(0, 0).setZero();
	jacob.block<3, 3>(9, 0) = A.getRotationMatrix();
	Eigen::Matrix<double, 3, 3> aux;
	for (int i = 0; i < 3; i++)
	{
		mrpt::math::skew_symmetric3_neg(
			D.getRotationMatrix().block<3, 1>(0, i), aux);
		jacob.block<3, 3>(3 * i, 3) = Arot * aux;
	}
	mrpt::math::skew_symmetric3_neg(D.m_coords, aux);
	jacob.block<3, 3>(9, 3) = Arot * aux;
	return jacob;
}

void SE<3>::jacob_dP1DP2inv_de1e2(
	const CPose3D& P1DP2inv, mrpt::optional_ref<matrix_TxT> df_de1,
	mrpt::optional_ref<matrix_TxT> df_de2)
{
	const CMatrixDouble33& R =
		P1DP2inv.getRotationMatrix();  // The rotation matrix.

	// Common part: d_Ln(R)_dR:
	const CMatrixDouble39 dLnRot_dRot = SO<3>::jacob_dlogv_dv(R);

	if (df_de1)
	{
		matrix_TxT& J1 = df_de1.value().get();
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
		matrix_TxT& J2 = df_de2.value().get();
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

void SE<3>::jacob_dDinvP1invP2_de1e2(
	const CPose3D& Dinv, const CPose3D& P1, const CPose3D& P2,
	mrpt::optional_ref<matrix_TxT> df_de1,
	mrpt::optional_ref<matrix_TxT> df_de2)
{
	using namespace mrpt::math;

	// The rotation matrix of the overall error expression:
	const CPose3D P1inv = -P1;
	const CPose3D DinvP1invP2 = Dinv + P1inv + P2;

	// Common part: d_PseudoLn(T)_dT:
	// (6x12 matrix)
	const auto dLnT_dT = SE<3>::jacob_dlogv_dv(DinvP1invP2);

	// See section 10.3.10 of Tech. report:
	// "A tutorial on SE(3) transformation parameterizations and on-manifold
	// optimization"
	if (df_de1)
	{
		matrix_TxT& J1 = df_de1.value().get();

		const CMatrixFixedNumeric<double, 12, 12> J1a =
			SE<3>::jacob_dAB_dA(Dinv, P1inv + P2);
		const CMatrixDouble12_6 J1b = -SE<3>::jacob_dDexpe_de(Dinv);

		J1 = dLnT_dT * J1a * J1b;
	}
	if (df_de2)
	{
		matrix_TxT& J2 = df_de2.value().get();
		const auto dAe_de = SE<3>::jacob_dDexpe_de(DinvP1invP2);
		J2 = dLnT_dT * dAe_de;
	}
}

SE<3>::matrix_MxM SE<3>::jacob_dAB_dA(
	const SE<3>::type& A, const SE<3>::type& B)
{
	using namespace mrpt::math;

	matrix_MxM J = matrix_MxM::Zero();
	// J_wrt_A = kron(B',eye(3));
	const auto B_HM =
		B.getHomogeneousMatrixVal<CMatrixDouble44>().transpose().eval();
	for (int c = 0; c < 4; c++)
		for (int r = 0; r < 4; r++)
			for (int q = 0; q < 3; q++) J(c * 3 + q, r * 3 + q) = B_HM(r, c);

	return J;
}

SE<3>::matrix_MxM SE<3>::jacob_dAB_dB(
	const SE<3>::type& A, const SE<3>::type& B)
{
	matrix_MxM J = matrix_MxM::Zero();

	return J;
}

// ====== SE(2) ===========
SE<2>::type SE<2>::exp(const SE<2>::tangent_vector& x)
{
	SE<2>::type P;
	P.x(x[0]);
	P.y(x[1]);
	P.phi(x[2]);
	return P;
}

SE<2>::tangent_vector SE<2>::log(const SE<2>::type& P)
{
	SE<2>::tangent_vector x;
	x[0] = P.x();
	x[1] = P.y();
	x[2] = P.phi();
	return x;
}

void SE<2>::jacob_dP1DP2inv_de1e2(
	const CPose2D& P1DP2inv, mrpt::optional_ref<matrix_TxT> df_de1,
	mrpt::optional_ref<matrix_TxT> df_de2)
{
	if (df_de1)
	{
		matrix_TxT& J1 = df_de1.value().get();
		// This Jacobian has the structure:
		//           [   I_2    |  -[d_t]_x      ]
		//  Jacob1 = [ ---------+--------------- ]
		//           [   0      |   1            ]
		//
		J1.unit(DOFs, 1.0);
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
		matrix_TxT& J2 = df_de2.value().get();

		const double ccos = cos(P1DP2inv.phi());
		const double csin = sin(P1DP2inv.phi());

		const double vals[] = {-ccos, csin, 0, -csin, -ccos, 0, 0, 0, -1};
		J2 = CMatrixFixedNumeric<double, 3, 3>(vals);
	}
}

void SE<2>::jacob_dDinvP1invP2_de1e2(
	const CPose2D& Dinv, const CPose2D& P1, const CPose2D& P2,
	mrpt::optional_ref<matrix_TxT> df_de1,
	mrpt::optional_ref<matrix_TxT> df_de2)
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
