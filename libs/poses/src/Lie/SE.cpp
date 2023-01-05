/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"	// Precompiled headers
//
#include <mrpt/math/geometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/Lie/SO.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::poses::Lie;

// See .h for documentation

// ====== SE(3) ===========
SE<3>::type SE<3>::exp(const SE<3>::tangent_vector& x)
{
	return CPose3D(Lie::SO<3>::exp(x.tail<3>()), x.head<3>());
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

SE<3>::manifold_vector SE<3>::asManifoldVector(const SE<3>::type& pose)
{
	manifold_vector v;
	pose.getAs12Vector(v);
	return v;
}

SE<3>::type SE<3>::fromManifoldVector(const SE<3>::manifold_vector& v)
{
	return type(v);
}

// See 10.3.1 in \cite blanco_se3_tutorial
SE<3>::tang2mat_jacob SE<3>::jacob_dexpe_de(const SE<3>::tangent_vector& x)
{
	// 12x6 Jacobian:
	tang2mat_jacob J = tang2mat_jacob::Zero();
	J.block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();
	const auto w = SO<3>::tangent_vector(x.tail<3>());
	J.block<9, 3>(0, 3) = SO<3>::jacob_dexpe_de(w).asEigen();
	return J;
}

SE<3>::mat2tang_jacob SE<3>::jacob_dlogv_dv(const SE<3>::type& P)
{
	mrpt::math::CMatrixDouble6_12 J;
	J.setZero();
	const CMatrixDouble33& R = P.getRotationMatrix();
	J.block<3, 9>(3, 0) = SO<3>::jacob_dlogv_dv(R).asEigen();
	J(0, 9) = J(1, 10) = J(2, 11) = 1.0;
	return J;
}

// Section 10.3.3 in tech report
// http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
SE<3>::tang2mat_jacob SE<3>::jacob_dexpeD_de(const CPose3D& D)
{
	mrpt::math::CMatrixDouble12_6 jacob;
	jacob.block<9, 3>(0, 0).setZero();
	jacob.block<3, 3>(9, 0).setIdentity();
	for (int i = 0; i < 3; i++)
	{
		auto trg_blc = jacob.block<3, 3>(3 * i, 3);
		mrpt::math::skew_symmetric3_neg(
			D.getRotationMatrix().blockCopy<3, 1>(0, i), trg_blc);
	}
	{
		auto trg_blc = jacob.block<3, 3>(9, 3);
		mrpt::math::skew_symmetric3_neg(D.m_coords, trg_blc);
	}
	return jacob;
}

// Section 10.3.4 in tech report
// http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
SE<3>::tang2mat_jacob SE<3>::jacob_dDexpe_de(const CPose3D& D)
{
	mrpt::math::CMatrixDouble12_6 jacob;
	const auto& dRot = D.getRotationMatrix();
	jacob.setZero();
	jacob.block<3, 3>(9, 0) = dRot.asEigen();

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
SE<3>::tang2mat_jacob SE<3>::jacob_dAexpeD_de(
	const CPose3D& A, const CPose3D& D)
{
	const auto& Arot = A.getRotationMatrix();

	mrpt::math::CMatrixDouble12_6 jacob;
	jacob.block<9, 3>(0, 0).setZero();
	jacob.block<3, 3>(9, 0) = A.getRotationMatrix().asEigen();
	Eigen::Matrix<double, 3, 3> aux;
	for (int i = 0; i < 3; i++)
	{
		mrpt::math::skew_symmetric3_neg(
			D.getRotationMatrix().blockCopy<3, 1>(0, i), aux);
		jacob.block<3, 3>(3 * i, 3) = Arot.asEigen() * aux;
	}
	mrpt::math::skew_symmetric3_neg(D.m_coords, aux);
	jacob.block<3, 3>(9, 3) = Arot.asEigen() * aux;
	return jacob;
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

		const CMatrixFixed<double, 12, 12> J1a =
			SE<3>::jacob_dAB_dA(Dinv, P1inv + P2);
		const auto J1b = CMatrixDouble12_6(-SE<3>::jacob_dDexpe_de(Dinv));

		J1 = dLnT_dT.asEigen() * J1a.asEigen() * J1b.asEigen();
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
			for (int q = 0; q < 3; q++)
				J(r * 3 + q, c * 3 + q) = B_HM(r, c);

	return J;
}

SE<3>::matrix_MxM SE<3>::jacob_dAB_dB(
	const SE<3>::type& A, const SE<3>::type& B)
{
	matrix_MxM J = matrix_MxM::Zero();
	// J_wrt_B = kron(eye(3),A_rot);
	const auto& AR = A.getRotationMatrix();
	for (int c = 0; c < 4; c++)
		J.block<3, 3>(c * 3, c * 3) = AR.asEigen();
	return J;
}

// See .h for documentation
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
	x[2] = mrpt::math::wrapToPi(P.phi());
	return x;
}

SE<2>::manifold_vector SE<2>::asManifoldVector(const SE<2>::type& pose)
{
	manifold_vector v;
	v[0] = pose.x();
	v[1] = pose.y();
	v[2] = mrpt::math::wrapToPi(pose.phi());
	return v;
}

SE<2>::type SE<2>::fromManifoldVector(const SE<2>::manifold_vector& v)
{
	return type(v[0], v[1], mrpt::math::wrapToPi(v[2]));
}

SE<2>::matrix_MxM SE<2>::jacob_dAB_dA(
	const SE<2>::type& A, const SE<2>::type& B)
{
	const auto bx = B.x(), by = B.y();
	const auto cphia = A.phi_cos(), sphia = A.phi_sin();

	matrix_MxM J = matrix_MxM::Identity();
	J(0, 2) = -bx * sphia - by * cphia;
	J(1, 2) = +bx * cphia - by * sphia;
	return J;
}

SE<2>::matrix_MxM SE<2>::jacob_dAB_dB(
	const SE<2>::type& A, const SE<2>::type& B)
{
	matrix_MxM J = matrix_MxM::Identity();
	const auto cphia = A.phi_cos(), sphia = A.phi_sin();
	J(0, 0) = cphia;
	J(0, 1) = -sphia;
	J(1, 0) = sphia;
	J(1, 1) = cphia;
	return J;
}

SE<2>::tang2mat_jacob SE<2>::jacob_dDexpe_de(const SE<2>::type& D)
{
	const auto c = D.phi_cos(), s = D.phi_sin();

	// clang-format off
	return SE<2>::tang2mat_jacob((Eigen::Matrix3d() <<
	        c, -s, 0,
	        s,  c, 0,
	        0,  0, 1
	        ).finished());
	// clang-format on
}

void SE<2>::jacob_dDinvP1invP2_de1e2(
	const CPose2D& Dinv, const CPose2D& P1, const CPose2D& P2,
	mrpt::optional_ref<matrix_TxT> df_de1,
	mrpt::optional_ref<matrix_TxT> df_de2)
{
	const CPose2D P1inv = -P1;
	const CPose2D P1invP2 = P1inv + P2;
	const CPose2D DinvP1invP2 = Dinv + P1invP2;

	if (df_de1)
	{
		auto& J1 = df_de1.value().get();
		J1 = jacob_dAB_dA(Dinv, P1invP2).asEigen() * (-jacob_dDexpe_de(Dinv));
	}

	if (df_de2)
	{
		auto& J2 = df_de2.value().get();
		J2 = SE<2>::jacob_dDexpe_de(DinvP1invP2);
	}
}
