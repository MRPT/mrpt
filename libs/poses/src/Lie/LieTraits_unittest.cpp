/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/math/num_jacobian.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

// List of "random" poses to test with (x,y,z,yaw,pitch,roll) (angles in
// degrees)
static const std::vector<mrpt::poses::CPose3D> ptc = {
	{.0, .0, .0, DEG2RAD(.0), DEG2RAD(.0), DEG2RAD(.0)},
	{1.0, 2.0, 3.0, DEG2RAD(.0), DEG2RAD(.0), DEG2RAD(.0)},
	{1.0, 2.0, 3.0, DEG2RAD(10.0), DEG2RAD(.0), DEG2RAD(.0)},
	{1.0, 2.0, 3.0, DEG2RAD(.0), DEG2RAD(1.0), DEG2RAD(.0)},
	{1.0, 2.0, 3.0, DEG2RAD(.0), DEG2RAD(.0), DEG2RAD(1.0)},
	{-6.0, 2.0, 3.0, DEG2RAD(40.0), DEG2RAD(20.0), DEG2RAD(15.0)},
	{1.0, 8.0, 0.0, DEG2RAD(-45.0), DEG2RAD(10.0), DEG2RAD(70.0)}};

template <class POSE_TYPE>
class SE_traits_tests : public ::testing::Test
{
   protected:
	void SetUp() override {}
	void TearDown() override {}
	using SE_TYPE = mrpt::poses::Lie::SE<POSE_TYPE::rotation_dimensions>;

	struct TParams
	{
		typename SE_TYPE::type P1, D, P2;
	};

	template <unsigned int MAT_LEN>
	struct TParamsMat
	{
		CArrayDouble<MAT_LEN> Avec, Bvec;
	};

	static void func_numeric_P1DP2inv(
		const CArrayDouble<2 * SE_TYPE::DOFs>& x, const TParams& params,
		CArrayDouble<SE_TYPE::DOFs>& Y)
	{
		typename SE_TYPE::tangent_vector eps1, eps2;
		for (size_t i = 0; i < SE_TYPE::DOFs; i++)
		{
			eps1[i] = x[0 + i];
			eps2[i] = x[SE_TYPE::DOFs + i];
		}

		const POSE_TYPE incr1 = SE_TYPE::exp(eps1);
		const POSE_TYPE incr2 = SE_TYPE::exp(eps2);

		const POSE_TYPE P1 = incr1 + params.P1;
		const POSE_TYPE P2 = incr2 + params.P2;
		const POSE_TYPE& Pd = params.D;

		CMatrixDouble44 HM_P2inv, P1hm, Pdhm;
		P1.getHomogeneousMatrix(P1hm);
		Pd.getHomogeneousMatrix(Pdhm);
		P2.getInverseHomogeneousMatrix(HM_P2inv);

		const CPose3D P1DP2inv_(CMatrixDouble44(P1hm * Pdhm * HM_P2inv));
		const POSE_TYPE P1DP2inv(P1DP2inv_);

		// Pseudo-logarithm:
		Y = SE_TYPE::log(P1DP2inv);
	}

	void test_jacobs_P1DP2inv(
		const CPose3D& P1_, const CPose3D& Pd_, const CPose3D& P2_)
	{
		const POSE_TYPE P1(P1_);
		const POSE_TYPE Pd(Pd_);
		const POSE_TYPE P2(P2_);

		CMatrixDouble44 HM_P2inv, P1hm, Pdhm;
		P1.getHomogeneousMatrix(P1hm);
		Pd.getHomogeneousMatrix(Pdhm);
		P2.getInverseHomogeneousMatrix(HM_P2inv);
		const CPose3D P1DP2inv_(CMatrixDouble44((P1hm * Pdhm) * HM_P2inv));
		const POSE_TYPE P1DP2inv(P1DP2inv_);  // Convert to 2D if needed

		static const int DIMS = SE_TYPE::DOFs;

		// Theoretical results:
		CMatrixFixedNumeric<double, DIMS, DIMS> J1, J2;
		SE_TYPE::jacob_dP1DP2inv_de1e2(P1DP2inv, J1, J2);

		// Numerical approx:
		CMatrixFixedNumeric<double, DIMS, DIMS> num_J1, num_J2;
		{
			CArrayDouble<2 * DIMS> x_mean;
			for (int i = 0; i < DIMS + DIMS; i++) x_mean[i] = 0;

			TParams params;
			params.P1 = P1;
			params.D = Pd;
			params.P2 = P2;

			CArrayDouble<DIMS + DIMS> x_incrs;
			x_incrs.assign(1e-4);
			CMatrixDouble numJacobs;
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<2 * SE_TYPE::DOFs>& x,
					const TParams& params, CArrayDouble<SE_TYPE::DOFs>& Y)>(
					&func_numeric_P1DP2inv),
				x_incrs, params, numJacobs);

			numJacobs.extractMatrix(0, 0, num_J1);
			numJacobs.extractMatrix(0, DIMS, num_J2);
		}

		const double max_eror = 1e-3;

		EXPECT_NEAR(0, (num_J1 - J1).array().abs().sum(), max_eror)
			<< "p1: " << P1 << endl
			<< "d: " << Pd << endl
			<< "p2: " << P2 << endl
			<< "Numeric J1:\n"
			<< num_J1 << endl
			<< "Implemented J1:\n"
			<< J1 << endl
			<< "Error:\n"
			<< J1 - num_J1 << endl;

		EXPECT_NEAR(0, (num_J2 - J2).array().abs().sum(), max_eror)
			<< "p1: " << P1 << endl
			<< "d: " << Pd << endl
			<< "p2: " << P2 << endl
			<< "Numeric J2:\n"
			<< num_J2 << endl
			<< "Implemented J2:\n"
			<< J2 << endl
			<< "Error:\n"
			<< J2 - num_J2 << endl;
	}

	static void func_numeric_DinvP1InvP2(
		const CArrayDouble<2 * SE_TYPE::DOFs>& x, const TParams& params,
		CArrayDouble<SE_TYPE::DOFs>& Y)
	{
		typename SE_TYPE::tangent_vector eps1, eps2;
		for (size_t i = 0; i < SE_TYPE::DOFs; i++)
		{
			eps1[i] = x[0 + i];
			eps2[i] = x[SE_TYPE::DOFs + i];
		}

		const POSE_TYPE incr1 = SE_TYPE::exp(eps1);
		const POSE_TYPE incr2 = SE_TYPE::exp(eps2);

		const POSE_TYPE P1 = params.P1 + incr1;
		const POSE_TYPE P2 = params.P2 + incr2;
		const POSE_TYPE& Pd = params.D;

		CMatrixDouble44 P1_inv_hm, P2hm, Pd_inv_hm;
		P1.getInverseHomogeneousMatrix(P1_inv_hm);
		P2.getHomogeneousMatrix(P2hm);
		Pd.getInverseHomogeneousMatrix(Pd_inv_hm);

		const CPose3D DinvP1invP2_(
			CMatrixDouble44(Pd_inv_hm * P1_inv_hm * P2hm));
		const POSE_TYPE DinvP1invP2(DinvP1invP2_);

		// Pseudo-logarithm:
		Y = SE_TYPE::log(DinvP1invP2);
	}

	void test_jacobs_DinvP1InvP2(
		const CPose3D& P1_, const CPose3D& Pd_, const CPose3D& P2_)
	{
		const POSE_TYPE P1(P1_);
		const POSE_TYPE Pd(Pd_);
		const POSE_TYPE P2(P2_);
		const POSE_TYPE Pdinv = -Pd;

		static const int DIMS = SE_TYPE::DOFs;

		// Theoretical results:
		CMatrixFixedNumeric<double, DIMS, DIMS> J1, J2;
		SE_TYPE::jacob_dDinvP1invP2_de1e2(Pdinv, P1, P2, J1, J2);

		// Numerical approx:
		CMatrixFixedNumeric<double, DIMS, DIMS> num_J1, num_J2;
		{
			CArrayDouble<2 * DIMS> x_mean;
			for (int i = 0; i < DIMS + DIMS; i++) x_mean[i] = 0;

			TParams params;
			params.P1 = P1;
			params.D = Pd;
			params.P2 = P2;

			CArrayDouble<DIMS + DIMS> x_incrs;
			x_incrs.assign(1e-4);
			CMatrixDouble numJacobs;
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<2 * SE_TYPE::DOFs>& x,
					const TParams& params, CArrayDouble<SE_TYPE::DOFs>& Y)>(
					&func_numeric_DinvP1InvP2),
				x_incrs, params, numJacobs);

			numJacobs.extractMatrix(0, 0, num_J1);
			numJacobs.extractMatrix(0, DIMS, num_J2);
		}

		const double max_eror = 1e-3;

		EXPECT_NEAR(0, (num_J1 - J1).array().abs().sum(), max_eror)
			<< std::setprecision(3) << "p1: " << P1 << endl
			<< "d: " << Pd << endl
			<< "p2: " << P2 << endl
			<< "Numeric J1:\n"
			<< num_J1 << endl
			<< "Implemented J1:\n"
			<< J1 << endl
			<< "Error:\n"
			<< J1 - num_J1 << endl;

		EXPECT_NEAR(0, (num_J2 - J2).array().abs().sum(), max_eror)
			<< "p1: " << P1 << endl
			<< "d: " << Pd << endl
			<< "p2: " << P2 << endl
			<< "Numeric J2:\n"
			<< num_J2 << endl
			<< "Implemented J2:\n"
			<< J2 << endl
			<< "Error:\n"
			<< J2 - num_J2 << endl;
	}

	static void func_numeric_dAB_dA(
		const CArrayDouble<SE_TYPE::MANIFOLD_DIM>& x,
		const TParamsMat<SE_TYPE::MANIFOLD_DIM>& params,
		CArrayDouble<SE_TYPE::MANIFOLD_DIM>& Y)
	{
		const POSE_TYPE A = SE_TYPE::fromManifoldVector(params.Avec + x);
		const POSE_TYPE B = SE_TYPE::fromManifoldVector(params.Bvec);
		Y = SE_TYPE::asManifoldVector(A + B);
	}

	static void func_numeric_dAB_dB(
		const CArrayDouble<SE_TYPE::MANIFOLD_DIM>& x,
		const TParamsMat<SE_TYPE::MANIFOLD_DIM>& params,
		CArrayDouble<SE_TYPE::MANIFOLD_DIM>& Y)
	{
		const POSE_TYPE A = SE_TYPE::fromManifoldVector(params.Avec);
		const POSE_TYPE B = SE_TYPE::fromManifoldVector(params.Bvec + x);
		Y = SE_TYPE::asManifoldVector(A + B);
	}

	void test_jacobs_AB(const CPose3D& A_, const CPose3D& B_)
	{
		const POSE_TYPE A(A_);
		const POSE_TYPE B(B_);

		constexpr auto N = SE_TYPE::MANIFOLD_DIM;

		// Theoretical results:
		const auto dAB_A = SE_TYPE::jacob_dAB_dA(A, B);
		const auto dAB_B = SE_TYPE::jacob_dAB_dB(A, B);

		// Numerical approx: dAB_A
		CMatrixFixedNumeric<double, N, N> num_dAB_A;
		{
			CArrayDouble<N> x_mean;
			x_mean.assign(0);

			TParamsMat<N> params;
			params.Avec = SE_TYPE::asManifoldVector(A);
			params.Bvec = SE_TYPE::asManifoldVector(B);

			CArrayDouble<N> x_incrs;
			x_incrs.assign(1e-4);
			CMatrixDouble numJacobs;
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<N>& x, const TParamsMat<N>& params,
					CArrayDouble<N>& Y)>(&func_numeric_dAB_dA),
				x_incrs, params, numJacobs);

			num_dAB_A = numJacobs;
		}

		// Numerical approx: dAB_B
		CMatrixFixedNumeric<double, N, N> num_dAB_B;
		{
			CArrayDouble<N> x_mean;
			x_mean.assign(0);

			TParamsMat<N> params;
			params.Avec = SE_TYPE::asManifoldVector(A);
			params.Bvec = SE_TYPE::asManifoldVector(B);

			CArrayDouble<N> x_incrs;
			x_incrs.assign(1e-4);
			CMatrixDouble numJacobs;
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<N>& x, const TParamsMat<N>& params,
					CArrayDouble<N>& Y)>(&func_numeric_dAB_dB),
				x_incrs, params, numJacobs);

			num_dAB_B = numJacobs;
		}

		const double max_eror = 1e-3;

		EXPECT_NEAR(0, (num_dAB_A - dAB_A).array().abs().sum(), max_eror)
			<< std::setprecision(3) << "A: " << A << endl
			<< "B: " << B << endl
			<< "Numeric dAB_A:\n"
			<< num_dAB_A << endl
			<< "Implemented dAB_A:\n"
			<< dAB_A << endl
			<< "Error:\n"
			<< dAB_A - num_dAB_A << endl;

		EXPECT_NEAR(0, (num_dAB_B - dAB_B).array().abs().sum(), max_eror)
			<< std::setprecision(3) << "A: " << A << endl
			<< "B: " << B << endl
			<< "Numeric dAB_B:\n"
			<< num_dAB_B << endl
			<< "Implemented dAB_B:\n"
			<< dAB_B << endl
			<< "Error:\n"
			<< dAB_B - num_dAB_B << endl;
	}

	void tests_jacobs_P1DP2inv()
	{
		for (const auto& p1 : ptc)
			for (const auto& p2 : ptc)
				for (const auto& pd : ptc) test_jacobs_P1DP2inv(p1, pd, p2);
	}

	void tests_jacobs_DinvP1InvP2()
	{
		for (const auto& p1 : ptc)
			for (const auto& p2 : ptc)
				for (const auto& pd : ptc) test_jacobs_DinvP1InvP2(p1, pd, p2);
	}

	void tests_jacobs_dAB_dAB()
	{
		for (const auto& p1 : ptc)
			for (const auto& p2 : ptc) test_jacobs_AB(p1, p2);
	}
};

using SE3_traits_tests = SE_traits_tests<mrpt::poses::CPose3D>;
using SE2_traits_tests = SE_traits_tests<mrpt::poses::CPose2D>;

TEST_F(SE3_traits_tests, SE3_jacobs_P1DP2inv) { tests_jacobs_P1DP2inv(); }
TEST_F(SE3_traits_tests, SE3_jacobs_DinvP1InvP2) { tests_jacobs_DinvP1InvP2(); }
TEST_F(SE3_traits_tests, SE3_jacobs_dAB_dAB) { tests_jacobs_dAB_dAB(); }

TEST_F(SE2_traits_tests, SE2_jacobs_P1DP2inv) { tests_jacobs_P1DP2inv(); }
TEST_F(SE2_traits_tests, SE2_jacobs_DinvP1InvP2) { tests_jacobs_DinvP1InvP2(); }
TEST_F(SE2_traits_tests, SE2_jacobs_dAB_dAB) { tests_jacobs_dAB_dAB(); }
