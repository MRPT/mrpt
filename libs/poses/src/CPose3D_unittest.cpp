/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/math/num_jacobian.h>
#include <CTraitsTest.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

template class mrpt::CTraitsTest<CPose3D>;

class Pose3DTests : public ::testing::Test
{
   protected:
	void SetUp() override {}
	void TearDown() override {}
	void test_inverse(const CPose3D& p1)
	{
		const auto HM = p1.getHomogeneousMatrixVal<CMatrixDouble44>();
		CMatrixDouble44 HMi;
		p1.getInverseHomogeneousMatrix(HMi);

		CMatrixDouble44 I4;
		I4.unit(4, 1.0);

		EXPECT_NEAR((HM * HMi - I4).array().abs().sum(), 0, 1e-3)
			<< "HM:\n"
			<< HM << "inv(HM):\n"
			<< HMi << "inv(HM)*HM:\n"
			<< HM * HMi << endl;

		CPose3D p1_inv_inv = p1;

		p1_inv_inv.inverse();
		const auto HMi_from_p1_inv =
			p1_inv_inv.getHomogeneousMatrixVal<CMatrixDouble44>();

		p1_inv_inv.inverse();

		EXPECT_NEAR(
			(p1.getAsVectorVal() - p1_inv_inv.getAsVectorVal())
				.array()
				.abs()
				.sum(),
			0, 1e-3)
			<< "p1: " << p1 << "p1_inv_inv: " << p1_inv_inv << endl;

		EXPECT_NEAR((HMi_from_p1_inv - HMi).array().abs().sum(), 0, 1e-4)
			<< "HMi_from_p1_inv:\n"
			<< HMi_from_p1_inv << "HMi:\n"
			<< HMi << endl;
	}

	void test_compose(const CPose3D& p1, const CPose3D& p2)
	{
		const CPose3D p1_c_p2 = p1 + p2;
		const CPose3D p1_i_p2 = p1 - p2;

		const CPose3D p1_c_p2_i_p2 = p1_c_p2 - p1;  // should be -> p2
		const CPose3D p2_c_p1_i_p2 = p2 + p1_i_p2;  // Should be -> p1

		EXPECT_NEAR(
			0,
			(p1_c_p2_i_p2.getAsVectorVal() - p2.getAsVectorVal())
				.array()
				.abs()
				.sum(),
			1e-5)
			<< "p2          : " << p2 << endl
			<< "p1_c_p2_i_p2: " << p1_c_p2_i_p2 << endl;

		EXPECT_NEAR(
			0,
			(p2_c_p1_i_p2.getAsVectorVal() - p1.getAsVectorVal())
				.array()
				.abs()
				.sum(),
			1e-5)
			<< "p1          : " << p1 << endl
			<< "p2          : " << p2 << endl
			<< "p2 matrix   : " << endl
			<< p2.getHomogeneousMatrixVal<CMatrixDouble44>() << endl
			<< "p1_i_p2     : " << p1_i_p2 << endl
			<< "p1_i_p2 matrix: " << endl
			<< p1_i_p2.getHomogeneousMatrixVal<CMatrixDouble44>() << endl
			<< "p2_c_p1_i_p2: " << p2_c_p1_i_p2 << endl;

		// Test + operator: trg new var
		{
			CPose3D C = p1;
			CPose3D A = C + p2;
			EXPECT_NEAR(
				0,
				(A.getAsVectorVal() - p1_c_p2.getAsVectorVal())
					.array()
					.abs()
					.sum(),
				1e-6);
		}
		// Test + operator: trg same var
		{
			CPose3D A = p1;
			A = A + p2;
			EXPECT_NEAR(
				0,
				(A.getAsVectorVal() - p1_c_p2.getAsVectorVal())
					.array()
					.abs()
					.sum(),
				1e-6);
		}
		// Test =+ operator
		{
			CPose3D A = p1;
			A += p2;
			EXPECT_NEAR(
				0,
				(A.getAsVectorVal() - p1_c_p2.getAsVectorVal())
					.array()
					.abs()
					.sum(),
				1e-6);
		}
	}

	void test_to_from_2d(double x, double y, double phi)
	{
		const CPose2D p2d = CPose2D(x, y, phi);
		const CPose3D p3d = CPose3D(p2d);

		const CPose2D p2d_bis = CPose2D(p3d);

		EXPECT_FLOAT_EQ(p2d.x(), p2d_bis.x()) << "p2d: " << p2d << endl;
		EXPECT_FLOAT_EQ(p2d.y(), p2d_bis.y()) << "p2d: " << p2d << endl;
		EXPECT_FLOAT_EQ(p2d.phi(), p2d_bis.phi()) << "p2d: " << p2d << endl;

		EXPECT_FLOAT_EQ(p2d.phi(), p3d.yaw()) << "p2d: " << p2d << endl;
	}

	void test_composeFrom(const CPose3D& p1, const CPose3D& p2)
	{
		const CPose3D p1_plus_p2 = p1 + p2;

		{
			CPose3D p1_plus_p2bis;
			p1_plus_p2bis.composeFrom(p1, p2);

			EXPECT_NEAR(
				0,
				(p1_plus_p2bis.getAsVectorVal() - p1_plus_p2.getAsVectorVal())
					.array()
					.abs()
					.sum(),
				1e-5)
				<< "p2 : " << p2 << endl
				<< "p1 : " << p1 << endl
				<< "p1_plus_p2    : " << p1_plus_p2 << endl
				<< "p1_plus_p2bis : " << p1_plus_p2bis << endl;
		}

		{
			CPose3D p1_plus_p2bis = p1;
			p1_plus_p2bis.composeFrom(p1_plus_p2bis, p2);

			EXPECT_NEAR(
				0,
				(p1_plus_p2bis.getAsVectorVal() - p1_plus_p2.getAsVectorVal())
					.array()
					.abs()
					.sum(),
				1e-5)
				<< "p2 : " << p2 << endl
				<< "p1 : " << p1 << endl
				<< "p1_plus_p2    : " << p1_plus_p2 << endl
				<< "p1_plus_p2bis : " << p1_plus_p2bis << endl;
		}

		{
			CPose3D p1_plus_p2bis = p2;
			p1_plus_p2bis.composeFrom(p1, p1_plus_p2bis);

			EXPECT_NEAR(
				0,
				(p1_plus_p2bis.getAsVectorVal() - p1_plus_p2.getAsVectorVal())
					.array()
					.abs()
					.sum(),
				1e-5)
				<< "p2 : " << p2 << endl
				<< "p1 : " << p1 << endl
				<< "p1_plus_p2    : " << p1_plus_p2 << endl
				<< "p1_plus_p2bis : " << p1_plus_p2bis << endl;
		}
	}

	void test_composePoint(const CPose3D& p1, double x, double y, double z)
	{
		const CPoint3D p(x, y, z);
		CPoint3D p1_plus_p = p1 + p;

		CPoint3D p1_plus_p2;
		p1.composePoint(
			p.x(), p.y(), p.z(), p1_plus_p2.x(), p1_plus_p2.y(),
			p1_plus_p2.z());

		EXPECT_NEAR(
			0,
			(p1_plus_p2.getAsVectorVal() - p1_plus_p.getAsVectorVal())
				.array()
				.abs()
				.sum(),
			1e-5);

		// Repeat using same input/output variables:
		{
			double lx = p.x();
			double ly = p.y();
			double lz = p.z();

			p1.composePoint(lx, ly, lz, lx, ly, lz);
			EXPECT_NEAR(
				0,
				std::abs(lx - p1_plus_p.x()) + std::abs(ly - p1_plus_p.y()) +
					std::abs(lz - p1_plus_p.z()),
				1e-5);
		}

		// Inverse:
		CPoint3D p_recov = p1_plus_p - p1;
		CPoint3D p_recov2;
		p1.inverseComposePoint(
			p1_plus_p.x(), p1_plus_p.y(), p1_plus_p.z(), p_recov2.x(),
			p_recov2.y(), p_recov2.z());

		EXPECT_NEAR(
			0,
			(p_recov2.getAsVectorVal() - p_recov.getAsVectorVal())
				.array()
				.abs()
				.sum(),
			1e-5);

		EXPECT_NEAR(
			0,
			(p.getAsVectorVal() - p_recov.getAsVectorVal()).array().abs().sum(),
			1e-5);
	}

	static void func_compose_point(
		const CArrayDouble<6 + 3>& x, const double& dummy, CArrayDouble<3>& Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		CPose3D q(x[0], x[1], x[2], x[3], x[4], x[5]);
		const CPoint3D p(x[6 + 0], x[6 + 1], x[6 + 2]);
		const CPoint3D pp = q + p;
		for (int i = 0; i < 3; i++) Y[i] = pp[i];
	}

	static void func_inv_compose_point(
		const CArrayDouble<6 + 3>& x, const double& dummy, CArrayDouble<3>& Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		CPose3D q(x[0], x[1], x[2], x[3], x[4], x[5]);
		const CPoint3D p(x[6 + 0], x[6 + 1], x[6 + 2]);
		const CPoint3D pp = p - q;
		Y[0] = pp.x();
		Y[1] = pp.y();
		Y[2] = pp.z();
	}

	void test_composePointJacob(
	    const CPose3D& p1, double x, double y, double z, bool use_aprox = false)
	{
		const CPoint3D p(x, y, z);

		CMatrixFixedNumeric<double, 3, 3> df_dpoint;
		CMatrixFixedNumeric<double, 3, 6> df_dpose;

		TPoint3D pp;
		p1.composePoint(
			x, y, z, pp.x, pp.y, pp.z, &df_dpoint, &df_dpose, nullptr,
			use_aprox);

		// Numerical approx:
		CMatrixFixedNumeric<double, 3, 3> num_df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixedNumeric<double, 3, 6> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<6 + 3> x_mean;
			for (int i = 0; i < 6; i++) x_mean[i] = p1[i];
			x_mean[6 + 0] = x;
			x_mean[6 + 1] = y;
			x_mean[6 + 2] = z;

			double DUMMY = 0;
			CArrayDouble<6 + 3> x_incrs;
			x_incrs.assign(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<6 + 3>& x, const double& dummy,
					CArrayDouble<3>& Y)>(&func_compose_point),
				x_incrs, DUMMY, numJacobs);

			numJacobs.extractMatrix(0, 0, num_df_dpose);
			numJacobs.extractMatrix(0, 6, num_df_dpoint);
		}

		const double max_eror = use_aprox ? 0.1 : 3e-3;

		EXPECT_NEAR(
			0, (df_dpoint - num_df_dpoint).array().abs().sum(), max_eror)
			<< "p1: " << p1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpoint: " << endl
			<< num_df_dpoint << endl
			<< "Implemented method: " << endl
			<< df_dpoint << endl
			<< "Error: " << endl
			<< df_dpoint - num_df_dpoint << endl;

		EXPECT_NEAR(0, (df_dpose - num_df_dpose).array().abs().sum(), max_eror)
			<< "p1: " << p1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpose: " << endl
			<< num_df_dpose << endl
			<< "Implemented method: " << endl
			<< df_dpose << endl
			<< "Error: " << endl
			<< df_dpose - num_df_dpose << endl;
	}

	void test_ExpLnEqual(const CPose3D& p1)
	{
		const CPose3D p2 = CPose3D::exp(p1.ln());
		EXPECT_NEAR(
			(p1.getAsVectorVal() - p2.getAsVectorVal()).array().abs().sum(), 0,
			1e-5)
			<< "p1: " << p1 << endl;
	}

	void test_invComposePointJacob(
	    const CPose3D& p1, double x, double y, double z)
	{
		const CPoint3D p(x, y, z);

		CMatrixFixedNumeric<double, 3, 3> df_dpoint;
		CMatrixFixedNumeric<double, 3, 6> df_dpose;

		TPoint3D pp;
		p1.inverseComposePoint(
			x, y, z, pp.x, pp.y, pp.z, &df_dpoint, &df_dpose);

		// Numerical approx:
		CMatrixFixedNumeric<double, 3, 3> num_df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixedNumeric<double, 3, 6> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<6 + 3> x_mean;
			for (int i = 0; i < 6; i++) x_mean[i] = p1[i];
			x_mean[6 + 0] = x;
			x_mean[6 + 1] = y;
			x_mean[6 + 2] = z;

			double DUMMY = 0;
			CArrayDouble<6 + 3> x_incrs;
			x_incrs.assign(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<6 + 3>& x, const double& dummy,
					CArrayDouble<3>& Y)>(&func_inv_compose_point),
				x_incrs, DUMMY, numJacobs);

			numJacobs.extractMatrix(0, 0, num_df_dpose);
			numJacobs.extractMatrix(0, 6, num_df_dpoint);
		}

		EXPECT_NEAR(0, (df_dpoint - num_df_dpoint).array().abs().sum(), 3e-3)
			<< "p1: " << p1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpoint: " << endl
			<< num_df_dpoint << endl
			<< "Implemented method: " << endl
			<< df_dpoint << endl
			<< "Error: " << endl
			<< df_dpoint - num_df_dpoint << endl;

		EXPECT_NEAR(0, (df_dpose - num_df_dpose).array().abs().sum(), 3e-3)
			<< "p1: " << p1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpose: " << endl
			<< num_df_dpose << endl
			<< "Implemented method: " << endl
			<< df_dpose << endl
			<< "Error: " << endl
			<< df_dpose - num_df_dpose << endl;
	}

	void test_default_values(const CPose3D& p, const std::string& label)
	{
		EXPECT_EQ(p.x(), 0);
		EXPECT_EQ(p.y(), 0);
		EXPECT_EQ(p.z(), 0);
		EXPECT_EQ(p.yaw(), 0);
		EXPECT_EQ(p.pitch(), 0);
		EXPECT_EQ(p.roll(), 0);
		for (size_t i = 0; i < 4; i++)
			for (size_t j = 0; j < 4; j++)
				EXPECT_NEAR(
					p.getHomogeneousMatrixVal<CMatrixDouble44>()(i, j),
					i == j ? 1.0 : 0.0, 1e-8)
					<< "Failed for (i,j)=" << i << "," << j << endl
					<< "Matrix is: " << endl
					<< p.getHomogeneousMatrixVal<CMatrixDouble44>() << endl
					<< "case was: " << label << endl;
	}

	static void func_compose_point_se3(
		const CArrayDouble<6>& x, const CArrayDouble<3>& P, CArrayDouble<3>& Y)
	{
		CPose3D q = CPose3D::exp(x);
		const CPoint3D p(P[0], P[1], P[2]);
		const CPoint3D pp = q + p;
		for (int i = 0; i < 3; i++) Y[i] = pp[i];
	}

	static void func_invcompose_point_se3(
		const CArrayDouble<6>& x, const CArrayDouble<3>& P, CArrayDouble<3>& Y)
	{
		CPose3D q = CPose3D::exp(x);
		const CPoint3D p(P[0], P[1], P[2]);
		const CPoint3D pp = p - q;
		for (int i = 0; i < 3; i++) Y[i] = pp[i];
	}

	void test_composePointJacob_se3(const CPose3D& p, const TPoint3D x_l)
	{
		CMatrixFixedNumeric<double, 3, 6> df_dse3;

		TPoint3D pp;
		p.composePoint(
			x_l.x, x_l.y, x_l.z, pp.x, pp.y, pp.z, nullptr, nullptr, &df_dse3);

		// Numerical approx:
		CMatrixFixedNumeric<double, 3, 6> num_df_dse3(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<6> x_mean;
			for (int i = 0; i < 6; i++) x_mean[i] = 0;

			CArrayDouble<3> P;
			for (int i = 0; i < 3; i++) P[i] = pp[i];

			CArrayDouble<6> x_incrs;
			x_incrs.assign(1e-9);
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<6>& x, const CArrayDouble<3>& P,
					CArrayDouble<3>& Y)>(&func_compose_point_se3),
				x_incrs, P, num_df_dse3);
		}

		EXPECT_NEAR(0, (df_dse3 - num_df_dse3).array().abs().sum(), 3e-3)
			<< "p: " << p << endl
			<< "x_l:  " << x_l << endl
			<< "Numeric approximation of df_dse3: " << endl
			<< num_df_dse3 << endl
			<< "Implemented method: " << endl
			<< df_dse3 << endl
			<< "Error: " << endl
			<< df_dse3 - num_df_dse3 << endl;
	}

	void test_invComposePointJacob_se3(const CPose3D& p, const TPoint3D x_g)
	{
		CMatrixFixedNumeric<double, 3, 6> df_dse3;

		TPoint3D pp;
		p.inverseComposePoint(
			x_g.x, x_g.y, x_g.z, pp.x, pp.y, pp.z, nullptr, nullptr, &df_dse3);

		// Numerical approx:
		CMatrixFixedNumeric<double, 3, 6> num_df_dse3(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<6> x_mean;
			for (int i = 0; i < 6; i++) x_mean[i] = 0;

			CArrayDouble<3> P;
			for (int i = 0; i < 3; i++) P[i] = pp[i];

			CArrayDouble<6> x_incrs;
			x_incrs.assign(1e-9);
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<6>& x, const CArrayDouble<3>& P,
					CArrayDouble<3>& Y)>(&func_invcompose_point_se3),
				x_incrs, P, num_df_dse3);
		}

		EXPECT_NEAR(0, (df_dse3 - num_df_dse3).array().abs().sum(), 3e-3)
			<< "p: " << p << endl
			<< "x_g:  " << x_g << endl
			<< "Numeric approximation of df_dse3: " << endl
			<< num_df_dse3 << endl
			<< "Implemented method: " << endl
			<< df_dse3 << endl
			<< "Error: " << endl
			<< df_dse3 - num_df_dse3 << endl;
	}

	static void func_jacob_expe_e(
		const CArrayDouble<6>& x, const double& dummy, CArrayDouble<12>& Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		const CPose3D p = CPose3D::exp(x);
		// const CMatrixDouble44 R =
		// p.getHomogeneousMatrixVal<CMatrixDouble44>();
		p.getAs12Vector(Y);
	}

	// Check dexp(e)_de
	void check_jacob_expe_e_at_0()
	{
		CArrayDouble<6> x_mean;
		for (int i = 0; i < 6; i++) x_mean[i] = 0;

		double dummy = 0.;
		CArrayDouble<6> x_incrs;
		x_incrs.assign(1e-9);
		CMatrixDouble numJacobs;
		mrpt::math::estimateJacobian(
			x_mean,
			std::function<void(
				const CArrayDouble<6>& x, const double& dummy,
				CArrayDouble<12>& Y)>(&func_jacob_expe_e),
			x_incrs, dummy, numJacobs);

		// Theoretical matrix:
		// [ 0   -[e1]_x ]
		// [ 0   -[e2]_x ]
		// [ 0   -[e3]_x ]
		// [ I_3    0    ]
		double vals[12 * 6] = {
			0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  0, 1, 0, 0, 0, 0, -1, 0,

			0, 0, 0, 0, 0, -1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 1, 0,  0,

			0, 0, 0, 0, 1, 0,  0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0,  0,

			1, 0, 0, 0, 0, 0,  0, 1, 0, 0,  0, 0, 0, 0, 1, 0, 0,  0};
		CMatrixFixedNumeric<double, 12, 6> M(vals);

		EXPECT_NEAR((numJacobs - M).array().abs().maxCoeff(), 0, 1e-5)
			<< "M:\n"
			<< M << "numJacobs:\n"
			<< numJacobs << "\n";
	}

	static void func_jacob_LnT_T(
		const CArrayDouble<12>& x, const double& dummy, CArrayDouble<6>& Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		CPose3D p;

		p.setFrom12Vector(x);
		// ensure 3x3 rot vector is orthonormal (Sophus complains otherwise):
		auto R = p.getRotationMatrix();
		const auto Rsvd =
			R.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
		R = Rsvd.matrixU() * Rsvd.matrixV().transpose();
		p.setRotationMatrix(R);

		Y = p.ln();
	}

	// Jacobian of Ln(T) wrt T
	void check_jacob_LnT_T(const CPose3D& p)
	{
		const CMatrixFixedNumeric<double, 6, 12> theor_jacob = p.ln_jacob();

		CMatrixDouble numJacobs;
		{
			CArrayDouble<12> x_mean;
			p.getAs12Vector(x_mean);

			double dummy = 0.;
			CArrayDouble<12> x_incrs;
			x_incrs.assign(1e-6);
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<12>& x, const double& dummy,
					CArrayDouble<6>& Y)>(&func_jacob_LnT_T),
				x_incrs, dummy, numJacobs);
		}

		EXPECT_NEAR((numJacobs - theor_jacob).array().abs().sum(), 0, 1e-3)
			<< "Pose: " << p << endl
			<< "Pose matrix:\n"
			<< p.getHomogeneousMatrixVal<CMatrixDouble44>() << "Num. Jacob:\n"
			<< numJacobs << endl
			<< "Theor. Jacob:\n"
			<< theor_jacob << endl
			<< "ERR:\n"
			<< theor_jacob - numJacobs << endl;
	}

	static void func_jacob_expe_D(
		const CArrayDouble<6>& eps, const CPose3D& D, CArrayDouble<12>& Y)
	{
		CPose3D incr;
		CPose3D::exp(eps, incr);
		const CPose3D expe_D = incr + D;
		expe_D.getAs12Vector(Y);
	}

	// Test Jacobian: d exp(e)*D / d e
	// 10.3.3 in tech report
	void test_Jacob_dexpeD_de(const CPose3D& p)
	{
		const mrpt::math::CMatrixDouble12_6 theor_jacob =
		    CPose3D::jacob_dexpeD_de(p);

		CMatrixDouble numJacobs;
		{
			CArrayDouble<6> x_mean;
			x_mean.setZero();

			CArrayDouble<6> x_incrs;
			x_incrs.assign(1e-6);
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<6>& eps, const CPose3D& D,
					CArrayDouble<12>& Y)>(&func_jacob_expe_D),
				x_incrs, p, numJacobs);
		}

		EXPECT_NEAR((numJacobs - theor_jacob).array().abs().maxCoeff(), 0, 1e-3)
			<< "Pose: " << p << endl
			<< "Pose matrix:\n"
			<< p.getHomogeneousMatrixVal<CMatrixDouble44>() << "Num. Jacob:\n"
			<< numJacobs << endl
			<< "Theor. Jacob:\n"
			<< theor_jacob << endl
			<< "ERR:\n"
			<< theor_jacob - numJacobs << endl;
	}

	static void func_jacob_D_expe(
		const CArrayDouble<6>& eps, const CPose3D& D, CArrayDouble<12>& Y)
	{
		CPose3D incr;
		CPose3D::exp(eps, incr);
		const CPose3D expe_D = D + incr;
		expe_D.getAs12Vector(Y);
	}

	// Test Jacobian: d D*exp(e) / d e
	// 10.3.4 in tech report
	void test_Jacob_dDexpe_de(const CPose3D& p)
	{
		const mrpt::math::CMatrixDouble12_6 theor_jacob =
		    CPose3D::jacob_dDexpe_de(p);

		CMatrixDouble numJacobs;
		{
			CArrayDouble<6> x_mean;
			x_mean.setZero();

			CArrayDouble<6> x_incrs;
			x_incrs.assign(1e-6);
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<6>& eps, const CPose3D& D,
					CArrayDouble<12>& Y)>(&func_jacob_D_expe),
				x_incrs, p, numJacobs);
		}

		EXPECT_NEAR((numJacobs - theor_jacob).array().abs().maxCoeff(), 0, 1e-3)
			<< "Pose: " << p << endl
			<< "Pose matrix:\n"
			<< p.getHomogeneousMatrixVal<CMatrixDouble44>() << "Num. Jacob:\n"
			<< numJacobs << endl
			<< "Theor. Jacob:\n"
			<< theor_jacob << endl
			<< "ERR:\n"
			<< theor_jacob - numJacobs << endl;
	}

	struct TParams_func_jacob_Aexpe_D
	{
		CPose3D A, D;
	};

	static void func_jacob_Aexpe_D(
		const CArrayDouble<6>& eps, const TParams_func_jacob_Aexpe_D& params,
		CArrayDouble<12>& Y)
	{
		CPose3D incr;
		CPose3D::exp(eps, incr);
		const CPose3D res = params.A + incr + params.D;
		res.getAs12Vector(Y);
	}

	// Test Jacobian: d A*exp(e)*D / d e
	// 10.3.7 in tech report
	// http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
	void test_Jacob_dAexpeD_de(const CPose3D& A, const CPose3D& D)
	{
		mrpt::math::CMatrixDouble12_6 theor_jacob =
		    CPose3D::jacob_dAexpeD_de(A, D);

		CMatrixDouble numJacobs;
		{
			CArrayDouble<6> x_mean;
			x_mean.setZero();

			TParams_func_jacob_Aexpe_D params;
			params.A = A;
			params.D = D;
			CArrayDouble<6> x_incrs;
			x_incrs.assign(1e-6);
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CArrayDouble<6>& eps,
					const TParams_func_jacob_Aexpe_D& params,
					CArrayDouble<12>& Y)>(&func_jacob_Aexpe_D),
				x_incrs, params, numJacobs);
		}

		EXPECT_NEAR((numJacobs - theor_jacob).array().abs().maxCoeff(), 0, 1e-3)
			<< "Pose A: " << A << endl
			<< "Pose D: " << D << endl
			<< "Num. Jacob:\n"
			<< numJacobs << endl
			<< "Theor. Jacob:\n"
			<< theor_jacob << endl
			<< "ERR:\n"
			<< theor_jacob - numJacobs << endl;
	}
};

// Elemental tests:
TEST_F(Pose3DTests, DefaultValues)
{
	{
		CPose3D p;
		test_default_values(p, "Default");
	}
	{
		CPose3D p2;
		CPose3D p = p2;
		test_default_values(p, "p=p2");
	}
	{
		CPose3D p1, p2;
		test_default_values(p1 + p2, "p1+p2");
		CPose3D p = p1 + p2;
		test_default_values(p, "p=p1+p2");
	}
	{
		CPose3D p1, p2;
		CPose3D p = p1 - p2;
		test_default_values(p, "p1-p2");
	}
}

TEST_F(Pose3DTests, Initialization)
{
	CPose3D p(1, 2, 3, 0.2, 0.3, 0.4);
	EXPECT_NEAR(p.x(), 1, 1e-7);
	EXPECT_NEAR(p.y(), 2, 1e-7);
	EXPECT_NEAR(p.z(), 3, 1e-7);
	EXPECT_NEAR(p.yaw(), 0.2, 1e-7);
	EXPECT_NEAR(p.pitch(), 0.3, 1e-7);
	EXPECT_NEAR(p.roll(), 0.4, 1e-7);
}

TEST_F(Pose3DTests, OperatorBracket)
{
	CPose3D p(1, 2, 3, 0.2, 0.3, 0.4);
	EXPECT_NEAR(p[0], 1, 1e-7);
	EXPECT_NEAR(p[1], 2, 1e-7);
	EXPECT_NEAR(p[2], 3, 1e-7);
	EXPECT_NEAR(p[3], 0.2, 1e-7);
	EXPECT_NEAR(p[4], 0.3, 1e-7);
	EXPECT_NEAR(p[5], 0.4, 1e-7);
}

// List of "random" poses to test with (x,y,z,yaw,pitch,roll) (angles in
// degrees)
static const std::vector<mrpt::poses::CPose3D> ptc = {
    {.0, .0, .0, DEG2RAD(.0), DEG2RAD(.0), DEG2RAD(.0)},
    {1.0, 2.0, 3.0, DEG2RAD(.0), DEG2RAD(.0), DEG2RAD(.0)},
    {1.0, 2.0, 3.0, DEG2RAD(10.0), DEG2RAD(.0), DEG2RAD(.0)},
    {1.0, 2.0, 3.0, DEG2RAD(.0), DEG2RAD(1.0), DEG2RAD(.0)},
    {1.0, 2.0, 3.0, DEG2RAD(.0), DEG2RAD(.0), DEG2RAD(1.0)},
    {1.0, 2.0, 3.0, DEG2RAD(80.0), DEG2RAD(5.0), DEG2RAD(5.0)},
    {1.0, 2.0, 3.0, DEG2RAD(-20.0), DEG2RAD(-30.0), DEG2RAD(-40.0)},
    {1.0, 2.0, 3.0, DEG2RAD(-45.0), DEG2RAD(10.0), DEG2RAD(70.0)},
    {1.0, 2.0, 3.0, DEG2RAD(40.0), DEG2RAD(-5.0), DEG2RAD(25.0)},
    {1.0, 2.0, 3.0, DEG2RAD(40.0), DEG2RAD(20.0), DEG2RAD(-15.0)},
    {-6.0, 2.0, 3.0, DEG2RAD(40.0), DEG2RAD(20.0), DEG2RAD(15.0)},
    {6.0, -5.0, 3.0, DEG2RAD(40.0), DEG2RAD(20.0), DEG2RAD(15.0)},
    {6.0, 2.0, -9.0, DEG2RAD(40.0), DEG2RAD(20.0), DEG2RAD(15.0)},
    {0.0, 8.0, 5.0, DEG2RAD(-45.0), DEG2RAD(10.0), DEG2RAD(70.0)},
    {1.0, 0.0, 5.0, DEG2RAD(-45.0), DEG2RAD(10.0), DEG2RAD(70.0)},
    {1.0, 8.0, 0.0, DEG2RAD(-45.0), DEG2RAD(10.0), DEG2RAD(70.0)}};

// More complex tests:
TEST_F(Pose3DTests, InverseHM)
{
	for (const auto& p : ptc) test_inverse(p);
}

TEST_F(Pose3DTests, Compose)
{
	for (const auto& p1 : ptc)
		for (const auto& p2 : ptc) test_compose(p1, p2);
}
TEST_F(Pose3DTests, composeFrom)
{
	for (const auto& p1 : ptc)
		for (const auto& p2 : ptc) test_composeFrom(p1, p2);
}

TEST_F(Pose3DTests, ToFromCPose2D)
{
	for (const auto& p : ptc) test_to_from_2d(p.x(), p.y(), p.yaw());
}

TEST_F(Pose3DTests, ComposeAndInvComposeWithPoint)
{
	for (const auto& p : ptc)
	{
		test_composePoint(p, 10, 11, 12);
		test_composePoint(p, -5, 1, 2);
		test_composePoint(p, 5, -1, 2);
		test_composePoint(p, 5, 1, -2);
	}
}

TEST_F(Pose3DTests, ComposePointJacob)
{
	for (const auto& p : ptc)
	{
		test_composePointJacob(p, 10, 11, 12);
		test_composePointJacob(p, -5, 1, 2);
	}
}

TEST_F(Pose3DTests, ComposePointJacobApprox)
{  // Test approximated Jacobians for very small rotations
	test_composePointJacob(
	    mrpt::poses::CPose3D(1.0, 2.0, 3.0, DEG2RAD(0), DEG2RAD(0), DEG2RAD(0)),
	    10, 11, 12, true);
	test_composePointJacob(
	    mrpt::poses::CPose3D(
	        1.0, 2.0, 3.0, DEG2RAD(0.1), DEG2RAD(0), DEG2RAD(0)),
	    10, 11, 12, true);
	test_composePointJacob(
	    mrpt::poses::CPose3D(
	        1.0, 2.0, 3.0, DEG2RAD(0), DEG2RAD(0.1), DEG2RAD(0)),
	    10, 11, 12, true);
	test_composePointJacob(
	    mrpt::poses::CPose3D(
	        1.0, 2.0, 3.0, DEG2RAD(0), DEG2RAD(0), DEG2RAD(0.1)),
	    10, 11, 12, true);
}

TEST_F(Pose3DTests, InvComposePointJacob)
{
	for (const auto& p : ptc)
	{
		test_invComposePointJacob(p, 10, 11, 12);
		test_invComposePointJacob(p, -5, 1, 2);
		test_invComposePointJacob(p, 5, -1, 2);
		test_invComposePointJacob(p, 5, 1, -2);
	}
}

TEST_F(Pose3DTests, ComposePointJacob_se3)
{
	for (const auto& p : ptc)
	{
		test_composePointJacob_se3(p, TPoint3D(0, 0, 0));
		test_composePointJacob_se3(p, TPoint3D(10, 11, 12));
		test_composePointJacob_se3(p, TPoint3D(-5.0, -15.0, 8.0));
	}
}
TEST_F(Pose3DTests, InvComposePointJacob_se3)
{
	for (const auto& p : ptc)
	{
		test_invComposePointJacob_se3(p, TPoint3D(0, 0, 0));
		test_invComposePointJacob_se3(p, TPoint3D(10, 11, 12));
		test_invComposePointJacob_se3(p, TPoint3D(-5.0, -15.0, 8.0));
	}
}

TEST_F(Pose3DTests, ExpLnEqual)
{
	for (const auto& p : ptc) test_ExpLnEqual(p);
}

TEST_F(Pose3DTests, Jacob_dExpe_de_at_0) { check_jacob_expe_e_at_0(); }
TEST_F(Pose3DTests, Jacob_dLnT_dT)
{
	check_jacob_LnT_T(
	    mrpt::poses::CPose3D(0, 0, 0, DEG2RAD(0), DEG2RAD(0), DEG2RAD(0)));
	// JL NOTE:
	//  This function cannot be properly tested numerically, since the logm()
	//  implementation
	//  is not generic and does NOT depends on all matrix entries, thus the
	//  numerical Jacobian
	//  contains entire columns of zeros, even if the theorethical doesn't.
	//	check_jacob_LnT_T(1.0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) ); ...
}

TEST_F(Pose3DTests, Jacob_dexpeD_de)
{
	for (const auto& p : ptc) test_Jacob_dexpeD_de(p);
}

TEST_F(Pose3DTests, Jacob_dDexpe_de)
{
	for (const auto& p : ptc) test_Jacob_dDexpe_de(p);
}

TEST_F(Pose3DTests, Jacob_dAexpeD_de)
{
	for (const auto& p1 : ptc)
		for (const auto& p2 : ptc) test_Jacob_dAexpeD_de(p1, p2);
}
