/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/math/num_jacobian.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/random.h>
#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

template class mrpt::CTraitsTest<CPose3DQuat>;

class Pose3DQuatTests : public ::testing::Test
{
   protected:
	void SetUp() override {}
	void TearDown() override {}
	void test_compose(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1, double x2, double y2, double z2, double yaw2,
		double pitch2, double roll2)
	{
		const CPose3D p1(x1, y1, z1, yaw1, pitch1, roll1);
		const CPose3D p2(x2, y2, z2, yaw2, pitch2, roll2);

		const CPose3D p1_c_p2 = p1 + p2;
		const CPose3D p1_i_p2 = p1 - p2;

		CPose3DQuat q1 = CPose3DQuat(p1);
		CPose3DQuat q2 = CPose3DQuat(p2);

		CPose3DQuat q1_c_q2 = q1 + q2;
		CPose3DQuat q1_i_q2 = q1 - q2;

		CPose3D p_q1_c_q2 = CPose3D(q1_c_q2);
		CPose3D p_q1_i_q2 = CPose3D(q1_i_q2);

		EXPECT_NEAR(
			0,
			(p1_c_p2.asVectorVal() - p_q1_c_q2.asVectorVal())
				.array()
				.abs()
				.sum(),
			1e-5)
			<< "p1_c_p2: " << p1_c_p2 << endl
			<< "q1_c_p2: " << p_q1_c_q2 << endl;

		EXPECT_NEAR(
			0,
			(p1_i_p2.asVectorVal() - p_q1_i_q2.asVectorVal())
				.array()
				.abs()
				.sum(),
			1e-5)
			<< "p1_i_p2: " << p1_i_p2 << endl
			<< "q1_i_p2: " << p_q1_i_q2 << endl;

		// Test + operator: trg new var
		{
			CPose3DQuat C = q1;
			CPose3DQuat A = C + q2;
			EXPECT_NEAR(
				0,
				(A.asVectorVal() - q1_c_q2.asVectorVal()).array().abs().sum(),
				1e-6);
		}
		// Test + operator: trg same var
		{
			CPose3DQuat A = q1;
			A = A + q2;
			EXPECT_NEAR(
				0,
				(A.asVectorVal() - q1_c_q2.asVectorVal()).array().abs().sum(),
				1e-6);
		}
		// Test =+ operator
		{
			CPose3DQuat A = q1;
			A += q2;
			EXPECT_NEAR(
				0,
				(A.asVectorVal() - q1_c_q2.asVectorVal()).array().abs().sum(),
				1e-6);
		}
	}

	void test_composePoint(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1, double x, double y, double z)
	{
		const CPose3D p1(x1, y1, z1, yaw1, pitch1, roll1);
		const CPose3DQuat q1(p1);
		const CPoint3D p(x, y, z);

		CPoint3D p1_plus_p = p1 + p;
		CPoint3D q1_plus_p = q1 + p;

		EXPECT_NEAR(
			0,
			(p1_plus_p.asVectorVal() - q1_plus_p.asVectorVal())
				.array()
				.abs()
				.sum(),
			1e-5)
			<< "p1: " << p1 << endl
			<< "q1: " << q1 << endl
			<< "p: " << p << endl
			<< "p1_plus_p: " << p1_plus_p << endl
			<< "q1_plus_p: " << q1_plus_p << endl;
	}

	static void func_compose_point(
		const CVectorFixedDouble<7 + 3>& x, const double& dummy,
		CVectorFixedDouble<3>& Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		CPose3DQuat q(
			x[0], x[1], x[2], CQuaternionDouble(x[3], x[4], x[5], x[6]));
		q.quat().normalize();
		const CPoint3D p(x[7 + 0], x[7 + 1], x[7 + 2]);
		const CPoint3D pp = q + p;
		for (int i = 0; i < 3; i++) Y[i] = pp[i];
	}

	void test_composePointJacob(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1, double x, double y, double z)
	{
		const CPose3DQuat q1(CPose3D(x1, y1, z1, yaw1, pitch1, roll1));
		const CPoint3D p(x, y, z);

		CMatrixFixed<double, 3, 3> df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixed<double, 3, 7> df_dpose(UNINITIALIZED_MATRIX);

		TPoint3D l;
		q1.composePoint(x, y, z, l.x, l.y, l.z, &df_dpoint, &df_dpose);

		// Numerical approximation:
		CMatrixFixed<double, 3, 3> num_df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixed<double, 3, 7> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CVectorFixedDouble<7 + 3> x_mean;
			for (int i = 0; i < 7; i++) x_mean[i] = q1[i];
			x_mean[7 + 0] = x;
			x_mean[7 + 1] = y;
			x_mean[7 + 2] = z;

			double DUMMY = 0;
			CVectorFixedDouble<7 + 3> x_incrs;
			x_incrs.fill(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CVectorFixedDouble<7 + 3>& x, const double& dummy,
					CVectorFixedDouble<3>& Y)>(&func_compose_point),
				x_incrs, DUMMY, numJacobs);

			num_df_dpose = numJacobs.asEigen().block<3, 7>(0, 0);
			num_df_dpoint = numJacobs.asEigen().block<3, 3>(0, 7);
		}

		// Compare:
		EXPECT_NEAR(
			0,
			(df_dpoint.asEigen() - num_df_dpoint.asEigen()).array().abs().sum(),
			3e-3)
			<< "q1: " << q1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpoint: " << endl
			<< num_df_dpoint.asEigen() << endl
			<< "Implemented method: " << endl
			<< df_dpoint << endl
			<< "Error: " << endl
			<< df_dpoint.asEigen() - num_df_dpoint.asEigen() << endl;

		EXPECT_NEAR(
			0,
			(df_dpose.asEigen() - num_df_dpose.asEigen()).array().abs().sum(),
			3e-3)
			<< "q1: " << q1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpose: " << endl
			<< num_df_dpose.asEigen() << endl
			<< "Implemented method: " << endl
			<< df_dpose.asEigen() << endl
			<< "Error: " << endl
			<< df_dpose.asEigen() - num_df_dpose.asEigen() << endl;
	}

	void test_invComposePoint(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1, double x, double y, double z)
	{
		const CPose3D p1(x1, y1, z1, yaw1, pitch1, roll1);
		const CPose3DQuat q1(p1);
		const CPoint3D p(x, y, z);

		CPoint3D p_minus_p1 = p - p1;
		CPoint3D p_minus_q1 = p - q1;

		CPoint3D p_rec = q1 + p_minus_q1;

		EXPECT_NEAR(
			0,
			(p_minus_p1.asVectorVal() - p_minus_q1.asVectorVal())
				.array()
				.abs()
				.sum(),
			1e-5)
			<< "p_minus_p1: " << p_minus_p1 << endl
			<< "p_minus_q1: " << p_minus_q1 << endl;

		EXPECT_NEAR(
			0, (p_rec.asVectorVal() - p.asVectorVal()).array().abs().sum(),
			1e-5)
			<< "p_rec: " << p_rec << endl
			<< "p: " << p << endl;
	}

	static void func_inv_compose_point(
		const CVectorFixedDouble<7 + 3>& x, const double& dummy,
		CVectorFixedDouble<3>& Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		CPose3DQuat q(
			x[0], x[1], x[2], CQuaternionDouble(x[3], x[4], x[5], x[6]));
		q.quat().normalize();
		const CPoint3D p(x[7 + 0], x[7 + 1], x[7 + 2]);
		const CPoint3D pp = p - q;
		Y[0] = pp.x();
		Y[1] = pp.y();
		Y[2] = pp.z();
	}

	void test_invComposePointJacob(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1, double x, double y, double z)
	{
		const CPose3DQuat q1(CPose3D(x1, y1, z1, yaw1, pitch1, roll1));
		const CPoint3D p(x, y, z);

		CMatrixFixed<double, 3, 3> df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixed<double, 3, 7> df_dpose(UNINITIALIZED_MATRIX);

		TPoint3D l;
		q1.inverseComposePoint(x, y, z, l.x, l.y, l.z, &df_dpoint, &df_dpose);

		// Also check the returned point, not just the jacobian:
		TPoint3D theorical;
		{
			const double qr = q1.quat().r();
			const double qx = q1.quat().x();
			const double qy = q1.quat().y();
			const double qz = q1.quat().z();
			const double Ax = x - x1;  // ax  -  x;
			const double Ay = y - y1;  // ay  -  y;
			const double Az = z - z1;  // az  -  z;
			theorical.x = Ax + 2 * (Ay) * (qr * qz + qx * qy) -
						  2 * (Az) * (qr * qy - qx * qz) -
						  2 * (square(qy) + square(qz)) * (Ax);
			theorical.y = Ay - 2 * (Ax) * (qr * qz - qx * qy) +
						  2 * (Az) * (qr * qx + qy * qz) -
						  2 * (square(qx) + square(qz)) * (Ay);
			theorical.z = Az + 2 * (Ax) * (qr * qy + qx * qz) -
						  2 * (Ay) * (qr * qx - qy * qz) -
						  2 * (square(qx) + square(qy)) * (Az);
		}
		EXPECT_NEAR(theorical.x, l.x, 1e-5);
		EXPECT_NEAR(theorical.y, l.y, 1e-5);
		EXPECT_NEAR(theorical.z, l.z, 1e-5);

		// Numerical approximation:
		CMatrixFixed<double, 3, 3> num_df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixed<double, 3, 7> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CVectorFixedDouble<7 + 3> x_mean;
			for (int i = 0; i < 7; i++) x_mean[i] = q1[i];
			x_mean[7 + 0] = x;
			x_mean[7 + 1] = y;
			x_mean[7 + 2] = z;

			double DUMMY = 0;
			CVectorFixedDouble<7 + 3> x_incrs;
			x_incrs.fill(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CVectorFixedDouble<7 + 3>& x, const double& dummy,
					CVectorFixedDouble<3>& Y)>(&func_inv_compose_point),
				x_incrs, DUMMY, numJacobs);

			num_df_dpose = numJacobs.block<3, 7>(0, 0);
			num_df_dpoint = numJacobs.block<3, 3>(0, 7);
		}

		// Compare:
		EXPECT_NEAR(
			0,
			(df_dpoint.asEigen() - num_df_dpoint.asEigen()).array().abs().sum(),
			3e-3)
			<< "q1: " << q1 << endl
			<< "from pose: " << CPose3D(x1, y1, z1, yaw1, pitch1, roll1) << endl
			<< "p:  " << p << endl
			<< "local:  " << l << endl
			<< "Numeric approximation of df_dpoint: " << endl
			<< num_df_dpoint.asEigen() << endl
			<< "Implemented method: " << endl
			<< df_dpoint.asEigen() << endl
			<< "Error: " << endl
			<< df_dpoint - num_df_dpoint << endl;

		EXPECT_NEAR(
			0,
			(df_dpose.asEigen() - num_df_dpose.asEigen()).array().abs().sum(),
			3e-3)
			<< "q1: " << q1 << endl
			<< "from pose: " << CPose3D(x1, y1, z1, yaw1, pitch1, roll1) << endl
			<< "p:  " << p << endl
			<< "local:  " << l << endl
			<< "Numeric approximation of df_dpose: " << endl
			<< num_df_dpose.asEigen() << endl
			<< "Implemented method: " << endl
			<< df_dpose.asEigen() << endl
			<< "Error: " << endl
			<< df_dpose.asEigen() - num_df_dpose.asEigen() << endl;
	}

	void test_fromYPRAndBack(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1)
	{
		const CPose3D p1(x1, y1, z1, yaw1, pitch1, roll1);
		const CPose3DQuat q1(p1);
		const CPose3D p1r = CPose3D(q1);

		EXPECT_NEAR(
			0,
			(p1.getHomogeneousMatrixVal<CMatrixDouble44>() -
			 q1.getHomogeneousMatrixVal<CMatrixDouble44>())
				.array()
				.abs()
				.sum(),
			1e-5)
			<< "p1.getHomogeneousMatrixVal<CMatrixDouble44>():\n"
			<< p1.getHomogeneousMatrixVal<CMatrixDouble44>() << endl
			<< "q1.getHomogeneousMatrixVal<CMatrixDouble44>():\n"
			<< q1.getHomogeneousMatrixVal<CMatrixDouble44>() << endl;

		EXPECT_NEAR(
			0, (p1.asVectorVal() - p1r.asVectorVal()).array().abs().sum(), 1e-5)
			<< "p1: " << p1 << endl
			<< "q1: " << q1 << endl
			<< "p1r: " << p1r << endl;
	}

	void test_unaryInverse(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1)
	{
		const CPose3D p1(x1, y1, z1, yaw1, pitch1, roll1);
		const CPose3D p1_inv = -p1;

		const CPose3DQuat q1(p1);
		const CPose3DQuat q1_inv = -q1;

		EXPECT_NEAR(
			0,
			(p1_inv.getHomogeneousMatrixVal<CMatrixDouble44>() -
			 q1_inv.getHomogeneousMatrixVal<CMatrixDouble44>())
				.array()
				.abs()
				.sum(),
			1e-5)
			<< "p1_inv.getHomogeneousMatrixVal<CMatrixDouble44>():\n"
			<< p1_inv.getHomogeneousMatrixVal<CMatrixDouble44>() << endl
			<< "q1_inv.getHomogeneousMatrixVal<CMatrixDouble44>():\n"
			<< q1_inv.getHomogeneousMatrixVal<CMatrixDouble44>() << endl;
	}

	void test_copy(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1)
	{
		const CPose3D p1(x1, y1, z1, yaw1, pitch1, roll1);

		const CPose3DQuat q1(p1);
		const CPose3DQuat q2 = q1;

		EXPECT_NEAR(
			0,
			(q1.getHomogeneousMatrixVal<CMatrixDouble44>() -
			 q2.getHomogeneousMatrixVal<CMatrixDouble44>())
				.array()
				.abs()
				.sum(),
			1e-5)
			<< "q1.getHomogeneousMatrixVal<CMatrixDouble44>():\n"
			<< q1.getHomogeneousMatrixVal<CMatrixDouble44>() << endl
			<< "q2.getHomogeneousMatrixVal<CMatrixDouble44>():\n"
			<< q2.getHomogeneousMatrixVal<CMatrixDouble44>() << endl;
	}

	void test_composeAndInvComposePoint(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1, double x, double y, double z)
	{
		const CPose3DQuat q1(CPose3D(x1, y1, z1, yaw1, pitch1, roll1));
		TPoint3D pp(x, y, z), aux;
		q1.composePoint(x, y, z, pp.x, pp.y, pp.z);
		q1.inverseComposePoint(pp.x, pp.y, pp.z, aux.x, aux.y, aux.z);

		EXPECT_NEAR(x, aux.x, 1e-7);
		EXPECT_NEAR(y, aux.y, 1e-7);
		EXPECT_NEAR(z, aux.z, 1e-7);
	}

	void test_composePoint_vs_CPose3D(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1, double x, double y, double z)
	{
		const CPose3D p1(x1, y1, z1, yaw1, pitch1, roll1);
		const CPose3DQuat q1(p1);
		TPoint3D pt1, pt2;
		p1.composePoint(x, y, z, pt1.x, pt1.y, pt1.z);
		q1.composePoint(x, y, z, pt2.x, pt2.y, pt2.z);

		EXPECT_NEAR(pt1.x, pt2.x, 1e-7);
		EXPECT_NEAR(pt1.y, pt2.y, 1e-7);
		EXPECT_NEAR(pt1.z, pt2.z, 1e-7);
	}

	void test_invComposePoint_vs_CPose3D(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1, double x, double y, double z)
	{
		const CPose3D p1(x1, y1, z1, yaw1, pitch1, roll1);
		const CPose3DQuat q1(p1);
		TPoint3D pt1, pt2;
		p1.inverseComposePoint(x, y, z, pt1.x, pt1.y, pt1.z);
		q1.inverseComposePoint(x, y, z, pt2.x, pt2.y, pt2.z);

		EXPECT_NEAR(pt1.x, pt2.x, 1e-7);
		EXPECT_NEAR(pt1.y, pt2.y, 1e-7);
		EXPECT_NEAR(pt1.z, pt2.z, 1e-7);

		{
			CPose3DQuat q = mrpt::poses::CPose3DQuat(p1);

			float gx = x, gy = y, gz = z;

			double dist, yaw, pitch;
			p1.sphericalCoordinates(TPoint3D(gx, gy, gz), dist, yaw, pitch);

			double lx, ly, lz;
			p1.inverseComposePoint(gx, gy, gz, lx, ly, lz);

			double lx2, ly2, lz2;
			q.inverseComposePoint(gx, gy, gz, lx2, ly2, lz2);

			EXPECT_NEAR(lx, lx2, 1e-7);
			EXPECT_NEAR(ly, ly2, 1e-7);
			EXPECT_NEAR(lz, lz2, 1e-7);
		}
	}

	static void func_spherical_coords(
		const CVectorFixedDouble<7 + 3>& x, const double& dummy,
		CVectorFixedDouble<3>& Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		CPose3DQuat q(
			x[0], x[1], x[2], CQuaternionDouble(x[3], x[4], x[5], x[6]));
		q.quat().normalize();
		const TPoint3D p(x[7 + 0], x[7 + 1], x[7 + 2]);
		q.sphericalCoordinates(p, Y[0], Y[1], Y[2]);
	}

	void test_sphericalCoords(
		double x1, double y1, double z1, double yaw1, double pitch1,
		double roll1, double x, double y, double z)
	{
		const CPose3DQuat q1(CPose3D(x1, y1, z1, yaw1, pitch1, roll1));
		const TPoint3D p(x, y, z);

		CMatrixFixed<double, 3, 3> df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixed<double, 3, 7> df_dpose(UNINITIALIZED_MATRIX);

		double hr, hy, hp;
		q1.sphericalCoordinates(p, hr, hy, hp, &df_dpoint, &df_dpose);

		// Numerical approximation:
		CMatrixFixed<double, 3, 3> num_df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixed<double, 3, 7> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CVectorFixedDouble<7 + 3> x_mean;
			for (int i = 0; i < 7; i++) x_mean[i] = q1[i];
			x_mean[7 + 0] = x;
			x_mean[7 + 1] = y;
			x_mean[7 + 2] = z;

			double DUMMY = 0;
			CVectorFixedDouble<7 + 3> x_incrs;
			x_incrs.fill(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CVectorFixedDouble<7 + 3>& x, const double& dummy,
					CVectorFixedDouble<3>& Y)>(&func_spherical_coords),
				x_incrs, DUMMY, numJacobs);

			num_df_dpose = numJacobs.block<3, 7>(0, 0);
			num_df_dpoint = numJacobs.block<3, 3>(0, 7);
		}

		// Compare:
		EXPECT_NEAR(
			0,
			(df_dpoint.asEigen() - num_df_dpoint.asEigen()).array().abs().sum(),
			3e-3)
			<< "q1: " << q1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpoint: " << endl
			<< num_df_dpoint.asEigen() << endl
			<< "Implemented method: " << endl
			<< df_dpoint.asEigen() << endl
			<< "Error: " << endl
			<< df_dpoint.asEigen() - num_df_dpoint.asEigen() << endl;

		EXPECT_NEAR(
			0,
			(df_dpose.asEigen() - num_df_dpose.asEigen()).array().abs().sum(),
			3e-3)
			<< "q1: " << q1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpose: " << endl
			<< num_df_dpose.asEigen() << endl
			<< "Implemented method: " << endl
			<< df_dpose.asEigen() << endl
			<< "Error: " << endl
			<< df_dpose.asEigen() - num_df_dpose.asEigen() << endl;
	}

	static void func_normalizeJacob(
		const CVectorFixedDouble<4>& x, const double& dummy,
		CVectorFixedDouble<4>& Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		CQuaternionDouble q;
		for (int i = 0; i < 4; i++) q[i] = x[i];
		q.normalize();
		for (int i = 0; i < 4; i++) Y[i] = q[i];
	}

	void test_normalizeJacob(double yaw1, double pitch1, double roll1)
	{
		const CPose3D pp(0, 0, 0, yaw1, pitch1, roll1);
		CQuaternionDouble q1;
		pp.getAsQuaternion(q1);

		CMatrixFixed<double, 4, 4> df_dpose(UNINITIALIZED_MATRIX);
		q1.normalizationJacobian(df_dpose);

		// Numerical approximation:
		CMatrixFixed<double, 4, 4> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CVectorFixedDouble<4> x_mean;
			for (int i = 0; i < 4; i++) x_mean[i] = q1[i];

			double DUMMY = 0;
			CVectorFixedDouble<4> x_incrs;
			x_incrs.fill(1e-5);
			CMatrixDouble numJacobs;
			mrpt::math::estimateJacobian(
				x_mean,
				std::function<void(
					const CVectorFixedDouble<4>& x, const double& dummy,
					CVectorFixedDouble<4>& Y)>(&func_normalizeJacob),
				x_incrs, DUMMY, numJacobs);

			num_df_dpose = numJacobs.block<4, 4>(0, 0);
		}

		// Compare:
		EXPECT_NEAR(
			0,
			(df_dpose.asEigen() - num_df_dpose.asEigen()).array().abs().sum(),
			3e-3)
			<< "q1: " << q1 << endl
			<< "Numeric approximation of df_dpose: " << endl
			<< num_df_dpose.asEigen() << endl
			<< "Implemented method: " << endl
			<< df_dpose.asEigen() << endl
			<< "Error: " << endl
			<< df_dpose.asEigen() - num_df_dpose.asEigen() << endl;
	}
};

// Check yaw-pitch-roll [rad] vs its [qx,qy,qz,qw] representation:
static void quat_vs_YPR(
	double yaw, double pitch, double roll, double qx, double qy, double qz,
	double qw, const std::string& sRotMat)
{
	const double eps = 1e-4;

	// First, test Yaw-pitch-roll to Rot matrix:
	const mrpt::poses::CPose3D p(0, 0, 0, yaw, pitch, roll);
	mrpt::math::CMatrixDouble33 R_gt;
	std::stringstream sErr;
	if (!R_gt.fromMatlabStringFormat(sRotMat, sErr))
		GTEST_FAIL() << "Incorrect R_gt matrix: '" << sRotMat
					 << "'. Error: " << sErr.str() << "\n";
	EXPECT_EQ(R_gt.cols(), 3) << " for: sRotMat='" << sRotMat << "'\n";
	EXPECT_EQ(R_gt.rows(), 3) << " for: sRotMat='" << sRotMat << "'\n";

	EXPECT_NEAR(
		0,
		(R_gt.asEigen() - p.getRotationMatrix().asEigen())
			.array()
			.abs()
			.maxCoeff(),
		eps)
		<< "R_gt=\n"
		<< R_gt << "\np.R=\n"
		<< p.getRotationMatrix() << std::endl;

	// Convert to quat:
	const auto q_gt = mrpt::math::CQuaternionDouble(qw, qx, qy, qz);
	mrpt::math::CQuaternionDouble q;
	p.getAsQuaternion(q);

	if (std::abs(q.w() - q_gt.w()) > eps || std::abs(q.x() - q_gt.x()) > eps ||
		std::abs(q.y() - q_gt.y()) > eps || std::abs(q.z() - q_gt.z()) > eps)
	{
		GTEST_FAIL() << "q = " << q.asString()
					 << "\nExpected = " << q_gt.asString() << "\n";
	}

	auto R = q.rotationMatrix<mrpt::math::CMatrixDouble33>();
	EXPECT_NEAR(
		0,
		(R.asEigen() - p.getRotationMatrix().asEigen())
			.array()
			.abs()
			.maxCoeff(),
		eps)
		<< "q.R=\n"
		<< R << "\np.R=" << p.getRotationMatrix() << std::endl;
}

// Check yaw-pitch-roll vs its [qx,qy,qz,qw] representation:
TEST(QuatTests, Quat_vs_YPR)
{
	// Ground truth values obtained from:
	// https://www.andre-gaschler.com/rotationconverter/
	// Using: ZYX Euler angles convention

	quat_vs_YPR(
		0.0_deg, 0.0_deg, 0.0_deg,  // Input Yaw-pitch-roll
		0, 0, 0, 1,  // Expected quaternion
		"[  1.0000000  0.0000000  0.0000000; "
		"   0.0000000  1.0000000  0.0000000; "
		"   0.0000000  0.0000000  1.0000000 ]");
	quat_vs_YPR(
		90.0_deg, 0.0_deg, 0.0_deg,  // Input Yaw-pitch-roll
		0, 0, 0.7071068, 0.7071068,  // Expected quaternion
		"[  0.0000000 -1.0000000  0.0000000;"
		"   1.0000000  0.0000000  0.0000000;"
		"   0.0000000  0.0000000  1.0000000 ]");
	quat_vs_YPR(
		30.0_deg, 10.0_deg, 60.0_deg,  // Input Yaw-pitch-roll
		0.4615897, 0.2018243, 0.1811979, 0.8446119,  // Expected quaternion
		"[  0.8528686 -0.1197639  0.5082046;"
		"   0.4924039  0.5082046 -0.7065880;"
		"  -0.1736482  0.8528686  0.4924039 ]");
	quat_vs_YPR(
		-10.0_deg, -20.0_deg, -30.0_deg,  // Input Yaw-pitch-roll
		-0.2685358, -0.1448781, -0.1276794, 0.9437144,  // Expected quaternion
		"[  0.9254166  0.3187958 -0.2048741;"
		"  -0.1631759  0.8231729  0.5438381;"
		"   0.3420202 -0.4698463  0.8137977 ]");
	quat_vs_YPR(
		-179.9995949_deg, -90.0_deg, 0.0_deg,  // Input Yaw-pitch-roll
		0.7071068, 0, 0.7071068, -0.000005,  // Expected quaternion
		"[  0.0000000  0.0000071  1.0000000;"
		"  -0.0000071 -1.0000000  0.0000071;"
		"   1.0000000 -0.0000071  0.0000000 ]");
}

TEST_F(Pose3DQuatTests, FromYPRAndBack)
{
	test_fromYPRAndBack(1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg);
	test_fromYPRAndBack(1.0, 2.0, 3.0, 90.0_deg, 0.0_deg, 0.0_deg);
	test_fromYPRAndBack(1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg);
	test_fromYPRAndBack(1.0, 2.0, 3.0, 179.0_deg, 0.0_deg, 60.0_deg);
	test_fromYPRAndBack(1.0, 2.0, 3.0, -179.0_deg, 0.0_deg, 60.0_deg);
	test_fromYPRAndBack(1.0, 2.0, 3.0, 30.0_deg, 89.0_deg, 0.0_deg);
	test_fromYPRAndBack(1.0, 2.0, 3.0, 30.0_deg, -89.0_deg, 0.0_deg);
}

TEST_F(Pose3DQuatTests, UnaryInverse)
{
	test_unaryInverse(1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg);
	test_unaryInverse(1.0, 2.0, 3.0, 90.0_deg, 0.0_deg, 0.0_deg);
	test_unaryInverse(1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg);
	test_unaryInverse(1.0, 2.0, 3.0, 179.0_deg, 0.0_deg, 60.0_deg);
	test_unaryInverse(1.0, 2.0, 3.0, -179.0_deg, 0.0_deg, 60.0_deg);
	test_unaryInverse(1.0, 2.0, 3.0, 30.0_deg, 89.0_deg, 0.0_deg);
	test_unaryInverse(1.0, 2.0, 3.0, 30.0_deg, -89.0_deg, 0.0_deg);
}

TEST_F(Pose3DQuatTests, CopyOperator)
{
	test_copy(1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg);
	test_copy(1.0, 2.0, 3.0, 90.0_deg, 0.0_deg, 0.0_deg);
	test_copy(1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg);
	test_copy(1.0, 2.0, 3.0, 30.0_deg, -89.0_deg, 0.0_deg);
}

TEST_F(Pose3DQuatTests, Compose)
{
	test_compose(
		1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg, 2.0, -5.0, 8.0, 40.0_deg,
		-5.0_deg, 25.0_deg);

	test_compose(
		25.0, 2.0, 3.0, -30.0_deg, 90.0_deg, 0.0_deg, -10.0, 4.0, -8.0,
		20.0_deg, 9.0_deg, 0.0_deg);
}

TEST_F(Pose3DQuatTests, ComposeWithPoint)
{
	test_composePoint(1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_composePoint(1.0, 2.0, 3.0, 10.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_composePoint(1.0, 2.0, 3.0, 0.0_deg, 10.0_deg, 0.0_deg, 10, 11, 12);
	test_composePoint(1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 10.0_deg, 10, 11, 12);
	test_composePoint(
		1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg, 10.0, 20.0, 30.0);
	test_composePoint(
		1.0, 2.0, 3.0, 10.0_deg, -50.0_deg, -40.0_deg, -5.0, -15.0, 8.0);
}

TEST_F(Pose3DQuatTests, ComposeWithPointJacob)
{
	test_composePointJacob(
		1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_composePointJacob(
		1.0, 2.0, 3.0, 10.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_composePointJacob(
		1.0, 2.0, 3.0, 0.0_deg, 10.0_deg, 0.0_deg, 10, 11, 12);
	test_composePointJacob(
		1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 10.0_deg, 10, 11, 12);
	test_composePointJacob(
		1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg, 10.0, 20.0, 30.0);
	test_composePointJacob(
		1.0, 2.0, 3.0, 10.0_deg, -50.0_deg, -40.0_deg, -5.0, -15.0, 8.0);
}

TEST_F(Pose3DQuatTests, InvComposeWithPoint)
{
	test_invComposePoint(1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_invComposePoint(1.0, 2.0, 3.0, 10.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_invComposePoint(1.0, 2.0, 3.0, 0.0_deg, 10.0_deg, 0.0_deg, 10, 11, 12);
	test_invComposePoint(1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 10.0_deg, 10, 11, 12);
	test_invComposePoint(
		1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg, 10.0, 20.0, 30.0);
	test_invComposePoint(
		1.0, 2.0, 3.0, 10.0_deg, -50.0_deg, -40.0_deg, -5.0, -15.0, 8.0);
}

TEST_F(Pose3DQuatTests, InvComposeWithPointJacob)
{
	test_invComposePointJacob(0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg, 0, 0, 0);
	test_invComposePointJacob(0, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg, 1, 2, 3);
	test_invComposePointJacob(
		1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg, 0, 0, 0);
	test_invComposePointJacob(
		1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_invComposePointJacob(
		1.0, 2.0, 3.0, 10.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_invComposePointJacob(
		1.0, 2.0, 3.0, 0.0_deg, 10.0_deg, 0.0_deg, 10, 11, 12);
	test_invComposePointJacob(
		1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 10.0_deg, 10, 11, 12);
	test_invComposePointJacob(
		1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg, 10.0, 20.0, 30.0);
	test_invComposePointJacob(
		1.0, 2.0, 3.0, 10.0_deg, -50.0_deg, -40.0_deg, -5.0, -15.0, 8.0);
}

TEST_F(Pose3DQuatTests, ComposeInvComposePoint)
{
	test_composeAndInvComposePoint(
		1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_composeAndInvComposePoint(
		1.0, 2.0, 3.0, 10.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_composeAndInvComposePoint(
		1.0, 2.0, 3.0, 0.0_deg, 10.0_deg, 0.0_deg, 10, 11, 12);
	test_composeAndInvComposePoint(
		1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 10.0_deg, 10, 11, 12);
	test_composeAndInvComposePoint(
		1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg, 10.0, 20.0, 30.0);
	test_composeAndInvComposePoint(
		1.0, 2.0, 3.0, 10.0_deg, -50.0_deg, -40.0_deg, -5.0, -15.0, 8.0);
}

TEST_F(Pose3DQuatTests, ComposePoint_vs_CPose3D)
{
	test_composePoint_vs_CPose3D(
		1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_composePoint_vs_CPose3D(
		1.0, 2.0, 3.0, 10.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_composePoint_vs_CPose3D(
		1.0, 2.0, 3.0, 0.0_deg, 10.0_deg, 0.0_deg, 10, 11, 12);
	test_composePoint_vs_CPose3D(
		1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 10.0_deg, 10, 11, 12);
	test_composePoint_vs_CPose3D(
		1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg, 10.0, 20.0, 30.0);
	test_composePoint_vs_CPose3D(
		1.0, 2.0, 3.0, 10.0_deg, -50.0_deg, -40.0_deg, -5.0, -15.0, 8.0);
}

TEST_F(Pose3DQuatTests, InvComposePoint_vs_CPose3D)
{
	test_invComposePoint_vs_CPose3D(
		1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_invComposePoint_vs_CPose3D(
		1.0, 2.0, 3.0, 10.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_invComposePoint_vs_CPose3D(
		1.0, 2.0, 3.0, 0.0_deg, 10.0_deg, 0.0_deg, 10, 11, 12);
	test_invComposePoint_vs_CPose3D(
		1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 10.0_deg, 10, 11, 12);

	for (size_t i = 0; i < 10; i++)
	{
		std::vector<double> v(9);
		mrpt::random::getRandomGenerator().drawGaussian1DVector(v, 0, 1);
		test_invComposePoint_vs_CPose3D(
			v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
	}
}

TEST_F(Pose3DQuatTests, SphericalCoordsJacobian)
{
	test_sphericalCoords(1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_sphericalCoords(1.0, 2.0, 3.0, 10.0_deg, 0.0_deg, 0.0_deg, 10, 11, 12);
	test_sphericalCoords(1.0, 2.0, 3.0, 0.0_deg, 10.0_deg, 0.0_deg, 10, 11, 12);
	test_sphericalCoords(1.0, 2.0, 3.0, 0.0_deg, 0.0_deg, 10.0_deg, 10, 11, 12);
	test_sphericalCoords(
		1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg, 10.0, 20.0, 30.0);
	test_sphericalCoords(
		1.0, 2.0, 3.0, 10.0_deg, -50.0_deg, -40.0_deg, -5.0, -15.0, 8.0);
}

TEST_F(Pose3DQuatTests, NormalizationJacobian)
{
	test_normalizeJacob(0.0_deg, 0.0_deg, 0.0_deg);
	test_normalizeJacob(10.0_deg, 0.0_deg, 0.0_deg);
	test_normalizeJacob(0.0_deg, 10.0_deg, 0.0_deg);
	test_normalizeJacob(0.0_deg, 0.0_deg, 10.0_deg);
	test_normalizeJacob(-30.0_deg, 10.0_deg, 60.0_deg);
	test_normalizeJacob(10.0_deg, -50.0_deg, -40.0_deg);
}
