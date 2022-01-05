/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/random/RandomGenerators.h>

#include <Eigen/Dense>	// for 6x8 fixed-matrices

#include "common.h"

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace std;

// ------------------------------------------------------
//				Benchmark Poses
// ------------------------------------------------------

// 3D ======================
double poses_test_compose3D(int a1, int a2)
{
	const long N = 100000;
	CTicTac tictac;

	CPose3D a(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg);
	CPose3D b(8.0, -5.0, -1.0, -40.0_deg, 10.0_deg, -45.0_deg);

	CPose3D p;
	for (long i = 0; i < N; i++)
	{
		p = a + b;
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", p.x()));
	return T;
}

double poses_test_compose3D2(int a1, int a2)
{
	const long N = 100000;
	CTicTac tictac;

	CPose3D a(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg);
	CPose3D b(8.0, -5.0, -1.0, -40.0_deg, 10.0_deg, -45.0_deg);

	CPose3D p;
	for (long i = 0; i < N; i++)
	{
		p.composeFrom(a, b);
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", p.x()));
	return T;
}

double poses_test_compose3Dpoint(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose3D a(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg);
	CPoint3D b(8.0, -5.0, -1.0);

	CPoint3D p;
	for (long i = 0; i < N; i++)
	{
		p = a + b;
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", p.x()));
	return T;
}

double poses_test_compose3Dpoint2(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose3D a(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg);
	CPoint3D b(8.0, -5.0, -1.0);

	double x, y, z;
	for (long i = 0; i < N; i++)
	{
		a.composePoint(b.x(), b.y(), b.z(), x, y, z);
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", x));
	return T;
}

double poses_test_compose3Dpoint3(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose3D a(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg);
	CPoint3D b(8.0, -5.0, -1.0);

	double x, y, z;
	mrpt::math::CMatrixFixed<double, 3, 3> df_dpoint;
	mrpt::math::CMatrixFixed<double, 3, 6> df_dpose;
	mrpt::math::CMatrixFixed<double, 3, 6> df_dse3;

	for (long i = 0; i < N; i++)
	{
		a.composePoint(
			b.x(), b.y(), b.z(), x, y, z, df_dpoint, df_dpose, df_dse3);
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", x));
	return T;
}

double poses_test_invcompose3Dpoint(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose3D a(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg);
	CPoint3D b(8.0, -5.0, -1.0);

	CPoint3D p;
	for (long i = 0; i < N; i++)
	{
		p = b - a;
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", p.x()));
	return T;
}

double poses_test_invcompose3Dpoint2(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose3D a(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg);
	CPoint3D b(8.0, -5.0, -1.0);

	double x, y, z;
	for (long i = 0; i < N; i++)
	{
		a.inverseComposePoint(b.x(), b.y(), b.z(), x, y, z);
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", x));
	return T;
}

// 2D =============

double poses_test_compose2D(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose2D a(1.0, 2.0, 10.0_deg);
	CPose2D b(8.0, -5.0, -40.0_deg);

	CPose2D p;
	for (long i = 0; i < N; i++)
	{
		p = a + b;
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", p.x()));
	return T;
}

double poses_test_compose2D2(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose2D a(1.0, 2.0, 10.0_deg);
	CPose2D b(8.0, -5.0, -40.0_deg);

	CPose2D p;
	for (long i = 0; i < N; i++)
	{
		p.composeFrom(a, b);
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", p.x()));
	return T;
}

double poses_test_compose2Dpoint(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose2D a(1.0, 2.0, 10.0_deg);
	CPoint2D b(8.0, -5.0);

	CPoint2D p;
	for (long i = 0; i < N; i++)
	{
		p = a + b;
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", p.x()));
	return T;
}

double poses_test_compose2Dpoint2(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose2D a(1.0, 2.0, 10.0_deg);
	CPoint2D b(8.0, -5.0);

	double x, y;
	for (long i = 0; i < N; i++)
	{
		a.composePoint(b.x(), b.y(), x, y);
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", x));
	return T;
}

// 3D QUAT ======================
double poses_test_compose3DQuat(int a1, int a2)
{
	const long N = 100000;
	CTicTac tictac;

	CPose3DQuat a(CPose3D(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg));
	CPose3DQuat b(CPose3D(8.0, -5.0, -1.0, -40.0_deg, 10.0_deg, -45.0_deg));

	CPose3DQuat p;
	for (long i = 0; i < N; i++)
	{
		p = a + b;
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", p.x()));
	return T;
}
double poses_test_compose3DQuat2(int a1, int a2)
{
	const long N = 100000;
	CTicTac tictac;

	CPose3DQuat a(CPose3D(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg));
	CPose3DQuat b(CPose3D(8.0, -5.0, -1.0, -40.0_deg, 10.0_deg, -45.0_deg));

	CPose3DQuat p;
	for (long i = 0; i < N; i++)
	{
		p.composeFrom(a, b);
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", p.x()));
	return T;
}
double poses_test_compose3DQuatpoint(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose3DQuat a(CPose3D(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg));
	CPoint3D b(8.0, -5.0, -1.0);

	CPoint3D p;
	for (long i = 0; i < N; i++)
	{
		p = a + b;
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", p.x()));
	return T;
}

double poses_test_compose3DQuatpoint2(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose3DQuat a(CPose3D(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg));
	CPoint3D b(8.0, -5.0, -1.0);

	double x, y, z;
	for (long i = 0; i < N; i++)
	{
		a.composePoint(b.x(), b.y(), b.z(), x, y, z);
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", x));
	return T;
}

double poses_test_invcompose3DQuatpoint2(int a1, int a2)
{
	const long N = 500000;
	CTicTac tictac;

	CPose3DQuat a(CPose3D(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg));
	CPoint3D b(8.0, -5.0, -1.0);

	double x, y, z;
	for (long i = 0; i < N; i++)
	{
		a.inverseComposePoint(b.x(), b.y(), b.z(), x, y, z);
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", x));
	return T;
}

// CONVERSIONS
double poses_test_convert_ypr_quat(int a1, int a2)
{
	const long N = 1000000;
	CTicTac tictac;

	CPose3D a(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg);

	double x;
	for (long i = 0; i < N; i++)
	{
		CPose3DQuat q(a);
		x = q.x();
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", x));
	return T;
}

double poses_test_convert_quat_ypr(int a1, int a2)
{
	const long N = 1000000;
	CTicTac tictac;

	CPose3DQuat a(CPose3D(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg));

	double x;
	for (long i = 0; i < N; i++)
	{
		CPose3D p(a);
		x = p.x();
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", x));
	return T;
}

// CONVERSIONS PDF Gauss
double poses_test_convert_ypr_quat_pdf(int a1, int a2)
{
	if (a1 >= 0) mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION(a1 != 0);
	const long N = 10000;

	CPose3D a_mean(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg);
	CMatrixDouble66 a_cov;

	{
		CMatrixFixed<double, 6, 8> v;
		mrpt::random::getRandomGenerator().randomize(1234);
		mrpt::random::getRandomGenerator().drawGaussian1DMatrix(v);
		v.asEigen() *= 0.1;
		a_cov.matProductOf_AAt(v);	// COV = v*vt
		for (int i = 3; i < 6; i++)
			a_cov(i, i) += square(2.0_deg);
	}

	CPose3DPDFGaussian a(a_mean, a_cov);

	double x;
	CTicTac tictac;
	for (long i = 0; i < N; i++)
	{
		CPose3DQuatPDFGaussian q(a);
		x = q.mean.x();
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", x));
	CPose3DQuatPDFGaussian q(a);
	// cout << q.cov << endl;
	return T;
}

double poses_test_convert_quat_ypr_pdf(int a1, int a2)
{
	if (a1 >= 0) mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION(a1 != 0);

	const long N = 2000;

	CPose3D a_mean(1.0, 2.0, 3.0, 10.0_deg, 50.0_deg, -30.0_deg);
	CMatrixDouble66 a_cov;
	{
		CMatrixFixed<double, 6, 8> v;
		mrpt::random::getRandomGenerator().randomize(1234);
		mrpt::random::getRandomGenerator().drawGaussian1DMatrix(v);
		v.asEigen() *= 0.1;
		a_cov.matProductOf_AAt(v);	// COV = v*vt
		for (int i = 3; i < 6; i++)
			a_cov(i, i) += square(2.0_deg);
	}
	CPose3DPDFGaussian a(a_mean, a_cov);

	CPose3DQuatPDFGaussian b = CPose3DQuatPDFGaussian(a);

	double x;
	CTicTac tictac;
	for (long i = 0; i < N; i++)
	{
		CPose3DPDFGaussian q(b);
		x = q.mean.x();
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", x));
	CPose3DPDFGaussian q(b);
	// cout << q.cov << endl;
	return T;
}

// ------------------------------------------------------
// register_tests_poses
// ------------------------------------------------------
void register_tests_poses()
{
	getRandomGenerator().randomize(1234);

	lstTests.emplace_back("poses: CPose3D (+) CPose3D", poses_test_compose3D);
	lstTests.emplace_back(
		"poses: CPose3D.composeFrom()", poses_test_compose3D2);
	lstTests.emplace_back(
		"poses: CPose3D (+) CPoint3D", poses_test_compose3Dpoint);
	lstTests.emplace_back(
		"poses: CPose3D.composePoint()", poses_test_compose3Dpoint2);
	lstTests.emplace_back(
		"poses: CPose3D.composePoint()+Jacobs", poses_test_compose3Dpoint3);

	lstTests.emplace_back(
		"poses: CPoint3D (-) CPose3D", poses_test_invcompose3Dpoint);
	lstTests.emplace_back(
		"poses: CPose3D.inverseComposePoint()", poses_test_invcompose3Dpoint2);

	lstTests.emplace_back("poses: CPose2D (+) CPose2D", poses_test_compose2D);
	lstTests.emplace_back(
		"poses: CPose2D.composeFrom()", poses_test_compose2D2);
	lstTests.emplace_back(
		"poses: CPose2D (+) CPoint2D", poses_test_compose2Dpoint);
	lstTests.emplace_back(
		"poses: CPose2D.composePoint()", poses_test_compose2Dpoint2);

	lstTests.emplace_back(
		"poses: CPose3DQuat (+) CPose3DQuat", poses_test_compose3DQuat);
	lstTests.emplace_back(
		"poses: CPose3DQuat.composeFrom()", poses_test_compose3DQuat2);
	lstTests.emplace_back(
		"poses: CPose3DQuat (+) CPoint3D", poses_test_compose3DQuatpoint);
	lstTests.emplace_back(
		"poses: CPose3DQuat.composePoint()", poses_test_compose3DQuatpoint2);

	lstTests.emplace_back(
		"poses: CPose3DQuat.invcomposePoint()",
		poses_test_invcompose3DQuatpoint2);

	lstTests.emplace_back(
		"poses: Conv CPose3DQuat <- CPose3D", poses_test_convert_quat_ypr);
	lstTests.emplace_back(
		"poses: Conv CPose3D -> CPose3DQuat", poses_test_convert_ypr_quat);

	lstTests.emplace_back(
		"poses: Conv CPose3DQuat Gauss <- CPose3D Gauss (DEF)",
		poses_test_convert_quat_ypr_pdf, -1);
	lstTests.emplace_back(
		"poses: Conv CPose3DQuat Gauss <- CPose3D Gauss (Lin)",
		poses_test_convert_quat_ypr_pdf, 0);
	lstTests.emplace_back(
		"poses: Conv CPose3DQuat Gauss <- CPose3D Gauss (SUT)",
		poses_test_convert_quat_ypr_pdf, 1);
	lstTests.emplace_back(
		"poses: Conv CPose3D Gauss <- CPose3DQuat Gauss (DEF)",
		poses_test_convert_ypr_quat_pdf, -1);
	lstTests.emplace_back(
		"poses: Conv CPose3D Gauss <- CPose3DQuat Gauss (Lin)",
		poses_test_convert_ypr_quat_pdf, 0);
	lstTests.emplace_back(
		"poses: Conv CPose3D Gauss <- CPose3DQuat Gauss (SUT)",
		poses_test_convert_ypr_quat_pdf, 1);
}
