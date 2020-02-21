/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/transform_gaussian.h>
#include <mrpt/math/utils.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/system/CTicTac.h>
#include <Eigen/Dense>
#include <iostream>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

// Example non-linear function for SUT
//   f: R^5   => R^3
void myFun1(
	const CVectorFixedDouble<3>& x, const double& user_param,
	CVectorFixedDouble<3>& y)
{
	y[0] = cos(x[0]) * exp(x[1]) + x[0];
	y[1] = x[1] / (1 + square(x[0]));
	y[2] = x[2] / (1 + square(x[2])) + sin(x[1] * x[0]);
}

/* ------------------------------------------------------------------------
					Test_SUT: Scaled Unscented Transform
   ------------------------------------------------------------------------ */
void Test_SUT()
{
	// Inputs:
	const double x0[] = {1.8, 0.7, 0.9};
	// clang-format off
	const double x0cov[] = {
	    0.049400,  0.011403,  -0.006389,
	    0.011403,  0.026432,  0.005382,
	    -0.006389, 0.005382,  0.063268};
	// clang-format on

	const CVectorFixedDouble<3> x_mean(x0);
	const CMatrixFixed<double, 3, 3> x_cov(x0cov);
	const double dumm = 0;

	// Outputs:
	CVectorFixedDouble<3> y_mean;
	CMatrixDouble33 y_cov;

	// Do SUT:
	CTicTac tictac;
	size_t N = 10000;

	tictac.Tic();
	for (size_t i = 0; i < N; i++)
		mrpt::math::transform_gaussian_unscented(
			x_mean, x_cov, myFun1,
			dumm,  // fixed parameter: not used in this example
			y_mean, y_cov);

	cout << "SUT: Time (ms): " << 1e3 * tictac.Tac() / N << endl;

	// Print:
	cout << " ======= Scaled Unscented Transform ======== " << endl;
	cout << "y_mean: " << y_mean << endl;
	cout << "y_cov: " << endl << y_cov << endl << endl;

	// 3D view:
	mrpt::opengl::COpenGLScene::Ptr scene =
		mrpt::opengl::COpenGLScene::Create();
	scene->insert(opengl::CGridPlaneXY::Create(-10, 10, -10, 10, 0, 1));

	{
		opengl::CEllipsoid3D::Ptr el = opengl::CEllipsoid3D::Create();
		el->enableDrawSolid3D(false);
		el->setLocation(y_mean[0], y_mean[1], y_mean[2]);
		el->setCovMatrix(y_cov);
		el->setColor(0, 0, 1);
		scene->insert(el);
	}

	// Do Montecarlo for comparison:
	N = 10;

	std::vector<CVectorFixedDouble<3>> MC_samples;

	tictac.Tic();
	for (size_t i = 0; i < N; i++)
		mrpt::math::transform_gaussian_montecarlo(
			x_mean, x_cov, myFun1,
			dumm,  // fixed parameter: not used in this example
			y_mean, y_cov,
			5e5,  // Samples
			&MC_samples  // we want the samples.
		);

	cout << "MC: Time (ms): " << 1e3 * tictac.Tac() / N << endl;

	CVectorDouble MC_y[3];

	for (int i = 0; i < 3; i++)
		extractColumnFromVectorOfVectors(i, MC_samples, MC_y[i]);

	{
		auto el = opengl::CEllipsoid3D::Create();
		el->enableDrawSolid3D(false);
		el->setLocation(y_mean[0], y_mean[1], y_mean[2]);
		el->setCovMatrix(y_cov);
		el->setColor(0, 1, 0);
		scene->insert(el);
	}

	// Print:
	cout << " ======= Montecarlo Transform ======== " << endl;
	cout << "y_mean: " << y_mean << endl;
	cout << "y_cov: " << endl << y_cov << endl;

	// Do Linear for comparison:
	N = 100;

	CVectorFixedDouble<3> x_incrs;
	x_incrs.fill(1e-6);

	tictac.Tic();
	for (size_t i = 0; i < N; i++)
		mrpt::math::transform_gaussian_linear(
			x_mean, x_cov, myFun1,
			dumm,  // fixed parameter: not used in this example
			y_mean, y_cov, x_incrs);

	cout << "LIN: Time (ms): " << 1e3 * tictac.Tac() / N << endl;

	// Print:
	cout << " ======= Linear Transform ======== " << endl;
	cout << "y_mean: " << y_mean << endl;
	cout << "y_cov: " << endl << y_cov << endl;

	{
		auto el = opengl::CEllipsoid3D::Create();
		el->enableDrawSolid3D(false);
		el->setLocation(y_mean[0], y_mean[1], y_mean[2]);
		el->setCovMatrix(y_cov);
		el->setColor(1, 0, 0);
		scene->insert(el);
	}

	mrpt::gui::CDisplayWindow3D win(
		"Comparison SUT (blue), Linear (red), MC (green)", 400, 300);
	win.get3DSceneAndLock() = scene;
	win.unlockAccess3DScene();

	win.setCameraPointingToPoint(y_mean[0], y_mean[1], y_mean[2]);
	win.setCameraZoom(5.0);

	// MC-based histograms:
	mrpt::gui::CDisplayWindowPlots::Ptr winHistos[3];

	for (int i = 0; i < 3; i++)
	{
		winHistos[i] = mrpt::gui::CDisplayWindowPlots::Create(
			format("MC-based histogram of the %i dim", i), 300, 150);

		std::vector<double> X;
		std::vector<double> H = mrpt::math::histogram(
			MC_y[i], MC_y[i].minCoeff(), MC_y[i].maxCoeff(), 40, true, &X);

		winHistos[i]->plot(X, H, "b");
		winHistos[i]->axis_fit();
	}

	win.forceRepaint();

	cout << endl << "Press any key to exit" << endl;
	win.waitForKey();
}

// Calibration of SUT parameters for Quat -> 3D pose
// -----------------------------------------------------------

static void aux_posequat2poseypr(
	const CVectorFixedDouble<7>& x, const double& dummy,
	CVectorFixedDouble<6>& y)
{
	const CPose3DQuat p(
		x[0], x[1], x[2],
		mrpt::math::CQuaternionDouble(x[3], x[4], x[5], x[6]));
	const CPose3D p2 = CPose3D(p);
	for (int i = 0; i < 6; i++) y[i] = p2[i];
	// cout << "p2: " << y[3] << endl;
}

void TestCalibrate_pose2quat()
{
	// Take a 7x7 representation:
	CPose3DQuatPDFGaussian o;
	o.mean = CPose3DQuat(CPose3D(1.0, 2.0, 3.0, -30.0_deg, 10.0_deg, 60.0_deg));
	// o.mean = CPose3D(1.0,2.0,3.0, 00.0_deg,90.0_deg,0.0_deg);

	CMatrixFixed<double, 7, 1> v;
	mrpt::random::getRandomGenerator().drawGaussian1DMatrix(v);
	v *= 1e-3;
	o.cov.matProductOf_AAt(v);  // COV = v*vt
	for (int i = 0; i < 7; i++) o.cov(i, i) += 0.01;

	o.cov(0, 1) = o.cov(1, 0) = 0.007;

	cout << "p1quat: " << endl << o << endl;

	// Use UT transformation:
	//   f: R^7 => R^6
	const CVectorFixedDouble<7> x_mean(o.mean);
	CVectorFixedDouble<6> y_mean;
	static const bool elements_do_wrapPI[6] = {
		false, false, false, true, true, true};  // xyz yaw pitch roll

	static const double dummy = 0;
	// MonteCarlo:
	CVectorFixedDouble<6> MC_y_mean;
	CMatrixDouble66 MC_y_cov;
	mrpt::math::transform_gaussian_montecarlo(
		x_mean, o.cov, aux_posequat2poseypr,
		dummy,  // fixed parameter: not used in this example
		MC_y_mean, MC_y_cov, 500);
	cout << "MC: " << endl
		 << MC_y_mean << endl
		 << endl
		 << MC_y_cov << endl
		 << endl;

	// SUT:

	CPose3DPDFGaussian p_ypr;

	// double  = 1e-3;
	// alpha = x_mean.size()-3;

	mrpt::math::transform_gaussian_unscented(
		x_mean, o.cov, aux_posequat2poseypr, dummy, y_mean, p_ypr.cov,
		elements_do_wrapPI,
		1e-3,  // alpha
		0,  // K
		2.0  // beta
	);

	cout << "SUT: " << endl
		 << y_mean << endl
		 << endl
		 << p_ypr.cov << endl
		 << endl;
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		Test_SUT();

		// TestCalibrate_pose2quat();

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!");
		return -1;
	}
}
