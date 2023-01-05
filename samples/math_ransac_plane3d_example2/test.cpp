/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/ransac.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random.h>
#include <mrpt/system/CTicTac.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

// Define as needed for your application:
using MyRansac = mrpt::math::RANSAC_Template<
	float,	// Numeric data type: float since pointclouds use float
	mrpt::maps::CPointsMap,	 // Dataset type (can be an abstract base type)
	mrpt::math::TPlane	// Model type to be estimated
	>;

namespace mrpt::math
{
// Overload for your custom dataset type:
size_t ransacDatasetSize(const mrpt::maps::CPointsMap& dataset)
{
	return dataset.size();
}
}  // namespace mrpt::math

void ransac3Dplane_fit(
	const mrpt::maps::CPointsMap& allData,
	const std::vector<size_t>& useIndices,
	vector<mrpt::math::TPlane>& fitModels)
{
	ASSERT_(useIndices.size() == 3);

	TPoint3D pt[3];
	for (int i = 0; i < 3; i++)
		allData.getPoint(useIndices[i], pt[i]);

	try
	{
		fitModels.resize(1);
		fitModels[0] = TPlane(pt[0], pt[1], pt[2]);
	}
	catch (exception&)
	{
		// If the three points are degenerated, an exception may be thrown in
		// the plane ctor
		fitModels.clear();
		return;
	}
}

void ransac3Dplane_distance(
	const mrpt::maps::CPointsMap& allData,
	const vector<mrpt::math::TPlane>& testModels,
	const double distanceThreshold, unsigned int& out_bestModelIndex,
	std::vector<size_t>& out_inlierIndices)
{
	ASSERT_(testModels.size() == 1);
	out_bestModelIndex = 0;
	const mrpt::math::TPlane& plane = testModels[0];

	const size_t N = allData.size();
	out_inlierIndices.clear();
	out_inlierIndices.reserve(100);
	for (size_t i = 0; i < N; i++)
	{
		mrpt::math::TPoint3D pt;
		allData.getPoint(i, pt);
		const double d = plane.distance(pt);
		if (d < distanceThreshold) out_inlierIndices.push_back(i);
	}
}

/** Return "true" if the selected points are a degenerate (invalid) case.
 */
bool ransac3Dplane_degenerate(
	const mrpt::maps::CPointsMap& allData,
	const std::vector<size_t>& useIndices)
{
	return false;
}

// ------------------------------------------------------
//				TestRANSAC
// ------------------------------------------------------
void TestRANSAC()
{
	auto& rng = getRandomGenerator();
	rng.randomize();

	// Generate random points:
	// ------------------------------------
	const size_t N_plane = 300;
	const size_t N_noise = 100;

	const double PLANE_EQ[4] = {1, -1, 1, -2};

	mrpt::maps::CSimplePointsMap data;
	data.reserve(N_plane + N_noise);

	for (size_t i = 0; i < N_plane; i++)
	{
		const double xx = rng.drawUniform(-3, 3);
		const double yy = rng.drawUniform(-3, 3);
		const double zz =
			-(PLANE_EQ[3] + PLANE_EQ[0] * xx + PLANE_EQ[1] * yy) / PLANE_EQ[2];
		data.insertPointFast(xx, yy, zz);
	}

	for (size_t i = 0; i < N_noise; i++)
	{
		data.insertPointFast(
			rng.drawUniform(-4, 4), rng.drawUniform(-4, 4),
			rng.drawUniform(-4, 4));
	}

	// Run RANSAC
	// ------------------------------------
	mrpt::math::TPlane best_model;
	std::vector<size_t> best_inliers;
	const double DIST_THRESHOLD = 0.05;

	CTicTac tictac;
	MyRansac myransac;

	myransac.setVerbosityLevel(mrpt::system::LVL_DEBUG);

	myransac.execute(
		data, &ransac3Dplane_fit, &ransac3Dplane_distance,
		&ransac3Dplane_degenerate, DIST_THRESHOLD,
		3,	// Minimum set of points
		best_inliers, best_model);

	cout << "Computation time: " << tictac.Tac() * 1000.0 << " ms\n";

	cout << "RANSAC finished: Best model: " << best_model.coefs[0] << " "
		 << best_model.coefs[1] << " " << best_model.coefs[2] << " "
		 << best_model.coefs[3] << endl;

	// Show GUI
	// --------------------------
	mrpt::gui::CDisplayWindow3D win("Set of points", 500, 500);
	opengl::COpenGLScene::Ptr scene = opengl::COpenGLScene::Create();

	scene->insert(opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1));
	scene->insert(opengl::stock_objects::CornerXYZ());

	opengl::CPointCloud::Ptr points = opengl::CPointCloud::Create();
	points->setColor(0, 0, 1);
	points->setPointSize(3);
	points->enableColorFromZ();
	points->loadFromPointsMap(&data);

	scene->insert(points);

	opengl::CTexturedPlane::Ptr glPlane =
		opengl::CTexturedPlane::Create(-4, 4, -4, 4);

	glPlane->setColor_u8(mrpt::img::TColor(0xff, 0x00, 0x00, 0x80));  // RGBA

	TPose3D glPlanePose;
	best_model.getAsPose3D(glPlanePose);
	glPlane->setPose(glPlanePose);

	scene->insert(glPlane);

	win.get3DSceneAndLock() = scene;
	win.unlockAccess3DScene();
	win.forceRepaint();

	win.waitForKey();
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestRANSAC();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
