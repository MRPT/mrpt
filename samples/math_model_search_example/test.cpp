/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/math/model_search.h>
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

struct Fit3DPlane
{
	typedef TPlane3D Model;
	typedef double Real;

	const std::vector<TPoint3D>& allData;

	Fit3DPlane(const std::vector<TPoint3D>& _allData) : allData(_allData) {}
	size_t getSampleCount(void) const { return allData.size(); }
	bool fitModel(const std::vector<size_t>& useIndices, TPlane3D& model) const
	{
		ASSERT_(useIndices.size() == 3);
		TPoint3D p1(allData[useIndices[0]]);
		TPoint3D p2(allData[useIndices[1]]);
		TPoint3D p3(allData[useIndices[2]]);

		try
		{
			model = TPlane(p1, p2, p3);
		}
		catch (exception&)
		{
			return false;
		}

		return true;
	}

	double testSample(size_t index, const TPlane3D& model) const
	{
		return model.distance(allData[index]);
	}
};

// ------------------------------------------------------
//				TestRANSAC
// ------------------------------------------------------
void TestRANSAC()
{
	getRandomGenerator().randomize(789);

	// Generate random points:
	// ------------------------------------
	const size_t N_plane = 300;
	const size_t N_noise = 100;

	const double PLANE_EQ[4] = {1, -1, 1, -2};

	std::vector<TPoint3D> data;
	for (size_t i = 0; i < N_plane; i++)
	{
		const double xx = getRandomGenerator().drawUniform(-3, 3);
		const double yy = getRandomGenerator().drawUniform(-3, 3);
		const double zz =
			-(PLANE_EQ[3] + PLANE_EQ[0] * xx + PLANE_EQ[1] * yy) / PLANE_EQ[2];
		data.push_back(TPoint3D(xx, yy, zz));
	}

	for (size_t i = 0; i < N_noise; i++)
	{
		TPoint3D& p = data[i];
		p += TPoint3D(
			getRandomGenerator().drawUniform(-4, 4),
			getRandomGenerator().drawUniform(-4, 4),
			getRandomGenerator().drawUniform(-4, 4));
	}

	// Run RANSAC
	// ------------------------------------
	TPlane3D best_model;
	std::vector<size_t> best_inliers;
	const double DIST_THRESHOLD = 0.2;

	CTicTac tictac;
	const size_t TIMES = 100;

	bool found = false;
	for (size_t iters = 0; iters < TIMES; iters++)
	{
		ModelSearch search;
		Fit3DPlane fit(data);
		found = search.geneticSingleModel(
			fit, 3, DIST_THRESHOLD, 10, 100, best_model, best_inliers);
	}
	cout << "Computation time (genetic): " << tictac.Tac() * 1000.0 / TIMES
		 << " ms" << endl;

	for (size_t iters = 0; iters < TIMES; iters++)
	{
		ModelSearch search;
		Fit3DPlane fit(data);
		found = search.ransacSingleModel(
			fit, 3, DIST_THRESHOLD, best_model, best_inliers);
	}
	cout << "Computation time (ransac): " << tictac.Tac() * 1000.0 / TIMES
		 << " ms" << endl;

	if (!found) return;

	//	cout << "RANSAC finished: Best model: " << best_model << endl;
	//	cout << "Best inliers: " << best_inliers << endl;

	// Show GUI
	// --------------------------
	mrpt::gui::CDisplayWindow3D win("Set of points", 500, 500);
	opengl::COpenGLScene::Ptr scene = opengl::COpenGLScene::Create();

	scene->insert(opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1));
	scene->insert(opengl::stock_objects::CornerXYZ());

	opengl::CPointCloud::Ptr points = opengl::CPointCloud::Create();
	points->setColor(0.0f, 0.0f, 1.0f);
	points->setPointSize(3);
	points->enableColorFromZ();
	points->setAllPoints(data);

	scene->insert(points);

	opengl::CTexturedPlane::Ptr glPlane =
		opengl::CTexturedPlane::Create(-4, 4, -4, 4);

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
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
