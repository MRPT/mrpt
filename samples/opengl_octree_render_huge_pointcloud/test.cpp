/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/random.h>

#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::img;

void insertRandomPoints_uniform(
	const size_t N, opengl::CPointCloud::Ptr& gl, const TPoint3D& p_min,
	const TPoint3D& p_max)
{
	auto& rnd = random::getRandomGenerator();
	for (size_t i = 0; i < N; i++)
		gl->insertPoint(
			rnd.drawUniform<float>(p_min.x, p_max.x),
			rnd.drawUniform<float>(p_min.y, p_max.y),
			rnd.drawUniform<float>(p_min.z, p_max.z));
}

void insertRandomPoints_screw(
	const size_t N, opengl::CPointCloud::Ptr& gl, const TPoint3D& p_start,
	const TPoint3D& p_end)
{
	TPoint3D d = p_end - p_start;
	d *= 1.0 / N;

	TPoint3D up(0, 0, 1);
	TPoint3D lat;
	mrpt::math::crossProduct3D(d, up, lat);
	lat *= 1.0 / lat.norm();

	TPoint3D p = p_start;
	for (size_t i = 0; i < N; i++)
	{
		const double ang = i * 0.01;
		TPoint3D pp = p + up * 30 * cos(ang) + lat * 30 * sin(ang);
		const auto ppf = TPoint3Df(pp);
		gl->insertPoint(ppf.x, ppf.y, ppf.z);
		p += d;
	}
}

void insertRandomPoints_gauss(
	const size_t N, opengl::CPointCloud::Ptr& gl, const TPoint3D& p_mean,
	const TPoint3D& p_stddevs)
{
	auto& rnd = random::getRandomGenerator();
	for (size_t i = 0; i < N; i++)
		gl->insertPoint(
			rnd.drawGaussian1D<float>(p_mean.x, p_stddevs.x),
			rnd.drawGaussian1D<float>(p_mean.y, p_stddevs.y),
			rnd.drawGaussian1D<float>(p_mean.z, p_stddevs.z));
}

// ------------------------------------------------------
//				TestOctreeRenderHugePointCloud
// ------------------------------------------------------
void TestOctreeRenderHugePointCloud()
{
	// Change this in your program as needed:
	// mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL(0.1f);

	CDisplayWindow3D win("Demo of MRPT's octree pointclouds", 640, 480);

	COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();

	// CPointCloud
	opengl::CPointCloud::Ptr gl_pointcloud = opengl::CPointCloud::Create();
	theScene->insert(gl_pointcloud);

	gl_pointcloud->setPointSize(3.0);
	gl_pointcloud->enableColorFromZ();

	// Set the list of all points:

	const double L = 1e3;

	cout << "Building point cloud...";
	cout.flush();

	for (int XX = -10; XX <= 10; XX++)
	{
		const double off_x = XX * 2 * L;

		for (int YY = -10; YY <= 10; YY++)
		{
			const double off_y = YY * 2 * L;

			insertRandomPoints_screw(
				1e4, gl_pointcloud, TPoint3D(off_x + 0, off_y + 0, 0),
				TPoint3D(off_x + L, off_y + 0, 500));

			insertRandomPoints_screw(
				1e4, gl_pointcloud, TPoint3D(off_x + L, off_y + 0, 500),
				TPoint3D(off_x + L, off_y + L, -500));

			insertRandomPoints_screw(
				1e4, gl_pointcloud, TPoint3D(off_x + L, off_y + L, -500),
				TPoint3D(off_x + 0, off_y + L, 500));

			insertRandomPoints_screw(
				1e4, gl_pointcloud, TPoint3D(off_x + 0, off_y + L, 500),
				TPoint3D(off_x + 0, off_y + 0, 0));
		}
	}

	cout << "Done.\n";
	cout.flush();

	printf("Point count: %e\n", (double)gl_pointcloud->size());

	// Draw the octree bounding boxes:
	mrpt::opengl::CSetOfObjects::Ptr gl_bb =
		mrpt::opengl::CSetOfObjects::Create();
	gl_pointcloud->octree_get_graphics_boundingboxes(*gl_bb);
	theScene->insert(gl_bb);

	// gl_pointcloud->octree_debug_dump_tree(std::cout);

	win.setCameraZoom(600);
	{
		mrpt::opengl::COpenGLViewport::Ptr view = theScene->getViewport("main");
		view->setViewportClipDistances(0.1f, 1e6f);
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();
	win.repaint();

	cout << "Close the window or press any key to end.\n";
	bool end = false;
	while (win.isOpen() && !end)
	{
		std::this_thread::sleep_for(5ms);

		if (win.keyHit())
		{
			switch (win.getPushedKey())
			{
				case 'q': end = true; break;
				case 'b': gl_bb->setVisibility(!gl_bb->isVisible()); break;
			};
		}

		// Update the texts on the gl display:
		string s = mrpt::format(
			"FPS=%5.02f | Rendered points=%.02e/%.02e (%.02f%%) | "
			"Visib.oct.nodes: %u/%u",
			win.getRenderingFPS(), (double)gl_pointcloud->getActuallyRendered(),
			(double)gl_pointcloud->size(),
			100 * double(gl_pointcloud->getActuallyRendered()) /
				double(gl_pointcloud->size()),
			(unsigned int)gl_pointcloud->octree_get_visible_nodes(),
			(unsigned int)gl_pointcloud->octree_get_node_count());

		win.get3DSceneAndLock();
		win.addTextMessage(5, 5, s, 0);
		win.addTextMessage(
			5, 35, "'b': switch bounding-boxes visible, 'q': quit", 1);
		win.unlockAccess3DScene();
		win.repaint();
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestOctreeRenderHugePointCloud();

		std::this_thread::sleep_for(500ms);

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
