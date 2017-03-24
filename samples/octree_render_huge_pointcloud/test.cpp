/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/system/threads.h>
#include <mrpt/random.h>
#include <mrpt/math/geometry.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::utils;

void insertRandomPoints_uniform(
	const size_t N,
	opengl::CPointCloudPtr &gl,
	const TPoint3D &p_min,
	const TPoint3D &p_max)
{
	for (size_t i=0;i<N;i++)
		gl->insertPoint(
			random::randomGenerator.drawUniform(p_min.x,p_max.x),
			random::randomGenerator.drawUniform(p_min.y,p_max.y),
			random::randomGenerator.drawUniform(p_min.z,p_max.z) );
}

void insertRandomPoints_screw(
	const size_t N,
	opengl::CPointCloudPtr &gl,
	const TPoint3D &p_start,
	const TPoint3D &p_end)
{
	TPoint3D d = p_end-p_start;
	d*= 1.0/N;

	TPoint3D up(0,0,1);
	TPoint3D lat;
	mrpt::math::crossProduct3D(d,up, lat);
	lat*=1.0/lat.norm();

	TPoint3D p = p_start;
	for (size_t i=0;i<N;i++)
	{
		const double ang = i*0.01;
		TPoint3D pp = p + up * 30*cos(ang) + lat * 30*sin(ang);
		gl->insertPoint(pp.x,pp.y,pp.z);
		p+=d;
	}
}

void insertRandomPoints_gauss(
	const size_t N,
	opengl::CPointCloudPtr &gl,
	const TPoint3D &p_mean,
	const TPoint3D &p_stddevs)
{
	for (size_t i=0;i<N;i++)
		gl->insertPoint(
			random::randomGenerator.drawGaussian1D(p_mean.x,p_stddevs.x),
			random::randomGenerator.drawGaussian1D(p_mean.y,p_stddevs.y),
			random::randomGenerator.drawGaussian1D(p_mean.z,p_stddevs.z) );
}


// ------------------------------------------------------
//				TestOctreeRenderHugePointCloud
// ------------------------------------------------------
void TestOctreeRenderHugePointCloud()
{
	// Change this in your program as needed:
	//mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL = 0.1f;


	CDisplayWindow3D	win("Demo of MRPT's octree pointclouds",640,480);

	COpenGLScenePtr &theScene = win.get3DSceneAndLock();

	// CPointCloud
	opengl::CPointCloudPtr gl_pointcloud = opengl::CPointCloud::Create();
	theScene->insert( gl_pointcloud );

	gl_pointcloud->setPointSize(3.0);
	gl_pointcloud->enablePointSmooth();
	gl_pointcloud->enableColorFromZ();

	// Set the list of all points:

	const double L = 1e3;

	cout << "Building point cloud..."; cout.flush();

	for (int XX=-10;XX<=10;XX++)
	{
		const double off_x = XX * 2*L;

		for (int YY=-10;YY<=10;YY++)
		{
			const double off_y = YY * 2*L;

			insertRandomPoints_screw(
				1e4,
				gl_pointcloud,
				TPoint3D(off_x+0,off_y+0,0),
				TPoint3D(off_x+L,off_y+0,500)
				);

			insertRandomPoints_screw(
				1e4,
				gl_pointcloud,
				TPoint3D(off_x+L,off_y+0,500),
				TPoint3D(off_x+L,off_y+L,-500)
				);

			insertRandomPoints_screw(
				1e4,
				gl_pointcloud,
				TPoint3D(off_x+L,off_y+L,-500),
				TPoint3D(off_x+0,off_y+L,500)
				);

			insertRandomPoints_screw(
				1e4,
				gl_pointcloud,
				TPoint3D(off_x+0,off_y+L,500),
				TPoint3D(off_x+0,off_y+0,0)
				);

		}
	}

	cout << "Done.\n"; cout.flush();

	printf("Point count: %e\n", (double)gl_pointcloud->size());

	// Draw the octree bounding boxes:
	mrpt::opengl::CSetOfObjectsPtr gl_bb = mrpt::opengl::CSetOfObjects::Create();
	gl_pointcloud->octree_get_graphics_boundingboxes(*gl_bb);
	theScene->insert( gl_bb );

	//gl_pointcloud->octree_debug_dump_tree(std::cout);

	win.setCameraZoom(600);
	{
		mrpt::opengl::COpenGLViewportPtr view=theScene->getViewport("main");
		view->setViewportClipDistances(0.1, 1e6);
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();
	win.repaint();

	cout << "Close the window or press any key to end.\n";
	bool end=false;
	while (win.isOpen() && !end)
	{
		mrpt::system::sleep(5);

		if (win.keyHit())
		{
			switch (win.getPushedKey())
			{
			case 'q':
				end=true;
				break;
			case 'b':
				gl_bb->setVisibility( !gl_bb->isVisible() );
				break;
			};
		}

		// Update the texts on the gl display:
		string s = mrpt::format("FPS=%5.02f | Rendered points=%.02e/%.02e (%.02f%%) | Visib.oct.nodes: %u/%u",
			win.getRenderingFPS(),
			(double)gl_pointcloud->getActuallyRendered(),
			(double)gl_pointcloud->size(),
			100*double(gl_pointcloud->getActuallyRendered())/double(gl_pointcloud->size()),
			(unsigned int)gl_pointcloud->octree_get_visible_nodes(),
			(unsigned int)gl_pointcloud->octree_get_node_count()
			);

		win.get3DSceneAndLock();
			win.addTextMessage(5,5, s, TColorf(1,1,1),0, MRPT_GLUT_BITMAP_HELVETICA_18 );
			win.addTextMessage(5,35, "'b': switch bounding-boxes visible, 'q': quit", TColorf(1,1,1),1, MRPT_GLUT_BITMAP_HELVETICA_18 );
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

		mrpt::system::sleep(500);

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
