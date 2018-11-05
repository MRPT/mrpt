/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/img/CImage.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CSetOfTexturedTriangles.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/system/os.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::img;

#include <mrpt/examples_config.h>

string myDataDir(
	MRPT_EXAMPLES_BASE_DIRECTORY +
	string("opengl_textured_triangles_example/"));

// ------------------------------------------------------
//				TestDisplay3D
// ------------------------------------------------------
void TestDisplay3D()
{
	CDisplayWindow3D win("Test of CSetOfTexturedTriangles");
	COpenGLScene::Ptr& scene = win.get3DSceneAndLock();

	// Modify the scene:
	// ------------------------------------------------------
	{
		opengl::CGridPlaneXY::Ptr obj =
			mrpt::make_aligned_shared<opengl::CGridPlaneXY>(
				-20, 20, -20, 20, 0, 1);
		obj->setColor(0.4, 0.4, 0.4);
		scene->insert(obj);
	}
	{
		opengl::CAxis::Ptr obj = mrpt::make_aligned_shared<opengl::CAxis>();
		obj->setFrequency(5);
		obj->enableTickMarks();
		obj->setAxisLimits(-10, -10, -10, 10, 10, 10);
		scene->insert(obj);
	}
	{
		opengl::CSphere::Ptr obj = mrpt::make_aligned_shared<opengl::CSphere>();
		obj->setColor(0, 0, 1);
		obj->setRadius(0.3f);
		obj->setLocation(0, 0, 1);
		obj->setName("ball_1");
		scene->insert(obj);
	}
	{
		opengl::CSphere::Ptr obj = mrpt::make_aligned_shared<opengl::CSphere>();
		obj->setColor(1, 0, 0);
		obj->setRadius(0.3f);
		obj->setLocation(-1, -1, 1);
		obj->setName("ball_2");
		scene->insert(obj);
	}
	{
		CImage image, alpha;

		//		image.loadFromFile(myDataDir + string("texture.png"), 0); //
		// grayscale
		image.loadFromFile(myDataDir + string("texture.png"), 1);  // color
		alpha.loadFromFile(myDataDir + string("mask.png"), 0);  // transparency

		opengl::CSetOfTexturedTriangles::Ptr obj =
			mrpt::make_aligned_shared<opengl::CSetOfTexturedTriangles>();
		opengl::CSetOfTexturedTriangles::TTriangle tri;

		tri = opengl::CSetOfTexturedTriangles::TTriangle(
			opengl::CSetOfTexturedTriangles::TVertex(
				-2.0, -2.0, 0, 0, 256),  // 3D coord (x,y,z) Pixel coord (u,v)
			opengl::CSetOfTexturedTriangles::TVertex(2.0, -2.0, 0, 256, 256),
			opengl::CSetOfTexturedTriangles::TVertex(2.0, 2.0, 0, 256, 0));
		obj->insertTriangle(tri);

		tri = opengl::CSetOfTexturedTriangles::TTriangle(
			opengl::CSetOfTexturedTriangles::TVertex(-2.0, 2.0, 1, 0, 0),
			opengl::CSetOfTexturedTriangles::TVertex(2.0, 2.0, 0, 256, 0),
			opengl::CSetOfTexturedTriangles::TVertex(-2.0, -2.0, 0, 0, 256));
		obj->insertTriangle(tri);

		obj->setName("set");
		//		obj->assignImage(image);
		obj->assignImage(image, alpha);  // transparency
		scene->insert(obj);
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	win.setCameraElevationDeg(25.0f);

	while (!mrpt::system::os::kbhit() && win.isOpen())
	{
		// Move the scene:
		COpenGLScene::Ptr& scene = win.get3DSceneAndLock();

		opengl::CRenderizable::Ptr obj = scene->getByName("ball_1");
		obj->setLocation(
			obj->getPoseX() + cos(obj->getPoseY() / 2) * 0.05,
			obj->getPoseY() - sin(obj->getPoseX() / 2) * 0.09,
			obj->getPoseZ() - sin(obj->getPoseX() / 2) * 0.08);

		obj = scene->getByName("ball_2");
		obj->setLocation(
			obj->getPoseX() + cos(obj->getPoseY() / 2) * 0.05,
			obj->getPoseY() - sin(obj->getPoseX() / 2) * 0.09,
			obj->getPoseZ() - sin(obj->getPoseX() / 2) * 0.08);

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();

		// Update window:
		win.forceRepaint();

		std::this_thread::sleep_for(20ms);
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char* argv[])
{
	try
	{
		TestDisplay3D();
		return 0;
	}
	catch (const std::exception& e)
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
