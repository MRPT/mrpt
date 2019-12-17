/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/img/CImage.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSphere.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::img;
using namespace std::literals;

// ------------------------------------------------------
//				TestDisplay3D
// ------------------------------------------------------
void TestDisplay3D()
{
	COpenGLScene scene;

	// Modify the scene:
	// ------------------------------------------------------
	{
		opengl::CGridPlaneXY::Ptr obj =
			opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
		obj->setColor(0.4, 0.4, 0.4);
		scene.insert(obj);
	}
	{
		opengl::CAxis::Ptr obj = opengl::CAxis::Create();
		obj->setFrequency(5);
		obj->enableTickMarks();
		obj->setAxisLimits(-10, -10, -10, 10, 10, 10);
		scene.insert(obj);
	}
	{
		opengl::CSphere::Ptr obj = opengl::CSphere::Create();
		obj->setColor(0, 0, 1);
		obj->setRadius(0.3);
		obj->setLocation(0, 0, 1);
		obj->setName("ball_1");
		scene.insert(obj);
	}
	{
		opengl::CSphere::Ptr obj = opengl::CSphere::Create();
		obj->setColor(1, 0, 0);
		obj->setRadius(0.3);
		obj->setLocation(-1, -1, 1);
		obj->setName("ball_2");
		scene.insert(obj);
	}

	CDisplayWindow win("output");

	int c = 0, width = 640, height = 480;

	CFBORender render(width, height);
	CImage frame(width, height, CH_RGB);

	{
		CCamera& camera = render.getCamera(scene);
		camera.setZoomDistance(50);
	}

	while (!mrpt::system::os::kbhit())
	{
		CRenderizable::Ptr obj = scene.getByName("ball_1");
		obj->setLocation(
			obj->getPoseX() + cos(obj->getPoseY() / 2) * 0.05,
			obj->getPoseY() - sin(obj->getPoseX() / 2) * 0.09,
			obj->getPoseZ() - sin(obj->getPoseX() / 2) * 0.08);

		obj = scene.getByName("ball_2");
		obj->setLocation(
			obj->getPoseX() + cos(obj->getPoseY() / 2) * 0.05,
			obj->getPoseY() - sin(obj->getPoseX() / 2) * 0.09,
			obj->getPoseZ() - sin(obj->getPoseX() / 2) * 0.08);

		// change the size
		if (++c > 100)
		{
			width = 800, height = 600;
			frame.resize(width, height, CH_RGB);
			render.resize(width, height);
		}

		// render the scene
		render.getFrame2(scene, frame);

		// show the redered image
		win.showImage(frame);

		std::this_thread::sleep_for(50ms);
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
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
