/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/system/os.h>
#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;

// ------------------------------------------------------
//				TestDisplay3D
// ------------------------------------------------------
void TestDisplay3D()
{
	CDisplayWindow3D win("Example of 3D Scene Visualization - MRPT", 640, 480);

	COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();

	// Add a clone viewport:
	if (true)
	{
		COpenGLViewport::Ptr vi = theScene->createViewport("myClone");
		vi->setViewportPosition(0.7, 0.05, 0.28, 0.28);
		vi->setCloneView("main");
		vi->setTransparent(true);
		vi->getCamera().setAzimuthDegrees(45);
		vi->getCamera().setElevationDegrees(45);
		vi->getCamera().setZoomDistance(10);
	}

	// Modify the scene:
	// ------------------------------------------------------
	{
		opengl::CGridPlaneXY::Ptr obj =
			opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
		obj->setColor(0.4f, 0.4f, 0.4f);
		theScene->insert(obj);
	}

	{
		opengl::CAxis::Ptr obj = opengl::CAxis::Create();
		obj->setFrequency(5);
		obj->enableTickMarks();
		obj->setAxisLimits(-10, -10, -10, 10, 10, 10);
		theScene->insert(obj);
	}

	{
		opengl::CBox::Ptr obj = opengl::CBox::Create();
		obj->setWireframe(false);
		obj->setColor(1, 0, 0);
		obj->setLineWidth(3.0);
		obj->setPose(mrpt::math::TPose3D(10, 0, 0, 0.2, 0.3, 0.1));
		theScene->insert(obj);
	}

	{
		opengl::CSphere::Ptr obj = opengl::CSphere::Create();
		obj->setColor(0, 0, 1);
		obj->setRadius(0.3f);
		obj->setLocation(0, 0, 1);
		obj->setName("ball_1");
		theScene->insert(obj);
	}
	{
		opengl::CSphere::Ptr obj = opengl::CSphere::Create();
		obj->setColor(1, 0, 0);
		obj->setRadius(0.3f);
		obj->setLocation(-1, -1, 1);
		obj->setName("ball_2");
		theScene->insert(obj);
	}

	{
		opengl::CSphere::Ptr obj = opengl::CSphere::Create();
		obj->setColor(0, 1, 0);
		obj->setRadius(0.5);
		obj->setLocation(0, 0, 0);
		obj->setName("USER_MOUSE_PICK");
		theScene->insert(obj);
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	win.captureImagesStart();

	// Texts:
	win.addTextMessage(0.05, 0.05, "This is a 2D message");

	win.setCameraElevationDeg(25.0f);
	// win.setCameraProjective(false);

	cout << endl;
	cout << "Control with mouse or keyboard. Valid keys:" << endl;
	cout << "  ESC                        -> Exit" << endl;
	cout << "  Left/right cursor arrow    -> Camera azimuth" << endl;
	cout << endl;

	bool end = false;

	while (!end && win.isOpen())
	{
		// Move the scene:
		COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();

		opengl::CRenderizable::Ptr obj1 = theScene->getByName("ball_1");
		obj1->setLocation(
			obj1->getPoseX() + cos(obj1->getPoseY() / 2) * 0.05,
			obj1->getPoseY() - sin(obj1->getPoseX() / 2) * 0.09,
			obj1->getPoseZ() - sin(obj1->getPoseX() / 2) * 0.08);

		obj1 = theScene->getByName("ball_2");
		obj1->setLocation(
			obj1->getPoseX() + cos(obj1->getPoseY() / 2) * 0.05,
			obj1->getPoseY() - sin(obj1->getPoseX() / 2) * 0.09,
			obj1->getPoseZ() - sin(obj1->getPoseX() / 2) * 0.08);

		win.addTextMessage(
			0.02, 0.98,
			format(
				"ball#1 pos: %.02f %.02f %.02f ", obj1->getPoseX(),
				obj1->getPoseY(), obj1->getPoseZ()),
			mrpt::img::TColorf(0, 0, 1),
			10,  // An arbitrary ID to always overwrite the same, previous 2D
			// text message
			MRPT_GLUT_BITMAP_HELVETICA_12);

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();

		// Update window:
		win.forceRepaint();
		std::this_thread::sleep_for(10ms);

		// Grab frame:
		mrpt::img::CImage::Ptr img = win.getLastWindowImagePtr();
		if (img)
		{
			static int i = 0;
			const string s = format("GRAB_%06i.png", ++i);
			img->saveToFile(s);
			printf("Saved frame image to: %s \r", s.c_str());  // "\ r" is to
			// overwrite the
			// same line over
			// and over
			// again..
		}

		if (mrpt::system::os::kbhit()) end = true;
		if (win.keyHit())
		{
			mrptKeyModifier kmods;
			int key = win.getPushedKey(&kmods);
			printf(
				"Key pushed: %c (%i) - modifiers: 0x%04X\n", char(key), key,
				kmods);

			if (key == MRPTK_ESCAPE) end = true;

			if (key == MRPTK_RIGHT)
				win.setCameraAzimuthDeg(win.getCameraAzimuthDeg() + 5);
			if (key == MRPTK_LEFT)
				win.setCameraAzimuthDeg(win.getCameraAzimuthDeg() - 5);
		}
	};
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
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
