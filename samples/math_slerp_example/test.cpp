/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/**
 * slerp_demo
 * Execute a Spherical Linear Interpolation given 2 poses.
 */

#include <mrpt/math/slerp.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/img/TColor.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::img;

// ------------------------------------------------------
//				TestSLERP
// ------------------------------------------------------
void TestSLERP()
{
	CDisplayWindow3D win("Example of SLERP animation", 640, 480);

	COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();

	win.setCameraAzimuthDeg(-50);
	win.setCameraElevationDeg(40);
	win.setCameraZoom(19);
	win.setCameraPointingToPoint(2, 2, 0);

	// Modify the scene:
	// ------------------------------------------------------
	{
		opengl::CGridPlaneXY::Ptr obj =
			mrpt::make_aligned_shared<opengl::CGridPlaneXY>(
				-20, 20, -20, 20, 0, 1);
		obj->setColor(0.4, 0.4, 0.4);
		theScene->insert(obj);
	}

	// Initialize the start, end pose of the animation
	const TPose3D pose_a(0, 0, 0, DEG2RAD(0), DEG2RAD(0), DEG2RAD(0));
	const TPose3D pose_b(3, 4, 1, DEG2RAD(120), DEG2RAD(40), DEG2RAD(50));

	{
		// XYZ corner at A:
		opengl::CSetOfObjects::Ptr obj =
			opengl::stock_objects::CornerXYZSimple(1.0, 2.0);
		obj->setPose(pose_a);
		theScene->insert(obj);
	}
	{
		// XYZ corner at B:
		opengl::CSetOfObjects::Ptr obj =
			opengl::stock_objects::CornerXYZSimple(1.0, 2.0);
		obj->setPose(pose_b);
		theScene->insert(obj);
	}
	{
		// SLERP animated corner:
		opengl::CSetOfObjects::Ptr obj =
			opengl::stock_objects::CornerXYZSimple(1.0, 4.0);
		obj->setName("slerp_obj");
		obj->setPose(pose_a);
		theScene->insert(obj);
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	cout << "\n Close the window to exit.\n";

	mrpt::system::CTicTac tic;
	static const double MOVE_PERIOD = 1.0;
	static const double MOVE_PERIOD2 = 2 * MOVE_PERIOD;

	while (win.isOpen())
	{
		// Compute the time:
		double t = ::fmod(tic.Tac(), MOVE_PERIOD2);
		if (t < MOVE_PERIOD)
			t /= MOVE_PERIOD;
		else
			t = 1 - (t - MOVE_PERIOD) / MOVE_PERIOD;

		// SLERP & LERP interpolation:
		TPose3D pose_interp;
		mrpt::math::slerp(pose_a, pose_b, t, pose_interp);

		// Move the scene:
		COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();

		opengl::CRenderizable::Ptr obj1 = theScene->getByName("slerp_obj");
		obj1->setPose(pose_interp);

		// Show text:
		win.addTextMessage(
			5, 5, format("t=%.03f", t), TColorf(1, 1, 1), 0,
			MRPT_GLUT_BITMAP_TIMES_ROMAN_24);

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();

		// Update window:
		win.forceRepaint();
		std::this_thread::sleep_for(5ms);
	};
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestSLERP();
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
