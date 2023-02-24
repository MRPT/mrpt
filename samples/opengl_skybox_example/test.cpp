/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/img/TColor.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CSkyBox.h>
#include <mrpt/opengl/CSphere.h>

#include <iostream>
#include <thread>

// ------------------------------------------------------
//				TestSkyBox
// ------------------------------------------------------
void TestSkyBox()
{
	mrpt::gui::CDisplayWindow3D win("Example of MRPT skybox", 800, 600);

	mrpt::opengl::COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();

	// Modify the scene:
	// ------------------------------------------------------
	{
		auto obj = mrpt::opengl::CSkyBox::Create();

		theScene->insert(obj);
	}

	{
		auto obj = mrpt::opengl::CAxis::Create();
		obj->setFrequency(5);
		obj->enableTickMarks();
		obj->setAxisLimits(-10, -10, -10, 10, 10, 10);
		theScene->insert(obj);
	}

	{
		auto obj = mrpt::opengl::CBox::Create();
		obj->setWireframe(false);
		obj->setColor(1, 0, 0);
		obj->setLineWidth(3.0);
		obj->setPose(mrpt::math::TPose3D(1, 2, 3, 0.2, 0.3, 0.1));
		theScene->insert(obj);
	}

	{
		auto obj = mrpt::opengl::CSphere::Create();
		obj->setColor(0, 0, 1);
		obj->setRadius(0.3f);
		obj->setLocation(0, -2, 0);
		obj->setName("ball_1");
		theScene->insert(obj);
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	// Update window:
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
		TestSkyBox();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
