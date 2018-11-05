/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::obs;

// ------------------------------------------------------
//				TestOpenGLVideo
// ------------------------------------------------------
void TestOpenGLVideo()
{
	// Show to the user a list of possible camera drivers and creates and open
	// the selected camera.
	cout << "Please, select the input video file or camera...\n";

	mrpt::hwdrivers::CCameraSensor::Ptr cam =
		mrpt::hwdrivers::prepareVideoSourceFromUserSelection();
	if (!cam) return;

	cout << "Video stream open OK\n";

	// Create 3D window:
	CDisplayWindow3D win("#1: Demo of image mode viewport", 320, 240);
	CDisplayWindow3D win2("#2: Demo of image mode viewport", 500, 400);

	// Win #1:
	//  Get the smart pointer to the main viewport object in this window:
	COpenGLViewport::Ptr gl_view_main;
	{
		COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();
		gl_view_main = theScene->getViewport("main");
		ASSERT_(gl_view_main);
		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();
	}

	// Win #2:
	//  Get the smart pointer to the main viewport object in this window:
	COpenGLViewport::Ptr gl_view_aux;
	{
		COpenGLScene::Ptr& theScene = win2.get3DSceneAndLock();
		theScene->insert(
			mrpt::make_aligned_shared<mrpt::opengl::CGridPlaneXY>());

		// Create small auxiliary viewport
		gl_view_aux = theScene->createViewport("aux");

		gl_view_aux->setViewportPosition(10, 10, 300, 200);
		gl_view_aux->setTransparent(true);

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win2.unlockAccess3DScene();
	}

	win.setPos(10, 10);
	win2.setPos(400, 100);

	cout << "Close any window to end.\n";
	while (win.isOpen() && win2.isOpen())
	{
		win.addTextMessage(5, 5, format("%.02fFPS", win.getRenderingFPS()));
		win2.addTextMessage(5, 5, format("%.02fFPS", win2.getRenderingFPS()));
		std::this_thread::sleep_for(1ms);

		// Grab new video frame:
		CObservation::Ptr obs = cam->getNextFrame();
		if (obs)
		{
			if (IS_CLASS(obs, CObservationImage))
			{
				CObservationImage::Ptr o =
					std::dynamic_pointer_cast<CObservationImage>(obs);
				win.get3DSceneAndLock();
				gl_view_main->setImageView(o->image);
				win.unlockAccess3DScene();
				win.repaint();

				win2.get3DSceneAndLock();
				gl_view_aux->setImageView_fast(o->image);
				win2.unlockAccess3DScene();
				win2.repaint();
			}
		}
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestOpenGLVideo();

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
