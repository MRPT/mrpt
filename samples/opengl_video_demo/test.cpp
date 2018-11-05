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
	CDisplayWindow3D win(
		"Demo of video textures with MRPT's OpenGL objects", 640, 480);

	// XY Grid
	opengl::CGridPlaneXY::Ptr gl_ground =
		mrpt::make_aligned_shared<opengl::CGridPlaneXY>(-7, 7, -7, 7, 0, 1);
	gl_ground->setColor(0.7, 0.7, 0.7);

	// An opengl plane with the video texture
	opengl::CTexturedPlane::Ptr gl_plane1 =
		mrpt::make_aligned_shared<opengl::CTexturedPlane>(
			0, 1, 0, 0.75);  // 4/3 aspect ratio
	opengl::CTexturedPlane::Ptr gl_plane2 =
		mrpt::make_aligned_shared<opengl::CTexturedPlane>(0, 1, 0, 0.75);
	opengl::CTexturedPlane::Ptr gl_plane3 =
		mrpt::make_aligned_shared<opengl::CTexturedPlane>(0, 1, 0, 0.75);

	gl_plane1->setPose(
		mrpt::poses::CPose3D(0, 0, 1, DEG2RAD(0), DEG2RAD(0), DEG2RAD(-90)));
	gl_plane2->setPose(
		mrpt::poses::CPose3D(1, 0, 1, DEG2RAD(120), DEG2RAD(0), DEG2RAD(-90)));
	gl_plane3->setPose(
		mrpt::poses::CPose3D(0, 0, 1, DEG2RAD(60), DEG2RAD(0), DEG2RAD(-90)));

	win.setCameraZoom(5);

	// Insert objects in scene:
	{
		COpenGLScene::Ptr& theScene = win.get3DSceneAndLock();
		theScene->insert(gl_ground);
		theScene->insert(gl_plane1);
		theScene->insert(gl_plane2);
		theScene->insert(gl_plane3);
		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();
	}
	win.repaint();

	cout << "Close the window to end.\n";
	while (win.isOpen())
	{
		win.addTextMessage(5, 5, format("%.02fFPS", win.getRenderingFPS()));
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
				gl_plane1->assignImage(o->image);
				gl_plane2->assignImage(o->image);
				gl_plane3->assignImage(o->image);
				win.unlockAccess3DScene();
				win.repaint();
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
