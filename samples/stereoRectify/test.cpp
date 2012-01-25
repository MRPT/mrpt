/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/system/filesystem.h> // for ASSERT_FILE_EXISTS_
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/vision.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::vision;
using namespace std;


// ------------------------------------------------------
//				TestStereoRectify
// ------------------------------------------------------
void TestStereoRectify(int argc, char** argv)
{
	mrpt::vision::CStereoRectifyMap  rectifyMap;

	// Parse optional arguments:
	if (argc!=1 && argc!=2)
	{
		cout<< "Usage:\n"
			<< argv[0] << " ==> Run with default camera parameters (from rawlog file)\n"
			<< argv[0] << "[params.cfg] ==> Load stereo camera parameters from cfg file\n";
	}
	if (argc==2)
	{
		const string sCfgFile = argv[1];
		ASSERT_FILE_EXISTS_(sCfgFile)

		// rectifyMap.setFromCamParams
	}

	// Show to the user a list of possible camera drivers and creates and open the selected camera.
	cout << "Please, select the input stereo camera or rawlog file (with stereo images)...\n";

	mrpt::hwdrivers::CCameraSensorPtr cam = mrpt::hwdrivers::prepareVideoSourceFromUserSelection();
	if (!cam) return;

	cout << "Video stream open OK\n";

	// Create 3D window:
	CDisplayWindow3D	win("Demo of stereo rectification",1280,600);

	// Create 2 viewports, one for each image:
	std::vector<COpenGLViewportPtr>  gl_views(2);
	{
		COpenGLScenePtr &theScene = win.get3DSceneAndLock();
		gl_views[0]  = theScene->getViewport("main");
		ASSERT_(gl_views[0])
		gl_views[1] = theScene->createViewport("right_image");

		// Assign sizes:
		gl_views[0]->setViewportPosition(0,0, .5,1.);
		gl_views[1]->setViewportPosition(.5,0, .5,1.);

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();
	}

	win.setPos(10,10);
//	win.addTextMessage(...

	bool enable_rectify = true;


	cout << "Close the window to end.\n";
	while (win.isOpen())
	{
		win.addTextMessage (5,5,
			format("%.02fFPS - 'r': Switch rectify (Now is: %s)",
				win.getRenderingFPS(),
				enable_rectify ? "ON":"OFF"
				),
			TColorf(1,1,1),"sans",8, mrpt::opengl::FILL, 0
			);
		mrpt::system::sleep(1);

		// Grab new video frame:
		CObservationPtr obs = cam->getNextFrame();
		if (obs)
		{
			if (IS_CLASS(obs,CObservationStereoImages))
			{
				// Get the observation object:
				CObservationStereoImagesPtr o = CObservationStereoImagesPtr(obs);

				win.get3DSceneAndLock();

				CImage img_left_rectified, img_right_rectified;

				if (enable_rectify)
				{
					// Rectify:
					img_left_rectified = o->imageLeft;
					img_right_rectified = o->imageRight;
				}
				else
				{
					// Don't rectify:
					img_left_rectified = o->imageLeft;
					img_right_rectified = o->imageRight;
				}

				// Warning: I can use _fast() here because I don't mind
				//  destroying the temporary rectified images.
				gl_views[0]->setImageView_fast(img_left_rectified);
				gl_views[1]->setImageView_fast(img_right_rectified);

				win.addTextMessage(
					5,25,
					mrpt::system::timeToString(o->timestamp),
					TColorf(1,1,1),"sans",8, mrpt::opengl::FILL, 1
					);

				win.unlockAccess3DScene();
				win.repaint();
			}

			if (win.keyHit())
			{
				mrptKeyModifier kmods;
				int key = win.getPushedKey(&kmods);

				if (key==MRPTK_ESCAPE) break;
				if (key=='r' || key=='R') enable_rectify= !enable_rectify;
			}

		}
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		TestStereoRectify(argc,argv);
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
