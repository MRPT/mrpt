/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
#include <mrpt/hwdrivers/CCameraSensor.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;


// ------------------------------------------------------
//				TestOpenGLVideo
// ------------------------------------------------------
void TestOpenGLVideo()
{
	// Show to the user a list of possible camera drivers and creates and open the selected camera.
	cout << "Please, select the input video file or camera...\n";

	mrpt::hwdrivers::CCameraSensorPtr cam = mrpt::hwdrivers::prepareVideoSourceFromUserSelection();
	if (!cam) return;

	cout << "Video stream open OK\n";

	// Create 3D window:
	CDisplayWindow3D	win ("#1: Demo of image mode viewport",320,240);
	CDisplayWindow3D	win2("#2: Demo of image mode viewport",500,400);

	// Win #1:
	//  Get the smart pointer to the main viewport object in this window:
	COpenGLViewportPtr gl_view_main;
	{
		COpenGLScenePtr &theScene = win.get3DSceneAndLock();
		gl_view_main  = theScene->getViewport("main");
		ASSERT_(gl_view_main)
		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();
	}

	// Win #2:
	//  Get the smart pointer to the main viewport object in this window:
	COpenGLViewportPtr gl_view_aux;
	{
		COpenGLScenePtr &theScene = win2.get3DSceneAndLock();
		theScene->insert( mrpt::opengl::CGridPlaneXY::Create() );

		// Create small auxiliary viewport
		gl_view_aux  = theScene->createViewport("aux");

		gl_view_aux->setViewportPosition(10,10, 300,200);
		gl_view_aux->setTransparent(true);


		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win2.unlockAccess3DScene();
	}

	win.setPos(10,10);
	win2.setPos(400,100);


	cout << "Close any window to end.\n";
	while (win.isOpen() && win2.isOpen())
	{
		win.addTextMessage (5,5, format("%.02fFPS", win.getRenderingFPS()));
		win2.addTextMessage(5,5, format("%.02fFPS", win2.getRenderingFPS()));
		mrpt::system::sleep(1);

		// Grab new video frame:
		CObservationPtr obs = cam->getNextFrame();
		if (obs)
		{
			if (IS_CLASS(obs,CObservationImage))
			{
				CObservationImagePtr o = CObservationImagePtr(obs);
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
