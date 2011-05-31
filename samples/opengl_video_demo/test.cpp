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
	CDisplayWindow3D	win("Demo of video textures with MRPT's OpenGL objects",640,480);

	// XY Grid
	opengl::CGridPlaneXYPtr gl_ground = opengl::CGridPlaneXY::Create(-7,7,-7,7,0,1);
	gl_ground->setColor(0.7,0.7,0.7);

	// An opengl plane with the video texture
	opengl::CTexturedPlanePtr gl_plane1 = opengl::CTexturedPlane::Create(0,1,0,0.75); // 4/3 aspect ratio
	opengl::CTexturedPlanePtr gl_plane2 = opengl::CTexturedPlane::Create(0,1,0,0.75);
	opengl::CTexturedPlanePtr gl_plane3 = opengl::CTexturedPlane::Create(0,1,0,0.75);

	gl_plane1->setPose(mrpt::poses::CPose3D(0,0,1, DEG2RAD(0), DEG2RAD(0), DEG2RAD(-90)));
	gl_plane2->setPose(mrpt::poses::CPose3D(1,0,1, DEG2RAD(120), DEG2RAD(0), DEG2RAD(-90)));
	gl_plane3->setPose(mrpt::poses::CPose3D(0,0,1, DEG2RAD(60), DEG2RAD(0), DEG2RAD(-90)));

	win.setCameraZoom(5);

	// Insert objects in scene:
	{
		COpenGLScenePtr &theScene = win.get3DSceneAndLock();
		theScene->insert( gl_ground );
		theScene->insert( gl_plane1 );
		theScene->insert( gl_plane2 );
		theScene->insert( gl_plane3 );
		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();
	}
	win.repaint();

	cout << "Close the window to end.\n";
	while (win.isOpen())
	{
		win.addTextMessage(5,5, format("%.02fFPS", win.getRenderingFPS()));
		mrpt::system::sleep(1);

		// Grab new video frame:
		CObservationPtr obs = cam->getNextFrame();
		if (obs)
		{
			if (IS_CLASS(obs,CObservationImage))
			{
				CObservationImagePtr o = CObservationImagePtr(obs);
				win.get3DSceneAndLock();
					gl_plane1->assignImage( o->image );
					gl_plane2->assignImage( o->image );
					gl_plane3->assignImage( o->image );
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
