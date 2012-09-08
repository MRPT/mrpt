/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
