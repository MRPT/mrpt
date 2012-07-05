/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
