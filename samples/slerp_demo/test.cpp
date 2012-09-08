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

#include <mrpt/base.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::poses;

// ------------------------------------------------------
//				TestSLERP
// ------------------------------------------------------
void TestSLERP()
{
	CDisplayWindow3D	win("Example of SLERP animation",640,480);

	COpenGLScenePtr &theScene = win.get3DSceneAndLock();

	win.setCameraAzimuthDeg(-50);
	win.setCameraElevationDeg(40);
	win.setCameraZoom(19);
	win.setCameraPointingToPoint(2,2,0);

	// Modify the scene:
	// ------------------------------------------------------
	{
		opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-20,20,-20,20,0,1);
		obj->setColor(0.4,0.4,0.4);
		theScene->insert( obj);
	}

	const CPose3D  pose_a(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));
	const CPose3D  pose_b(3,4,1, DEG2RAD(120),DEG2RAD(40),DEG2RAD(50));

	{
		// XYZ corner at A:
		opengl::CSetOfObjectsPtr obj = opengl::stock_objects::CornerXYZSimple(1.0, 2.0);
		obj->setPose(pose_a);
		theScene->insert(obj);
	}
	{
		// XYZ corner at B:
		opengl::CSetOfObjectsPtr obj = opengl::stock_objects::CornerXYZSimple(1.0, 2.0);
		obj->setPose(pose_b);
		theScene->insert(obj);
	}
	{
		// SLERP animated corner:
		opengl::CSetOfObjectsPtr obj = opengl::stock_objects::CornerXYZSimple(1.0, 4.0);
		obj->setName("slerp_obj");
		obj->setPose(pose_a);
		theScene->insert(obj);
	}


	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	cout << "\n Close the window to exit.\n";

	mrpt::utils::CTicTac  tic;
	static const double MOVE_PERIOD = 1.0;
	static const double MOVE_PERIOD2 = 2*MOVE_PERIOD;

	while (win.isOpen() )
	{
		// Compute the time:
		double t = ::fmod(tic.Tac(), MOVE_PERIOD2);
		if (t<MOVE_PERIOD)
		     t /= MOVE_PERIOD;
		else t = 1-(t-MOVE_PERIOD)/MOVE_PERIOD;

		// SLERP & LERP interpolation:
		CPose3D pose_interp;
		mrpt::math::slerp(pose_a,pose_b,t,pose_interp);

		// Move the scene:
		COpenGLScenePtr &theScene = win.get3DSceneAndLock();

		opengl::CRenderizablePtr obj1 = theScene->getByName("slerp_obj");
		obj1->setPose(pose_interp);


		// Show text:
		win.addTextMessage(5,5, format("t=%.03f",t), TColorf(1,1,1),0,MRPT_GLUT_BITMAP_TIMES_ROMAN_24 );

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();

		// Update window:
		win.forceRepaint();
		mrpt::system::sleep(5);

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
