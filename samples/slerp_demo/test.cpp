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
