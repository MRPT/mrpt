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
using namespace mrpt::gui;
using namespace mrpt::opengl;

// ------------------------------------------------------
//				TestDisplay3D
// ------------------------------------------------------
void TestDisplay3D()
{
	CDisplayWindow3D	win("Example of 3D Scene Visualization - MRPT",640,480);

	COpenGLScenePtr &theScene = win.get3DSceneAndLock();


	// Add a clone viewport, using [0,1] factor X,Y,Width,Height coordinates:
	{
		COpenGLViewportPtr vi= theScene->createViewport("myClone");
		vi->setViewportPosition(0.7,0.05,0.28,0.28);
		vi->setCloneView("main");
		vi->setTransparent(true);
		vi->getCamera().setAzimuthDegrees(45);
		vi->getCamera().setElevationDegrees(45);
		vi->getCamera().setZoomDistance(10);
	}

	// Another clone viewport, using absolute coordinates
	{
		COpenGLViewportPtr vi= theScene->createViewport("myClone2");
		vi->setViewportPosition(/*x px*/ -250, /*y px*/-250, /*width px*/ 250, /*height px*/ 200);  // x,y negative means pixels from the top/right, instead of from the bottom/left.
		vi->setCloneView("main");
		vi->setTransparent(false);
		vi->getCamera().setAzimuthDegrees(-95);
		vi->getCamera().setElevationDegrees(30);
		vi->getCamera().setZoomDistance(8);
	}

	// And another transparent viewport just to show 3D text:
	if (0)
	{
		mrpt::opengl::CTextPtr txt1 = mrpt::opengl::CText::Create();
		COpenGLViewportPtr vi= theScene->createViewport("flat_viewport");
		vi->setViewportPosition(0,0,0.3,0.3);
		vi->setTransparent(true);
		vi->setBorderSize(0);
		vi->getCamera().setAzimuthDegrees(0);
		vi->getCamera().setElevationDegrees(90);
		vi->getCamera().setZoomDistance(5);
		vi->getCamera().setOrthogonal(true);

		vi->insert(txt1);
	}

	// Modify the scene:
	// ------------------------------------------------------
	{
		opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-20,20,-20,20,0,1);
		obj->setColor(0.8,0.8,0.8);
		theScene->insert( obj );
	}

	{
		opengl::CAxisPtr obj = opengl::CAxis::Create();
		obj->setFrequency(5);
		obj->enableTickMarks();
		obj->setAxisLimits(-10,-10,-10, 10,10,10);
		theScene->insert( obj );
	}

	{
		opengl::CBoxPtr obj = opengl::CBox::Create();
		obj->setWireframe(false);
		obj->setColor(1,0,0);
		obj->setLineWidth(3.0);
		obj->setPose(TPose3D(10,0,0,0.2,0.3,0.1));
		theScene->insert( obj );
	}

	{
		opengl::CSpherePtr obj = opengl::CSphere::Create();
		obj->setColor(0,0,1);
		obj->setRadius(0.3);
		obj->setLocation(0,0,1);
		obj->setName( "ball_1" );
		theScene->insert( obj );
	}
	{
		opengl::CSpherePtr obj = opengl::CSphere::Create();
		obj->setColor(1,0,0);
		obj->setRadius(0.3);
		obj->setLocation(-1,-1,1);
		obj->setName( "ball_2");
		theScene->insert( obj );
	}

	{
		opengl::CSpherePtr obj = opengl::CSphere::Create();
		obj->setColor(0,1,0);
		obj->setRadius(0.5);
		obj->setLocation(0,0,0);
		obj->setName( "USER_MOUSE_PICK");
		theScene->insert( obj );
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	// Texts:
	win.addTextMessage(0.01,0.85, "This is a 2D message", TColorf(1,1,1),"sans",11, mrpt::opengl::NICE, 0);

	win.setCameraElevationDeg( 25.0f );
	//win.setCameraProjective(false);

	cout << endl;
	cout << "Control with mouse or keyboard. Valid keys:" << endl;
	cout << "  ESC                        -> Exit" << endl;
	cout << "  Left/right cursor arrow    -> Camera azimuth" << endl;
	cout << "  P                          -> Enable / disable 'place object' mode" << endl;
	cout << endl;

	bool end = false;
	bool placeMode = false;

	CTicTac  timer;
	timer.Tic();

	while (!end && win.isOpen() )
	{
		const double t = timer.Tac();

		// Move the scene:
		COpenGLScenePtr &theScene = win.get3DSceneAndLock();

		const double R1 = 8;
		const double W1= 5.0, Q1 = 3.3;
		opengl::CRenderizablePtr obj1 = theScene->getByName("ball_1");
		obj1->setLocation(
			R1* cos(W1*t)*sin(Q1*t),
			R1* sin(W1*t),
			R1* cos(W1*t)*cos(Q1*t) );


		const double R2 = 6;
		const double W2= 1.3, Q2 = 7.2;
		opengl::CRenderizablePtr obj2 = theScene->getByName("ball_2");
		obj2->setLocation(
			R2* cos(W2*t)*sin(Q2*t),
			R2* sin(W2*t),
			R2* cos(W2*t)*cos(Q2*t) );


		win.addTextMessage(0.01,0.85, "This is a 2D message", TColorf(1,0,0),"sans",8, mrpt::opengl::NICE, 0);

		win.addTextMessage(0.02,0.02,  // X,Y<=1 means coordinates are factors over the entire viewport area.
			format("ball#1 pos: %.02f %.02f %.02f ",obj1->getPoseX(),obj1->getPoseY(),obj1->getPoseZ()),
			TColorf(.8,.8,.8),
			"sans",14,  // font name & size
			mrpt::opengl::FILL,
			10 // An arbitrary ID to always overwrite the same, previous 2D text message
			);

		win.addTextMessage(5,-15,  // |X|,|Y|>1 means absolute coordinates, negative means from the top instead of the bottom.
			format("Time: %s", mrpt::system::dateTimeLocalToString( mrpt::system::now() ).c_str() ),
			TColorf(1,1,1),
			"mono",9,  // font name & size
			mrpt::opengl::NICE,
			20 // An arbitrary ID to always overwrite the same, previous 2D text message
			);

		// Show management of (x,y) mouse coordinates and 3D rays:
		// ------------------------------------------------------------
		int mouse_x,mouse_y;
		if (placeMode && win.getLastMousePosition(mouse_x,mouse_y))  // See also: getLastMousePositionRay()
		{
			// Get the ray in 3D for the latest mouse (X,Y):
			mrpt::math::TLine3D ray;
			theScene->getViewport("main")->get3DRayForPixelCoord(mouse_x,mouse_y,ray);

			// Create a 3D plane, e.g. Z=0
			const mrpt::math::TPlane ground_plane(TPoint3D(0,0,0),TPoint3D(1,0,0),TPoint3D(0,1,0));

			// Intersection of the line with the plane:
			mrpt::math::TObject3D inters;
			mrpt::math::intersect(ray,ground_plane, inters);

			// Interpret the intersection as a point, if there is an intersection:
			mrpt::math::TPoint3D inters_pt;
			if (inters.getPoint(inters_pt))
			{
				// Move an object to the position picked by the user:
				//printf("PT: %f %f %f\n",);
				theScene->getByName("USER_MOUSE_PICK")->setLocation(inters_pt.x,inters_pt.y,inters_pt.z);
			}
		}

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();

		// Update window:
		win.forceRepaint();
		mrpt::system::sleep(20);

		if (mrpt::system::os::kbhit()) end = true;
		if (win.keyHit())
		{
			mrptKeyModifier kmods;
			int key = win.getPushedKey(&kmods);
			printf("Key pushed: %c (%i) - modifiers: 0x%04X\n",char(key),key,kmods);

			if (key==MRPTK_ESCAPE) end = true;

			if (key==MRPTK_RIGHT) win.setCameraAzimuthDeg( win.getCameraAzimuthDeg() + 5 );
			if (key==MRPTK_LEFT)  win.setCameraAzimuthDeg( win.getCameraAzimuthDeg() - 5 );

			if (key=='p' || key=='P')
			{
				placeMode = !placeMode;
				win.setCursorCross(placeMode);
			}
		}

	};
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestDisplay3D();

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
