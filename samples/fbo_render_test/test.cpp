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
#include <mrpt/opengl.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;

// ------------------------------------------------------
//				TestDisplay3D
// ------------------------------------------------------
void TestDisplay3D()
{
	COpenGLScene scene;

	// Modify the scene:
	// ------------------------------------------------------
	{
		opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-20,20,-20,20,0,1);
		obj->setColor(0.4,0.4,0.4);
		scene.insert( obj );
	}
	{
		opengl::CAxisPtr obj = opengl::CAxis::Create();
		obj->setFrequency(5);
		obj->enableTickMarks();
		obj->setAxisLimits(-10,-10,-10, 10,10,10);
		scene.insert( obj );
	}
	{
		opengl::CSpherePtr obj = opengl::CSphere::Create();
		obj->setColor(0,0,1);
		obj->setRadius(0.3);
		obj->setLocation(0,0,1);
		obj->setName( "ball_1" );
		scene.insert( obj );
	}
	{
		opengl::CSpherePtr obj = opengl::CSphere::Create();
		obj->setColor(1,0,0);
		obj->setRadius(0.3);
		obj->setLocation(-1,-1,1);
		obj->setName( "ball_2");
		scene.insert( obj );
	}

	CDisplayWindow win("output");

	int c = 0, width = 640, height = 480;

	CFBORender render(width, height);
	CImage frame(width, height, 3, false);

	{
		CCamera& camera = render.getCamera(scene);
		camera.setZoomDistance(50);
	}

	while (!mrpt::system::os::kbhit())
	{
		CRenderizablePtr obj = scene.getByName("ball_1");
		obj->setLocation(
			obj->getPoseX() + cos(obj->getPoseY()/2)*0.05,
			obj->getPoseY() - sin(obj->getPoseX()/2)*0.09,
			obj->getPoseZ() - sin(obj->getPoseX()/2)*0.08 );

		obj = scene.getByName("ball_2");
		obj->setLocation(
			obj->getPoseX() + cos(obj->getPoseY()/2)*0.05,
			obj->getPoseY() - sin(obj->getPoseX()/2)*0.09,
			obj->getPoseZ() - sin(obj->getPoseX()/2)*0.08 );

		// change the size
		if(++c > 100)
		{
			width = 800, height = 600;
			frame.resize(width, height, 3, false);
			render.resize(width, height);
		}

		// render the scene
		render.getFrame2(scene, frame);

		// show the redered image
		win.showImage(frame);

		mrpt::system::sleep(50);
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char* argv[])
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
