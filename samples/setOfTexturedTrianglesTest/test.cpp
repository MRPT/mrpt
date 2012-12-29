/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
#include <mrpt/slam.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::vision;
using namespace mrpt::opengl;

#include <mrpt/examples_config.h>

string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("setOfTexturedTrianglesTest/") );

// ------------------------------------------------------
//				TestDisplay3D
// ------------------------------------------------------
void TestDisplay3D()
{
	CDisplayWindow3D win("Test of CSetOfTexturedTriangles");
	COpenGLScenePtr &scene = win.get3DSceneAndLock();

	// Modify the scene:
	// ------------------------------------------------------
	{
		opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-20,20,-20,20,0,1);
		obj->setColor(0.4,0.4,0.4);
		scene->insert( obj );
	}
	{
		opengl::CAxisPtr obj = opengl::CAxis::Create();
		obj->setFrequency(5);
		obj->enableTickMarks();
		obj->setAxisLimits(-10,-10,-10, 10,10,10);
		scene->insert( obj );
	}
	{
		opengl::CSpherePtr obj = opengl::CSphere::Create();
		obj->setColor(0,0,1);
		obj->setRadius(0.3);
		obj->setLocation(0,0,1);
		obj->setName( "ball_1" );
		scene->insert( obj );
	}
	{
		opengl::CSpherePtr obj = opengl::CSphere::Create();
		obj->setColor(1,0,0);
		obj->setRadius(0.3);
		obj->setLocation(-1,-1,1);
		obj->setName( "ball_2");
		scene->insert( obj );
	}
	{
		vision::CImage image, alpha;

//		image.loadFromFile(myDataDir + string("texture.png"), 0); // grayscale
		image.loadFromFile(myDataDir + string("texture.png"), 1); // color
		alpha.loadFromFile(myDataDir + string("mask.png"),    0); // transparency

		opengl::CSetOfTexturedTrianglesPtr obj = opengl::CSetOfTexturedTriangles::Create();
		opengl::CSetOfTexturedTriangles::TTriangle tri;

		tri = opengl::CSetOfTexturedTriangles::TTriangle(
					opengl::CSetOfTexturedTriangles::TVertex( -2.0, -2.0, 0,   0, 256 ),	// 3D coord (x,y,z) Pixel coord (u,v)
					opengl::CSetOfTexturedTriangles::TVertex(  2.0, -2.0, 0, 256, 256 ),
					opengl::CSetOfTexturedTriangles::TVertex(  2.0,  2.0, 0, 256,   0 ) );
		obj->insertTriangle(tri);

		tri = opengl::CSetOfTexturedTriangles::TTriangle(
					opengl::CSetOfTexturedTriangles::TVertex( -2.0,  2.0, 1,   0,   0 ),
					opengl::CSetOfTexturedTriangles::TVertex(  2.0,  2.0, 0, 256,   0 ),
					opengl::CSetOfTexturedTriangles::TVertex( -2.0, -2.0, 0,   0, 256 ) );
		obj->insertTriangle(tri);

		obj->setName( "set" );
//		obj->assignImage(image);
		obj->assignImage(image, alpha);  // transparency
		scene->insert( obj );
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	win.setCameraElevationDeg( 25.0f );

	while( !mrpt::system::os::kbhit() && win.isOpen() )
	{
		// Move the scene:
		COpenGLScenePtr &scene = win.get3DSceneAndLock();

		opengl::CRenderizablePtr obj = scene->getByName("ball_1");
		obj->setLocation(
			obj->getPoseX() + cos(obj->getPoseY()/2)*0.05,
			obj->getPoseY() - sin(obj->getPoseX()/2)*0.09,
			obj->getPoseZ() - sin(obj->getPoseX()/2)*0.08 );

		obj = scene->getByName("ball_2");
		obj->setLocation(
			obj->getPoseX() + cos(obj->getPoseY()/2)*0.05,
			obj->getPoseY() - sin(obj->getPoseX()/2)*0.09,
			obj->getPoseZ() - sin(obj->getPoseX()/2)*0.08 );

		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();

		// Update window:
		win.forceRepaint();

		mrpt::system::sleep(20);
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

