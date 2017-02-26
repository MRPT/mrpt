/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CImage.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CSetOfTexturedTriangles.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/gui/CDisplayWindow3D.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
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
		CImage image, alpha;

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

