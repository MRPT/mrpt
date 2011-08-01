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
using namespace mrpt::math;


// ------------------------------------------------------
//				TestOpenGLObjects
// ------------------------------------------------------
void TestOpenGLObjects()
{
mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 10000;

	CDisplayWindow3D	win("Demo of MRPT's OpenGL objects",640,480);

	COpenGLScenePtr &theScene = win.get3DSceneAndLock();

	double off_x = 0;
	const double off_y_label = 20;
	const double STEP_X = 25;

	// XY Grid
	{
		opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-7,7,-7,7,0,1);
		obj->setColor(0.7,0.7,0.7);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("CGridPlaneXY");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;


	// XZ Grid
	{
		opengl::CGridPlaneXZPtr obj = opengl::CGridPlaneXZ::Create(-7,7,-7,7,0,1);
		obj->setColor(0.7,0.7,0.7);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("CGridPlaneXZ");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// Arrow
	{
		opengl::CArrowPtr obj = opengl::CArrow::Create(0,0,0, 3,0,0, 0.2, 0.1,0.2, 0,0,0 );
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("CArrow");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;


	// Axis
	{
		opengl::CAxisPtr obj = opengl::CAxis::Create(-6,-6,-6, 6,6,6, 2,2, true);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("CAxis");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// Box
	{
		opengl::CBoxPtr obj = opengl::CBox::Create(TPoint3D(0,0,0),TPoint3D(1,1,1), true, 3.0);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CBoxPtr obj2 = opengl::CBox::Create(TPoint3D(0,0,0),TPoint3D(1,1,1), false);
		obj2->setLocation(off_x,4,0);
		theScene->insert( obj2 );

		opengl::CTextPtr gl_txt = opengl::CText::Create("CBox");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// Cylinder
	{
		opengl::CCylinderPtr obj = opengl::CCylinder::Create(2,2, 4,  20,10);
		obj->setLocation(off_x,0,0);
		obj->setColor(0,0,0.8);
		theScene->insert( obj );

		opengl::CCylinderPtr obj2 = opengl::CCylinder::Create(2,1, 4,  20,10);
		obj2->setLocation(off_x,6,0);
		obj2->setColor(0,0,0.8);
		theScene->insert( obj2 );

		opengl::CCylinderPtr obj3 = opengl::CCylinder::Create(2,0, 4,  20,10);
		obj3->setLocation(off_x,-6,0);
		obj3->setColor(0,0,0.8);
		theScene->insert( obj3 );

		opengl::CTextPtr gl_txt = opengl::CText::Create("CCylinder");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// CDisk
	{
		{
			opengl::CDiskPtr obj = opengl::CDisk::Create(2.0,1.8, 50);
			obj->setLocation(off_x,0,0);
			obj->setColor(0.8,0,0);
			theScene->insert( obj );
		}

		{
			opengl::CDiskPtr obj = opengl::CDisk::Create(2.0,0, 50);
			obj->setLocation(off_x,5,0);
			obj->setColor(0.8,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("CDisk");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// CEllipsoid
	{
		const double cov3d_dat[] = {
			0.9, 0.7, -0.4,
			0.7, 1.6, -0.6,
			-0.4,-0.6, 1.5 };
		const double cov2d_dat[] = {
			0.9, 0.7,
			0.7, 1.6 };
		mrpt::math::CMatrixDouble22 cov2d(cov2d_dat);
		mrpt::math::CMatrixDouble33 cov3d(cov3d_dat);

		{
			opengl::CEllipsoidPtr obj = opengl::CEllipsoid::Create();
			obj->setCovMatrix(cov2d);
			obj->setLocation(off_x,6,0);
			obj->setQuantiles(2.0);
			theScene->insert( obj );
		}
		{
			opengl::CEllipsoidPtr obj = opengl::CEllipsoid::Create();
			obj->setCovMatrix(cov3d);
			obj->setQuantiles(2.0);
			obj->enableDrawSolid3D(false);
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}
		{
			opengl::CEllipsoidPtr obj = opengl::CEllipsoid::Create();
			obj->setCovMatrix(cov3d);
			obj->setQuantiles(2.0);
			obj->enableDrawSolid3D(true);
			obj->setLocation(off_x,-6,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("CEllipsoid");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;


	// CMesh
	{
		opengl::CMeshPtr obj = opengl::CMesh::Create();
		obj->setLocation(off_x,0,0);

		mrpt::math::CMatrixFloat  Zs(40,40); // Height
//		mrpt::math::CMatrixFloat  Us,Vs; // Texture
//		Us.resize(Zs.size());
//		Vs.resize(Zs.size());

		for (size_t i=0;i<size(Zs,1);i++)
			for (size_t j=0;j<size(Zs,2);j++)
			{
				double x = i*0.25; double y= j*0.25;
				Zs(i,j) = 4*cos((x+y)*0.6)+sin((x-0.5)*(y+1.2)*0.3)*3;
			}

		obj->setGridLimits(-10,10,-10,10);
		obj->setZ(Zs);
		//obj->enableWireFrame(true);
		obj->enableColorFromZ(true, mrpt::utils::cmJET );
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("CMesh");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;


	// CPointCloud
	{
		opengl::CPointCloudPtr obj = opengl::CPointCloud::Create();
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		obj->setPointSize(3.0);
		obj->enablePointSmooth();
		obj->enableColorFromY();


		for (int i=0;i<100000;i++)
			obj->insertPoint(
				mrpt::random::randomGenerator.drawUniform(-5,5),
				mrpt::random::randomGenerator.drawUniform(-5,5),
				mrpt::random::randomGenerator.drawUniform(-5,5) );

		opengl::CTextPtr gl_txt = opengl::CText::Create("CPointCloud");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// CPointCloudColoured
	{
		opengl::CPointCloudColouredPtr obj = opengl::CPointCloudColoured::Create();
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		obj->setPointSize(3.0);
		obj->enablePointSmooth();

		for (int i=0;i<200;i++)
			obj->push_back(
				mrpt::random::randomGenerator.drawUniform(-5,5),
				mrpt::random::randomGenerator.drawUniform(-5,5),
				mrpt::random::randomGenerator.drawUniform(-5,5),
				mrpt::random::randomGenerator.drawUniform(0,1),
				mrpt::random::randomGenerator.drawUniform(0,1),
				mrpt::random::randomGenerator.drawUniform(0,1) );

		opengl::CTextPtr gl_txt = opengl::CText::Create("CPointCloudColoured");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// CPolyhedron
	{
		{
			opengl::CPolyhedronPtr obj = opengl::CPolyhedron::CreateCuboctahedron(1.0);
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}
		{
			opengl::CPolyhedronPtr obj = opengl::CPolyhedron::CreateDodecahedron(1.0);
			obj->setLocation(off_x,-5,0);
			theScene->insert( obj );
		}
		{
			opengl::CPolyhedronPtr obj = opengl::CPolyhedron::CreateIcosahedron(1.0);
			obj->setLocation(off_x,5,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("CPolyhedron");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// CSphere
	{
		{
			opengl::CSpherePtr obj = opengl::CSphere::Create(3.0);
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("CSphere");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

#if 1
	// CText
	{
		{
			opengl::CTextPtr obj = opengl::CText::Create("This is a CText example! My size is invariant to eye-distance");
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("CText");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// CText3D
	{
		{
			opengl::CText3DPtr obj = opengl::CText3D::Create("I'm a cool CText3D!");
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("CText3D");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;


	// CSetOfLines
	{
		{
			opengl::CSetOfLinesPtr obj = opengl::CSetOfLines::Create();
			obj->setLocation(off_x,0,0);

			for (int i=0;i<15;i++)
				obj->appendLine(
					mrpt::random::randomGenerator.drawUniform(-5,5),mrpt::random::randomGenerator.drawUniform(-5,5),mrpt::random::randomGenerator.drawUniform(-5,5),
					mrpt::random::randomGenerator.drawUniform(-5,5),mrpt::random::randomGenerator.drawUniform(-5,5),mrpt::random::randomGenerator.drawUniform(-5,5) );

			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("CSetOfLines");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// stock_objects::BumblebeeCamera
	{
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::BumblebeeCamera();
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("stock_objects::BumblebeeCamera()");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// stock_objects::CornerXYSimple
	{
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::CornerXYSimple(1,3);
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("stock_objects::CornerXYSimple()");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// stock_objects::CornerXYZSimple
	{
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::CornerXYZSimple(1,3);
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("stock_objects::CornerXYZSimple()");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// stock_objects::CornerXYZ
	{
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::CornerXYZ();
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("stock_objects::CornerXYZ()");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// stock_objects::RobotPioneer
	{
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("stock_objects::RobotPioneer()");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

#endif

	win.setCameraZoom(150);

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();
	win.repaint();

	cout << "Close the window to end.\n";
	while (win.isOpen())
	{
		win.addTextMessage(5,5, format("%.02fFPS", win.getRenderingFPS()));
		mrpt::system::sleep(2);
		win.repaint();
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestOpenGLObjects();

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
