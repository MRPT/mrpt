/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

	// Lights:
	//theScene->getViewport()->setNumberOfLights(1);
	//mrpt::opengl::CLight & light0 = theScene->getViewport()->getLight(0);
	//light0.light_ID = 0;
	//light0.setPosition(1,1,0,0);


	// Objects:
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
		obj->setColor(1,0,0);
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

	// Frustum
	{
		opengl::CFrustumPtr obj = opengl::CFrustum::Create(1,5, 60,45, 1.5f, true, false);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CFrustumPtr obj2 = opengl::CFrustum::Create(1,5, 60,45, 2.5f, true, true);
		obj2->setLocation(off_x,6,0);
		theScene->insert( obj2 );

		opengl::CTextPtr gl_txt = opengl::CText::Create("CFrustum");
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


	// CEllipsoidRangeBearing2D
	{	// (range,bearing) -> (x,y)
		const double cov_params_dat[] = {
			0.2,  0,
			0,   0.1
		};
		const double mean_params_dat[] = {
			3.0, 0.5
		};
		mrpt::math::CMatrixFixedNumeric<double,2,2> cov_params(cov_params_dat);
		mrpt::math::CMatrixFixedNumeric<double,2,1> mean_params(mean_params_dat);

		{
			opengl::CEllipsoidRangeBearing2DPtr obj = opengl::CEllipsoidRangeBearing2D::Create();
			obj->setCovMatrixAndMean(cov_params,mean_params);
			obj->setLocation(off_x,6,0);
			obj->setQuantiles(2.0f);
			//obj->setNumberOfSegments(50);
			theScene->insert( obj );

			opengl::CSetOfObjectsPtr obj_corner = opengl::stock_objects::CornerXYSimple(1,3);
			obj_corner->setLocation(off_x,6,0);
			theScene->insert( obj_corner );
		}
	}
	{	// (range,bearing) -> (x,y)
		const double cov_params_dat[] = {
			0.2,  0.09,
			0.09,   0.1
		};
		const double mean_params_dat[] = {
			5.0, -0.5
		};
		mrpt::math::CMatrixFixedNumeric<double,2,2> cov_params(cov_params_dat);
		mrpt::math::CMatrixFixedNumeric<double,2,1> mean_params(mean_params_dat);

		{
			opengl::CEllipsoidRangeBearing2DPtr obj = opengl::CEllipsoidRangeBearing2D::Create();
			obj->setCovMatrixAndMean(cov_params,mean_params);
			obj->setLocation(off_x,0,0);
			obj->setQuantiles(2.0f);
			//obj->setNumberOfSegments(50);
			theScene->insert( obj );

			opengl::CSetOfObjectsPtr obj_corner = opengl::stock_objects::CornerXYSimple(1,3);
			obj_corner->setLocation(off_x,0,0);
			theScene->insert( obj_corner );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("CEllipsoidRangeBearing2D");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// CEllipsoidInverseDepth2D
	{	// (inv_range,yaw) -> (x,y)
		// Formula from our book (ch8) for confidence intervals of 3sigmas:
		const double max_dist = 1e4;
		const double min_dist = 1;
		const double rho_mean = 0.5*(1./min_dist+1./max_dist);
		const double rho_std  = (1./6.)*(1./min_dist-1./max_dist);

		const double cov_params_dat[] = {
			square(rho_std),  0,
			0,   square(DEG2RAD(2))
		};
		const double mean_params_dat[] = {
			rho_mean, DEG2RAD(70)
		};
		mrpt::math::CMatrixFixedNumeric<double,2,2> cov_params(cov_params_dat);
		mrpt::math::CMatrixFixedNumeric<double,2,1> mean_params(mean_params_dat);

		{
			opengl::CEllipsoidInverseDepth2DPtr obj = opengl::CEllipsoidInverseDepth2D::Create();
			obj->setCovMatrixAndMean(cov_params,mean_params);
			obj->setLocation(off_x,6,0);
			obj->setQuantiles(3.f);
			obj->setNumberOfSegments(100);
			theScene->insert( obj );

			opengl::CSetOfObjectsPtr obj_corner = opengl::stock_objects::CornerXYSimple(1,3);
			obj_corner->setLocation(off_x,6,0);
			theScene->insert( obj_corner );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("CEllipsoidInverseDepth2D");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// CEllipsoidInverseDepth3D
	{	// (inv_range,yaw,pitch) -> (x,y,z)
		// Formula from our book (ch8) for confidence intervals of 3sigmas:
		const double max_dist = 1e2;
		const double min_dist = 1;
		const double rho_mean = 0.5*(1./min_dist+1./max_dist);
		const double rho_std  = (1./6.)*(1./min_dist-1./max_dist);

		const double cov_params_dat[] = {
			square(rho_std),      0,                    0,
			0,   square(DEG2RAD(2)),                    0,
			0,                    0,   square(DEG2RAD(2))
		};
		const double mean_params_dat[] = {
			rho_mean, DEG2RAD(30), DEG2RAD(-45)
		};
		mrpt::math::CMatrixFixedNumeric<double,3,3> cov_params(cov_params_dat);
		mrpt::math::CMatrixFixedNumeric<double,3,1> mean_params(mean_params_dat);

		{
			opengl::CEllipsoidInverseDepth3DPtr obj = opengl::CEllipsoidInverseDepth3D::Create();
			obj->setCovMatrixAndMean(cov_params,mean_params);
			obj->setLocation(off_x,0,0);
			obj->setQuantiles(3.f);
			//obj->setNumberOfSegments(50);
			theScene->insert( obj );

			opengl::CSetOfObjectsPtr obj_corner = opengl::stock_objects::CornerXYZSimple(1,3);
			obj_corner->setLocation(off_x,0,0);
			theScene->insert( obj_corner );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("CEllipsoidInverseDepth3D");
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

	// CSimpleLine
	{
		{
			opengl::CSimpleLinePtr obj = opengl::CSimpleLine::Create();
			obj->setLocation(off_x,0,0);

			obj->setLineCoords(
				mrpt::random::randomGenerator.drawUniform(-5,5),mrpt::random::randomGenerator.drawUniform(-5,5),mrpt::random::randomGenerator.drawUniform(-5,5),
				mrpt::random::randomGenerator.drawUniform(-5,5),mrpt::random::randomGenerator.drawUniform(-5,5),mrpt::random::randomGenerator.drawUniform(-5,5) );

			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("CSimpleLine");
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


	// stock_objects::Hokuyo_URG
	{
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::Hokuyo_URG();
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("stock_objects::Hokuyo_URG()");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// stock_objects::Hokuyo_UTM
	{
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::Hokuyo_UTM();
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("stock_objects::Hokuyo_UTM()");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// stock_objects::Househam_Sprayer
	{
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::Househam_Sprayer();
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("stock_objects::Househam_Sprayer()");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	// stock_objects::RobotRhodon
	{
		{
			opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotRhodon();
			obj->setLocation(off_x,0,0);
			theScene->insert( obj );
		}

		opengl::CTextPtr gl_txt = opengl::CText::Create("stock_objects::RobotRhodon()");
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
