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

#include <mrpt/opengl.h>  // Precompiled header

#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/C3DSScene.h>


#include <mrpt/system.h>
#include <mrpt/utils/CFileOutputStream.h>

using namespace mrpt::utils;
using namespace mrpt::opengl;


/*---------------------------------------------------------------
					RobotPioneer
  ---------------------------------------------------------------*/
CSetOfObjectsPtr stock_objects::RobotPioneer()
{
	CSetOfObjectsPtr	ret = CSetOfObjects::Create();

	ret->setName("theRobot");

	CSetOfTrianglesPtr obj = CSetOfTriangles::Create();

	// Add triangles:
	CSetOfTriangles::TTriangle	trian;

	trian.r[0]=trian.r[1]=trian.r[2]= 1;
	trian.g[0]=trian.g[1]=trian.g[2]= 0;
	trian.b[0]=trian.b[1]=trian.b[2]= 0;
	trian.a[0]=trian.a[1]=trian.a[2]= 1;

	trian.x[0] = 0.10f; trian.x[1] =-0.20f; trian.x[2] =-0.20f;
	trian.y[0] =-0.10f; trian.y[1] = 0.10f; trian.y[2] =-0.10f;
	trian.z[0] = 0.20f; trian.z[1] = 0.25f; trian.z[2] = 0.25f;
	obj->insertTriangle( trian );	// 0
	trian.x[0] = 0.10f; trian.x[1] = 0.10f; trian.x[2] =-0.20f;
	trian.y[0] =-0.10f; trian.y[1] = 0.10f; trian.y[2] = 0.10f;
	trian.z[0] = 0.20f; trian.z[1] = 0.20f; trian.z[2] = 0.25f;
	obj->insertTriangle( trian );	// 1

	//trian.r = 0.9f; trian.g = 0; trian.b = 0; trian.a = 1;

	trian.x[0] = 0.10f; trian.x[1] = 0.10f; trian.x[2] = 0.10f;
	trian.y[0] =-0.10f; trian.y[1] =-0.10f; trian.y[2] = 0.10f;
	trian.z[0] = 0.05f; trian.z[1] = 0.20f; trian.z[2] = 0.20f;
	obj->insertTriangle( trian );	// 2
	trian.x[0] = 0.10f; trian.x[1] = 0.10f; trian.x[2] = 0.10f;
	trian.y[0] =-0.10f; trian.y[1] = 0.10f; trian.y[2] = 0.10f;
	trian.z[0] = 0.05f; trian.z[1] = 0.05f; trian.z[2] = 0.20f;
	obj->insertTriangle( trian );	// 3

	trian.x[0] =-0.20f; trian.x[1] =-0.20f; trian.x[2] =-0.20f;
	trian.y[0] =-0.10f; trian.y[1] =-0.10f; trian.y[2] = 0.10f;
	trian.z[0] = 0.05f; trian.z[1] = 0.25f; trian.z[2] = 0.25f;
	obj->insertTriangle( trian );	// 2b
	trian.x[0] =-0.20f; trian.x[1] =-0.20f; trian.x[2] =-0.20f;
	trian.y[0] =-0.10f; trian.y[1] = 0.10f; trian.y[2] = 0.10f;
	trian.z[0] = 0.05f; trian.z[1] = 0.05f; trian.z[2] = 0.25f;
	obj->insertTriangle( trian );	// 3b

	//trian.r = 0.8f; trian.g = 0; trian.b = 0; trian.a = 1;
	trian.x[0] = 0.10f; trian.x[1] =-0.20f; trian.x[2] =-0.20f;
	trian.y[0] =-0.10f; trian.y[1] =-0.10f; trian.y[2] =-0.10f;
	trian.z[0] = 0.20f; trian.z[1] = 0.25f; trian.z[2] = 0.05f;
	obj->insertTriangle( trian );	// 4
	trian.x[0] = 0.10f; trian.x[1] = 0.10f; trian.x[2] =-0.20f;
	trian.y[0] =-0.10f; trian.y[1] =-0.10f; trian.y[2] =-0.10f;
	trian.z[0] = 0.20f; trian.z[1] = 0.05f; trian.z[2] = 0.05f;
	obj->insertTriangle( trian );	// 5

	trian.x[0] = 0.10f; trian.x[1] =-0.20f; trian.x[2] =-0.20f;
	trian.y[0] = 0.10f; trian.y[1] = 0.10f; trian.y[2] = 0.10f;
	trian.z[0] = 0.20f; trian.z[1] = 0.25f; trian.z[2] = 0.05f;
	obj->insertTriangle( trian );	// 6
	trian.x[0] = 0.10f; trian.x[1] = 0.10f; trian.x[2] =-0.20f;
	trian.y[0] = 0.10f; trian.y[1] = 0.10f; trian.y[2] = 0.10f;
	trian.z[0] = 0.20f; trian.z[1] = 0.05f; trian.z[2] = 0.05f;
	obj->insertTriangle( trian );	// 7

	trian.r[0]=trian.r[1]=trian.r[2]= 0.05f;
	trian.g[0]=trian.g[1]=trian.g[2]= 0.05f;
	trian.b[0]=trian.b[1]=trian.b[2]= 0.05f;
	trian.a[0]=trian.a[1]=trian.a[2]= 1;

	trian.x[0] = 0.00f; trian.x[1] = 0.00f; trian.x[2] = 0.05f;
	trian.y[0] = 0.11f; trian.y[1] = 0.11f; trian.y[2] = 0.11f;
	trian.z[0] = 0.00f; trian.z[1] = 0.10f; trian.z[2] = 0.05f;
	obj->insertTriangle( trian );	// 8
	trian.x[0] = 0.00f; trian.x[1] = 0.00f; trian.x[2] = -0.05f;
	trian.y[0] = 0.11f; trian.y[1] = 0.11f; trian.y[2] = 0.11f;
	trian.z[0] = 0.00f; trian.z[1] = 0.10f; trian.z[2] = 0.05f;
	obj->insertTriangle( trian );	// 9

	trian.x[0] = 0.00f; trian.x[1] = 0.00f; trian.x[2] = 0.05f;
	trian.y[0] =-0.11f; trian.y[1] =-0.11f; trian.y[2] =-0.11f;
	trian.z[0] = 0.00f; trian.z[1] = 0.10f; trian.z[2] = 0.05f;
	obj->insertTriangle( trian );	// 10
	trian.x[0] = 0.00f; trian.x[1] = 0.00f; trian.x[2] = -0.05f;
	trian.y[0] =-0.11f; trian.y[1] =-0.11f; trian.y[2] =-0.11f;
	trian.z[0] = 0.00f; trian.z[1] = 0.10f; trian.z[2] = 0.05f;
	obj->insertTriangle( trian );	// 11

	ret->insert( obj );

	return ret;
}


/*---------------------------------------------------------------
					CornerXYZ
  ---------------------------------------------------------------*/
CSetOfObjectsPtr stock_objects::CornerXYZ(float scale)
{
	CSetOfObjectsPtr ret = CSetOfObjects::Create();

	CArrowPtr obj = CArrow::Create(
			0,0,0,
			scale,0,0,
			0.25f*scale,0.02f*scale,0.05f*scale);

	obj->setColor(1,0,0);

	ret->insert( obj );

	obj = CArrow::Create(
			0,0,0,
			0,scale,0,
			0.25f*scale,0.02f*scale,0.05f*scale);
	obj->setColor(0,1,0);

	ret->insert( obj );

	obj = CArrow::Create(
		0,0,0,
		0,0,scale,
		0.25f*scale,0.02f*scale,0.05f*scale);
	obj->setColor(0,0,1);

	ret->insert( obj );

	return ret;
}
CSetOfObjectsPtr stock_objects::CornerXYZEye()
{
	CSetOfObjectsPtr ret = CSetOfObjects::Create();
	CPose3D rotation;

	CArrowPtr obj = CArrow::Create(
			0,0,0,
			1.0,0,0,
			0.25f,0.02f,0.05f);

	obj->setColor(1,0,0);

	ret->insert( obj );

	obj = CArrow::Create(
			0,0,0,
			0,1.0,0,
			0.25f,0.02f,0.05f);
	obj->setColor(0,1,0);

	ret->insert( obj );

	obj = CArrow::Create(
		0,0,-1.0,
		0,0,0,
		0.25f,0.02f,0.05f);
	obj->setColor(0,0,1);

	ret->insert( obj );

	return ret;
}

/*---------------------------------------------------------------
					BumblebeeCamera
  ---------------------------------------------------------------*/
CSetOfObjectsPtr stock_objects::BumblebeeCamera()
{

	CSetOfObjectsPtr camera = opengl::CSetOfObjects::Create();

	CPolyhedronPtr rect = opengl::CPolyhedron::CreateCubicPrism( -0.02, 0.14, -0.02, 0.02, 0, -0.04 );
	rect->setColor( 1, 0.8, 0 );

	camera->insert( rect );

	CCylinderPtr lCam = opengl::CCylinder::Create( 0.01,0.01, 0.003, 10, 10 );
	lCam->setColor( 1,0,0 );

	CCylinderPtr rCam = opengl::CCylinder::Create( 0.01,0.01, 0.003, 10, 10 );
	rCam->setPose( CPose3D(0.12,0,0) );
	rCam->setColor( 0,0,0 );

	camera->insert( lCam );
	camera->insert( rCam );

	return camera;
}


CSetOfObjectsPtr stock_objects::CornerXYZSimple(float scale, float lineWidth)
{
	CSetOfObjectsPtr ret = CSetOfObjects::Create();

	{
		CSimpleLinePtr lin = CSimpleLine::Create();
		lin->setLineWidth(lineWidth);
		lin->setColor(1,0,0);
		lin->setLineCoords(0,0,0, scale,0,0);
		ret->insert(lin);
	}
	{
		CSimpleLinePtr lin = CSimpleLine::Create();
		lin->setLineWidth(lineWidth);
		lin->setColor(0,1,0);
		lin->setLineCoords(0,0,0, 0,scale,0);
		ret->insert(lin);
	}
	{
		CSimpleLinePtr lin = CSimpleLine::Create();
		lin->setLineWidth(lineWidth);
		lin->setColor(0,0,1);
		lin->setLineCoords(0,0,0, 0,0,scale);
		ret->insert(lin);
	}
    return ret;
}

CSetOfObjectsPtr stock_objects::CornerXYSimple(float scale, float lineWidth)
{
	CSetOfObjectsPtr ret = CSetOfObjects::Create();

	{
		CSimpleLinePtr lin = CSimpleLine::Create();
		lin->setLineWidth(lineWidth);
		lin->setColor(1,0,0);
		lin->setLineCoords(0,0,0, scale,0,0);
		ret->insert(lin);
	}
	{
		CSimpleLinePtr lin = CSimpleLine::Create();
		lin->setLineWidth(lineWidth);
		lin->setColor(0,1,0);
		lin->setLineCoords(0,0,0, 0,scale,0);
		ret->insert(lin);
	}
    return ret;
}
