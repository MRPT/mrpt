/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/C3DSScene.h>
#include <mrpt/utils/CFileOutputStream.h>

using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;


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

/*---------------------------------------------------------------
					RobotRhodon
  ---------------------------------------------------------------*/
CSetOfObjectsPtr stock_objects::RobotRhodon()
{
	CSetOfObjectsPtr ret = CSetOfObjects::Create();
	float height = 0;

	vector<TPoint2D> level1;
	level1.push_back(TPoint2D(0.31, 0));
	level1.push_back(TPoint2D(0.22, 0.24));
	level1.push_back(TPoint2D(-0.22, 0.24));
	level1.push_back(TPoint2D(-0.31, 0));
	level1.push_back(TPoint2D(-0.22, -0.24));
	level1.push_back(TPoint2D(0.22, -0.24));

	CPolyhedronPtr obj1 = opengl::CPolyhedron::CreateCustomPrism(level1, 0.38);
	obj1->setLocation(0,0,height);
	height+=0.38f;
	obj1->setColor(0.6,0.6,0.6);
	ret->insert( obj1 );


	vector<TPoint2D> level2;
	level2.push_back(TPoint2D(0.16, 0.21));
	level2.push_back(TPoint2D(-0.16, 0.21));
	level2.push_back(TPoint2D(-0.16, -0.21));
	level2.push_back(TPoint2D(0.16, -0.21));

	CPolyhedronPtr obj2 = opengl::CPolyhedron::CreateCustomPrism(level2, 0.35);
	obj2->setLocation(0,0,height);
	height+=0.35f;
	obj2->setColor(0.2,0.2,0.2);
	ret->insert( obj2 );


	vector<TPoint2D> level3;
	level3.push_back(TPoint2D(-0.12, 0.12));
	level3.push_back(TPoint2D(-0.16, 0.12));
	level3.push_back(TPoint2D(-0.16, -0.12));
	level3.push_back(TPoint2D(-0.12, -0.12));

	CPolyhedronPtr obj3 = opengl::CPolyhedron::CreateCustomPrism(level3, 1);
	obj3->setLocation(0,0,height);
	//height+=1;
	obj3->setColor(0.6,0.6,0.6);
	ret->insert( obj3 );


	opengl::CCylinderPtr obj4 = opengl::CCylinder::Create(0.05f, 0.05f, 0.4f, 20, 20);
	obj4->setLocation(0,0,0.73);
	obj4->setColor(0,0,0.9);
	ret->insert( obj4 );

	opengl::CCylinderPtr obj5 = opengl::CCylinder::Create(0.05f, 0.05f, 0.4f, 20, 20);
	obj5->setPose(CPose3D(0.32,0,0.89,0,-1,0));
	obj5->setColor(0,0,0.9);
	ret->insert( obj5 );

	return ret;
}

/*---------------------------------------------------------------
					RobotGiraff
  ---------------------------------------------------------------*/
CSetOfObjectsPtr stock_objects::RobotGiraff()
{
	CSetOfObjectsPtr ret = CSetOfObjects::Create();
	float height = 0;
	
	//Base
	vector<TPoint2D> level1;
	level1.push_back(TPoint2D(0.31, 0));
	level1.push_back(TPoint2D(0.22, 0.24));
	level1.push_back(TPoint2D(-0.22, 0.24));
	level1.push_back(TPoint2D(-0.31, 0));
	level1.push_back(TPoint2D(-0.22, -0.24));
	level1.push_back(TPoint2D(0.22, -0.24));

	CPolyhedronPtr obj1 = opengl::CPolyhedron::CreateCustomPrism(level1, 0.23);
	obj1->setLocation(0,0,height);
	height+=0.23f;
	obj1->setColor(1.0,0.6,0.0);
	ret->insert( obj1 );

	//Electronic's cage
	vector<TPoint2D> level2;
	level2.push_back(TPoint2D(0.13, 0.1));
	level2.push_back(TPoint2D(-0.13, 0.1));
	level2.push_back(TPoint2D(-0.13, -0.1));
	level2.push_back(TPoint2D(0.13, -0.1));

	CPolyhedronPtr obj2 = opengl::CPolyhedron::CreateCustomPrism(level2, 0.45);
	obj2->setLocation(0,0,height);
	height+=0.45f;
	obj2->setColor(1.0,0.6,0.2);
	ret->insert( obj2 );

	//Neck
	vector<TPoint2D> level3;
	level3.push_back(TPoint2D(0.03, 0.03));
	level3.push_back(TPoint2D(-0.03, 0.03));
	level3.push_back(TPoint2D(-0.03, -0.03));
	level3.push_back(TPoint2D(0.03, -0.03));
	

	CPolyhedronPtr obj3 = opengl::CPolyhedron::CreateCustomPrism(level3, 0.55);
	obj3->setLocation(0,0,height);
	height+=0.55f;
	obj3->setColor(0.6,0.6,0.6);
	ret->insert( obj3 );

	//Screen
	vector<TPoint2D> level4;
	level4.push_back(TPoint2D(0.03, 0.11));
	level4.push_back(TPoint2D(-0.03, 0.11));
	level4.push_back(TPoint2D(-0.03, -0.11));
	level4.push_back(TPoint2D(0.03, -0.11));
	
	
	CPolyhedronPtr obj4 = opengl::CPolyhedron::CreateCustomPrism(level4, 0.4);
	obj4->setLocation(0,0,height);
	height+=0.4f;
	obj4->setColor(1.0,0.6,0.0);
	ret->insert( obj4 );	

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

	CCylinderPtr lCam = opengl::CCylinder::Create( 0.01f,0.01f, 0.003f, 10, 10 );
	lCam->setColor( 1,0,0 );

	CCylinderPtr rCam = opengl::CCylinder::Create( 0.01f,0.01f, 0.003f, 10, 10 );
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

CSetOfObjectsPtr stock_objects::Hokuyo_URG()
{
	CSetOfObjectsPtr ret = CSetOfObjects::Create();

	{
		CBoxPtr base = CBox::Create(TPoint3D(-0.025,-0.025,-0.0575),TPoint3D(0.025,0.025,-0.0185));
		base->setColor(0.7,0.7,0.7);
		ret->insert(base);
	}
	{
		CCylinderPtr cyl1 = CCylinder::Create(0.02f,0.02f,0.01f);
		cyl1->setColor(0,0,0);
		cyl1->setPose(CPoint3D(0,0,-0.014));
		ret->insert(cyl1);
	}
	{
		CCylinderPtr cyl2 = CCylinder::Create(0.02f,0.0175f,0.01f);
		cyl2->setColor(0,0,0);
		cyl2->setPose(CPoint3D(0,0,-0.004));
		ret->insert(cyl2);
	}
	{
		CCylinderPtr cyl3 = CCylinder::Create(0.0175f,0.0175f,0.01f);
		cyl3->setColor(0,0,0);
		cyl3->setPose(CPoint3D(0,0,0.004));
		ret->insert(cyl3);
	}

    return ret;
}

CSetOfObjectsPtr stock_objects::Hokuyo_UTM()
{
	CSetOfObjectsPtr ret = CSetOfObjects::Create();

	{
		CBoxPtr base = CBox::Create(TPoint3D(-0.03,-0.03,-0.055),TPoint3D(0.03,0.03,-0.014));
		base->setColor(0,0,0);
		ret->insert(base);
	}
	{
		CCylinderPtr cyl1 = CCylinder::Create(0.028f,0.024f,0.028f);
		cyl1->setColor(0,0,0);
		cyl1->setPose(CPose3D(0,0,-0.014));
		ret->insert(cyl1);
	}
	{
		CCylinderPtr cyl2 = CCylinder::Create(0.028f,0.028f,0.01f);
		cyl2->setColor(1,69/255.0,0);
		cyl2->setPose(CPoint3D(0,0,0.014));
		ret->insert(cyl2);
	}
	{
		CCylinderPtr cyl3 = CCylinder::Create(0.028f,0.028f,0.01f);
		cyl3->setColor(0,0,0);
		cyl3->setPose(CPoint3D(0,0,0.024));
		ret->insert(cyl3);
	}

    return ret;
}

CSetOfObjectsPtr stock_objects::Househam_Sprayer()
{
	CSetOfObjectsPtr ret = CSetOfObjects::Create();

	{
		CBoxPtr cabin = CBox::Create(TPoint3D(0.878,0.723,-0.12),TPoint3D(-0.258,-0.723,-1.690));
		cabin->setColor(0.7,0.7,0.7);
		ret->insert(cabin);
	}
	{
		CBoxPtr back = CBox::Create(TPoint3D(-0.258,0.723,-0.72),TPoint3D(-5.938,-0.723,-1.690));
		back->setColor(1,1,1);
		ret->insert(back);
	}
    {
		CBoxPtr boomAxis = CBox::Create(TPoint3D(-5.938,0.723,-1.0),TPoint3D(-6.189,-0.723,-1.690));
		boomAxis->setColor(0,0,0);
		ret->insert(boomAxis);
	}
	{
		CBoxPtr boom1 = CBox::Create(TPoint3D(-5.938,0.723,-1.0),TPoint3D(-6.189,11.277,-1.620));
		boom1->setColor(0,1,0);
		ret->insert(boom1);
	}
	{
		CBoxPtr boom2 = CBox::Create(TPoint3D(-5.938,-0.723,-1.0),TPoint3D(-6.189,-11.277,-1.620));
		boom2->setColor(0,1,0);
		ret->insert(boom2);
	}
	{
		CCylinderPtr cyl1 = CCylinder::Create(0.716f,0.716f,0.387f,30);
		cyl1->setColor(0,0,0);
		cyl1->setPose(CPose3D(-0.710,0.923,-2.480,0,0,DEG2RAD(90)));
		ret->insert(cyl1);
	}
	{
		CCylinderPtr cyl2 = CCylinder::Create(0.716f,0.716f,0.387f,30);
		cyl2->setColor(0,0,0);
		cyl2->setPose(CPose3D(-3.937,0.923,-2.480,0,0,DEG2RAD(90)));
		ret->insert(cyl2);
	}
	{
		CCylinderPtr cyl1 = CCylinder::Create(0.716f,0.716f,0.387f,30);
		cyl1->setColor(0,0,0);
		cyl1->setPose(CPose3D(-0.710,-0.423,-2.480,0,0,DEG2RAD(90)));
		ret->insert(cyl1);
	}
	{
		CCylinderPtr cyl2 = CCylinder::Create(0.716f,0.716f,0.387f,30);
		cyl2->setColor(0,0,0);
		cyl2->setPose(CPose3D(-3.937,-0.423,-2.480,0,0,DEG2RAD(90)));
		ret->insert(cyl2);
	}
    return ret;
}
