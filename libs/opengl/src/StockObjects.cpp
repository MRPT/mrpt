/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

/*---------------------------------------------------------------
					RobotPioneer
  ---------------------------------------------------------------*/
CSetOfObjects::Ptr stock_objects::RobotPioneer()
{
	CSetOfObjects::Ptr ret = std::make_shared<CSetOfObjects>();

	ret->setName("theRobot");

	CSetOfTriangles::Ptr obj = std::make_shared<CSetOfTriangles>();

	// Add triangles:
	mrpt::opengl::TTriangle t;

	t.r(0) = t.r(1) = t.r(2) = 1;
	t.g(0) = t.g(1) = t.g(2) = 0;
	t.b(0) = t.b(1) = t.b(2) = 0;
	t.a(0) = t.a(1) = t.a(2) = 1;

	t.vertex(0) = {0.10f, -0.10f, 0.20f};
	t.vertex(1) = {-0.20f, 0.10f, 0.25f};
	t.vertex(2) = {-0.20f, -0.10f, 0.25f};
	obj->insertTriangle(t);  // 0
	t.vertex(0) = {0.10f, -0.10f, 0.20f};
	t.vertex(1) = {0.10f, 0.10f, 0.20f};
	t.vertex(2) = {-0.20f, 0.10f, 0.25f};
	obj->insertTriangle(t);  // 1

	t.vertex(0) = {0.10f, -0.10f, 0.05f};
	t.vertex(1) = {0.10f, 0.10f, 0.20f};
	t.vertex(2) = {0.10f, -0.10f, 0.20f};
	obj->insertTriangle(t);  // 2
	t.vertex(0) = {0.10f, -0.10f, 0.05f};
	t.vertex(1) = {0.10f, 0.10f, 0.05f};
	t.vertex(2) = {0.10f, 0.10f, 0.20f};
	obj->insertTriangle(t);  // 3

	t.vertex(0) = {-0.20f, -0.10f, 0.05f};
	t.vertex(1) = {-0.20f, -0.10f, 0.25f};
	t.vertex(2) = {-0.20f, 0.10f, 0.25f};
	obj->insertTriangle(t);  // 2b
	t.vertex(0) = {-0.20f, -0.10f, 0.05f};
	t.vertex(1) = {-0.20f, 0.10f, 0.25f};
	t.vertex(2) = {-0.20f, 0.10f, 0.05f};
	obj->insertTriangle(t);  // 3b

	t.vertex(0) = {0.10f, -0.10f, 0.20f};
	t.vertex(1) = {-0.20f, -0.10f, 0.25f};
	t.vertex(2) = {-0.20f, -0.10f, 0.05f};
	obj->insertTriangle(t);  // 4

	t.vertex(0) = {0.10f, -0.10f, 0.20f};
	t.vertex(1) = {-0.20f, -0.10f, 0.05f};
	t.vertex(2) = {0.10f, -0.10f, 0.05f};
	obj->insertTriangle(t);  // 5

	t.vertex(0) = {0.10f, 0.10f, 0.20f};
	t.vertex(1) = {-0.20f, 0.10f, 0.05f};
	t.vertex(2) = {-0.20f, 0.10f, 0.25f};
	obj->insertTriangle(t);  // 6

	t.vertex(0) = {0.10f, 0.10f, 0.20f};
	t.vertex(1) = {0.10f, 0.10f, 0.05f};
	t.vertex(2) = {-0.20f, 0.10f, 0.05f};
	obj->insertTriangle(t);  // 7

	t.setColor(mrpt::img::TColorf(0.05f, 0.05f, 0.05f, 1));

	t.vertex(0) = {0.00f, 0.11f, 0.00f};
	t.vertex(1) = {0.00f, 0.11f, 0.10f};
	t.vertex(2) = {0.05f, 0.11f, 0.05f};
	obj->insertTriangle(t);  // 8
	t.vertex(0) = {0.00f, 0.11f, 0.00f};
	t.vertex(1) = {0.00f, 0.11f, 0.10f};
	t.vertex(2) = {-0.05f, 0.11f, 0.05f};
	obj->insertTriangle(t);  // 9

	t.vertex(0) = {0.00f, -0.11f, 0.00f};
	t.vertex(1) = {0.00f, -0.11f, 0.10f};
	t.vertex(2) = {0.05f, -0.11f, 0.05f};
	obj->insertTriangle(t);  // 10
	t.vertex(0) = {0.00f, -0.11f, 0.00f};
	t.vertex(1) = {0.00f, -0.11f, 0.10f};
	t.vertex(2) = {-0.05f, -0.11f, 0.05f};
	obj->insertTriangle(t);  // 11

	ret->insert(obj);

	return ret;
}

/*---------------------------------------------------------------
					CornerXYZ
  ---------------------------------------------------------------*/
CSetOfObjects::Ptr stock_objects::CornerXYZ(float scale)
{
	CSetOfObjects::Ptr ret = std::make_shared<CSetOfObjects>();

	CArrow::Ptr obj = CArrow::Create(
		0, 0, 0, scale, 0, 0, 0.25f * scale, 0.02f * scale, 0.05f * scale);

	obj->setColor(1, 0, 0);

	ret->insert(obj);

	obj = CArrow::Create(
		0, 0, 0, 0, scale, 0, 0.25f * scale, 0.02f * scale, 0.05f * scale);
	obj->setColor(0, 1, 0);

	ret->insert(obj);

	obj = CArrow::Create(
		0, 0, 0, 0, 0, scale, 0.25f * scale, 0.02f * scale, 0.05f * scale);
	obj->setColor(0, 0, 1);

	ret->insert(obj);

	return ret;
}

/*---------------------------------------------------------------
					RobotRhodon
  ---------------------------------------------------------------*/
CSetOfObjects::Ptr stock_objects::RobotRhodon()
{
	CSetOfObjects::Ptr ret = std::make_shared<CSetOfObjects>();
	float height = 0;

	vector<TPoint2D> level1;
	level1.emplace_back(0.31, 0);
	level1.emplace_back(0.22, 0.24);
	level1.emplace_back(-0.22, 0.24);
	level1.emplace_back(-0.31, 0);
	level1.emplace_back(-0.22, -0.24);
	level1.emplace_back(0.22, -0.24);

	CPolyhedron::Ptr obj1 =
		opengl::CPolyhedron::CreateCustomPrism(level1, 0.38);
	obj1->setLocation(0, 0, height);
	height += 0.38f;
	obj1->setColor(0.6f, 0.6f, 0.6f);
	ret->insert(obj1);

	vector<TPoint2D> level2;
	level2.emplace_back(0.16, 0.21);
	level2.emplace_back(-0.16, 0.21);
	level2.emplace_back(-0.16, -0.21);
	level2.emplace_back(0.16, -0.21);

	CPolyhedron::Ptr obj2 =
		opengl::CPolyhedron::CreateCustomPrism(level2, 0.35);
	obj2->setLocation(0, 0, height);
	height += 0.35f;
	obj2->setColor(0.2f, 0.2f, 0.2f);
	ret->insert(obj2);

	vector<TPoint2D> level3;
	level3.emplace_back(-0.12, 0.12);
	level3.emplace_back(-0.16, 0.12);
	level3.emplace_back(-0.16, -0.12);
	level3.emplace_back(-0.12, -0.12);

	CPolyhedron::Ptr obj3 = opengl::CPolyhedron::CreateCustomPrism(level3, 1);
	obj3->setLocation(0, 0, height);
	// height+=1;
	obj3->setColor(0.6f, 0.6f, 0.6f);
	ret->insert(obj3);

	opengl::CCylinder::Ptr obj4 =
		std::make_shared<opengl::CCylinder>(0.05f, 0.05f, 0.4f, 20);
	obj4->setLocation(0, 0, 0.73);
	obj4->setColor(0.0f, 0.0f, 0.9f);
	ret->insert(obj4);

	opengl::CCylinder::Ptr obj5 =
		std::make_shared<opengl::CCylinder>(0.05f, 0.05f, 0.4f, 20);
	obj5->setPose(CPose3D(0.32, 0, 0.89, 0, -1, 0));
	obj5->setColor(0.0f, 0.0f, 0.9f);
	ret->insert(obj5);

	return ret;
}

/*---------------------------------------------------------------
					RobotGiraff
  ---------------------------------------------------------------*/
CSetOfObjects::Ptr stock_objects::RobotGiraff()
{
	CSetOfObjects::Ptr ret = std::make_shared<CSetOfObjects>();
	float height = 0;

	// Base
	vector<TPoint2D> level1;
	level1.emplace_back(0.31, 0);
	level1.emplace_back(0.22, 0.24);
	level1.emplace_back(-0.22, 0.24);
	level1.emplace_back(-0.31, 0);
	level1.emplace_back(-0.22, -0.24);
	level1.emplace_back(0.22, -0.24);

	CPolyhedron::Ptr obj1 =
		opengl::CPolyhedron::CreateCustomPrism(level1, 0.23);
	obj1->setLocation(0, 0, height);
	height += 0.23f;
	obj1->setColor(1.0f, 0.6f, 0.0f);
	ret->insert(obj1);

	// Electronic's cage
	vector<TPoint2D> level2;
	level2.emplace_back(0.13, 0.1);
	level2.emplace_back(-0.13, 0.1);
	level2.emplace_back(-0.13, -0.1);
	level2.emplace_back(0.13, -0.1);

	CPolyhedron::Ptr obj2 =
		opengl::CPolyhedron::CreateCustomPrism(level2, 0.45);
	obj2->setLocation(0, 0, height);
	height += 0.45f;
	obj2->setColor(1.0f, 0.6f, 0.2f);
	ret->insert(obj2);

	// Neck
	vector<TPoint2D> level3;
	level3.emplace_back(0.03, 0.03);
	level3.emplace_back(-0.03, 0.03);
	level3.emplace_back(-0.03, -0.03);
	level3.emplace_back(0.03, -0.03);

	CPolyhedron::Ptr obj3 =
		opengl::CPolyhedron::CreateCustomPrism(level3, 0.55);
	obj3->setLocation(0, 0, height);
	height += 0.55f;
	obj3->setColor(0.6f, 0.6f, 0.6f);
	ret->insert(obj3);

	// Screen
	vector<TPoint2D> level4;
	level4.emplace_back(0.03, 0.11);
	level4.emplace_back(-0.03, 0.11);
	level4.emplace_back(-0.03, -0.11);
	level4.emplace_back(0.03, -0.11);

	CPolyhedron::Ptr obj4 = opengl::CPolyhedron::CreateCustomPrism(level4, 0.4);
	obj4->setLocation(0, 0, height);
	obj4->setColor(1.0f, 0.6f, 0.0f);
	ret->insert(obj4);

	return ret;
}

CSetOfObjects::Ptr stock_objects::CornerXYZEye()
{
	CSetOfObjects::Ptr ret = std::make_shared<CSetOfObjects>();
	CPose3D rotation;

	CArrow::Ptr obj = CArrow::Create(0, 0, 0, 1.0, 0, 0, 0.25f, 0.02f, 0.05f);

	obj->setColor(1, 0, 0);

	ret->insert(obj);

	obj = CArrow::Create(0, 0, 0, 0, 1.0, 0, 0.25f, 0.02f, 0.05f);
	obj->setColor(0, 1, 0);

	ret->insert(obj);

	obj = CArrow::Create(0, 0, -1.0, 0, 0, 0, 0.25f, 0.02f, 0.05f);
	obj->setColor(0, 0, 1);

	ret->insert(obj);

	return ret;
}

/*---------------------------------------------------------------
					BumblebeeCamera
  ---------------------------------------------------------------*/
CSetOfObjects::Ptr stock_objects::BumblebeeCamera()
{
	CSetOfObjects::Ptr camera = std::make_shared<opengl::CSetOfObjects>();

	CPolyhedron::Ptr rect = opengl::CPolyhedron::CreateCubicPrism(
		-0.02, 0.14, -0.02, 0.02, 0, -0.04);
	rect->setColor(1.0f, 0.8f, 0.0f);

	camera->insert(rect);

	CCylinder::Ptr lCam =
		std::make_shared<opengl::CCylinder>(0.01f, 0.01f, 0.003f, 10);
	lCam->setColor(1, 0, 0);

	CCylinder::Ptr rCam =
		std::make_shared<opengl::CCylinder>(0.01f, 0.01f, 0.003f, 10);
	rCam->setPose(CPose3D(0.12, 0, 0));
	rCam->setColor(0, 0, 0);

	camera->insert(lCam);
	camera->insert(rCam);

	return camera;
}

CSetOfObjects::Ptr stock_objects::CornerXYZSimple(float scale, float lineWidth)
{
	CSetOfObjects::Ptr ret = std::make_shared<CSetOfObjects>();

	{
		CSimpleLine::Ptr lin = std::make_shared<CSimpleLine>();
		lin->setLineWidth(lineWidth);
		lin->setColor(1, 0, 0);
		lin->setLineCoords(0, 0, 0, scale, 0, 0);
		ret->insert(lin);
	}
	{
		CSimpleLine::Ptr lin = std::make_shared<CSimpleLine>();
		lin->setLineWidth(lineWidth);
		lin->setColor(0, 1, 0);
		lin->setLineCoords(0, 0, 0, 0, scale, 0);
		ret->insert(lin);
	}
	{
		CSimpleLine::Ptr lin = std::make_shared<CSimpleLine>();
		lin->setLineWidth(lineWidth);
		lin->setColor(0, 0, 1);
		lin->setLineCoords(0, 0, 0, 0, 0, scale);
		ret->insert(lin);
	}
	return ret;
}

CSetOfObjects::Ptr stock_objects::CornerXYSimple(float scale, float lineWidth)
{
	CSetOfObjects::Ptr ret = std::make_shared<CSetOfObjects>();

	{
		CSimpleLine::Ptr lin = std::make_shared<CSimpleLine>();
		lin->setLineWidth(lineWidth);
		lin->setColor(1, 0, 0);
		lin->setLineCoords(0, 0, 0, scale, 0, 0);
		ret->insert(lin);
	}
	{
		CSimpleLine::Ptr lin = std::make_shared<CSimpleLine>();
		lin->setLineWidth(lineWidth);
		lin->setColor(0, 1, 0);
		lin->setLineCoords(0, 0, 0, 0, scale, 0);
		ret->insert(lin);
	}
	return ret;
}

CSetOfObjects::Ptr stock_objects::Hokuyo_URG()
{
	CSetOfObjects::Ptr ret = std::make_shared<CSetOfObjects>();

	{
		CBox::Ptr base = std::make_shared<CBox>(
			TPoint3D(-0.025, -0.025, -0.0575), TPoint3D(0.025, 0.025, -0.0185));
		base->setColor(0.7f, 0.7f, 0.7f);
		ret->insert(base);
	}
	{
		CCylinder::Ptr cyl1 = std::make_shared<CCylinder>(0.02f, 0.02f, 0.01f);
		cyl1->setColor(0, 0, 0);
		cyl1->setLocation(0, 0, -0.014);
		ret->insert(cyl1);
	}
	{
		CCylinder::Ptr cyl2 =
			std::make_shared<CCylinder>(0.02f, 0.0175f, 0.01f);
		cyl2->setColor(0, 0, 0);
		cyl2->setLocation(0, 0, -0.004);
		ret->insert(cyl2);
	}
	{
		CCylinder::Ptr cyl3 =
			std::make_shared<CCylinder>(0.0175f, 0.0175f, 0.01f);
		cyl3->setColor(0, 0, 0);
		cyl3->setLocation(0, 0, 0.004);
		ret->insert(cyl3);
	}

	return ret;
}

CSetOfObjects::Ptr stock_objects::Hokuyo_UTM()
{
	CSetOfObjects::Ptr ret = std::make_shared<CSetOfObjects>();

	{
		CBox::Ptr base = std::make_shared<CBox>(
			TPoint3D(-0.03, -0.03, -0.055), TPoint3D(0.03, 0.03, -0.014));
		base->setColor(0, 0, 0);
		ret->insert(base);
	}
	{
		CCylinder::Ptr cyl1 =
			std::make_shared<CCylinder>(0.028f, 0.024f, 0.028f);
		cyl1->setColor(0, 0, 0);
		cyl1->setPose(CPose3D(0, 0, -0.014));
		ret->insert(cyl1);
	}
	{
		CCylinder::Ptr cyl2 =
			std::make_shared<CCylinder>(0.028f, 0.028f, 0.01f);
		cyl2->setColor(1.0f, 69.0f / 255.0f, 0);
		cyl2->setLocation(0, 0, 0.014);
		ret->insert(cyl2);
	}
	{
		CCylinder::Ptr cyl3 =
			std::make_shared<CCylinder>(0.028f, 0.028f, 0.01f);
		cyl3->setColor(0, 0, 0);
		cyl3->setLocation(0, 0, 0.024);
		ret->insert(cyl3);
	}

	return ret;
}

CSetOfObjects::Ptr stock_objects::Househam_Sprayer()
{
	CSetOfObjects::Ptr ret = std::make_shared<CSetOfObjects>();

	{
		CBox::Ptr cabin = std::make_shared<CBox>(
			TPoint3D(0.878, 0.723, -0.12), TPoint3D(-0.258, -0.723, -1.690));
		cabin->setColor(0.7f, 0.7f, 0.7f);
		ret->insert(cabin);
	}
	{
		CBox::Ptr back = std::make_shared<CBox>(
			TPoint3D(-0.258, 0.723, -0.72), TPoint3D(-5.938, -0.723, -1.690));
		back->setColor(1, 1, 1);
		ret->insert(back);
	}
	{
		CBox::Ptr boomAxis = std::make_shared<CBox>(
			TPoint3D(-5.938, 0.723, -1.0), TPoint3D(-6.189, -0.723, -1.690));
		boomAxis->setColor(0, 0, 0);
		ret->insert(boomAxis);
	}
	{
		CBox::Ptr boom1 = std::make_shared<CBox>(
			TPoint3D(-5.938, 0.723, -1.0), TPoint3D(-6.189, 11.277, -1.620));
		boom1->setColor(0, 1, 0);
		ret->insert(boom1);
	}
	{
		CBox::Ptr boom2 = std::make_shared<CBox>(
			TPoint3D(-5.938, -0.723, -1.0), TPoint3D(-6.189, -11.277, -1.620));
		boom2->setColor(0, 1, 0);
		ret->insert(boom2);
	}
	{
		CCylinder::Ptr cyl1 =
			std::make_shared<CCylinder>(0.716f, 0.716f, 0.387f, 30);
		cyl1->setColor(0, 0, 0);
		cyl1->setPose(CPose3D(-0.710, 0.923, -2.480, 0, 0, 90.0_deg));
		ret->insert(cyl1);
	}
	{
		CCylinder::Ptr cyl2 =
			std::make_shared<CCylinder>(0.716f, 0.716f, 0.387f, 30);
		cyl2->setColor(0, 0, 0);
		cyl2->setPose(CPose3D(-3.937, 0.923, -2.480, 0, 0, 90.0_deg));
		ret->insert(cyl2);
	}
	{
		CCylinder::Ptr cyl1 =
			std::make_shared<CCylinder>(0.716f, 0.716f, 0.387f, 30);
		cyl1->setColor(0, 0, 0);
		cyl1->setPose(CPose3D(-0.710, -0.423, -2.480, 0, 0, 90.0_deg));
		ret->insert(cyl1);
	}
	{
		CCylinder::Ptr cyl2 =
			std::make_shared<CCylinder>(0.716f, 0.716f, 0.387f, 30);
		cyl2->setColor(0, 0, 0);
		cyl2->setPose(CPose3D(-3.937, -0.423, -2.480, 0, 0, 90.0_deg));
		ret->insert(cyl2);
	}
	return ret;
}
