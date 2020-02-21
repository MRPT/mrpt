/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/**
 * rayTrace
 * Ray tracing is a technique for generating an image by tracing the path of
 * light through pixels in an image plane and simulating the effects of its
 * encounters with virtual objects
 *
 */

#include <mrpt/gui.h>
#include <mrpt/opengl/CAngularObservationMesh.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random.h>
#include <mrpt/system/CTicTac.h>
#include <iostream>

#define COLORR 1.0f
#define COLORG 0.0f
#define COLORB 0.0f

#define GRID_R 1.0f
#define GRID_G 1.0f
#define GRID_B 1.0f

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::serialization;
using namespace mrpt::system;

using mrpt::opengl::CAngularObservationMesh;

// Increase this values to get more precision. It will also increase run time.
const size_t HOW_MANY_YAWS = 150;
const size_t HOW_MANY_PITCHS = 75;

const float RANDOM_POSE_DISTANCE = 10;

inline double MYRAND1()
{
	//	return static_cast<float>(rand()%prec)/prec;
	return getRandomGenerator().drawUniform(0, 1);
}

inline double MYRANDG(double scale, double shift = 0)
{
	//	return shift+(static_cast<float>(rand()%prec)/prec)*scale;
	return shift + scale * getRandomGenerator().drawUniform(0, 1);
}

CPose3D randomPose()
{
	return CPose3D(
		MYRANDG(2 * RANDOM_POSE_DISTANCE, -RANDOM_POSE_DISTANCE),
		MYRANDG(2 * RANDOM_POSE_DISTANCE, -RANDOM_POSE_DISTANCE),
		MYRANDG(2 * RANDOM_POSE_DISTANCE, -RANDOM_POSE_DISTANCE), MYRAND1(),
		MYRAND1(), MYRAND1());
}

/**
 * Call configRandom given the address of an object and assign random pose and
 * color to it
 */
void configRandom(const CRenderizable::Ptr& obj)
{
	obj->setColor(MYRAND1(), MYRAND1(), MYRAND1(), MYRANDG(0.75, 0.25));
	obj->setPose(randomPose());
}

void guideLines(const CPose3D& base, CSetOfLines::Ptr& lines, float dist)
{
	CPoint3D pDist = CPoint3D(dist, 0, 0);
	CPoint3D pps[4];
	pps[0] = base + pDist;
	pps[1] = base + CPose3D(0, 0, 0, 0, -M_PI / 2, 0) + pDist;
	pps[2] = base + CPose3D(0, 0, 0, -M_PI / 2, 0, 0) + pDist;
	pps[3] = base + CPose3D(0, 0, 0, M_PI / 2, 0, 0) + pDist;
	for (size_t i = 0; i < 4; i++)
		lines->appendLine(
			base.x(), base.y(), base.z(), pps[i].x(), pps[i].y(), pps[i].z());
	lines->setLineWidth(5);
	lines->setColor(0, 0, 1);
}

// Add objects at your will to check results
void generateObjects(CSetOfObjects::Ptr& world)
{
	// create object, give it a random pose/color, insert it in the world
	CDisk::Ptr dsk = CDisk::Create();
	dsk->setDiskRadius(MYRANDG(5, 5), MYRANDG(5));
	configRandom(dsk);
	world->insert(dsk);

	CSphere::Ptr sph = CSphere::Create(MYRANDG(5, 1));
	configRandom(sph);
	world->insert(sph);

	CTexturedPlane::Ptr pln = CTexturedPlane::Create(
		MYRANDG(10, -10), MYRANDG(10), MYRANDG(10, -10), MYRANDG(10));
	configRandom(pln);
	world->insert(pln);

	for (size_t i = 0; i < 5; i++)
	{
		CPolyhedron::Ptr poly =
			CPolyhedron::CreateRandomPolyhedron(MYRANDG(2, 2));
		configRandom(poly);
		world->insert(poly);
	}

	CCylinder::Ptr cil = CCylinder::Create(
		MYRANDG(3.0, 3.0), MYRANDG(3.0, 1.0), MYRANDG(2.0f, 3.0f), 50, 1);
	configRandom(cil);
	world->insert(cil);

	CEllipsoid3D::Ptr ell = CEllipsoid3D::Create();
	CMatrixDouble md = CMatrixDouble(3, 3);
	for (size_t i = 0; i < 3; i++) md(i, i) = MYRANDG(8.0, 1.0);
	for (size_t i = 0; i < 3; i++)
	{
		size_t ii = (i + 1) % 3;
		md(i, ii) = md(ii, i) = MYRANDG(sqrt(md(i, i) * md(ii, ii)));
	}
	ell->setCovMatrix(md);
	configRandom(std::dynamic_pointer_cast<CRenderizable>(ell));
	world->insert(ell);
}

void display()
{
	CDisplayWindow3D window("Ray trace demo", 640, 480);
	window.setPos(10, 10);
	std::this_thread::sleep_for(20ms);
	COpenGLScene::Ptr scene1 = COpenGLScene::Create();
	// COpenGLScene::Ptr &scene1=window.get3DSceneAndLock();
	opengl::CGridPlaneXY::Ptr plane1 =
		CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
	plane1->setColor(GRID_R, GRID_G, GRID_B);
	scene1->insert(plane1);
	scene1->insert(CAxis::Create(-5, -5, -5, 5, 5, 5, 2.5, 3, true));
	CSetOfObjects::Ptr world = CSetOfObjects::Create();
	generateObjects(world);
	scene1->insert(world);
	CPose3D basePose = randomPose();
	CAngularObservationMesh::Ptr aom = CAngularObservationMesh::Create();
	CTicTac t;
	t.Tic();
	CAngularObservationMesh::trace2DSetOfRays(
		scene1, basePose, aom,
		CAngularObservationMesh::TDoubleRange::CreateFromAmount(
			-M_PI / 2, 0, HOW_MANY_PITCHS),
		CAngularObservationMesh::TDoubleRange::CreateFromAperture(
			M_PI, HOW_MANY_YAWS));
	cout << "Elapsed time: " << t.Tac() << " seconds.\n";
	aom->setColor(0, 1, 0);
	aom->setWireframe(true);
	// Comment to stop showing traced rays and scan range guidelines.
	CSetOfLines::Ptr traced = CSetOfLines::Create();
	CSetOfLines::Ptr guides = CSetOfLines::Create();
	aom->getTracedRays(traced);
	traced->setLineWidth(1.5);
	traced->setColor(1, 0, 0);
	guideLines(basePose, guides, 10);
	scene1->insert(traced);
	scene1->insert(guides);

	// Uncomment to show also traced rays who got lost.
	/*
	CSetOfLines::Ptr untraced=CSetOfLines::Create();
	aom->getUntracedRays(untraced,20);
	untraced->setLineWidth(1);
	untraced->setColor(1,1,1,0.5);
	scene1->insert(untraced);
	*/
	CSphere::Ptr point = CSphere::Create(0.2);
	point->setColor(0, 1, 0);
	point->setPose(basePose);
	scene1->insert(point);
	CDisplayWindow3D window2("Observed mesh", 640, 480);
	window2.setPos(660, 10);
	std::this_thread::sleep_for(20ms);
	window.get3DSceneAndLock() = scene1;
	window.unlockAccess3DScene();
	window.setCameraElevationDeg(25.0f);
	COpenGLScene::Ptr& scene2 = window2.get3DSceneAndLock();
	scene2->insert(aom);
	opengl::CGridPlaneXY::Ptr plane2 =
		CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
	plane2->setColor(GRID_R, GRID_G, GRID_B);
	scene2->insert(plane2);
	scene2->insert(CAxis::Create(-5, -5, -5, 5, 5, 5, 2.5, 3, true));
	window2.unlockAccess3DScene();
	window2.setCameraElevationDeg(25.0f);

	window.waitForKey();
}

int main()
{
	mrpt::random::getRandomGenerator().randomize();
	try
	{
		display();
		return 0;
	}
	catch (const exception& e)
	{
		cout << "Error: " << e.what() << '.' << endl;
		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		cout << "Unknown Error.\n";
		return -1;
	}
}
