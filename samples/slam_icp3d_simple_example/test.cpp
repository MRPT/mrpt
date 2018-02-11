/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/**
 * icp3d
 * Execute an Iterative Closest Point algorithm using two 3D point clouds.
 */

#include <mrpt/slam/CICP.h>

#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CAngularObservationMesh.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/stock_objects.h>
#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;

// Increase this values to get more precision. It will also increase run time.
const size_t HOW_MANY_YAWS = 120;
const size_t HOW_MANY_PITCHS = 120;

// The scans of the 3D object, taken from 2 different places:
vector<CObservation2DRangeScan> sequence_scans1, sequence_scans2;

// The two origins for the 3D scans
CPose3D viewpoint1(-0.3, 0.7, 3, DEG2RAD(5), DEG2RAD(80), DEG2RAD(3));
CPose3D viewpoint2(0.5, -0.2, 2.6, DEG2RAD(-5), DEG2RAD(100), DEG2RAD(-7));

CPose3D SCAN2_POSE_ERROR(0.15, -0.07, 0.10, -0.03, 0.1, 0.1);

/**
 * Generate 3 objects to work with - 1 sphere, 2 disks
 */
void generateObjects(CSetOfObjects::Ptr& world)
{
	CSphere::Ptr sph = mrpt::make_aligned_shared<CSphere>(0.5);
	sph->setLocation(0, 0, 0);
	sph->setColor(1, 0, 0);
	world->insert(sph);

	CDisk::Ptr pln = mrpt::make_aligned_shared<opengl::CDisk>();
	pln->setDiskRadius(2);
	pln->setPose(CPose3D(0, 0, 0, 0, DEG2RAD(5), DEG2RAD(5)));
	pln->setColor(0.8, 0, 0);
	world->insert(pln);

	{
		CDisk::Ptr pln = mrpt::make_aligned_shared<opengl::CDisk>();
		pln->setDiskRadius(2);
		pln->setPose(CPose3D(0, 0, 0, DEG2RAD(30), DEG2RAD(-20), DEG2RAD(-2)));
		pln->setColor(0.9, 0, 0);
		world->insert(pln);
	}
}

void test_icp3D()
{
	// Create the reference objects:
	COpenGLScene::Ptr scene1 = mrpt::make_aligned_shared<COpenGLScene>();
	COpenGLScene::Ptr scene2 = mrpt::make_aligned_shared<COpenGLScene>();
	COpenGLScene::Ptr scene3 = mrpt::make_aligned_shared<COpenGLScene>();

	opengl::CGridPlaneXY::Ptr plane1 =
		mrpt::make_aligned_shared<CGridPlaneXY>(-20, 20, -20, 20, 0, 1);
	plane1->setColor(0.3, 0.3, 0.3);
	scene1->insert(plane1);
	scene2->insert(plane1);
	scene3->insert(plane1);

	CSetOfObjects::Ptr world = mrpt::make_aligned_shared<CSetOfObjects>();
	generateObjects(world);
	scene1->insert(world);

	// Perform the 3D scans:
	CAngularObservationMesh::Ptr aom1 =
		mrpt::make_aligned_shared<CAngularObservationMesh>();
	CAngularObservationMesh::Ptr aom2 =
		mrpt::make_aligned_shared<CAngularObservationMesh>();

	cout << "Performing ray-tracing..." << endl;
	CAngularObservationMesh::trace2DSetOfRays(
		scene1, viewpoint1, aom1,
		CAngularObservationMesh::TDoubleRange::CreateFromAperture(
			M_PI, HOW_MANY_PITCHS),
		CAngularObservationMesh::TDoubleRange::CreateFromAperture(
			M_PI, HOW_MANY_YAWS));
	CAngularObservationMesh::trace2DSetOfRays(
		scene1, viewpoint2, aom2,
		CAngularObservationMesh::TDoubleRange::CreateFromAperture(
			M_PI, HOW_MANY_PITCHS),
		CAngularObservationMesh::TDoubleRange::CreateFromAperture(
			M_PI, HOW_MANY_YAWS));
	cout << "Ray-tracing done" << endl;

	// Put the viewpoints origins:
	{
		CSetOfObjects::Ptr origin1 = opengl::stock_objects::CornerXYZ();
		origin1->setPose(viewpoint1);
		origin1->setScale(0.6);
		scene1->insert(origin1);
		scene2->insert(origin1);
	}
	{
		CSetOfObjects::Ptr origin2 = opengl::stock_objects::CornerXYZ();
		origin2->setPose(viewpoint2);
		origin2->setScale(0.6);
		scene1->insert(origin2);
		scene2->insert(origin2);
	}

	// Show the scanned points:
	CSimplePointsMap M1, M2;

	aom1->generatePointCloud(&M1);
	aom2->generatePointCloud(&M2);

	// Create the wrongly-localized M2:
	CSimplePointsMap M2_noisy;
	M2_noisy = M2;
	M2_noisy.changeCoordinatesReference(SCAN2_POSE_ERROR);

	CSetOfObjects::Ptr PTNS1 = mrpt::make_aligned_shared<CSetOfObjects>();
	CSetOfObjects::Ptr PTNS2 = mrpt::make_aligned_shared<CSetOfObjects>();

	CPointsMap::COLOR_3DSCENE(mrpt::img::TColorf(1, 0, 0));
	M1.getAs3DObject(PTNS1);

	CPointsMap::COLOR_3DSCENE(mrpt::img::TColorf(0, 0, 1));
	M2_noisy.getAs3DObject(PTNS2);

	scene2->insert(PTNS1);
	scene2->insert(PTNS2);

	// --------------------------------------
	// Do the ICP-3D
	// --------------------------------------
	float run_time;
	CICP icp;
	CICP::TReturnInfo icp_info;

	icp.options.thresholdDist = 0.40;
	icp.options.thresholdAng = 0;

	std::vector<double> xs, ys, zs;
	M1.getAllPoints(xs, ys, ys);
	cout << "Size of  xs in M1: " << xs.size() << endl;
	M2.getAllPoints(xs, ys, ys);
	cout << "Size of  xs in M2: " << xs.size() << endl;

	CPose3DPDF::Ptr pdf = icp.Align3D(
		&M2_noisy,  // Map to align
		&M1,  // Reference map
		CPose3D(),  // Initial gross estimate
		&run_time, &icp_info);

	CPose3D mean = pdf->getMeanVal();

	cout << "ICP run took " << run_time << " secs." << endl;
	cout << "Goodness: " << 100 * icp_info.goodness
		 << "% , # of iterations= " << icp_info.nIterations
		 << " Quality: " << icp_info.quality << endl;
	cout << "ICP output: mean= " << mean << endl;
	cout << "Real displacement: " << SCAN2_POSE_ERROR << endl;

	// Aligned maps:
	CSetOfObjects::Ptr PTNS2_ALIGN = mrpt::make_aligned_shared<CSetOfObjects>();

	M2_noisy.changeCoordinatesReference(CPose3D() - mean);
	M2_noisy.getAs3DObject(PTNS2_ALIGN);

	scene3->insert(PTNS1);
	scene3->insert(PTNS2_ALIGN);

	// Show in Windows:
	CDisplayWindow3D window("ICP-3D demo: scene", 500, 500);
	CDisplayWindow3D window2("ICP-3D demo: UNALIGNED scans", 500, 500);
	CDisplayWindow3D window3("ICP-3D demo: ICP-ALIGNED scans", 500, 500);

	window.setPos(10, 10);
	window2.setPos(530, 10);
	window3.setPos(10, 520);

	window.get3DSceneAndLock() = scene1;
	window.unlockAccess3DScene();

	window2.get3DSceneAndLock() = scene2;
	window2.unlockAccess3DScene();

	window3.get3DSceneAndLock() = scene3;
	window3.unlockAccess3DScene();

	std::this_thread::sleep_for(20ms);
	window.forceRepaint();
	window2.forceRepaint();

	window.setCameraElevationDeg(15);
	window.setCameraAzimuthDeg(90);
	window.setCameraZoom(15);

	window2.setCameraElevationDeg(15);
	window2.setCameraAzimuthDeg(90);
	window2.setCameraZoom(15);

	window3.setCameraElevationDeg(15);
	window3.setCameraAzimuthDeg(90);
	window3.setCameraZoom(15);

	cout << "Press any key to exit..." << endl;
	window.waitForKey();
}

int main()
{
	try
	{
		test_icp3D();
		return 0;
	}
	catch (exception& e)
	{
		cout << "Error: " << e.what() << '.' << endl;
		return -1;
	}
	catch (...)
	{
		cout << "Unknown Error.\n";
		return -1;
	}
}
