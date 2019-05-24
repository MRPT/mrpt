/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/opengl/CAngularObservationMesh.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/slam/CICP.h>
#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace std;

class ICPTests : public ::testing::Test
{
   protected:
	void SetUp() override {}
	void TearDown() override {}
	void align2scans(const TICPAlgorithm icp_method)
	{
		CSimplePointsMap m1, m2;
		float runningTime;
		CICP::TReturnInfo info;
		CICP ICP;

		// Load scans:
		CObservation2DRangeScan scan1;
		stock_observations::example2DRangeScan(scan1, 0);

		CObservation2DRangeScan scan2;
		stock_observations::example2DRangeScan(scan2, 1);

		// Build the points maps from the scans:
		m1.insertObservation(&scan1);
		m2.insertObservation(&scan2);

		// -----------------------------------------------------
		ICP.options.ICP_algorithm = icp_method;

		ICP.options.maxIterations = 100;
		ICP.options.thresholdAng = DEG2RAD(10.0f);
		ICP.options.thresholdDist = 0.75f;
		ICP.options.ALFA = 0.5f;
		ICP.options.smallestThresholdDist = 0.05f;
		ICP.options.doRANSAC = false;
		// ICP.options.dumpToConsole();
		// -----------------------------------------------------
		CPose2D initialPose(0.8f, 0.0f, (float)DEG2RAD(0.0f));

		CPosePDF::Ptr pdf =
			ICP.Align(&m1, &m2, initialPose, &runningTime, (void*)&info);

		/*printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%%
		   goodness\n -> ",
				runningTime*1000,
				info.nIterations,
				runningTime*1000.0f/info.nIterations,
				info.goodness*100 );*/

		// cout << "Mean of estimation: " << pdf->getEstimatedPose() << endl<<
		// endl;
		// Should be around: Mean of estimation: (0.820,0.084,8.73deg)

		const CPose2D good_pose(0.820, 0.084, DEG2RAD(8.73));

		EXPECT_NEAR(good_pose.distanceTo(pdf->getMeanVal()), 0, 0.02);
	}

	static void generateObjects(CSetOfObjects::Ptr& world)
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
			CDisk::Ptr pln2 = mrpt::make_aligned_shared<opengl::CDisk>();
			pln2->setDiskRadius(2);
			pln2->setPose(
				CPose3D(0, 0, 0, DEG2RAD(30), DEG2RAD(-20), DEG2RAD(-2)));
			pln2->setColor(0.9, 0, 0);
			world->insert(pln2);
		}
	}
};

TEST_F(ICPTests, AlignScans_icpClassic) { align2scans(icpClassic); }
TEST_F(ICPTests, AlignScans_icpLevenbergMarquardt)

{
	align2scans(icpLevenbergMarquardt);
}

TEST_F(ICPTests, RayTracingICP3D)
{
	// Increase this values to get more precision. It will also increase run
	// time.
	const size_t HOW_MANY_YAWS = 150;
	const size_t HOW_MANY_PITCHS = 150;

	// The two origins for the 3D scans
	CPose3D viewpoint1(-0.3, 0.7, 3, DEG2RAD(5), DEG2RAD(80), DEG2RAD(3));
	CPose3D viewpoint2(0.5, -0.2, 2.6, DEG2RAD(-5), DEG2RAD(100), DEG2RAD(-7));

	CPose3D SCAN2_POSE_ERROR(0.15, -0.07, 0.10, -0.03, 0.1, 0.1);

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

	// Put the viewpoints origins:
	{
		CSetOfObjects::Ptr origin1 = opengl::stock_objects::CornerXYZ();
		origin1->setPose(viewpoint1);
		origin1->setScale(0.6f);
		scene1->insert(origin1);
		scene2->insert(origin1);
	}
	{
		CSetOfObjects::Ptr origin2 = opengl::stock_objects::CornerXYZ();
		origin2->setPose(viewpoint2);
		origin2->setScale(0.6f);
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

	M1.renderOptions.color = mrpt::img::TColorf(1, 0, 0);
	M1.getAs3DObject(PTNS1);

	M2_noisy.renderOptions.color = mrpt::img::TColorf(0, 0, 1);
	M2_noisy.getAs3DObject(PTNS2);

	scene2->insert(PTNS1);
	scene2->insert(PTNS2);

	// --------------------------------------
	// Do the ICP-3D
	// --------------------------------------
	float run_time;
	CICP icp;
	CICP::TReturnInfo icp_info;

	icp.options.thresholdDist = 0.40f;
	icp.options.thresholdAng = 0;

	CPose3DPDF::Ptr pdf = icp.Align3D(
		&M2_noisy,  // Map to align
		&M1,  // Reference map
		CPose3D(),  // Initial gross estimate
		&run_time, &icp_info);

	CPose3D mean = pdf->getMeanVal();

	// Checks:
	EXPECT_NEAR(
		0,
		(mean.asVectorVal() - SCAN2_POSE_ERROR.asVectorVal())
			.array()
			.abs()
			.mean(),
		0.02)
		<< "ICP output: mean= " << mean << endl
		<< "Real displacement: " << SCAN2_POSE_ERROR << endl;
}
