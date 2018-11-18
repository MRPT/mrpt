/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/slam/CICP.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/CAngularObservationMesh.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/stock_objects.h>
#include <gtest/gtest.h>

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
		float SCAN_RANGES_1[] = {
			0.910f,  0.900f,  0.910f,  0.900f,  0.900f,  0.890f,  0.890f,
			0.880f,  0.890f,  0.880f,  0.880f,  0.880f,  0.880f,  0.880f,
			0.880f,  0.870f,  0.880f,  0.870f,  0.870f,  0.870f,  0.880f,
			0.880f,  0.880f,  0.880f,  0.880f,  0.880f,  0.880f,  0.880f,
			0.880f,  0.880f,  0.880f,  0.880f,  0.880f,  0.880f,  0.880f,
			0.880f,  0.890f,  0.880f,  0.880f,  0.880f,  0.890f,  0.880f,
			0.890f,  0.890f,  0.880f,  0.890f,  0.890f,  0.880f,  0.890f,
			0.890f,  0.890f,  0.890f,  0.890f,  0.890f,  0.900f,  0.900f,
			0.900f,  0.900f,  0.900f,  0.910f,  0.910f,  0.910f,  0.910f,
			0.920f,  0.920f,  0.920f,  0.920f,  0.920f,  0.930f,  0.930f,
			0.930f,  0.930f,  0.940f,  0.940f,  0.950f,  0.950f,  0.950f,
			0.950f,  0.960f,  0.960f,  0.970f,  0.970f,  0.970f,  0.980f,
			0.980f,  0.990f,  1.000f,  1.000f,  1.000f,  1.010f,  1.010f,
			1.020f,  1.030f,  1.030f,  1.030f,  1.040f,  1.050f,  1.060f,
			1.050f,  1.060f,  1.070f,  1.070f,  1.080f,  1.080f,  1.090f,
			1.100f,  1.110f,  1.120f,  1.120f,  1.130f,  1.140f,  1.140f,
			1.160f,  1.170f,  1.180f,  1.180f,  1.190f,  1.200f,  1.220f,
			1.220f,  1.230f,  1.230f,  1.240f,  1.250f,  1.270f,  1.280f,
			1.290f,  1.300f,  1.320f,  1.320f,  1.350f,  1.360f,  1.370f,
			1.390f,  1.410f,  1.410f,  1.420f,  1.430f,  1.450f,  1.470f,
			1.490f,  1.500f,  1.520f,  1.530f,  1.560f,  1.580f,  1.600f,
			1.620f,  1.650f,  1.670f,  1.700f,  1.730f,  1.750f,  1.780f,
			1.800f,  1.830f,  1.850f,  1.880f,  1.910f,  1.940f,  1.980f,
			2.010f,  2.060f,  2.090f,  2.130f,  2.180f,  2.220f,  2.250f,
			2.300f,  2.350f,  2.410f,  2.460f,  2.520f,  2.570f,  2.640f,
			2.700f,  2.780f,  2.850f,  2.930f,  3.010f,  3.100f,  3.200f,
			3.300f,  3.390f,  3.500f,  3.620f,  3.770f,  3.920f,  4.070f,
			4.230f,  4.430f,  4.610f,  4.820f,  5.040f,  5.290f,  5.520f,
			8.970f,  8.960f,  8.950f,  8.930f,  8.940f,  8.930f,  9.050f,
			9.970f,  9.960f,  10.110f, 13.960f, 18.870f, 19.290f, 81.910f,
			20.890f, 48.750f, 48.840f, 48.840f, 19.970f, 19.980f, 19.990f,
			15.410f, 20.010f, 19.740f, 17.650f, 17.400f, 14.360f, 12.860f,
			11.260f, 11.230f, 8.550f,  8.630f,  9.120f,  9.120f,  8.670f,
			8.570f,  7.230f,  7.080f,  7.040f,  6.980f,  6.970f,  5.260f,
			5.030f,  4.830f,  4.620f,  4.440f,  4.390f,  4.410f,  4.410f,
			4.410f,  4.430f,  4.440f,  4.460f,  4.460f,  4.490f,  4.510f,
			4.540f,  3.970f,  3.820f,  3.730f,  3.640f,  3.550f,  3.460f,
			3.400f,  3.320f,  3.300f,  3.320f,  3.320f,  3.340f,  2.790f,
			2.640f,  2.600f,  2.570f,  2.540f,  2.530f,  2.510f,  2.490f,
			2.490f,  2.480f,  2.470f,  2.460f,  2.460f,  2.460f,  2.450f,
			2.450f,  2.450f,  2.460f,  2.460f,  2.470f,  2.480f,  2.490f,
			2.490f,  2.520f,  2.510f,  2.550f,  2.570f,  2.610f,  2.640f,
			2.980f,  3.040f,  3.010f,  2.980f,  2.940f,  2.920f,  2.890f,
			2.870f,  2.830f,  2.810f,  2.780f,  2.760f,  2.740f,  2.720f,
			2.690f,  2.670f,  2.650f,  2.630f,  2.620f,  2.610f,  2.590f,
			2.560f,  2.550f,  2.530f,  2.510f,  2.500f,  2.480f,  2.460f,
			2.450f,  2.430f,  2.420f,  2.400f,  2.390f,  2.380f,  2.360f,
			2.350f,  2.340f,  2.330f,  2.310f,  2.300f,  2.290f,  2.280f,
			2.270f,  2.260f,  2.250f,  2.240f,  2.230f,  2.230f,  2.220f,
			2.210f,  2.200f,  2.190f,  2.180f,  2.170f,  1.320f,  1.140f,
			1.130f,  1.130f,  1.120f,  1.120f,  1.110f,  1.110f,  1.110f,
			1.110f,  1.100f,  1.110f,  1.100f};
		char SCAN_VALID_1[] = {
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

		float SCAN_RANGES_2[] = {
			0.720f,  0.720f,  0.720f,  0.720f,  0.720f,  0.720f,  0.710f,
			0.720f,  0.710f,  0.710f,  0.710f,  0.710f,  0.710f,  0.710f,
			0.710f,  0.710f,  0.710f,  0.710f,  0.710f,  0.710f,  0.720f,
			0.720f,  0.720f,  0.720f,  0.730f,  0.730f,  0.730f,  0.730f,
			0.730f,  0.730f,  0.730f,  0.740f,  0.740f,  0.740f,  0.740f,
			0.740f,  0.740f,  0.750f,  0.750f,  0.750f,  0.750f,  0.750f,
			0.750f,  0.750f,  0.750f,  0.760f,  0.760f,  0.760f,  0.760f,
			0.760f,  0.760f,  0.760f,  0.760f,  0.770f,  0.770f,  0.770f,
			0.770f,  0.780f,  0.780f,  0.780f,  0.790f,  0.790f,  0.800f,
			0.800f,  0.800f,  0.800f,  0.800f,  0.800f,  0.810f,  0.810f,
			0.820f,  0.820f,  0.830f,  0.830f,  0.840f,  0.840f,  0.850f,
			0.850f,  0.860f,  0.860f,  0.860f,  0.870f,  0.870f,  0.880f,
			0.890f,  0.890f,  0.900f,  0.900f,  0.910f,  0.920f,  0.930f,
			0.930f,  0.940f,  0.940f,  0.940f,  0.950f,  0.960f,  0.960f,
			0.970f,  0.980f,  0.990f,  1.000f,  1.010f,  1.020f,  1.030f,
			1.040f,  1.050f,  1.060f,  1.070f,  1.080f,  1.080f,  1.100f,
			1.100f,  1.120f,  1.120f,  1.140f,  1.140f,  1.170f,  1.160f,
			1.180f,  1.190f,  1.210f,  1.220f,  1.240f,  1.250f,  1.280f,
			1.290f,  1.300f,  1.320f,  1.340f,  1.350f,  1.380f,  1.390f,
			1.420f,  1.440f,  1.460f,  1.470f,  1.500f,  1.520f,  1.550f,
			1.570f,  1.600f,  1.630f,  1.670f,  1.690f,  1.730f,  1.760f,
			1.790f,  1.820f,  1.870f,  1.900f,  1.940f,  1.970f,  2.030f,
			2.080f,  2.130f,  2.170f,  2.230f,  2.280f,  2.340f,  2.400f,
			2.490f,  2.550f,  2.630f,  2.700f,  2.810f,  2.880f,  3.010f,
			3.090f,  3.240f,  3.340f,  3.500f,  3.620f,  3.810f,  3.950f,
			4.180f,  4.340f,  4.620f,  8.170f,  8.140f,  8.150f,  8.120f,
			8.110f,  8.100f,  8.100f,  8.300f,  9.040f,  9.130f,  9.130f,
			13.030f, 18.050f, 19.150f, 81.910f, 20.070f, 47.980f, 48.040f,
			48.030f, 19.140f, 19.180f, 19.180f, 19.190f, 14.550f, 19.210f,
			16.850f, 16.840f, 7.800f,  7.770f,  7.770f,  7.750f,  7.770f,
			7.760f,  7.780f,  7.760f,  8.320f,  8.350f,  8.350f,  8.090f,
			7.720f,  7.730f,  6.430f,  6.360f,  6.290f,  6.260f,  6.230f,
			6.220f,  6.160f,  5.800f,  4.510f,  4.410f,  4.240f,  4.140f,
			4.000f,  3.910f,  3.790f,  3.680f,  3.660f,  3.680f,  3.680f,
			3.700f,  3.710f,  3.730f,  3.730f,  3.760f,  3.770f,  3.790f,
			3.820f,  3.850f,  3.900f,  3.940f,  3.980f,  3.250f,  3.180f,
			3.140f,  3.070f,  3.030f,  2.970f,  2.930f,  2.880f,  2.850f,
			2.790f,  2.760f,  2.710f,  2.680f,  2.660f,  2.670f,  2.690f,
			2.710f,  2.720f,  2.740f,  2.760f,  2.770f,  2.780f,  2.800f,
			2.170f,  2.120f,  2.090f,  2.060f,  2.020f,  2.010f,  1.990f,
			1.980f,  1.970f,  1.960f,  1.950f,  1.950f,  1.940f,  1.940f,
			1.950f,  1.940f,  1.940f,  1.950f,  1.930f,  1.940f,  1.940f,
			1.940f,  1.940f,  1.940f,  1.950f,  1.960f,  1.960f,  1.980f,
			1.980f,  2.000f,  2.010f,  2.030f,  2.060f,  2.090f,  2.120f,
			2.190f,  2.560f,  2.540f,  2.530f,  2.520f,  2.500f,  2.490f,
			2.470f,  2.460f,  2.450f,  2.440f,  2.420f,  2.410f,  2.400f,
			2.390f,  2.380f,  2.370f,  2.360f,  2.350f,  2.340f,  2.340f,
			2.330f,  2.320f,  2.310f,  2.300f,  2.290f,  2.290f,  2.290f,
			2.280f,  2.270f,  2.260f,  2.260f,  2.250f,  2.240f,  2.240f,
			2.230f,  2.230f,  2.220f,  2.220f,  2.210f,  2.210f,  2.200f,
			2.200f,  2.190f,  2.190f,  2.190f,  2.180f,  2.180f,  2.170f,
			2.170f,  2.170f,  2.160f,  2.160f};
		char SCAN_VALID_2[] = {
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

#define SCANS_SIZE (sizeof(SCAN_RANGES_1) / sizeof(SCAN_RANGES_1[0]))

		CSimplePointsMap m1, m2;
		float runningTime;
		CICP::TReturnInfo info;
		CICP ICP;

		// Load scans:
		CObservation2DRangeScan scan1;
		scan1.aperture = M_PIf;
		scan1.rightToLeft = true;

		ASSERT_(sizeof(SCAN_RANGES_1) == sizeof(float) * SCANS_SIZE);
		scan1.loadFromVectors(SCANS_SIZE, SCAN_RANGES_1, SCAN_VALID_1);

		CObservation2DRangeScan scan2 = scan1;
		scan2.loadFromVectors(SCANS_SIZE, SCAN_RANGES_2, SCAN_VALID_2);

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
		(mean.getAsVectorVal() - SCAN2_POSE_ERROR.getAsVectorVal())
			.array()
			.abs()
			.mean(),
		0.02)
		<< "ICP output: mean= " << mean << endl
		<< "Real displacement: " << SCAN2_POSE_ERROR << endl;
}
