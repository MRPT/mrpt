/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/**
 * Execute the iterating closest point algorithm (ICP) on a hardcoded pair of
 * laser data. The algorithm computes the transformation (translation and
 * rotation) for aligning the 2 sets of laser scans and plots the
 */

#include <mrpt/gui.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/utils.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/slam/CICP.h>

#include <fstream>
#include <iostream>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;

bool skip_window = false;
int ICP_method = (int)icpClassic;

// ------------------------------------------------------
//				TestICP
// ------------------------------------------------------
void TestICP()
{
	CSimplePointsMap m1, m2;
	CICP::TReturnInfo info;
	CICP ICP;

	// Load scans:
	CObservation2DRangeScan scan1;
	stock_observations::example2DRangeScan(scan1, 0);

	CObservation2DRangeScan scan2;
	stock_observations::example2DRangeScan(scan2, 1);

	// Build the points maps from the scans:
	m1.insertObservation(scan1);
	m2.insertObservation(scan2);

	// -----------------------------------------------------

	/**
	 * user-set parameters for the icp algorithm.
	 * For a full list of the available options check the online tutorials
	 * page:
	 * https://www.mrpt.org/Iterative_Closest_Point_(ICP)_and_other_matching_algorithms
	 */

	//	select which algorithm version to use
	//	ICP.options.ICP_algorithm = icpLevenbergMarquardt;
	//	ICP.options.ICP_algorithm = icpClassic;
	ICP.options.ICP_algorithm = (TICPAlgorithm)ICP_method;

	// configuration options for the icp algorithm
	ICP.options.maxIterations = 100;
	ICP.options.thresholdAng = DEG2RAD(10.0f);
	ICP.options.thresholdDist = 0.75f;
	ICP.options.ALFA = 0.5f;
	ICP.options.smallestThresholdDist = 0.05f;
	ICP.options.doRANSAC = false;

	ICP.options.dumpToConsole();
	// -----------------------------------------------------

	/**
	 * Scans alignment procedure.
	 * Given an initial guess (initialPose) and the maps to be aligned, the
	 * algorithm returns the probability density function (pdf) of the alignment
	 * Additional arguments are provided to investigate the performance of the
	 * algorithm
	 */
	CPose2D initialPose(0.8f, 0.0f, (float)DEG2RAD(0.0f));

	CPosePDF::Ptr pdf = ICP.Align(&m1, &m2, initialPose, info);

	printf(
		"ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n "
		"-> ",
		info.executionTime * 1000, info.nIterations,
		info.executionTime * 1000.0f / info.nIterations, info.goodness * 100);

	cout << "Mean of estimation: " << pdf->getMeanVal() << endl << endl;

	CPosePDFGaussian gPdf;
	gPdf.copyFrom(*pdf);

	cout << "Covariance of estimation: " << endl << gPdf.cov << endl;

	cout << " std(x): " << sqrt(gPdf.cov(0, 0)) << endl;
	cout << " std(y): " << sqrt(gPdf.cov(1, 1)) << endl;
	cout << " std(phi): " << RAD2DEG(sqrt(gPdf.cov(2, 2))) << " (deg)" << endl;

	// cout << "Covariance of estimation (MATLAB format): " << endl <<
	// gPdf.cov.inMatlabFormat()  << endl;

	/**
	 * Save the results for potential postprocessing (in txt and in matlab
	 * format)
	 */
	cout << "-> Saving reference map as scan1.txt" << endl;
	m1.save2D_to_text_file("scan1.txt");

	cout << "-> Saving map to align as scan2.txt" << endl;
	m2.save2D_to_text_file("scan2.txt");

	cout << "-> Saving transformed map to align as scan2_trans.txt" << endl;
	CSimplePointsMap m2_trans = m2;
	m2_trans.changeCoordinatesReference(gPdf.mean);
	m2_trans.save2D_to_text_file("scan2_trans.txt");

	cout << "-> Saving MATLAB script for drawing 2D ellipsoid as view_ellip.m"
		 << endl;
	CMatrixFloat COV22 = CMatrixFloat(CMatrixDouble(gPdf.cov));
	COV22.setSize(2, 2);
	CVectorFloat MEAN2D(2);
	MEAN2D[0] = gPdf.mean.x();
	MEAN2D[1] = gPdf.mean.y();
	{
		ofstream f("view_ellip.m");
		f << math::MATLAB_plotCovariance2D(COV22, MEAN2D, 3.0f);
	}

// If we have 2D windows, use'em:
#if MRPT_HAS_WXWIDGETS
	/**
	 * Plotting the icp results:
	 * Aligned maps + transformation uncertainty ellipsis
	 */
	if (!skip_window)
	{
		gui::CDisplayWindowPlots win("ICP results");

		// Reference map:
		vector<float> map1_xs, map1_ys, map1_zs;
		m1.getAllPoints(map1_xs, map1_ys, map1_zs);
		win.plot(map1_xs, map1_ys, "b.3", "map1");

		// Translated map:
		vector<float> map2_xs, map2_ys, map2_zs;
		m2_trans.getAllPoints(map2_xs, map2_ys, map2_zs);
		win.plot(map2_xs, map2_ys, "r.3", "map2");

		// Uncertainty
		win.plotEllipse(MEAN2D[0], MEAN2D[1], COV22, 3.0, "b2", "cov");

		win.axis(-1, 10, -6, 6);
		win.axis_equal();

		cout << "Close the window to exit" << endl;
		win.waitForKey();
	}
#endif
}

int main(int argc, char** argv)
{
	try
	{
		skip_window = (argc > 2);
		if (argc > 1)
		{
			ICP_method = atoi(argv[1]);
		}

		TestICP();

		return 0;
	}
	catch (exception& e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
