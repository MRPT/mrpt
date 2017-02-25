/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/random.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/math/geometry.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/tfest/se2.h>
#include <mrpt/maps/CSimplePointsMap.h>

// Method explained in paper: 
//  J.L. Blanco, J. Gonzalez-Jimenez, J.A. Fernandez-Madrigal, 
//   "A Robust, Multi-Hypothesis Approach to Matching Occupancy Grid Maps", 
//    Robotica, 2013. 
// http://dx.doi.org/10.1017/S0263574712000732

// ============= PARAMETERS ===================
const size_t NUM_OBSERVATIONS_TO_SIMUL = 10; 
const size_t RANSAC_MINIMUM_INLIERS    = 9;  // Min. # of inliers to accept

#define LOAD_MAP_FROM_FILE  0  // 1: load from "sMAP_FILE", 0: random map.
#define SHOW_POINT_LABELS   0

const float normalizationStd = 0.15f; // 1 sigma noise (meters)
const float ransac_mahalanobisDistanceThreshold = 5.0f;
const size_t MINIMUM_RANSAC_ITERS = 100000;

#if !LOAD_MAP_FROM_FILE
	const size_t NUM_MAP_FEATS = 100;
	const double MAP_SIZE_X    = 50;
	const double MAP_SIZE_Y    = 25;
#else
	// Expected format of the 2D map is, for each line (one per landmark):
	//  ID X Y
	const std::string sMAP_FILE = string("./DLRMap.txt");
#endif
// ==============================================

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::maps;
using namespace std;


struct TObs
{
	size_t ID; // Ground truth ID
	double x,y;
};

// ------------------------------------------------------
//				TestRANSAC
// ------------------------------------------------------
void TestRANSAC()
{
	mrpt::gui::CDisplayWindow3D win("MRPT example: ransac-data-association",800,600);

	mrpt::utils::CTimeLogger timelog;  // For dumping stats at the end
	mrpt::utils::CTicTac     timer;


	randomGenerator.randomize(); // randomize with time

	// --------------------------------
	// Load feature map:
	// --------------------------------
	CSimplePointsMap  the_map;
#if LOAD_MAP_FROM_FILE
	{
		CMatrixDouble  M;
		M.loadFromTextFile(sMAP_FILE); // Launch except. on error
		ASSERT_(M.getColCount()==3 && M.getRowCount()>2)

		const size_t nPts = M.getRowCount();
		the_map.resize(nPts);
		for (size_t i=0;i<nPts;i++)
			the_map.setPoint(i,M(i,1),M(i,2));
	}
#else
	// Generate random MAP:
	the_map.resize(NUM_MAP_FEATS);
	for (size_t i=0;i<NUM_MAP_FEATS;i++)
	{
		the_map.setPoint(i, 
			randomGenerator.drawUniform(0,MAP_SIZE_X),
			randomGenerator.drawUniform(0,MAP_SIZE_Y)
			);
	}
#endif

	const size_t nMapPts = the_map.size();
	cout << "Loaded/generated map with " << nMapPts << " landmarks.\n";


	const size_t nObs=NUM_OBSERVATIONS_TO_SIMUL;

	mrpt::opengl::CPointCloudPtr gl_obs_map = mrpt::opengl::CPointCloud::Create();
	mrpt::opengl::CPointCloudPtr gl_result = mrpt::opengl::CPointCloud::Create();
	mrpt::opengl::CSetOfObjectsPtr gl_obs = mrpt::opengl::CSetOfObjects::Create();
	mrpt::opengl::CSetOfObjectsPtr gl_obs_txts = mrpt::opengl::CSetOfObjects::Create();
	mrpt::opengl::CSetOfLinesPtr gl_lines = mrpt::opengl::CSetOfLines::Create();
	{
		mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();

		scene->getViewport("main")->setCustomBackgroundColor( TColorf(0.8f,0.8f,0.8f));
		win.setCameraPointingToPoint( MAP_SIZE_X*0.5, MAP_SIZE_Y*0.5, 0);
		win.setCameraZoom( 2*MAP_SIZE_X );

		//
		scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

		//
		mrpt::opengl::CPointCloudPtr gl_map = mrpt::opengl::CPointCloud::Create();
		gl_map->loadFromPointsMap(&the_map);
		gl_map->setColor(0,0,1);
		gl_map->setPointSize(3);

		scene->insert(gl_map);

#if SHOW_POINT_LABELS
		for (size_t i=0;i<the_map.size();i++)
		{
			mrpt::opengl::CTextPtr gl_txt = mrpt::opengl::CText::Create( mrpt::format("%u",static_cast<unsigned int>(i)) );
			double x,y;
			the_map.getPoint(i,x,y);
			gl_txt->setLocation(x+0.05,y+0.05,0.01);

			scene->insert(gl_txt);
		}
#endif

		//
		scene->insert(gl_lines);

		//
		gl_obs_map->setColor(1,0,0);
		gl_obs_map->setPointSize(5);

		gl_result->setColor(0,1,0);
		gl_result->setPointSize(4);

		//
		gl_obs->insert( mrpt::opengl::stock_objects::CornerXYZ(0.6) );
		gl_obs->insert(gl_obs_map);
		gl_obs->insert(gl_obs_txts);
		scene->insert(gl_obs);
		scene->insert(gl_result);

		win.unlockAccess3DScene();
		win.repaint();
	}


	// Repeat for each set of observations in the input file
	while (win.isOpen())
	{
		// Read the observations themselves:
		vector<TObs> observations;
		observations.resize(nObs);

		const mrpt::poses::CPose2D  GT_pose(
			mrpt::random::randomGenerator.drawUniform(-10,10+MAP_SIZE_X),
			mrpt::random::randomGenerator.drawUniform(-10,10+MAP_SIZE_Y),
			mrpt::random::randomGenerator.drawUniform(-M_PI,M_PI) );

		const mrpt::poses::CPose2D  GT_pose_inv = -GT_pose;

		std::vector<std::pair<size_t,float> > idxs;
		the_map.kdTreeRadiusSearch2D(GT_pose.x(),GT_pose.y(), 1000, idxs);
		ASSERT_(idxs.size()>=nObs)

		for (size_t i=0;i<nObs;i++)
		{
			double gx,gy;
			the_map.getPoint(idxs[i].first, gx,gy);

			double lx,ly;
			GT_pose_inv.composePoint(gx,gy, lx,ly);

			observations[i].ID = idxs[i].first;
			observations[i].x = lx + mrpt::random::randomGenerator.drawGaussian1D(0,normalizationStd);
			observations[i].y = ly + mrpt::random::randomGenerator.drawGaussian1D(0,normalizationStd);
		}

		// ----------------------------------------------------
		// Generate list of individual-compatible pairings
		// ----------------------------------------------------
		TMatchingPairList all_correspondences;

		all_correspondences.reserve(nMapPts*nObs);

		// ALL possibilities: 
		for (size_t j=0;j<nObs;j++)
		{
			TMatchingPair match;

			match.other_idx = j;
			match.other_x = observations[j].x;
			match.other_y = observations[j].y;

			for (size_t i=0;i<nMapPts;i++)
			{
				match.this_idx = i;
				the_map.getPoint(i, match.this_x, match.this_y );

				all_correspondences.push_back(match);
			}
		}
		cout << "Generated " << all_correspondences.size() << " potential pairings.\n";

		// ----------------------------------------------------
		//  Run RANSAC-based D-A
		// ----------------------------------------------------
		timelog.enter("robustRigidTransformation");
		timer.Tic();

		mrpt::tfest::TSE2RobustParams params;
		mrpt::tfest::TSE2RobustResult results;

		params.ransac_minSetSize = RANSAC_MINIMUM_INLIERS;     // ransac_minSetSize (to add the solution to the SOG)
		params.ransac_maxSetSize = all_correspondences.size(); // ransac_maxSetSize: Test with all data points
		params.ransac_mahalanobisDistanceThreshold = ransac_mahalanobisDistanceThreshold;
		params.ransac_nSimulations = 0; // 0=auto
		params.ransac_fuseByCorrsMatch = true;
		params.ransac_fuseMaxDiffXY = 0.01f;
		params.ransac_fuseMaxDiffPhi = DEG2RAD(0.1);
		params.ransac_algorithmForLandmarks = true;
		params.probability_find_good_model = 0.999999;
		params.ransac_min_nSimulations  = MINIMUM_RANSAC_ITERS; // (a lower limit to the auto-detected value of ransac_nSimulations)
		params.verbose = true;

		// Run ransac data-association:
		mrpt::tfest::se2_l2_robust(all_correspondences, normalizationStd, params, results);

		timelog.leave("robustRigidTransformation");

		mrpt::poses::CPosePDFSOG  & best_poses  = results.transformation;
		TMatchingPairList         & out_best_pairings = results.largestSubSet;

		const double tim = timer.Tac();
		cout << "RANSAC time: " << mrpt::system::formatTimeInterval(tim) << endl;

		cout << "# of SOG modes: " << best_poses.size() << endl;
		cout << "Best match has " <<out_best_pairings.size() << " features:\n";
		for (size_t i=0;i<out_best_pairings.size();i++)
			cout << out_best_pairings[i].this_idx << " <-> " << out_best_pairings[i].other_idx << endl;
		cout << endl;

		// Generate "association vector":
		vector<int> obs2map_pairings(nObs,-1);
		for (size_t i=0;i<out_best_pairings.size();i++)
			obs2map_pairings[out_best_pairings[i].other_idx] = out_best_pairings[i].this_idx==((unsigned int)-1) ? -1 : out_best_pairings[i].this_idx;

		cout << "1\n";
		for (size_t i=0;i<nObs;i++)
			cout << obs2map_pairings[i] << " ";
		cout << endl;


		gl_result->clear();

		// Reconstruct the SE(2) transformation for these pairings:
		mrpt::poses::CPosePDFGaussian  solution_pose;
		mrpt::tfest::se2_l2(out_best_pairings, solution_pose);

		// Normalized covariance: scale!
		solution_pose.cov *= square(normalizationStd);

		cout << "Solution pose: " << solution_pose.mean << endl;
		cout << "Ground truth pose: " << GT_pose << endl;


		{
			//mrpt::opengl::COpenGLScenePtr &scene =
			win.get3DSceneAndLock();

			win.addTextMessage(
				5,5, "Blue: map landmarks | Red: Observations | White lines: Found correspondences",
				mrpt::utils::TColorf(0,0,0),"mono",12,mrpt::opengl::NICE, 0);

			//
			gl_obs_map->clear();
			for (size_t k=0;k<nObs;k++)
				gl_obs_map->insertPoint( observations[k].x,observations[k].y, 0.0 );

			gl_obs->setPose( solution_pose.mean );

#if SHOW_POINT_LABELS
			gl_obs_txts->clear();
			for (size_t i=0;i<nObs;i++)
			{
				mrpt::opengl::CTextPtr gl_txt = mrpt::opengl::CText::Create( mrpt::format("%u",static_cast<unsigned int>(i)) );
				const double x = observations[i].x;
				const double y = observations[i].y;
				gl_txt->setLocation(x+0.05,y+0.05,0.01);
				gl_obs_txts->insert(gl_txt);
			}
#endif


			//
			gl_lines->clear();
			double sqerr = 0;
			size_t nPairs = 0;
			for (size_t k=0;k<nObs;k++)
			{
				int map_idx = obs2map_pairings[k];
				if (map_idx<0) continue;
				nPairs++;

				double map_x,map_y;
				the_map.getPoint(map_idx,map_x,map_y);

				double obs_x,obs_y;
				solution_pose.mean.composePoint(
					observations[k].x,observations[k].y,
					obs_x, obs_y);

				const double z = 0;

				gl_lines->appendLine(
					map_x,map_y,0,
					obs_x,obs_y,z );

				sqerr+= mrpt::math::distanceSqrBetweenPoints<double>(map_x,map_y,obs_x,obs_y);
			}

			win.addTextMessage(
				5,20, "Ground truth pose    : " + GT_pose.asString(),
				mrpt::utils::TColorf(0,0,0),"mono",12,mrpt::opengl::NICE, 1);
			win.addTextMessage(
				5,35, "RANSAC estimated pose: " + solution_pose.mean.asString() + mrpt::format(" | RMSE=%f",(nPairs ? sqerr/nPairs : 0.0) ),
				mrpt::utils::TColorf(0,0,0),"mono",12,mrpt::opengl::NICE, 2);

			win.unlockAccess3DScene();
			win.repaint();

			cout << "nPairings: " << nPairs << " RMSE = " << (nPairs ? sqerr/nPairs : 0.0)<< endl;

			win.waitForKey();
		}


	} // end of for each set of observations


}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestRANSAC();
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
