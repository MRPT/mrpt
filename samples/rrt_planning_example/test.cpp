/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/nav.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/system/filesystem.h> // directoryExists(), ...
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/random.h>
#include <mrpt/gui/CDisplayWindow3D.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace mrpt::maps;
using namespace std;

// Load example grid map
#include <mrpt/examples_config.h>
string   mySimpleMap( MRPT_EXAMPLES_BASE_DIRECTORY + string("../share/mrpt/datasets/malaga-cs-fac-building.simplemap.gz") );
string   myCfgFileName( MRPT_EXAMPLES_BASE_DIRECTORY + string("../share/mrpt/config_files/navigation-ptgs/ptrrt_config_example1.ini") );

// ------------------------------------------------------
//				TestRRT1
// ------------------------------------------------------
void TestRRT1()
{
	mrpt::random::Randomize();

	// Load the gridmap:
	CSimpleMap simplemap;
	
	ASSERT_FILE_EXISTS_(mySimpleMap);

	cout << "Loading map...";
	CFileGZInputStream(mySimpleMap) >> simplemap;
	cout << "Done! Number of sensory frames: " << simplemap.size() << endl;

	// Set planner params:
	// ------------------------------
	mrpt::nav::PlannerRRT_SE2_TPS  planner;

	// Parameters:
	planner.loadConfig( mrpt::utils::CConfigFile(myCfgFileName) );

	planner.params.maxLength = 2.0; 
	planner.params.minDistanceBetweenNewNodes = 0.10;
	planner.params.minAngBetweenNewNodes = mrpt::utils::DEG2RAD(20);
	planner.params.goalBias = 0.05;

	// Logging:
	planner.params.save_3d_log_freq = 0; //500; // save some iterations for debugging

	// End criteria:
	planner.end_criteria.acceptedDistToTarget = 0.25;
	planner.end_criteria.acceptedAngToTarget  = DEG2RAD(180); // 180d=Any orientation is ok
	planner.end_criteria.maxComputationTime = 15.0;
	planner.end_criteria.minComputationTime = 1.0; // 0=accept first found acceptable solution

	// Init planner:
	// ------------------------------
	planner.initialize();

	// Set up planning problem:
	// ------------------------------
	PlannerRRT_SE2_TPS::TPlannerResult planner_result;
	PlannerRRT_SE2_TPS::TPlannerInput planner_input;

	// Start & goal:
	planner_input.start_pose = mrpt::math::TPose2D(0,0,0);
	planner_input.goal_pose  = mrpt::math::TPose2D(-20,-30,0);

	// Obstacles:
	planner_input.obstacles_points.loadFromSimpleMap( simplemap );
	mrpt::math::TPoint3D bbox_min,bbox_max;
	planner_input.obstacles_points.boundingBox(bbox_min,bbox_max);
	// Convert gridmap -> obstacle points:
	//gridmap.getAsPointCloud( planner_input.obstacles_points );

	// Workspace bounding box:
	planner_input.world_bbox_min = mrpt::math::TPoint2D(bbox_min.x,bbox_min.y);
	planner_input.world_bbox_max = mrpt::math::TPoint2D(bbox_max.x,bbox_max.y);

	//size_t iters=0;
	// Show results in a GUI and keep improving:
#if MRPT_HAS_WXWIDGETS
	mrpt::gui::CDisplayWindow3D  win("Result",1024,800);
	while (win.isOpen())
#else
	for (size_t i=0;i<1;i++)
#endif
	{
		// Refine solution or start over:
		bool refine_solution = false; // (iters++ % 5 != 0);

		// Start from scratch: 
		if (!refine_solution)
			planner_result = PlannerRRT_SE2_TPS::TPlannerResult();

		// Do path planning:
		planner.solve( planner_input, planner_result);

		cout << "Found goal_distance: " << planner_result.goal_distance << endl;
		cout << "Found path_cost: " << planner_result.path_cost << endl;
		cout << "Acceptable goal nodes: " << planner_result.acceptable_goal_node_ids.size() << endl;

#if MRPT_HAS_WXWIDGETS
		// Show result in a GUI:
		mrpt::opengl::COpenGLScenePtr & scene = win.get3DSceneAndLock();

		scene->clear();
	
		PlannerRRT_SE2_TPS::TRenderPlannedPathOptions render_opts;
		render_opts.highlight_path_to_node_id = planner_result.best_goal_node_id;

		planner.renderMoveTree(*scene, planner_input, planner_result,render_opts );

		win.unlockAccess3DScene();
		win.repaint();
		win.waitForKey();
#endif
	}
	

}

int main(int argc, char **argv)
{
	try
	{
		TestRRT1();
		return 0;
	} catch (exception &e)
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


