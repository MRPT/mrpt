/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/nav.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
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
string   myGridMap( MRPT_EXAMPLES_BASE_DIRECTORY + string("../share/mrpt/datasets/2006-MalagaCampus.gridmap.gz") );
string   myCfgFileName( MRPT_EXAMPLES_BASE_DIRECTORY + string("../share/mrpt/config_files/navigation-ptgs/ptrrt_config_example1.ini") );

// ------------------------------------------------------
//				TestRRT1
// ------------------------------------------------------
void TestRRT1()
{
	mrpt::random::Randomize();

	// Load the gridmap:
	COccupancyGridMap2D		gridmap;

	if (!mrpt::system::fileExists(myGridMap))
		THROW_EXCEPTION_CUSTOM_MSG1("Map file '%s' not found",myGridMap.c_str());

	printf("Loading gridmap...");
	CFileGZInputStream(myGridMap) >> gridmap;
	printf("Done! %f x %f m\n", gridmap.getXMax()-gridmap.getXMin(), gridmap.getYMax()-gridmap.getYMin());

	// Set planner params:
	// ------------------------------
	mrpt::nav::PlannerRRT_SE2_TPS  planner;

	// Parameters:
	planner.loadConfig( mrpt::utils::CConfigFile(myCfgFileName) );

	planner.params.minDistanceBetweenNewNodes = 0.25;
	//planner.params.goalBias = 0.10;

	// Logging:
	planner.params.save_3d_log_freq = 50; // save some iterations for debugging

	// End criteria:
	planner.end_criteria.acceptedDistToTarget = 0.25;
	planner.end_criteria.maxComputationTime = 0;
	planner.end_criteria.minComputationTime = 1.0; // 0=accept first found acceptable solution


	// Init planner:
	// ------------------------------
	planner.initialize();

	// Find path:
	// ------------------------------
	PlannerRRT_SE2_TPS::TPlannerResult planner_result;
	PlannerRRT_SE2_TPS::TPlannerInput planner_input;

	planner_input.start_pose = mrpt::math::TPose2D(0,0,0);
	planner_input.goal_pose  = mrpt::math::TPose2D(10,1,0);
	planner_input.world_bbox_min = mrpt::math::TPoint2D(-20,-20);
	planner_input.world_bbox_max = mrpt::math::TPoint2D( 20, 20);
	


	// Show results in a GUI and keep improving:
#if MRPT_HAS_WXWIDGETS
	mrpt::gui::CDisplayWindow3D  win("Result",1024,800);
	while (win.isOpen())
#else
	for (size_t i=0;i<1;i++)
#endif
	{
		planner.solve( planner_input, planner_result);

		cout << "Found goal_distance: " << planner_result.goal_distance << endl;
		cout << "Found path_cost: " << planner_result.path_cost << endl;
		cout << "Acceptable goal nodes: " << planner_result.acceptable_goal_node_ids.size() << endl;

#if MRPT_HAS_WXWIDGETS
		// Show result in a GUI:
		mrpt::opengl::COpenGLScenePtr & scene = win.get3DSceneAndLock();
	
		planner.renderMoveTree(
			*scene,
			planner_input, planner_result,
			planner_result.best_goal_node_id
			);

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


