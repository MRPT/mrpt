/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <gtest/gtest.h>

// Defined in tests/test_main.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}

TEST(NavTests, PTGs_tests)
{
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::nav;

	const string sFil = mrpt::utils::MRPT_GLOBAL_UNITTEST_SRC_DIR + string("/tests/PTGs_for_tests.ini");
	if (!mrpt::system::fileExists(sFil))
	{
		cerr << "**WARNING* Skipping tests since file cannot be found: '" << sFil << "'\n";
		return;
	}

	mrpt::utils::CConfigFile cfg(sFil);

	const unsigned int PTG_COUNT = cfg.read_int("PTG_UNIT_TESTS","PTG_COUNT",0, true );
	EXPECT_TRUE(PTG_COUNT>0);
	vector<CParameterizedTrajectoryGenerator *> PTGs(PTG_COUNT);

	for ( unsigned int n=0;n<PTG_COUNT;n++)
	{
		// Factory:
		const string sPTGName = cfg.read_string("PTG_UNIT_TESTS",format("PTG%u_Type", n ),"", true );
		PTGs[n] = CParameterizedTrajectoryGenerator::CreatePTG(sPTGName,cfg,"PTG_UNIT_TESTS", format("PTG%u_",n) );
		EXPECT_TRUE(PTGs[n]!=NULL) << "Failed creating PTG #" << n << endl;

		try
		{
			PTGs[n]->initialize( string(), false /*verbose */ );
		} catch (std::exception &e)
		{
			GTEST_FAIL() << "Failed initializing PTG #" << n << endl << e.what() << endl;
		}
	}

	// Run tests:
	// ---------------------------------------------------------
	for ( unsigned int n=0;n<PTG_COUNT;n++)
	{
		CParameterizedTrajectoryGenerator *ptg = PTGs[n];
		
		const std::string sPTGDesc = ptg->getDescription();
		const double refDist   = ptg->getRefDistance();
		const size_t num_paths = ptg->getPathCount();
		size_t num_tests_run = 0;

		// TEST: step <-> dist match
		{
			for (double dist=0.1;dist<refDist*0.5;dist+=0.2)
			{
				bool any_good = false;
				for (size_t k=0;k<num_paths;k++)
				{
					uint16_t step;

					if (ptg->getPathStepForDist(k,dist,step))
					{
						any_good = true;
						double d = ptg->getPathDist(k,step);
						EXPECT_NEAR(d,dist, 0.05) << "Test: step <-> dist match\n PTG: " << sPTGDesc << endl << "dist:" << dist << endl;
						num_tests_run++;
					}
				}
				EXPECT_TRUE(any_good) << "Test: step <-> dist match\n PTG: " << sPTGDesc << endl << "dist:" << dist << endl;
			}
		}

		// TEST: inverseMap_WS2TP
		{
			bool any_ok = false;
			bool skip_this_ptg = false;
			for (double tx=-refDist*0.5;!skip_this_ptg && tx<refDist*0.5;tx+=0.1)
			{
				for (double ty=-refDist*0.5;!skip_this_ptg && ty<refDist*0.5;ty+=0.1)
				{
					if (std::abs(tx)<1e-2 && std::abs(ty)<1e-2)
						continue; // TP-Space does not include the WS point (0,0) in its domain

					const double tolerance_dist = std::max(0.10,  10.0 * std::sqrt( tx*tx+ty*ty ) * M_PI *2 / ptg->getPathCount() );

					int k;
					double normalized_d;
					bool valid= ptg->inverseMap_WS2TP(tx,ty,k,normalized_d);
					if (valid && normalized_d<1.0)
					{
						any_ok = true;
						// Now, do the inverse operation:
						uint16_t step;
						bool step_ok = ptg->getPathStepForDist(k,normalized_d*refDist,step);
						EXPECT_TRUE(step_ok) << "PTG: " << sPTGDesc << endl << "(tx,ty): " << tx << " " << ty << " k= " << k << " normalized_d=" << normalized_d << endl;
						if (step_ok)
						{
							mrpt::math::TPose2D pose;
							ptg->getPathPose(k,step,pose);
							EXPECT_NEAR(pose.x, tx, tolerance_dist)  << "Test: inverseMap_WS2TP\n PTG#"<<n<< ": " << sPTGDesc << endl << "(tx,ty): " << tx << " " << ty << " k= " << k << " normalized_d=" << normalized_d << endl;
							EXPECT_NEAR(pose.y, ty, tolerance_dist)  << "Test: inverseMap_WS2TP\n PTG#"<<n<< ": " << sPTGDesc << endl << "(tx,ty): " << tx << " " << ty << " k= " << k << " normalized_d=" << normalized_d << endl;
							
							if (std::abs(pose.x-tx)>=tolerance_dist ||std::abs(pose.y-ty)>=tolerance_dist)
							     skip_this_ptg = true;
							else num_tests_run++;
						}
					}
				}
			}
			EXPECT_TRUE(any_ok) << "PTG: " << sPTGDesc << endl;
		}

		// TEST: TP_obstacles
		{
			bool skip_this_ptg = false;
			bool any_change_all = false;
			for (double ox=-refDist*0.5;!skip_this_ptg && ox<refDist*0.5;ox+=0.1)
			{
				for (double oy=-refDist*0.5;!skip_this_ptg && oy<refDist*0.5;oy+=0.1)
				{
					if (std::abs(ox)<1e-2 && std::abs(oy)<1e-2)
						continue; // TP-Space does not include the WS point (0,0) in its domain

					std::vector<double> TP_obstacles;
					ptg->initTPObstacles(TP_obstacles);

					const std::vector<double> TP_obstacles_org = TP_obstacles;
					ptg->updateTPObstacle(ox,oy, TP_obstacles);

					const bool any_change = (TP_obstacles_org!=TP_obstacles);
					if (any_change) any_change_all=true;
					num_tests_run++;
				}
			}
			EXPECT_TRUE(any_change_all);
		}


		printf("PTG `%50s` run %6u tests.\n", sPTGDesc.c_str(), (unsigned int)num_tests_run );

	} // for each ptg

	// Clean up:
	for ( unsigned int n=0;n<PTG_COUNT;n++)
		delete PTGs[n];


}

