/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace std;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("gridMapLikelihoodCharacterization/") );

// Default .ini file:
string   iniFile( myDataDir+string("config_likelihood.ini") );


// ------------------------------------------------------
//				theMainThing
// ------------------------------------------------------
void theMainThing()
{
	COccupancyGridMap2D			gridMap;
	CObservation2DRangeScan		obsScan;
	CTicTac						tictac;

	// Load the grid map from file
	// ----------------------------------------------
	obsScan.aperture = M_2PIf;

	ASSERT_(mrpt::system::fileExists(iniFile));
	CConfigFile	cfgFile(iniFile);

	string  bmp=cfgFile.read_string("Params","bitmap_file","",true);
	float	res=cfgFile.read_float("Params","evaluation_grid_resolution",0.1f,true);

	float evalgrid_x_min=0,evalgrid_x_max=0;
	float evalgrid_y_min=0,evalgrid_y_max=0;

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(evalgrid_x_min, float, cfgFile, "Params");
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(evalgrid_x_max, float, cfgFile, "Params");
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(evalgrid_y_min, float, cfgFile, "Params");
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(evalgrid_y_max, float, cfgFile, "Params");

	int type_experiment=0;
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(type_experiment, int, cfgFile, "Params");

	// Gridmap:
	string   GRIDMAP_FILE( myDataDir+bmp );

	gridMap.loadFromBitmapFile( GRIDMAP_FILE, 0.05f );
	gridMap.saveAsBitmapFile("./out_gridmap.png");

	if (type_experiment==0)
	{
		// Simulate scan:
		// ----------------------------------------------
		obsScan.aperture = M_PIf;
		obsScan.maxRange = 80.0f;
		obsScan.rightToLeft = true;
		gridMap.laserScanSimulator( obsScan, CPose2D(0,0,0), 0.5f, 180 );
	}
	else
	{
		// Lik. of each 2D  position
		// ------------------------------------
		obsScan.aperture = 0;
		obsScan.maxRange = 80.0f;
		obsScan.rightToLeft = true;
        obsScan.resizeScanAndAssign(1, 0.0, true);
	}

	// Set options:
	// ----------------------------------------------
	gridMap.likelihoodOptions.loadFromConfigFile( cfgFile, "LikelihoodOptions" );
	gridMap.likelihoodOptions.dumpToConsole();

	// Perform simulation:
	// ----------------------------------------------
	printf("Performing simulation (saving to out_lik.txt)...");

	FILE	*f=os::fopen("out_lik.txt","wt");
	ASSERT_(f);

	CSimplePointsMap	pointsMap;
	pointsMap.clear();
	pointsMap.insertPoint(0,0);

	tictac.Tic();


	float	phi = (float)DEG2RAD( 0 );

	CPose2D		nullPose(0,0,phi);

	for (float y=evalgrid_y_min;y<evalgrid_y_max;y+=res)
	{
		for (float x=evalgrid_x_min;x<evalgrid_x_max;x+=res)
		{
			nullPose.x( x );
			nullPose.y( y );
			fprintf(f,"%e ", gridMap.computeObservationLikelihood( &obsScan, nullPose) );
		} // for y
		fprintf(f,"\n");
	} // for x

	printf("Done!\n");

	printf("Time:%fms\n", 1000.0f*tictac.Tac() );

	os::fclose(f);
}


int main()
{
	try
	{
		theMainThing();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception:\n" << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Runtime exception!!");
		return -1;
	}

}
