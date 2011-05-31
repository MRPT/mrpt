/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>
#include <mrpt/slam.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
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
		obsScan.validRange.resize(1,1);
		obsScan.scan.resize(1,0);
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
