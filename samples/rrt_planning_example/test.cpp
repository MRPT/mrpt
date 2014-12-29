/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/nav.h>
//#include <mrpt/gui.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/system/filesystem.h> // directoryExists(), ...
#include <mrpt/utils/CFileGZInputStream.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace mrpt::slam; // for maps, etc.
using namespace std;

// Load example grid map
#include <mrpt/examples_config.h>
string   myGridMap( MRPT_EXAMPLES_BASE_DIRECTORY + string("../share/mrpt/datasets/2006-MalagaCampus.gridmap.gz") );


// ------------------------------------------------------
//				TestRRT1
// ------------------------------------------------------
void TestRRT1()
{
	// Load the gridmap:
	COccupancyGridMap2D		gridmap;

	if (!mrpt::system::fileExists(myGridMap))
		THROW_EXCEPTION_CUSTOM_MSG1("Map file '%s' not found",myGridMap.c_str());

	printf("Loading gridmap...");
	CFileGZInputStream(myGridMap) >> gridmap;
	printf("Done! %f x %f m\n", gridmap.getXMax()-gridmap.getXMin(), gridmap.getYMax()-gridmap.getYMin());


	// Find path:
	// ....

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

