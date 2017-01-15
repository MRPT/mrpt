/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/poses/CPose2D.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::maps;
using namespace mrpt::nav;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;

#include <mrpt/examples_config.h>

string   myGridMap( MRPT_EXAMPLES_BASE_DIRECTORY + string("../share/mrpt/datasets/2006-MalagaCampus.gridmap.gz") );


// ------------------------------------------------------
//				TestPathPlanning
// ------------------------------------------------------
void TestPathPlanning()
{
	// Load the gridmap:
	COccupancyGridMap2D		gridmap;

	if (!mrpt::system::fileExists(myGridMap))
		THROW_EXCEPTION_CUSTOM_MSG1("Map file '%s' not found",myGridMap.c_str());

	printf("Loading gridmap...");
	CFileGZInputStream(myGridMap) >> gridmap;
	printf("Done! %f x %f m\n", gridmap.getXMax()-gridmap.getXMin(), gridmap.getYMax()-gridmap.getYMin());


	// Find path:
	PlannerSimple2D	pathPlanning;
	pathPlanning.robotRadius = 0.30f;

	std::deque<TPoint2D>		thePath;
	bool	notFound;
	CTicTac	tictac;

	CPose2D  origin( 20, -110, 0 );
	CPose2D  target( 90, 40, 0 );

	cout << "Origin: " << origin << endl;
	cout << "Target: " << target << endl;

	cout << "Searching path..."; cout.flush();
	tictac.Tic();

	pathPlanning.computePath( gridmap, origin, target, thePath, notFound, 100.0f /* Max. distance */ );

	double t = tictac.Tac();
	cout << "Done in " << t*1000 << " ms" << endl;

	printf("Path found: %s\n", notFound ? "NO":"YES");
	printf("Path has %u steps\n", (unsigned)thePath.size());

	// Save result:
	CImage		img;
	gridmap.getAsImage(img,false, true);  // Force a RGB image

	// Draw the path:
	// ---------------------
	int R = round(pathPlanning.robotRadius / gridmap.getResolution() );

	for (std::deque<TPoint2D>::const_iterator it=thePath.begin();it!=thePath.end();++it)
		img.drawCircle( gridmap.x2idx(it->x),gridmap.getSizeY()-1-gridmap.y2idx(it->y),R, TColor(0,0,255) );

	img.cross(gridmap.x2idx(origin.x()),gridmap.getSizeY()-1-gridmap.y2idx(origin.y()),TColor(0x20,0x20,0x20),'+',10);
	img.cross(gridmap.x2idx(target.x()),gridmap.getSizeY()-1-gridmap.y2idx(target.y()),TColor(0x50,0x50,0x50),'x',10);

	const std::string dest = "path_planning.png";
	cout << "Saving output to: " << dest << endl;
	img.saveToFile(dest);
	printf("Done\n");

#if MRPT_HAS_WXWIDGETS
	mrpt::gui::CDisplayWindow	win("Computed path");
	win.showImage(img.scaleHalf().scaleHalf());

	win.waitForKey();
#endif

}

int main(int argc, char **argv)
{
	try
	{
		TestPathPlanning();
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

