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
#include <mrpt/gui.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
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
	CPathPlanningCircularRobot	pathPlanning;
	pathPlanning.robotRadius = 0.30f;

	std::deque<poses::TPoint2D>		thePath;
	bool	notFound;
	CTicTac	tictac;

//	CPoint2D  origin( 20, 1 );
	CPoint2D  origin( 20, -110 );
	CPoint2D  target( 90, 40 );

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

	for (std::deque<poses::TPoint2D>::const_iterator it=thePath.begin();it!=thePath.end();++it)
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

