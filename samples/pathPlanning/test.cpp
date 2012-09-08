/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

