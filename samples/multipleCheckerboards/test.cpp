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

#include <mrpt/vision.h>
#include <mrpt/gui.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;

#include <mrpt/examples_config.h>
std::string   myDataDir = MRPT_EXAMPLES_BASE_DIRECTORY + string("multipleCheckerboards/");


// ------------------------------------------------------
//				TestMultipleCheckerboard
// ------------------------------------------------------
void TestMultipleCheckerboard()
{
	CTimeLogger  timlog;

	// Load img:
	CImage img;
	if (!img.loadFromFile( myDataDir + string("test_3_checkerboards_5x4.jpg") ))
		throw std::runtime_error("Can't load demo image!");

	// Detect multiple-checkerboards:
	vector<vector<TPixelCoordf> > 	listCornerCoords;
	const unsigned int  checkerboard_size_x  = 5;
	const unsigned int  checkerboard_size_y  = 4;

	timlog.enter("findMultipleChessboardsCorners");
	
	mrpt::vision::findMultipleChessboardsCorners(
		img,
		listCornerCoords,
		checkerboard_size_x, checkerboard_size_y );

	timlog.leave("findMultipleChessboardsCorners");

	cout << "Number of checkerboards detected: " << listCornerCoords.size() << endl;

	// Draw:
	CImage img_detect = img;
	for (size_t i=0;i<listCornerCoords.size();i++)
		img_detect.drawChessboardCorners(listCornerCoords[i],checkerboard_size_x,checkerboard_size_y);

	// Show results:
	CDisplayWindow  win1("Detected checkerboards ");
	win1.showImage(img_detect);

	timlog.dumpAllStats();
	timlog.clear();

	// wait till user closes any window:
	win1.waitForKey();
}



// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestMultipleCheckerboard();
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

