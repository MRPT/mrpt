/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
	if (!img.loadFromFile(
			"D:/imgs_temp/o.jpg"
			//myDataDir + string("test_3_checkerboards.jpg")
			))
		throw std::runtime_error("Can't load demo image!");

	// Detect multiple-checkerboards:
	vector<TPixelCoordf> 	cornerCoords;
	const unsigned int  checkerboard_size_x  = 6;
	const unsigned int  checkerboard_size_y  = 9;

	// Detect:
	timlog.enter("findChessboardCorners [OpenCV]");
	
	bool detectOk1 = img.findChessboardCorners(
				cornerCoords,
				checkerboard_size_x, checkerboard_size_y,
				true, // normalize_image
				false // useScaramuzzaMethod
				);

	timlog.leave("findChessboardCorners [OpenCV]");

	// Draw:
	CImage img_detect1 = img;
	img_detect1.drawChessboardCorners(cornerCoords,checkerboard_size_x,checkerboard_size_y);


	timlog.enter("findChessboardCorners [Scaramuzza]");
	
	bool detectOk2 = img.findChessboardCorners(
				cornerCoords,
				checkerboard_size_x, checkerboard_size_y,
				true, // normalize_image
				true // useScaramuzzaMethod
				);

	timlog.leave("findChessboardCorners [Scaramuzza]");

	// Draw:
	CImage img_detect2 = img;
	img_detect2.drawChessboardCorners(cornerCoords,checkerboard_size_x,checkerboard_size_y);

	// Show results:
	CDisplayWindow  win1("Detected checkerboard (OpenCV)");
	win1.showImage(img_detect1);

	CDisplayWindow  win2("Detected checkerboard (Scaramuzza)");
	win2.showImage(img_detect2);

	timlog.dumpAllStats();
	timlog.clear();

	// wait till user closes any window:
	while (win1.isOpen() && win2.isOpen())
	{
		mrpt::system::sleep(10);
	}
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

