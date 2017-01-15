/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/vision/chessboard_find_corners.h>
#include <mrpt/system/threads.h>
#include <mrpt/gui.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;

#include <mrpt/examples_config.h>
std::string   myDataDir = MRPT_EXAMPLES_BASE_DIRECTORY + string("checkerboardDetectors/");

// ------------------------------------------------------
//				TestCheckerboardDetectors
// ------------------------------------------------------
void TestCheckerboardDetectors()
{
	CTimeLogger  timlog;

	// Load img:
	CImage img;
	if (!img.loadFromFile( myDataDir + string("test_1_checkerboard_9x6.jpg")) )
		throw std::runtime_error("Can't load demo image!");

	// Detect multiple-checkerboards:
	vector<TPixelCoordf> 	cornerCoords;
	const unsigned int  checkerboard_size_x  = 6;
	const unsigned int  checkerboard_size_y  = 9;

	// Detect:
	timlog.enter("findChessboardCorners [OpenCV]");

	//bool detectOk1 = 
	mrpt::vision::findChessboardCorners(
		img,
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

	//bool detectOk2 = 
	mrpt::vision::findChessboardCorners(
		img,
		cornerCoords,
		checkerboard_size_x, checkerboard_size_y,
		true, // normalize_image
		true // useScaramuzzaMethod
		);

	timlog.leave("findChessboardCorners [Scaramuzza]");

	// Draw:
	CImage img_detect2 = img;
	img_detect2.drawChessboardCorners(cornerCoords,checkerboard_size_x,checkerboard_size_y);

	//ASSERT_(detectOk1 && detectOk2);

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
		TestCheckerboardDetectors();
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

