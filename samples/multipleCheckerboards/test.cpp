/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/vision/chessboard_camera_calib.h>
#include <mrpt/vision/chessboard_find_corners.h>
#include <mrpt/gui/CDisplayWindow.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;

#include <mrpt/examples_config.h>
std::string   myDataDir = MRPT_EXAMPLES_BASE_DIRECTORY + string("multipleCheckerboards/");


// ------------------------------------------------------
//				TestMultipleCheckerboard
// ------------------------------------------------------
void TestMultipleCheckerboard(
	const std::string &img_filename,
	const unsigned int  checkerboard_size_x,
	const unsigned int  checkerboard_size_y
	)
{
	CTimeLogger  timlog;

	// Load img:
	CImage img;
	if (!img.loadFromFile( img_filename ))
		throw std::runtime_error("Can't load image!");

	// Detect multiple-checkerboards:
	vector<vector<TPixelCoordf> > 	listCornerCoords;

	timlog.enter("findMultipleChessboardsCorners");

	mrpt::vision::findMultipleChessboardsCorners(
		img,
		listCornerCoords,
		checkerboard_size_x, checkerboard_size_y );

	timlog.leave("findMultipleChessboardsCorners");

	cout << "Number of checkerboards detected: " << listCornerCoords.size() << endl;

	// Draw:
	CImage img_detect;
	img.colorImage(img_detect);
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
int main(int argc, char **argv)
{
	try
	{
		std::string sFile = myDataDir + string("test_3_checkerboards_5x4.jpg");
		unsigned int  checkerboard_size_x  = 5;
		unsigned int  checkerboard_size_y  = 4;

		if (argc==4)
		{
			sFile = std::string(argv[1]);
			checkerboard_size_x = atoi(argv[2]);
			checkerboard_size_y = atoi(argv[3]);
		}
		else if (argc!=1)
		{
			std::cerr << "Usage: " << argv[0] << " [IMAGE_FILE NX NY]\n";
			return 1;
		}


		TestMultipleCheckerboard(sFile,checkerboard_size_x,checkerboard_size_y);
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

