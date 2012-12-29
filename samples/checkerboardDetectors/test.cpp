/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
#include <mrpt/vision.h>
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

