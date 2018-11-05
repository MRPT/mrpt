/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/system/CTicTac.h>
#include <mrpt/img/CImage.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <iostream>

using namespace mrpt::gui;
using namespace mrpt::img;
using namespace mrpt::system;
using namespace std;

#include <mrpt/examples_config.h>
string myDataDir(
	MRPT_EXAMPLES_BASE_DIRECTORY + string("img_gauss_filtering_example/"));

// ------------------------------------------------------
//					Test
// ------------------------------------------------------
void Test_GaussWindows()
{
	CTicTac tictac;
	CImage inImg, outImg;

	inImg.loadFromFile(myDataDir + "test_in.jpg");

	// Smoothed image:
	// ---------------------------
	tictac.Tic();

	inImg.filterGaussian(outImg, 11, 11);  // Window size

	printf("Smoothed image in %.03fms\n", 1000 * tictac.Tac());

	CDisplayWindow win1("Original Image");
	CDisplayWindow win2("Smoothed Image");

	win1.showImage(inImg);
	win2.showImage(outImg);

	mrpt::system::pause();
}

int main()
{
	try
	{
		Test_GaussWindows();

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
