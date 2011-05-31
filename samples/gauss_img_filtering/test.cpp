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

#include <mrpt/slam.h>
#include <mrpt/gui.h>

using namespace mrpt::utils;
using namespace mrpt::vision;
using namespace mrpt::gui;
using namespace std;


#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("gauss_img_filtering/") );

// ------------------------------------------------------
//					Test
// ------------------------------------------------------
void Test_GaussWindows()
{
	CTicTac         tictac;
	CImage      inImg, outImg;

	inImg.loadFromFile(myDataDir+"test_in.jpg");

	// Smoothed image:
	// ---------------------------
	tictac.Tic();

	inImg.filterGaussian(outImg, 11,11);  // Window size

	printf("Smoothed image in %.03fms\n",1000 * tictac.Tac());


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
	} catch (std::exception &e)
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

