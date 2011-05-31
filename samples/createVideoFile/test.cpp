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
#include <mrpt/vision/CVideoFileWriter.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::vision;
using namespace std;

/* ------------------------------------------------------------------------
					Test_VideoFile
   ------------------------------------------------------------------------ */
void Test_VideoFile()
{
	CVideoFileWriter  vid;

	cout << "Creating test.avi..." << endl;

	const int W = 352;
	const int H = 288;

	vid.open("test.avi",15,TImageSize(W,H));  // Use default codec
//	vid.open("test.avi",15,TImageSize(W,H),"XVID");

	for (int i=1;i<100;i++)
	{
		CImage  img(W,H);

		img.rectangle(0,0,320,200,TColor::black);

		img.drawCircle( 160 + 50*cos(0.05*i), 120 + 50*sin(0.05*i), 30, TColor(255,255,255) );

		vid << img;

		cout << "frame " << i << endl;
	}
	vid.close();

	cout << "Video closed "  << endl;
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		Test_VideoFile();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!");
		return -1;
	}
}
