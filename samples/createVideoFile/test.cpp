/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CImage.h>
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
