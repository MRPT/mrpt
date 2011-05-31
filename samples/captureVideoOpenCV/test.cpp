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

#include <mrpt/hwdrivers.h>

using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace mrpt::slam;
using namespace std;



// ------------------------------------------------------
//				TestCaptureOpenCV
// ------------------------------------------------------
bool LIVE_CAM = true;
int N_CAM_TO_OPEN = 0;
std::string AVI_TO_OPEN;


void TestCapture_OpenCV()
{
	CImageGrabber_OpenCV	*capture = NULL;

	if (LIVE_CAM)
	{
#if 0	// test: Select the desired resolution
		mrpt::vision::TCaptureCVOptions	opts;
		opts.frame_width = 320;
		opts.frame_height = 240;
		capture = new CImageGrabber_OpenCV( 0, CAMERA_CV_AUTODETECT, opts );
#else
		capture = new CImageGrabber_OpenCV( N_CAM_TO_OPEN, CAMERA_CV_AUTODETECT);
#endif
	}
	else
	{
		capture = new CImageGrabber_OpenCV( AVI_TO_OPEN );
	}

	CTicTac						tictac;

	cout << "Press any key to stop capture to 'capture.rawlog'..." << endl;

	CFileGZOutputStream fil("./capture.rawlog");

	CDisplayWindow		win("Capturing...");

	int cnt = 0;

	while (!mrpt::system::os::kbhit())
	{
		if ( (cnt++ % 20) == 0 )
		{
			if (cnt>0)
			{
				double t = tictac.Tac();
				double FPS = 20 / t;
				printf("\n %f FPS\n", FPS);
			}
			tictac.Tic();
		}

		CObservationImagePtr obs= CObservationImage::Create();  // Memory will be freed by SF destructor in each loop.
		if (!capture->getObservation( *obs ))
		{
			cerr << "Error retrieving images!" << endl;
			break;
		}

		fil << obs;

		cout << "."; cout.flush();
		if (win.isOpen())
			win.showImage( obs->image );
	}

	delete capture;
}


int main(int argc, char **argv)
{
	try
	{
		if (argc>1)
		{
			if ( !strstr(argv[1],".avi") )
			{
				LIVE_CAM = true;
				N_CAM_TO_OPEN = atoi(argv[1]);
			}
			else
			{
				LIVE_CAM = false;
				AVI_TO_OPEN = argv[1];
			}
		}

		TestCapture_OpenCV();

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
