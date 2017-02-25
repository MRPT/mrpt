/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CImageGrabber_OpenCV.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/gui/CDisplayWindow.h>

using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace mrpt::obs;
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
