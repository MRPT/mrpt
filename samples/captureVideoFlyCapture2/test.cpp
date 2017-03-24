/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CImageGrabber_FlyCapture2.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::gui;
using namespace std;


// ------------------------------------------------------
//				TestCapture_FlyCapture2
// ------------------------------------------------------

void TestCapture_FlyCapture2()
{
	cout << " FlyCapture2 version: " << CImageGrabber_FlyCapture2::getFC2version() << std::endl;

	// Create camera object:
	CImageGrabber_FlyCapture2 capture;

	// Open camera:
	TCaptureOptions_FlyCapture2 cam_options;

	cam_options.framerate="FRAMERATE_30";
	cam_options.videomode="VIDEOMODE_1280x960RGB";
	//cam_options.videomode="VIDEOMODE_1280x960Y8";

	capture.open(cam_options);


	CTicTac tictac;
	cout << "Press any key to stop capture to 'capture.rawlog'..." << endl;

	CFileGZOutputStream fil("./capture.rawlog");

	CDisplayWindow win("Capturing...");

	int cnt = 0;

	CObservationImagePtr obs= CObservationImage::Create();  // Memory will be freed by SF destructor in each loop.

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

		if (!capture.getObservation( *obs ))
		{
			cerr << "Error retrieving images!" << endl;
			break;
		}

		cout << "."; cout.flush();
		if (win.isOpen())
			win.showImage( obs->image );

		fil << obs;
	}

}


int main(int argc, char **argv)
{
	try
	{
		TestCapture_FlyCapture2();
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
