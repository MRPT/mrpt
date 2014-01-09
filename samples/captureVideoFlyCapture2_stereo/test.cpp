/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers.h>

using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace std;


// ------------------------------------------------------
//				TestCapture_FlyCapture2_stereo
// ------------------------------------------------------

void TestCapture_FlyCapture2_stereo()
{
	cout << " FlyCapture2 version: " << CImageGrabber_FlyCapture2::getFC2version() << std::endl;

	// Left camera:
	// ------------------------------------------
	CImageGrabber_FlyCapture2   capture_left;
	TCaptureOptions_FlyCapture2 cam_options_left;

	cam_options_left.framerate="FRAMERATE_30";
	cam_options_left.videomode="VIDEOMODE_1280x960RGB";
	cam_options_left.camera_index = 0;

	cam_options_left.strobe_enabled = true;
	cam_options_left.strobe_duration = 1.0; // ms
	
	capture_left.open(cam_options_left,false /*only open, don't start grabbing*/);

	// Right camera:
	// ------------------------------------------
	CImageGrabber_FlyCapture2   capture_right;
	TCaptureOptions_FlyCapture2 cam_options_right;

	cam_options_right.framerate = cam_options_left.framerate;
	cam_options_right.videomode = cam_options_left.videomode;
	cam_options_right.camera_index = 0;

	cam_options_right.trigger_enabled = true;

	capture_right.open(cam_options_right,false /*only open, don't start grabbing*/);

	// Open both cameras simultaneously:
	const CImageGrabber_FlyCapture2 *cameras[2] = { &capture_left, &capture_right };

	CImageGrabber_FlyCapture2::startSyncCapture(2 /*numCameras*/, cameras);


	CTicTac tictac;
	cout << "Press any key to stop capture to 'capture.rawlog'..." << endl;

	CFileGZOutputStream fil("./capture.rawlog");

	CDisplayWindow3D winL("Left"), winR("Right");

	int cnt = 0;

	CObservationImagePtr obsL= CObservationImage::Create();  // Memory will be freed by SF destructor in each loop.
	obsL->sensorLabel="LEFT";

	CObservationImagePtr obsR= CObservationImage::Create();  // Memory will be freed by SF destructor in each loop.
	obsR->sensorLabel="RIGHT";

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

		if (!capture_left.getObservation( *obsL ) || 
			!capture_right.getObservation( *obsR ) )
		{
			cerr << "Error retrieving images!" << endl;
			break;
		}

		cout << "."; cout.flush();
		if (winL.isOpen()) winL.setImageView( obsL->image );
		if (winR.isOpen()) winR.setImageView( obsL->image );

		fil << obsL << obsR;
	}

}


int main(int argc, char **argv)
{
	try
	{
		TestCapture_FlyCapture2_stereo();
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
