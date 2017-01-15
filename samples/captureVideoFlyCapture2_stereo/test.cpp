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
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace mrpt::obs;
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

	cam_options_left.open_by_guid = true;
	cam_options_left.camera_guid[0] = 0x63A8D3CE;
	cam_options_left.camera_guid[1] = 0xB49580D6;
	cam_options_left.camera_guid[2] = 0x004AED1C;
	cam_options_left.camera_guid[3] = 0xDDE4EF14;

	cam_options_left.strobe_enabled = true;
	cam_options_left.strobe_source = 1;  // GPIO pin #
	cam_options_left.strobe_duration = 1.0; // ms

	capture_left.open(cam_options_left,false /*only open, don't start grabbing*/);

	// Right camera:
	// ------------------------------------------
	CImageGrabber_FlyCapture2   capture_right;
	TCaptureOptions_FlyCapture2 cam_options_right;

	cam_options_right.framerate = cam_options_left.framerate;
	cam_options_right.videomode = cam_options_left.videomode;

	cam_options_right.open_by_guid = true;
	cam_options_right.camera_guid[0] = 0xB9862FD2;
	cam_options_right.camera_guid[1] = 0x7AE0E03A;
	cam_options_right.camera_guid[2] = 0xA6BC0321;
	cam_options_right.camera_guid[3] = 0x16654DC9;

	cam_options_right.trigger_enabled = true;
	cam_options_right.trigger_source = 0;  // GPIO pin #

	capture_right.open(cam_options_right,false /*only open, don't start grabbing*/);

	// Open both cameras simultaneously:
#if 0
	const CImageGrabber_FlyCapture2 *cameras[2] = { &capture_left, &capture_right };
	CImageGrabber_FlyCapture2::startSyncCapture(2 /*numCameras*/, cameras);
#else
	capture_right.startCapture(); // Will not start until a strobe is got from the "master" camera:
	capture_left.startCapture();
#endif

	CTicTac tictac;
	cout << "Press any key to stop capture to 'capture.rawlog'..." << endl;

	CFileGZOutputStream fil("./capture.rawlog");

	CDisplayWindow winL("Left"), winR("Right");

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

		const bool ok1 = capture_left.getObservation( *obsL );
		const bool ok2 = capture_right.getObservation( *obsR );
		if (!ok1 || !ok2)
		{
			cerr << "Error retrieving images!" << endl;
			break;
		}

		cout << "."; cout.flush();
		if (winL.isOpen()) winL.showImage( obsL->image );
		if (winR.isOpen()) winR.showImage( obsR->image );

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
