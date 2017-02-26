/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/gui.h>
#include <mrpt/utils/CTicTac.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace std;


// ------------------------------------------------------
//				TestCameraCaptureAsk
// ------------------------------------------------------
void TestCameraCaptureAsk()
{
	CCameraSensorPtr cam = prepareVideoSourceFromUserSelection();

	if (!cam)
	{
		cerr << "The user didn't pick any camera. Exiting." << endl;
		return;
	}

	CDisplayWindow  win("Live video");

	cout << "Close the window to exit." << endl;

	double counter = 0;
	mrpt::utils::CTicTac	tictac;

	while (win.isOpen())
	{
		if( !counter )
			tictac.Tic();

		mrpt::obs::CObservationPtr  obs = cam->getNextFrame();
		ASSERT_(obs);

		if (IS_CLASS(obs,CObservationImage))
		{
			CObservationImagePtr o=CObservationImagePtr(obs);
			win.showImage(o->image);
		}
		else if (IS_CLASS(obs,CObservationStereoImages))
		{
			CObservationStereoImagesPtr o=CObservationStereoImagesPtr(obs);
			win.showImage(o->imageRight);
		}
		if( ++counter == 10 )
		{
			double t = tictac.Tac();
			cout << "Frame Rate: " << counter/t << " fps" << endl;
			counter = 0;
		}
		mrpt::system::sleep(2);
	}

	cout << "Closing..." << endl;
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestCameraCaptureAsk();

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

