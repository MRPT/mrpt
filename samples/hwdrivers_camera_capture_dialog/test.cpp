/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/img/CImage.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;
using namespace mrpt::img;
using namespace std;

// ------------------------------------------------------
//				TestCameraCaptureAsk
// ------------------------------------------------------
void TestCameraCaptureAsk()
{
	CCameraSensor::Ptr cam = prepareVideoSourceFromUserSelection();

	if (!cam)
	{
		cerr << "The user didn't pick any camera. Exiting." << endl;
		return;
	}

	CDisplayWindow win("Live video");

	cout << "Press 's' to save frames.\nClose the window to exit.\n";

	double counter = 0;
	mrpt::system::CTicTac tictac;

	while (win.isOpen())
	{
		if (!counter) tictac.Tic();

		mrpt::obs::CObservation::Ptr obs = cam->getNextFrame();
		ASSERT_(obs);

		CImage* img = nullptr;

		if (IS_CLASS(obs, CObservationImage))
		{
			CObservationImage::Ptr o =
				std::dynamic_pointer_cast<CObservationImage>(obs);
			img = &o->image;
		}
		else if (IS_CLASS(obs, CObservationStereoImages))
		{
			CObservationStereoImages::Ptr o =
				std::dynamic_pointer_cast<CObservationStereoImages>(obs);
			img = &o->imageRight;
		}
		else if (IS_CLASS(obs, CObservation3DRangeScan))
		{
			CObservation3DRangeScan::Ptr o =
				std::dynamic_pointer_cast<CObservation3DRangeScan>(obs);
			if (o->hasIntensityImage) img = &o->intensityImage;
		}

		if (img) win.showImage(*img);

		if (++counter == 10)
		{
			double t = tictac.Tac();
			cout << "Frame Rate: " << counter / t << " fps" << endl;
			counter = 0;
		}

		// Process keystrokes:
		if (mrpt::system::os::kbhit())
		{
			const int key_code = mrpt::system::os::getch();
			switch (key_code)
			{
				case 's':
				case 'S':
				{
					static int cnt = 0;
					const std::string sFile =
						mrpt::format("frame%05i.png", cnt++);
					cout << "Saving frame to: " << sFile << endl;
					img->saveToFile(sFile);
				}
				break;
				default:
					break;
			};
		}

		std::this_thread::sleep_for(2ms);
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
	}
	catch (const std::exception& e)
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
