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


//#define DO_CAPTURE		1
#define DO_CAPTURE		0

// ------------------------------------------------------
//				TestCapture
// ------------------------------------------------------

void TestCapture_1394()
{
	TCaptureOptions_dc1394 	options;

	uint64_t	cameraGUID = 0;
	uint16_t	cameraUnit = 0;

	options.frame_width = 1024; //640;
	options.frame_height = 768; // 480;
	options.color_coding = COLOR_CODING_YUV422;

	// Other capture options:
	//options.shutter = 900;

	// For stereo Bumblebee tests/debugging (Use the Bumblebee class in mrpt::vision instead!)
//	options.mode7 = 3;
//	options.deinterlace_stereo = true;


	CImageGrabber_dc1394	capture( cameraGUID, cameraUnit, options, true /* Verbose */ );

	CTicTac		tictac;

	cout << "Press any key to stop capture to 'capture.rawlog'..." << endl;

#if DO_CAPTURE
	CFileGZOutputStream fil("./capture.rawlog");
#endif

	CDisplayWindow		win("Capturing...");

	int cnt = 0;

	while (!mrpt::system::os::kbhit())
	{
		if ( (cnt++ % 10) == 0 )
		{
			if (cnt>0)
			{
				double t = tictac.Tac();
				double FPS = 10 / t;
				printf("\n %f FPS\n", FPS);

				// Other capture options:
				//options.shutter = cnt + 1;
				//capture.changeCaptureOptions(options);
			}
			tictac.Tic();
		}

		CObservationImagePtr obs= CObservationImage::Create(); // Memory will be freed by SF destructor in each loop.
		if (!capture.getObservation( *obs ))
		{
			cerr << "Error retrieving images!" << endl;
			break;
		}

#if DO_CAPTURE
		fil << obs;
#endif
		cout << "."; cout.flush();
		if (win.isOpen())
			win.showImage( obs->image );
	}

}


int main(int argc, char **argv)
{
	try
	{
		TestCapture_1394();

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
