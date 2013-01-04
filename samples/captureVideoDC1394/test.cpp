/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
