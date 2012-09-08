/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
