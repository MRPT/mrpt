/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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

#include <mrpt/obs.h>
#include <mrpt/gui.h>
#include <mrpt/hwdrivers.h>

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

		mrpt::slam::CObservationPtr  obs = cam->getNextFrame();
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

