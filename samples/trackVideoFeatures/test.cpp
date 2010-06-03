/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#include <mrpt/gui.h>

using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace std;


// ------------------------------------------------------
//			TestTrackVideo
// ------------------------------------------------------
void TestTrackVideo()
{
	CImageGrabber_OpenCV		capture( -1 );
//	CImageGrabber_OpenCV		capture( "aaa.avi" );

	
	CTicTac						tictac;

	cout << "Press any key to exit..." << endl;

	CDisplayWindow		win("Capturing...");

	int cnt = 0;
	CObservationImage  imgObs;

	while (!mrpt::system::os::kbhit() && win.isOpen())
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

		if (!capture.getObservation( imgObs ))
		{
			cerr << "Error retrieving images!" << endl;
			break;
		}
		
		cout << "."; cout.flush();
		if (win.isOpen())  
			win.showImage( imgObs.image );
	}

	cout << endl << "Exiting..." << endl;
}


int main()
{
	try
	{
		TestTrackVideo();

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
