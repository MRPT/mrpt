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

#include <mrpt/vision.h>
#include <mrpt/hwdrivers.h>
#include <mrpt/slam.h>

#include <mrpt/vision/CObjectDetection.h>
#include <mrpt/vision/CCascadeClassifierDetection.h>
#include <mrpt/vision/CFaceDetection.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::vision;
using namespace mrpt::hwdrivers;
using namespace std;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("face_detection/") );
string   myIniFile( MRPT_EXAMPLES_BASE_DIRECTORY + string("face_detection/FACE_DETECTION_TEST.INI") );

CFaceDetection faceDetector;


// ------------------------------------------------------
//				TestCameraFaceDetection
// ------------------------------------------------------
void TestCameraFaceDetection()
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
			vector_detectable_object detected;
			faceDetector.detectObjects( obs, detected );

			CObservationImagePtr o = CObservationImagePtr(obs);
			if ( detected.size() > 0 )
			{	
				CDetectable2D obj = (CDetectable2D*)(detected[0].pointer());
				//CDetectable2DPtr obj = CDetectable2DPtr( new CDetectable2D(  ) );
				o->image.rectangle( obj.m_x, obj.m_y, obj.m_x+obj.m_width, obj.m_y + obj.m_height, 154 );
			}

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

void TestImagesFaceDetection()
{

}

void TestPrepareDetector()
{
	faceDetector.init( myIniFile );
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char *argv[])
{
	try
	{
		TestPrepareDetector();

		if ( argc > 2 )
			TestImagesFaceDetection();
		else
			TestCameraFaceDetection();
		
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