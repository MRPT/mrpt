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


#ifdef MRPT_OPENCV_SRC_DIR
static string OPENCV_SRC_DIR = MRPT_OPENCV_SRC_DIR;
#else
static string OPENCV_SRC_DIR = "./";
#endif

// ------------------------------------------------------
//				TestCamera3DFaceDetection
// ------------------------------------------------------
void TestCamera3DFaceDetection( CCameraSensorPtr cam )
{
	CDisplayWindow  win("Live video");

	cout << "Close the window to exit." << endl;

	mrpt::gui::CDisplayWindow3D  win3D("3D camera view",800,600);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(6.0);
	win3D.setCameraPointingToPoint(2.5,0,0);

	mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

	mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(4.5);

	// Create the Opengl object for the point cloud:
	scene->insert( gl_points );
	scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
	scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

	win3D.unlockAccess3DScene();

	double counter = 0;
	mrpt::utils::CTicTac	tictac;

	while (win.isOpen())
	{
		if( !counter )
			tictac.Tic();

		mrpt::slam::CObservationPtr  obs = cam->getNextFrame();
		ASSERT_(obs);

		vector_detectable_object detected;

		CObservation3DRangeScanPtr o = CObservation3DRangeScanPtr(obs);
			
		faceDetector.detectObjects( obs, detected );
		
		if ( detected.size() > 0 )
		{	
			for ( unsigned int i = 0; i < detected.size(); i++ )
			{
				ASSERT_( IS_CLASS(detected[i],CDetectable3D ) )
				CDetectable3DPtr obj = CDetectable3DPtr( detected[i] );
				o->intensityImage.rectangle( obj->m_x, obj->m_y, obj->m_x+obj->m_width, obj->m_y + obj->m_height, TColor(255,0,0) );
			}
		}

		win.showImage(o->intensityImage);			

		win3D.get3DSceneAndLock();
		CColouredPointsMap pntsMap;
		pntsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
		pntsMap.loadFromRangeScan(*(o.pointer()));		

		gl_points->loadFromPointsMap(&pntsMap);
		win3D.unlockAccess3DScene();
		win3D.repaint();

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

	mrpt::slam::CObservationPtr  obs = cam->getNextFrame();
	ASSERT_(obs);

	if ( IS_CLASS(obs, CObservation3DRangeScan) )
	{
		TestCamera3DFaceDetection( cam );
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
				//CDetectable2D obj = (CDetectable2D*)(detected[0].pointer());
				ASSERT_( IS_CLASS(detected[0],CDetectable2D ) )
				CDetectable2DPtr obj = CDetectable2DPtr( detected[0] );
				//CDetectable2DPtr obj = CDetectable2DPtr( new CDetectable2D(  ) );
				o->image.rectangle( obj->m_x, obj->m_y, obj->m_x+obj->m_width, obj->m_y + obj->m_height, TColor(255,0,0) );
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

void TestImagesFaceDetection(int argc, char *argv[])
{
	CImage		img;
	CDisplayWindow  win("Result");
	mrpt::utils::CTicTac	tictac;

	for ( int i = 1; i < argc; i++ )
	{
		string fileName( argv[i] );
		
		if (!img.loadFromFile(myDataDir+fileName))
		{
			cerr << "Cannot load " << myDataDir+fileName << endl;
			continue;
		}

		vector_detectable_object detected;

		tictac.Tic();
		faceDetector.detectObjects( &(CImage(img)), detected );

		cout << "Detection time: " << tictac.Tac() << " s" << endl;

		for ( unsigned int i = 0; i < detected.size() ; i++ )
		{	
			ASSERT_( IS_CLASS(detected[i],CDetectable2D ) )
			CDetectable2DPtr obj = CDetectable2DPtr( detected[i] );
			img.rectangle( obj->m_x, obj->m_y, obj->m_x+obj->m_width, obj->m_y + obj->m_height, TColor(255,0,0) );
		}

		win.showImage(img);

		mrpt::system::pause();
	}
}

void TestPrepareDetector()
{
	CStringList  lst;
	lst.loadFromFile(myIniFile);
	CConfigFileMemory  cfg; 
	cfg.setContent(lst);
	cfg.write("CascadeClassifier","cascadeFileName", OPENCV_SRC_DIR + "/data/haarcascades/haarcascade_frontalface_alt2.xml");
	//cfg.write("CascadeClassifier","cascadeFileName", OPENCV_SRC_DIR + "/data/lbpcascades/lbpcascade_frontalface.xml");
	faceDetector.init( cfg );
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char *argv[])
{
	try
	{
		registerClass( CLASS_ID( CDetectableObject ) );
		registerClass( CLASS_ID( CDetectable2D ) );
		registerClass( CLASS_ID( CDetectable3D ) );

		TestPrepareDetector();

		if ( argc > 1 )
			TestImagesFaceDetection( argc, argv );
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