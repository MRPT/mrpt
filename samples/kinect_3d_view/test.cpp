/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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

/*
  Example  : kinect_3d_view
  Web page : http://www.mrpt.org/Kinect_and_MRPT

  Purpose  : Demonstrate grabbing from CKinect, multi-threading
             and live 3D rendering.
*/


#include <mrpt/hwdrivers.h>
#include <mrpt/gui.h>
#include <mrpt/maps.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace std;

// Thread for grabbing: Do this is another thread so we divide rendering and grabbing
//   and exploit multicore CPUs.
struct TThreadParam
{
	TThreadParam() : quit(false), pushed_key(0), tilt_ang_deg(0), Hz(0) { }

	volatile bool   quit;
	volatile int    pushed_key;
	volatile double tilt_ang_deg;
	volatile double Hz;

	mrpt::synch::CThreadSafeVariable<CObservation3DRangeScanPtr> new_obs;     // RGB+D (+3D points)
	mrpt::synch::CThreadSafeVariable<CObservationIMUPtr>         new_obs_imu; // Accelerometers
};

void thread_grabbing(TThreadParam &p)
{
	try
	{
		CKinect  kinect;

		// Set params:
		// kinect.enableGrab3DPoints(true);
		// kinect.enablePreviewRGB(true);
		//...

		// Open:
		cout << "Calling CKinect::initialize()...";
		kinect.initialize();
		cout << "OK\n";

		CTicTac tictac;
		int nImgs = 0;
		bool there_is_obs=true, hard_error=false;

		while (!hard_error && !p.quit)
		{
			// Grab new observation from the camera:
			CObservation3DRangeScanPtr  obs     = CObservation3DRangeScan::Create(); // Smart pointers to observations
			CObservationIMUPtr          obs_imu = CObservationIMU::Create();       

			kinect.getNextObservation(*obs,*obs_imu,there_is_obs,hard_error);

			if (!hard_error && there_is_obs)
			{
				p.new_obs.set(obs);
				p.new_obs_imu.set(obs_imu);
			}

			if (p.pushed_key!=0)
			{
				switch (p.pushed_key)
				{
					case 's':
						p.tilt_ang_deg-=1;
						if (p.tilt_ang_deg<-31) p.tilt_ang_deg=-31;
						kinect.setTiltAngleDegrees(p.tilt_ang_deg);
						break;
					case 'w':
						p.tilt_ang_deg+=1;
						if (p.tilt_ang_deg>31) p.tilt_ang_deg=31;
						kinect.setTiltAngleDegrees(p.tilt_ang_deg);
						break;
					case 27:
						p.quit = true;
						break;
				}

				// Clear pushed key flag:
				p.pushed_key = 0;
			}

			nImgs++;
			if (nImgs>10)
			{
				p.Hz = nImgs / tictac.Tac();
				nImgs=0;
				tictac.Tic();
			}
		}
	}
	catch(std::exception &e)
	{
		cout << "Exception in Kinect thread: " << e.what() << endl;
		p.quit = true;
	}
}

// ------------------------------------------------------
//				Test_Kinect
// ------------------------------------------------------
void Test_Kinect()
{
	// Launch grabbing thread:
	// --------------------------------------------------------
	TThreadParam thrPar;
	mrpt::system::TThreadHandle thHandle= mrpt::system::createThreadRef(thread_grabbing ,thrPar);

	// Create window and prepare OpenGL object in the scene:
	// --------------------------------------------------------
	mrpt::gui::CDisplayWindow3D  win3D("Kinect 3D view",800,600);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(8.0);
	win3D.setFOV(90);
	win3D.setCameraPointingToPoint(2.5,0,0);

	mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(2.5);

	const double aspect_ratio =  480.0 / 640.0; // kinect.getRowCount() / double( kinect.getColCount() );

	mrpt::opengl::CTexturedPlanePtr gl_img_range 			=  mrpt::opengl::CTexturedPlane::Create(0.5,-0.5,-0.5*aspect_ratio,0.5*aspect_ratio);
	mrpt::opengl::CTexturedPlanePtr gl_img_intensity 		=  mrpt::opengl::CTexturedPlane::Create(0.5,-0.5,-0.5*aspect_ratio,0.5*aspect_ratio);

	{
		mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

		// Create the Opengl object for the point cloud:
		scene->insert( gl_points );
		scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
		scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

		const int VW_WIDTH = 250;	// Size of the viewport into the window, in pixel units.
		const int VW_HEIGHT = aspect_ratio*VW_WIDTH;
		const int VW_GAP = 10;

		// Create the Opengl objects for the planar images, as textured planes, each in a separate viewport:
		win3D.addTextMessage(30,-10-1*(VW_GAP+VW_HEIGHT),"Range data",TColorf(1,1,1), 1, MRPT_GLUT_BITMAP_HELVETICA_12 );
		opengl::COpenGLViewportPtr viewRange = scene->createViewport("view2d_range");
		scene->insert( gl_img_range, "view2d_range");
		viewRange->setViewportPosition(5,-10-1*(VW_GAP+VW_HEIGHT), VW_WIDTH,VW_HEIGHT);
		viewRange->setTransparent(true);
		viewRange->getCamera().setOrthogonal(true);
		viewRange->getCamera().setAzimuthDegrees(90);
		viewRange->getCamera().setElevationDegrees(90);
		viewRange->getCamera().setZoomDistance(1.0);

		win3D.addTextMessage(30, -10-2*(VW_GAP+VW_HEIGHT),"Intensity data",TColorf(1,1,1), 2, MRPT_GLUT_BITMAP_HELVETICA_12 );
		opengl::COpenGLViewportPtr viewInt = scene->createViewport("view2d_int");
		scene->insert( gl_img_intensity, "view2d_int");
		viewInt->setViewportPosition(5, -10-2*(VW_GAP+VW_HEIGHT), VW_WIDTH,VW_HEIGHT );
		viewInt->setTransparent(true);
		viewInt->getCamera().setOrthogonal(true);
		viewInt->getCamera().setAzimuthDegrees(90);
		viewInt->getCamera().setElevationDegrees(90);
		viewInt->getCamera().setZoomDistance(1.0);

		win3D.unlockAccess3DScene();
		win3D.repaint();
	}


	CObservation3DRangeScanPtr  last_obs;
	CObservationIMUPtr          last_obs_imu;

	while (win3D.isOpen() && !thrPar.quit)
	{
		CObservation3DRangeScanPtr possiblyNewObs = thrPar.new_obs.get();
		if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP &&
			(!last_obs  || possiblyNewObs->timestamp!=last_obs->timestamp ) )
		{
			// It IS a new observation:
			last_obs     = possiblyNewObs;
			last_obs_imu = thrPar.new_obs_imu.get();

			// Update visualization ---------------------------------------

			// Show ranges as 2D:
			if (last_obs->hasRangeImage )
			{
				mrpt::utils::CImage  img;
				// Normalize the image
				const CMatrixFloat  range2D = last_obs->rangeImage * (1.0/ 5.0); //kinect.getMaxRange());
				img.setFromMatrix(range2D);

				win3D.get3DSceneAndLock();
					gl_img_range->assignImage_fast(img);
				win3D.unlockAccess3DScene();
			}

			// Show intensity image:
			if (last_obs->hasIntensityImage )
			{
				win3D.get3DSceneAndLock();
					gl_img_intensity->assignImage(last_obs->intensityImage);
				win3D.unlockAccess3DScene();
			}

			// Show 3D points:
			if (last_obs->hasPoints3D )
			{
				//mrpt::slam::CSimplePointsMap  pntsMap;
				CColouredPointsMap pntsMap;
				pntsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
				pntsMap.loadFromRangeScan(*last_obs);

				win3D.get3DSceneAndLock();
					gl_points->loadFromPointsMap(&pntsMap);
				win3D.unlockAccess3DScene();
				win3D.repaint();
			}


			// Estimated grabbing rate:
			win3D.get3DSceneAndLock();
				win3D.addTextMessage(-100,-20, format("%.02f Hz", thrPar.Hz ), TColorf(1,1,1), 100, MRPT_GLUT_BITMAP_HELVETICA_18 );
			win3D.unlockAccess3DScene();

			// Do we have accelerometer data?
			if (last_obs_imu && last_obs_imu->dataIsPresent[IMU_X_ACC])
			{
				;
				win3D.get3DSceneAndLock();
					win3D.addTextMessage(10,60, 
						format("Acc: x=%.02f y=%.02f z=%.02f", last_obs_imu->rawMeasurements[IMU_X_ACC], last_obs_imu->rawMeasurements[IMU_Y_ACC], last_obs_imu->rawMeasurements[IMU_Z_ACC] ), 
						TColorf(0,0,1), 102, MRPT_GLUT_BITMAP_HELVETICA_18 );
				win3D.unlockAccess3DScene();
			}

			win3D.repaint();

		} // end update visualization:


		// Process possible keyboard commands:
		// --------------------------------------
		if (win3D.keyHit() && thrPar.pushed_key==0)
		{
			const int key = tolower( win3D.getPushedKey() );

			switch(key)
			{
				// Some of the keys are processed in this thread:
				case 'o':
					win3D.setCameraZoom( win3D.getCameraZoom() * 1.2 );
					win3D.repaint();
					break;
				case 'i':
					win3D.setCameraZoom( win3D.getCameraZoom() / 1.2 );
					win3D.repaint();
					break;
				// ...and the rest in the kinect thread:
				default:
					thrPar.pushed_key = key;
					break;
			};
		}

		win3D.get3DSceneAndLock();
		win3D.addTextMessage(10,10,
			format("'o'/'i'-zoom out/in, 'w'-tilt up,'s'-tilt down, mouse: orbit 3D, ESC: quit"),
				TColorf(0,0,1), 110, MRPT_GLUT_BITMAP_HELVETICA_18 );
		win3D.addTextMessage(10,35,
			format("Tilt angle: %.01f deg", thrPar.tilt_ang_deg),
				TColorf(0,0,1), 111, MRPT_GLUT_BITMAP_HELVETICA_18 );
		win3D.unlockAccess3DScene();

		mrpt::system::sleep(1);
	}


	cout << "Waiting for grabbing thread to exit...\n";
	thrPar.quit = true;
	mrpt::system::joinThread(thHandle);
	cout << "Bye!\n";
}


int main(int argc, char **argv)
{
	try
	{
		Test_Kinect();

		mrpt::system::sleep(50);
		return 0;

	} catch (std::exception &e)
	{
		std::cout << "EXCEPCION: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}

}
