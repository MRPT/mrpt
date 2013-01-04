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

/*
  Example  : kinect_3d_view
  Web page : http://www.mrpt.org/Kinect_and_MRPT

  Purpose  : Demonstrate grabbing from CKinect, multi-threading
             and live 3D rendering.
*/


#include <mrpt/hwdrivers.h>
#include <mrpt/gui.h>
#include <mrpt/maps.h>
#include <mrpt/system/filesystem.h>

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
		const std::string cfgFile = "kinect_calib.cfg";
		if (mrpt::system::fileExists(cfgFile))
		{
			cout << "Loading calibration from: "<< cfgFile << endl;
			kinect.loadConfig( mrpt::utils::CConfigFile(cfgFile), "KINECT" );
		}
		else cerr << "Warning: Calibration file ["<< cfgFile <<"] not found -> Using default params.\n";

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
					case 'c':
						// Switch video input:
						kinect.setVideoChannel( kinect.getVideoChannel()==CKinect::VIDEO_CHANNEL_RGB ?  CKinect::VIDEO_CHANNEL_IR : CKinect::VIDEO_CHANNEL_RGB);
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

	// Wait until data stream starts so we can say for sure the sensor has been initialized OK:
	cout << "Waiting for sensor initialization...\n";
	do {
		CObservation3DRangeScanPtr possiblyNewObs = thrPar.new_obs.get();
		if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP)
				break;
		else 	mrpt::system::sleep(10);
	} while (!thrPar.quit);

	// Check error condition:
	if (thrPar.quit) return;


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

	opengl::COpenGLViewportPtr viewRange, viewInt; // Extra viewports for the RGB & D images.
	{
		mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

		// Create the Opengl object for the point cloud:
		scene->insert( gl_points );
		scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
		scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

		const int VW_WIDTH = 250;	// Size of the viewport into the window, in pixel units.
		const int VW_HEIGHT = aspect_ratio*VW_WIDTH;
		const int VW_GAP = 30;

		// Create the Opengl objects for the planar images, as textured planes, each in a separate viewport:
		win3D.addTextMessage(30,-25-1*(VW_GAP+VW_HEIGHT),"Range data",TColorf(1,1,1), 1, MRPT_GLUT_BITMAP_HELVETICA_12 );
		viewRange = scene->createViewport("view2d_range");
		viewRange->setViewportPosition(5,-10-1*(VW_GAP+VW_HEIGHT), VW_WIDTH,VW_HEIGHT);

		win3D.addTextMessage(30, -25-2*(VW_GAP+VW_HEIGHT),"Intensity data",TColorf(1,1,1), 2, MRPT_GLUT_BITMAP_HELVETICA_12 );
		viewInt = scene->createViewport("view2d_int");
		viewInt->setViewportPosition(5, -10-2*(VW_GAP+VW_HEIGHT), VW_WIDTH,VW_HEIGHT );

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
			bool do_refresh = false;

			// Show ranges as 2D:
			if (last_obs->hasRangeImage )
			{
				mrpt::utils::CImage  img;

				// Normalize the image
				static CMatrixFloat  range2D;   // Static to save time allocating the matrix in every iteration
				range2D = last_obs->rangeImage * (1.0/ 5.0); //kinect.getMaxRange());

				img.setFromMatrix(range2D);

				win3D.get3DSceneAndLock();
					viewRange->setImageView_fast(img);
				win3D.unlockAccess3DScene();
				do_refresh=true;
			}

			// Show intensity image:
			if (last_obs->hasIntensityImage )
			{
				win3D.get3DSceneAndLock();
					viewInt->setImageView(last_obs->intensityImage); // This is not "_fast" since the intensity image is used below in the coloured point cloud.
				win3D.unlockAccess3DScene();
				do_refresh=true;
			}

			// Show 3D points:
			if (last_obs->hasPoints3D )
			{
				// For alternative ways to generate the 3D point cloud, read:
				// http://www.mrpt.org/Generating_3D_point_clouds_from_RGB_D_observations
				CColouredPointsMap pntsMap;
				pntsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
				pntsMap.loadFromRangeScan(*last_obs);

				win3D.get3DSceneAndLock();
					gl_points->loadFromPointsMap(&pntsMap);
				win3D.unlockAccess3DScene();
				do_refresh=true;
			}


			// Estimated grabbing rate:
			win3D.get3DSceneAndLock();
				win3D.addTextMessage(-100,-20, format("%.02f Hz", thrPar.Hz ), TColorf(1,1,1), 100, MRPT_GLUT_BITMAP_HELVETICA_18 );
			win3D.unlockAccess3DScene();

			// Do we have accelerometer data?
			if (last_obs_imu && last_obs_imu->dataIsPresent[IMU_X_ACC])
			{
				win3D.get3DSceneAndLock();
					win3D.addTextMessage(10,60,
						format("Acc: x=%.02f y=%.02f z=%.02f", last_obs_imu->rawMeasurements[IMU_X_ACC], last_obs_imu->rawMeasurements[IMU_Y_ACC], last_obs_imu->rawMeasurements[IMU_Z_ACC] ),
						TColorf(0,0,1), "mono", 10, mrpt::opengl::FILL, 102);
				win3D.unlockAccess3DScene();
				do_refresh=true;
			}

			// Force opengl repaint:
			if (do_refresh) win3D.repaint();

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
				case '9':
					{
						// Save latest image (intensity or IR) to disk:
						static int cnt = 0;
						if (last_obs->hasIntensityImage )
						{
							const std::string s = mrpt::format("kinect_image_%04i.png",cnt++);
							std::cout << "Writing intensity/IR image to disk: " << s << std::endl;
							if (!last_obs->intensityImage.saveToFile(s))
								std::cerr << "(error writing file!)\n";
						}
					}
					break;
				// ...and the rest in the kinect thread:
				default:
					thrPar.pushed_key = key;
					break;
			};
		}

		win3D.get3DSceneAndLock();
		win3D.addTextMessage(10,10,
			format("'o'/'i'-zoom out/in, 'w'-tilt up,'s'-tilt down, mouse: orbit 3D,'c':Switch RGB/IR,'9':Save image,ESC: quit"),
				TColorf(0,0,1), "mono", 10, mrpt::opengl::FILL, 110);
		win3D.addTextMessage(10,35,
			format("Tilt angle: %.01f deg", thrPar.tilt_ang_deg),
				TColorf(0,0,1), "mono", 10, mrpt::opengl::FILL, 111);
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
