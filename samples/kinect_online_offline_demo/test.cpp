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

/*
  Example  : kinect-online-offline-demo
  Web page : http://www.mrpt.org/Switching_between_reading_live_Kinect_RGBD_dataset_for_debugging

  Purpose  : Demonstrate how to programatically switch between online Kinect
              grabbing and offline parsing a Rawlog dataset. Refer to the launch
              of the grabbing thread in Test_KinectOnlineOffline()
*/


#include <mrpt/hwdrivers.h>
#include <mrpt/gui.h>
#include <mrpt/maps.h>
#include <mrpt/slam/CRawlog.h>
#include <memory> // for std::auto_ptr<>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace std;


// Thread for online/offline capturing: This should be done in another thread
//   specially in the online mode, but also in offline to emulate the possibility
//   of losing frames if our program doesn't process them quickly.
struct TThreadParam
{
	TThreadParam(
		bool _is_online,
		const string &_rawlog_file = string(),
		bool _generate_3D_pointcloud_in_this_thread = false)
		:	is_online(_is_online),
			rawlog_file(_rawlog_file),
			generate_3D_pointcloud_in_this_thread(_generate_3D_pointcloud_in_this_thread),
			quit(false),
			Hz(0)
	{ }

	const bool     is_online;   //!< true: live grabbing from the sensor, false: from rawlog
	const string   rawlog_file; //!< Only when is_online==false
	const bool generate_3D_pointcloud_in_this_thread; //!< true: populate the 3D point fields in the output observation; false: only RGB and Depth images.

	volatile bool   quit;       //!< In/Out variable: Forces the thread to exit or indicates an error in the thread that caused it to end.
	volatile double Hz;         //!< Out variable: Approx. capturing rate from the thread.

	mrpt::synch::CThreadSafeVariable<CObservation3DRangeScanPtr> new_obs;  //!< RGB+D (+ optionally, 3D point cloud)
};

// Only for offline operation:
//  Uncoment to force the simulated sensor to run at this given rate.
//  If commented out, the simulated sensor will reproduce the real rate
//  as indicated by timestamps in the dataset.
//#define FAKE_KINECT_FPS_RATE  20   // In Hz

// ----------------------------------------------------
// The online/offline grabbing thread
// ----------------------------------------------------
void thread_grabbing(TThreadParam &p)
{
	try
	{
		typedef std::auto_ptr<CKinect> CKinectPtr;  // This assures automatic destruction

		// Only one of these will be actually used:
		CKinectPtr          kinect;
		CFileGZInputStream  dataset;

		mrpt::system::TTimeStamp  dataset_prev_tim     = INVALID_TIMESTAMP;
		mrpt::system::TTimeStamp  my_last_read_obs_tim = INVALID_TIMESTAMP;

		if (p.is_online)
		{
			// Online:
			// ---------------------
			kinect = CKinectPtr(new CKinect());

			// Set params:
			kinect->enableGrabRGB(true);
			kinect->enableGrabDepth(true);
			kinect->enableGrabAccelerometers(false);
			kinect->enableGrab3DPoints(p.generate_3D_pointcloud_in_this_thread);

			// Open:
			cout << "Calling CKinect::initialize()...";
			kinect->initialize();
			cout << "OK\n";
		}
		else
		{
			// Offline:
			// ---------------------
			if (!dataset.open(p.rawlog_file))
				throw std::runtime_error("Couldn't open rawlog dataset file for input...");

			// Set external images directory:
			CImage::IMAGES_PATH_BASE = CRawlog::detectImagesDirectory(p.rawlog_file);
		}

		CTicTac tictac;
		int nImgs = 0;

		while (!p.quit)
		{
			if (p.is_online)
			{
				// Grab new observation from the camera:
				bool there_is_obs=true, hard_error=false;

				CObservation3DRangeScanPtr  obs = CObservation3DRangeScan::Create(); // Smart pointers to observations. Memory pooling is used internally
				kinect->getNextObservation(*obs,there_is_obs,hard_error);

				if(hard_error)
					throw std::runtime_error("Sensor returned 'hardware error'");
				else if (there_is_obs)
				{
					// Send object to the main thread:
					p.new_obs.set(obs);
				}
			}
			else
			{
				// Offline:
				CObservationPtr obs;
				do
				{
					try {
						dataset >> obs;
					}
					catch (std::exception &e) {
						throw std::runtime_error( string("Error reading from dataset file:\n")+string(e.what()) );
					}
					ASSERT_(obs.present())
				} while (!IS_CLASS(obs,CObservation3DRangeScan));

				// We have one observation:
				CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);

				// Do we have to wait to emulate real-time behavior?
				const mrpt::system::TTimeStamp cur_tim = obs3D->timestamp;
				const mrpt::system::TTimeStamp now_tim = mrpt::system::now();

				if (dataset_prev_tim!=INVALID_TIMESTAMP && my_last_read_obs_tim!=INVALID_TIMESTAMP)
				{
					const double At_dataset = mrpt::system::timeDifference( dataset_prev_tim, cur_tim );
					const double At_actual  = mrpt::system::timeDifference( my_last_read_obs_tim, now_tim );

					const double need_to_wait_ms = 1000.*( At_dataset-At_actual );
					//cout << "[Kinect grab thread] Need to wait (ms): " << need_to_wait_ms << endl;
					if (need_to_wait_ms>0)
						mrpt::system::sleep( need_to_wait_ms );
				}

				// Send observation to main thread:
				p.new_obs.set(obs3D);

				dataset_prev_tim     = cur_tim;
				my_last_read_obs_tim = now_tim;
			}

			// Update Hz rate estimate
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
//				Test_KinectOnlineOffline
// ------------------------------------------------------
void Test_KinectOnlineOffline(bool is_online, const string &rawlog_file = string())
{
	// Launch grabbing thread:
	// --------------------------------------------------------
	TThreadParam thrPar(
		is_online,
		rawlog_file,
		false // generate_3D_pointcloud_in_this_thread -> Don't, we'll do it in this main thread.
		);

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
	cout << "OK! Sensor started to emit observations.\n";

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

		win3D.addTextMessage(10,10,
			format("'o'/'i'-zoom out/in, ESC: quit"),
				TColorf(0,0,1), 110, MRPT_GLUT_BITMAP_HELVETICA_18 );


		win3D.unlockAccess3DScene();
		win3D.repaint();
	}


	CObservation3DRangeScanPtr  last_obs;

	while (win3D.isOpen() && !thrPar.quit)
	{
		CObservation3DRangeScanPtr possiblyNewObs = thrPar.new_obs.get();
		if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP &&
			(!last_obs  || possiblyNewObs->timestamp!=last_obs->timestamp ) )
		{
			// It IS a new observation:
			last_obs = possiblyNewObs;

			// Update visualization ---------------------------------------

			// Show ranges as 2D:
			win3D.get3DSceneAndLock();

			if (last_obs->hasRangeImage )
			{
				mrpt::utils::CImage  img;

				// Normalize the image
				static CMatrixFloat  range2D;   // Static to save time allocating the matrix in every iteration
				range2D = last_obs->rangeImage * (1.0/ 5.0); //kinect.getMaxRange());

				img.setFromMatrix(range2D);

				viewRange->setImageView_fast(img);
			}

			// Show intensity image:
			if (last_obs->hasIntensityImage )
			{
				viewInt->setImageView(last_obs->intensityImage); // This is not "_fast" since the intensity image is used below in the coloured point cloud.
			}
			win3D.unlockAccess3DScene();

			// Show 3D points:
			if (0 && last_obs->hasPoints3D )
			{
				//mrpt::slam::CSimplePointsMap  pntsMap;
				CColouredPointsMap pntsMap;
				pntsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
				pntsMap.loadFromRangeScan(*last_obs);

				win3D.get3DSceneAndLock();
					gl_points->loadFromPointsMap(&pntsMap);
				win3D.unlockAccess3DScene();
			}

			win3D.repaint();

		} // end update visualization:


		// Process possible keyboard commands:
		// --------------------------------------
		if (win3D.keyHit())
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
				case 27: // ESC
					thrPar.quit = true;
					break;
				default:
					break;
			};
		}

		{
			win3D.get3DSceneAndLock();
				// Estimated grabbing rate:
				win3D.addTextMessage(-100,-20, format("%.02f Hz", thrPar.Hz ), TColorf(1,1,1), 100, MRPT_GLUT_BITMAP_HELVETICA_18 );
			win3D.unlockAccess3DScene();
		}

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
		// Determine online/offline operation:
		if (argc!=1 && argc!=2)
		{
			cerr << "Usage:\n"
			     << argv[0] << "                  --> Online grab from sensor\n"
			     << argv[0] << " [DATASET.rawlog] --> Offline from dataset\n";
			return 1;
		}

		if (argc==1)
		{
			// Online
			cout << "Using online operation" << endl;
			Test_KinectOnlineOffline(true);
		}
		else
		{
			// Offline:
			cout << "Using offline operation with: " << argv[1] << endl;
			Test_KinectOnlineOffline(false, string(argv[1]));
		}

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
