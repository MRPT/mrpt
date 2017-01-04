/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*
  Example  : kinect-online-offline-demo
  Web page : http://www.mrpt.org/Switching_between_reading_live_Kinect_RGBD_dataset_for_debugging

  Purpose  : Demonstrate how to programatically switch between online Kinect
              grabbing and offline parsing a Rawlog dataset. Refer to the launch
              of the grabbing thread in Test_KinectOnlineOffline()
*/


#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/gui.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/synch/CThreadSafeVariable.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <memory> // for std::auto_ptr, unique_ptr

// Demonstrate MRPT RGB+D --> PCL point cloud conversion:
#if MRPT_HAS_PCL
#	include <mrpt/maps/PCL_adapters.h>
#endif

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace mrpt::obs;
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
//#define FAKE_KINECT_FPS_RATE  10   // In Hz

// ----------------------------------------------------
// The online/offline grabbing thread
// ----------------------------------------------------
void thread_grabbing(TThreadParam &p)
{
	try
	{
#if MRPT_HAS_CXX11
		typedef std::unique_ptr<CKinect> CKinectPtr;  // This assures automatic destruction
#else
		typedef std::auto_ptr<CKinect> CKinectPtr;  // This assures automatic destruction
#endif

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
						throw std::runtime_error( string("\nError reading from dataset file (EOF?):\n")+string(e.what()) );
					}
					ASSERT_(obs.present())
				} while (!IS_CLASS(obs,CObservation3DRangeScan));

				// We have one observation:
				CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
				obs3D->load(); // *Important* This is needed to load the range image if stored as a separate file.

				// Do we have to wait to emulate real-time behavior?
				const mrpt::system::TTimeStamp cur_tim = obs3D->timestamp;
				const mrpt::system::TTimeStamp now_tim = mrpt::system::now();

				if (dataset_prev_tim!=INVALID_TIMESTAMP && my_last_read_obs_tim!=INVALID_TIMESTAMP)
				{
#ifndef FAKE_KINECT_FPS_RATE
					const double At_dataset = mrpt::system::timeDifference( dataset_prev_tim, cur_tim );
#else
					const double At_dataset = 1.0/FAKE_KINECT_FPS_RATE;
#endif
					const double At_actual  = mrpt::system::timeDifference( my_last_read_obs_tim, now_tim );

					const double need_to_wait_ms = 1000.*( At_dataset-At_actual );
					//cout << "[Kinect grab thread] Need to wait (ms): " << need_to_wait_ms << endl;
					if (need_to_wait_ms>0)
						mrpt::system::sleep( need_to_wait_ms );
				}

				// Send observation to main thread:
				p.new_obs.set(obs3D);

				dataset_prev_tim     = cur_tim;
				my_last_read_obs_tim = mrpt::system::now();
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
		CObservation3DRangeScanPtr newObs = thrPar.new_obs.get();
		if (newObs && newObs->timestamp!=INVALID_TIMESTAMP)
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

	opengl::COpenGLViewportPtr viewInt; // Extra viewports for the RGB images.
	{
		mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

		// Create the Opengl object for the point cloud:
		scene->insert( gl_points );
		scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
		scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

		const double aspect_ratio =  480.0 / 640.0;
		const int VW_WIDTH = 400;	// Size of the viewport into the window, in pixel units.
		const int VW_HEIGHT = aspect_ratio*VW_WIDTH;

		// Create an extra opengl viewport for the RGB image:
		viewInt = scene->createViewport("view2d_int");
		viewInt->setViewportPosition(5, 30, VW_WIDTH,VW_HEIGHT );
		win3D.addTextMessage(10, 30+VW_HEIGHT+10,"Intensity data",TColorf(1,1,1), 2, mrpt::opengl::MRPT_GLUT_BITMAP_HELVETICA_12 );

		win3D.addTextMessage(5,5,
			format("'o'/'i'-zoom out/in, ESC: quit"),
				TColorf(0,0,1), 110, mrpt::opengl::MRPT_GLUT_BITMAP_HELVETICA_18 );


		win3D.unlockAccess3DScene();
		win3D.repaint();
	}



	mrpt::system::TTimeStamp  last_obs_tim = INVALID_TIMESTAMP;

	while (win3D.isOpen() && !thrPar.quit)
	{
		CObservation3DRangeScanPtr newObs = thrPar.new_obs.get();
		if (newObs && newObs->timestamp!=INVALID_TIMESTAMP &&
			newObs->timestamp!=last_obs_tim )
		{
			// It IS a new observation:
			last_obs_tim = newObs->timestamp;

			// Update visualization ---------------------------------------

			win3D.get3DSceneAndLock();

			// Estimated grabbing rate:
			win3D.addTextMessage(-350,-13, format("Timestamp: %s", mrpt::system::dateTimeLocalToString(last_obs_tim).c_str()), TColorf(0.6,0.6,0.6),"mono",10,mrpt::opengl::FILL, 100);
			win3D.addTextMessage(-100,-30, format("%.02f Hz", thrPar.Hz ), TColorf(1,1,1),"mono",10,mrpt::opengl::FILL, 101);

			// Show intensity image:
			if (newObs->hasIntensityImage )
			{
				viewInt->setImageView(newObs->intensityImage); // This is not "_fast" since the intensity image may be needed later on.
			}
			win3D.unlockAccess3DScene();

			// -------------------------------------------------------
			//           Create 3D points from RGB+D data
			//
			// There are several methods to do this.
			//  Switch the #if's to select among the options:
			// See also: http://www.mrpt.org/Generating_3D_point_clouds_from_RGB_D_observations
			// -------------------------------------------------------
			if (newObs->hasRangeImage)
			{
				static mrpt::utils::CTimeLogger logger;
				logger.enter("RGBD->3D");

// Pathway: RGB+D --> PCL <PointXYZ> --> XYZ opengl
#if 0
				static pcl::PointCloud<pcl::PointXYZ> cloud;
				logger.enter("RGBD->3D.projectInto");
				newObs->project3DPointsFromDepthImageInto(cloud, false /* without obs.sensorPose */);
				logger.leave("RGBD->3D.projectInto");

				win3D.get3DSceneAndLock();
				logger.enter("RGBD->3D.load in OpenGL");
					gl_points->loadFromPointsMap(&cloud);
				logger.leave("RGBD->3D.load in OpenGL");
				win3D.unlockAccess3DScene();
#endif

// Pathway: RGB+D --> PCL <PointXYZRGB> --> XYZ+RGB opengl
#if 0
				static pcl::PointCloud<pcl::PointXYZRGB> cloud;
				logger.enter("RGBD->3D.projectInto");
				newObs->project3DPointsFromDepthImageInto(cloud, false /* without obs.sensorPose */);
				logger.leave("RGBD->3D.projectInto");

				win3D.get3DSceneAndLock();
				logger.enter("RGBD->3D.load in OpenGL");
					gl_points->loadFromPointsMap(&cloud);
				logger.leave("RGBD->3D.load in OpenGL");
				win3D.unlockAccess3DScene();
#endif

// Pathway: RGB+D --> XYZ+RGB opengl
#if 1
				win3D.get3DSceneAndLock();
				logger.enter("RGBD->3D.projectInto");
				mrpt::obs::T3DPointsProjectionParams pp;
				pp.takeIntoAccountSensorPoseOnRobot = false;

				newObs->project3DPointsFromDepthImageInto(*gl_points, pp);

				logger.leave("RGBD->3D.projectInto");
				win3D.unlockAccess3DScene();
#endif

// Pathway: RGB+D --> XYZ+RGB opengl (With a 6D global pose of the robot)
#if 0
				const CPose3D globalPose(1,2,3,DEG2RAD(10),DEG2RAD(20),DEG2RAD(30));
				win3D.get3DSceneAndLock();
				logger.enter("RGBD->3D.projectInto");
					newObs->project3DPointsFromDepthImageInto(*gl_points, false /* without obs.sensorPose */, &globalPose);
				logger.leave("RGBD->3D.projectInto");
				win3D.unlockAccess3DScene();
#endif

// Pathway: RGB+D --> internal local XYZ pointcloud --> XYZ+RGB point cloud map --> XYZ+RGB opengl
#if 0
				// Project 3D points:
				if (!newObs->hasPoints3D)
				{
				logger.enter("RGBD->3D.projectInto");
					newObs->project3DPointsFromDepthImage();
				logger.leave("RGBD->3D.projectInto");
				}

				CColouredPointsMap pntsMap;
				pntsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
				pntsMap.loadFromRangeScan(*newObs);

				win3D.get3DSceneAndLock();
				logger.enter("RGBD->3D.load in OpenGL");
					gl_points->loadFromPointsMap(&pntsMap);
				logger.leave("RGBD->3D.load in OpenGL");
				win3D.unlockAccess3DScene();
#endif

				logger.leave("RGBD->3D");
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
