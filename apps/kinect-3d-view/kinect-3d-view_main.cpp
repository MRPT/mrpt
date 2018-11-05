/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*
  Example  : kinect_3d_view
  Web page : http://www.mrpt.org/Kinect_and_MRPT

  Purpose  : Demonstrate grabbing from CKinect, multi-threading
			 and live 3D rendering.
*/

#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CColouredOctoMap.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt::gui;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace std;

//#define VIEW_AS_OCTOMAP

// Thread for grabbing: Do this is another thread so we divide rendering and
// grabbing
//   and exploit multicore CPUs.
struct TThreadParam
{
	TThreadParam() = default;
	volatile bool quit{false};
	volatile int pushed_key{0};
	volatile double tilt_ang_deg{0};
	volatile double Hz{0};

	CObservation3DRangeScan::Ptr new_obs;  // RGB+D (+3D points)
	CObservationIMU::Ptr new_obs_imu;  // Accelerometers
};

void thread_grabbing(TThreadParam& p)
{
	try
	{
		CKinect kinect;

		// Set params:
		// kinect.enableGrab3DPoints(true);
		// kinect.enablePreviewRGB(true);
		//...
		const std::string cfgFile = "kinect_calib.cfg";
		if (mrpt::system::fileExists(cfgFile))
		{
			cout << "Loading calibration from: " << cfgFile << endl;
			kinect.loadConfig(mrpt::config::CConfigFile(cfgFile), "KINECT");
		}
		else
			cerr << "Warning: Calibration file [" << cfgFile
				 << "] not found -> Using default params.\n";

		// Open:
		cout << "Calling CKinect::initialize()...";
		kinect.initialize();
		cout << "OK\n";

		mrpt::system::CTicTac tictac;
		int nImgs = 0;
		bool there_is_obs = true, hard_error = false;

		while (!hard_error && !p.quit)
		{
			// Grab new observation from the camera:
			CObservation3DRangeScan::Ptr obs = mrpt::make_aligned_shared<
				CObservation3DRangeScan>();  // Smart pointers to observations
			CObservationIMU::Ptr obs_imu =
				mrpt::make_aligned_shared<CObservationIMU>();

			kinect.getNextObservation(*obs, *obs_imu, there_is_obs, hard_error);

			if (!hard_error && there_is_obs)
			{
				std::atomic_store(&p.new_obs, obs);
				std::atomic_store(&p.new_obs_imu, obs_imu);
			}

			if (p.pushed_key != 0)
			{
				switch (p.pushed_key)
				{
					case 's':
						p.tilt_ang_deg -= 1;
						if (p.tilt_ang_deg < -31) p.tilt_ang_deg = -31;
						kinect.setTiltAngleDegrees(p.tilt_ang_deg);
						break;
					case 'w':
						p.tilt_ang_deg += 1;
						if (p.tilt_ang_deg > 31) p.tilt_ang_deg = 31;
						kinect.setTiltAngleDegrees(p.tilt_ang_deg);
						break;
					case 'c':
						// Switch video input:
						kinect.setVideoChannel(
							kinect.getVideoChannel() ==
									CKinect::VIDEO_CHANNEL_RGB
								? CKinect::VIDEO_CHANNEL_IR
								: CKinect::VIDEO_CHANNEL_RGB);
						break;
					case 27:
						p.quit = true;
						break;
				}

				// Clear pushed key flag:
				p.pushed_key = 0;
			}

			nImgs++;
			if (nImgs > 10)
			{
				p.Hz = nImgs / tictac.Tac();
				nImgs = 0;
				tictac.Tic();
			}
		}
	}
	catch (const std::exception& e)
	{
		cout << "Exception in Kinect thread: " << mrpt::exception_to_str(e)
			 << endl;
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
	std::thread thHandle(thread_grabbing, std::ref(thrPar));

	// Wait until data stream starts so we can say for sure the sensor has been
	// initialized OK:
	cout << "Waiting for sensor initialization...\n";
	do
	{
		CObservation3DRangeScan::Ptr possiblyNewObs =
			std::atomic_load(&thrPar.new_obs);
		if (possiblyNewObs && possiblyNewObs->timestamp != INVALID_TIMESTAMP)
			break;
		else
			std::this_thread::sleep_for(10ms);
	} while (!thrPar.quit);

	// Check error condition:
	if (thrPar.quit) return;

	// Create window and prepare OpenGL object in the scene:
	// --------------------------------------------------------
	mrpt::gui::CDisplayWindow3D win3D("Kinect 3D view", 800, 600);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(8.0);
	win3D.setFOV(90);
	win3D.setCameraPointingToPoint(2.5, 0, 0);

#if !defined(VIEW_AS_OCTOMAP)
	mrpt::opengl::CPointCloudColoured::Ptr gl_points =
		mrpt::make_aligned_shared<mrpt::opengl::CPointCloudColoured>();
	gl_points->setPointSize(2.5);
#else
	mrpt::opengl::COctoMapVoxels::Ptr gl_voxels =
		mrpt::make_aligned_shared<mrpt::opengl::COctoMapVoxels>();
#endif

	const double aspect_ratio =
		480.0 / 640.0;  // kinect.rows() / double( kinect.cols() );

	opengl::COpenGLViewport::Ptr viewRange,
		viewInt;  // Extra viewports for the RGB & D images.
	{
		mrpt::opengl::COpenGLScene::Ptr& scene = win3D.get3DSceneAndLock();

// Create the Opengl object for the point cloud:
#if !defined(VIEW_AS_OCTOMAP)
		scene->insert(gl_points);
#else
		scene->insert(gl_voxels);
#endif
		scene->insert(mrpt::make_aligned_shared<mrpt::opengl::CGridPlaneXY>());
		scene->insert(mrpt::opengl::stock_objects::CornerXYZ());

		const int VW_WIDTH =
			250;  // Size of the viewport into the window, in pixel units.
		const int VW_HEIGHT = aspect_ratio * VW_WIDTH;
		const int VW_GAP = 30;

		// Create the Opengl objects for the planar images, as textured planes,
		// each in a separate viewport:
		win3D.addTextMessage(
			30, -25 - 1 * (VW_GAP + VW_HEIGHT), "Range data", TColorf(1, 1, 1),
			1, MRPT_GLUT_BITMAP_HELVETICA_12);
		viewRange = scene->createViewport("view2d_range");
		viewRange->setViewportPosition(
			5, -10 - 1 * (VW_GAP + VW_HEIGHT), VW_WIDTH, VW_HEIGHT);

		win3D.addTextMessage(
			30, -25 - 2 * (VW_GAP + VW_HEIGHT), "Intensity data",
			TColorf(1, 1, 1), 2, MRPT_GLUT_BITMAP_HELVETICA_12);
		viewInt = scene->createViewport("view2d_int");
		viewInt->setViewportPosition(
			5, -10 - 2 * (VW_GAP + VW_HEIGHT), VW_WIDTH, VW_HEIGHT);

		win3D.unlockAccess3DScene();
		win3D.repaint();
	}

	CObservation3DRangeScan::Ptr last_obs;
	CObservationIMU::Ptr last_obs_imu;

	while (win3D.isOpen() && !thrPar.quit)
	{
		CObservation3DRangeScan::Ptr possiblyNewObs =
			std::atomic_load(&thrPar.new_obs);
		if (possiblyNewObs && possiblyNewObs->timestamp != INVALID_TIMESTAMP &&
			(!last_obs || possiblyNewObs->timestamp != last_obs->timestamp))
		{
			// It IS a new observation:
			last_obs = possiblyNewObs;
			last_obs_imu = std::atomic_load(&thrPar.new_obs_imu);

			// Update visualization ---------------------------------------
			bool do_refresh = false;

			// Show ranges as 2D:
			if (last_obs->hasRangeImage)
			{
				mrpt::img::CImage img;

				// Normalize the image
				static CMatrixFloat range2D;  // Static to save time allocating
				// the matrix in every iteration
				range2D = last_obs->rangeImage *
						  (1.0f / 5.0f);  // kinect.getMaxRange());

				img.setFromMatrix(range2D);

				win3D.get3DSceneAndLock();
				viewRange->setImageView_fast(img);
				win3D.unlockAccess3DScene();
				do_refresh = true;
			}

			// Show intensity image:
			if (last_obs->hasIntensityImage)
			{
				win3D.get3DSceneAndLock();
				viewInt->setImageView(
					last_obs->intensityImage);  // This is not "_fast" since the
				// intensity image is used below
				// in the coloured point cloud.
				win3D.unlockAccess3DScene();
				do_refresh = true;
			}

			// Show 3D points:
			if (last_obs->hasPoints3D)
			{
#if !defined(VIEW_AS_OCTOMAP)
				// For alternative ways to generate the 3D point cloud, read:
				// http://www.mrpt.org/tutorials/programming/miscellaneous/generating_3d_point_clouds_from_rgb_d_observations/
				win3D.get3DSceneAndLock();
				mrpt::obs::T3DPointsProjectionParams pp;
				pp.takeIntoAccountSensorPoseOnRobot = false;
				last_obs->project3DPointsFromDepthImageInto(*gl_points, pp);
				win3D.unlockAccess3DScene();
#else
				mrpt::maps::CColouredOctoMap octoMap(0.10);
				octoMap.setVoxelColourMethod(
					mrpt::maps::CColouredOctoMap::INTEGRATE);
				octoMap.insertObservationPtr(last_obs);

				win3D.get3DSceneAndLock();
				octoMap.getAsOctoMapVoxels(*gl_voxels);
				gl_voxels->showVoxels(VOXEL_SET_FREESPACE, false);
				win3D.unlockAccess3DScene();
#endif
				do_refresh = true;
			}

			// Estimated grabbing rate:
			win3D.get3DSceneAndLock();
			win3D.addTextMessage(
				-100, -20, format("%.02f Hz", thrPar.Hz), TColorf(1, 1, 1), 100,
				MRPT_GLUT_BITMAP_HELVETICA_18);
			win3D.unlockAccess3DScene();

			// Do we have accelerometer data?
			if (last_obs_imu && last_obs_imu->dataIsPresent[IMU_X_ACC])
			{
				win3D.get3DSceneAndLock();
				win3D.addTextMessage(
					10, 60,
					format(
						"Acc: x=%.02f y=%.02f z=%.02f",
						last_obs_imu->rawMeasurements[IMU_X_ACC],
						last_obs_imu->rawMeasurements[IMU_Y_ACC],
						last_obs_imu->rawMeasurements[IMU_Z_ACC]),
					TColorf(0, 0, 1), "mono", 10, mrpt::opengl::FILL, 102);
				win3D.unlockAccess3DScene();
				do_refresh = true;
			}

			// Force opengl repaint:
			if (do_refresh) win3D.repaint();

		}  // end update visualization:

		// Process possible keyboard commands:
		// --------------------------------------
		if (win3D.keyHit() && thrPar.pushed_key == 0)
		{
			const int key = tolower(win3D.getPushedKey());

			switch (key)
			{
				// Some of the keys are processed in this thread:
				case 'o':
					win3D.setCameraZoom(win3D.getCameraZoom() * 1.2);
					win3D.repaint();
					break;
				case 'i':
					win3D.setCameraZoom(win3D.getCameraZoom() / 1.2);
					win3D.repaint();
					break;
				case '9':
				{
					// Save latest image (intensity or IR) to disk:
					static int cnt = 0;
					if (last_obs->hasIntensityImage)
					{
						const std::string s =
							mrpt::format("kinect_image_%04i.png", cnt++);
						std::cout << "Writing intensity/IR image to disk: " << s
								  << std::endl;
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
		win3D.addTextMessage(
			10, 10,
			format("'o'/'i'-zoom out/in, 'w'-tilt up,'s'-tilt down, mouse: "
				   "orbit 3D,'c':Switch RGB/IR,'9':Save image,ESC: quit"),
			TColorf(0, 0, 1), "mono", 10, mrpt::opengl::FILL, 110);
		win3D.addTextMessage(
			10, 35, format("Tilt angle: %.01f deg", thrPar.tilt_ang_deg),
			TColorf(0, 0, 1), "mono", 10, mrpt::opengl::FILL, 111);
		win3D.unlockAccess3DScene();

		std::this_thread::sleep_for(1ms);
	}

	cout << "Waiting for grabbing thread to exit...\n";
	thrPar.quit = true;
	thHandle.join();
	cout << "Bye!\n";
}

int main()
{
	try
	{
		Test_Kinect();

		std::this_thread::sleep_for(50ms);
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << "EXCEPCION: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
