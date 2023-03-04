/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui.h>
#include <mrpt/hwdrivers/COpenNI2Sensor.h>
#include <mrpt/img/TColor.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // This class is in mrpt-maps
#include <mrpt/system/CTicTac.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::hwdrivers;
using namespace mrpt::img;
using namespace std;

const float vert_FOV = 4.0_deg;

// This demo records from an OpenNI2 device, convert observations to 2D scans
// and runs 2d-icp-slam with them.

int main(int argc, char** argv)
{
	try
	{
		if (argc > 2)
		{
			cerr << "Usage: " << argv[0] << " <sensor_id/sensor_serial\n";
			cerr << "Example: " << argv[0] << " 0 \n";
			return 1;
		}

		// const unsigned sensor_id = 0;
		COpenNI2Sensor rgbd_sensor;
		// rgbd_sensor.loadConfig_sensorSpecific(const
		// mrpt::config::CConfigFileBase &configSource,	const std::string
		// &iniSection );

		unsigned sensor_id_or_serial = 0;
		if (argc == 2)
		{
			sensor_id_or_serial = atoi(argv[1]);
			if (sensor_id_or_serial > 10)
				rgbd_sensor.setSerialToOpen(sensor_id_or_serial);
			else
				rgbd_sensor.setSensorIDToOpen(sensor_id_or_serial);
		}

		// Open:
		// cout << "Calling COpenNI2Sensor::initialize()...";
		rgbd_sensor.initialize();

		if (rgbd_sensor.getNumDevices() == 0) return 0;

		cout << "OK " << rgbd_sensor.getNumDevices() << " available devices."
			 << endl;
		cout << "\nUse device " << sensor_id_or_serial << endl << endl;

		// Create window and prepare OpenGL object in the scene:
		// --------------------------------------------------------
		mrpt::gui::CDisplayWindow3D win3D("OpenNI2 3D view", 800, 600);

		win3D.setCameraAzimuthDeg(140);
		win3D.setCameraElevationDeg(20);
		win3D.setCameraZoom(8.0);
		win3D.setFOV(90);
		win3D.setCameraPointingToPoint(2.5, 0, 0);

		mrpt::opengl::CPointCloudColoured::Ptr gl_points =
			mrpt::opengl::CPointCloudColoured::Create();
		gl_points->setPointSize(2.5);

		// The 2D "laser scan" OpenGL object:
		mrpt::opengl::CPlanarLaserScan::Ptr gl_2d_scan =
			mrpt::opengl::CPlanarLaserScan::Create();
		gl_2d_scan->enablePoints(true);
		gl_2d_scan->enableLine(true);
		gl_2d_scan->enableSurface(true);
		gl_2d_scan->setSurfaceColor(0, 0, 1, 0.3);	// RGBA

		opengl::Viewport::Ptr viewInt;	// Extra viewports for the RGB images.
		{
			mrpt::opengl::Scene::Ptr& scene = win3D.get3DSceneAndLock();

			// Create the Opengl object for the point cloud:
			scene->insert(gl_points);
			scene->insert(mrpt::opengl::CGridPlaneXY::Create());
			scene->insert(mrpt::opengl::stock_objects::CornerXYZ());
			scene->insert(gl_2d_scan);

			const double aspect_ratio = 480.0 / 640.0;
			const int VW_WIDTH =
				400;  // Size of the viewport into the window, in pixel units.
			const int VW_HEIGHT = aspect_ratio * VW_WIDTH;

			// Create an extra opengl viewport for the RGB image:
			viewInt = scene->createViewport("view2d_int");
			viewInt->setViewportPosition(5, 30, VW_WIDTH, VW_HEIGHT);
			win3D.addTextMessage(10, 30 + VW_HEIGHT + 10, "Intensity data", 2);

			win3D.addTextMessage(5, 5, "'o'/'i'-zoom out/in, ESC: quit", 110);

			win3D.unlockAccess3DScene();
			win3D.repaint();
		}

		//							Grab frames continuously and show
		//========================================================================================

		bool bObs = false, bError = true;
		mrpt::system::TTimeStamp last_obs_tim = INVALID_TIMESTAMP;

		while (!win3D.keyHit())	 // Push any key to exit // win3D.isOpen()
		{
			//    cout << "Get new observation\n";
			CObservation3DRangeScan::Ptr newObs =
				CObservation3DRangeScan::Create();
			rgbd_sensor.getNextObservation(*newObs, bObs, bError);

			CObservation2DRangeScan::Ptr obs_2d;  // The equivalent 2D scan

			if (bObs && !bError && newObs &&
				newObs->timestamp != INVALID_TIMESTAMP &&
				newObs->timestamp != last_obs_tim)
			{
				// It IS a new observation:
				last_obs_tim = newObs->timestamp;

				// Convert ranges to an equivalent 2D "fake laser" scan:
				if (newObs->hasRangeImage)
				{
					// Convert to scan:
					obs_2d = CObservation2DRangeScan::Create();

					T3DPointsTo2DScanParams p2s;
					p2s.angle_sup = .5f * vert_FOV;
					p2s.angle_inf = .5f * vert_FOV;
					p2s.sensorLabel = "KINECT_2D_SCAN";
					newObs->convertTo2DScan(*obs_2d, p2s);
				}

				// Update visualization ---------------------------------------

				win3D.get3DSceneAndLock();

				// Estimated grabbing rate:
				win3D.addTextMessage(
					-350, -13,
					format(
						"Timestamp: %s",
						mrpt::system::dateTimeLocalToString(last_obs_tim)
							.c_str()),
					100);

				// Show intensity image:
				if (newObs->hasIntensityImage)
				{
					viewInt->setImageView(
						newObs->intensityImage);  // This is not "_fast" since
					// the intensity image may be
					// needed later on.
				}
				win3D.unlockAccess3DScene();

				// -------------------------------------------------------
				//           Create 3D points from RGB+D data
				//
				// There are several methods to do this.
				//  Switch the #if's to select among the options:
				// See also:
				// https://www.mrpt.org/Generating_3D_point_clouds_from_RGB_D_observations
				// -------------------------------------------------------
				if (newObs->hasRangeImage)
				{
					// Pathway: RGB+D --> XYZ+RGB opengl
					win3D.get3DSceneAndLock();
					mrpt::obs::T3DPointsProjectionParams pp;
					pp.takeIntoAccountSensorPoseOnRobot = false;
					newObs->unprojectInto(
						*gl_points, pp /* without obs.sensorPose */);
					win3D.unlockAccess3DScene();
				}

				// And load scan in the OpenGL object:
				gl_2d_scan->setScan(*obs_2d);

				win3D.repaint();
			}  // end update visualization:
		}

		cout << "\nClosing RGBD sensor...\n";

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
