/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*
  Example app : velodyne-view
  Web page : http://www.mrpt.org/list-of-mrpt-apps/application-velodyne-view/

  Purpose  : Demonstrate grabbing from a live Velodyne scanner, multi-threading
             and live 3D rendering.
*/

#include <mrpt/hwdrivers/CVelodyneScanner.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/synch/CThreadSafeVariable.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::utils;
using namespace mrpt::opengl;
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

	mrpt::synch::CThreadSafeVariable<CObservationVelodyneScanPtr> new_obs;     // Raw scans
	mrpt::synch::CThreadSafeVariable<CObservationGPSPtr>          new_obs_gps; // GPS, if any
};

void thread_grabbing(TThreadParam &p)
{
	try
	{
		mrpt::hwdrivers::CVelodyneScanner velodyne;

		// Set params:
		//velodyne.setDeviceIP("192.168.1.201"); // Default: from any IP
		velodyne.setPCAPInputFile("d:/Rawlogs/ual-datasets/Velodyne/2015-12-18_dataset_velodyne_despacho.pcap");

		// Enable this block if you have a calibration file, better than default values:
#if 0	
		mrpt::obs::VelodyneCalibration calib;
		calib.loadFromXMLFile("my_sensor_calib_file.xml");
		velodyne.setCalibration(calib);
#endif

		// Open:
		cout << "Calling CVelodyneScanner::initialize()...";
		velodyne.initialize();
		cout << "OK\n";

		cout << "Waiting for first data packets (Press CTRL+C to abort)...\n";

		CTicTac tictac;
		int nScans = 0;
		bool hard_error=false;

		while (!hard_error && !p.quit)
		{
			// Grab new observations from the camera:
			CObservationVelodyneScanPtr  obs; // (initially empty) Smart pointers to observations
			CObservationGPSPtr           obs_gps;

			hard_error = !velodyne.getNextObservation(obs,obs_gps);

			if (obs)      {p.new_obs.set(obs); nScans++;}
			if (obs_gps)  p.new_obs_gps.set(obs_gps);

			if (p.pushed_key!=0)
			{
				switch (p.pushed_key)
				{
					case 27:
						p.quit = true;
						break;
				}

				// Clear pushed key flag:
				p.pushed_key = 0;
			}

			if (nScans>5)
			{
				p.Hz = nScans / tictac.Tac();
				nScans=0;
				tictac.Tic();
			}
		}
	}
	catch(std::exception &e)
	{
		cout << "Exception in Velodyne thread: " << e.what() << endl;
		p.quit = true;
	}
}

// ------------------------------------------------------
//				Test_Velodyne
// ------------------------------------------------------
void Test_Velodyne()
{
	// Launch grabbing thread:
	// --------------------------------------------------------
	TThreadParam thrPar;
	mrpt::system::TThreadHandle thHandle= mrpt::system::createThreadRef(thread_grabbing ,thrPar);

	// Wait until data stream starts so we can say for sure the sensor has been initialized OK:
	cout << "Waiting for sensor initialization...\n";
	do {
		CObservationPtr possiblyNewObs = thrPar.new_obs.get();
		if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP)
				break;
		else 	mrpt::system::sleep(10);
	} while (!thrPar.quit);

	// Check error condition:
	if (thrPar.quit) return;

	// Create window and prepare OpenGL object in the scene:
	// --------------------------------------------------------
	mrpt::gui::CDisplayWindow3D  win3D("Velodyne 3D view",800,600);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(8.0);
	win3D.setFOV(90);
	win3D.setCameraPointingToPoint(0,0,0);
	mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(2.5);

	{
		mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

		// Create the Opengl object for the point cloud:
		scene->insert( gl_points );
		scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
		scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

		win3D.unlockAccess3DScene();
		win3D.repaint();
	}

	CObservationVelodyneScanPtr  last_obs;
	CObservationGPSPtr           last_obs_gps;

	while (win3D.isOpen() && !thrPar.quit)
	{
		bool do_view_refresh = false;

		CObservationVelodyneScanPtr possiblyNewObs    = thrPar.new_obs.get();
		CObservationGPSPtr          possiblyNewObsGps = thrPar.new_obs_gps.get();

		if (possiblyNewObsGps && possiblyNewObsGps->timestamp!=INVALID_TIMESTAMP &&
			(!last_obs_gps  || possiblyNewObsGps->timestamp!=last_obs_gps->timestamp ) )
		{
			// It IS a new observation:
			last_obs_gps = thrPar.new_obs_gps.get();

			std::string rmc_datum;
			if (last_obs_gps->has_RMC_datum)
			     rmc_datum = mrpt::format("Lon=%.09f deg  Lat=%.09f deg  Valid?: '%c'\n",
				last_obs_gps->RMC_datum.longitude_degrees,
				last_obs_gps->RMC_datum.latitude_degrees,
				last_obs_gps->RMC_datum.validity_char);
			else rmc_datum = "NO";

			win3D.get3DSceneAndLock();
			win3D.addTextMessage(5,25,
				format("POS frame rx at %s, RMC=%s",mrpt::system::dateTimeLocalToString(last_obs_gps->timestamp).c_str(),rmc_datum.c_str()),
				TColorf(1,1,1),"mono",10.0, mrpt::opengl::NICE, 101 );
			win3D.unlockAccess3DScene();
			do_view_refresh=true;
		}

		if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP &&
			(!last_obs  || possiblyNewObs->timestamp!=last_obs->timestamp ) )
		{
			// It IS a new observation:
			last_obs     = possiblyNewObs;

			if (!last_obs->scan_packets.empty())
				printf("[%.03f] RX LIDAR scan with %u packets.\n",  mrpt::system::timestampToDouble(last_obs->timestamp) , static_cast<unsigned int>(last_obs->scan_packets.size()));

			// Update visualization ---------------------------------------

			// Show 3D points:
			{
				CObservationVelodyneScan::TGeneratePointCloudParameters pc_params;
				last_obs->generatePointCloud(pc_params);

				CColouredPointsMap pntsMap;
				pntsMap.loadFromVelodyneScan(*last_obs);

				win3D.get3DSceneAndLock();
					gl_points->loadFromPointsMap(&pntsMap);
				win3D.unlockAccess3DScene();
			}

			// Estimated grabbing rate:
			win3D.get3DSceneAndLock();
				win3D.addTextMessage(-150,-20, format("%.02f Hz", thrPar.Hz ), TColorf(1,1,1), 100, MRPT_GLUT_BITMAP_HELVETICA_18 );
			win3D.unlockAccess3DScene();
			do_view_refresh=true;
		} // end update visualization:

		// Force opengl repaint:
		if (do_view_refresh) win3D.repaint();

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
				// ...and the rest in the sensor thread:
				default:
					thrPar.pushed_key = key;
					break;
			};
		}

		win3D.get3DSceneAndLock();
		win3D.addTextMessage(5,10,"'o'/'i'-zoom out/in, mouse: orbit 3D,ESC: quit", TColorf(1,1,1),"mono",10.0, mrpt::opengl::NICE, 110 );
		win3D.unlockAccess3DScene();

		mrpt::system::sleep(50);
	}

	cout << "Waiting for grabbing thread to exit...\n";
	thrPar.quit = true;
	mrpt::system::joinThread(thHandle);
	cout << "Bye!\n";
}

int main()
{
	try
	{
		Test_Velodyne();
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
