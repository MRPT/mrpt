/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*
  Application: velodyne-view
  Web page   : https://www.mrpt.org/list-of-mrpt-apps/application-velodyne-view/

  Purpose  : Demonstrate grabbing from a live Velodyne scanner or a PCAP file,
			 multi-threading and live 3D rendering.
*/

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/hwdrivers/CVelodyneScanner.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>  // MRPT_getVersion()

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::io;
using namespace mrpt::img;
using namespace mrpt::config;
using namespace mrpt::system;
using namespace mrpt::serialization;
using namespace mrpt::opengl;
using namespace std;

// Declare the supported options.
TCLAP::CmdLine cmd(
	"velodyne-view", ' ', mrpt::system::MRPT_getVersion().c_str());

TCLAP::ValueArg<std::string> arg_out_rawlog(
	"o", "out-rawlog", "If set, grab dataset in rawlog format", false, "",
	"out.rawlog", cmd);
TCLAP::ValueArg<std::string> arg_in_pcap(
	"i", "in-pcap",
	"Instead of listening to a live sensor, read data from a PCAP file", false,
	"", "in_dataset.pcap", cmd);
TCLAP::ValueArg<std::string> arg_out_pcap(
	"", "out-pcap",
	"If set, grab all packets to a PCAP log file. Set name prefix only.", false,
	"", "out", cmd);
TCLAP::ValueArg<std::string> arg_ip_filter(
	"", "ip-filter", "Only listen to a LIDAR emitting commands from a given IP",
	false, "", "192.168.1.201", cmd);
TCLAP::ValueArg<std::string> arg_calib_file(
	"c", "calib", "Optionally, select the calibration XML file for the LIDAR",
	false, "", "calib.xml", cmd);
TCLAP::ValueArg<std::string> arg_model(
	"m", "model",
	"If no calibration file is specified, set the model to load default values",
	false, "VLP16",
	CVelodyneScanner::TModelPropertiesFactory::getListKnownModels(), cmd);
TCLAP::SwitchArg arg_nologo(
	"n", "nologo", "Skip the logo at startup", cmd, false);
TCLAP::SwitchArg arg_verbose(
	"v", "verbose", "Verbose debug output", cmd, false);

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

	CObservationVelodyneScan::Ptr new_obs;  // Raw scans
	CObservationGPS::Ptr new_obs_gps;  // GPS, if any
};

void thread_grabbing(TThreadParam& p)
{
	try
	{
		CFileGZOutputStream f_out_rawlog;
		if (arg_out_rawlog.isSet())
		{
			if (!f_out_rawlog.open(arg_out_rawlog.getValue()))
				THROW_EXCEPTION_FMT(
					"Error creating output rawlog file: %s",
					arg_out_rawlog.getValue().c_str());
		}
		auto arch = mrpt::serialization::archiveFrom(f_out_rawlog);

		mrpt::hwdrivers::CVelodyneScanner velodyne;

		if (arg_verbose.isSet()) velodyne.enableVerbose(true);

		// Set params:
		velodyne.setModelName(mrpt::typemeta::TEnumType<
							  mrpt::hwdrivers::CVelodyneScanner::model_t>::
								  name2value(arg_model.getValue()));
		if (arg_ip_filter.isSet())
			velodyne.setDeviceIP(
				arg_ip_filter.getValue());  // Default: from any IP
		if (arg_in_pcap.isSet())
			velodyne.setPCAPInputFile(arg_in_pcap.getValue());
		if (arg_out_pcap.isSet())
			velodyne.setPCAPOutputFile(arg_out_pcap.getValue());

		// If you have a calibration file, better than default values:
		if (arg_calib_file.isSet())
		{
			mrpt::obs::VelodyneCalibration calib;
			if (!calib.loadFromXMLFile(arg_calib_file.getValue()))
				throw std::runtime_error(
					"Aborting: error loading calibration file.");
			velodyne.setCalibration(calib);
		}

		// Open:
		cout << "Calling CVelodyneScanner::initialize()...";
		velodyne.initialize();
		cout << "OK\n";

		cout << "Waiting for first data packets (Press CTRL+C to abort)...\n";

		CTicTac tictac;
		int nScans = 0;
		bool hard_error = false;

		while (!hard_error && !p.quit)
		{
			// Grab new observations from the camera:
			CObservationVelodyneScan::Ptr
				obs;  // (initially empty) Smart pointers to observations
			CObservationGPS::Ptr obs_gps;

			hard_error = !velodyne.getNextObservation(obs, obs_gps);

			// Save to log file:
			if (f_out_rawlog.fileOpenCorrectly())
			{
				if (obs) arch << *obs;
				if (obs_gps) arch << *obs_gps;
			}

			if (obs)
			{
				std::atomic_store(&p.new_obs, obs);
				nScans++;
			}
			if (obs_gps) std::atomic_store(&p.new_obs_gps, obs_gps);

			if (p.pushed_key != 0)
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

			if (nScans > 5)
			{
				p.Hz = nScans / tictac.Tac();
				nScans = 0;
				tictac.Tic();
			}
		}
	}
	catch (const std::exception& e)
	{
		cout << "Exception in Velodyne thread: " << mrpt::exception_to_str(e)
			 << endl;
		p.quit = true;
	}
}

// ------------------------------------------------------
//				VelodyneView main entry point
// ------------------------------------------------------
int VelodyneView(int argc, char** argv)
{
	// Parse arguments:
	if (!cmd.parse(argc, argv)) return 1;  // should exit.

	if (!arg_nologo.isSet())
	{
		printf(" velodyne-view - Part of the MRPT\n");
		printf(
			" MRPT C++ Library: %s - Sources timestamp: %s\n",
			mrpt::system::MRPT_getVersion().c_str(),
			mrpt::system::MRPT_getCompilationDate().c_str());
	}

	// Launch grabbing thread:
	// --------------------------------------------------------
	TThreadParam thrPar;
	std::thread thHandle(thread_grabbing, std::ref(thrPar));

	// Wait until data stream starts so we can say for sure the sensor has been
	// initialized OK:
	cout << "Waiting for sensor initialization...\n";
	do
	{
		CObservation::Ptr possiblyNewObs = std::atomic_load(&thrPar.new_obs);
		if (possiblyNewObs && possiblyNewObs->timestamp != INVALID_TIMESTAMP)
			break;
		else
			std::this_thread::sleep_for(10ms);
	} while (!thrPar.quit);

	// Check error condition:
	if (thrPar.quit) return 0;

	// Create window and prepare OpenGL object in the scene:
	// --------------------------------------------------------
	mrpt::gui::CDisplayWindow3D win3D("Velodyne 3D view", 800, 600);

	// Allow rendering large number of points without decimation:
	mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL(1);
	mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE(1e7);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(8.0);
	win3D.setFOV(90);
	win3D.setCameraPointingToPoint(0, 0, 0);
	mrpt::opengl::CPointCloudColoured::Ptr gl_points =
		mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(2.5);

	{
		mrpt::opengl::COpenGLScene::Ptr& scene = win3D.get3DSceneAndLock();

		// Create the Opengl object for the point cloud:
		scene->insert(gl_points);
		scene->insert(mrpt::opengl::CGridPlaneXY::Create());
		scene->insert(mrpt::opengl::stock_objects::CornerXYZ());

		win3D.unlockAccess3DScene();
		win3D.repaint();
	}

	CObservationVelodyneScan::Ptr last_obs;
	CObservationGPS::Ptr last_obs_gps;
	bool view_freeze = false;  // for pausing the view
	CObservationVelodyneScan::TGeneratePointCloudParameters pc_params;

	while (win3D.isOpen() && !thrPar.quit)
	{
		bool do_view_refresh = false;

		CObservationVelodyneScan::Ptr possiblyNewObs =
			std::atomic_load(&thrPar.new_obs);
		CObservationGPS::Ptr possiblyNewObsGps =
			std::atomic_load(&thrPar.new_obs_gps);

		if (possiblyNewObsGps &&
			possiblyNewObsGps->timestamp != INVALID_TIMESTAMP &&
			(!last_obs_gps ||
			 possiblyNewObsGps->timestamp != last_obs_gps->timestamp))
		{
			// It IS a new observation:
			last_obs_gps = std::atomic_load(&thrPar.new_obs_gps);

			std::string rmc_datum;
			if (last_obs_gps->has_RMC_datum())
			{
				rmc_datum = mrpt::format(
					"Lon=%.09f deg  Lat=%.09f deg  Valid?: '%c'\n",
					last_obs_gps->getMsgByClass<gnss::Message_NMEA_RMC>()
						.fields.longitude_degrees,
					last_obs_gps->getMsgByClass<gnss::Message_NMEA_RMC>()
						.fields.latitude_degrees,
					last_obs_gps->getMsgByClass<gnss::Message_NMEA_RMC>()
						.fields.validity_char);
			}
			else
				rmc_datum = "NO";

			win3D.get3DSceneAndLock();
			win3D.addTextMessage(
				5, 40,
				format(
					"POS. frame rx at %s, RMC=%s",
					mrpt::system::dateTimeLocalToString(last_obs_gps->timestamp)
						.c_str(),
					rmc_datum.c_str()),
				102);
			win3D.unlockAccess3DScene();
			do_view_refresh = true;
		}

		if (possiblyNewObs && possiblyNewObs->timestamp != INVALID_TIMESTAMP &&
			(!last_obs || possiblyNewObs->timestamp != last_obs->timestamp))
		{
			// It IS a new observation:
			last_obs = possiblyNewObs;

			if (!last_obs->scan_packets.empty())
			{
				win3D.get3DSceneAndLock();
				win3D.addTextMessage(
					5, 55,
					format(
						"LIDAR scan rx at %s with %u packets",
						mrpt::system::dateTimeLocalToString(last_obs->timestamp)
							.c_str(),
						static_cast<unsigned int>(
							last_obs->scan_packets.size())),
					103);
				win3D.unlockAccess3DScene();
				do_view_refresh = true;
			}

			// Update visualization ---------------------------------------

			// Show 3D points:
			if (!view_freeze)
			{
				last_obs->generatePointCloud(pc_params);

				CColouredPointsMap pntsMap;
				pntsMap.loadFromVelodyneScan(*last_obs);

				win3D.get3DSceneAndLock();
				gl_points->loadFromPointsMap(&pntsMap);
				win3D.unlockAccess3DScene();
			}

			// Estimated grabbing rate:
			win3D.get3DSceneAndLock();
			win3D.addTextMessage(-150, -20, format("%.02f Hz", thrPar.Hz), 100);
			win3D.unlockAccess3DScene();
			do_view_refresh = true;
		}  // end update visualization:

		// Force opengl repaint:
		if (do_view_refresh) win3D.repaint();

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
				case ' ':
					view_freeze = !view_freeze;
					break;
				case '1':
					pc_params.dualKeepLast = !pc_params.dualKeepLast;
					break;
				case '2':
					pc_params.dualKeepStrongest = !pc_params.dualKeepStrongest;
					break;
				// ...and the rest in the sensor thread:
				default:
					thrPar.pushed_key = key;
					break;
			};
		}

		win3D.get3DSceneAndLock();
		win3D.addTextMessage(
			5, 10,
			"'o'/'i'-zoom out/in, mouse: orbit 3D, spacebar: freeze, ESC: quit",
			110);
		win3D.addTextMessage(
			5, 25,
			mrpt::format(
				"'1'/'2': Toggle view dual last (%s)/strongest(%s) returns.",
				pc_params.dualKeepLast ? "ON" : "OFF",
				pc_params.dualKeepStrongest ? "ON" : "OFF"),
			111);
		win3D.unlockAccess3DScene();

		std::this_thread::sleep_for(50ms);
	}

	cout << "Waiting for grabbing thread to exit...\n";
	thrPar.quit = true;
	if (thHandle.joinable()) thHandle.join();
	cout << "Bye!\n";
	return 0;
}

int main(int argc, char** argv)
{
	try
	{
		int ret = VelodyneView(argc, argv);
		std::this_thread::sleep_for(
			50ms);  // to allow GUI threads to end gracefully.
		return ret;
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
