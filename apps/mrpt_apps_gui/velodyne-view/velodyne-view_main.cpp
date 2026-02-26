/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

/*
  Application: velodyne-view
  Web page   : https://www.mrpt.org/list-of-mrpt-apps/application-velodyne-view/

  Purpose  : Demonstrate grabbing from a live Velodyne scanner or a PCAP file,
       multi-threading and live 3D rendering.
*/

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/hwdrivers/CVelodyneScanner.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>  // MRPT_getVersion()
#include <mrpt/viz/CGridPlaneXY.h>

#include <CLI/CLI.hpp>

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
  CObservationGPS::Ptr new_obs_gps;       // GPS, if any

  // Command-line parameters
  std::string out_rawlog;
  std::string in_pcap;
  std::string out_pcap;
  std::string ip_filter;
  std::string calib_file;
  std::string model;
  bool verbose{false};
};

void thread_grabbing(TThreadParam& p)
{
  try
  {
    CCompressedOutputStream f_out_rawlog;
    if (!p.out_rawlog.empty())
    {
      if (!f_out_rawlog.open(p.out_rawlog))
        THROW_EXCEPTION_FMT("Error creating output rawlog file: %s", p.out_rawlog.c_str());
    }
    auto arch = mrpt::serialization::archiveFrom(f_out_rawlog);

    mrpt::hwdrivers::CVelodyneScanner velodyne;

    if (p.verbose) velodyne.enableVerbose(true);

    // Set params:
    velodyne.setModelName(
        mrpt::typemeta::TEnumType<mrpt::hwdrivers::CVelodyneScanner::model_t>::name2value(p.model));
    if (!p.ip_filter.empty()) velodyne.setDeviceIP(p.ip_filter);  // Default: from any IP
    if (!p.in_pcap.empty()) velodyne.setPCAPInputFile(p.in_pcap);
    if (!p.out_pcap.empty()) velodyne.setPCAPOutputFile(p.out_pcap);

    // If you have a calibration file, better than default values:
    if (!p.calib_file.empty())
    {
      mrpt::obs::VelodyneCalibration calib;
      if (!calib.loadFromXMLFile(p.calib_file))
        throw std::runtime_error("Aborting: error loading calibration file.");
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
      CObservationVelodyneScan::Ptr obs;  // (initially empty) Smart pointers to observations
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
    cout << "Exception in Velodyne thread: " << mrpt::exception_to_str(e) << endl;
    p.quit = true;
  }
}

// ------------------------------------------------------
//				VelodyneView main entry point
// ------------------------------------------------------
int VelodyneView(
    const std::string& out_rawlog,
    const std::string& in_pcap,
    const std::string& out_pcap,
    const std::string& ip_filter,
    const std::string& calib_file,
    const std::string& model,
    bool verbose,
    bool nologo)
{
  if (!nologo)
  {
    printf(" velodyne-view - Part of the MRPT\n");
    printf(
        " MRPT C++ Library: %s - Sources timestamp: %s\n", mrpt::system::MRPT_getVersion().c_str(),
        mrpt::system::MRPT_getCompilationDate().c_str());
  }

  // Launch grabbing thread:
  // --------------------------------------------------------
  TThreadParam thrPar;
  thrPar.out_rawlog = out_rawlog;
  thrPar.in_pcap = in_pcap;
  thrPar.out_pcap = out_pcap;
  thrPar.ip_filter = ip_filter;
  thrPar.calib_file = calib_file;
  thrPar.model = model;
  thrPar.verbose = verbose;

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
  if (thrPar.quit)
  {
    return 0;
  }

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
  mrpt::opengl::CPointCloudColoured::Ptr gl_points = mrpt::opengl::CPointCloudColoured::Create();
  gl_points->setPointSize(2.5);

  {
    mrpt::opengl::Scene::Ptr& scene = win3D.get3DSceneAndLock();

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

    CObservationVelodyneScan::Ptr possiblyNewObs = std::atomic_load(&thrPar.new_obs);
    CObservationGPS::Ptr possiblyNewObsGps = std::atomic_load(&thrPar.new_obs_gps);

    if (possiblyNewObsGps && possiblyNewObsGps->timestamp != INVALID_TIMESTAMP &&
        (!last_obs_gps || possiblyNewObsGps->timestamp != last_obs_gps->timestamp))
    {
      // It IS a new observation:
      last_obs_gps = std::atomic_load(&thrPar.new_obs_gps);

      std::string rmc_datum;
      if (last_obs_gps->has_RMC_datum())
      {
        rmc_datum = mrpt::format(
            "Lon=%.09f deg  Lat=%.09f deg  Valid?: '%c'\n",
            last_obs_gps->getMsgByClass<gnss::Message_NMEA_RMC>().fields.longitude_degrees,
            last_obs_gps->getMsgByClass<gnss::Message_NMEA_RMC>().fields.latitude_degrees,
            last_obs_gps->getMsgByClass<gnss::Message_NMEA_RMC>().fields.validity_char);
      }
      else
        rmc_datum = "NO";

      win3D.get3DSceneAndLock();
      win3D.addTextMessage(
          5, 40,
          format(
              "POS. frame rx at %s, RMC=%s",
              mrpt::system::dateTimeLocalToString(last_obs_gps->timestamp).c_str(),
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
                mrpt::system::dateTimeLocalToString(last_obs->timestamp).c_str(),
                static_cast<unsigned int>(last_obs->scan_packets.size())),
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
        5, 10, "'o'/'i'-zoom out/in, mouse: orbit 3D, spacebar: freeze, ESC: quit", 110);
    win3D.addTextMessage(
        5, 25,
        mrpt::format(
            "'1'/'2': Toggle view dual last (%s)/strongest(%s) returns.",
            pc_params.dualKeepLast ? "ON" : "OFF", pc_params.dualKeepStrongest ? "ON" : "OFF"),
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
    CLI::App app("velodyne-view");
    app.set_version_flag("--version", mrpt::system::MRPT_getVersion());

    std::string out_rawlog;
    app.add_option("-o,--out-rawlog", out_rawlog, "If set, grab dataset in rawlog format");

    std::string in_pcap;
    app.add_option(
        "-i,--in-pcap", in_pcap,
        "Instead of listening to a live sensor, read data from a PCAP file");

    std::string out_pcap;
    app.add_option(
        "--out-pcap", out_pcap,
        "If set, grab all packets to a PCAP log file. Set name prefix only.");

    std::string ip_filter;
    app.add_option(
        "--ip-filter", ip_filter, "Only listen to a LIDAR emitting commands from a given IP");

    std::string calib_file;
    app.add_option(
        "-c,--calib", calib_file, "Optionally, select the calibration XML file for the LIDAR");

    std::string model = "VLP16";
    app.add_option(
        "-m,--model", model,
        "If no calibration file is specified, set the model to load default values");

    bool nologo = false;
    app.add_flag("-n,--nologo", nologo, "Skip the logo at startup");

    bool verbose = false;
    app.add_flag("-v,--verbose", verbose, "Verbose debug output");

    // Parse arguments:
    CLI11_PARSE(app, argc, argv);

    int ret =
        VelodyneView(out_rawlog, in_pcap, out_pcap, ip_filter, calib_file, model, verbose, nologo);
    std::this_thread::sleep_for(50ms);  // to allow GUI threads to end gracefully.
    return ret;
  }
  catch (const std::exception& e)
  {
    std::cout << "EXCEPCION: " << mrpt::exception_to_str(e) << "\n";
    return -1;
  }
  catch (...)
  {
    printf("Another exception!!");
    return -1;
  }
}
