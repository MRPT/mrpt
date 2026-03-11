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

#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::random;
using namespace mrpt::math;
using namespace mrpt::io;
using namespace mrpt::poses;
using namespace std;

void do_simulation();
void simulOdometry(
    const CPose2D& real_pose,
    CPose2D& last_pose,
    CPose2D& Apose,
    const CActionRobotMovement2D::TMotionModelOptions& odo_opts);

int LASER_N_RANGES;
double LASER_STD_ERROR;
double LASER_BEARING_STD_ERROR;
string grid_file;
string gt_file;
string out_rawlog_file, in_rawlog_file;

int main(int argc, char** argv)
{
  try
  {
    // Declare the supported options.
    CLI::App app("simul-gridmap");
    app.set_version_flag("--version", MRPT_getVersion());

    std::string grid_file_arg;
    app.add_option("-g,--grid", grid_file_arg, "grid map file (*.gridmap or *.gridmap.gz)")
        ->required();

    std::string poses_arg;
    app.add_option("-p,--poses", poses_arg, "poses text file, one 'time x y phi' line per pose")
        ->required();

    std::string out_rawlog_arg;
    app.add_option(
           "-o,--out-rawlog", out_rawlog_arg,
           "the output rawlog to generate from which to take noisy odometry")
        ->required();

    std::string in_rawlog_arg;
    app.add_option(
        "-i,--in-rawlog", in_rawlog_arg, "(optional) the rawlog from which to take noisy odometry");

    int ranges = 361;
    app.add_option("-r,--ranges", ranges, "number of laser ranges per scan (default=361)");

    double span = 180.0;
    app.add_option("-s,--span", span, "span of the laser scans (default=180 degrees)");

    double std_r = 0.01;
    app.add_option("-R,--std_r", std_r, "range noise sigma (default=0.01 meters)");

    double std_b = 0.05;
    app.add_option("-B,--std_b", std_b, "bearing noise sigma (default=0.05 degrees)");

    bool nologo = false;
    app.add_flag("-n,--nologo", nologo, "skip the logo at startup");

    // Parse arguments:
    CLI11_PARSE(app, argc, argv);

    grid_file = grid_file_arg;
    gt_file = poses_arg;
    out_rawlog_file = out_rawlog_arg;
    in_rawlog_file = in_rawlog_arg;
    LASER_N_RANGES = ranges;
    LASER_STD_ERROR = std_r;
    LASER_BEARING_STD_ERROR = DEG2RAD(std_b);

    if (!nologo)
    {
      printf(" simul-gridmap - Part of the MRPT\n");
      printf(
          " MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(),
          MRPT_getCompilationDate().c_str());
    }

    // Invoke method:
    do_simulation();

    return 0;
  }
  catch (const std::exception& e)
  {
    cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
  catch (...)
  {
    cerr << "Untyped exception."
         << "\n";
    return 1;
  }
}

void do_simulation()
{
  ASSERT_FILE_EXISTS_(gt_file);

  bool have_in_rawlog;
  if (!in_rawlog_file.empty())
  {
    ASSERT_FILE_EXISTS_(in_rawlog_file);
    have_in_rawlog = true;
  }
  else
    have_in_rawlog = false;

  ASSERT_FILE_EXISTS_(grid_file);

  // Load the grid:
  COccupancyGridMap2D the_grid;
  CCompressedInputStream f(grid_file);
  mrpt::serialization::archiveFrom(f) >> the_grid;

  // GT file rows are:
  //  time x y z
  CMatrixDouble GT;
  GT.loadFromTextFile(gt_file);

  // Rawlog:
  CRawlog rawlog;
  if (have_in_rawlog)
  {
    rawlog.loadFromRawLogFile(in_rawlog_file);
    ASSERT_(rawlog.size() > 0);
    ASSERT_(rawlog.getType(0) == CRawlog::etActionCollection);
    ASSERT_(int(rawlog.size() / 2) == GT.rows());
  }

  // # of simulation steps:
  size_t N = GT.rows();

  // Assert sizes:
  ASSERT_(GT.cols() >= 4);

  // ------------------------------------------
  // Resimulate scans:
  // ------------------------------------------
  if (have_in_rawlog)
  {
    // Modify old rawlog in-place:
    for (size_t i = 1; i < rawlog.size(); i += 2)
    {
      ASSERT_(rawlog.getType(i) == CRawlog::etSensoryFrame);

      CSensoryFrame::Ptr sf = rawlog.getAsObservations(i);
      CPose2D gt_pose(GT(i / 2, 1), GT(i / 2, 2), GT(i / 2, 3));

      CObservation2DRangeScan::Ptr the_scan = sf->getObservationByClass<CObservation2DRangeScan>();
      the_grid.laserScanSimulator(
          *the_scan, gt_pose, 0.5f, LASER_N_RANGES, LASER_STD_ERROR, 1, LASER_BEARING_STD_ERROR);
    }
  }
  else
  {
    rawlog.clear();

    CActionCollection acts;
    CActionRobotMovement2D act;
    CActionRobotMovement2D::TMotionModelOptions odo_opts;

    odo_opts.modelSelection = CActionRobotMovement2D::mmGaussian;
    // odo_opts.gaussianModel.

    CPose2D last_pose(GT(0, 1), GT(0, 2), GT(0, 3));
    CPose2D real_pose = last_pose;
    CPose2D Apose;

    simulOdometry(real_pose, last_pose, Apose, odo_opts);
    act.computeFromOdometry(CPose2D(0, 0, 0), odo_opts);
    act.timestamp = mrpt::Clock::now();
    acts.insert(act);
    rawlog.insert(acts);

    // Create a rawlog from scratch:
    for (size_t i = 1; i < N; i++)
    {
      // simulate scan:
      real_pose = CPose2D(GT(i, 1), GT(i, 2), GT(i, 3));

      CSensoryFrame::Ptr sf = std::make_shared<CSensoryFrame>();

      CObservation2DRangeScan::Ptr the_scan = std::make_shared<CObservation2DRangeScan>();
      the_scan->aperture = M_PIf;
      the_scan->timestamp = mrpt::Clock::now();
      the_grid.laserScanSimulator(
          *the_scan, real_pose, 0.5f, LASER_N_RANGES, LASER_STD_ERROR, 1, LASER_BEARING_STD_ERROR);
      sf->insert(the_scan);
      rawlog.insert(sf);

      // Robot moves:
      simulOdometry(real_pose, last_pose, Apose, odo_opts);
      act.computeFromOdometry(CPose2D(0, 0, 0), odo_opts);
      act.timestamp = mrpt::Clock::now();
      acts.clear();
      acts.insert(act);
      rawlog.insert(acts);
    }
  }

  // Save the new rawlog:
  rawlog.saveToRawLogFile(out_rawlog_file);
}

void simulOdometry(
    const CPose2D& real_pose,
    CPose2D& last_pose,
    CPose2D& Apose,
    const CActionRobotMovement2D::TMotionModelOptions& odo_opts)
{
  Apose = real_pose - last_pose;
  last_pose = real_pose;

  // Noise:
  Apose.x_incr(getRandomGenerator().drawGaussian1D_normalized() * odo_opts.gaussianModel.minStdXY);
  Apose.y_incr(getRandomGenerator().drawGaussian1D_normalized() * odo_opts.gaussianModel.minStdXY);
  Apose.phi_incr(
      getRandomGenerator().drawGaussian1D_normalized() * odo_opts.gaussianModel.minStdPHI);
  Apose.normalizePhi();
}
