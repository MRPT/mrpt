/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/random.h>

#include <mrpt/otherlibs/tclap/CmdLine.h>


using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::random;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;


void do_simulation();
void simulOdometry(const CPose2D & real_pose, CPose2D &last_pose, CPose2D &Apose, const CActionRobotMovement2D::TMotionModelOptions	&odo_opts );

int 	LASER_N_RANGES;
double 	LASER_STD_ERROR;
double 	LASER_BEARING_STD_ERROR;
string	grid_file;
string  gt_file;
string 	out_rawlog_file, in_rawlog_file;

int main(int argc, char ** argv)
{
    try
    {
		// Declare the supported options.
		TCLAP::CmdLine cmd("simul-gridmap", ' ', MRPT_getVersion().c_str());

		TCLAP::ValueArg<std::string>	arg_grid("g","grid","grid map file (*.gridmap or *.gridmap.gz)",true,"","icp_goodness.txt",cmd);
		TCLAP::ValueArg<std::string>	arg_poses("p","poses","poses text file, one 'time x y phi' line per pose",true,"","poses.txt",cmd);
		TCLAP::ValueArg<std::string>	arg_out_rawlog("o","out-rawlog","the output rawlog to generate  from which to take noisy odometry",true,"","out.rawlog",cmd);
		TCLAP::ValueArg<std::string>	arg_in_rawlog("i","in-rawlog","(optional) the rawlog from which to take noisy odometry",false,"","input.rawlog",cmd);
		TCLAP::ValueArg<int>			arg_ranges("r","ranges","number of laser ranges per scan (default=361)",false,361,"icp_goodness.txt",cmd);
		TCLAP::ValueArg<double>			arg_span("s","span","span of the laser scans (default=180 degrees)",false,180,"span",cmd);
		TCLAP::ValueArg<double>			arg_std_r("R","std_r","range noise sigma (default=0.01 meters)",false,0.01,"std_r",cmd);
		TCLAP::ValueArg<double>			arg_std_b("B","std_b","bearing noise sigma (default=0.05 degrees)",false,0.05,"std_b",cmd);

		TCLAP::SwitchArg				arg_nologo("n","nologo","skip the logo at startup",cmd, false);

		// Parse arguments:
		if (!cmd.parse( argc, argv ))
			return 0; // should exit.

		grid_file		= arg_grid.getValue();
		gt_file			= arg_poses.getValue();
		out_rawlog_file	= arg_out_rawlog.getValue();
		in_rawlog_file	= arg_in_rawlog.getValue();
		LASER_N_RANGES	= arg_ranges.getValue();
		LASER_STD_ERROR	= arg_std_r.getValue();
		LASER_BEARING_STD_ERROR = DEG2RAD( arg_std_b.getValue() );

		if (arg_nologo.getValue())
		{
			printf(" simul-gridmap - Part of the MRPT\n");
			printf(" MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		}

		// Invoke method:
		do_simulation();

		return 0;
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return -1;
	}
	catch(...)
	{
		cerr << "Untyped exception." << endl;
		return -1;
	}
}



void do_simulation()
{
	ASSERT_FILE_EXISTS_(gt_file)

	bool have_in_rawlog;
	if (!in_rawlog_file.empty())
	{
		ASSERT_FILE_EXISTS_(in_rawlog_file)
		have_in_rawlog = true;
	}
	else have_in_rawlog = false;

	ASSERT_FILE_EXISTS_(grid_file)

	// Load the grid:
	COccupancyGridMap2D the_grid;
	CFileGZInputStream(grid_file) >> the_grid;

	// GT file rows are:
	//  time x y z
	CMatrixDouble	GT;
	GT.loadFromTextFile(gt_file);

	// Rawlog:
	CRawlog  rawlog;
	if (have_in_rawlog)
	{
		rawlog.loadFromRawLogFile(in_rawlog_file);
	    ASSERT_( rawlog.size()>0 );
	    ASSERT_( rawlog.getType(0) == CRawlog::etActionCollection );
		ASSERT_( rawlog.size()/2 == GT.getRowCount() );
	}

	// # of simulation steps:
	size_t  N = GT.getRowCount();

	// Assert sizes:
    ASSERT_( GT.getColCount() >= 4 );

    // ------------------------------------------
    // Resimulate scans:
    // ------------------------------------------
	if (have_in_rawlog)
	{
		// Modify old rawlog in-place:
		for (size_t i=1;i<rawlog.size();i+=2)
		{
    		ASSERT_( rawlog.getType(i) == CRawlog::etSensoryFrame );

    		CSensoryFramePtr sf = rawlog.getAsObservations(i);
    		CPose2D  gt_pose( GT(i/2,1),GT(i/2,2),GT(i/2,3) );

			CObservation2DRangeScanPtr the_scan = sf->getObservationByClass<CObservation2DRangeScan>();
			the_grid.laserScanSimulator( *the_scan, gt_pose, 0.5f, LASER_N_RANGES, LASER_STD_ERROR, 1, LASER_BEARING_STD_ERROR );
		}
	}
	else
	{
		rawlog.clear();

		CActionCollection	acts;
		CActionRobotMovement2D	act;
		CActionRobotMovement2D::TMotionModelOptions	odo_opts;

		odo_opts.modelSelection = CActionRobotMovement2D::mmGaussian;
		//odo_opts.gaussianModel.

		CPose2D	last_pose( GT(0,1),GT(0,2),GT(0,3) );
		CPose2D	real_pose = last_pose;
		CPose2D Apose;

		simulOdometry(real_pose,last_pose,Apose, odo_opts);
		act.computeFromOdometry( CPose2D(0,0,0), odo_opts );
		act.timestamp = mrpt::system::now();
		acts.insert(act);
		rawlog.addActions(acts);

		// Create a rawlog from scratch:
		for (size_t i=1;i<N;i++)
		{
			// simulate scan:
    		real_pose = CPose2D( GT(i,1),GT(i,2),GT(i,3) );

			CSensoryFramePtr sf = CSensoryFrame::Create();

			CObservation2DRangeScanPtr the_scan = CObservation2DRangeScan::Create();
			the_scan->aperture = M_PIf;
			the_scan->timestamp = mrpt::system::now();
			the_grid.laserScanSimulator( *the_scan, real_pose, 0.5f, LASER_N_RANGES, LASER_STD_ERROR, 1, LASER_BEARING_STD_ERROR );
			sf->insert(the_scan);
			rawlog.addObservationsMemoryReference(sf);

			// Robot moves:
			simulOdometry(real_pose,last_pose,Apose, odo_opts);
			act.computeFromOdometry( CPose2D(0,0,0), odo_opts );
			act.timestamp = mrpt::system::now();
			acts.clear();
			acts.insert(act);
			rawlog.addActions(acts);

		}
	}

    // Save the new rawlog:
    rawlog.saveToRawLogFile(out_rawlog_file);
}


void simulOdometry(
	const CPose2D	&real_pose,
	CPose2D			&last_pose,
	CPose2D			&Apose,
	const CActionRobotMovement2D::TMotionModelOptions	&odo_opts )
{
	Apose = real_pose-last_pose;
	last_pose = real_pose;

	// Noise:
	Apose.x_incr( randomGenerator.drawGaussian1D_normalized() * odo_opts.gaussianModel.minStdXY );
	Apose.y_incr( randomGenerator.drawGaussian1D_normalized() * odo_opts.gaussianModel.minStdXY );
	Apose.phi_incr( randomGenerator.drawGaussian1D_normalized() * odo_opts.gaussianModel.minStdPHI );
	Apose.normalizePhi();
}


