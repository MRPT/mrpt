/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: mrpt-performance
	PURPOSE: Do an exhaustive test of performance for various algos of MRPT
  ---------------------------------------------------------------*/

#include <mrpt/version.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/stl_serialization.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;


std::list<TestData> lstTests;

// Data for performance comparatives between different versions:
#ifdef MRPT_DOC_PERF_DIR
const std::string PERF_DATA_DIR = std::string(MRPT_DOC_PERF_DIR);
#else
const std::string PERF_DATA_DIR;
#endif

vector<pair<string,double> >  all_perf_data; // pair: description, time


const float SCAN_RANGES_1[361] = {0.910f,0.900f,0.910f,0.900f,0.900f,0.890f,0.890f,0.880f,0.890f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.870f,0.880f,0.870f,0.870f,0.870f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.880f,0.890f,0.880f,0.880f,0.880f,0.890f,0.880f,0.890f,0.890f,0.880f,0.890f,0.890f,0.880f,0.890f,0.890f,0.890f,0.890f,0.890f,0.890f,0.900f,0.900f,0.900f,0.900f,0.900f,0.910f,0.910f,0.910f,0.910f,0.920f,0.920f,0.920f,0.920f,0.920f,0.930f,0.930f,0.930f,0.930f,0.940f,0.940f,0.950f,0.950f,0.950f,0.950f,0.960f,0.960f,0.970f,0.970f,0.970f,0.980f,0.980f,0.990f,1.000f,1.000f,1.000f,1.010f,1.010f,1.020f,1.030f,1.030f,1.030f,1.040f,1.050f,1.060f,1.050f,1.060f,1.070f,1.070f,1.080f,1.080f,1.090f,1.100f,1.110f,1.120f,1.120f,1.130f,1.140f,1.140f,1.160f,1.170f,1.180f,1.180f,1.190f,1.200f,1.220f,1.220f,1.230f,1.230f,1.240f,1.250f,1.270f,1.280f,1.290f,1.300f,1.320f,1.320f,1.350f,1.360f,1.370f,1.390f,1.410f,1.410f,1.420f,1.430f,1.450f,1.470f,1.490f,1.500f,1.520f,1.530f,1.560f,1.580f,1.600f,1.620f,1.650f,1.670f,1.700f,1.730f,1.750f,1.780f,1.800f,1.830f,1.850f,1.880f,1.910f,1.940f,1.980f,2.010f,2.060f,2.090f,2.130f,2.180f,2.220f,2.250f,2.300f,2.350f,2.410f,2.460f,2.520f,2.570f,2.640f,2.700f,2.780f,2.850f,2.930f,3.010f,3.100f,3.200f,3.300f,3.390f,3.500f,3.620f,3.770f,3.920f,4.070f,4.230f,4.430f,4.610f,4.820f,5.040f,5.290f,5.520f,8.970f,8.960f,8.950f,8.930f,8.940f,8.930f,9.050f,9.970f,9.960f,10.110f,13.960f,18.870f,19.290f,81.910f,20.890f,48.750f,48.840f,48.840f,19.970f,19.980f,19.990f,15.410f,20.010f,19.740f,17.650f,17.400f,14.360f,12.860f,11.260f,11.230f,8.550f,8.630f,9.120f,9.120f,8.670f,8.570f,7.230f,7.080f,7.040f,6.980f,6.970f,5.260f,5.030f,4.830f,4.620f,4.440f,4.390f,4.410f,4.410f,4.410f,4.430f,4.440f,4.460f,4.460f,4.490f,4.510f,4.540f,3.970f,3.820f,3.730f,3.640f,3.550f,3.460f,3.400f,3.320f,3.300f,3.320f,3.320f,3.340f,2.790f,2.640f,2.600f,2.570f,2.540f,2.530f,2.510f,2.490f,2.490f,2.480f,2.470f,2.460f,2.460f,2.460f,2.450f,2.450f,2.450f,2.460f,2.460f,2.470f,2.480f,2.490f,2.490f,2.520f,2.510f,2.550f,2.570f,2.610f,2.640f,2.980f,3.040f,3.010f,2.980f,2.940f,2.920f,2.890f,2.870f,2.830f,2.810f,2.780f,2.760f,2.740f,2.720f,2.690f,2.670f,2.650f,2.630f,2.620f,2.610f,2.590f,2.560f,2.550f,2.530f,2.510f,2.500f,2.480f,2.460f,2.450f,2.430f,2.420f,2.400f,2.390f,2.380f,2.360f,2.350f,2.340f,2.330f,2.310f,2.300f,2.290f,2.280f,2.270f,2.260f,2.250f,2.240f,2.230f,2.230f,2.220f,2.210f,2.200f,2.190f,2.180f,2.170f,1.320f,1.140f,1.130f,1.130f,1.120f,1.120f,1.110f,1.110f,1.110f,1.110f,1.100f,1.110f,1.100f};
const char  SCAN_VALID_1[361] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};


// All this mess is to avoid the smart compiler optimizer to remove "code with no effect" in some test functions...
void dummy_do_nothing_with_string(const std::string &s)
{
	static std::string S;
	S=s;
}

// Benchmark of sample images:
#include "../common/sample_image1.h"
#include "../common/sample_image2.h"

void getTestImage(unsigned int img_index, mrpt::utils::CImage &out_img )
{
	CMemoryStream buf;
	switch(img_index)
	{
		case 0: // RIGHT image of a stereo pair at lab 2.3.7 in Malaga (640x480)
			buf.assignMemoryNotOwn(sample_image1,sizeof(sample_image1));
			break;
		case 1: // LEFT image of a stereo pair at lab 2.3.7 in Malaga (640x480)
			buf.assignMemoryNotOwn(sample_image2,sizeof(sample_image2));
			break;
		default: THROW_EXCEPTION("Sample image index out of range!")
	}
	buf >> out_img;
}


#include "run_build_tables.h"


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		TCLAP::CmdLine cmd("mrpt-performance", ' ', MRPT_getVersion().c_str());

		TCLAP::ValueArg<std::string> arg_contains("c","match-contains","Run only the tests containing the given substring",false,"NAME","NAME",cmd);

		TCLAP::SwitchArg arg_build_tables("t","tables","Don't run any test, instead build the tables of compared performances in SOURCE_DIR/doc/",cmd,false);
		TCLAP::SwitchArg arg_release("r","release","Don't use the postfix 'dev' in the performance stats file",cmd,false);

		// Parse arguments:
		if (!cmd.parse( argc, argv ))
			throw std::runtime_error(""); // should exit.

		if (arg_build_tables.isSet())
			return run_build_tables();

		const std::string filName = "./mrpt-performance.html";

		std::string  match_contains;
		if (arg_contains.isSet())
		{
			cout << "Using match filter: " << match_contains << endl;
			match_contains = arg_contains.getValue();
		}


		bool  doLog = true;
		bool  HAVE_PERF_DATA_DIR = !PERF_DATA_DIR.empty() && mrpt::system::directoryExists(PERF_DATA_DIR);
		if (HAVE_PERF_DATA_DIR)
			cout << "Using perf-data dir: " << PERF_DATA_DIR << endl;


		CTicTac  globalTime;
		globalTime.Tic();

		CFileOutputStream fo;
		doLog = fo.open(filName);
		if (doLog)
			cout << "Saving log to: " << filName << endl;
		else
			cout << "Cannot save log, error opening " << filName << " for writing..." << endl;

		cout << endl;

		if (doLog)
		{
			fo.printf("<html><head><title>mrpt-performance results</title></head><body>\n");
			fo.printf("\n");
		}

		// Start tests:
		// --------------------
		register_tests_icpslam();
		register_tests_poses();
		register_tests_matrices();
		register_tests_grids();
		register_tests_pointmaps();
		register_tests_random();
		register_tests_math();
		register_tests_image();
		register_tests_scan_matching();
		register_tests_feature_extraction();
		register_tests_feature_matching();
		register_tests_graph();
		register_tests_graphslam();
		register_tests_CObservation3DRangeScan();
		register_tests_atan2lut();

		if (doLog)
		{
			fo.printf("<div align=\"center\"><h3>Results</h3></div><br>");
			fo.printf("<div align=\"center\"><table border=\"1\">\n");
			fo.printf("<tr> <td align=\"center\"><b>Test description</b></td> "
					  "<td align=\"center\"><b>Execution time</b></td>"
					  "<td align=\"center\"><b>Execution rate (Hz)</b></td> </tr>\n");
		}

		for (std::list<TestData>::const_iterator it=lstTests.begin();it!=lstTests.end();it++)
		{
			// Filter tests?
			if (!match_contains.empty())
				if (string::npos==string(it->name).find(match_contains))
					continue; // doesn't have the substring

			printf("%-60s",it->name); cout.flush();

			try
			{
				const double t = it->func(it->arg1,it->arg2); // Run it.

				mrpt::system::setConsoleColor(CONCOL_GREEN);
				cout << mrpt::system::intervalFormat(t);
				mrpt::system::setConsoleColor(CONCOL_NORMAL);
				cout << endl;

				// Make list of all data:
				all_perf_data.push_back( pair<string,double>(it->name, t) );

				if (doLog)
				{
					fo.printf("<tr> <td>%s</td> <td align=\"right\">%s</td> <td align=\"right\">%sHz</td>  </tr>\n",
						it->name,
						mrpt::system::intervalFormat(t).c_str(),
						mrpt::system::unitsFormat(1.0/t).c_str());
				}
			}
			catch (std::exception &e)
			{
				cerr << "Skipped due to exception:\n" << e.what() << endl;
			}
		}

		// Finish log:
		if (doLog)
		{
			fo.printf("</table></div>\n");
			fo.printf("<p> &nbsp; </p>\n");

			if (mrpt::system::fileExists("/proc/cpuinfo"))
			{
				fo.printf("<div align=\"center\"><h3>cpuinfo</h3></div>");
				fo.printf("<p> &nbsp; </p>\n");

				fo.printf("<verbatim><small>");
				ifstream cpuFil("/proc/cpuinfo");
				if (cpuFil.good())
				{
					string s;
					while(!cpuFil.eof())
					{
						getline(cpuFil,s);
						fo.printf("%s <br>\n",s.c_str());
					}
				}
				fo.printf("</small></verbatim>");
				fo.printf("<p> &nbsp; </p>\n");
			}

			fo.printf("<hr><small>Automated test run at %s with %s using program 'mrpt-performance'.<br>Overall run took %s</small>\n",
				mrpt::system::dateTimeLocalToString(now()).c_str(),
				MRPT_getVersion().c_str(),
				mrpt::system::intervalFormat(globalTime.Tac()).c_str() );

			fo.printf("</body></html>\n");
			cout << endl << "Checkout the logfile: " << filName << endl;
		}

		// Save to perf-data dir?
		if (HAVE_PERF_DATA_DIR)
		{
			const char* version_postfix = arg_release.isSet() ? "":"dev";

			// Macros to create strings with the compiler version:
#define ___STR2__(x) #x
#define ___STR1__(x) ___STR2__(x)
#define COMP_VER(NAME,MAJ,MIN,PATCH)  NAME ___STR1__(MAJ) ___STR1__(MIN)  ___STR1__(PATCH)

#if defined(_MSC_VER)
#		if _MSC_VER<=1399
			const char* compiler_name = "MSVC7";
#		elif _MSC_VER<=1499
			const char* compiler_name = "MSVC8";
#		elif _MSC_VER<=1599
			const char* compiler_name = "MSVC9";
#		elif _MSC_VER<=1699
			const char* compiler_name = "MSVC10";
#		elif _MSC_VER<=1799
			const char* compiler_name = "MSVC11";
#		elif _MSC_VER<=1899
			const char* compiler_name = "MSVC12";
#		else
			const char* compiler_name = "MSVC";
#		endif
#elif defined(__clang__)
			const char* compiler_name = COMP_VER("CLANG",__clang_major__,__clang_minor__,__clang_patchlevel__);
#elif defined(__GNUC__)
			const char* compiler_name = COMP_VER("GCC",__GNUC__,__GNUC_MINOR__ ,__GNUC_PATCHLEVEL__);
#else
			const char* compiler_name = "unknowncompiler";
#endif

			const string fil_name =
				PERF_DATA_DIR +
				mrpt::format("/perf-results-%i.%i.%i%s-%s-%ibit.dat",
					int( (MRPT_VERSION >> 8) & 0x0F ),
					int( (MRPT_VERSION >> 4) & 0x0F ),
					int( (MRPT_VERSION >> 0) & 0x0F ),
					version_postfix,
					compiler_name,
					int(MRPT_WORD_SIZE) );
			cout << "Saving perf-data to: " << fil_name << endl;
			CFileOutputStream f( fil_name );
			f << all_perf_data;
		}

		return 0;
	}
	catch (std::exception &e)
	{
		if (::strlen(e.what()))
		{
			setConsoleColor(CONCOL_RED,true);
			std::cerr << "Program finished for an exception!!" << std::endl;
			setConsoleColor(CONCOL_NORMAL,true);

			std::cerr << e.what() << std::endl;

			mrpt::system::pause();
		}
		return -1;
	}
	catch (...)
	{
		setConsoleColor(CONCOL_RED,true);
		std::cerr << "Program finished for an untyped exception!!" << std::endl;
		setConsoleColor(CONCOL_NORMAL,true);

		mrpt::system::pause();
		return -1;
	}
}

