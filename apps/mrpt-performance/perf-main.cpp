/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: mrpt-performance
	PURPOSE: Do an exhaustive test of performance for various algos of MRPT
  ---------------------------------------------------------------*/

#include <mrpt/version.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace std;

std::list<TestData> lstTests;

// Data for performance comparatives between different versions:
#ifdef MRPT_DOC_PERF_DIR
const std::string PERF_DATA_DIR = std::string(MRPT_DOC_PERF_DIR);
#else
const std::string PERF_DATA_DIR;
#endif

vector<pair<string, double>> all_perf_data;  // pair: description, time

// All this mess is to avoid the smart compiler optimizer to remove "code with
// no effect" in some test functions...
void dummy_do_nothing_with_string(const std::string& s)
{
	static std::string S;
	S = s;
}

// Benchmark of sample images:
#include "../common/sample_image1.h"
#include "../common/sample_image2.h"

void getTestImage(unsigned int img_index, mrpt::img::CImage& out_img)
{
	CMemoryStream buf;
	switch (img_index)
	{
		case 0:  // RIGHT image of a stereo pair at lab 2.3.7 in Malaga
			// (640x480)
			buf.assignMemoryNotOwn(sample_image1, sizeof(sample_image1));
			break;
		case 1:  // LEFT image of a stereo pair at lab 2.3.7 in Malaga (640x480)
			buf.assignMemoryNotOwn(sample_image2, sizeof(sample_image2));
			break;
		default:
			THROW_EXCEPTION("Sample image index out of range!");
	}
	archiveFrom(buf) >> out_img;
}

#include "run_build_tables.h"

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		TCLAP::CmdLine cmd("mrpt-performance", ' ', MRPT_getVersion().c_str());

		TCLAP::ValueArg<std::string> arg_contains(
			"c", "match-contains",
			"Run only the tests containing the given substring", false, "NAME",
			"NAME", cmd);

		TCLAP::SwitchArg arg_build_tables(
			"t", "tables",
			"Don't run any test, instead build the tables of compared "
			"performances in SOURCE_DIR/doc/",
			cmd, false);
		TCLAP::SwitchArg arg_release(
			"r", "release",
			"Don't use the postfix 'dev' in the performance stats file", cmd,
			false);

		// Parse arguments:
		if (!cmd.parse(argc, argv))
			throw std::runtime_error("");  // should exit.

		if (arg_build_tables.isSet()) return run_build_tables();

		const std::string filName = "./mrpt-performance.html";

		std::string match_contains;
		if (arg_contains.isSet())
		{
			cout << "Using match filter: " << match_contains << endl;
			match_contains = arg_contains.getValue();
		}

		bool doLog = true;
		bool HAVE_PERF_DATA_DIR = !PERF_DATA_DIR.empty() &&
								  mrpt::system::directoryExists(PERF_DATA_DIR);
		if (HAVE_PERF_DATA_DIR)
			cout << "Using perf-data dir: " << PERF_DATA_DIR << endl;

		CTicTac globalTime;
		globalTime.Tic();

		CFileOutputStream fo;
		doLog = fo.open(filName);
		if (doLog)
			cout << "Saving log to: " << filName << endl;
		else
			cout << "Cannot save log, error opening " << filName
				 << " for writing..." << endl;

		cout << endl;

		if (doLog)
		{
			fo.printf(
				"<html><head><title>mrpt-performance "
				"results</title></head><body>\n");
			fo.printf("\n");
		}

		// Start tests:
		// --------------------
		register_tests_icpslam();
		register_tests_poses();
		register_tests_pose_interp();
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
		register_tests_strings();
		register_tests_octomaps();

		if (doLog)
		{
			fo.printf("<div align=\"center\"><h3>Results</h3></div><br>");
			fo.printf("<div align=\"center\"><table border=\"1\">\n");
			fo.printf(
				"<tr> <td align=\"center\"><b>Test description</b></td> "
				"<td align=\"center\"><b>Execution time</b></td>"
				"<td align=\"center\"><b>Execution rate (Hz)</b></td> </tr>\n");
		}

		for (auto it = lstTests.begin(); it != lstTests.end(); it++)
		{
			// Filter tests?
			if (!match_contains.empty())
				if (string::npos == string(it->name).find(match_contains))
					continue;  // doesn't have the substring

			printf("%-60s", it->name);
			cout.flush();

			try
			{
				const double t = it->func(it->arg1, it->arg2);  // Run it.

				mrpt::system::setConsoleColor(CONCOL_GREEN);
				cout << mrpt::system::intervalFormat(t);
				mrpt::system::setConsoleColor(CONCOL_NORMAL);
				cout << endl;

				// Make list of all data:
				all_perf_data.emplace_back(it->name, t);

				if (doLog)
				{
					fo.printf(
						"<tr> <td>%s</td> <td align=\"right\">%s</td> <td "
						"align=\"right\">%sHz</td>  </tr>\n",
						it->name, mrpt::system::intervalFormat(t).c_str(),
						mrpt::system::unitsFormat(1.0 / t).c_str());
				}
			}
			catch (const std::exception& e)
			{
				cerr << "Skipped due to exception:\n"
					 << mrpt::exception_to_str(e) << endl;
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
					while (!cpuFil.eof())
					{
						getline(cpuFil, s);
						fo.printf("%s <br>\n", s.c_str());
					}
				}
				fo.printf("</small></verbatim>");
				fo.printf("<p> &nbsp; </p>\n");
			}

			fo.printf(
				"<hr><small>Automated test run at %s with %s using program "
				"'mrpt-performance'.<br>Overall run took %s</small>\n",
				mrpt::system::dateTimeLocalToString(now()).c_str(),
				MRPT_getVersion().c_str(),
				mrpt::system::intervalFormat(globalTime.Tac()).c_str());

			fo.printf("</body></html>\n");
			cout << endl << "Checkout the logfile: " << filName << endl;
		}

		// Save to perf-data dir?
		if (HAVE_PERF_DATA_DIR)
		{
			const char* version_postfix = arg_release.isSet() ? "" : "dev";

// Macros to create strings with the compiler version:
#define ___STR2__(x) #x
#define ___STR1__(x) ___STR2__(x)
#define COMP_VER(NAME, MAJ, MIN, PATCH) \
	NAME ___STR1__(MAJ) ___STR1__(MIN) ___STR1__(PATCH)

#if defined(_MSC_VER)
#if _MSC_VER == 1700
			const char* compiler_name = "MSVC2012";
#elif _MSC_VER == 1800
			const char* compiler_name = "MSVC2013";
#elif _MSC_VER == 1900
			const char* compiler_name = "MSVC2015";
#elif (_MSC_VER >= 1910) && (_MSC_VER <= 1919)
			const char* compiler_name = "MSVC2017";
#else
			const char* compiler_name = "MSVC";
#endif
#elif defined(__clang__)
			const char* compiler_name = COMP_VER(
				"CLANG", __clang_major__, __clang_minor__,
				__clang_patchlevel__);
#elif defined(__GNUC__)
			const char* compiler_name =
				COMP_VER("GCC", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#else
			const char* compiler_name = "unknowncompiler";
#endif

			const string fil_name =
				PERF_DATA_DIR + mrpt::format(
									"/perf-results-%i.%i.%i%s-%s-%ibit.dat",
									int((MRPT_VERSION >> 8) & 0x0F),
									int((MRPT_VERSION >> 4) & 0x0F),
									int((MRPT_VERSION >> 0) & 0x0F),
									version_postfix, compiler_name,
									int(MRPT_WORD_SIZE));
			cout << "Saving perf-data to: " << fil_name << endl;
			CFileOutputStream f(fil_name);
			auto arch = archiveFrom(f);
			arch << all_perf_data;
		}

		return 0;
	}
	catch (const std::exception& e)
	{
		if (::strlen(e.what()))
		{
			setConsoleColor(CONCOL_RED, true);
			std::cerr << "Program finished for an exception!!" << std::endl;
			setConsoleColor(CONCOL_NORMAL, true);

			std::cerr << mrpt::exception_to_str(e) << std::endl;

			mrpt::system::pause();
		}
		return -1;
	}
	catch (...)
	{
		setConsoleColor(CONCOL_RED, true);
		std::cerr << "Program finished for an untyped exception!!" << std::endl;
		setConsoleColor(CONCOL_NORMAL, true);

		mrpt::system::pause();
		return -1;
	}
}
