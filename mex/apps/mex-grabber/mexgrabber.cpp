/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */

/*-----------------------------------------------------------------------------
	APPLICATION: MEX-grabber
	FILE: mexgrabber.cpp
	AUTHORS: Jesus Briales Garcia <jesusbriales@gmail.com>
			 Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	For instructions and details, see:
	 http://www.mrpt.org/list-of-mrpt-apps/application-MEX-grabber
  -----------------------------------------------------------------------------*/

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/img/CImage.h>
#include <mrpt/core/round.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>

// Matlab MEX interface headers
#include <mexplus.h>

// Temporal includes
MRPT_TODO(
	"This won't be necessary when CObservation2DRangeScan::writeToMatlab is "
	"fully functional")
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <mutex>
#include <thread>

using namespace mrpt::obs;

// Force here using mexPrintf instead of printf
#define printf mexPrintf

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace std;
using namespace mexplus;

const std::string GLOBAL_SECTION_NAME = "global";

// Forward declarations:
struct TThreadParams
{
	CConfigFile* cfgFile;
	string sensor_label;
};

void SensorThread(TThreadParams params);

CGenericSensor::TListObservations global_list_obs;
std::mutex cs_global_list_obs;

bool allThreadsMustExit = false;

// Thread handlers vector stored as global (persistent MEX variables)
vector<std::thread> lstThreads;

// State variables
bool mex_is_running = false;

// Configuration variables
MRPT_TODO("Set as variable controlled from Matlab")
size_t max_num_obs = 50;

/* Important:
 * All global variables will be stored between MEX calls,
 * and can be used by running threads backstage. */

namespace
{
// Defines MEX API for new.
MEX_DEFINE(new)(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	printf(" mex-grabber - Part of the MRPT\n");
	printf(
		" MRPT C++ Library: %s - Sources timestamp: %s\n",
		MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
	printf(
		"-------------------------------------------------------------------"
		"\n");

	mexplus::InputArguments input(nrhs, prhs, 1);
	//    mexplus::OutputArguments output(nlhs, plhs, 1);

	//    if (mex_is_running)
	//    {
	//        printf("[mex-grabber::new] Application is already running\n");
	//        return;
	//    }

	// Initialize global (persistent) variables
	mex_is_running = true;
	allThreadsMustExit = false;

	string INI_FILENAME(input.get<string>(0));
	ASSERT_FILE_EXISTS_(INI_FILENAME)

	CConfigFile iniFile(INI_FILENAME);

	// ------------------------------------------
	//			Load config from file:
	// ------------------------------------------
	int time_between_launches = 300;
	double SF_max_time_span = 0.25;  // Seconds
	bool use_sensoryframes = false;
	int GRABBER_PERIOD_MS = 1000;

	MRPT_LOAD_CONFIG_VAR(
		time_between_launches, int, iniFile, GLOBAL_SECTION_NAME);
	MRPT_LOAD_CONFIG_VAR(SF_max_time_span, float, iniFile, GLOBAL_SECTION_NAME);
	MRPT_LOAD_CONFIG_VAR(use_sensoryframes, bool, iniFile, GLOBAL_SECTION_NAME);
	MRPT_LOAD_CONFIG_VAR(GRABBER_PERIOD_MS, int, iniFile, GLOBAL_SECTION_NAME);

	// ----------------------------------------------
	// Launch threads:
	// ----------------------------------------------
	std::vector<std::string> sections;
	iniFile.getAllSections(sections);

	for (std::vector<std::string>::iterator it = sections.begin();
		 it != sections.end(); ++it)
	{
		if (*it == GLOBAL_SECTION_NAME || it->empty() ||
			iniFile.read_bool(*it, "rawlog-grabber-ignore", false, false))
			continue;  // This is not a sensor:

		// cout << "Launching thread for sensor '" << *it << "'" << endl;

		TThreadParams threParms;
		threParms.cfgFile = &iniFile;
		threParms.sensor_label = *it;

		std::thread thre = std::thread(SensorThread, threParms);

		lstThreads.emplace_back(std::move(thre));
		std::this_thread::sleep_for(
			std::chrono::milliseconds(time_between_launches));
	}

	printf("[mex-grabber::new] All threads launched\n");
}  // End of "new" method

// Defines MEX API for read (acquire observations in Matlab form)
MEX_DEFINE(read)(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	// ----------------------------------------------
	// Run:
	// ----------------------------------------------
	//    mexplus::InputArguments input(nrhs, prhs, 1);
	mexplus::OutputArguments output(nlhs, plhs, 1);

	CGenericSensor::TListObservations copy_of_global_list_obs;

	// See if we have observations and process them:
	{
		std::lock_guard<std::mutex> lock(cs_global_list_obs);
		copy_of_global_list_obs.clear();

		if (!global_list_obs.empty())
		{
			CGenericSensor::TListObservations::iterator itEnd =
				global_list_obs.begin();
			std::advance(itEnd, global_list_obs.size() / 2);
			copy_of_global_list_obs.insert(global_list_obs.begin(), itEnd);
			global_list_obs.erase(global_list_obs.begin(), itEnd);
		}
	}  // End of cs lock

	// Read from list of observations to mxArray cell array (store any kind of
	// objects)
	MxArray cell_obs(MxArray::Cell(1, copy_of_global_list_obs.size()));
	size_t index = 0;

	for (CGenericSensor::TListObservations::iterator it =
			 copy_of_global_list_obs.begin();
		 it != copy_of_global_list_obs.end(); ++it)
	{
		MxArray struct_obs(it->second->writeToMatlab());

		// Special cases (TEMPORAL)
		if (IS_CLASS(it->second, CObservation2DRangeScan))
		{
			// Get Points Map from 2D Range Scan
			CObservation2DRangeScan::Ptr LRF_obs =
				std::dynamic_pointer_cast<CObservation2DRangeScan>(it->second);
			struct_obs.set(
				"map",
				LRF_obs->buildAuxPointsMap<mrpt::maps::CSimplePointsMap>()
					->writeToMatlab());
		}
		cell_obs.set(index, struct_obs.release());
		index++;
	}

	// Returns created struct
	output.set(0, cell_obs.release());

	// No need to sleep, since this function only applies when user requested
	// from Matlab
}  // End of "read" method

// Defines MEX API for delete.
MEX_DEFINE(delete)(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	//    mexplus::InputArguments input(nrhs, prhs, 1);
	//    mexplus::OutputArguments output(nlhs, plhs, 0);
	if (allThreadsMustExit)
	{
		printf(
			"[main thread] Ended due to other thread signal to exit "
			"application.\n");
	}

	// Wait all threads:
	// ----------------------------
	allThreadsMustExit = true;
	std::this_thread::sleep_for(300ms);
	printf("\nWaiting for all threads to close...\n");
	for (vector<std::thread>::iterator th = lstThreads.begin();
		 th != lstThreads.end(); ++th)
		th->join();

	cout << endl
		 << "[mex-grabber::delete] mex-grabber application finished" << endl;
	std::this_thread::sleep_for(1000ms);  // Time for the wxSubsystem to close
	// all remaining windows and avoid
	// crash... (any better way?)
	mex_is_running = false;
}  // End of "delete" method

}  // End of namespace

MEX_DISPATCH  // Don't forget to add this if MEX_DEFINE() is used.

	// ------------------------------------------------------
	//				SensorThread
	// ------------------------------------------------------
	void
	SensorThread(TThreadParams params)
{
	try
	{
		string driver_name = params.cfgFile->read_string(
			params.sensor_label, "driver", "", true);

		CGenericSensor::Ptr sensor =
			CGenericSensor::createSensorPtr(driver_name);
		if (!sensor)
		{
			cerr << endl
				 << "***ERROR***: Class name not recognized: " << driver_name
				 << endl;
			allThreadsMustExit = true;
		}

		// Load common & sensor specific parameters:
		sensor->loadConfig(*params.cfgFile, params.sensor_label);

		cout << format("[thread_%s] Starting...", params.sensor_label.c_str())
			 << " at " << sensor->getProcessRate() << " Hz" << endl;

		ASSERTMSG_(
			sensor->getProcessRate() > 0,
			"process_rate must be set to a valid value (>0 Hz).");
		int process_period_ms = round(1000.0 / sensor->getProcessRate());

		// Init device:
		sensor->initialize();

		while (!allThreadsMustExit)
		{
			TTimeStamp t0 = now();

			// Process
			sensor->doProcess();

			// Get new observations
			CGenericSensor::TListObservations lstObjs;
			sensor->getObservations(lstObjs);

			{
				std::lock_guard<std::mutex> lock(cs_global_list_obs);
				// Control maximum number of stored observations to prevent
				// excesive growth of list between calls
				if (global_list_obs.size() < 2 * max_num_obs)  // .size() is
					// returning 2
					// countings for
					// each pair
					global_list_obs.insert(lstObjs.begin(), lstObjs.end());
			}

			lstObjs.clear();

			// wait until the process period:
			TTimeStamp t1 = now();
			double At = timeDifference(t0, t1);
			int At_rem_ms = process_period_ms - At * 1000;
			if (At_rem_ms > 0)
				std::this_thread::sleep_for(
					std::chrono::milliseconds(At_rem_ms));
		}

		sensor.reset();
		cout << format("[thread_%s] Closing...", params.sensor_label.c_str())
			 << endl;
	}
	catch (const std::exception& e)
	{
		printf("[mex-grabber::Exception] %s\n", e.what());
		allThreadsMustExit = true;
	}
	catch (...)
	{
		printf("[mex-grabber::UntypedException]\n");
		allThreadsMustExit = true;
	}
}

// ------------------------------------------------------
//					MAIN THREAD
//
// For testing outside Matlab
// ------------------------------------------------------
int main(int argc, const char* argv[])
{
	try
	{
		printf(" MEX-grabber - Part of the MRPT\n");
		printf(
			" MRPT C++ Library: %s - Sources timestamp: %s\n",
			MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf(
			"------------------------------------------------------------------"
			"-\n");
		printf(" This is a test for Matlab MEX functionalities\n");
		printf(
			"------------------------------------------------------------------"
			"-\n");

		// Launch threads with "new"
		const mxArray* mxIn[2];
		mxIn[0] = mexplus::from("new");
		mxIn[1] = mexplus::from(
			argv[1]);  // Read config file path, first argv is function name
		mxArray* mxOut[1];
		mexFunction(1, mxOut, 2, mxIn);

		std::this_thread::sleep_for(
			5000ms);  // Time for the sensor to read before collecting

		// Read frames with "read"
		mxIn[0] = mexplus::from("read");
		mexFunction(1, mxOut, 1, mxIn);

		// Finish applicatin with "delete"
		mxIn[0] = mexplus::from("delete");
		mexFunction(1, mxOut, 1, mxIn);

		// Repete whole process
		// Launch threads with "new"
		mxIn[0] = mexplus::from("new");
		mxIn[1] = mexplus::from(
			argv[1]);  // Read config file path, first argv is function name
		mexFunction(1, mxOut, 2, mxIn);

		// Read frames with "read"
		mxIn[0] = mexplus::from("read");
		mexFunction(1, mxOut, 1, mxIn);

		// Finish applicatin with "delete"
		mxIn[0] = mexplus::from("delete");
		mexFunction(1, mxOut, 1, mxIn);

		return 0;
	}
	catch (const std::exception& e)
	{
		printf("[mex-grabber::Exception] %s\n", e.what());
		return -1;
	}
	catch (...)
	{
		printf("[mex-grabber::UntypedException]\n");
		return -1;
	}
}
