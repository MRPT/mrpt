/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CNationalInstrumentsDAQ.h>
#include <mrpt/system/os.h>
#include <cstdio>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;
using namespace std;

// ------------------------------------------------------
//				Test_NIDAQ
// ------------------------------------------------------
void Test_NIDAQ()
{
	CNationalInstrumentsDAQ	 daq;

	// Load config:
#if 0
	//daq.loadConfig( CConfigFile( "./DAQ_example.ini") ,"DAQ1" );
#else
	// Or set params programatically:

	
	if (1)
	{
		// Define a task with analog inputs:
		CNationalInstrumentsDAQ::TaskDescription task;
		task.has_ai = true;
		task.ai.physicalChannel = "Dev1/ai0:7";
		task.ai.physicalChannelCount = 8; // Must be the number of channels encoded in the "physicalChannel" string.
		task.ai.terminalConfig  = "DAQmx_Val_RSE";
		task.ai.minVal = -10;
		task.ai.maxVal =  10;

		daq.task_definitions.push_back(task);
	}

	if (0)
	{
		// Define a task with 1 analog output:
		CNationalInstrumentsDAQ::TaskDescription task;
		task.has_ao = true;
		task.ao.physicalChannel = "Dev1/ao0";
		task.ao.physicalChannelCount = 1; // Must be the number of channels encoded in the "physicalChannel" string.
		task.ao.minVal = -10;
		task.ao.maxVal =  10;

		daq.task_definitions.push_back(task);
	}

	if (1)
	{
		{
			// Define a task with 1 digital output:
			CNationalInstrumentsDAQ::TaskDescription task;
			task.has_do = true;
			task.douts.line = "Dev1/port1/line0";

			daq.task_definitions.push_back(task);
		}
		{
			// Define a task with 1 digital output:
			CNationalInstrumentsDAQ::TaskDescription task;
			task.has_do = true;
			task.douts.line = "Dev1/port1/line1";

			daq.task_definitions.push_back(task);
		}
	}

#endif

	printf("[Example] Initializing DAQ...\n");
	daq.initialize();
	printf("[Example] Init passed.\n");

	printf("\n ** Press any key to stop grabbing ** \n");

	// Test analog output:
	if (0)
	{
		double volt_values[1]= { -4.0 };
		daq.writeAnalogOutputTask(1, sizeof(volt_values)/sizeof(volt_values[0]),volt_values,0.100, true);
	}

	// Test digital output:
	if (0)
	{
		while (1)
		{
			cout << "1 ";
			daq.writeDigitalOutputTask(1, true, 0.1);
			daq.writeDigitalOutputTask(2, false, 0.1);
			mrpt::system::sleep(5000);
			cout << "0 ";
			daq.writeDigitalOutputTask(1, false, 0.1);
			daq.writeDigitalOutputTask(2, true, 0.1);
			mrpt::system::sleep(5000);
		}
	}

	// Loop reading:
	while (!mrpt::system::os::kbhit())
	{
		std::vector<mrpt::obs::CObservationRawDAQPtr> readObs;
		bool hardError;

		try
		{
			daq.readFromDAQ( readObs, hardError );
		}
		catch (std::exception &e)
		{
			cerr << e.what() << endl;
			hardError = true;
		}

		if (hardError)
			printf("[TEST] Hardware error=true!!\n");

		if (!readObs.empty())
		{
			// Look for analog readings:
			for (size_t i=0;i<readObs.size();i++)
			{
				if (readObs[i]->AIN_double.empty())
					continue; // Skip

				const size_t nSamplPerChan = readObs[i]->AIN_double.size() / readObs[i]->AIN_channel_count;
				cout << "Read " << nSamplPerChan << " samples. a[0]=" << readObs[i]->AIN_double[0] << endl;
			}
		}

		mrpt::system::sleep(1);
	};
}

int main()
{
	try
	{
		Test_NIDAQ();
		return 0;

	} catch (std::exception &e)
	{
		std::cout << "EXCEPTION: " << e.what() << std::endl;
		return -1;
	}
}

