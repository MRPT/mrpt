/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers.h> // Precompiled headers

#include <mrpt/hwdrivers/CNationalInstrumentsDAQ.h>

#if MRPT_HAS_NIDAQmx
#	if (MRPT_WORD_SIZE==64) && !defined(WIN64)
#		define WIN64
#	endif
#	include "NIDAQmx.h"  // Include file for NI-DAQmx API
#if defined(_MSC_VER)
#	pragma comment (lib,"NIDAQmx.lib")
#endif
#endif

#if MRPT_HAS_NIDAQmx
//#	define DEV_HANDLER  reinterpret_cast<NiHandle*>(m_niDevHandle)
#endif

using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::synch;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CNationalInstrumentsDAQ,mrpt::hwdrivers)


/* -----------------------------------------------------
                Constructor
   ----------------------------------------------------- */
CNationalInstrumentsDAQ::CNationalInstrumentsDAQ()
{
	m_sensorLabel = "NIDAQ";
}

/* -----------------------------------------------------
                loadConfig_sensorSpecific
   ----------------------------------------------------- */
void  CNationalInstrumentsDAQ::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
#ifdef MRPT_OS_WINDOWS
	//m_COMname = configSource.read_string(iniSection, "COM_port_WIN", m_COMname, true );
#else
	//m_COMname = configSource.read_string(iniSection, "COM_port_LIN", m_COMname, true );
#endif
}

/* -----------------------------------------------------
                Destructor
   ----------------------------------------------------- */
CNationalInstrumentsDAQ::~CNationalInstrumentsDAQ()
{
}


/* -----------------------------------------------------
				doProcess
----------------------------------------------------- */
void  CNationalInstrumentsDAQ::doProcess()
{
	// Is the COM open?
	//if (!tryToOpenTheCOM())
	//{
	//	m_state = ssError;
	//	THROW_EXCEPTION("Cannot open the serial port");
	//}

}

#if 0
// Dev1/ai0, Dev1/ai3:6 
#define CHANNEL_NAME "cDAQ1Mod1/ai0"

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

int32 my_callbackFunction(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData)
{
	mrpt::vector_double  data(1000);
	int32       read=0;
	const int32 toRead = 1000;

	DAQmxReadAnalogF64(taskHandle,toRead,1.0,DAQmx_Val_GroupByChannel,&data[0],data.size(),&read,NULL);
	if ( read == toRead)
	{
		last_read = data.mean();
		const double eps = gauge_volt2strain_quarter(last_read-offset_volt,K_GAUGE_FACTOR);
		printf("Volt = %f --> \t  eps = %f \t uEps = %f\n",last_read, eps, 1e6*eps  );

		win.plot(data);
	}
	return 0;
}

	int32       error=0;
	TaskHandle  taskHandle=0;
	char        errBuff[2048]={'\0'};


	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk (DAQmxCreateTask("",&taskHandle));
	DAQmxErrChk (DAQmxCreateAIVoltageChan(taskHandle,CHANNEL_NAME,"",
		DAQmx_Val_Cfg_Default,
		-10.0,10.0,
		DAQmx_Val_Volts,
		NULL));
	DAQmxErrChk (DAQmxCfgSampClkTiming(taskHandle,"",10000.0,DAQmx_Val_Rising,
		DAQmx_Val_ContSamps, //DAQmx_Val_FiniteSamps,
		1000));

	DAQmxRegisterEveryNSamplesEvent(taskHandle, DAQmx_Val_Acquired_Into_Buffer, 1000, 0, &my_callbackFunction, NULL);

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk (DAQmxStartTask(taskHandle));


	bool end = false;
	while (!end)
	{
		mrpt::system::sleep(100);
		if (kbhit())
		{
			const int c = getch();
			switch(c)
			{
			case 'r':
				offset_volt = last_read;
				printf("** Setting offset level = %f\n",last_read);
				break;

			case 'q': 
				end=true;
				break;
			};
		}
	}

Error:
	if( DAQmxFailed(error) )
		DAQmxGetExtendedErrorInfo(errBuff,2048);
	if( taskHandle!=0 )  {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if( DAQmxFailed(error) )
		printf("DAQmx Error: %s\n",errBuff);
	printf("End of program, press Enter key to quit\n");
	getchar();
	return 0;

#endif

