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

#if MRPT_HAS_NIDAQMXBASE
#	include "NIDAQmxBase.h"  // Include file for NI-DAQmx API
#endif


// An auxiliary macro to check and report errors in the DAQmx library as exceptions with a well-explained message.
#define MRPT_DAQmx_ErrChk(functionCall) \
	if( (functionCall)<0) \
	{ \
		char errBuff[2048]; \
		DAQmxBaseGetExtendedErrorInfo(errBuff,2048); \
		std::string sErr = mrpt::format("DAQ error: '%s'\nCalling: '%s'",errBuff,#functionCall); \
		THROW_EXCEPTION(sErr) \
	}


using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CNationalInstrumentsDAQ,mrpt::hwdrivers)

// -------------  CNationalInstrumentsDAQ::TInfoPerTask  -----------
// Default ctor:
CNationalInstrumentsDAQ::TInfoPerTask::TInfoPerTask() : 
	taskHandle(0),
	must_close(false), 
	is_closed(false)
{ }

// Copy ctor (needed for the auto_ptr semantics)
CNationalInstrumentsDAQ::TInfoPerTask::TInfoPerTask(const TInfoPerTask &o) : 
	taskHandle(o.taskHandle),
	hThread(o.hThread),
	read_pipe(o.read_pipe.get()),
	write_pipe(o.write_pipe.get()),
	must_close(o.must_close), 
	is_closed(o.is_closed)
{ 
	const_cast<TInfoPerTask*>(&o)->read_pipe.release(); 
	const_cast<TInfoPerTask*>(&o)->write_pipe.release(); 
}


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
	//m_COMname = configSource.read_string(iniSection, "COM_port_WIN", m_COMname, true );

}

/* -----------------------------------------------------
                Destructor
   ----------------------------------------------------- */
CNationalInstrumentsDAQ::~CNationalInstrumentsDAQ()
{
	this->stop();
}


/* -----------------------------------------------------
				initialize
----------------------------------------------------- */
void  CNationalInstrumentsDAQ::initialize()
{
#if MRPT_HAS_NIDAQMXBASE

	MRPT_TODO("Parse config and create the needed tasks")

//#define CHANNEL_NAME "cDAQ1Mod1/ai0"
	#define CHANNEL_NAME "Dev1/ai0"

	// Try to create a new task:
	m_running_tasks.push_back(TInfoPerTask());
	TInfoPerTask &ipt = m_running_tasks.back();

	try
	{
		TaskHandle  &taskHandle= *reinterpret_cast<TaskHandle*>(&ipt.taskHandle);

		MRPT_DAQmx_ErrChk (DAQmxBaseCreateTask("",&taskHandle));
		MRPT_DAQmx_ErrChk (DAQmxBaseCreateAIVoltageChan(taskHandle,CHANNEL_NAME,"",
			DAQmx_Val_Cfg_Default,
			-10.0,10.0,
			DAQmx_Val_Volts,
			NULL));
		MRPT_DAQmx_ErrChk (DAQmxBaseCfgSampClkTiming(taskHandle,"",10000.0,DAQmx_Val_Rising,
			DAQmx_Val_ContSamps, //DAQmx_Val_FiniteSamps,
			0));

		// Create pipe:
		mrpt::synch::CPipe::createPipe(ipt.read_pipe, ipt.write_pipe);

		MRPT_DAQmx_ErrChk (DAQmxBaseCfgInputBuffer(taskHandle,200000)); // sample DMA buffer
		MRPT_DAQmx_ErrChk (DAQmxBaseStartTask(taskHandle));

		ipt.hThread = mrpt::system::createThreadFromObjectMethodRef<CNationalInstrumentsDAQ,TInfoPerTask>(this, &CNationalInstrumentsDAQ::grabbing_thread, ipt);

	}
	catch (std::exception &e)
	{
		if( ipt.taskHandle!=NULL )  
		{
			TaskHandle  &taskHandle= *reinterpret_cast<TaskHandle*>(&ipt.taskHandle);
			DAQmxBaseStopTask(taskHandle);
			DAQmxBaseClearTask(taskHandle);
		}

		// Stop thread:
		if (!ipt.hThread.isClear())
		{
			ipt.must_close=true;
			cerr << "[CNationalInstrumentsDAQ::initialize] Waiting for the grabbing thread to end due to exception...\n";
			mrpt::system::joinThread(ipt.hThread);
			cerr << "[CNationalInstrumentsDAQ::initialize] Grabbing thread ended.\n";
		}
		
		// Remove from list:
		m_running_tasks.erase(--m_running_tasks.end());

		throw e;
	}
#else
	THROW_EXCEPTION("MRPT was compiled without support for NI DAQmx!!")
#endif
}

/** Stop the grabbing threads for DAQ tasks. It is automatically called at destruction. */
void CNationalInstrumentsDAQ::stop()
{
	// Stop all threads:
	for (list<TInfoPerTask>::iterator it=m_running_tasks.begin();it!=m_running_tasks.end();++it)
	{
		it->must_close=true;
	}
	for (list<TInfoPerTask>::iterator it=m_running_tasks.begin();it!=m_running_tasks.end();++it)
	{
		if (!it->hThread.isClear())
		{
			mrpt::system::joinThread(it->hThread);
			it->hThread.clear();
		}
	}

	// Stop all NI tasks:
#if MRPT_HAS_NIDAQMXBASE
	for (list<TInfoPerTask>::iterator it=m_running_tasks.begin();it!=m_running_tasks.end();++it)
	{
		TaskHandle  &taskHandle= *reinterpret_cast<TaskHandle*>(&it->taskHandle);
	
		DAQmxBaseStopTask(taskHandle);
		DAQmxBaseClearTask(taskHandle);
		taskHandle=NULL;
	}
#endif
}

/** Returns true if initialize() was called successfully. */
bool CNationalInstrumentsDAQ::checkDAQIsConnected() const
{
	MRPT_TODO("XX")
	return true;
}


/*-------------------------------------------------------------
						readFromDAQ
-------------------------------------------------------------*/
void  CNationalInstrumentsDAQ::readFromDAQ(
	bool							&outThereIsObservation,
	mrpt::slam::CObservationRawDAQ	&outObservation,
	bool							&hardwareError )
{
	outThereIsObservation	= false;
	hardwareError			= false;

	if ( !checkDAQIsConnected() )
	{
		hardwareError = true;
		return;
	}

	// Read from the pipe:
    m_state = ssWorking;

	if (1) 
		return;

	// Yes, we have a new scan:

	// -----------------------------------------------
	//   Extract the observation:
	// -----------------------------------------------
	outObservation.timestamp = mrpt::system::now();
	outObservation.sensorLabel  = m_sensorLabel;	// Set label

	//outObservation....

	outThereIsObservation = true;
}


/* -----------------------------------------------------
				doProcess
----------------------------------------------------- */
void  CNationalInstrumentsDAQ::doProcess()
{
	bool	thereIs, hwError;

	if (!m_nextObservation)
		m_nextObservation =  CObservationRawDAQ::Create();

	readFromDAQ( thereIs, *m_nextObservation, hwError );

	if (hwError)
	{
		m_state = ssError;
	    THROW_EXCEPTION("Couldn't start DAQ task!");
	}

	if (thereIs)
	{
		m_state = ssWorking;
		appendObservation( m_nextObservation );
		m_nextObservation.clear_unique(); // Create a new object in the next call
	}
}


/* -----------------------------------------------------
				grabbing_thread
----------------------------------------------------- */
void CNationalInstrumentsDAQ::grabbing_thread(TInfoPerTask &ipt)
{
#if MRPT_HAS_NIDAQMXBASE
	try 
	{
		TaskHandle  &taskHandle= *reinterpret_cast<TaskHandle*>(&ipt.taskHandle);

		const int pointsToRead = 1024; // samples per channel to read
		const float timeout = 10e-3;

		double buf[8*1024];

		while (!ipt.must_close)
		{
			int32  pointsReadPerChan;
			int err = DAQmxBaseReadAnalogF64(taskHandle,pointsToRead,timeout,DAQmx_Val_GroupByScanNumber,buf,sizeof(buf)/2,&pointsReadPerChan,NULL);
			if (err<0 && 
				err!=DAQmxErrorSamplesNotYetAvailable // That's not so bad...
				)
			{
				MRPT_DAQmx_ErrChk(err<0)
			}
			else
			{
				// Read without errors:
				cout << "[CNationalInstrumentsDAQ::grabbing_thread] " << pointsReadPerChan << " samples read.\n";
			}

		} // end of main thread loop
	}
	catch(std::exception &e)
	{
		std::cerr << "[CNationalInstrumentsDAQ::grabbing_thread] Exception:\n" << e.what() << std::endl;
	}
#endif //MRPT_HAS_NIDAQMXBASE
	
	ipt.is_closed = true;
}

