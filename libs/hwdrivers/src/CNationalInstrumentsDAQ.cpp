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
	is_closed(false),
	task()
{ }

// Copy ctor (needed for the auto_ptr semantics)
CNationalInstrumentsDAQ::TInfoPerTask::TInfoPerTask(const TInfoPerTask &o) :
	taskHandle(o.taskHandle),
	hThread(o.hThread),
	read_pipe(o.read_pipe.get()),
	write_pipe(o.write_pipe.get()),
	must_close(o.must_close),
	is_closed(o.is_closed),
	task(o.task)
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
	//std::vector<TaskDescription>  task_definitions; 
	task_definitions.clear();

	const unsigned int nTasks = configSource.read_uint64_t(iniSection, "num_tasks", 0, true );
	if (!nTasks) {
		std::cerr << "[CNationalInstrumentsDAQ] Warning: Number of tasks is zero. No datalogging will be done.\n";
	}

	task_definitions.resize(nTasks);
	for (unsigned int i=0;i<nTasks;i++)
	{
		TaskDescription & t = task_definitions[i];
		const string sTask = mrpt::format("task%u",i);
		
		MRPT_TODO("XXX")


	} // end for i
	
}

/* -----------------------------------------------------
                Destructor
   ----------------------------------------------------- */
CNationalInstrumentsDAQ::~CNationalInstrumentsDAQ()
{
	this->stop();
}

#if MRPT_HAS_NIDAQMXBASE
// Declare a table to convert strings to their DAQmx #define values:
struct daqmx_str_val {
	const char* str;
	int val;
};

const daqmx_str_val daqmx_vals[] = {
	{ "DAQmx_Val_Cfg_Default", DAQmx_Val_Cfg_Default },
	{ "DAQmx_Val_RSE", DAQmx_Val_RSE },
	{ "DAQmx_Val_NRSE", DAQmx_Val_NRSE },
	{ "DAQmx_Val_Diff", DAQmx_Val_Diff },
	{ "DAQmx_Val_Seconds", DAQmx_Val_Seconds },
	{ "DAQmx_Val_Rising", DAQmx_Val_Rising },
	{ "DAQmx_Val_Falling", DAQmx_Val_Falling },
	{ "DAQmx_Val_CountUp", DAQmx_Val_CountUp },
	{ "DAQmx_Val_CountDown", DAQmx_Val_CountDown },
	{ "DAQmx_Val_ExtControlled", DAQmx_Val_ExtControlled },
	{ "DAQmx_Val_AHighBHigh", DAQmx_Val_AHighBHigh },
	{ "DAQmx_Val_AHighBLow", DAQmx_Val_AHighBLow },
	{ "DAQmx_Val_ALowBHigh", DAQmx_Val_ALowBHigh },
	{ "DAQmx_Val_ALowBLow", DAQmx_Val_ALowBLow },
	{ "DAQmx_Val_Meters", DAQmx_Val_Meters },
	{ "DAQmx_Val_Inches", DAQmx_Val_Inches },
	{ "DAQmx_Val_Ticks", DAQmx_Val_Ticks },
	{ "DAQmx_Val_Degrees", DAQmx_Val_Degrees },
	{ "DAQmx_Val_Radians", DAQmx_Val_Radians }
};

int daqmx_defstr2num(const std::string &str)
{
	const std::string s = mrpt::utils::trim(str);

	for (unsigned int i=0;i<sizeof(daqmx_vals)/sizeof(daqmx_vals[0]);i++)
	{
		if (!strcmpi(daqmx_vals[i].str,s.c_str()))
			return daqmx_vals[i].val;
	}
	THROW_EXCEPTION_CUSTOM_MSG1("Error: Unknown DAQmxBase constant: %s",s.c_str())
}
#endif

/* -----------------------------------------------------
				initialize
----------------------------------------------------- */
void  CNationalInstrumentsDAQ::initialize()
{
#if MRPT_HAS_NIDAQMXBASE
	this->stop();

	for (size_t i=0;i<task_definitions.size();i++)
	{
		const TaskDescription & tf = task_definitions[i];

		// Try to create a new task:
		m_running_tasks.push_back(TInfoPerTask());
		TInfoPerTask &ipt = m_running_tasks.back();
		ipt.task = tf;  // Save a copy of the task info for the thread to have all the needed info

		try
		{
			TaskHandle  &taskHandle= *reinterpret_cast<TaskHandle*>(&ipt.taskHandle);

			MRPT_DAQmx_ErrChk (DAQmxBaseCreateTask("",&taskHandle));

			if (tf.has_ai) {
				ASSERTMSG_(tf.ai.physicalChannelCount>0, "ai.physicalChannelCount is zero! Please, define it correctly.")

				MRPT_DAQmx_ErrChk(DAQmxBaseCreateAIVoltageChan(taskHandle,
					tf.ai.physicalChannel.c_str(),NULL,
					daqmx_defstr2num(tf.ai.terminalConfig),
					tf.ai.minVal, tf.ai.maxVal,DAQmx_Val_Volts,NULL));
			}
			if (tf.has_ao) {
				ASSERTMSG_(tf.ao.physicalChannelCount>0, "ai.physicalChannelCount is zero! Please, define it correctly.")

				MRPT_DAQmx_ErrChk(DAQmxBaseCreateAOVoltageChan(taskHandle,
					tf.ao.physicalChannel.c_str(),NULL,
					tf.ao.minVal, tf.ao.maxVal,DAQmx_Val_Volts,NULL));
			}
			if (tf.has_di) {
				MRPT_DAQmx_ErrChk(DAQmxBaseCreateDIChan(taskHandle,
					tf.di.lines.c_str(),NULL,DAQmx_Val_ChanForAllLines));
			}
			if (tf.has_do) {
				MRPT_DAQmx_ErrChk(DAQmxBaseCreateDOChan(taskHandle,
					tf.douts.lines.c_str(),NULL,DAQmx_Val_ChanForAllLines));
			}
			if (tf.has_ci_period) {
				MRPT_DAQmx_ErrChk(DAQmxBaseCreateCIPeriodChan(taskHandle,
					tf.ci_period.counter.c_str(),NULL,
					tf.ci_period.minVal, tf.ci_period.maxVal,
					daqmx_defstr2num(tf.ci_period.units),
					daqmx_defstr2num(tf.ci_period.edge),
					DAQmx_Val_LowFreq1Ctr,
					tf.ci_period.measTime,
					tf.ci_period.divisor,NULL));
			}
			if (tf.has_ci_count_edges) {
				MRPT_DAQmx_ErrChk(DAQmxBaseCreateCICountEdgesChan(taskHandle,
					tf.ci_count_edges.counter.c_str(),NULL,
					daqmx_defstr2num(tf.ci_count_edges.edge),
					tf.ci_count_edges.initialCount,
					daqmx_defstr2num(tf.ci_count_edges.countDirection)));
			}
			if (tf.has_ci_pulse_width) {
				MRPT_DAQmx_ErrChk(DAQmxBaseCreateCIPulseWidthChan(taskHandle,
					tf.ci_pulse_width.counter.c_str(),NULL,
					tf.ci_pulse_width.minVal, tf.ci_pulse_width.maxVal,
					daqmx_defstr2num(tf.ci_pulse_width.units),
					daqmx_defstr2num(tf.ci_pulse_width.startingEdge),
					NULL));
			}
			if (tf.has_ci_lin_encoder) {
				MRPT_DAQmx_ErrChk(DAQmxBaseCreateCILinEncoderChan(taskHandle,
					tf.ci_lin_encoder.counter.c_str(),NULL,
					daqmx_defstr2num(tf.ci_lin_encoder.decodingType),
					tf.ci_lin_encoder.ZidxEnable,
					tf.ci_lin_encoder.ZidxVal,
					daqmx_defstr2num(tf.ci_lin_encoder.ZidxPhase),
					daqmx_defstr2num(tf.ci_lin_encoder.units),
					tf.ci_lin_encoder.distPerPulse, 
					tf.ci_lin_encoder.initialPos,
					NULL));
			}
			if (tf.has_ci_ang_encoder) {
				MRPT_DAQmx_ErrChk(DAQmxBaseCreateCIAngEncoderChan(taskHandle,
					tf.ci_ang_encoder.counter.c_str(),NULL,
					daqmx_defstr2num(tf.ci_ang_encoder.decodingType),
					tf.ci_ang_encoder.ZidxEnable,
					tf.ci_ang_encoder.ZidxVal,
					daqmx_defstr2num(tf.ci_ang_encoder.ZidxPhase),
					daqmx_defstr2num(tf.ci_ang_encoder.units),
					tf.ci_ang_encoder.pulsesPerRev, 
					tf.ci_ang_encoder.initialAngle,
					NULL));
			}
			MRPT_TODO("Add: DAQmxBaseCreateCOPulseChanFreq")

			// sample rate:
			ASSERT_ABOVE_(tf.samplesPerSecond,0)
			MRPT_DAQmx_ErrChk (DAQmxBaseCfgSampClkTiming(taskHandle,"",tf.samplesPerSecond,DAQmx_Val_Rising, DAQmx_Val_ContSamps,0));

			// samples/ch buffer:
			MRPT_DAQmx_ErrChk (DAQmxBaseCfgInputBuffer(taskHandle,tf.bufferSamplesPerChannel));

			// Create pipe:
			mrpt::synch::CPipe::createPipe(ipt.read_pipe, ipt.write_pipe);

			MRPT_DAQmx_ErrChk (DAQmxBaseStartTask(taskHandle));

			ipt.hThread = mrpt::system::createThreadFromObjectMethodRef<CNationalInstrumentsDAQ,TInfoPerTask>(this, &CNationalInstrumentsDAQ::grabbing_thread, ipt);

		}
		catch (std::exception const &)
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

			std::cerr << "[CNationalInstrumentsDAQ] Error while creating tasks. Closing other tasks before returning...\n";
			this->stop();
			std::cerr << "[CNationalInstrumentsDAQ] Closing tasks done.\n";

			throw; // Rethrow
		}
	} // end for each task_definitions[i]

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
bool CNationalInstrumentsDAQ::checkDAQIsWorking() const
{
	return (!m_running_tasks.empty() && !m_running_tasks.begin()->is_closed);
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

	if ( !checkDAQIsWorking() )
	{
		hardwareError = true;
		return;
	}

	// Read from the pipe:
    m_state = ssWorking;

	for (list<TInfoPerTask>::iterator it=m_running_tasks.begin();it!=m_running_tasks.end();++it)
	{
		it->read_pipe->ReadObject(&outObservation);
		outThereIsObservation = true;
		return;
		MRPT_TODO("Multiple objs!")
	}

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

		const float timeout = 1.5*ipt.task.samplesPerChannelToRead/ipt.task.samplesPerSecond;

		int err=0;
		vector<double> dBuf;

		const mrpt::slam::CObservationRawDAQ clean_obs;
		mrpt::slam::CObservationRawDAQ obs;

		while (!ipt.must_close)
		{
			obs = clean_obs; // Start with an empty observation

			// Common stuff:
			obs.timestamp = mrpt::system::now();
			obs.sample_rate = ipt.task.samplesPerSecond;
			obs.sensorLabel = m_sensorLabel;

			bool there_are_data = false; // At least one channel?

			// Read from each channel in this task:
			// -----------------------------------------------
			if (ipt.task.has_ai) 
			{
				obs.AIN_channel_count = ipt.task.ai.physicalChannelCount;
				obs.AIN_interleaved = true;

				const int32 totalSamplesToRead = ipt.task.ai.physicalChannelCount * ipt.task.samplesPerChannelToRead;
				dBuf.resize(totalSamplesToRead);
				int32  pointsReadPerChan;

				if ((err = DAQmxBaseReadAnalogF64(
					taskHandle,
					ipt.task.samplesPerChannelToRead,timeout, obs.AIN_interleaved ? DAQmx_Val_GroupByScanNumber : DAQmx_Val_GroupByChannel,
					&dBuf[0],dBuf.size(),
					&pointsReadPerChan,NULL))<0 && err!=DAQmxErrorSamplesNotYetAvailable) 
				{
					MRPT_DAQmx_ErrChk(err)
				}
				else if (pointsReadPerChan>0)
				{
					ASSERT_EQUAL_(totalSamplesToRead,pointsReadPerChan*ipt.task.ai.physicalChannelCount)
					obs.AIN_double = dBuf;

					there_are_data = true;
					//cout << "[CNationalInstrumentsDAQ::grabbing_thread] " << pointsReadPerChan << " samples read.\n";
				}
			} // end AI


			// Send the observation to the main thread:
			if (there_are_data)
			{
				ipt.write_pipe->WriteObject(&obs);
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



// Ctor:
CNationalInstrumentsDAQ::TaskDescription::TaskDescription() :
	has_ai(false), has_ao(false), has_di(false), has_do(false),
	has_ci_period(false), has_ci_count_edges(false), has_ci_pulse_width(false), has_ci_lin_encoder(false),has_ci_ang_encoder(false),
	samplesPerSecond(1000.0),
	bufferSamplesPerChannel(0),
	samplesPerChannelToRead(1000)
{
}