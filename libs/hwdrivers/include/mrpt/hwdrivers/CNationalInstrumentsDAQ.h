/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CNationalInstrumentsDAQ_H
#define CNationalInstrumentsDAQ_H

#include <mrpt/obs/CObservationRawDAQ.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/synch/CPipe.h>
#include <mrpt/system/threads.h>
#include <list>
#include <memory>

namespace mrpt
{
	namespace hwdrivers
	{
		/** An interface to read from data acquisition boards compatible with National Instruments "DAQmx Base" or "DAQmx".
		* Refer to DAQmx Base C API reference online to learn more on the concepts of "channels", "tasks" (which in this MRPT class 
		*  are mapped to independent grabbing threads), etc. 
		* If both DAQmx and DAQmxBase are installed in the system, DAQmx will be used. This class API isolate the user from the usage of one or another specific library.
		*
		*  This class can be used as a sensor from the application "rawlog-grabber", or directly as a C++ class from a user program.
		*  Refer to the example:  [MRPT]/samples/NIDAQ_test
		*
		*  Samples will be returned inside mrpt::obs::CObservationRawDAQ in "packets" of a predefined number of samples, which can 
		*  be changed by the user through the "samplesPerChannelToRead" parameter of each task.
		*
		*  For multichannels tasks, samples will be **interleaved**. For example, the readings from succesive timesteps for 4 ADC channels 
		*  will be available in the ADC vector inside mrpt::obs::CObservationRawDAQ in this order:
		*
		*   - A0[0] A1[0] A2[0] A3[0]  A0[1] A1[1] A2[1] A3[1]  A0[2] A1[2] A2[2] A3[2] ...
		*
		*  The sensor label (field "m_sensorLabel") of each grabbed observation will be the concatenation of this class sensor label,
		*  a dot (".") and the task label (default="task###", with ### the task index).
		*
		*  \code
		*  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		* -------------------------------------------------------
		*   [supplied_section_name]
		* ; Number of tasks (each will run in a thread). Task indices are 0-based.
		* ; (Parameters below follow NIs DAQmx API notation)
		* num_tasks  = 1
		* 
		* ; Channels, separated by commas if more than one.
		* ;  - "ai": Analog inputs
		* ;  - "ao": Analog outputs
		* ;  - "di": Digital inputs
		* ;  - "do": Digital inputs
		* ;  - "ci_period", 
		* ;    "ci_count_edges", "ci_pulse_width",
		* ;    "ci_lin_encoder", "ci_ang_encoder" : Counters & encoders (WARNING: NI says "a task can include only one counter input channel")
		* ;  - "co_pulses": Output digital pulses (WARNING: NI says "a task can include only one counter output channel")
		* ;
		* task0.channels = ai  //, ao, di, do, ci_ang_encoder
		* ;task0.taskLabel= MY_LABEL     // Optional textual label to build the mrpt::obs::CObservation sensor label (default: task number)
		* task0.samplesPerSecond = 1000 // Samples per second. Continuous (infinite) sampling is assumed.
		* task0.samplesPerChannelToRead = 1000  // The number of samples to grab at once from each channel.
		* ;task0.bufferSamplesPerChannel = 200000 // Increase if you have errors about " Onboard device memory overflow.(...)"
		* 
		* ; Analog input channel params. 
		* task0.ai.physicalChannel = Dev1/ai0:3, Dev1/ai6
		* task0.ai.physicalChannelCount = 5  // *IMPORTANT* This must be the total number of channels listed in "physicalChannel" (e.g. 4 for "Dev1/ai0:3")
		* task0.ai.terminalConfig  = DAQmx_Val_Cfg_Default | DAQmx_Val_RSE | DAQmx_Val_NRSE | DAQmx_Val_Diff   // One of these strings
		* task0.ai.minVal          = -10.0    // Volts
		* task0.ai.maxVal          =  10.0    // Volts
		* 
		* ; Analog output channel params.
		* task0.ao.physicalChannel = Dev1/ao0, Dev1/ao2:4
		* task0.ao.physicalChannelCount = 4  // *IMPORTANT* This must be the total number of channels listed in "physicalChannel" (e.g. 1 for "Dev1/ao0")
		* task0.ao.minVal          = -10.0    // Volts
		* task0.ao.maxVal          =  10.0    // Volts
		* 
		* ; Digital input channel params.
		* task0.di.line           = Dev1/port1/line0
		* 
		* ; Digital input channel params.
		* task0.do.line           = Dev1/port1/line2
		* 
		* ; Counter: period of a digital signal
		* task0.ci_period.counter  = Dev1/ctr0
		* task0.ci_period.minVal   = 0   // The minimum value, in units, that you expect to measure.
		* task0.ci_period.maxVal   = 0   // The minimum value, in units, that you expect to measure.
		* task0.ci_period.units    = DAQmx_Val_Seconds | DAQmx_Val_Ticks  // One of these strings
		* task0.ci_period.edge     = DAQmx_Val_Rising | DAQmx_Val_Falling // One of these strings
		* task0.ci_period.measTime = 0   // NI says: "Always pass 0 for this parameter."
		* task0.ci_period.divisor  = 1   // NI says: "Always pass 1 for this parameter."
		* 
		* ; Counter: count the number of rising or falling edges of a digital signal 
		* task0.ci_count_edges.counter        = Dev1/ctr0
		* task0.ci_count_edges.edge           = DAQmx_Val_Rising | DAQmx_Val_Falling // One of these strings
		* task0.ci_count_edges.initialCount   = 0    // The value from which to start counting
		* task0.ci_count_edges.countDirection = DAQmx_Val_CountUp | DAQmx_Val_CountDown | DAQmx_Val_ExtControlled  // One of these strings
		* 
		* ; Counter:  measure the width of a digital pulse
		* task0.ci_pulse_width.counter      = Dev1/ctr0
		* task0.ci_pulse_width.minVal       = 0   // The minimum value, in units, that you expect to measure.
		* task0.ci_pulse_width.maxVal       = 0   // The minimum value, in units, that you expect to measure.
		* task0.ci_pulse_width.units        = DAQmx_Val_Seconds | DAQmx_Val_Ticks  // One of these strings
		* task0.ci_pulse_width.startingEdge = DAQmx_Val_Rising | DAQmx_Val_Falling // One of these strings
		* 
		* ; Counter:  uses a linear encoder to measure linear position
		* task0.ci_lin_encoder.counter      = Dev1/ctr0
		* task0.ci_lin_encoder.decodingType = DAQmx_Val_X1 | DAQmx_Val_X2 | DAQmx_Val_X4 | DAQmx_Val_TwoPulseCounting // One of these strings
		* task0.ci_lin_encoder.ZidxEnable   = false | true | 0 | 1    //  enable z indexing?
		* task0.ci_lin_encoder.ZidxVal      = 0 // The value, in units, to which to reset the measurement when signal Z is high and signal A and signal B are at the states you specify with ZidxPhase.
		* task0.ci_lin_encoder.ZidxPhase    = DAQmx_Val_AHighBHigh | DAQmx_Val_AHighBLow | DAQmx_Val_ALowBHigh | DAQmx_Val_ALowBLow  // One of these strings
		* task0.ci_lin_encoder.units        = DAQmx_Val_Meters | DAQmx_Val_Inches | DAQmx_Val_Ticks  // One of these strings
		* task0.ci_lin_encoder.distPerPulse = 0.1  // The distance measured for each pulse the encoder generates. Specify this value in units.
		* task0.ci_lin_encoder.initialPos   = 0.0 // The position of the encoder when the measurement begins. This value is in units.
		* 
		* ; Counter:  uses an angular encoder to measure angular position
		* task0.ci_ang_encoder.counter      = Dev1/ctr0
		* task0.ci_ang_encoder.decodingType = DAQmx_Val_X1 | DAQmx_Val_X2 | DAQmx_Val_X4 | DAQmx_Val_TwoPulseCounting // One of these strings
		* task0.ci_ang_encoder.ZidxEnable   = 0 | 1 | false | true  //  enable z indexing
		* task0.ci_ang_encoder.ZidxVal      = 0 // The value, in units, to which to reset the measurement when signal Z is high and signal A and signal B are at the states you specify with ZidxPhase.
		* task0.ci_ang_encoder.ZidxPhase    = DAQmx_Val_AHighBHigh | DAQmx_Val_AHighBLow | DAQmx_Val_ALowBHigh | DAQmx_Val_ALowBLow  // One of these strings
		* task0.ci_ang_encoder.units        = DAQmx_Val_Degrees | DAQmx_Val_Radians | DAQmx_Val_Ticks  // One of these strings
		* task0.ci_ang_encoder.pulsesPerRev = 512  // The number of pulses the encoder generates per revolution. 
		* task0.ci_ang_encoder.initialAngle = 0.0 // The position of the encoder when the measurement begins. This value is in units.
		* task0.ci_ang_encoder.decimate     = 1   // Grab 1 out of N readings
		*
		* ; Output digital pulses:
		* task0.co_pulses.counter           = Dev1/ctr1
		* task0.co_pulses.idleState         = DAQmx_Val_High | DAQmx_Val_Low
		* task0.co_pulses.initialDelay      = 0  // The amount of time in seconds to wait before generating the first pulse.
		* task0.co_pulses.freq              = 100 // The frequency of the pulses to generate (Hertz)
		* task0.co_pulses.dutyCycle         = 0.5  // The width of the pulse divided by the pulse period.
		*  \endcode
		*
		* See also: 
		*  - [MRPT]/samples/NIDAQ_test 
		*  - Sample .ini files for rawlog-grabber in [MRPT]/share/mrpt/config_files/rawlog-grabber/
		*  - NI DAQmx C reference: http://others-help.mrpt.org/ni-daqmx_c_reference_help/
		*  - NI DAQmx Base 3.x C reference: http://others-help.mrpt.org/ni-daqmx_base_3.x_c_function_reference/
		*
		* DAQmx Base Installation
		* ------------------------
		* Go to http://ni.com and download the "DAQmx Base" package for your OS. Install following NI's instructions. 
		* As of 2013, the latest version is 3.7. 
		*
		* \note This class requires compiling MRPT with support for "NI DAQmx" or "NI DAQmx Base". While compiling MRPT,
		*        check the "MRPT_HAS_NI_DAQmx"/"MRPT_HAS_NI_DAQmxBASE" option and correctly set the new variables to
		*        the library include directory and library file.
		*
		* \note As of 2013, NI seems not to support compiling 64bit programs, so you can must build MRPT for 32bits if you need this class.
		*
		* \ingroup mrpt_hwdrivers_grp
		*/
		class HWDRIVERS_IMPEXP CNationalInstrumentsDAQ : public mrpt::utils::COutputLogger, public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CNationalInstrumentsDAQ)
		public:
			/** Constructor */
			CNationalInstrumentsDAQ();

			/** Destructor */
			virtual ~CNationalInstrumentsDAQ();

			/** Setup and launch the DAQ tasks, in parallel threads. 
			  * Access to grabbed data with CNationalInstrumentsDAQ::readFromDAQ() or the standard CGenericSensor::doProcess() */
			virtual void initialize();

			/** Stop the grabbing threads for DAQ tasks. It is automatically called at destruction. */
			void stop();

			// See docs in parent class
			void  doProcess();

			/** Receives data from the DAQ thread(s). It returns a maximum number of one observation object per running grabber threads, that is, per each DAQmx "task".
			  *  This method MUST BE CALLED in a timely fashion by the user to allow the proccessing of incoming data. It can be run in a different thread safely.
			  *  This is internally called when using the alternative CGenericSensor::doProcess() interface.
			  *  No observations may be returned if there are not samples enough yet from any task.
			  */
			void  readFromDAQ(
				std::vector<mrpt::obs::CObservationRawDAQPtr> &outObservations,
				bool & hardwareError );

			/** Set voltage outputs to all the outputs in an AOUT task 
			  * For the meaning of parameters, refere to NI DAQmx docs for DAQmxBaseWriteAnalogF64()
			  * \note The number of samples in \a volt_values must match the number of channels in the task when it was initiated.
			  */
			void writeAnalogOutputTask(size_t task_index, size_t nSamplesPerChannel, const double * volt_values, double timeout, bool groupedByChannel);

			/** Changes the boolean state of one digital output line.
			  * For the meaning of parameters, refere to NI DAQmx docs for DAQmxBaseWriteAnalogF64()
			  * \note The number of samples in \a volt_values must match the number of channels in the task when it was initiated.
			  */
			void writeDigitalOutputTask(size_t task_index, bool line_value, double timeout);

			/** Returns true if initialize() was called and at least one task is running. */
			bool checkDAQIsWorking() const;


			/** Each of the tasks to create in CNationalInstrumentsDAQ::initialize(). 
			  * Refer to the docs on config file formats of mrpt::hwdrivers::CNationalInstrumentsDAQ to learn on the meaning 
			  * of each field. Also, see National Instruments' DAQmx API docs online.
			  */
			struct HWDRIVERS_IMPEXP TaskDescription
			{
				TaskDescription();

				bool has_ai, has_ao, has_di, has_do;
				bool has_ci_period, has_ci_count_edges,has_ci_pulse_width,has_ci_lin_encoder,has_ci_ang_encoder, has_co_pulses;


                double   samplesPerSecond;   //!< Sample clock config: samples per second. Continuous (infinite) sampling is assumed.
                std::string sampleClkSource; //!< Sample clock source: may be empty (default value) for some channels.
				uint32_t bufferSamplesPerChannel; //!< (Default=0) From NI's docs: The number of samples the buffer can hold for each channel in the task. Zero indicates no buffer should be allocated. Use a buffer size of 0 to perform a hardware-timed operation without using a buffer.
				uint32_t samplesPerChannelToRead; //!< (Default=1000) The number of samples to grab at once from each channel.
                std::string taskLabel;            //!< (Default="task###")

				struct HWDRIVERS_IMPEXP desc_ai_t
				{
					desc_ai_t() : terminalConfig("DAQmx_Val_NRSE"),minVal(-10), maxVal(10),physicalChannelCount(0) { }

					std::string physicalChannel, terminalConfig;
					double minVal, maxVal;
					unsigned int physicalChannelCount; //!< *IMPORTANT* This must be the total number of channels listed in "physicalChannel" (e.g. 4 for "Dev1/ai0:3")
				} 
				ai; //!< Analog inputs
				
				struct HWDRIVERS_IMPEXP desc_ao_t
				{
					desc_ao_t() : physicalChannelCount(0),minVal(-10), maxVal(10) { }

					std::string physicalChannel;
					unsigned int physicalChannelCount; //!< *IMPORTANT* This must be the total number of channels listed in "physicalChannel" (e.g. 1 for "Dev1/ao0")
					double minVal, maxVal;
				} 
				ao; //!< Analog outputs

				struct HWDRIVERS_IMPEXP desc_di_t
				{
					std::string line;  //!< The digital line (for example "Dev1/port0/line1")
				} 
				di; //!< Digital inputs (di)

				struct HWDRIVERS_IMPEXP desc_do_t
				{
					std::string line;   //!< The digital line (for example "Dev1/port0/line1")
				} 
				douts; //!< Digital outs (do)

				struct HWDRIVERS_IMPEXP desc_ci_period_t
				{
					desc_ci_period_t() : minVal(0),maxVal(0),measTime(0),divisor(1) { }

					std::string counter, units, edge;
					double      minVal,maxVal; 
					double      measTime;
					int         divisor; 
				} 
				ci_period; //!< Counter: period of a digital signal

				struct HWDRIVERS_IMPEXP desc_ci_count_edges_t
				{
					desc_ci_count_edges_t() : countDirection("DAQmx_Val_CountUp"),initialCount(0) { }

					std::string counter, edge, countDirection;
					int         initialCount; 
				} 
				ci_count_edges; //!< Counter: period of a digital signal

				struct HWDRIVERS_IMPEXP desc_ci_pulse_width_t
				{
					desc_ci_pulse_width_t() : minVal(0),maxVal(0) { }

					std::string counter, units, startingEdge;
					double      minVal,maxVal; 
				} 
				ci_pulse_width; //!< Counter: measure the width of a digital pulse

				struct HWDRIVERS_IMPEXP desc_ci_lin_encoder_t
				{
					desc_ci_lin_encoder_t() : ZidxEnable(false),ZidxVal(0),distPerPulse(0.1),initialPos(0) { }

					std::string counter, decodingType, ZidxPhase,units;
					bool        ZidxEnable;
					double      ZidxVal;
					double      distPerPulse;
					double      initialPos;
				} 
				ci_lin_encoder; //!< Counter: uses a linear encoder to measure linear position

				struct HWDRIVERS_IMPEXP desc_ci_ang_encoder_t
				{
					desc_ci_ang_encoder_t() : ZidxEnable(false),ZidxVal(0),pulsesPerRev(512),initialAngle(0),decimate(1),decimate_cnt(0) { }

					std::string counter, decodingType, ZidxPhase,units;
					bool        ZidxEnable;
					double      ZidxVal;
					int         pulsesPerRev;
					double      initialAngle;
					int         decimate, decimate_cnt; 
				} 
				ci_ang_encoder; //!< Counter: uses an angular encoder to measure angular position

				struct HWDRIVERS_IMPEXP desc_co_pulses_t
				{
					desc_co_pulses_t() : idleState("DAQmx_Val_Low"),initialDelay(0),freq(1000),dutyCycle(0.5) { }

					std::string counter, idleState;
					double      initialDelay,freq,dutyCycle;
				} 
				co_pulses; //!< Output counter: digital pulses output

			}; // end of TaskDescription
			
			/** Publicly accessible vector with the list of tasks to be launched upon call to CNationalInstrumentsDAQ::initialize().
			  * Changing these while running will have no effects.
			  */
			std::vector<TaskDescription>  task_definitions; 
			
		protected:
			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		private:
			std::vector<mrpt::obs::CObservationRawDAQPtr> m_nextObservations; //!< A buffer for doProcess

			struct TInfoPerTask
			{
				TInfoPerTask();
				TInfoPerTask(const TInfoPerTask &o); //!< Copy ctor (needed for the auto_ptr semantics)

				void * taskHandle;
				mrpt::system::TThreadHandle hThread;
#if MRPT_HAS_CXX11
				std::unique_ptr<mrpt::synch::CPipeReadEndPoint> read_pipe;
				std::unique_ptr<mrpt::synch::CPipeWriteEndPoint> write_pipe;
#else
				std::auto_ptr<mrpt::synch::CPipeReadEndPoint> read_pipe;
				std::auto_ptr<mrpt::synch::CPipeWriteEndPoint> write_pipe;
#endif
				bool must_close, is_closed;
				mrpt::synch::CAtomicCounter  new_obs_available;

				TaskDescription task; //!< A copy of the original task description that generated this thread.
			};

			std::list<TInfoPerTask> m_running_tasks;

			/** Method to be executed in each parallel thread. */
			void grabbing_thread(TInfoPerTask &ipt);
			
		}; // end class

	} // end namespace
} // end namespace

#endif
