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

#ifndef CNationalInstrumentsDAQ_H
#define CNationalInstrumentsDAQ_H

#include <mrpt/slam/CObservationRawDAQ.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/synch/CPipe.h>
#include <mrpt/system/threads.h>

namespace mrpt
{
	namespace slam { class CObservationRawDAQ; }

	namespace hwdrivers
	{
		/** An interface to read from data acquisition boards compatible with National Instruments "DAQmx Base".
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    num_tasks  = 2
		  *    task0_ai_volt_channels    =  ai0, ai3:6     // Any NI-compliant sequence of channels
		  *    task1_ang_encoder_channel =  Dev1/ai3:6     // Any NI-compliant sequence of channels
		  *
		  *  \endcode
		  *
		  * See also: 
		  *  - [MRPT]samples/NIDAQ_test 
		  *  - Sample .ini files for rawlog-grabber in [MRPT]/share/mrpt/config_files/rawlog-grabber/
		  *
		  * DAQmx Base Installation
		  * ------------------------
		  * Go to http://ni.com and download the "DAQmx Base" package for your OS. Install following NI's instructions. 
		  * As of 2013, the latest version is 3.7 and these are the download links:
		  * - Windows: http://joule.ni.com/nidu/cds/view/p/id/4281/lang/en
		  * - Linux: http://joule.ni.com/nidu/cds/view/p/id/4269/lang/en
		  * - MacOS: http://joule.ni.com/nidu/cds/view/p/id/4272/lang/en
		  *
		  * While compiling MRPT, make sure that CMake detects "DAQmx Base" by setting the appropriate NI_* variables (may need to check "Advanced" variables in cmake-gui). 
		  *
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CNationalInstrumentsDAQ : public utils::CDebugOutputCapable, public CGenericSensor
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

			/** Receives data from the DAQ thread(s). It only returns a new observation, filled with samples, if there was new data waiting. 
			  *  This method MUST BE CALLED in a timely fashion by the user to allow the proccessing of incoming data. It can be run in a different thread safely.
			  *  This is internally called when using the alternative CGenericSensor::doProcess() interface.
			  */
			void  readFromDAQ(
				bool							&outThereIsObservation,
				mrpt::slam::CObservationRawDAQ	&outObservation,
				bool							&hardwareError );

			/** Returns true if initialize() was called successfully. */
			bool checkDAQIsConnected() const;
			
		protected:
			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		private:
			mrpt::slam::CObservationRawDAQPtr	m_nextObservation; //!< A dynamic object used as buffer in doProcess

			struct TInfoPerTask
			{
				TInfoPerTask();
				TInfoPerTask(const TInfoPerTask &o); //!< Copy ctor (needed for the auto_ptr semantics)

				void * taskHandle;
				mrpt::system::TThreadHandle hThread;
				std::auto_ptr<mrpt::synch::CPipeReadEndPoint> read_pipe;
				std::auto_ptr<mrpt::synch::CPipeWriteEndPoint> write_pipe;
				bool must_close, is_closed;
			};

			std::list<TInfoPerTask> m_running_tasks;

			/** Method to be executed in each parallel thread. */
			void grabbing_thread(TInfoPerTask &ipt);

			
		}; // end class

	} // end namespace
} // end namespace

#endif
