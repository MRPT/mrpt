/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

#ifndef CGPSInterface_H
#define CGPSInterface_H

#include <mrpt/slam/CObservationGPS.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/hwdrivers/CGenericSensor.h>

namespace mrpt
{
	namespace slam { class CObservationGPS; }

	namespace hwdrivers
	{
		/** A parser of NMEA commands, for connecting to a GPS by a serial port.
		  * This class also supports more advanced GPS equipped with RTK corrections. See the JAVAD/TopCon extra initialization parameters.
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    COM_port_WIN = COM3
		  *    COM_port_LIN = ttyS0
		  *    baudRate     = 4800   // The baudrate of the communications (typ. 4800 bauds)
		  *    pose_x       = 0      // 3D position of the sensed point relative to the robot (meters)
		  *    pose_y       = 0
		  *    pose_z       = 0
		  *    customInit   =       // See below for possible values
		  *
		  *	   // The next parameters are optional and will be used only
		  *    // if customInit=="JAVAD" to enable/configure the usage of RTK corrections:
		  *    //JAVAD_rtk_src_port=/dev/ser/b
		  *    //JAVAD_rtk_src_baud=9600
		  *    //JAVAD_rtk_format=cmr
		  *
		  *  \endcode
		  *
		  * - customInit: Custom commands to send, depending on the sensor. Valid values are:
		  *		- "": Empty string
		  *		- "JAVAD": JAVAD or TopCon devices. Extra initialization commands will be sent.
		  *		- "TopCon": A synonymous with "JAVAD".
		  *
		  *  VERSIONS HISTORY:
		  *		-9/JUN/2006: First version (JLBC)
		  *		-4/JUN/2008: Added virtual methods for device-specific initialization commands.
		  *		-10/JUN/2008: Converted into CGenericSensor class (there are no inhirited classes anymore).
		  * \ingroup mrpt_hwdrivers_grp
		*/
		class HWDRIVERS_IMPEXP CGPSInterface : public utils::CDebugOutputCapable, public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CGPSInterface)

		public:
			/** Constructor
			  * \param BUFFER_LENGTH The size of the communications buffer (default value should be fine always)
			  */
//			CGPSInterface( int BUFFER_LENGTH = 500 );

			// MAR'11 -------------------------------------
			CGPSInterface( int BUFFER_LENGTH = 500, mrpt::hwdrivers::CSerialPort *outPort = NULL, mrpt::synch::CCriticalSection *csOutPort = NULL);
            // --------------------------------------------

			/** Destructor
			  */
			virtual ~CGPSInterface();

			/** This method should be called periodically (at least at 1Hz to capture ALL the real-time data)
			*  It is thread safe, i.e. you can call this from one thread, then to other methods from other threads.
			*  This method processes data from the GPS and update the object state accordingly.
			*/
			void  doProcess();

			/** Returns true if communications work.
			*/
			bool  isGPS_connected();

			/** Returns true if the last message from the GPS indicates that the signal from sats has been acquired.
			*/
			bool  isGPS_signalAcquired();

			void  setSerialPortName(const std::string &COM_port);  //!< Set the serial port to use (COM1, ttyUSB0, etc).
			std::string getSerialPortName() const;  //!< Get the serial port to use (COM1, ttyUSB0, etc).

			inline void setExternCOM( CSerialPort *outPort, mrpt::synch::CCriticalSection *csOutPort )
			{ m_out_COM = outPort; m_cs_out_COM = csOutPort; }

			inline bool isAIMConfigured() { return m_AIMConfigured; }

		protected:
			/** Implements custom messages to be sent to the GPS unit just after connection and before normal use.
			  *  Returns false or raise an exception if something goes wrong.
			  */
			bool OnConnectionEstablished();

			CSerialPort		m_COM;

            // MAR'11 -------------------------------------
			CSerialPort		                *m_out_COM;
            mrpt::synch::CCriticalSection   *m_cs_out_COM;
			// --------------------------------------------

			poses::CPoint3D	m_sensorPose;

			std::string		m_customInit;

			/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
			  *  See hwdrivers::CGPSInterface for the possible parameters
			  */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

			/** If not empty, will send a cmd "set,/par/pos/pd/port,...". Example value: "/dev/ser/b" */
			void setJAVAD_rtk_src_port( const std::string &s) { m_JAVAD_rtk_src_port = s; }

			/** Only used when "m_JAVAD_rtk_src_port" is not empty */
			void setJAVAD_rtk_src_baud(unsigned int baud) { m_JAVAD_rtk_src_baud = baud; }

			/** Only used when "m_JAVAD_rtk_src_port" is not empty: format of RTK corrections: "cmr", "rtcm", "rtcm3", etc. */
			void setJAVAD_rtk_format(const std::string &s) {m_JAVAD_rtk_format=s;}

			/** Set Advanced Input Mode for the primary port.
                This can be used to send RTK corrections to the device using the same port that it's used for the commands.
                The RTK correction stream must be re-packaged into a special frame with prefix ">>" */
            bool setJAVAD_AIM_mode();

			/** Unset Advanced Input Mode for the primary port and use it only as a command port. */
            bool unsetJAVAD_AIM_mode();

            // MAR'11 -------------------------------------
            inline bool useExternCOM() { return (m_out_COM!=NULL); }
            // --------------------------------------------

		private:
			std::string 	m_COMname;
			int				m_COMbauds;
			bool			m_GPS_comsWork;
			bool			m_GPS_signalAcquired;
			int				m_BUFFER_LENGTH;

			char			*m_buffer;
			size_t			m_bufferLength;
			size_t			m_bufferWritePos;

			std::string		m_JAVAD_rtk_src_port; 	//!< If not empty, will send a cmd "set,/par/pos/pd/port,...". Example value: "/dev/ser/b"
			unsigned int	m_JAVAD_rtk_src_baud; 	//!< Only used when "m_JAVAD_rtk_src_port" is not empty
			std::string		m_JAVAD_rtk_format; 	//!< Only used when "m_JAVAD_rtk_src_port" is not empty: format of RTK corrections: "cmr", "rtcm", "rtcm3", etc.

			// MAR'11 -----------------------------------------
			bool            m_useAIMMode;           //!< Use this mode for receive RTK corrections from a external source through the primary port
            // ------------------------------------------------
			TTimeStamp      m_last_timestamp;

			// MAR'11 -----------------------------------------
			bool            m_AIMConfigured;        //!< Indicates if the AIM has been properly set up.
			double          m_data_period;          //!< The period in seconds which the data should be provided by the GPS
            // ------------------------------------------------

			/** Returns true if the COM port is already open, or try to open it in other case.
			  * \return true if everything goes OK, or false if there are problems opening the port.
			  */
			bool  tryToOpenTheCOM();

			/** Process data in "m_buffer" to extract GPS messages, and remove them from the buffer.
			  */
			void  processBuffer();

			/** Process a complete string from the GPS:
			  */
			void  processGPSstring( const std::string &s);

			/** Tokenize a string "str" into commas separated tokens
			  */
			void  getNextToken(
				const std::string	&str,
				std::string			&token,
				unsigned int		&parserPos);

			/* A private copy of the last received gps datum:
			 */
			mrpt::slam::CObservationGPS	            m_latestGPS_data;
			mrpt::slam::CObservationGPS::TUTCTime   m_last_UTC_time;

			void JAVAD_sendMessage(const char*str, bool waitForAnswer = true); //!< Private auxiliary method. Raises exception on error.

		}; // end class

	} // end namespace
} // end namespace

#endif
