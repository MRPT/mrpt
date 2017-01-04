/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/utils/CClientTCPSocket.h>
#include <list>

using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::synch;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CGPSInterface,mrpt::hwdrivers)

struct TParsersRegistry
{
	std::list<CGPSInterface::ptr_parser_t> all_parsers;

	static const TParsersRegistry & getInstance()
	{
		static TParsersRegistry reg;
		return reg;
	}

private:
	TParsersRegistry()
	{
		all_parsers.push_back( &CGPSInterface::implement_parser_NMEA );
		all_parsers.push_back( &CGPSInterface::implement_parser_NOVATEL_OEM6 );
	}
};
	
/* -----------------------------------------------------
                Constructor
   ----------------------------------------------------- */
CGPSInterface::CGPSInterface() :
	mrpt::utils::COutputLogger("CGPSInterface"),
	m_data_stream(NULL),    // Typically a CSerialPort created by this class, but may be set externally.
	m_data_stream_cs(NULL),
	m_data_stream_is_external(false),
	m_customInit         (),
	m_rx_buffer          (0x10000),
	m_parser             (CGPSInterface::AUTO),
	m_raw_dump_file_prefix(),
	m_COMname            (),
	m_COMbauds           (4800),
	m_sensorLabelAppendMsgType (true),
	m_GPS_comsWork			(false),
	m_last_timestamp        ( INVALID_TIMESTAMP ),
	m_custom_cmds_delay   (0.1),
	m_custom_cmds_append_CRLF(true),

	m_JAVAD_rtk_src_port	(),
	m_JAVAD_rtk_src_baud	(0),
	m_JAVAD_rtk_format		("cmr"),
	m_topcon_useAIMMode            ( false ),
	m_topcon_AIMConfigured         ( false ),
	m_topcon_data_period           ( 0.2 ) // 20 Hz
{
	m_sensorLabel = "GPS";
}

/* -----------------------------------------------------
                loadConfig_sensorSpecific
   ----------------------------------------------------- */
void  CGPSInterface::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
	m_parser = configSource.read_enum<CGPSInterface::PARSERS>(iniSection,"parser",m_parser,false /*Allow default values*/);
	m_raw_dump_file_prefix = configSource.read_string(iniSection,"raw_dump_file_prefix",m_raw_dump_file_prefix,false /*Allow default values*/);

#ifdef MRPT_OS_WINDOWS
	m_COMname = configSource.read_string(iniSection, "COM_port_WIN", m_COMname, true );
#else
	m_COMname = configSource.read_string(iniSection, "COM_port_LIN", m_COMname, true );
#endif

	m_COMbauds		= configSource.read_int( iniSection, "baudRate",m_COMbauds, true );
	m_sensorLabelAppendMsgType  = configSource.read_bool(iniSection,"sensor_label_append_msg_type",m_sensorLabelAppendMsgType );

	// legacy custom cmds:
	m_customInit	= configSource.read_string( iniSection, "customInit", m_customInit, false );

	// new custom cmds:
	m_custom_cmds_delay = configSource.read_float( iniSection, "custom_cmds_delay",m_custom_cmds_delay );
	m_custom_cmds_append_CRLF = configSource.read_bool( iniSection, "custom_cmds_append_CRLF",m_custom_cmds_append_CRLF);
	// Load as many strings as found on the way:
	m_setup_cmds.clear();
	for (int i=1; true; i++)
	{
		std::string sLine = configSource.read_string(iniSection, mrpt::format("setup_cmd%i",i),std::string() ); 
		sLine = mrpt::system::trim( sLine );
		if (sLine.empty()) 
			break;
		m_setup_cmds.push_back(sLine);
	}

	m_shutdown_cmds.clear();
	for (int i=1; true; i++)
	{
		std::string sLine = configSource.read_string(iniSection, mrpt::format("shutdown_cmd%i",i),std::string() ); 
		sLine = mrpt::system::trim( sLine );
		if (sLine.empty()) 
			break;
		m_shutdown_cmds.push_back(sLine);
	}

	m_sensorPose.setFromValues(
		configSource.read_float(iniSection,"pose_x",0),
		configSource.read_float(iniSection,"pose_y",0),
		configSource.read_float(iniSection,"pose_z",0),
		DEG2RAD( configSource.read_float(iniSection,"pose_yaw",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_pitch",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_roll",0) )
		);

	m_JAVAD_rtk_src_port = configSource.read_string(iniSection, "JAVAD_rtk_src_port",m_JAVAD_rtk_src_port );
	m_JAVAD_rtk_src_baud = configSource.read_int(iniSection, "JAVAD_rtk_src_baud",m_JAVAD_rtk_src_baud );
	m_JAVAD_rtk_format   = configSource.read_string(iniSection,"JAVAD_rtk_format", m_JAVAD_rtk_format );

    m_topcon_useAIMMode = configSource.read_bool( iniSection,"JAVAD_useAIMMode", m_topcon_useAIMMode );
    m_topcon_data_period = 1.0/configSource.read_double( iniSection,"outputRate", m_topcon_data_period );
}

CGPSInterface::~CGPSInterface()
{
	OnConnectionShutdown();

	if (!m_data_stream_is_external)
	{
		delete m_data_stream;
		m_data_stream = NULL;
	}
}

void CGPSInterface::setParser(CGPSInterface::PARSERS parser) {
	m_parser = parser;
}
CGPSInterface::PARSERS CGPSInterface::getParser() const {
	return m_parser;
}
void CGPSInterface::bindStream(mrpt::utils::CStream * external_stream, mrpt::synch::CCriticalSection *csOptionalExternalStream)
{
	if (!m_data_stream_is_external) {
		delete m_data_stream;
		m_data_stream = NULL;
	}

	m_data_stream_is_external = true;
	m_data_stream = external_stream;
	m_data_stream_cs = csOptionalExternalStream;
}
void CGPSInterface::setSetupCommandsDelay(const double delay_secs) {
	m_custom_cmds_delay = delay_secs;
}
double CGPSInterface::getSetupCommandsDelay() const {
	return m_custom_cmds_delay;
}
void CGPSInterface::setSetupCommands(const std::vector<std::string> &cmds) {
	m_setup_cmds = cmds;
}
const std::vector<std::string> & CGPSInterface::getSetupCommands() const {
	return m_setup_cmds;
}
void CGPSInterface::setShutdownCommands(const std::vector<std::string> &cmds) {
	m_shutdown_cmds = cmds;
}
const std::vector<std::string> & CGPSInterface::getShutdownCommands() const {
	return m_shutdown_cmds;
}
void CGPSInterface::enableSetupCommandsAppendCRLF(const bool enable) {
	m_custom_cmds_append_CRLF = enable;
}
bool CGPSInterface::isEnabledSetupCommandsAppendCRLF() const {
	return m_custom_cmds_append_CRLF;
}

/* -----------------------------------------------------
				setSerialPortName
----------------------------------------------------- */
void  CGPSInterface::setSerialPortName(const std::string &COM_port)
{
	// Dont allow changing the serial port if:
	if (m_data_stream_is_external)
		THROW_EXCEPTION("Cannot change serial port name: an external stream has been already bound manually.")

	if (m_data_stream)
	{
		CCriticalSectionLocker lock(m_data_stream_cs);
		CSerialPort *serial = dynamic_cast<CSerialPort*>(m_data_stream);
		if (serial && serial->isOpen())
			THROW_EXCEPTION("Cannot change serial port name when it is already open")
	}

	// OK:
	m_COMname = COM_port;
}

/* -----------------------------------------------------
				getSerialPortName
----------------------------------------------------- */
std::string CGPSInterface::getSerialPortName() const
{
	return m_COMname;
}

/* -----------------------------------------------------
				tryToOpenTheCOM
----------------------------------------------------- */
bool  CGPSInterface::tryToOpenTheCOM()
{
	// If this is the first use of the COM port, create it:
	if (!m_data_stream)
	{
		m_data_stream = new CSerialPort();
		m_data_stream_is_external = false;
	}

	CSerialPort *serial = dynamic_cast<CSerialPort*>(m_data_stream);
	if (serial)
	{
		CCriticalSectionLocker lock(m_data_stream_cs);
		if (serial->isOpen())
			return true;  // Already open

		if (m_verbose) cout << "[CGPSInterface] Opening " << m_COMname << " @ " << m_COMbauds << endl;

		try
		{
			serial->open(m_COMname);
			// Config:
			serial->setConfig( m_COMbauds, 0, 8, 1 );
			serial->setTimeouts( 1, 0, 1, 1, 1 );

			// Do extra initialization?
			if (! OnConnectionEstablished() )
			{
				serial->close();
				return false;
			}
			return true; // All OK
		}
		catch (std::exception &e)
		{
			std::cerr << "[CGPSInterface::tryToOpenTheCOM] Error opening or configuring serial port:" << std::endl << e.what();
			serial->close();
			return false;
		}
	} // end of this is a serial port

	return true; // All OK
}

/* -----------------------------------------------------
				isGPS_connected
----------------------------------------------------- */
bool  CGPSInterface::isGPS_connected()
{
	return m_GPS_comsWork;
}

/* -----------------------------------------------------
				doProcess
----------------------------------------------------- */
void  CGPSInterface::doProcess()
{
	// Is the COM open?
	if (!tryToOpenTheCOM()) {
		m_state = ssError;
		THROW_EXCEPTION("Could not open the input stream");
	}
	ASSERT_(m_data_stream!=NULL)
	CSerialPort *stream_serial = dynamic_cast<CSerialPort*>(m_data_stream);
	CClientTCPSocket *stream_tcpip  = dynamic_cast<CClientTCPSocket*>(m_data_stream);

	// Read as many bytes as available:
	uint8_t       buf[0x1000];
	const size_t  to_read=std::min(m_rx_buffer.available()-1,sizeof(buf)-1);
	try
	{
		size_t nRead=0;
		if (to_read>0)
		{
			CCriticalSectionLocker lock(m_data_stream_cs);
			if (stream_tcpip) {
				nRead = stream_tcpip->readAsync( buf, to_read, 100, 10 );
			}
			else if (stream_serial) {
				nRead = stream_serial->Read(buf,to_read);
			}
			else{
				nRead = m_data_stream->ReadBuffer(buf,to_read);
			}
		}

		if (nRead) m_rx_buffer.push_many(buf,nRead);
		
		// Also dump to raw file:
		if (!m_raw_dump_file_prefix.empty() && !m_raw_output_file.fileOpenCorrectly()) {
			// 1st time open:
			mrpt::system::TTimeParts parts;
			mrpt::system::timestampToParts(now(), parts, true);
			string	sFilePostfix = "_";
			sFilePostfix += format("%04u-%02u-%02u_%02uh%02um%02us",(unsigned int)parts.year, (unsigned int)parts.month, (unsigned int)parts.day, (unsigned int)parts.hour, (unsigned int)parts.minute, (unsigned int)parts.second );
			const string sFileName = m_raw_dump_file_prefix + mrpt::system::fileNameStripInvalidChars( sFilePostfix ) + string(".gps");
			
			if (m_verbose) std::cout << "[CGPSInterface] Creating RAW dump file: `" << sFileName << "`\n";
			m_raw_output_file.open(sFileName);
		}
		if (nRead && m_raw_output_file.fileOpenCorrectly()) {
			m_raw_output_file.WriteBuffer(buf,nRead);
		}
	}
	catch (std::exception &)
	{
		// ERROR:
		MRPT_LOG_ERROR("[CGPSInterface::doProcess] Error reading stream of data: Closing communications\n");
		if(stream_serial) {
			CCriticalSectionLocker lock(m_data_stream_cs);
			stream_serial->close();
		}
		m_GPS_comsWork			= false;
		return;
	}

	// Try to parse incomming data as messages:
	parseBuffer( );
	
	// Decide whether to push out a new observation in old legacy mode.
	if (!m_customInit.empty())
	{	// "Advanced" (RTK,mmGPS) device  (kept for backwards-compatibility)
		bool do_append_obs = false;
		// FAMD
		// Append observation if:
		// 0. the timestamp seems to be correct!
		// 1. it contains both synched GGA and RMC data
		// 2. it contains only GGA or RMC but the next one is not synched with it
		if( m_last_timestamp == INVALID_TIMESTAMP )
		{
			if (m_verbose) cout << "[CGPSInterface] Initial timestamp: " << mrpt::system::timeToString(m_just_parsed_messages.timestamp) << endl;
			// Check if the initial timestamp seems to be OK (not a spurio one)
			TTimeStamp tmNow = mrpt::system::now();
			const double tdif = mrpt::system::timeDifference( m_just_parsed_messages.timestamp, tmNow );
			if( tdif >= 0 && tdif < 7500 /*Up to two hours*/)
				m_last_timestamp = m_just_parsed_messages.timestamp;
			else
				{  if (m_verbose) cout << "[CGPSInterface] Warning: The initial timestamp seems to be wrong! : " << tdif << endl;}
		} // end-if
		else
		{
			const double time_diff = mrpt::system::timeDifference( m_last_timestamp, m_just_parsed_messages.timestamp );
			if( time_diff < 0 || time_diff > 300 )     // Assert that the current timestamp is after the previous one and not more than 5 minutes later -> remove spurious
				{ if (m_verbose) cout << "[CGPSInterface ] Bad timestamp difference" << endl; return; }

			if( time_diff-m_topcon_data_period > 0.25*m_topcon_data_period )
				{ if (m_verbose) cout << "[CGPSInterface] WARNING: According to the timestamps, we probably skipped one frame!" << endl; }

			// a. These GPS data have both synched RMC and GGA data
			// don't append observation until we have both data
			do_append_obs = ( m_just_parsed_messages.has_GGA_datum && m_just_parsed_messages.has_RMC_datum );
		} // end-else

		if (do_append_obs)
			flushParsedMessagesNow();
	}

}

//!< Queue out now the messages in \a m_just_parsed_messages, leaving it empty
void  CGPSInterface::flushParsedMessagesNow()
{
	// Generic observation data:
	m_just_parsed_messages.sensorPose     = m_sensorPose;
	if (m_sensorLabelAppendMsgType)
	     m_just_parsed_messages.sensorLabel = m_sensorLabel + string("_")+ m_just_parsed_messages.sensorLabel;
	else m_just_parsed_messages.sensorLabel = m_sensorLabel;
	// Add observation to the output queue:
	CObservationGPSPtr newObs = CObservationGPS::Create();
	m_just_parsed_messages.swap(*newObs);
	CGenericSensor::appendObservation( newObs );
	m_just_parsed_messages.clear();
	m_last_timestamp = m_just_parsed_messages.timestamp;

	// And this means the comms works:
	m_GPS_comsWork = true;
	m_state = ssWorking;
}

/* -----------------------------------------------------
					parseBuffer
----------------------------------------------------- */
void  CGPSInterface::parseBuffer()
{
	if (m_parser == CGPSInterface::NONE) return; // Dont try to parse data

	// Only one parser selected?
	ptr_parser_t parser_ptr = NULL;
	switch (m_parser)
	{
	case CGPSInterface::NMEA:           parser_ptr=&CGPSInterface::implement_parser_NMEA;           break;
	case CGPSInterface::NOVATEL_OEM6:   parser_ptr=&CGPSInterface::implement_parser_NOVATEL_OEM6;   break;
	case CGPSInterface::AUTO:   break; // Leave it as NULL
	default:
		throw std::runtime_error("[CGPSInterface] Unknown parser!");
	};
	if (parser_ptr)
	{
		// Use only one parser ----------
		size_t min_bytes;
		do
		{
			if (!(*this.*parser_ptr)(min_bytes)) 
			{
				m_rx_buffer.pop(); // Not the start of a frame, skip 1 byte
			}
			if (m_customInit.empty() /* If we are not in old legacy mode */ && !m_just_parsed_messages.messages.empty() )
				flushParsedMessagesNow();
		} while (m_rx_buffer.size()>=min_bytes);
	} // end one parser mode ----------
	else
	{
		// AUTO mode --------
		const std::list<CGPSInterface::ptr_parser_t> &all_parsers = TParsersRegistry::getInstance().all_parsers;

		size_t global_min_bytes_max=0;
		do
		{
			bool all_parsers_want_to_skip = true;
			for (std::list<CGPSInterface::ptr_parser_t>::const_iterator it=all_parsers.begin();it!=all_parsers.end();++it)
			{
				parser_ptr = *it;
				size_t this_parser_min_bytes;
				if ((*this.*parser_ptr)(this_parser_min_bytes))
					all_parsers_want_to_skip = false;
				mrpt::utils::keep_max(global_min_bytes_max, this_parser_min_bytes);
			}

			if (all_parsers_want_to_skip)
				m_rx_buffer.pop(); // Not the start of a frame, skip 1 byte

			if (m_customInit.empty() /* If we are not in old legacy mode */ && !m_just_parsed_messages.messages.empty() )
				flushParsedMessagesNow();
		} while (m_rx_buffer.size()>=global_min_bytes_max);
	} // end AUTO mode ----
}

/* -----------------------------------------------------
					JAVAD_sendMessage
----------------------------------------------------- */
void CGPSInterface::JAVAD_sendMessage(const char *str, bool waitForAnswer )
{
	if (!str) return;
	const size_t len = strlen(str);
	CSerialPort *stream_serial = dynamic_cast<CSerialPort*>(m_data_stream);
	if (!stream_serial) return;

	size_t written;

	{
		CCriticalSectionLocker lock( m_data_stream_cs);
		written = stream_serial->Write(str,len);
	}

	if (m_verbose)
		std::cout << "[CGPSInterface] TX: " << str;

	if (written != len )
		throw std::runtime_error(format("Error sending command: '%s'",str).c_str());
	mrpt::system::sleep(5);

	if (!waitForAnswer) return;

	mrpt::system::sleep(200);
	char buf[200];
	buf[0]='\0';

	int bad_counter = 0;
	while(bad_counter < 10)
	{
		size_t nRead;
		{
			CCriticalSectionLocker lock( m_data_stream_cs);
			written = stream_serial->Write(str,len);
			nRead = stream_serial->Read(buf,sizeof(buf));
		}

		if (m_verbose)
			std::cout << "[CGPSInterface] RX: " << buf << std::endl;

		if (nRead<3 )
			throw std::runtime_error(format("ERROR: Invalid response '%s' for command '%s'",buf,str));

		if (nRead>=3 && buf[0]=='R' && buf[1]=='E')
			return; // Ok!
		else
			++bad_counter;
	}
	throw std::runtime_error(format("ERROR: Invalid response '%s' for command '%s'",buf,str));
}

bool CGPSInterface::OnConnectionShutdown()
{
	CSerialPort *stream_serial = dynamic_cast<CSerialPort*>(m_data_stream);

	if (stream_serial && !stream_serial->isOpen())
		return false;

	// Send commands:
	for (size_t i=0;i<m_shutdown_cmds.size();i++)
	{
		if (m_verbose) 
			cout << "[CGPSInterface] TX shutdown command: `" << m_shutdown_cmds[i] << "`\n";

		std::string sTx = m_shutdown_cmds[i];
		if (m_custom_cmds_append_CRLF)
			sTx+=std::string("\r\n");
		try 
		{
			CCriticalSectionLocker lock(m_data_stream_cs);
			m_data_stream->WriteBuffer(&sTx[0],sTx.size());
		} catch (...) {
			return false; // On any I/O error
		}

		mrpt::system::sleep(m_custom_cmds_delay*1000);
	}
	return true;
}

/* -----------------------------------------------------
					OnConnectionEstablished
----------------------------------------------------- */
bool CGPSInterface::OnConnectionEstablished()
{
	m_last_GGA.clear();  // On comms reset, empty this cache
	m_just_parsed_messages.clear();

	// Legacy behavior: 
	if ( !os::_strcmpi( m_customInit.c_str(), "JAVAD" ) || !os::_strcmpi( m_customInit.c_str(), "TOPCON" ) ) {
		return legacy_topcon_setup_commands();
	}

	// Purge input:
	CSerialPort *stream_serial = dynamic_cast<CSerialPort*>(m_data_stream);
	if (stream_serial)
	{
		CCriticalSectionLocker lock( m_data_stream_cs );
		stream_serial->purgeBuffers();
	}	

	// New behavior: Send generic commands set-up by the user in the config file.

	// Send commands:
	for (size_t i=0;i<m_setup_cmds.size();i++)
	{
		if (m_verbose) 
			cout << "[CGPSInterface] TX setup command: `" << m_setup_cmds[i] << "`\n";

		std::string sTx = m_setup_cmds[i];
		if (m_custom_cmds_append_CRLF) 
			sTx+=std::string("\r\n");

		try {
			CCriticalSectionLocker lock( m_data_stream_cs );
			m_data_stream->WriteBuffer(&sTx[0],sTx.size());
		} catch (std::exception &e) {
			std::cerr << "[CGPSInterface::OnConnectionEstablished] Error sending setup cmds: " << e.what() << std::endl;
			return false;
		}
		mrpt::system::sleep(m_custom_cmds_delay*1000);
	}
	mrpt::system::sleep(m_custom_cmds_delay*1000);
	return true;
}

bool CGPSInterface::unsetJAVAD_AIM_mode()
{
	MRPT_START
	if ( !os::_strcmpi( m_customInit.c_str(), "JAVAD" ) || !os::_strcmpi( m_customInit.c_str(), "TOPCON" ) )
	{
		// Stop messaging:
		JAVAD_sendMessage("%%dm\r\n", false);
		mrpt::system::sleep(500);
		JAVAD_sendMessage("%%dm\r\n",false);
		mrpt::system::sleep(1000);

		// Purge input:
		CSerialPort *stream_serial = dynamic_cast<CSerialPort*>(m_data_stream);
		if (stream_serial)
		{
			CCriticalSectionLocker lock( m_data_stream_cs );
			stream_serial->purgeBuffers();
		}

		JAVAD_sendMessage("%%set,/par/cur/term/imode,cmd\r\n");                // set the current port in command mode
		return true;
	}
	else
		return true;
	MRPT_END
} // end-unsetJAVAD_AIM_mode

bool CGPSInterface::setJAVAD_AIM_mode()
{
	MRPT_START
	if ( !os::_strcmpi( m_customInit.c_str(), "JAVAD" ) || !os::_strcmpi( m_customInit.c_str(), "TOPCON" ) )
	{
		JAVAD_sendMessage(format("%%%%set,/par%s/imode,cmd\r\n",m_JAVAD_rtk_src_port.c_str()).c_str());  // set the port in command mode
		JAVAD_sendMessage("%%set,/par/cur/term/jps/0,{nscmd,37,n,\"\"}\r\n");               // any command starting with % will be treated as normal

		ASSERT_(!m_JAVAD_rtk_format.empty())
		cout << "Formato de correcciones para GR3: " << m_JAVAD_rtk_format << endl;
		if( m_JAVAD_rtk_format == "cmr" )
		{
			JAVAD_sendMessage(format("%%%%set,/par/cur/term/jps/1,{cmr,-1,y,%s}\r\n", m_JAVAD_rtk_src_port.c_str()).c_str());   // set corrections type CMR or CMR+
			JAVAD_sendMessage("%%set,/par/cur/term/jps/2,{none,-1,n,\"\"}\r\n");
			JAVAD_sendMessage(format("%%%%set,/par%s/imode,cmr\r\n", m_JAVAD_rtk_src_port.c_str()).c_str());
		}
		else if( m_JAVAD_rtk_format == "rtcm" )
		{
			JAVAD_sendMessage(format("%%%%set,/par/cur/term/jps/1,{rtcm,-1,y,%s}\r\n", m_JAVAD_rtk_src_port.c_str()).c_str());  // set corrections type RTCM
			JAVAD_sendMessage("%%set,/par/cur/term/jps/2,{none,-1,n,\"\"}\r\n");
			JAVAD_sendMessage(format("%%%%set,/par%s/imode,rtcm\r\n", m_JAVAD_rtk_src_port.c_str()).c_str());
		}
		else if( m_JAVAD_rtk_format == "rtcm3" )
		{
			JAVAD_sendMessage(format("%%%%set,/par/cur/term/jps/1,{rtcm3,-1,y,%s}\r\n", m_JAVAD_rtk_src_port.c_str()).c_str()); // set corrections type RTCM 3.x
			JAVAD_sendMessage("%%set,/par/cur/term/jps/2,{none,-1,n,\"\"}\r\n");
			JAVAD_sendMessage(format("%%%%set,/par%s/imode,rtcm3\r\n", m_JAVAD_rtk_src_port.c_str()).c_str());
		}
		else
		{
			cout << "Unknown RTK corrections format. Only supported: CMR, RTCM or RTCM3" << endl;
			return false;
		}
		JAVAD_sendMessage("%%set,/par/cur/term/imode,jps\r\n");                         // sets current port into "JPS" mode

		return true;

	} // end-if
	else
		return true;
	MRPT_END
} // end-setJAVAD_AIM_mode

std::string CGPSInterface::getLastGGA(bool reset)
{
	std::string ret = m_last_GGA;
	if (reset) m_last_GGA.clear();
	return ret;
}

bool CGPSInterface::legacy_topcon_setup_commands()
{
	// Stop messaging:
	JAVAD_sendMessage("%%dm\r\n", false);
	mrpt::system::sleep(500);
	JAVAD_sendMessage("%%dm\r\n",false);
	mrpt::system::sleep(1000);

	// Purge input:
	CSerialPort *stream_serial = dynamic_cast<CSerialPort*>(m_data_stream);
	if (stream_serial)
	{
		CCriticalSectionLocker lock( m_data_stream_cs );
		stream_serial->purgeBuffers();
	}

	// Configure RTK mode and source:
	if (m_verbose)
		cout << "[CGPSInterface] Configure RTK options" << endl;

	if (!m_JAVAD_rtk_src_port.empty())
	{
		const int elevation_mask = 5; // Degs

		JAVAD_sendMessage(format("%%%%set,/par/lock/elm,%i\r\n", elevation_mask).c_str());  // Set elevation mask to track satellites
		JAVAD_sendMessage("%%set,/par/base/mode/,off\r\n");  // Set Base Mode off
		JAVAD_sendMessage("%%set,/par/pos/pd/period,1.0\r\n"); // Differential Correction Interval
		//JAVAD_sendMessage("%%set,hd/mode,off\r\n");  // fixed distance to rtk base: Off
		//JAVAD_sendMessage("%%set,/par/pos/pd/hd/mode,off\r\n");  // fixed distance to rtk base: Off  <-- Not working with TopCon GR3! (option disabled)
		JAVAD_sendMessage("%%set,/par/pos/pd/qcheck,off\r\n"); // Set Quality Checks Off
		JAVAD_sendMessage("%%set,/par/pos/mode/cur,pd\r\n");  // Pos Mode Phase Diff
		JAVAD_sendMessage("%%set,/par/pos/pd/textr,10\r\n");  // RTK Extrapolation Limit
		JAVAD_sendMessage("%%set,/par/pos/pd/inuse,-1\r\n");  // Set Rovers Reference Station
		JAVAD_sendMessage("%%set,/par/pos/pd/nrs/mode,y\r\n");  // Enable Nearest Reference Station Mode
		JAVAD_sendMessage("%%set,/par/pos/pd/mode,extrap\r\n");// Enable EXTRAPOLATED mode in RTK corrections

		// Set Differential Correction Source
		JAVAD_sendMessage(format("%%%%set,/par/pos/pd/port,%s\r\n",m_JAVAD_rtk_src_port.c_str()).c_str());

		// Set port bauds:
		if( ! m_topcon_useAIMMode && m_JAVAD_rtk_src_baud!=0 && !mrpt::system::strCmp(m_JAVAD_rtk_src_port,"/dev/usb/a") )
			JAVAD_sendMessage(format("%%%%set,/par%s/rate,%u\r\n",m_JAVAD_rtk_src_port.c_str(), m_JAVAD_rtk_src_baud).c_str());

		// Set Input Mode: CMR,RTCM,...
		if( ! m_topcon_useAIMMode && !m_JAVAD_rtk_format.empty())
			JAVAD_sendMessage(format("%%%%set,/par%s/imode,%s\r\n", m_JAVAD_rtk_src_port.c_str(), m_JAVAD_rtk_format.c_str()).c_str());
	}

	// Start NMEA messaging:
//		JAVAD_sendMessage("%%em,,/msg/nmea/GGA:0.2\r\n");
//		JAVAD_sendMessage("%%em,,/msg/nmea/RMC:0.2\r\n");
	//JAVAD_sendMessage("%%em,,/msg/jps/PS:0.2\r\n");

	if( m_topcon_useAIMMode )
	{
		if (m_verbose) cout << "[CGPSInterface] Using Advanced Input Mode";
		m_topcon_AIMConfigured = setJAVAD_AIM_mode();
		if (m_verbose) cout << "... done" << endl;
	}
	JAVAD_sendMessage(format("%%%%em,,/msg/nmea/GGA:%.1f\r\n", m_topcon_data_period ).c_str());
	JAVAD_sendMessage(format("%%%%em,,/msg/nmea/RMC:%.1f\r\n", m_topcon_data_period ).c_str());       // FAMD: 10 Hz

	if( m_topcon_useAIMMode )
		{ if (m_verbose) cout << "[CGPSInterface::OnConnectionEstablished] JAVAD/TopCon commands sent successfully with AIM." << endl;}
	else
		{ if (m_verbose) cout << "[CGPSInterface::OnConnectionEstablished] JAVAD/TopCon commands sent successfully." << endl;}

	return true;
}

/** Send a custom data block to the GNSS device right now. Can be used to change its behavior online as needed. */
bool CGPSInterface::sendCustomCommand(const void* data, const size_t datalen)
{
	try 
	{
		CCriticalSectionLocker lock( m_data_stream_cs );
		m_data_stream->WriteBuffer(data,datalen);
		return true;
	} 
	catch (std::exception &e) 
	{
		std::cerr << "[CGPSInterface::sendCustomCommand] Error sending cmd: " << e.what() << std::endl;
		return false;
	}
}
