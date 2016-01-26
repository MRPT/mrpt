/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/hwdrivers/CGPSInterface.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::synch;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CGPSInterface,mrpt::hwdrivers)

MRPT_TODO("Offer method to write commands to the GPS");
MRPT_TODO("Parse (some) novatel binary frames");

MRPT_TODO("Export to binary file from rawlog-edit")
MRPT_TODO("Import from ASCII/binary file with a new app: gps2rawlog")

MRPT_TODO("new parse unit tests") // Example cmds: https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf

/* -----------------------------------------------------
                Constructor
   ----------------------------------------------------- */
CGPSInterface::CGPSInterface() :
	m_rx_buffer          (0x10000),
	m_parser             (CGPSInterface::NMEA),
	m_raw_dump_file_prefix(),
	m_COM                (),
	m_out_COM            (NULL),
	m_cs_out_COM         (NULL),
	m_customInit         (),
	m_COMname            (),
	m_COMbauds           (4800),
	m_GPS_comsWork			(false),
	m_GPS_signalAcquired	(false),
	m_last_timestamp        ( INVALID_TIMESTAMP ),
	m_setup_cmds_delay   (0.1),
	m_setup_cmds_append_CRLF(true),

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

	// legacy custom cmds:
	m_customInit	= configSource.read_string( iniSection, "customInit", m_customInit, false );

	// new custom cmds:
	m_setup_cmds_delay = configSource.read_float( iniSection, "setup_cmds_delay",m_setup_cmds_delay );
	m_setup_cmds_append_CRLF = configSource.read_bool( iniSection, "m_setup_cmds_append_CRLF",m_setup_cmds_append_CRLF);
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

	m_sensorPose.x( configSource.read_float( iniSection, "pose_x",0, false ) );
	m_sensorPose.y( configSource.read_float( iniSection, "pose_y",0, false ) );
	m_sensorPose.z( configSource.read_float( iniSection, "pose_z",0, false ) );
	MRPT_TODO("ORIENTATION");

	m_JAVAD_rtk_src_port = configSource.read_string(iniSection, "JAVAD_rtk_src_port",m_JAVAD_rtk_src_port );
	m_JAVAD_rtk_src_baud = configSource.read_int(iniSection, "JAVAD_rtk_src_baud",m_JAVAD_rtk_src_baud );
	m_JAVAD_rtk_format   = configSource.read_string(iniSection,"JAVAD_rtk_format", m_JAVAD_rtk_format );

    m_topcon_useAIMMode = configSource.read_bool( iniSection,"JAVAD_useAIMMode", m_topcon_useAIMMode );
    m_topcon_data_period = 1.0/configSource.read_double( iniSection,"outputRate", m_topcon_data_period );
}

CGPSInterface::~CGPSInterface()
{
}

void CGPSInterface::setParser(CGPSInterface::PARSERS parser) {
	m_parser = parser;
}
CGPSInterface::PARSERS CGPSInterface::getParser() const {
	return m_parser;
}
void CGPSInterface::setExternCOM( CSerialPort *outPort, mrpt::synch::CCriticalSection *csOutPort )
{ 
	m_out_COM = outPort; 
	m_cs_out_COM = csOutPort; 
}
void CGPSInterface::setSetupCommandsDelay(const double delay_secs) {
	m_setup_cmds_delay = delay_secs;
}
double CGPSInterface::getSetupCommandsDelay() const {
	return m_setup_cmds_delay;
}
void CGPSInterface::setSetupCommands(const std::vector<std::string> &cmds) {
	m_setup_cmds = cmds;
}
const std::vector<std::string> & CGPSInterface::setSetupCommands() const {
	return m_setup_cmds;
}
void CGPSInterface::enableSetupCommandsAppendCRLF(const bool enable) {
	m_setup_cmds_append_CRLF = enable;
}
bool CGPSInterface::isEnabledSetupCommandsAppendCRLF() const {
	return m_setup_cmds_append_CRLF;
}

/* -----------------------------------------------------
				setSerialPortName
----------------------------------------------------- */
void  CGPSInterface::setSerialPortName(const std::string &COM_port)
{
    // MAR'11
    if( useExternCOM() )
    {
        bool res = false;
        {
            CCriticalSectionLocker lock( m_cs_out_COM );
            res = m_out_COM->isOpen();
        }
        if( res )
            THROW_EXCEPTION("Cannot change serial port name when it's already open")
	}
	else
        if (m_COM.isOpen())
            THROW_EXCEPTION("Cannot change serial port name when it's already open")

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
    if( useExternCOM() )
    {
        bool res = false;
        {
            CCriticalSectionLocker loc( m_cs_out_COM );
            res = m_out_COM->isOpen();
        }
        if( res )
            return true;	// Already open
    }
    else
        if (m_COM.isOpen())
            return true;	// Already open

    if (m_verbose) cout << "[CGPSInterface] Opening " << m_COMname << " @ " << m_COMbauds << endl;

	m_last_GGA.clear();  // On comms reset, empty this cache
	m_just_parsed_messages.clear();

	try
	{
        if( useExternCOM() )
        {
            CCriticalSectionLocker lock( m_cs_out_COM );
            m_out_COM->open(m_COMname);
            // Config:
            m_out_COM->setConfig( m_COMbauds, 0, 8, 1 );
            m_out_COM->setTimeouts( 1, 0, 1, 1, 1 );
        }
        else
        {
            m_COM.open(m_COMname);
            // Config:
            m_COM.setConfig( m_COMbauds, 0, 8, 1 );
            m_COM.setTimeouts( 1, 0, 1, 1, 1 );
        }

		// Do extra initialization?
		if (! OnConnectionEstablished() )
		{
		    if( useExternCOM() )
		    {
		        CCriticalSectionLocker lock( m_cs_out_COM );
                m_out_COM->close();
            }
            else
                m_COM.close();
			return false;
		}

		return true; // All OK!
	}
	catch (std::exception &e)
	{
		std::cerr << "[CGPSInterface::tryToOpenTheCOM] Error opening or configuring the serial port:" << std::endl << e.what();
        if( useExternCOM() )
        {
            CCriticalSectionLocker lock( m_cs_out_COM );
            m_out_COM->close();
        }
        else
            m_COM.close();
		return false;
	}
}

/* -----------------------------------------------------
				isGPS_connected
----------------------------------------------------- */
bool  CGPSInterface::isGPS_connected()
{
	return m_GPS_comsWork;
}

/* -----------------------------------------------------
				isGPS_signalAcquired
----------------------------------------------------- */
bool  CGPSInterface::isGPS_signalAcquired()
{
	return m_GPS_signalAcquired;
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

	// Read as many bytes as available:
	uint8_t       buf[0x1000];
	const size_t  to_read=std::min(m_rx_buffer.available()-1,sizeof(buf)-1);
	try
	{
		size_t nRead=0;
		if (to_read>0)
		{
			MRPT_TODO("handle tcp/ip streams")
			/*if ( dynamic_cast<>(XXX) ) {
				CClientTCPSocket	*client = dynamic_cast<CClientTCPSocket*>(m_stream);
				nRead = client->readAsync( buf, to_read, 100, 10 );
			}
			else {*/
			nRead = m_COM.Read(buf,to_read);
			//}
		}

		if (nRead) {
			m_rx_buffer.push_many(buf,nRead);
			if (m_verbose) {
				buf[nRead] = '\0';
				printf("RX: %s",(char*)buf);
			}
		}
		
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
		printf_debug("[CGPSInterface::doProcess] Error reading COM port: Closing communications\n");
		if( useExternCOM() ) {
			CCriticalSectionLocker lock( m_cs_out_COM );
			m_out_COM->close();
		}
		else {
			m_COM.close();
		}
		m_GPS_comsWork			= false;
		m_GPS_signalAcquired	= false;
		return;
	}

	// Try to parse incomming data as messages:
	parseBuffer( );
	
	// Decide whether to push out a new observation:
	bool do_append_obs = false;
	if (m_customInit.empty())
	{   // General case:
		do_append_obs = ( !m_just_parsed_messages.messages.empty() );
	}
	else
	{	// "Advanced" (RTK,mmGPS) device  (kept for backwards-compatibility)
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
	}

	if (do_append_obs)
	{
		// Add observation to the output queue:
		CObservationGPSPtr newObs = CObservationGPS::Create();
		m_just_parsed_messages.swap(*newObs);
		CGenericSensor::appendObservation( newObs );
		m_just_parsed_messages.clear();
		m_last_timestamp = m_just_parsed_messages.timestamp;
	}
}

/* -----------------------------------------------------
					parseBuffer
----------------------------------------------------- */
void  CGPSInterface::parseBuffer()
{
	switch (m_parser)
	{
	case CGPSInterface::NMEA:           implement_parser_NMEA();           break;
	case CGPSInterface::NOVATEL_OEM6:   implement_parser_NOVATEL_OEM6();   break;

	default: throw std::runtime_error("[CGPSInterface] Unknown parser!");
	};
}

/* -----------------------------------------------------
					processGPSstring
----------------------------------------------------- */
void  CGPSInterface::processGPSstring(const std::string &s)
{
	const bool did_have_gga = m_just_parsed_messages.has_GGA_datum;
	// Parse:
	CGPSInterface::parse_NMEA(s,m_just_parsed_messages, m_verbose);
	
	// Save GGA cache (useful for NTRIP,...)
	const bool has_gga = m_just_parsed_messages.has_GGA_datum;
	if (has_gga && !did_have_gga) {
		m_last_GGA = s;
	}

	// Generic observation data:
	m_just_parsed_messages.sensorPose     = m_sensorPose;
	m_just_parsed_messages.sensorLabel    = m_sensorLabel;
}


/* -----------------------------------------------------
					JAVAD_sendMessage
----------------------------------------------------- */
void CGPSInterface::JAVAD_sendMessage(const char *str, bool waitForAnswer )
{
	if (!str) return;
	const size_t len = strlen(str);

    size_t written;

    if( useExternCOM() )
    {
        CCriticalSectionLocker lock( m_cs_out_COM );
        written = m_out_COM->Write(str,len);
    }
    else
        written = m_COM.Write(str,len);

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
        if( useExternCOM() )
        {
            CCriticalSectionLocker lock( m_cs_out_COM );
            nRead = m_out_COM->Read(buf,sizeof(buf));
        }
        else
            nRead = m_COM.Read(buf,sizeof(buf));

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

/* -----------------------------------------------------
					OnConnectionEstablished
----------------------------------------------------- */
bool CGPSInterface::OnConnectionEstablished()
{
	// Legacy behavior: 
	if ( !os::_strcmpi( m_customInit.c_str(), "JAVAD" ) || !os::_strcmpi( m_customInit.c_str(), "TOPCON" ) ) {
		return legacy_topcon_setup_commands();
	}

	// New behavior: Send generic commands set-up by the user in the config file.

	// Send commands:
	for (size_t i=0;i<m_setup_cmds.size();i++)
	{
		if (m_verbose) 
			cout << "[CGPSInterface] TX setup command: `" << m_setup_cmds[i] << "`\n";

		std::string sTx = m_setup_cmds[i];
		if (m_setup_cmds_append_CRLF) 
			sTx+=std::string("\r\n");
		const size_t written = m_COM.Write(&sTx[0],sTx.size());
		ASSERT_EQUAL_(written,sTx.size());

		mrpt::system::sleep(m_setup_cmds_delay*1000);
	}

	// Purge input:
	MRPT_TODO("Replace m_COM here and above by generic stream object")
	m_COM.purgeBuffers();

	mrpt::system::sleep(m_setup_cmds_delay*1000);
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
		if( useExternCOM() )
		{
		    CCriticalSectionLocker lock( m_cs_out_COM );
		    m_out_COM->purgeBuffers();
		}
		else
			m_COM.purgeBuffers();

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
	if( useExternCOM() )
	{
		CCriticalSectionLocker lock( m_cs_out_COM );
		m_out_COM->purgeBuffers();
	}
	else
		m_COM.purgeBuffers();

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

