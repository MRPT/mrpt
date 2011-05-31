/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers.h> // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/hwdrivers/CGPSInterface.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CGPSInterface,mrpt::hwdrivers)

/* -----------------------------------------------------
                Constructor
   ----------------------------------------------------- */
CGPSInterface::CGPSInterface( int BUFFER_LENGTH ) :
	m_COM					(),
	m_customInit			(),

	m_COMname				(),
	m_COMbauds				(4800),
	m_GPS_comsWork			(false),
	m_GPS_signalAcquired	(false),
	m_BUFFER_LENGTH			(BUFFER_LENGTH),
	m_buffer				( new char[m_BUFFER_LENGTH] ),
	m_bufferLength			(0),
	m_bufferWritePos		(0),

	m_JAVAD_rtk_src_port	(),
	m_JAVAD_rtk_src_baud	(0),
	m_JAVAD_rtk_format		("cmr")
{
	m_sensorLabel = "GPS";
	m_latestGPS_data.has_GGA_datum = false;
	m_latestGPS_data.has_RMC_datum = false;
}

/* -----------------------------------------------------
                loadConfig_sensorSpecific
   ----------------------------------------------------- */
void  CGPSInterface::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
#ifdef MRPT_OS_WINDOWS
	m_COMname = configSource.read_string(iniSection, "COM_port_WIN", m_COMname, true );
#else
	m_COMname = configSource.read_string(iniSection, "COM_port_LIN", m_COMname, true );
#endif

	m_COMbauds		= configSource.read_int( iniSection, "baudRate",m_COMbauds, true );

	m_customInit	= configSource.read_string( iniSection, "customInit", m_customInit, false );

	m_sensorPose.x( configSource.read_float( iniSection, "pose_x",0, false ) );
	m_sensorPose.y( configSource.read_float( iniSection, "pose_y",0, false ) );
	m_sensorPose.z( configSource.read_float( iniSection, "pose_z",0, false ) );

	m_JAVAD_rtk_src_port = configSource.read_string(iniSection, "JAVAD_rtk_src_port",m_JAVAD_rtk_src_port );
	m_JAVAD_rtk_src_baud = configSource.read_int(iniSection, "JAVAD_rtk_src_baud",m_JAVAD_rtk_src_baud );
	m_JAVAD_rtk_format   = configSource.read_string(iniSection,"JAVAD_rtk_format", m_JAVAD_rtk_format );
}


/* -----------------------------------------------------
                Destructor
   ----------------------------------------------------- */
CGPSInterface::~CGPSInterface()
{
	delete[] m_buffer;
}

/* -----------------------------------------------------
				setSerialPortName
----------------------------------------------------- */
void  CGPSInterface::setSerialPortName(const std::string &COM_port)
{
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
	if (m_COM.isOpen())
		return true;	// Already open

	try
	{
		m_COM.open(m_COMname);

		// Config:
		m_COM.setConfig( m_COMbauds, 0,8,1 );

		m_COM.setTimeouts( 1,0,1,1,1 );

		m_latestGPS_data.has_GGA_datum = false;
		m_latestGPS_data.has_RMC_datum = false;

		// Do extra initialization?
		if (! OnConnectionEstablished() )
		{
			m_COM.close();
			return false;
		}

		return true; // All OK!
	}
	catch (std::exception &e)
	{
		std::cerr << "[CGPSInterface::tryToOpenTheCOM] Error opening or configuring the serial port:" << std::endl << e.what();
		m_COM.close();
		return false;
	}
	catch (...)
	{
		std::cerr << "[CGPSInterface::tryToOpenTheCOM] Error opening or configuring the serial port." << std::endl;
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
	size_t	bytesToRead, bytesRead;

	// Is the COM open?
	if (!tryToOpenTheCOM())
	{
		m_state = ssError;
		THROW_EXCEPTION("Cannot open the serial port");
	}

	// Read as many bytes as available:
	bytesRead = 1;
	while (bytesRead)
	{
		bytesToRead = (m_BUFFER_LENGTH-10) - m_bufferWritePos;

		try
		{
			bytesRead = m_COM.Read(m_buffer + m_bufferWritePos, bytesToRead);
		}
		catch (...)
		{
			// ERROR:
			printf_debug("[CGPSInterface::doProcess] Error reading COM port: Closing communications\n");
			m_COM.close();
			m_GPS_comsWork			= false;
			m_GPS_signalAcquired	= false;
			return;
		}


		// Process the data:
		if (bytesRead)
		{
			m_bufferLength += bytesRead;
			processBuffer( );
		}
	};

	// Write command to buffer:
	if (m_latestGPS_data.has_GGA_datum ||
	    m_latestGPS_data.has_RMC_datum )
	{
		// Add observation to the output queue:
		CObservationGPSPtr newObs = CObservationGPSPtr( new  CObservationGPS( m_latestGPS_data ) );
		appendObservation( newObs );

		m_latestGPS_data.has_GGA_datum = false;
		m_latestGPS_data.has_RMC_datum = false;
	}
}

/* -----------------------------------------------------
					processBuffer
----------------------------------------------------- */
void  CGPSInterface::processBuffer()
{
	unsigned int	i=0, lineStart = 0;

	while (i<m_bufferLength)
	{
		// Loof for the first complete line of text:
		while (i<m_bufferLength && m_buffer[i]!='\r' && m_buffer[i]!='\n')
			i++;

		// End of buffer or end of line?
		if (i<m_bufferLength)
		{
			// It is the end of a line: Build the string:
			std::string			str;
			int				lenOfLine = i-1 - lineStart;
			if (lenOfLine>0)
			{
				str.resize( lenOfLine );
				memcpy( (void*)str.c_str(), m_buffer + lineStart, lenOfLine );

				// Process it!:
				processGPSstring(str);
			}

			// And this means the comms works!
			m_GPS_comsWork			= true;

            m_state = ssWorking;

			// Pass over this end of line:
			while (i<m_bufferLength && (m_buffer[i]=='\r' || m_buffer[i]=='\n'))
				i++;

			// And start a new line:
			lineStart = i;
		}

	};

	// Dejar en el buffer desde "lineStart" hasta "i-1", inclusive.
	size_t	newBufLen = m_bufferLength - lineStart;

	memcpy( m_buffer, m_buffer + lineStart, newBufLen);

	m_bufferLength = newBufLen;

	// Seguir escribiendo por aqui:
	m_bufferWritePos = i - lineStart;

}

/* -----------------------------------------------------
					processGPSstring
----------------------------------------------------- */
void  CGPSInterface::processGPSstring(const std::string &s)
{
	//printf_debug("[GPS raw string:] %s\n",s.c_str());

	// Firstly! If the string does not start with "$GP" it is not valid:
	if ( s[0]!='$' ||
	        s[1]!='G' ) return;

	// Try to determine the kind of command:
	if ( s[3]=='G' &&
	        s[4]=='G' &&
	        s[5]=='A' &&
	        s[6]==',')
	{
		// ---------------------------------------------
		//					GGA
		// ---------------------------------------------
		unsigned int	parserPos = 7;
		std::string			token;

		// Fill out the output structure:

		// Time:

		getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
		if (token.size()>=6)
		{
			m_latestGPS_data.GGA_datum.UTCTime.hour		= 10 * (token[0]-'0') + token[1]-'0';
			m_latestGPS_data.GGA_datum.UTCTime.minute	= 10 * (token[2]-'0') + token[3]-'0';
			m_latestGPS_data.GGA_datum.UTCTime.sec		= atof( & (token.c_str()[4]) );
		}

		// Latitude:
		getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
		if (token.size()>=4)
		{
			double	lat = 10 * (token[0]-'0') + token[1]-'0';
			lat += atof( & (token.c_str()[2]) ) / 60.0;
			m_latestGPS_data.GGA_datum.latitude_degrees = lat;
		}

		// N/S:
		getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
		if (token[0]=='S')
			m_latestGPS_data.GGA_datum.latitude_degrees = -m_latestGPS_data.GGA_datum.latitude_degrees;

		// Longitude:
		getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
		if (token.size()>=5)
		{
			double	lat = 100 * (token[0]-'0') + 10 * (token[1]-'0')+ token[2]-'0';
			lat += atof( & (token.c_str()[3]) ) / 60.0;
			m_latestGPS_data.GGA_datum.longitude_degrees = lat;
		}

		// E_W:
		getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
		if (token[0]=='W')
			m_latestGPS_data.GGA_datum.longitude_degrees = -m_latestGPS_data.GGA_datum.longitude_degrees;

		// fix quality:
		getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
		m_latestGPS_data.GGA_datum.fix_quality = (unsigned char)atoi(token.c_str());

		// sats:
		getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
		m_latestGPS_data.GGA_datum.satellitesUsed = (unsigned char)atoi( token.c_str() );

		// HDOP:
		getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
		m_latestGPS_data.GGA_datum.HDOP = (float)atof( token.c_str() );

		// Altitude:
		getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
		m_latestGPS_data.GGA_datum.altitude_meters = atof( token.c_str() );

		m_latestGPS_data.has_GGA_datum = true;

		// Generic observation data:
		m_latestGPS_data.timestamp = mrpt::system::now();
		m_latestGPS_data.sensorPose = m_sensorPose;
		m_latestGPS_data.sensorLabel = m_sensorLabel;

		//printf_debug("[GPS decoded GGA string] %s\n",s.c_str());
	}
	else
	{
		// Try to determine the kind of command:
		if ( s[3]=='R' &&
		        s[4]=='M' &&
		        s[5]=='C' &&
		        s[6]==',')
		{

			// ---------------------------------------------
			//					GPRMC
			//
			// ---------------------------------------------
			unsigned int	parserPos = 7;
			std::string			token;

			// Rellenar la estructura de "ultimo dato RMC recibido"

			// Time:
			getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
			if (token.size()>=6)
			{
				m_latestGPS_data.RMC_datum.UTCTime.hour		= 10 * (token[0]-'0') + token[1]-'0';
				m_latestGPS_data.RMC_datum.UTCTime.minute	= 10 * (token[2]-'0') + token[3]-'0';
				m_latestGPS_data.RMC_datum.UTCTime.sec		= atof( & (token.c_str()[4]) );
			}

			// Valid?
			getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
			m_latestGPS_data.RMC_datum.validity_char = token.c_str()[0];

			// Latitude:
			getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
			if (token.size()>=4)
			{
				double	lat = 10 * (token[0]-'0') + token[1]-'0';
				lat += atof( & (token.c_str()[2]) ) / 60.0;
				m_latestGPS_data.RMC_datum.latitude_degrees = lat;
			}

			// N/S:
			getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
			if (token[0]=='S')
				m_latestGPS_data.RMC_datum.latitude_degrees = -m_latestGPS_data.RMC_datum.latitude_degrees;

			// Longitude:
			getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
			if (token.size()>=5)
			{
				double	lat = 100 * (token[0]-'0') + 10 * (token[1]-'0')+ token[2]-'0';
				lat += atof( & (token.c_str()[3]) ) / 60.0;
				m_latestGPS_data.RMC_datum.longitude_degrees = lat;
			}

			// E_W:
			getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
			if (token[0]=='W')
				m_latestGPS_data.RMC_datum.longitude_degrees = -m_latestGPS_data.RMC_datum.longitude_degrees;

			// Speed:
			getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
			m_latestGPS_data.RMC_datum.speed_knots = atof( token.c_str() );

			// Directorion:
			getNextToken(s,token,parserPos); //printf("TOKEN: %s\n",token.c_str());
			m_latestGPS_data.RMC_datum.direction_degrees= atof( token.c_str() );

			m_latestGPS_data.has_RMC_datum = true;


			// Generic observation data:
			m_latestGPS_data.timestamp = mrpt::system::now();
			m_latestGPS_data.sensorPose = m_sensorPose;
			m_latestGPS_data.sensorLabel = m_sensorLabel;

			//printf_debug("[GPS decoded RMC string] %s\n",s.c_str());
		}
		else
		{
			// ... parse other commands
		}
	}
}

/* -----------------------------------------------------
					processGPSstring
----------------------------------------------------- */
void  CGPSInterface::getNextToken(
    const std::string	&str,
    std::string			&token,
    unsigned int		&parserPos)
{
	unsigned int	start = parserPos;
	char			c;

	if (parserPos>=str.size())
	{
		token.resize(0);
		return;
	}

	do
	{
		c = str[parserPos++];
	} while (parserPos<str.size() && c!=',');

	unsigned int	tokenSize = parserPos - 1 - start;
	if (tokenSize<1)
	{
		token.resize(0);
	}
	else
	{
		token.resize( tokenSize );
		memcpy( (void*)token.c_str(), str.c_str()+start, tokenSize );
	}

}

/* -----------------------------------------------------
					getLastData
----------------------------------------------------- */
/*void  CGPSInterface::getLastData(
    bool			&out_thereIsData,
    mrpt::slam::CObservationGPS &out_data
)
{
	if (m_latestGPS_data.has_GGA_datum ||
	        m_latestGPS_data.has_RMC_datum )
	{
		out_data		= m_latestGPS_data;
		out_thereIsData = true;

		m_latestGPS_data.has_GGA_datum = false;
		m_latestGPS_data.has_RMC_datum = false;
	}
	else
	{
		out_thereIsData = false;
	}
}*/


/* -----------------------------------------------------
					JAVAD_sendMessage
----------------------------------------------------- */
void CGPSInterface::JAVAD_sendMessage(const char *str, bool waitForAnswer )
{
	if (!str) return;
	const size_t len = strlen(str);

	const size_t written = m_COM.Write(str,len);

#if 0
	std::cout << "TX: " << str;
#endif

	if (written != len )
		throw std::runtime_error(format("Error sending command: '%s'",str).c_str());
	mrpt::system::sleep(5);

	if (!waitForAnswer) return;

	mrpt::system::sleep(200);
	char buf[200];
	buf[0]='\0';

	const size_t nRead = m_COM.Read(buf,sizeof(buf));

#if 0
	std::cout << "RX: " << buf << std::endl;
#endif

	if (nRead<3 )
		throw std::runtime_error(format("ERROR: Invalid response '%s' for command '%s'",buf,str));

	if (nRead>=3 && buf[0]=='R' && buf[1]=='E')
		return; // Ok!
	else
		throw std::runtime_error(format("ERROR: Invalid response '%s' for command '%s'",buf,str));
}

/* -----------------------------------------------------
					OnConnectionEstablished
----------------------------------------------------- */
bool CGPSInterface::OnConnectionEstablished()
{
	if ( !os::_strcmpi( m_customInit.c_str(), "JAVAD" ) ||
	     !os::_strcmpi( m_customInit.c_str(), "TOPCON" ) )
	{
		// Stop messaging:
		JAVAD_sendMessage("%%dm\r\n", false);
		mrpt::system::sleep(500);
		JAVAD_sendMessage("%%dm\r\n",false);
		mrpt::system::sleep(1000);

		// Purge input:
		m_COM.purgeBuffers();

		// Configure RTK mode and source:
		if (!m_JAVAD_rtk_src_port.empty())
		{
			const int elevation_mask = 5; // Degs

			JAVAD_sendMessage(format("%%%%set,/par/lock/elm,%i\r\n", elevation_mask).c_str());  // Set elevation mask to track satellites
			JAVAD_sendMessage("%%set,/par/base/mode/,off\r\n");  // Set Base Mode off
			JAVAD_sendMessage("%%set,/par/pos/pd/period,1.0\r\n"); // Differential Correction Interval
			JAVAD_sendMessage("%%set,hd/mode,off\r\n");  // fixed distance to rtk base: Off
			JAVAD_sendMessage("%%set,/par/pos/pd/qcheck,off\r\n"); // Set Quality Checks Off
			JAVAD_sendMessage("%%set,/par/pos/mode/cur,pd\r\n");  // Pos Mode Phase Diff
			JAVAD_sendMessage("%%set,/par/pos/pd/textr,10\r\n");  // RTK Extrapolation Limit
			JAVAD_sendMessage("%%set,/par/pos/pd/inuse,-1\r\n");  // Set Rovers Reference Station
			JAVAD_sendMessage("%%set,/par/pos/pd/nrs/mode,y\r\n");  // Enable Nearest Reference Station Mode
			JAVAD_sendMessage("%%set,/par/pos/pd/mode,extrap\r\n");// Enable EXTRAPOLATED mode in RTK corrections

			// Set Differential Correction Source
			JAVAD_sendMessage(format("%%%%set,/par/pos/pd/port,%s\r\n",m_JAVAD_rtk_src_port.c_str()).c_str());

			// Set port bauds:
			if (m_JAVAD_rtk_src_baud!=0)
				JAVAD_sendMessage(format("%%%%set,/par%s/rate,%u\r\n",m_JAVAD_rtk_src_port.c_str(), m_JAVAD_rtk_src_baud).c_str());

			// Set Input Mode: CMR,RTCM,...
			if (!m_JAVAD_rtk_format.empty())
				JAVAD_sendMessage(format("%%%%set,/par%s/imode,%s\r\n", m_JAVAD_rtk_src_port.c_str(), m_JAVAD_rtk_format.c_str()).c_str());
		}

		// Start NMEA messaging:
		JAVAD_sendMessage("%%em,,/msg/nmea/GGA:0.2\r\n");
		JAVAD_sendMessage("%%em,,/msg/nmea/RMC:0.2\r\n");
		//JAVAD_sendMessage("%%em,,/msg/jps/PS:0.2\r\n");

		cout << "[CGPSInterface::OnConnectionEstablished] JAVAD/TopCon commands sent successfully." << endl;
		return true;
	}
	else
	{
		return true; // Nothing extra
	}
}
