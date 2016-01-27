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

MRPT_TODO("store originalReceivedTimestamp");


void  CGPSInterface::implement_parser_NMEA()
{
#if 0
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
			std::string		str;
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
#endif
}

/* -----------------------------------------------------
					parse_NMEA
----------------------------------------------------- */
bool CGPSInterface::parse_NMEA(const std::string &s, mrpt::obs::CObservationGPS &out_obs, const bool verbose)
{
	if (verbose)
		cout << "[CGPSInterface] GPS raw string: " << s << endl;

	// Firstly! If the string does not start with "$GP" it is not valid:
	if (s.size()<7) return false;
	if ( s[0]!='$' || s[1]!='G' ) return false;

	std::vector<std::string> lstTokens;
	mrpt::system::tokenize(s," *,\t\r\n",lstTokens, false /* do not skip blank tokens */);
	if (lstTokens.size()<3) return false;


	bool parsed_ok = false;
	// Try to determine the kind of command:
	if (lstTokens[0]=="$GPGGA" && lstTokens.size()>=13)
	{
		// ---------------------------------------------
		//					GGA
		// ---------------------------------------------
		bool all_fields_ok=true;
		std::string  token;

		// Fill out the output structure:
		gnss::Message_NMEA_GGA gga;

		// Time:
		token = lstTokens[1];
		if (token.size()>=6)
		{
			gga.fields.UTCTime.hour		= 10 * (token[0]-'0') + token[1]-'0';
			gga.fields.UTCTime.minute	= 10 * (token[2]-'0') + token[3]-'0';
			gga.fields.UTCTime.sec		= atof( & (token.c_str()[4]) );
		}
		else all_fields_ok = false;

		// Latitude:
		token = lstTokens[2];
		if (token.size()>=4)
		{
			double	lat = 10 * (token[0]-'0') + token[1]-'0';
			lat += atof( & (token.c_str()[2]) ) / 60.0;
			gga.fields.latitude_degrees = lat;
		}
		else all_fields_ok = false;

		// N/S:
		token = lstTokens[3];
		if (token.empty())
			all_fields_ok = false;
		else if (token[0]=='S')
			gga.fields.latitude_degrees = -gga.fields.latitude_degrees;

		// Longitude:
		token = lstTokens[4];
		if (token.size()>=5)
		{
			double	lat = 100 * (token[0]-'0') + 10 * (token[1]-'0')+ token[2]-'0';
			lat += atof( & (token.c_str()[3]) ) / 60.0;
			gga.fields.longitude_degrees = lat;
		}
		else all_fields_ok = false;

		// E_W:
		token = lstTokens[5];
		if (token.empty())
			all_fields_ok = false;
		else
		if (token[0]=='W')
			gga.fields.longitude_degrees = -gga.fields.longitude_degrees;

		// fix quality:
		token = lstTokens[6];
		if (!token.empty())
			gga.fields.fix_quality = (unsigned char)atoi(token.c_str());

		// sats:
		token = lstTokens[7];
		if (!token.empty())
			gga.fields.satellitesUsed = (unsigned char)atoi( token.c_str() );

		// HDOP:
		token = lstTokens[8];
		if (!token.empty())
		{
			gga.fields.HDOP = (float)atof( token.c_str() );
			gga.fields.thereis_HDOP = true;
		}

		// Altitude:
		token = lstTokens[9];
		if (token.empty()) all_fields_ok = false;
		else gga.fields.altitude_meters = atof( token.c_str() );

		// Units of the altitude:
//		token = lstTokens[10];
//		ASSERT_(token == "M");

		// Geoidal separation [B] (undulation)
		token = lstTokens[11];
		if (!token.empty())
			gga.fields.geoidal_distance = atof( token.c_str() );

		// Units of the geoidal separation:
//		token = lstTokens[12];
//		ASSERT_(token == "M");

		// Total altitude [A]+[B] and mmGPS Corrected total altitude Corr([A]+[B]):
		gga.fields.orthometric_altitude =
		gga.fields.corrected_orthometric_altitude =
		gga.fields.altitude_meters + gga.fields.geoidal_distance;

		if (all_fields_ok) {
			out_obs.setMsg(gga);
			out_obs.timestamp = gga.fields.UTCTime.getAsTimestamp( mrpt::system::now() );
		}
		parsed_ok = all_fields_ok;
	}
	else
	{
		// Try to determine the kind of command:
		if ( lstTokens[0]=="$GPRMC" && lstTokens.size()>=13)
		{
			// ---------------------------------------------
			//					GPRMC
			// ---------------------------------------------
			bool all_fields_ok = true;
			std::string token;

			// Rellenar la estructura de "ultimo dato RMC recibido"
			// Fill out the output structure:
			gnss::Message_NMEA_RMC rmc;

			// Time:
			token = lstTokens[1];
			if (token.size()>=6)
			{
				rmc.fields.UTCTime.hour		= 10 * (token[0]-'0') + token[1]-'0';
				rmc.fields.UTCTime.minute	= 10 * (token[2]-'0') + token[3]-'0';
				rmc.fields.UTCTime.sec		= atof( & (token.c_str()[4]) );
			}
			else all_fields_ok = false;

			// Valid?
			token = lstTokens[2];
			if (token.empty()) all_fields_ok = false;
			else rmc.fields.validity_char = token.c_str()[0];

			// Latitude:
			token = lstTokens[3];
			if (token.size()>=4)
			{
				double	lat = 10 * (token[0]-'0') + token[1]-'0';
				lat += atof( & (token.c_str()[2]) ) / 60.0;
				rmc.fields.latitude_degrees = lat;
			}
			else all_fields_ok = false;

			// N/S:
			token = lstTokens[4];
			if (token.empty()) all_fields_ok = false;
			else if (token[0]=='S')
				rmc.fields.latitude_degrees = -rmc.fields.latitude_degrees;

			// Longitude:
			token = lstTokens[5];
			if (token.size()>=5)
			{
				double	lat = 100 * (token[0]-'0') + 10 * (token[1]-'0')+ token[2]-'0';
				lat += atof( & (token.c_str()[3]) ) / 60.0;
				rmc.fields.longitude_degrees = lat;
			}
			else all_fields_ok = false;

			// E/W:
			token = lstTokens[6];
			if (token.empty()) all_fields_ok = false;
			else if (token[0]=='W')
				rmc.fields.longitude_degrees = -rmc.fields.longitude_degrees;

			// Speed:
			token = lstTokens[7];
			if (!token.empty()) rmc.fields.speed_knots = atof( token.c_str() );

			// Direction:
			token = lstTokens[8];
			if (!token.empty()) rmc.fields.direction_degrees= atof( token.c_str() );

			// Date:
			token = lstTokens[9];
			if (token.size()>=6) {
				rmc.fields.date_day   = 10 * (token[0]-'0') + token[1]-'0';
				rmc.fields.date_month = 10 * (token[2]-'0') + token[3]-'0';
				rmc.fields.date_year  = atoi( & (token.c_str()[4]) );
			}
			else all_fields_ok = false;

			// Magnatic var
			token = lstTokens[10];
			if (token.size()>=2)
				rmc.fields.magnetic_dir = atof(token.c_str());
			else all_fields_ok = false;

			// E/W:
			token = lstTokens[11];
			if (token.empty()) all_fields_ok = false;
			else if (token[0]=='W')
				rmc.fields.magnetic_dir = -rmc.fields.magnetic_dir;

			// Mode ind.
			token = lstTokens[12];
			if (token.empty()) all_fields_ok = false;
			else rmc.fields.positioning_mode = token.c_str()[0];

			if (all_fields_ok) {
				out_obs.setMsg(rmc);
				out_obs.timestamp = rmc.fields.UTCTime.getAsTimestamp( mrpt::system::now() );
			}
			parsed_ok = all_fields_ok;
		}
		else
		{
			// ... parse other commands
		}
	}

	return parsed_ok;
}

