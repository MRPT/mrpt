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

using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace std;

const size_t MAX_NMEA_LINE_LENGTH = 1024;

bool  CGPSInterface::implement_parser_NMEA(size_t &out_minimum_rx_buf_to_decide)
{
	out_minimum_rx_buf_to_decide = 3;

	if (m_rx_buffer.size()<3)
		return true; // no need to skip a byte

	const size_t nBytesAval = m_rx_buffer.size();  // Available for read

	// If the string does not start with "$GP" it is not valid:
	uint8_t peek_buffer[3];
	m_rx_buffer.peek_many(&peek_buffer[0],3);
	if (peek_buffer[0]!='$' || peek_buffer[1]!='G' || peek_buffer[2]!='P') {
		// Not the start of a NMEA string, skip 1 char:
		return false;
	}
	else 
	{
		// It starts OK: try to find the end of the line
		std::string  line;
		bool line_is_ended = false;
		for (size_t i=0;i<nBytesAval && i<MAX_NMEA_LINE_LENGTH;i++)
		{
			const char val = static_cast<char>(m_rx_buffer.peek(i));
			if (val=='\r' || val=='\n') {
				line_is_ended = true;
				break;
			}
			line.push_back(val);
		}
		if (line_is_ended)
		{
			// Pop from buffer:
			for (size_t i=0;i<line.size();i++) m_rx_buffer.pop();

			// Parse:
			const bool did_have_gga = m_just_parsed_messages.has_GGA_datum;
			if (CGPSInterface::parse_NMEA(line,m_just_parsed_messages, false /*verbose*/))
			{
				// Parsers must set only the part of the msg type:
				m_just_parsed_messages.sensorLabel = "NMEA";

				// Save GGA cache (useful for NTRIP,...)
				const bool now_has_gga = m_just_parsed_messages.has_GGA_datum;
				if (now_has_gga && !did_have_gga) {
					m_last_GGA = line;
				}
			}
			else
			{
				if (m_verbose)
					std::cerr << "[CGPSInterface::implement_parser_NMEA] Line of unknown format ignored: `"<<line<<"`\n";
			}
			return true;
		}
		else
		{
			// We still need to wait for more data to be read:
			out_minimum_rx_buf_to_decide = nBytesAval+1;
			return true;
		}
	}
}

/* -----------------------------------------------------
					parse_NMEA
----------------------------------------------------- */
bool CGPSInterface::parse_NMEA(const std::string &s, mrpt::obs::CObservationGPS &out_obs, const bool verbose)
{
	static mrpt::system::TTimeStamp last_known_date = mrpt::system::now(); // For building complete date+time in msgs without a date.
	static mrpt::system::TTimeStamp last_known_time = mrpt::system::now();

	if (verbose)
		cout << "[CGPSInterface] GPS raw string: " << s << endl;

	// Firstly! If the string does not start with "$GP" it is not valid:
	if (s.size()<7) return false;
	if ( s[0]!='$' || s[1]!='G' ) return false;

	std::vector<std::string> lstTokens;
	mrpt::system::tokenize(s,"*,\t\r\n",lstTokens, false /* do not skip blank tokens */);
	if (lstTokens.size()<3) return false;

	for (size_t i=0;i<lstTokens.size();i++) lstTokens[i] = mrpt::system::trim(lstTokens[i]); // Trim whitespaces

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
			out_obs.originalReceivedTimestamp = mrpt::system::now();
			out_obs.timestamp = gga.fields.UTCTime.getAsTimestamp( last_known_date );
			out_obs.has_satellite_timestamp = true;
		}
		parsed_ok = all_fields_ok;
	}
	else if ( lstTokens[0]=="$GPRMC" && lstTokens.size()>=13)
	{
		// ---------------------------------------------
		//					GPRMC
		// ---------------------------------------------
		bool all_fields_ok = true;
		std::string token;

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

		// Magnetic var
		token = lstTokens[10];
		if (token.size()>=2) 
		{
			rmc.fields.magnetic_dir = atof(token.c_str());
			// E/W:
			token = lstTokens[11];
			if (token.empty()) all_fields_ok = false;
			else if (token[0]=='W')
				rmc.fields.magnetic_dir = -rmc.fields.magnetic_dir;
		}

		// Mode ind.
		if (lstTokens.size()>=14) {
			// Only for NMEA 2.3
			token = lstTokens[12];
			if (token.empty()) all_fields_ok = false;
			else rmc.fields.positioning_mode = token.c_str()[0];
		} else rmc.fields.positioning_mode = 'A'; // Default for older receiver

		if (all_fields_ok) {
			out_obs.setMsg(rmc);
			out_obs.originalReceivedTimestamp = mrpt::system::now();
			out_obs.timestamp = rmc.fields.UTCTime.getAsTimestamp( rmc.getDateAsTimestamp() );
			last_known_date = rmc.getDateAsTimestamp();
			last_known_time = out_obs.timestamp;
			out_obs.has_satellite_timestamp = true;
		}
		parsed_ok = all_fields_ok;
	}
	else if ( lstTokens[0]=="$GPGLL" && lstTokens.size()>=5)
	{
		// ---------------------------------------------
		//					GPGLL
		// ---------------------------------------------
		bool all_fields_ok = true;
		std::string token;

		// Fill out the output structure:
		gnss::Message_NMEA_GLL gll;
		// Latitude:
		token = lstTokens[1];
		if (token.size()>=4)
		{
			double	lat = 10 * (token[0]-'0') + token[1]-'0';
			lat += atof( & (token.c_str()[2]) ) / 60.0;
			gll.fields.latitude_degrees = lat;
		}
		else all_fields_ok = false;

		// N/S:
		token = lstTokens[2];
		if (token.empty()) all_fields_ok = false;
		else if (token[0]=='S')
			gll.fields.latitude_degrees = -gll.fields.latitude_degrees;

		// Longitude:
		token = lstTokens[3];
		if (token.size()>=5)
		{
			double	lat = 100 * (token[0]-'0') + 10 * (token[1]-'0')+ token[2]-'0';
			lat += atof( & (token.c_str()[3]) ) / 60.0;
			gll.fields.longitude_degrees = lat;
		}
		else all_fields_ok = false;

		// E/W:
		token = lstTokens[4];
		if (token.empty()) all_fields_ok = false;
		else if (token[0]=='W')
			gll.fields.longitude_degrees = -gll.fields.longitude_degrees;

		if (lstTokens.size()>=7) {
			// Time:
			token = lstTokens[5];
			if (token.size()>=6)
			{
				gll.fields.UTCTime.hour		= 10 * (token[0]-'0') + token[1]-'0';
				gll.fields.UTCTime.minute	= 10 * (token[2]-'0') + token[3]-'0';
				gll.fields.UTCTime.sec		= atof( & (token.c_str()[4]) );
			}
			else all_fields_ok = false;

			// Valid?
			token = lstTokens[6];
			if (token.empty()) all_fields_ok = false;
			else gll.fields.validity_char = token.c_str()[0];
		}

		if (all_fields_ok) {
			out_obs.setMsg(gll);
			out_obs.originalReceivedTimestamp = mrpt::system::now();
			out_obs.timestamp = gll.fields.UTCTime.getAsTimestamp( last_known_date );
			last_known_time = out_obs.timestamp;
			out_obs.has_satellite_timestamp = true;
		}
		parsed_ok = all_fields_ok;
	}
	else if ( lstTokens[0]=="$GPVTG" && lstTokens.size()>=9)
	{
		// ---------------------------------------------
		//					GPVTG
		// ---------------------------------------------
		bool all_fields_ok = true;
		std::string token;

		// Fill out the output structure:
		gnss::Message_NMEA_VTG vtg;

		vtg.fields.true_track = atof(lstTokens[1].c_str());
		vtg.fields.magnetic_track = atof(lstTokens[3].c_str());
		vtg.fields.ground_speed_knots = atof(lstTokens[5].c_str());
		vtg.fields.ground_speed_kmh = atof(lstTokens[7].c_str());

		if (lstTokens[2]!="T" ||lstTokens[4]!="M" ||lstTokens[6]!="N" ||lstTokens[8]!="K")
			all_fields_ok=false;

		if (all_fields_ok) {
			out_obs.setMsg(vtg);
			out_obs.originalReceivedTimestamp = mrpt::system::now();
			out_obs.timestamp = last_known_time;
			out_obs.has_satellite_timestamp = false;
		}
		parsed_ok = all_fields_ok;

	}
	else if ( lstTokens[0]=="$GPZDA" && lstTokens.size()>=5)
	{
		// ---------------------------------------------
		//					GPZDA
		// ---------------------------------------------
		bool all_fields_ok = true;
		std::string token;

		// Fill out the output structure:
		gnss::Message_NMEA_ZDA zda;
		//$--ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx
		//hhmmss.ss = UTC 
		//xx = Day, 01 to 31 
		//xx = Month, 01 to 12 
		//xxxx = Year 
		//xx = Local zone description, 00 to +/- 13 hours 
		//xx = Local zone minutes description (same sign as hours)

		// Time:
		token = lstTokens[1];
		if (token.size()>=6) {
			zda.fields.UTCTime.hour		= 10 * (token[0]-'0') + token[1]-'0';
			zda.fields.UTCTime.minute	= 10 * (token[2]-'0') + token[3]-'0';
			zda.fields.UTCTime.sec		= atof( & (token.c_str()[4]) );
		}
		else all_fields_ok = false;

		// Day:
		token = lstTokens[2];
		if (!token.empty())
			zda.fields.date_day = atoi(token.c_str());
		// Month:
		token = lstTokens[3];
		if (!token.empty())
			zda.fields.date_month = atoi(token.c_str());
		// Year:
		token = lstTokens[4];
		if (!token.empty())
			zda.fields.date_year = atoi(token.c_str());

		if (all_fields_ok) {
			out_obs.setMsg(zda);
			out_obs.originalReceivedTimestamp = mrpt::system::now();
			try {
				out_obs.timestamp = zda.getDateTimeAsTimestamp();
				last_known_date = zda.getDateAsTimestamp();
				out_obs.has_satellite_timestamp = true;
				last_known_time = out_obs.timestamp;
			} catch (...) {
				// Invalid date:
				out_obs.timestamp 
					= out_obs.originalReceivedTimestamp;
			}
		}
		parsed_ok = all_fields_ok;
	}
	else
	{
		// other commands?
	}

	return parsed_ok;
}
