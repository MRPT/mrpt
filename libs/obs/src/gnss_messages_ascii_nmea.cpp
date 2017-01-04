/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/gnss_messages_ascii_nmea.h>

using namespace std;
using namespace mrpt::obs::gnss;

// ---------------------------------------
Message_NMEA_GGA::content_t::content_t() :
	UTCTime(),
	latitude_degrees(0),
	longitude_degrees(0),
	fix_quality(0),
	altitude_meters(0),
	geoidal_distance(),
	orthometric_altitude(),
	corrected_orthometric_altitude(),
	satellitesUsed(0),
	thereis_HDOP(false),
	HDOP(0)
{ }

void Message_NMEA_GGA::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("[NMEA GGA datum]\n");
	out.printf("  Longitude: %.09f deg  Latitude: %.09f deg  Height: %.03f m\n",
		fields.longitude_degrees,
		fields.latitude_degrees,
		fields.altitude_meters );

	out.printf("  Geoidal distance: %.03f m  Orthometric alt.: %.03f m  Corrected ort. alt.: %.03f m\n",
		fields.geoidal_distance,
		fields.orthometric_altitude,
		fields.corrected_orthometric_altitude );

	out.printf("  UTC time-stamp: %02u:%02u:%02.03f  #sats=%2u  ",
		fields.UTCTime.hour,
		fields.UTCTime.minute,
		fields.UTCTime.sec,
		fields.satellitesUsed );

	out.printf("Fix mode: %u ",fields.fix_quality);
	switch( fields.fix_quality )
	{
		case 0: out.printf("(Invalid)\n"); break;
		case 1: out.printf("(GPS fix)\n"); break;
		case 2: out.printf("(DGPS fix)\n"); break;
		case 3: out.printf("(PPS fix)\n"); break;
		case 4: out.printf("(Real Time Kinematic/RTK Fixed)\n"); break;
		case 5: out.printf("(Real Time Kinematic/RTK Float)\n"); break;
		case 6: out.printf("(Dead Reckoning)\n"); break;
		case 7: out.printf("(Manual)\n"); break;
		case 8: out.printf("(Simulation)\n"); break;
		case 9: out.printf("(mmGPS + RTK Fixed)\n"); break;
		case 10: out.printf("(mmGPS + RTK Float)\n"); break;
		default: out.printf("(UNKNOWN!)\n"); break;
	};

	out.printf("  HDOP (Horizontal Dilution of Precision): ");
	if (fields.thereis_HDOP)
			out.printf(" %f\n", fields.HDOP);
	else 	out.printf(" N/A\n");
}

bool Message_NMEA_GGA::getAllFieldDescriptions( std::ostream &o ) const
{
	o << "lon_deg lat_deg hgt_m undulation_m hour min sec num_sats fix_quality hdop";
	return true;
}
bool Message_NMEA_GGA::getAllFieldValues( std::ostream &o ) const
{
	o << mrpt::format("%.09f %.09f %.04f %.04f %02u %02u %02.03f %2u %u %f",
		fields.longitude_degrees,
		fields.latitude_degrees,
		fields.altitude_meters,
		fields.geoidal_distance,
		fields.UTCTime.hour,
		fields.UTCTime.minute,
		fields.UTCTime.sec,
		fields.satellitesUsed,
		fields.fix_quality,
		fields.HDOP
		);
	return true;
}

// ---------------------------------------
Message_NMEA_GLL::content_t::content_t() :
	UTCTime(),
	latitude_degrees(0),
	longitude_degrees(0),
	validity_char('V')
{ }

void Message_NMEA_GLL::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("[NMEA GLL datum]\n");
	out.printf("  Longitude: %.09f deg  Latitude: %.09f deg Validity: '%c'\n",
		fields.longitude_degrees,
		fields.latitude_degrees,
		fields.validity_char );
	out.printf("  UTC time-stamp: %02u:%02u:%02.03f\n",
		fields.UTCTime.hour,
		fields.UTCTime.minute,
		fields.UTCTime.sec);
}

bool Message_NMEA_GLL::getAllFieldDescriptions( std::ostream &o ) const
{
	o << "lon_deg lat_deg hour min sec validity";
	return true;
}
bool Message_NMEA_GLL::getAllFieldValues( std::ostream &o ) const
{
	o << mrpt::format("%.09f %.09f %02u %02u %02.03f %u",
		fields.longitude_degrees,
		fields.latitude_degrees,
		fields.UTCTime.hour,
		fields.UTCTime.minute,
		fields.UTCTime.sec,
		static_cast<unsigned int>(fields.validity_char=='A' ? 1:0)
		);
	return true;
}

// ---------------------------------------
Message_NMEA_VTG::content_t::content_t() :
	true_track(), magnetic_track(),ground_speed_knots(), ground_speed_kmh()
{ }

void Message_NMEA_VTG::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("[NMEA VTG datum]\n");
	out.printf("  True track: %.03f deg  Magnetic track: %.03f deg\n",fields.true_track, fields.magnetic_track);
	out.printf("  Ground speed: %.03f knots  %.03f km/h\n",fields.ground_speed_knots, fields.ground_speed_kmh);
}

bool Message_NMEA_VTG::getAllFieldDescriptions( std::ostream &o ) const
{
	o << "true_track mag_track gnd_speed_knots gnd_speed_kmh";
	return true;
}
bool Message_NMEA_VTG::getAllFieldValues( std::ostream &o ) const
{
	o << mrpt::format("%.09f %.09f %.09f %.09f",fields.true_track, fields.magnetic_track, fields.ground_speed_knots, fields.ground_speed_kmh);
	return true;
}

// ---------------------------------------
Message_NMEA_RMC::content_t::content_t() :
	UTCTime(),
	validity_char('V'),
	latitude_degrees(0),
	longitude_degrees(0),
	speed_knots(0),
	direction_degrees(0),
	date_day(0),
	date_month(0),
	date_year(0),
	magnetic_dir(),
	positioning_mode('N')
{ }

//!< Build an MRPT timestamp with the year/month/day of this observation.
mrpt::system::TTimeStamp Message_NMEA_RMC::getDateAsTimestamp() const
{
	using namespace mrpt::system;

	// Detect current century:
	uint16_t years_century; {
		TTimeParts dec_parts;
		timestampToParts(now(), dec_parts);
		years_century = (dec_parts.year/100)*100;
	}

	TTimeParts parts;
	parts.second = 
	parts.minute = 
	parts.hour = 0;

	parts.day   = fields.date_day;
	parts.month = fields.date_month;
	parts.year  = years_century + fields.date_year;

	return buildTimestampFromParts(parts);
}

void Message_NMEA_RMC::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("[NMEA RMC datum]\n");
	out.printf(" Positioning mode: `%c`\n ", (char)fields.positioning_mode);
	out.printf(" UTC time-stamp: %02u:%02u:%02.03f\n",
		fields.UTCTime.hour,
		fields.UTCTime.minute,
		fields.UTCTime.sec
		);
	out.printf(" Date (DD/MM/YY): %02u/%02u/%02u\n ",
		(unsigned)fields.date_day,(unsigned)fields.date_month, (unsigned)fields.date_year);
	out.printf("  Longitude: %.09f deg  Latitude: %.09f deg  Valid?: '%c'\n",
		fields.longitude_degrees,
		fields.latitude_degrees,
		fields.validity_char
		);
	out.printf(" Speed: %.05f knots  Direction:%.03f deg.\n ",
		fields.speed_knots,
		fields.direction_degrees
		);
	out.printf(" Magnetic variation direction: %.04f deg\n ", fields.magnetic_dir);
}

bool Message_NMEA_RMC::getAllFieldDescriptions( std::ostream &o ) const
{
	o << "lon_deg lat_deg hour min sec speed_knots direction_deg year month day";
	return true;
}
bool Message_NMEA_RMC::getAllFieldValues( std::ostream &o ) const
{
	o << mrpt::format("%.09f %.09f %02u %02u %02.03f %.05f %.03f %02u %02u %02u",
		fields.longitude_degrees,
		fields.latitude_degrees,
		fields.UTCTime.hour,
		fields.UTCTime.minute,
		fields.UTCTime.sec,
		fields.speed_knots,
		fields.direction_degrees,
		fields.date_year,
		fields.date_month,
		fields.date_day
		);
	return true;
}

// ---------------------------------------
Message_NMEA_ZDA::content_t::content_t() :
	UTCTime(), date_day(),date_month(),date_year()
{ }

void Message_NMEA_ZDA::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("[NMEA ZDA datum]\n");
	out.printf(" UTC time-stamp: %02u:%02u:%02.03f\n",
		fields.UTCTime.hour,
		fields.UTCTime.minute,
		fields.UTCTime.sec
		);
	out.printf(" Date (DD/MM/YY): %02u/%02u/%04u\n ",
		(unsigned)fields.date_day,(unsigned)fields.date_month, (unsigned)fields.date_year);
}

bool Message_NMEA_ZDA::getAllFieldDescriptions( std::ostream &o ) const
{
	o << "year month day hour minute second";
	return true;
}
bool Message_NMEA_ZDA::getAllFieldValues( std::ostream &o ) const
{
	o << mrpt::format("%04u %02u %02u %02u %02u %.05f",
		fields.date_year,
		fields.date_month,
		fields.date_day,
		fields.UTCTime.hour,
		fields.UTCTime.minute,
		fields.UTCTime.sec);
	return true;
}

mrpt::system::TTimeStamp Message_NMEA_ZDA::getDateTimeAsTimestamp() const
{
	return fields.UTCTime.getAsTimestamp( this->getDateAsTimestamp() );
}

//!< Build an MRPT timestamp with the year/month/day of this observation.
mrpt::system::TTimeStamp Message_NMEA_ZDA::getDateAsTimestamp() const
{
	using namespace mrpt::system;
	TTimeParts parts;
	parts.second = 
	parts.minute = 
	parts.hour = 0;
	parts.day   = fields.date_day;
	parts.month = fields.date_month;
	parts.year  = fields.date_year;
	return buildTimestampFromParts(parts);
}
