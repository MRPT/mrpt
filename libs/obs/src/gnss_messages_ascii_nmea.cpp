/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/gnss_messages_ascii_nmea.h>
#include <iostream>

using namespace std;
using namespace mrpt::obs::gnss;

// ---------------------------------------
Message_NMEA_GGA::content_t::content_t() : UTCTime() {}
void Message_NMEA_GGA::dumpToStream(std::ostream& out) const
{
	out << "[NMEA GGA datum]\n";
	out << mrpt::format(
		"  Longitude: %.09f deg  Latitude: %.09f deg  Height: %.03f m\n",
		fields.longitude_degrees, fields.latitude_degrees,
		fields.altitude_meters);

	out << mrpt::format(
		"  Geoidal distance: %.03f m  Orthometric alt.: %.03f m  Corrected "
		"ort. alt.: %.03f m\n",
		fields.geoidal_distance, fields.orthometric_altitude,
		fields.corrected_orthometric_altitude);

	out << mrpt::format(
		"  UTC time-stamp: %02u:%02u:%02.03f  #sats=%2u  ", fields.UTCTime.hour,
		fields.UTCTime.minute, fields.UTCTime.sec, fields.satellitesUsed);

	out << mrpt::format("Fix mode: %u ", fields.fix_quality);

	const char* fix_names[] = {"0:Invalid",
							   "1:GPS fix",
							   "2:DGPS fix",
							   "3:PPS fix",
							   "4:RTK Fixed",
							   "5:RTK Float",
							   "6:Dead Reckoning",
							   "7:Manual",
							   "8:Simulation",
							   "9:mmGPS + RTK Fixed",
							   "10: mmGPS + RTK Float"};

	if (fields.fix_quality < sizeof(fix_names) / sizeof(fix_names[0]))
		out << "(" << fix_names[fields.fix_quality] << ")\n";
	else
		out << "(UNKNOWN!)\n";

	out << "  HDOP (Horizontal Dilution of Precision): ";
	if (fields.thereis_HDOP)
		out << mrpt::format(" %f\n", fields.HDOP);
	else
		out << " N/A\n";
}

bool Message_NMEA_GGA::getAllFieldDescriptions(std::ostream& o) const
{
	o << "lon_deg lat_deg hgt_m undulation_m hour min sec num_sats fix_quality "
		 "hdop";
	return true;
}
bool Message_NMEA_GGA::getAllFieldValues(std::ostream& o) const
{
	o << mrpt::format(
		"%.09f %.09f %.04f %.04f %02u %02u %02.03f %2u %u %f",
		fields.longitude_degrees, fields.latitude_degrees,
		fields.altitude_meters, fields.geoidal_distance, fields.UTCTime.hour,
		fields.UTCTime.minute, fields.UTCTime.sec, fields.satellitesUsed,
		fields.fix_quality, fields.HDOP);
	return true;
}

// ---------------------------------------
Message_NMEA_GLL::content_t::content_t() : UTCTime() {}
void Message_NMEA_GLL::dumpToStream(std::ostream& out) const
{
	out << "[NMEA GLL datum]\n";
	out << mrpt::format(
		"  Longitude: %.09f deg  Latitude: %.09f deg Validity: '%c'\n",
		fields.longitude_degrees, fields.latitude_degrees,
		fields.validity_char);
	out << mrpt::format(
		"  UTC time-stamp: %02u:%02u:%02.03f\n", fields.UTCTime.hour,
		fields.UTCTime.minute, fields.UTCTime.sec);
}

bool Message_NMEA_GLL::getAllFieldDescriptions(std::ostream& o) const
{
	o << "lon_deg lat_deg hour min sec validity";
	return true;
}
bool Message_NMEA_GLL::getAllFieldValues(std::ostream& o) const
{
	o << mrpt::format(
		"%.09f %.09f %02u %02u %02.03f %u", fields.longitude_degrees,
		fields.latitude_degrees, fields.UTCTime.hour, fields.UTCTime.minute,
		fields.UTCTime.sec,
		static_cast<unsigned int>(fields.validity_char == 'A' ? 1 : 0));
	return true;
}

// ---------------------------------------
Message_NMEA_VTG::content_t::content_t()

	= default;

void Message_NMEA_VTG::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[NMEA VTG datum]\n");
	out << mrpt::format(
		"  True track: %.03f deg  Magnetic track: %.03f deg\n",
		fields.true_track, fields.magnetic_track);
	out << mrpt::format(
		"  Ground speed: %.03f knots  %.03f km/h\n", fields.ground_speed_knots,
		fields.ground_speed_kmh);
}

bool Message_NMEA_VTG::getAllFieldDescriptions(std::ostream& o) const
{
	o << "true_track mag_track gnd_speed_knots gnd_speed_kmh";
	return true;
}
bool Message_NMEA_VTG::getAllFieldValues(std::ostream& o) const
{
	o << mrpt::format(
		"%.09f %.09f %.09f %.09f", fields.true_track, fields.magnetic_track,
		fields.ground_speed_knots, fields.ground_speed_kmh);
	return true;
}

// ---------------------------------------
Message_NMEA_RMC::content_t::content_t() : UTCTime() {}
/** Build an MRPT timestamp with the year/month/day of this observation. */

mrpt::system::TTimeStamp Message_NMEA_RMC::getDateAsTimestamp() const
{
	using namespace mrpt::system;

	// Detect current century:
	uint16_t years_century;
	{
		TTimeParts dec_parts;
		timestampToParts(now(), dec_parts);
		years_century = (dec_parts.year / 100) * 100;
	}

	TTimeParts parts;
	parts.second = parts.minute = parts.hour = 0;

	parts.day = fields.date_day;
	parts.month = fields.date_month;
	parts.year = years_century + fields.date_year;

	return buildTimestampFromParts(parts);
}

void Message_NMEA_RMC::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[NMEA RMC datum]\n");
	out << mrpt::format(
		" Positioning mode: `%c`\n ", (char)fields.positioning_mode);
	out << mrpt::format(
		" UTC time-stamp: %02u:%02u:%02.03f\n", fields.UTCTime.hour,
		fields.UTCTime.minute, fields.UTCTime.sec);
	out << mrpt::format(
		" Date (DD/MM/YY): %02u/%02u/%02u\n ", (unsigned)fields.date_day,
		(unsigned)fields.date_month, (unsigned)fields.date_year);
	out << mrpt::format(
		"  Longitude: %.09f deg  Latitude: %.09f deg  Valid?: '%c'\n",
		fields.longitude_degrees, fields.latitude_degrees,
		fields.validity_char);
	out << mrpt::format(
		" Speed: %.05f knots  Direction:%.03f deg.\n ", fields.speed_knots,
		fields.direction_degrees);
	out << mrpt::format(
		" Magnetic variation direction: %.04f deg\n ", fields.magnetic_dir);
}

bool Message_NMEA_RMC::getAllFieldDescriptions(std::ostream& o) const
{
	o << "lon_deg lat_deg hour min sec speed_knots direction_deg year month "
		 "day";
	return true;
}
bool Message_NMEA_RMC::getAllFieldValues(std::ostream& o) const
{
	o << mrpt::format(
		"%.09f %.09f %02u %02u %02.03f %.05f %.03f %02u %02u %02u",
		fields.longitude_degrees, fields.latitude_degrees, fields.UTCTime.hour,
		fields.UTCTime.minute, fields.UTCTime.sec, fields.speed_knots,
		fields.direction_degrees, fields.date_year, fields.date_month,
		fields.date_day);
	return true;
}

// ---------------------------------------
Message_NMEA_ZDA::content_t::content_t() : UTCTime() {}
void Message_NMEA_ZDA::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("[NMEA ZDA datum]\n");
	out << mrpt::format(
		" UTC time-stamp: %02u:%02u:%02.03f\n", fields.UTCTime.hour,
		fields.UTCTime.minute, fields.UTCTime.sec);
	out << mrpt::format(
		" Date (DD/MM/YY): %02u/%02u/%04u\n ", (unsigned)fields.date_day,
		(unsigned)fields.date_month, (unsigned)fields.date_year);
}

bool Message_NMEA_ZDA::getAllFieldDescriptions(std::ostream& o) const
{
	o << "year month day hour minute second";
	return true;
}
bool Message_NMEA_ZDA::getAllFieldValues(std::ostream& o) const
{
	o << mrpt::format(
		"%04u %02u %02u %02u %02u %.05f", fields.date_year, fields.date_month,
		fields.date_day, fields.UTCTime.hour, fields.UTCTime.minute,
		fields.UTCTime.sec);
	return true;
}

mrpt::system::TTimeStamp Message_NMEA_ZDA::getDateTimeAsTimestamp() const
{
	return fields.UTCTime.getAsTimestamp(this->getDateAsTimestamp());
}

/** Build an MRPT timestamp with the year/month/day of this observation. */

mrpt::system::TTimeStamp Message_NMEA_ZDA::getDateAsTimestamp() const
{
	using namespace mrpt::system;
	TTimeParts parts;
	parts.second = parts.minute = parts.hour = 0;
	parts.day = fields.date_day;
	parts.month = fields.date_month;
	parts.year = fields.date_year;
	return buildTimestampFromParts(parts);
}
