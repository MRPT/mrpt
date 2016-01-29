/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
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

