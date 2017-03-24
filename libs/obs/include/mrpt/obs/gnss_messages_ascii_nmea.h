/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include "gnss_messages_common.h"

namespace mrpt {
namespace obs {
namespace gnss {

// Pragma to ensure we can safely serialize some of these structures
#pragma pack(push,1)

/** NMEA datum: GGA. \sa mrpt::obs::CObservationGPS  */
struct OBS_IMPEXP Message_NMEA_GGA : public gnss_message
{
	GNSS_MESSAGE_BINARY_BLOCK(&fields,sizeof(fields))
	enum { msg_type = NMEA_GGA };  //!< Static msg type (member expected by templates)
	Message_NMEA_GGA() : gnss_message((gnss_message_type_t)msg_type)
	{}

	struct OBS_IMPEXP content_t
	{
		UTC_time UTCTime; //!< The GPS sensor measured timestamp (in UTC time)
		double   latitude_degrees; //!< The measured latitude, in degrees (North:+ , South:-)
		double   longitude_degrees; //!< The measured longitude, in degrees (East:+ , West:-)
		uint8_t  fix_quality; //!< NMEA standard values: 0 = invalid, 1 = GPS fix (SPS), 2 = DGPS fix, 3 = PPS fix, 4 = Real Time Kinematic, 5 = Float RTK, 6 = estimated (dead reckoning) (2.3 feature), 7 = Manual input mode, 8 = Simulation mode */
		double   altitude_meters; //!< The measured altitude, in meters (A).
		double   geoidal_distance; //!< Undulation: Difference between the measured altitude and the geoid, in meters (B).
		double   orthometric_altitude; //!< The measured orthometric altitude, in meters (A)+(B).
		double   corrected_orthometric_altitude; //!< The corrected (only for TopCon mmGPS) orthometric altitude, in meters mmGPS(A+B).
		uint32_t satellitesUsed; //!< The number of satelites used to compute this estimation.
		bool     thereis_HDOP; //!< This states whether to take into account the value in the HDOP field.
		float    HDOP; //!< The HDOP (Horizontal Dilution of Precision) as returned by the sensor.

		content_t();
	};
	content_t  fields; //!< Message content, accesible by individual fields
	
	void dumpToStream( mrpt::utils::CStream &out ) const MRPT_OVERRIDE; // See docs in base

	/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords structure (requires linking against mrpt-topography)
		*   Call as: getAsStruct<TGeodeticCoords>(); */
	template <class TGEODETICCOORDS>
	inline TGEODETICCOORDS getOrthoAsStruct() const {
		return TGEODETICCOORDS(fields.latitude_degrees,fields.longitude_degrees,fields.orthometric_altitude);
	}
	/**  Return the corrected geodetic coords as a mrpt::topography::TGeodeticCoords structure (requires linking against mrpt-topography)
		*   Call as: getAsStruct<TGeodeticCoords>(); */
	template <class TGEODETICCOORDS>
	inline TGEODETICCOORDS getCorrectedOrthoAsStruct() const {
		return TGEODETICCOORDS(fields.latitude_degrees,fields.longitude_degrees,fields.corrected_orthometric_altitude);
	}
	/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords structure (requires linking against mrpt-topography)
		*   Call as: getAsStruct<TGeodeticCoords>(); */
	template <class TGEODETICCOORDS>
	inline TGEODETICCOORDS getAsStruct() const {
		return TGEODETICCOORDS(fields.latitude_degrees,fields.longitude_degrees,fields.altitude_meters);
	}

	bool getAllFieldDescriptions( std::ostream &o ) const MRPT_OVERRIDE;
	bool getAllFieldValues( std::ostream &o ) const MRPT_OVERRIDE;
};

/** NMEA datum: GLL. \sa mrpt::obs::CObservationGPS  */
struct OBS_IMPEXP Message_NMEA_GLL : public gnss_message
{
	GNSS_MESSAGE_BINARY_BLOCK(&fields,sizeof(fields))
	enum { msg_type = NMEA_GLL };  //!< Static msg type (member expected by templates)
	Message_NMEA_GLL() : gnss_message((gnss_message_type_t)msg_type)
	{}
	struct OBS_IMPEXP content_t
	{
		UTC_time UTCTime; //!< The GPS sensor measured timestamp (in UTC time)
		double   latitude_degrees; //!< The measured latitude, in degrees (North:+ , South:-)
		double   longitude_degrees; //!< The measured longitude, in degrees (East:+ , West:-)
		int8_t    validity_char; //!< This will be: 'A'=OK or 'V'=void
		content_t();
	};
	content_t  fields; //!< Message content, accesible by individual fields
	void dumpToStream( mrpt::utils::CStream &out ) const MRPT_OVERRIDE; // See docs in base
	bool getAllFieldDescriptions( std::ostream &o ) const MRPT_OVERRIDE;
	bool getAllFieldValues( std::ostream &o ) const MRPT_OVERRIDE;
};

/** NMEA datum: RMC. \sa mrpt::obs::CObservationGPS   */
struct OBS_IMPEXP Message_NMEA_RMC : public gnss_message
{
	GNSS_MESSAGE_BINARY_BLOCK(&fields,sizeof(fields))
	enum { msg_type = NMEA_RMC };  //!< Static msg type (member expected by templates)
	Message_NMEA_RMC() : gnss_message((gnss_message_type_t)msg_type)
	{}

	struct OBS_IMPEXP content_t
	{
		UTC_time  UTCTime;       //!< The GPS sensor measured timestamp (in UTC time)
		int8_t    validity_char; //!< This will be: 'A'=OK or 'V'=void
		double    latitude_degrees; //!< The measured latitude, in degrees (North:+ , South:-)
		double    longitude_degrees; //!< The measured longitude, in degrees (East:+ , West:-)
		double    speed_knots; //!< Measured speed (in knots)
		double    direction_degrees; //!< Measured speed direction (in degrees)
		uint8_t   date_day, date_month,date_year; //!< Date: day (1-31), month (1-12), two-digits year (00-99)
		double    magnetic_dir;      //!< Magnetic variation direction (East:+, West:-)
		char      positioning_mode;  //!< 'A': Autonomous, 'D': Differential, 'N': Not valid, 'E': Estimated, 'M': Manual
		content_t();
	};
	content_t  fields; //!< Message content, accesible by individual fields

	mrpt::system::TTimeStamp getDateAsTimestamp() const; //!< Build an MRPT timestamp with the year/month/day of this observation.

	void dumpToStream( mrpt::utils::CStream &out ) const MRPT_OVERRIDE; // See docs in base
	bool getAllFieldDescriptions( std::ostream &o ) const MRPT_OVERRIDE;
	bool getAllFieldValues( std::ostream &o ) const MRPT_OVERRIDE;
};

/** NMEA datum: VTG. \sa mrpt::obs::CObservationGPS  */
struct OBS_IMPEXP Message_NMEA_VTG : public gnss_message
{
	GNSS_MESSAGE_BINARY_BLOCK(&fields,sizeof(fields))
	enum { msg_type = NMEA_VTG };  //!< Static msg type (member expected by templates)
	Message_NMEA_VTG() : gnss_message((gnss_message_type_t)msg_type)
	{}
	struct OBS_IMPEXP content_t
	{
		double  true_track, magnetic_track; //!< Degrees
		double  ground_speed_knots, ground_speed_kmh;
		content_t();
	};
	content_t  fields; //!< Message content, accesible by individual fields
	void dumpToStream( mrpt::utils::CStream &out ) const MRPT_OVERRIDE; // See docs in base
	bool getAllFieldDescriptions( std::ostream &o ) const MRPT_OVERRIDE;
	bool getAllFieldValues( std::ostream &o ) const MRPT_OVERRIDE;
};

/** NMEA datum: ZDA. \sa mrpt::obs::CObservationGPS   */
struct OBS_IMPEXP Message_NMEA_ZDA : public gnss_message
{
	GNSS_MESSAGE_BINARY_BLOCK(&fields,sizeof(fields))
	enum { msg_type = NMEA_ZDA };  //!< Static msg type (member expected by templates)
	Message_NMEA_ZDA() : gnss_message((gnss_message_type_t)msg_type)
	{}

	struct OBS_IMPEXP content_t
	{
		UTC_time  UTCTime;  //!< The GPS sensor measured timestamp (in UTC time)
		uint8_t   date_day;      //!< 1-31
		uint8_t   date_month;    //!< 1-12
		uint16_t  date_year;     //!< 2000-...
		content_t();
	};
	content_t  fields; //!< Message content, accesible by individual fields

	mrpt::system::TTimeStamp getDateTimeAsTimestamp() const; //!< Build an MRPT UTC timestamp with the year/month/day + hour/minute/sec of this observation.
	mrpt::system::TTimeStamp getDateAsTimestamp() const; //!< Build an MRPT timestamp with the year/month/day of this observation.

	void dumpToStream( mrpt::utils::CStream &out ) const MRPT_OVERRIDE; // See docs in base
	bool getAllFieldDescriptions( std::ostream &o ) const MRPT_OVERRIDE;
	bool getAllFieldValues( std::ostream &o ) const MRPT_OVERRIDE;
};
#pragma pack(pop) // End of pack = 1
} } } // End of namespaces

