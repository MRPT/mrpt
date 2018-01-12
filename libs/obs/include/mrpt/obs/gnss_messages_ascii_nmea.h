/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include "gnss_messages_common.h"

namespace mrpt
{
namespace obs
{
namespace gnss
{
// Pragma to ensure we can safely serialize some of these structures
#pragma pack(push, 1)

/** NMEA datum: GGA. \sa mrpt::obs::CObservationGPS  */
struct Message_NMEA_GGA : public gnss_message
{
	GNSS_MESSAGE_BINARY_BLOCK(&fields, sizeof(fields))
	/** Static msg type (member expected by templates) */
	enum
	{
		msg_type = NMEA_GGA
	};
	Message_NMEA_GGA() : gnss_message((gnss_message_type_t)msg_type) {}
	struct content_t
	{
		/** The GPS sensor measured timestamp (in UTC time) */
		UTC_time UTCTime;
		/** The measured latitude, in degrees (North:+ , South:-) */
		double latitude_degrees;
		/** The measured longitude, in degrees (East:+ , West:-) */
		double longitude_degrees;
		/** NMEA standard values: 0 = invalid, 1 = GPS fix (SPS), 2 = DGPS fix,
		 * 3 = PPS fix, 4 = Real Time Kinematic, 5 = Float RTK, 6 = estimated
		 * (dead reckoning) (2.3 feature), 7 = Manual input mode, 8 = Simulation
		 * mode */
		uint8_t fix_quality;
		/** The measured altitude, in meters (A). */
		double altitude_meters;
		/** Undulation: Difference between the measured altitude and the geoid,
		 * in meters (B). */
		double geoidal_distance;
		/** The measured orthometric altitude, in meters (A)+(B). */
		double orthometric_altitude;
		/** The corrected (only for TopCon mmGPS) orthometric altitude, in
		 * meters mmGPS(A+B). */
		double corrected_orthometric_altitude;
		/** The number of satelites used to compute this estimation. */
		uint32_t satellitesUsed;
		/** This states whether to take into account the value in the HDOP
		 * field. */
		bool thereis_HDOP;
		/** The HDOP (Horizontal Dilution of Precision) as returned by the
		 * sensor. */
		float HDOP;

		content_t();
	};
	/** Message content, accesible by individual fields */
	content_t fields;

	void dumpToStream(std::ostream& out) const override;  // See docs in base

	/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords
	 * structure (requires linking against mrpt-topography)
	 *   Call as: getAsStruct<TGeodeticCoords>(); */
	template <class TGEODETICCOORDS>
	inline TGEODETICCOORDS getOrthoAsStruct() const
	{
		return TGEODETICCOORDS(
			fields.latitude_degrees, fields.longitude_degrees,
			fields.orthometric_altitude);
	}
	/**  Return the corrected geodetic coords as a
	 * mrpt::topography::TGeodeticCoords structure (requires linking against
	 * mrpt-topography)
	 *   Call as: getAsStruct<TGeodeticCoords>(); */
	template <class TGEODETICCOORDS>
	inline TGEODETICCOORDS getCorrectedOrthoAsStruct() const
	{
		return TGEODETICCOORDS(
			fields.latitude_degrees, fields.longitude_degrees,
			fields.corrected_orthometric_altitude);
	}
	/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords
	 * structure (requires linking against mrpt-topography)
	 *   Call as: getAsStruct<TGeodeticCoords>(); */
	template <class TGEODETICCOORDS>
	inline TGEODETICCOORDS getAsStruct() const
	{
		return TGEODETICCOORDS(
			fields.latitude_degrees, fields.longitude_degrees,
			fields.altitude_meters);
	}

	bool getAllFieldDescriptions(std::ostream& o) const override;
	bool getAllFieldValues(std::ostream& o) const override;
};

/** NMEA datum: GLL. \sa mrpt::obs::CObservationGPS  */
struct Message_NMEA_GLL : public gnss_message
{
	GNSS_MESSAGE_BINARY_BLOCK(&fields, sizeof(fields))
	/** Static msg type (member expected by templates) */
	enum
	{
		msg_type = NMEA_GLL
	};
	Message_NMEA_GLL() : gnss_message((gnss_message_type_t)msg_type) {}
	struct content_t
	{
		/** The GPS sensor measured timestamp (in UTC time) */
		UTC_time UTCTime;
		/** The measured latitude, in degrees (North:+ , South:-) */
		double latitude_degrees;
		/** The measured longitude, in degrees (East:+ , West:-) */
		double longitude_degrees;
		/** This will be: 'A'=OK or 'V'=void */
		int8_t validity_char;
		content_t();
	};
	/** Message content, accesible by individual fields */
	content_t fields;
	void dumpToStream(std::ostream& out) const override;  // See docs in base
	bool getAllFieldDescriptions(std::ostream& o) const override;
	bool getAllFieldValues(std::ostream& o) const override;
};

/** NMEA datum: RMC. \sa mrpt::obs::CObservationGPS   */
struct Message_NMEA_RMC : public gnss_message
{
	GNSS_MESSAGE_BINARY_BLOCK(&fields, sizeof(fields))
	/** Static msg type (member expected by templates) */
	enum
	{
		msg_type = NMEA_RMC
	};
	Message_NMEA_RMC() : gnss_message((gnss_message_type_t)msg_type) {}
	struct content_t
	{
		/** The GPS sensor measured timestamp (in UTC time) */
		UTC_time UTCTime;
		/** This will be: 'A'=OK or 'V'=void */
		int8_t validity_char;
		/** The measured latitude, in degrees (North:+ , South:-) */
		double latitude_degrees;
		/** The measured longitude, in degrees (East:+ , West:-) */
		double longitude_degrees;
		/** Measured speed (in knots) */
		double speed_knots;
		/** Measured speed direction (in degrees) */
		double direction_degrees;
		/** Date: day (1-31), month (1-12), two-digits year (00-99) */
		uint8_t date_day, date_month, date_year;
		/** Magnetic variation direction (East:+, West:-) */
		double magnetic_dir;
		/** 'A': Autonomous, 'D': Differential, 'N': Not valid, 'E': Estimated,
		 * 'M': Manual */
		char positioning_mode;
		content_t();
	};
	/** Message content, accesible by individual fields */
	content_t fields;

	/** Build an MRPT timestamp with the year/month/day of this observation. */
	mrpt::system::TTimeStamp getDateAsTimestamp() const;

	void dumpToStream(std::ostream& out) const override;  // See docs in base
	bool getAllFieldDescriptions(std::ostream& o) const override;
	bool getAllFieldValues(std::ostream& o) const override;
};

/** NMEA datum: VTG. \sa mrpt::obs::CObservationGPS  */
struct Message_NMEA_VTG : public gnss_message
{
	GNSS_MESSAGE_BINARY_BLOCK(&fields, sizeof(fields))
	/** Static msg type (member expected by templates) */
	enum
	{
		msg_type = NMEA_VTG
	};
	Message_NMEA_VTG() : gnss_message((gnss_message_type_t)msg_type) {}
	struct content_t
	{
		/** Degrees */
		double true_track, magnetic_track;
		double ground_speed_knots, ground_speed_kmh;
		content_t();
	};
	/** Message content, accesible by individual fields */
	content_t fields;
	void dumpToStream(std::ostream& out) const override;  // See docs in base
	bool getAllFieldDescriptions(std::ostream& o) const override;
	bool getAllFieldValues(std::ostream& o) const override;
};

/** NMEA datum: ZDA. \sa mrpt::obs::CObservationGPS   */
struct Message_NMEA_ZDA : public gnss_message
{
	GNSS_MESSAGE_BINARY_BLOCK(&fields, sizeof(fields))
	/** Static msg type (member expected by templates) */
	enum
	{
		msg_type = NMEA_ZDA
	};
	Message_NMEA_ZDA() : gnss_message((gnss_message_type_t)msg_type) {}
	struct content_t
	{
		/** The GPS sensor measured timestamp (in UTC time) */
		UTC_time UTCTime;
		/** 1-31 */
		uint8_t date_day;
		/** 1-12 */
		uint8_t date_month;
		/** 2000-... */
		uint16_t date_year;
		content_t();
	};
	/** Message content, accesible by individual fields */
	content_t fields;

	/** Build an MRPT UTC timestamp with the year/month/day + hour/minute/sec of
	 * this observation. */
	mrpt::system::TTimeStamp getDateTimeAsTimestamp() const;
	/** Build an MRPT timestamp with the year/month/day of this observation. */
	mrpt::system::TTimeStamp getDateAsTimestamp() const;

	void dumpToStream(std::ostream& out) const override;  // See docs in base
	bool getAllFieldDescriptions(std::ostream& o) const override;
	bool getAllFieldValues(std::ostream& o) const override;
};
#pragma pack(pop)  // End of pack = 1
}  // namespace gnss
}  // namespace obs
}  // namespace mrpt
