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
#include <mrpt/math/CMatrixFixedNumeric.h>

namespace mrpt::obs::gnss
{
/** GPS datum for TopCon's mmGPS devices: PZS. \sa mrpt::obs::CObservationGPS */
struct Message_TOPCON_PZS : public gnss_message
{
	/** Static msg type (member expected by templates) */
	enum
	{
		msg_type = TOPCON_PZS
	};

	/** The measured latitude, in degrees (North:+ , South:-) */
	double latitude_degrees{0};
	/** The measured longitude, in degrees (East:+ , West:-) */
	double longitude_degrees{0};
	/** ellipsoidal height from N-beam [m] perhaps weighted with regular gps */
	double height_meters{0};
	/** ellipsoidal height [m] without N-beam correction */
	double RTK_height_meters{0};
	/** position SEP [m] */
	float PSigma{0};
	/** Vertical angle of N-beam */
	double angle_transmitter{0};
	/** ID of the transmitter [1-4], 0 if none. */
	uint8_t nId{0};
	/** 1: GPS, 2: mmGPS */
	uint8_t Fix{0};
	/** battery level on transmitter */
	uint8_t TXBattery{0};
	/** battery level on receiver */
	uint8_t RXBattery{0};
	uint8_t error{0};  //! system error indicator

	bool hasCartesianPosVel{false};
	/** Only if hasCartesianPosVel is true */
	double cartesian_x{0}, cartesian_y{0}, cartesian_z{0};
	/** Only if hasCartesianPosVel is true */
	double cartesian_vx{0}, cartesian_vy{0}, cartesian_vz{0};

	bool hasPosCov{false};
	/** Only if hasPosCov is true */
	mrpt::math::CMatrixFloat44 pos_covariance;

	bool hasVelCov{false};
	/** Only if hasPosCov is true */
	mrpt::math::CMatrixFloat44 vel_covariance;

	bool hasStats{false};
	uint8_t stats_GPS_sats_used{0},
		stats_GLONASS_sats_used{0};  //<! Only if hasStats is true
	/** [0,100] %, only in modes other than RTK FIXED. */
	uint8_t stats_rtk_fix_progress;

	Message_TOPCON_PZS();
	/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords
	 * structure (requires linking against mrpt-topography)
	 *   Call as: getAsStruct<TGeodeticCoords>(); */
	template <class TGEODETICCOORDS>
	inline TGEODETICCOORDS getAsStruct() const
	{
		return TGEODETICCOORDS(
			latitude_degrees, longitude_degrees, height_meters);
	}
	void dumpToStream(std::ostream& out) const override;  // See docs in base

   protected:
	void internal_writeToStream(
		mrpt::serialization::CArchive& out) const override;
	void internal_readFromStream(mrpt::serialization::CArchive& in) override;

   public:
	MRPT_MAKE_ALIGNED_OPERATOR_NEW
};

/** TopCon mmGPS devices: SATS, a generic structure for statistics about tracked
 * satelites and their positions. \sa mrpt::obs::CObservationGPS   */
struct Message_TOPCON_SATS : public gnss_message
{
	/** Static msg type (member expected by templates) */
	enum
	{
		msg_type = TOPCON_SATS
	};

	Message_TOPCON_SATS();

	/** The list of USI (Universal Sat ID) for the detected sats (See GRIL
	 * Manual, pag 4-31). */
	std::vector<uint8_t> USIs;
	/** Elevation (in degrees, 0-90) for each satellite in USIs. */
	std::vector<int8_t> ELs;
	/** Azimuth (in degrees, 0-360) for each satellite in USIs. */
	std::vector<int16_t> AZs;

	void dumpToStream(std::ostream& out) const override;  // See docs in base
   protected:
	void internal_writeToStream(
		mrpt::serialization::CArchive& out) const override;
	void internal_readFromStream(mrpt::serialization::CArchive& in) override;
};
}  // namespace mrpt::obs::gnss
