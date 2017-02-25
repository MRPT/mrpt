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
#include <mrpt/math/CMatrixFixedNumeric.h>

namespace mrpt {
namespace obs {
namespace gnss {

/** GPS datum for TopCon's mmGPS devices: PZS. \sa mrpt::obs::CObservationGPS   */
struct OBS_IMPEXP Message_TOPCON_PZS : public gnss_message
{
	enum { msg_type = TOPCON_PZS };  //!< Static msg type (member expected by templates)

	double   latitude_degrees;	//!< The measured latitude, in degrees (North:+ , South:-)
	double   longitude_degrees;	//!< The measured longitude, in degrees (East:+ , West:-)
	double   height_meters;		//!< ellipsoidal height from N-beam [m] perhaps weighted with regular gps
	double   RTK_height_meters;	//!< ellipsoidal height [m] without N-beam correction
	float    PSigma;				//!< position SEP [m]
	double   angle_transmitter;	//!< Vertical angle of N-beam
	uint8_t  nId;		//!< ID of the transmitter [1-4], 0 if none.
	uint8_t  Fix;		//!< 1: GPS, 2: mmGPS
	uint8_t  TXBattery;	//!< battery level on transmitter
	uint8_t  RXBattery;	//!< battery level on receiver
	uint8_t  error;		//! system error indicator

	bool   hasCartesianPosVel;
	double cartesian_x,cartesian_y,cartesian_z;  //!< Only if hasCartesianPosVel is true
	double cartesian_vx,cartesian_vy,cartesian_vz;  //!< Only if hasCartesianPosVel is true

	bool hasPosCov;
	mrpt::math::CMatrixFloat44 pos_covariance;	//!< Only if hasPosCov is true

	bool hasVelCov;
	mrpt::math::CMatrixFloat44 vel_covariance;	//!< Only if hasPosCov is true

	bool hasStats;
	uint8_t stats_GPS_sats_used, stats_GLONASS_sats_used; //<! Only if hasStats is true
	uint8_t stats_rtk_fix_progress; //!< [0,100] %, only in modes other than RTK FIXED.

	Message_TOPCON_PZS();
	/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords structure (requires linking against mrpt-topography)
		*   Call as: getAsStruct<TGeodeticCoords>(); */
	template <class TGEODETICCOORDS>
	inline TGEODETICCOORDS getAsStruct() const {
		return TGEODETICCOORDS(latitude_degrees,longitude_degrees,height_meters);
	}
	void dumpToStream( mrpt::utils::CStream &out ) const MRPT_OVERRIDE; // See docs in base
protected:
	void internal_writeToStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE;
	void internal_readFromStream(mrpt::utils::CStream &in) MRPT_OVERRIDE;
};

/** TopCon mmGPS devices: SATS, a generic structure for statistics about tracked satelites and their positions. \sa mrpt::obs::CObservationGPS   */
struct OBS_IMPEXP Message_TOPCON_SATS : public gnss_message
{
	enum { msg_type = TOPCON_SATS };  //!< Static msg type (member expected by templates)

	Message_TOPCON_SATS();

	mrpt::vector_byte  USIs;  //!< The list of USI (Universal Sat ID) for the detected sats (See GRIL Manual, pag 4-31).
	mrpt::vector_signed_byte ELs; //!< Elevation (in degrees, 0-90) for each satellite in USIs.
	mrpt::vector_signed_word AZs; //!< Azimuth (in degrees, 0-360) for each satellite in USIs.

	void dumpToStream( mrpt::utils::CStream &out ) const MRPT_OVERRIDE; // See docs in base
protected:
	void internal_writeToStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE;
	void internal_readFromStream(mrpt::utils::CStream &in) MRPT_OVERRIDE;
};

} } } // End of namespaces

