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
#ifndef CObservationGPS_H
#define CObservationGPS_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace slam
{
	using namespace mrpt::utils;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationGPS , CObservation, OBS_IMPEXP)

	/** Declares a class derived from "CObservation" that represents a Global Positioning System (GPS) reading.
	 *
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationGPS : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationGPS )

	 public:
		/** Constructor.
		 */
		CObservationGPS(  );

		/** Dumps the contents of the observation in a human-readable form to a given output stream
		  */
		void  dumpToStream( CStream &out );

		/** Dumps the contents of the observation in a human-readable form to the console
		  */
		void  dumpToConsole( );


		 /** The sensor pose on the robot.
		  */
		CPose3D			sensorPose;

		/** A UTC time-stamp structure for GPS messages
		  */
		struct OBS_IMPEXP TUTCTime
		{
			TUTCTime();

			uint8_t	hour;
			uint8_t	minute;
			double  sec;

			bool operator == (const TUTCTime& o) const { return hour==o.hour && minute==o.minute && sec==o.sec; }
			bool operator != (const TUTCTime& o) const { return hour!=o.hour || minute!=o.minute || sec!=o.sec; }
			inline TUTCTime& operator = (const TUTCTime& o)
			{
			    this->hour = o.hour;
			    this->minute = o.minute;
			    this->sec = o.sec;
			    return *this;
            }
		};

		/** The GPS datum for GGA commands
		  */
		struct OBS_IMPEXP TGPSDatum_GGA
		{
			TGPSDatum_GGA();

			/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords structure (requires linking against mrpt-topography)
			  *   Call as: getAsStruct<TGeodeticCoords>();
			  */
			template <class TGEODETICCOORDS>
			inline TGEODETICCOORDS getOrthoAsStruct() const {
				return TGEODETICCOORDS(latitude_degrees,longitude_degrees,corrected_orthometric_altitude);
			}

			/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords structure (requires linking against mrpt-topography)
			  *   Call as: getAsStruct<TGeodeticCoords>();
			  */
			template <class TGEODETICCOORDS>
			inline TGEODETICCOORDS getAsStruct() const {
				return TGEODETICCOORDS(latitude_degrees,longitude_degrees,altitude_meters);
			}

			/** The GPS sensor measured timestamp (in UTC time)
			*/
			TUTCTime	UTCTime;

			/** The measured latitude, in degrees (North:+ , South:-)
			*/
			double			latitude_degrees;

			/** The measured longitude, in degrees (East:+ , West:-)
			*/
			double			longitude_degrees;

			/** The values defined in the NMEA standard are the following:
			  *
			  *	0 = invalid
              *	1 = GPS fix (SPS)
			  *	2 = DGPS fix
              * 3 = PPS fix
			  * 4 = Real Time Kinematic
			  * 5 = Float RTK
			  * 6 = estimated (dead reckoning) (2.3 feature)
			  * 7 = Manual input mode
			  * 8 = Simulation mode
			  */
			uint8_t		fix_quality;

			/** The measured altitude, in meters (A).
			*/
			double			altitude_meters;

			/** Difference between the measured altitude and the geoid, in meters (B).
			*/
			double          geoidal_distance;

			/** The measured orthometric altitude, in meters (A)+(B).
			*/
			double          orthometric_altitude;

			/** The corrected (mmGPS) orthometric altitude, in meters mmGPS(A+B).
			*/
			double          corrected_orthometric_altitude;

			/** The number of satelites used to compute this estimation.
			*/
			uint32_t		satellitesUsed;

			/** This states whether to take into account the value in the HDOP field.
			*/
			bool			thereis_HDOP;

			/** The HDOP (Horizontal Dilution of Precision) as returned by the sensor.
			*/
			float			HDOP;
		};

		/** The GPS datum for RMC commands
		  */
		struct OBS_IMPEXP TGPSDatum_RMC
		{
			TGPSDatum_RMC();

			/** The GPS sensor measured timestamp (in UTC time)
			*/
			TUTCTime	UTCTime;

			/** This will be: 'A'=OK or 'V'=void
			 */
			int8_t		validity_char;

			/** The measured latitude, in degrees (North:+ , South:-)
			  */
			double		latitude_degrees;

			/** The measured longitude, in degrees (East:+ , West:-)
			  */
			double		longitude_degrees;

			/** The measured speed (in knots)
			  */
			double		speed_knots;

			/** The measured speed direction (in degrees)
			  */
			double		direction_degrees;
		};

		/** The GPS datum for TopCon's mmGPS devices
		  */
		struct OBS_IMPEXP TGPSDatum_PZS
		{
			TGPSDatum_PZS();

			/**  Return the geodetic coords as a mrpt::topography::TGeodeticCoords structure (requires linking against mrpt-topography)
			  *   Call as: getAsStruct<TGeodeticCoords>();
			  */
			template <class TGEODETICCOORDS>
			inline TGEODETICCOORDS getAsStruct() const {
				return TGEODETICCOORDS(latitude_degrees,longitude_degrees,height_meters);
			}

			double		latitude_degrees;	//!< The measured latitude, in degrees (North:+ , South:-)
			double		longitude_degrees;	//!< The measured longitude, in degrees (East:+ , West:-)
			double		height_meters;		//!< ellipsoidal height from N-beam [m] perhaps weighted with regular gps
			double		RTK_height_meters;	//!< ellipsoidal height [m] without N-beam correction
			float		PSigma;				//!< position SEP [m]
			double		angle_transmitter;	//!< Vertical angle of N-beam
			uint8_t		nId;		//!< ID of the transmitter [1-4], 0 if none.
			uint8_t		Fix;		//!< 1: GPS, 2: mmGPS
			uint8_t		TXBattery;	//!< battery level on transmitter
			uint8_t		RXBattery;	//!< battery level on receiver
			uint8_t		error;		//! system error indicator

			bool hasCartesianPosVel;
			double		cartesian_x,cartesian_y,cartesian_z;  //!< Only if hasCartesianPosVel is true
			double		cartesian_vx,cartesian_vy,cartesian_vz;  //!< Only if hasCartesianPosVel is true

			bool hasPosCov;
			mrpt::math::CMatrixFloat44   pos_covariance;	//!< Only if hasPosCov is true

			bool hasVelCov;
			mrpt::math::CMatrixFloat44   vel_covariance;	//!< Only if hasPosCov is true

			bool hasStats;
			uint8_t  stats_GPS_sats_used, stats_GLONASS_sats_used; //<! Only if hasStats is true
			uint8_t  stats_rtk_fix_progress; //!< [0,100] %, only in modes other than RTK FIXED.

		};


		/** A generic structure for statistics about tracked satelites and their positions.
		  */
		struct OBS_IMPEXP TGPSDatum_SATS
		{
			TGPSDatum_SATS();
			vector_byte  USIs;  //!< The list of USI (Universal Sat ID) for the detected sats (See GRIL Manual, pag 4-31).
			vector_signed_byte ELs; //!< Elevation (in degrees, 0-90) for each satellite in USIs.
			vector_signed_word AZs; //!< Azimuth (in degrees, 0-360) for each satellite in USIs.
		};


		/** Will be true if the corresponding field contains data read from the sensor, or false if it is not available.
		  * \sa GGA_datum
		  */
		bool				has_GGA_datum;

		/** Will be true if the corresponding field contains data read from the sensor, or false if it is not available.
		  * \sa RMC_datum
		  */
		bool				has_RMC_datum;

		/** Will be true if the corresponding field contains data read from the sensor, or false if it is not available.
		  * \sa PZS_datum
		  */
		bool				has_PZS_datum;

		/** Will be true if the corresponding field contains data read from the sensor, or false if it is not available.
		  * \sa SATS_datum
		  */
		bool				has_SATS_datum;

		TGPSDatum_GGA		GGA_datum;	//!< If "has_GGA_datum" is true, this contains the read GGA datum.
		TGPSDatum_RMC		RMC_datum;	//!< If "has_RMC_datum" is true, this contains the read RMC datum.
		TGPSDatum_PZS		PZS_datum;	//!< If "has_PZS_datum" is true, this contains the read PZS datum (TopCon's mmGPS devices only)
		TGPSDatum_SATS		SATS_datum;	//!< If "has_SATS_datum" is true, this contains the read PZS datum (TopCon's mmGPS devices only)

		/** A general method to retrieve the sensor pose on the robot.
		  *  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose = sensorPose; }


		/** A general method to change the sensor pose on the robot.
		  *  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose ) { sensorPose = newSensorPose; }


	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
