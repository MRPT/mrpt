/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/math/matrix_serialization.h> // for << of matrices
#include <mrpt/utils/CMemoryStream.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationGPS, CObservation,mrpt::obs)

MRPT_TODO("Refactor so a much larger number of datums can be stored. Once once per observation object.")
MRPT_TODO("Add two timestamps: GPS vs computer")
MRPT_TODO("Export to binary file from rawlog-edit")
MRPT_TODO("Import from ASCII/binary file with a new app: gps2rawlog")

MRPT_TODO("new parse unit tests") // Example cmds: https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf

CObservationGPS::CObservationGPS( ) :
	sensorPose(),
	originalReceivedTimestamp(INVALID_TIMESTAMP),
	messages(),
	has_GGA_datum (messages),
	has_RMC_datum (messages),
	has_PZS_datum (messages),
	has_SATS_datum(messages)
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationGPS::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 10;
	else
	{
		out << timestamp << originalReceivedTimestamp << sensorLabel << sensorPose;

		const uint32_t nMsgs = messages.size();
		out << nMsgs;
		for (message_list_t::const_iterator it=messages.begin();it!=messages.end();++it)
			it->second->writeToStream(out);
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationGPS::readFromStream(mrpt::utils::CStream &in, int version)
{
	this->clear();

	switch(version)
	{
	case 10:
		{
			in >> timestamp >> originalReceivedTimestamp >> sensorLabel >>sensorPose;
			uint32_t nMsgs;
			in >> nMsgs;
			for (unsigned i=0;i<nMsgs;i++) {
				gnss::gnss_message * msg = gnss::gnss_message::readAndBuildFromStream(in);
				messages[msg->message_type] = gnss::gnss_message_ptr(msg);
			}
		};
		break;

	// OLD VERSIONS: Ensure we can load datasets from many years ago ==========
	case 0:
		{
			bool has_GGA_datum_;
			in >> has_GGA_datum_;
			if (has_GGA_datum_) {
				gnss::Message_NMEA_GGA * datum = new gnss::Message_NMEA_GGA();
				in.ReadBuffer( &datum->fields, sizeof(datum->fields) );
				messages[gnss::NMEA_GGA] = gnss::gnss_message_ptr(datum);
			}

			bool has_RMC_datum_;
			in >> has_RMC_datum_;
			if (has_RMC_datum_) {
				gnss::Message_NMEA_RMC * datum = new gnss::Message_NMEA_RMC();
				in.ReadBuffer( &datum->fields, sizeof(datum->fields) );
				messages[gnss::NMEA_RMC] = gnss::gnss_message_ptr(datum);
			}
		} break;
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
		{
			if (version>=3)
					in >> timestamp;
			else 	timestamp = INVALID_TIMESTAMP;
			this->originalReceivedTimestamp = timestamp;

			bool has_GGA_datum_;
			in >> has_GGA_datum_;
			if (has_GGA_datum_)
			{
				gnss::Message_NMEA_GGA * datum = new gnss::Message_NMEA_GGA();
				messages[gnss::NMEA_GGA] = gnss::gnss_message_ptr(datum);
				gnss::Message_NMEA_GGA::content_t & GGA_datum = datum->fields;

				in  >> GGA_datum.UTCTime.hour >> GGA_datum.UTCTime.minute >> GGA_datum.UTCTime.sec >> GGA_datum.latitude_degrees
					>> GGA_datum.longitude_degrees >> GGA_datum.fix_quality >> GGA_datum.altitude_meters;
				if( version >= 9 ) {
					in  >> GGA_datum.geoidal_distance
						>> GGA_datum.orthometric_altitude
						>> GGA_datum.corrected_orthometric_altitude;
				}
				else {
					GGA_datum.geoidal_distance                  = 0.0f;
					GGA_datum.orthometric_altitude              = 0.0f;
					GGA_datum.corrected_orthometric_altitude    = 0.0f;
				}

				in  >> GGA_datum.satellitesUsed >> GGA_datum.thereis_HDOP >> GGA_datum.HDOP;
			}

			bool has_RMC_datum_;
			in >> has_RMC_datum_;
			if (has_RMC_datum_)
			{
				gnss::Message_NMEA_RMC * datum = new gnss::Message_NMEA_RMC();
				messages[gnss::NMEA_RMC] = gnss::gnss_message_ptr(datum);
				gnss::Message_NMEA_RMC::content_t & RMC_datum = datum->fields;

				in  >> RMC_datum.UTCTime.hour >> RMC_datum.UTCTime.minute >> RMC_datum.UTCTime.sec
					>> RMC_datum.validity_char >> RMC_datum.latitude_degrees >> RMC_datum.longitude_degrees
					>> RMC_datum.speed_knots >> RMC_datum.direction_degrees;
			}
			if (version>1)
					in >> sensorLabel;
			else 	sensorLabel = "";
			if (version>=4)
					in >> sensorPose;
			else	sensorPose.setFromValues(0,0,0,0,0,0);
			if (version>=5)
			{
				bool has_PZS_datum_;
				in >> has_PZS_datum_;
				if (has_PZS_datum_)
				{
					gnss::Message_TopCon_PZS * datum = new gnss::Message_TopCon_PZS();
					messages[gnss::TOPCON_PZS] = gnss::gnss_message_ptr(datum);
					gnss::Message_TopCon_PZS & PZS_datum = *datum;

					in >>
						PZS_datum.latitude_degrees >> PZS_datum.longitude_degrees >> PZS_datum.height_meters >>
						PZS_datum.RTK_height_meters >> PZS_datum.PSigma >> PZS_datum.angle_transmitter >> PZS_datum.nId >>
						PZS_datum.Fix >> PZS_datum.TXBattery >> PZS_datum.RXBattery >> PZS_datum.error;
					// extra data?
					if (version>=6) {
						in >>
							PZS_datum.hasCartesianPosVel >> PZS_datum.cartesian_x >> PZS_datum.cartesian_y >> PZS_datum.cartesian_z >>
							PZS_datum.cartesian_vx >> PZS_datum.cartesian_vy >> PZS_datum.cartesian_vz >>
							PZS_datum.hasPosCov >> PZS_datum.pos_covariance >> PZS_datum.hasVelCov >>
							PZS_datum.vel_covariance >> PZS_datum.hasStats >> PZS_datum.stats_GPS_sats_used >> PZS_datum.stats_GLONASS_sats_used;
						if (version>=8)
							in >> PZS_datum.stats_rtk_fix_progress;
						else
							PZS_datum.stats_rtk_fix_progress=0;
					}
					else {
						PZS_datum.hasCartesianPosVel = PZS_datum.hasPosCov = PZS_datum.hasVelCov = PZS_datum.hasStats = false;
					}
				}
			} // end version >=5

			// Added in V7:
			if (version>=7) {
				gnss::Message_TopCon_SATS * datum = new gnss::Message_TopCon_SATS();
				messages[gnss::TOPCON_SATS] = gnss::gnss_message_ptr(datum);
				gnss::Message_TopCon_SATS & SATS_datum = *datum;
				bool has_SATS_datum_;
				in >> has_SATS_datum_;
				if (has_SATS_datum_)
				{
					in >> SATS_datum.USIs >> SATS_datum.ELs >> SATS_datum.AZs;
				}
			}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}


/*---------------------------------------------------------------
					dumpToStream
 ---------------------------------------------------------------*/
void  CObservationGPS::dumpToStream( CStream &out ) const 
{
	out.printf("\n--------------------- [CObservationGPS] Dump: -----------------------\n");

	out.printf("\n[GGA datum: ");
	if (has_GGA_datum)
			out.printf("YES]\n");
	else	out.printf("NO]\n");
	if (has_GGA_datum)
	{
		out.printf("  Longitude: %.09f deg  Latitude: %.09f deg  Height: %.03f m\n",
			GGA_datum.longitude_degrees,
			GGA_datum.latitude_degrees,
			GGA_datum.altitude_meters );

        out.printf("  Geoidal distance: %.03f m  Orthometric alt.: %.03f m  Corrected ort. alt.: %.03f m\n",
            GGA_datum.geoidal_distance,
            GGA_datum.orthometric_altitude,
            GGA_datum.corrected_orthometric_altitude );

		out.printf("  UTC time-stamp: %02u:%02u:%02.03f  #sats=%2u  ",
			GGA_datum.UTCTime.hour,
			GGA_datum.UTCTime.minute,
			GGA_datum.UTCTime.sec,
			GGA_datum.satellitesUsed );

		out.printf("Fix mode: %u ",GGA_datum.fix_quality);
		switch( GGA_datum.fix_quality )
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

		out.printf(" HDOP (Horizontal Dilution of Precision): ");
		if (GGA_datum.thereis_HDOP)
				out.printf(" %f\n", GGA_datum.HDOP);
		else 	out.printf(" N/A\n");

	} // END GGA

	out.printf("\n[RMC datum: ");
	if (has_RMC_datum)
			out.printf("YES]\n");
	else	out.printf("NO]\n");
	if (has_RMC_datum)
	{
		out.printf("  Longitude: %.09f deg  Latitude: %.09f deg  Valid?: '%c'\n",
			RMC_datum.longitude_degrees,
			RMC_datum.latitude_degrees,
			RMC_datum.validity_char
			);
		out.printf("  UTC time-stamp: %02u:%02u:%02.03f  ",
			RMC_datum.UTCTime.hour,
			RMC_datum.UTCTime.minute,
			RMC_datum.UTCTime.sec
			);

		out.printf(" Speed: %.05f knots  Direction:%.03f deg.\n ",
			RMC_datum.speed_knots,
			RMC_datum.direction_degrees
			);

	} // END RMC


	// PZS datum:
	if (has_PZS_datum)
	{
		out.printf("\n[PZS datum: YES]\n");
		out.printf("  Longitude: %.09f deg  Latitude: %.09f deg Height: %.03f m (%.03f m without NBeam) \n",
			PZS_datum.longitude_degrees,
			PZS_datum.latitude_degrees,
			PZS_datum.height_meters,
			PZS_datum.RTK_height_meters);

		out.printf(" PZL-ID: %i  Angle trans: %.05f deg\n ",
			(int)PZS_datum.nId,
			PZS_datum.angle_transmitter
			);

		out.printf(" Fix: %i  ",(int)PZS_datum.Fix);
		out.printf(" Error: %i ",(int)PZS_datum.error);
		out.printf(" Battery levels: TX=%i  RX=%i\n ",PZS_datum.TXBattery,PZS_datum.RXBattery);

		out.printf(" hasCartesianPosVel= %s", PZS_datum.hasCartesianPosVel ? "YES -> ":"NO\n");
		if (PZS_datum.hasCartesianPosVel)
		{
			out.printf(" x=%f  y=%f  z=%f\n",PZS_datum.cartesian_x,PZS_datum.cartesian_y,PZS_datum.cartesian_z);
			out.printf(" vx=%f  vy=%f  vz=%f\n",PZS_datum.cartesian_vx,PZS_datum.cartesian_vy,PZS_datum.cartesian_vz);
		}
		out.printf("hasPosCov = %s", PZS_datum.hasPosCov ? "YES\n":"NO\n");
		if (PZS_datum.hasPosCov)
			out.printf("%s\n", PZS_datum.pos_covariance.inMatlabFormat().c_str() );

		out.printf("hasVelCov = %s", PZS_datum.hasVelCov ? "YES\n":"NO\n");
		if (PZS_datum.hasVelCov)
			out.printf("%s\n", PZS_datum.vel_covariance.inMatlabFormat().c_str() );

		out.printf("hasStats = %s", PZS_datum.hasStats? "YES: ":"NO\n");
		if(PZS_datum.hasStats)
			out.printf("GPS sats used: %i  GLONASS sats used: %i  RTK Fix progress:%i%%\n", (int)PZS_datum.stats_GPS_sats_used, (int)PZS_datum.stats_GLONASS_sats_used,(int)PZS_datum.stats_rtk_fix_progress);
	} // END PZS

	// Stats:
	out.printf("\n[SATS datum: ");
	if (has_SATS_datum)
			out.printf("YES]\n");
	else	out.printf("NO]\n");
	if (has_SATS_datum)
	{
		out.printf("  USI   ELEV    AZIM \n");
		out.printf("---------------------------\n");

		ASSERT_(SATS_datum.USIs.size()==SATS_datum.AZs.size() && SATS_datum.USIs.size()==SATS_datum.ELs.size());
		for (size_t i=0;i<SATS_datum.USIs.size();i++)
			out.printf(" %03i   %02i    %03i\n", (int)SATS_datum.USIs[i], (int)SATS_datum.ELs[i], (int)SATS_datum.AZs[i] );

	} // end SATS_datum

	out.printf("---------------------------------------------------------------------\n\n");
}

void  CObservationGPS::dumpToConsole(std::ostream &o) const
{
	mrpt::utils::CMemoryStream memStr;
	this->dumpToStream( memStr );
	
	if (memStr.getTotalBytesCount()) {
		o.write((const char*)memStr.getRawBufferData(),memStr.getTotalBytesCount());
	}
}

void CObservationGPS::clear()
{
	has_GGA_datum = false;
	has_RMC_datum = false;
	has_PZS_datum = false;
	has_SATS_datum = false;
}


// Build an MRPT timestamp with the hour/minute/sec of this structure and the date from the given timestamp.
mrpt::system::TTimeStamp CObservationGPS::TUTCTime::getAsTimestamp(const mrpt::system::TTimeStamp &date) const
{
    using namespace mrpt::system;

    TTimeParts parts;
    timestampToParts(date,parts, false /* UTC, not local */);

    parts.hour   = this->hour;
    parts.minute = this->minute;
    parts.second = this->sec;

    return buildTimestampFromParts(parts);
}

void CObservationGPS::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

	if (has_GGA_datum)
		o << std::endl << "Satellite time: " << format("%02u:%02u:%02.3f",GGA_datum.UTCTime.hour,GGA_datum.UTCTime.minute,GGA_datum.UTCTime.sec) << std::endl;

	o << "Sensor position on the robot: " << sensorPose << std::endl;

	this->dumpToConsole(o);
}
