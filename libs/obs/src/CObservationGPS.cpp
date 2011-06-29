/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/obs.h>   // Precompiled headers

#include <mrpt/slam/CObservationGPS.h>
#include <mrpt/utils/CStdOutStream.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationGPS, CObservation,mrpt::slam)

CStdOutStream	gps_my_cout;

/** Constructor
 */
CObservationGPS::CObservationGPS( ) :
	sensorPose(),
	has_GGA_datum (false),
	has_RMC_datum (false),
	has_PZS_datum (false),
	has_SATS_datum(false),
	GGA_datum(),
	RMC_datum(),
	PZS_datum(),
	SATS_datum()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationGPS::writeToStream(CStream &out, int *version) const
{
	MRPT_UNUSED_PARAM(out);
	if (version)
		*version = 9;
	else
	{
		out << timestamp;

		out << has_GGA_datum;
		if (has_GGA_datum)
		{
			// OLD Version 0: out.WriteBuffer( &GGA_datum, sizeof(GGA_datum) );
			// New version:
			out << GGA_datum.UTCTime.hour
			    << GGA_datum.UTCTime.minute
				<< GGA_datum.UTCTime.sec
				<< GGA_datum.latitude_degrees
				<< GGA_datum.longitude_degrees
				<< GGA_datum.fix_quality
				<< GGA_datum.altitude_meters
                << GGA_datum.geoidal_distance                   // Added in V9
                << GGA_datum.orthometric_altitude               // Added in V9
                << GGA_datum.corrected_orthometric_altitude     // Added in V9
                << GGA_datum.satellitesUsed
				<< GGA_datum.thereis_HDOP
				<< GGA_datum.HDOP;
		}

		out << has_RMC_datum;
		if (has_RMC_datum)
		{
			// OLD Version 0: out.WriteBuffer( &RMC_datum, sizeof(RMC_datum) );
			// New version:
			out << RMC_datum.UTCTime.hour
				<< RMC_datum.UTCTime.minute
				<< RMC_datum.UTCTime.sec
				<< RMC_datum.validity_char
				<< RMC_datum.latitude_degrees
				<< RMC_datum.longitude_degrees
				<< RMC_datum.speed_knots
				<< RMC_datum.direction_degrees;
		}

		out << sensorLabel << sensorPose;

		// Added in V5.
		out << has_PZS_datum;
		if (has_PZS_datum)
		{
			out <<
				PZS_datum.latitude_degrees <<
				PZS_datum.longitude_degrees <<
				PZS_datum.height_meters <<
				PZS_datum.RTK_height_meters <<
				PZS_datum.PSigma <<
				PZS_datum.angle_transmitter <<
				PZS_datum.nId <<
				PZS_datum.Fix <<
				PZS_datum.TXBattery <<
				PZS_datum.RXBattery <<
				PZS_datum.error <<
			// Added in V6:
				PZS_datum.hasCartesianPosVel <<
				PZS_datum.cartesian_x << PZS_datum.cartesian_y << PZS_datum.cartesian_z <<
				PZS_datum.cartesian_vx << PZS_datum.cartesian_vy << PZS_datum.cartesian_vz <<
				PZS_datum.hasPosCov <<
				PZS_datum.pos_covariance <<
				PZS_datum.hasVelCov <<
				PZS_datum.vel_covariance <<
				PZS_datum.hasStats <<
				PZS_datum.stats_GPS_sats_used <<
				PZS_datum.stats_GLONASS_sats_used <<
			// Added V8:
				PZS_datum.stats_rtk_fix_progress;
		}

		// Added in V7:
		out << has_SATS_datum;
		if (has_SATS_datum)
		{
			out << SATS_datum.USIs << SATS_datum.ELs << SATS_datum.AZs;
		}
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationGPS::readFromStream(CStream &in, int version)
{
	MRPT_UNUSED_PARAM(in);
	switch(version)
	{
	case 0:
		{
			in >> has_GGA_datum;
			if (has_GGA_datum)
				in.ReadBuffer( &GGA_datum, sizeof(GGA_datum) );

			in >> has_RMC_datum;
			if (has_RMC_datum)
				in.ReadBuffer( &RMC_datum, sizeof(RMC_datum) );
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

			in >> has_GGA_datum;
			if (has_GGA_datum)
			{
				in  >> GGA_datum.UTCTime.hour
					>> GGA_datum.UTCTime.minute
					>> GGA_datum.UTCTime.sec
					>> GGA_datum.latitude_degrees
					>> GGA_datum.longitude_degrees
					>> GGA_datum.fix_quality
					>> GGA_datum.altitude_meters;
					if( version >= 9 )
					{
                        in  >> GGA_datum.geoidal_distance
                            >> GGA_datum.orthometric_altitude
                            >> GGA_datum.corrected_orthometric_altitude;
                    }
                    else
                    {
                        GGA_datum.geoidal_distance                  = 0.0f;
                        GGA_datum.orthometric_altitude              = 0.0f;
                        GGA_datum.corrected_orthometric_altitude    = 0.0f;
                    }

                in  >> GGA_datum.satellitesUsed
					>> GGA_datum.thereis_HDOP
					>> GGA_datum.HDOP;
			}

			in >> has_RMC_datum;
			if (has_RMC_datum)
			{
				in  >> RMC_datum.UTCTime.hour
					>> RMC_datum.UTCTime.minute
					>> RMC_datum.UTCTime.sec
					>> RMC_datum.validity_char
					>> RMC_datum.latitude_degrees
					>> RMC_datum.longitude_degrees
					>> RMC_datum.speed_knots
					>> RMC_datum.direction_degrees;
			}

			if (version>1)
					in >> sensorLabel;
			else 	sensorLabel = "";

			if (version>=4)
					in >> sensorPose;
			else	sensorPose.setFromValues(0,0,0,0,0,0);

			if (version>=5)
			{
				in >> has_PZS_datum;
				if (has_PZS_datum)
				{
					in >>
						PZS_datum.latitude_degrees >>
						PZS_datum.longitude_degrees >>
						PZS_datum.height_meters >>
						PZS_datum.RTK_height_meters >>
						PZS_datum.PSigma >>
						PZS_datum.angle_transmitter >>
						PZS_datum.nId >>
						PZS_datum.Fix >>
						PZS_datum.TXBattery >>
						PZS_datum.RXBattery >>
						PZS_datum.error;
					// extra data?
					if (version>=6)
					{
						in >>
							PZS_datum.hasCartesianPosVel >>
							PZS_datum.cartesian_x >> PZS_datum.cartesian_y >> PZS_datum.cartesian_z >>
							PZS_datum.cartesian_vx >> PZS_datum.cartesian_vy >> PZS_datum.cartesian_vz >>
							PZS_datum.hasPosCov >>
							PZS_datum.pos_covariance >>
							PZS_datum.hasVelCov >>
							PZS_datum.vel_covariance >>
							PZS_datum.hasStats >>
							PZS_datum.stats_GPS_sats_used >>
							PZS_datum.stats_GLONASS_sats_used;

						if (version>=8)
							in >> PZS_datum.stats_rtk_fix_progress;
						else
							PZS_datum.stats_rtk_fix_progress=0;
					}
					else
					{
						PZS_datum.hasCartesianPosVel =
						PZS_datum.hasPosCov =
						PZS_datum.hasVelCov =
						PZS_datum.hasStats = false;
					}
				}
			} // end version >=5
			else has_PZS_datum = false;

			// Added in V7:
			if (version>=7)
			{
				in >> has_SATS_datum;
				if (has_SATS_datum)
				{
					in >> SATS_datum.USIs >> SATS_datum.ELs >> SATS_datum.AZs;
				}
			}
			else has_SATS_datum = false;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}


/*---------------------------------------------------------------
					dumpToStream
 ---------------------------------------------------------------*/
void  CObservationGPS::dumpToStream( CStream &out )
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

/*---------------------------------------------------------------
					dumpToStream
 ---------------------------------------------------------------*/
void  CObservationGPS::dumpToConsole()
{
	dumpToStream( gps_my_cout );
}


// Ctor:
CObservationGPS::TUTCTime::TUTCTime() :
	hour(0), minute(0), sec(0)
{
}

// Ctor:
CObservationGPS::TGPSDatum_RMC::TGPSDatum_RMC() :
	UTCTime(),
	validity_char('V'),
	latitude_degrees(0),
	longitude_degrees(0),
	speed_knots(0),
	direction_degrees(0)
{ }

// Ctor:
CObservationGPS::TGPSDatum_GGA::TGPSDatum_GGA() :
	UTCTime(),
	latitude_degrees(0),
	longitude_degrees(0),
	fix_quality(0),
	altitude_meters(0),
	satellitesUsed(0),
	thereis_HDOP(false),
	HDOP(0)
{ }

// Ctor:
CObservationGPS::TGPSDatum_PZS::TGPSDatum_PZS() :
	latitude_degrees(0),
	longitude_degrees(0),
	height_meters(0),
	RTK_height_meters(0),
	PSigma(0),
	angle_transmitter(0),
	nId(0),
	Fix(0),
	TXBattery(0),
	RXBattery(0),
	error(0),
	hasCartesianPosVel(false),
	cartesian_x(0),cartesian_y(0),cartesian_z(0),
	cartesian_vx(0),cartesian_vy(0),cartesian_vz(0),
	hasPosCov(false),
	pos_covariance(),
	hasVelCov(false),
	vel_covariance(),
	hasStats(false),
	stats_GPS_sats_used(0),
	stats_GLONASS_sats_used(0)
{ }

// Ctor:
CObservationGPS::TGPSDatum_SATS::TGPSDatum_SATS()
{ }

