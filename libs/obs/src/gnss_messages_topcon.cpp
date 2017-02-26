/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/gnss_messages_topcon.h>
#include <mrpt/math/matrix_serialization.h> // for << of matrices

using namespace std;
using namespace mrpt::obs::gnss;

Message_TOPCON_PZS::Message_TOPCON_PZS() :
	gnss_message(TOPCON_PZS),
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
{}

void Message_TOPCON_PZS::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("\n[TopCon PZS datum]\n");
	out.printf("  Longitude: %.09f deg  Latitude: %.09f deg Height: %.03f m (%.03f m without NBeam) \n",
		longitude_degrees,
		latitude_degrees,
		height_meters,
		RTK_height_meters);

	out.printf(" PZL-ID: %i  Angle trans: %.05f deg\n ",
		(int)nId,
		angle_transmitter
		);

	out.printf(" Fix: %i  ",(int)Fix);
	out.printf(" Error: %i ",(int)error);
	out.printf(" Battery levels: TX=%i  RX=%i\n ",TXBattery,RXBattery);

	out.printf(" hasCartesianPosVel= %s", hasCartesianPosVel ? "YES -> ":"NO\n");
	if (hasCartesianPosVel)
	{
		out.printf(" x=%f  y=%f  z=%f\n",cartesian_x,cartesian_y,cartesian_z);
		out.printf(" vx=%f  vy=%f  vz=%f\n",cartesian_vx,cartesian_vy,cartesian_vz);
	}
	out.printf("hasPosCov = %s", hasPosCov ? "YES\n":"NO\n");
	if (hasPosCov)
		out.printf("%s\n", pos_covariance.inMatlabFormat().c_str() );

	out.printf("hasVelCov = %s", hasVelCov ? "YES\n":"NO\n");
	if (hasVelCov)
		out.printf("%s\n", vel_covariance.inMatlabFormat().c_str() );

	out.printf("hasStats = %s", hasStats? "YES: ":"NO\n");
	if(hasStats)
		out.printf("GPS sats used: %i  GLONASS sats used: %i  RTK Fix progress:%i%%\n", (int)stats_GPS_sats_used, (int)stats_GLONASS_sats_used,(int)stats_rtk_fix_progress);
}

void Message_TOPCON_PZS::internal_writeToStream(mrpt::utils::CStream &out) const
{
	out <<
	latitude_degrees << longitude_degrees << height_meters <<
	RTK_height_meters <<
	PSigma << angle_transmitter << nId <<
	Fix << 	TXBattery << RXBattery << error <<
	hasCartesianPosVel << cartesian_x << cartesian_y << cartesian_z <<
	cartesian_vx << cartesian_vy << cartesian_vz <<
	hasPosCov << pos_covariance << hasVelCov <<
	vel_covariance << hasStats << stats_GPS_sats_used <<
	stats_GLONASS_sats_used << stats_rtk_fix_progress;
}

void Message_TOPCON_PZS::internal_readFromStream(mrpt::utils::CStream &in)
{
	in  >>
	latitude_degrees >> longitude_degrees >> height_meters >>
	RTK_height_meters >>
	PSigma >> angle_transmitter >> nId >>
	Fix >> 	TXBattery >> RXBattery >> error >>
	hasCartesianPosVel >> cartesian_x >> cartesian_y >> cartesian_z >>
	cartesian_vx >> cartesian_vy >> cartesian_vz >>
	hasPosCov >> pos_covariance >> hasVelCov >>
	vel_covariance >> hasStats >> stats_GPS_sats_used >>
	stats_GLONASS_sats_used >> stats_rtk_fix_progress;
}

// -------------
Message_TOPCON_SATS::Message_TOPCON_SATS() : 
	gnss_message(TOPCON_SATS)
{
}

void Message_TOPCON_SATS::dumpToStream( mrpt::utils::CStream &out ) const
{
	out.printf("\n[TopCon SATS datum]\n");
	out.printf("  USI   ELEV    AZIM      (%u entries) \n",static_cast<unsigned int>(USIs.size()));

	ASSERT_(USIs.size()==AZs.size() && USIs.size()==ELs.size());
	for (size_t i=0;i<USIs.size();i++)
		out.printf(" %03i   %02i    %03i\n", (int)USIs[i], (int)ELs[i], (int)AZs[i] );
}

void Message_TOPCON_SATS::internal_writeToStream(mrpt::utils::CStream &out) const
{
	out << USIs << ELs << AZs;
}

void Message_TOPCON_SATS::internal_readFromStream(mrpt::utils::CStream &in)
{
	in >> USIs >> ELs >> AZs;
}

