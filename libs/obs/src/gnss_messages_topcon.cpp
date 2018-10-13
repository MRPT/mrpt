/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/gnss_messages_topcon.h>
#include <mrpt/math/matrix_serialization.h>  // for << of matrices

using namespace std;
using namespace mrpt::obs::gnss;

Message_TOPCON_PZS::Message_TOPCON_PZS()
	: gnss_message(TOPCON_PZS),

	  pos_covariance(),

	  vel_covariance()

{
}

void Message_TOPCON_PZS::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("\n[TopCon PZS datum]\n");
	out << mrpt::format(
		"  Longitude: %.09f deg  Latitude: %.09f deg Height: %.03f m (%.03f m "
		"without NBeam) \n",
		longitude_degrees, latitude_degrees, height_meters, RTK_height_meters);

	out << mrpt::format(
		" PZL-ID: %i  Angle trans: %.05f deg\n ", (int)nId, angle_transmitter);

	out << mrpt::format(" Fix: %i  ", (int)Fix);
	out << mrpt::format(" Error: %i ", (int)error);
	out << mrpt::format(
		" Battery levels: TX=%i  RX=%i\n ", TXBattery, RXBattery);

	out << mrpt::format(
		" hasCartesianPosVel= %s", hasCartesianPosVel ? "YES -> " : "NO\n");
	if (hasCartesianPosVel)
	{
		out << mrpt::format(
			" x=%f  y=%f  z=%f\n", cartesian_x, cartesian_y, cartesian_z);
		out << mrpt::format(
			" vx=%f  vy=%f  vz=%f\n", cartesian_vx, cartesian_vy, cartesian_vz);
	}
	out << mrpt::format("hasPosCov = %s", hasPosCov ? "YES\n" : "NO\n");
	if (hasPosCov)
		out << mrpt::format("%s\n", pos_covariance.inMatlabFormat().c_str());

	out << mrpt::format("hasVelCov = %s", hasVelCov ? "YES\n" : "NO\n");
	if (hasVelCov)
		out << mrpt::format("%s\n", vel_covariance.inMatlabFormat().c_str());

	out << mrpt::format("hasStats = %s", hasStats ? "YES: " : "NO\n");
	if (hasStats)
		out << mrpt::format(
			"GPS sats used: %i  GLONASS sats used: %i  RTK Fix progress:%i%%\n",
			(int)stats_GPS_sats_used, (int)stats_GLONASS_sats_used,
			(int)stats_rtk_fix_progress);
}

void Message_TOPCON_PZS::internal_writeToStream(
	mrpt::serialization::CArchive& out) const
{
	out << latitude_degrees << longitude_degrees << height_meters
		<< RTK_height_meters << PSigma << angle_transmitter << nId << Fix
		<< TXBattery << RXBattery << error << hasCartesianPosVel << cartesian_x
		<< cartesian_y << cartesian_z << cartesian_vx << cartesian_vy
		<< cartesian_vz << hasPosCov << pos_covariance << hasVelCov
		<< vel_covariance << hasStats << stats_GPS_sats_used
		<< stats_GLONASS_sats_used << stats_rtk_fix_progress;
}

void Message_TOPCON_PZS::internal_readFromStream(
	mrpt::serialization::CArchive& in)
{
	in >> latitude_degrees >> longitude_degrees >> height_meters >>
		RTK_height_meters >> PSigma >> angle_transmitter >> nId >> Fix >>
		TXBattery >> RXBattery >> error >> hasCartesianPosVel >> cartesian_x >>
		cartesian_y >> cartesian_z >> cartesian_vx >> cartesian_vy >>
		cartesian_vz >> hasPosCov >> pos_covariance >> hasVelCov >>
		vel_covariance >> hasStats >> stats_GPS_sats_used >>
		stats_GLONASS_sats_used >> stats_rtk_fix_progress;
}

// -------------
Message_TOPCON_SATS::Message_TOPCON_SATS() : gnss_message(TOPCON_SATS) {}
void Message_TOPCON_SATS::dumpToStream(std::ostream& out) const
{
	out << mrpt::format("\n[TopCon SATS datum]\n");
	out << mrpt::format(
		"  USI   ELEV    AZIM      (%u entries) \n",
		static_cast<unsigned int>(USIs.size()));

	ASSERT_(USIs.size() == AZs.size() && USIs.size() == ELs.size());
	for (size_t i = 0; i < USIs.size(); i++)
		out << mrpt::format(
			" %03i   %02i    %03i\n", (int)USIs[i], (int)ELs[i], (int)AZs[i]);
}

void Message_TOPCON_SATS::internal_writeToStream(
	mrpt::serialization::CArchive& out) const
{
	out << USIs << ELs << AZs;
}

void Message_TOPCON_SATS::internal_readFromStream(
	mrpt::serialization::CArchive& in)
{
	in >> USIs >> ELs >> AZs;
}
