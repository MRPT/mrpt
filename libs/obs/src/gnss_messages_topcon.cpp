/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/gnss_messages_topcon.h>

using namespace std;
using namespace mrpt::obs::gnss;

Message_TopCon_PZS::Message_TopCon_PZS() :
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

void Message_TopCon_PZS::dumpToStream( mrpt::utils::CStream &out ) const
{
	MRPT_TODO("Implement");
}

void Message_TopCon_PZS::internal_writeToStream(mrpt::utils::CStream &out) const
{
	MRPT_TODO("Implement");
}

void Message_TopCon_PZS::internal_readFromStream(mrpt::utils::CStream &in)
{
	MRPT_TODO("Implement");
}

// -------------
Message_TopCon_SATS::Message_TopCon_SATS() : 
	gnss_message(TOPCON_SATS)
{
}

void Message_TopCon_SATS::dumpToStream( mrpt::utils::CStream &out ) const
{
	MRPT_TODO("Implement");
}

void Message_TopCon_SATS::internal_writeToStream(mrpt::utils::CStream &out) const
{
	MRPT_TODO("Implement");
}

void Message_TopCon_SATS::internal_readFromStream(mrpt::utils::CStream &in)
{
	MRPT_TODO("Implement");
}



