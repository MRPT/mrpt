/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled header
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::kinematics;

IMPLEMENTS_SERIALIZABLE(CVehicleVelCmd_Holo, CVehicleVelCmd, mrpt::kinematics)

CVehicleVelCmd_Holo::CVehicleVelCmd_Holo() = default;

CVehicleVelCmd_Holo::CVehicleVelCmd_Holo(
	double vel_, double dir_local_, double ramp_time_, double rot_speed_)
	: vel(vel_),
	  dir_local(dir_local_),
	  ramp_time(ramp_time_),
	  rot_speed(rot_speed_)
{
}

CVehicleVelCmd_Holo::~CVehicleVelCmd_Holo() = default;
size_t CVehicleVelCmd_Holo::getVelCmdLength() const { return 4; }
std::string CVehicleVelCmd_Holo::getVelCmdDescription(const int index) const
{
	switch (index)
	{
		case 0:
			return "vel";
			break;
		case 1:
			return "dir_local";
			break;
		case 2:
			return "ramp_time";
			break;
		case 3:
			return "rot_speed";
			break;
		default:
			THROW_EXCEPTION_FMT("index out of bounds: %i", index);
	};
}

double CVehicleVelCmd_Holo::getVelCmdElement(const int index) const
{
	switch (index)
	{
		case 0:
			return vel;
			break;
		case 1:
			return dir_local;
			break;
		case 2:
			return ramp_time;
			break;
		case 3:
			return rot_speed;
			break;
		default:
			THROW_EXCEPTION_FMT("index out of bounds: %i", index);
	};
}

void CVehicleVelCmd_Holo::setVelCmdElement(const int index, const double val)
{
	switch (index)
	{
		case 0:
			vel = val;
			break;
		case 1:
			dir_local = val;
			break;
		case 2:
			ramp_time = val;
			break;
		case 3:
			rot_speed = val;
			break;
		default:
			THROW_EXCEPTION_FMT("index out of bounds: %i", index);
	};
}

bool CVehicleVelCmd_Holo::isStopCmd() const
{
	return vel == 0 && rot_speed == 0;
}

void CVehicleVelCmd_Holo::setToStop()
{
	vel = dir_local = ramp_time = rot_speed = .0;
}
void CVehicleVelCmd_Holo::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
			in >> vel >> dir_local >> ramp_time >> rot_speed;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

uint8_t CVehicleVelCmd_Holo::serializeGetVersion() const { return 0; }
void CVehicleVelCmd_Holo::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << vel << dir_local << ramp_time << rot_speed;
}

void CVehicleVelCmd_Holo::cmdVel_scale(double vel_scale)
{
	vel *= vel_scale;  // |(vx,vy)|
	// rot_speed *= vel_scale; // rot_speed
	// Note: No need to scale "rot_speed" since a holonomic robot's path will be
	// invariant
	// ramp_time: leave unchanged
}

double CVehicleVelCmd_Holo::cmdVel_limits(
	const mrpt::kinematics::CVehicleVelCmd& prev_vel_cmd, const double beta,
	const TVelCmdParams& params)
{
	ASSERTMSG_(
		params.robotMax_V_mps >= .0,
		"[CVehicleVelCmd_Holo] `robotMax_V_mps` must be set to valid values: "
		"either assign values programmatically or call loadConfigFile()");

	double f = 1.0;
	if (vel > params.robotMax_V_mps) f = params.robotMax_V_mps / vel;

	vel *= f;  // |(vx,vy)|
	rot_speed *= f;  // rot_speed
	// ramp_time: leave unchanged
	// Blending with "beta" not required, since the ramp_time already blends
	// cmds for holo robots.

	return f;
}
