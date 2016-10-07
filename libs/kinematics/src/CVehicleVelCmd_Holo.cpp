/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled header
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::kinematics;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(CVehicleVelCmd_Holo, CVehicleVelCmd, mrpt::kinematics)

CVehicleVelCmd_Holo::CVehicleVelCmd_Holo() :
	vel(.0),
	dir_local(.0),
	ramp_time(.0),
	rot_speed(.0)
{
}
CVehicleVelCmd_Holo::~CVehicleVelCmd_Holo()
{
}

size_t CVehicleVelCmd_Holo::getVelCmdLength() const
{
	return 4;
}

std::string CVehicleVelCmd_Holo::getVelCmdDescription(const int index) const
{
	switch (index)
	{
	case 0: return "vel"; break;
	case 1: return "dir_local"; break;
	case 2: return "ramp_time"; break;
	case 3: return "rot_speed"; break;
	default:
		THROW_EXCEPTION_CUSTOM_MSG1("index out of bounds: %i",index);
	};
}

double CVehicleVelCmd_Holo::getVelCmdElement(const int index) const
{
	switch (index)
	{
	case 0: return vel; break;
	case 1: return dir_local; break;
	case 2: return ramp_time; break;
	case 3: return rot_speed; break;
	default:
		THROW_EXCEPTION_CUSTOM_MSG1("index out of bounds: %i", index);
	};
}

void CVehicleVelCmd_Holo::setVelCmdElement(const int index, const double val)
{
	switch (index)
	{
	case 0: vel=val; break;
	case 1: dir_local = val; break;
	case 2: ramp_time=val; break;
	case 3: rot_speed=val; break;
	default:
		THROW_EXCEPTION_CUSTOM_MSG1("index out of bounds: %i", index);
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
void CVehicleVelCmd_Holo::readFromStream(mrpt::utils::CStream &in, int version)
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

void CVehicleVelCmd_Holo::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
	{
		*version = 0;
		return;
	}
	out << vel << dir_local << ramp_time << rot_speed;
}
