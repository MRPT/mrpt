/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled header
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::kinematics;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(CVehicleVelCmd_DiffDriven, CVehicleVelCmd, mrpt::kinematics)

CVehicleVelCmd_DiffDriven::CVehicleVelCmd_DiffDriven() :
	lin_vel(.0),
	ang_vel(.0)
{
}
CVehicleVelCmd_DiffDriven::~CVehicleVelCmd_DiffDriven()
{
}
size_t CVehicleVelCmd_DiffDriven::getVelCmdLength() const
{
	return 2;
}

std::string CVehicleVelCmd_DiffDriven::getVelCmdDescription(const int index) const
{
	switch (index)
	{
	case 0: return "lin_vel"; break;
	case 1: return "ang_vel"; break;
	default:
		THROW_EXCEPTION_CUSTOM_MSG1("index out of bounds: %i", index);
	};
}

double CVehicleVelCmd_DiffDriven::getVelCmdElement(const int index) const
{
	switch (index)
	{
	case 0: return lin_vel; break;
	case 1: return ang_vel; break;
	default:
		THROW_EXCEPTION_CUSTOM_MSG1("index out of bounds: %i", index);
	};
}

void CVehicleVelCmd_DiffDriven::setVelCmdElement(const int index, const double val)
{
	switch (index)
	{
	case 0: lin_vel = val; break;
	case 1: ang_vel = val; break;
	default:
		THROW_EXCEPTION_CUSTOM_MSG1("index out of bounds: %i", index);
	};
}

bool CVehicleVelCmd_DiffDriven::isStopCmd() const
{
	return lin_vel == .0 && ang_vel == .0;
}

void CVehicleVelCmd_DiffDriven::setToStop()
{
	lin_vel = ang_vel = .0;
}

void CVehicleVelCmd_DiffDriven::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch (version)
	{
	case 0:
		in >> lin_vel >> ang_vel;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CVehicleVelCmd_DiffDriven::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
	{
		*version = 0;
		return;
	}
	out << lin_vel << ang_vel;
}
