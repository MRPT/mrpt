/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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

void CVehicleVelCmd_DiffDriven::cmdVel_scale(double vel_scale)
{
	lin_vel *= vel_scale;
	ang_vel *= vel_scale;
}

void CVehicleVelCmd_DiffDriven::cmdVel_limits(const mrpt::kinematics::CVehicleVelCmd &prev_vel_cmd, const double beta, const TVelCmdParams &params)
{
	ASSERT_(params.robotMax_V_mps>0);
	ASSERT_(params.robotMax_W_radps>0);
	const mrpt::kinematics::CVehicleVelCmd_DiffDriven *prevcmd = dynamic_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven*>(&prev_vel_cmd);
	ASSERTMSG_(prevcmd, "Expected prevcmd of type `CVehicleVelCmd_DiffDriven`");
	filter_max_vw(lin_vel, ang_vel, params);
	if (std::abs(lin_vel) < 0.01) // i.e. new behavior is nearly a pure rotation
	{ // thus, it's OK to blend the rotational component
		ang_vel = beta*ang_vel + (1 - beta)*prevcmd->ang_vel;
	}
	else // there is a non-zero translational component
	{
		// must maintain the ratio of w to v (while filtering v)
		float ratio = ang_vel / lin_vel;
		lin_vel = beta*lin_vel + (1 - beta)*prevcmd->lin_vel;   // blend new v value
		ang_vel = ratio * lin_vel;  // ensure new w implements expected path curvature

		filter_max_vw(lin_vel, ang_vel, params);
	}
}

void CVehicleVelCmd_DiffDriven::filter_max_vw(double &v, double &w, const TVelCmdParams &p)
{
	// Ensure maximum speeds:
	if (std::abs(v) > p.robotMax_V_mps) {
		// Scale:
		const double F = std::abs(p.robotMax_V_mps / v);
		v *= F;
		w *= F;
	}

	if (std::abs(w) > p.robotMax_W_radps) {
		// Scale:
		const double F = std::abs(p.robotMax_W_radps / w);
		v *= F;
		w *= F;
	}
}

