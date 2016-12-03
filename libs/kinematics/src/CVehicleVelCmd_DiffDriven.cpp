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

void CVehicleVelCmd_DiffDriven::TVelCmdParams_DiffDriven::loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(robotMax_V_mps, double, cfg, section);
	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(robotMax_W_degps, double, robotMax_W_radps, cfg, section);
	robotMax_W_radps = mrpt::utils::DEG2RAD(robotMax_W_radps);
}

CVehicleVelCmd_DiffDriven::TVelCmdParams_DiffDriven::TVelCmdParams_DiffDriven() :
	robotMax_V_mps(-1.0),
	robotMax_W_radps(-1.0)
{}

void CVehicleVelCmd_DiffDriven::cmdVel_scale(mrpt::kinematics::CVehicleVelCmd &vel_cmd, double vel_scale)
{
	mrpt::kinematics::CVehicleVelCmd_DiffDriven *cmd = dynamic_cast<mrpt::kinematics::CVehicleVelCmd_DiffDriven*>(&vel_cmd);
	ASSERTMSG_(cmd, "Expected velcmd of type `CVehicleVelCmd_DiffDriven`");
	cmd->lin_vel *= vel_scale;
	cmd->ang_vel *= vel_scale;
}

void CVehicleVelCmd_DiffDriven::cmdVel_limits(mrpt::kinematics::CVehicleVelCmd &vel_cmd, const mrpt::kinematics::CVehicleVelCmd &prev_vel_cmd, const double beta, const TVelCmdParams &params)
{
	const TVelCmdParams_DiffDriven *pParams = dynamic_cast<const TVelCmdParams_DiffDriven*>(&params);
	ASSERTMSG_(pParams!=nullptr, "Expected params of type `TVelCmdParams_DiffDriven`");

	ASSERT_(pParams->robotMax_V_mps>0);
	ASSERT_(pParams->robotMax_W_radps>0);
	mrpt::kinematics::CVehicleVelCmd_DiffDriven *newcmd = dynamic_cast<mrpt::kinematics::CVehicleVelCmd_DiffDriven*>(&vel_cmd);
	const mrpt::kinematics::CVehicleVelCmd_DiffDriven *prevcmd = dynamic_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven*>(&prev_vel_cmd);
	ASSERTMSG_(newcmd && prevcmd, "Expected velcmd of type `CVehicleVelCmd_DiffDriven`");
	filter_max_vw(newcmd->lin_vel, newcmd->ang_vel, *pParams);
	if (fabs(newcmd->lin_vel) < 0.01) // i.e. new behavior is nearly a pure rotation
	{                        // thus, it's OK to blend the rotational component
		newcmd->ang_vel = beta*newcmd->ang_vel + (1 - beta)*prevcmd->ang_vel;
	}
	else                     // there is a non-zero translational component
	{
		// must maintain the ratio of w to v (while filtering v)
		float ratio = newcmd->ang_vel / newcmd->lin_vel;
		newcmd->lin_vel = beta*newcmd->lin_vel + (1 - beta)*prevcmd->lin_vel;   // blend new v value
		newcmd->ang_vel = ratio * newcmd->lin_vel;  // ensure new w implements expected path curvature

		filter_max_vw(newcmd->lin_vel, newcmd->ang_vel, *pParams);
	}
}

void CVehicleVelCmd_DiffDriven::filter_max_vw(double &v, double &w, const TVelCmdParams_DiffDriven &p)
{
	// Ensure maximum speeds:
	if (fabs(v) > p.robotMax_V_mps) {
		// Scale:
		const double F = std::abs(p.robotMax_V_mps / v);
		v *= F;
		w *= F;
	}

	if (fabs(w) > p.robotMax_W_radps) {
		// Scale:
		const double F = std::abs(p.robotMax_W_radps / w);
		v *= F;
		w *= F;
	}
}

