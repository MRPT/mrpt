/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled header

#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::kinematics;

IMPLEMENTS_SERIALIZABLE(
	CVehicleVelCmd_DiffDriven, CVehicleVelCmd, mrpt::kinematics)

CVehicleVelCmd_DiffDriven::~CVehicleVelCmd_DiffDriven() = default;
size_t CVehicleVelCmd_DiffDriven::getVelCmdLength() const { return 2; }
std::string CVehicleVelCmd_DiffDriven::getVelCmdDescription(
	const int index) const
{
	switch (index)
	{
		case 0:
			return "lin_vel";
			break;
		case 1:
			return "ang_vel";
			break;
		default:
			THROW_EXCEPTION_FMT("index out of bounds: %i", index);
	};
}

double CVehicleVelCmd_DiffDriven::getVelCmdElement(const int index) const
{
	switch (index)
	{
		case 0:
			return lin_vel;
			break;
		case 1:
			return ang_vel;
			break;
		default:
			THROW_EXCEPTION_FMT("index out of bounds: %i", index);
	};
}

void CVehicleVelCmd_DiffDriven::setVelCmdElement(
	const int index, const double val)
{
	switch (index)
	{
		case 0:
			lin_vel = val;
			break;
		case 1:
			ang_vel = val;
			break;
		default:
			THROW_EXCEPTION_FMT("index out of bounds: %i", index);
	};
}

bool CVehicleVelCmd_DiffDriven::isStopCmd() const
{
	return lin_vel == .0 && ang_vel == .0;
}

void CVehicleVelCmd_DiffDriven::setToStop() { lin_vel = ang_vel = .0; }
void CVehicleVelCmd_DiffDriven::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
			in >> lin_vel >> ang_vel;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

uint8_t CVehicleVelCmd_DiffDriven::serializeGetVersion() const { return 0; }
void CVehicleVelCmd_DiffDriven::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << lin_vel << ang_vel;
}

void CVehicleVelCmd_DiffDriven::cmdVel_scale(double vel_scale)
{
	lin_vel *= vel_scale;
	ang_vel *= vel_scale;
}

double CVehicleVelCmd_DiffDriven::cmdVel_limits(
	const mrpt::kinematics::CVehicleVelCmd& prev_vel_cmd, const double beta,
	const TVelCmdParams& params)
{
	ASSERT_(params.robotMax_V_mps > 0);
	ASSERT_(params.robotMax_W_radps > 0);
	const auto* prevcmd =
		dynamic_cast<const mrpt::kinematics::CVehicleVelCmd_DiffDriven*>(
			&prev_vel_cmd);
	ASSERTMSG_(prevcmd, "Expected prevcmd of type `CVehicleVelCmd_DiffDriven`");

	double speed_scale = filter_max_vw(lin_vel, ang_vel, params);

	// i.e. new behavior is nearly a pure rotation
	if (std::abs(lin_vel) < 0.01)
	{  // thus, it's OK to blend the rotational component
		ang_vel = beta * ang_vel + (1 - beta) * prevcmd->ang_vel;
	}
	else  // there is a non-zero translational component
	{
		// must maintain the ratio of w to v (while filtering v)
		double ratio = ang_vel / lin_vel;
		// blend new v value
		lin_vel = beta * lin_vel + (1 - beta) * prevcmd->lin_vel;
		// ensure new w implements expected path curvature
		ang_vel = ratio * lin_vel;

		speed_scale *= filter_max_vw(lin_vel, ang_vel, params);
	}

	return speed_scale;
}

double CVehicleVelCmd_DiffDriven::filter_max_vw(
	double& v, double& w, const TVelCmdParams& p)
{
	double speed_scale = 1.0;
	// Ensure maximum speeds:
	if (std::abs(v) > p.robotMax_V_mps)
	{
		// Scale:
		const double F = std::abs(p.robotMax_V_mps / v);
		v *= F;
		w *= F;
		speed_scale *= F;
	}

	if (std::abs(w) > p.robotMax_W_radps)
	{
		// Scale:
		const double F = std::abs(p.robotMax_W_radps / w);
		v *= F;
		w *= F;
		speed_scale *= F;
	}
	return speed_scale;
}
