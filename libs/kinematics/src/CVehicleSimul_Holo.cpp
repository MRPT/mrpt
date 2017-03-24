/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled header

#include <mrpt/kinematics/CVehicleSimul_Holo.h>
#include <mrpt/math/wrap2pi.h>

using namespace mrpt::kinematics;

CVehicleSimul_Holo::CVehicleSimul_Holo()
{
	resetStatus();
	resetTime();
}

void CVehicleSimul_Holo::internal_simulControlStep(const double dt)
{
	// Control:
	if (m_vel_ramp_cmd.issue_time>=0 && m_time>m_vel_ramp_cmd.issue_time) // are we executing any cmd?
	{
		const double t = m_time - m_vel_ramp_cmd.issue_time;
		const double T = m_vel_ramp_cmd.ramp_time;
		const double vxi = m_vel_ramp_cmd.init_vel.vx;
		const double vyi = m_vel_ramp_cmd.init_vel.vy;
		const double wi  = m_vel_ramp_cmd.init_vel.omega;
		const double vxf = m_vel_ramp_cmd.target_vel_x;
		const double vyf = m_vel_ramp_cmd.target_vel_y;

		// "Blending" for vx,vy
		if (t<=m_vel_ramp_cmd.ramp_time)
		{
			m_odometric_vel.vx = vxi + t*(vxf-vxi) /T;
			m_odometric_vel.vy = vyi + t*(vyf-vyi) /T;
		}
		else
		{
			m_odometric_vel.vx = m_vel_ramp_cmd.target_vel_x;
			m_odometric_vel.vy = m_vel_ramp_cmd.target_vel_y;
		}

		// Ramp rotvel until aligned:
		const double Aang = mrpt::math::wrapToPi(m_vel_ramp_cmd.dir - m_odometry.phi);
		if (std::abs(Aang) < mrpt::utils::DEG2RAD(1.0)) {
			m_odometric_vel.omega = .0; // we are aligned.
		}
		else
		{
			const double wf = mrpt::utils::sign(Aang) * std::abs(m_vel_ramp_cmd.rot_speed);
			if (t <= m_vel_ramp_cmd.ramp_time)
			{
				m_odometric_vel.omega = wi + t*(wf - wi) / T;
			}
			else
			{
				m_odometric_vel.omega = wf;
			}
		}

	}
}

void CVehicleSimul_Holo::internal_clear()
{
	m_vel_ramp_cmd = TVelRampCmd();
}


void CVehicleSimul_Holo::sendVelRampCmd(double vel, double dir, double ramp_time, double rot_speed)
{
	ASSERT_ABOVE_(ramp_time,0);

	m_vel_ramp_cmd.issue_time = m_time;
	m_vel_ramp_cmd.ramp_time = ramp_time;
	m_vel_ramp_cmd.rot_speed = rot_speed;
	m_vel_ramp_cmd.init_vel = m_odometric_vel;
	m_vel_ramp_cmd.target_vel_x = cos(dir) * vel;
	m_vel_ramp_cmd.target_vel_y = sin(dir) * vel;
	m_vel_ramp_cmd.dir = dir;
}
