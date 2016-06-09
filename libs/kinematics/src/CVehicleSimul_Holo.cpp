/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled header


#include <mrpt/kinematics/CVehicleSimul_Holo.h>
#include <mrpt/math/wrap2pi.h>

using namespace mrpt::kinematics;

void CVehicleSimul_Holo::internal_simulStep(const double dt)
{
	// Update state (forward Euler integration):
	m_pose.x   += m_vel.x   * dt;
	m_pose.y   += m_vel.y   * dt;
	m_pose.phi += m_vel.phi * dt;
	mrpt::math::wrapToPi(m_pose.phi);

	// Control:
	if (m_vel_ramp_cmd.issue_time>=0 && m_time>m_vel_ramp_cmd.issue_time) // are we executing any cmd?
	{
		const double t = m_time - m_vel_ramp_cmd.issue_time;
		const double T = m_vel_ramp_cmd.ramp_time;
		const double vxi = m_vel_ramp_cmd.init_vel.x;
		const double vyi = m_vel_ramp_cmd.init_vel.y;
		const double vxf = m_vel_ramp_cmd.target_vel_x;
		const double vyf = m_vel_ramp_cmd.target_vel_y;

		// "Blending" for vx,vy
		if (t<=m_vel_ramp_cmd.ramp_time)
		{
			m_vel.x = vxi + t*(vxf-vxi) /T;
			m_vel.y = vyi + t*(vyf-vyi) /T;
		}
		else 
		{
			m_vel.x = m_vel_ramp_cmd.target_vel_x;
			m_vel.y = m_vel_ramp_cmd.target_vel_y;
		}
#if 0
		// Proportional controller in angle:
		//const double KP = 1.0; 
		//m_vel.phi = KP* mrpt::math::wrapToPi(m_vel_ramp_cmd.dir - m_pose.phi) * m_vel_ramp_cmd.rot_speed;
#else
		// Constant rotational velocity:
		const double Aang = mrpt::math::wrapToPi(m_vel_ramp_cmd.dir - m_pose.phi);
		m_vel.phi = ( std::abs(Aang)<mrpt::utils::DEG2RAD(1.0) ? 0.0 : mrpt::utils::sign(Aang) )  * m_vel_ramp_cmd.rot_speed;
#endif
	}
}

void CVehicleSimul_Holo::internal_clear()
{
	m_vel_ramp_cmd = TVelRampCmd();
}


