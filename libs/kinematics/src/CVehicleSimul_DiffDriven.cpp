/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled header

#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>

using namespace mrpt::kinematics;


CVehicleSimul_DiffDriven::CVehicleSimul_DiffDriven() :
	cTAU(.0),
	cDELAY(.0)
{
	resetStatus();
	resetTime();
}
CVehicleSimul_DiffDriven::~CVehicleSimul_DiffDriven()
{
}

void CVehicleSimul_DiffDriven::internal_clear()
{
	Command_Time = .0;
	Command_v = .0;
	Command_w = .0;
	m_v=m_w=0;
}

void CVehicleSimul_DiffDriven::internal_simulControlStep(const double AAt)
{
	// Change velocities:
	// ----------------------------------------------------------------
	double elapsed_time = this->m_time - Command_Time;
	elapsed_time-=cDELAY;
	elapsed_time = std::max(0.0,elapsed_time);

	if (cTAU==0 && cDELAY==0)
	{
		m_v = Command_v;
		m_w = Command_w;
	}
	else
	{
		m_v = Command_v0 + (Command_v-Command_v0)*(1-exp(-elapsed_time/cTAU));
		m_w = Command_w0 + (Command_w-Command_w0)*(1-exp(-elapsed_time/cTAU));
	}

	// Local to global frame:
	m_odometric_vel.vx   = cos(m_odometry.phi) * m_v;
	m_odometric_vel.vy   = sin(m_odometry.phi) * m_v;
	m_odometric_vel.omega = m_w;

}

/*************************************************************************
        Gives a movement command to the robot:
 This actually saves the command for execution later on, if we have
 to take into account the robot low-pass behavior in velocities.
*************************************************************************/
void CVehicleSimul_DiffDriven::movementCommand ( double lin_vel, double ang_vel )
{
	// Just save the command:
	Command_Time = m_time;
	Command_v = lin_vel;
	Command_w = ang_vel;

	// Current values:
	Command_v0 = m_v;
	Command_w0 = m_w;
}
