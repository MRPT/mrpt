/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled header

#include <mrpt/kinematics/CVehicleSimulVirtualBase.h>

using namespace mrpt::kinematics;

CVehicleSimulVirtualBase::CVehicleSimulVirtualBase() :
m_firmware_control_period(500e-6)
{
}

CVehicleSimulVirtualBase::~CVehicleSimulVirtualBase()
{
}

void CVehicleSimulVirtualBase::setCurrentGTPose(const mrpt::math::TPose2D  &pose)
{
	m_pose=pose;
}

void CVehicleSimulVirtualBase::simulateOneTimeStep(const double dt)
{
	const double final_t = m_time + dt;
	while (m_time <= final_t)
	{
		this->internal_simulStep(m_firmware_control_period);
		m_time += m_firmware_control_period; // Move forward
	}
}

void CVehicleSimulVirtualBase::resetStatus()
{
	m_pose= mrpt::math::TPose2D(.0,.0,.0);
	m_vel = mrpt::math::TTwist2D(.0,.0,.0);
	m_odometry = mrpt::math::TPose2D(.0,.0,.0);
	internal_clear();
}

void CVehicleSimulVirtualBase::resetTime()
{
	m_time = .0;
}

mrpt::math::TTwist2D CVehicleSimulVirtualBase::getCurrentGTVelLocal() const
{
	mrpt::math::TTwist2D tl = this->m_vel;
	tl.rotate(-m_pose.phi);
	return tl;
}
