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
#include <mrpt/math/wrap2pi.h>
#include <mrpt/random.h>

using namespace mrpt::kinematics;

CVehicleSimulVirtualBase::CVehicleSimulVirtualBase() :
m_firmware_control_period(500e-6), m_use_odo_error(false)
{
}

CVehicleSimulVirtualBase::~CVehicleSimulVirtualBase()
{
}

void CVehicleSimulVirtualBase::setCurrentGTPose(const mrpt::math::TPose2D  &pose)
{
	m_GT_pose=pose;
}

void CVehicleSimulVirtualBase::simulateOneTimeStep(const double dt)
{
	using mrpt::poses::CPose2D;
	const double final_t = m_time + dt;
	while (m_time <= final_t)
	{
		// Simulate movement during At:
		CPose2D nextOdometry(mrpt::poses::UNINITIALIZED_POSE), nextGT(mrpt::poses::UNINITIALIZED_POSE);
		nextOdometry.composeFrom(CPose2D(m_odometry),
			CPose2D (
				m_odometric_vel.vx*m_firmware_control_period,
				m_odometric_vel.vy*m_firmware_control_period,
				m_odometric_vel.omega*m_firmware_control_period
			));
		nextGT.composeFrom(CPose2D(m_GT_pose),
			CPose2D(
				m_GT_vel.vx*m_firmware_control_period,
				m_GT_vel.vy*m_firmware_control_period,
				m_GT_vel.omega*m_firmware_control_period
			));
		nextOdometry.phi(mrpt::math::wrapToPi(nextOdometry.phi()));
		nextGT.phi(mrpt::math::wrapToPi(nextGT.phi()));

		this->internal_simulControlStep(m_firmware_control_period);

		//Now rotate our current Odo velocity into GT coordinates
		m_GT_vel = getCurrentOdometricVelLocal();
		m_GT_vel.rotate(m_GT_pose.phi);

		//Add some errors
		if (m_use_odo_error)
		{
			m_GT_vel.vx    -= m_Ax_err_bias   + m_Ax_err_std * mrpt::random::randomGenerator.drawGaussian1D_normalized();
			m_GT_vel.vy    -= m_Ay_err_bias   + m_Ay_err_std * mrpt::random::randomGenerator.drawGaussian1D_normalized();
			m_GT_vel.omega -= m_Aphi_err_bias + m_Aphi_err_std * mrpt::random::randomGenerator.drawGaussian1D_normalized();
		}

		m_odometry = nextOdometry;
		m_GT_pose = nextGT;

		m_time += m_firmware_control_period; // Move forward

	}
}

void CVehicleSimulVirtualBase::resetStatus()
{
	m_GT_pose= mrpt::math::TPose2D(.0,.0,.0);
	m_GT_vel = mrpt::math::TTwist2D(.0,.0,.0);
	m_odometry = mrpt::math::TPose2D(.0,.0,.0);
	m_odometric_vel = mrpt::math::TTwist2D(.0,.0,.0);
	internal_clear();
}

void CVehicleSimulVirtualBase::resetTime()
{
	m_time = .0;
}

mrpt::math::TTwist2D CVehicleSimulVirtualBase::getCurrentGTVelLocal() const
{
	mrpt::math::TTwist2D tl = this->m_GT_vel;
	tl.rotate(-m_GT_pose.phi);
	return tl;
}

mrpt::math::TTwist2D CVehicleSimulVirtualBase::getCurrentOdometricVelLocal() const
{
	mrpt::math::TTwist2D tl = this->m_odometric_vel;
	tl.rotate(-m_odometry.phi);
	return tl;
}
