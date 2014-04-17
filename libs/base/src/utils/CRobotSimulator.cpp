/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled header

#include <mrpt/utils/CRobotSimulator.h>
#include <mrpt/random.h>

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::random;
using std::max;


/*************************************************************************
        Constructor
*************************************************************************/
CRobotSimulator::CRobotSimulator(float TAU , float DELAY ) :
	m_pose(0,0,0),
	m_odometry(0,0,0),
	v(0),w(0),
	t(0),
	usar_error_odometrico(false),
	Command_Time(0),
	Command_v(0),Command_w(0),
	Command_v0(0),Command_w0(0),
	cTAU(TAU),
	cDELAY(DELAY)
{
	resetStatus();
}

/*************************************************************************
        Destructor
*************************************************************************/
CRobotSimulator::~CRobotSimulator()
{
}

/*************************************************************************
    Simulate the robot behavior during a time interval
*************************************************************************/
void    CRobotSimulator::simulateInterval( double At )
{
	CPose2D  dP,dPodo;  // Delta poses

	double tt = 0.0;
	const double AAt = 0.001;  // Minimum step size

	while (tt<At)
	{
		t+= AAt;
		tt+=AAt;

        // Change velocities:
        // ----------------------------------------------------------------
        double elapsed_time = t - Command_Time;
		elapsed_time-=cDELAY;
		elapsed_time = max(0.0,elapsed_time);

		if (cTAU==0 && cDELAY==0)
		{
			v = Command_v;
			w = Command_w;
		}
		else
		{
			v = Command_v0 + (Command_v-Command_v0)*(1-exp(-elapsed_time/cTAU));
			w = Command_w0 + (Command_w-Command_w0)*(1-exp(-elapsed_time/cTAU));
		}

        // Simulate movement during At:
        // ----------------------------------------------------------------
		dP.x( v*AAt );
		dP.y( 0 );
		dP.phi( w*AAt );
		m_pose = m_pose + dP;

		// odometry:
		dPodo = dP;
		if (usar_error_odometrico)
		{
			dPodo.x( dP.x() + m_Ax_err_bias + m_Ax_err_std * randomGenerator.drawGaussian1D_normalized() );
			dPodo.y( dP.y() + m_Ay_err_bias + m_Ay_err_std * randomGenerator.drawGaussian1D_normalized() );
			dPodo.phi( dP.phi() + m_Aphi_err_bias + m_Aphi_err_std * randomGenerator.drawGaussian1D_normalized() );
		}
		m_odometry = m_odometry + dPodo;
      }

}

/*************************************************************************
        Gives a movement command to the robot: 
 This actually saves the command for execution later on, if we have 
 to take into account the robot low-pass behavior in velocities.
*************************************************************************/
void CRobotSimulator::movementCommand ( double lin_vel, double ang_vel )
{
	// Just save the command:
	Command_Time = t;
	Command_v = lin_vel;
	Command_w = ang_vel;

	// Current values:
	Command_v0 = v;
	Command_w0 = w;
}


void CRobotSimulator::resetStatus()
{
	m_pose = CPose2D(0,0,0);
	m_odometry = CPose2D(0,0,0);

	t=0;
	v=w=0;
	Command_Time = 0;
	Command_v = Command_w = Command_v0 = Command_w0 = 0;
}

