/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled header


#include <mrpt/utils/CRobotSimulator.h>
#include <mrpt/random.h>
#include <mrpt/math/utils.h>

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

