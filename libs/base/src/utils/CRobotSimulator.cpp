/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
                        Incremento de tiempo

 Simula el robot durante este tiempo: Velocidades, giros, movimientos,...
*************************************************************************/
void    CRobotSimulator::simulateInterval( double At )
{
	CPose2D  dP,dPodo;  // Delta poses
	double  AAt, tt;

	tt = 0.0;
	AAt = 0.001;     // Minimum step size

	while (tt<At)
	{
		t+= AAt;
		tt+=AAt;

        // Change velocities:
        // ----------------------------------------------------------------
        double t_transcurrido = t - Command_Time;
		t_transcurrido-=cDELAY;
		t_transcurrido = max(0.0,t_transcurrido);

		if (cTAU==0 && cDELAY==0)
		{
			v = Command_v;
			w = Command_w;
		}
		else
		{
			v = Command_v0 + (Command_v-Command_v0)*(1-exp(-t_transcurrido/cTAU));
			w = Command_w0 + (Command_w-Command_w0)*(1-exp(-t_transcurrido/cTAU));
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
			dPodo.y( dP.y() + m_Ay_err_bias + m_Ax_err_std * randomGenerator.drawGaussian1D_normalized() );
			dPodo.phi( dP.phi() + m_Aphi_err_bias + m_Aphi_err_std * randomGenerator.drawGaussian1D_normalized() );
		}
		m_odometry = m_odometry + dPodo;
      }

}

/*************************************************************************
        Dar un comando de movimiento al robot:

   Guarda los valores para ir ejecutando el movimiento poco a poco,
    teniendo en cuenta los tiempos de reaccion.
*************************************************************************/
void CRobotSimulator::movementCommand ( double lin_vel, double ang_vel )
{
        // Apuntar datos para ejecutarlos lentamente, segun
        //   la respuesta del robot:
        Command_Time = t;
        Command_v = lin_vel;
        Command_w = ang_vel;

        // Actuales:
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

