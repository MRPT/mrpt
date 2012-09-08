/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
#ifndef CRobotSimulator_H
#define CRobotSimulator_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/poses/CPose2D.h>

#include <mrpt/base/link_pragmas.h>

namespace mrpt
{
namespace utils
{
	using namespace mrpt::poses;
	using namespace mrpt::math;

	/** This class can be used to simulate the kinematics and dynamics of a differential driven planar mobile robot, including odometry errors and dynamics limitations.
	 *  The main methods are:
			- movementCommand: Call this for send a command to the robot. This comamnd will be
								delayed and passed throught a first order low-pass filter to simulate
								robot dynamics.
			- simulateInterval: Call this for run the simulator for the desired time period.
	 *
		Versions:
			- 23/MAR/2009: (JLBC) Changed to reuse MRPT poses and methods renamed to conform to MRPT style.
			- 29/AUG/2008: (JLBC) Added parameters for odometry noise.
			- 27/JAN/2008: (JLBC) Translated to English!!! :-)
			- 17/OCT/2005: (JLBC) Integration into the MRML library.
			- 1/DIC/2004: (JLBC) Odometry, cumulative errors added.
			- 18/JUN/2004: (JLBC) First creation.
	 *
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CRobotSimulator
	{
		private:
			//	Internal state variables:
			// ---------------------------------------
			mrpt::poses::CPose2D  m_pose;	//!< Global, absolute and error-free robot coordinates
			mrpt::poses::CPose2D m_odometry;	//!< Used to simulate odometry (with optional error)

			/** Instantaneous velocity of the robot (linear, m/s)
			  */
			double          v;

			/** Instantaneous velocity of the robot (angular, rad/s)
			  */
			double          w;

			/** Simulation time variable
			  */
			double          t;

			/** Whether to corrupt odometry with noise  */
			bool            usar_error_odometrico;

			/** Dynamic limitations of the robot.
			  * Approximation to non-infinity motor forces: A first order low-pass filter, using:
			  *   Command_Time: Time "t" when the last order was received.
			  *   Command_v, Command_w: The user-desired velocities.
			  *   Command_v0, Command_w0: Actual robot velocities at the moment of user request.
			  */
			double Command_Time,
			       Command_v, Command_w,
				   Command_v0, Command_w0;

			/** The time-constants for the first order low-pass filter for the velocities changes. */
			float			cTAU;                   // 1.8 sec

			/** The delay constant for the velocities changes.  */
			float			cDELAY;

			double m_Ax_err_bias, m_Ax_err_std;
			double m_Ay_err_bias, m_Ay_err_std;
			double m_Aphi_err_bias, m_Aphi_err_std;

		public:
			/** Constructor with default dynamic model-parameters
			  */
			CRobotSimulator( float TAU = 0, float DELAY = 0);

			/** Destructor
			  */
			virtual ~CRobotSimulator();

			/** Change the model of delays used for the orders sent to the robot \sa movementCommand */
			void setDelayModelParams(float TAU_delay_sec=1.8f, float CMD_delay_sec=0.3f) {
				cTAU = TAU_delay_sec;
				cDELAY = CMD_delay_sec;
			}

			/** Enable/Disable odometry errors
			  *  Errors in odometry are introduced per millisecond.
			  */
			void setOdometryErrors(
				bool enabled,
				double Ax_err_bias  =  1e-6,
				double Ax_err_std   = 10e-6,
				double Ay_err_bias =  1e-6,
				double Ay_err_std  = 10e-6,
				double Aphi_err_bias =  DEG2RAD(1e-6),
				double Aphi_err_std  = DEG2RAD(10e-6)
				)
			{
				usar_error_odometrico=enabled;
				m_Ax_err_bias=Ax_err_bias;
				m_Ax_err_std=Ax_err_std;
				m_Ay_err_bias=Ay_err_bias;
				m_Ay_err_std=Ay_err_std;
				m_Aphi_err_bias=Aphi_err_bias;
				m_Aphi_err_std=Aphi_err_std;
			}

			/** Reset actual robot pose (inmediately, without simulating the movement along time)
			  */
			void  setRealPose(mrpt::poses::CPose2D &p ) { this->m_pose = p; }

			/** Read the instantaneous, error-free status of the simulated robot
			  */
			double  getX() const { return m_pose.x(); }

			/** Read the instantaneous, error-free status of the simulated robot
			  */
			double  getY() { return m_pose.y(); }

			/** Read the instantaneous, error-free status of the simulated robot
			  */
			double  getPHI() { return m_pose.phi(); }

			/** Read the instantaneous, error-free status of the simulated robot
			  */
			double  getT()   { return t; }

			/** Read the instantaneous, error-free status of the simulated robot
			  */
			double  getV() { return v; }
			/** Read the instantaneous, error-free status of the simulated robot
			  */
			double  getW() { return w; }

			/** Set actual robot pose (inmediately, without simulating the movement along time) (Not to be called normally!!)
			  * \sa MovementCommand
			  */
			void    setV(double v) { this->v=v; }
			void    setW(double w) { this->w=w; }

			/** Used to command the robot a desired movement (velocities)
			  */
			void    movementCommand ( double lin_vel, double ang_vel );

			/** Reset all the simulator variables to 0 (All but current simulator time).
			  */
			void    resetStatus();

			/** Reset time counter
			  */
			void    resetTime()  { t = 0.0; }

			/** This method must be called periodically to simulate discrete time intervals.
			  */
			void    simulateInterval( double At);

			/** Forces odometry to be set to a specified values.
			  */
			void    resetOdometry( const mrpt::poses::CPose2D &newOdo = mrpt::poses::CPose2D() ) {
				m_odometry = newOdo;
			}

			/** Reads the simulated robot odometry (this is NOT equal to the actual error-free robot coordinates).
			  * \sa getRealPose
			  */
			void    getOdometry ( CPose2D &pose ) const {
				pose = m_odometry;
			}

			/** Reads the simulated robot odometry (this is NOT equal to the actual error-free robot coordinates).
			  * \sa getRealPose
			  */
			void    getOdometry ( TPose2D &pose ) const {
				pose = m_odometry;
			}

			/** Reads the real robot pose. \sa getOdometry  */
			void    getRealPose ( CPose2D &pose ) const {
				pose = m_pose;
			}

			/** Reads the real robot pose. \sa getOdometry  */
			void    getRealPose ( TPose2D &pose ) const {
				pose = m_pose;
			}
	};

	} // End of namespace
} // End of namespace

#endif
