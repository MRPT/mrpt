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

#include <mrpt/reactivenav.h>  // Precomp header

#include <mrpt/system/threads.h>

using namespace mrpt::reactivenav;
using namespace mrpt::synch;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;


/* --------------------------------------------------------
					thread_path_tracking

	Kalman-Filter tracking of the current robot state
   -------------------------------------------------------- */
void CPRRTNavigator::thread_path_tracking()
{
	// Set highest priority
	mrpt::system::changeCurrentProcessPriority(ppVeryHigh);
	mrpt::system::changeThreadPriority( mrpt::system::getCurrentThreadHandle(),tpHighest );

	cout << "[CPRRTNavigator:thread_path_tracking] Thread alive.\n";

	const double DESIRED_RATE = 15.0;
	const double DESIRED_PERIOD = 1.0/DESIRED_RATE;

	TTimeStamp  tim_last_iter = INVALID_TIMESTAMP;

	// Buffered data:
	TPathData	next_planned_point;  // target
	TTimeStamp	planned_path_time = INVALID_TIMESTAMP;

	while (!m_closingThreads)
	{
		if ( !m_initialized )  // Do nothing until we're loaded and ready.
		{
			mrpt::system::sleep(100);
			continue;
		}

		// Acquire the latest path plan --------------
		{
			CCriticalSectionLocker lock(& m_planned_path_cs );
			if (m_planned_path_time==INVALID_TIMESTAMP || m_planned_path.empty())
			{
				// There's no plan: Stop the robot:
				planned_path_time = INVALID_TIMESTAMP;
			}
			else  // Only update our copy if needed:
			{
				planned_path_time = m_planned_path_time;
				next_planned_point = m_planned_path.front();
			}
		} // end of CS

		// Path following ------------------------------
		if (planned_path_time == INVALID_TIMESTAMP)
		{
			// Stop the robot, now.
			onMotionCommand(0,0);
		}
		else
		{
			const bool ignore_trg_heading = INVALID_HEADING==next_planned_point.p.phi;

			// Acquire the current estimate of the robot state:
			// ----------------------------------------------------
			TPose2D  robotPose;
			float    robot_v,robot_w;
			if (!m_robotStateFilter.getCurrentEstimate(robotPose,robot_v,robot_w)) // Thread-safe call
			{
				// Error: Stop the robot, now.
				onMotionCommand(0,0);
			}
			else
			{	// we have proper localization

				// ==============================================================================
				// We want to approach to "next_planned_point"
				//   - Apply a sequence of tests to discover the shortests & easiest movement
				//      to that target.
				// ==============================================================================
				// Compute target in coordinates relative to the robot just now:
				const CPose2D trg_rel = CPose2D(next_planned_point.p) - CPose2D(robotPose);

//				if (next_planned_point.p.y==-1)	{
//					int a=0;
//				}

				// Current distances to target:
				const double trg_dist_lin = trg_rel.norm();
				const double trg_dist_ang = std::abs(trg_rel.phi());

				// Remaining Estimated Time for Arrival
				double ETA_target;
				if (std::abs(robot_v)>1e-4)
						ETA_target = std::max(0.05, (trg_dist_lin-params.pathtrack.radius_checkpoints)/std::abs(robot_v) );
				else if (std::abs(robot_w)>1e-4)
						ETA_target = trg_dist_ang/std::abs(robot_w);
				else	ETA_target = 1000.0;

				//cout << format("ETA: %f\n",ETA_target);
				//cout << format("d_lin=%f d_ang=%f\n",trg_dist_lin,RAD2DEG(trg_dist_ang));

				// If we have reached one point, remove it from the front of the list:
				if ((trg_dist_lin<params.pathtrack.radius_checkpoints || trg_dist_lin<std::abs(robot_v)*3*DESIRED_PERIOD )&&
					(ignore_trg_heading || trg_dist_ang<DEG2RAD(10) || trg_dist_ang<std::abs(robot_w)*3*DESIRED_PERIOD )
					)
				{
					CCriticalSectionLocker lock(& m_planned_path_cs );
					if (m_planned_path_time==planned_path_time)
						m_planned_path.pop_front();
				}
				else
				{
					// ------------------------------------------------------------------------------------
					//   CASE 1: Direct arc.
					//     (tx,ty,ta) all are compatible with a direct arc from our current pose
					// ------------------------------------------------------------------------------------
					bool   good_cmd_found = false;
					double good_cmd_v=0,good_cmd_w=0;

					{
						// predicted heading at (tx,ty):
						double  predict_phi;
						double cmd_v=0,cmd_w=0;

						if (trg_rel.y()==0)
						{	// In real-world conditions this might never happen, but just in case:
							predict_phi = 0;
							cmd_v = next_planned_point.max_v * sign(trg_rel.x());
							cmd_w = 0;
						}
						else
						{
							const double R = square(trg_dist_lin)/(2*trg_rel.y());
							// predicted heading at (tx,ty):
							predict_phi = atan2(trg_rel.x(),R-trg_rel.y());

							cmd_v = next_planned_point.max_v * sign(trg_rel.x());
							cmd_w = cmd_v/R;
						}

						// Good enough?
						if (ignore_trg_heading ||
							std::abs( wrapToPi(trg_rel.phi()-predict_phi))<DEG2RAD(7) )
						{
							good_cmd_found = true;
							good_cmd_v=cmd_v;
							good_cmd_w=cmd_w;
						}
					} // end case 1


					// ------------------------------------------
					// Scale velocities to the actual ranges:
					// ------------------------------------------
					if (good_cmd_found)
					{
						// SCALE: For maximum segment speeds:
						// ----------------------------------------
						if (std::abs(good_cmd_v)>next_planned_point.max_v)
						{
							const double K = next_planned_point.max_v / std::abs(good_cmd_v);
							good_cmd_v *= K;
							good_cmd_w *= K;
						}
						if (std::abs(good_cmd_w)>next_planned_point.max_w)
						{
							const double K = next_planned_point.max_w / std::abs(good_cmd_w);
							good_cmd_v *= K;
							good_cmd_w *= K;
						}

						// SCALE: Take into account the desired velocities when arriving at the target.
						// ------------------------------------------------------------------------------
						const double Av_to_trg = next_planned_point.trg_v - robot_v;
						const double min_At_to_change_to_trg_v = std::abs(Av_to_trg)/params.max_accel_v;

						if (good_cmd_v!=0 && ETA_target<=min_At_to_change_to_trg_v)
						{
							// We have to adapt velocity so we can get to the target right:
							const double new_v = next_planned_point.trg_v - (ETA_target/min_At_to_change_to_trg_v)*Av_to_trg;

							const double K = new_v / std::abs(good_cmd_v);
							good_cmd_v *= K;
							good_cmd_w *= K;
						}


						// SCALE: Is the command compatible with the maximum accelerations, wrt the current speed??
						// ----------------------------------------
						const double max_Av = params.max_accel_v * 0.2 /*sec*/;
						const double max_Aw = params.max_accel_w * 0.2 /*sec*/;
						if ( std::abs( good_cmd_v - robot_v )>max_Av )
						{
							if (good_cmd_v!=0)
							{
								const double rho = good_cmd_w/good_cmd_v;
								good_cmd_v = (good_cmd_v>robot_v) ? robot_v+max_Av :  robot_v-max_Av;
								good_cmd_w = rho*good_cmd_v;
							}
						}
						if ( std::abs( good_cmd_w - robot_w )>max_Aw )
						{
							if (good_cmd_w!=0)
							{
								const double R = good_cmd_v/good_cmd_w;
								good_cmd_w = (good_cmd_w>robot_w) ? robot_w+max_Aw :  robot_w-max_Aw;
								good_cmd_v = R*good_cmd_w;
							}
						}




						// Ok, send the command:
						// ----------------------------------------
						onMotionCommand(good_cmd_v,good_cmd_w);
					}
					else
					{
						// No valid motion found??!!
						// TODO: Raise some error.
						onMotionCommand(0,0);
					}

				} // end we haven't reached the target
			} // end we have proper localization
		} // end do path following


		// Run at XX Hz -------------------------------
		const TTimeStamp tim_now = now();
		int delay_ms;
		if (tim_last_iter != INVALID_TIMESTAMP)
		{
			const double tim_since_last = mrpt::system::timeDifference(tim_last_iter,tim_now);
			delay_ms = std::max(1, round( 1000.0*(DESIRED_PERIOD-tim_since_last) ) );
		}
		else delay_ms = 20;

		tim_last_iter = tim_now; // for the next iter.
		mrpt::system::sleep(delay_ms); // do the delay
	};
	cout << "[CPRRTNavigator:thread_path_tracking] Exit.\n";
}
