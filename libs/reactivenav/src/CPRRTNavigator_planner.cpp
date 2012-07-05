/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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

#include <mrpt/reactivenav.h>  // Precomp header

#include <mrpt/graphs/CAStarAlgorithm.h>

using namespace mrpt::reactivenav;
using namespace mrpt::synch;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace std;


struct CPath
{
 bool operator ==(const CPath&) const{return true;
 }
};

class CPathPlannerAstar : public mrpt::graphs::CAStarAlgorithm<CPath>
{
public:
	/**
	  * Client code must implement this method.
	  * Returns true if the given solution is complete.
	  */
	virtual bool isSolutionEnded(const CPath &sol)
	{
    return true;
	}

	/**
	  * Client code must implement this method.
	  * Returns true if the given solution is acceptable, that is, doesn't violate the problem logic.
	  */
	virtual bool isSolutionValid(const CPath &sol)
	{
    return false;
	}

	/**
	  * Client code must implement this method.
	  * Given a partial solution, returns all its children solution, regardless of their validity or completeness.
	  */
	virtual void generateChildren(const CPath &sol,std::vector<CPath> &sols)
	{

	}
	/**
	  * Client code must implement this method.
	  * Given a partial solution, estimates the cost of the remaining (unknown) part.
	  * This cost must always be greater or equal to zero, and not greater than the actual cost. Thus, must be 0 if the solution is complete.
	  */
	virtual double getHeuristic(const CPath &sol)
	{
    return 0;
	}
	/**
	  * Client code must implement this method.
	  * Given a (possibly partial) solution, calculates its cost so far.
	  * This cost must not decrease with each step. That is, a solution cannot have a smaller cost than the previous one from which it was generated.
	  */
    virtual double getCost(const CPath &sol)
    {
    return 0;
    }

};

/* --------------------------------------------------------
					thread_planner

	Anytime planner of the best robot trajectory to a target.
   -------------------------------------------------------- */
void CPRRTNavigator::thread_planner()
{
	cout << "[CPRRTNavigator:thread_planner] Thread alive.\n";

	const double DESIRED_RATE = 1.0;
	const double DESIRED_PERIOD = 1.0/DESIRED_RATE;

	TTimeStamp  tim_last_iter = INVALID_TIMESTAMP;

	// Buffered data:
	TPose2D				curTarget;
	CSimplePointsMap	curObstacles;
	TTimeStamp			curObstacles_time;

	while (!m_closingThreads)
	{
		if ( !m_initialized )  // Do nothing until we're loaded and ready.
		{
			mrpt::system::sleep(100); // make a sleep to no colapse the CPU
			continue;
		}

		// Get the latest commanded target:
		{
			CCriticalSectionLocker lock(&m_target_pose_cs);
			if (m_target_pose_time==INVALID_TIMESTAMP)
			{	// There is no target pose.
				mrpt::system::sleep(100);
				continue;
			}
			curTarget = m_target_pose;
		}

		// Get a copy of the last observed obstacles:
		{
			CCriticalSectionLocker lock(&m_last_obstacles_cs);
			curObstacles.setAllPoints(m_last_obstacles_x,m_last_obstacles_y);
			curObstacles_time = m_last_obstacles_time;
		}

		// if obstacles are too old, we cannot plan:
		if (curObstacles_time==INVALID_TIMESTAMP ||
			timeDifference(curObstacles_time,now())>params.max_age_observations
			)
		{
			// There are no recent obstacles...
			// Show some warning text??
			mrpt::system::sleep(100);
			continue;
		}


		// Obtain an estimate of the robot pose a bit ahead in the future,
		// such as when we're done planning the robot will be approx. there.
		TPose2D  robotPose;
		float    robot_v,robot_w;
		const TTimeStamp queryTime = now() + secondsToTimestamp(params.planner.max_time_expend_planning);
		if (!m_robotStateFilter.getCurrentEstimate(robotPose,robot_v,robot_w,queryTime)) // Thread-safe call
		{
			// Error
			// Show some warning text??
			mrpt::system::sleep(100);
			continue;
		}

		// ======================== Do the planning ==============================
		cout << format(
			"[CPRRTNavigator:thread_planner] Planning: (%.02f,%.02f,%.02fdeg) -> (%.02f,%.02f,%.02fdeg)\n",
			robotPose.x,robotPose.y,RAD2DEG(robotPose.phi),
			curTarget.x,curTarget.y,RAD2DEG(curTarget.phi)
			);

		CPathPlannerAstar planner;
		CPath pathini, pathoptimal;
		//int ret =
		planner.getOptimalSolution(pathini, pathoptimal);


		// Run at XX Hz -------------------------------
		const TTimeStamp tim_now = now();
		int delay_ms;
		if (tim_last_iter != INVALID_TIMESTAMP)
		{
			const double tim_since_last = mrpt::system::timeDifference(tim_last_iter,tim_now);
			delay_ms = std::max(1, round( 1000.0*(DESIRED_PERIOD-tim_since_last) ) );
		}
		else delay_ms = round(1000.0*DESIRED_PERIOD);

		tim_last_iter = tim_now; // for the next iter.
		mrpt::system::sleep(delay_ms); // do the delay
	}; // end of main while loop

	cout << "[CPRRTNavigator:thread_planner] Exit.\n";
}
