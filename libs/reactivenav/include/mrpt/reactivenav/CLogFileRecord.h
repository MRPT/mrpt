/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef CLogFileRecord_H
#define CLogFileRecord_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CSimplePointsMap.h>

#include "CHolonomicLogFileRecord.h"


namespace mrpt
{
  namespace reactivenav
  {
	  using namespace mrpt::utils;

	  DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CLogFileRecord, mrpt::utils::CSerializable, REACTIVENAV_IMPEXP )

	/** A class for storing, saving and loading a reactive navigation
	 *   log record for the CReactiveNavigationSystem class.
	 * \sa CReactiveNavigationSystem, CHolonomicLogFileRecord
	 */
	class REACTIVENAV_IMPEXP  CLogFileRecord : public CSerializable
	{
		DEFINE_SERIALIZABLE( CLogFileRecord )


	 public:
		 /** Constructor, builds an empty record.
		   */
		 CLogFileRecord();

		 /** Copy .
		   */
		 void operator =( CLogFileRecord &);

		 /** Destructor, free all objects.
		   */
		 virtual ~CLogFileRecord();

		 /** The structure used to store all relevant information about each
		   *  transformation into TP-Space.
		   */
		 struct TInfoPerPTG
		 {
			 /** A short description for the applied PTG
			   */
			 std::string				PTG_desc;

			 /** Distances until obstacles, in "pseudometers", first index for -PI direction, last one for PI direction.
			   */
			 vector_float				TP_Obstacles;

			 /** Target location in TP-Space
			   */
			 mrpt::poses::CPoint2D				TP_Target;

			 /** Time, in seconds.
			   */
			 float						timeForTPObsTransformation,timeForHolonomicMethod;

			 /** The results from the holonomic method.
			   */
			 float						desiredDirection,desiredSpeed, evaluation;

			 /** Evaluation factors
			   */
			 vector_float				evalFactors;

			 /** Other useful info about holonomic method execution.
			   */
			 CHolonomicLogFileRecordPtr	HLFR;
		 };

		 /** The number of PTGS:
		   */
		uint32_t nPTGs;

		/** The security distances:
		 */
		vector_float				securityDistances;

		 /** The info for each applied PTG: must contain "nPTGs·nSecDistances" elements
		   */
		 std::vector<TInfoPerPTG, Eigen::aligned_allocator<TInfoPerPTG> >	infoPerPTG;

		 /**  The selected PTG.
		   */
		 int32_t					nSelectedPTG;

		 /** The total computation time, excluding sensing.
		   */
		 float						executionTime;

		 /** The estimated execution period.
		   */
		 float						estimatedExecutionPeriod;

		 /** The WS-Obstacles
		   */
		 mrpt::slam::CSimplePointsMap		WS_Obstacles;

		 /** The raw odometry measurement.
		   */
		 mrpt::poses::CPose2D				robotOdometryPose;

		 /** The relative location of target point in WS.
		   */
		 mrpt::poses::CPoint2D				WS_target_relative;

		 /** The final motion command sent to robot, in "m/sec" and "rad/sec".
		   */
		 float						v,w;

		 /** The actual robot velocities, as read from sensors, in "m/sec" and "rad/sec".
		   */
		 float						actual_v,actual_w;

		 /** Some recent values from previous iterations:
		   */
		 vector_float				prevV,prevW,prevSelPTG;

		 /** The used robot shape in WS.
		   */
		 vector_float				robotShape_x,robotShape_y;

		 /** The navigator behavior.
		   */
		 int32_t					navigatorBehavior;

		 /** The segment of the door-crossing behaviors, when applicable, in relative coordinates.
		   */
		 mrpt::poses::CPoint2D				doorCrossing_P1,doorCrossing_P2;

	 private:
		 /** Free all objects in infoPerPTGs structures (used internally).
		   */
		void  freeInfoPerPTGs();


	};

  }
}


#endif

