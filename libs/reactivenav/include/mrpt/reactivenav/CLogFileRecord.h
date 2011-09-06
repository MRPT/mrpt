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
	 *  \ingroup mrpt_reactivenav_grp
	 */
	class REACTIVENAV_IMPEXP  CLogFileRecord : public CSerializable
	{
		DEFINE_SERIALIZABLE( CLogFileRecord )

	public:
        CLogFileRecord();  //!< Constructor, builds an empty record.
        virtual ~CLogFileRecord();   //!< Destructor, free all objects.

        void operator =( CLogFileRecord &);  //!< Copy .

        /** The structure used to store all relevant information about each
          *  transformation into TP-Space.
	  *  \ingroup mrpt_reactivenav_grp
          */
        struct TInfoPerPTG
        {
            std::string				PTG_desc;      //!< A short description for the applied PTG
            vector_float				TP_Obstacles;  //!< Distances until obstacles, in "pseudometers", first index for -PI direction, last one for PI direction.
            mrpt::poses::CPoint2D		TP_Target;     //!< Target location in TP-Space
            float						timeForTPObsTransformation,timeForHolonomicMethod;  //!< Time, in seconds.
            float						desiredDirection,desiredSpeed, evaluation;          //!< The results from the holonomic method.
            vector_float				evalFactors;   //!< Evaluation factors
            CHolonomicLogFileRecordPtr	HLFR;          //!< Other useful info about holonomic method execution.
        };

        mrpt::system::TTimeStamp   timestamp;  //!< The timestamp of when this log was processed by the reactive algorithm (It can be INVALID_TIMESTAMP for navigation logs in MRPT <0.9.5)
		uint32_t       nPTGs;  //!< The number of PTGS:
		vector_float   securityDistances;  //!< The security distances:

		 /** The info for each applied PTG: must contain "nPTGs·nSecDistances" elements
		   */
		 std::vector<TInfoPerPTG, Eigen::aligned_allocator<TInfoPerPTG> >	infoPerPTG;

		 int32_t					nSelectedPTG;   //!< The selected PTG.
		 float						executionTime;  //!< The total computation time, excluding sensing.
		 float						estimatedExecutionPeriod;  //!< The estimated execution period.
		 mrpt::slam::CSimplePointsMap  WS_Obstacles;  //!< The WS-Obstacles
		 mrpt::poses::CPose2D          robotOdometryPose; //!< The robot pose (from raw odometry or a localization system).
		 mrpt::poses::CPoint2D         WS_target_relative;  //!< The relative location of target point in WS.

		 float						v,w;  //!< The final motion command sent to robot, in "m/sec" and "rad/sec".
		 float						actual_v,actual_w; //!< The actual robot velocities, as read from sensors, in "m/sec" and "rad/sec".
		 vector_float				robotShape_x,robotShape_y;  //!< The used robot shape in WS.
		 int32_t					navigatorBehavior;  //!< The navigator behavior.

	 private:
		 /** Free all objects in infoPerPTGs structures (used internally).
		   */
		void  freeInfoPerPTGs();


	};

  }
}


#endif

