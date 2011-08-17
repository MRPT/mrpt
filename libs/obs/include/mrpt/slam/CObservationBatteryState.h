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
#ifndef CObservationBatteryState_H
#define CObservationBatteryState_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationBatteryState, CObservation, OBS_IMPEXP)

	/** This represents a measurement of the batteries on the robot.
	 *  The battery levels are in volts in the form of the public members:
	 *	- voltageMainRobotBattery
	 *	- voltageMainRobotComputer
	 *  - voltageOtherBatteries
	 *
	 *  There are boolean flags for signaling when the corresponding values have been filled out or not.
	 *
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationBatteryState : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationBatteryState )

	 public:
		/** Constructor
		 */
		CObservationBatteryState( );

		 /** The data members
		  * \sa voltageMainRobotBatteryIsValid,voltageMainRobotComputerIsValid
		  */
		double voltageMainRobotBattery, voltageMainRobotComputer;

		/** These values must be true if the corresponding fields contain valid values.
		  * \sa voltageMainRobotBattery,voltageMainRobotComputer
		  */
		bool   voltageMainRobotBatteryIsValid,voltageMainRobotComputerIsValid;

		/** The users can use this vector for any arbitrary number of batteries or any other analog measurements.
		  * \sa voltageOtherBatteriesValid
		  */
		vector_double voltageOtherBatteries;

		/** These values must be true if the corresponding fields contain valid values (it MUST has the same size than voltageOtherBatteries)
		  */
		vector_bool   voltageOtherBatteriesValid;

		/** A general method to retrieve the sensor pose on the robot.
		  *  It has no effects in this class
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose=CPose3D(0,0,0); }


		/** A general method to change the sensor pose on the robot.
		  *  It has no effects in this class
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose ) {  }


	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
