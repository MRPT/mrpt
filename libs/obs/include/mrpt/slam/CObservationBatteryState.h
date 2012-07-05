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
