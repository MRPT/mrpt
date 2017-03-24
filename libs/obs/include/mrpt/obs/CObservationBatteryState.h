/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationBatteryState_H
#define CObservationBatteryState_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace obs
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
		mrpt::math::CVectorDouble voltageOtherBatteries;

		/** These values must be true if the corresponding fields contain valid values (it MUST has the same size than voltageOtherBatteries)
		  */
		vector_bool   voltageOtherBatteriesValid;

		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const MRPT_OVERRIDE;// See base class docs.
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) MRPT_OVERRIDE;// See base class docs.
		void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE;// See base class docs

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationBatteryState, CObservation, OBS_IMPEXP)


	} // End of namespace
} // End of namespace

#endif
