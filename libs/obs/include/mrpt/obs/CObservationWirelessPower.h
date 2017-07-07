/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CObservationWirelessPower_H
#define CObservationWirelessPower_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace obs
{

	/** This represents a measurement of the wireless strength perceived by the robot.
	 *  The signal level is given as a percentage.
	 *
	 * \sa CObservation, mrpt::hwdrivers::CWirelessPower for a software sensor capable of reading this kind of observations.
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationWirelessPower : public CObservation
	{
		DEFINE_SERIALIZABLE( CObservationWirelessPower )

	 public:
		/** Constructor */
		CObservationWirelessPower( );

		 /** @name The data members
		  * @{ */

		/** The power or signal strength as sensed by the Wifi receiver (In percentage: [0-100]) */
		double power;  
		/** The location of the sensing antenna on the robot coordinate framework */
		mrpt::poses::CPose3D  sensorPoseOnRobot; 

		/** @} */

		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const override;// See base class docs
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) override;// See base class docs
		void getDescriptionAsText(std::ostream &o) const override;// See base class docs

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationWirelessPower, CObservation, OBS_IMPEXP)


	} // End of namespace
} // End of namespace

#endif
