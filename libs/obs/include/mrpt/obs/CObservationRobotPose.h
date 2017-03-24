/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationRobotPose_H
#define CObservationRobotPose_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

namespace mrpt
{
namespace obs
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationRobotPose, CObservation, OBS_IMPEXP  )

	/** An observation providing an alternative robot pose from an external source.
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationRobotPose : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationRobotPose )
	 public:
		CObservationRobotPose( );  //!< Default ctor

		mrpt::poses::CPose3DPDFGaussian pose; //!< The observed robot pose

		mrpt::poses::CPose3D sensorPose; //!< The pose of the sensor on the robot/vehicle

		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const MRPT_OVERRIDE;// See base class docs.
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) MRPT_OVERRIDE;// See base class docs.
		void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE;// See base class docs

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationRobotPose, CObservation, OBS_IMPEXP  )

	} // End of namespace
} // End of namespace

#endif
