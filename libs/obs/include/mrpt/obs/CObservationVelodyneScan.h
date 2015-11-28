/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationVelodyneScan_H
#define CObservationVelodyneScan_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt {
namespace obs 
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationVelodyneScan, CObservation, OBS_IMPEXP)

	/** A "CObservation"-derived class representing the RAW DATA of a complete 360deg scan from a Velodyne scanner. 
	  * A scan comprises multiple "velodyne packets".
	  *
	  * Methods to convert this RAW data into a point cloud:
	  * - XXX
	  *
	  * \note New in MRPT 1.3.3
	  * \sa CObservation, CPointsMap
	  * \ingroup mrpt_obs_grp
	  */
	class OBS_IMPEXP CObservationVelodyneScan : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationVelodyneScan )

	 public:
		CObservationVelodyneScan();
		virtual ~CObservationVelodyneScan( );

		/** @name Scan data
		    @{ */
		double               maxRange; //!< The maximum range allowed by the device, in meters (e.g. 100m). Stored here by the driver while capturing based on the sensor model.
		mrpt::poses::CPose3D sensorPose; //!< The 6D pose of the sensor on the robot/vehicle frame of reference
		/** @} */
		
		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const { out_sensorPose = sensorPose; } // See base class docs
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) { sensorPose = newSensorPose; } // See base class docs
		virtual void getDescriptionAsText(std::ostream &o) const; // See base class docs

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationVelodyneScan, CObservation, OBS_IMPEXP)


	} // End of namespace

	namespace utils { // Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME_PTR_NAMESPACE(CObservationVelodyneScan, ::mrpt::obs)
	}

} // End of namespace

#endif
