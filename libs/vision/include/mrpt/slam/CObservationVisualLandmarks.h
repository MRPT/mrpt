/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationVisualLandmarks_H
#define CObservationVisualLandmarks_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/slam/CLandmarksMap.h>

namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationVisualLandmarks , CObservation, VISION_IMPEXP )

	/** Declares a class derived from "CObservation" that stores a Landmarks Map as seen from a stereo camera at a given instant of time.
	 *
	 * \sa CLandmarksMap, CObservation
	 * \ingroup mrpt_vision_grp
	 */
	class VISION_IMPEXP CObservationVisualLandmarks : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationVisualLandmarks )

	 public:
		/** Constructor
		 */
		CObservationVisualLandmarks( );

		/** The 3D pose of the reference camera relative to robot coordinates.
		  */
		CPose3D				refCameraPose;

		 /** The landmarks, with coordinates origin in the camera reference system.
		  */
		CLandmarksMap		landmarks;

		/** Implements the virtual method in charge of finding the likelihood between this
		  *   and another observation, probably only of the same derived class. The operator
		  *   may be asymmetric.
		  *
		  * \param anotherObs The other observation to compute likelihood with.
		  * \param anotherObsPose If known, the belief about the robot pose when the other observation was taken can be supplied here, or NULL if it is unknown.
		  *
		  * \return Returns a likelihood measurement, in the range [0,1].
		  *	\exception std::exception On any error, as another observation being of an invalid class.
		  */
		 float  likelihoodWith( const CObservation *anotherObs, const CPosePDF *anotherObsPose = NULL ) const;

		// See base class docs
		void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose = refCameraPose; }
		// See base class docs
		void setSensorPose( const CPose3D &newSensorPose ) { refCameraPose = newSensorPose; }

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationVisualLandmarks , CObservation, VISION_IMPEXP )


	} // End of namespace
} // End of namespace

#endif
