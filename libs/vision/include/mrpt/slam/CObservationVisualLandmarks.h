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

		/** A general method to retrieve the sensor pose on the robot.
		  *  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose = refCameraPose; }


		/** A general method to change the sensor pose on the robot.
		  *  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose ) { refCameraPose = newSensorPose; }


	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
