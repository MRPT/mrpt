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
