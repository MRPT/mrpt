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
#ifndef CObservationStereoImages_H
#define CObservationStereoImages_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose2D.h>
//#include <mrpt/slam/CLandmark.h>
//#include <mrpt/slam/CLandmarksMap.h>
//#include <mrpt/vision/CCamModel.h>

namespace mrpt
{
namespace slam
{
	using namespace mrpt::utils;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationStereoImages , CObservation,OBS_IMPEXP )

	/** Observation class for either a pair of left+right or left+disparity images from a stereo camera.
	  *
	  *  To find whether the observation contains a right image and/or a disparity image, see the
	  *   fields hasImageDisparity and hasImageRight, respectively.
	  *
	  *   This figure illustrates the coordinate frames involved in this class:
	  *
	  *	 <center>
	  *   <img src="CObservationStereoImages_figRefSystem.png">
	  *  </center>
	  *
	  * \note The images stored in this class can be raw or undistorted images. In the latter case, the "distortion" params of the corresponding "leftCamera" and "rightCamera" fields should be all zeros.
	  * \sa CObservation
	 * \ingroup mrpt_obs_grp
	  */
	class OBS_IMPEXP CObservationStereoImages : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationStereoImages )

	 public:
		/** Default Constructor */
		CObservationStereoImages( );

		/** Constructor from "IplImage*" images, which could be NULL.
		  *  The fields hasImageDisparity and hasImageRight will be set to true/false depending on them being !=NULL.
		  * Note that the IplImage's will be COPIED, so it's still the caller's reponsibility to free the original images,
		  *  unless ownMemory is set to true: in that case the IplImage pointers are copied and those IplImage's will be automatically freed by this object.
		  *
		  */
		CObservationStereoImages( void *iplImageLeft, void *iplImageRight, void *iplImageDisparity = NULL, bool ownMemory = false );

		/** Destructor
		 */
		~CObservationStereoImages(  );


		/** @name Main observation data members
		    @{ */

		/** Image from the left camera (this image will be ALWAYS present) */
		mrpt::utils::CImage	imageLeft;

		/** Image from the right camera, only contains a valid image if hasImageRight == true. */
		mrpt::utils::CImage	imageRight;

		/** Disparity image, only contains a valid image if hasImageDisparity == true.
		  *  The relation between the actual disparity and pixels and each value in this image is... ???????????  */
		mrpt::utils::CImage imageDisparity;

		bool   hasImageDisparity; //!< Whether imageDisparity actually contains data (Default upon construction: false)
		bool   hasImageRight;        //!< Whether imageRight actually contains data  (Default upon construction: true)

		/** Parameters for the left/right cameras: individual intrinsic and distortion parameters of the cameras.
		  * See the <a href="http://www.mrpt.org/Camera_Parameters" >tutorial</a> for a discussion of these parameters.
		  */
		TCamera		leftCamera, rightCamera;

		CPose3DQuat	cameraPose;		//!< The pose of the LEFT camera, relative to the robot.

		/** The pose of the right camera, relative to the left one:
		  *  Note that using the conventional reference coordinates for the left
		  *   camera (x points to the right, y down), the "right" camera is situated
		  *   at position (BL, 0, 0) with yaw=pitch=roll=0, where BL is the BASELINE.
		  */
		CPose3DQuat	rightCameraPose;

		/** @} */


		/** A general method to retrieve the sensor pose on the robot.
		  *  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose = cameraPose; }


		/** A general method to change the sensor pose on the robot.
		  *  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose ) { cameraPose = CPose3DQuat(newSensorPose); }

		void swap( CObservationStereoImages &o); //!< Do an efficient swap of all data members of this object with "o".

	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
