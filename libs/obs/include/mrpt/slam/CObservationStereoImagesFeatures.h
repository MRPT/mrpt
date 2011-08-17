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
#ifndef CObservationStereoImagesFeatures_H
#define CObservationStereoImagesFeatures_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/TCamera.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace slam
{
	using namespace mrpt::utils;
	using namespace mrpt::slam;

	struct OBS_IMPEXP TStereoImageFeatures
	{
		std::pair<TPixelCoordf,TPixelCoordf>	pixels;
		unsigned int 							ID;
	};

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationStereoImagesFeatures , CObservation, OBS_IMPEXP )
	/** Declares a class derived from "CObservation" that encapsules a pair of cameras and a set of matched image features extracted from them.
	 *
	 <b>NOTE:</b> The image features stored in this class are NOT supposed to be UNDISTORTED, but the TCamera members must provide their distortion params. 
	 A zero-vector of distortion params means a set of UNDISTORTED pixels.<br>
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationStereoImagesFeatures : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationStereoImagesFeatures )

	 public:
		/** Default Constructor.
		 */
		CObservationStereoImagesFeatures( );

		/** Other constructor providing members initialization.
		 */
		CObservationStereoImagesFeatures( 
			const CMatrixDouble33 &iPLeft /*left intrinsic params*/, const CMatrixDouble33 &iPRight /*right intrinsic params*/,
			const CArrayDouble<5> &dPLeft /*left distortion params*/, const CArrayDouble<5> &dPRight /*right distortion params*/,
			const CPose3DQuat &rCPose /*rightCameraPose*/, const CPose3DQuat &cPORobot /*cameraPoseOnRobot*/ );
		
		/** Other constructor providing members initialization.
		 */
		CObservationStereoImagesFeatures( 
			const TCamera &cLeft /*left camera*/, const TCamera &cRight /*right camera*/,
			const CPose3DQuat &rCPose /*rightCameraPose*/, const CPose3DQuat &cPORobot /*cameraPoseOnRobot*/ );

		/** Destructor
		 */
		~CObservationStereoImagesFeatures( );

		/** A method for storing the set of observed features in a text file in the format: <br>
		  * ID ul vl ur vr <br>
		  * being (ul,vl) and (ur,vr) the "x" and "y" coordinates for the left and right feature, respectively.
		 */
		void  saveFeaturesToTextFile( const std::string &filename );

		// ------------------
		// Class Members
		// ------------------
		TCamera cameraLeft, cameraRight;
		
		/** The pose of the right camera, relative to the left one:
		  *  Note that for the Bumblebee stereo camera and using the conventional reference coordinates for the left
		  *   camera ("x" points to the right, "y" down), the "right" camera is situated
		  *   at position (BL, 0, 0) with q = [1 0 0 0], where BL is the BASELINE.
		  */
		CPose3DQuat	rightCameraPose;

		/** The pose of the LEFT camera, relative to the robot.
		  */
		CPose3DQuat	cameraPoseOnRobot;

		/** Vectors of image feature pairs (with ID).
		  */
		std::vector<TStereoImageFeatures> theFeatures;

		/** A general method to retrieve the sensor pose on the robot in CPose3D form.
		  *  Note that most sensors will return a full (6D) CPose3DQuat, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		inline void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose = CPose3D(cameraPoseOnRobot); }

		/** A general method to retrieve the sensor pose on the robot in CPose3DQuat form.
		  *  Note that most sensors will return a full (6D) CPose3DQuat, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		inline void getSensorPose( CPose3DQuat &out_sensorPose ) const { out_sensorPose = cameraPoseOnRobot; }

		/** A general method to change the sensor pose on the robot in a CPose3D form.
		  *  Note that most sensors will use the full (6D) CPose3DQuat, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		inline void setSensorPose( const CPose3D &newSensorPose ) { cameraPoseOnRobot = CPose3DQuat(newSensorPose); }

		/** A general method to change the sensor pose on the robot in a CPose3DQuat form.
		  *  Note that most sensors will use the full (6D) CPose3DQuat, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		inline void setSensorPose( const CPose3DQuat &newSensorPose ) { cameraPoseOnRobot = newSensorPose; }
	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
