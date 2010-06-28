/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
		std::pair<TPixelCoordf,TPixelCoordf> pixels;
		unsigned int 					ID;
	};

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationStereoImagesFeatures , CObservation, OBS_IMPEXP )

	/** Declares a class derived from "CObservation" that encapsule a pair of images taken by a stereo camera.
	     The next figure illustrate the coordinates reference systems involved in this class:<br>
		 <center>
		 <img src="CObservationStereoImages_figRefSystem.png">
		 </center>
	 *
	 <br>
	 <b>NOTE:</b> The images stored in this class are supposed to be UNDISTORTED images already.<br>
	 * \sa CObservation
	 */
	class OBS_IMPEXP CObservationStereoImagesFeatures : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationStereoImagesFeatures )

	 public:
		/** Default Constructor.
		 */
		CObservationStereoImagesFeatures( );

		/** Constructor.
		 */
		CObservationStereoImagesFeatures( 
			const CMatrixDouble33 &iPLeft, const CMatrixDouble33 &iPRight,
			const CArrayDouble<5> &dPLeft, const CArrayDouble<5> &dPRight,
			const CPose3DQuat &rCPose, const CPose3DQuat &cPORobot );
		
		CObservationStereoImagesFeatures( 
			const TCamera &cLeft, const TCamera &cRight,
			const CPose3DQuat &rCPose, const CPose3DQuat &cPORobot );

		/** Destructor
		 */
		~CObservationStereoImagesFeatures( );

		// ------------------
		// Class Members
		// ------------------
		TCamera cameraLeft, cameraRight;
		
		/** The pose of the right camera, relative to the left one:
		  *  Note that using the conventional reference coordinates for the left
		  *   camera ("x" points to the right, "y" down), the "right" camera is situated
		  *   at position (BL, 0, 0) with q = [1 0 0 0], where BL is the BASELINE.
		  */
		CPose3DQuat	rightCameraPose;

		/** The pose of the LEFT camera, relative to the robot.
		  */
		CPose3DQuat	cameraPoseOnRobot;

		/** Vectors of image feature pairs (with ID).
		  */
		vector<TStereoImageFeatures> theFeatures;
		// vector<TPixelCoordf> featsLeft, featsRight;

		///** Vectors of image features IDs. Equal IDs corresponds to the projection of the same 3D points.
		//  */
		//vector<unsigned int> featsLeftIDs, featsRightIDs;

		/** A general method to retrieve the sensor pose on the robot.
		  *  Note that most sensors will return a full (6D) CPose3DQuat, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose = CPose3D(cameraPoseOnRobot); }


		/** A general method to change the sensor pose on the robot.
		  *  Note that most sensors will use the full (6D) CPose3DQuat, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose ) { cameraPoseOnRobot = CPose3DQuat(newSensorPose); }

	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
