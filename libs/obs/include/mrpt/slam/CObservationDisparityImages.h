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
#ifndef CObservationDisparityImages_H
#define CObservationDisparityImages_H

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
	//using namespace mrpt::vision;
	//class CLandmarksMap;

        DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationDisparityImages , CObservation,OBS_IMPEXP )

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
        class OBS_IMPEXP CObservationDisparityImages : public CObservation
	{
		// This must be added to any CSerializable derived class:
                DEFINE_SERIALIZABLE( CObservationDisparityImages )

		/** If buildAuxiliarMap is called before, this will contain the landmarks-map representation of the observation, for the robot located at the origin.
		  */
		//class CAuxMapWrapper
		//{
		//	CLandmarksMap	*auxMap;
		//public:
		//	CAuxMapWrapper() : auxMap(NULL)  { }
		//	CAuxMapWrapper(const CAuxMapWrapper &o) : auxMap(NULL) {  }
		//	CAuxMapWrapper & operator =(const CAuxMapWrapper &o) { clear(); return *this; }

		//	~CAuxMapWrapper() { clear(); }

		//	CLandmarksMap * get() { return auxMap; }
		//	const CLandmarksMap * get() const { return auxMap; }

		//	void set(CLandmarksMap	*m);

		//	void clear();
		//};
		//mutable CAuxMapWrapper	m_auxMap;


	 public:
		/** Default Constructor.
		 *
		 */
                CObservationDisparityImages( );

		/** Constructor.
		 * \param iplImageLeft An OpenCV "IplImage*" object with the image to be loaded in the member "imageLeft", or NULL (default) for an empty image.
		 * \param iplImageRight An OpenCV "IplImage*" object with the image to be loaded in the member "imageRight", or NULL (default) for an empty image.
		 *
		 */
                CObservationDisparityImages( void *iplImageLeft, void *iplImageRight );

		/** Destructor
		 */
                ~CObservationDisparityImages(  );

		 /** The pose of the LEFT camera, relative to the robot.
		  */
		CPose3DQuat	cameraPose;

		 /** Parameters for the left/right cameras: individual intrinsic and distortion parameters of the cameras.
		   * See the <a href="http://www.mrpt.org/Camera_Parameters">tutorial</a> for a discussion of these parameters.
		  */
                TCamera		leftCamera;

		/** The pair of images.
		  */
                CImage		imageLeft, imageDisparity;



		/** This method build the map in "m_auxMap", only the first time this is called.
		  */
//		const CLandmarksMap * buildAuxiliaryMap( CLandmark::TLandmarkID fID, const CLandmarksMap::TInsertionOptions *insOpts = NULL) const;

		/** A general method to retrieve the sensor pose on the robot.
		  *  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose = cameraPose; }


		/** A general method to change the sensor pose on the robot.
		  *  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose ) { cameraPose = newSensorPose; }

		//void getRectifiedImages(


	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
