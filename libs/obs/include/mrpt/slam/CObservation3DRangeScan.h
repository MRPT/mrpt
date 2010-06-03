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
#ifndef CObservation3DRangeScan_H
#define CObservation3DRangeScan_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

#include <mrpt/math/CPolygon.h>


namespace mrpt
{
namespace slam
{

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservation3DRangeScan, CObservation,OBS_IMPEXP )

	/** Declares a class derived from "CObservation" that
	 *      encapsules a 3D range scan measurement (e.g. from a time of flight range camera).
	 *  This kind of observations can carry one or more of these data fields:
	 *    - 3D point cloud (as float's instead of double's to save storage space - precision is not a problem in this case).
	 *    - 2D range image (as a matrix).
	 *    - 2D intensity image (as a CImage).
	 *    - 2D confidence image (as a matrix).
	 *
	 *  The coordinates of the 3D point cloud are in meters with respect to the optical center of the camera,
	 *    with the +X axis pointing forward, +Y pointing left-hand and +Z pointing up.
	 *
	 *  The 2D images and matrices are stored as common images, with an up->down rows order and left->right, as usual.
	 *
	 *  A class that grabs observations of this type is mrpt::hwdrivers::CSwissRanger3DCamera
	 *
	 * \sa mrpt::hwdrivers::CSwissRanger3DCamera, CObservation
	 */
	class OBS_IMPEXP CObservation3DRangeScan : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservation3DRangeScan )

	public:
		/** Default constructor
		 */
		CObservation3DRangeScan( );

		/** Destructor
		 */
		virtual ~CObservation3DRangeScan( );

		bool hasPoints3D; 								//!< true means the field points3D contains valid data.
		vector_float points3D_x;   //!< If hasPoints3D=true, the X coordinates of the 3D point cloud detected by the camera.
		vector_float points3D_y;   //!< If hasPoints3D=true, the Y coordinates of the 3D point cloud detected by the camera.
		vector_float points3D_z;   //!< If hasPoints3D=true, the Z coordinates of the 3D point cloud detected by the camera.

		bool hasRangeImage; 				//!< true means the field rangeImage contains valid data
		mrpt::math::CMatrix rangeImage; 	//!< If hasRangeImage=true, a matrix of floats with the range data as captured by the camera (in meters).

		bool hasIntensityImage; 			//!< true means the field intensityImage contains valid data
		mrpt::utils::CImage intensityImage; 	//!< If hasIntensityImage=true, a gray-level intensity image of the same size than "rangeImage"

		bool hasConfidenceImage; 			//!< true means the field confidenceImage contains valid data
		mrpt::math::CMatrix confidenceImage;  //!< If hasConfidenceImage=true, a matrix of floats with the "confidence" value [range 0-1] as estimated by the capture drivers.


		float  	maxRange;	//!< The maximum range allowed by the device, in meters (e.g. 8.0m, 5.0m,...)
		CPose3D	sensorPose;	//!< The 6D pose of the sensor on the robot.
		float	stdError;	//!< The "sigma" error of the device in meters, used while inserting the scan in an occupancy grid.


		/** A general method to retrieve the sensor pose on the robot.
		  *  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose = sensorPose; }

		/** A general method to change the sensor pose on the robot.
		  *  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose ) { sensorPose = newSensorPose; }


		void swap(CObservation3DRangeScan &o);	//!< Very efficient method to swap the contents of two observations.

	}; // End of class def.


	} // End of namespace

	namespace utils
	{
		using namespace ::mrpt::slam;
		// Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME_PTR(CObservation3DRangeScan)
	}

} // End of namespace

#endif
