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
	 *    - 2D range image (as a matrix): Each entry in the matrix "rangeImage(x,y)" contains the distance of the pixel (x,y), in meters.
	 *    - 2D intensity image (as a CImage): A logarithmic A-law compression is used to convert the original 16bit intensity to a more standard 8bit graylevel.
	 *    - 2D confidence image (as a CImage): For each pixel, a 0x00 and a 0xFF mean the lowest and highest confidence levels, respectively.
	 *
	 *  The coordinates of the 3D point cloud are in meters with respect to the front face of the camera (i.e. a small offset ~1cm in front of the physical focal point),
	 *    with the +X axis pointing forward, +Y pointing left-hand and +Z pointing up.
	 *
	 *  The 2D images and matrices are stored as common images, with an up->down rows order and left->right, as usual.
	 *   Optionally, the intensity and confidence channels can be set to delayed-load images for off-rawlog storage so it saves
	 *   memory by having loaded in memory just the needed images. See the methods load() and unload(). 
	 *  Due to the intensive storage requirements of this kind of observations, this observation is the only one in MRPT
	 *   for which it's recommended to always call "load()" and "unload()" before and after using the observation, *ONLY* when
	 *   the observation was read from a rawlog dataset, in order to make sure that all the externally stored data fields are
	 *   loaded and ready in memory.
	 *
	 *  A class that grabs observations of this type is mrpt::hwdrivers::CSwissRanger3DCamera
	 *
	 *  \note Starting at serialization version 2 (MRPT 0.9.1+), the confidence channel is stored as an image instead of a matrix to optimize memory and disk space.
	 *  \note Starting at serialization version 3 (MRPT 0.9.1+), the 3D point cloud and the rangeImage can both be stored externally to save rawlog space.
	 *
	 * \sa mrpt::hwdrivers::CSwissRanger3DCamera, CObservation
	 */
	class OBS_IMPEXP CObservation3DRangeScan : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservation3DRangeScan )

	protected:
		bool			m_points3D_external_stored; //!< If set to true, m_points3D_external_file is valid.
		std::string		m_points3D_external_file;   //!< 3D points are in CImage::IMAGES_PATH_BASE+<this_file_name> 

		bool			m_rangeImage_external_stored; //!< If set to true, m_rangeImage_external_file is valid.
		std::string		m_rangeImage_external_file;   //!< rangeImage is in CImage::IMAGES_PATH_BASE+<this_file_name> 

	public:
		CObservation3DRangeScan( );				//!< Default constructor
		virtual ~CObservation3DRangeScan( ); 	//!< Destructor

		/** @name Delayed-load manual control methods.
		    @{ */
		/** Makes sure all images and other fields which may be externally stored are loaded in memory.
		  *  Note that for all CImages, calling load() is not required since the images will be automatically loaded upon first access, so load() shouldn't be needed to be called in normal cases by the user.
		  *  If all the data were alredy loaded or this object has no externally stored data fields, calling this method has no effects.
		  * \sa unload
		  */
		virtual void load() const;   
		/** Unload all images, for the case they being delayed-load images stored in external files (othewise, has no effect).
		  * \sa load
		  */
		virtual void unload(); 
		/** @} */


		bool hasPoints3D; 								//!< true means the field points3D contains valid data.
		vector_float points3D_x;   //!< If hasPoints3D=true, the X coordinates of the 3D point cloud detected by the camera.
		vector_float points3D_y;   //!< If hasPoints3D=true, the Y coordinates of the 3D point cloud detected by the camera.
		vector_float points3D_z;   //!< If hasPoints3D=true, the Z coordinates of the 3D point cloud detected by the camera.

		// 3D points external storage functions ---------
		inline bool points3D_isExternallyStored() const { return m_points3D_external_stored; }
		inline std::string points3D_getExternalStorageFile() const { return m_points3D_external_file; }
		void points3D_getExternalStorageFileAbsolutePath(std::string &out_path) const;
		inline std::string points3D_getExternalStorageFileAbsolutePath() const {
				std::string tmp;
				points3D_getExternalStorageFileAbsolutePath(tmp);
				return tmp;
		}
		void points3D_convertToExternalStorage( const std::string &fileName, const std::string &use_this_base_dir ); //!< Users won't normally want to call this, it's only used from internal MRPT programs.
		// ---------

		bool hasRangeImage; 				//!< true means the field rangeImage contains valid data
		mrpt::math::CMatrix rangeImage; 	//!< If hasRangeImage=true, a matrix of floats with the range data as captured by the camera (in meters).

		// Range Matrix external storage functions ---------
		inline bool rangeImage_isExternallyStored() const { return m_rangeImage_external_stored; }
		inline std::string rangeImage_getExternalStorageFile() const { return m_rangeImage_external_file; }
		void rangeImage_getExternalStorageFileAbsolutePath(std::string &out_path) const;
		inline std::string rangeImage_getExternalStorageFileAbsolutePath() const {
				std::string tmp;
				rangeImage_getExternalStorageFileAbsolutePath(tmp);
				return tmp;
		}
		void rangeImage_convertToExternalStorage( const std::string &fileName, const std::string &use_this_base_dir ); //!< Users won't normally want to call this, it's only used from internal MRPT programs.
		// ---------

		bool hasIntensityImage; 			//!< true means the field intensityImage contains valid data
		mrpt::utils::CImage intensityImage; 	//!< If hasIntensityImage=true, a gray-level intensity image of the same size than "rangeImage"

		bool hasConfidenceImage; 			//!< true means the field confidenceImage contains valid data
		mrpt::utils::CImage confidenceImage;  //!< If hasConfidenceImage=true, an image with the "confidence" value [range 0-255] as estimated by the capture drivers.

		mrpt::utils::TCamera	cameraParams;	//!< Projection parameters of the camera.


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

		void getZoneAsObs( CObservation3DRangeScan &obs, const unsigned int &r1, const unsigned int &r2, const unsigned int &c1, const unsigned int &c2 );

		/** A Levenberg-Marquart-based optimizer to recover the calibration parameters of a 3D camera given a range (depth) image and the corresponding 3D point cloud. 
		  * \param camera_offset The offset (in meters) in the +X direction of the point cloud. It's 1cm for SwissRanger SR4000.
		  * \return The final average reprojection error per pixel (typ <0.05 px)
		  */
		static double recoverCameraCalibrationParameters(
			const CObservation3DRangeScan	&in_obs,
			mrpt::utils::TCamera			&out_camParams,
			const double camera_offset = 0.01 );


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
