/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationRGBD360_H
#define CObservationRGBD360_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/adapters.h>

namespace mrpt
{
namespace obs
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationRGBD360, CObservation, OBS_IMPEXP )

//	namespace detail {
//		// Implemented in CObservationRGBD360_project3D_impl.h
//		template <class POINTMAP>
//		void project3DPointsFromDepthImageInto(CObservationRGBD360 &src_obs, POINTMAP &dest_pointcloud, const bool takeIntoAccountSensorPoseOnRobot, const mrpt::poses::CPose3D * robotPoseInTheWorld, const bool PROJ3D_USE_LUT);
//	}

	/** Declares a class derived from "CObservation" that
	 *      encapsules an omnidirectional RGBD measurement from a set of RGBD sensors.
	 *  This kind of observations can carry one or more of these data fields:
	 *    - 3D point cloud (as float's).
	 *    - 2D range image (as a matrix): Each entry in the matrix "rangeImage(ROW,COLUMN)" contains a distance or a depth (in meters), depending on \a range_is_depth.
	 *    - 2D intensity (grayscale or RGB) image (as a mrpt::utils::CImage): For SwissRanger cameras, a logarithmic A-law compression is used to convert the original 16bit intensity to a more standard 8bit graylevel.
	 *
	 *  The coordinates of the 3D point cloud are in millimeters with respect to the RGB camera origin of coordinates
	 *
	 *  The 2D images and matrices are stored as common images, with an up->down rows order and left->right, as usual.
	 *   Optionally, the intensity and confidence channels can be set to delayed-load images for off-rawlog storage so it saves
	 *   memory by having loaded in memory just the needed images. See the methods load() and unload().
	 *  Due to the intensive storage requirements of this kind of observations, this observation is the only one in MRPT
	 *   for which it's recommended to always call "load()" and "unload()" before and after using the observation, *ONLY* when
	 *   the observation was read from a rawlog dataset, in order to make sure that all the externally stored data fields are
	 *   loaded and ready in memory.
	 *
	 *  Classes that grab observations of this type are:
	 *		- mrpt::hwdrivers::CSwissRanger3DCamera
	 *		- mrpt::hwdrivers::CKinect
	 *
	 *
	 *  3D point clouds can be generated at any moment after grabbing with CObservationRGBD360::project3DPointsFromDepthImage() and CObservationRGBD360::project3DPointsFromDepthImageInto(), provided the correct
	 *   calibration parameters.
	 *
	 *  \note Starting at serialization version 3 (MRPT 0.9.1+), the 3D point cloud and the rangeImage can both be stored externally to save rawlog space.
	 *  \note Starting at serialization version 5 (MRPT 0.9.5+), the new field \a range_is_depth
	 *  \note Starting at serialization version 6 (MRPT 0.9.5+), the new field \a intensityImageChannel
	 *
	 * \sa mrpt::hwdrivers::CSwissRanger3DCamera, mrpt::hwdrivers::COpenNI2_RGBD360, CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationRGBD360 : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationRGBD360 )

	protected:
		bool			m_points3D_external_stored; //!< If set to true, m_points3D_external_file is valid.
		std::string		m_points3D_external_file;   //!< 3D points are in CImage::IMAGES_PATH_BASE+<this_file_name>

		bool			m_rangeImage_external_stored; //!< If set to true, m_rangeImage_external_file is valid.
		std::string		m_rangeImage_external_file;   //!< rangeImage is in CImage::IMAGES_PATH_BASE+<this_file_name>

	public:
		CObservationRGBD360( );				//!< Default constructor
		virtual ~CObservationRGBD360( ); 	//!< Destructor

		static const unsigned int NUM_SENSORS = 2;

		mrpt::system::TTimeStamp  timestamps[NUM_SENSORS];

		bool hasRangeImage; 				//!< true means the field rangeImage contains valid data
		mrpt::math::CMatrix rangeImages[NUM_SENSORS]; 	//!< If hasRangeImage=true, a matrix of floats with the range data as captured by the camera (in meters) \sa range_is_depth

		void rangeImage_setSize(const int HEIGHT, const int WIDTH, const unsigned sensor_id); //!< Similar to calling "rangeImage.setSize(H,W)" but this method provides memory pooling to speed-up the memory allocation.

		bool hasIntensityImage;                    //!< true means the field intensityImage contains valid data
		mrpt::utils::CImage intensityImages[NUM_SENSORS];        //!< If hasIntensityImage=true, a color or gray-level intensity image of the same size than "rangeImage"
		mrpt::utils::TCamera sensorParamss[NUM_SENSORS];	//!< Projection parameters of the 8 RGBD sensor.

		float  	maxRange;	//!< The maximum range allowed by the device, in meters (e.g. 8.0m, 5.0m,...)
		mrpt::poses::CPose3D	sensorPose;	//!< The 6D pose of the sensor on the robot.
		float	stdError;	//!< The "sigma" error of the device in meters, used while inserting the scan in an occupancy grid.

		// See base class docs
		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const MRPT_OVERRIDE { out_sensorPose = sensorPose; }
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) MRPT_OVERRIDE { sensorPose = newSensorPose; }
		void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationRGBD360, CObservation, OBS_IMPEXP )


	} // End of namespace

} // End of namespace

#endif
