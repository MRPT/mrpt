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
#ifndef CObservation3DRangeScan_H
#define CObservation3DRangeScan_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/utils/adapters.h>


namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservation3DRangeScan, CObservation,OBS_IMPEXP )

	namespace detail {
		// Implemented in CObservation3DRangeScan_project3D_impl.h
		template <class POINTMAP>
		void project3DPointsFromDepthImageInto(CObservation3DRangeScan    & src_obs,POINTMAP                   & dest_pointcloud,const bool                   takeIntoAccountSensorPoseOnRobot,const mrpt::poses::CPose3D * robotPoseInTheWorld,const bool                   PROJ3D_USE_LUT);
	}

	/** Declares a class derived from "CObservation" that
	 *      encapsules a 3D range scan measurement (e.g. from a time of flight range camera).
	 *  This kind of observations can carry one or more of these data fields:
	 *    - 3D point cloud (as float's).
	 *    - 2D range image (as a matrix): Each entry in the matrix "rangeImage(ROW,COLUMN)" contains a distance or a depth (in meters), depending on \a range_is_depth.
	 *    - 2D intensity (grayscale or RGB) image (as a mrpt::utils::CImage): For SwissRanger cameras, a logarithmic A-law compression is used to convert the original 16bit intensity to a more standard 8bit graylevel.
	 *    - 2D confidence image (as a mrpt::utils::CImage): For each pixel, a 0x00 and a 0xFF mean the lowest and highest confidence levels, respectively.
	 *
	 *  The coordinates of the 3D point cloud are in meters with respect to the depth camera origin of coordinates
	 *    (in SwissRanger, the front face of the camera: a small offset ~1cm in front of the physical focal point),
	 *    with the +X axis pointing forward, +Y pointing left-hand and +Z pointing up.
	 *  The field CObservation3DRangeScan::relativePoseIntensityWRTDepth describes the change of coordinates from
	 *    the depth camera to the intensity (RGB or grayscale) camera. In a SwissRanger camera both cameras coincide,
	 *    so this pose is just a rotation (0,0,0,-90deg,0,-90deg). But in
	 *    Microsoft Kinect there is also an offset, as shown in this figure:
	 *
	 *  <div align=center>
	 *   <img src="CObservation3DRangeScan_figRefSystem.png">
	 *  </div>
	 *
	 *  In any case, check the field \a relativePoseIntensityWRTDepth, or the method \a doDepthAndIntensityCamerasCoincide()
	 *    to determine if both frames of reference coincide, since even for Kinect cameras both can coincide if the images
	 *    have been rectified.
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
	 *  There are two sets of calibration parameters:
	 *		- cameraParams: Projection parameters of the depth camera.
	 *		- cameraParamsIntensity: Projection parameters of the intensity (gray-level or RGB) camera.
	 *
	 *  In some cameras, like SwissRanger, both are the same. It is possible in Kinect to rectify the range images such both cameras
	 *   seem to coincide and then both sets of camera parameters will be identical.
	 *
	 *  Range data can be interpreted in two different ways depending on the 3D camera (this field is already set to the
	 *    correct setting when grabbing observations from an mrpt::hwdrivers sensor):
	 *		- range_is_depth=true  -> Kinect-like ranges: entries of \a rangeImage are distances along the +X axis
	 *		- range_is_depth=false -> Ranges in \a rangeImage are actual distances in 3D.
	 *
	 *  The "intensity" channel may come from different channels in sesnsors as Kinect. Look at field \a intensityImageChannel to
	 *    find out if the image was grabbed from the visible (RGB) or IR channels.
	 *
	 *  3D point clouds can be generated at any moment after grabbing with CObservation3DRangeScan::project3DPointsFromDepthImage(), provided the correct
	 *   calibration parameters.
	 *
	 *  \note Starting at serialization version 2 (MRPT 0.9.1+), the confidence channel is stored as an image instead of a matrix to optimize memory and disk space.
	 *  \note Starting at serialization version 3 (MRPT 0.9.1+), the 3D point cloud and the rangeImage can both be stored externally to save rawlog space.
	 *  \note Starting at serialization version 5 (MRPT 0.9.5+), the new field \a range_is_depth
	 *  \note Starting at serialization version 6 (MRPT 0.9.5+), the new field \a intensityImageChannel
	 *
	 * \sa mrpt::hwdrivers::CSwissRanger3DCamera, mrpt::hwdrivers::CKinect, CObservation
	 * \ingroup mrpt_obs_grp
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

		/** Project the RGB+D images into a 3D point cloud (with color if the target map supports it) and optionally at a given 3D pose.
		  *  The 3D point coordinates are computed from the depth image (\a rangeImage) and the depth camera camera parameters (\a cameraParams).
		  *  There exist two set of formulas for projecting the i'th point, depending on the value of "range_is_depth".
		  *   In all formulas below, "rangeImage" is the matrix of ranges and the pixel coordinates are (r,c).
		  *
		  *  1) [range_is_depth=true] With "range equals depth" or "Kinect-like depth mode": the range values
		  *      are in fact distances along the "+X" axis, not real 3D ranges (this is the way Kinect reports ranges):
		  *
		  * \code
		  *   x(i) = rangeImage(r,c)
		  *   y(i) = (r_cx - c) * x(i) / r_fx
		  *   z(i) = (r_cy - r) * x(i) / r_fy
		  * \endcode
		  *
		  *
		  *  2) [range_is_depth=false] With "normal ranges": range means distance in 3D. This must be set when
		  *      processing data from the SwissRange 3D camera, among others.
		  *
		  * \code
		  *   Ky = (r_cx - c)/r_fx
		  *   Kz = (r_cy - r)/r_fy
		  *
		  *   x(i) = rangeImage(r,c) / sqrt( 1 + Ky^2 + Kz^2 )
		  *   y(i) = Ky * x(i)
		  *   z(i) = Kz * x(i)
		  * \endcode
		  *
		  *  The color of each point is determined by projecting the 3D local point into the RGB image using \a cameraParamsIntensity.
		  *
		  *  By default the local coordinates of points are directly stored into the local map, but if indicated so in \a takeIntoAccountSensorPoseOnRobot
		  *  the points are transformed with \a sensorPose. Furthermore, if provided, those coordinates are transformed with \a robotPoseInTheWorld
		  *
		  * \param[in] PROJ3D_USE_LUT (Only when range_is_depth=true) Whether to use a Look-up-table (LUT) to speed up the conversion. It's thread safe in all situations <b>except</b> when you call this method from different threads <b>and</b> with different camera parameter matrices. In all other cases, it's a good idea to left it enabled.
		  * \tparam POINTMAP Supported maps are all those covered by mrpt::utils::PointCloudAdapter (mrpt::slam::CPointsMap and derived, mrpt::opengl::CPointCloudColoured, PCL point clouds,...)
		  *
		  * \note In MRPT < 0.9.5, this method always assumes that ranges were in Kinect-like format.
		  */
		template <class POINTMAP>
		inline void project3DPointsFromDepthImageInto(
			POINTMAP                   & dest_pointcloud,
			const bool takeIntoAccountSensorPoseOnRobot,
			const mrpt::poses::CPose3D *robotPoseInTheWorld=NULL,
			const bool PROJ3D_USE_LUT=true)
		{
			detail::project3DPointsFromDepthImageInto<POINTMAP>(*this,dest_pointcloud,takeIntoAccountSensorPoseOnRobot,robotPoseInTheWorld,PROJ3D_USE_LUT);
		}

		/** This method is equivalent to \c project3DPointsFromDepthImageInto() storing the projected 3D points (without color, in local coordinates) in this same class.
		  *  For new code it's recommended to use instead \c project3DPointsFromDepthImageInto() which is much more versatile.
		  */
		inline void project3DPointsFromDepthImage(const bool PROJ3D_USE_LUT=true) {
			this->project3DPointsFromDepthImageInto(*this,false,NULL,PROJ3D_USE_LUT);
		}

		bool hasPoints3D; 								//!< true means the field points3D contains valid data.
		std::vector<float> points3D_x;   //!< If hasPoints3D=true, the X coordinates of the 3D point cloud detected by the camera. \sa resizePoints3DVectors
		std::vector<float> points3D_y;   //!< If hasPoints3D=true, the Y coordinates of the 3D point cloud detected by the camera. \sa resizePoints3DVectors
		std::vector<float> points3D_z;   //!< If hasPoints3D=true, the Z coordinates of the 3D point cloud detected by the camera. \sa resizePoints3DVectors

		/** Use this method instead of resizing all three \a points3D_x, \a points3D_y & \a points3D_z to allow the usage of the internal memory pool. */
		void resizePoints3DVectors(const size_t nPoints);

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
		mrpt::math::CMatrix rangeImage; 	//!< If hasRangeImage=true, a matrix of floats with the range data as captured by the camera (in meters) \sa range_is_depth
		bool range_is_depth;				//!< true: Kinect-like ranges: entries of \a rangeImage are distances along the +X axis; false: Ranges in \a rangeImage are actual distances in 3D.

		void rangeImage_setSize(const int HEIGHT, const int WIDTH); //!< Similar to calling "rangeImage.setSize(H,W)" but this method provides memory pooling to speed-up the memory allocation.

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
		/** Forces marking this observation as non-externally stored - it doesn't anything else apart from reseting the corresponding flag (Users won't normally want to call this, it's only used from internal MRPT programs) */
		void rangeImage_forceResetExternalStorage() { m_rangeImage_external_stored=false; }
		// ---------

		/** Enum type for intensityImageChannel */
		enum TIntensityChannelID
		{
			CH_VISIBLE = 0, //!< Grayscale or RGB visible channel of the camera sensor.
			CH_IR      = 1  //!< Infrarred (IR) channel
		};

		bool hasIntensityImage;                    //!< true means the field intensityImage contains valid data
		mrpt::utils::CImage intensityImage;        //!< If hasIntensityImage=true, a color or gray-level intensity image of the same size than "rangeImage"
		TIntensityChannelID intensityImageChannel; //!< The source of the intensityImage; typically the visible channel \sa TIntensityChannelID

		bool hasConfidenceImage; 			//!< true means the field confidenceImage contains valid data
		mrpt::utils::CImage confidenceImage;  //!< If hasConfidenceImage=true, an image with the "confidence" value [range 0-255] as estimated by the capture drivers.

		mrpt::utils::TCamera	cameraParams;	//!< Projection parameters of the depth camera.
		mrpt::utils::TCamera	cameraParamsIntensity;	//!< Projection parameters of the intensity (graylevel or RGB) camera.

		/** Relative pose of the intensity camera wrt the depth camera (which is the coordinates origin for this observation).
		  *  In a SwissRanger camera, this will be (0,0,0,-90deg,0,-90deg) since both cameras coincide.
		  *  In a Kinect, this will include a small lateral displacement and a rotation, according to the drawing on the top of this page.
		  *  \sa doDepthAndIntensityCamerasCoincide
		  */
		mrpt::poses::CPose3D    relativePoseIntensityWRTDepth;

		/** Return true if \a relativePoseIntensityWRTDepth equals the pure rotation (0,0,0,-90deg,0,-90deg) (with a small comparison epsilon)
		  * \sa relativePoseIntensityWRTDepth
		  */
		bool doDepthAndIntensityCamerasCoincide() const;


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

		/** Look-up-table struct for project3DPointsFromDepthImageInto() */
		struct TCached3DProjTables
		{
			mrpt::vector_float Kzs,Kys;
			TCamera  prev_camParams;
		};
		static TCached3DProjTables m_3dproj_lut; //!< 3D point cloud projection look-up-table \sa project3DPointsFromDepthImage

	}; // End of class def.


	} // End of namespace

	namespace utils
	{
		using namespace ::mrpt::slam;
		// Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME_PTR(CObservation3DRangeScan)

		// Enum <-> string converter:
		template <>
		struct TEnumTypeFiller<slam::CObservation3DRangeScan::TIntensityChannelID>
		{
			typedef slam::CObservation3DRangeScan::TIntensityChannelID enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(slam::CObservation3DRangeScan::CH_VISIBLE, "CH_VISIBLE");
				m_map.insert(slam::CObservation3DRangeScan::CH_IR, "CH_IR");
			}
		};
	}

	namespace utils
	{
		using mrpt::slam::CObservation3DRangeScan;

		/** Specialization mrpt::utils::PointCloudAdapter<CObservation3DRangeScan> \ingroup mrpt_adapters_grp */
		template <>
		class PointCloudAdapter<CObservation3DRangeScan> : public detail::PointCloudAdapterHelperNoRGB<CObservation3DRangeScan,float>
		{
		private:
			CObservation3DRangeScan &m_obj;
		public:
			typedef float  coords_t;  //!< The type of each point XYZ coordinates
			static const int HAS_RGB   = 0;  //!< Has any color RGB info?
			static const int HAS_RGBf  = 0;  //!< Has native RGB info (as floats)?
			static const int HAS_RGBu8 = 0;  //!< Has native RGB info (as uint8_t)?

			/** Constructor (accept a const ref for convenience) */
			inline PointCloudAdapter(const CObservation3DRangeScan &obj) : m_obj(*const_cast<CObservation3DRangeScan*>(&obj)) { }
			/** Get number of points */
			inline size_t size() const { return m_obj.points3D_x.size(); }
			/** Set number of points (to uninitialized values) */
			inline void resize(const size_t N) {
				if (N) m_obj.hasPoints3D = true;
				m_obj.resizePoints3DVectors(N);
			}

			/** Get XYZ coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ(const size_t idx, T &x,T &y, T &z) const {
				x=m_obj.points3D_x[idx];
				y=m_obj.points3D_y[idx];
				z=m_obj.points3D_z[idx];
			}
			/** Set XYZ coordinates of i'th point */
			inline void setPointXYZ(const size_t idx, const coords_t x,const coords_t y, const coords_t z) {
				m_obj.points3D_x[idx]=x;
				m_obj.points3D_y[idx]=y;
				m_obj.points3D_z[idx]=z;
			}
		}; // end of PointCloudAdapter<CObservation3DRangeScan>
	}
} // End of namespace

#include "CObservation3DRangeScan_project3D_impl.h"

#endif
