/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationRGBD360_H
#define CObservationRGBD360_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/adapters.h>

namespace mrpt
{
namespace slam
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

		/** @name Delayed-load manual control methods.
		    @{ */
//		/** Makes sure all images and other fields which may be externally stored are loaded in memory.
//		  *  Note that for all CImages, calling load() is not required since the images will be automatically loaded upon first access, so load() shouldn't be needed to be called in normal cases by the user.
//		  *  If all the data were alredy loaded or this object has no externally stored data fields, calling this method has no effects.
//		  * \sa unload
//		  */
//		virtual void load() const;
//		/** Unload all images, for the case they being delayed-load images stored in external files (othewise, has no effect).
//		  * \sa load
//		  */
//		virtual void unload();
//		/** @} */
//
//		/** Project the RGB+D images into a 3D point cloud (with color if the target map supports it) and optionally at a given 3D pose.
//		  *  The 3D point coordinates are computed from the depth image (\a rangeImage) and the depth camera camera parameters (\a cameraParams).
//		  *  There exist two set of formulas for projecting the i'th point, depending on the value of "range_is_depth".
//		  *   In all formulas below, "rangeImage" is the matrix of ranges and the pixel coordinates are (r,c).
//		  *
//		  *  1) [range_is_depth=true] With "range equals depth" or "Kinect-like depth mode": the range values
//		  *      are in fact distances along the "+X" axis, not real 3D ranges (this is the way Kinect reports ranges):
//		  *
//		  * \code
//		  *   x(i) = rangeImage(r,c)
//		  *   y(i) = (r_cx - c) * x(i) / r_fx
//		  *   z(i) = (r_cy - r) * x(i) / r_fy
//		  * \endcode
//		  *
//		  *
//		  *  2) [range_is_depth=false] With "normal ranges": range means distance in 3D. This must be set when
//		  *      processing data from the SwissRange 3D camera, among others.
//		  *
//		  * \code
//		  *   Ky = (r_cx - c)/r_fx
//		  *   Kz = (r_cy - r)/r_fy
//		  *
//		  *   x(i) = rangeImage(r,c) / sqrt( 1 + Ky^2 + Kz^2 )
//		  *   y(i) = Ky * x(i)
//		  *   z(i) = Kz * x(i)
//		  * \endcode
//		  *
//		  *  The color of each point is determined by projecting the 3D local point into the RGB image using \a cameraParamsIntensity.
//		  *
//		  *  By default the local coordinates of points are directly stored into the local map, but if indicated so in \a takeIntoAccountSensorPoseOnRobot
//		  *  the points are transformed with \a sensorPose. Furthermore, if provided, those coordinates are transformed with \a robotPoseInTheWorld
//		  *
//		  * \param[in] PROJ3D_USE_LUT (Only when range_is_depth=true) Whether to use a Look-up-table (LUT) to speed up the conversion. It's thread safe in all situations <b>except</b> when you call this method from different threads <b>and</b> with different camera parameter matrices. In all other cases, it's a good idea to left it enabled.
//		  * \tparam POINTMAP Supported maps are all those covered by mrpt::utils::PointCloudAdapter (mrpt::slam::CPointsMap and derived, mrpt::opengl::CPointCloudColoured, PCL point clouds,...)
//		  *
//		  * \note In MRPT < 0.9.5, this method always assumes that ranges were in Kinect-like format.
//		  */
//		template <class POINTMAP>
//		inline void project3DPointsFromDepthImageInto(
//			POINTMAP                   & dest_pointcloud,
//			const bool takeIntoAccountSensorPoseOnRobot,
//			const mrpt::poses::CPose3D *robotPoseInTheWorld=NULL,
//			const bool PROJ3D_USE_LUT=true)
//		{
//			detail::project3DPointsFromDepthImageInto<POINTMAP>(*this,dest_pointcloud,takeIntoAccountSensorPoseOnRobot,robotPoseInTheWorld,PROJ3D_USE_LUT);
//		}
//
//		/** This method is equivalent to \c project3DPointsFromDepthImageInto() storing the projected 3D points (without color, in local coordinates) in this same class.
//		  *  For new code it's recommended to use instead \c project3DPointsFromDepthImageInto() which is much more versatile.
//		  */
//		inline void project3DPointsFromDepthImage(const bool PROJ3D_USE_LUT=true) {
//			this->project3DPointsFromDepthImageInto(*this,false,NULL,PROJ3D_USE_LUT);
//		}
//
//
//		/** Convert this 3D observation into an "equivalent 2D fake laser scan", with a configurable vertical FOV.
//		  *
//		  *  The result is a 2D laser scan with more "rays" (N) than columns has the 3D observation (W), exactly: N = W * oversampling_ratio.
//		  *  This oversampling is required since laser scans sample the space at evenly-separated angles, while
//		  *  a range camera follows a tangent-like distribution. By oversampling we make sure we don't leave "gaps" unseen by the virtual "2D laser".
//		  *
//		  *  All obstacles within a frustum are considered and the minimum distance is kept in each direction.
//		  *  The horizontal FOV of the frustum is automatically computed from the intrinsic parameters, but the
//		  *  vertical FOV must be provided by the user, and can be set to be assymetric which may be useful
//		  *  depending on the zone of interest where to look for obstacles.
//		  *
//		  *  All spatial transformations are riguorosly taken into account in this class, using the depth camera
//		  *  intrinsic calibration parameters.
//		  *
//		  *  The timestamp of the new object is copied from the 3D object.
//		  *  Obviously, a requisite for calling this method is the 3D observation having range data,
//		  *  i.e. hasRangeImage must be true. It's not needed to have RGB data nor the raw 3D point clouds
//		  *  for this method to work.
//		  *
//		  *  \param[out] out_scan2d The resulting 2D equivalent scan.
//		  *  \param[in] sensorLabel The sensor label that will have the newly created observation.
//		  *  \param[in] angle_sup (Default=5deg) The upper half-FOV angle (in radians)
//		  *  \param[in] angle_sup (Default=5deg) The lower half-FOV angle (in radians)
//		  *  \param[in] oversampling_ratio (Default=1.2=120%) How many more laser scans rays to create (read above).
//		  *
//		  * \sa The example in http://www.mrpt.org/Example_Kinect_To_2D_laser_scan
//		  */
//		void convertTo2DScan(
//			mrpt::slam::CObservation2DRangeScan & out_scan2d,
//			const std::string       & sensorLabel,
//			const double angle_sup = DEG2RAD(5),
//			const double angle_inf = DEG2RAD(5),
//			const double oversampling_ratio = 1.2 );
//
//
//		bool hasPoints3D; 								//!< true means the field points3D contains valid data.
//		std::vector<float> points3D_x;   //!< If hasPoints3D=true, the X coordinates of the 3D point cloud detected by the camera. \sa resizePoints3DVectors
//		std::vector<float> points3D_y;   //!< If hasPoints3D=true, the Y coordinates of the 3D point cloud detected by the camera. \sa resizePoints3DVectors
//		std::vector<float> points3D_z;   //!< If hasPoints3D=true, the Z coordinates of the 3D point cloud detected by the camera. \sa resizePoints3DVectors
//
//		/** Use this method instead of resizing all three \a points3D_x, \a points3D_y & \a points3D_z to allow the usage of the internal memory pool. */
//		void resizePoints3DVectors(const size_t nPoints);
//
//		// 3D points external storage functions ---------
//		inline bool points3D_isExternallyStored() const { return m_points3D_external_stored; }
//		inline std::string points3D_getExternalStorageFile() const { return m_points3D_external_file; }
//		void points3D_getExternalStorageFileAbsolutePath(std::string &out_path) const;
//		inline std::string points3D_getExternalStorageFileAbsolutePath() const {
//				std::string tmp;
//				points3D_getExternalStorageFileAbsolutePath(tmp);
//				return tmp;
//		}
//		void points3D_convertToExternalStorage( const std::string &fileName, const std::string &use_this_base_dir ); //!< Users won't normally want to call this, it's only used from internal MRPT programs.
//		// ---------
//

		static const int NUM_SENSORS = 2;

		mrpt::system::TTimeStamp  timestamps[NUM_SENSORS];

		bool hasRangeImage; 				//!< true means the field rangeImage contains valid data
		mrpt::math::CMatrix rangeImages[NUM_SENSORS]; 	//!< If hasRangeImage=true, a matrix of floats with the range data as captured by the camera (in meters) \sa range_is_depth

		void rangeImage_setSize(const int HEIGHT, const int WIDTH, const unsigned sensor_id); //!< Similar to calling "rangeImage.setSize(H,W)" but this method provides memory pooling to speed-up the memory allocation.

//		// Range Matrix external storage functions ---------
//		inline bool rangeImage_isExternallyStored() const { return m_rangeImage_external_stored; }
//		inline std::string rangeImage_getExternalStorageFile() const { return m_rangeImage_external_file; }
//		void rangeImage_getExternalStorageFileAbsolutePath(std::string &out_path) const;
//		inline std::string rangeImage_getExternalStorageFileAbsolutePath() const {
//				std::string tmp;
//				rangeImage_getExternalStorageFileAbsolutePath(tmp);
//				return tmp;
//		}
//		void rangeImage_convertToExternalStorage( const std::string &fileName, const std::string &use_this_base_dir ); //!< Users won't normally want to call this, it's only used from internal MRPT programs.
//		/** Forces marking this observation as non-externally stored - it doesn't anything else apart from reseting the corresponding flag (Users won't normally want to call this, it's only used from internal MRPT programs) */
//		void rangeImage_forceResetExternalStorage() { m_rangeImage_external_stored=false; }
//		// ---------
//
//		/** Enum type for intensityImageChannel */
//		enum TIntensityChannelID
//		{
//			CH_VISIBLE = 0, //!< Grayscale or RGB visible channel of the camera sensor.
//			CH_IR      = 1  //!< Infrarred (IR) channel
//		};
//
		bool hasIntensityImage;                    //!< true means the field intensityImage contains valid data
		mrpt::utils::CImage intensityImages[NUM_SENSORS];        //!< If hasIntensityImage=true, a color or gray-level intensity image of the same size than "rangeImage"
//
//		bool hasConfidenceImage; 			//!< true means the field confidenceImage contains valid data
//		mrpt::utils::CImage confidenceImage;  //!< If hasConfidenceImage=true, an image with the "confidence" value [range 0-255] as estimated by the capture drivers.
//
		mrpt::utils::TCamera sensorParamss[NUM_SENSORS];	//!< Projection parameters of the 8 RGBD sensor.


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
//
//		void swap(CObservationRGBD360 &o);	//!< Very efficient method to swap the contents of two observations.
//
//		void getZoneAsObs( CObservationRGBD360 &obs, const unsigned int &r1, const unsigned int &r2, const unsigned int &c1, const unsigned int &c2 );
//
//		/** A Levenberg-Marquart-based optimizer to recover the calibration parameters of a 3D camera given a range (depth) image and the corresponding 3D point cloud.
//		  * \param camera_offset The offset (in meters) in the +X direction of the point cloud. It's 1cm for SwissRanger SR4000.
//		  * \return The final average reprojection error per pixel (typ <0.05 px)
//		  */
//		static double recoverCameraCalibrationParameters(
//			const CObservationRGBD360	&in_obs,
//			mrpt::utils::TCamera			&out_camParams,
//			const double camera_offset = 0.01 );
//
//		/** Look-up-table struct for project3DPointsFromDepthImageInto() */
//		struct TCached3DProjTables
//		{
//			mrpt::vector_float Kzs,Kys;
//			TCamera  prev_camParams;
//		};
//		static TCached3DProjTables m_3dproj_lut; //!< 3D point cloud projection look-up-table \sa project3DPointsFromDepthImage

	}; // End of class def.


	} // End of namespace

//	namespace utils
//	{
//		using namespace ::mrpt::slam;
//		// Specialization must occur in the same namespace
//		MRPT_DECLARE_TTYPENAME_PTR(CObservationRGBD360)
//
//		// Enum <-> string converter:
//		template <>
//		struct TEnumTypeFiller<slam::CObservationRGBD360::TIntensityChannelID>
//		{
//			typedef slam::CObservationRGBD360::TIntensityChannelID enum_t;
//			static void fill(bimap<enum_t,std::string>  &m_map)
//			{
//				m_map.insert(slam::CObservationRGBD360::CH_VISIBLE, "CH_VISIBLE");
//				m_map.insert(slam::CObservationRGBD360::CH_IR, "CH_IR");
//			}
//		};
//	}

//	namespace utils
//	{
//		using mrpt::slam::CObservationRGBD360;
//
//		/** Specialization mrpt::utils::PointCloudAdapter<CObservationRGBD360> \ingroup mrpt_adapters_grp */
//		template <>
//		class PointCloudAdapter<CObservationRGBD360> : public detail::PointCloudAdapterHelperNoRGB<CObservationRGBD360,float>
//		{
//		private:
//			CObservationRGBD360 &m_obj;
//		public:
//			typedef float  coords_t;  //!< The type of each point XYZ coordinates
//			static const int HAS_RGB   = 0;  //!< Has any color RGB info?
//			static const int HAS_RGBf  = 0;  //!< Has native RGB info (as floats)?
//			static const int HAS_RGBu8 = 0;  //!< Has native RGB info (as uint8_t)?
//
//			/** Constructor (accept a const ref for convenience) */
//			inline PointCloudAdapter(const CObservationRGBD360 &obj) : m_obj(*const_cast<CObservationRGBD360*>(&obj)) { }
//			/** Get number of points */
//			inline size_t size() const { return m_obj.points3D_x.size(); }
//			/** Set number of points (to uninitialized values) */
//			inline void resize(const size_t N) {
//				if (N) m_obj.hasPoints3D = true;
//				m_obj.resizePoints3DVectors(N);
//			}
//
//			/** Get XYZ coordinates of i'th point */
//			template <typename T>
//			inline void getPointXYZ(const size_t idx, T &x,T &y, T &z) const {
//				x=m_obj.points3D_x[idx];
//				y=m_obj.points3D_y[idx];
//				z=m_obj.points3D_z[idx];
//			}
//			/** Set XYZ coordinates of i'th point */
//			inline void setPointXYZ(const size_t idx, const coords_t x,const coords_t y, const coords_t z) {
//				m_obj.points3D_x[idx]=x;
//				m_obj.points3D_y[idx]=y;
//				m_obj.points3D_z[idx]=z;
//			}
//		}; // end of PointCloudAdapter<CObservationRGBD360>
//	}
} // End of namespace

//#include "CObservationRGBD360_project3D_impl.h"

#endif
