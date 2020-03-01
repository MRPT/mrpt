/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/aligned_std_vector.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/T3DPointsProjectionParams.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TPixelLabelInfo.h>
#include <mrpt/obs/TRangeImageFilter.h>
#include <mrpt/opengl/pointcloud_adapters.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/typemeta/TEnumType.h>
#include <optional>

namespace mrpt::obs
{
namespace detail
{
// Implemented in CObservation3DRangeScan_project3D_impl.h
template <class POINTMAP>
void unprojectInto(
	mrpt::obs::CObservation3DRangeScan& src_obs, POINTMAP& dest_pointcloud,
	const mrpt::obs::T3DPointsProjectionParams& projectParams,
	const mrpt::obs::TRangeImageFilterParams& filterParams);
}  // namespace detail

/** A range or depth 3D scan measurement, as from a time-of-flight range camera
 *or a structured-light depth RGBD sensor.
 *
 * This kind of observations can carry one or more of these data fields:
 *    - 3D point cloud (as float's).
 *    - Each 3D point has its associated (u,v) pixel coordinates in \a
 *points3D_idxs_x & \a points3D_idxs_y (New in MRPT 1.4.0)
 *    - 2D range image (as a matrix): Each entry in the matrix
 *"rangeImage(ROW,COLUMN)" contains a distance or a depth, depending
 *on \a range_is_depth. Ranges are stored as uint16_t for efficiency. The units
 *of ranges are stored separately in rangeUnits.
 *    - 2D intensity (grayscale or RGB) image (as a mrpt::img::CImage): For
 *SwissRanger cameras, a logarithmic A-law compression is used to convert the
 *original 16bit intensity to a more standard 8bit graylevel.
 *    - 2D confidence image (as a mrpt::img::CImage): For each pixel, a 0x00
 *and a 0xFF mean the lowest and highest confidence levels, respectively.
 *    - Semantic labels: Stored as a matrix of bitfields, each bit having a
 *user-defined meaning.
 *
 *  The coordinates of the 3D point cloud are in meters with respect to the
 *depth camera origin of coordinates
 *    (in SwissRanger, the front face of the camera: a small offset ~1cm in
 *front of the physical focal point),
 *    with the +X axis pointing forward, +Y pointing left-hand and +Z pointing
 *up. By convention, a 3D point with its coordinates set to (0,0,0), will be
 *considered as invalid.
 *  The field CObservation3DRangeScan::relativePoseIntensityWRTDepth describes
 *the change of coordinates from
 *    the depth camera to the intensity (RGB or grayscale) camera. In a
 *SwissRanger camera both cameras coincide,
 *    so this pose is just a rotation (0,0,0,-90deg,0,-90deg). But in
 *    Microsoft Kinect there is also an offset, as shown in this figure:
 *
 *  <div align=center>
 *   <img src="CObservation3DRangeScan_figRefSystem.png">
 *  </div>
 *
 *  In any case, check the field \a relativePoseIntensityWRTDepth, or the method
 *\a doDepthAndIntensityCamerasCoincide()
 *    to determine if both frames of reference coincide, since even for Kinect
 *cameras both can coincide if the images
 *    have been rectified.
 *
 *  The 2D images and matrices are stored as common images, with an up->down
 *rows order and left->right, as usual.
 *   Optionally, the intensity and confidence channels can be set to
 *delayed-load images for off-rawlog storage so it saves
 *   memory by having loaded in memory just the needed images. See the methods
 *load() and unload().
 *  Due to the intensive storage requirements of this kind of observations, this
 *observation is the only one in MRPT
 *   for which it's recommended to always call "load()" and "unload()" before
 *and after using the observation, *ONLY* when
 *   the observation was read from a rawlog dataset, in order to make sure that
 *all the externally stored data fields are
 *   loaded and ready in memory.
 *
 *  Classes that grab observations of this type are:
 *		- mrpt::hwdrivers::CSwissRanger3DCamera
 *		- mrpt::hwdrivers::CKinect
 *		- mrpt::hwdrivers::COpenNI2Sensor
 *
 *  There are two sets of calibration parameters (see
 *mrpt::vision::checkerBoardStereoCalibration() or the ready-to-use GUI program
 *<a href="http://www.mrpt.org/Application:kinect-calibrate"
 *>kinect-calibrate</a>):
 *		- cameraParams: Projection parameters of the depth camera.
 *		- cameraParamsIntensity: Projection parameters of the intensity
 *(gray-level or RGB) camera.
 *
 *  In some cameras, like SwissRanger, both are the same. It is possible in
 *Kinect to rectify the range images such both cameras
 *   seem to coincide and then both sets of camera parameters will be identical.
 *
 *  Range data can be interpreted in two different ways depending on the 3D
 *camera (this field is already set to the
 *    correct setting when grabbing observations from an mrpt::hwdrivers
 *sensor):
 *		- range_is_depth=true  -> Kinect-like ranges: entries of \a rangeImage
 *are
 *distances along the +X axis
 *		- range_is_depth=false -> Ranges in \a rangeImage are actual distances
 *in
 *3D.
 *
 *  The "intensity" channel may come from different channels in sesnsors as
 *Kinect. Look at field \a intensityImageChannel to
 *    find out if the image was grabbed from the visible (RGB) or IR channels.
 *
 * 3D point clouds can be generated at any moment after grabbing with
 * CObservation3DRangeScan::unprojectInto(), provided the
 *correct
 *   calibration parameters. Note that unprojectInto() will
 *store the point cloud in sensor-centric local coordinates. Use
 *unprojectInto() to directly obtain vehicle or world
 *coordinates.
 *
 *  Example of how to assign labels to pixels (for object segmentation, semantic
 *information, etc.):
 *
 * \code
 *   // Assume obs of type CObservation3DRangeScan::Ptr
 *   obs->pixelLabels =TPixelLabelInfo::Ptr( new
 *CObservation3DRangeScan::TPixelLabelInfo<NUM_BYTES>() );
 *   obs->pixelLabels->setSize(ROWS,COLS);
 *   obs->pixelLabels->setLabel(col,row, label_idx);   // label_idxs =
 *[0,2^NUM_BYTES-1]
 *   //...
 * \endcode
 *
 *  \note Starting at serialization version 2 (MRPT 0.9.1+), the confidence
 *channel is stored as an image instead of a matrix to optimize memory and disk
 *space.
 *  \note Starting at serialization version 3 (MRPT 0.9.1+), the 3D point cloud
 *and the rangeImage can both be stored externally to save rawlog space.
 *  \note Starting at serialization version 5 (MRPT 0.9.5+), the new field \a
 *range_is_depth
 *  \note Starting at serialization version 6 (MRPT 0.9.5+), the new field \a
 *intensityImageChannel
 *  \note Starting at serialization version 7 (MRPT 1.3.1+), new fields for
 *semantic labeling
 *  \note Since MRPT 1.5.0, external files format can be selected at runtime
 *with `CObservation3DRangeScan::EXTERNALS_AS_TEXT`
 *
 * \sa mrpt::hwdrivers::CSwissRanger3DCamera, mrpt::hwdrivers::CKinect,
 *CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservation3DRangeScan : public CObservation
{
	DEFINE_SERIALIZABLE(CObservation3DRangeScan, mrpt::obs)

   protected:
	/** If set to true, m_points3D_external_file is valid. */
	bool m_points3D_external_stored{false};
	/** 3D points are in CImage::getImagesPathBase()+<this_file_name> */
	std::string m_points3D_external_file;

	/** If set to true, m_rangeImage_external_file is valid. */
	bool m_rangeImage_external_stored{false};
	/** rangeImage is in CImage::getImagesPathBase()+<this_file_name> */
	std::string m_rangeImage_external_file;

   public:
	/** Default constructor */
	CObservation3DRangeScan();
	/** Destructor */
	~CObservation3DRangeScan() override;

	/** @name Delayed-load manual control methods.
		@{ */
	/** Makes sure all images and other fields which may be externally stored
	 * are loaded in memory.
	 *  Note that for all CImages, calling load() is not required since the
	 * images will be automatically loaded upon first access, so load()
	 * shouldn't be needed to be called in normal cases by the user.
	 *  If all the data were alredy loaded or this object has no externally
	 * stored data fields, calling this method has no effects.
	 * \sa unload
	 */
	void load() const override;
	/** Unload all images, for the case they being delayed-load images stored in
	 * external files (othewise, has no effect).
	 * \sa load
	 */
	void unload() override;
	/** @} */

	/** Unprojects the RGB+D image pair into a 3D point cloud (with color if the
	 * target map supports it) and optionally at a given 3D pose. The 3D point
	 * coordinates are computed from the depth image (\a rangeImage) and the
	 * depth camera camera parameters (\a cameraParams). There exist two set of
	 * formulas for projecting the i'th point, depending on the value of
	 * "range_is_depth". In all formulas below, "rangeImage" is the matrix of
	 * ranges and the pixel coordinates are (r,c).
	 *
	 *  1) [range_is_depth=true] With "range equals depth" or "Kinect-like
	 * depth mode": the range values
	 *      are in fact distances along the "+X" axis, not real 3D ranges (this
	 * is the way Kinect reports ranges):
	 *
	 * \code
	 *   x(i) = rangeImage(r,c) * rangeUnits
	 *   y(i) = (r_cx - c) * x(i) / r_fx
	 *   z(i) = (r_cy - r) * x(i) / r_fy
	 * \endcode
	 *
	 *
	 *  2) [range_is_depth=false] With "normal ranges": range means distance in
	 * 3D. This must be set when
	 *      processing data from the SwissRange 3D camera, among others.
	 *
	 * \code
	 *   Ky = (r_cx - c)/r_fx
	 *   Kz = (r_cy - r)/r_fy
	 *
	 *   x(i) = rangeImage(r,c) * rangeUnits / sqrt( 1 + Ky^2 + Kz^2 )
	 *   y(i) = Ky * x(i)
	 *   z(i) = Kz * x(i)
	 * \endcode
	 *
	 *  The color of each point is determined by projecting the 3D local point
	 * into the RGB image using \a cameraParamsIntensity.
	 *
	 *  By default the local (sensor-centric) coordinates of points are
	 * directly stored into the local map, but if indicated so in \a
	 * takeIntoAccountSensorPoseOnRobot
	 *  the points are transformed with \a sensorPose. Furthermore, if
	 * provided, those coordinates are transformed with \a robotPoseInTheWorld
	 *
	 * \tparam POINTMAP Supported maps are all those covered by
	 * mrpt::opengl::PointCloudAdapter (mrpt::maps::CPointsMap and derived,
	 * mrpt::opengl::CPointCloudColoured, PCL point clouds,...)
	 */
	template <class POINTMAP>
	inline void unprojectInto(
		POINTMAP& dest_pointcloud,
		const T3DPointsProjectionParams& projectParams,
		const TRangeImageFilterParams& filterParams = TRangeImageFilterParams())
	{
		detail::unprojectInto<POINTMAP>(
			*this, dest_pointcloud, projectParams, filterParams);
	}

	/** Convert this 3D observation into an "equivalent 2D fake laser scan",
	 * with a configurable vertical FOV.
	 *
	 *  The result is a 2D laser scan with more "rays" (N) than columns has the
	 * 3D observation (W), exactly: N = W * oversampling_ratio.
	 *  This oversampling is required since laser scans sample the space at
	 * evenly-separated angles, while
	 *  a range camera follows a tangent-like distribution. By oversampling we
	 * make sure we don't leave "gaps" unseen by the virtual "2D laser".
	 *
	 *  All obstacles within a frustum are considered and the minimum distance
	 * is kept in each direction.
	 *  The horizontal FOV of the frustum is automatically computed from the
	 * intrinsic parameters, but the
	 *  vertical FOV must be provided by the user, and can be set to be
	 * assymetric which may be useful
	 *  depending on the zone of interest where to look for obstacles.
	 *
	 *  All spatial transformations are riguorosly taken into account in this
	 * class, using the depth camera
	 *  intrinsic calibration parameters.
	 *
	 *  The timestamp of the new object is copied from the 3D object.
	 *  Obviously, a requisite for calling this method is the 3D observation
	 * having range data,
	 *  i.e. hasRangeImage must be true. It's not needed to have RGB data nor
	 * the raw 3D point clouds
	 *  for this method to work.
	 *
	 *  If `scanParams.use_origin_sensor_pose` is `true`, the points will be
	 * projected to 3D and then reprojected
	 *  as seen from a different sensorPose at the vehicle frame origin.
	 * Otherwise (the default), the output 2D observation will share the
	 * sensorPose of the input 3D scan
	 *  (using a more efficient algorithm that avoids trigonometric functions).
	 *
	 *  \param[out] out_scan2d The resulting 2D equivalent scan.
	 *
	 * \sa The example in
	 * https://www.mrpt.org/tutorials/mrpt-examples/example-kinect-to-2d-laser-demo/
	 */
	void convertTo2DScan(
		mrpt::obs::CObservation2DRangeScan& out_scan2d,
		const T3DPointsTo2DScanParams& scanParams,
		const TRangeImageFilterParams& filterParams =
			TRangeImageFilterParams());

	/** Whether external files (3D points, range and confidence) are to be
	 * saved as `.txt` text files (MATLAB compatible) or `*.bin` binary
	 *(faster).
	 * Loading always will determine the type by inspecting the file extension.
	 * \note Default=false
	 **/
	static void EXTERNALS_AS_TEXT(bool value);
	static bool EXTERNALS_AS_TEXT();

	/** \name Point cloud
	 * @{ */
	/** true means the field points3D contains valid data. */
	bool hasPoints3D{false};
	/** If hasPoints3D=true, the (X,Y,Z) coordinates of the 3D point cloud
	 * detected by the camera. \sa resizePoints3DVectors */
	std::vector<float> points3D_x, points3D_y, points3D_z;
	/** If hasPoints3D=true, the (x,y) pixel coordinates for each (X,Y,Z) point
	 * in \a points3D_x, points3D_y, points3D_z */
	std::vector<uint16_t> points3D_idxs_x, points3D_idxs_y;  //!<

	/** Use this method instead of resizing all three \a points3D_x, \a
	 * points3D_y & \a points3D_z to allow the usage of the internal memory
	 * pool. */
	void resizePoints3DVectors(const size_t nPoints);

	/** Get the size of the scan pointcloud. \note Method is added for
	 * compatibility with its CObservation2DRangeScan counterpart */
	size_t getScanSize() const;
	/** @} */

	/** \name Point cloud external storage functions
	 * @{ */
	inline bool points3D_isExternallyStored() const
	{
		return m_points3D_external_stored;
	}
	inline std::string points3D_getExternalStorageFile() const
	{
		return m_points3D_external_file;
	}
	void points3D_getExternalStorageFileAbsolutePath(
		std::string& out_path) const;
	inline std::string points3D_getExternalStorageFileAbsolutePath() const
	{
		std::string tmp;
		points3D_getExternalStorageFileAbsolutePath(tmp);
		return tmp;
	}
	/** Users won't normally want to call this, it's only used from internal
	 * MRPT programs. \sa EXTERNALS_AS_TEXT */
	void points3D_convertToExternalStorage(
		const std::string& fileName, const std::string& use_this_base_dir);
	/** Users normally won't need to use this */
	inline void points3D_overrideExternalStoredFlag(bool isExternal)
	{
		m_points3D_external_stored = isExternal;
	}
	/** @} */

	/** \name Range (depth) image
	 * @{ */
	/** true means the field rangeImage contains valid data */
	bool hasRangeImage{false};

	/** If hasRangeImage=true, a matrix of floats with the range data as
	 * captured by the camera (in meters) \sa range_is_depth, rangeUnits */
	mrpt::math::CMatrix_u16 rangeImage;

	/** The conversion factor from integer units in rangeImage and actual
	 * distances in meters. Default is 0.001 m, that is 1 millimeter. \sa
	 * rangeImage */
	float rangeUnits = 0.001f;

	/** true: Kinect-like ranges: entries of \a rangeImage are distances
	 * along the +X axis; false: Ranges in \a rangeImage are actual
	 * distances in 3D.
	 */
	bool range_is_depth{true};

	/** Similar to calling "rangeImage.setSize(H,W)" but this method provides
	 * memory pooling to speed-up the memory allocation. */
	void rangeImage_setSize(const int HEIGHT, const int WIDTH);

	/** Builds a visualization from the rangeImage.
	 * The image is built with the given color map (default: grayscale) and such
	 * that the colormap range is mapped to ranges 0 meters to the field
	 * "maxRange" in this object, unless overriden with the optional parameters.
	 * Note that the usage of optional<> allows any parameter to be left to its
	 * default placing `std::nullopt`.
	 */
	mrpt::img::CImage rangeImage_getAsImage(
		const std::optional<mrpt::img::TColormap> color = std::nullopt,
		const std::optional<float> normMinRange = std::nullopt,
		const std::optional<float> normMaxRange = std::nullopt) const;

	/** @} */

	/** \name Range Matrix external storage functions
	 * @{ */
	inline bool rangeImage_isExternallyStored() const
	{
		return m_rangeImage_external_stored;
	}
	inline std::string rangeImage_getExternalStorageFile() const
	{
		return m_rangeImage_external_file;
	}
	void rangeImage_getExternalStorageFileAbsolutePath(
		std::string& out_path) const;
	inline std::string rangeImage_getExternalStorageFileAbsolutePath() const
	{
		std::string tmp;
		rangeImage_getExternalStorageFileAbsolutePath(tmp);
		return tmp;
	}
	/** Users won't normally want to call this, it's only used from internal
	 * MRPT programs. \sa EXTERNALS_AS_TEXT */
	void rangeImage_convertToExternalStorage(
		const std::string& fileName, const std::string& use_this_base_dir);
	/** Forces marking this observation as non-externally stored - it doesn't
	 * anything else apart from reseting the corresponding flag (Users won't
	 * normally want to call this, it's only used from internal MRPT programs)
	 */
	void rangeImage_forceResetExternalStorage()
	{
		m_rangeImage_external_stored = false;
	}
	/** @} */

	/** \name Intensity (RGB) channels
	 * @{ */
	/** Enum type for intensityImageChannel */
	enum TIntensityChannelID
	{
		/** Grayscale or RGB visible channel of the camera sensor. */
		CH_VISIBLE = 0,
		/** Infrarred (IR) channel */
		CH_IR = 1
	};

	/** true means the field intensityImage contains valid data */
	bool hasIntensityImage{false};

	/** If hasIntensityImage=true, a color or gray-level intensity image of the
	 * same size than "rangeImage" */
	mrpt::img::CImage intensityImage;

	/** The source of the intensityImage; typically the visible channel \sa
	 * TIntensityChannelID */
	TIntensityChannelID intensityImageChannel{CH_VISIBLE};
	/** @} */

	/** \name Confidence "channel"
	 * @{ */
	/** true means the field confidenceImage contains valid data */
	bool hasConfidenceImage{false};
	/** If hasConfidenceImage=true, an image with the "confidence" value [range
	 * 0-255] as estimated by the capture drivers. */
	mrpt::img::CImage confidenceImage;
	/** @} */

	/** \name Pixel-wise classification labels (for semantic labeling, etc.)
	 * @{ */
	/** Returns true if the field CObservation3DRangeScan::pixelLabels contains
	 * a non-NULL smart pointer.
	 * To enhance a 3D point cloud with labeling info, just assign an
	 * appropiate object to \a pixelLabels
	 */
	bool hasPixelLabels() const { return pixelLabels ? true : false; }

	/** All information about pixel labeling is stored in this (smart pointer
	 * to) structure; refer to TPixelLabelInfo for details on the contents
	 * User is responsible of creating a new object of the desired data type.
	 * It will be automatically (de)serialized no matter its specific type. */
	TPixelLabelInfoBase::Ptr pixelLabels;

	/** @} */

	/** \name Sensor parameters
	 * @{ */

	/** Projection parameters of the depth camera. */
	mrpt::img::TCamera cameraParams;
	/** Projection parameters of the intensity (graylevel or RGB) camera. */
	mrpt::img::TCamera cameraParamsIntensity;

	/** Relative pose of the intensity camera wrt the depth camera (which is the
	 * coordinates origin for this observation).
	 *  In a SwissRanger camera, this will be (0,0,0,-90deg,0,-90deg) since
	 * both cameras coincide.
	 *  In a Kinect, this will include a small lateral displacement and a
	 * rotation, according to the drawing on the top of this page.
	 *  \sa doDepthAndIntensityCamerasCoincide
	 */
	mrpt::poses::CPose3D relativePoseIntensityWRTDepth;

	/** Return true if \a relativePoseIntensityWRTDepth equals the pure rotation
	 * (0,0,0,-90deg,0,-90deg) (with a small comparison epsilon)
	 * \sa relativePoseIntensityWRTDepth
	 */
	bool doDepthAndIntensityCamerasCoincide() const;

	/** The maximum range allowed by the device, in meters (e.g. 8.0m, 5.0m,...)
	 */
	float maxRange{5.0f};
	/** The 6D pose of the sensor on the robot. */
	mrpt::poses::CPose3D sensorPose;
	/** The "sigma" error of the device in meters, used while inserting the scan
	 * in an occupancy grid. */
	float stdError{0.01f};

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = sensorPose;
	}
	// See base class docs
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override
	{
		sensorPose = newSensorPose;
	}

	/** @} */  // end sensor params

	/** Removes the distortion in both, depth and intensity images. Intrinsics
	 * (fx,fy,cx,cy) remains the same for each image after undistortion.
	 */
	void undistort();

	// See base class docs
	void getDescriptionAsText(std::ostream& o) const override;

	/** Very efficient method to swap the contents of two observations. */
	void swap(CObservation3DRangeScan& o);
	/** Extract a ROI of the 3D observation as a new one. \note PixelLabels are
	 * *not* copied to the output subimage. */
	void getZoneAsObs(
		CObservation3DRangeScan& obs, const unsigned int& r1,
		const unsigned int& r2, const unsigned int& c1, const unsigned int& c2);

	/** A Levenberg-Marquart-based optimizer to recover the calibration
	 * parameters of a 3D camera given a range (depth) image and the
	 * corresponding 3D point cloud.
	 * \param camera_offset The offset (in meters) in the +X direction of the
	 * point cloud. It's 1cm for SwissRanger SR4000.
	 * \return The final average reprojection error per pixel (typ <0.05 px)
	 */
	static double recoverCameraCalibrationParameters(
		const CObservation3DRangeScan& in_obs,
		mrpt::img::TCamera& out_camParams, const double camera_offset = 0.01);

	/** Look-up-table struct for unprojectInto() */
	struct unproject_LUT_t
	{
		// x,y,z: +X pointing forward (depth), +z up
		mrpt::aligned_std_vector<float> Kxs, Kys, Kzs;
	};

	/** Gets (or generates upon first request) the 3D point cloud projection
	 * look-up-table for the current depth camera intrinsics & distortion
	 * parameters.
	 * Returns a const reference to a global variable. Multithread safe.
	 * \sa unprojectInto */
	const unproject_LUT_t& get_unproj_lut() const;

};  // End of class def.
}  // namespace mrpt::obs

namespace mrpt::opengl
{
/** Specialization mrpt::opengl::PointCloudAdapter<CObservation3DRangeScan>
 * \ingroup mrpt_adapters_grp */
template <>
class PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>
{
   private:
	mrpt::obs::CObservation3DRangeScan& m_obj;

   public:
	/** The type of each point XYZ coordinates */
	using coords_t = float;
	/** Has any color RGB info? */
	static constexpr bool HAS_RGB = false;
	/** Has native RGB info (as floats)? */
	static constexpr bool HAS_RGBf = false;
	/** Has native RGB info (as uint8_t)? */
	static constexpr bool HAS_RGBu8 = false;

	/** Constructor (accept a const ref for convenience) */
	inline PointCloudAdapter(const mrpt::obs::CObservation3DRangeScan& obj)
		: m_obj(*const_cast<mrpt::obs::CObservation3DRangeScan*>(&obj))
	{
	}
	/** Get number of points */
	inline size_t size() const { return m_obj.points3D_x.size(); }
	/** Set number of points (to uninitialized values) */
	inline void resize(const size_t N)
	{
		if (N) m_obj.hasPoints3D = true;
		m_obj.resizePoints3DVectors(N);
	}
	/** Does nothing as of now */
	inline void setDimensions(size_t height, size_t width) {}
	/** Get XYZ coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ(const size_t idx, T& x, T& y, T& z) const
	{
		x = m_obj.points3D_x[idx];
		y = m_obj.points3D_y[idx];
		z = m_obj.points3D_z[idx];
	}
	/** Set XYZ coordinates of i'th point */
	inline void setPointXYZ(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z)
	{
		m_obj.points3D_x[idx] = x;
		m_obj.points3D_y[idx] = y;
		m_obj.points3D_z[idx] = z;
	}
	/** Set XYZ coordinates of i'th point */
	inline void setInvalidPoint(const size_t idx)
	{
		THROW_EXCEPTION(
			"mrpt::obs::CObservation3DRangeScan requires needs to be dense");
	}

};  // end of PointCloudAdapter<CObservation3DRangeScan>
}  // namespace mrpt::opengl
MRPT_ENUM_TYPE_BEGIN(mrpt::obs::CObservation3DRangeScan::TIntensityChannelID)
MRPT_FILL_ENUM_MEMBER(mrpt::obs::CObservation3DRangeScan, CH_VISIBLE);
MRPT_FILL_ENUM_MEMBER(mrpt::obs::CObservation3DRangeScan, CH_IR);
MRPT_ENUM_TYPE_END()

#include "CObservation3DRangeScan_project3D_impl.h"
