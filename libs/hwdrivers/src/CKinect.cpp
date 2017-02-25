/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/TStereoCamera.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::synch;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CKinect,mrpt::hwdrivers)

// Whether to profile memory allocations:
//#define KINECT_PROFILE_MEM_ALLOC

#if MRPT_HAS_KINECT_FREENECT
#	include <libfreenect.h>
#else
#	define KINECT_W 640
#	define KINECT_H 480
#endif

#if MRPT_HAS_KINECT_FREENECT
	// Macros to convert the opaque pointers in the class header:
	#define f_ctx  reinterpret_cast<freenect_context*>(m_f_ctx)
	#define f_ctx_ptr  reinterpret_cast<freenect_context**>(&m_f_ctx)
	#define f_dev  reinterpret_cast<freenect_device*>(m_f_dev)
	#define f_dev_ptr  reinterpret_cast<freenect_device**>(&m_f_dev)
#endif // MRPT_HAS_KINECT_FREENECT


#ifdef KINECT_PROFILE_MEM_ALLOC
mrpt::utils::CTimeLogger alloc_tim;
#endif
//int int a;

void CKinect::calculate_range2meters()
{
#ifdef MRPT_KINECT_DEPTH_10BIT
	const float k1 = 1.1863f;
	const float k2 = 2842.5f;
	const float k3 = 0.1236f;

	for (size_t i=0; i<KINECT_RANGES_TABLE_LEN; i++)
			m_range2meters[i] = k3 * tanf(i/k2 + k1);

#else
	for (size_t i=0; i<KINECT_RANGES_TABLE_LEN; i++)
		m_range2meters[i] = 1.0f / (i * (-0.0030711016) + 3.3309495161);
#endif

	// Minimum/Maximum range means error:
	m_range2meters[0] = 0;
	m_range2meters[KINECT_RANGES_TABLE_LEN-1] = 0;
}

/*-------------------------------------------------------------
		ctor
 -------------------------------------------------------------*/
CKinect::CKinect()  :
	m_sensorPoseOnRobot(),
	m_preview_window(false),
	m_preview_window_decimation(1),
	m_preview_decim_counter_range(0),
	m_preview_decim_counter_rgb(0),

#if MRPT_HAS_KINECT_FREENECT
	m_f_ctx(NULL), // The "freenect_context", or NULL if closed
	m_f_dev(NULL), // The "freenect_device", or NULL if closed
	m_tim_latest_depth(0),
	m_tim_latest_rgb(0),
	m_latest_obs_cs("m_latest_obs_cs"),
#endif
	m_relativePoseIntensityWRTDepth(0,-0.02,0, DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-90)),
	m_initial_tilt_angle(360),
	m_user_device_number(0),
	m_grab_image(true),
	m_grab_depth(true),
	m_grab_3D_points(true),
	m_grab_IMU(true),
	m_video_channel(VIDEO_CHANNEL_RGB)
{
	calculate_range2meters();

	// Get maximum range:
	m_maxRange=m_range2meters[KINECT_RANGES_TABLE_LEN-2];  // Recall: r[Max-1] means error.

	// Default label:
	m_sensorLabel = "KINECT";

	// =========== Default params ===========
	// ----- RGB -----
	m_cameraParamsRGB.ncols = 640; // By default set 640x480, on connect we'll update this.
	m_cameraParamsRGB.nrows = 480;

	m_cameraParamsRGB.cx(328.94272028759258);
	m_cameraParamsRGB.cy(267.48068171871557);
	m_cameraParamsRGB.fx(529.2151);
	m_cameraParamsRGB.fy(525.5639);

	m_cameraParamsRGB.dist.zeros();

	// ----- Depth -----
	m_cameraParamsDepth.ncols = 640;
	m_cameraParamsDepth.nrows = 488;

	m_cameraParamsDepth.cx(339.30781);
	m_cameraParamsDepth.cy(242.7391);
	m_cameraParamsDepth.fx(594.21434);
	m_cameraParamsDepth.fy(591.04054);

	m_cameraParamsDepth.dist.zeros();

#if !MRPT_HAS_KINECT
	THROW_EXCEPTION("MRPT was compiled without support for Kinect. Please, rebuild it.")
#endif
}

/*-------------------------------------------------------------
			dtor
 -------------------------------------------------------------*/
CKinect::~CKinect()
{
	this->close();
}

/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void CKinect::initialize()
{
	open();
}

/** This method will be invoked at a minimum rate of "process_rate" (Hz)
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void CKinect::doProcess()
{
	bool	thereIs, hwError;

	CObservation3DRangeScanPtr newObs     = CObservation3DRangeScan::Create();
	CObservationIMUPtr         newObs_imu = CObservationIMU::Create();

	getNextObservation( *newObs, *newObs_imu, thereIs, hwError );

	if (hwError)
	{
		m_state = ssError;
	    THROW_EXCEPTION("Couldn't communicate to the Kinect sensor!");
	}

	if (thereIs)
	{
		m_state = ssWorking;

		vector<CSerializablePtr> objs;
		if (m_grab_image || m_grab_depth || m_grab_3D_points)  objs.push_back(newObs);
		if (m_grab_IMU)  objs.push_back(newObs_imu);

		appendObservations( objs );
	}
}

/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
*  \exception This method must throw an exception with a descriptive message if some critical parameter is missing or has an invalid value.
*/
void  CKinect::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
	m_sensorPoseOnRobot.setFromValues(
		configSource.read_float(iniSection,"pose_x",0),
		configSource.read_float(iniSection,"pose_y",0),
		configSource.read_float(iniSection,"pose_z",0),
		DEG2RAD( configSource.read_float(iniSection,"pose_yaw",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_pitch",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_roll",0) )
		);

	m_preview_window = configSource.read_bool(iniSection,"preview_window",m_preview_window);

	// "Stereo" calibration data:
	// [<SECTION>_LEFT]  // Depth
	//   ...
	// [<SECTION>_RIGHT] // RGB
	//   ...
	// [<SECTION>_LEFT2RIGHT_POSE]
	//  pose_quaternion = [x y z qr qx qy qz]

	const mrpt::poses::CPose3D twist(0,0,0,DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-90));

	mrpt::utils::TStereoCamera  sc;
	sc.leftCamera  = m_cameraParamsDepth;  // Load default values so that if we fail to load from cfg at least we have some reasonable numbers.
	sc.rightCamera = m_cameraParamsRGB;
	sc.rightCameraPose = mrpt::poses::CPose3DQuat(m_relativePoseIntensityWRTDepth - twist);

	try {
		sc.loadFromConfigFile(iniSection,configSource);
	} catch (std::exception &e) {
		std::cout << "[CKinect::loadConfig_sensorSpecific] Warning: Ignoring error loading calibration parameters:\n" << e.what();
	}
	m_cameraParamsDepth = sc.leftCamera;
	m_cameraParamsRGB   = sc.rightCamera;
	m_relativePoseIntensityWRTDepth = twist + mrpt::poses::CPose3D(sc.rightCameraPose);

	// Id:
	m_user_device_number = configSource.read_int(iniSection,"device_number",m_user_device_number );

	m_grab_image = configSource.read_bool(iniSection,"grab_image",m_grab_image);
	m_grab_depth = configSource.read_bool(iniSection,"grab_depth",m_grab_depth);
	m_grab_3D_points = configSource.read_bool(iniSection,"grab_3D_points",m_grab_3D_points);
	m_grab_IMU = configSource.read_bool(iniSection,"grab_IMU",m_grab_IMU );

	m_video_channel = configSource.read_enum<TVideoChannel>(iniSection,"video_channel",m_video_channel);

	{
		std::string s = configSource.read_string(iniSection,"relativePoseIntensityWRTDepth","");
		if (!s.empty())
			m_relativePoseIntensityWRTDepth.fromString(s);
	}

	m_initial_tilt_angle = configSource.read_int(iniSection,"initial_tilt_angle",m_initial_tilt_angle);
}

bool CKinect::isOpen() const
{
#if MRPT_HAS_KINECT_FREENECT
	return f_dev != NULL;
#else
	return false;
#endif

#
}


#if MRPT_HAS_KINECT_FREENECT
// ========  GLOBAL CALLBACK FUNCTIONS ========
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	const freenect_frame_mode frMode = freenect_get_current_video_mode(dev);

	uint16_t *depth = reinterpret_cast<uint16_t *>(v_depth);

	CKinect *obj = reinterpret_cast<CKinect*>(freenect_get_user(dev));

	// Update of the timestamps at the end:
	mrpt::synch::CCriticalSectionLocker lock( &obj->internal_latest_obs_cs() );
	CObservation3DRangeScan &obs = obj->internal_latest_obs();

	obs.hasRangeImage  = true;
	obs.range_is_depth = true;

#ifdef KINECT_PROFILE_MEM_ALLOC
	alloc_tim.enter("depth_cb alloc");
#endif

	// This method will try to exploit memory pooling if possible:
	obs.rangeImage_setSize(frMode.height,frMode.width);

#ifdef KINECT_PROFILE_MEM_ALLOC
	alloc_tim.leave("depth_cb alloc");
#endif

	const CKinect::TDepth2RangeArray &r2m = obj->getRawDepth2RangeConversion();
	for (int r=0;r<frMode.height;r++)
		for (int c=0;c<frMode.width;c++)
		{
			// For now, quickly save the depth as it comes from the sensor, it'll
			//  transformed later on in getNextObservation()
			const uint16_t v = *depth++;
			obs.rangeImage.coeffRef(r,c) = r2m[v & KINECT_RANGES_TABLE_MASK];
		}
	obj->internal_tim_latest_depth() = timestamp;
}

void rgb_cb(freenect_device *dev, void *img_data, uint32_t timestamp)
{
	CKinect *obj = reinterpret_cast<CKinect*>(freenect_get_user(dev));
	const freenect_frame_mode frMode = freenect_get_current_video_mode(dev);

	// Update of the timestamps at the end:
	mrpt::synch::CCriticalSectionLocker lock( &obj->internal_latest_obs_cs() );
	CObservation3DRangeScan &obs = obj->internal_latest_obs();

#ifdef KINECT_PROFILE_MEM_ALLOC
	alloc_tim.enter("depth_rgb loadFromMemoryBuffer");
#endif

	obs.hasIntensityImage = true;
	if (obj->getVideoChannel()==CKinect::VIDEO_CHANNEL_RGB)
	{
	     // Color image: We asked for Bayer data, so we can decode it outselves here
	     //  and avoid having to reorder Green<->Red channels, as would be needed with
	     //  the RGB image from freenect.
          obs.intensityImageChannel = mrpt::obs::CObservation3DRangeScan::CH_VISIBLE;
          obs.intensityImage.resize(frMode.width, frMode.height, CH_RGB, true /* origin=top-left */ );

#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM<0x200
		  // Version for VERY OLD OpenCV versions:
		  IplImage *src_img_bayer = cvCreateImageHeader(cvSize(frMode.width,frMode.height),8,1);
		  src_img_bayer->imageDataOrigin = reinterpret_cast<char*>(img_data);
		  src_img_bayer->imageData = src_img_bayer->imageDataOrigin;
		  src_img_bayer->widthStep = frMode.width;

		  IplImage *dst_img_RGB = obs.intensityImage.getAs<IplImage>();

          // Decode Bayer image:
		  cvCvtColor(src_img_bayer, dst_img_RGB, CV_BayerGB2BGR);

#	else
		  // Version for modern OpenCV:
          const cv::Mat  src_img_bayer( frMode.height, frMode.width, CV_8UC1, img_data, frMode.width );

          cv::Mat        dst_img_RGB= cv::cvarrToMat( obs.intensityImage.getAs<IplImage>(), false /* dont copy buffers */ );

          // Decode Bayer image:
          cv::cvtColor(src_img_bayer, dst_img_RGB, CV_BayerGB2BGR);
#	endif
#else
     THROW_EXCEPTION("Need building with OpenCV!")
#endif

	}
	else
	{
	     // IR data: grayscale 8bit
          obs.intensityImageChannel = mrpt::obs::CObservation3DRangeScan::CH_IR;
          obs.intensityImage.loadFromMemoryBuffer(
               frMode.width,
               frMode.height,
               false, // Color image?
               reinterpret_cast<unsigned char*>(img_data)
               );

	}

	//obs.intensityImage.setChannelsOrder_RGB();

#ifdef KINECT_PROFILE_MEM_ALLOC
	alloc_tim.leave("depth_rgb loadFromMemoryBuffer");
#endif

	obj->internal_tim_latest_rgb() = timestamp;
}
// ========  END OF GLOBAL CALLBACK FUNCTIONS ========
#endif // MRPT_HAS_KINECT_FREENECT


void CKinect::open()
{
	if (isOpen())
		close();

	// Alloc memory, if this is the first time:
	m_buf_depth.resize(640*480*3); // We'll resize this below if needed
	m_buf_rgb.resize(640*480*3);

#if MRPT_HAS_KINECT_FREENECT  // ----> libfreenect
	// Try to open the device:
	if (freenect_init(f_ctx_ptr, NULL) < 0)
		THROW_EXCEPTION("freenect_init() failed")

	freenect_set_log_level(f_ctx,
#ifdef _DEBUG
		FREENECT_LOG_DEBUG
#else
		FREENECT_LOG_WARNING
#endif
		);

	int nr_devices = freenect_num_devices(f_ctx);
	//printf("[CKinect] Number of devices found: %d\n", nr_devices);

	if (!nr_devices)
		THROW_EXCEPTION("No Kinect devices found.")

	// Open the given device number:
	if (freenect_open_device(f_ctx, f_dev_ptr, m_user_device_number) < 0)
		THROW_EXCEPTION_CUSTOM_MSG1("Error opening Kinect sensor with index: %d",m_user_device_number)

	// Setup:
	if (m_initial_tilt_angle!=360) // 360 means no motor command.
          setTiltAngleDegrees(m_initial_tilt_angle);
	freenect_set_led(f_dev,LED_RED);
	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_callback(f_dev, rgb_cb);

	// rgb or IR channel:
	const freenect_frame_mode desiredFrMode = freenect_find_video_mode(
		FREENECT_RESOLUTION_MEDIUM,
		m_video_channel==VIDEO_CHANNEL_IR ?
			FREENECT_VIDEO_IR_8BIT
			:
			FREENECT_VIDEO_BAYER // FREENECT_VIDEO_RGB: Use Bayer instead so we can directly decode it here
		);

	// Switch to that video mode:
	if (freenect_set_video_mode(f_dev, desiredFrMode)<0)
		THROW_EXCEPTION("Error setting Kinect video mode.")


	// Get video mode:
	const freenect_frame_mode frMode = freenect_get_current_video_mode(f_dev);

	// Realloc mem:
	m_buf_depth.resize(frMode.width*frMode.height*3);
	m_buf_rgb.resize(frMode.width*frMode.height*3);

	// Save resolution:
	m_cameraParamsRGB.ncols = frMode.width;
	m_cameraParamsRGB.nrows = frMode.height;

	m_cameraParamsDepth.ncols = frMode.width;
	m_cameraParamsDepth.nrows = frMode.height;

	freenect_set_video_buffer(f_dev, &m_buf_rgb[0]);
	freenect_set_depth_buffer(f_dev, &m_buf_depth[0]);

	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_10BIT));

	// Set user data = pointer to "this":
	freenect_set_user(f_dev, this);

	if (freenect_start_depth(f_dev)<0)
		THROW_EXCEPTION("Error starting depth streaming.")

	if (freenect_start_video(f_dev)<0)
		THROW_EXCEPTION("Error starting video streaming.")

#endif // MRPT_HAS_KINECT_FREENECT
}

void CKinect::close()
{
#if MRPT_HAS_KINECT_FREENECT
	if (f_dev)
	{
		freenect_stop_depth(f_dev);
		freenect_stop_video(f_dev);
		freenect_close_device(f_dev);
	}
	m_f_dev = NULL;

	if (f_ctx)
		freenect_shutdown(f_ctx);
	m_f_ctx = NULL;
#endif // MRPT_HAS_KINECT_FREENECT
}

/** Changes the video channel to open (RGB or IR) - you can call this method before start grabbing or in the middle of streaming and the video source will change on the fly.
	Default is RGB channel.
*/
void  CKinect::setVideoChannel(const TVideoChannel vch)
{
#if MRPT_HAS_KINECT_FREENECT
	m_video_channel = vch;
	if (!isOpen()) return; // Nothing else to do here.

	// rgb or IR channel:
	freenect_stop_video(f_dev);

	// rgb or IR channel:
	const freenect_frame_mode desiredFrMode = freenect_find_video_mode(
		FREENECT_RESOLUTION_MEDIUM,
		m_video_channel==VIDEO_CHANNEL_IR ?
			FREENECT_VIDEO_IR_8BIT
			:
			FREENECT_VIDEO_BAYER // FREENECT_VIDEO_RGB: Use Bayer instead so we can directly decode it here
		);

	// Switch to that video mode:
	if (freenect_set_video_mode(f_dev, desiredFrMode)<0)
		THROW_EXCEPTION("Error setting Kinect video mode.")

	freenect_start_video(f_dev);

#else
	MRPT_UNUSED_PARAM(vch);
#endif // MRPT_HAS_KINECT_FREENECT
}


/** The main data retrieving function, to be called after calling loadConfig() and initialize().
  *  \param out_obs The output retrieved observation (only if there_is_obs=true).
  *  \param there_is_obs If set to false, there was no new observation.
  *  \param hardware_error True on hardware/comms error.
  *
  * \sa doProcess
  */
void CKinect::getNextObservation(
	mrpt::obs::CObservation3DRangeScan &_out_obs,
	bool &there_is_obs,
	bool &hardware_error )
{
	there_is_obs=false;
	hardware_error = false;

#if MRPT_HAS_KINECT_FREENECT

	static const double max_wait_seconds = 1./25.;
	static const TTimeStamp max_wait = mrpt::system::secondsToTimestamp(max_wait_seconds);

	// Mark previous observation's timestamp as out-dated:
	m_latest_obs.hasPoints3D        = false;
	m_latest_obs.hasRangeImage      = false;
	m_latest_obs.hasIntensityImage  = false;
	m_latest_obs.hasConfidenceImage = false;

	const TTimeStamp tim0 = mrpt::system::now();

	// Reset these timestamp flags so if they are !=0 in the next call we're sure they're new frames.
	m_latest_obs_cs.enter();
	m_tim_latest_rgb   = 0;
	m_tim_latest_depth = 0;
	m_latest_obs_cs.leave();

	while (freenect_process_events(f_ctx)>=0 && mrpt::system::now()<(tim0+max_wait) )
	{
		// Got a new frame?
		if ( (!m_grab_image || m_tim_latest_rgb!=0) &&   // If we are NOT grabbing RGB or we are and there's a new frame...
			 (!m_grab_depth || m_tim_latest_depth!=0)    // If we are NOT grabbing Depth or we are and there's a new frame...
		   )
		{
			// Approx: 0.5ms delay between depth frame (first) and RGB frame (second).
			//cout << "m_tim_latest_rgb: " << m_tim_latest_rgb << " m_tim_latest_depth: "<< m_tim_latest_depth <<endl;
			there_is_obs=true;
			break;
		}
	}

	// Handle the case when there is NOT depth frames (if there's something very close blocking the IR sensor) but we have RGB:
	if ( (m_grab_image && m_tim_latest_rgb!=0) &&
		 (m_grab_depth && m_tim_latest_depth==0) )
	{
		// Mark the entire range data as invalid:
		m_latest_obs.hasRangeImage = true;
		m_latest_obs.range_is_depth = true;

		m_latest_obs_cs.enter(); // Important: if system is running slow, etc. we cannot tell for sure that the depth buffer is not beeing filled right now:
		m_latest_obs.rangeImage.setSize(m_cameraParamsDepth.nrows,m_cameraParamsDepth.ncols);
		m_latest_obs.rangeImage.setConstant(0); // "0" means: error in range
		m_latest_obs_cs.leave();
		there_is_obs=true;
	}


	if (!there_is_obs)
		return;


	// We DO have a fresh new observation:

	// Quick save the observation to the user's object:
	m_latest_obs_cs.enter();
		_out_obs.swap(m_latest_obs);
	m_latest_obs_cs.leave();
#endif

	// Set common data into observation:
	// --------------------------------------
	_out_obs.sensorLabel = m_sensorLabel;
	_out_obs.timestamp = mrpt::system::now();
	_out_obs.sensorPose = m_sensorPoseOnRobot;
	_out_obs.relativePoseIntensityWRTDepth = m_relativePoseIntensityWRTDepth;

	_out_obs.cameraParams          = m_cameraParamsDepth;
	_out_obs.cameraParamsIntensity = m_cameraParamsRGB;

	// 3D point cloud:
	if ( _out_obs.hasRangeImage && m_grab_3D_points )
	{
		_out_obs.project3DPointsFromDepthImage();

		if ( !m_grab_depth )
		{
			_out_obs.hasRangeImage = false;
			_out_obs.rangeImage.resize(0,0);
		}

	}

	// preview in real-time?
	if (m_preview_window)
	{
		if ( _out_obs.hasRangeImage )
		{
			if (++m_preview_decim_counter_range>m_preview_window_decimation)
			{
				m_preview_decim_counter_range=0;
				if (!m_win_range)	{ m_win_range = mrpt::gui::CDisplayWindow::Create("Preview RANGE"); m_win_range->setPos(5,5); }

				// Normalize the image
				mrpt::utils::CImage  img;
				img.setFromMatrix(_out_obs.rangeImage);
				CMatrixFloat r = _out_obs.rangeImage * float(1.0/this->m_maxRange);
				m_win_range->showImage(img);
			}
		}
		if ( _out_obs.hasIntensityImage )
		{
			if (++m_preview_decim_counter_rgb>m_preview_window_decimation)
			{
				m_preview_decim_counter_rgb=0;
				if (!m_win_int)		{ m_win_int = mrpt::gui::CDisplayWindow::Create("Preview INTENSITY"); m_win_int->setPos(300,5); }
				m_win_int->showImage(_out_obs.intensityImage );
			}
		}
	}
	else
	{
		if (m_win_range) m_win_range.clear();
		if (m_win_int) m_win_int.clear();
	}
}

/* -----------------------------------------------------
				getNextObservation (with IMU)
----------------------------------------------------- */
void CKinect::getNextObservation(
	mrpt::obs::CObservation3DRangeScan &out_obs,
	mrpt::obs::CObservationIMU         &out_obs_imu,
	bool &there_is_obs,
	bool &hardware_error )
{
	// First, try getting the RGB+Depth data:
	getNextObservation(out_obs,there_is_obs, hardware_error);

	// If successful, fill out the accelerometer data:
	if (there_is_obs && this->m_grab_IMU)
	{
		double acc_x=0,acc_y=0,acc_z=0; // In m/s^2
		bool  has_good_acc=false;

#if MRPT_HAS_KINECT_FREENECT
		{
			freenect_update_tilt_state(f_dev);
			freenect_raw_tilt_state* state = freenect_get_tilt_state(f_dev);
			if (state)
			{
				has_good_acc = true;
				double lx,ly,lz;
				freenect_get_mks_accel(state, &lx, &ly, &lz);

				// Convert to a unified coordinate system:
				// +x: forward
				// +y: left
				// +z: upward
				acc_x = -lz;
				acc_y = -lx;
				acc_z = -ly;
			}
		}
#endif

		// Common part for any implementation:
		if (has_good_acc)
		{
			out_obs_imu.sensorLabel = out_obs.sensorLabel + "_IMU";
			out_obs_imu.timestamp   = out_obs.timestamp;
			out_obs_imu.sensorPose = out_obs.sensorPose;

			for (size_t i=0;i<out_obs_imu.dataIsPresent.size();i++)
				out_obs_imu.dataIsPresent[i] = false;

			out_obs_imu.dataIsPresent[IMU_X_ACC] = true;
			out_obs_imu.dataIsPresent[IMU_Y_ACC] = true;
			out_obs_imu.dataIsPresent[IMU_Z_ACC] = true;

			out_obs_imu.rawMeasurements[IMU_X_ACC] = acc_x;
			out_obs_imu.rawMeasurements[IMU_Y_ACC] = acc_y;
			out_obs_imu.rawMeasurements[IMU_Z_ACC] = acc_z;
		}
	}
}


/* -----------------------------------------------------
				setPathForExternalImages
----------------------------------------------------- */
void CKinect::setPathForExternalImages( const std::string &directory )
{
	MRPT_UNUSED_PARAM(directory);
	// Ignore for now. It seems performance is better grabbing everything
	// to a single big file than creating hundreds of smaller files per second...
	return;

//	if (!mrpt::system::createDirectory( directory ))
//	{
//		THROW_EXCEPTION_CUSTOM_MSG1("Error: Cannot create the directory for externally saved images: %s",directory.c_str() )
//	}
//	m_path_for_external_images = directory;
}


/** Change tilt angle \note Sensor must be open first. */
void CKinect::setTiltAngleDegrees(double angle)
{
	ASSERTMSG_(isOpen(),"Sensor must be open first")

#if MRPT_HAS_KINECT_FREENECT
	freenect_set_tilt_degs(f_dev,angle);
#else
	MRPT_UNUSED_PARAM(angle);
#endif

}

double CKinect::getTiltAngleDegrees()
{
	ASSERTMSG_(isOpen(),"Sensor must be open first")

#if MRPT_KINECT_WITH_FREENECT
	freenect_update_tilt_state(f_dev);
	freenect_raw_tilt_state *ts=freenect_get_tilt_state(f_dev);
	return freenect_get_tilt_degs(ts);
#else
	return 0;
#endif
}
