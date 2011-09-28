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

#include <mrpt/hwdrivers.h> // Precompiled header

#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/utils/CTimeLogger.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::synch;

IMPLEMENTS_GENERIC_SENSOR(CKinect,mrpt::hwdrivers)

// Whether to profile memory allocations:
//#define KINECT_PROFILE_MEM_ALLOC

#if MRPT_HAS_KINECT_FREENECT
#	include <libfreenect.h>
#elif MRPT_HAS_KINECT_CL_NUI
#	include <CLNUIDevice.h>
#	define KINECT_W 640
#	define KINECT_H 480
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


#if MRPT_HAS_KINECT_CL_NUI
	// Macros to convert the opaque pointers in the class header:
	#define clnui_motor  reinterpret_cast<CLNUIMotor>(m_clnui_motor)
	#define clnui_cam    reinterpret_cast<CLNUICamera>(m_clnui_cam)
#endif // MRPT_HAS_KINECT_CL_NUI


#ifdef KINECT_PROFILE_MEM_ALLOC
mrpt::utils::CTimeLogger alloc_tim;
#endif

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

#if MRPT_HAS_KINECT_CL_NUI
	m_clnui_cam(NULL),
	m_clnui_motor(NULL),
#endif

	m_relativePoseIntensityWRTDepth(0,-0.02,0, DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-90)),
	m_initial_tilt_angle(0),
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
	m_cameraParamsDepth.nrows = 480;

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

	m_cameraParamsRGB.cx( configSource.read_double(iniSection,"rgb_cx", m_cameraParamsRGB.cx() ) );
	m_cameraParamsRGB.cy( configSource.read_double(iniSection,"rgb_cy", m_cameraParamsRGB.cy() ) );
	m_cameraParamsRGB.fx( configSource.read_double(iniSection,"rgb_fx", m_cameraParamsRGB.fx() ) );
	m_cameraParamsRGB.fy( configSource.read_double(iniSection,"rgb_fy", m_cameraParamsRGB.fy() ) );

	m_cameraParamsDepth.cx( configSource.read_double(iniSection,"d_cx", m_cameraParamsDepth.cx() ) );
	m_cameraParamsDepth.cy( configSource.read_double(iniSection,"d_cy", m_cameraParamsDepth.cy() ) );
	m_cameraParamsDepth.fx( configSource.read_double(iniSection,"d_fx", m_cameraParamsDepth.fx() ) );
	m_cameraParamsDepth.fy( configSource.read_double(iniSection,"d_fy", m_cameraParamsDepth.fy() ) );

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
#elif MRPT_HAS_KINECT_CL_NUI
	return m_clnui_cam != NULL;
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
	CObservation3DRangeScan &obs = obj->internal_latest_obs();
	mrpt::synch::CCriticalSectionLocker lock( &obj->internal_latest_obs_cs() );

	obs.hasRangeImage  = true;
	obs.range_is_depth = true;

#ifdef KINECT_PROFILE_MEM_ALLOC
	alloc_tim.enter("depth_cb alloc");
#endif

	// This method will try to exploit memory pooling if possible:
	obs.rangeImage_setSize(frMode.height,frMode.width); // Was: obs.rangeImage.setSize(frMode.height,frMode.width);

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

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	CKinect *obj = reinterpret_cast<CKinect*>(freenect_get_user(dev));
	const freenect_frame_mode frMode = freenect_get_current_video_mode(dev);

	// Update of the timestamps at the end:
	CObservation3DRangeScan &obs = obj->internal_latest_obs();
	mrpt::synch::CCriticalSectionLocker lock( &obj->internal_latest_obs_cs() );

#ifdef KINECT_PROFILE_MEM_ALLOC
	alloc_tim.enter("depth_rgb loadFromMemoryBuffer");
#endif

	MRPT_TODO("Try to optimize this: either avoid R-B swap, or reuse memory block, etc.")

	obs.hasIntensityImage = true;
	obs.intensityImageChannel = obj->getVideoChannel()==CKinect::VIDEO_CHANNEL_RGB ? mrpt::slam::CObservation3DRangeScan::CH_VISIBLE : mrpt::slam::CObservation3DRangeScan::CH_IR;
	obs.intensityImage.loadFromMemoryBuffer(
		frMode.width,
		frMode.height,
		obj->getVideoChannel()==CKinect::VIDEO_CHANNEL_RGB, // Color image?
		reinterpret_cast<unsigned char*>(rgb),
		true  // Swap red/blue
		);

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
			FREENECT_VIDEO_RGB
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

#if MRPT_HAS_KINECT_CL_NUI  // ----->  CL NUI SDK
	// Open handles:
	m_clnui_motor = CreateNUIMotor
	(
	GetNUIDeviceSerial(0)  // Comment this line if you use a version older than CL NUI SDK v1.0.0.1210
	);
	if (!m_clnui_motor)
		THROW_EXCEPTION("Can't open Kinect camera (m_clnui_motor()==NULL)")

	m_clnui_cam   = CreateNUICamera
	(
	GetNUIDeviceSerial(0)  // Comment this line if you use a version older than CL NUI SDK v1.0.0.1210
	);
	if (!m_clnui_cam)
		THROW_EXCEPTION("Can't open Kinect camera (CreateNUICamera()==NULL)")

	//PCHAR pStr = GetNUIMotorSerial(m_clnui_motor);

	bool ret = StartNUICamera(clnui_cam);
	if (!ret)
		THROW_EXCEPTION("Can't start grabbing from Kinect camera (StartNUICamera failed)")

#endif // MRPT_HAS_KINECT_CL_NUI
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

#if MRPT_HAS_KINECT_CL_NUI
	// Stop grabbing & close handles:
	if (m_clnui_cam)
	{
		StopNUICamera(clnui_cam);
		mrpt::system::sleep(10);
		DestroyNUICamera(clnui_cam);
		m_clnui_cam   = NULL;
	}

	if (m_clnui_motor)
	{
		DestroyNUIMotor(clnui_motor);
		m_clnui_motor = NULL;
	}
#endif // MRPT_HAS_KINECT_CL_NUI
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
			FREENECT_VIDEO_RGB
		);

	// Switch to that video mode:
	if (freenect_set_video_mode(f_dev, desiredFrMode)<0)
		THROW_EXCEPTION("Error setting Kinect video mode.")

	freenect_start_video(f_dev);

#endif // MRPT_HAS_KINECT_FREENECT

#if MRPT_HAS_KINECT_CL_NUI
	THROW_EXCEPTION("Grabbing IR intensity is not available with CL NUI Kinect driver.")
#endif // MRPT_HAS_KINECT_CL_NUI


}


/** The main data retrieving function, to be called after calling loadConfig() and initialize().
  *  \param out_obs The output retrieved observation (only if there_is_obs=true).
  *  \param there_is_obs If set to false, there was no new observation.
  *  \param hardware_error True on hardware/comms error.
  *
  * \sa doProcess
  */
void CKinect::getNextObservation(
	mrpt::slam::CObservation3DRangeScan &_out_obs,
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
		m_latest_obs.rangeImage.setSize(m_cameraParamsDepth.nrows,m_cameraParamsDepth.ncols);
		m_latest_obs.rangeImage.setConstant(0); // "0" means: error in range
		there_is_obs=true;
	}


	if (!there_is_obs)
		return;


	// We DO have a fresh new observation:

	// Quick save the observation to the user's object:
	m_latest_obs_cs.enter();
		_out_obs.swap(m_latest_obs);
	m_latest_obs_cs.leave();

#elif MRPT_HAS_KINECT_CL_NUI

	const int waitTimeout = 200;
	const bool there_is_rgb   = GetNUICameraColorFrameRGB24(m_clnui_cam, &m_buf_rgb[0],waitTimeout);
	const bool there_is_depth = GetNUICameraDepthFrameRAW(m_clnui_cam, (PUSHORT)&m_buf_depth[0],waitTimeout);

	there_is_obs = (!m_grab_image  || there_is_rgb) &&
	               (!m_grab_depth  || there_is_depth);

	if (!there_is_obs)
		return;

	// We DO have a fresh new observation:
	{
		CObservation3DRangeScan  newObs;

		newObs.hasConfidenceImage = false;

		// Set intensity image ----------------------
		if (m_grab_image)
		{
			newObs.hasIntensityImage  = true;
			newObs.intensityImageChannel = mrpt::slam::CObservation3DRangeScan::CH_VISIBLE;
			newObs.intensityImage.loadFromMemoryBuffer(KINECT_W,KINECT_H,true,&m_buf_rgb[0]);
		}

		// Set range image --------------------------
		if (m_grab_depth || m_grab_3D_points)
		{
			newObs.hasRangeImage = true;
			newObs.range_is_depth = true;
			newObs.rangeImage.setSize(KINECT_H,KINECT_W);
			PUSHORT depthPtr = (PUSHORT)&m_buf_depth[0];
			for (int r=0;r<KINECT_H;r++)
				for (int c=0;c<KINECT_W;c++)
				{
					const uint16_t v = (*depthPtr++);
					newObs.rangeImage.coeffRef(r,c) = m_range2meters[v & KINECT_RANGES_TABLE_MASK];
				}
		}

		// Save 3D point cloud ---------------------
		// 3d points are generated above, in the code common to libfreenect & CL NUI.


		// Save the observation to the user's object:
		_out_obs.swap(newObs);
	}

#endif  // end MRPT_HAS_KINECT_CL_NUI

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
	mrpt::slam::CObservation3DRangeScan &out_obs,
	mrpt::slam::CObservationIMU         &out_obs_imu,
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
#elif MRPT_HAS_KINECT_CL_NUI
		{
			SHORT x, y, z;
			if (GetNUIMotorAccelerometer( clnui_motor, x,y,z))
			{
				has_good_acc = true;

				//the documentation for the accelerometer (http://www.kionix.com/Product%20Sheets/KXSD9%20Product%20Brief.pdf)
				//states there are 819 counts/g.
				// Also: Convert to a unified coordinate system:
				// +x: forward
				// +y: left
				// +z: upward
				acc_x = -y * 9.80665 / 819;
				acc_y = -x * 9.80665 / 819;
				acc_z = -z * 9.80665 / 819;
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
#elif MRPT_HAS_KINECT_CL_NUI
	// JL: I deduced this formula empirically, since CLNUI seems not to have documented this API!!
	const short int send_angle =  ( (angle<-31) ? -31 : ((angle>31) ? 31 : angle) ) * (0x4000)/31.0;
	SetNUIMotorPosition(clnui_motor, send_angle);
#endif

}

double CKinect::getTiltAngleDegrees()
{
	ASSERTMSG_(isOpen(),"Sensor must be open first")

#if MRPT_KINECT_WITH_FREENECT
	freenect_update_tilt_state(f_dev);
	freenect_raw_tilt_state *ts=freenect_get_tilt_state(f_dev);
	return freenect_get_tilt_degs(ts);

#elif MRPT_HAS_KINECT_CL_NUI
	// TODO: Does CL NUI provides this??
	return 0;
#else
	return 0;
#endif
}
