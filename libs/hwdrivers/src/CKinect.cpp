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

#include <mrpt/hwdrivers.h> // Precompiled header

#include <mrpt/hwdrivers/CKinect.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::synch;

IMPLEMENTS_GENERIC_SENSOR(CKinect,mrpt::hwdrivers)


#if MRPT_HAS_KINECT
#	include "libfreenect/libfreenect.h"
#endif

// Macros to convert the opaque pointers in the class header:
#define f_ctx  reinterpret_cast<freenect_context*>(m_f_ctx)
#define f_ctx_ptr  reinterpret_cast<freenect_context**>(&m_f_ctx)
#define f_dev  reinterpret_cast<freenect_device*>(m_f_dev)
#define f_dev_ptr  reinterpret_cast<freenect_device**>(&m_f_dev)


// GLOBAL DATA USED TO REFER A "freenect_device*" TO ITS MRPT OBJECT
// ----------------------------------------------------------------------
struct TInfoPerSensor
{
	TInfoPerSensor() : running_obj(NULL),tim_latest_depth(0),tim_latest_rgb(0) { }

	CKinect                 *running_obj; //!< My parent object
	CObservation3DRangeScan  latest_obs;
	uint32_t                 tim_latest_depth, tim_latest_rgb; // 0 = not updated
};
std::map<freenect_device*,TInfoPerSensor> m_info_per_sensor;
CCriticalSection                          m_info_per_sensor_cs;

// Look-up table for converting range raw uint16_t numbers to ranges:
bool  range2meters_done = false;
#define RANGES_TABLE       1024   // 10bit,   2048 : 11bit
#define RANGES_TABLE_MASK  0x03FF // 10bit //0x07FF

mrpt::vector_float range2meters(RANGES_TABLE);

void calculate_range2meters()
{
	const float k1 = 1.1863f;
	const float k2 = 2842.5f;
	const float k3 = 0.1236f;

	for (size_t i=0; i<RANGES_TABLE; i++)
			range2meters[i] = k3 * tanf(i/k2 + k1);

	// Minimum/Maximum range means error:
	range2meters[0] = 0;
	range2meters[RANGES_TABLE-1] = 0;

	range2meters_done = true;
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
	m_f_ctx(NULL), // The "freenect_context", or NULL if closed
	m_f_dev(NULL), // The "freenect_device", or NULL if closed
	m_user_device_number(0),
	m_grab_image(true),
	m_grab_depth(true),
	m_grab_IMU(true)
{
	if (!range2meters_done)	// Build LUT:
		calculate_range2meters();

	// Get maximum range:
	m_maxRange=range2meters[RANGES_TABLE-2];  // Recall: r[Max-1] means error.

	// Default label:
	m_sensorLabel = "KINECT";

	// =========== Default params ===========
	// ----- RGB -----
	m_cameraParamsRGB.ncols = FREENECT_FRAME_W;
	m_cameraParamsRGB.nrows = FREENECT_FRAME_H;

	m_cameraParamsRGB.cx(328.94272028759258);
	m_cameraParamsRGB.cy(267.48068171871557);
	m_cameraParamsRGB.fx(529.2151);
	m_cameraParamsRGB.fy(525.5639);

	m_cameraParamsRGB.dist.zeros();

	// ----- Depth -----
	m_cameraParamsDepth.ncols = FREENECT_FRAME_W;
	m_cameraParamsDepth.nrows = FREENECT_FRAME_H;

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

	CObservation3DRangeScanPtr newObs = CObservation3DRangeScan::Create();

	getNextObservation( *newObs, thereIs, hwError );

	if (hwError)
	{
		m_state = ssError;
	    THROW_EXCEPTION("Couldn't communicate to the Kinect sensor!");
	}

	if (thereIs)
	{
		m_state = ssWorking;
		appendObservation( newObs );
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

	MRPT_TODO("go on")
//	m_save_3d = configSource.read_bool(iniSection,"save_3d",m_save_3d);

//	m_external_images_format = mrpt::utils::trim( configSource.read_string( iniSection, "external_images_format", m_external_images_format ) );
//	m_external_images_jpeg_quality = configSource.read_int( iniSection, "external_images_jpeg_quality", m_external_images_jpeg_quality );

	try
	{
		m_cameraParamsRGB.loadFromConfigFile(iniSection, configSource);
	}
	catch(std::exception &)
	{
		// If there's some missing field, just keep the default values.
	}
}

bool CKinect::isOpen() const
{
	return f_dev != NULL;
}


// ========  GLOBAL CALLBACK FUNCTIONS ========
#if MRPT_HAS_KINECT
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	uint16_t *depth = reinterpret_cast<uint16_t *>(v_depth);

	{
		CCriticalSectionLocker lock(&m_info_per_sensor_cs);
		std::map<freenect_device*,TInfoPerSensor>::iterator it=m_info_per_sensor.find(dev);
		if (it!=m_info_per_sensor.end())
		{
			TInfoPerSensor &ips = it->second;
			ips.tim_latest_depth = timestamp;

			ips.latest_obs.hasRangeImage = true;
			ips.latest_obs.rangeImage.setSize(FREENECT_FRAME_H,FREENECT_FRAME_W);
			for (int r=0;r<FREENECT_FRAME_H;r++)
				for (int c=0;c<FREENECT_FRAME_W;c++)
				{
					// For now, quickly save the depth as it comes from the sensor, it'll
					//  transformed later on in getNextObservation()
					const uint16_t v = *depth++;
					ips.latest_obs.rangeImage.coeffRef(r,c) = range2meters[v % RANGES_TABLE_MASK]; // It's >= RANGES_TABLE
				}
		}
	}


//	for (int i=0; i<FREENECT_FRAME_PIX; i++)

}

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	{
		CCriticalSectionLocker lock(&m_info_per_sensor_cs);
		std::map<freenect_device*,TInfoPerSensor>::iterator it=m_info_per_sensor.find(dev);
		if (it!=m_info_per_sensor.end())
		{
			TInfoPerSensor &ips = it->second;
			ips.tim_latest_rgb  = timestamp;

			ips.latest_obs.hasIntensityImage = true;
			ips.latest_obs.intensityImage.loadFromMemoryBuffer(
				FREENECT_FRAME_W,
				FREENECT_FRAME_H,
				CH_RGB,
				reinterpret_cast<unsigned char*>(rgb),
				true );
		}
	}

}
#endif
// ========  END OF GLOBAL CALLBACK FUNCTIONS ========


void CKinect::open()
{
#if MRPT_HAS_KINECT
	if (isOpen())
		close();

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

	// Register my f_dev for the callback functions:
	{
		CCriticalSectionLocker lock(&m_info_per_sensor_cs);
		m_info_per_sensor[f_dev].running_obj = this;
	}

	// Save resolution:
	m_cameraParamsRGB.ncols = FREENECT_FRAME_W;
	m_cameraParamsRGB.nrows = FREENECT_FRAME_H;

	// Alloc memory, if this is the first time:
	m_buf_depth.resize(m_cameraParamsRGB.ncols*m_cameraParamsRGB.nrows*3);
	m_buf_rgb.resize(m_cameraParamsRGB.ncols*m_cameraParamsRGB.nrows*3);

	// Setup:
	setTiltAngleDegrees(0);
	freenect_set_led(f_dev,LED_RED);
	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_callback(f_dev, rgb_cb);

	freenect_set_video_format(f_dev, FREENECT_VIDEO_RGB);
	freenect_set_depth_format(f_dev, FREENECT_DEPTH_10BIT); // FREENECT_DEPTH_11BIT);
	freenect_set_video_buffer(f_dev, &m_buf_rgb[0]);
	// freenect_set_depth_buffer(f_dev, &m_buf_depth[0]);  // JL: not needed??

	freenect_start_depth(f_dev);
	freenect_start_video(f_dev);

#endif
}

void CKinect::close()
{
#if MRPT_HAS_KINECT
	if (f_dev)
	{
		freenect_stop_depth(f_dev);
		freenect_stop_video(f_dev);
		freenect_close_device(f_dev);

		// Un-register from global list for the callbacks:
		mrpt::system::sleep(20);
		{
			CCriticalSectionLocker lock(&m_info_per_sensor_cs);
			freenect_device *the_dev  = f_dev;
			m_info_per_sensor.erase(the_dev);
		}
	}
	m_f_dev = NULL;

	if (f_ctx)
		freenect_shutdown(f_ctx);
	m_f_ctx = NULL;
#endif
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
#if MRPT_HAS_KINECT
	there_is_obs=false;
	hardware_error = false;

	static const double max_wait_seconds = 0.1;
	static const TTimeStamp max_wait = mrpt::system::secondsToTimestamp(max_wait_seconds);

	// Mark previous observation's timestamp as out-dated:
	TInfoPerSensor *ips; // We'll use this direct pointer to avoid the map::find in the inner loop, below:
	{
		CCriticalSectionLocker lock(&m_info_per_sensor_cs);
		std::map<freenect_device*,TInfoPerSensor>::iterator it=m_info_per_sensor.find(f_dev);
		ASSERT_(it!=m_info_per_sensor.end())
		ips = &it->second;

		ips->latest_obs.hasPoints3D        = false;
		ips->latest_obs.hasRangeImage      = false;
		ips->latest_obs.hasIntensityImage  = false;
		ips->latest_obs.hasConfidenceImage = false;

		ips->tim_latest_rgb   = 0;
		ips->tim_latest_depth = 0;
	}

	const TTimeStamp tim0 = mrpt::system::now();

	while (freenect_process_events(f_ctx)>=0 && mrpt::system::now()<(tim0+max_wait) )
	{
		// Got a new frame?
		{
			CCriticalSectionLocker lock(&m_info_per_sensor_cs);
			if ( (!m_grab_image || ips->tim_latest_rgb!=0) &&
				 (!m_grab_depth || ips->tim_latest_depth!=0) )
			{
				there_is_obs=true;
				break;
			}
		}
	}

	if (!there_is_obs)
		return;

	// We DO have a fresh new observation:

	// Quick save the observation to the user's object:
	{
		CCriticalSectionLocker lock(&m_info_per_sensor_cs);
		_out_obs.swap(ips->latest_obs);
	}

	// Set common data into observation:
	// --------------------------------------
	_out_obs.sensorLabel = m_sensorLabel;
	_out_obs.timestamp = mrpt::system::now();
	_out_obs.sensorPose = m_sensorPoseOnRobot;

	_out_obs.cameraParams          = m_cameraParamsDepth;
	_out_obs.cameraParamsIntensity = m_cameraParamsRGB;

	// 3D point cloud:
	if ( _out_obs.hasRangeImage )
	{
		_out_obs.hasPoints3D = true;
		_out_obs.points3D_x.resize( FREENECT_FRAME_W * FREENECT_FRAME_H );
		_out_obs.points3D_y.resize( FREENECT_FRAME_W * FREENECT_FRAME_H );
		_out_obs.points3D_z.resize( FREENECT_FRAME_W * FREENECT_FRAME_H );

		// originally based on:
		// depth2cloud.py  - convert Kinect depth image into 3D point cloud, in PLY format
		//   Ben Bongalon (ben@borglabs.com)
		// See also: http://nicolas.burrus.name/index.php/Research/KinectCalibration
		float *xs= &_out_obs.points3D_x[0];
		float *ys= &_out_obs.points3D_y[0];
		float *zs= &_out_obs.points3D_z[0];

		const float r_cx = m_cameraParamsDepth.cx();
		const float r_cy = m_cameraParamsDepth.cy();
		const float r_fx_inv = 1.0f/m_cameraParamsDepth.fx();
		const float r_fy_inv = 1.0f/m_cameraParamsDepth.fy();

		for (int r=0;r<FREENECT_FRAME_H;r++)
			for (int c=0;c<FREENECT_FRAME_W;c++)
			{
				*xs = _out_obs.rangeImage.coeff(r,c);
				*zs++ = (r_cy - r) * (*xs) * r_fx_inv;
				*ys++ = (r_cx - c) * (*xs) * r_fy_inv;
				xs++;
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

	return;
#endif
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
#if MRPT_HAS_KINECT
	ASSERTMSG_(isOpen(),"Sensor must be open first")

	freenect_set_tilt_degs(f_dev,angle);
#endif
}

double CKinect::getTiltAngleDegrees()
{
#if MRPT_HAS_KINECT
	ASSERTMSG_(isOpen(),"Sensor must be open first")

	freenect_update_tilt_state(f_dev);
	freenect_raw_tilt_state *ts=freenect_get_tilt_state(f_dev);
	return freenect_get_tilt_degs(ts);
#else
	return 0;
#endif
}
