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

#include <mrpt/hwdrivers.h> // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CTypeSelector.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/slam/CObservationImage.h>
#include <mrpt/slam/CObservationStereoImages.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/gui/WxSubsystem.h>

#if MRPT_HAS_BUMBLEBEE
	#include <PGRFlyCapture.h>
	#include <triclops.h>
	#include <pgrflycapturestereo.h>
	#include <pnmutils.h>
#endif

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CCameraSensor,mrpt::hwdrivers)

/* -----------------------------------------------------
                Constructor
   ----------------------------------------------------- */
CCameraSensor::CCameraSensor() :
	m_sensorPose			(),
	m_grabber_type			("opencv"),
	m_capture_grayscale		(false),
	m_cv_camera_index		(0),
	m_cv_camera_type		("CAMERA_CV_AUTODETECT"),
	m_cv_options			(),
	m_dc1394_camera_guid	(0),
	m_dc1394_camera_unit	(0),
	m_dc1394_options		(),
	m_preview_decimation	(0),
	m_preview_reduction		(1),
	m_bumblebee_camera_index(0),
	m_bumblebee_options		(),
	m_bumblebee_monocam		(-1),
	m_svs_camera_index(0),
	m_svs_options                   (),
	m_sr_open_from_usb		(true),
	m_sr_save_3d			(true),
	m_sr_save_range_img		(true),
	m_sr_save_intensity_img	(true),
	m_sr_save_confidence	(true),

	m_kinect_save_3d		(true),
	m_kinect_save_range_img (true),
	m_kinect_save_intensity_img(true),
	m_kinect_video_rgb		(true),

	m_external_images_own_thread(false),
	m_cap_cv			(NULL),
	m_cap_dc1394		(NULL),
	m_cap_bumblebee		(NULL),
	m_cap_svs           (NULL),
	m_cap_ffmpeg		(NULL),
	m_cap_rawlog		(NULL),
	m_cap_swissranger	(NULL),
	m_cap_kinect        (NULL),
	m_camera_grab_decimator (0),
	m_camera_grab_decimator_counter(0),
	m_preview_counter	(0),
	m_external_image_saver_count( mrpt::system::getNumberOfProcessors() ),
	m_threadImagesSaverShouldEnd(false)
{
	m_sensorLabel = "CAMERA";
	m_state = CGenericSensor::ssInitializing;
}

/* -----------------------------------------------------
                initialize
   ----------------------------------------------------- */
void CCameraSensor::initialize()
{
	cout << "[CCameraSensor::initialize] Opening camera..." << endl;
	close();

	// Select type of device
	m_grabber_type = trim( lowerCase( m_grabber_type ) );
	m_cv_camera_type= trim( upperCase(m_cv_camera_type) );

	if (m_grabber_type=="opencv")
	{
		// OpenCV driver:
		mrpt::utils::CTypeSelector	camera_type(
		"CAMERA_CV_AUTODETECT,CAMERA_CV_DC1394,CAMERA_CV_VFL,CAMERA_CV_VFW,CAMERA_CV_MIL",
		"CAMERA_CV_AUTODETECT");

		int idx = camera_type.checkTypeIndex(m_cv_camera_type);
		if (idx<0)
		{
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION_CUSTOM_MSG1("Invalid value of :'%s'",m_cv_camera_type.c_str())
		}

		cout << format("[CCameraSensor::initialize] opencv camera, index: %i type: %i...\n", int(m_cv_camera_index),idx);
		m_cap_cv = new CImageGrabber_OpenCV( m_cv_camera_index, TCameraType(idx), m_cv_options );

		if (!m_cap_cv->isOpen())
		{
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("[CCameraSensor::initialize] ERROR: Couldn't open OpenCV camera.")
		}
	}
	else if (m_grabber_type=="dc1394")
	{
		//m_cap_dc1394
		cout << format("[CCameraSensor::initialize] dc1394 camera, GUID: 0x%lX  UNIT:%d...\n", long(m_dc1394_camera_guid),m_dc1394_camera_unit);
		m_cap_dc1394 = new CImageGrabber_dc1394( m_dc1394_camera_guid, m_dc1394_camera_unit, m_dc1394_options, true /* verbose */ );

		if (!m_cap_dc1394->isOpen())
		{
			m_state 	= CGenericSensor::ssError;
			THROW_EXCEPTION("[CCameraSensor::initialize] ERROR: Couldn't open dc1394 camera.")
		}
	}
	else if (m_grabber_type=="bumblebee")
	{
		//m_cap_bumblebee
		cout << format("[CCameraSensor::initialize] bumblebee camera: %u...\n", (unsigned int)( m_bumblebee_camera_index ) );
		m_cap_bumblebee = new CStereoGrabber_Bumblebee( m_bumblebee_camera_index, m_bumblebee_options );
	}
	else if(m_grabber_type=="svs")
	{
		cout << format("[CCameraSensor::initialize] SVS camera: %u...\n", (unsigned int)( m_svs_camera_index ) );
		m_cap_svs = new CStereoGrabber_SVS( m_svs_camera_index, m_svs_options );

	}
	else if (m_grabber_type=="ffmpeg")
	{
		//m_cap_ffmpeg
		cout << format("[CCameraSensor::initialize] FFmpeg stream: %s...\n", m_ffmpeg_url.c_str() );
		m_cap_ffmpeg = new CFFMPEG_InputStream();

		if (!m_cap_ffmpeg->openURL( m_ffmpeg_url, m_capture_grayscale ))
		{
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION_CUSTOM_MSG1("Error opening FFmpeg stream: %s", m_ffmpeg_url.c_str())
		}
	}
	else if (m_grabber_type=="swissranger")
	{
		cout << "[CCameraSensor::initialize] SwissRanger camera...\n";
		m_cap_swissranger = new CSwissRanger3DCamera();

		m_cap_swissranger->setOpenFromUSB( m_sr_open_from_usb );
		m_cap_swissranger->setOpenIPAddress(m_sr_ip_address);

		m_cap_swissranger->setSave3D(m_sr_save_3d);
		m_cap_swissranger->setSaveRangeImage(m_sr_save_range_img);
		m_cap_swissranger->setSaveIntensityImage(m_sr_save_intensity_img);
		m_cap_swissranger->setSaveConfidenceImage(m_sr_save_confidence);

		if (!m_path_for_external_images.empty())
			m_cap_swissranger->setPathForExternalImages( m_path_for_external_images );

		// Open it:
		try
		{
			m_cap_swissranger->initialize(); // This will launch an exception if needed.
		} catch (std::exception &e)
		{
			m_state = CGenericSensor::ssError;
			throw e;
		}
	}
	else if (m_grabber_type=="kinect")
	{
		cout << "[CCameraSensor::initialize] Kinect camera...\n";
		m_cap_kinect = new CKinect();

		m_cap_kinect->enableGrab3DPoints( m_kinect_save_3d );
		m_cap_kinect->enableGrabDepth ( m_kinect_save_range_img );
		m_cap_kinect->enableGrabRGB( m_kinect_save_intensity_img );
		m_cap_kinect->setVideoChannel( m_kinect_video_rgb ? CKinect::VIDEO_CHANNEL_RGB : CKinect::VIDEO_CHANNEL_IR );

		if (!m_path_for_external_images.empty())
			m_cap_kinect->setPathForExternalImages( m_path_for_external_images );

		// Open it:
		try
		{
			m_cap_kinect->initialize(); // This will launch an exception if needed.
		} catch (std::exception &e)
		{
			m_state = CGenericSensor::ssError;
			throw e;
		}
	}
	else if (m_grabber_type=="rawlog")
	{
		//m_cap_rawlog
		cout << format("[CCameraSensor::initialize] Rawlog stream: %s...\n", m_rawlog_file.c_str() );
		m_cap_rawlog = new CFileGZInputStream();

		if (! m_cap_rawlog->open(m_rawlog_file) )
		{
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION_CUSTOM_MSG1("Error opening rawlog file: %s", m_rawlog_file.c_str())
		}
		// File open OK.
		// Localize the external images directory of this rawlog, if it exists:
		m_rawlog_detected_images_dir = CRawlog::detectImagesDirectory(m_rawlog_file);
	}
	else
		THROW_EXCEPTION_CUSTOM_MSG1("Unknown 'grabber_type' found: %s", m_grabber_type.c_str() )

	// Change state:
	cout << "[CCameraSensor::initialize] Done!" << endl;
	m_state = CGenericSensor::ssWorking;


	// Launch independent thread?
	if (m_external_images_own_thread)
	{
		m_threadImagesSaverShouldEnd = false;

		m_threadImagesSaver.clear();
		m_threadImagesSaver.resize(m_external_image_saver_count);

		m_toSaveList.clear();
		m_toSaveList.resize(m_external_image_saver_count);

		for (unsigned int i=0;i<m_external_image_saver_count;++i)
			m_threadImagesSaver[i] = mrpt::system::createThreadFromObjectMethod(this,&CCameraSensor::thread_save_images, i);

	}

}

/* -----------------------------------------------------
                close
   ----------------------------------------------------- */
void CCameraSensor::close()
{
	delete_safe(m_cap_cv);
	delete_safe(m_cap_dc1394);
	delete_safe(m_cap_bumblebee);
	delete_safe(m_cap_ffmpeg);
	delete_safe(m_cap_rawlog);
	delete_safe(m_cap_swissranger);
	delete_safe(m_cap_kinect);
	delete_safe(m_cap_svs);

	m_state = CGenericSensor::ssInitializing;

	// Wait for threads:
	if (!m_threadImagesSaver.empty())
	{
		m_threadImagesSaverShouldEnd = true;
        for (size_t i=0;i<m_threadImagesSaver.size();i++)
            mrpt::system::joinThread( m_threadImagesSaver[i] );
	}
}

/* -----------------------------------------------------
                loadConfig_sensorSpecific
   ----------------------------------------------------- */
void  CCameraSensor::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
	// At this point, my parent class CGenericSensor has already loaded its params:
	//  Since cameras are special, we'll take control over "m_grab_decimation" so
	//  external image files are not saved just to be discarded later on...
	if (m_grab_decimation>0)
	{
		m_camera_grab_decimator = m_grab_decimation;
		m_camera_grab_decimator_counter = 0;
		// Reset in parent:
		m_grab_decimation = 0;
	}
	else
		m_camera_grab_decimator = m_camera_grab_decimator_counter = 0;

	m_grabber_type	= configSource.read_string_first_word( iniSection, "grabber_type", m_grabber_type );
	MRPT_LOAD_HERE_CONFIG_VAR( preview_decimation, int, m_preview_decimation , configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( preview_reduction, int, m_preview_reduction, configSource, iniSection )

	// OpenCV options:
	m_cv_camera_type= configSource.read_string_first_word( iniSection, "cv_camera_type", m_cv_camera_type );
	m_cv_camera_index = configSource.read_int( iniSection, "cv_camera_index", m_cv_camera_index);

	m_cv_options.frame_width 	= configSource.read_int( iniSection, "cv_frame_width", m_cv_options.frame_width );
	m_cv_options.frame_height	= configSource.read_int( iniSection, "cv_frame_height", m_cv_options.frame_height );
	m_cv_options.gain			= configSource.read_double( iniSection, "cv_gain", m_cv_options.gain );
	m_cv_options.ieee1394_fps	= configSource.read_double( iniSection, "cv_fps", m_cv_options.ieee1394_fps );

	m_capture_grayscale         = configSource.read_bool( iniSection, "capture_grayscale", false );

	m_cv_options.ieee1394_grayscale	= m_capture_grayscale;

	// dc1394 options:
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_camera_guid, uint64_t, m_dc1394_camera_guid, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_camera_unit, int, m_dc1394_camera_unit, configSource, iniSection )

	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_frame_width, int, m_dc1394_options.frame_width, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_frame_height, int, m_dc1394_options.frame_height, configSource, iniSection )

	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_mode7, int, m_dc1394_options.mode7, configSource, iniSection )

	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_shutter, int, m_dc1394_options.shutter, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_gain, int, m_dc1394_options.gain, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_gamma, int, m_dc1394_options.gamma, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_brightness, int, m_dc1394_options.brightness, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_exposure, int, m_dc1394_options.exposure, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_sharpness, int, m_dc1394_options.sharpness, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_white_balance, int, m_dc1394_options.white_balance, configSource, iniSection )

	// Bumblebee options:
	m_bumblebee_camera_index			= configSource.read_int( iniSection, "bumblebee_camera_index", m_bumblebee_camera_index );
	m_bumblebee_options.frame_width 	= configSource.read_int( iniSection, "bumblebee_frame_width", m_bumblebee_options.frame_width );
	m_bumblebee_options.frame_height	= configSource.read_int( iniSection, "bumblebee_frame_height", m_bumblebee_options.frame_height );
	m_bumblebee_options.color			= !m_capture_grayscale;
	m_bumblebee_monocam                 = configSource.read_int( iniSection, "bumblebee_mono", m_bumblebee_monocam );
	m_bumblebee_options.getRectified	= configSource.read_bool( iniSection, "bumblebee_get_rectified", m_bumblebee_options.getRectified );


	// SVS options:
	m_svs_camera_index                              = configSource.read_int( iniSection, "svs_camera_index", m_svs_camera_index );
	m_svs_options.frame_width                       = configSource.read_int( iniSection, "svs_frame_width", m_svs_options.frame_width );
	m_svs_options.frame_height                      = configSource.read_int( iniSection, "svs_frame_height", m_svs_options.frame_height );
	m_svs_options.framerate                         = configSource.read_double(iniSection,"svs_framerate",m_svs_options.framerate );
	m_svs_options.m_NDisp                           = configSource.read_int( iniSection, "svs_NDisp", m_svs_options.m_NDisp );
	m_svs_options.m_Corrsize                        = configSource.read_int( iniSection, "svs_Corrsize", m_svs_options.m_Corrsize );
	m_svs_options.m_LR                              = configSource.read_int( iniSection, "svs_LR", m_svs_options.m_LR );
	m_svs_options.m_Thresh                          = configSource.read_int( iniSection, "svs_Thresh", m_svs_options.m_Thresh );
	m_svs_options.m_Unique                          = configSource.read_int( iniSection, "svs_Unique", m_svs_options.m_Unique );
	m_svs_options.m_Horopter                        = configSource.read_int( iniSection, "svs_Horopter", m_svs_options.m_Horopter );
	m_svs_options.m_SpeckleSize                     = configSource.read_int( iniSection, "svs_SpeckleSize", m_svs_options.m_SpeckleSize );
        m_svs_options.m_procesOnChip                    = configSource.read_bool( iniSection, "svs_procesOnChip",m_svs_options.m_procesOnChip);
        m_svs_options.m_calDisparity                    = configSource.read_bool( iniSection, "svs_calDisparity",m_svs_options.m_calDisparity);

	// FFmpeg options:
	m_ffmpeg_url  = mrpt::system::trim( configSource.read_string( iniSection, "ffmpeg_url", m_ffmpeg_url ) );

	// Rawlog options:
	m_rawlog_file = mrpt::system::trim( configSource.read_string( iniSection, "rawlog_file", m_rawlog_file ) );
	m_rawlog_camera_sensor_label = mrpt::system::trim( configSource.read_string( iniSection, "rawlog_camera_sensor_label", m_rawlog_camera_sensor_label ) );

	// SwissRanger options:
	m_sr_open_from_usb = configSource.read_bool( iniSection, "sr_use_usb", m_sr_open_from_usb );
	m_sr_ip_address = configSource.read_string( iniSection, "sr_IP", m_sr_ip_address );

	m_sr_save_3d = configSource.read_bool( iniSection, "sr_grab_3d", m_sr_save_3d );
	m_sr_save_intensity_img = configSource.read_bool( iniSection, "sr_grab_grayscale", m_sr_save_intensity_img );
	m_sr_save_range_img = configSource.read_bool( iniSection, "sr_grab_range", m_sr_save_range_img );
	m_sr_save_confidence = configSource.read_bool( iniSection, "sr_grab_confidence", m_sr_save_confidence );

	m_kinect_save_3d = configSource.read_bool( iniSection, "kinect_grab_3d", m_kinect_save_3d );
	m_kinect_save_intensity_img = configSource.read_bool( iniSection, "kinect_grab_intensity", m_kinect_save_intensity_img );
	m_kinect_save_range_img = configSource.read_bool( iniSection, "kinect_grab_range", m_kinect_save_range_img );
	m_kinect_video_rgb = configSource.read_bool( iniSection, "kinect_video_rgb", m_kinect_video_rgb);
	

	// Special stuff: FPS
	map<double,grabber_dc1394_framerate_t>	map_fps;
	map<double,grabber_dc1394_framerate_t>::iterator it_fps;
	map_fps[1.875]	= FRAMERATE_1_875;
	map_fps[3.75]	= FRAMERATE_3_75;
	map_fps[7.5]	= FRAMERATE_7_5;
	map_fps[15]		= FRAMERATE_15;
	map_fps[30]		= FRAMERATE_30;
	map_fps[60]		= FRAMERATE_60;
	map_fps[120]	= FRAMERATE_120;
	map_fps[240]	= FRAMERATE_240;

	// ... for dc1394
	double the_fps = configSource.read_double( iniSection, "dc1394_framerate", 15.0 );
	it_fps = map_fps.find( the_fps );
	if ( it_fps == map_fps.end() )
		THROW_EXCEPTION_CUSTOM_MSG1("ERROR: DC1394 framerate seems to be not a valid number: %f",the_fps);

	m_dc1394_options.framerate =  it_fps->second;

	// ... for bumblebee
#if MRPT_HAS_BUMBLEBEE
	map<double,FlyCaptureFrameRate>	map_fps_bb;
	map<double,FlyCaptureFrameRate>::iterator it_fps_bb;
	map_fps_bb[1.875]	= FLYCAPTURE_FRAMERATE_1_875;
	map_fps_bb[3.75]	= FLYCAPTURE_FRAMERATE_3_75;
	map_fps_bb[7.5]		= FLYCAPTURE_FRAMERATE_7_5;
	map_fps_bb[15]		= FLYCAPTURE_FRAMERATE_15;
	map_fps_bb[30]		= FLYCAPTURE_FRAMERATE_30;
//	map_fps_bb[50]		= FLYCAPTURE_FRAMERATE_50;
	map_fps_bb[60]		= FLYCAPTURE_FRAMERATE_60;
	map_fps_bb[120]		= FLYCAPTURE_FRAMERATE_120;
	map_fps_bb[240]		= FLYCAPTURE_FRAMERATE_240;

	the_fps = configSource.read_double( iniSection, "bumblebee_fps", -1 );
	if (the_fps>0)
	{
		it_fps_bb = map_fps_bb.find( the_fps );
		if ( it_fps_bb == map_fps_bb.end() )
			THROW_EXCEPTION_CUSTOM_MSG1("ERROR: Bumblebee framerate seems to be not a valid number: %f",the_fps);

		m_bumblebee_options.framerate = it_fps_bb->second;
	}
#endif

	// Special stuff: color encoding:
	map<string,grabber_dc1394_color_coding_t>			map_color;
	map<string,grabber_dc1394_color_coding_t>::iterator it_color;
#define ADD_COLOR_MAP(c)  map_color[#c] = c;
	ADD_COLOR_MAP(COLOR_CODING_MONO8)
	ADD_COLOR_MAP(COLOR_CODING_YUV411)
	ADD_COLOR_MAP(COLOR_CODING_YUV422)
	ADD_COLOR_MAP(COLOR_CODING_YUV444)
	ADD_COLOR_MAP(COLOR_CODING_RGB8)
	ADD_COLOR_MAP(COLOR_CODING_MONO16)

	string the_color_coding = mrpt::system::upperCase( configSource.read_string_first_word( iniSection, "dc1394_color_coding", "COLOR_CODING_YUV422" ) );
	it_color = map_color.find(the_color_coding );
	if (it_color == map_color.end())
		THROW_EXCEPTION_CUSTOM_MSG1("ERROR: Color coding seems not to be valid : '%s'",the_color_coding.c_str() );
	m_dc1394_options.color_coding = it_color->second;


	m_external_images_format = mrpt::utils::trim( configSource.read_string( iniSection, "external_images_format", m_external_images_format ) );
	m_external_images_jpeg_quality = configSource.read_int( iniSection, "external_images_jpeg_quality", m_external_images_jpeg_quality );
	m_external_images_own_thread = configSource.read_bool( iniSection, "external_images_own_thread", m_external_images_own_thread );
	m_external_image_saver_count = configSource.read_int( iniSection, "external_images_own_thread_count", m_external_image_saver_count );

	// Sensor pose:
	m_sensorPose.setFromValues(
		configSource.read_float(iniSection,"pose_x",0),
		configSource.read_float(iniSection,"pose_y",0),
		configSource.read_float(iniSection,"pose_z",0),
		DEG2RAD( configSource.read_float(iniSection,"pose_yaw",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_pitch",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_roll",0) )
		);
}

/* -----------------------------------------------------
                Destructor
   ----------------------------------------------------- */
CCameraSensor::~CCameraSensor()
{
	close();

	m_preview_win1.clear();
	m_preview_win2.clear();
}

/* -----------------------------------------------------
				getNextFrame
----------------------------------------------------- */
CObservationPtr CCameraSensor::getNextFrame()
{
	CObservationImagePtr		obs;
	CObservationStereoImagesPtr	stObs;
	CObservation3DRangeScanPtr	obs3D;  // 3D range image, also with an intensity channel
	bool  capture_ok = false;

	if (m_cap_cv)
	{
		obs = CObservationImage::Create();
		if (!m_cap_cv->getObservation( *obs ))
		{	// Error
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error grabbing image");
		}
		else capture_ok = true;
	}
	else if (m_cap_dc1394)
	{
		obs = CObservationImage::Create();
		if (!m_cap_dc1394->getObservation( *obs ))
		{	// Error
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error grabbing image");
		}
		else capture_ok = true;
	}
	else if (m_cap_swissranger)
	{
		obs3D = CObservation3DRangeScan::Create();

		bool there_is_obs, hardware_error;
		m_cap_swissranger->getNextObservation(*obs3D, there_is_obs, hardware_error);

		if (!there_is_obs || hardware_error)
		{	// Error
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error grabbing image from SwissRanger camera.");
		}
		else capture_ok = true;
	}
	else if (m_cap_kinect)
	{
		obs3D = CObservation3DRangeScan::Create();

		// Specially at start-up, there may be a delay grabbing so a few calls return false: add a timeout.
		const mrpt::system::TTimeStamp t0 = mrpt::system::now();
		static const double max_timeout = 3.0; // seconds

		bool there_is_obs, hardware_error;
		do 
		{
			m_cap_kinect->getNextObservation(*obs3D, there_is_obs, hardware_error);
			if (!there_is_obs) mrpt::system::sleep(1);
		} while (!there_is_obs && mrpt::system::timeDifference(t0,mrpt::system::now())<max_timeout);

		if (!there_is_obs || hardware_error)
		{	// Error
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error grabbing image from Kinect camera.");
		}
		else capture_ok = true;
	}
	else if (m_cap_bumblebee)
	{
		stObs = CObservationStereoImages::Create();
		if (!m_cap_bumblebee->getStereoObservation( *stObs ))
		{	// Error
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error grabbing stereo images");
		}
		else
		{
			// Stereo or mono?
			if (m_bumblebee_monocam==0 || m_bumblebee_monocam==1)
			{
				obs = CObservationImage::Create();
				obs->timestamp = stObs->timestamp;
				obs->image.copyFastFrom( m_bumblebee_monocam==0 ? stObs->imageLeft : stObs->imageRight );
				stObs.clear();
			}
			capture_ok = true;
		}
	}
	else if (m_cap_svs)
	{
		stObs = CObservationStereoImages::Create();

		if(!m_cap_svs->getStereoObservation(*stObs))
		{
			// Error
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error grabbing disparity images");

		}
		else capture_ok = true;

	}
	else if (m_cap_ffmpeg)
	{
		obs = CObservationImage::Create();

		if (!m_cap_ffmpeg->retrieveFrame( obs->image ))
		{	// Error
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error grabbing image");
		}
		else capture_ok = true;
	}
	else if (m_cap_rawlog)
	{
		// Read in a loop until we found at least one image:
		//  Assign to: obs && stObs

		CSerializablePtr  newObs;
		while (!obs.present() && !stObs.present() && !obs3D.present())
		{
			*m_cap_rawlog  >> newObs;
			if (IS_DERIVED( newObs, CObservation) )
			{
				CObservationPtr o = CObservationPtr(newObs);
				if (!m_rawlog_camera_sensor_label.empty() && m_rawlog_camera_sensor_label!=o->sensorLabel)
					continue;

				if (IS_CLASS(o,CObservationImage))
					obs = CObservationImagePtr(o);
				else if (IS_CLASS(o,CObservationStereoImages))
					stObs = CObservationStereoImagesPtr(o);
				else if (IS_CLASS(o,CObservation3DRangeScan))
					obs3D = CObservation3DRangeScanPtr(o);
			}
			else if (IS_CLASS( newObs, CSensoryFrame) )
			{
				CSensoryFramePtr sf = CSensoryFramePtr(newObs);

				for (CSensoryFrame::iterator i=sf->begin();i!=sf->end();++i)
				{
					CObservationPtr &o = *i;

					if (!m_rawlog_camera_sensor_label.empty() && m_rawlog_camera_sensor_label!=o->sensorLabel)
						continue;

					if (IS_CLASS(o,CObservationImage))
					{
						obs = CObservationImagePtr(o);
						break;
					}
					else if (IS_CLASS(o,CObservationStereoImages))
					{
						stObs = CObservationStereoImagesPtr(o);
						break;
					}
					else if (IS_CLASS(o,CObservation3DRangeScan))
					{
						obs3D = CObservation3DRangeScanPtr(o);
						break;
					}
				}
			}
			if (obs || stObs || obs3D)
			{
				// We must convert externally stored images into "normal in-memory" images.
				const std::string old_dir = CImage::IMAGES_PATH_BASE; // Save current
				CImage::IMAGES_PATH_BASE = m_rawlog_detected_images_dir;

				if (obs && obs->image.isExternallyStored())
					obs->image.loadFromFile( obs->image.getExternalStorageFileAbsolutePath() );

				if (obs3D && obs3D->hasIntensityImage && obs3D->intensityImage.isExternallyStored())
					obs3D->intensityImage.loadFromFile( obs3D->intensityImage.getExternalStorageFileAbsolutePath() );

				if (stObs && stObs->imageLeft.isExternallyStored())
					stObs->imageLeft.loadFromFile( stObs->imageLeft.getExternalStorageFileAbsolutePath() );

				if (stObs && stObs->hasImageRight && stObs->imageRight.isExternallyStored())
					stObs->imageRight.loadFromFile( stObs->imageRight.getExternalStorageFileAbsolutePath() );

				if (stObs && stObs->hasImageDisparity && stObs->imageDisparity.isExternallyStored())
					stObs->imageDisparity.loadFromFile( stObs->imageDisparity.getExternalStorageFileAbsolutePath() );

				CImage::IMAGES_PATH_BASE = old_dir; // Restore
			}
			else continue;  // Keep reading
		}
		capture_ok = true;
	}
	else
	{
		THROW_EXCEPTION("There is no initialized camera driver: has 'initialize()' been called?")
	}

	ASSERT_(capture_ok)

	// Are we supposed to do a decimation??
	m_camera_grab_decimator_counter++;
	if (m_camera_grab_decimator_counter<m_camera_grab_decimator)
		// Done here:
		return CObservationPtr();

	// Continue as normal:
	m_camera_grab_decimator_counter = 0;

	ASSERT_(obs || stObs || obs3D)

	// If we grabbed an image: prepare it and add it to the internal queue:
	if (obs) {
		obs->sensorLabel = m_sensorLabel;
		obs->setSensorPose( m_sensorPose );
	}
	else if (stObs) {
		stObs->sensorLabel = m_sensorLabel;
		stObs->setSensorPose( m_sensorPose );
	}
	else {
		obs3D->sensorLabel = m_sensorLabel;
		obs3D->setSensorPose( m_sensorPose );
	}

	// Convert to grayscale if the user wants so and  the driver did ignored us:
	if (m_capture_grayscale)
	{
		if (obs)
		{
			if (obs->image.isColor())  obs->image.grayscaleInPlace();
		}
		else if (stObs)
		{
			if (stObs->imageLeft.isColor())   stObs->imageLeft.grayscaleInPlace();
			if (stObs->hasImageRight && stObs->imageRight.isColor())  stObs->imageRight.grayscaleInPlace();
			if (stObs->hasImageDisparity && stObs->imageDisparity.isColor())  stObs->imageDisparity.grayscaleInPlace();  // Shouldn't happen, but...
		}
		else if (obs3D)
		{
			if (obs3D->hasIntensityImage && obs3D->intensityImage.isColor()) obs3D->intensityImage.grayscaleInPlace();
		}
	}

	// External storage?
	bool delayed_insertion_in_obs_queue = false; // If true, we'll return nothing, but the observation will be inserted from the thread.

	if (!m_path_for_external_images.empty())
	{
		if( stObs )			// If we have grabbed an stereo observation ...
		{	// Stereo obs  -------
			if (m_external_images_own_thread)
			{
				m_csToSaveList.enter();

				// Select the "m_toSaveList" with the shortest pending queue:
				size_t idx_min = 0;
				for (size_t i=0;i<m_toSaveList.size();++i)
				    if (m_toSaveList[i].size()<m_toSaveList[idx_min].size())
                        idx_min = i;
                // Insert:
				m_toSaveList[idx_min].insert( TListObsPair( stObs->timestamp, stObs ) );

				m_csToSaveList.leave();

				delayed_insertion_in_obs_queue = true;
			}
			else
			{
				const string filNameL = fileNameStripInvalidChars( trim(m_sensorLabel) ) + format( "_L_%f.%s", (double)timestampTotime_t( stObs->timestamp ), m_external_images_format.c_str() );
				const string filNameR = fileNameStripInvalidChars( trim(m_sensorLabel) ) + format( "_R_%f.%s", (double)timestampTotime_t( stObs->timestamp ), m_external_images_format.c_str() );
				const string filNameD = fileNameStripInvalidChars( trim(m_sensorLabel) ) + format( "_D_%f.%s", (double)timestampTotime_t( stObs->timestamp ), m_external_images_format.c_str() );
				//cout << "[CCameraSensor] Saving " << filName << endl;
				stObs->imageLeft.saveToFile( m_path_for_external_images + string("/") + filNameL, m_external_images_jpeg_quality );
				stObs->imageLeft.setExternalStorage( filNameL );

				if (stObs->hasImageRight) {
					stObs->imageRight.saveToFile( m_path_for_external_images + string("/") + filNameR, m_external_images_jpeg_quality );
					stObs->imageRight.setExternalStorage( filNameR );
				}
				if (stObs->hasImageDisparity) {
					stObs->imageDisparity.saveToFile( m_path_for_external_images + string("/") + filNameD, m_external_images_jpeg_quality );
					stObs->imageDisparity.setExternalStorage( filNameD );
				}
			}
		}
		else if (obs)
		{	// Monocular image obs  -------
			if (m_external_images_own_thread)
			{
				m_csToSaveList.enter();

				// Select the "m_toSaveList" with the shortest pending queue:
				size_t idx_min = 0;
				for (size_t i=0;i<m_toSaveList.size();++i)
				    if (m_toSaveList[i].size()<m_toSaveList[idx_min].size())
                        idx_min = i;

                // Insert:
				m_toSaveList[idx_min].insert( TListObsPair( obs->timestamp, obs ) );

				m_csToSaveList.leave();
				delayed_insertion_in_obs_queue = true;
			}
			else
			{
				string filName = fileNameStripInvalidChars( trim(m_sensorLabel) ) + format( "_%f.%s", (double)timestampTotime_t( obs->timestamp ), m_external_images_format.c_str() );
				//cout << "[CCameraSensor] Saving " << filName << endl;
				obs->image.saveToFile( m_path_for_external_images + string("/") +filName, m_external_images_jpeg_quality );
				obs->image.setExternalStorage( filName );
			}
		} // end else
	}

	// Show preview??
	if (m_preview_decimation>0)
	{	// Yes
		if (++m_preview_counter > m_preview_decimation)
		{
			m_preview_counter = 0;

			// Create window the first time:
			if (!m_preview_win1)
			{
				string caption = string("Preview of ")+m_sensorLabel;
				if (stObs) caption+="-LEFT";
				if (m_preview_decimation>1)
					caption += format(" (decimation: %i)",m_preview_decimation);
				m_preview_win1 = mrpt::gui::CDisplayWindow::Create(caption);
			}
			if (stObs && !m_preview_win2)
			{
				string caption = string("Preview of ")+m_sensorLabel;
				if (stObs) caption+="-RIGHT";
				if (m_preview_decimation>1)
					caption += format(" (decimation: %i)",m_preview_decimation);
				m_preview_win2 = mrpt::gui::CDisplayWindow::Create(caption);
			}
			// Monocular image or Left from a stereo pair:
			if (m_preview_win1->isOpen())
			{
				CImage *img;
				if (stObs)
					img = &stObs->imageLeft;
				else if (obs)
					img = &obs->image;
				else
					img = &obs3D->intensityImage;

				// Apply image reduction?
				if (m_preview_reduction>=2)
				{
					unsigned int w = img->getWidth();
					unsigned int h = img->getHeight();
					CImage auxImg;
					img->scaleImage(auxImg, w/m_preview_reduction, h/m_preview_reduction, IMG_INTERP_NN);
					m_preview_win1->showImage(auxImg);
				}
				else
					m_preview_win1->showImage(*img);
			}

			// Right from a stereo pair:
			if (m_preview_win2 && m_preview_win2->isOpen() && stObs && stObs->hasImageRight)
			{
				// Apply image reduction?
				if (m_preview_reduction>=2)
				{
					unsigned int w = stObs->imageRight.getWidth();
					unsigned int h = stObs->imageRight.getHeight();
					CImage auxImg;
					stObs->imageRight.scaleImage(auxImg, w/m_preview_reduction, h/m_preview_reduction, IMG_INTERP_NN);
					m_preview_win2->showImage(auxImg);
				}
				else
					m_preview_win2->showImage(stObs->imageRight);
			}

			// Disparity from a stereo pair:
			if (m_preview_win2 && m_preview_win2->isOpen() && stObs && stObs->hasImageDisparity)
			{
				// Apply image reduction?
				if (m_preview_reduction>=2)
				{
					unsigned int w = stObs->imageDisparity.getWidth();
					unsigned int h = stObs->imageDisparity.getHeight();
					CImage auxImg;
					stObs->imageDisparity.scaleImage(auxImg, w/m_preview_reduction, h/m_preview_reduction, IMG_INTERP_NN);
					m_preview_win2->showImage(auxImg);
				}
				else
					m_preview_win2->showImage(stObs->imageDisparity);
			}
		}
	} // end show preview

	if (delayed_insertion_in_obs_queue)
		return CObservationPtr();
	else
		return stObs ? CObservationPtr(stObs) : (obs ? CObservationPtr(obs) : CObservationPtr(obs3D));
}



/* -----------------------------------------------------
				doProcess
----------------------------------------------------- */
void  CCameraSensor::doProcess()
{
	CObservationPtr obs = getNextFrame();
	if (obs)
		appendObservation(obs);
}

/* -----------------------------------------------------
				setPathForExternalImages
----------------------------------------------------- */
void CCameraSensor::setPathForExternalImages( const std::string &directory )
{
	if (!mrpt::system::createDirectory( directory ))
	{
		THROW_EXCEPTION_CUSTOM_MSG1("Error: Cannot create the directory for externally saved images: %s",directory.c_str() )
	}
	m_path_for_external_images = directory;
}

/* ------------------------------------------------------------------------
						prepareVideoSourceFromUserSelection
   ------------------------------------------------------------------------ */
CCameraSensorPtr mrpt::hwdrivers::prepareVideoSourceFromUserSelection()
{
#if MRPT_HAS_WXWIDGETS
	// Create the main wxThread, if it doesn't exist yet:
	if (!mrpt::gui::WxSubsystem::createOneInstanceMainThread() )
	{
		std::cerr << "[mrpt::hwdrivers::prepareVideoSourceFromUserSelection] Error initiating Wx subsystem." << std::endl;
		return CCameraSensorPtr(); // Error!
	}

	mrpt::synch::CSemaphore  semDlg(0,10);
	mrpt::gui::detail::TReturnAskUserOpenCamera dlgSelection;

    // Create window:
    WxSubsystem::TRequestToWxMainThread  *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->OPCODE   = 700;
    REQ->sourceCameraSelectDialog = true;
	REQ->voidPtr = reinterpret_cast<void*>(&semDlg);
	REQ->voidPtr2 = reinterpret_cast<void*>(&dlgSelection);
    WxSubsystem::pushPendingWxRequest( REQ );

    // Wait for the window to realize and signal it's alive:
    if (!WxSubsystem::isConsoleApp)
    {
    	mrpt::system::sleep(20);	// Force at least 1-2 timer ticks for processing the event:
    	wxApp::GetInstance()->Yield(true);
    }

    // wait for window construction:
	int maxTimeout =
#ifdef _DEBUG
		30000;
#else
		6000;
#endif
	if(!semDlg.waitForSignal(maxTimeout))
	{
		cerr << "[prepareVideoSourceFromUserSelection] Timeout waiting window creation." << endl;
		return CCameraSensorPtr();
	}

    // wait for user selection:
   	if(!semDlg.waitForSignal())
	{
		cerr << "[prepareVideoSourceFromUserSelection] Timeout waiting user selection." << endl;
		return CCameraSensorPtr();
	}

	// If the user didn't accept the dialog, return now:
	if (!dlgSelection.accepted_by_user)
		return CCameraSensorPtr();

	CCameraSensorPtr cam = CCameraSensorPtr(new CCameraSensor);
	cam->loadConfig(dlgSelection.selectedConfig,"CONFIG");
	cam->initialize();	// This will raise an exception if neccesary

	return cam;
#else
	THROW_EXCEPTION("MRPT compiled without wxWidgets")
#endif // MRPT_HAS_WXWIDGETS
}

/* ------------------------------------------------------------------------
						prepareVideoSourceFromPanel
   ------------------------------------------------------------------------ */
CCameraSensorPtr mrpt::hwdrivers::prepareVideoSourceFromPanel(void *_panel)
{
#if MRPT_HAS_WXWIDGETS

	try
	{
		CConfigFileMemory	cfg;
		writeConfigFromVideoSourcePanel(_panel,"CONFIG",&cfg);

		// Try to open the camera:
		CCameraSensorPtr video = CCameraSensorPtr( new CCameraSensor());
		video->loadConfig(cfg,"CONFIG");

		// This will raise an exception if neccesary
		video->initialize();

		return video;
	}
	catch(std::exception &e)
	{
		cerr << endl << e.what() << endl;
		wxMessageBox(_("Couldn't open video source"),_("Error"));
		return CCameraSensorPtr();
	}
#else
	THROW_EXCEPTION("MRPT compiled without wxWidgets")
#endif // MRPT_HAS_WXWIDGETS
}

/* ------------------------------------------------------------------------
						writeConfigFromVideoSourcePanel
   ------------------------------------------------------------------------ */
void mrpt::hwdrivers::writeConfigFromVideoSourcePanel(
	void *_panel,
	const std::string &sect,
	mrpt::utils::CConfigFileBase *cfg
	)
{
	MRPT_START
#if MRPT_HAS_WXWIDGETS
	ASSERT_(_panel)
	mrpt::gui::CPanelCameraSelection *panel = reinterpret_cast<mrpt::gui::CPanelCameraSelection *>(_panel);
	ASSERTMSG_(panel,"panel must be of type mrpt::gui::CPanelCameraSelection *")

	panel->writeConfigFromVideoSourcePanel(sect,cfg);

#else
	THROW_EXCEPTION("MRPT compiled without wxWidgets")
#endif // MRPT_HAS_WXWIDGETS
	MRPT_END
}

/* ------------------------------------------------------------------------
						readConfigIntoVideoSourcePanel
   ------------------------------------------------------------------------ */
void mrpt::hwdrivers::readConfigIntoVideoSourcePanel(
	void *_panel,
	const std::string &sect,
	const mrpt::utils::CConfigFileBase *cfg
	)
{
	MRPT_START
#if MRPT_HAS_WXWIDGETS
	ASSERT_(_panel)
	mrpt::gui::CPanelCameraSelection *panel = reinterpret_cast<mrpt::gui::CPanelCameraSelection *>(_panel);
	ASSERTMSG_(panel,"panel must be of type mrpt::gui::CPanelCameraSelection *")

	panel->readConfigIntoVideoSourcePanel(sect,cfg);

#else
	THROW_EXCEPTION("MRPT compiled without wxWidgets")
#endif // MRPT_HAS_WXWIDGETS
	MRPT_END
}


/* -----------------------------------------------------
		THREAD: Saver of external images
   ----------------------------------------------------- */
void CCameraSensor::thread_save_images(unsigned int my_working_thread_index)
{
	while (!m_threadImagesSaverShouldEnd)
	{
		TListObservations newObs;

		// is there any new image?
		m_csToSaveList.enter();
		m_toSaveList[my_working_thread_index].swap(newObs);
		m_csToSaveList.leave();

		for (TListObservations::const_iterator i=newObs.begin();i!=newObs.end();i++)
		{
			if (IS_CLASS(i->second, CObservationImage))
			{
				CObservationImagePtr obs = CObservationImagePtr(i->second);

				string filName = fileNameStripInvalidChars( trim(m_sensorLabel) ) + format( "_%f.%s", (double)timestampTotime_t( obs->timestamp ), m_external_images_format.c_str() );

				obs->image.saveToFile( m_path_for_external_images + string("/") +filName, m_external_images_jpeg_quality );
				obs->image.setExternalStorage( filName );
			}
			else if (IS_CLASS(i->second, CObservationStereoImages))
			{
				CObservationStereoImagesPtr stObs = CObservationStereoImagesPtr(i->second);

				const string filNameL = fileNameStripInvalidChars( trim(m_sensorLabel) ) + format( "_L_%f.%s", (double)timestampTotime_t( stObs->timestamp ), m_external_images_format.c_str() );
				const string filNameR = fileNameStripInvalidChars( trim(m_sensorLabel) ) + format( "_R_%f.%s", (double)timestampTotime_t( stObs->timestamp ), m_external_images_format.c_str() );
				const string filNameD = fileNameStripInvalidChars( trim(m_sensorLabel) ) + format( "_D_%f.%s", (double)timestampTotime_t( stObs->timestamp ), m_external_images_format.c_str() );

				stObs->imageLeft.saveToFile( m_path_for_external_images + string("/") + filNameL, m_external_images_jpeg_quality );
				stObs->imageLeft.setExternalStorage( filNameL );

				if (stObs->hasImageRight) {
					stObs->imageRight.saveToFile( m_path_for_external_images + string("/") + filNameR, m_external_images_jpeg_quality );
					stObs->imageRight.setExternalStorage( filNameR );
				}
				if (stObs->hasImageDisparity) {
					stObs->imageDisparity.saveToFile( m_path_for_external_images + string("/") + filNameD, m_external_images_jpeg_quality );
					stObs->imageDisparity.setExternalStorage( filNameD );
				}
			}

			// Append now:
			appendObservation(i->second);
		}

		mrpt::system::sleep(2);
	}
}
