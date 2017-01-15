/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CTypeSelector.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/gui/WxSubsystem.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CCameraSensor,mrpt::hwdrivers)

/* -----------------------------------------------------
                Constructor
   ----------------------------------------------------- */
CCameraSensor::CCameraSensor() :
	mrpt::utils::COutputLogger("CCameraSensor"),
	m_sensorPose			(),
	m_grabber_type			("opencv"),
	m_capture_grayscale		(false),
	m_cv_camera_index		(0),
	m_cv_camera_type		("CAMERA_CV_AUTODETECT"),
	m_cv_options			(),
	// ---
	m_dc1394_camera_guid	(0),
	m_dc1394_camera_unit	(0),
	m_dc1394_options		(),
	m_preview_decimation	(0),
	m_preview_reduction		(1),
	// ---
	m_bumblebee_dc1394_camera_guid(0),
	m_bumblebee_dc1394_camera_unit(0),
	m_bumblebee_dc1394_framerate(15),
	// ---
	m_svs_camera_index(0),
	m_svs_options                   (),
	// ---
	m_sr_open_from_usb		(true),
	m_sr_save_3d			(true),
	m_sr_save_range_img		(true),
	m_sr_save_intensity_img	(true),
	m_sr_save_confidence	(true),
	// ---
	m_kinect_save_3d		(true), // These options are also used for OpenNI2 grabber
	m_kinect_save_range_img (true),
	m_kinect_save_intensity_img(true),
	m_kinect_video_rgb		(true),
	// ---
	m_fcs_start_synch_capture(false),
	// ---
	m_img_dir_url			(""),
	m_img_dir_left_format	("imL_%04d.jpg"),
	m_img_dir_right_format	("imR_%04d.jpg"),
	m_img_dir_start_index	(0),
	m_img_dir_end_index		(100),
	m_img_dir_is_stereo		(true),
	m_img_dir_counter		(0),
	// ---
	m_external_images_own_thread(false),
	m_cap_cv             (NULL),
	m_cap_dc1394         (NULL),
	m_cap_flycap         (NULL),
	m_cap_flycap_stereo_l(NULL),
	m_cap_flycap_stereo_r(NULL),
	m_cap_bumblebee_dc1394(NULL),
	m_cap_svs            (NULL),
	m_cap_ffmpeg         (NULL),
	m_cap_rawlog         (NULL),
	m_cap_swissranger    (NULL),
	m_cap_kinect         (NULL),
	m_cap_openni2        (NULL),
	m_cap_image_dir		(NULL),
	m_cap_duo3d			(NULL),
	m_camera_grab_decimator (0),
	m_camera_grab_decimator_counter(0),
	m_preview_counter	(0),
	m_external_image_saver_count( mrpt::system::getNumberOfProcessors() ),
	m_threadImagesSaverShouldEnd(false),
	m_hook_pre_save      (NULL),
	m_hook_pre_save_param(NULL)
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
	else if (m_grabber_type=="bumblebee_dc1394")
	{
		cout << format("[CCameraSensor::initialize] bumblebee_libdc1394 camera: GUID:0x%08X Index:%i FPS:%f...\n", (unsigned int)( m_bumblebee_dc1394_camera_guid ), m_bumblebee_dc1394_camera_unit, m_bumblebee_dc1394_framerate );
		m_cap_bumblebee_dc1394 = new CStereoGrabber_Bumblebee_libdc1394( m_bumblebee_dc1394_camera_guid,m_bumblebee_dc1394_camera_unit, m_bumblebee_dc1394_framerate );
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
		} catch (std::exception &)
		{
			m_state = CGenericSensor::ssError;
			throw;
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
		} catch (std::exception &)
		{
			m_state = CGenericSensor::ssError;
			throw;
		}
	}
	else if (m_grabber_type=="openni2")
	{
		cout << "[CCameraSensor::initialize] OpenNI2 sensor...\n";
        m_cap_openni2 = new COpenNI2Sensor();
		m_cap_openni2->enableGrab3DPoints( m_kinect_save_3d ); // It uses the same options as the Kinect grabber
		m_cap_openni2->enableGrabDepth ( m_kinect_save_range_img );
		m_cap_openni2->enableGrabRGB( m_kinect_save_intensity_img );

		if (!m_path_for_external_images.empty())
			m_cap_openni2->setPathForExternalImages( m_path_for_external_images );

		// Open it:
		try
		{
			m_cap_openni2->initialize(); // This will launch an exception if needed.
		} catch (std::exception &e)
		{
			m_state = CGenericSensor::ssError;
			throw e;
		}
	}
	else if (m_grabber_type=="image_dir" )
	{
		//m_cap_image_dir
		cout << format("[CCameraSensor::initialize] Image dir: %s...\n", m_img_dir_url.c_str() );
		m_cap_image_dir = new std::string();
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
	else if (m_grabber_type=="flycap")
	{
		cout << "[CCameraSensor::initialize] PGR FlyCapture2 camera...\n";
		try
		{
			// Open camera and start capture:
			m_cap_flycap = new CImageGrabber_FlyCapture2( m_flycap_options );
		} catch (std::exception &)
		{
			m_state = CGenericSensor::ssError;
			throw;
		}
	}
	else if (m_grabber_type=="flycap_stereo")
	{
		cout << "[CCameraSensor::initialize] PGR FlyCapture2 stereo camera...\n";
		try
		{
			// Open camera and start capture:
			m_cap_flycap_stereo_l = new CImageGrabber_FlyCapture2();
			m_cap_flycap_stereo_r = new CImageGrabber_FlyCapture2();

			cout << "[CCameraSensor::initialize] PGR FlyCapture2 stereo camera: Openning LEFT camera...\n";
			m_cap_flycap_stereo_l->open(m_flycap_stereo_options[0], false /* don't start grabbing */ );

			cout << "[CCameraSensor::initialize] PGR FlyCapture2 stereo camera: Openning RIGHT camera...\n";
			m_cap_flycap_stereo_r->open(m_flycap_stereo_options[1], false /* don't start grabbing */ );

			// Now, start grabbing "simultaneously":
			if (m_fcs_start_synch_capture)
			{
				const CImageGrabber_FlyCapture2 *cams[2];
				cams[0] = m_cap_flycap_stereo_l;
				cams[1] = m_cap_flycap_stereo_r;
				CImageGrabber_FlyCapture2::startSyncCapture(2,cams);
			}
			else
			{
				m_cap_flycap_stereo_l->startCapture();
				m_cap_flycap_stereo_r->startCapture();
			}

		} catch (std::exception &)
		{
			m_state = CGenericSensor::ssError;
			throw;
		}
	}
	else if (m_grabber_type=="duo3d")
	{
		//m_cap_duo3D
		cout << format("[CCameraSensor::initialize] DUO3D stereo camera ...\n" );

		// Open it:
		try
		{
			m_cap_duo3d = new CDUO3DCamera( m_duo3d_options );		
		} catch (std::exception &e)
		{
			m_state = CGenericSensor::ssError;
			throw e;
		}
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
	delete_safe(m_cap_flycap);
	delete_safe(m_cap_flycap_stereo_l);
	delete_safe(m_cap_flycap_stereo_r);
	delete_safe(m_cap_bumblebee_dc1394);
	delete_safe(m_cap_ffmpeg);
	delete_safe(m_cap_rawlog);
	delete_safe(m_cap_swissranger);
	delete_safe(m_cap_kinect);
	delete_safe(m_cap_svs);
	delete_safe(m_cap_image_dir);
	delete_safe(m_cap_duo3d);

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

	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_shutter_mode, int, m_dc1394_options.shutter_mode, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_gain_mode, int, m_dc1394_options.gain_mode, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_gamma_mode, int, m_dc1394_options.gamma_mode, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_brightness_mode, int, m_dc1394_options.brightness_mode, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_exposure_mode, int, m_dc1394_options.exposure_mode, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_sharpness_mode, int, m_dc1394_options.sharpness_mode, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_white_balance_mode, int, m_dc1394_options.white_balance_mode, configSource, iniSection )

	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_trigger_power, int, m_dc1394_options.trigger_power, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_trigger_mode, int, m_dc1394_options.trigger_mode, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_trigger_source, int, m_dc1394_options.trigger_source, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_trigger_polarity, int, m_dc1394_options.trigger_polarity, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( dc1394_ring_buffer_size, int, m_dc1394_options.ring_buffer_size, configSource, iniSection )

	// Bumblebee_dc1394 options:
	MRPT_LOAD_HERE_CONFIG_VAR( bumblebee_dc1394_camera_guid, uint64_t, m_bumblebee_dc1394_camera_guid, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( bumblebee_dc1394_camera_unit, int, m_bumblebee_dc1394_camera_unit, configSource, iniSection )
	MRPT_LOAD_HERE_CONFIG_VAR( bumblebee_dc1394_framerate, double, m_bumblebee_dc1394_framerate, configSource, iniSection )

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

	// Image directory options:
	m_img_dir_url			=  mrpt::system::trim( configSource.read_string( iniSection, "image_dir_url", m_img_dir_url ) );
	m_img_dir_left_format	= mrpt::system::trim( configSource.read_string( iniSection, "left_format", m_img_dir_left_format ) );
	m_img_dir_right_format	= mrpt::system::trim( configSource.read_string( iniSection, "right_format", "" ) );
	m_img_dir_start_index	= configSource.read_int( iniSection, "start_index", m_img_dir_start_index );;
	m_img_dir_end_index		= configSource.read_int( iniSection, "end_index", m_img_dir_end_index );

	m_img_dir_is_stereo = !m_img_dir_right_format.empty();
	m_img_dir_counter   = m_img_dir_start_index;

	// DUO3D Camera options:
	m_duo3d_options.loadOptionsFrom( configSource, "DUO3DOptions" );

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

	// FlyCap:
	m_flycap_options.loadOptionsFrom(configSource,iniSection, "flycap_");

	// FlyCap stereo
	m_fcs_start_synch_capture = configSource.read_bool( iniSection, "fcs_start_synch_capture", m_fcs_start_synch_capture );
	m_flycap_stereo_options[0].loadOptionsFrom(configSource,iniSection, "fcs_LEFT_");
	m_flycap_stereo_options[1].loadOptionsFrom(configSource,iniSection, "fcs_RIGHT_");

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


	m_external_images_format = mrpt::system::trim( configSource.read_string( iniSection, "external_images_format", m_external_images_format ) );
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
CObservationPtr CCameraSensor::getNextFrame( )
{
	vector<CSerializablePtr> out_obs;
	getNextFrame(out_obs);
	return static_cast<CObservationPtr>(out_obs[0]);
}

/* -----------------------------------------------------
				getNextFrame
----------------------------------------------------- */
void CCameraSensor::getNextFrame( vector<CSerializablePtr> & out_obs )
{
	CObservationImagePtr		obs;
	CObservationStereoImagesPtr	stObs;
	CObservation3DRangeScanPtr	obs3D;		// 3D range image, also with an intensity channel
	CObservationIMUPtr			obsIMU;		// IMU observation grabbed by DUO3D cameras

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
		double max_timeout = 3.0; // seconds

		// If we have an "MRPT_CCAMERA_KINECT_TIMEOUT_MS" environment variable, use that timeout instead:
		const char *envVal = getenv("MRPT_CCAMERA_KINECT_TIMEOUT_MS");
		if (envVal) max_timeout = atoi(envVal)*0.001;

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
	else if (m_cap_openni2)
	{
		obs3D = CObservation3DRangeScan::Create();
		// Specially at start-up, there may be a delay grabbing so a few calls return false: add a timeout.
		const mrpt::system::TTimeStamp t0 = mrpt::system::now();
		double max_timeout = 3.0; // seconds
		bool there_is_obs, hardware_error;
		do
		{
			m_cap_openni2->getNextObservation(*obs3D, there_is_obs, hardware_error);
			if (!there_is_obs) mrpt::system::sleep(1);
		} while (!there_is_obs && mrpt::system::timeDifference(t0,mrpt::system::now())<max_timeout);

		if (!there_is_obs || hardware_error)
		{	// Error
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error grabbing image from OpenNI2 camera.");
		}
		else capture_ok = true;
	}
	else if (m_cap_bumblebee_dc1394)
	{
		stObs = CObservationStereoImages::Create();
		if (!m_cap_bumblebee_dc1394->getStereoObservation( *stObs )) {
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error grabbing stereo images");
		}
		else {
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
	else if (m_cap_image_dir)
	{
		if( m_img_dir_counter > m_img_dir_end_index )
		{
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Reached end index.");
		}

		std::string auxL = format( "%s/%s", m_img_dir_url.c_str(), m_img_dir_left_format.c_str() );
		if( m_img_dir_is_stereo )
		{

			stObs = CObservationStereoImages::Create();
			if( !stObs->imageLeft.loadFromFile( format(auxL.c_str(), m_img_dir_counter) ) )
			{
				m_state = CGenericSensor::ssError;
				THROW_EXCEPTION("Error reading images from directory");
			}
			std::string auxR = format( "%s/%s", m_img_dir_url.c_str(), m_img_dir_right_format.c_str() );			
			if( !stObs->imageRight.loadFromFile( format(auxR.c_str(), m_img_dir_counter++)) )
			{
				m_state = CGenericSensor::ssError;
				THROW_EXCEPTION("Error reading images from directory");
			}
			else capture_ok = true;
		}
		else
		{
			// use only left image prefix
			obs = CObservationImage::Create();
			if( !obs->image.loadFromFile( format(auxL.c_str(), m_img_dir_counter++) ) )
			{
				m_state = CGenericSensor::ssError;
				THROW_EXCEPTION("Error reading images from directory");
			}
			else capture_ok = true;
		}
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
	else if (m_cap_flycap)
	{
		bool ok;
		if (!m_cap_flycap->isStereo()) // Mono image
		{
			obs = CObservationImage::Create();
			ok = m_cap_flycap->getObservation(*obs);
		}
		else // Stereo camera connected
		{
			stObs = CObservationStereoImages::Create();
			ok = m_cap_flycap->getObservation(*stObs);
		}

		if (!ok)
		{	// Error
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error grabbing image");
		}
		else capture_ok = true;
	}
	else if (m_cap_flycap_stereo_l && m_cap_flycap_stereo_r)
	{
		stObs = CObservationStereoImages::Create();

		CObservationImage obsL,obsR;

		bool ok1, ok2=false;

		ok1 = m_cap_flycap_stereo_r->getObservation(obsL);
		if (ok1)
			ok2 = m_cap_flycap_stereo_l->getObservation(obsR);

		if (!ok1 || !ok2)
		{
			// Error
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error grabbing disparity images");
		}
		else
		{
			// Joint the two images as one stereo:
			const double At = mrpt::system::timeDifference(obsL.timestamp,obsR.timestamp);
			if (std::abs(At)>0.1) {
				cout << "[CCamera, flycap_stereo] Warning: Too large delay between left & right images: " << At << " sec.\n";
			}

			// It seems that the timestamp is not always filled in from FlyCap driver?
			stObs->timestamp = (obsL.timestamp!=0) ? obsL.timestamp : mrpt::system::now();
			stObs->imageLeft.copyFastFrom(obsL.image);
			stObs->imageRight.copyFastFrom(obsR.image);
			capture_ok = true;
		}
	}
	else if( m_cap_duo3d )
	{
		stObs = CObservationStereoImages::Create();
		obsIMU = CObservationIMU::Create();

		bool thereIsIMG, thereIsIMU;
		m_cap_duo3d->getObservations(*stObs,*obsIMU,thereIsIMG,thereIsIMU);
		if( !thereIsIMG )
		{
			m_state = CGenericSensor::ssError;
			THROW_EXCEPTION("Error getting observations from DUO3D camera.");
		}
		else if( m_cap_duo3d->captureIMUIsSet() && !thereIsIMU )
		{
			cout << "[CCamera, duo3d] Warning: There are no IMU data from the device. Only images are being grabbed.";
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
	{
		// Done here:
		out_obs.push_back( CObservationPtr() );
		return;
	}
	// Continue as normal:
	m_camera_grab_decimator_counter = 0;

	ASSERT_(obs || stObs || obs3D || obsIMU)
	// If we grabbed an image: prepare it and add it to the internal queue:
	if (obs) {
		obs->sensorLabel = m_sensorLabel;
		obs->setSensorPose( m_sensorPose );
	}
	else if (stObs) {
		stObs->sensorLabel = (m_cap_duo3d && m_cap_duo3d->captureIMUIsSet()) ? m_sensorLabel + "_IMG" : m_sensorLabel;
		stObs->setSensorPose( m_sensorPose );
	}
	else if (obs3D) {
		obs3D->sensorLabel = m_sensorLabel;
		obs3D->setSensorPose( m_sensorPose );
	}
	if (obsIMU)	{
		obsIMU->sensorLabel = m_sensorLabel + "_IMU";
		obsIMU->setSensorPose( m_sensorPose );
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
	{
		if( m_cap_duo3d && m_cap_duo3d->captureIMUIsSet() && obsIMU )	
			out_obs.push_back( CObservationPtr(obsIMU) );
	}
	else
	{
		if( stObs )		out_obs.push_back( CObservationPtr(stObs) );
		if( obs )		out_obs.push_back( CObservationPtr(obs) );
		if( obs3D )		out_obs.push_back( CObservationPtr(obs3D) );
	} 
	return;
}

/* -----------------------------------------------------
				doProcess
----------------------------------------------------- */
void  CCameraSensor::doProcess()
{
	vector<CSerializablePtr> out_obs;
	getNextFrame(out_obs);
	appendObservations(out_obs);
}

/* -----------------------------------------------------
                setSoftwareTriggerLevel
----------------------------------------------------- */
void CCameraSensor::setSoftwareTriggerLevel( bool level )
{
    if (m_cap_dc1394)
    {
        if (!m_cap_dc1394->setSoftwareTriggerLevel(level))
        {	// Error
            m_state = CGenericSensor::ssError;
            THROW_EXCEPTION("Error setting Trigger level by software");
        }
    }
    else
    {
        THROW_EXCEPTION("Software trigger is not implemented for this camera type")
    }
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
	// If we have an "MRPT_WXSUBSYS_TIMEOUT_MS" environment variable, use that timeout instead:
	const char *envVal = getenv("MRPT_WXSUBSYS_TIMEOUT_MS");
	if (envVal) maxTimeout = atoi(envVal);

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

		for (TListObservations::const_iterator i=newObs.begin();i!=newObs.end();++i)
		{
			// Optional user-code hook:
			if (m_hook_pre_save)
			{
				if (IS_DERIVED(i->second, CObservation))
				{
					mrpt::obs::CObservationPtr obs = mrpt::obs::CObservationPtr(i->second);
					(*m_hook_pre_save)(obs,m_hook_pre_save_param);
				}
			}

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
