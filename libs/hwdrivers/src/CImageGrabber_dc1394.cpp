/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/config.h>
#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>

// Include the libdc1394-2 headers:
#if MRPT_HAS_LIBDC1394_2
#	include <dc1394/control.h>
#	include <dc1394/conversions.h>
#	include <dc1394/utils.h>
#	include <dc1394/register.h>
#endif

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;


#define THE_CAMERA   static_cast<dc1394camera_t*>(m_dc1394camera)
#define THE_CONTEXT  static_cast<dc1394_t*>(m_dc1394_lib_context)

/*-------------------------------------------------------------
					Constructor
 -------------------------------------------------------------*/
CImageGrabber_dc1394::CImageGrabber_dc1394(
	uint64_t	cameraGUID,
	uint16_t	cameraUnit,
	const TCaptureOptions_dc1394 &options,
	bool  verbose ) :
		m_bInitialized 		(false),
		m_dc1394_lib_context(NULL),
		m_dc1394camera		(NULL),
		m_options			(options)
{
	MRPT_START

#if MRPT_HAS_LIBDC1394_2
	// Open lib:
	m_dc1394_lib_context = dc1394_new ();
	ASSERT_( m_dc1394_lib_context );

	// Enumerate cameras:
    dc1394camera_list_t * list;
    dc1394error_t err;

	err=dc1394_camera_enumerate(THE_CONTEXT, &list);
	if (err!=DC1394_SUCCESS)
	{
		cerr << "[CImageGrabber_dc1394] ERROR: Failed to enumerate cameras (Maybe your user has no rights to access IEEE1394?)." << endl;
		return;
	}

	if (!list->num)
	{
		cerr << "[CImageGrabber_dc1394] ERROR: No cameras found." << endl;
		return;
	}

	// Open the first camera or the one given by the user:
	if (!cameraGUID)
	{
		// Default: first one:
		m_dc1394camera = dc1394_camera_new(THE_CONTEXT, list->ids[0].guid);
		if (!THE_CAMERA)
		{
			dc1394_camera_free_list(list);
			cerr << "[CImageGrabber_dc1394] ERROR: Failed to initialize camera with GUID "<<list->ids[0].guid<<"\n";
			return;
		}
	}
	else
	{
		// Look for user-given IDs:
		for (uint32_t i=0;i<list->num;i++)
		{
			if (list->ids[i].guid==cameraGUID && list->ids[i].unit==cameraUnit)
			{
				m_dc1394camera = dc1394_camera_new_unit(THE_CONTEXT, cameraGUID, cameraUnit);
				if (!THE_CAMERA)
				{
					dc1394_camera_free_list(list);
					cerr << "[CImageGrabber_dc1394] ERROR: Failed to initialize camera with GUID "<<list->ids[0].guid<<"\n";
					return;
				}
				break;
			}
		}

		if (!m_dc1394camera)
		{
			dc1394_camera_free_list(list);
			cerr << "[CImageGrabber_dc1394] ERROR: Camera with GUID="<<cameraGUID<<" and UNIT="<<cameraUnit<<" not found.\n";
			return;
		}

	}
	dc1394_camera_free_list(list);

	// Camera open OK:
	if (verbose)
	{
		dc1394_camera_print_info(THE_CAMERA,stdout);
	}

	// get all supported modes:
	dc1394video_modes_t modes;
	err=dc1394_video_get_supported_modes(THE_CAMERA, &modes);
	if (err!=DC1394_SUCCESS)
	{
		cerr << "[CImageGrabber_dc1394] ERROR: Could not get list of modes." << endl;
		return;
	}

    // Is mode7? treat differently:
	if (options.mode7>=0)
	{
		m_desired_mode = DC1394_VIDEO_MODE_FORMAT7_MIN + options.mode7;
		if (verbose)
			cout << "[CImageGrabber_dc1394] Mode is mode7: " << options.mode7 << endl;
	}
	else
    {
    #define TEST_MODE(W,H,COLORMODEL)  else if (options.frame_width==W && options.frame_height==H && options.color_coding==COLOR_CODING_##COLORMODEL)  m_desired_mode=DC1394_VIDEO_MODE_##W##x##H##_##COLORMODEL;

		if (0) { }
		TEST_MODE(160,120,YUV444)
		TEST_MODE(320,240,YUV422)
		TEST_MODE(640,480,YUV411)
		TEST_MODE(640,480,YUV422)
		TEST_MODE(640,480,RGB8)
		TEST_MODE(640,480,MONO8)
		TEST_MODE(640,480,MONO16)
		TEST_MODE(800,600,YUV422)
		TEST_MODE(800,600,RGB8)
		TEST_MODE(800,600,MONO8)
		TEST_MODE(800,600,MONO16)
		TEST_MODE(1024,768,YUV422)
		TEST_MODE(1024,768,RGB8)
		TEST_MODE(1024,768,MONO8)
		TEST_MODE(1024,768,MONO16)
		TEST_MODE(1280,960,YUV422)
		TEST_MODE(1280,960,RGB8)
		TEST_MODE(1280,960,MONO8)
		TEST_MODE(1280,960,MONO16)
		TEST_MODE(1600,1200,YUV422)
		TEST_MODE(1600,1200,RGB8)
		TEST_MODE(1600,1200,MONO8)
		TEST_MODE(1600,1200,MONO16)
	}
	// Display all supported modes and chosen:
	if (verbose) cout << "------ Supported video modes ------" << endl;
	bool valid_video_mode = false;
	for(uint32_t i=0; i<modes.num; i++)
	{
		string mode;
		switch( modes.modes[i] )
		{
		case DC1394_VIDEO_MODE_160x120_YUV444: mode = "160x120_YUV444"; break;
		case DC1394_VIDEO_MODE_320x240_YUV422: mode = "320x240_YUV422"; break;
		case DC1394_VIDEO_MODE_640x480_YUV411: mode = "640x480_YUV411"; break;
		case DC1394_VIDEO_MODE_640x480_YUV422: mode = "640x480_YUV422"; break;
		case DC1394_VIDEO_MODE_640x480_RGB8: mode = "640x480_RGB8"; break;
		case DC1394_VIDEO_MODE_640x480_MONO8: mode = "640x480_MONO8"; break;
		case DC1394_VIDEO_MODE_640x480_MONO16: mode = "640x480_MONO16"; break;
		case DC1394_VIDEO_MODE_800x600_YUV422: mode = "800x600_YUV422"; break;
		case DC1394_VIDEO_MODE_800x600_RGB8: mode = "800x600_RGB8"; break;
		case DC1394_VIDEO_MODE_800x600_MONO8: mode = "800x600_MONO8"; break;
		case DC1394_VIDEO_MODE_1024x768_YUV422: mode = "1024x768_YUV422"; break;
		case DC1394_VIDEO_MODE_1024x768_RGB8: mode = "1024x768_RGB8"; break;
		case DC1394_VIDEO_MODE_1024x768_MONO8: mode = "1024x768_MONO8"; break;
		case DC1394_VIDEO_MODE_800x600_MONO16: mode = "800x600_MONO16"; break;
		case DC1394_VIDEO_MODE_1024x768_MONO16: mode = "1024x768_MONO16"; break;
		case DC1394_VIDEO_MODE_1280x960_YUV422: mode = "1280x960_YUV422"; break;
		case DC1394_VIDEO_MODE_1280x960_RGB8: mode = "1280x960_RGB8"; break;
		case DC1394_VIDEO_MODE_1280x960_MONO8: mode = "1280x960_MONO8"; break;
		case DC1394_VIDEO_MODE_1600x1200_YUV422: mode = "1600x1200_YUV422"; break;
		case DC1394_VIDEO_MODE_1600x1200_RGB8: mode = "1600x1200_RGB8"; break;
		case DC1394_VIDEO_MODE_1600x1200_MONO8: mode = "1600x1200_MONO8"; break;
		case DC1394_VIDEO_MODE_1280x960_MONO16: mode = "1280x960_MONO16"; break;
		case DC1394_VIDEO_MODE_1600x1200_MONO16: mode = "1600x1200_MONO16"; break;
		case DC1394_VIDEO_MODE_EXIF: mode = "EXIF"; break;
		case DC1394_VIDEO_MODE_FORMAT7_0: mode = "FORMAT7_0"; break;
		case DC1394_VIDEO_MODE_FORMAT7_1: mode = "FORMAT7_1"; break;
		case DC1394_VIDEO_MODE_FORMAT7_2: mode = "FORMAT7_2"; break;
		case DC1394_VIDEO_MODE_FORMAT7_3: mode = "FORMAT7_3"; break;
		case DC1394_VIDEO_MODE_FORMAT7_4: mode = "FORMAT7_4"; break;
		case DC1394_VIDEO_MODE_FORMAT7_5: mode = "FORMAT7_5"; break;
		case DC1394_VIDEO_MODE_FORMAT7_6: mode = "FORMAT7_6"; break;
		case DC1394_VIDEO_MODE_FORMAT7_7: mode = "FORMAT7_7"; break;
		default:
			cerr << "[CImageGrabber_dc1394] ERROR: Requested video mode is not valid." << endl;
			return;
		}
		if (modes.modes[i] == m_desired_mode) valid_video_mode = true;
		if (verbose)
		{
			if (modes.modes[i] == m_desired_mode) cout << mode << " (*)" << endl;
			else cout << mode << endl;
		}
	}
	if (!valid_video_mode)
	{
		cerr << format("[CImageGrabber_dc1394] ERROR: Requested mode %ix%i color_model:%i is not available for this camera.", options.frame_width,options.frame_height, int(options.color_coding) ) << endl;
		return;
	}

	// Reset to bus just in case:
	// And only once in a program, at start up:
//	static bool reset_1394bus = true;
//	if (reset_1394bus)
	{
//		reset_1394bus = false;
//		dc1394_reset_bus(THE_CAMERA);
	}

	/*-----------------------------------------------------------------------
	 *  setup capture
	 *-----------------------------------------------------------------------*/
	const int SIZE_RING_BUFFER = options.ring_buffer_size;

	err=dc1394_video_set_iso_speed(THE_CAMERA, DC1394_ISO_SPEED_400);
	if (err!=DC1394_SUCCESS)
	{
		cerr << "[CImageGrabber_dc1394] ERROR: Could not set iso speed." << endl;
		return;
	}

	err=dc1394_video_set_mode(THE_CAMERA, dc1394video_mode_t(m_desired_mode));
	// This checking only assures that m_desired_mode is inside dc1394video_mode_t enum range
	if (err!=DC1394_SUCCESS)
	{
		cerr << "[CImageGrabber_dc1394] ERROR: Could not set video mode." << endl;
		return;
	}


	dc1394framerate_t	the_framerate;
	switch( m_options.framerate )
	{
		case FRAMERATE_1_875: 	the_framerate=DC1394_FRAMERATE_1_875; break;
		case FRAMERATE_3_75: 	the_framerate=DC1394_FRAMERATE_3_75; break;
		case FRAMERATE_7_5: 	the_framerate=DC1394_FRAMERATE_7_5; break;
		case FRAMERATE_15: 		the_framerate=DC1394_FRAMERATE_15; break;
		case FRAMERATE_30: 		the_framerate=DC1394_FRAMERATE_30; break;
		case FRAMERATE_60: 		the_framerate=DC1394_FRAMERATE_60; break;
		case FRAMERATE_120: 	the_framerate=DC1394_FRAMERATE_120; break;
		case FRAMERATE_240: 	the_framerate=DC1394_FRAMERATE_240; break;

		default:
			cerr << "[CImageGrabber_dc1394] ERROR: Requested framerate is not valid." << endl;
			return;
	}

	err=dc1394_video_set_framerate(THE_CAMERA, the_framerate);
	if (err!=DC1394_SUCCESS)
	{
		cerr << "[CImageGrabber_dc1394] ERROR: Could not set framerate." << endl;
		return;
	}

	err=dc1394_capture_setup(THE_CAMERA, SIZE_RING_BUFFER, DC1394_CAPTURE_FLAGS_DEFAULT);
	if (err!=DC1394_SUCCESS)
	{
		cerr << "[CImageGrabber_dc1394] ERROR: Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera." << endl;
		return;
	}

	cout << "------ Other options ------" << endl;
	uint32_t iso_chan;
	if ((err = dc1394_video_get_iso_channel(THE_CAMERA, &iso_chan)) == DC1394_SUCCESS)
		if (verbose)
			cout << "ISO Channel: " << iso_chan << endl;

	dc1394speed_t iso_speed;
	if ((err = dc1394_video_get_iso_speed(THE_CAMERA, &iso_speed)) == DC1394_SUCCESS)
		if (verbose)
			cout << "ISO Speed: " << iso_speed << endl;

	// set trigger options:
	#define SET_TRIGGER(opt,OPT,TYPE) \
	if (options.trigger_##opt>=0) \
	{ \
		err=dc1394_external_trigger_set_##opt(THE_CAMERA, static_cast<dc1394trigger_##opt##_t>(DC1394_TRIGGER_##TYPE##_MIN + options.trigger_##opt)); \
		DC1394_WRN(err, "[CImageGrabber_dc1394::changeCaptureOptions] Could not set trigger opt"); \
	}
	SET_TRIGGER(mode,MODE,MODE)
	SET_TRIGGER(source,SOURCE,SOURCE)
	SET_TRIGGER(polarity,POLARITY,ACTIVE)
	if (options.trigger_power>=0)
	{
		err=dc1394_external_trigger_set_power(THE_CAMERA, dc1394switch_t(options.trigger_power));
		DC1394_WRN(err, "[CImageGrabber_dc1394::changeCaptureOptions] Could not set trigger power");
	}
	#undef SET_TRIGGER

	/*-----------------------------------------------------------------------
	 *  have the camera start sending us data
	 *-----------------------------------------------------------------------*/
    err=dc1394_video_set_transmission(THE_CAMERA, DC1394_ON);
    if (err!=DC1394_SUCCESS)
    {
        cerr << "[CImageGrabber_dc1394] ERROR: Could not start camera iso transmission." << endl;
        return;
    }

	// remember that we successfully initialized everything
	m_bInitialized = true;

	changeCaptureOptions(m_options);

	// Camera current features:
	if (verbose)
	{
		dc1394featureset_t features;
		if( (err=dc1394_feature_get_all(THE_CAMERA,&features)) == DC1394_SUCCESS )
			dc1394_feature_print_all(&features, stdout);
	}


#else
	THROW_EXCEPTION("[CImageGrabber_dc1394] ERROR: MRPT compiled with MRPT_HAS_LIBDC1394_2=0 !");
#endif
	MRPT_END
}

/*-------------------------------------------------------------
					Destructor
 -------------------------------------------------------------*/
CImageGrabber_dc1394::~CImageGrabber_dc1394()
{
#if MRPT_HAS_LIBDC1394_2
	m_bInitialized = false;
	if (THE_CAMERA)
	{
		dc1394_video_set_transmission( THE_CAMERA, DC1394_OFF );
		dc1394_capture_stop( THE_CAMERA );

		// Release BW:
//		uint32_t val;
//		if (dc1394_video_get_bandwidth_usage(THE_CAMERA, &val) == DC1394_SUCCESS)
//			 dc1394_iso_release_bandwidth(THE_CAMERA, val);
//		if ( dc1394_video_get_iso_channel(THE_CAMERA, &val) == DC1394_SUCCESS)
//			 dc1394_iso_release_channel(THE_CAMERA, val);

		dc1394_camera_free( THE_CAMERA );
	}
	if (THE_CONTEXT)
	{
		dc1394_free( THE_CONTEXT );
	}
#endif
}


/*-------------------------------------------------------------
					get the image - MONO
 -------------------------------------------------------------*/
bool  CImageGrabber_dc1394::getObservation( mrpt::obs::CObservationImage &out_observation)
{
   MRPT_START

   if (!m_bInitialized) return false;

#if MRPT_HAS_LIBDC1394_2
    dc1394video_frame_t *frame=NULL;
    dc1394error_t err;

    err=dc1394_video_set_transmission(THE_CAMERA, DC1394_ON);
    if (err!=DC1394_SUCCESS)
    {
        cerr << "[CImageGrabber_dc1394] ERROR: Could not start camera iso transmission." << endl;
        return false;
    }

	// get frame from ring buffer:
    err = dc1394_capture_dequeue(THE_CAMERA, DC1394_CAPTURE_POLICY_WAIT, &frame);
    //dc1394error_t err=dc1394_capture_dequeue(THE_CAMERA, DC1394_CAPTURE_POLICY_POLL, &frame);
	if (err!=DC1394_SUCCESS)
	{
		cerr << "[CImageGrabber_dc1394] ERROR: Could not capture a frame" << endl;
		return false;
	}

	out_observation.timestamp = mrpt::system::now();

	const unsigned int width = frame->size[0];
	const unsigned int height = frame->size[1];

	if (!m_options.deinterlace_stereo)
	{
		/*-----------------------------------------------------------------------
		 *  convert the image from what ever format it is to its RGB8
		 *-----------------------------------------------------------------------*/
		//dc1394_get_image_size_from_video_mode(THE_CAMERA, m_desired_mode, &width, &height);

		dc1394video_frame_t *new_frame= static_cast<dc1394video_frame_t*>( calloc(1,sizeof(dc1394video_frame_t)) );
		new_frame->color_coding=DC1394_COLOR_CODING_RGB8;
		dc1394_convert_frames(frame, new_frame);

		// Fill the output class:
		out_observation.image.loadFromMemoryBuffer(width,height,true, new_frame->image, true /* BGR -> RGB */ );

		// Free temporary frame:
		free(new_frame->image);
		free(new_frame);
	}
	else
	{
		// Stereo images:
		dc1394error_t err;

		uint8_t *imageBuf = new uint8_t[width*height*2];
		uint8_t *imageBufRGB = new uint8_t[width*height*2*3];

		if ((err = dc1394_deinterlace_stereo(frame->image, imageBuf, width, 2*height)) != DC1394_SUCCESS)
		{
			cerr << "[CImageGrabber_dc1394] ERROR: Could not deinterlace stereo images: " << err << endl;
			return false;
		}

		if ((err = dc1394_bayer_decoding_8bit(imageBuf, imageBufRGB,
		  width, 2*height,
		  DC1394_COLOR_FILTER_GBRG, // Has to be this value for Bumblebee!
		  DC1394_BAYER_METHOD_HQLINEAR)) != DC1394_SUCCESS)
		{
			cerr << "[CImageGrabber_dc1394] ERROR: Could not apply Bayer conversion: " << err << endl;
			return false;
		}

		out_observation.image.loadFromMemoryBuffer(width,height,true, imageBufRGB ); // Left cam.
		//out_observation.image.loadFromMemoryBuffer(width,height,true, imageBufRGB+ width*height*3 ); // Right cam.

		delete[] imageBuf;
		delete[] imageBufRGB;
	}

	// Now we can return the frame to the ring buffer:
    err = dc1394_capture_enqueue(THE_CAMERA, frame);
    if (err!=DC1394_SUCCESS)
    {
        cerr << "[CImageGrabber_dc1394] ERROR: Could not enqueue the ring buffer frame" << endl;
        return false;
    }

    return true;
#else
   THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_LIBDC1394_2=0 !");
#endif
   MRPT_END
}

/*-------------------------------------------------------------
					get the image - STEREO
 -------------------------------------------------------------*/
bool  CImageGrabber_dc1394::getObservation( mrpt::obs::CObservationStereoImages &out_observation)
{
   MRPT_START

   if (!m_bInitialized) return false;

#if MRPT_HAS_LIBDC1394_2
    dc1394video_frame_t *frame=NULL;

	// get frame from ring buffer:
	dc1394error_t err=dc1394_capture_dequeue(THE_CAMERA, DC1394_CAPTURE_POLICY_WAIT, &frame);
	if (err!=DC1394_SUCCESS)
	{
		cerr << "[CImageGrabber_dc1394] ERROR: Could not capture a frame" << endl;
		return false;
	}

	out_observation.timestamp = mrpt::system::now();

	const unsigned int width = frame->size[0];
	const unsigned int height = frame->size[1];

	if (!m_options.deinterlace_stereo)
	{
		THROW_EXCEPTION("Call to getObservation(stereo) but the camera was not set as stereo!");
	}
	else
	{
		// Stereo images:
		dc1394error_t err;

		uint8_t *imageBuf = new uint8_t[width*height*2];
		uint8_t *imageBufRGB = new uint8_t[width*height*2*3];

		if ((err = dc1394_deinterlace_stereo(frame->image, imageBuf, width, 2*height)) != DC1394_SUCCESS)
		{
			cerr << "[CImageGrabber_dc1394] ERROR: Could not deinterlace stereo images: " << err << endl;
			return false;
		}

		if ((err = dc1394_bayer_decoding_8bit(imageBuf, imageBufRGB,
											  width, 2*height,
											  DC1394_COLOR_FILTER_GBRG, // Has to be this value for Bumblebee!
											  DC1394_BAYER_METHOD_HQLINEAR)) != DC1394_SUCCESS)
		{
			cerr << "[CImageGrabber_dc1394] ERROR: Could not apply Bayer conversion: " << err << endl;
			return false;
		}

		out_observation.imageLeft.loadFromMemoryBuffer(width,height,true, imageBufRGB ); // Left cam.
		out_observation.imageRight.loadFromMemoryBuffer(width,height,true, imageBufRGB+ width*height*3 ); // Right cam.

		delete[] imageBuf;
		delete[] imageBufRGB;
	}

	// Now we can return the frame to the ring buffer:
	dc1394_capture_enqueue(THE_CAMERA, frame);

	return true;
#else
   THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_LIBDC1394_2=0 !");
#endif
   MRPT_END
}


/*-------------------------------------------------------------
					changeCaptureOptions
 -------------------------------------------------------------*/
bool CImageGrabber_dc1394::changeCaptureOptions( const TCaptureOptions_dc1394 &options  )
{
   MRPT_START

   if (!m_bInitialized) return false;

#if MRPT_HAS_LIBDC1394_2
	dc1394error_t err;
    // set features modes:
    #define SET_MODE(feat,FEAT) \
    if (options.feat##_mode>=0) \
    { \
        err=dc1394_feature_set_mode(THE_CAMERA, DC1394_FEATURE_##FEAT, static_cast<dc1394feature_mode_t>(DC1394_FEATURE_MODE_MIN + options.feat##_mode)); \
        DC1394_WRN(err, "[CImageGrabber_dc1394::changeCaptureOptions] Could not set feat mode"); \
    }
    SET_MODE(shutter,SHUTTER)
    SET_MODE(gain,GAIN)
    SET_MODE(gamma,GAMMA)
    SET_MODE(brightness,BRIGHTNESS)
    SET_MODE(exposure,EXPOSURE)
    SET_MODE(sharpness,SHARPNESS)
    SET_MODE(white_balance,WHITE_BALANCE)
    #undef SET_MODE

    // Set features values:
    #define SET_VALUE(feat,FEAT) \
    if (options.feat>=0) \
    { \
        err=dc1394_feature_set_value(THE_CAMERA, DC1394_FEATURE_##FEAT, options.feat); \
        DC1394_WRN(err, "[CImageGrabber_dc1394::changeCaptureOptions] Could not set feat value"); \
    }
    SET_VALUE(shutter,SHUTTER)
    SET_VALUE(gain,GAIN)
    SET_VALUE(gamma,GAMMA)
    SET_VALUE(brightness,BRIGHTNESS)
    SET_VALUE(exposure,EXPOSURE)
    SET_VALUE(sharpness,SHARPNESS)
    SET_VALUE(white_balance,WHITE_BALANCE)
    #undef SET_VALUE

	return true;
#else
   THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_LIBDC1394_2=0 !");
#endif
   MRPT_END
}

/*-------------------------------------------------------------
                    setSoftwareTriggerLevel
 -------------------------------------------------------------*/
bool CImageGrabber_dc1394::setSoftwareTriggerLevel( bool level  )
{
   MRPT_START

   if (!m_bInitialized) return false;

#if MRPT_HAS_LIBDC1394_2
    dc1394error_t err;
    err = dc1394_software_trigger_set_power(THE_CAMERA, (dc1394switch_t)level);
    DC1394_WRN(err, "[CImageGrabber_dc1394::setSoftwareTriggerLevel] Could not set software trigger level");

    return true;
#else
   THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_LIBDC1394_2=0 !");
#endif
   MRPT_END
}



/** Generates a list with the information on all the existing (Firewire) cameras in the system.
  * \exception std::runtime_error On any error calling libdc1394.
  */
void CImageGrabber_dc1394::enumerateCameras( TCameraInfoList &out_list )
{
	MRPT_START
#if MRPT_HAS_LIBDC1394_2

	dc1394_t			* lib_context = NULL;
	dc1394camera_list_t * list=NULL;
	out_list.clear();

	try
	{
		lib_context = dc1394_new ();
		if (!lib_context)
			throw std::runtime_error("[CImageGrabber_dc1394] ERROR: Failed to enumerate cameras (Maybe your user has no rights to access IEEE1394?).");

		// Enumerate cameras:
		dc1394error_t err;

		err=dc1394_camera_enumerate(lib_context, &list);
		if (err!=DC1394_SUCCESS)
			throw std::runtime_error("[CImageGrabber_dc1394] ERROR: Failed to enumerate cameras (Maybe your user has no rights to access IEEE1394?).");

		for (unsigned int i=0;i< list->num; i++)
		{
			TCameraInfo info;

			info.guid = list->ids[i].guid;
			info.unit = list->ids[i].unit;

			// Try to open it:
			dc1394camera_t *cam = dc1394_camera_new_unit(lib_context, list->ids[i].guid, list->ids[i].unit);
			if (!cam)
				throw std::runtime_error(format("[CImageGrabber_dc1394] ERROR: Failed to query camera with GUID %u\n", static_cast<unsigned int>(list->ids[i].guid) ));



			info.unit_spec_ID = cam->unit_spec_ID;
			info.unit_sw_version = cam->unit_sw_version;
			info.unit_sub_sw_version = cam->unit_sub_sw_version;
			info.command_registers_base = cam->command_registers_base;
			info.unit_directory = cam->unit_directory;
			info.unit_dependent_directory = cam->unit_dependent_directory;
			info.advanced_features_csr = cam->advanced_features_csr;
			info.PIO_control_csr = cam->PIO_control_csr;
			info.SIO_control_csr = cam->SIO_control_csr;
			info.strobe_control_csr = cam->strobe_control_csr;
			for (int j=0;j<DC1394_VIDEO_MODE_FORMAT7_NUM;j++)
				info.format7_csr[j]  = cam->format7_csr[j];
			info.iidc_version = cam->iidc_version;
			info.vendor = std::string(cam->vendor ? cam->vendor : "");
			info.model = std::string(cam->model ? cam->model : "");
			info.vendor_id = cam->vendor_id;
			info.model_id = cam->model_id;
			info.bmode_capable = cam->bmode_capable;
			info.one_shot_capable = cam->one_shot_capable;
			info.multi_shot_capable = cam->multi_shot_capable;
			info.can_switch_on_off = cam->can_switch_on_off;
			info.has_vmode_error_status = cam->has_vmode_error_status;
			info.has_feature_error_status = cam->has_feature_error_status;
			info.max_mem_channel = cam->max_mem_channel;

			//dc1394_camera_print_info(cam,stdout);

			dc1394_camera_free(cam); // Close camera

			out_list.push_back(info);
		}

		// Free context:
		dc1394_free( lib_context ); 	lib_context = NULL;
		dc1394_camera_free_list(list);  list = NULL;
	}
	catch(std::exception &e)
	{
		if (list) 			dc1394_camera_free_list(list);
		if (lib_context) 	dc1394_free( lib_context );

		THROW_STACKED_EXCEPTION(e)
	}
#else
   THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_LIBDC1394_2=0 !")
#endif
   MRPT_END
}


