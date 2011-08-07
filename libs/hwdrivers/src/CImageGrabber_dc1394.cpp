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

#include <mrpt/config.h>


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
			cerr << format("[CImageGrabber_dc1394] ERROR: Failed to initialize camera with GUID %"PRIX64"\n", list->ids[0].guid );
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
					cerr << format("[CImageGrabber_dc1394] ERROR: Failed to initialize camera with GUID %"PRIX64"\n", list->ids[0].guid );
					return;
				}
				break;
			}
		}

		if (!m_dc1394camera)
		{
			dc1394_camera_free_list(list);
			cerr << format("[CImageGrabber_dc1394] ERROR: Camera with GUID=0x%"PRIX64" and UNIT=%i not found.", cameraGUID, int(cameraUnit) );
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

	// Display all supported modes:
	if (verbose)
	{
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
		// Build the mode value from the user request:
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
		else
		{
			cerr << format("[CImageGrabber_dc1394] ERROR: Requested mode %ix%i color_model:%i is unknown.", options.frame_width,options.frame_height, int(options.color_coding) ) << endl;
			return;
		}
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
	const int SIZE_RING_BUFFER = 15;

	err=dc1394_video_set_iso_speed(THE_CAMERA, DC1394_ISO_SPEED_400);
	if (err!=DC1394_SUCCESS)
	{
		cerr << "[CImageGrabber_dc1394] ERROR: Could not set iso speed." << endl;
		return;
	}

	err=dc1394_video_set_mode(THE_CAMERA, dc1394video_mode_t(m_desired_mode));
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

	uint32_t iso_chan;
	if ((err = dc1394_video_get_iso_channel(THE_CAMERA, &iso_chan)) == DC1394_SUCCESS)
		if (verbose)
			cout << "ISO Channel: " << iso_chan << endl;

	dc1394speed_t iso_speed;
	if ((err = dc1394_video_get_iso_speed(THE_CAMERA, &iso_speed)) == DC1394_SUCCESS)
		if (verbose)
			cout << "ISO Speed: " << iso_speed << endl;


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
bool  CImageGrabber_dc1394::getObservation( mrpt::slam::CObservationImage &out_observation)
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
											  DC1394_BAYER_METHOD_NEAREST)) != DC1394_SUCCESS)
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
	dc1394_capture_enqueue(THE_CAMERA, frame);

	return true;
#else
   THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_LIBDC1394_2=0 !");
#endif
   MRPT_END
}

/*-------------------------------------------------------------
					get the image - STEREO
 -------------------------------------------------------------*/
bool  CImageGrabber_dc1394::getObservation( mrpt::slam::CObservationStereoImages &out_observation)
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
											  DC1394_BAYER_METHOD_NEAREST)) != DC1394_SUCCESS)
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

	// Set features:
	if (options.shutter>=0)
	{
		err=dc1394_feature_set_value(THE_CAMERA, DC1394_FEATURE_SHUTTER, options.shutter );
		DC1394_WRN(err, "[CImageGrabber_dc1394::changeCaptureOptions] Could not set shutter");
	}
	if (options.gain>=0)
	{
		err=dc1394_feature_set_value(THE_CAMERA, DC1394_FEATURE_GAIN, options.gain );
		DC1394_WRN(err, "[CImageGrabber_dc1394::changeCaptureOptions] Could not set gain");
	}
	if (options.gamma>=0)
	{
		err=dc1394_feature_set_value(THE_CAMERA, DC1394_FEATURE_GAMMA, options.gamma );
		DC1394_WRN(err, "[CImageGrabber_dc1394::changeCaptureOptions] Could not set gamma");
	}
	if (options.brightness>=0)
	{
		err=dc1394_feature_set_value(THE_CAMERA, DC1394_FEATURE_BRIGHTNESS, options.brightness );
		DC1394_WRN(err, "[CImageGrabber_dc1394::changeCaptureOptions] Could not set brightness");
	}
	if (options.exposure>=0)
	{
		err=dc1394_feature_set_value(THE_CAMERA, DC1394_FEATURE_EXPOSURE, options.exposure );
		DC1394_WRN(err, "[CImageGrabber_dc1394::changeCaptureOptions] Could not set exposure");
	}
	if (options.sharpness>=0)
	{
		err=dc1394_feature_set_value(THE_CAMERA, DC1394_FEATURE_SHARPNESS, options.sharpness );
		DC1394_WRN(err, "[CImageGrabber_dc1394::changeCaptureOptions] Could not set sharpness");
	}
	if (options.white_balance>=0)
	{
		err=dc1394_feature_set_value(THE_CAMERA, DC1394_FEATURE_WHITE_BALANCE, options.white_balance );
		DC1394_WRN(err, "[CImageGrabber_dc1394::changeCaptureOptions] Could not set white_balance");
	}

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
				throw std::runtime_error(format("[CImageGrabber_dc1394] ERROR: Failed to query camera with GUID %"PRIX64"\n", list->ids[i].guid ));



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


