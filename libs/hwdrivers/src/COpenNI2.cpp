/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
//#if MRPT_HAS_OPENNI2

#include <OpenNI.h>
#include <PS1080.h>

#include <mrpt/hwdrivers.h> // Precompiled header

#include <mrpt/hwdrivers/COpenNI2.h>
#include <mrpt/utils/CTimeLogger.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::synch;

IMPLEMENTS_GENERIC_SENSOR(COpenNI2,mrpt::hwdrivers)

// Whether to profile memory allocations:
//#define KINECT_PROFILE_MEM_ALLOC
#ifdef KINECT_PROFILE_MEM_ALLOC
mrpt::utils::CTimeLogger alloc_tim;
#endif

#define DEVICE_ID_PTR (reinterpret_cast<openni::Device*>(vp_devices[sensor_id]))
#define DEPTH_STREAM_PTR (reinterpret_cast<openni::VideoStream*>(p_depth_stream))
#define RGB_STREAM_PTR (reinterpret_cast<openni::VideoStream*>(p_rgb_stream))
#define DEPTH_FRAME_PTR (reinterpret_cast<openni::VideoFrameRef*>(framed))
#define RGB_FRAME_PTR (reinterpret_cast<openni::VideoFrameRef*>(framergb))

//int int a; // Check compilation

/*-------------------------------------------------------------
		ctor
 -------------------------------------------------------------*/
COpenNI2::COpenNI2() :
	m_sensorPoseOnRobot(),
	m_preview_window(false),
	m_preview_window_decimation(1),
	m_preview_decim_counter_range(0),
	m_preview_decim_counter_rgb(0),

  width(320),
  height(240),
  fps(30),

	m_relativePoseIntensityWRTDepth(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0)),
	m_user_device_number(0),
	m_grab_image(true),
	m_grab_depth(true),
	m_grab_3D_points(true)
////	m_video_channel(VIDEO_CHANNEL_RGB)
{
//	// Get maximum range:
////	m_maxRange=m_range2meters[KINECT_RANGES_TABLE_LEN-2];  // Recall: r[Max-1] means error.

	// Default label:
	m_sensorLabel = "OPENNI2";

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
}

/*-------------------------------------------------------------
			dtor
 -------------------------------------------------------------*/
COpenNI2::~COpenNI2()
{
	this->close();
}

/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void COpenNI2::initialize()
{
  int rc = openni::OpenNI::initialize();
	if(rc != openni::STATUS_OK)
    THROW_EXCEPTION(mrpt::format("After initialization:\n %s\n", openni::OpenNI::getExtendedError()))
//  printf("After initialization:\n %s\n", openni::OpenNI::getExtendedError());

  // Show devices list
  openni::Array<openni::DeviceInfo> deviceList;
  openni::OpenNI::enumerateDevices(&deviceList);
  printf("Get device list. %d devices connected\n", deviceList.getSize() );
  vp_devices.resize(deviceList.getSize());
  for(unsigned i=0; i < deviceList.getSize(); i++)
  {
    int product_id = deviceList[i].getUsbProductId();
    printf("Device %u: name=%s uri=%s vendor=%s product=%i \n", i+1 , deviceList[i].getName(), deviceList[i].getUri(), deviceList[i].getVendor(), product_id);
    vp_devices[i] = new openni::Device;
  }
  if(deviceList.getSize() == 0)
  {
    cout << "No devices connected -> EXIT\n";
    return;
  }

	open();
}

/** This method will be invoked at a minimum rate of "process_rate" (Hz)
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void COpenNI2::doProcess()
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

		vector<CSerializablePtr> objs;
		if (m_grab_image || m_grab_depth || m_grab_3D_points)  objs.push_back(newObs);

		appendObservations( objs );
	}
}

/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
*  \exception This method must throw an exception with a descriptive message if some critical parameter is missing or has an invalid value.
*/
void  COpenNI2::loadConfig_sensorSpecific(
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

  width = configSource.read_int(iniSection,"width",0);
  height = configSource.read_int(iniSection,"height",0);
  fps = configSource.read_float(iniSection,"fps",0);
std::cout << "width " << width << " height " << height << " fps " << fps << endl;

	const mrpt::poses::CPose3D twist(0,0,0,DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-90));

	mrpt::utils::TStereoCamera  sc;
	sc.leftCamera  = m_cameraParamsDepth;  // Load default values so that if we fail to load from cfg at least we have some reasonable numbers.
	sc.rightCamera = m_cameraParamsRGB;
	sc.rightCameraPose = mrpt::poses::CPose3DQuat(m_relativePoseIntensityWRTDepth - twist);

	try {
		sc.loadFromConfigFile(iniSection,configSource);
	} catch (std::exception &e) {
		std::cout << "[COpenNI2::loadConfig_sensorSpecific] Warning: Ignoring error loading calibration parameters:\n" << e.what();
	}
	m_cameraParamsDepth = sc.leftCamera;
	m_cameraParamsRGB   = sc.rightCamera;
	m_relativePoseIntensityWRTDepth = twist + mrpt::poses::CPose3D(sc.rightCameraPose);

	// Id:
	m_user_device_number = configSource.read_int(iniSection,"device_number",m_user_device_number );

	m_grab_image = configSource.read_bool(iniSection,"grab_image",m_grab_image);
	m_grab_depth = configSource.read_bool(iniSection,"grab_depth",m_grab_depth);
	m_grab_3D_points = configSource.read_bool(iniSection,"grab_3D_points",m_grab_3D_points);
//	m_grab_IMU = configSource.read_bool(iniSection,"grab_IMU",m_grab_IMU );

//	m_video_channel = configSource.read_enum<TVideoChannel>(iniSection,"video_channel",m_video_channel);

	{
		std::string s = configSource.read_string(iniSection,"relativePoseIntensityWRTDepth","");
		if (!s.empty())
			m_relativePoseIntensityWRTDepth.fromString(s);
	}

}

bool COpenNI2::isOpen() const
{

}

//void getDepthFrame(void *v_depth, uint32_t timestamp)
//{
//
//}
//
//void getRGBFrame(void *v_depth, uint32_t timestamp)
//{
//
//}

void COpenNI2::open()
{
//	if(isOpen())
//		close();

	// Alloc memory, if this is the first time:
	m_buf_depth.resize(640*480*3); // We'll resize this below if needed
	m_buf_rgb.resize(640*480*3);

  openni::Array<openni::DeviceInfo> deviceList;
  openni::OpenNI::enumerateDevices(&deviceList);
	int nr_devices = deviceList.getSize();
	//printf("[COpenNI2] Number of devices found: %d\n", nr_devices);

	if (!nr_devices)
		THROW_EXCEPTION("No Kinect devices found.")

	// Open the given device number:
  int sensor_id = 0;
	const char* deviceURI = openni::ANY_DEVICE;
//	int rc = reinterpret_cast<openni::Device*>(vp_devices[sensor_id])->open(deviceURI);
	int rc = DEVICE_ID_PTR->open(deviceURI);
	if(rc != openni::STATUS_OK)
	{
    THROW_EXCEPTION_CUSTOM_MSG1("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
//		printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
	}
	//cout << endl << "Do we have IR sensor? " << DEVICE_ID_PTR->hasSensor(openni::SENSOR_IR);
	//cout << endl << "Do we have RGB sensor? " << DEVICE_ID_PTR->hasSensor(openni::SENSOR_COLOR);
	//cout << endl << "Do we have Depth sensor? " << DEVICE_ID_PTR->hasSensor(openni::SENSOR_DEPTH);

	//								Create RGB and Depth channels
	//========================================================================================
	p_depth_stream = new openni::VideoStream;
	p_rgb_stream = new openni::VideoStream;
	rc = DEPTH_STREAM_PTR->create(*DEVICE_ID_PTR, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = DEPTH_STREAM_PTR->start();
		if (rc != openni::STATUS_OK)
		{
      THROW_EXCEPTION_CUSTOM_MSG1("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
//			printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			DEPTH_STREAM_PTR->destroy();
		}
	}
	else
	{
		printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	rc = RGB_STREAM_PTR->create(*DEVICE_ID_PTR, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		rc = RGB_STREAM_PTR->start();
		if (rc != openni::STATUS_OK)
		{
      THROW_EXCEPTION_CUSTOM_MSG1("Couldn't start infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
//			printf("Couldn't start infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
			RGB_STREAM_PTR->destroy();
		}
	}
	else
	{
		printf("Couldn't find infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	if (!DEPTH_STREAM_PTR->isValid() || !RGB_STREAM_PTR->isValid())
	{
		printf("No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return;
	}

	if (rc != openni::STATUS_OK)
	{
		openni::OpenNI::shutdown();
		return;
	}

	//						Configure some properties (resolution)
	//========================================================================================
	openni::VideoMode	options;
	if (DEVICE_ID_PTR->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
		rc = DEVICE_ID_PTR->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	else
		cout << "Device doesn't do image registration!" << endl;

  if (DEVICE_ID_PTR->setDepthColorSyncEnabled(true) == openni::STATUS_OK)
    cout << "setDepthColorSyncEnabled" << endl;
  else
    cout << "setDepthColorSyncEnabled failed!" << endl;

	//rc = DEVICE_ID_PTR->setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);

	options = RGB_STREAM_PTR->getVideoMode();
	printf("\nInitial resolution RGB (%d, %d)", options.getResolutionX(), options.getResolutionY());
	options.setResolution(width,height);
	options.setFps(fps);
	options.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	rc = RGB_STREAM_PTR->setVideoMode(options);
	if (rc!=openni::STATUS_OK)
	{
		printf("Failed to change RGB resolution!\n");
		return;
	}
	rc = RGB_STREAM_PTR->setMirroringEnabled(false);

	options = DEPTH_STREAM_PTR->getVideoMode();
	printf("\nInitial resolution Depth(%d, %d)", options.getResolutionX(), options.getResolutionY());
	options.setResolution(width,height);
	options.setFps(30);
	options.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	rc = DEPTH_STREAM_PTR->setVideoMode(options);
	if (rc!=openni::STATUS_OK)
	{
		printf("Failed to change depth resolution!\n");
		return;
	}
	rc = DEPTH_STREAM_PTR->setMirroringEnabled(false);

	options = DEPTH_STREAM_PTR->getVideoMode();
	printf("\nNew resolution (%d, %d) \n", options.getResolutionX(), options.getResolutionY());

	//Allow detection of closer points (although they will flicker)
	//bool CloseRange;
	//DEPTH_STREAM_PTR->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, 1);
	//DEPTH_STREAM_PTR->getProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, &CloseRange);
	//printf("\nClose range: %s", CloseRange?"On":"Off");


//	// Setup:
//	if(m_initial_tilt_angle!=360) // 360 means no motor command.
//    setTiltAngleDegrees(m_initial_tilt_angle);
//
//	// rgb or IR channel:
//	const freenect_frame_mode desiredFrMode = freenect_find_video_mode(
//		FREENECT_RESOLUTION_MEDIUM,
//		m_video_channel==VIDEO_CHANNEL_IR ?
//			FREENECT_VIDEO_IR_8BIT
//			:
//			FREENECT_VIDEO_BAYER // FREENECT_VIDEO_RGB: Use Bayer instead so we can directly decode it here
//		);
//
//	// Switch to that video mode:
//	if (freenect_set_video_mode(f_dev, desiredFrMode)<0)
//		THROW_EXCEPTION("Error setting Kinect video mode.")

	// Realloc mem:
	m_buf_depth.resize(width*height*3);
	m_buf_rgb.resize(width*height*3);

	// Save resolution:
	m_cameraParamsRGB.ncols = width;
	m_cameraParamsRGB.nrows = height;

	m_cameraParamsDepth.ncols = width;
	m_cameraParamsDepth.nrows = height;

	framed = new openni::VideoFrameRef;
	framergb = new openni::VideoFrameRef;
}

void COpenNI2::close()
{
	DEPTH_STREAM_PTR->destroy();
	RGB_STREAM_PTR->destroy();

  delete DEPTH_FRAME_PTR;
  delete RGB_FRAME_PTR;

  delete DEPTH_STREAM_PTR;
  delete RGB_STREAM_PTR;

  for(unsigned i=0; i < vp_devices.size(); i++)
    delete vp_devices[i];

	openni::OpenNI::shutdown();
}


//
/////** Changes the video channel to open (RGB or IR) - you can call this method before start grabbing or in the middle of streaming and the video source will change on the fly.
////	Default is RGB channel.
////*/
////void  COpenNI2::setVideoChannel(const TVideoChannel vch)
////{
////#if MRPT_HAS_KINECT_FREENECT
////	m_video_channel = vch;
////	if (!isOpen()) return; // Nothing else to do here.
////
////	// rgb or IR channel:
////	freenect_stop_video(f_dev);
////
////	// rgb or IR channel:
////	const freenect_frame_mode desiredFrMode = freenect_find_video_mode(
////		FREENECT_RESOLUTION_MEDIUM,
////		m_video_channel==VIDEO_CHANNEL_IR ?
////			FREENECT_VIDEO_IR_8BIT
////			:
////			FREENECT_VIDEO_BAYER // FREENECT_VIDEO_RGB: Use Bayer instead so we can directly decode it here
////		);
////
////	// Switch to that video mode:
////	if (freenect_set_video_mode(f_dev, desiredFrMode)<0)
////		THROW_EXCEPTION("Error setting Kinect video mode.")
////
////	freenect_start_video(f_dev);
////
////#endif // MRPT_HAS_KINECT_FREENECT
////
////#if MRPT_HAS_KINECT_CL_NUI
////	THROW_EXCEPTION("Grabbing IR intensity is not available with CL NUI Kinect driver.")
////#endif // MRPT_HAS_KINECT_CL_NUI
////
////
////}
////
//
/** The main data retrieving function, to be called after calling loadConfig() and initialize().
  *  \param out_obs The output retrieved observation (only if there_is_obs=true).
  *  \param there_is_obs If set to false, there was no new observation.
  *  \param hardware_error True on hardware/comms error.
  *
  * \sa doProcess
  */
void COpenNI2::getNextObservation(
	mrpt::slam::CObservation3DRangeScan &_out_obs,
	bool &there_is_obs,
	bool &hardware_error )
{
cout << "COpenNI2::getNextObservation \n";
	there_is_obs=false;
	hardware_error = false;

  // Read a frame (depth + rgb)
  DEPTH_STREAM_PTR->readFrame(DEPTH_FRAME_PTR);
cout << "COpenNI2::getNextObservation2 \n";

  RGB_STREAM_PTR->readFrame(RGB_FRAME_PTR);

  if ((DEPTH_FRAME_PTR->getWidth() != RGB_FRAME_PTR->getWidth()) || (DEPTH_FRAME_PTR->getHeight() != RGB_FRAME_PTR->getHeight()))
  {
    cout << "\nBoth frames don't have the same size.";
  }
  else
  {
		there_is_obs=true;

		CObservation3DRangeScan  newObs;
		newObs.hasConfidenceImage = false;

		// Set intensity image ----------------------
		if (m_grab_image)
		{
			newObs.hasIntensityImage  = true;
		}

		// Set range image --------------------------
		if (m_grab_depth || m_grab_3D_points)
		{
			newObs.hasRangeImage = true;
			newObs.range_is_depth = true;
		}

    newObs.timestamp = mrpt::system::getCurrentTime();
    newObs.rangeImage_setSize(height,width);

    // Read one frame
    const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)DEPTH_FRAME_PTR->getData();
    const openni::RGB888Pixel* pRgbRow = (const openni::RGB888Pixel*)RGB_FRAME_PTR->getData();
    int rowSize = DEPTH_FRAME_PTR->getStrideInBytes() / sizeof(openni::DepthPixel);

    utils::CImage iimage(width,height,CH_RGB);
    for (int yc = 0; yc < DEPTH_FRAME_PTR->getHeight(); ++yc)
    {
      const openni::DepthPixel* pDepth = pDepthRow;
      const openni::RGB888Pixel* pRgb = pRgbRow;
      for (int xc = 0; xc < DEPTH_FRAME_PTR->getWidth(); ++xc, ++pDepth, ++pRgb)
      {
        newObs.rangeImage(yc,xc) = (*pDepth)*1.0/1000;
        iimage.setPixel(xc,yc,(pRgb->r<<16)+(pRgb->g<<8)+pRgb->b);

        //newObs.intensityImage.setPixel(xc,yc,(*pRgb));
      }

      pDepthRow += rowSize;
      pRgbRow += rowSize;
    }
    newObs.intensityImage = iimage;

		// Save the observation to the user's object:
		_out_obs.swap(newObs);

    // Set common data into observation:
    // --------------------------------------
    _out_obs.sensorLabel = m_sensorLabel;
    _out_obs.timestamp = mrpt::system::now();
    _out_obs.sensorPose = m_sensorPoseOnRobot;
  //	_out_obs.relativePoseIntensityWRTDepth = m_relativePoseIntensityWRTDepth;

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
}


/* -----------------------------------------------------
				setPathForExternalImages
----------------------------------------------------- */
void COpenNI2::setPathForExternalImages( const std::string &directory )
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
