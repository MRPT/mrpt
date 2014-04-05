/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers.h> // Precompiled header

#include <mrpt/hwdrivers/COpenNI2_RGBD360.h>
#include <mrpt/utils/CTimeLogger.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

#if MRPT_HAS_OPENNI2
#	include <OpenNI.h>
#	include <PS1080.h>
#endif

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::synch;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(COpenNI2_RGBD360,mrpt::hwdrivers)

#define DEVICE_LIST_PTR (reinterpret_cast< openni::Array<openni::DeviceInfo>* >(deviceListPtr))
#define DEVICE_ID_PTR (reinterpret_cast<openni::Device*>(vp_devices[sensor_id]))
#define DEPTH_STREAM_ID_PTR (reinterpret_cast<openni::VideoStream*>(vp_depth_stream[sensor_id]))
#define RGB_STREAM_ID_PTR (reinterpret_cast<openni::VideoStream*>(vp_rgb_stream[sensor_id]))
#define DEPTH_FRAME_ID_PTR (reinterpret_cast<openni::VideoFrameRef*>(vp_frame_depth[sensor_id]))
#define RGB_FRAME_ID_PTR (reinterpret_cast<openni::VideoFrameRef*>(vp_frame_rgb[sensor_id]))


/*-------------------------------------------------------------
ctor
-------------------------------------------------------------*/
COpenNI2_RGBD360::COpenNI2_RGBD360() :
	numDevices(0),
	width(320),
	height(240),
	//  width(640),
	//  height(420),
	fps(30),

	m_sensorPoseOnRobot(),
	m_preview_window(false),
	m_preview_window_decimation(1),
	m_preview_decim_counter_range(0),
	m_preview_decim_counter_rgb(0),

	m_grab_rgb(true),
	m_grab_depth(true),
	m_grab_3D_points(true)
{

	// Default label:
	m_sensorLabel = "RGBD360";

}

/*-------------------------------------------------------------
dtor
-------------------------------------------------------------*/
COpenNI2_RGBD360::~COpenNI2_RGBD360()
{
#if MRPT_HAS_OPENNI2
	cout << "Destroy COpenNI2_RGBD360... \n";

	//	this->close();
	for(unsigned i=0; i < vOpenDevices.size(); i++)
		this->close(vOpenDevices[i]);

	if(DEVICE_LIST_PTR)
		delete DEVICE_LIST_PTR; // Delete the pointer to the list of devices

	openni::OpenNI::shutdown();
#else 
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void COpenNI2_RGBD360::initialize()
{
#if MRPT_HAS_OPENNI2
	int rc = openni::OpenNI::initialize();
	if(rc != openni::STATUS_OK)
		THROW_EXCEPTION(mrpt::format("After initialization:\n %s\n", openni::OpenNI::getExtendedError()))
		//  printf("After initialization:\n %s\n", openni::OpenNI::getExtendedError());

		// Show devices list
		deviceListPtr = new openni::Array<openni::DeviceInfo>;
	openni::OpenNI::enumerateDevices(DEVICE_LIST_PTR);
	numDevices = (*DEVICE_LIST_PTR).getSize();

	printf("Get device list. %d devices connected\n", numDevices );
	vp_devices.resize(numDevices);
	vp_depth_stream.resize(numDevices);
	vp_rgb_stream.resize(numDevices);
	vp_frame_depth.resize(numDevices);
	vp_frame_rgb.resize(numDevices);

	for(unsigned i=0; i < numDevices; i++)
	{
		int product_id = (*DEVICE_LIST_PTR)[i].getUsbProductId();
		printf("Device %u: name=%s uri=%s vendor=%s product=%i \n", i, (*DEVICE_LIST_PTR)[i].getName(), (*DEVICE_LIST_PTR)[i].getUri(), (*DEVICE_LIST_PTR)[i].getVendor(), product_id);
	}

	//  if(numDevices == 0)
	if(numDevices < NUM_SENSORS)
	{
		cout << "Num required sensors " << NUM_SENSORS << endl;
		cout << "Not enough devices connected -> EXIT\n";
		return;
	}
	cout << "COpenNI2_RGBD360 initializes correctly.\n";

	for(unsigned sensor_id=0; sensor_id < NUM_SENSORS; sensor_id++)
		open(sensor_id);
#else 
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

/** This method will be invoked at a minimum rate of "process_rate" (Hz)
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void COpenNI2_RGBD360::doProcess()
{
	cout << "COpenNI2_RGBD360::doProcess...\n";

	bool	thereIs, hwError;

	CObservationRGBD360Ptr newObs = CObservationRGBD360::Create();
	//	CObservation3DRangeScanPtr newObs = CObservation3DRangeScan::Create();

	assert(!vOpenDevices.empty());
	//  unsigned sensor_id = vOpenDevices.front();
	getNextObservation( *newObs, thereIs, hwError );

	if (hwError)
	{
		m_state = ssError;
		THROW_EXCEPTION("Couldn't communicate to the OpenNI2 sensor!");
	}

	if (thereIs)
	{
		m_state = ssWorking;

		vector<CSerializablePtr> objs;
		if (m_grab_rgb || m_grab_depth || m_grab_3D_points) objs.push_back(newObs);
		//      for(unsigned sensor_id=0; sensor_id < newObs.size(); sensor_id++)
		//        objs.push_back(newObs[sensor_id]);

		appendObservations( objs );
	}
}

/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
*  \exception This method must throw an exception with a descriptive message if some critical parameter is missing or has an invalid value.
*/
void  COpenNI2_RGBD360::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
	cout << "COpenNI2_RGBD360::loadConfig_sensorSpecific...\n";

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

	// Id:
	m_user_device_number = configSource.read_int(iniSection,"device_number",m_user_device_number );

	m_grab_rgb = configSource.read_bool(iniSection,"grab_image",m_grab_rgb);
	m_grab_depth = configSource.read_bool(iniSection,"grab_depth",m_grab_depth);
	m_grab_3D_points = configSource.read_bool(iniSection,"grab_3D_points",m_grab_3D_points);

	//  m_num_sensors = configSource.read_int(iniSection,"m_num_sensors",0);
}

bool COpenNI2_RGBD360::isOpen(const unsigned sensor_id) const
{
	for(unsigned i=0; i < vOpenDevices.size(); i++)
		if(sensor_id == vOpenDevices[i])
			return true;

	return false;
}

//void COpenNI2_RGBD360::open(const int serial_num)
void COpenNI2_RGBD360::open(unsigned sensor_id)
{
#if MRPT_HAS_OPENNI2
	cout << "COpenNI2_RGBD360::open... " << sensor_id << " \n";
	if(isOpen(sensor_id))
	{
		//		close(sensor_id);
		cout << "The sensor " << sensor_id << " is already open\n";
		return;
	}

	if (!numDevices)
		THROW_EXCEPTION("No OpenNI2 devices found.")

		if (sensor_id >= numDevices)
			THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")

			int rc;
	//  unsigned sensor_id; // To use with serial_num

	// Open the given device number:
	vp_devices[sensor_id] = new openni::Device;
	vp_depth_stream[sensor_id] = new openni::VideoStream;
	vp_rgb_stream[sensor_id] = new openni::VideoStream;
	vp_frame_depth[sensor_id] = new openni::VideoFrameRef;
	vp_frame_rgb[sensor_id] = new openni::VideoFrameRef;

	vOpenDevices.push_back(sensor_id);

	rc = DEVICE_ID_PTR->open((*DEVICE_LIST_PTR)[sensor_id].getUri());

	//    bool serial_found = false;
	//    for(sensor_id=0; sensor_id < numDevices; sensor_id++)
	//    {
	//      if(serial_num == static_cast<int>((*DEVICE_LIST_PTR)[sensor_id].getUsbProductId()) )
	//      {
	//        serial_found = true;
	//        vOpenDevices.push_back(sensor_id);
	//        rc = DEVICE_ID_PTR->open((*DEVICE_LIST_PTR)[sensor_id].getUri());
	//        break;
	//      }
	//    }
	//    if(!serial_found)
	//    {
	//      THROW_EXCEPTION_CUSTOM_MSG1("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
	//      openni::OpenNI::shutdown();
	//      return;
	//    }

	if(rc != openni::STATUS_OK)
	{
		THROW_EXCEPTION_CUSTOM_MSG1("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		//		printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
	}
	//cout << endl << "Do we have IR sensor? " << DEVICE_ID_PTR->hasSensor(openni::SENSOR_IR);
	//cout << endl << "Do we have RGB sensor? " << DEVICE_ID_PTR->hasSensor(openni::SENSOR_COLOR);
	//cout << endl << "Do we have Depth sensor? " << DEVICE_ID_PTR->hasSensor(openni::SENSOR_DEPTH);

	char serialNumber[1024];
	DEVICE_ID_PTR->getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER, &serialNumber);
	cout << "Serial " << serialNumber << endl;

	//								Create RGB and Depth channels
	//========================================================================================
	rc = DEPTH_STREAM_ID_PTR->create(*DEVICE_ID_PTR, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = DEPTH_STREAM_ID_PTR->start();
		if (rc != openni::STATUS_OK)
		{
			THROW_EXCEPTION_CUSTOM_MSG1("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			//			printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			DEPTH_STREAM_ID_PTR->destroy();
		}
	}
	else
	{
		printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	rc = RGB_STREAM_ID_PTR->create(*DEVICE_ID_PTR, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		rc = RGB_STREAM_ID_PTR->start();
		if (rc != openni::STATUS_OK)
		{
			THROW_EXCEPTION_CUSTOM_MSG1("Couldn't start infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
			//			printf("Couldn't start infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
			RGB_STREAM_ID_PTR->destroy();
		}
	}
	else
	{
		printf("Couldn't find infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	if (!DEPTH_STREAM_ID_PTR->isValid() || !RGB_STREAM_ID_PTR->isValid())
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

	options = RGB_STREAM_ID_PTR->getVideoMode();
	printf("\nInitial resolution RGB (%d, %d)", options.getResolutionX(), options.getResolutionY());
	options.setResolution(width,height);
	options.setFps(fps);
	options.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	rc = RGB_STREAM_ID_PTR->setVideoMode(options);
	if (rc!=openni::STATUS_OK)
	{
		printf("Failed to change RGB resolution!\n");
		return;
	}
	rc = RGB_STREAM_ID_PTR->setMirroringEnabled(false);

	options = DEPTH_STREAM_ID_PTR->getVideoMode();
	printf("\nInitial resolution Depth(%d, %d)", options.getResolutionX(), options.getResolutionY());
	options.setResolution(width,height);
	options.setFps(30);
	options.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	rc = DEPTH_STREAM_ID_PTR->setVideoMode(options);
	if (rc!=openni::STATUS_OK)
	{
		printf("Failed to change depth resolution!\n");
		return;
	}
	rc = DEPTH_STREAM_ID_PTR->setMirroringEnabled(false);

	options = DEPTH_STREAM_ID_PTR->getVideoMode();
	printf("\nNew resolution (%d, %d) \n", options.getResolutionX(), options.getResolutionY());

	//Allow detection of closer points (although they will flicker)
	bool CloseRange;
	DEPTH_STREAM_ID_PTR->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, 1);
	DEPTH_STREAM_ID_PTR->getProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, &CloseRange);
	printf("\nClose range: %s", CloseRange?"On\n":"Off\n");


	//	// Setup:
	//	if(m_initial_tilt_angle!=360) // 360 means no motor command.
	//    setTiltAngleDegrees(m_initial_tilt_angle);

	mrpt::system::sleep(2000); // Sleep 2s
	cout << "Device " << sensor_id << " opens succesfully.\n";
#else 
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

void COpenNI2_RGBD360::close(unsigned sensor_id)
{
#if MRPT_HAS_OPENNI2
	DEPTH_STREAM_ID_PTR->destroy();
	RGB_STREAM_ID_PTR->destroy();

	if(DEPTH_FRAME_ID_PTR)
		delete DEPTH_FRAME_ID_PTR;
	if(RGB_FRAME_ID_PTR)
		delete RGB_FRAME_ID_PTR;

	if(DEPTH_STREAM_ID_PTR)
		delete DEPTH_STREAM_ID_PTR;
	if(RGB_STREAM_ID_PTR)
		delete RGB_STREAM_ID_PTR;

	if(DEVICE_ID_PTR)
		delete DEVICE_ID_PTR;

	for(vector<unsigned>::iterator it=vOpenDevices.begin(); it != vOpenDevices.end(); it++)
		if(sensor_id == *it)
			vOpenDevices.erase(it);
#else 
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

/** The main data retrieving function, to be called after calling loadConfig() and initialize().
*  \param out_obs The output retrieved observation (only if there_is_obs=true).
*  \param there_is_obs If set to false, there was no new observation.
*  \param hardware_error True on hardware/comms error.
*
* \sa doProcess
*/
void COpenNI2_RGBD360::getNextObservation(
	//	mrpt::slam::CObservation3DRangeScan &_out_obs,
	//	vector<mrpt::slam::CObservation3DRangeScanPtr> _out_obs,
	CObservationRGBD360 &_out_obs,
	bool &there_is_obs,
	bool &hardware_error )
{
#if MRPT_HAS_OPENNI2
	CTicTac	tictac;
	tictac.Tic();
	cout << "COpenNI2_RGBD360::getNextObservation \n";

	there_is_obs=false;
	hardware_error = false;

	CObservationRGBD360  newObs;
	// Set intensity image ----------------------
	if (m_grab_rgb)
		newObs.hasIntensityImage  = true;
	// Set range image --------------------------
	if (m_grab_depth || m_grab_3D_points)
		newObs.hasRangeImage = true;

	newObs.timestamp = mrpt::system::getCurrentTime();

	for(unsigned sensor_id=0; sensor_id < NUM_SENSORS; sensor_id++)
	{
		cout << "Get sensor " << sensor_id << " \n";

		// Read a frame (depth + rgb)
		DEPTH_STREAM_ID_PTR->readFrame(DEPTH_FRAME_ID_PTR);

		RGB_STREAM_ID_PTR->readFrame(RGB_FRAME_ID_PTR);
		cout << "Get sensor_1 \n";

		if ((DEPTH_FRAME_ID_PTR->getWidth() != RGB_FRAME_ID_PTR->getWidth()) || (DEPTH_FRAME_ID_PTR->getHeight() != RGB_FRAME_ID_PTR->getHeight()))
		{
			cout << "\nBoth frames don't have the same size.";
		}
		else
		{
			there_is_obs=true;

			newObs.timestamps[sensor_id] = mrpt::system::getCurrentTime();

			// Read one frame
			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)DEPTH_FRAME_ID_PTR->getData();
			const openni::RGB888Pixel* pRgbRow = (const openni::RGB888Pixel*)RGB_FRAME_ID_PTR->getData();
			int rowSize = DEPTH_FRAME_ID_PTR->getStrideInBytes() / sizeof(openni::DepthPixel);

			utils::CImage iimage(width,height,CH_RGB);
			newObs.rangeImage_setSize(height, width, sensor_id);
			for (int yc = 0; yc < DEPTH_FRAME_ID_PTR->getHeight(); ++yc)
			{
				const openni::DepthPixel* pDepth = pDepthRow;
				const openni::RGB888Pixel* pRgb = pRgbRow;
				for (int xc = 0; xc < DEPTH_FRAME_ID_PTR->getWidth(); ++xc, ++pDepth, ++pRgb)
				{
					newObs.rangeImages[sensor_id](yc,xc) = (*pDepth)*1.0/1000;
					iimage.setPixel(xc,yc,(pRgb->r<<16)+(pRgb->g<<8)+pRgb->b);
				}

				pDepthRow += rowSize;
				pRgbRow += rowSize;
			}
			newObs.intensityImages[sensor_id] = iimage;
		}
	}

	//      // Read one frame
	//      const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)DEPTH_FRAME_ID_PTR->getData();
	//      const openni::RGB888Pixel* pRgbRow = (const openni::RGB888Pixel*)RGB_FRAME_ID_PTR->getData();
	//      int rowSize = DEPTH_FRAME_ID_PTR->getStrideInBytes() / sizeof(openni::DepthPixel);
	//
	//      utils::CImage iimage(width,height,CH_RGB);
	//      for (int yc = 0; yc < DEPTH_FRAME_ID_PTR->getHeight(); ++yc)
	//      {
	//        const openni::DepthPixel* pDepth = pDepthRow;
	//        const openni::RGB888Pixel* pRgb = pRgbRow;
	//        for (int xc = 0; xc < DEPTH_FRAME_ID_PTR->getWidth(); ++xc, ++pDepth, ++pRgb)
	//        {
	//          newObs.rangeImage(yc,xc) = (*pDepth)*1.0/1000;
	//          iimage.setPixel(xc,yc,(pRgb->r<<16)+(pRgb->g<<8)+pRgb->b);
	//
	//          //newObs.intensityImage.setPixel(xc,yc,(*pRgb));
	//        }
	//
	//        pDepthRow += rowSize;
	//        pRgbRow += rowSize;
	//      }
	//      newObs.intensityImage = iimage;


	// Save the observation to the user's object:
	_out_obs = newObs;
	cout << "Copy observation \n";

	//      _out_obs[sensor_id] = CObservation3DRangeScan::Create();
	//      _out_obs[sensor_id]->swap(newObs);
	//
	//      // Set common data into observation:
	//      // --------------------------------------
	//      _out_obs[sensor_id]->sensorLabel = m_sensorLabel;
	//      _out_obs[sensor_id]->timestamp = mrpt::system::now();
	//      _out_obs[sensor_id]->sensorPose = m_sensorPoseOnRobot;
	//    //	_out_obs[sensor_id]->relativePoseIntensityWRTDepth = m_relativePoseIntensityWRTDepth;
	//
	//      // 3D point cloud:
	//      if ( _out_obs[sensor_id]->hasRangeImage && m_grab_3D_points )
	//      {
	//        _out_obs[sensor_id]->project3DPointsFromDepthImage();
	//
	//        if ( !m_grab_depth )
	//        {
	//          _out_obs[sensor_id]->hasRangeImage = false;
	//          _out_obs[sensor_id]->rangeImage.resize(0,0);
	//        }
	//
	//      }
	//
	//    }

	// preview in real-time?
	for(unsigned sensor_id=0; sensor_id < NUM_SENSORS; sensor_id++)
		if (m_preview_window)
		{
			if ( _out_obs.hasRangeImage )
			{
				if (++m_preview_decim_counter_range>m_preview_window_decimation)
				{
					m_preview_decim_counter_range=0;
					if (!m_win_range[sensor_id])	{ m_win_range[sensor_id] = mrpt::gui::CDisplayWindow::Create("Preview RANGE"); m_win_range[sensor_id]->setPos(5,5+250*sensor_id); }

					// Normalize the image
					mrpt::utils::CImage  img;
					img.setFromMatrix(_out_obs.rangeImages[sensor_id]);
					CMatrixFloat r = _out_obs.rangeImages[sensor_id] * float(1.0/this->m_maxRange);
					m_win_range[sensor_id]->showImage(img);
				}
			}
			if ( _out_obs.hasIntensityImage )
			{
				if (++m_preview_decim_counter_rgb>m_preview_window_decimation)
				{
					m_preview_decim_counter_rgb=0;
					if (!m_win_int[sensor_id])		{ m_win_int[sensor_id] = mrpt::gui::CDisplayWindow::Create("Preview INTENSITY"); m_win_int[sensor_id]->setPos(330,5+250*sensor_id); }
					m_win_int[sensor_id]->showImage(_out_obs.intensityImages[sensor_id] );
				}
			}
		}
		else
		{
			if (m_win_range[sensor_id]) m_win_range[sensor_id].clear();
			if (m_win_int[sensor_id]) m_win_int[sensor_id].clear();
		}
		cout << "getNextObservation took " << 1000*tictac.Tac() << "ms\n";
#else 
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}


/* -----------------------------------------------------
setPathForExternalImages
----------------------------------------------------- */
void COpenNI2_RGBD360::setPathForExternalImages( const std::string &directory )
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
