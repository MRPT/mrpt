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

#include <mrpt/hwdrivers/COpenNI2Sensor.h>
#include <mrpt/utils/CTimeLogger.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::synch;

IMPLEMENTS_GENERIC_SENSOR(COpenNI2Sensor,mrpt::hwdrivers)

#define DEVICE_LIST_PTR (reinterpret_cast< openni::Array<openni::DeviceInfo>* >(deviceListPtr))
#define DEVICE_ID_PTR (reinterpret_cast<openni::Device*>(vp_devices[sensor_id]))
//#define DEPTH_STREAM_PTR (reinterpret_cast<openni::VideoStream*>(p_depth_stream))
//#define RGB_STREAM_PTR (reinterpret_cast<openni::VideoStream*>(p_rgb_stream))
//#define DEPTH_FRAME_PTR (reinterpret_cast<openni::VideoFrameRef*>(framed))
//#define RGB_FRAME_PTR (reinterpret_cast<openni::VideoFrameRef*>(framergb))
#define DEPTH_STREAM_ID_PTR (reinterpret_cast<openni::VideoStream*>(vp_depth_stream[sensor_id]))
#define RGB_STREAM_ID_PTR (reinterpret_cast<openni::VideoStream*>(vp_rgb_stream[sensor_id]))
#define DEPTH_FRAME_ID_PTR (reinterpret_cast<openni::VideoFrameRef*>(vp_frame_depth[sensor_id]))
#define RGB_FRAME_ID_PTR (reinterpret_cast<openni::VideoFrameRef*>(vp_frame_rgb[sensor_id]))


/*-------------------------------------------------------------
		ctor
 -------------------------------------------------------------*/
COpenNI2Sensor::COpenNI2Sensor() :
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

	m_relativePoseIntensityWRTDepth(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0)),
	m_user_device_number(0),
	m_grab_image(true),
	m_grab_depth(true),
	m_grab_3D_points(true)
{

	// Default label:
	m_sensorLabel = "OPENNI2";

	// =========== Default params ===========
	// ----- RGB -----
//	m_cameraParamsRGB.ncols = 640; // By default set 640x480, on connect we'll update this.
//	m_cameraParamsRGB.nrows = 480;
	m_cameraParamsRGB.ncols = width;
	m_cameraParamsRGB.nrows = height;

	m_cameraParamsRGB.cx(328.94272028759258);
	m_cameraParamsRGB.cy(267.48068171871557);
	m_cameraParamsRGB.fx(529.2151);
	m_cameraParamsRGB.fy(525.5639);

	m_cameraParamsRGB.dist.zeros();

	// ----- Depth -----
//	m_cameraParamsDepth.ncols = 640;
//	m_cameraParamsDepth.nrows = 488;
	m_cameraParamsDepth.ncols = width;
	m_cameraParamsDepth.nrows = height;

	m_cameraParamsDepth.cx(339.30781);
	m_cameraParamsDepth.cy(242.7391);
	m_cameraParamsDepth.fx(594.21434);
	m_cameraParamsDepth.fy(591.04054);

	m_cameraParamsDepth.dist.zeros();
}

/*-------------------------------------------------------------
			dtor
 -------------------------------------------------------------*/
COpenNI2Sensor::~COpenNI2Sensor()
{
//cout << "Destroy COpenNI2Sensor... \n";
//
////	this->close();
//  for(unsigned i=0; vOpenDevices.size(); i++)
//    this->close(vOpenDevices[i]);
//
//  if(DEVICE_LIST_PTR)
//    delete DEVICE_LIST_PTR; // Delete the pointer to the list of devices
//
//	openni::OpenNI::shutdown();
}

/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void COpenNI2Sensor::initialize()
{
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

  if(numDevices == 0)
  {
    cout << "No devices connected -> EXIT\n";
    return;
  }
cout << "COpenNI2Sensor initializes correctly.\n";

	open(m_user_device_number);


//	if (!checkCOMisOpen()) return;
//
//	if (!turnOn())
//	{
//		cerr << "[CHokuyoURG::initialize] Error initializing RGBD scanner" << endl;
//		return;
//	}
}

/** This method will be invoked at a minimum rate of "process_rate" (Hz)
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void COpenNI2Sensor::doProcess()
{
cout << "COpenNI2Sensor::doProcess...\n";

	bool	thereIs, hwError;

	CObservation3DRangeScanPtr newObs = CObservation3DRangeScan::Create();

  assert(!vOpenDevices.empty());
  unsigned sensor_id = vOpenDevices.front();
	getNextObservation( *newObs, thereIs, hwError, sensor_id );

	if (hwError)
	{
		m_state = ssError;
	    THROW_EXCEPTION("Couldn't communicate to the OpenNI2 sensor!");
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
void  COpenNI2Sensor::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
cout << "COpenNI2Sensor::loadConfig_sensorSpecific...\n";

	m_sensorPoseOnRobot.setFromValues(
                                    configSource.read_float(iniSection,"pose_x",0),
                                    configSource.read_float(iniSection,"pose_y",0),
                                    configSource.read_float(iniSection,"pose_z",0),
                                    DEG2RAD( configSource.read_float(iniSection,"pose_yaw",0) ),
                                    DEG2RAD( configSource.read_float(iniSection,"pose_pitch",0) ),
                                    DEG2RAD( configSource.read_float(iniSection,"pose_roll",0) )
                                    );

//#ifdef MRPT_OS_WINDOWS
//	m_com_port = configSource.read_string(iniSection, "COM_port_WIN", m_com_port, true );
//#else
//	m_com_port = configSource.read_string(iniSection, "COM_port_LIN", m_com_port, true );
//#endif

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
		std::cout << "[COpenNI2Sensor::loadConfig_sensorSpecific] Warning: Ignoring error loading calibration parameters:\n" << e.what();
	}
	m_cameraParamsDepth = sc.leftCamera;
	m_cameraParamsRGB   = sc.rightCamera;
	m_relativePoseIntensityWRTDepth = twist + mrpt::poses::CPose3D(sc.rightCameraPose);

	// Id:
	m_user_device_number = configSource.read_int(iniSection,"device_number",m_user_device_number );
cout << "LOAD m_user_device_number " << m_user_device_number << endl;

	m_grab_image = configSource.read_bool(iniSection,"grab_image",m_grab_image);
	m_grab_depth = configSource.read_bool(iniSection,"grab_depth",m_grab_depth);
	m_grab_3D_points = configSource.read_bool(iniSection,"grab_3D_points",m_grab_3D_points);

	{
		std::string s = configSource.read_string(iniSection,"relativePoseIntensityWRTDepth","");
		if (!s.empty())
			m_relativePoseIntensityWRTDepth.fromString(s);
	}

}


///*-------------------------------------------------------------
//						bindIO
//-------------------------------------------------------------*/
//void  COpenNI2Sensor::bindIO( CStream	*streamIO )
//{
//	m_csChangeStream.enter();
//	m_stream = streamIO;
//	m_csChangeStream.leave();
//}
//
///*-------------------------------------------------------------
//						switchSensorOn
//-------------------------------------------------------------*/
//bool  COpenNI2Sensor::switchSensorOn()
//{
//	char			cmd[20];
//	char			rcv_status0,rcv_status1;
//	char			rcv_data[100];
//	size_t			toWrite;
//	int				rcv_dataLength;
//
//	if (!checkCOMisOpen()) return false;
//
//	printf_debug("[COpenNI2Sensor::switchSensorOn] Switching sensor ON...");
//
//	// Send command:
//	os::strcpy(cmd,20, "BM\x0A");
//	toWrite = 3;
//
//	m_stream->WriteBuffer(cmd,toWrite);
//
//	// Receive response:
//	if (!receiveResponse( cmd, rcv_status0,rcv_status1, rcv_data, rcv_dataLength ) )
//	{
//		printf("ERROR!\n");
//		return false;
//	}
//
//	// DECODE:
//	if (rcv_status0!='0')
//	{
//		printf("ERROR!\n");
//		return false;
//	}
//
//	printf_debug("OK\n");
//
//	return true;
//}
//
///*-------------------------------------------------------------
//						switchSensorOff
//-------------------------------------------------------------*/
//bool  COpenNI2Sensor::switchSensorOff()
//{
//	char			cmd[20];
//	char			rcv_status0,rcv_status1;
//	char			rcv_data[100];
//	size_t			toWrite;
//	int				rcv_dataLength;
//
//	if (!checkCOMisOpen()) return false;
//
//	printf_debug("[COpenNI2Sensor::switchSensorOff] Switching sensor OFF...");
//
//	// Send command:
//	os::strcpy(cmd,20, "QT\x0A");
//	toWrite = 3;
//
//	m_stream->WriteBuffer(cmd,toWrite);
//
//	// Receive response:
//	if (!receiveResponse( cmd, rcv_status0,rcv_status1, rcv_data, rcv_dataLength ) )
//	{
//		printf("ERROR!\n");
//		return false;
//	}
//
//	// DECODE:
//	if (rcv_status0!='0')
//	{
//		printf("ERROR!\n");
//		return false;
//	}
//
//	printf_debug("OK\n");
//	return true;
//}
//
///*-------------------------------------------------------------
//						turnOn
//-------------------------------------------------------------*/
//bool  COpenNI2Sensor::turnOn()
//{
//	MRPT_START
//
//	// Bound?
//	if (!checkCOMisOpen()) return false;
//
//	// If we are over a serial link, set it up:
//	if ( m_ip_dir.empty() )
//	{
//		CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
//
//		if (COM!=NULL)
//		{
//			// It is a COM:
//			COM->setConfig( 19200 );
//			COM->setTimeouts(100,0,200,0,0);
//
//			// Assure the sensor is off and quiet:
//			switchSensorOff();
//			mrpt::system::sleep(10);
//
//			COM->purgeBuffers();
//			mrpt::system::sleep(10);
//
//			COM->setConfig( 115200 );
//			switchSensorOff();
//			mrpt::system::sleep(10);
//			COM->purgeBuffers();
//			mrpt::system::sleep(10);
//			COM->setConfig( 19200 );
//		}
//
//		if (COM!=NULL)
//		{
//			// Set 115200 baud rate:
//			setHighBaudrate();
//			enableSCIP20();
//			COM->setConfig( 115200 );
//		}
//	}
//	else
//	{
//		CClientTCPSocket* COM = dynamic_cast<CClientTCPSocket*>(m_stream);
//
//
//		if ( COM!=NULL )
//		{
//			// Assure the sensor is off and quiet:
//			switchSensorOff();
//			mrpt::system::sleep(10);
//
//			purgeBuffers();
//			mrpt::system::sleep(10);
//
//			switchSensorOff();
//			mrpt::system::sleep(10);
//			purgeBuffers();
//		}
//
//	}
//
//	if (!enableSCIP20()) return false;
//
//	// Turn on the sensor:
//	if (!switchSensorOn()) return false;
//
//	// Set the motor speed:
//	if (m_motorSpeed_rpm)
//		if (!setMotorSpeed( m_motorSpeed_rpm )) return false;
//
//	// Set HS mode:
//	setHighSensitivityMode(m_highSensMode);
//
//	// Display sensor information:
//	if (!displaySensorInfo(&m_sensor_info )) return false;
//
//	// Set for scanning angles:
//	m_firstRange = m_sensor_info.scan_first;
//	m_lastRange  = m_sensor_info.scan_last;
//
//	// Artificially reduced FOV?
//	if (m_reduced_fov>0 && m_reduced_fov<2*M_PI)
//	{
//		int center=(m_lastRange+m_firstRange)>>1;
//		const int half_range = static_cast<int>((m_sensor_info.scans_per_360deg / 360) * RAD2DEG(m_reduced_fov))>>1;
//		m_firstRange = center-half_range;
//		m_lastRange = center+half_range;
//		cout << "[RGBD::turnOn] Using reduced FOV: ranges [" << m_firstRange  << "-" << m_lastRange << "] for " << RAD2DEG(m_reduced_fov) << " deg. FOV" << endl;
//	}
//
//	if (!displayVersionInfo()) {
//		//return false; // It's not SO important
//	}
//
//	// Start!
//	if (!startScanningMode()) return false;
//
//	return true;
//
//	MRPT_END
//}
//
///*-------------------------------------------------------------
//						turnOff
//-------------------------------------------------------------*/
//bool  COpenNI2Sensor::turnOff()
//{
//	// Turn off the sensor:
//	if (!switchSensorOff()) return false;
//
//	return true;
//}
//
///*-------------------------------------------------------------
//						checkCOMisOpen
//-------------------------------------------------------------*/
//bool  COpenNI2Sensor::checkCOMisOpen()
//{
//	MRPT_START
//
//	if (m_stream)
//	{
//		// Socket or USB connection?
//		if ( !m_ip_dir.empty() && m_port_dir )
//		{
//			// Has the port been disconected (USB serial ports)??
//			CClientTCPSocket* COM = dynamic_cast<CClientTCPSocket*>(m_stream);
//
//			if (COM!=NULL)
//			{
//				if (COM->isConnected())
//					return true;
//
//				// It has been disconnected... try to reconnect:
//				cerr << "[COpenNI2Sensor] Socket connection lost! trying to reconnect..." << endl;
//
//				try
//				{
//					COM->connect( m_ip_dir, m_port_dir );
//					// OK, reconfigure the sensor:
//					turnOn();
//					return true;
//				}
//				catch (...)
//				{
//					// Not yet..
//					return false;
//				}
//			}
//			else
//			{
//				return true;		// Assume OK
//			}
//		}
//		else
//		{
//			// Has the port been disconected (USB serial ports)??
//			CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
//			if (COM!=NULL)
//			{
//				if (COM->isOpen())
//					return true;
//
//				// It has been disconnected... try to reconnect:
//				cerr << "[COpenNI2Sensor] Serial port connection lost! Trying to reconnect..." << endl;
//
//				try
//				{
//					COM->open();
//					// OK, reconfigure the sensor:
//					turnOn();
//					return true;
//				}
//				catch (...)
//				{
//					// Not yet..
//					return false;
//				}
//			}
//			else
//			{
//				return true;		// Assume OK
//			}
//		}
//	}
//	else
//	{
//		if ( m_com_port.empty() && m_ip_dir.empty() && !m_port_dir )
//		{
//			THROW_EXCEPTION("No stream bound to the sensor nor COM serial port or ip and port provided in 'm_com_port','m_ip_dir' and 'm_port_dir'");
//		}
//
//		if ( !m_ip_dir.empty() && m_port_dir )
//		{
//			// Try to open the serial port:
//			CClientTCPSocket	*theCOM = new CClientTCPSocket();
//
//			theCOM->connect( m_ip_dir, m_port_dir );
//
//			if (!theCOM->isConnected())
//			{
//				cerr << "[COpenNI2Sensor] Cannot connect with the server '" << m_com_port << "'" << endl;
//				delete theCOM;
//				return false;
//			}
//
//			// Bind:
//			bindIO( theCOM );
//
//			m_I_am_owner_serial_port=true;
//
//		}
//
//		else
//		{
//			// Try to open the serial port:
//			CSerialPort	*theCOM = new CSerialPort(m_com_port, true);
//
//			if (!theCOM->isOpen())
//			{
//				cerr << "[COpenNI2Sensor] Cannot open serial port '" << m_com_port << "'" << endl;
//				delete theCOM;
//				return false;
//			}
//
//			// Bind:
//			bindIO( theCOM );
//
//			m_I_am_owner_serial_port=true;
//
//		}
//
//		return true;
//	}
//	MRPT_END
//}


bool COpenNI2Sensor::isOpen(const unsigned sensor_id) const
{
  for(unsigned i=0; vOpenDevices.size(); i++)
    if(sensor_id == vOpenDevices[i])
      return true;

  return false;
}

//void COpenNI2Sensor::open(const int serial_num)
void COpenNI2Sensor::open(unsigned sensor_id)
{
cout << "OpenNI2Sensor::open " << sensor_id << " ...\n";
	if(isOpen(sensor_id))
	{
//		close(sensor_id);
    cout << "The sensor " << sensor_id << "is already opened\n";
    return;
	}

	if (!numDevices)
		THROW_EXCEPTION("No OpenNI2 devices found.")

	if (sensor_id >= numDevices)
		THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")

  int rc;
//  unsigned sensor_id; // To use with serial_num

	// Open the given device number:
//  if(sensor_id == 100)
//  {
//    const char* deviceURI = openni::ANY_DEVICE;
//    rc = DEVICE_ID_PTR->open(deviceURI);
//  }
//  else
  {
    vp_devices[sensor_id] = new openni::Device;
    vp_depth_stream[sensor_id] = new openni::VideoStream;
    vp_rgb_stream[sensor_id] = new openni::VideoStream;
  //	p_depth_stream = new openni::VideoStream;
  //	p_rgb_stream = new openni::VideoStream;
    vp_frame_depth[sensor_id] = new openni::VideoFrameRef;
    vp_frame_rgb[sensor_id] = new openni::VideoFrameRef;
  //	framed = new openni::VideoFrameRef;
  //	framergb = new openni::VideoFrameRef;

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
  }

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
cout << "Device " << sensor_id << " opens succesfully.\n\n";
}

void COpenNI2Sensor::close(unsigned sensor_id)
{
//  assert();
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
}

/** The main data retrieving function, to be called after calling loadConfig() and initialize().
  *  \param out_obs The output retrieved observation (only if there_is_obs=true).
  *  \param there_is_obs If set to false, there was no new observation.
  *  \param hardware_error True on hardware/comms error.
  *
  * \sa doProcess
  */
void COpenNI2Sensor::getNextObservation(
	mrpt::slam::CObservation3DRangeScan &_out_obs,
	bool &there_is_obs,
	bool &hardware_error,
	unsigned sensor_id )
{
//CTicTac	tictac;
//tictac.Tic();
cout << "COpenNI2Sensor::getNextObservation \n";

	there_is_obs=false;
	hardware_error = false;

  // Read a frame (depth + rgb)
  DEPTH_STREAM_ID_PTR->readFrame(DEPTH_FRAME_ID_PTR);

  RGB_STREAM_ID_PTR->readFrame(RGB_FRAME_ID_PTR);

  if ((DEPTH_FRAME_ID_PTR->getWidth() != RGB_FRAME_ID_PTR->getWidth()) || (DEPTH_FRAME_ID_PTR->getHeight() != RGB_FRAME_ID_PTR->getHeight()))
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
    const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)DEPTH_FRAME_ID_PTR->getData();
    const openni::RGB888Pixel* pRgbRow = (const openni::RGB888Pixel*)RGB_FRAME_ID_PTR->getData();
    int rowSize = DEPTH_FRAME_ID_PTR->getStrideInBytes() / sizeof(openni::DepthPixel);

    utils::CImage iimage(width,height,CH_RGB);
    for (int yc = 0; yc < DEPTH_FRAME_ID_PTR->getHeight(); ++yc)
    {
      const openni::DepthPixel* pDepth = pDepthRow;
      const openni::RGB888Pixel* pRgb = pRgbRow;
      for (int xc = 0; xc < DEPTH_FRAME_ID_PTR->getWidth(); ++xc, ++pDepth, ++pRgb)
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
//  cout << "getNextObservation took " << 1000*tictac.Tac() << "ms\n";
}


/* -----------------------------------------------------
				setPathForExternalImages
----------------------------------------------------- */
void COpenNI2Sensor::setPathForExternalImages( const std::string &directory )
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
