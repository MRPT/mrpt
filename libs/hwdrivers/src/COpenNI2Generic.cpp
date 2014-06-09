/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h" // Precompiled header

#include <mrpt/hwdrivers/COpenNI2Generic.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/system/threads.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

#if MRPT_HAS_OPENNI2
#	include <OpenNI.h>
#	include <PS1080.h>
#endif

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::slam;
using namespace mrpt::synch;
using namespace std;

#define DEVICE_LIST_PTR (reinterpret_cast<openni::Array<openni::DeviceInfo>* >(deviceListPtr))
#define DEVICE_ID_PTR (reinterpret_cast<openni::Device*>(vp_devices[sensor_id]))
#define DEPTH_STREAM_ID_PTR (reinterpret_cast<openni::VideoStream*>(vp_depth_stream[sensor_id]))
#define RGB_STREAM_ID_PTR (reinterpret_cast<openni::VideoStream*>(vp_rgb_stream[sensor_id]))
#define DEPTH_FRAME_ID_PTR (reinterpret_cast<openni::VideoFrameRef*>(vp_frame_depth[sensor_id]))
#define RGB_FRAME_ID_PTR (reinterpret_cast<openni::VideoFrameRef*>(vp_frame_rgb[sensor_id]))

// Initialize static member
std::vector<unsigned> COpenNI2Generic::vOpenDevices = std::vector<unsigned>();

/*
void openni::VideoMode::setResolution()
Setter function for the resolution of this VideoMode. Application use of this function is not recommended.
Instead, use SensorInfo::getSupportedVideoModes() to obtain a list of valid video modes

-- cited from OpenNI2 help. setResolution() is not recommended.
*/
#if MRPT_HAS_OPENNI2
bool setONI2StreamMode(openni::VideoStream& stream, int w, int h, int fps, openni::PixelFormat format)
{
//std::cout << "Ask mode: " << w << "x" << h << " " << fps << " fps. format " << format << std::endl;
	bool found = false;
	const openni::Array<openni::VideoMode>& modes = stream.getSensorInfo().getSupportedVideoModes();
	for(int i = 0, i_end = modes.getSize();i < i_end;++i){
//	  std::cout << "Mode: " << modes[i].getResolutionX() << "x" << modes[i].getResolutionY() << " " << modes[i].getFps() << " fps. format " << modes[i].getPixelFormat() << std::endl;
		if(modes[i].getResolutionX() != w){
			continue;
		}
		if(modes[i].getResolutionY() != h){
			continue;
		}
		if(modes[i].getFps() != fps){
			continue;
		}
		if(modes[i].getPixelFormat() != format){
			continue;
		}
		openni::Status rc = stream.setVideoMode(modes[i]);
		if(rc != openni::STATUS_OK){
			printf("%s:Couldn't find RGB stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
			return false;
		}
		return true;
	}
	return false;
}
#endif


/*-------------------------------------------------------------
ctor
-------------------------------------------------------------*/
COpenNI2Generic::COpenNI2Generic() :
	numDevices(0),
  width(640),
  height(480),
	fps(30),
  m_grab_image(true),
	m_grab_depth(true),
	m_grab_3D_points(true)
{
#if MRPT_HAS_OPENNI2
	rgb_pFormat = new openni::PixelFormat(openni::PIXEL_FORMAT_RGB888);
	depth_pFormat = new openni::PixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
#endif
}

/*-------------------------------------------------------------
dtor
-------------------------------------------------------------*/
COpenNI2Generic::~COpenNI2Generic()
{
#if MRPT_HAS_OPENNI2
  delete reinterpret_cast<openni::PixelFormat*>(rgb_pFormat);
  delete reinterpret_cast<openni::PixelFormat*>(depth_pFormat);
#endif
}

/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
int COpenNI2Generic::getConnectedDevices()
{
#if MRPT_HAS_OPENNI2
	int rc = openni::OpenNI::initialize();
	if(rc != openni::STATUS_OK)
		THROW_EXCEPTION(mrpt::format("After initialization:\n %s\n", openni::OpenNI::getExtendedError()))

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
	}
	else
    cout << "COpenNI2Generic initializes correctly.\n";

  return numDevices;

#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

void COpenNI2Generic::kill()
{
#if MRPT_HAS_OPENNI2
    for(unsigned i=0; COpenNI2Generic::vOpenDevices.size(); i++)
        this->close(COpenNI2Generic::vOpenDevices[i]);

    if(DEVICE_LIST_PTR)
        delete DEVICE_LIST_PTR; // Delete the pointer to the list of devices

    openni::OpenNI::shutdown();
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

bool COpenNI2Generic::isOpen(const unsigned sensor_id) const
{
	for(unsigned i=0; i < COpenNI2Generic::vOpenDevices.size(); i++)
		if(sensor_id == COpenNI2Generic::vOpenDevices[i])
			return true;

	return false;
}

bool COpenNI2Generic::initONI2RGBStream(unsigned sensor_id, int w, int h, int fps, void *pFormat)
{
	openni::Status rc = openni::STATUS_OK;
	rc = RGB_STREAM_ID_PTR->create(*DEVICE_ID_PTR, openni::SENSOR_COLOR);
	if(rc != openni::STATUS_OK){
		printf("%s: Couldn't find RGB stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		return false;
	}
	rc = RGB_STREAM_ID_PTR->setMirroringEnabled(false);
	if (rc != openni::STATUS_OK){
		printf("%s: setMirroringEnabled(false) failed:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		return false;
	}
	openni::VideoMode options = RGB_STREAM_ID_PTR->getVideoMode();
	printf("Initial resolution RGB (%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	if(setONI2StreamMode(*RGB_STREAM_ID_PTR, w, h, fps, *(reinterpret_cast<openni::PixelFormat*>(pFormat))) == false){
		printf("%s: Can't find desired rgb mode\n", __FUNCTION__ );
		return false;
	}
	options = RGB_STREAM_ID_PTR->getVideoMode();
	printf("  -> (%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	rc = RGB_STREAM_ID_PTR->start();
	if (rc != openni::STATUS_OK){
		printf("%s: Couldn't start RGB stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		RGB_STREAM_ID_PTR->destroy();
		return false;
	}
	return true;
}

bool COpenNI2Generic::initONI2DepthStream(unsigned sensor_id, int w, int h, int fps, void *pFormat)
{
	openni::Status rc = DEPTH_STREAM_ID_PTR->create(*DEVICE_ID_PTR, openni::SENSOR_DEPTH);
	if (rc != openni::STATUS_OK){
		printf("%s: Couldn't find depth stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		return false;
	}
	openni::VideoMode options = DEPTH_STREAM_ID_PTR->getVideoMode();
	printf("Initial resolution Depth(%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	if(setONI2StreamMode(*DEPTH_STREAM_ID_PTR, w, h, fps, *(reinterpret_cast<openni::PixelFormat*>(pFormat))) == false){
		printf("%s: Can't find desired depth mode\n", __FUNCTION__ );
		return false;
	}
	options = DEPTH_STREAM_ID_PTR->getVideoMode();
	printf("  -> (%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	DEPTH_STREAM_ID_PTR->setMirroringEnabled(false);
	rc = DEPTH_STREAM_ID_PTR->start();
	if (rc != openni::STATUS_OK){
		printf(" Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		DEPTH_STREAM_ID_PTR->destroy();
		return false;
	}
	return true;
}

void COpenNI2Generic::open(unsigned sensor_id)
{
#if MRPT_HAS_OPENNI2
	cout << "OpenNI2Sensor::open " << sensor_id << " ...\n";
//	cout << "Already Open (" << COpenNI2Generic::vOpenDevices.size() << ") :\n";
//	for(unsigned i=0; i < COpenNI2Generic::vOpenDevices.size(); i++)
//    cout << COpenNI2Generic::vOpenDevices[i] << endl;
//  cout << endl;

	if(isOpen(sensor_id))
	{
		//		close(sensor_id);
		cout << "The sensor " << sensor_id << " is already opened\n";
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
		vp_frame_depth[sensor_id] = new openni::VideoFrameRef;
		vp_frame_rgb[sensor_id] = new openni::VideoFrameRef;

		COpenNI2Generic::vOpenDevices.push_back(sensor_id);

		rc = DEVICE_ID_PTR->open((*DEVICE_LIST_PTR)[sensor_id].getUri());

		//    bool serial_found = false;
		//    for(sensor_id=0; sensor_id < numDevices; sensor_id++)
		//    {
		//      if(serial_num == static_cast<int>((*DEVICE_LIST_PTR)[sensor_id].getUsbProductId()) )
		//      {
		//        serial_found = true;
		//        COpenNI2Generic::vOpenDevices.push_back(sensor_id);
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
		openni::OpenNI::shutdown();
	}

//	char serialNumber[1024];
//	DEVICE_ID_PTR->getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER, &serialNumber);
//	cout << "Serial " << serialNumber << endl;

	//								Create RGB and Depth channels
	//========================================================================================
	initONI2DepthStream(sensor_id, width, height, fps, depth_pFormat); // reinterpret_cast<void*>(depth_pFormat)

  // Check that the OpenNI2 sensor has RGB camera
	m_has_color = DEVICE_ID_PTR->hasSensor(openni::SENSOR_COLOR);
	if(m_has_color)
	{
    initONI2RGBStream(sensor_id, width, height, fps, rgb_pFormat);
    if (DEVICE_ID_PTR->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)){
      rc = DEVICE_ID_PTR->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    }else{
      cout << "Device doesn't do image registration!" << endl;
    }
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

	//Allow detection of closer points (although they will flicker)
	bool CloseRange;
	DEPTH_STREAM_ID_PTR->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, 1);
	DEPTH_STREAM_ID_PTR->getProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, &CloseRange);
	printf("\nClose range: %s", CloseRange?"On\n":"Off\n");


	//	// Setup:
	//	if(m_initial_tilt_angle!=360) // 360 means no motor command.
	//    setTiltAngleDegrees(m_initial_tilt_angle);

	mrpt::system::sleep(2000); // Sleep 2s
	cout << "Device " << sensor_id << " open successfully.\n\n"; //cout << "Streaming " << width << "x" << height << " open successfully.\n\n";
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

void COpenNI2Generic::close(unsigned sensor_id)
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

	for(vector<unsigned>::iterator it=COpenNI2Generic::vOpenDevices.begin(); it != COpenNI2Generic::vOpenDevices.end(); it++)
		if(sensor_id == *it)
		{
			COpenNI2Generic::vOpenDevices.erase(it);
		  break;
		}
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

/** The main data retrieving function, to be called after calling loadConfig() and initialize().
*  \param out_obs The output retrieved observation (only if there_is_obs=true).
*  \param timestamp The timestamp of the capture (only if there_is_obs=true).
*  \param there_is_obs If set to false, there was no new observation.
*  \param hardware_error True on hardware/comms error.
*  \param sensor_id The index of the sensor accessed.
*
*/
void COpenNI2Generic::getNextFrameRGB(
	mrpt::utils::CImage &rgb_img,
	uint64_t &timestamp,
	bool &there_is_obs,
	bool &hardware_error,
	unsigned sensor_id )
{
#if MRPT_HAS_OPENNI2
//	cout << "COpenNI2Generic::getNextObservation \n";

	there_is_obs=false;
	hardware_error = false;

  if(!m_has_color)
    THROW_EXCEPTION("This OpenNI2 device does not support color imaging")

  // Read a frame (rgb)
  RGB_STREAM_ID_PTR->readFrame(RGB_FRAME_ID_PTR);

  there_is_obs=true;

  timestamp = mrpt::system::getCurrentTime();

  // Read one frame
  const openni::RGB888Pixel* pRgbRow = (const openni::RGB888Pixel*)RGB_FRAME_ID_PTR->getData();
  int rowSize = RGB_FRAME_ID_PTR->getStrideInBytes() / sizeof(openni::RGB888Pixel);

  utils::CImage iimage(width,height,CH_RGB);
  for (int yc = 0; yc < RGB_FRAME_ID_PTR->getHeight(); ++yc)
  {
    const openni::RGB888Pixel* pRgb = pRgbRow;
    for (int xc = 0; xc < RGB_FRAME_ID_PTR->getWidth(); ++xc, ++pRgb)
      iimage.setPixel(xc,yc,(pRgb->r<<16)+(pRgb->g<<8)+pRgb->b);

    pRgbRow += rowSize;
  }

  // Save the observation to the user's object:
  rgb_img.swap(iimage);

#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

/** The main data retrieving function, to be called after calling loadConfig() and initialize().
*  \param depth_img The output retrieved depth image (only if there_is_obs=true).
*  \param timestamp The timestamp of the capture (only if there_is_obs=true).
*  \param there_is_obs If set to false, there was no new observation.
*  \param hardware_error True on hardware/comms error.
*  \param sensor_id The index of the sensor accessed.
*
*/
void COpenNI2Generic::getNextFrameD(
	mrpt::math::CMatrix &depth_img,
    uint64_t &timestamp,
	bool &there_is_obs,
	bool &hardware_error,
	unsigned sensor_id )
{
#if MRPT_HAS_OPENNI2
	cout << "COpenNI2Generic::getNextObservation \n";

	there_is_obs = false;
	hardware_error = false;

    // Read a frame (depth)
    DEPTH_STREAM_ID_PTR->readFrame(DEPTH_FRAME_ID_PTR);

    there_is_obs=true;

    timestamp = mrpt::system::getCurrentTime();

    // Read one frame
    const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)DEPTH_FRAME_ID_PTR->getData();
    int rowSize = DEPTH_FRAME_ID_PTR->getStrideInBytes() / sizeof(openni::DepthPixel);

//    depth_img = new mrpt::math::CMatrix(DEPTH_FRAME_ID_PTR->getHeight(), DEPTH_FRAME_ID_PTR->getWidth());
    depth_img.resize(DEPTH_FRAME_ID_PTR->getHeight(), DEPTH_FRAME_ID_PTR->getWidth());
    for (int yc = 0; yc < DEPTH_FRAME_ID_PTR->getHeight(); ++yc)
    {
      const openni::DepthPixel* pDepth = pDepthRow;
      for (int xc = 0; xc < DEPTH_FRAME_ID_PTR->getWidth(); ++xc, ++pDepth)
        depth_img(yc,xc) = (*pDepth)*1.0/1000;

      pDepthRow += rowSize;
    }

#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

/** The main data retrieving function, to be called after calling loadConfig() and initialize().
*  \param out_obs The output retrieved observation (only if there_is_obs=true).
*  \param there_is_obs If set to false, there was no new observation.
*  \param hardware_error True on hardware/comms error.
*  \param sensor_id The index of the sensor accessed.
*
*/
void COpenNI2Generic::getNextFrameRGBD(
	mrpt::slam::CObservation3DRangeScan &out_obs,
	bool &there_is_obs,
	bool &hardware_error,
	unsigned sensor_id )
{
#if MRPT_HAS_OPENNI2
//	cout << "COpenNI2Generic::getNextFrameRGBD \n";

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
      if(!m_has_color)
        THROW_EXCEPTION("This OpenNI2 device does not support color imaging")
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
		out_obs.swap(newObs);

		// Set common data into observation:
		// --------------------------------------
		out_obs.timestamp = mrpt::system::now();
	}
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}
