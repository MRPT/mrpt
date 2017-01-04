/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h" // Precompiled header

#include <mrpt/hwdrivers/COpenNI2Generic.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/system/threads.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

#if MRPT_HAS_OPENNI2

// This seems to be assumed by OpenNI.h and undefined for some reason in GCC/Ubuntu
#if !defined(MRPT_OS_WINDOWS)
#   define linux 1
#endif

#	include <OpenNI.h>
#	include <PS1080.h>
#endif

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace mrpt::synch;
using namespace std;


// Initialize static member
std::vector<stlplus::smart_ptr<COpenNI2Generic::CDevice> > COpenNI2Generic::vDevices = std::vector<stlplus::smart_ptr<COpenNI2Generic::CDevice> >();
int COpenNI2Generic::numInstances = 0;

#if MRPT_HAS_OPENNI2
bool        setONI2StreamMode(openni::VideoStream& stream, int w, int h, int fps, openni::PixelFormat format);
std::string oni2DevInfoStr(const openni::DeviceInfo& info, int tab = 0);
bool        cmpONI2Device(const openni::DeviceInfo& i1, const openni::DeviceInfo& i2);
#endif // MRPT_HAS_OPENNI2


#include <mrpt/hwdrivers/COpenNI2Generic_CDevice.h>
/*-------------------------------------------------------------
ctor
-------------------------------------------------------------*/
COpenNI2Generic::COpenNI2Generic() :
	m_width(640),
	m_height(480),
	m_fps(30.0f),
#if MRPT_HAS_OPENNI2
	m_rgb_format(openni::PIXEL_FORMAT_RGB888),
	m_depth_format(openni::PIXEL_FORMAT_DEPTH_1_MM),
#endif // MRPT_HAS_OPENNI2
	m_verbose(false),
	m_grab_image(true),
	m_grab_depth(true),
	m_grab_3D_points(true)
{
	const char * sVerbose = getenv("MRPT_HWDRIVERS_VERBOSE");
	m_verbose = (sVerbose!=NULL) && atoi(sVerbose)!=0;
	// Start automatically:
	if (!this->start()) {
#if MRPT_HAS_OPENNI2
		THROW_EXCEPTION(mrpt::format("After initialization:\n %s\n", openni::OpenNI::getExtendedError()))
#endif
	}
}

COpenNI2Generic::COpenNI2Generic(int width, int height, float fps, bool open_streams_now ) :
	m_width(width),
	m_height(height),
	m_fps(fps),
#if MRPT_HAS_OPENNI2
	m_rgb_format(openni::PIXEL_FORMAT_RGB888),
	m_depth_format(openni::PIXEL_FORMAT_DEPTH_1_MM),
#endif // MRPT_HAS_OPENNI2
	m_verbose(false),
	m_grab_image(true),
	m_grab_depth(true),
	m_grab_3D_points(true)
{
	const char * sVerbose = getenv("MRPT_HWDRIVERS_VERBOSE");
	m_verbose = (sVerbose!=NULL) && atoi(sVerbose)!=0;
	// Open?
	if (open_streams_now) {
		if (!this->start()) {
#if MRPT_HAS_OPENNI2
			THROW_EXCEPTION(mrpt::format("After initialization:\n %s\n", openni::OpenNI::getExtendedError()))
#endif
		}
	}
}

bool COpenNI2Generic::start()
{
#if MRPT_HAS_OPENNI2
	if(numInstances == 0){
		if(openni::OpenNI::initialize() != openni::STATUS_OK){
			return false;
		}else{
			std::cerr << "[" << __FUNCTION__ << "]" << std::endl << " Initialized OpenNI2." << std::endl;
		}
	}
	numInstances++;
	return true;
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

/*-------------------------------------------------------------
dtor
-------------------------------------------------------------*/
COpenNI2Generic::~COpenNI2Generic()
{
	numInstances--;
	if(numInstances == 0){
		kill();
	}
}

int COpenNI2Generic::getNumDevices()const{
	return vDevices.size();
}

void COpenNI2Generic::setVerbose(bool verbose){
	m_verbose = verbose;
}

bool COpenNI2Generic::isVerbose()const{
	return m_verbose;
}

void  COpenNI2Generic::showLog(const std::string& message)const{
	if(isVerbose() == false){
	return;
	}
	std::cerr << message;
}
/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
int COpenNI2Generic::getConnectedDevices()
{
#if MRPT_HAS_OPENNI2
	// Get devices list
	openni::Array<openni::DeviceInfo> oni2InfoArray;
	openni::OpenNI::enumerateDevices(&oni2InfoArray);

	const size_t numDevices = oni2InfoArray.getSize();
	showLog(mrpt::format("[%s]\n", __FUNCTION__));
	showLog(mrpt::format(" Get device list. %d devices connected.\n", (int)numDevices));

	// Search new devices.
	std::set<int> newDevices;
	for(unsigned i=0; i < numDevices; i++){
		const openni::DeviceInfo& info = oni2InfoArray[i];
		showLog(mrpt::format("  Device[%d]\n", i));
		showLog(oni2DevInfoStr(info, 3) + "\n");

		bool isExist = false;
		for(unsigned int j = 0, j_end = vDevices.size();j < j_end && isExist == false;++j){
			if(cmpONI2Device(info, vDevices[j]->getInfo())){
				isExist = true;
			}
		}
		if(isExist == false){
			newDevices.insert(i);
		}
	}
	// Add new devices to device list(static member).
	for(std::set<int>::const_iterator it = newDevices.begin(), it_end = newDevices.end();it != it_end;++it){
		const openni::DeviceInfo& info = oni2InfoArray[*it];
		CDevice::Ptr device = CDevice::create(info, (openni::PixelFormat)m_rgb_format, (openni::PixelFormat)m_depth_format, m_verbose);
		vDevices.push_back(device);
		{
			unsigned int sn;
			if (device->getSerialNumber(sn)) {
				showLog(mrpt::format("Device[%d]: serial number: `%u`\n",*it,sn) );
			}
		}
	}

	if(getNumDevices() == 0){
	  showLog(" No devices connected -> EXIT\n");
	}else{
	  showLog(mrpt::format(" %d devices were found.\n", getNumDevices()));
	}
	return getNumDevices();
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

void COpenNI2Generic::kill()
{
#if MRPT_HAS_OPENNI2
	vDevices.clear();
	openni::OpenNI::shutdown();
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

bool COpenNI2Generic::isOpen(const unsigned sensor_id) const
{
#if MRPT_HAS_OPENNI2
  if((int)sensor_id >= getNumDevices()){
		return false;
	}
	return vDevices[sensor_id]->isOpen();
#else
	MRPT_UNUSED_PARAM(sensor_id);
    THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

void COpenNI2Generic::open(unsigned sensor_id)
{
#if MRPT_HAS_OPENNI2
	// Sensor index validation.
	if (!getNumDevices()){
		THROW_EXCEPTION("No OpenNI2 devices found.")
	}
	if ((int)sensor_id >= getNumDevices()){
		THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")
	}
	showLog(mrpt::format("[%s]\n", __FUNCTION__));
	showLog(mrpt::format(" open[%d] ...\n", sensor_id));

	if(isOpen(sensor_id)){
	  showLog(mrpt::format(" The sensor [%d] is already opened\n", sensor_id));
		return;
	}
    if (m_verbose) printf("[COpenNI2Generic] DBG: [%s] about to call vDevices[%d]->open()\n",__FUNCTION__,sensor_id);
	vDevices[sensor_id]->open(m_width, m_height, m_fps);
	showLog(vDevices[sensor_id]->getLog() + "\n");
	showLog(mrpt::format(" Device [%d] ", sensor_id));
	if(vDevices[sensor_id]->isOpen()){
	  showLog(" open successfully.\n");
	}else{
	  showLog(" open failed.\n");
	}
	mrpt::system::sleep(1000); // Sleep
#else
	MRPT_UNUSED_PARAM(sensor_id);
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

unsigned int COpenNI2Generic::openDevicesBySerialNum(const std::set<unsigned>& serial_required)
{
#if MRPT_HAS_OPENNI2
  showLog(mrpt::format("[%s]\n", __FUNCTION__));
  unsigned num_open_dev = 0;
  for(unsigned sensor_id=0; sensor_id < vDevices.size(); sensor_id++)
  {
    unsigned int serialNum;
    if(vDevices[sensor_id]->getSerialNumber(serialNum) == false){
      showLog(vDevices[sensor_id]->getLog());
      continue;
    }
    if (m_verbose) printf("[COpenNI2Generic::openDevicesBySerialNum] checking device with serial '%d'\n",serialNum);

	if(serial_required.find(serialNum) == serial_required.end()){
	  vDevices[sensor_id]->close();
      continue;
    }
	if(vDevices[sensor_id]->isOpen()){
      num_open_dev++;
      continue;
    }
    if (m_verbose) printf("[COpenNI2Generic] DBG: [%s] about to call vDevices[%d]->open(%d,%d,%d)\n",
    __FUNCTION__,sensor_id,m_width,m_height,(int)m_fps);
    if(vDevices[sensor_id]->open(m_width, m_height, m_fps) == false){
      showLog(vDevices[sensor_id]->getLog());
      continue;
    }
    num_open_dev++;
    if (m_verbose) printf("[COpenNI2Generic] DBG: [%s] now has %d devices open\n", __FUNCTION__,num_open_dev);
  }
  return num_open_dev;
#else
  MRPT_UNUSED_PARAM(serial_required);
  THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

unsigned int COpenNI2Generic::openDeviceBySerial(const unsigned int SerialRequired){
  std::set<unsigned> serial_required;
  serial_required.insert(SerialRequired);
  return openDevicesBySerialNum(serial_required);
}

bool COpenNI2Generic::getDeviceIDFromSerialNum(const unsigned int SerialRequired, int& sensor_id) const{
#if MRPT_HAS_OPENNI2
  for(size_t i = 0, i_end = vDevices.size();i < i_end;++i){
    unsigned int sn;
    if(vDevices[i]->getSerialNumber(sn) == false){
      continue;
    }
    if(sn == SerialRequired){
      sensor_id = (int)i;
      return true;
    }
  }
  return false;
#else
	MRPT_UNUSED_PARAM(SerialRequired); MRPT_UNUSED_PARAM(sensor_id);
    THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

void COpenNI2Generic::close(unsigned sensor_id)
{
#if MRPT_HAS_OPENNI2
	// Sensor index validation.
	if (!getNumDevices()){
		THROW_EXCEPTION("No OpenNI2 devices found.")
	}
	if ((int)sensor_id >= getNumDevices()){
		THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")
	}
	vDevices[sensor_id]->close();
#else
	MRPT_UNUSED_PARAM(sensor_id);
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
	// Sensor index validation.
	if (!getNumDevices()){
		THROW_EXCEPTION("No OpenNI2 devices found.")
	}
	if ((int)sensor_id >= getNumDevices()){
		THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")
	}
	if(vDevices[sensor_id]->getNextFrameRGB(rgb_img, timestamp, there_is_obs, hardware_error) == false){
	  showLog(mrpt::format("[%s]\n", __FUNCTION__ ));
	  showLog(mrpt::format(" Error [%d]th Sensor.\n", sensor_id));
	  showLog(std::string(" ") + vDevices[sensor_id]->getLog() + "\n");
	}
#else
	MRPT_UNUSED_PARAM(rgb_img); MRPT_UNUSED_PARAM(timestamp);
	MRPT_UNUSED_PARAM(there_is_obs); MRPT_UNUSED_PARAM(hardware_error); MRPT_UNUSED_PARAM(sensor_id);
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
	// Sensor index validation.
	if (getNumDevices() == 0){
		THROW_EXCEPTION("No OpenNI2 devices found.")
	}
	if ((int)sensor_id >= getNumDevices()){
		THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")
	}
	if(vDevices[sensor_id]->getNextFrameD(depth_img, timestamp, there_is_obs, hardware_error) == false){
	  showLog(mrpt::format("[%s]\n", __FUNCTION__));
	  showLog(mrpt::format(" Error [%d]th Sensor.\n", sensor_id));
	  showLog(std::string(" ") + vDevices[sensor_id]->getLog() + "\n");
	}
#else
	MRPT_UNUSED_PARAM(depth_img); MRPT_UNUSED_PARAM(timestamp);
	MRPT_UNUSED_PARAM(there_is_obs); MRPT_UNUSED_PARAM(hardware_error); MRPT_UNUSED_PARAM(sensor_id);
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
	mrpt::obs::CObservation3DRangeScan &out_obs,
	bool &there_is_obs,
	bool &hardware_error,
	unsigned sensor_id )
{
#if MRPT_HAS_OPENNI2
	// Sensor index validation.
	if (!getNumDevices()){
		THROW_EXCEPTION("No OpenNI2 devices found.")
	}
	if ((int)sensor_id >= getNumDevices()){
		THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")
	}
	if(vDevices[sensor_id]->getNextFrameRGBD(out_obs, there_is_obs, hardware_error) == false){
	  showLog(mrpt::format("[%s]\n", __FUNCTION__));
	  showLog(mrpt::format(" Error [%d]th Sensor.\n", sensor_id));
	  showLog(std::string(" ") + vDevices[sensor_id]->getLog() + "\n");
	}
#else
	MRPT_UNUSED_PARAM(out_obs); MRPT_UNUSED_PARAM(there_is_obs); MRPT_UNUSED_PARAM(hardware_error);
	MRPT_UNUSED_PARAM(sensor_id);
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

bool  COpenNI2Generic::getColorSensorParam(mrpt::utils::TCamera& param, unsigned sensor_id)const{
#if MRPT_HAS_OPENNI2
	if(isOpen(sensor_id) == false){
		return false;
	}
	return vDevices[sensor_id]->getCameraParam(CDevice::COLOR_STREAM, param);
#else
	MRPT_UNUSED_PARAM(param); MRPT_UNUSED_PARAM(sensor_id);
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

bool  COpenNI2Generic::getDepthSensorParam(mrpt::utils::TCamera& param, unsigned sensor_id)const{
#if MRPT_HAS_OPENNI2
    if(isOpen(sensor_id) == false){
		return false;
	}
	return vDevices[sensor_id]->getCameraParam(CDevice::DEPTH_STREAM, param);
#else
	MRPT_UNUSED_PARAM(param); MRPT_UNUSED_PARAM(sensor_id);
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}


#if MRPT_HAS_OPENNI2
/*
void openni::VideoMode::setResolution()
Setter function for the resolution of this VideoMode. Application use of this function is not recommended.
Instead, use SensorInfo::getSupportedVideoModes() to obtain a list of valid video modes

-- cited from OpenNI2 help. setResolution() is not recommended.
*/
bool setONI2StreamMode(openni::VideoStream& stream, int w, int h, int fps, openni::PixelFormat format){
	// std::cout << "[COpenNI2Generic] Ask mode: " << w << "x" << h << " " << fps << " fps. format " << format << std::endl;
	bool found = false;
	const openni::Array<openni::VideoMode>& modes = stream.getSensorInfo().getSupportedVideoModes();
	for(int i = 0, i_end = modes.getSize();i < i_end;++i){
	   // if (m_verbose) std::cout << "[COpenNI2Generic] Mode: " << modes[i].getResolutionX() << "x" << modes[i].getResolutionY() << " " << modes[i].getFps() << " fps. format " << modes[i].getPixelFormat() << std::endl;
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
			return false;
		}
		return true;
	}
	return false;
}

std::string oni2DevInfoStr(const openni::DeviceInfo& info, int tab){
        std::stringstream sst;
	std::string space;
	for(int i = 0;i < tab;++i){
	  space += " ";
	}
	sst << space << "name="    << info.getName()        << std::endl;
	sst << space << "uri="     << info.getUri()         << std::endl;
	sst << space << "vendor="  << info.getVendor()      << std::endl;
	sst << space << "product=" << info.getUsbProductId();
	return sst.str();
}

bool cmpONI2Device(const openni::DeviceInfo& i1, const openni::DeviceInfo& i2){
	return (strcmp(i1.getUri(), i2.getUri()) == 0);
}
//
COpenNI2Generic::CDevice::CDevice(const openni::DeviceInfo& info, openni::PixelFormat rgb, openni::PixelFormat depth,bool verbose)
 :m_info(info), m_mirror(true), m_verbose(verbose)
{
	m_streams[COLOR_STREAM] = CStream::create(m_device, openni::SENSOR_COLOR, rgb  , m_log, m_verbose);
	m_streams[IR_STREAM]    = CStream::create(m_device, openni::SENSOR_IR,    rgb  , m_log, m_verbose);
	m_streams[DEPTH_STREAM] = CStream::create(m_device, openni::SENSOR_DEPTH, depth, m_log, m_verbose);
}

COpenNI2Generic::CDevice::~CDevice(){
	close();
}

bool COpenNI2Generic::CDevice::synchMirrorMode(){
	m_mirror = false;
	// Check whether both stream support mirroring.
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		if (!m_streams[i]) continue;
		bool mirror_support;
		try{
			mirror_support = m_streams[i]->isMirrorSupported();
		}catch(std::logic_error& e){
			throw(e);
		}
		if(mirror_support == false){
		  m_log << "[" << __FUNCTION__ << "]" << std::endl;
			m_log << " openni::STREAM_PROPERTY_MIRRORING is not supported on " << m_streams[i]->getName() << "." << std::endl;
			m_log << " We assume this is MS Kinect and taken images are inverted to right and left." << std::endl;
			// In this case, getMirroringEnabled() method always returns false. So we cannot confirm whether the images are inverted or not.
			m_mirror  = true;
			break;
		}
	}
	// Set both stream to same mirror mode.
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		if (!m_streams[i]) continue;
		if(m_streams[i]->isMirrorSupported() == false){
			break;
		}
		if(m_streams[i]->setMirror(m_mirror) == false){
			return false;
		}
	}
	return true;
}

bool COpenNI2Generic::CDevice::startStreams(){
	MRPT_START
	int num_ok = 0;
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		if (!m_streams[i]) continue;
		if (m_verbose) printf("  [%s] calling m_streams[%d]->start()\n",__FUNCTION__,i);
		if(m_streams[i]->start() == false){
			if (m_verbose) printf("  [%s] m_streams[%d]->start() returned FALSE!\n",__FUNCTION__,i);
		}
		else {
			num_ok++;
		}
		if (m_verbose) printf("  [%s] m_streams[%d]->start() returned TRUE\n",__FUNCTION__,i);
	}
	if (m_verbose) printf("  [COpenNI2Generic::CDevice::startStreams()] %d streams were started.\n", num_ok);
	return num_ok>0;
	MRPT_END
}

bool COpenNI2Generic::CDevice::isOpen()const{
	return (m_streams[COLOR_STREAM] && m_streams[COLOR_STREAM]->isValid()) ||
		   (m_streams[DEPTH_STREAM] && m_streams[DEPTH_STREAM]->isValid());
}

void COpenNI2Generic::CDevice::close(){
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		if (!m_streams[i]) continue;
		m_streams[i]->destroy();
	}
	m_device.close();
}

bool COpenNI2Generic::CDevice::open(int w, int h, int fps){
	MRPT_START
	if (m_verbose) printf("  [COpenNI2Generic::CDevice::open()] Called with w=%i h=%i fps=%i\n",w,h,fps);
	clearLog();
	close();
	openni::Status rc = m_device.open(getInfo().getUri());
	if(rc != openni::STATUS_OK){
	  m_log << "[" <<  __FUNCTION__ << "]" << std::endl << " Failed to open device " << getInfo().getUri() << " " << openni::OpenNI::getExtendedError() << std::endl;
		return false;
	}
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		if(!m_streams[i]) continue;
		if (m_verbose) printf("   [%s] calling m_streams[%d]->open()\n",__FUNCTION__,i);

		if(m_streams[i]->open(w, h, fps) == false)
		{
			if (m_verbose) printf("   [%s] m_streams[%d]->open() returned FALSE\n",__FUNCTION__,i);
			return false;
		}
		if (m_verbose) printf("   [%s] m_streams[%d]->open() returned OK\n",__FUNCTION__,i);
	}

	if(synchMirrorMode() == false){
		close();
		return false;
	}

	if (m_streams[DEPTH_STREAM]) {
		int CloseRange=0;
		m_streams[DEPTH_STREAM]->setCloseRange(CloseRange);
		m_log << " Close range: " <<  (CloseRange? "On" : "Off") << std::endl;
	}

	if (m_verbose) printf("   DBG: checking if imageRegistrationMode is supported\n");
	if(m_device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) &&
	   m_streams[DEPTH_STREAM] && m_streams[DEPTH_STREAM]->isValid() &&
	   m_streams[COLOR_STREAM] && m_streams[COLOR_STREAM]->isValid() )
	{
//SEB		if(m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF) != openni::STATUS_OK){
		if(m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) != openni::STATUS_OK){
			m_log << " setImageRegistrationMode() Failed:" << openni::OpenNI::getExtendedError() << endl;
		}else{
			m_log << " setImageRegistrationMode() Success" << endl;
		}
	}else{
		m_log << "  Device doesn't do image registration!" << endl;
	}

	if (0) // hasColor())
		{ 	// printf("DBG: hasColor() returned TRUE\n");
			m_streams[COLOR_STREAM]->disableAutoExposure();
			printf("DBG: returned from disableAutoExposure()\n");
		}

	if(startStreams() == false){
		close();
		return false;
	}
	return true;
	MRPT_END
}

bool COpenNI2Generic::CDevice::getNextFrameRGB(mrpt::utils::CImage &img, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error){
	MRPT_START
	if(!hasColor()){
		THROW_EXCEPTION("This OpenNI2 device does not support color imaging")
	}
	openni::VideoFrameRef frame;
	if(m_streams[COLOR_STREAM]->getFrame(frame, timestamp, there_is_obs, hardware_error) == false){
		return false;
	}
	copyFrame<openni::RGB888Pixel, mrpt::utils::CImage>(frame, img);

	return true;
	MRPT_END
}

bool COpenNI2Generic::CDevice::getNextFrameD(mrpt::math::CMatrix &img, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error){
	MRPT_START
	if(!hasDepth()){
		THROW_EXCEPTION("This OpenNI2 device does not support depth imaging")
	}
	openni::VideoFrameRef frame;
	if(m_streams[DEPTH_STREAM]->getFrame(frame, timestamp, there_is_obs, hardware_error) == false){
		return false;
	}
	copyFrame<openni::DepthPixel, mrpt::math::CMatrix>(frame, img);

	return true;
	MRPT_END
}

bool COpenNI2Generic::CDevice::getNextFrameRGBD(mrpt::obs::CObservation3DRangeScan &obs, bool &there_is_obs, bool &hardware_error){
	MRPT_START
	clearLog();
	there_is_obs   = false;
	hardware_error = false;

	if(!hasColor()){
		THROW_EXCEPTION("This OpenNI2 device does not support color imaging")
	}
	if(!hasDepth()){
		THROW_EXCEPTION("This OpenNI2 device does not support depth imaging")
	}
	// Read a frame (depth + rgb)
	mrpt::system::TTimeStamp tm;
	openni::VideoFrameRef    frame[STREAM_TYPE_SIZE];
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		if (!m_streams[i] || !m_streams[i]->isValid()) continue;
		if(m_streams[i]->getFrame(frame[i], tm, there_is_obs, hardware_error) == false){
			return false;
		}
		if(there_is_obs == false || hardware_error == true){
			return false;
		}
	}

	const int width  = frame[COLOR_STREAM].getWidth();
	const int height = frame[COLOR_STREAM].getHeight();
	if((frame[DEPTH_STREAM].getWidth() != width) || (frame[DEPTH_STREAM].getHeight() != height)){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " Both frames don't have the same size." << std::endl;
		return false;
	}
	there_is_obs = true;
	obs.hasConfidenceImage = false;
	obs.hasIntensityImage  = true;
	obs.hasRangeImage      = true;
	obs.range_is_depth     = true;
	obs.hasPoints3D        = false;
	obs.timestamp          = mrpt::system::getCurrentTime();
	resize(obs, width, height);

	const char* data[STREAM_TYPE_SIZE] =
	{
		(const char*)frame[COLOR_STREAM].getData(),
		(const char*)frame[DEPTH_STREAM].getData()
	};
	const int   step[STREAM_TYPE_SIZE] =
	{
		frame[COLOR_STREAM].getStrideInBytes(),
		frame[DEPTH_STREAM].getStrideInBytes()
	};

	for (int yc = 0; yc < height; ++yc){
		const openni::RGB888Pixel* pRgb   = (const openni::RGB888Pixel*)data[COLOR_STREAM];
		const openni::DepthPixel * pDepth = (const openni::DepthPixel *)data[DEPTH_STREAM];
		for (int xc = 0; xc < width; ++xc, ++pDepth, ++pRgb){
			int x = xc;
			if(isMirrorMode()){
				x = width - x - 1;
			}
			setPixel(*pRgb  , obs.intensityImage, x, yc);
			setPixel(*pDepth, obs.rangeImage    , x, yc);
		}
		data[COLOR_STREAM] += step[COLOR_STREAM];
		data[DEPTH_STREAM] += step[DEPTH_STREAM];
	}

	return true;
	MRPT_END
}

COpenNI2Generic::CDevice::Ptr COpenNI2Generic::CDevice::create(const openni::DeviceInfo& info, openni::PixelFormat rgb, openni::PixelFormat depth, bool verbose){
	return Ptr(new CDevice(info, rgb, depth,verbose));
}

bool COpenNI2Generic::CDevice::getSerialNumber(std::string& sn){
  clearLog();
  openni::Status rc;
  bool isOpened = isOpen();
  if(isOpened == false){
    rc = m_device.open(getInfo().getUri());
    if(rc != openni::STATUS_OK){
      m_log << "[" <<  __FUNCTION__ << "]" << std::endl << " Failed to open device " << getInfo().getUri() << " " << openni::OpenNI::getExtendedError() << std::endl;
      return false;
    }
  }
  char serialNumber[16];
  rc = m_device.getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER, &serialNumber);
  if(rc != openni::STATUS_OK){
    m_log << "[" <<  __FUNCTION__ << "]" << std::endl << " Failed to getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER) " << getInfo().getUri() << " " << openni::OpenNI::getExtendedError() << std::endl;
    return false;
  }
  sn = std::string(serialNumber);
  if(isOpened == false){
    m_device.close();
  }
  return true;
}

bool COpenNI2Generic::CDevice::getSerialNumber(unsigned int& sn){
  std::string str;
  if(getSerialNumber(str) == false){
    return false;
  }
  std::stringstream sst;
  sst.str(str);
  sst >> sn;
  return !sst.fail();
}
//
COpenNI2Generic::CDevice::CStream::CStream(openni::Device& device, openni::SensorType type, openni::PixelFormat format, std::ostream& log,bool verbose)
	:m_log(log), m_device(device), m_strName("Unknown"), m_type(type), m_format(format), m_verbose(verbose)
{
	if(m_type == openni::SENSOR_COLOR){
		m_strName = "openni::SENSOR_COLOR";
	}else if(m_type == openni::SENSOR_DEPTH){
		m_strName = "openni::SENSOR_DEPTH";
	}else if(m_type == openni::SENSOR_IR){
		m_strName = "openni::SENSOR_IR";
	}else{
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " Unknown SensorType -> " << m_type << std::endl;
	}
}

COpenNI2Generic::CDevice::CStream::~CStream(){
	destroy();
}

bool COpenNI2Generic::CDevice::CStream::isMirrorSupported()const{
	if(isValid() == false){
		THROW_EXCEPTION(getName() + " is not opened.");
	}
	return m_stream.isPropertySupported(openni::STREAM_PROPERTY_MIRRORING);
}

bool COpenNI2Generic::CDevice::CStream::setMirror(bool flag){
	if(isValid() == false){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " " << getName() << " is not opened." << std::endl;
		return false;
	}
	if(m_stream.isPropertySupported(openni::STREAM_PROPERTY_MIRRORING) == false){
		return false;
	}
	if(m_stream.setMirroringEnabled(flag) != openni::STATUS_OK){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " setMirroringEnabled() failed: " << openni::OpenNI::getExtendedError() << std::endl;
		return false;
	}
	return true;
}

bool COpenNI2Generic::CDevice::CStream::isValid()const{
	return m_stream.isValid();
}

void COpenNI2Generic::CDevice::CStream::destroy(){
	m_stream.destroy();
}

void COpenNI2Generic::CDevice::CStream::setCloseRange(int& value){
	if (m_verbose) printf("      [CDevice::CStream::setCloseRange] entry with value=%d\n",value);
	m_stream.setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, value);
	if (m_verbose) printf("      [CDevice::CStream::setCloseRange] returned from mstream.setProperty()\n");
	m_stream.getProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, &value);
	if (m_verbose) printf("      [CDevice::CStream::setCloseRange] returned from mstream.getProperty() ... value %d\n",value);
}

bool COpenNI2Generic::CDevice::CStream::open(int w, int h, int fps){
	destroy();
	if(m_type != openni::SENSOR_COLOR && m_type != openni::SENSOR_DEPTH && m_type != openni::SENSOR_IR){  // SEB added IR
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " Unknown SensorType -> " << m_type << std::endl;
		return false;
	}
	if (m_verbose) printf("      [COpenNI2Generic::CDevice::CStream::open] opening sensor stream with m_type == %d\n",(int)m_type);
	openni::Status rc = openni::STATUS_OK;
//	if(m_type == openni::SENSOR_COLOR) {
//		m_type = openni::SENSOR_IR;
//		m_strName="openni::SENSOR_IR";	// SEB added
//		if (m_verbose) printf("DBG: changing type to SENSOR_IR (%d)\n",(int)m_type);
//	}  // added whole if stmt
	rc = m_stream.create(m_device, m_type);
	if(rc != openni::STATUS_OK){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " Couldn't find sensor "
	  	    << m_strName << ":" << openni::OpenNI::getExtendedError() << std::endl;
		if(m_type == openni::SENSOR_COLOR) {
			m_type = openni::SENSOR_IR;
			m_strName="openni::SENSOR_IR";	// SEB added
 			if (m_verbose) printf("DBG: changing type to SENSOR_IR (%d)\n",(int)m_type);
			rc = m_stream.create(m_device, m_type);
		}  // SEB added whole if stmt
		else
	  return false;
	}
    if (m_verbose) printf("returned OK from stream.create()\n");
	openni::VideoMode options = m_stream.getVideoMode();
	m_log << "[" << __FUNCTION__ << "]" << std::endl;
	m_log << " " << m_strName << std::endl;
	m_log << " " << mrpt::format("Initial resolution (%d, %d) FPS %d Format %d", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat()) << std::endl;
    if (m_verbose) printf("DBG: calling setONI2StreamMode()\n");
	if(setONI2StreamMode(m_stream, w, h, fps, m_format) == false){
		m_log << " Can't find desired mode in the " << getName() << std::endl;
		destroy();
		return false;
	}
    if (m_verbose) printf("DBG: returned OK from setONI2StreamMode()\n");
    if (m_verbose) printf("DBG: calling stream.getVideoMode()\n");
	options = m_stream.getVideoMode();
	m_log << " " << mrpt::format("-> (%d, %d) FPS %d Format %d", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat()) << std::endl;
	if (m_verbose) printf("      [COpenNI2Generic::CDevice::CStream::open] returning TRUE\n");
	return true;
}

bool COpenNI2Generic::CDevice::CStream::start(){
	if(isValid() == false){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " " << getName() << " is not opened." << std::endl;
		return false;
	}
	if(m_stream.start() != openni::STATUS_OK){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " Couldn't start " << getName() << " stream:" << openni::OpenNI::getExtendedError() << std::endl;
	  this->destroy();
	  return false;
	}
	return true;
}

COpenNI2Generic::CDevice::CStream::Ptr COpenNI2Generic::CDevice::CStream::create(openni::Device& device, openni::SensorType type, openni::PixelFormat format, std::ostream& log, bool verbose){
	return Ptr(new CStream(device, type, format, log,verbose));
}

bool COpenNI2Generic::CDevice::CStream::getFrame(openni::VideoFrameRef& frame, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error)
{
	there_is_obs   = false;
	hardware_error = false;
	if(isValid() == false){
		return false;
	}
	openni::Status rc = m_stream.readFrame(&frame);
	if(rc != openni::STATUS_OK){
		hardware_error = true;
		std::string message = mrpt::format("Failed to grab frame from %s", getName().c_str());
		THROW_EXCEPTION(message);
	}
	there_is_obs = true;
	timestamp    = mrpt::system::getCurrentTime();
	return true;
}

#endif // MRPT_HAS_OPENNI2
