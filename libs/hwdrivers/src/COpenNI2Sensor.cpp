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
#include <mrpt/hwdrivers/COpenNI2Sensor.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/TStereoCamera.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>


using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::synch;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace std;
using mrpt::utils::DEG2RAD;

bool isValidParameter(const mrpt::utils::TCamera& param){
  return param.ncols > 0 && param.nrows > 0;
}

IMPLEMENTS_GENERIC_SENSOR(COpenNI2Sensor,mrpt::hwdrivers)

/*-------------------------------------------------------------
ctor
-------------------------------------------------------------*/
COpenNI2Sensor::COpenNI2Sensor() :
	m_sensorPoseOnRobot(),
	m_preview_window(false),
	m_preview_window_decimation(1),
	m_preview_decim_counter_range(0),
	m_preview_decim_counter_rgb(0),

	m_relativePoseIntensityWRTDepth(0,0,0, DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-90)),
	m_user_device_number(0),
	m_serial_number(0)
{

	// Default label:
	m_sensorLabel = "OPENNI2";

	// =========== Default params ===========
	// ----- RGB -----
	m_cameraParamsRGB.ncols = 0;
	m_cameraParamsRGB.nrows = 0;

	m_cameraParamsRGB.cx(-1);
	m_cameraParamsRGB.cy(-1);
	m_cameraParamsRGB.fx(-1);
	m_cameraParamsRGB.fy(-1);

	m_cameraParamsRGB.dist.zeros();

	// ----- Depth -----
	m_cameraParamsDepth.ncols = 0;
	m_cameraParamsDepth.nrows = 0;

	m_cameraParamsDepth.cx(-1);
	m_cameraParamsDepth.cy(-1);
	m_cameraParamsDepth.fx(-1);
	m_cameraParamsDepth.fy(-1);

	m_cameraParamsDepth.dist.zeros();
}

/*-------------------------------------------------------------
dtor
-------------------------------------------------------------*/
COpenNI2Sensor::~COpenNI2Sensor()
{
#if MRPT_HAS_OPENNI2
  close(m_user_device_number);
#endif // MRPT_HAS_OPENNI2
}

/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void COpenNI2Sensor::initialize()
{
#if MRPT_HAS_OPENNI2
  try{
    if(getConnectedDevices() <= 0){ // Check and list the available devices. If there is at least one device connected, open the first in the list.
      return;
    }
	{
    if(m_serial_number != 0)
    {
      openDeviceBySerial(m_serial_number);
      if(getDeviceIDFromSerialNum(m_serial_number, m_user_device_number) == false){
        THROW_EXCEPTION(mrpt::format("Failed to find sensor_id from serial number(%d).", m_serial_number))
      }
    }
    else
      open(m_user_device_number);
	}
    if(isOpen(m_user_device_number) == false){
      THROW_EXCEPTION(mrpt::format("Failed to open OpenNI2 device(%d).", m_user_device_number))
    }
    /* If camera parameter is not read from ini file, we get the parameters from OpenNI2. */
    if(isValidParameter(m_cameraParamsDepth) == false){
      if(getDepthSensorParam(m_cameraParamsDepth, m_user_device_number) == false){
        THROW_EXCEPTION("Failed to get Depth camera parameters.")
      }
    }
    if(isValidParameter(m_cameraParamsRGB) == false){
      if(getColorSensorParam(m_cameraParamsRGB, m_user_device_number) == false){
        THROW_EXCEPTION("Failed to get RGB camera parameters.")
      }
    }
  }catch(std::logic_error& e){
    throw(e);
  }
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

/** This method will be invoked at a minimum rate of "process_rate" (Hz)
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void COpenNI2Sensor::doProcess()
{
#if MRPT_HAS_OPENNI2
//	cout << "COpenNI2Sensor::doProcess...\n";

	bool	thereIs, hwError;

	CObservation3DRangeScanPtr newObs = CObservation3DRangeScan::Create();

    assert(getNumDevices() > 0);
	getNextObservation( *newObs, thereIs, hwError );

	if (hwError)
	{
		m_state = ssError;
		THROW_EXCEPTION("Couldn't communicate to the OpenNI2 sensor!");
	}

	if (thereIs)
	{
		m_state = ssWorking;

		std::vector<mrpt::utils::CSerializablePtr> objs;
		if (m_grab_image || m_grab_depth || m_grab_3D_points)  objs.push_back(newObs);

		appendObservations( objs );
	}
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
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

	m_preview_window = configSource.read_bool(iniSection,"preview_window",m_preview_window);

    m_width  = configSource.read_int(iniSection,"width",0);
    m_height = configSource.read_int(iniSection,"height",0);
    m_fps    = configSource.read_float(iniSection,"fps",0);
    std::cout << "width " << m_width << " height " << m_height << " fps " << m_fps << endl;

    bool hasRightCameraSection = configSource.sectionExists(iniSection + string("_RIGHT"));
    bool hasLeftCameraSection  = configSource.sectionExists(iniSection + string("_LEFT"));
    bool hasLeft2RightPose     = configSource.sectionExists(iniSection + string("_LEFT2RIGHT_POSE"));

	mrpt::utils::TStereoCamera  sc;

	try {
		sc.loadFromConfigFile(iniSection,configSource);
	} catch (std::exception &e) {
		std::cout << "[COpenNI2Sensor::loadConfig_sensorSpecific] Warning: Ignoring error loading calibration parameters:\n" << e.what();
	}
    if(hasRightCameraSection){
    m_cameraParamsRGB   = sc.rightCamera;
    }
    if(hasLeftCameraSection){
    m_cameraParamsDepth = sc.leftCamera;
    }
    if(hasLeft2RightPose){
    const mrpt::poses::CPose3D  twist(0,0,0,DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-90));
    m_relativePoseIntensityWRTDepth = twist + mrpt::poses::CPose3D(sc.rightCameraPose);
    }

	// Id:
	m_user_device_number = configSource.read_int(iniSection, "device_number", m_user_device_number);
	//cout << "LOAD m_user_device_number " << m_user_device_number << endl;
	m_serial_number = configSource.read_int(iniSection, "serial_number", m_serial_number);

	m_grab_image = configSource.read_bool(iniSection,"grab_image",m_grab_image);
	m_grab_depth = configSource.read_bool(iniSection,"grab_depth",m_grab_depth);
	m_grab_3D_points = configSource.read_bool(iniSection,"grab_3D_points",m_grab_3D_points);

	{
		std::string s = configSource.read_string(iniSection,"relativePoseIntensityWRTDepth","");
		if (!s.empty())
			m_relativePoseIntensityWRTDepth.fromString(s);
	}

}


/** The main data retrieving function, to be called after calling loadConfig() and initialize().
*  \param out_obs The output retrieved observation (only if there_is_obs=true).
*  \param there_is_obs If set to false, there was no new observation.
*  \param hardware_error True on hardware/comms error.
*
* \sa doProcess
*/
void COpenNI2Sensor::getNextObservation(
	mrpt::obs::CObservation3DRangeScan &out_obs,
	bool &there_is_obs,
	bool &hardware_error)
{
#if MRPT_HAS_OPENNI2
//	cout << "COpenNI2Sensor::getNextObservation \n";

    // Read a frame (depth + rgb)
    getNextFrameRGBD(out_obs, there_is_obs, hardware_error, m_user_device_number );


    // Set common data into observation:
    // --------------------------------------
    out_obs.sensorLabel = m_sensorLabel;
    out_obs.sensorPose = m_sensorPoseOnRobot;
    out_obs.relativePoseIntensityWRTDepth = m_relativePoseIntensityWRTDepth;

    out_obs.cameraParams          = m_cameraParamsDepth;
    out_obs.cameraParamsIntensity = m_cameraParamsRGB;

    // 3D point cloud:
    if ( out_obs.hasRangeImage && m_grab_3D_points )
    {
        out_obs.project3DPointsFromDepthImage();

        if ( !m_grab_depth )
        {
            out_obs.hasRangeImage = false;
            out_obs.rangeImage.resize(0,0);
        }

    }

    // preview in real-time?
    if (m_preview_window)
    {
        if ( out_obs.hasRangeImage )
        {
            if (++m_preview_decim_counter_range>m_preview_window_decimation)
            {
                m_preview_decim_counter_range=0;
                if (!m_win_range)	{ m_win_range = mrpt::gui::CDisplayWindow::Create("Preview RANGE"); m_win_range->setPos(5,5); }

                // Normalize the image
                mrpt::utils::CImage  img;
                img.setFromMatrix(out_obs.rangeImage);
                CMatrixFloat r = out_obs.rangeImage * float(1.0/this->m_maxRange);
                m_win_range->showImage(img);
            }
        }
        if ( out_obs.hasIntensityImage )
        {
            if (++m_preview_decim_counter_rgb>m_preview_window_decimation)
            {
                m_preview_decim_counter_rgb=0;
                if (!m_win_int)		{ m_win_int = mrpt::gui::CDisplayWindow::Create("Preview INTENSITY"); m_win_int->setPos(300,5); }
                m_win_int->showImage(out_obs.intensityImage );
            }
        }
    }
    else
    {
        if (m_win_range) m_win_range.clear();
        if (m_win_int) m_win_int.clear();
    }

//	cout << "COpenNI2Sensor::getNextObservation finish\n";
#else
	MRPT_UNUSED_PARAM(out_obs); MRPT_UNUSED_PARAM(there_is_obs); MRPT_UNUSED_PARAM(hardware_error);
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}


/* -----------------------------------------------------
setPathForExternalImages
----------------------------------------------------- */
void COpenNI2Sensor::setPathForExternalImages( const std::string &directory )
{
	MRPT_UNUSED_PARAM(directory);
	// Ignore for now. It seems performance is better grabbing everything
	// to a single big file than creating hundreds of smaller files per second...
	return;

	//	if (!mrpt::system::createDirectory( directory ))
	//	{
	//		THROW_EXCEPTION_CUSTOM_MSG1("Error: Cannot create the directory for externally saved images: %s",directory.c_str() )
	//	}
	//	m_path_for_external_images = directory;
}
