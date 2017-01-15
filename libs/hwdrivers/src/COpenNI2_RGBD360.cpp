/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h" // Precompiled header

#include <mrpt/hwdrivers/COpenNI2_RGBD360.h>
#include <mrpt/obs/CObservationRGBD360.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/system/threads.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace mrpt::synch;
using namespace mrpt::math;
using namespace std;
using mrpt::utils::CTicTac;
using mrpt::utils::DEG2RAD;
using mrpt::obs::CObservationRGBD360;

IMPLEMENTS_GENERIC_SENSOR(COpenNI2_RGBD360,mrpt::hwdrivers)

/*-------------------------------------------------------------
ctor
-------------------------------------------------------------*/
COpenNI2_RGBD360::COpenNI2_RGBD360() :
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
	kill();
#endif // MRPT_HAS_OPENNI2
}

/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
void COpenNI2_RGBD360::initialize()
{
#if MRPT_HAS_OPENNI2
    // Check and list the available devices
	getConnectedDevices();

  if(getNumDevices() < NUM_SENSORS)
	{
		cout << "Num required sensors " << NUM_SENSORS << endl;
		cout << "Not enough devices connected -> EXIT\n";
		return;
	}
	cout << "COpenNI2_RGBD360 initializes correctly.\n";

	for(unsigned sensor_id=0; sensor_id < (unsigned int)NUM_SENSORS; sensor_id++)
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
#if MRPT_HAS_OPENNI2
	cout << "COpenNI2_RGBD360::doProcess...\n";

	bool	thereIs, hwError;

	CObservationRGBD360Ptr newObs = CObservationRGBD360::Create();
	//	CObservation3DRangeScanPtr newObs = CObservation3DRangeScan::Create();

   assert(getNumDevices() > 0);
	//  unsigned sensor_id = COpenNI2Generic::vOpenDevices.front();
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
		if (m_grab_rgb || m_grab_depth || m_grab_3D_points) objs.push_back(newObs);

		appendObservations( objs );
	}
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
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

	m_width = configSource.read_int(iniSection,"width",0);
	m_height = configSource.read_int(iniSection,"height",0);
	m_fps = configSource.read_float(iniSection,"fps",0);
	std::cout << "width " << m_width << " height " << m_height << " fps " << m_fps << endl;

	m_grab_rgb = configSource.read_bool(iniSection,"grab_image",m_grab_rgb);
	m_grab_depth = configSource.read_bool(iniSection,"grab_depth",m_grab_depth);
	m_grab_3D_points = configSource.read_bool(iniSection,"grab_3D_points",m_grab_3D_points);

	//  m_num_sensors = configSource.read_int(iniSection,"m_num_sensors",0);
}


/** The main data retrieving function, to be called after calling loadConfig() and initialize().
*  \param out_obs The output retrieved observation (only if there_is_obs=true).
*  \param there_is_obs If set to false, there was no new observation.
*  \param hardware_error True on hardware/comms error.
*
* \sa doProcess
*/
void COpenNI2_RGBD360::getNextObservation(
	CObservationRGBD360 &out_obs,
	bool &there_is_obs,
	bool &hardware_error )
{
#if MRPT_HAS_OPENNI2
	CTicTac	tictac;
	tictac.Tic();
//	cout << "COpenNI2_RGBD360::getNextObservation \n";

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

	for(unsigned sensor_id=0; sensor_id < (unsigned int)NUM_SENSORS; sensor_id++)
	{
//		cout << "Get sensor " << sensor_id << " \n";
    bool there_is_obs, hardware_error;
    getNextFrameRGB(newObs.intensityImages[sensor_id],newObs.timestamps[sensor_id], there_is_obs, hardware_error, sensor_id);
    getNextFrameD(newObs.rangeImages[sensor_id],newObs.timestamps[sensor_id], there_is_obs, hardware_error, sensor_id);
	}


	// preview in real-time?
	for(unsigned sensor_id=0; sensor_id < (unsigned int)NUM_SENSORS; sensor_id++) {
		if (m_preview_window)
		{
			if ( out_obs.hasRangeImage )
			{
				if (++m_preview_decim_counter_range>m_preview_window_decimation)
				{
					m_preview_decim_counter_range=0;
					if (!m_win_range[sensor_id])	{ m_win_range[sensor_id] = mrpt::gui::CDisplayWindow::Create("Preview RANGE"); m_win_range[sensor_id]->setPos(5,5+250*sensor_id); }

					// Normalize the image
					mrpt::utils::CImage  img;
					img.setFromMatrix(out_obs.rangeImages[sensor_id]);
					CMatrixFloat r = out_obs.rangeImages[sensor_id] * float(1.0/this->m_maxRange);
					m_win_range[sensor_id]->showImage(img);
				}
			}
			if ( out_obs.hasIntensityImage )
			{
				if (++m_preview_decim_counter_rgb>m_preview_window_decimation)
				{
					m_preview_decim_counter_rgb=0;
					if (!m_win_int[sensor_id])		{ m_win_int[sensor_id] = mrpt::gui::CDisplayWindow::Create("Preview INTENSITY"); m_win_int[sensor_id]->setPos(330,5+250*sensor_id); }
					m_win_int[sensor_id]->showImage(out_obs.intensityImages[sensor_id] );
				}
			}
		}
		else
		{
			if (m_win_range[sensor_id]) m_win_range[sensor_id].clear();
			if (m_win_int[sensor_id]) m_win_int[sensor_id].clear();
		}
	}
	cout << "getNextObservation took " << 1000*tictac.Tac() << "ms\n";
#else
	MRPT_UNUSED_PARAM(out_obs); MRPT_UNUSED_PARAM(there_is_obs); MRPT_UNUSED_PARAM(hardware_error);
	MRPT_UNUSED_PARAM(out_obs);
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}


/* -----------------------------------------------------
setPathForExternalImages
----------------------------------------------------- */
void COpenNI2_RGBD360::setPathForExternalImages( const std::string &directory )
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
