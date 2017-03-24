/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef mrpt_COpenNI2Sensor_H
#define mrpt_COpenNI2Sensor_H

#include <mrpt/hwdrivers/COpenNI2Generic.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/utils/TEnumType.h>
#include <mrpt/gui/CDisplayWindow.h>

#include <mrpt/hwdrivers/link_pragmas.h>


namespace mrpt
{
	namespace hwdrivers
	{
		/** A class for grabing "range images", intensity images (either RGB or IR) and other information from an OpenNI2 sensor.
		  * This class permits to access several sensors simultaneously. The same options (resolution, fps, etc.) are used for every sensor.
		  *
		  *  <h2>Configuration and usage:</h2> <hr>
		  * Data is returned as observations of type mrpt::obs::CObservation3DRangeScan.
		  *  See those classes for documentation on their fields.
		  *
		  * As with any other CGenericSensor class, the normal sequence of methods to be called is:
		  *   - CGenericSensor::loadConfig() - Or calls to the individual setXXX() to configure the sensor parameters.
		  *   - COpenNI2Sensor::initialize() - to start the communication with the sensor.
		  *   - call COpenNI2Sensor::getNextObservation() for getting the data.
		  *
		  * <h2>Calibration parameters</h2><hr>
		  *  In this class we employ the OpenNI2 method to return depth images refered to the RGB camera. Otherwise we could specify
		  *   an accurate transformation of depth images to 3D points, you'll have to calibrate your RGBD sensor for that, and supply
		  *   the following <b>threee pieces of information</b> (default calibration data will be used otherwise, but they'll be not optimal for all sensors!):
		  *    - Camera parameters for the RGB camera. See COpenNI2Sensor::setCameraParamsIntensity()
		  *    - Camera parameters for the depth camera. See COpenNI2Sensor::setCameraParamsDepth()
		  *    - The 3D relative pose of the two cameras. See COpenNI2Sensor::setRelativePoseIntensityWrtDepth()
		  *
		  *   See http://www.mrpt.org/Kinect_calibration for a procedure to calibrate RGBD sensors with an interactive GUI program.
		  *
		  * <h2>Coordinates convention</h2><hr>
		  *   The origin of coordinates is the focal point of the depth camera, with the axes oriented as in the
		  *   diagram shown in mrpt::obs::CObservation3DRangeScan. Notice in that picture that the RGB camera is
		  *   assumed to have axes as usual in computer vision, which differ from those for the depth camera.
		  *
		  *   The X,Y,Z axes used to report the data from accelerometers coincide with those of the depth camera
		  *    (e.g. the camera standing on a table would have an ACC_Z=-9.8m/s2).
		  *
		  *   Notice however that, for consistency with stereo cameras, when loading the calibration parameters from
		  *    a configuration file, the left-to-right pose increment is expected as if both RGB & IR cameras had
		  *    their +Z axes pointing forward, +X to the right, +Y downwards (just like it's the standard in stereo cameras
		  *    and in computer vision literature). In other words: the pose stored in this class uses a different
		  *    axes convention for the depth camera than in a stereo camera, so when a pose L2R is loaded from a calibration file
		  *    it's actually converted like:
		  *
		  *      L2R(this class convention) = CPose3D(0,0,0,-90deg,0deg,-90deg) (+) L2R(in the config file)
		  *
		  *
		  * <h2>Some general comments</h2><hr>
		  *		- Depth is grabbed in millimeters
		  *		- This sensor can be also used from within rawlog-grabber to grab datasets within a robot with more sensors.
		  *		- There is no built-in threading support, so if you use this class manually (not with-in rawlog-grabber),
		  *			the ideal would be to create a thread and continuously request data from that thread (see mrpt::system::createThread ).
		  *		- The intensity channel default to the RGB images, but it can be changed with setVideoChannel() to read the IR camera images (useful for calibrating).
		  *		- There is a built-in support for an optional preview of the data on a window, so you don't need to even worry on creating a window to show them.
		  *		- This class relies on an embedded version of libfreenect (you do NOT need to install it in your system). Thanks guys for the great job!
		  *
		  * <h2>Converting to 3D point cloud </h2><hr>
		  *   You can convert the 3D observation into a 3D point cloud with this piece of code:
		  *
		  * \code
		  * mrpt::obs::CObservation3DRangeScan  obs3D;
		  * mrpt::maps::CColouredPointsMap       pntsMap;
		  * pntsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
		  * pntsMap.loadFromRangeScan(obs3D);
		  * \endcode
		  *
		  *   Then the point cloud mrpt::maps::CColouredPointsMap can be converted into an OpenGL object for
		  *    rendering with mrpt::maps::CMetricMap::getAs3DObject() or alternatively with:
		  *
		  *  \code
		  *    mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
		  *    gl_points->loadFromPointsMap(&pntsMap);
		  *  \endcode
		  *
		  *
		  * <h2>Platform-specific comments</h2><hr>
		  *   For more details, refer to <a href="http://openkinect.org/wiki/Main_Page" >libfreenect</a> documentation:
		  *		- Linux: You'll need root privileges to access Kinect. Or, install <code> MRPT/scripts/51-kinect.rules </code> in <code>/etc/udev/rules.d/</code> to allow access to all users.
		  *		- Windows:
		  *			- Since MRPT 0.9.4 you'll only need to install <a href="http://sourceforge.net/projects/libusb-win32/files/libusb-win32-releases/" >libusb-win32</a>: download and extract the latest libusb-win32-bin-x.x.x.x.zip
		  *			- To install the drivers, read this: http://openkinect.org/wiki/Getting_Started#Windows
		  *		- MacOS: (write me!)
		  *
		  *
		  * <h2>Format of parameters for loading from a .ini file</h2><hr>
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    sensorLabel     = OPENNI2       // A text description
		  *    preview_window  = false        // Show a window with a preview of the grabbed data in real-time
		  *
		  *    device_number   = 0           // Device index to open (0:first Kinect, 1:second Kinect,...)
		  *
		  *    grab_image      = true        // Grab the RGB image channel? (Default=true)
		  *    grab_depth      = true        // Grab the depth channel? (Default=true)
		  *    grab_3D_points  = true        // Grab the 3D point cloud? (Default=true) If disabled, points can be generated later on.
		  *
		  *    video_channel   = VIDEO_CHANNEL_RGB // Optional. Can be: VIDEO_CHANNEL_RGB (default) or VIDEO_CHANNEL_IR
		  *
		  *    pose_x=0	// Camera position in the robot (meters)
		  *    pose_y=0
		  *    pose_z=0
		  *    pose_yaw=0	// Angles in degrees
		  *    pose_pitch=0
		  *    pose_roll=0
		  *
		  *
		  *    // Kinect sensor calibration:
		  *    // See http://www.mrpt.org/Kinect_and_MRPT
		  *
		  *    // Left/Depth camera
		  *    [supplied_section_name_LEFT]
		  *    rawlog-grabber-ignore = true // Instructs rawlog-grabber to ignore this section (it is not a separate device!)
		  *
		  *    resolution = [640 488]
		  *    cx         = 314.649173
		  *    cy         = 240.160459
		  *    fx         = 572.882768
		  *    fy         = 542.739980
		  *    dist       = [-4.747169e-03 -4.357976e-03 0.000000e+00 0.000000e+00 0.000000e+00]    // The order is: [K1 K2 T1 T2 K3]
		  *
		  *    // Right/RGB camera
		  *    [supplied_section_name_RIGHT]
		  *    rawlog-grabber-ignore = true // Instructs rawlog-grabber to ignore this section (it is not a separate device!)
		  *
		  *    resolution = [640 480]
		  *    cx         = 322.515987
		  *    cy         = 259.055966
		  *    fx         = 521.179233
		  *    fy         = 493.033034
		  *    dist       = [5.858325e-02 3.856792e-02 0.000000e+00 0.000000e+00 0.000000e+00]    // The order is: [K1 K2 T1 T2 K3]
		  *
		  *    // Relative pose of the right camera wrt to the left camera:
		  *    // This assumes that both camera frames are such that +Z points
		  *    // forwards, and +X and +Y to the right and downwards.
		  *    // For the actual coordinates employed in 3D observations, see figure in mrpt::obs::CObservation3DRangeScan
		  *    [supplied_section_name_LEFT2RIGHT_POSE]
		  *    rawlog-grabber-ignore = true // Instructs rawlog-grabber to ignore this section (it is not a separate device!)
		  *
		  *    pose_quaternion      = [0.025575 -0.000609 -0.001462 0.999987 0.002038 0.004335 -0.001693]
		  *
		  *  \endcode
		  *
		  *  More references to read:IMPEXP mrpt
		  *		- http://http://www.openni.org/
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP  COpenNI2Sensor : public mrpt::hwdrivers::CGenericSensor, public mrpt::hwdrivers::COpenNI2Generic
		{
			DEFINE_GENERIC_SENSOR(COpenNI2Sensor)

		 public:

			COpenNI2Sensor();	 //!< Default ctor
			~COpenNI2Sensor();	 //!< Default ctor

			/** Set the serial number of the device to open.
			  *  \exception This method must throw an exception when such serial number is not found among the connected devices.
			  */
            inline void setSerialToOpen(const unsigned serial) { m_serial_number = serial; }

			/** Set the sensor_id of the device to open.
			  *  \exception This method must throw an exception when such serial number is not found among the connected devices.
			  */
            inline void setSensorIDToOpen(const unsigned sensor_id) { m_user_device_number = sensor_id; }

			/** Initializes the 3D camera - should be invoked after calling loadConfig() or setting the different parameters with the set*() methods.
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			virtual void initialize();

			/** To be called  at a high rate (>XX Hz), this method populates the internal buffer of received observations.
			  *  This method is mainly intended for usage within rawlog-grabber or similar programs.
			  *  For an alternative, see getNextObservation()
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  * \sa getNextObservation
			  */
			virtual void doProcess();

			/** The main data retrieving function, to be called after calling loadConfig() and initialize().
			  *  \param out_obs The output retrieved observation (only if there_is_obs=true).
			  *  \param there_is_obs If set to false, there was no new observation.
			  *  \param hardware_error True on hardware/comms error.
			  *
			  * \sa doProcess
			  */
			void getNextObservation(
				mrpt::obs::CObservation3DRangeScan &out_obs,
				bool &there_is_obs,
				bool &hardware_error);

			/**  Set the path where to save off-rawlog image files (this class DOES take into account this path).
			  *  An  empty string (the default value at construction) means to save images embedded in the rawlog, instead of on separate files.
			  * \exception std::exception If the directory doesn't exists and cannot be created.
			  */
			virtual void setPathForExternalImages( const std::string &directory );


			/** @name Sensor parameters (alternative to \a loadConfig ) and manual control
			    @{ */

			/** Get the maximum range (meters) that can be read in the observation field "rangeImage" */
			inline double getMaxRange() const { return m_maxRange; }

			/** Get the row count in the camera images, loaded automatically upon camera open(). */
			inline size_t getRowCount() const { return m_cameraParamsRGB.nrows; }
			/** Get the col count in the camera images, loaded automatically upon camera open(). */
			inline size_t getColCount() const { return m_cameraParamsRGB.ncols; }

			/** Get a const reference to the depth camera calibration parameters */
			inline const mrpt::utils::TCamera  & getCameraParamsIntensity() const { return m_cameraParamsRGB; }
			inline void setCameraParamsIntensity(const mrpt::utils::TCamera  &p) { m_cameraParamsRGB=p; }

			/** Get a const reference to the depth camera calibration parameters */
			inline const mrpt::utils::TCamera  & getCameraParamsDepth() const { return m_cameraParamsDepth; }
			inline void setCameraParamsDepth(const mrpt::utils::TCamera  &p) { m_cameraParamsDepth=p; }

			/** Set the pose of the intensity camera wrt the depth camera \sa See mrpt::obs::CObservation3DRangeScan for a 3D diagram of this pose */
			inline void setRelativePoseIntensityWrtDepth(const mrpt::poses::CPose3D &p) { m_relativePoseIntensityWRTDepth=p; }
			inline const mrpt::poses::CPose3D &getRelativePoseIntensityWrtDepth() const { return m_relativePoseIntensityWRTDepth; }

			/** Enable/disable the grabbing of the RGB channel */
			inline void enableGrabRGB(bool enable=true) { m_grab_image=enable; }
			inline bool isGrabRGBEnabled() const { return m_grab_image; }

			/** Enable/disable the grabbing of the depth channel */
			inline void enableGrabDepth(bool enable=true) { m_grab_depth=enable; }
			inline bool isGrabDepthEnabled() const { return m_grab_depth; }

			/** Enable/disable the grabbing of the 3D point clouds */
			inline void enableGrab3DPoints(bool enable=true) { m_grab_3D_points=enable; }
			inline bool isGrab3DPointsEnabled() const { return m_grab_3D_points; }

			/** @} */

		protected:

			virtual void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section );

			mrpt::poses::CPose3D	m_sensorPoseOnRobot;

			bool		m_preview_window; //!< Show preview window while grabbing
			size_t 		m_preview_window_decimation; //!< If preview is enabled, only show 1 out of N images.
			size_t      m_preview_decim_counter_range, m_preview_decim_counter_rgb;
			mrpt::gui::CDisplayWindowPtr  m_win_range, m_win_int;

			mrpt::utils::TCamera  	m_cameraParamsRGB;  //!< Params for the RGB camera
			mrpt::utils::TCamera  	m_cameraParamsDepth;  //!< Params for the Depth camera
			mrpt::poses::CPose3D    m_relativePoseIntensityWRTDepth; //!< See mrpt::obs::CObservation3DRangeScan for a diagram of this pose

			double  m_maxRange; //!< Sensor max range (meters)

			int  m_user_device_number; //!< Number of device to open (0:first,...)
			int  m_serial_number; //!< Serial number of device to open

		};	// End of class
	} // End of NS

} // End of NS



#endif
