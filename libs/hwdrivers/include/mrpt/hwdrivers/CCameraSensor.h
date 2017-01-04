/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CCameraSensor_H
#define CCameraSensor_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/hwdrivers/CGenericSensor.h>

#include <mrpt/hwdrivers/CFFMPEG_InputStream.h>
#include <mrpt/hwdrivers/CImageGrabber_OpenCV.h>
#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>
#include <mrpt/hwdrivers/CImageGrabber_FlyCapture2.h>
#include <mrpt/hwdrivers/CStereoGrabber_Bumblebee_libdc1394.h>
#include <mrpt/hwdrivers/CSwissRanger3DCamera.h>
#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/hwdrivers/COpenNI2Sensor.h>
#include <mrpt/hwdrivers/CDUO3DCamera.h>

#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/hwdrivers/CStereoGrabber_SVS.h>

#include <mrpt/gui/CDisplayWindow.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** The central class for camera grabbers in MRPT, implementing the "generic sensor" interface.
		  *   This class provides the user with a uniform interface to a variety of other classes which manage only one specific camera "driver" (opencv, ffmpeg, PGR FlyCapture,...)
		  *
		  *   Following the "generic sensor" interface, all the parameters must be passed int the form of a configuration file, 
		  *   which may be also formed on the fly (without being a real config file) as in this example:
		  *
		  *  \code
		  *   CCameraSensor myCam;
		  *   const string str =
		  *      "[CONFIG]\n"
		  *      "grabber_type=opencv\n";
		  *
		  *   CConfigFileMemory	cfg(str);
		  *   myCam.loadConfig(cfg,"CONFIG");
		  *   myCam.initialize();
		  *   CObservationPtr obs = myCam.getNextFrame();
		  *  \endcode
		  *
		  *  Images can be retrieved through the normal "doProcess()" interface, or the specific method "getNextFrame()".
		  *
		  * Some notes:
		  *  - "grabber_type" determines the class to use internally for image capturing (see below).
		  *  - For the meaning of cv_camera_type and other parameters, refer to mrpt::hwdrivers::CImageGrabber_OpenCV
		  *  - For the parameters of dc1394 parameters, refer to generic IEEE1394 documentation, and to mrpt::hwdrivers::TCaptureOptions_dc1394.
		  *  - If the high number of existing parameters annoy you, try the function prepareVideoSourceFromUserSelection(), 
		  *     which displays a GUI dialog to the user so he/she can choose the desired camera & its parameters.
		  *
		  *  Images can be saved in the "external storage" mode. Detached threads are created for this task. See \a setPathForExternalImages() and \a setExternalImageFormat(). 
		  *  These methods are called automatically from the app rawlog-grabber.
		  *
		  *  These is the list of all accepted parameters:
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    # Select one of the grabber implementations -----------------------
		  *    grabber_type       = opencv | dc1394 | bumblebee_dc1394 | ffmpeg | rawlog | swissranger | svs | kinect | flycap | flycap_stereo | image_dir | duo3d
		  *
		  *    #  Options for any grabber_type ------------------------------------
		  *    preview_decimation = 0     // N<=0 (or not present): No preview; N>0, display 1 out of N captured frames.
		  *    preview_reduction  = 0     // 0 or 1 (or not present): The preview shows the actual image. For 2,3,..., reduces the size of the image by that factor, only for the preview window.
		  *    capture_grayscale  = 0     // 1:capture in grayscale, whenever the driver allows it. Default=0
		  *    #  For externaly stored images, the format of image files (default=jpg)
		  *    #external_images_format  = jpg
		  *
		  *    #  For externaly stored images: whether to spawn independent threads to save the image files.
		  *    #external_images_own_thread  = 1   // 0 or 1
		  *
		  *    # If external_images_own_thread=1, this changes the number of threads to launch
		  *    #  to save image files. The default is determined from mrpt::system::getNumberOfProcessors()
		  *    #  and should be OK unless you want to save processor time for other things.
		  *    #external_images_own_thread_count = 2    // >=1
		  *
		  *    # (Only when external_images_format=jpg): Optional parameter to set the JPEG compression quality:
		  *    #external_images_jpeg_quality = 95    // [1-100]. Default: 95
		  *
		  *    # Pose of the sensor on the robot:
		  *    pose_x=0		; (meters)
		  *    pose_y=0
		  *    pose_z=0
		  *    pose_yaw=0	; (Angles in degrees)
		  *    pose_pitch=0
		  *    pose_roll=0
		  *
		  *    # Options for grabber_type= opencv  ------------------------------------
		  *    cv_camera_index  = 0       // [opencv] Number of camera to open
		  *    cv_camera_type   = CAMERA_CV_AUTODETECT
		  *    cv_frame_width   = 640     // [opencv] Capture width (not present or set to 0 for default)
		  *    cv_frame_height  = 480     // [opencv] Capture height (not present or set to 0 for default)
		  *    cv_fps           = 15      // [opencv] IEEE1394 cams only: Capture FPS (not present or 0 for default)
		  *    cv_gain          = 0       // [opencv] Camera gain, if available (nor present or set to 0 for default).
		  *
		  *    # Options for grabber_type= dc1394 -------------------------------------
		  *    dc1394_camera_guid   = 0 | 0x11223344    // 0 (or not present): the first camera; A hexadecimal number: The GUID of the camera to open
		  *    dc1394_camera_unit   = 0     			// 0 (or not present): the first camera; 0,1,2,...: The unit number (within the given GUID) of the camera to open (Stereo cameras: 0 or 1)
		  *    dc1394_frame_width	= 640
		  *    dc1394_frame_height	= 480
		  *    dc1394_framerate		= 15					// eg: 7.5, 15, 30, 60, etc... For posibilities see mrpt::hwdrivers::TCaptureOptions_dc1394
		  *    dc1394_mode7         = -1                    // -1: Ignore, i>=0, set to MODE7_i
		  *    dc1394_color_coding	= COLOR_CODING_YUV422	// For posibilities see mrpt::hwdrivers::TCaptureOptions_dc1394
		  *    # Options for setting feature values: dc1394_<feature> = <n>
		  *    # with <feature> = brightness | exposure | sharpness | white_balance | gamma | shutter | gain
		  *    #      <n> a value, or -1 (or not present) for not to change this feature value in the camera, possible values are shown in execution
		  *    dc1394_shutter       = -1
		  *    # Options for setting feature modes: dc1394_<feature>_mode = <n>
		  *    # with <feature> = brightness | exposure | sharpness | white_balance | gamma | shutter | gain
		  *    #      <n> = -1 (or not present) [not to change] | 0 [manual] | 1 [auto] | 2 [one_push_auto]
		  *    dc1394_shutter_mode	= -1
		  *    # Options for setting trigger options:
		  *    dc1394_trigger_power	= -1    // -1 (or not present) for not to change | 0 [OFF] | 1 [ON]
		  *    dc1394_trigger_mode	= -1    // -1 (or not present) for not to change | 0..7 corresponding to possible modes 0,1,2,3,4,5,14,15
		  *    dc1394_trigger_source= -1    // -1 (or not present) for not to change | 0..4 corresponding to possible sources 0,1,2,3,SOFTWARE
		  *    dc1394_trigger_polarity = -1 // -1 (or not present) for not to change | 0 [ACTIVE_LOW] | 1 [ACTIVE_HIGH]
		  *    dc1394_ring_buffer_size = 15  // Length of frames ring buffer (internal to libdc1394)
		  *
		  *    # Options for grabber_type= bumblebee_dc1394 ----------------------------------
		  *    bumblebee_dc1394_camera_guid   = 0 | 0x11223344  // 0 (or not present): the first camera; A hexadecimal number: The GUID of the camera to open
		  *    bumblebee_dc1394_camera_unit   = 0     			// 0 (or not present): the first camera; 0,1,2,...: The unit number (within the given GUID) of the camera to open (Stereo cameras: 0 or 1)
		  *    bumblebee_dc1394_framerate     = 15				// eg: 7.5, 15, 30, 60, etc... For posibilities see mrpt::hwdrivers::TCaptureOptions_dc1394
		  *
		  *    # Options for grabber_type= ffmpeg -------------------------------------
		  *    ffmpeg_url             = rtsp://127.0.0.1      // [ffmpeg] The video file or IP camera to open
		  *
		  *    # Options for grabber_type= rawlog -------------------------------------
		  *    rawlog_file            = mylog.rawlog          // [rawlog] This can be used to simulate the capture of images already grabbed in the past in the form of a MRPT rawlog.
		  *    rawlog_camera_sensor_label  = CAMERA1          // [rawlog] If this field is not present, all images found in the rawlog will be retrieved. Otherwise, only those observations with a matching sensor label.
		  *
		  *    # Options for grabber_type= svs -------------------------------------
		  *    svs_camera_index = 0
		  *    svs_frame_width = 800
		  *    svs_frame_height = 600
		  *    svs_framerate = 25.0
		  *    svs_NDisp = ...
		  *    svs_Corrsize = ...
		  *    svs_LR = ...
		  *    svs_Thresh = ...
		  *    svs_Unique = ...
		  *    svs_Horopter = ...
		  *    svs_SpeckleSize = ...
		  *    svs_procesOnChip = false
		  *    svs_calDisparity = true
		  *
		  *    # Options for grabber_type= swissranger -------------------------------------
		  *    sr_use_usb         = true	        // True: use USB, false: use ethernet
		  *    sr_IP              = 192.168.2.14    // If sr_use_usb=false, the camera IP
		  *    sr_grab_grayscale  = true            // whether to save the intensity channel
		  *    sr_grab_3d         = true            // whether to save the 3D points
		  *    sr_grab_range      = true            // whether to save the range image
		  *    sr_grab_confidence = true            // whether to save the confidence image
		  *
		  *    # Options for grabber_type= XBox kinect -------------------------------------
		  *    kinect_grab_intensity  = true            // whether to save the intensity (RGB) channel
		  *    kinect_grab_3d         = true            // whether to save the 3D points
		  *    kinect_grab_range      = true            // whether to save the depth image
		  *    #kinect_video_rgb       = true            // Optional. If set to "false", the IR intensity channel will be grabbed instead of the color RGB channel.
		  *
		  *    # Options for grabber_type= flycap (Point Grey Research's FlyCapture 2 for Monocular and Stereo cameras, e.g. Bumblebee2) --------
		  *    flycap_camera_index           = 0
		  *    #... (all the parameters enumerated in mrpt::hwdrivers::TCaptureOptions_FlyCapture2 with the prefix "flycap_")
		  *
		  *    # Options for grabber_type= flycap_stereo (Point Grey Research's FlyCapture 2, two cameras setup as a stereo pair) ------
		  *    # fcs_start_synch_capture   = false  // *Important*: Only set to true if using Firewire cameras: the "startSyncCapture()" command is unsupported in USB3 and GigaE cameras.
		  *
		  *    fcs_LEFT_camera_index           = 0
		  *    #... (all the parameters enumerated in mrpt::hwdrivers::TCaptureOptions_FlyCapture2 with the prefix "fcs_LEFT_")
		  *    fcs_RIGHT_camera_index          = 0
		  *    #... (all the parameters enumerated in mrpt::hwdrivers::TCaptureOptions_FlyCapture2 with the prefix "fcs_RIGHT_")
		  *
		  *    # Options for grabber_type= image_dir
		  *    image_dir_url					= 				// [string] URL of the directory 
		  *    left_filename_format				= imL_%05d.jpg	// [string] Format including prefix, number of trailing zeros, digits and image format (extension)
		  *    right_filename_format			= imR_%05d.jpg	// [string] Format including prefix, number of trailing zeros, digits and image format (extension). Leave blank if only images from one camera will be used.
		  *    start_index						= 0				// [int]	Starting index for images
		  *    end_index						= 100			// [int]	End index for the images
		  *
		  *    # Options for grabber_type= duo3d
		  *    Create a section like this:
		  *    [DUO3DOptions]
		  *    rawlog-grabber-ignore	= true // Instructs rawlog-grabber to ignore this section (it is not a separate device!)
		  *
		  *    image_width   			= 640			// [int]	x Resolution
		  *    image_height  			= 480			// [int]	y Resolution
		  *    fps						= 30			// [int]	Frames per second (<= 30)
		  *    exposure				= 50			// [int]	Exposure value (1..100)
		  *    led						= 0				// [int]	Led intensity (only for some device models) (1..100).
		  *    gain					= 50			// [int]	Camera gain (1..100)
		  *    capture_rectified 		= false			// [bool]	Rectify captured images
		  *    capture_imu 			= true			// [bool]	Capture IMU data from DUO3D device (if available)
		  *    calibration_from_file	= true			// [bool]	Use YML calibration files provided by calibration application supplied with DUO3D device
		  *    intrinsic_filename		= ""			// [string]	Intrinsic parameters file. This filename should contain a substring _RWWWxHHH_ with WWW being the image width and HHH the image height, as provided by the calibration application.
		  *    extrinsic_filename		= ""			// [string]	Extrinsic parameters file. This filename should contain a substring _RWWWxHHH_ with WWW being the image width and HHH the image height, as provided by the calibration application.
		  *    rectify_map_filename	= ""			// [string]	Rectification map file. This filename should contain a substring _RWWWxHHH_ with WWW being the image width and HHH the image height, as provided by the calibration application.
		  *
		  *    // if 'calibration_from_file' = false, three more sections containing the calibration must be provided:
		  *    [DUO3D_LEFT]
		  *    rawlog-grabber-ignore	= true // Instructs rawlog-grabber to ignore this section (it is not a separate device!)
		  *    resolution 		= [640 480]
		  *    cx 				= 320
		  *    cy 				= 240
		  *    fx 				= 700
		  *    fy 				= 700
		  *    dist 			= [0 0 0 0 0]
		  *
		  *    [DUO3D_RIGHT]
		  *    rawlog-grabber-ignore	= true // Instructs rawlog-grabber to ignore this section (it is not a separate device!)
		  *    resolution 		= [640 480]
		  *    cx 				= 320
		  *    cy 				= 240
		  *    fx 				= 700
		  *    fy 				= 700
		  *    dist 			= [0 0 0 0 0]
		  *
		  *    [DUO3D_LEFT2RIGHT_POSE]
		  *    rawlog-grabber-ignore	= true // Instructs rawlog-grabber to ignore this section (it is not a separate device!)
		  *    pose_quaternion = [0.12 0 0 1 0 0 0]
		  *
		  *  \endcode
		  *
		  *  \note The execution rate, in rawlog-grabber or the user code calling doProcess(), should be greater than the required capture FPS.
		  *  \note In Linux you may need to execute "chmod 666 /dev/video1394/ * " and "chmod 666 /dev/raw1394" for allowing any user R/W access to firewire cameras.
		  * \note [New in MRPT 1.4.0] The `bumblebee` driver has been deleted, use the `flycap` driver in stereo mode.
		  *  \sa mrpt::hwdrivers::CImageGrabber_OpenCV, mrpt::hwdrivers::CImageGrabber_dc1394, CGenericSensor, prepareVideoSourceFromUserSelection()
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CCameraSensor : public mrpt::utils::COutputLogger, public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CCameraSensor)

		public:
			/** Constructor. The camera is not open until "initialize" is called. */
			CCameraSensor();

			/** Destructor */
			virtual ~CCameraSensor();

			// See docs in parent class
			void  doProcess();

			/** Retrieves the next frame from the video source, raising an exception on any error.
			  * Note: The returned observations can be of one of these classes (you can use IS_CLASS(obs,CObservationXXX) to determine it):
			  *		- mrpt::obs::CObservationImage (For normal cameras or video sources)
			  *		- mrpt::obs::CObservationStereoImages (For stereo cameras)
			  *		- mrpt::obs::CObservation3DRangeScan (For 3D cameras)
			  */
			mrpt::obs::CObservationPtr getNextFrame( );
			void getNextFrame( std::vector<mrpt::utils::CSerializablePtr> & out_obs );

			/** Tries to open the camera, after setting all the parameters with a call to loadConfig.
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			virtual void initialize();

			/** Close the camera (if open).
			  *   This method is called automatically on destruction.
			  */
			void close();

            /** Set Software trigger level value (ON or OFF) for cameras with this function available.
              */
            void setSoftwareTriggerLevel( bool level );

			/**  Set the path where to save off-rawlog image files (this class DOES take into account this path).
			  *  An  empty string (the default value at construction) means to save images embedded in the rawlog, instead of on separate files.
			  * \exception std::exception If the directory doesn't exists and cannot be created.
			  */
			virtual void setPathForExternalImages( const std::string &directory );

			/** This must be called before initialize() */
			void enableLaunchOwnThreadForSavingImages(bool enable=true) { m_external_images_own_thread = enable; };

			/** Functor type */
			typedef void (*TPreSaveUserHook)(const mrpt::obs::CObservationPtr &obs, void* user_ptr);

			/** Provides a "hook" for user-code to be run BEFORE an image is going to be saved to disk if external storage is enabled (e.g. to rectify images, preprocess them, etc.)
			  * Notice that this code may be called from detached threads, so it must be thread safe.
			  * If used, call this before initialize() */
			void addPreSaveHook( TPreSaveUserHook user_function, void *user_ptr ) { m_hook_pre_save=user_function; m_hook_pre_save_param=user_ptr; };

		protected:
			// Options for any grabber_type ------------------------------------
			poses::CPose3D m_sensorPose;

			std::string                m_grabber_type; //!< Can be "opencv",...
			bool									m_capture_grayscale;

			// Options for grabber_type= opencv  ------------------------------------
			int										m_cv_camera_index;
			std::string								m_cv_camera_type;
			TCaptureCVOptions			m_cv_options;

			// Options for grabber_type= dc1394 -------------------------------------
			uint64_t								m_dc1394_camera_guid;
			int										m_dc1394_camera_unit;
			TCaptureOptions_dc1394	m_dc1394_options;
			int										m_preview_decimation;
			int										m_preview_reduction;

			// Options for grabber_type= bumblebee_dc1394 ----------------------------------
			uint64_t m_bumblebee_dc1394_camera_guid;
			int	     m_bumblebee_dc1394_camera_unit;
			double   m_bumblebee_dc1394_framerate;

			// Options for grabber type= svs -----------------------------------------
			int										m_svs_camera_index;
			TCaptureOptions_SVS    m_svs_options;

			// Options for grabber_type= ffmpeg -------------------------------------
			std::string								m_ffmpeg_url;

			// Options for grabber_type= rawlog -------------------------------------
			std::string								m_rawlog_file;
			std::string								m_rawlog_camera_sensor_label;
			std::string								m_rawlog_detected_images_dir;

			// Options for grabber_type= swissranger -------------------------------------
			bool 			m_sr_open_from_usb; //!< true: USB, false: ETH
			std::string  	m_sr_ip_address;
			bool 			m_sr_save_3d;			//!< Save the 3D point cloud (default: true)
			bool 			m_sr_save_range_img;	//!< Save the 2D range image (default: true)
			bool 			m_sr_save_intensity_img; //!< Save the 2D intensity image (default: true)
			bool 			m_sr_save_confidence;	//!< Save the estimated confidence 2D image (default: false)

			// Options for grabber_type= XBox kinect -------------------------------------
			bool 			m_kinect_save_3d;			//!< Save the 3D point cloud (default: true)
			bool 			m_kinect_save_range_img;	//!< Save the 2D range image (default: true)
			bool 			m_kinect_save_intensity_img; //!< Save the 2D intensity image (default: true)
			bool			m_kinect_video_rgb;			//!< Save RGB or IR channels (default:true)

			// Options for grabber type= flycap -----------------------------------------
			TCaptureOptions_FlyCapture2   m_flycap_options;

			// Options for grabber type= flycap_stereo -----------------------------------------
			bool            m_fcs_start_synch_capture;
			TCaptureOptions_FlyCapture2   m_flycap_stereo_options[2]; // [0]:left, [1]:right

			// Options for grabber type= image_dir
			std::string		m_img_dir_url;
			std::string		m_img_dir_left_format;
			std::string		m_img_dir_right_format;
			int				m_img_dir_start_index;
			int				m_img_dir_end_index;
			
			bool			m_img_dir_is_stereo;
			int				m_img_dir_counter;
			
			// Options for grabber type= duo3d
			TCaptureOptions_DUO3D	m_duo3d_options;

			// Other options:
			bool				m_external_images_own_thread; //!< Whether to launch independent thread

			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		private:
			// Only one of these will be !=NULL at a time ===========
			CImageGrabber_OpenCV      * m_cap_cv;		//!< The OpenCV capture object.
			CImageGrabber_dc1394      * m_cap_dc1394;	//!< The dc1394 capture object.
			CImageGrabber_FlyCapture2 * m_cap_flycap;     //!< The FlyCapture2 object
			CImageGrabber_FlyCapture2 * m_cap_flycap_stereo_l, *m_cap_flycap_stereo_r;     //!< The FlyCapture2 object for stereo pairs
			CStereoGrabber_Bumblebee_libdc1394 * m_cap_bumblebee_dc1394;
			CStereoGrabber_SVS        * m_cap_svs;	//!< The svs capture object.
			CFFMPEG_InputStream       * m_cap_ffmpeg;	//!< The FFMPEG capture object
			mrpt::utils::CFileGZInputStream * m_cap_rawlog;	//!< The input file for rawlogs
			CSwissRanger3DCamera      * m_cap_swissranger; //!< SR 3D camera object.
			CKinect                   * m_cap_kinect;    //!< Kinect camera object.
			COpenNI2Sensor 		      * m_cap_openni2;  //!< OpenNI2 object.
			std::string				  * m_cap_image_dir;	//!< Read images from directory
			CDUO3DCamera			  * m_cap_duo3d;		//!< The DUO3D capture object
			// =========================

			int			m_camera_grab_decimator;
			int			m_camera_grab_decimator_counter;

			int							 m_preview_counter;
			mrpt::gui::CDisplayWindowPtr m_preview_win1,m_preview_win2; //!< Normally we'll use only one window, but for stereo images we'll use two of them.

			/** @name Stuff related to working threads to save images to disk
			    @{ */
			unsigned int		m_external_image_saver_count; //!< Number of working threads. Default:1, set to 2 in quad cores.
			std::vector<mrpt::system::TThreadHandle>  m_threadImagesSaver;

			bool 	m_threadImagesSaverShouldEnd;
			mrpt::synch::CCriticalSection	m_csToSaveList;		//!< The critical section for m_toSaveList
			std::vector<TListObservations>	m_toSaveList;		//!< The queues of objects to be returned by getObservations, one for each working thread.
			void thread_save_images(unsigned int my_working_thread_index); //!< Thread to save images to files.
			
			TPreSaveUserHook  m_hook_pre_save;
			void            * m_hook_pre_save_param;
			/**  @} */

		}; // end class

		typedef stlplus::smart_ptr<CCameraSensor>    CCameraSensorPtr; //!< A smart pointer to a CCameraSensor

		/** Used only from MRPT apps: Use with caution since "panel" MUST be a "mrpt::gui::CPanelCameraSelection *"
		  */
		CCameraSensorPtr HWDRIVERS_IMPEXP prepareVideoSourceFromPanel(void *panel);

		/** Parse the user options in the wxWidgets "panel" and write the configuration into the given section of the given configuration file.
		  * Use with caution since "panel" MUST be a "mrpt::gui::CPanelCameraSelection *"
		  * \sa prepareVideoSourceFromUserSelection, prepareVideoSourceFromPanel, readConfigIntoVideoSourcePanel
		  */
		void HWDRIVERS_IMPEXP writeConfigFromVideoSourcePanel(
			void *panel,
			const std::string &in_cfgfile_section_name,
			mrpt::utils::CConfigFileBase *out_cfgfile
			);

		/** Parse the given section of the given configuration file and set accordingly the controls of the wxWidgets "panel".
		  * Use with caution since "panel" MUST be a "mrpt::gui::CPanelCameraSelection *"
		  * \sa prepareVideoSourceFromUserSelection, prepareVideoSourceFromPanel, writeConfigFromVideoSourcePanel
		  */
		void HWDRIVERS_IMPEXP readConfigIntoVideoSourcePanel(
			void *panel,
			const std::string &in_cfgfile_section_name,
			const mrpt::utils::CConfigFileBase *in_cfgfile
			);

		/** Show to the user a list of possible camera drivers and creates and open the selected camera.
		  */
		CCameraSensorPtr HWDRIVERS_IMPEXP prepareVideoSourceFromUserSelection();


	} // end namespace
} // end namespace

#endif
