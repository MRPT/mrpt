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

#ifndef CCameraSensor_H
#define CCameraSensor_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/hwdrivers/CGenericSensor.h>

#include <mrpt/hwdrivers/CFFMPEG_InputStream.h>
#include <mrpt/hwdrivers/CImageGrabber_OpenCV.h>
#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>
#include <mrpt/hwdrivers/CStereoGrabber_Bumblebee.h>
#include <mrpt/hwdrivers/CSwissRanger3DCamera.h>
#include <mrpt/hwdrivers/CKinect.h>

#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/hwdrivers/CStereoGrabber_SVS.h>

#include <mrpt/gui/CDisplayWindow.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** The central class for camera grabbers in MRPT, implementing the "generic sensor" interface.
		  *   This class provides the user with a uniform interface to a variety of other classes which manage only one specific camera "driver" (opencv, ffmpeg, bumblebee,...)
		  *
		  *   Following the "generic sensor" interface, all the parameters must be passed int the form of a configuration file, which may be also formed on the fly (without being a real config file) as in this example:
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
		  *  - If all the existing parameter annoy you, try the function prepareVideoSourceFromUserSelection(), which displays a GUI dialog to the user so he/she can choose the desired camera & its parameters.
		  *
		  *  Images can be saved in the "external storage" mode. See setPathForExternalImages and setExternalImageFormat. These methods
		  *   are called automatically from rawlog-grabber.
		  *
		  *  These is the list of all accepted parameters:
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    // Select one of the grabber implementations -----------------------
		  *    grabber_type       = opencv | dc1394 | bumblebee | ffmpeg | rawlog | swissranger | kinect
		  *
		  *    // Options for any grabber_type ------------------------------------
		  *    preview_decimation = 0     // N<=0 (or not present): No preview; N>0, display 1 out of N captured frames.
		  *    preview_reduction  = 0     // 0 or 1 (or not present): The preview shows the actual image. For 2,3,..., reduces the size of the image by that factor, only for the preview window.
		  *    capture_grayscale  = 0     // 1:capture in grayscale, whenever the driver allows it. Default=0
		  *    // For externaly stored images, the format of image files (default=jpg)
		  *    //external_images_format  = jpg
		  *
		  *    // For externaly stored images: whether to spawn independent threads to save the image files.
		  *    //external_images_own_thread  = 1   // 0 or 1
		  *
		  *    // If external_images_own_thread=1, this changes the number of threads to launch
		  *    //  to save image files. The default is determined from mrpt::system::getNumberOfProcessors()
		  *    //  and should be OK unless you want to save processor time for other things.
		  *    //external_images_own_thread_count = 2    // >=1
		  *
		  *    // (Only when external_images_format=jpg): Optional parameter to set the JPEG compression quality:
		  *    //external_images_jpeg_quality = 95    // [1-100]. Default: 95
		  *
		  *    // Pose of the sensor on the robot:
		  *    pose_x=0		; (meters)
		  *    pose_y=0
		  *    pose_z=0
		  *    pose_yaw=0	; (Angles in degrees)
		  *    pose_pitch=0
		  *    pose_roll=0
		  *
		  *    // Options for grabber_type= opencv  ------------------------------------
		  *    cv_camera_index  = 0       // [opencv] Number of camera to open
		  *    cv_camera_type   = CAMERA_CV_AUTODETECT
		  *    cv_frame_width   = 640     // [opencv] Capture width (not present or set to 0 for default)
		  *    cv_frame_height  = 480     // [opencv] Capture height (not present or set to 0 for default)
		  *    cv_fps           = 15      // [opencv] IEEE1394 cams only: Capture FPS (not present or 0 for default)
		  *    cv_gain          = 0       // [opencv] Camera gain, if available (nor present or set to 0 for default).
		  *
		  *    // Options for grabber_type= dc1394 -------------------------------------
		  *    dc1394_camera_guid   = 0 | 0x11223344    // 0 (or not present): the first camera; A hexadecimal number: The GUID of the camera to open
		  *    dc1394_camera_unit   = 0     			// 0 (or not present): the first camera; 0,1,2,...: The unit number (within the given GUID) of the camera to open (Stereo cameras: 0 or 1)
		  *    dc1394_frame_width	= 640
		  *    dc1394_frame_height	= 480
		  *    dc1394_framerate		= 15					// eg: 7.5, 15, 30, 60, etc... For posibilities see mrpt::hwdrivers::TCaptureOptions_dc1394
		  *    dc1394_mode7         = -1                    // -1: Ignore, i>=0, set to MODE7_i
		  *    dc1394_color_coding	= COLOR_CODING_YUV422	// For posibilities see mrpt::hwdrivers::TCaptureOptions_dc1394
		  *    dc1394_shutter		= -1	// A value, or -1 (or not present) for not to change this parameter in the camera
		  *    dc1394_gain			= -1	// A value, or -1 (or not present) for not to change this parameter in the camera
		  *    dc1394_gamma			= -1	// A value, or -1 (or not present) for not to change this parameter in the camera
		  *    dc1394_brightness	= -1	// A value, or -1 (or not present) for not to change this parameter in the camera
		  *    dc1394_exposure		= -1	// A value, or -1 (or not present) for not to change this parameter in the camera
		  *    dc1394_sharpness		= -1	// A value, or -1 (or not present) for not to change this parameter in the camera
		  *    dc1394_white_balance	= -1	// A value, or -1 (or not present) for not to change this parameter in the camera
		  *
		  *    // Options for grabber_type= bumblebee ----------------------------------
		  *    bumblebee_camera_index  = 0       // [bumblebee] Number of camera within the firewire bus to open (typically = 0)
		  *    bumblebee_frame_width   = 640     // [bumblebee] Capture width (not present or set to 0 for default)
		  *    bumblebee_frame_height  = 480     // [bumblebee] Capture height (not present or set to 0 for default)
		  *    bumblebee_fps           = 15      // [bumblebee] Capture FPS (not present or 0 for default)
		  *    bumblebee_mono          = 0|1     // [bumblebee] OPTIONAL: If this parameter is present, monocular (0:left, 1:right) images will be grabbed instead of stereo pairs.
		  *    bumblebee_get_rectified = 0|1     // [bumblebee] Determines if the camera should grab rectified or raw images (1 is the default)
		  *
		  *    // Options for grabber_type= ffmpeg -------------------------------------
		  *    ffmpeg_url             = rtsp://127.0.0.1      // [ffmpeg] The video file or IP camera to open
		  *
		  *    // Options for grabber_type= rawlog -------------------------------------
		  *    rawlog_file            = mylog.rawlog          // [rawlog] This can be used to simulate the capture of images already grabbed in the past in the form of a MRPT rawlog.
		  *    rawlog_camera_sensor_label  = CAMERA1          // [rawlog] If this field is not present, all images found in the rawlog will be retrieved. Otherwise, only those observations with a matching sensor label.
		  *
		  *    // Options for grabber_type= swissranger -------------------------------------
		  *    sr_use_usb         = true	        // True: use USB, false: use ethernet
		  *    sr_IP              = 192.168.2.14    // If sr_use_usb=false, the camera IP
		  *    sr_grab_grayscale  = true            // whether to save the intensity channel
		  *    sr_grab_3d         = true            // whether to save the 3D points
		  *    sr_grab_range      = true            // whether to save the range image
		  *    sr_grab_confidence = true            // whether to save the confidence image
		  *
		  *    // Options for grabber_type= kinect -------------------------------------
		  *    kinect_grab_intensity  = true            // whether to save the intensity (RGB) channel
		  *    kinect_grab_3d         = true            // whether to save the 3D points
		  *    kinect_grab_range      = true            // whether to save the depth image
		  *
		  *    kinect_video_rgb       = true            // Optional. If set to "false", the IR intensity channel will be grabbed instead of the color RGB channel.
		  *
		  *  \endcode
		  *
		  *  \note The execution rate (in rawlog-grabber) should be greater than the required capture FPS.
		  *  \note In Linux you may need to execute "chmod 666 /dev/video1394/ * " and "chmod 666 /dev/raw1394" for allowing any user R/W access to firewire cameras.
		  *  \sa mrpt::hwdrivers::CImageGrabber_OpenCV, mrpt::hwdrivers::CImageGrabber_dc1394, CGenericSensor, prepareVideoSourceFromUserSelection
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CCameraSensor : public utils::CDebugOutputCapable, public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CCameraSensor)

		public:
			/** Constructor
			  *  The camera is not open until "initialize" is called.
			  */
			CCameraSensor();

			/** Destructor
			  */
			virtual ~CCameraSensor();

			/** This method should be called periodically (at least at 1Hz to capture ALL the real-time data)
			*  It is thread safe, i.e. you can call this from one thread, then to other methods from other threads.
			*/
			void  doProcess();

			/** Retrieves the next frame from the video source, raising an exception on any error.
			  * Note: The returned observations can be of one of these classes (you can use IS_CLASS(obs,CObservationXXX) to determine it):
			  *		- mrpt::slam::CObservationImage (For normal cameras or video sources)
			  *		- mrpt::slam::CObservationStereoImages (For stereo cameras)
			  *		- mrpt::slam::CObservation3DRangeScan (For 3D cameras)
			  */
			mrpt::slam::CObservationPtr getNextFrame();

			/** Tries to open the camera, after setting all the parameters with a call to loadConfig.
			  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
			  */
			virtual void initialize();

			/** Close the camera (if open).
			  *   This method is called automatically on destruction.
			  */
			void close();

			/**  Set the path where to save off-rawlog image files (this class DOES take into account this path).
			  *  An  empty string (the default value at construction) means to save images embedded in the rawlog, instead of on separate files.
			  * \exception std::exception If the directory doesn't exists and cannot be created.
			  */
			virtual void setPathForExternalImages( const std::string &directory );

			/** This must be called before initialize() */
			void enableLaunchOwnThreadForSavingImages(bool enable=true) { m_external_images_own_thread = enable; };

		protected:
			poses::CPose3D		m_sensorPose;

			std::string								m_grabber_type; //!< Can be "opencv",...
			bool									m_capture_grayscale;
			int										m_cv_camera_index;
			std::string								m_cv_camera_type;
			mrpt::hwdrivers::TCaptureCVOptions			m_cv_options;

			uint64_t								m_dc1394_camera_guid;
			int										m_dc1394_camera_unit;
			mrpt::hwdrivers::TCaptureOptions_dc1394	m_dc1394_options;
			int										m_preview_decimation;
			int										m_preview_reduction;

			int										m_bumblebee_camera_index;
			mrpt::hwdrivers::TCaptureOptions_bumblebee	m_bumblebee_options;
			int										m_bumblebee_monocam; // 0:Left, 1: Right, <0,>1 -> Stereo

			int										m_svs_camera_index;
			mrpt::hwdrivers::TCaptureOptions_SVS    m_svs_options;

			std::string								m_ffmpeg_url;

			std::string								m_rawlog_file;
			std::string								m_rawlog_camera_sensor_label;
			std::string								m_rawlog_detected_images_dir;

			bool 			m_sr_open_from_usb; //!< true: USB, false: ETH
			std::string  	m_sr_ip_address;
			bool 			m_sr_save_3d;			//!< Save the 3D point cloud (default: true)
			bool 			m_sr_save_range_img;	//!< Save the 2D range image (default: true)
			bool 			m_sr_save_intensity_img; //!< Save the 2D intensity image (default: true)
			bool 			m_sr_save_confidence;	//!< Save the estimated confidence 2D image (default: false)

			bool 			m_kinect_save_3d;			//!< Save the 3D point cloud (default: true)
			bool 			m_kinect_save_range_img;	//!< Save the 2D range image (default: true)
			bool 			m_kinect_save_intensity_img; //!< Save the 2D intensity image (default: true)
			bool			m_kinect_video_rgb;			//!< Save RGB or IR channels (default:true)

			bool				m_external_images_own_thread; //!< Whether to launch independent thread

			/** Loads specific configuration for the device from a given source of configuration parameters, for example, an ".ini" file, loading from the section "[iniSection]" (see utils::CConfigFileBase and derived classes)
			  *  See hwdrivers::CCameraSensor for the possible parameters
			  */
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string	  &iniSection );

		private:
			// Only one of these will be !=NULL at a time ===========
			CImageGrabber_OpenCV 				*m_cap_cv;		//!< The OpenCV capture object.
			CImageGrabber_dc1394 				*m_cap_dc1394;	//!< The dc1394 capture object.
			CStereoGrabber_Bumblebee 			*m_cap_bumblebee;	//!< The bumblebee capture object.
			mrpt::hwdrivers::CStereoGrabber_SVS        *m_cap_svs;	//!< The svs capture object.
			CFFMPEG_InputStream					*m_cap_ffmpeg;	//!< The FFMPEG capture object
			mrpt::utils::CFileGZInputStream		*m_cap_rawlog;	//!< The input file for rawlogs
			CSwissRanger3DCamera				*m_cap_swissranger; //!< SR 3D camera object.
			CKinect                             *m_cap_kinect;    //!< Kinect camera object.
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
