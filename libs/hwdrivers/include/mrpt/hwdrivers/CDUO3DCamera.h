/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CDUO3DCamera_H
#define CDUO3DCamera_H

#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/vision/CStereoRectifyMap.h>

#include <mrpt/hwdrivers/link_pragmas.h>
#include <mrpt/utils/CUncopiable.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** Options used when creating a camera capture object of type CImageGrabber_FlyCapture2   \ingroup mrpt_hwdrivers_grp */
		struct HWDRIVERS_IMPEXP TCaptureOptions_DUO3D
		{
			enum TYMLReadResult {yrr_NAME_NON_CONSISTENT,yrr_EMPTY,yrr_OK};

			TCaptureOptions_DUO3D();
			~TCaptureOptions_DUO3D();

			/** @name Image settings
			  * @{ */
			int			m_img_width;			//!< (Default = 640) Width of the captured image.
			int			m_img_height;			//!< (Default = 480) Height of the captured image.
			float		m_fps;					//!< (Default = 30) Frames per second <= 30.
			float		m_exposure;				//!< (Default = 50) Exposure value.
			float		m_led;					//!< (Default = 25) Led intensity (some device models).
			float		m_gain;					//!< (Default = 10) Camera gain.
			/** @} */

			/** @name Behaviour selection
			  * @{ */
			bool		m_capture_imu;			//!< (Default = false) Capture IMU data.
			bool		m_capture_rectified;	//!< (Default = true) Rectify images. Rectification map must be provided \sa m_rectify_map_filename.
			bool		m_calibration_from_file;//!< (Default = true) Get calibration information from files provided by DUO3D Calibration App.
			/** @} */
			
			/** @name Files specification
			  * @{ */
			std::string	m_rectify_map_filename;	//!< Rectification map file provided by DUO3D Calibration App (YML format).
			std::string	m_intrinsic_filename;	//!< Intrinsic parameters file provided by DUO3D Calibration App (YML format).
			std::string	m_extrinsic_filename;	//!< Extrinsic parameters file provided by DUO3D Calibration App (YML format).
			/** @} */

			/** @name Others
			  * @{ */
			mrpt::utils::TStereoCamera	m_stereo_camera;
			/** @} */

			/** Loads all the options from a config file.
			  * Expected format:
			  *
			  * \code
			  * [sectionName]
			  *		image_width   			= 640			// [int]	x Resolution
			  *		image_height  			= 480			// [int]	y Resolution
			  *		fps						= 30			// [int]	Frames per second (<= 30)
			  *		exposure				= 50			// [int]	Exposure value (1..100)
			  *		led						= 0				// [int]	Led intensity (only for some device models) (1..100).
			  *		gain					= 50			// [int]	Camera gain (1..100)
			  *		capture_rectified 		= false			// [bool]	Rectify captured images
			  *		capture_imu 			= true			// [bool]	Capture IMU data from DUO3D device (if available)
			  *		calibration_from_file	= true			// [bool]	Use YML calibration files provided by calibration application supplied with DUO3D device
			  *	  	intrinsic_filename		= ""			// [string]	Intrinsic parameters file. This filename should contain a substring _RWWWxHHH_ with WWW being the image width and HHH the image height, as provided by the calibration application.
			  *		extrinsic_filename		= ""			// [string]	Extrinsic parameters file. This filename should contain a substring _RWWWxHHH_ with WWW being the image width and HHH the image height, as provided by the calibration application.
			  *		rectify_map_filename	= ""			// [string]	Rectification map file. This filename should contain a substring _RWWWxHHH_ with WWW being the image width and HHH the image height, as provided by the calibration application.
			  *
			  * \endcode
			  * \note All parameter names may have an optional prefix, set with the "prefix" parameter.
			  *  For example, if prefix="LEFT_", the expected variable name "camera_index" in the config section will be "LEFT_camera_index", and so on.
			  */
			void  loadOptionsFrom(
				const mrpt::utils::CConfigFileBase & configSource,
				const std::string & sectionName,
				const std::string & prefix = std::string() );

			TYMLReadResult m_camera_int_params_from_yml( const std::string & _file_name = std::string() );
			TYMLReadResult m_camera_ext_params_from_yml( const std::string & _file_name = std::string() );
			TYMLReadResult m_rectify_map_from_yml( const std::string & _file_name = std::string() );

		}; // end-TCaptureOptions_DUO3D

		/** This "software driver" implements the communication protocol for interfacing a DUO3D Stereo Camera 
		  *
		  *   See also the example configuration file for rawlog-grabber in "share/mrpt/config_files/rawlog-grabber".
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *		image_width   			= 640			// [int]	x Resolution
		  *		image_height  			= 480			// [int]	y Resolution
		  *		fps						= 30			// [int]	Frames per second (<= 30)
		  *		exposure				= 50			// [int]	Exposure value (1..100)
		  *		led						= 0				// [int]	Led intensity (only for some device models) (1..100).
		  *		gain					= 50			// [int]	Camera gain (1..100)
		  *		capture_rectified 		= false			// [bool]	Rectify captured images
		  *		capture_imu 			= true			// [bool]	Capture IMU data from DUO3D device (if available)
		  *		calibration_from_file	= true			// [bool]	Use YML calibration files provided by calibration application supplied with DUO3D device
		  *	  	intrinsic_filename		= ""			// [string]	Intrinsic parameters file. This filename should contain a substring _RWWWxHHH_ with WWW being the image width and HHH the image height, as provided by the calibration application.
		  *		extrinsic_filename		= ""			// [string]	Extrinsic parameters file. This filename should contain a substring _RWWWxHHH_ with WWW being the image width and HHH the image height, as provided by the calibration application.
		  *		rectify_map_filename	= ""			// [string]	Rectification map file. This filename should contain a substring _RWWWxHHH_ with WWW being the image width and HHH the image height, as provided by the calibration application.
		  *
		  *   pose_x=0.21	// camera 3D position in the robot (meters)
		  *   pose_y=0
		  *   pose_z=0.34
		  *   pose_yaw=0	// Angles in degrees
		  *   pose_pitch=0
		  *   pose_roll=0
		  *  \endcode
		  *
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CDUO3DCamera : public mrpt::utils::CUncopiable
		{

		protected:
			// members
			// [USER-DEFINED]
			TCaptureOptions_DUO3D	m_options;
			
			// [INTERNAL]
			mrpt::vision::CStereoRectifyMap m_rectify_map;

			void * m_duo;          //!< Opaque pointer to DUO's DUOInstance
			void * m_pframe_data; //!< Pointer, to be reinterpreted as "PDUOFrame"
			void * m_evFrame;     //!< DUO's HANDLE

		public:
			/** Default Constructor (does not open the camera) */
			CDUO3DCamera();

			/** Constructor: tries to open the camera with the given options. Raises an exception on error. \sa open() */
			CDUO3DCamera( const TCaptureOptions_DUO3D & options );

			/** Destructor  */
			virtual ~CDUO3DCamera();

			/** Returns the current settings of the camera */
			const TCaptureOptions_DUO3D & getCameraOptions() const { return m_options; }

			/** Tries to open the camera with the given options, and starts capturing. Raises an exception on error.
			  * \param[in] startCapture If set to false, the camera is only opened and configured, but a posterior call to startCapture() is required to start grabbing data.
			  * \sa close(), startCapture()
			  */
			void open( const TCaptureOptions_DUO3D & options, const bool startCapture = true );

			/** Start the actual data capture of the camera. Must be called after open(), only when "startCapture" was set to false.
			  */
			void startCapture();

			/** Stop capture. */
			void stopCapture();

			/** Stop capture and closes the opened camera, if any. Called automatically on object destruction. */
			void close();

			/** Specific laser scanner "software drivers" must process here new data from the I/O stream, and, if a whole scan has arrived, return it.
			  *  This method will be typically called in a different thread than other methods, and will be called in a timely fashion.
			  */
			void getObservations(
				mrpt::obs::CObservationStereoImages	& outObservation_img,
				mrpt::obs::CObservationIMU				& outObservation_imu,
				bool									& there_is_img,
				bool									& there_is_imu );

			/** Indicates if the camera is grabbing IMU data */
			inline bool captureIMUIsSet() { return m_options.m_capture_imu; }

			/** Returned pointer to be reinterpreted as DUO3D's "HANDLE" */
			inline void* getEvent() { return this->m_evFrame; }

			/** frame is a reinterpreted PDUOFrame */
			inline void setDataFrame( void* frame ) { this->m_pframe_data = frame; }

		protected:
			/** Queries the DUO3D Camera firmware version */
			bool queryVersion(std::string version, bool printOutVersion = false);
			
			/** Gets a stereo frame from the DUO3D Camera (void* to be reinterpreted as PDUOFrame) */
			void * m_get_duo_frame(); 

			/** Opens DUO3D camera */
			bool m_open_duo_camera(int width, int height, float fps);
			
			/** Closes DUO3D camera */
			void m_close_duo_camera();
			
			/** Sets DUO3D camera Exposure setting */
			void m_set_exposure(float value);
			
			/** Sets DUO3D camera Gain setting  */
			void m_set_gain(float value);
			
			/** Sets DUO3D camera LED setting  */
			void m_set_led(float value);

		public:
			MRPT_MAKE_ALIGNED_OPERATOR_NEW

		};	// End of class
	} // End of namespace
} // End of namespace
#endif
