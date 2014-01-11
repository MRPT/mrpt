/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CImageGrabber_FlyCapture2_H
#define CImageGrabber_FlyCapture2_H

#include <mrpt/slam/CObservationImage.h>
#include <mrpt/hwdrivers/link_pragmas.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** Options used when creating a camera capture object of type CImageGrabber_FlyCapture2   \ingroup mrpt_hwdrivers_grp */
		struct HWDRIVERS_IMPEXP TCaptureOptions_FlyCapture2
		{
			TCaptureOptions_FlyCapture2();
			
			/** @name Camera to open 
			  * @{ */
			unsigned int camera_index;   //!< (Default=0) If open_by_guid==false, will open the i'th camera based on this 0-based index.
			bool         open_by_guid;   //!< (Default=false) Set to true to force opening a camera by its GUID, in \a camera_guid
			unsigned int camera_guid[4]; //!< GUID of the camera to open, only when open_by_guid==true.
			/** @} */

			/** @name Camera settings 
			  * @{ */
			std::string   videomode;  //!< (Default="", which means default) A string with a video mode, from the list available in [FlyCapture2::VideoMode](http://www.ptgrey.com/support/downloads/documents/flycapture/Doxygen/html/), eg. "VIDEOMODE_640x480Y8", etc.
			std::string   framerate;  //!< (Default="", which means default) A string with a framerate, from the list available in [FlyCapture2::FrameRate](http://www.ptgrey.com/support/downloads/documents/flycapture/Doxygen/html/), eg. "FRAMERATE_30", etc.
			std::string   grabmode;   //!< (Default="BUFFER_FRAMES") A string with a grab mode, from the list available in [FlyCapture2::GrabMode](http://www.ptgrey.com/support/downloads/documents/flycapture/Doxygen/html/)
			int           grabTimeout; //!< (Default=5000) Time in milliseconds that RetrieveBuffer() and WaitForBufferEvent() will wait for an image before timing out and returning. 

			bool         trigger_enabled;   //!< (default=false) Enable non-free-running mode, only capturing when a given input trigger signal is detected. Refer to PGR docs.
			unsigned int trigger_polarity;  //!< (default=0) Refer to PGR docs.
			unsigned int trigger_source;    //!< (default=0) Refer to PGR docs.
			unsigned int trigger_mode;      //!< (default=0) Refer to PGR docs.

			bool         strobe_enabled;    //!< (default=false) Enable the generation of a strobe signal in GPIO. Refer to PGR docs.
			unsigned int strobe_source;     //!< (default=0)  Refer to PGR docs.
			unsigned int strobe_polarity;   //!< (default=0)  Refer to PGR docs.
			float        strobe_delay;      //!< (default=0.0) Delay in ms. Refer to PGR docs.
			float        strobe_duration;   //!< (default=1.0) Pulse durationin ms. Refer to PGR docs.
			/** @} */

			/** Loads all the options from a config file. 
			  * Expected format: 
			  *
			  * \code
			  * [sectionName]
			  * # Camera selection:
			  * camera_index = 0      // (Default=0) If open_by_guid==false, will open the i'th camera based on this 0-based index.
			  * open_by_guid = false  // (Default=false) Set to true to force opening a camera by its GUID, in \a camera_guid
			  * camera_guid  = 11223344-55667788-99AABBCC-DDEEFF00  // GUID of the camera to open, only when open_by_guid==true. Hexadecimal blocks separated by dashes ("-")
			  * 
			  * # Camera settings:
			  * videomode   = VIDEOMODE_640x480Y8 // (Default="", which means default) A string with a video mode, from the list available in [FlyCapture2::VideoMode](http://www.ptgrey.com/support/downloads/documents/flycapture/Doxygen/html/), eg. "VIDEOMODE_640x480Y8", etc.
			  * framerate   = FRAMERATE_30        // (Default="", which means default) A string with a framerate, from the list available in [FlyCapture2::FrameRate](http://www.ptgrey.com/support/downloads/documents/flycapture/Doxygen/html/), eg. "FRAMERATE_30", etc.
			  * grabmode    = BUFFER_FRAMES       // (Default="BUFFER_FRAMES") A string with a grab mode, from the list available in [FlyCapture2::GrabMode](http://www.ptgrey.com/support/downloads/documents/flycapture/Doxygen/html/)
			  * grabTimeout = 5000                // (Default=5000) Time in milliseconds that RetrieveBuffer() and WaitForBufferEvent() will wait for an image before timing out and returning. 
			  * 
			  * trigger_enabled = false // (default=false) Enable non-free-running mode, only capturing when a given input trigger signal is detected. Refer to PGR docs.
			  * #trigger_polarity = 0      // (default=0) Refer to PGR docs.
			  * #trigger_source   = 0      // (default=0) Refer to PGR docs.
			  * #trigger_mode     = 0      // (default=0) Refer to PGR docs.
			  * 
			  * strobe_enabled   = false // (default=false) Enable the generation of a strobe signal in GPIO. Refer to PGR docs.
			  * #strobe_source    = 1     // (default=0)  Refer to PGR docs.
			  * #strobe_polarity  = 0     // (default=0)  Refer to PGR docs.
			  * #strobe_delay     = 0.0   // (default=0.0) Delay in ms. Refer to PGR docs.
			  * #strobe_duration  = 1.0   // (default=1.0) Pulse durationin ms. Refer to PGR docs.
			  * 
			  * \endcode
			  * \note All parameter names may have an optional prefix, set with the "prefix" parameter. 
			  *  For example, if prefix="LEFT_", the expected variable name "camera_index" in the config section will be "LEFT_camera_index", and so on.
			  */
			void  loadOptionsFrom(
				const mrpt::utils::CConfigFileBase & configSource,
				const std::string & sectionName,
				const std::string & prefix = std::string() );

		};

		/** A wrapper for Point Gray Research (PGR) FlyCapture2 API for capturing images from Firewire, USB3 or GigaE cameras.
		  *  This class is only available when compiling MRPT with "MRPT_HAS_PGR_FLYCAPTURE2".
		  *
		  * \sa See the most generic camera grabber in MRPT: mrpt::hwdrivers::CCameraSensor
		  * \sa See example code in [samples]/captureVideoFlyCapture2 and [samples]/captureVideoFlyCapture2_stereo.
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP  CImageGrabber_FlyCapture2 : public mrpt::utils::CUncopiable
		{
		protected:
			void *m_camera;      //!< Opaque pointer to the FlyCapture2::Camera object. NULL if no camera is grabbing.
			void *m_camera_info; //!< Opaque pointer to the FlyCapture2::CameraInfo object. NULL if no camera is grabbing.
			void *m_img_buffer;  //!< Opaque pointer to the FlyCapture2::Image, used as a temporary buffer and to avoid mem alloc/reallocs.

			TCaptureOptions_FlyCapture2	 m_options;			//!< Camera options

		public:
			/** Constructor that does not open a camera. \sa open() */
			CImageGrabber_FlyCapture2();

			/** Constructor: tries to open the camera with the given options. Raises an exception on error. \sa open() */
			CImageGrabber_FlyCapture2( const TCaptureOptions_FlyCapture2 &options);

			/** Destructor */
			virtual ~CImageGrabber_FlyCapture2();

			/** Returns the current settings of the camera */
			const TCaptureOptions_FlyCapture2 & getCameraOptions() const { return m_options; }

			/** Tries to open the camera with the given options, and starts capture. Raises an exception on error. 
			  * \param[in] startCapture If set to false, the camera is only opened and configured, but a posterior call to startCapture() is required to start grabbing images.
			  * \sa close(), startCapture() 
			  */
			void open( const TCaptureOptions_FlyCapture2 &options, const bool startCapture = true );

			/** Start the actual image capture of the camera. Must be called after open(), only when "startCapture" was set to false.
			  * \sa startSyncCapture
			  */
			void startCapture();

			/** Starts a synchronous capture of several cameras, which must have been already opened.
			  * NOTE: This method only works with Firewire cameras, not with USB3 or GigaE ones (as confirmed by PGR support service).
			  * \sa startCapture
			  */
			static void startSyncCapture( int numCameras, const CImageGrabber_FlyCapture2 **cameras_array );

			/** Stop capture. */
			void stopCapture();

			/** Stop capture and closes the opened camera, if any. Called automatically on object destruction. */
			void close();

			/** Returns the PGR FlyCapture2 library version */
			static std::string getFC2version();

			/** Grab image from the camera. This method blocks until the next frame is captured.
			 * \return false on any error. */
			bool  getObservation( mrpt::slam::CObservationImage &out_observation );


		};	// End of class

	} // End of NS
} // End of NS


#endif
