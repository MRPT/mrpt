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
			/** @} */

		};

		/** A wrapper for Point Gray Research (PGR) FlyCapture2 API for capturing images from a USB3 and other cameras.
		  *  This class is only available when compiling MRPT with "MRPT_HAS_PGR_FLYCAPTURE2".
		  *
		  * \sa See the most generic camera grabber in MRPT: mrpt::hwdrivers::CCameraSensor
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP  CImageGrabber_FlyCapture2 : public mrpt::utils::CUncopiable
		{
		protected:
			void *m_camera;      //!< Opaque pointer to the FlyCapture2::Camera object. NULL if no camera is grabbing.
			void *m_camera_info; //!< Opaque pointer to the FlyCapture2::CameraInfo object. NULL if no camera is grabbing.

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
