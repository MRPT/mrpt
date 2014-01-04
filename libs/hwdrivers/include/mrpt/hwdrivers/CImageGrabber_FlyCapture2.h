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



			int	frame_width, frame_height;	//!< Capture resolution (Default: 640x480)
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

			/** Tries to open the camera with the given options. Raises an exception on error. \sa close() */
			void open( const TCaptureOptions_FlyCapture2 &options );

			/** Closes the opened camera, if any. Called automatically on object destruction. */
			void close();

			/** Returns the PGR FlyCapture2 library version */
			static std::string getFC2version();

			/** Grab image from the camera. This method blocks until the next frame is captured.
			 * \return false on any error.
			*/
			bool  getObservation( mrpt::slam::CObservationImage &out_observation );


		};	// End of class

	} // End of NS
} // End of NS


#endif
