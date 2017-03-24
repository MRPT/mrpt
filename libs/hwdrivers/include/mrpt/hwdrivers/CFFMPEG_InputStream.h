/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef  CFFMPEG_InputStream_H
#define  CFFMPEG_InputStream_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/safe_pointers.h>

#include <mrpt/hwdrivers/link_pragmas.h>


/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace hwdrivers
	{
		/** A generic class which process a video file or other kind of input stream (http, rtsp) and allows the extraction of images frame by frame.
		  *  Video sources can be open with "openURL", which can manage both video files and "rtsp://" sources (IP cameras).
		  *
		  *  Frames are retrieved by calling CFFMPEG_InputStream::retrieveFrame
		  *
		  *   For an example of usage, see the file "samples/grab_camera_ffmpeg"
		  *
		  * \note This class is an easy to use C++ wrapper for ffmpeg libraries (libavcodec). In Unix systems these libraries must be installed in the system as explained in <a href="http://www.mrpt.org/Building_and_Installing_Instructions" > MRPT's wiki</a>. In Win32, a precompiled version for Visual Studio must be also downloaded as explained in <a href="http://www.mrpt.org/Building_and_Installing_Instructions" >the wiki</a>.
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CFFMPEG_InputStream
		{
		private:
			mrpt::utils::void_ptr_noncopy	m_state;	//!< The internal ffmpeg state
			std::string  m_url;	//!< The open URL
			bool  m_grab_as_grayscale;

		public:
			CFFMPEG_InputStream();				//!< Default constructor, does not open any video source at startup
			virtual ~CFFMPEG_InputStream();		//!< Destructor

			/** Open a video file or a video stream (rtsp://)
			  *  This can be used to open local video files (eg. "myVideo.avi", "c:\a.mpeg") and also IP cameras (e. "rtsp://a.b.c.d/live.sdp").
			  *  However, note that there is currently no support for user/password in IP access.
			  *  If verbose is set to true, more information about the video will be dumped to cout.
			  *
			  * \sa close, retrieveFrame
			  * \return false on any error (and error info dumped to cerr), true on success.
			  */
			bool openURL( const std::string &url, bool grab_as_grayscale = false, bool verbose = false );

			bool isOpen() const;	//!< Return whether the video source was open correctly

			/** Close the video stream (this is called automatically at destruction).
			  * \sa openURL
			  */
			void close();

			double getVideoFPS() const;	//!< Get the frame-per-second (FPS) of the video source, or "-1" if the video is not open.

			/** Get the next frame from the video stream.
			  *  Note that for remote streams (IP cameras) this method may block until enough information is read to generate a new frame.
			  *  Images are returned as 8-bit depth grayscale if "grab_as_grayscale" is true.
			  *  \return false on any error, true on success.
			  *  \sa openURL, close, isOpen
			  */
			bool retrieveFrame( mrpt::utils::CImage &out_img );


		};

	}
}


#endif
