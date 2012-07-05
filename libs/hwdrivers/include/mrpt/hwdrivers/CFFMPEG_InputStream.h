/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
