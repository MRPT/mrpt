/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#ifndef CVideoFileWriter_H
#define CVideoFileWriter_H

#include <mrpt/vision/utils.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/safe_pointers.h>

namespace mrpt
{
	namespace vision
	{
		/**  An output stream which takes a sequence of images and writes a video file in any of a given of compatible formats.
		  *
		  *  The output file is open when calling "open", and it's closed at destructor or after calling "close".
		  *
		  *   Example of usage:
		  *
		  *  \code
		  *    CVideoFileWriter  vid;
		  *    vid.open("test.avi",15,TPixelCoord(320,200), "MJPG");
		  *    CImage  img(320,200);
		  *    vid << img;
		  *    vid.close;
		  *  \endcode
		  *
		  *  There are two methods for adding frames to the video:
		  *    - The operator <<: Which will raise an exception on any error.
		  *    - The method writeImage, which does not raise any exception on errors.
		  *
		  * \note This class is a wrapper for OpenCV's CvVideoWriter.
		  *  \ingroup mrpt_vision_grp
 		  */
		class VISION_IMPEXP CVideoFileWriter
		{
		private:
			mrpt::utils::void_ptr_noncopy	m_video; //!< A pointer to CvVideoWriter
			mrpt::vision::TImageSize		m_img_size; //!< A copy of the video size

		public:
			CVideoFileWriter();	//!< Default constructor, which does not open any file
			virtual ~CVideoFileWriter();		//!< Destructor

			/** Open a file for writing the video.
			  *  \param out_file The video file to create for output.
			  *  \param fourcc The video codec, as a string. See notes below.
			  *  \paam fps The video FPS (frames per seconds).
			  *  \param frameSize The size of the video frames. All subsequent images must be of this size.
			  *  \param isColor Set to false to create a grayscale video.
			  *
			  * \note If fourcc is left as an empty string a default codec will be seleceted (e.g. "IYUV").
			  * \note Other valid values for "fourcc" are: "PIM1" -> MPEG1, "MJPG" -> Motion JPEG, "XVID", etc...
			  *
			  * \return false on any error, true on success.
			  */
			bool open(
				const std::string &out_file,
				double fps,
				const mrpt::vision::TImageSize & frameSize,
				const std::string &fourcc = std::string(""),
				bool isColor = true );

			/** Finish the file writing and close the file output
			  */
			void close();

			/** Return true if already successfully open with open() and not closed yet. */
			bool isOpen() const;

			/** Write image to the video file.
			  * \exception std::exception On any error
			  */
			const CVideoFileWriter& operator << (const mrpt::utils::CImage& img) const;

			/**  Write image to the video file (method function, alternative to the operator <<).
			  * \return false on any error
			  */
			bool writeImage(const mrpt::utils::CImage& img) const;


		}; // end of class

	} // end of namespace
} // end of namespace

#endif
