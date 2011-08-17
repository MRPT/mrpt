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
#ifndef CVideoFileWritter_H
#define CVideoFileWritter_H

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
		  *    vid.open("test.avi","MJPG",15,TPixelCoord(320,200));
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
