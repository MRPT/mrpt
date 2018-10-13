/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/vision/utils.h>
#include <mrpt/img/CImage.h>
#include <mrpt/core/safe_pointers.h>

namespace mrpt::vision
{
/**  An output stream which takes a sequence of images and writes a video file
 * in any of a given of compatible formats.
 *
 *  The output file is open when calling "open", and it's closed at destructor
 * or after calling "close".
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
class CVideoFileWriter
{
   private:
	/** A pointer to CvVideoWriter */
	mrpt::void_ptr_noncopy m_video;
	/** A copy of the video size */
	mrpt::img::TImageSize m_img_size;

   public:
	/** Default constructor, which does not open any file */
	CVideoFileWriter();
	/** Destructor */
	virtual ~CVideoFileWriter();

	/** Open a file for writing the video.
	 *  \param out_file The video file to create for output.
	 *  \param fourcc The video codec, as a string. See notes below.
	 *  \paam fps The video FPS (frames per seconds).
	 *  \param frameSize The size of the video frames. All subsequent images
	 * must be of this size.
	 *  \param isColor Set to false to create a grayscale video.
	 *
	 * \note If fourcc is left as an empty string a default codec will be
	 * seleceted (e.g. "IYUV").
	 * \note Other valid values for "fourcc" are: "PIM1" -> MPEG1, "MJPG" ->
	 * Motion JPEG, "XVID", etc...
	 *
	 * \return false on any error, true on success.
	 */
	bool open(
		const std::string& out_file, double fps,
		const mrpt::img::TImageSize& frameSize,
		const std::string& fourcc = std::string(""), bool isColor = true);

	/** Finish the file writing and close the file output
	 */
	void close();

	/** Return true if already successfully open with open() and not closed yet.
	 */
	bool isOpen() const;

	/** Write image to the video file.
	 * \exception std::exception On any error
	 */
	const CVideoFileWriter& operator<<(const mrpt::img::CImage& img) const;

	/**  Write image to the video file (method function, alternative to the
	 * operator <<).
	 * \return false on any error
	 */
	bool writeImage(const mrpt::img::CImage& img) const;

};  // end of class

}  // namespace mrpt::vision
