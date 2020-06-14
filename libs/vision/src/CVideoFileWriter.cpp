/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"	 // Precompiled headers
//
#include <mrpt/vision/CVideoFileWriter.h>

// Universal include for all versions of OpenCV
#include <mrpt/3rdparty/do_opencv_includes.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::system;

/** cv::VideoWriter object */
struct Impl;
mrpt::pimpl<Impl> m_video;

struct CVideoFileWriter::Impl
{
#if MRPT_HAS_OPENCV
	cv::VideoWriter obj;
#endif
};

CVideoFileWriter::CVideoFileWriter()
	: m_video(mrpt::make_impl<CVideoFileWriter::Impl>())
{
}

CVideoFileWriter::~CVideoFileWriter() { close(); }

bool CVideoFileWriter::open(
	const std::string& out_file, double fps,
	const mrpt::img::TImageSize& frameSize, const std::string& fourcc,
	bool isColor)
{
#if MRPT_HAS_OPENCV
	close();

	int cc;

	if (fourcc.empty())
	{
		cc = CV_FOURCC_DEFAULT;	 // Default
	}
	else if (fourcc.size() == 4)
	{
		cc = CV_FOURCC(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
	}
	else
	{
		std::cerr << "[CVideoFileWriter::open] fourcc string must be four "
					 "character length or empty for default."
				  << std::endl;
		return false;
	}

	m_img_size = frameSize;

	return m_video->obj.open(
		out_file, cc, fps, cv::Size(frameSize.x, frameSize.y), isColor);

#else
	std::cerr << "[CVideoFileWriter::open] ERROR: MRPT was compiled without "
				 "OpenCV support "
			  << std::endl;
	return false;
#endif
}

void CVideoFileWriter::close()
{
#if MRPT_HAS_OPENCV
	m_video->obj.release();
#endif
}

/* ----------------------------------------------------------
						isOpen
   ---------------------------------------------------------- */
bool CVideoFileWriter::isOpen() const
{
#if MRPT_HAS_OPENCV
	return m_video->obj.isOpened();
#else
	return false;
#endif
}

const CVideoFileWriter& CVideoFileWriter::operator<<(
	const mrpt::img::CImage& img)
{
	writeImage(img);
	return *this;
}

bool CVideoFileWriter::writeImage(const mrpt::img::CImage& img)
{
	if (!m_video.get()) return false;

	if ((size_t)m_img_size.x != img.getWidth() ||
		(size_t)m_img_size.y != img.getHeight())
	{
		std::cout << format(
						 "[CVideoFileWriter::writeImage] Error: video frame "
						 "size is %ix%i but image is %ux%u",
						 m_img_size.x, m_img_size.y, (unsigned)img.getWidth(),
						 (unsigned)img.getHeight())
				  << std::endl;
		return false;
	}

#if MRPT_HAS_OPENCV
	const cv::Mat& m = img.asCvMatRef();
	m_video->obj.write(m);
	return true;
#else
	return false;
#endif
}
