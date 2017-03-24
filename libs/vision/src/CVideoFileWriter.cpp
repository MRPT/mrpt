/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/CVideoFileWriter.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

#define M_WRITER (const_cast<CvVideoWriter*>( static_cast<const CvVideoWriter*>(m_video.get())) )
#define M_WRITER_PTR (reinterpret_cast<CvVideoWriter**>(m_video.getPtrToPtr()))


using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace mrpt::utils;

/* ----------------------------------------------------------
						Ctor
   ---------------------------------------------------------- */
CVideoFileWriter::CVideoFileWriter() : m_video(), m_img_size(0,0)
{
}

/* ----------------------------------------------------------
						Dtor
   ---------------------------------------------------------- */
CVideoFileWriter::~CVideoFileWriter()
{
	close();
}

/* ----------------------------------------------------------
						open
   ---------------------------------------------------------- */
bool CVideoFileWriter::open(
	const std::string &out_file,
	double fps,
	const mrpt::utils::TImageSize & frameSize,
	const std::string &fourcc,
	bool isColor )
{
#if MRPT_HAS_OPENCV
	close();

	int cc;

	if (fourcc.empty())
	{
#if MRPT_OPENCV_VERSION_NUM<=0x100
		cc = 0; // Default
#else
	#ifdef MRPT_OS_WINDOWS
		cc = CV_FOURCC_DEFAULT; // Default CV_FOURCC_PROMPT;
	#else
		cc = CV_FOURCC_DEFAULT; // Default
	#endif
#endif
	}
	else
	if (fourcc.size()==4)
	{
		cc = CV_FOURCC( fourcc[0],fourcc[1],fourcc[2],fourcc[3] );
	}
	else
	{
		std::cerr << "[CVideoFileWriter::open] fourcc string must be four character length or empty for default." << std::endl;
		return false;
	}

	m_video = cvCreateVideoWriter(out_file.c_str(),cc,fps,cvSize(frameSize.x,frameSize.y),isColor ? 1:0);

	m_img_size = frameSize;

	return m_video.get() != NULL;
#else
	std::cerr << "[CVideoFileWriter::open] ERROR: MRPT was compiled without OpenCV support "  << std::endl;
	return false;
#endif
}


/* ----------------------------------------------------------
						close
   ---------------------------------------------------------- */
void CVideoFileWriter::close()
{
#if MRPT_HAS_OPENCV
	if (!M_WRITER) return;
	cvReleaseVideoWriter( M_WRITER_PTR );
	*M_WRITER_PTR = NULL;
#endif
}

/* ----------------------------------------------------------
						isOpen
   ---------------------------------------------------------- */
bool CVideoFileWriter::isOpen() const
{
#if MRPT_HAS_OPENCV
	return (M_WRITER!=NULL);
#else
	return false;
#endif
}


/* ----------------------------------------------------------
						operator <<
   ---------------------------------------------------------- */
const CVideoFileWriter& CVideoFileWriter::operator << (const mrpt::utils::CImage& img) const
{
	if (!m_video.get())
		THROW_EXCEPTION("Call open first")

	if ((size_t)m_img_size.x!=img.getWidth() || (size_t)m_img_size.y!=img.getHeight())
		THROW_EXCEPTION(format("Video frame size is %ix%i but image is %ux%u", m_img_size.x,m_img_size.y,(unsigned)img.getWidth(),(unsigned)img.getHeight() ));

#if MRPT_HAS_OPENCV
	if (! cvWriteFrame( M_WRITER, img.getAs<IplImage>() ) )
		THROW_EXCEPTION("Error writing image frame to video file")
#endif
	return *this;
}

/* ----------------------------------------------------------
						writeImage
   ---------------------------------------------------------- */
bool CVideoFileWriter::writeImage(const mrpt::utils::CImage& img) const
{
	if (!m_video.get())
		return false;

	if ((size_t)m_img_size.x!=img.getWidth() || (size_t)m_img_size.y!=img.getHeight())
	{
		std::cout << format("[CVideoFileWriter::writeImage] Error: video frame size is %ix%i but image is %ux%u", m_img_size.x,m_img_size.y,(unsigned)img.getWidth(),(unsigned)img.getHeight() ) << std::endl;
		return false;
	}

#if MRPT_HAS_OPENCV
	return 0!= cvWriteFrame( M_WRITER, img.getAs<IplImage>() );
#else
	return false;
#endif
}

