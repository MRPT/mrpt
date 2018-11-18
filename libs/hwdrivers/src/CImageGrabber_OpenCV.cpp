/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers

#include <mrpt/hwdrivers/CImageGrabber_OpenCV.h>
#include <mrpt/otherlibs/do_opencv_includes.h>
#include <thread>

#ifdef HAVE_OPENCV_VIDEOIO
// cv::VideoCapture moved from highgui in opencv2 to videoio in opencv3:
#include <opencv2/videoio.hpp>
#endif

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;

struct CImageGrabber_OpenCV::Impl
{
#if MRPT_HAS_OPENCV
	cv::VideoCapture cap;
#endif
};

/*-------------------------------------------------------------
				Constructor
-------------------------------------------------------------*/
CImageGrabber_OpenCV::CImageGrabber_OpenCV(
	int cameraIndex, TCameraType cameraType, const TCaptureCVOptions& options)
	: m_capture(mrpt::make_impl<CImageGrabber_OpenCV::Impl>())
{
	MRPT_START
	m_bInitialized = false;

#if MRPT_HAS_OPENCV
	int cv_cap_indx = 0;
	switch (cameraType)
	{
		case CAMERA_CV_AUTODETECT:
			cv_cap_indx = CV_CAP_ANY;
			break;
		case CAMERA_CV_DC1394:
			cv_cap_indx = CV_CAP_DC1394;
			break;
		case CAMERA_CV_VFL:
			cv_cap_indx = CV_CAP_V4L;
			break;
		case CAMERA_CV_VFW:
			cv_cap_indx = CV_CAP_VFW;
			break;
		case CAMERA_CV_MIL:
			cv_cap_indx = CV_CAP_MIL;
			break;
		case CAMERA_CV_DSHOW:
			cv_cap_indx = CV_CAP_DSHOW;
			break;
		default:
			THROW_EXCEPTION_FMT("Invalid camera type: %i", cameraType);
	}

	cv_cap_indx += cameraIndex;

	// Open camera:
	if (!m_capture->cap.open(cv_cap_indx))
	{
		cerr << format(
			"[CImageGrabber_OpenCV] ERROR: Can't open camera '%i'!!\n",
			cameraIndex);
		return;
	}

	// Set properties:
	// Based on code from Orocos project. Thanks!
	// ----------------------------------------
	// Global settings
	if (options.gain != 0)
	{
		if (!m_capture->cap.set(CV_CAP_PROP_GAIN, options.gain))
			cerr << "[CImageGrabber_OpenCV] Warning: Could not set the "
					"capturing gain property!"
				 << endl;
	}

	// Settings only for firewire
	if (cameraType == CAMERA_CV_DC1394)
	{
		if (options.frame_height != 0 && options.frame_width != 0)
		{
			// MODE_320x240_YUV422 ****
			//
			enum
			{
				MY_MODE_160x120_YUV444 = 64,
				MY_MODE_320x240_YUV422,  // ***
				MY_MODE_640x480_YUV411,
				MY_MODE_640x480_YUV422,  // ***
				MY_MODE_640x480_RGB,  // ?
				MY_MODE_640x480_MONO,  // ***
				MY_MODE_640x480_MONO16
			};

			int cvMode1394 = -1;
			if (options.frame_height == 320 && options.frame_width == 240)
				cvMode1394 = MY_MODE_320x240_YUV422;
			else if (
				options.frame_height == 640 && options.frame_width == 480 &&
				!options.ieee1394_grayscale)
				cvMode1394 = MY_MODE_640x480_YUV422;
			else if (
				options.frame_height == 640 && options.frame_width == 480 &&
				options.ieee1394_grayscale)
				cvMode1394 = MY_MODE_640x480_MONO;

			if (cvMode1394 > 0)
			{
				if (!m_capture->cap.set(CV_CAP_PROP_MODE, cvMode1394))
					cerr << "[CImageGrabber_OpenCV] Warning: Could not set the "
							"capturing mode "
						 << cvMode1394 << " property!" << endl;
			}
			else
				cerr << "[CImageGrabber_OpenCV] Warning: Not valid combination "
						"of width x height x color mode for OpenCV/IEEE1394 "
						"interface"
					 << endl;
		}

		// Not needed: Default seems to be = 1
		// if(cvSetCaptureProperty(M_CAPTURE,CV_CAP_PROP_CONVERT_RGB,_capture_convert.value())<1)
		//	cerr << "[CImageGrabber_OpenCV] Warning: Could not set the RGB
		// conversion property!" << endl;

		if (!m_capture->cap.set(CV_CAP_PROP_FPS, options.ieee1394_fps))
			cerr << "[CImageGrabber_OpenCV] Warning: Could not set the fps "
					"property!"
				 << endl;
	}

	// Settings only for V4L
	if (cameraType == CAMERA_CV_AUTODETECT || cameraType == CAMERA_CV_VFL ||
		cameraType == CAMERA_CV_VFW || cameraType == CAMERA_CV_DSHOW)
	{
		if (options.frame_width != 0 && options.frame_height != 0)
		{
			// First set width then height. The first command always returns a
			// error!
			m_capture->cap.set(CV_CAP_PROP_FRAME_WIDTH, options.frame_width);
			if (!m_capture->cap.set(
					CV_CAP_PROP_FRAME_HEIGHT, options.frame_height))
				cerr << "[CImageGrabber_OpenCV] Warning: Could not set the "
						"frame width & height property!"
					 << endl;
		}
	}

	// remember that we successfully initialized everything
	m_bInitialized = true;
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}

CImageGrabber_OpenCV::CImageGrabber_OpenCV(const std::string& AVI_fileName)
	: m_capture(mrpt::make_impl<CImageGrabber_OpenCV::Impl>())
{
	MRPT_START
	m_bInitialized = false;

#if MRPT_HAS_OPENCV
	if (!m_capture->cap.open(AVI_fileName))
	{
		printf(
			"[CImageGrabber_OpenCV] Warning! Can't open AVI file '%s'!!\n",
			AVI_fileName.c_str());
		return;
	}

	// remember that we successfully initialized everything
	m_bInitialized = true;
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}

/*-------------------------------------------------------------
					Destructor
 -------------------------------------------------------------*/
CImageGrabber_OpenCV::~CImageGrabber_OpenCV()
{
#if MRPT_HAS_OPENCV
	if (m_bInitialized)
	{
		m_capture->cap.release();
	}
#endif
}

/*-------------------------------------------------------------
					get the image
 -------------------------------------------------------------*/
bool CImageGrabber_OpenCV::getObservation(
	mrpt::obs::CObservationImage& out_observation)
{
	MRPT_START

	if (!m_bInitialized) return false;

#if MRPT_HAS_OPENCV

	// Capture the image:
	if (!m_capture->cap.grab()) return false;

	// JL: Sometimes there're errors in some frames: try not to return an error
	// unless it seems
	//  there's no way:
	for (int nTries = 0; nTries < 10; nTries++)
	{
		cv::Mat capImg;
		if (m_capture->cap.retrieve(capImg))
		{
			// Fill the output class:
			out_observation.timestamp = mrpt::system::now();
			out_observation.image.setFromMatNoCopy(capImg);
			return true;
		}
		cerr << "[CImageGrabber_OpenCV] WARNING: Ignoring error #" << nTries + 1
			 << " retrieving frame..." << endl;
		std::this_thread::sleep_for(1ms);
	}
	return false;
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}
