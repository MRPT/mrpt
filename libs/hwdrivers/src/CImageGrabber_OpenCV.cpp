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

#include <mrpt/hwdrivers.h> // Precompiled header

#include <mrpt/system/threads.h>

#if MRPT_HAS_OPENCV
	// Include the OPENCV libraries:
	#define CV_NO_CVV_IMAGE   // Avoid CImage name crash

#	if MRPT_OPENCV_VERSION_NUM>=0x211
#		include <opencv2/core/core.hpp>
#		include <opencv2/highgui/highgui.hpp>
#		include <opencv2/imgproc/imgproc.hpp>
#		include <opencv2/imgproc/imgproc_c.h>
#		include <opencv2/calib3d/calib3d.hpp>
#	else
#		include <cv.h>
#		include <highgui.h>
#	endif

	#ifdef CImage	// For old OpenCV versions (<=1.0.0)
	#undef CImage
	#endif

	#include <mrpt/hwdrivers/CImageGrabber_OpenCV.h>
#endif

#define M_CAPTURE  (static_cast<CvCapture*>(m_capture.get()))

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;

/*-------------------------------------------------------------
					Constructor
 -------------------------------------------------------------*/
CImageGrabber_OpenCV::CImageGrabber_OpenCV(
	int 			cameraIndex,
	TCameraType 	cameraType,
	const TCaptureCVOptions &options
	)
{
	MRPT_START
	m_bInitialized = false;

#if MRPT_HAS_OPENCV
	int	cv_cap_indx = 0;
	switch (cameraType)
	{
		case CAMERA_CV_AUTODETECT:	cv_cap_indx = CV_CAP_ANY; break;
		case CAMERA_CV_DC1394:	cv_cap_indx = CV_CAP_DC1394; break;
		case CAMERA_CV_VFL:	cv_cap_indx = CV_CAP_V4L; break;
		case CAMERA_CV_VFW:	cv_cap_indx = CV_CAP_VFW; break;
		case CAMERA_CV_MIL:	cv_cap_indx = CV_CAP_MIL; break;
#if MRPT_OPENCV_VERSION_NUM >= 0x111
		case CAMERA_CV_DSHOW: cv_cap_indx = CV_CAP_DSHOW; break;
		// *** HAVE YOU HAD A COMPILER ERROR NEAR THIS LINE?? : You need OpenCV >=1.1.1, (2.0 final release) or a SVN version ***
#endif
		default: THROW_EXCEPTION_CUSTOM_MSG1("Invalid camera type: %i", cameraType);
	}

	cv_cap_indx += cameraIndex;

	m_capture = cvCaptureFromCAM( cv_cap_indx );

	// If we have OPENCV>=1.1.1, we can determine the type of the capturer
	//  if it was CAMERA_CV_AUTODETECT
#if MRPT_OPENCV_VERSION_NUM >= 0x111
	if (cameraType==CAMERA_CV_AUTODETECT)
	{
		cv_cap_indx = cvGetCaptureDomain(M_CAPTURE);
		// *** HAVE YOU HAD A COMPILER ERROR NEAR THIS LINE?? : You need OpenCV >=1.1.0, (2.0 final release) or a SVN version ***
		switch (cv_cap_indx)
		{
			case CV_CAP_ANY:	cameraType = CAMERA_CV_AUTODETECT; break;
			case CV_CAP_DC1394:	cameraType = CAMERA_CV_DC1394; break;
			//case CV_CAP_V4L:
			case CV_CAP_VFW:	cameraType = CAMERA_CV_VFW; break;
			case CV_CAP_MIL:	cameraType = CAMERA_CV_MIL; break;
			case CV_CAP_DSHOW: cameraType = CAMERA_CV_DSHOW; break;
			default: THROW_EXCEPTION_CUSTOM_MSG1("Invalid camera type: %i", cameraType);
		}
	}
#endif

	if (!m_capture.get())
	{
	   cerr << format("[CImageGrabber_OpenCV] ERROR: Can't open camera '%i'!!\n", cameraIndex);
	   return;
	}

	// Set properties:
	// Based on code from Orocos project. Thanks!
	// ----------------------------------------
	// Global settings
	if (options.gain!=0)
	{
		if(cvSetCaptureProperty(M_CAPTURE,CV_CAP_PROP_GAIN,options.gain)<1)
		   cerr << "[CImageGrabber_OpenCV] Warning: Could not set the capturing gain property!" << endl;
	}

	// Settings only for firewire
	if(cameraType==CAMERA_CV_DC1394)
	{
		if (options.frame_height!=0 && options.frame_width!=0)
		{
			//MODE_320x240_YUV422 ****
			//
			enum {
				MY_MODE_160x120_YUV444= 64,
				MY_MODE_320x240_YUV422, // ***
				MY_MODE_640x480_YUV411,
				MY_MODE_640x480_YUV422, // ***
				MY_MODE_640x480_RGB, // ?
				MY_MODE_640x480_MONO, // ***
				MY_MODE_640x480_MONO16
			};

			int cvMode1394=-1;
			if (options.frame_height==320 && options.frame_width==240)
				cvMode1394 = MY_MODE_320x240_YUV422;
			else if (options.frame_height==640 && options.frame_width==480 && !options.ieee1394_grayscale)
				cvMode1394 = MY_MODE_640x480_YUV422;
			else if (options.frame_height==640 && options.frame_width==480 && options.ieee1394_grayscale)
				cvMode1394 = MY_MODE_640x480_MONO;

			if (cvMode1394>0)
			{
				if(cvSetCaptureProperty(M_CAPTURE,CV_CAP_PROP_MODE,cvMode1394)<1)
					cerr << "[CImageGrabber_OpenCV] Warning: Could not set the capturing mode "<< cvMode1394 << " property!" << endl;
			}
			else cerr << "[CImageGrabber_OpenCV] Warning: Not valid combination of width x height x color mode for OpenCV/IEEE1394 interface" << endl;
		}

		// Not needed: Default seems to be = 1
		//if(cvSetCaptureProperty(M_CAPTURE,CV_CAP_PROP_CONVERT_RGB,_capture_convert.value())<1)
		//	cerr << "[CImageGrabber_OpenCV] Warning: Could not set the RGB conversion property!" << endl;

		if(cvSetCaptureProperty(M_CAPTURE,CV_CAP_PROP_FPS, options.ieee1394_fps )<1)
			cerr << "[CImageGrabber_OpenCV] Warning: Could not set the fps property!" << endl;
	}

	// Settings only for V4L
	if(cameraType == CAMERA_CV_VFL || cameraType == CAMERA_CV_VFW || cameraType == CAMERA_CV_DSHOW  )
	{
		if (options.frame_width!=0 && options.frame_height!=0)
		{
			// First set width then height. The first command always returns a error!
			cvSetCaptureProperty(M_CAPTURE,CV_CAP_PROP_FRAME_WIDTH,options.frame_width);
			if(cvSetCaptureProperty(M_CAPTURE,CV_CAP_PROP_FRAME_HEIGHT,options.frame_height)<1)
				cerr << "[CImageGrabber_OpenCV] Warning: Could not set the frame width & height property!" << endl;
		}
	}

	// remember that we successfully initialized everything
	m_bInitialized = true;
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}

/*-------------------------------------------------------------
					Constructor
 -------------------------------------------------------------*/
CImageGrabber_OpenCV::CImageGrabber_OpenCV( const std::string &AVI_fileName )
{
   MRPT_START
   m_bInitialized = false;

#if MRPT_HAS_OPENCV
   m_bInitialized = false;

   m_capture = cvCaptureFromAVI( AVI_fileName.c_str() );

   if (!m_capture.get())
   {
	   printf("[CImageGrabber_OpenCV] Warning! Can't open AVI file '%s'!!\n", AVI_fileName.c_str());
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
	if ( m_bInitialized )
	{
		CvCapture *cap = M_CAPTURE;
		cvReleaseCapture( &cap );
		m_capture = NULL;
	}
#endif
}


/*-------------------------------------------------------------
					get the image
 -------------------------------------------------------------*/
bool  CImageGrabber_OpenCV::getObservation( mrpt::slam::CObservationImage &out_observation)
{
   MRPT_START

   if (!m_bInitialized) return false;

#if MRPT_HAS_OPENCV

	// Capture the image:
	if (!cvGrabFrame(M_CAPTURE))
		return false;

	IplImage *capImg = NULL;

	// JL: Sometimes there're errors in some frames: try not to return an error unless it seems
	//  there's no way:
	for (int nTries=0;nTries<10;nTries++)
	{
        capImg = cvRetrieveFrame(M_CAPTURE);
        if (capImg) break;
        cerr << "[CImageGrabber_OpenCV] WARNING: Ignoring error #" <<nTries+1 << " retrieving frame..." << endl;
        mrpt::system::sleep(1);
	}

	if(!capImg) return false;

	// Fill the output class:
	out_observation.image.setFromIplImageReadOnly( capImg );
	out_observation.timestamp = mrpt::system::now();

	return true;
#else
   THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
   MRPT_END
}
