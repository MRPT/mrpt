/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "detectors-precomp.h"  // Precompiled headers

#include <mrpt/detectors/CCascadeClassifierDetection.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/system/threads.h> // sleep()

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

using namespace mrpt::detectors;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace std;

#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
using namespace cv;
#endif

#define CASCADE  (reinterpret_cast<CascadeClassifier*>(m_cascade))
#define CASCADE_CONST  (reinterpret_cast<const CascadeClassifier*>(m_cascade))

// ------------------------------------------------------
//				CCascadeClassifierDetection
// ------------------------------------------------------

CCascadeClassifierDetection::CCascadeClassifierDetection( )
{
	// Check if MRPT is using OpenCV
#if !MRPT_HAS_OPENCV ||  MRPT_OPENCV_VERSION_NUM<0x200
	THROW_EXCEPTION("CCascadeClassifierDetection class requires MRPT built against OpenCV >=2.0")
#endif
}

// ------------------------------------------------------
//  			~CCascadeClassifierDetection
// ------------------------------------------------------

CCascadeClassifierDetection::~CCascadeClassifierDetection()
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
	delete CASCADE;
#endif
}

// ------------------------------------------------------
//						  init
// ------------------------------------------------------

void CCascadeClassifierDetection::init(const mrpt::utils::CConfigFileBase &config)
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
	// load configuration values
	m_options.cascadeFileName		= config.read_string("CascadeClassifier","cascadeFilename","");
	m_options.scaleFactor			= config.read_double("DetectionOptions","scaleFactor",1.1);
	m_options.minNeighbors			= config.read_int("DetectionOptions","minNeighbors",3);
	m_options.flags					= config.read_int("DetectionOptions","flags",0);
	m_options.minSize				= config.read_int("DetectionOptions","minSize",30);

	m_cascade = new CascadeClassifier();

	// Load cascade classifier from file
	CASCADE->load( m_options.cascadeFileName );

	// Check if cascade is empty
	if ( CASCADE->empty() )
		throw  std::runtime_error("Incorrect cascade file.");
#endif
}

// ------------------------------------------------------
//				detectObjects (*CObservation)
// ------------------------------------------------------

void CCascadeClassifierDetection::detectObjects_Impl(const CObservation *obs, vector_detectable_object &detected)
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
	// Obtain image from generic observation
	const mrpt::utils::CImage *img = NULL;

	if (IS_CLASS(obs,CObservationImage))
	{
		const CObservationImage* o = static_cast<const CObservationImage*>(obs);
		img = &o->image;
	}
	else if ( IS_CLASS(obs,CObservationStereoImages) )
	{
		const CObservationStereoImages* o = static_cast<const CObservationStereoImages*>(obs);
		img = &o->imageLeft;
	}
	else if (IS_CLASS(obs, CObservation3DRangeScan ) )
	{
		const CObservation3DRangeScan* o = static_cast<const CObservation3DRangeScan*>(obs);
		img = &o->intensityImage;
	}
	if (!img)
	{
	    mrpt::system::sleep(2);
	    return;
	}

	vector<Rect> objects;

	// Some needed preprocessing
	const CImage img_gray( *img, FAST_REF_OR_CONVERT_TO_GRAY );

	// Convert to IplImage and copy it
	const IplImage *image = img_gray.getAs<IplImage>();

	// Detect objects
	CASCADE->detectMultiScale( cv::cvarrToMat(image), objects, m_options.scaleFactor,
								m_options.minNeighbors, m_options.flags,
								Size(m_options.minSize,m_options.minSize) );

	unsigned int N = objects.size();
	//detected.resize( N );

	// Convert from cv::Rect to vision::CDetectable2D
	for ( unsigned int i = 0; i < N; i++ )
	{
		CDetectable2DPtr obj =
			CDetectable2DPtr( new CDetectable2D( objects[i].x, objects[i].y, objects[i].height, objects[i].width ) );

		detected.push_back((CDetectableObjectPtr)obj);
	}
#endif
}
