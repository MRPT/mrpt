/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/vision.h>  // Precompiled headers
#include "do_opencv_includes.h"
#include <mrpt/vision/CCascadeClassifierDetection.h>

using namespace mrpt::vision;
using namespace mrpt::slam;
using namespace cv;
using namespace std;

#define CASCADE  (reinterpret_cast<CascadeClassifier*>(m_cascade))
#define CASCADE_CONST  (reinterpret_cast<const CascadeClassifier*>(m_cascade))

CCascadeClassifierDetection::CCascadeClassifierDetection( string configFilename )
{
	// load configuration values
	CConfigFile config(configFilename);

	m_options.cascadeFileName		= config.read_string("CascadeClassifier","cascadeFilename","");
	m_options.scaleFactor			= config.read_double("DetectionOptions","scaleFactor",1.1);
	m_options.minNeighbors			= config.read_int("DetectionOptions","minNeighbors",3);
	m_options.flags					= config.read_int("DetectionOptions","flags",0);
	m_options.minSize				= config.read_int("DetectionOptions","minSize",30);

	m_cascade = new CascadeClassifier();

	CASCADE->load( m_options.cascadeFileName );

	if ( CASCADE->empty() )
		throw  std::runtime_error("Incorrect cascade file.");
}

CCascadeClassifierDetection::~CCascadeClassifierDetection()
{
	delete CASCADE;
}


void CCascadeClassifierDetection::init(const mrpt::utils::CConfigFileBase &config)
{
	// load configuration values
	m_options.cascadeFileName		= config.read_string("CascadeClassifier","cascadeFilename","");
	m_options.scaleFactor			= config.read_double("DetectionOptions","scaleFactor",1.1);
	m_options.minNeighbors			= config.read_int("DetectionOptions","minNeighbors",3);
	m_options.flags					= config.read_int("DetectionOptions","flags",0);
	m_options.minSize				= config.read_int("DetectionOptions","minSize",30);

	m_cascade = new CascadeClassifier();

	CASCADE->load( m_options.cascadeFileName );

	if ( CASCADE->empty() )
		throw  std::runtime_error("Incorrect cascade file.");
}


void CCascadeClassifierDetection::detectObjects(CObservation *obs, vector_detectable_object &detected)
{
#if MRPT_HAS_OPENCV
	
	mrpt::utils::CImage *img = NULL;

	if (IS_CLASS(obs,CObservationImage))
	{
		CObservationImage* o = static_cast<CObservationImage*>(obs);
		img = &o->image;
	}
	else if ( IS_CLASS(obs,CObservationStereoImages) )
	{
		CObservationStereoImages* o = static_cast<CObservationStereoImages*>(obs);
		img = &o->imageLeft;
	}
	if (!img)
	{
	    mrpt::system::sleep(2);
	    return;	
	}

	detectObjects(img, detected);

#else
	THROW_EXCEPTION("This method requires MRPT built against OpenCV")
#endif
}

void CCascadeClassifierDetection::detectObjects(CImage *img, vector_detectable_object &detected)
{
	vector<Rect> objects;

	img->grayscaleInPlace();

	// Convert to IplImage and copy it
	IplImage *image = static_cast<IplImage*>(img->getAsIplImage());

	CASCADE->detectMultiScale( cv::cvarrToMat(image), objects);

	unsigned int N = objects.size();
	//detected.resize( N );

	for ( unsigned int i = 0; i < N; i++ )
	{
		CDetectable2DPtr obj = CDetectable2DPtr( new CDetectable2D( objects[i].x, objects[i].y, objects[i].height, objects[i].width ) );
		detected.push_back((CDetectableObjectPtr)obj);
	}
}