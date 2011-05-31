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

#include <mrpt/hwdrivers.h>  // Only for precomp. headers, include all libmrpt-core headers.

#include <mrpt/hwdrivers/CStereoGrabber_Bumblebee.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::hwdrivers;

/*-------------------------------------------------------------
					Constructor
 -------------------------------------------------------------*/
CStereoGrabber_Bumblebee::CStereoGrabber_Bumblebee(
	int								cameraIndex,
	const TCaptureOptions_bumblebee &options ) :
		m_firewire_capture(NULL),
		m_bInitialized(false),
		m_resolutionX( options.frame_width ),
		m_resolutionY( options.frame_height ),
		m_baseline( 0 ),
		m_focalLength( 0 ),
		m_centerCol( 0 ),
		m_centerRow( 0 ),
		m_options( options )
{
    MRPT_TRY_START;

	TCaptureOptions_dc1394	opt1394;
	opt1394.mode7 				= 3; // stereo cameras are captured with MODE7-3
	opt1394.deinterlace_stereo	= true; // It is stereo.

	std::map<double,grabber_dc1394_framerate_t> Rs;
	Rs[1.875] = FRAMERATE_1_875;
	Rs[3.75] = FRAMERATE_3_75;
	Rs[7.5] = FRAMERATE_7_5;
	Rs[15] = FRAMERATE_15;
	Rs[30] = FRAMERATE_30;
	Rs[60] = FRAMERATE_60;
	Rs[120] = FRAMERATE_120;
	Rs[240] = FRAMERATE_240;

	if (Rs.find(options.framerate)!=Rs.end())
			opt1394.framerate = Rs[options.framerate];

	// TODO: Select a PGR Bumblebee camera, and the given index if there are many...
	m_firewire_capture = new CImageGrabber_dc1394(0,0,opt1394);

	if (!m_firewire_capture->isOpen())
		cerr << "[CStereoGrabber_Bumblebee] The camera couldn't be open" << endl;

	MRPT_TRY_END;
}

/*-------------------------------------------------------------
					Destructor
 -------------------------------------------------------------*/
CStereoGrabber_Bumblebee::~CStereoGrabber_Bumblebee()
{
	if (m_firewire_capture)
	{
		delete m_firewire_capture;
		m_firewire_capture = NULL;
	}
}

/*-------------------------------------------------------------
					get the image
 -------------------------------------------------------------*/
bool  CStereoGrabber_Bumblebee::getStereoObservation( mrpt::slam::CObservationStereoImages &out_observation )
{
	if (!m_firewire_capture->isOpen())
	{
		cerr << "[CStereoGrabber_Bumblebee] The camera couldn't be open" << endl;
		return false;
	}

	if (!m_firewire_capture->getObservation(out_observation))
		return false;

	// Change resolution?
	if (m_resolutionX>0 && m_resolutionX!=out_observation.imageLeft.getWidth())
	{
		out_observation.imageLeft.scaleImage(m_resolutionX,m_resolutionY, IMG_INTERP_NN);
		out_observation.imageRight.scaleImage(m_resolutionX,m_resolutionY, IMG_INTERP_NN);
	}

	// TODO: Fill the intrinsic matrix, etc...


	return true; // All ok
}
