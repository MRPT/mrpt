/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
