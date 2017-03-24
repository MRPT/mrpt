/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Only for precomp. headers, include all libmrpt-core headers.

#include <mrpt/hwdrivers/CStereoGrabber_Bumblebee_libdc1394.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;

CStereoGrabber_Bumblebee_libdc1394::CStereoGrabber_Bumblebee_libdc1394(uint64_t cameraGUID, uint16_t cameraUnit, double   frameRate) :
	m_firewire_capture(NULL),
	m_bInitialized(false)
{
	MRPT_TRY_START

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

	if (Rs.find(frameRate)!=Rs.end())
			opt1394.framerate = Rs[frameRate];
	else cerr << "[CStereoGrabber_Bumblebee_libdc1394] Ignoring unknown framerate: " << frameRate << "\n";

	m_firewire_capture = new CImageGrabber_dc1394(cameraGUID,cameraUnit,opt1394);

	if (!m_firewire_capture->isOpen())
		cerr << "[CStereoGrabber_Bumblebee] The camera couldn't be open" << endl;

	MRPT_TRY_END
}

/*-------------------------------------------------------------
					Destructor
 -------------------------------------------------------------*/
CStereoGrabber_Bumblebee_libdc1394::~CStereoGrabber_Bumblebee_libdc1394()
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
bool  CStereoGrabber_Bumblebee_libdc1394::getStereoObservation( mrpt::obs::CObservationStereoImages &out_observation )
{
	if (!m_firewire_capture->isOpen())
	{
		cerr << "[CStereoGrabber_Bumblebee] The camera couldn't be open" << endl;
		return false;
	}

	if (!m_firewire_capture->getObservation(out_observation))
		return false;

	return true; // All ok
}
