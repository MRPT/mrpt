/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CStereoGrabber_Bumblebee.h>


#if MRPT_HAS_BUMBLEBEE
	#include <PGRFlyCapture.h>
#endif

using namespace mrpt::hwdrivers;


//MRPT_TODO("Crear metodos open() y close() en vez del constructor!")

/*-------------------------------------------------------------
			TCaptureOptions_bumblebee Constructor
 -------------------------------------------------------------*/
TCaptureOptions_bumblebee::TCaptureOptions_bumblebee() :
				frame_width		(640),
				frame_height	(480),
				color			(false),
				getRectified	(true),
#if MRPT_HAS_BUMBLEBEE
				framerate		(FLYCAPTURE_FRAMERATE_15)
#else
				framerate		(15)
#endif
{
}

