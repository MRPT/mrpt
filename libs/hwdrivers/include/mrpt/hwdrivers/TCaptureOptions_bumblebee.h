/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/hwdrivers/link_pragmas.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** Options used when creating a bumblebee camera capture object
		  * \sa CStereoGrabber_Bumblebee, CStereoGrabber_Bumblebee_dc1394
		  * \ingroup mrpt_hwdrivers_grp
		  */
		struct HWDRIVERS_IMPEXP TCaptureOptions_bumblebee
		{
			TCaptureOptions_bumblebee();

			int	frame_width, frame_height;	//!< Capture resolution (Default: 640x480)
			bool color;						//!< Indicates if the Bumblebee camera must capture color images (Default: false -> grayscale)
			bool getRectified;				//!< Indicates if the Bumblebee camera must capture rectified images (Default: true -> rectified)
			double framerate;				//!< Bumblebee camera frame rate (Default: 15 fps)
		};
	}
}
