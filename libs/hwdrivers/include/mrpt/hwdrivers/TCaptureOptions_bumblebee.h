/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::hwdrivers
{
/** Options used when creating a bumblebee camera capture object
 * \sa CStereoGrabber_Bumblebee, CStereoGrabber_Bumblebee_dc1394
 * \ingroup mrpt_hwdrivers_grp
 */
struct TCaptureOptions_bumblebee
{
	TCaptureOptions_bumblebee();

	/** Capture resolution (Default: 640x480) */
	int frame_width, frame_height;
	/** Indicates if the Bumblebee camera must capture color images (Default:
	 * false -> grayscale) */
	bool color;
	/** Indicates if the Bumblebee camera must capture rectified images
	 * (Default: true -> rectified) */
	bool getRectified;
	/** Bumblebee camera frame rate (Default: 15 fps) */
	double framerate;
};
}  // namespace mrpt::hwdrivers
