/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*
  App      : kinect-calibrate
  Web page : https://www.mrpt.org/Kinect_and_MRPT

  Usage    : Run and follow on-screen instructions
*/

#ifndef KINECT_CALIBRATE_GUIAPP_H
#define KINECT_CALIBRATE_GUIAPP_H

#include <wx/app.h>

class kinect_calibrate_guiApp : public wxApp
{
   public:
	bool OnInit() override;
};

#endif	// KINECT_CALIBRATE_GUIAPP_H
