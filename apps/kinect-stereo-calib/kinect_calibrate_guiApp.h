/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*
  App      : kinect-calibrate
  Web page : http://www.mrpt.org/Kinect_and_MRPT

  Usage    : Run and follow on-screen instructions
*/

#ifndef KINECT_CALIBRATE_GUIAPP_H
#define KINECT_CALIBRATE_GUIAPP_H

#include <wx/app.h>

class kinect_calibrate_guiApp : public wxApp
{
    public:
        virtual bool OnInit();
};

#endif // KINECT_CALIBRATE_GUIAPP_H
