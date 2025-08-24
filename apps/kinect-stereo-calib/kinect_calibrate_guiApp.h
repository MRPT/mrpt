/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

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

#endif  // KINECT_CALIBRATE_GUIAPP_H
