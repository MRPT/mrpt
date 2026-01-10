/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef CAMERA_CALIB_GUIAPP_H
#define CAMERA_CALIB_GUIAPP_H

#include <wx/app.h>

class camera_calib_guiApp : public wxApp
{
 public:
  bool OnInit() override;
  // virtual bool Initialize( int& argc, wxChar **argv );
};

#endif  // CAMERA_CALIB_GUIAPP_H
