/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef CAMERA_CALIB_GUIAPP_H
#define CAMERA_CALIB_GUIAPP_H

#include <wx/app.h>

class camera_calib_guiApp : public wxApp
{
   public:
	bool OnInit() override;
	// virtual bool Initialize( int& argc, wxChar **argv );
};

#endif	// CAMERA_CALIB_GUIAPP_H
