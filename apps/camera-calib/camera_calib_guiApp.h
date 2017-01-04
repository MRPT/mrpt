/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CAMERA_CALIB_GUIAPP_H
#define CAMERA_CALIB_GUIAPP_H

#include <wx/app.h>

class camera_calib_guiApp : public wxApp
{
    public:
	virtual bool OnInit();
	//virtual bool Initialize( int& argc, wxChar **argv );
};

#endif // CAMERA_CALIB_GUIAPP_H
