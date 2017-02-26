/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef NAVLOG_VIEWER_GUI_DESIGNAPP_H
#define NAVLOG_VIEWER_GUI_DESIGNAPP_H

#include <wx/app.h>

class navlog_viewer_GUI_designApp : public wxApp
{
    public:
        virtual bool OnInit();
		virtual int OnRun();
};

#endif // NAVLOG_VIEWER_GUI_DESIGNAPP_H
