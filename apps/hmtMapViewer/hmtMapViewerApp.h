/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef HMTMAPVIEWERAPP_H
#define HMTMAPVIEWERAPP_H

#include <wx/app.h>

class hmtMapViewerApp : public wxApp
{
    public:
        virtual bool OnInit();
        virtual int  OnExit();
};

#endif // HMTMAPVIEWERAPP_H
