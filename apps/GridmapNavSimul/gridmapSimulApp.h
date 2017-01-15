/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef gridmapSimulAPP_H
#define gridmapSimulAPP_H

#include <wx/app.h>

class gridmapSimulApp : public wxApp
{
    public:
        virtual bool OnInit();
        virtual int  OnExit();
};

#endif // gridmapSimulAPP_H
