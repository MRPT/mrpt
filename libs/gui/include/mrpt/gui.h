/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _mrpt_gui_H
#define _mrpt_gui_H

// Only really include all headers if we come from a user program (anything
//  not defining mrpt_*_EXPORTS) or MRPT is being built with precompiled headers.
#if !defined(mrpt_gui_EXPORTS) || MRPT_ENABLE_PRECOMPILED_HDRS || defined(MRPT_ALWAYS_INCLUDE_ALL_HEADERS)

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindowPlots.h>

// Include these ones manually in your code only if you need them, since they require wxWidget headers:
// #include "gui/WxSubsystem.h" 
//#include "gui/WxUtils.h"
//#include "gui/CMyGLCanvasBase.h"

#endif // end precomp.headers

#endif
