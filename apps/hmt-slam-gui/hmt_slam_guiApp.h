/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef HMT_SLAM_GUIAPP_H
#define HMT_SLAM_GUIAPP_H

#include <wx/app.h>

class hmt_slam_guiApp : public wxApp
{
   public:
	bool OnInit() override;
};

#endif	// HMT_SLAM_GUIAPP_H
