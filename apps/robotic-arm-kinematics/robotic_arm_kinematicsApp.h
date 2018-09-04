/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef robotic_arm_KINEMATICSAPP_H
#define robotic_arm_KINEMATICSAPP_H

#include <wx/app.h>

class robotic_arm_kinematicsApp : public wxApp
{
   public:
	bool OnInit() override;
};

#endif  // robotic_arm_KINEMATICSAPP_H
