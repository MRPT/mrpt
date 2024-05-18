/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
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
