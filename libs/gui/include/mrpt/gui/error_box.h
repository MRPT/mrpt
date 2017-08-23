/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once
#include <mrpt/gui/link_pragmas.h>

#include <string>

namespace mrpt
{
namespace gui
{
void GUI_IMPEXP tryCatch(const std::function<void()>& tryPart, const std::string& catchMessage);
void GUI_IMPEXP showErrorMessage(const std::string& str);
}
}
