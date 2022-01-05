/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <functional>
#include <string>

namespace mrpt::gui
{
void tryCatch(
	const std::function<void()>& tryPart, const std::string& catchMessage);
void showErrorMessage(const std::string& str);
}  // namespace mrpt::gui
