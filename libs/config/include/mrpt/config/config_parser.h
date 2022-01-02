/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>

namespace mrpt::config
{
/** Parses a document and replaces all formulas, variables, etc. as defined
 * in \ref config_file_format
 *
 * \ingroup mrpt_config_grp
 */
std::string config_parser(const std::string& input);

}  // namespace mrpt::config