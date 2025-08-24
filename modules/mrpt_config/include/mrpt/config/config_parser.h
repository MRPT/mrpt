/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
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