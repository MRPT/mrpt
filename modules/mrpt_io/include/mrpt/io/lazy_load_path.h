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

namespace mrpt::io
{
/** Makes sure of building an absolute path for the given relative (or possibly
 * absolute) lazy-load object.
 *
 * \ingroup mrpt_io_grp
 */
std::string lazy_load_absolute_path(const std::string& relativeOrAbsolutePath);

/** Gets the current path to be used to locate relative lazy-load externally
 * stored objects via lazy_load_absolute_path(). Default is `"."`.
 *
 * \ingroup mrpt_io_grp
 */
const std::string& getLazyLoadPathBase();

/**  Changes the base path to be used to locate relative lazy-load externally
 * stored objects via lazy_load_absolute_path().
 *
 * \ingroup mrpt_io_grp
 */
void setLazyLoadPathBase(const std::string& path);

}  // namespace mrpt::io
