/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
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
