/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdint>

namespace mrpt::containers
{
/** Defines possible positions for a comment in a document (INI file, YAML).
 * Valid positions are: Top (normally the default), right, and bottom.
 *
 * \ingroup mrpt_containers_yaml
 * \note [New in MRPT 2.1.0]
 */
enum class CommentPosition : uint8_t
{
	TOP = 0,
	RIGHT,
	BOTTOM,
	//
	MAX
};

}  // namespace mrpt::containers
