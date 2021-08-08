/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdint>

namespace mrpt::io
{
/** Flags used when opening a file for writing.
 *
 * \sa CFileGZOutputStream, CFileGZOutputStream
 * \ingroup mrpt_io_grp
 */
enum class OpenMode : uint8_t
{
	TRUNCATE = 0,
	APPEND
};

}  // namespace mrpt::io
