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
