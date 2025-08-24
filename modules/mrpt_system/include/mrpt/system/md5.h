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

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mrpt::system
{
/** \addtogroup mrpt_md5 MD5 functions
 * Header: `#include <mrpt/system/md5.h>`.
 * Library: \ref mrpt_system_grp
 *  \ingroup mrpt_system_grp
 * @{ */
/** Computes the md5 of a block of data. */
std::string md5(const std::string& str);
/** Computes the md5 of a block of data. */
std::string md5(const std::vector<uint8_t>& str);
/** Computes the md5 of a block of data. */
std::string md5(const unsigned char* data, size_t len);
/** @} */
}  // namespace mrpt::system
