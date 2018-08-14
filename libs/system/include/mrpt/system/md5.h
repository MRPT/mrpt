/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>
#include <vector>
#include <cstddef>

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
std::string md5(const unsigned char* data, const size_t len);
/** @} */
}  // namespace mrpt::system
