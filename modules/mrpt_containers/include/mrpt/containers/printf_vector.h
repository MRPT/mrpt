/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/core/format.h>

#include <cstdio>
#include <string>
#include <vector>

namespace mrpt::containers
{
/** \addtogroup mrpt_containers_stlext_grp
 *  \ingroup mrpt_containers_grp
 * @{ */

/** Generates a string for a vector in the format [A,B,C,...] to std::cout, and
 * the fmt string for <b>each</b> vector element. */
template <typename VEC>
std::string sprintf_vector(const char* fmt, const VEC& V)
{
  std::string ret = "[";
  const size_t N = V.size();
  for (size_t i = 0; i < N; i++)
  {
    ret += format(fmt, V[i]);
    if (i != (N - 1)) ret += ",";
  }
  ret += "]";
  return ret;
}

/** Prints a vector in the format [A,B,C,...] to std::cout, and the fmt string
 * for <b>each</b> vector element. */
template <typename T>
void printf_vector(const char* fmt, const std::vector<T>& V)
{
  ::fputs(sprintf_vector(fmt, V).c_str(), stdout);
}

/** @} */  // end of grouping
}  // namespace mrpt::containers
