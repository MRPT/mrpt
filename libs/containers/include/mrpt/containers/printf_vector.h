/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/format.h>
#include <mrpt/math/eigen_frwds.h>
#include <cstdio>
#include <string>
#include <vector>

namespace mrpt::containers
{
/** \addtogroup
 * @{ */

/** Generates a string for a vector in the format [A,B,C,...] to std::cout, and
 * the fmt string for <b>each</b> vector element. */
template <typename T>
std::string sprintf_vector(const char* fmt, const std::vector<T>& V)
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
/// @overload
template <typename Derived>
std::string sprintf_vector(const char* fmt, const Eigen::MatrixBase<Derived>& V)
{
	std::string ret = "[";
	const size_t N = V.size();
	for (size_t i = 0; i < N; i++)
	{
		ret += mrpt::format(fmt, V[i]);
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
