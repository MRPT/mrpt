/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string_view>

namespace mrpt::containers
{
/** Storage for value-comment pairs \ingroup mrpt_containers_yaml
 * \note [New in MRPT 2.1.0]
 */
template <typename T>
struct ValueCommentPair
{
	ValueCommentPair(const T& v, const std::string_view& c)
		: value(v), comment(c)
	{
	}
	const T& value;
	const std::string_view& comment;
};

/** Helper syntax sugar for ValueCommentPair
 * \ingroup mrpt_containers_yaml
 * \note [New in MRPT 2.1.0]*/
template <typename T>
struct ValueCommentPair<T> vcp(const T& v, const std::string_view& c)
{
	return {v, c};
}

}  // namespace mrpt::containers