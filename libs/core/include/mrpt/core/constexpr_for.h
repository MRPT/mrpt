/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <utility>  // make_index_sequence

namespace mrpt
{
namespace internal
{
template <std::size_t N>
struct num
{
	static const constexpr auto value = N;
};
template <class F, std::size_t... Is>
void for_(F func, std::index_sequence<Is...>)
{
	(func(num<Is>{}), ...);
}
}  // namespace internal

/** constexpr for loop. Example:
 * \code
 * mrpt::for_<10>( [&](auto i) { std::cout << i.value << " "; } );
 * \endcode
 * \ingroup mrpt_core_grp
 * \note (New in MRPT 2.1.0)
 * \note Credits to [SO](https://stackoverflow.com/a/26912970/1631514).
 */
template <std::size_t N, typename F>
void for_(F func)
{
	internal::for_(func, std::make_index_sequence<N>());
}
}  // namespace mrpt