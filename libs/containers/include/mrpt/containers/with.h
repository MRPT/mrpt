/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <type_traits>
#include <array>
#include <functional>  // reference_wrapper

namespace mrpt
{
namespace containers
{
/** \addtogroup mrpt_containers_grp
 * @{ */
namespace internal
{
template <class T1, class... T>
struct first
{
	using type = T1;
};

template <class T, size_t N>
struct with_impl
{
	std::array<std::reference_wrapper<T>, N> m_lst;

	template <typename... Args>
	with_impl(Args... lstObjects) : m_lst{lstObjects...}
	{
	}

	template <typename Callable>
	void call(const Callable& func)
	{
		for (auto& o : m_lst) func(o.get());
	}
};
}  // namespace internal

/** Returns a wrapper on a set of objects (or smart pointers)
 *  on which one can apply any lambda: `with(a,b,c).call(...);`
 */
template <typename... Args>
internal::with_impl<typename internal::first<Args...>::type, sizeof...(Args)>
	with(Args... lstObjects)
{
	return internal::with_impl<
		typename internal::first<Args...>::type, sizeof...(Args)>(
		lstObjects...);
}

/** @} */

}  // namespace containers
}  // namespace mrpt
