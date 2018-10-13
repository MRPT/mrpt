/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/typemeta/TTypeName.h>
#include <mrpt/typemeta/num_to_string.h>
#include <mrpt/typemeta/static_string.h>
#include <chrono>
#include <list>
#include <vector>
#include <deque>
#include <set>
#include <map>
#include <array>

/** \file This file extends TTypeName.h for STL C++ types. */

namespace mrpt::typemeta
{
/** @name Conversion of type to string at compile time
@{ */

MRPT_DECLARE_TTYPENAME(std::string)

#define MRPT_DECLARE_TTYPENAME_CONTAINER(_CONTAINER)                           \
	template <typename V>                                                      \
	struct TTypeName<_CONTAINER<V>>                                            \
	{                                                                          \
		constexpr static auto get()                                            \
		{                                                                      \
			return literal(#_CONTAINER) + literal("<") + TTypeName<V>::get() + \
				   literal(">");                                               \
		}                                                                      \
	};

MRPT_DECLARE_TTYPENAME_CONTAINER(std::vector)
MRPT_DECLARE_TTYPENAME_CONTAINER(std::deque)
MRPT_DECLARE_TTYPENAME_CONTAINER(std::list)
MRPT_DECLARE_TTYPENAME_CONTAINER(std::set)

// array<T,N>
#define MRPT_DECLARE_TTYPENAME_CONTAINER_FIX_SIZE(_CONTAINER)                  \
	template <typename V, std::size_t N>                                       \
	struct TTypeName<_CONTAINER<V, N>>                                         \
	{                                                                          \
		constexpr static auto get()                                            \
		{                                                                      \
			return literal(#_CONTAINER) + literal("<") + TTypeName<V>::get() + \
				   literal(",") + literal(num_to_string<N>::value) +           \
				   literal(">");                                               \
		}                                                                      \
	};

MRPT_DECLARE_TTYPENAME_CONTAINER_FIX_SIZE(std::array)

#define MRPT_DECLARE_TTYPENAME_CONTAINER_ASSOC(_CONTAINER)                     \
	template <typename K, typename V>                                          \
	struct TTypeName<_CONTAINER<K, V>>                                         \
	{                                                                          \
		constexpr static auto get()                                            \
		{                                                                      \
			return literal(#_CONTAINER) + literal("<") + TTypeName<K>::get() + \
				   literal(",") + TTypeName<V>::get() + literal(">");          \
		}                                                                      \
	};

MRPT_DECLARE_TTYPENAME_CONTAINER_ASSOC(std::map)
MRPT_DECLARE_TTYPENAME_CONTAINER_ASSOC(std::multimap)

template <typename T1, typename T2>
struct TTypeName<std::pair<T1, T2>>
{
	constexpr static auto get()
	{
		return literal("std::pair<") + TTypeName<T1>::get() + literal(",") +
			   TTypeName<T2>::get() + literal(">");
	}
};

template <typename T>
struct TTypeName<std::chrono::time_point<T>>
{
	constexpr static auto get()
	{
		// uint64_t for backwards compatibility
		return literal("uint64_t");
	}
};

/** @} */

}  // namespace mrpt::typemeta
