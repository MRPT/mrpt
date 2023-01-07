/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <cstdint>

namespace mrpt
{
/** \addtogroup mrpt_int_select Templates to declare integers by byte count (in
 * #include <mrpt/core/integer_select.h>)
 *  \ingroup mrpt_core_grp
 * @{ */

/** Usage: `int_select_by_bytecount<N>::type  var;` allows defining var as a
 * signed integer with, at least, N bytes. */
template <unsigned int num_bytes>
struct int_select_by_bytecount;
template <>
struct int_select_by_bytecount<1>
{
	using type = int8_t;
};
template <>
struct int_select_by_bytecount<2>
{
	using type = int16_t;
};
template <>
struct int_select_by_bytecount<3>
{
	using type = int32_t;
};
template <>
struct int_select_by_bytecount<4>
{
	using type = int32_t;
};
template <>
struct int_select_by_bytecount<8>
{
	using type = int64_t;
};

/** Usage: `int_select_by_bytecount_t<N> var;`.
 * C++14 helper type for int_select_by_bytecount<> */
template <unsigned int num_bytes>
using int_select_by_bytecount_t =
	typename int_select_by_bytecount<num_bytes>::type;

/** Usage: `uint_select_by_bytecount<N>::type  var;` allows defining var as a
 * unsigned integer with, at least, N bytes. */
template <unsigned int num_bytes>
struct uint_select_by_bytecount;
template <>
struct uint_select_by_bytecount<1>
{
	using type = uint8_t;
};
template <>
struct uint_select_by_bytecount<2>
{
	using type = uint16_t;
};
template <>
struct uint_select_by_bytecount<3>
{
	using type = uint32_t;
};
template <>
struct uint_select_by_bytecount<4>
{
	using type = uint32_t;
};
template <>
struct uint_select_by_bytecount<8>
{
	using type = uint64_t;
};

/** Usage: `uint_select_by_bytecount_t<N> var;`.
 * C++14 helper type for uint_select_by_bytecount<> */
template <unsigned int num_bytes>
using uint_select_by_bytecount_t =
	typename uint_select_by_bytecount<num_bytes>::type;

/** @} */
}  // namespace mrpt
