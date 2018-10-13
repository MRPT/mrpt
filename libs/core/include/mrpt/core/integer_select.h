/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
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

/** @} */
}  // namespace mrpt
