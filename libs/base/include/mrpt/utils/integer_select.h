/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/utils/mrpt_stdint.h>

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup mrpt_int_select Templates to declare integers by byte count (in #include <mrpt/utils/integer_select.h>)
		  *  \ingroup mrpt_base_grp
		  * @{ */

		/** Usage: `int_select_by_bytecount<N>::type  var;` allows defining var as a signed integer with, at least, N bytes. */
		template<unsigned int num_bytes> struct int_select_by_bytecount;
		template<> struct int_select_by_bytecount<1> { typedef int8_t type; };
		template<> struct int_select_by_bytecount<2> { typedef int16_t type; };
		template<> struct int_select_by_bytecount<3> { typedef int32_t type; };
		template<> struct int_select_by_bytecount<4> { typedef int32_t type; };
		template<> struct int_select_by_bytecount<8> { typedef int64_t type; };

		/** Usage: `uint_select_by_bytecount<N>::type  var;` allows defining var as a unsigned integer with, at least, N bytes. */
		template<unsigned int num_bytes> struct uint_select_by_bytecount;
		template<> struct uint_select_by_bytecount<1> { typedef uint8_t type; };
		template<> struct uint_select_by_bytecount<2> { typedef uint16_t type; };
		template<> struct uint_select_by_bytecount<3> { typedef uint32_t type; };
		template<> struct uint_select_by_bytecount<4> { typedef uint32_t type; };
		template<> struct uint_select_by_bytecount<8> { typedef uint64_t type; };

		/** @} */
	} // End of namespace
} // end of namespace
