/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config.h>
#include <mrpt/core/Clock.h>

#include <cstdint>

namespace mrpt
{
/** \addtogroup mrpt_core_grp
 * @{ */

/** Reverse the order of the bytes of a given type (useful for transforming btw
 * little/big endian)  */
void reverseBytesInPlace(bool& v_in_out);
void reverseBytesInPlace(uint8_t& v_in_out);
void reverseBytesInPlace(int8_t& v_in_out);
void reverseBytesInPlace(uint16_t& v_in_out);
void reverseBytesInPlace(int16_t& v_in_out);
void reverseBytesInPlace(uint32_t& v_in_out);
void reverseBytesInPlace(int32_t& v_in_out);
void reverseBytesInPlace(uint64_t& v_in_out);
void reverseBytesInPlace(int64_t& v_in_out);
void reverseBytesInPlace(float& v_in_out);
void reverseBytesInPlace(double& v_in_out);
void reverseBytesInPlace(std::chrono::time_point<mrpt::Clock>& v_in_out);

/** Reverse the order of the bytes of a given type (useful for transforming btw
 * little/big endian)  */
template <class T>
inline void reverseBytes(const T& v_in, T& v_out)
{
	v_out = v_in;
	reverseBytesInPlace(v_out);
}

template <class T>
inline T reverseBytes(const T& v_in)
{
	T v_out = v_in;
	reverseBytesInPlace(v_out);
	return v_out;
}

template <class T>
inline T toNativeEndianness(const T& v_in)
{
#if MRPT_IS_BIG_ENDIAN
	T v_out = v_in;
	reverseBytesInPlace(v_out);
	return v_out;
#else
	return v_in;
#endif
}

template <
	typename enum_t,
	typename underlying_t = typename std::underlying_type<enum_t>::type>
inline void reverseBytesInPlace_enum(enum_t& v)
{
	underlying_t v_out = static_cast<underlying_t>(v);
	reverseBytesInPlace(v_out);
	v = static_cast<enum_t>(v_out);
}

/** @} */
}  // namespace mrpt
