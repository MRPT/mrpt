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
void reverseBytesInPlace(long double& v_in_out);

/** Reverse the order of the bytes of a given type (useful for transforming btw
 * little/big endian)  */
template <class T>
inline void reverseBytes(const T& v_in, T& v_out)
{
	v_out = v_in;
	reverseBytesInPlace(v_out);
}
/** @} */
}  // namespace mrpt
