/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/bits.h>
#include <mrpt/utils/mrpt_stdint.h>
#include <cstring>

template <typename T>
void reverseBytesInPlace_2b(T& v_in_out)
{
	const uint16_t org = *reinterpret_cast<uint16_t*>(&v_in_out);
	const uint16_t val_rev = ((org & 0xff00) >> 8) | ((org & 0x00ff) << 8);
	::memcpy(&v_in_out, &val_rev, sizeof(T)); // This avoid deref. puned pointer warning with:   *reinterpret_cast<const T*>(&val_rev);
}

template <typename T>
void reverseBytesInPlace_4b(T& v_in_out)
{
	const uint32_t org = *reinterpret_cast<uint32_t*>(&v_in_out);
	const uint32_t val_rev = ((org & 0xff000000) >> (3*8)) | ((org & 0x00ff0000) >> (1*8)) | ((org & 0x0000ff00) << (1*8)) | ((org & 0x000000ff) << (3*8));
	::memcpy(&v_in_out, &val_rev, sizeof(T)); // This avoid deref. puned pointer warning with:   *reinterpret_cast<const T*>(&val_rev);
}

template <typename T>
void reverseBytesInPlace_8b(T& v_in_out)
{
	const uint64_t org = *reinterpret_cast<uint64_t*>(&v_in_out);
	const uint64_t val_rev =
		((org & ( UINT64_C(0xff) << (7*8))) >> (7*8)) |
		((org & ( UINT64_C(0xff) << (6*8))) >> (5*8)) |
		((org & ( UINT64_C(0xff) << (5*8))) >> (3*8)) |
		((org & ( UINT64_C(0xff) << (4*8))) >> (1*8)) |
		((org & ( UINT64_C(0xff) << (3*8))) << (1*8)) |
		((org & ( UINT64_C(0xff) << (2*8))) << (3*8)) |
		((org & ( UINT64_C(0xff) << (1*8))) << (5*8)) |
		((org & ( UINT64_C(0xff) << (0*8))) << (7*8));
	::memcpy(&v_in_out, &val_rev, sizeof(T)); // This avoid deref. puned pointer warning with:   *reinterpret_cast<const T*>(&val_rev);
}

void mrpt::utils::reverseBytesInPlace(bool& ) {
	// Nothing to do.
}

void mrpt::utils::reverseBytesInPlace(uint8_t& v_in_out) {
	// Nothing to do.
}
void mrpt::utils::reverseBytesInPlace(int8_t& v_in_out) {
	// Nothing to do.
}

void mrpt::utils::reverseBytesInPlace(uint16_t& v_in_out) {
	reverseBytesInPlace_2b(v_in_out);
}

void mrpt::utils::reverseBytesInPlace(int16_t& v_in_out) {
	reverseBytesInPlace_2b(v_in_out);
}

void mrpt::utils::reverseBytesInPlace(uint32_t& v_in_out) {
	reverseBytesInPlace_4b(v_in_out);
}

void mrpt::utils::reverseBytesInPlace(int32_t& v_in_out) {
	reverseBytesInPlace_4b(v_in_out);
}

void mrpt::utils::reverseBytesInPlace(uint64_t& v_in_out) {
	reverseBytesInPlace_8b(v_in_out);
}

void mrpt::utils::reverseBytesInPlace(int64_t& v_in_out)  {
	reverseBytesInPlace_8b(v_in_out);
}

void mrpt::utils::reverseBytesInPlace(float& v_in_out)  {
	reverseBytesInPlace_4b(v_in_out);
}

void mrpt::utils::reverseBytesInPlace(double& v_in_out) {
	reverseBytesInPlace_8b(v_in_out);
}

