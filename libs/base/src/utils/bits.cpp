/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/bits.h>
#include <cstring>
#include <cstdlib>

// These #defines from: https://github.com/boostorg/endian/blob/master/include/boost/endian/detail/intrinsic.hpp
#ifndef __has_builtin
  #define __has_builtin(x) 0  // Compatibility with non-clang compilers
#endif
#if (defined(__clang__) && __has_builtin(__builtin_bswap32) && __has_builtin(__builtin_bswap64)) \
  || (defined(__GNUC__ ) && \
  (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 3)))
	#define HAVE_BSWAP_INTRINSICS
#endif



template <typename T>
void reverseBytesInPlace_2b(T& v_in_out)
{
	uint16_t org;
	::memcpy(&org, &v_in_out, sizeof(T));  // Was: = *reinterpret_cast<uint16_t*>(&v_in_out); but caused SIGBUS in some archs
	const uint16_t val_rev = ((org & 0xff00) >> 8) | ((org & 0x00ff) << 8);
	::memcpy(&v_in_out, &val_rev, sizeof(T)); // This avoid deref. puned pointer warning with:   *reinterpret_cast<const T*>(&val_rev);
}

template <typename T>
void reverseBytesInPlace_4b(T& v_in_out)
{
	uint32_t org;
	::memcpy(&org, &v_in_out, sizeof(T));  // Was: = = *reinterpret_cast<uint32_t*>(&v_in_out); but caused SIGBUS in some archs
	const uint32_t val_rev =
#if defined(_MSC_VER)
	_byteswap_ulong(org);
#elif defined(HAVE_BSWAP_INTRINSICS)
	__builtin_bswap32(org);
#else
		((org & 0xff000000) >> (3*8)) | ((org & 0x00ff0000) >> (1*8)) | ((org & 0x0000ff00) << (1*8)) | ((org & 0x000000ff) << (3*8));
#endif
	::memcpy(&v_in_out, &val_rev, sizeof(T)); // This avoid deref. puned pointer warning with:   *reinterpret_cast<const T*>(&val_rev);
}

template <typename T>
void reverseBytesInPlace_8b(T& v_in_out)
{
	uint64_t org;
	::memcpy(&org, &v_in_out, sizeof(T));  // Was: = *reinterpret_cast<uint64_t*>(&v_in_out); but caused SIGBUS in some archs
#if defined(_MSC_VER)
	uint64_t val_rev =_byteswap_uint64(org);
#elif defined(HAVE_BSWAP_INTRINSICS)
	uint64_t val_rev =__builtin_bswap64(org);
#else
	uint64_t val_rev = 0;
	int i,j;
	for (i=7,j=7;i>=4;i--,j-=2)
		val_rev |= ((org & ( UINT64_C(0xff) << (i*8))) >> (j*8));
	for (i=3,j=1;i>=0;i--,j+=2)
		val_rev |= ((org & ( UINT64_C(0xff) << (i*8))) << (j*8));
#endif
	::memcpy(&v_in_out, &val_rev, sizeof(T)); // This avoid deref. puned pointer warning with:   *reinterpret_cast<const T*>(&val_rev);
}

void mrpt::utils::reverseBytesInPlace(bool& ) {
	// Nothing to do.
}

void mrpt::utils::reverseBytesInPlace(uint8_t& /*v_in_out*/) {
	// Nothing to do.
}
void mrpt::utils::reverseBytesInPlace(int8_t& /*v_in_out*/) {
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

#ifdef HAVE_LONG_DOUBLE
void mrpt::utils::reverseBytesInPlace(long double& v_in_out) {
	uint64_t dat[2];
	::memcpy(&dat[0], &v_in_out, sizeof(long double)); // This avoid deref. puned pointer warning with:   *reinterpret_cast<const T*>(&val_rev);
	std::swap(dat[0],dat[1]);
	reverseBytesInPlace(dat[0]);
	reverseBytesInPlace(dat[1]);
	::memcpy(&v_in_out, &dat[0], sizeof(long double));
}
#endif

