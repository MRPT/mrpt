/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "containers-precomp.h"	 // Precompiled headers
//
#include <mrpt/containers/ts_hash_map.h>
#include <mrpt/core/byte_manip.h>  // MAKEWORD16B(), etc.

#include <cstdlib>
#include <cstring>

void mrpt::containers::reduced_hash(
	const std::string_view& value, uint64_t& out_hash)
{
	// dbj2 method:
	uint64_t hash = 5381;
	for (auto c : value)
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
	out_hash = hash;
}
void mrpt::containers::reduced_hash(
	const std::string_view& value, uint8_t& out_hash)
{
	uint64_t hash;
	reduced_hash(value, hash);
	out_hash =
		((SELBYTE0(hash) ^ SELBYTE1(hash)) ^ SELBYTE2(hash)) ^ SELBYTE3(hash);
}
void mrpt::containers::reduced_hash(
	const std::string_view& value, uint16_t& out_hash)
{
	uint64_t hash;
	reduced_hash(value, hash);
	out_hash = MAKEWORD16B(
		SELBYTE0(hash) ^ SELBYTE1(hash), SELBYTE2(hash) ^ SELBYTE3(hash));
}

void mrpt::containers::reduced_hash(
	const std::string_view& value, uint32_t& out_hash)
{
	uint64_t hash;
	reduced_hash(value, hash);
	out_hash = (hash & 0xffffffff) ^ ((hash >> 32) & 0xffffffff);
}
