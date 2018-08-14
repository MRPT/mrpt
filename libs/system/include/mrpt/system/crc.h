/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>

namespace mrpt::system
{
/** \addtogroup mrpt_crc CRC functions
 * Header: `#include <mrpt/system/crc.h>`.
 * Library: \ref mrpt_system_grp
 *  \ingroup mrpt_system_grp
 * @{ */

/** Computes the CRC16 checksum of a block of data.  */
uint16_t compute_CRC16(
	const std::vector<uint8_t>& data, const uint16_t gen_pol = 0x8005);
/** \overload */
uint16_t compute_CRC16(
	const uint8_t* data, const size_t len, const uint16_t gen_pol = 0x8005);
/** Computes the CRC32 checksum of a block of data.  */
uint32_t compute_CRC32(
	const std::vector<uint8_t>& data, const uint32_t gen_pol = 0xEDB88320L);
/** \overload */
uint32_t compute_CRC32(
	const uint8_t* data, const size_t len,
	const uint32_t gen_pol = 0xEDB88320L);
/** @} */
}  // namespace mrpt::system
