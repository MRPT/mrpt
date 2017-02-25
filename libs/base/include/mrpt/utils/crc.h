/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_UTILS_CRC_H
#define  MRPT_UTILS_CRC_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup mrpt_crc CRC functions (in #include <mrpt/utils/crc.h>)
		  *  \ingroup mrpt_base_grp
		  * @{ */

		/** Computes the CRC16 checksum of a block of data.  */
		uint16_t BASE_IMPEXP compute_CRC16( const std::vector<uint8_t> &data, const uint16_t gen_pol = 0x8005 );
		uint16_t BASE_IMPEXP compute_CRC16(const uint8_t *data, const size_t  len, const uint16_t gen_pol = 0x8005 ); //!< \overload
		/** Computes the CRC32 checksum of a block of data.  */
		uint32_t BASE_IMPEXP compute_CRC32( const std::vector<uint8_t> &data, const uint32_t gen_pol = 0xEDB88320L );
		uint32_t BASE_IMPEXP compute_CRC32(const uint8_t *data, const size_t  len, const uint32_t gen_pol = 0xEDB88320L ); //!< \overload
		/** @} */
	}
}

#endif
