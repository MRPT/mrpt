/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/crc.h>

using namespace mrpt::utils;
using namespace std;


/*---------------------------------------------------------------
					CRC16
 ---------------------------------------------------------------*/
uint16_t mrpt::utils::compute_CRC16( const std::vector<uint8_t> &data, const uint16_t gen_pol ) 
{
	return compute_CRC16(&data[0],data.size(),gen_pol);
}

uint32_t mrpt::utils::compute_CRC32( const std::vector<uint8_t> &data, const uint32_t gen_pol)
{
	return compute_CRC32(&data[0],data.size(),gen_pol);
}

/*---------------------------------------------------------------
					CRC16
 ---------------------------------------------------------------*/
uint16_t mrpt::utils::compute_CRC16( 
	const uint8_t *data, 
	const size_t  len_,
	const uint16_t gen_pol)
{
	uint16_t uCrc16;
	uint8_t  abData[2];
	
	size_t  len = len_;

	uCrc16 = 0;
	abData[0] = 0;

	while(len-- )
	{
		abData[1] = abData[0];
		abData[0] = *data++;

		if( uCrc16 & 0x8000 )
		{
			uCrc16 = (uCrc16 & 0x7fff) << 1;
			uCrc16 ^= gen_pol;
		}
		else
		{
			uCrc16 <<= 1;
		}
		uCrc16 ^= (abData[0] | (abData[1]<<8));
	}
	return uCrc16;
}

unsigned long CRC32Value(int i, const uint32_t CRC32_POLYNOMIAL)
{
	unsigned long ulCRC = i;
	for ( int j = 8 ; j > 0; j-- ) {
		if ( ulCRC & 1 )
			ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
		else ulCRC >>= 1;
	}
	return ulCRC;
}

uint32_t mrpt::utils::compute_CRC32(const uint8_t *data, const size_t  len_, const uint32_t gen_pol )
{
	size_t  len = len_;
	unsigned long ulCRC = 0;
	while ( len-- != 0 ) {
		unsigned long ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
		unsigned long ulTemp2 = CRC32Value( ((int) ulCRC ^ *data++ ) & 0xff, gen_pol );
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return ulCRC ;
}

