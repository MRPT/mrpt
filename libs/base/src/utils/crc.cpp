/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
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

