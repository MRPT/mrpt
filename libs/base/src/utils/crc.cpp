/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers


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

