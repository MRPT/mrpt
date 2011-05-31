/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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

