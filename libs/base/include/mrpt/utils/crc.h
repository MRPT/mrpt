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
#ifndef  MRPT_UTILS_CRC_H
#define  MRPT_UTILS_CRC_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace utils
	{
		/** Computes the CRC16 checksum of a block of data.  */
		uint16_t BASE_IMPEXP compute_CRC16( 
			const std::vector<uint8_t> &data, 
			const uint16_t gen_pol = 0x8005 );

		/** Computes the CRC16 checksum of a block of data.  */
		uint16_t BASE_IMPEXP compute_CRC16( 
			const uint8_t *data, 
			const size_t  len,
			const uint16_t gen_pol = 0x8005 );


	}
}

#endif
