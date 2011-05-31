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


#include <mrpt/utils/CMemoryChunk.h>

using namespace mrpt::utils;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CMemoryChunk, CSerializable, mrpt::utils)

/*---------------------------------------------------------------
						writeToStream
 ---------------------------------------------------------------*/
void  CMemoryChunk::writeToStream(CStream &out, int *out_Version) const
{
	if (out_Version)
		*out_Version = 0;
	else
	{
		out << static_cast<uint64_t>(m_bytesWritten);
		if (m_bytesWritten)
		{
			ASSERT_(m_memory.get())
			out.WriteBuffer(m_memory.get(),m_bytesWritten);
		}
	}

}

/*---------------------------------------------------------------
						readFromStream
 ---------------------------------------------------------------*/
void  CMemoryChunk::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			uint64_t   N;
			in >> N;
			resize(N);

			m_bytesWritten = N;
			m_position     = 0;
			in.ReadBuffer( m_memory.get(), N );

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


