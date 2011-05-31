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

#include <mrpt/hwdrivers.h> // Precompiled headers

#include <mrpt/hwdrivers/CInterfaceFTDI.h>
#include <mrpt/synch.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::synch;

/*-------------------------------------------------------------
					Read
-------------------------------------------------------------*/
size_t  CInterfaceFTDI::Read(void *Buffer, size_t Count)
{
	if (!Count) return 0;

	// Employ a circular_buffer to speed-up lots of small readings:
	if (m_readBuffer.size()>=Count)
	{
		// It's enough with the data in the buffer:
		m_readBuffer.pop_many(reinterpret_cast<uint8_t*>(Buffer),Count);
		return Count;
	}
	else
	{
		// More data must be read:
		uint8_t   buf[4000];

		unsigned long nActualRead=0;
		unsigned long to_read=std::min(m_readBuffer.available(),sizeof(buf));

		ftdi_read(buf,to_read,&nActualRead);		//ftdi_read(Buffer,(unsigned long)Count, &ret );

		// Save data into the circular buffer:
		m_readBuffer.push_many(buf,nActualRead);

		// Read the required amount of bytes:
		size_t nActualReturn = std::min( m_readBuffer.size(), Count );

		m_readBuffer.pop_many(reinterpret_cast<uint8_t*>(Buffer),nActualReturn);

		return nActualReturn;
	}
}

/*-------------------------------------------------------------
					Write
-------------------------------------------------------------*/
size_t  CInterfaceFTDI::Write(const void *Buffer, size_t Count)
{
	unsigned long	ret=0;
	ftdi_write(Buffer,(unsigned long)Count, &ret );
	return (size_t)ret;
}

/*-------------------------------------------------------------
					Seek
-------------------------------------------------------------*/
uint64_t CInterfaceFTDI::Seek(long Offset, CStream::TSeekOrigin Origin)
{
	MRPT_UNUSED_PARAM(Offset);
	MRPT_UNUSED_PARAM(Origin);
	return 0;
}

/*-------------------------------------------------------------
					getTotalBytesCount
-------------------------------------------------------------*/
uint64_t CInterfaceFTDI::getTotalBytesCount()
{
	return 0;
}

/*-------------------------------------------------------------
					getPosition
-------------------------------------------------------------*/
uint64_t CInterfaceFTDI::getPosition()
{
	return 0;
}

/*-------------------------------------------------------------
					ReadBufferImmediate
-------------------------------------------------------------*/
size_t CInterfaceFTDI::ReadBufferImmediate(void *Buffer, size_t Count)
{
	unsigned long nRead;
	ftdi_read(Buffer, Count, &nRead);
	return nRead;
}
