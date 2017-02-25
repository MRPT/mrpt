/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CInterfaceFTDI.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;

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
uint64_t CInterfaceFTDI::Seek(uint64_t Offset, CStream::TSeekOrigin Origin)
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
