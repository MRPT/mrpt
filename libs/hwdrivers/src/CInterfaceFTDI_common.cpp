/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
