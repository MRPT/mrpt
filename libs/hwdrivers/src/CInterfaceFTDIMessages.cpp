/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/system/os.h>
#include <mrpt/hwdrivers/CInterfaceFTDIMessages.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;

/*-------------------------------------------------------------
					CInterfaceFTDIMessages
-------------------------------------------------------------*/
CInterfaceFTDIMessages::CInterfaceFTDIMessages()
{
}

/*-------------------------------------------------------------
					~CInterfaceFTDIMessages
-------------------------------------------------------------*/
CInterfaceFTDIMessages::~CInterfaceFTDIMessages()
{
}

/*-------------------------------------------------------------
					sendMessage
-------------------------------------------------------------*/
void  CInterfaceFTDIMessages::sendMessage( const utils::CMessage &msg)
{
	MRPT_START;

	unsigned char	buf[1024];
	unsigned int	nBytesTx = 0;

	ASSERT_(msg.content.size()<256);

	// Build frame -------------------------------------
	buf[nBytesTx++] = 0x69;
	buf[nBytesTx++] = (unsigned char)(msg.type);
	buf[nBytesTx++] = (unsigned char)msg.content.size();
	if (msg.content.size())
		memcpy(	buf+nBytesTx, &msg.content[0], msg.content.size() );
	nBytesTx += (unsigned char)msg.content.size();
	buf[nBytesTx++] = 0x96;

	// Send buffer -------------------------------------
	WriteBuffer(buf,nBytesTx);  // Exceptions will be raised on errors here

	MRPT_END;
}


/*-------------------------------------------------------------
					receiveMessage
-------------------------------------------------------------*/
bool  CInterfaceFTDIMessages::receiveMessage( utils::CMessage &msg )
{
	MRPT_START;

	unsigned char		buf[66000];
	unsigned int		nBytesInFrame=0;
	unsigned long		nBytesToRx=0, nBytesRx;
	unsigned char		tries = 2;
	unsigned int		nB = 0;

	for (;;)
	{
		if (nBytesInFrame<3)
			nBytesToRx = 1;
		else
		{
			if( nBytesInFrame == 3 && buf[0] == 0x79 )
				nBytesToRx = 1;
			else
			{
				if( buf[0] == 0x69 )
					nBytesToRx = (buf[2]+4) - nBytesInFrame;
				if( buf[0] == 0x79 )
				{
					nB = (unsigned int)(buf[2]<<8) + (unsigned int)buf[3]; // Length of the content
					nBytesToRx = (nB + 5) - nBytesInFrame;
				}
			} // end else
		} // end else
		ftdi_read( buf+nBytesInFrame, nBytesToRx, &nBytesRx );

		// No more data! (read timeout is already included in the call to "Read")
		if (!nBytesRx)
			return false;

		if (!nBytesInFrame && buf[0]!=0x69 && buf[0]!=0x79 )
		{
			// Start flag is invalid:
			if (!tries--) return false;
		}
		else
		{
			// Is a new byte for the frame:
			nBytesInFrame += nBytesRx;

			if (nBytesInFrame == (unsigned int)(buf[2]+4) ||
			   nBytesInFrame == (nB + 5) )
			{
				// Frame complete
				// check for frame be ok:

				// copy out data:
				msg.type = buf[1];
				if( buf[0] == 0x69 )
				{
					msg.content.resize( buf[2] );
					if (msg.content.size())
						memcpy( &msg.content[0], &buf[3], buf[2] );
				} // end if
				if ( buf[0] == 0x79 )
				{
					msg.content.resize( nB );
					if (msg.content.size())
						memcpy( &msg.content[0], &buf[4], nB );
				} // end if
				return true;
			}

		}
	}

	MRPT_END;
}
