/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/hwdrivers/CGPSInterface.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::synch;
using namespace std;

void  CGPSInterface::implement_parser_NMEA()
{
#if 0
	unsigned int	i=0, lineStart = 0;

	while (i<m_bufferLength)
	{
		// Loof for the first complete line of text:
		while (i<m_bufferLength && m_buffer[i]!='\r' && m_buffer[i]!='\n')
			i++;

		// End of buffer or end of line?
		if (i<m_bufferLength)
		{
			// It is the end of a line: Build the string:
			std::string		str;
			int				lenOfLine = i-1 - lineStart;
			if (lenOfLine>0)
			{
				str.resize( lenOfLine );
				memcpy( (void*)str.c_str(), m_buffer + lineStart, lenOfLine );

				// Process it!:
				processGPSstring(str);
			}

			// And this means the comms works!
			m_GPS_comsWork			= true;

            m_state = ssWorking;

			// Pass over this end of line:
			while (i<m_bufferLength && (m_buffer[i]=='\r' || m_buffer[i]=='\n'))
				i++;

			// And start a new line:
			lineStart = i;
		}

	};

	// Dejar en el buffer desde "lineStart" hasta "i-1", inclusive.
	size_t	newBufLen = m_bufferLength - lineStart;

	memcpy( m_buffer, m_buffer + lineStart, newBufLen);

	m_bufferLength = newBufLen;

	// Seguir escribiendo por aqui:
	m_bufferWritePos = i - lineStart;
#endif
}
