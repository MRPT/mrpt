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

#include <mrpt/base.h>  // Precompiled headers 

#include <mrpt/synch/CPipe.h>

#ifdef MRPT_OS_WINDOWS
#	include <windows.h>
#else
#	include <sys/types.h>
#	include <unistd.h>
#	include <stdio.h>
#	include <stdlib.h>
#endif

using namespace mrpt::utils;
using namespace mrpt::synch;


CPipeBaseEndPoint::CPipeBaseEndPoint() :
	m_pipe_file(0)
{
}

CPipeBaseEndPoint::~CPipeBaseEndPoint()
{
	// Close:
	if (m_pipe_file)
	{
#		ifdef MRPT_OS_WINDOWS
		// Win32 pipes


#else
		// UNIX pipes

#endif
	}
	m_pipe_file = 0;
}

/** De-serializes one end-point description, for example, from a parent process. */
CPipeBaseEndPoint::CPipeBaseEndPoint(const std::string &serialized)
{
	std::istringstream ss;
	ss.str(serialized);

#ifdef MRPT_OS_WINDOWS
	// Win32 pipes
	uint64_t val;
	if (!(ss >> val))
	{ THROW_EXCEPTION("Error parsing PIPE handle!") }
	m_pipe_file = reinterpret_cast<void*>(val);
#else
	// UNIX pipes
	ss >> m_pipe_file;
#endif
}
			
#include <mrpt/utils/mrpt_inttypes.h>

/** Converts the end-point into a string suitable for reconstruction at a child process.
* This *invalidates* this object, since only one real end-point can exist at once.
*/
std::string CPipeBaseEndPoint::serialize()
{
	std::string ret;
#ifdef MRPT_OS_WINDOWS
	// Win32 pipes
	ret = mrpt::format("%"PRIu64 ,reinterpret_cast<uint64_t>(m_pipe_file) );
#else
	// UNIX pipes
	ret= mrpt::format("%i",m_pipe_file);
#endif
	m_pipe_file=0; // We don't own this file anymore...
	return ret;
}

