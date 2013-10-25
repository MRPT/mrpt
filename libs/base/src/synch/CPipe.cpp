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
#include <mrpt/utils/mrpt_inttypes.h>  // For PRIu64

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

// ------------------  CPipe ------------------

/** Creates a new pipe and returns the read & write end-points as newly allocated objects. */
void CPipe::createPipe(std::auto_ptr<CPipeReadEndPoint>& outReadPipe,std::auto_ptr<CPipeWriteEndPoint>& outWritePipe)
{
	outReadPipe  = std::auto_ptr<CPipeReadEndPoint>(new CPipeReadEndPoint);
	outWritePipe = std::auto_ptr<CPipeWriteEndPoint>(new CPipeWriteEndPoint);
#	ifdef MRPT_OS_WINDOWS
	// Win32 pipes
	HANDLE hRead, hWrite;
	if (!CreatePipe(&hRead, &hWrite, NULL, 0))
		THROW_EXCEPTION("Win32 error creating pipe endpoints!")

	outReadPipe->m_pipe_file = hRead;
	outWritePipe->m_pipe_file = hWrite;
#else
	// UNIX pipes
	int fds[2];
	if (::pipe(fds))
		THROW_EXCEPTION("Unix error creating pipe endpoints!")

	outReadPipe->m_pipe_file = fds[0];
	outWritePipe->m_pipe_file = fds[1];
#endif
}


// ------------------  CPipeBaseEndPoint ------------------
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

		// Flush the pipe to allow the client to read the pipe's contents
		// before disconnecting.
		FlushFileBuffers((HANDLE)m_pipe_file);

		DisconnectNamedPipe((HANDLE)m_pipe_file);
		CloseHandle((HANDLE)m_pipe_file);
#else
		// UNIX pipes
		::fsync(m_pipe_file);
		::close(m_pipe_file);
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

// Methods that don't make sense in pipes:
uint64_t CPipeBaseEndPoint::Seek(uint64_t Offset, CStream::TSeekOrigin Origin) {  return 0; }
uint64_t CPipeBaseEndPoint::getTotalBytesCount() { return 0; }
uint64_t CPipeBaseEndPoint::getPosition() { return 0; }

/** Introduces a pure virtual method responsible for reading from the stream */
size_t  CPipeBaseEndPoint::Read(void *Buffer, size_t Count)
{
#ifdef MRPT_OS_WINDOWS
	// Win32 pipes
	DWORD nActuallyRead;
	if (!ReadFile((HANDLE)m_pipe_file, Buffer, Count,&nActuallyRead, NULL ))
		return 0;
	else return static_cast<size_t>(nActuallyRead);
#else
	// UNIX pipes
	return ::read(m_pipe_file,Buffer,Count);
#endif
}

/** Introduces a pure virtual method responsible for writing to the stream.
	*  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written. */
size_t  CPipeBaseEndPoint::Write(const void *Buffer, size_t Count)
{
#ifdef MRPT_OS_WINDOWS
	// Win32 pipes
	DWORD nActuallyWritten;
	if (!WriteFile((HANDLE)m_pipe_file, Buffer, Count,&nActuallyWritten, NULL ))
		return 0;
	else return static_cast<size_t>(nActuallyWritten);
#else
	// UNIX pipes
	return ::write(m_pipe_file,Buffer,Count);
#endif
}


//  ------------- CPipeReadEndPoint  -------------
CPipeReadEndPoint::CPipeReadEndPoint() : CPipeBaseEndPoint()
{
}

CPipeReadEndPoint::CPipeReadEndPoint(const std::string &serialized) : CPipeBaseEndPoint(serialized)
{
}

//  ------------- CPipeWriteEndPoint  -------------
CPipeWriteEndPoint::CPipeWriteEndPoint() : CPipeBaseEndPoint()
{
}

CPipeWriteEndPoint::CPipeWriteEndPoint(const std::string &serialized) : CPipeBaseEndPoint(serialized)
{
}
