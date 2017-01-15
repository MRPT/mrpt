/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers
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
void CPipe::initializePipe(CPipeReadEndPoint& outReadPipe, CPipeWriteEndPoint& outWritePipe)
{
#	ifdef MRPT_OS_WINDOWS
	// Win32 pipes
	HANDLE hRead, hWrite;
	if (!CreatePipe(&hRead, &hWrite, NULL, 0))
		THROW_EXCEPTION("Win32 error creating pipe endpoints!")

	outReadPipe.m_pipe_file = hRead;
	outWritePipe.m_pipe_file = hWrite;
#else
	// UNIX pipes
	int fds[2];
	if (::pipe(fds))
		THROW_EXCEPTION("Unix error creating pipe endpoints!")

	outReadPipe.m_pipe_file = fds[0];
	outWritePipe.m_pipe_file = fds[1];
#endif
}


// ------------------  CPipeBaseEndPoint ------------------
CPipeBaseEndPoint::CPipeBaseEndPoint() :
	timeout_read_start_us(0), 
	timeout_read_between_us(0),
	m_pipe_file(0)
{
}

CPipeBaseEndPoint::~CPipeBaseEndPoint()
{
	this->close();
}

// Close:
void CPipeBaseEndPoint::close()
{
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
	ASSERTMSG_(m_pipe_file!=0, "Pipe is closed, can't serialize!")
	std::stringstream ss;
#ifdef MRPT_OS_WINDOWS
	// Win32 pipes
	ss << reinterpret_cast<uint64_t>(m_pipe_file);
#else
	// UNIX pipes
	ss << m_pipe_file;
#endif
	m_pipe_file=0; // We don't own this file anymore...
	return ss.str();
}

// Methods that don't make sense in pipes:
uint64_t CPipeBaseEndPoint::Seek(uint64_t , CStream::TSeekOrigin ) {  return 0; }
uint64_t CPipeBaseEndPoint::getTotalBytesCount() { return 0; }
uint64_t CPipeBaseEndPoint::getPosition() { return 0; }

/** Introduces a pure virtual method responsible for reading from the stream */
size_t  CPipeBaseEndPoint::Read(void *Buffer, size_t Count)
{
	ASSERTMSG_(m_pipe_file!=0, "Pipe is closed, can't read!")

#if defined(MRPT_OS_WINDOWS)
	// Win32 pipes
	DWORD nActuallyRead;
	if (!ReadFile((HANDLE)m_pipe_file, Buffer, Count,&nActuallyRead, NULL ))
		return 0;
	else return static_cast<size_t>(nActuallyRead);
#else
	// UNIX pipes
	if (!timeout_read_start_us && !timeout_read_between_us) {
		// Read without timeout:
		return ::read(m_pipe_file,Buffer,Count);
	}
	else
	{
		// Use timeouts:
		size_t alreadyRead = 0;
		bool   timeoutExpired = false;

		struct timeval	timeoutSelect;
		struct timeval	*ptrTimeout;

		// Init fd_set structure & add our socket to it:
		fd_set read_fds;
		FD_ZERO(&read_fds);
		FD_SET(m_pipe_file, &read_fds);

		// Loop until timeout expires or the socket is closed.
		while ( alreadyRead<Count && !timeoutExpired )
		{
			// Use the "first" or "between" timeouts:
			unsigned int curTimeout_us = alreadyRead==0 ? timeout_read_start_us : timeout_read_between_us;

			if (curTimeout_us==0)
				ptrTimeout = NULL;
			else
			{
				timeoutSelect.tv_sec = curTimeout_us / 1000000;
				timeoutSelect.tv_usec = (curTimeout_us % 1000000);
				ptrTimeout = &timeoutSelect;
			}

			// Wait for received data
			if (::select(
							 m_pipe_file+1,		// __nfds
							 &read_fds,		// Wait for read
							 NULL,			// Wait for write
							 NULL,			// Wait for except.
							 ptrTimeout)	// Timeout
					!= 1)
			{ // Timeout:
				timeoutExpired = true;
			}
			else
			{
				// Compute remaining part:
				const size_t remainToRead = Count - alreadyRead;

				// Receive bytes:
				const size_t readNow = ::read(m_pipe_file,((char*)Buffer) + alreadyRead, (int)remainToRead);

				if (readNow != static_cast<size_t>(-1))
				{
					// Accumulate the received length:
					alreadyRead += readNow;
				}
				else
				{
					// Error:
					this->close();
					return alreadyRead;
				}
				if (readNow==0 && remainToRead!=0)
				{
					// We had an event of data available, so if we have now a zero,
					//  the socket has been gracefully closed:
					timeoutExpired = true;
					close();
				}
			}
		} // end while
		return alreadyRead;
	}
#endif
}

/** Introduces a pure virtual method responsible for writing to the stream.
	*  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written. */
size_t  CPipeBaseEndPoint::Write(const void *Buffer, size_t Count)
{
	ASSERTMSG_(m_pipe_file!=0, "Pipe is closed, can't write!")

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
