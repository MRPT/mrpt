/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"  // Precompiled headers

#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <algorithm>  // min,max
#include <cstring>  // memcpy

using namespace mrpt::io;
using std::max;
using std::min;

CMemoryStream::CMemoryStream(const void* data, const uint64_t nBytesInData)
{
	MRPT_START
	ASSERT_(data != nullptr);

	// Set data:
	resize(nBytesInData);
	memcpy(m_memory.get(), data, nBytesInData);

	m_bytesWritten = nBytesInData;

	MRPT_END
}

void CMemoryStream::assignMemoryNotOwn(
	const void* data, const uint64_t nBytesInData)
{
	this->Clear();
	m_memory.set(data);
	m_size = nBytesInData;
	m_position = 0;
	m_bytesWritten = 0;
	m_read_only = true;
}

CMemoryStream::~CMemoryStream()
{
	if (!m_read_only)
	{
		// Free buffer:
		if (m_memory.get()) free(m_memory.get());
		m_memory = nullptr;
		m_size = 0;
		m_position = 0;
	}
}

void CMemoryStream::resize(uint64_t newSize)
{
	if (m_read_only)
		THROW_EXCEPTION(
			"[CMemoryStream::resize] Cannot change memory block size since it "
			"was set with 'assign'");

	if (!newSize)
	{  // Free buffer:
		if (m_memory.get()) free(m_memory.get());
		m_memory = nullptr;
		m_size = 0;
		m_position = 0;
	}
	else
	{  // Resize:
		m_memory.set(realloc(m_memory.get(), newSize));

		// Check for non-memory errors??
		if (newSize) ASSERT_(m_memory.get());

		m_size = newSize;
	}

	if (m_bytesWritten > m_size) m_bytesWritten = m_size;
}

size_t CMemoryStream::Read(void* Buffer, size_t Count)
{
	// Enought bytes?
	long maxAvail = (((long)m_size)) - ((long)m_position);
	size_t nToRead = (size_t)min(((long)Count), maxAvail);

	// Copy the memory block:
	if (nToRead > 0)
		memcpy(
			Buffer, reinterpret_cast<char*>(m_memory.get()) + m_position,
			nToRead);

	// Update cursor position:
	m_position += nToRead;
	return nToRead;
}

size_t CMemoryStream::Write(const void* Buffer, size_t Count)
{
	ASSERT_(Buffer != nullptr);
	// Enought space in current bufer?
	size_t requiredSize = m_position + Count;

	if (requiredSize >= m_size)
	{
		// Incrent the size of reserved memory:
		resize(requiredSize + m_alloc_block_size);
	}

	// Copy the memory block:
	memcpy(reinterpret_cast<char*>(m_memory.get()) + m_position, Buffer, Count);

	// New cursor position:
	m_position = requiredSize;

	m_bytesWritten = max(m_bytesWritten, m_position);

	return Count;
}

uint64_t CMemoryStream::Seek(int64_t Offset, CStream::TSeekOrigin Origin)
{
	switch (Origin)
	{
		case sFromBeginning:
			m_position = Offset;
			break;
		case sFromCurrent:
			m_position += Offset;
			break;
		case sFromEnd:
			m_position = m_bytesWritten - 1 + Origin;
			break;
	};

	if (m_position >= m_size) m_position = m_size - 1;

	return m_position;
}

uint64_t CMemoryStream::getTotalBytesCount() const { return m_bytesWritten; }
uint64_t CMemoryStream::getPosition() const { return m_position; }
void CMemoryStream::Clear()
{
	if (!m_read_only)
	{
		resize(0);
	}
	else
	{
		m_memory = nullptr;
		m_size = 0;
		m_position = 0;
		m_bytesWritten = 0;
		m_read_only = false;
	}
}

void* CMemoryStream::getRawBufferData() { return m_memory.get(); }
const void* CMemoryStream::getRawBufferData() const { return m_memory.get(); }
void CMemoryStream::changeSize(uint64_t newSize) { resize(newSize); }
bool CMemoryStream::saveBufferToFile(const std::string& file_name)
{
	try
	{
		CFileOutputStream fo(file_name);
		fo.Write(m_memory.get(), getTotalBytesCount());
		return true;
	}
	catch (...)
	{
		return false;
	}
}

/*---------------------------------------------------------------
					loadBufferFromFile
 ---------------------------------------------------------------*/
bool CMemoryStream::loadBufferFromFile(const std::string& file_name)
{
	try
	{
		CFileInputStream fi(file_name);
		uint64_t N = fi.getTotalBytesCount();

		// Read into the buffer:
		Clear();
		resize(N + 100);
		uint64_t N_read = fi.Read(m_memory.get(), N);

		m_position = N_read;
		m_bytesWritten = max(m_bytesWritten, m_position);

		return N_read == N;
	}
	catch (...)
	{
		return false;
	}
}

// Used in mrpt_send_to_zmq(). `hint` points to a `TFreeFnDataForZMQ` struct, to
// be freed here.
void mrpt::io::internal::free_fn_for_zmq(void* /* data*/, void* hint)
{
	auto* fd = reinterpret_cast<mrpt::io::internal::TFreeFnDataForZMQ*>(hint);
	if (fd->do_free) delete fd->buf;
	delete fd;
}
