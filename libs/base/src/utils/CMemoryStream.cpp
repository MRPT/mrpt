/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/system/os.h>
#include <cstring>

using namespace mrpt::utils;
using namespace std;

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CMemoryStream::CMemoryStream() :
	m_memory		(NULL),
	m_size		(0),
	m_position	(0),
	m_bytesWritten(0),
	m_alloc_block_size(0x1000),
	m_read_only (false)
{
}

/*---------------------------------------------------------------
			Constructor with data
 ---------------------------------------------------------------*/
CMemoryStream::CMemoryStream(
	const void *data,
	const uint64_t nBytesInData )  :
	m_memory		(NULL),
	m_size		(0),
	m_position	(0),
	m_bytesWritten(0),
	m_alloc_block_size(0x1000),
	m_read_only (false)
{
	MRPT_START
	ASSERT_(data!=NULL);

	// Set data:
	resize( nBytesInData );
	memcpy( m_memory.get(),data,nBytesInData );

	m_bytesWritten=nBytesInData;

	MRPT_END
}

/*---------------------------------------------------------------
				assignMemoryNotOwn
 ---------------------------------------------------------------*/
void CMemoryStream::assignMemoryNotOwn( const void *data, const uint64_t nBytesInData )
{
	this->Clear();
	m_memory.set(data);
	m_size		   = nBytesInData;
	m_position	   = 0;
	m_bytesWritten = 0;
	m_read_only    = true;
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/
CMemoryStream::~CMemoryStream()
{
	if (!m_read_only)
	{
		// Free memory buffer:
		resize(0);
	}
}

/*---------------------------------------------------------------
							resize
 ---------------------------------------------------------------*/
void CMemoryStream::resize(uint64_t newSize)
{
	if (m_read_only)
		THROW_EXCEPTION("[CMemoryStream::resize] Cannot change memory block size since it was set with 'assign'")

	if (!newSize)
	{	// Free buffer:
		if (m_memory.get())
			free(m_memory.get());
		m_memory=NULL;
		m_size=0;
		m_position=0;
	}
	else
	{	// Resize:
		m_memory.set( realloc( m_memory.get(), newSize ));

		// Check for non-memory errors??
		if (newSize) ASSERT_(m_memory.get());

		m_size = newSize;
	}

	if (m_bytesWritten>m_size) m_bytesWritten=m_size;
}

/*---------------------------------------------------------------
							Read
			Reads bytes from the stream into Buffer
 ---------------------------------------------------------------*/
size_t CMemoryStream::Read(void *Buffer, size_t Count)
{
	// Enought bytes?
	long maxAvail = (((long)m_size)) - ((long)m_position);
	size_t  nToRead = (size_t) min(((long)Count),maxAvail);

	// Copy the memory block:
	if (nToRead>0)
		memcpy(Buffer, ((char*)m_memory.get()) + m_position, nToRead );

	// Update cursor position:
	m_position+=nToRead;
	return nToRead;
}

/*---------------------------------------------------------------
							Write
			Writes a block of bytes to the stream.
 ---------------------------------------------------------------*/
size_t CMemoryStream::Write(const void *Buffer, size_t Count)
{
	// Enought space in current bufer?
	size_t requiredSize = m_position + Count;

	if (requiredSize>=m_size)
	{
		// Incrent the size of reserved memory:
		resize( requiredSize + m_alloc_block_size );
	}

	// Copy the memory block:
	memcpy(((char*)m_memory.get()) + m_position, Buffer, Count );

	// New cursor position:
	m_position = requiredSize;

	m_bytesWritten=max(m_bytesWritten,m_position);

	return Count;
}

/*---------------------------------------------------------------
							Seek
	Method for moving to a specified position in the streamed resource.
	 See documentation of CStream::Seek
 ---------------------------------------------------------------*/
uint64_t CMemoryStream::Seek(uint64_t Offset, CStream::TSeekOrigin Origin)
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

	if (m_position>=m_size) m_position=m_size-1;

	return m_position;
}

/*---------------------------------------------------------------
						getTotalBytesCount
 ---------------------------------------------------------------*/
uint64_t CMemoryStream::getTotalBytesCount()
{
	return m_bytesWritten;
}

/*---------------------------------------------------------------
						getPosition
 ---------------------------------------------------------------*/
uint64_t CMemoryStream::getPosition()
{
	return m_position;
}

/*---------------------------------------------------------------
						Clear
 ---------------------------------------------------------------*/
void  CMemoryStream::Clear()
{
	if (!m_read_only)
	{
		resize(0);
	}
	else
	{
		m_memory=0;
		m_size = 0;
		m_position=0;
		m_bytesWritten=0;
		m_read_only = false;
	}
}

/*---------------------------------------------------------------
						getRawBufferData
	Method for getting a pointer to the raw stored data. The
	lenght in bytes is given by getTotalBytesCount
 ---------------------------------------------------------------*/
void*  CMemoryStream::getRawBufferData()
{
	return m_memory.get();
}

/*---------------------------------------------------------------
						changeSize
Change size. This would be rarely used
 Use ">>" operators for writing to stream.
 ---------------------------------------------------------------*/
void  CMemoryStream::changeSize( uint64_t newSize )
{
	resize(newSize);
}

/*---------------------------------------------------------------
					saveBufferToFile
 ---------------------------------------------------------------*/
bool CMemoryStream::saveBufferToFile( const std::string &file_name )
{
	try
	{
		CFileOutputStream	fo(file_name);
		fo.WriteBuffer( m_memory.get(), getTotalBytesCount() );
		return true;
	}
	catch(...)
	{
		return false;
	}
}

/*---------------------------------------------------------------
					loadBufferFromFile
 ---------------------------------------------------------------*/
bool CMemoryStream::loadBufferFromFile( const std::string &file_name )
{
	try
	{
		CFileInputStream	fi(file_name);
		uint64_t  N = fi.getTotalBytesCount();

		// Read into the buffer:
		Clear();
		resize(N+100);
		uint64_t N_read = fi.ReadBuffer( m_memory.get(), N );

		m_position = N_read;
		m_bytesWritten = max(m_bytesWritten,m_position);

		return N_read==N;
	}
	catch(...)
	{
		return false;
	}
}


// Used in mrpt_send_to_zmq(). `hint` points to a `TFreeFnDataForZMQ` struct, to be freed here.
void mrpt::utils::internal::free_fn_for_zmq(void * /* data*/, void *hint)
{
	mrpt::utils::internal::TFreeFnDataForZMQ* fd = reinterpret_cast<mrpt::utils::internal::TFreeFnDataForZMQ*>(hint);
	if (fd->do_free) delete fd->buf;
	delete fd;
}
