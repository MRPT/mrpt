/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#ifdef _MSC_VER
#	define _SCL_SECURE_NO_WARNINGS
#endif

#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;
using namespace std;

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CFileOutputStream::CFileOutputStream(
	const string &fileName,
	bool  append ) : m_of()
{
	MRPT_START

	if (!open(fileName,append))
		THROW_EXCEPTION_CUSTOM_MSG1( "Error creating/opening for write file: '%s'",fileName.c_str() );

	MRPT_END
}

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CFileOutputStream::CFileOutputStream( ): m_of()
{
}

/*---------------------------------------------------------------
							open
 ---------------------------------------------------------------*/
bool CFileOutputStream::open(
	const string &fileName,
	bool  append )
{
	close();

	// Open for write/append:
	ios_base::openmode  openMode = ios_base::binary | ios_base::out;
	if ( append )
		openMode |= ios_base::app;

	m_of.open(fileName.c_str(),  openMode );
	return m_of.is_open();
}

/*---------------------------------------------------------------
							close
 ---------------------------------------------------------------*/
void CFileOutputStream::close()
{
	if (m_of.is_open()) m_of.close();
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/
CFileOutputStream::~CFileOutputStream()
{
	close();
}


/*---------------------------------------------------------------
							Read
			Reads bytes from the stream into Buffer
 ---------------------------------------------------------------*/

size_t  CFileOutputStream::Read(void *Buffer, size_t Count)
{
	MRPT_UNUSED_PARAM(Buffer); MRPT_UNUSED_PARAM(Count);
	THROW_EXCEPTION("Trying to read from a write file stream.");
}

/*---------------------------------------------------------------
							Write
			Writes a block of bytes to the stream.
 ---------------------------------------------------------------*/
size_t  CFileOutputStream::Write(const void *Buffer, size_t Count)
{
	if (!m_of.is_open()) return 0;

	m_of.write( static_cast<const char*>(Buffer),Count);
	return m_of.fail() ? 0:Count;
}

/*---------------------------------------------------------------
							Seek
	Method for moving to a specified position in the streamed resource.
	 See documentation of CStream::Seek
 ---------------------------------------------------------------*/
uint64_t CFileOutputStream::Seek(uint64_t Offset, CStream::TSeekOrigin Origin)
{
	if (!m_of.is_open()) return 0;

	ofstream::off_type  offset = Offset;
	ofstream::seekdir  way;

	switch(Origin)
	{
	case sFromBeginning: way = ios_base::beg; break;
	case sFromCurrent: way = ios_base::cur; break;
	case sFromEnd: way = ios_base::end; break;
	default: THROW_EXCEPTION("Invalid value for 'Origin'");
	}

	m_of.seekp(offset, way);
	return getPosition();
}

/*---------------------------------------------------------------
						getTotalBytesCount
 ---------------------------------------------------------------*/
uint64_t CFileOutputStream::getTotalBytesCount()
{
	if (!fileOpenCorrectly()) return 0;

	uint64_t previousPos = getPosition();
	uint64_t fileSize = Seek(0,sFromEnd);
	Seek(previousPos);
	return fileSize;
}

/*---------------------------------------------------------------
						getPosition
 ---------------------------------------------------------------*/
uint64_t CFileOutputStream::getPosition()
{
		if (m_of.is_open())
				return m_of.tellp();
		else	return 0;
}

/*---------------------------------------------------------------
						fileOpenCorrectly
 ---------------------------------------------------------------*/
bool  CFileOutputStream::fileOpenCorrectly()
{
	return m_of.is_open();
}
