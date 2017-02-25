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

#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;
using namespace std;

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CFileInputStream::CFileInputStream( const string		&fileName ) : m_if()
{
	MRPT_START

	// Try to open the file:
	// Open for input:
	if (!open(fileName))
		THROW_EXCEPTION_CUSTOM_MSG1( "Error trying to open file: '%s'",fileName.c_str() );

	MRPT_END
}

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CFileInputStream::CFileInputStream() : m_if()
{
}

/*---------------------------------------------------------------
							open
 ---------------------------------------------------------------*/
bool CFileInputStream::open( const string &fileName )
{
	// Try to open the file:
	// Open for input:
	m_if.open(fileName.c_str(), ios_base::binary | ios_base::in );
	return m_if.is_open();
}

/*---------------------------------------------------------------
							close
 ---------------------------------------------------------------*/
void CFileInputStream::close()
{
	if (m_if.is_open()) m_if.close();
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/
CFileInputStream::~CFileInputStream()
{
	close();
}

/*---------------------------------------------------------------
							Read
			Reads bytes from the stream into Buffer
 ---------------------------------------------------------------*/
size_t  CFileInputStream::Read(void *Buffer, size_t Count)
{
	if (!m_if.is_open()) return 0;

	m_if.read(static_cast<char*>(Buffer),Count);
	return m_if.fail() ? 0:Count;
}

/*---------------------------------------------------------------
							Write
			Writes a block of bytes to the stream.
 ---------------------------------------------------------------*/
size_t  CFileInputStream::Write(const void *Buffer, size_t Count)
{
	MRPT_UNUSED_PARAM(Buffer); MRPT_UNUSED_PARAM(Count);
	THROW_EXCEPTION("Trying to write to a read file stream.");
}

/*---------------------------------------------------------------
							Seek
	Method for moving to a specified position in the streamed resource.
	 See documentation of CStream::Seek
 ---------------------------------------------------------------*/
uint64_t CFileInputStream::Seek(uint64_t Offset, CStream::TSeekOrigin Origin)
{
	if (!m_if.is_open()) return 0;

	ifstream::off_type  offset = Offset;
	ifstream::seekdir  way;

	switch(Origin)
	{
	case sFromBeginning: way = ios_base::beg; break;
	case sFromCurrent: way = ios_base::cur; break;
	case sFromEnd: way = ios_base::end; break;
	default: THROW_EXCEPTION("Invalid value for 'Origin'");
	}

	m_if.seekg(offset, way);

	return getPosition();
}

/*---------------------------------------------------------------
						getTotalBytesCount
 ---------------------------------------------------------------*/
uint64_t CFileInputStream::getTotalBytesCount()
{
	if (!fileOpenCorrectly()) return 0;

	uint64_t previousPos = getPosition();
	uint64_t fileSize = Seek(0,sFromEnd);
	Seek( previousPos );
	return fileSize;
}

/*---------------------------------------------------------------
						getPosition
 ---------------------------------------------------------------*/
uint64_t CFileInputStream::getPosition()
{
	if (m_if.is_open())
			return m_if.tellg();
	else	return 0;
}

/*---------------------------------------------------------------
						fileOpenCorrectly
 ---------------------------------------------------------------*/
bool  CFileInputStream::fileOpenCorrectly()
{
	return m_if.is_open();
}

/*---------------------------------------------------------------
						readLine
 ---------------------------------------------------------------*/
bool CFileInputStream::readLine( string &str )
{
	str = string(); // clear() is not defined in VC6
	if (!m_if.is_open()) return false;

	std::getline( m_if, str );
	return !m_if.fail() && !m_if.eof();
}

/*---------------------------------------------------------------
						checkEOF
 ---------------------------------------------------------------*/
bool CFileInputStream::checkEOF()
{
	if (!m_if.is_open())
		return true;
	return m_if.eof();
}
