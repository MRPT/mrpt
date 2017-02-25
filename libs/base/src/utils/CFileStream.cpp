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

#include <mrpt/utils/CFileStream.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;
using namespace std;

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CFileStream::CFileStream() : m_f()
{
}

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CFileStream::CFileStream( const string &fileName, TFileOpenModes mode_ ) : m_f()
{
	MRPT_START

	std::ios_base::openmode mode = std::ios_base::in;
	if (mode_==fomRead) mode = std::ios_base::in;
	else if (mode_==fomWrite) mode  = std::ios_base::out | std::ios_base::trunc;
	else if (mode_==fomAppend) mode = std::ios_base::app | std::ios_base::out;
	else if (mode_==(fomRead|fomWrite))   mode = std::ios_base::in | std::ios_base::out | std::ios_base::trunc;
	else if (mode_==(fomAppend|fomWrite)) mode = std::ios_base::in | std::ios_base::out | std::ios_base::app;

	// Try to open the file:
	m_f.open(fileName.c_str(), mode);
	if (!m_f.is_open())
		THROW_EXCEPTION_CUSTOM_MSG1( "Error creating/opening: '%s'",fileName.c_str() );
	MRPT_END
}

/*---------------------------------------------------------------
							open
 ---------------------------------------------------------------*/
bool CFileStream::open(const std::string &fileName, TFileOpenModes mode_)
{
	MRPT_START

	std::ios_base::openmode mode = std::ios_base::in;
	if (mode_==fomRead) mode = std::ios_base::in;
	else if (mode_==fomWrite) mode  = std::ios_base::out | std::ios_base::trunc;
	else if (mode_==fomAppend) mode = std::ios_base::app | std::ios_base::out;
	else if (mode_==(fomRead|fomWrite))   mode = std::ios_base::in | std::ios_base::out | std::ios_base::trunc;
	else if (mode_==(fomAppend|fomWrite)) mode = std::ios_base::in | std::ios_base::out | std::ios_base::app;

	if (m_f.is_open())
		m_f.close();

	m_f.open(fileName.c_str(), ios_base::binary | mode );
	return m_f.is_open();

	MRPT_END
}

/*---------------------------------------------------------------
							close
 ---------------------------------------------------------------*/
void CFileStream::close()
{
	m_f.close();
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/
CFileStream::~CFileStream()
{
	m_f.close();
}


/*---------------------------------------------------------------
							Read
			Reads bytes from the stream into Buffer
 ---------------------------------------------------------------*/
size_t  CFileStream::Read(void *Buffer, size_t Count)
{
	if (! m_f.is_open()) return 0;
	m_f.read(static_cast<char*>(Buffer),Count);
	return m_f.fail() ? 0:Count;
}

/*---------------------------------------------------------------
							Write
			Writes a block of bytes to the stream.
 ---------------------------------------------------------------*/
size_t  CFileStream::Write(const void *Buffer, size_t Count)
{
	if (!m_f.is_open()) return 0;

	m_f.write( static_cast<const char*>(Buffer),Count);
	return m_f.fail() ? 0:Count;
}

/*---------------------------------------------------------------
							Seek
	Method for moving to a specified position in the streamed resource.
	 See documentation of CStream::Seek
 ---------------------------------------------------------------*/
uint64_t CFileStream::Seek(uint64_t Offset, CStream::TSeekOrigin Origin)
{
	if (!m_f.is_open()) return 0;

	fstream::off_type  offset = Offset;
	fstream::seekdir   way;

	switch(Origin)
	{
	case sFromBeginning: way = ios_base::beg; break;
	case sFromCurrent: way = ios_base::cur; break;
	case sFromEnd: way = ios_base::end; break;
	default: THROW_EXCEPTION("Invalid value for 'Origin'");
	}

	m_f.seekp(offset, way);
	m_f.seekg(offset, way);

	return getPosition();
}

/*---------------------------------------------------------------
						getTotalBytesCount
 ---------------------------------------------------------------*/
uint64_t CFileStream::getTotalBytesCount()
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
uint64_t CFileStream::getPosition()
{
	if (m_f.is_open())
			return m_f.tellg();
	else	return 0;
}

/*---------------------------------------------------------------
						getPositionI
 ---------------------------------------------------------------*/
uint64_t CFileStream::getPositionI()
{
	if (m_f.is_open())
			return m_f.tellg();
	else	return 0;
}

/*---------------------------------------------------------------
						getPositionO
 ---------------------------------------------------------------*/
uint64_t CFileStream::getPositionO()
{
	if (m_f.is_open())
			return m_f.tellp();
	else	return 0;
}

/*---------------------------------------------------------------
						fileOpenCorrectly
 ---------------------------------------------------------------*/
bool  CFileStream::fileOpenCorrectly()
{
	return m_f.is_open();
}

/*---------------------------------------------------------------
						readLine
 ---------------------------------------------------------------*/
bool CFileStream::readLine( string &str )
{
	str = string(); // clear() is not defined in VC6
	if (!m_f.is_open()) return false;

	std::getline( m_f, str );
	return !m_f.fail() && !m_f.eof();
}


/*---------------------------------------------------------------
						checkEOF
 ---------------------------------------------------------------*/
bool CFileStream::checkEOF()
{
	if (!m_f.is_open())
			return true;
	return	m_f.eof();
}
