/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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

#include <mrpt/base.h>  // Precompiled headers


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
	THROW_EXCEPTION("Trying to write to a read file stream.");
}

/*---------------------------------------------------------------
							Seek
	Method for moving to a specified position in the streamed resource.
	 See documentation of CStream::Seek
 ---------------------------------------------------------------*/
uint64_t CFileInputStream::Seek(long Offset, CStream::TSeekOrigin Origin)
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
	Seek(static_cast<long>(previousPos));
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
