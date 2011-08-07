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
uint64_t CFileOutputStream::Seek(long Offset, CStream::TSeekOrigin Origin)
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
	Seek(static_cast<long>(previousPos));
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
