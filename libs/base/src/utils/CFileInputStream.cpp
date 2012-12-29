/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
