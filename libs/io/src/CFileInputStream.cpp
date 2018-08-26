/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"  // Precompiled headers

#include <mrpt/io/CFileInputStream.h>
#include <mrpt/core/exceptions.h>

using namespace mrpt::io;
using namespace std;

static_assert(
	!std::is_copy_constructible_v<CFileInputStream> &&
		!std::is_copy_assignable_v<CFileInputStream>,
	"Copy Check");

CFileInputStream::CFileInputStream(const string& fileName) : m_if()
{
	MRPT_START

	// Try to open the file:
	// Open for input:
	if (!open(fileName))
		THROW_EXCEPTION_FMT(
			"Error trying to open file: '%s'", fileName.c_str());

	MRPT_END
}

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CFileInputStream::CFileInputStream() : m_if() {}
/*---------------------------------------------------------------
							open
 ---------------------------------------------------------------*/
bool CFileInputStream::open(const string& fileName)
{
	// Try to open the file:
	// Open for input:
	m_if.open(fileName.c_str(), ios_base::binary | ios_base::in);
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
CFileInputStream::~CFileInputStream() { close(); }
/*---------------------------------------------------------------
							Read
			Reads bytes from the stream into Buffer
 ---------------------------------------------------------------*/
size_t CFileInputStream::Read(void* Buffer, size_t Count)
{
	if (!m_if.is_open()) return 0;

	m_if.read(static_cast<char*>(Buffer), Count);
	return m_if.fail() ? 0 : Count;
}

/*---------------------------------------------------------------
							Write
			Writes a block of bytes to the stream.
 ---------------------------------------------------------------*/
size_t CFileInputStream::Write(const void* Buffer, size_t Count)
{
	MRPT_UNUSED_PARAM(Buffer);
	MRPT_UNUSED_PARAM(Count);
	THROW_EXCEPTION("Trying to write to a read file stream.");
}

/*---------------------------------------------------------------
							Seek
	Method for moving to a specified position in the streamed resource.
	 See documentation of CStream::Seek
 ---------------------------------------------------------------*/
uint64_t CFileInputStream::Seek(int64_t Offset, CStream::TSeekOrigin Origin)
{
	if (!m_if.is_open()) return 0;

	ifstream::off_type offset = Offset;
	ifstream::seekdir way;

	switch (Origin)
	{
		case sFromBeginning:
			way = ios_base::beg;
			break;
		case sFromCurrent:
			way = ios_base::cur;
			break;
		case sFromEnd:
			way = ios_base::end;
			break;
		default:
			THROW_EXCEPTION("Invalid value for 'Origin'");
	}

	m_if.seekg(offset, way);

	return getPosition();
}

/*---------------------------------------------------------------
						getTotalBytesCount
 ---------------------------------------------------------------*/
uint64_t CFileInputStream::getTotalBytesCount() const
{
	if (!fileOpenCorrectly()) return 0;

	auto& f = const_cast<std::ifstream&>(m_if);

	const uint64_t previousPos = f.tellg();
	f.seekg(0, ios_base::end);
	uint64_t fileSize = f.tellg();
	f.seekg(previousPos, ios_base::beg);
	return fileSize;
}

/*---------------------------------------------------------------
						getPosition
 ---------------------------------------------------------------*/
uint64_t CFileInputStream::getPosition() const
{
	auto& f = const_cast<std::ifstream&>(m_if);
	if (m_if.is_open())
		return f.tellg();
	else
		return 0;
}

/*---------------------------------------------------------------
						fileOpenCorrectly
 ---------------------------------------------------------------*/
bool CFileInputStream::fileOpenCorrectly() const { return m_if.is_open(); }
/*---------------------------------------------------------------
						readLine
 ---------------------------------------------------------------*/
bool CFileInputStream::readLine(string& str)
{
	str = string();  // clear() is not defined in VC6
	if (!m_if.is_open()) return false;

	std::getline(m_if, str);
	return !m_if.fail() && !m_if.eof();
}

/*---------------------------------------------------------------
						checkEOF
 ---------------------------------------------------------------*/
bool CFileInputStream::checkEOF()
{
	if (!m_if.is_open()) return true;
	return m_if.eof();
}

void CFileInputStream::clearError()
{
	if (m_if.is_open()) m_if.clear();
}
