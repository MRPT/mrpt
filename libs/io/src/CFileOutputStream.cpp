/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"  // Precompiled headers

#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/core/exceptions.h>

using namespace mrpt::io;
using namespace std;

CFileOutputStream::CFileOutputStream(const string& fileName, bool append)
	: m_of()
{
	MRPT_START

	if (!open(fileName, append))
		THROW_EXCEPTION_FMT(
			"Error creating/opening for write file: '%s'", fileName.c_str());

	MRPT_END
}

CFileOutputStream::CFileOutputStream() : m_of() {}
bool CFileOutputStream::open(const string& fileName, bool append)
{
	close();

	// Open for write/append:
	ios_base::openmode openMode = ios_base::binary | ios_base::out;
	if (append) openMode |= ios_base::app;

	m_of.open(fileName.c_str(), openMode);
	return m_of.is_open();
}

void CFileOutputStream::close()
{
	if (m_of.is_open()) m_of.close();
}

CFileOutputStream::~CFileOutputStream() { close(); }
size_t CFileOutputStream::Read(void* Buffer, size_t Count)
{
	MRPT_UNUSED_PARAM(Buffer);
	MRPT_UNUSED_PARAM(Count);
	THROW_EXCEPTION("Trying to read from a write file stream.");
}

size_t CFileOutputStream::Write(const void* Buffer, size_t Count)
{
	if (!m_of.is_open()) return 0;

	m_of.write(static_cast<const char*>(Buffer), Count);
	return m_of.fail() ? 0 : Count;
}

uint64_t CFileOutputStream::Seek(int64_t Offset, CStream::TSeekOrigin Origin)
{
	if (!m_of.is_open()) return 0;

	ofstream::off_type offset = Offset;
	ofstream::seekdir way;

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

	m_of.seekp(offset, way);
	return getPosition();
}

uint64_t CFileOutputStream::getTotalBytesCount() const
{
	if (!fileOpenCorrectly()) return 0;

	auto& f = const_cast<std::ofstream&>(m_of);

	const uint64_t previousPos = f.tellp();
	f.seekp(0, ios_base::end);
	uint64_t fileSize = f.tellp();
	f.seekp(previousPos, ios_base::beg);
	return fileSize;
}

uint64_t CFileOutputStream::getPosition() const
{
	auto& f = const_cast<std::ofstream&>(m_of);
	if (m_of.is_open())
		return f.tellp();
	else
		return 0;
}

bool CFileOutputStream::fileOpenCorrectly() const { return m_of.is_open(); }
