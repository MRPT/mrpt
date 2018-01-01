/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"  // Precompiled headers

#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/core/exceptions.h>

#include <zlib.h>

#define THE_GZFILE reinterpret_cast<gzFile>(m_f)

using namespace mrpt::io;
using namespace std;

CFileGZOutputStream::CFileGZOutputStream(const string& fileName) : m_f(nullptr)
{
	MRPT_START
	if (!open(fileName))
		THROW_EXCEPTION_FMT(
			"Error trying to open file: '%s'", fileName.c_str());
	MRPT_END
}

CFileGZOutputStream::CFileGZOutputStream() : m_f(nullptr) {}
bool CFileGZOutputStream::open(const string& fileName, int compress_level)
{
	MRPT_START

	if (m_f) gzclose(THE_GZFILE);

	// Open gz stream:
	m_f = gzopen(fileName.c_str(), format("wb%i", compress_level).c_str());
	return m_f != nullptr;

	MRPT_END
}

CFileGZOutputStream::~CFileGZOutputStream() { close(); }
void CFileGZOutputStream::close()
{
	if (m_f)
	{
		gzclose(THE_GZFILE);
		m_f = nullptr;
	}
}

size_t CFileGZOutputStream::Read(void*, size_t)
{
	THROW_EXCEPTION("Trying to read from an output file stream.");
}

size_t CFileGZOutputStream::Write(const void* Buffer, size_t Count)
{
	if (!m_f)
	{
		THROW_EXCEPTION("File is not open.");
	}
	return gzwrite(THE_GZFILE, const_cast<void*>(Buffer), Count);
}

uint64_t CFileGZOutputStream::getPosition() const
{
	if (!m_f)
	{
		THROW_EXCEPTION("File is not open.");
	}
	return gztell(THE_GZFILE);
}

bool CFileGZOutputStream::fileOpenCorrectly() const { return m_f != nullptr; }
uint64_t CFileGZOutputStream::Seek(int64_t, CStream::TSeekOrigin)
{
	THROW_EXCEPTION("Method not available in this class.");
}

uint64_t CFileGZOutputStream::getTotalBytesCount() const
{
	THROW_EXCEPTION("Method not available in this class.");
}
