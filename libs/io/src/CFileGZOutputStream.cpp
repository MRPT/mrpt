/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"  // Precompiled headers

#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/core/exceptions.h>

#include <zlib.h>

using namespace mrpt::io;
using namespace std;

struct CFileGZOutputStream::Impl
{
	gzFile f{nullptr};
};

CFileGZOutputStream::CFileGZOutputStream()
	: m_f(mrpt::make_impl<CFileGZOutputStream::Impl>())
{
}

CFileGZOutputStream::CFileGZOutputStream(const string& fileName)
	: CFileGZOutputStream()
{
	MRPT_START
	if (!open(fileName))
		THROW_EXCEPTION_FMT(
			"Error trying to open file: '%s'", fileName.c_str());
	MRPT_END
}

bool CFileGZOutputStream::open(const string& fileName, int compress_level)
{
	MRPT_START

	if (m_f->f) gzclose(m_f->f);

	// Open gz stream:
	m_f->f = gzopen(fileName.c_str(), format("wb%i", compress_level).c_str());
	return m_f->f != nullptr;

	MRPT_END
}

CFileGZOutputStream::~CFileGZOutputStream() { close(); }
void CFileGZOutputStream::close()
{
	if (m_f->f)
	{
		gzclose(m_f->f);
		m_f->f = nullptr;
	}
}

size_t CFileGZOutputStream::Read(void*, size_t)
{
	THROW_EXCEPTION("Trying to read from an output file stream.");
}

size_t CFileGZOutputStream::Write(const void* Buffer, size_t Count)
{
	if (!m_f->f)
	{
		THROW_EXCEPTION("File is not open.");
	}
	return gzwrite(m_f->f, const_cast<void*>(Buffer), Count);
}

uint64_t CFileGZOutputStream::getPosition() const
{
	if (!m_f->f)
	{
		THROW_EXCEPTION("File is not open.");
	}
	return gztell(m_f->f);
}

bool CFileGZOutputStream::fileOpenCorrectly() const
{
	return m_f->f != nullptr;
}
uint64_t CFileGZOutputStream::Seek(int64_t, CStream::TSeekOrigin)
{
	THROW_EXCEPTION("Method not available in this class.");
}

uint64_t CFileGZOutputStream::getTotalBytesCount() const
{
	THROW_EXCEPTION("Method not available in this class.");
}
