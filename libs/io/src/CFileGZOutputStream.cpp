/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"	 // Precompiled headers
//
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CFileGZOutputStream.h>

#include <cerrno>
#include <cstring>	// strerror
#include <type_traits>
//
#include <mrpt/config.h>
#if HAVE_UNISTD_H
#include <unistd.h>
#endif
//
#include <zlib.h>

using namespace mrpt::io;
using namespace std;

struct CFileGZOutputStream::Impl
{
	gzFile f = nullptr;
	std::string filename;
};

CFileGZOutputStream::CFileGZOutputStream()
	: m_f(mrpt::make_impl<CFileGZOutputStream::Impl>())
{
}

CFileGZOutputStream::CFileGZOutputStream(
	const std::string& fileName, const OpenMode mode, int compressionLevel)
	: CFileGZOutputStream()
{
	MRPT_START
	std::string err_msg;
	if (!open(fileName, compressionLevel, err_msg, mode))
		THROW_EXCEPTION_FMT(
			"Error trying to open file: '%s', error: '%s'", fileName.c_str(),
			err_msg.c_str());
	MRPT_END
}

bool CFileGZOutputStream::open(
	const string& fileName, int compress_level,
	mrpt::optional_ref<std::string> error_msg, const OpenMode mode)
{
	MRPT_START

	if (m_f->f) gzclose(m_f->f);

	// Open gz stream:
	m_f->f = gzopen(
		fileName.c_str(),
		format("%cb%i", mode == OpenMode::APPEND ? 'a' : 'w', compress_level)
			.c_str());
	if (m_f->f == nullptr && error_msg)
		error_msg.value().get() = std::string(strerror(errno));

	m_f->filename = fileName;

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
		m_f->filename.clear();
	}
}

size_t CFileGZOutputStream::Read(void*, size_t)
{
	THROW_EXCEPTION("Trying to read from an output file stream.");
}

size_t CFileGZOutputStream::Write(const void* Buffer, size_t Count)
{
	if (!m_f->f) { THROW_EXCEPTION("File is not open."); }
	return gzwrite(m_f->f, const_cast<void*>(Buffer), Count);
}

uint64_t CFileGZOutputStream::getPosition() const
{
	if (!m_f->f) { THROW_EXCEPTION("File is not open."); }
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
std::string CFileGZOutputStream::getStreamDescription() const
{
	return mrpt::format(
		"mrpt::io::CFileGZOutputStream for file '%s'", m_f->filename.c_str());
}

std::string CFileGZOutputStream::filePathAtUse() const
{
	// Returns opened file:
	return m_f->filename;
}
