/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"	 // Precompiled headers
//
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>

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

static_assert(
	!std::is_copy_constructible_v<CFileGZInputStream> &&
		!std::is_copy_assignable_v<CFileGZInputStream>,
	"Copy Check");

struct CFileGZInputStream::Impl
{
	gzFile f = nullptr;
	std::string filename;
};

CFileGZInputStream::CFileGZInputStream()
	: m_f(mrpt::make_impl<CFileGZInputStream::Impl>())
{
}

CFileGZInputStream::CFileGZInputStream(const string& fileName)
	: CFileGZInputStream()
{
	MRPT_START
	open(fileName);
	MRPT_END
}

bool CFileGZInputStream::open(
	const std::string& fileName, mrpt::optional_ref<std::string> error_msg)
{
	MRPT_START

	if (m_f->f) gzclose(m_f->f);

	// Get compressed file size:
	m_file_size = mrpt::system::getFileSize(fileName);
	if (m_file_size == uint64_t(-1))
	{
		if (error_msg)
			error_msg.value().get() =
				mrpt::format("Couldn't access the file '%s'", fileName.c_str());
		return false;
	}

	// Open gz stream:
	m_f->f = gzopen(fileName.c_str(), "rb");
	if (m_f->f == nullptr && error_msg)
		error_msg.value().get() = std::string(strerror(errno));

	m_f->filename = fileName;

	return m_f->f != nullptr;
	MRPT_END
}

void CFileGZInputStream::close()
{
	if (m_f->f)
	{
		gzclose(m_f->f);
		m_f->f = nullptr;
		m_f->filename.clear();
	}
}

CFileGZInputStream::~CFileGZInputStream() { close(); }
size_t CFileGZInputStream::Read(void* Buffer, size_t Count)
{
	if (!m_f->f) { THROW_EXCEPTION("File is not open."); }

	return gzread(m_f->f, Buffer, Count);
}

size_t CFileGZInputStream::Write(
	[[maybe_unused]] const void* Buffer, [[maybe_unused]] size_t Count)
{
	THROW_EXCEPTION("Trying to write to an input file stream.");
}

uint64_t CFileGZInputStream::getTotalBytesCount() const
{
	if (!m_f->f) { THROW_EXCEPTION("File is not open."); }
	return m_file_size;
}

uint64_t CFileGZInputStream::getPosition() const
{
	if (!m_f->f) { THROW_EXCEPTION("File is not open."); }
	return gztell(m_f->f);
}

bool CFileGZInputStream::fileOpenCorrectly() const { return m_f->f != nullptr; }
bool CFileGZInputStream::checkEOF()
{
	if (!m_f->f) return true;
	else
		return 0 != gzeof(m_f->f);
}

uint64_t CFileGZInputStream::Seek(int64_t, CStream::TSeekOrigin)
{
	THROW_EXCEPTION("Method not available in this class.");
}

std::string CFileGZInputStream::getStreamDescription() const
{
	return mrpt::format(
		"mrpt::io::CFileGZInputStream for file '%s'", m_f->filename.c_str());
}

std::string CFileGZInputStream::filePathAtUse() const
{
	// Returns opened file:
	return m_f->filename;
}
