/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/io/CStream.h>
#include <mrpt/core/pimpl.h>

namespace mrpt::io
{
/** Saves data to a file and transparently compress the data using the given
 * compression level.
 *   The generated files are in gzip format ("file.gz").
 *  This class requires compiling MRPT with wxWidgets. If wxWidgets is not
 * available then the class is actually mapped to the standard CFileOutputStream
 *
 * \sa CFileOutputStream
 * \ingroup mrpt_io_grp
 */
class CFileGZOutputStream : public CStream
{
   private:
	struct Impl;
	mrpt::pimpl<Impl> m_f;

   public:
	/** Constructor: opens an output file with compression level = 1 (minimum,
	 * fastest).
	 * \param fileName The file to be open in this stream
	 * \sa open
	 */
	CFileGZOutputStream(const std::string& fileName);

	/** Constructor, without opening the file.
	 * \sa open
	 */
	CFileGZOutputStream();

	CFileGZOutputStream(const CFileGZOutputStream&) = delete;
	CFileGZOutputStream& operator=(const CFileGZOutputStream&) = delete;

	/** Destructor */
	~CFileGZOutputStream() override;

	/** Open a file for write, choosing the compression level
	 * \param fileName The file to be open in this stream
	 * \param compress_level 0:no compression, 1:fastest, 9:best
	 * \return true on success, false on any error.
	 */
	bool open(const std::string& fileName, int compress_level = 1);
	/** Close the file */
	void close();
	/** Returns true if the file was open without errors. */
	bool fileOpenCorrectly() const;
	/** Returns true if the file was open without errors. */
	bool is_open() { return fileOpenCorrectly(); }
	/** Method for getting the current cursor position, where 0 is the first
	 * byte and TotalBytesCount-1 the last one. */
	uint64_t getPosition() const override;

	/** This method is not implemented in this class */
	uint64_t Seek(int64_t, CStream::TSeekOrigin = sFromBeginning) override;
	/** This method is not implemented in this class */
	uint64_t getTotalBytesCount() const override;
	size_t Read(void* Buffer, size_t Count) override;
	size_t Write(const void* Buffer, size_t Count) override;
};  // End of class def.
static_assert(
	!std::is_copy_constructible_v<CFileGZOutputStream> &&
		!std::is_copy_assignable_v<CFileGZOutputStream>,
	"Copy Check");
}  // namespace mrpt::io
