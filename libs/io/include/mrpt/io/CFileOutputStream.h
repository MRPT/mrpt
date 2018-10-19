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
#include <fstream>

namespace mrpt::io
{
/** This CStream derived class allow using a file as a write-only, binary
 * stream.
 *
 * \sa CStream, CFileStream, CFileGZOutputStream
 * \ingroup mrpt_io_grp
 */
class CFileOutputStream : public CStream
{
   private:
	/** The actual output file stream. */
	std::ofstream m_of;

   public:
	/** Constructor
	 * \param fileName The file to be open in this stream
	 * \param append If set to true, the file will be opened for writing and the
	 * current cursor position set at the end of the file. Otherwise, previous
	 * contents will be lost.
	 * \exception std::exception if the file cannot be opened.
	 */
	CFileOutputStream(const std::string& fileName, bool append = false);

	/** Default constructor */
	CFileOutputStream();

	CFileOutputStream(const CFileOutputStream&) = delete;
	CFileOutputStream& operator=(const CFileOutputStream&) = delete;

	/** Open the given file for write
	 * \param fileName The file to be open in this stream
	 * \param append If set to true, the file will be opened for writing and the
	 * current cursor position set at the end of the file. Otherwise, previous
	 * contents will be lost.
	 * \sa fileOpenCorrectly
	 * \return true on success.
	 */
	bool open(const std::string& fileName, bool append = false);

	/** Close the stream. */
	void close();

	/** Destructor */
	~CFileOutputStream() override;

	/** Returns true if the file was open without errors. */
	bool fileOpenCorrectly() const;
	/** Returns true if the file was open without errors. */
	bool is_open() { return fileOpenCorrectly(); }
	// See base class docs
	uint64_t Seek(
		int64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning) override;

	/** Method for getting the total number of bytes writen to buffer */
	uint64_t getTotalBytesCount() const override;

	/** Method for getting the current cursor position, where 0 is the first
	 * byte and TotalBytesCount-1 the last one */
	uint64_t getPosition() const override;

	size_t Read(void* Buffer, size_t Count) override;
	size_t Write(const void* Buffer, size_t Count) override;
};  // End of class def.
static_assert(
	!std::is_copy_constructible_v<CFileOutputStream> &&
		!std::is_copy_assignable_v<CFileOutputStream>,
	"Copy Check");
}  // namespace mrpt::io
