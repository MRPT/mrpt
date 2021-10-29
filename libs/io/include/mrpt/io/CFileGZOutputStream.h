/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/optional_ref.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/io/CStream.h>
#include <mrpt/io/open_flags.h>

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
	/** Constructor: opens an output file with the given compression level
	 * (Default= 1, the minimum, fastest).
	 *
	 * \param fileName The file to be open in this stream
	 * \param append If set to true, the file will be opened for writing and the
	 * current cursor position set at the end of the file. Otherwise, previous
	 * contents will be lost.
	 * \exception std::exception if the file cannot be opened.
	 */
	CFileGZOutputStream(
		const std::string& fileName, const OpenMode mode = OpenMode::TRUNCATE,
		int compressionLevel = 1);

	/** Constructor, without opening the file.
	 * \sa open
	 */
	CFileGZOutputStream();

	CFileGZOutputStream(const CFileGZOutputStream&) = delete;
	CFileGZOutputStream& operator=(const CFileGZOutputStream&) = delete;

	/** Destructor */
	~CFileGZOutputStream() override;

	std::string getStreamDescription() const override;

	/** Open a file for write, choosing the compression level
	 * \param fileName The file to be open in this stream
	 * \param compress_level 0:no compression, 1:fastest, 9:best
	 * \return true on success, false on any error.
	 */
	bool open(
		const std::string& fileName, int compress_level = 1,
		mrpt::optional_ref<std::string> error_msg = std::nullopt,
		const OpenMode mode = OpenMode::TRUNCATE);

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
};	// End of class def.
static_assert(
	!std::is_copy_constructible_v<CFileGZOutputStream> &&
		!std::is_copy_assignable_v<CFileGZOutputStream>,
	"Copy Check");
}  // namespace mrpt::io
