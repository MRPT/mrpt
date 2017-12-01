/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/io/CStream.h>

namespace mrpt
{
namespace io
{
/** Transparently opens a compressed "gz" file and reads uncompressed data from
 * it.
 *   If the file is not a .gz file, it silently reads data from the file.
 *  This class requires compiling MRPT with wxWidgets. If wxWidgets is not
 * available then the class is actually mapped to the standard CFileInputStream
 *
 * \sa CFileInputStream
 * \ingroup mrpt_io_grp
 */
class CFileGZInputStream : public CStream
{
   protected:
	size_t Read(void* Buffer, size_t Count) override;
	size_t Write(const void* Buffer, size_t Count) override;

   private:
	void* m_f;
	/** Compressed file size */
	uint64_t m_file_size;

   public:
	/** Constructor without open */
	CFileGZInputStream();

	/** Constructor and open
	 * \param fileName The file to be open in this stream
	 * \exception std::exception If there's an error opening the file.
	 */
	CFileGZInputStream(const std::string& fileName);

	CFileGZInputStream(const CFileGZInputStream&) = delete;
	CFileGZInputStream& operator=(const CFileGZInputStream&) = delete;

	/** Dtor */
	virtual ~CFileGZInputStream();

	/** Opens the file for read.
	 * \param fileName The file to be open in this stream
	 * \return false if there's an error opening the file, true otherwise
	 */
	bool open(const std::string& fileName);
	/** Closes the file */
	void close();
	/** Returns true if the file was open without errors. */
	bool fileOpenCorrectly();
	/** Returns true if the file was open without errors. */
	bool is_open() { return fileOpenCorrectly(); }
	/** Will be true if EOF has been already reached. */
	bool checkEOF();

	/** Method for getting the total number of <b>compressed</b> bytes of in the
	 * file (the physical size of the compressed file). */
	uint64_t getTotalBytesCount() override;
	/** Method for getting the current cursor position in the <b>compressed</b>,
	 * where 0 is the first byte and TotalBytesCount-1 the last one. */
	uint64_t getPosition() override;

	/** This method is not implemented in this class */
	uint64_t Seek(int64_t, CStream::TSeekOrigin = sFromBeginning) override;

};  // End of class def.

static_assert(
	!std::is_copy_constructible<CFileGZInputStream>::value &&
		!std::is_copy_assignable<CFileGZInputStream>::value,
	"Copy Check");
}  // End of namespace
}  // end of namespace
