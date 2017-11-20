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
#include <fstream>

namespace mrpt
{
namespace io
{
/** File open modes are used in CFileStream
  *  Posible values are:
	- fomRead
	- fomWrite (creates the file if it didn't exist, otherwise truncates it).
	- fomAppend (creates the file if it didn't exist)
  */
typedef int TFileOpenModes;
enum
{
	fomRead = 1,
	fomWrite = 2,
	fomAppend = 4
};

/** This CStream derived class allow using a file as a read/write binary stream,
 * creating it if the file didn't exist.
 *   The default behavior can be change to open as read, write, read and
 * write,... in the constructor.
 * \sa CStream, CFileInputStream, CFileOutputStrea, CFileGZInputStream
 * \ingroup mrpt_base_grp
 */
class CFileStream : public CStream
{
   protected:
	size_t Read(void* Buffer, size_t Count) override;
	size_t Write(const void* Buffer, size_t Count) override;

   private:
	/** The actual input file stream. */
	std::fstream m_f;

   public:
	/** Constructor and open a file
	 * \param fileName The file to be open in this stream
	 * \param mode The open mode: can be an or'd conbination of different
	 * values.
	 * \exception std::exception On error creating or accessing the file.
	 *  By default the file is opened for open and write and created if not
	 * found.
	 */
	CFileStream(
		const std::string& fileName, TFileOpenModes mode = fomRead | fomWrite);
	/** Constructor */
	CFileStream();

	CFileStream(const CFileStream&) = delete;
	CFileStream& operator=(const CFileStream&) = delete;

	/** Destructor */
	virtual ~CFileStream();

	/** Opens the file, returning true on success.
	  * \param fileName The file to be open in this stream
	  * \param mode The open mode: can be an or'd conbination of different
	 * values.
	  *  By default the file is opened for open and write and created if not
	 * found.
	  */
	bool open(
		const std::string& fileName, TFileOpenModes mode = fomRead | fomWrite);
	/** Closes the file */
	void close();
	/** Returns true if the file was open without errors. */
	bool fileOpenCorrectly();
	/** Returns true if the file was open without errors. */
	bool is_open() { return fileOpenCorrectly(); }
	/** Will be true if EOF has been already reached. */
	bool checkEOF();

	/** Method for moving to a specified position in the streamed resource.
	 *   See documentation of CStream::Seek
	 */
	uint64_t Seek(
		uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning) override;

	/** Method for getting the total number of bytes writen to buffer. */
	uint64_t getTotalBytesCount() override;
	/** Method for getting the current cursor position, where 0 is the first
	 * byte and TotalBytesCount-1 the last one */
	uint64_t getPosition() override;
	/** The current Input cursor position, where 0 is the first byte */
	uint64_t getPositionI();
	/** The current Input cursor position, where 0 is the first byte */
	uint64_t getPositionO();

	/** Reads one string line from the file (until a new-line character)
	  * \return true if a line has been read, false on EOF or error */
	bool readLine(std::string& str);

};  // End of class def.
static_assert(
	!std::is_copy_constructible<CFileStream>::value &&
		!std::is_copy_assignable<CFileStream>::value,
	"Copy Check");
}  // End of namespace
}  // end of namespace
