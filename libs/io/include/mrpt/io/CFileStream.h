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
/** File open modes are used in CFileStream
  *  Posible values are:
	- fomRead
	- fomWrite (creates the file if it didn't exist, otherwise truncates it).
	- fomAppend (creates the file if it didn't exist)
  */
using TFileOpenModes = int;
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
 * \ingroup mrpt_io_grp
 */
class CFileStream : public CStream
{
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
	~CFileStream() override;

	size_t Read(void* Buffer, size_t Count) override;
	size_t Write(const void* Buffer, size_t Count) override;

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
	bool fileOpenCorrectly() const;
	/** Returns true if the file was open without errors. */
	bool is_open() { return fileOpenCorrectly(); }
	/** Will be true if EOF has been already reached. */
	bool checkEOF();
	/** Resets stream error status bits (e.g. after an EOF) */
	void clearError();

	uint64_t Seek(
		int64_t off, CStream::TSeekOrigin org = sFromBeginning) override;
	uint64_t getTotalBytesCount() const override;
	// See docs in base class
	uint64_t getPosition() const override;
	/** The current Input cursor position, where 0 is the first byte */
	uint64_t getPositionI();
	/** The current Input cursor position, where 0 is the first byte */
	uint64_t getPositionO();

	/** Reads one string line from the file (until a new-line character)
	 * \return true if a line has been read, false on EOF or error */
	bool readLine(std::string& str);

};  // End of class def.
}  // namespace mrpt::io
