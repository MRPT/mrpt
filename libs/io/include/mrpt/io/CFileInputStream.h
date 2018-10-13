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
/** This CStream derived class allow using a file as a read-only, binary stream.
 *
 * \sa CStream, CFileStream, CFileGZInputStream
 * \ingroup mrpt_io_grp
 */
class CFileInputStream : public CStream
{
   private:
	/** The actual input file stream. */
	std::ifstream m_if;

   public:
	/** Constructor
	 * \param fileName The file to be open in this stream
	 * \exception std::exception On error trying to open the file.
	 */
	CFileInputStream(const std::string& fileName);
	/** Default constructor */
	CFileInputStream();

	CFileInputStream(const CFileInputStream&) = delete;
	CFileInputStream& operator=(const CFileInputStream&) = delete;

	~CFileInputStream() override;

	/** Open a file for reading
	 * \param fileName The file to be open in this stream
	 * \return true on success.
	 */
	bool open(const std::string& fileName);
	/** Close the stream */
	void close();
	/** Returns true if the file was open without errors. */
	bool fileOpenCorrectly() const;
	/** Returns true if the file was open without errors. */
	bool is_open() { return fileOpenCorrectly(); }
	/** Will be true if EOF has been already reached. */
	bool checkEOF();
	/** Resets stream error status bits (e.g. after an EOF) */
	void clearError();

	// See docs in base class
	uint64_t Seek(
		int64_t off, CStream::TSeekOrigin Origin = sFromBeginning) override;
	// See docs in base class
	uint64_t getTotalBytesCount() const override;
	// See docs in base class
	uint64_t getPosition() const override;

	/** Reads one string line from the file (until a new-line character)
	 * \return true if a line has been read, false on EOF or error. */
	bool readLine(std::string& str);

	size_t Read(void* Buffer, size_t Count) override;
	size_t Write(const void* Buffer, size_t Count) override;
};  // End of class def.
}  // namespace mrpt::io
