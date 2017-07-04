/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CFileInputStream_H
#define CFileInputStream_H

#include <mrpt/utils/CStream.h>
#include <fstream>

namespace mrpt
{
namespace utils
{
/** This CStream derived class allow using a file as a read-only, binary stream.
 *
 * \sa CStream, CFileStream, CFileGZInputStream
 * \ingroup mrpt_base_grp
 */
class BASE_IMPEXP CFileInputStream : public CStream
{
   protected:
	size_t Read(void* Buffer, size_t Count) override;
	size_t Write(const void* Buffer, size_t Count) override;

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

	virtual ~CFileInputStream();

	/** Open a file for reading
	 * \param fileName The file to be open in this stream
	 * \return true on success.
	 */
	bool open(const std::string& fileName);
	/** Close the stream */
	void close();
	/** Returns true if the file was open without errors. */
	bool fileOpenCorrectly();
	/** Returns true if the file was open without errors. */
	bool is_open() { return fileOpenCorrectly(); }
	/** Will be true if EOF has been already reached. */
	bool checkEOF();

	/** Method for moving to a specified position in the streamed resource.
	 *   See documentation of CStream::Seek */
	uint64_t Seek(
		uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning) override;

	/** Method for getting the total number of bytes in the buffer. */
	uint64_t getTotalBytesCount() override;

	/** Method for getting the current cursor position, where 0 is the first
	 * byte and TotalBytesCount-1 the last one. */
	uint64_t getPosition() override;

	/** Reads one string line from the file (until a new-line character)
	  * \return true if a line has been read, false on EOF or error. */
	bool readLine(std::string& str);

};  // End of class def.
static_assert(
	!std::is_copy_constructible<CFileInputStream>::value &&
		!std::is_copy_assignable<CFileInputStream>::value,
	"Copy Check");
}  // End of namespace
}  // end of namespace
#endif
