/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CFileGZOutputStream_H
#define CFileGZOutputStream_H

#include <mrpt/utils/CStream.h>

namespace mrpt
{
namespace utils
{
/** Saves data to a file and transparently compress the data using the given
 * compression level.
 *   The generated files are in gzip format ("file.gz").
 *  This class requires compiling MRPT with wxWidgets. If wxWidgets is not
 * available then the class is actually mapped to the standard CFileOutputStream
 *
 * \sa CFileOutputStream
 * \ingroup mrpt_base_grp
 */
#if !MRPT_HAS_GZ_STREAMS
// We don't have wxwidgets:
#define CFileGZOutputStream CFileOutputStream
#else
class BASE_IMPEXP CFileGZOutputStream : public CStream
{
   protected:
	size_t Read(void* Buffer, size_t Count) override;
	size_t Write(const void* Buffer, size_t Count) override;
	// DECLARE_UNCOPIABLE( CFileGZOutputStream )
   private:
	void* m_f;

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
	virtual ~CFileGZOutputStream();

	/** Open a file for write, choosing the compression level
	 * \param fileName The file to be open in this stream
	 * \param compress_level 0:no compression, 1:fastest, 9:best
	 * \return true on success, false on any error.
	 */
	bool open(const std::string& fileName, int compress_level = 1);
	/** Close the file */
	void close();
	/** Returns true if the file was open without errors. */
	bool fileOpenCorrectly();
	/** Returns true if the file was open without errors. */
	bool is_open() { return fileOpenCorrectly(); }
	/** Method for getting the current cursor position, where 0 is the first
	 * byte and TotalBytesCount-1 the last one. */
	uint64_t getPosition() override;

	/** This method is not implemented in this class */
	uint64_t Seek(
		uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning) override
	{
		MRPT_UNUSED_PARAM(Offset);
		MRPT_UNUSED_PARAM(Origin);
		THROW_EXCEPTION("Seek is not implemented in this class");
	}

	/** This method is not implemented in this class */
	uint64_t getTotalBytesCount() override
	{
		THROW_EXCEPTION("getTotalBytesCount is not implemented in this class");
	}
};  // End of class def.
static_assert(
	!std::is_copy_constructible<CFileGZOutputStream>::value &&
		!std::is_copy_assignable<CFileGZOutputStream>::value,
	"Copy Check");
#endif

}  // End of namespace
}  // end of namespace
#endif
