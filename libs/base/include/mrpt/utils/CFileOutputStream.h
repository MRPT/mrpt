/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CFileOutputStream_H
#define  CFileOutputStream_H

#include <mrpt/utils/CStream.h>

#include <fstream>

namespace mrpt
{
namespace utils
{
	/** This CStream derived class allow using a file as a write-only, binary stream.
	 *
	 * \sa CStream, CFileStream, CFileGZOutputStream
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CFileOutputStream : public CStream, public CUncopiable
	{
	protected:
		size_t Read(void *Buffer, size_t Count) MRPT_OVERRIDE;
		size_t Write(const void *Buffer, size_t Count) MRPT_OVERRIDE;
	private:
		std::ofstream 	m_of;		//!< The actual output file stream.

	public:
		 /** Constructor
		  * \param fileName The file to be open in this stream
		  * \param append If set to true, the file will be opened for writing and the current cursor position set at the end of the file. Otherwise, previous contents will be lost.
		  * \exception std::exception if the file cannot be opened.
		  */
		CFileOutputStream(
			const std::string &fileName,
			bool  append = false
			 );

		 /** Default constructor */
		CFileOutputStream();

		 /** Open the given file for write
		  * \param fileName The file to be open in this stream
		  * \param append If set to true, the file will be opened for writing and the current cursor position set at the end of the file. Otherwise, previous contents will be lost.
		  * \sa fileOpenCorrectly
		  * \return true on success.
		  */
		bool open(const std::string &fileName, bool  append = false );

		/** Close the stream. */
		void close();

		/** Destructor */
		virtual ~CFileOutputStream();

		bool fileOpenCorrectly(); //!< Returns true if the file was open without errors.
		bool is_open() { return fileOpenCorrectly(); } //!< Returns true if the file was open without errors.

		// See base class docs
		uint64_t Seek(uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning) MRPT_OVERRIDE;

		/** Method for getting the total number of bytes writen to buffer */
		uint64_t getTotalBytesCount() MRPT_OVERRIDE;

		/** Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one */
		uint64_t getPosition() MRPT_OVERRIDE;
	}; // End of class def.
	} // End of namespace
} // end of namespace
#endif
