/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CFileInputStream_H
#define  CFileInputStream_H

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
		class BASE_IMPEXP CFileInputStream : public CStream, public CUncopiable
		{
		protected:
			size_t  Read(void *Buffer, size_t Count) MRPT_OVERRIDE;
			size_t  Write(const void *Buffer, size_t Count) MRPT_OVERRIDE;
		private:
			std::ifstream 	m_if;		//!< The actual input file stream.
		public:
			 /** Constructor
			  * \param fileName The file to be open in this stream
			  * \exception std::exception On error trying to open the file.
			  */
			CFileInputStream(const std::string &fileName );
			 /** Default constructor */
			CFileInputStream();
			virtual ~CFileInputStream();

			 /** Open a file for reading
			  * \param fileName The file to be open in this stream
			  * \return true on success.
			  */
			bool open(const std::string &fileName );
			void close(); //!< Close the stream
			bool fileOpenCorrectly(); //!< Returns true if the file was open without errors.
			bool is_open() { return fileOpenCorrectly(); } //!< Returns true if the file was open without errors.
			bool checkEOF(); //!< Will be true if EOF has been already reached.

			/** Method for moving to a specified position in the streamed resource.
			 *   See documentation of CStream::Seek */
			uint64_t Seek( uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning) MRPT_OVERRIDE;

			/** Method for getting the total number of bytes in the buffer. */
			uint64_t getTotalBytesCount() MRPT_OVERRIDE;

			/** Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one. */
			uint64_t getPosition() MRPT_OVERRIDE;

			/** Reads one string line from the file (until a new-line character)
			  * \return true if a line has been read, false on EOF or error. */
			bool readLine( std::string &str );

		}; // End of class def.
	} // End of namespace
} // end of namespace
#endif
