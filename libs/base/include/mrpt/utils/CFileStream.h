/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CFILESTREAM_H
#define  CFILESTREAM_H

#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CUncopiable.h>

#include <fstream>

namespace mrpt
{
	namespace utils
	{
		/** File open modes are used in CFileStream
		  *  Posible values are:
			- fomRead
			- fomWrite (creates the file if it didn't exist, otherwise truncates it).
			- fomAppend (creates the file if it didn't exist)
		  */
		typedef int TFileOpenModes;
		enum {
			fomRead   = 1,
			fomWrite  = 2,
			fomAppend = 4
		};

		/** This CStream derived class allow using a file as a read/write binary stream, creating it if the file didn't exist.
		 *   The default behavior can be change to open as read, write, read and write,... in the constructor.
		 * \sa CStream, CFileInputStream, CFileOutputStrea, CFileGZInputStream
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP  CFileStream : public CStream, public CUncopiable
		{
		protected:
			size_t  Read(void *Buffer, size_t Count) MRPT_OVERRIDE;
			size_t  Write(const void *Buffer, size_t Count) MRPT_OVERRIDE;
		private:
			std::fstream 	m_f;		//!< The actual input file stream.
		public:
			 /** Constructor and open a file
			  * \param fileName The file to be open in this stream
			  * \param mode The open mode: can be an or'd conbination of different values.
			  * \exception std::exception On error creating or accessing the file.
			  *  By default the file is opened for open and write and created if not found.
			  */
			CFileStream( const std::string &fileName, TFileOpenModes mode = fomRead | fomWrite);
			/** Constructor */
			CFileStream();
			virtual ~CFileStream(); //!< Destructor

			/** Opens the file, returning true on success.
			  * \param fileName The file to be open in this stream
			  * \param mode The open mode: can be an or'd conbination of different values.
			  *  By default the file is opened for open and write and created if not found.
			  */
			bool open(const std::string &fileName, TFileOpenModes mode = fomRead | fomWrite );
			void close(); //!< Closes the file
			bool fileOpenCorrectly(); //!< Returns true if the file was open without errors.
			bool is_open() { return fileOpenCorrectly(); } //!< Returns true if the file was open without errors.
			bool checkEOF(); //!< Will be true if EOF has been already reached.

			/** Method for moving to a specified position in the streamed resource.
			 *   See documentation of CStream::Seek
			 */
			uint64_t Seek(uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning) MRPT_OVERRIDE;

			uint64_t getTotalBytesCount() MRPT_OVERRIDE; //!< Method for getting the total number of bytes writen to buffer.
			uint64_t getPosition() MRPT_OVERRIDE; //!< Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one
			uint64_t getPositionI(); //!< The current Input cursor position, where 0 is the first byte
			uint64_t getPositionO(); //!< The current Input cursor position, where 0 is the first byte

			/** Reads one string line from the file (until a new-line character)
			  * \return true if a line has been read, false on EOF or error */
			bool readLine( std::string &str );

		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
