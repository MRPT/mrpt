/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef  CFILESTREAM_H
#define  CFILESTREAM_H

#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CUncopiable.h>

#include <iostream>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
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
			 /** Method responsible for reading from the stream.
			 */
			size_t  Read(void *Buffer, size_t Count);

			/** Method responsible for writing to the stream.
			 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
			 */
			size_t  Write(const void *Buffer, size_t Count);

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

			/** Constructor
			  */
			CFileStream();


			/** Opens the file, returning true on success.
			  * \param fileName The file to be open in this stream
			  * \param mode The open mode: can be an or'd conbination of different values.
			  *  By default the file is opened for open and write and created if not found.
			  */
			bool open(const std::string &fileName, TFileOpenModes mode = fomRead | fomWrite );

			/** Closes the file
			  */
			void close();

			 /** Destructor
			 */
			 virtual ~CFileStream();

			 /** Says if file was open successfully or not.
			  */
			 bool  fileOpenCorrectly();

			 /** Will be true if EOF has been already reached.
			   */
			 bool checkEOF();

			/** Method for moving to a specified position in the streamed resource.
			 *   See documentation of CStream::Seek
			 */
			uint64_t Seek(long Offset, CStream::TSeekOrigin Origin = sFromBeginning);

			/** Method for getting the total number of bytes writen to buffer.
			 */
			uint64_t getTotalBytesCount();

			/** Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
			 */
			uint64_t getPosition();

			/** The current Input cursor position, where 0 is the first byte.
			 */
			uint64_t getPositionI();

			/** The current Input cursor position, where 0 is the first byte.
			 */
			uint64_t getPositionO();

			/** Reads one string line from the file (until a new-line character)
			  * \return true if a line has been read, false on EOF or error.
			  */
			bool readLine( std::string &str );


		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
