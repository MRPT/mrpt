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
#ifndef  CFileGZInputStream_H
#define  CFileGZInputStream_H

#include <mrpt/utils/CStream.h>

#include <iostream>


/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace utils
	{
		/** Transparently opens a compressed "gz" file and reads uncompressed data from it.
		 *   If the file is not a .gz file, it silently reads data from the file.
		 *  This class requires compiling MRPT with wxWidgets. If wxWidgets is not available then the class is actually mapped to the standard CFileInputStream
		 *
		 * \sa CFileInputStream
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CFileGZInputStream : public CStream, public CUncopiable
		{
		protected:
			 /** Method responsible for reading from the stream.
			 */
			size_t  Read(void *Buffer, size_t Count);

			/** Method responsible for writing to the stream.
			 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
			 */
			size_t  Write(const void *Buffer, size_t Count);

			// DECLARE_UNCOPIABLE( CFileGZInputStream )

		private:
			void		*m_f;
			uint64_t	m_file_size;	//!< Compressed file size

		public:
			 /** Constructor without open
			  */
			CFileGZInputStream();

			 /** Constructor and open
			  * \param fileName The file to be open in this stream
			  * \exception std::exception If there's an error opening the file.
			  */
			CFileGZInputStream(const std::string &fileName );

			 /** Destructor
			 */
			 virtual ~CFileGZInputStream();

			 /** Opens the file for read.
			  * \param fileName The file to be open in this stream
			  * \return false if there's an error opening the file, true otherwise
			  */
			 bool open(const std::string &fileName );

			 /** Closes the file */
			 void close();

			 /** Says if file was open successfully or not.
			  */
			 bool  fileOpenCorrectly();

			 /** Will be true if EOF has been already reached.
			   */
			 bool checkEOF();

			/** Method for getting the total number of <b>compressed</b> bytes of in the file (the physical size of the compressed file).
			 */
			uint64_t getTotalBytesCount();

			/** Method for getting the current cursor position in the <b>compressed</b>, where 0 is the first byte and TotalBytesCount-1 the last one.
			 */
			uint64_t getPosition();

			/** This method is not implemented in this class */
			uint64_t Seek(long Offset, CStream::TSeekOrigin Origin = sFromBeginning)
			{
				THROW_EXCEPTION("Seek is not implemented in this class");
			}

		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
