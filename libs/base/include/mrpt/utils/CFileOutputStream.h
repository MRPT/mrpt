/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#ifndef  CFileOutputStream_H
#define  CFileOutputStream_H

#include <mrpt/utils/CStream.h>

#include <iostream>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
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
		 /** Method responsible for reading from the stream.
		 */
		size_t  Read(void *Buffer, size_t Count);

		/** Method responsible for writing to the stream.
		 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
		 */
		size_t  Write(const void *Buffer, size_t Count);

		// DECLARE_UNCOPIABLE( CFileOutputStream )

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

		 /** Default constructor
		  */
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

		 /** Destructor
		 */
		 virtual ~CFileOutputStream();

		 /** Says if file was open successfully or not.
		  */
		 bool  fileOpenCorrectly();

		/** Method for moving to a specified position in the streamed resource.
		 *   See documentation of CStream::Seek
		 */
		uint64_t Seek(uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning);

		/** Method for getting the total number of bytes writen to buffer.
		 */
		uint64_t getTotalBytesCount();

		/** Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
		 */
		uint64_t getPosition();


	}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
