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
#ifndef  CFileGZOutputStream_H
#define  CFileGZOutputStream_H

#include <mrpt/utils/CStream.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace utils
	{
		/** Saves data to a file and transparently compress the data using the given compression level.
		 *   The generated files are in gzip format ("file.gz").
		 *  This class requires compiling MRPT with wxWidgets. If wxWidgets is not available then the class is actually mapped to the standard CFileOutputStream
		 *
		 * \sa CFileOutputStream
		 * \ingroup mrpt_base_grp
		 */
#if !MRPT_HAS_GZ_STREAMS
		// We don't have wxwidgets:
#	define CFileGZOutputStream	CFileOutputStream
#else
		class BASE_IMPEXP CFileGZOutputStream : public CStream, public CUncopiable
		{
		protected:
			 /** Method responsible for reading from the stream.
			 */
			size_t  Read(void *Buffer, size_t Count);

			/** Method responsible for writing to the stream.
			 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
			 */
			size_t  Write(const void *Buffer, size_t Count);

			// DECLARE_UNCOPIABLE( CFileGZOutputStream )

		private:
			void		*m_f;

		public:
			 /** Constructor: opens an output file with compression level = 1 (minimum, fastest).
			  * \param fileName The file to be open in this stream
			  * \sa open
			  */
			CFileGZOutputStream(const std::string &fileName);

			/** Constructor, without opening the file.
			  * \sa open
			  */
			CFileGZOutputStream();

			 /** Open a file for write, choosing the compression level
			  * \param fileName The file to be open in this stream
			  * \param compress_level 0:no compression, 1:fastest, 9:best
			  * \return true on success, false on any error.
			  */
			bool open(const std::string &fileName, int compress_level = 1 );

			/** Close the file. */
			void close();

			 /** Destructor
			 */
			 virtual ~CFileGZOutputStream();

			 /** Says if file was open successfully or not.
			  */
			 bool  fileOpenCorrectly();

			/** Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
			 */
			uint64_t getPosition();

			/** This method is not implemented in this class */
			uint64_t Seek(uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning)
			{
				THROW_EXCEPTION("Seek is not implemented in this class");
			}

			/** This method is not implemented in this class */
			uint64_t getTotalBytesCount()
			{
				THROW_EXCEPTION("getTotalBytesCount is not implemented in this class");
			}

		}; // End of class def.
#endif

	} // End of namespace
} // end of namespace
#endif
