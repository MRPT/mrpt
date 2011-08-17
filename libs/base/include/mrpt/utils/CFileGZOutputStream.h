/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
			uint64_t Seek(long Offset, CStream::TSeekOrigin Origin = sFromBeginning)
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
