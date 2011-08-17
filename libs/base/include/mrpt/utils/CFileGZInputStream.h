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
