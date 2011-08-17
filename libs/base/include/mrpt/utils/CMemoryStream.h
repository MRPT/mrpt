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
#ifndef  CMEMORYSTREAM_H
#define  CMEMORYSTREAM_H

#include <mrpt/utils/CStream.h>
#include <mrpt/utils/safe_pointers.h>

/*---------------------------------------------------------------
	Class
  ---------------------------------------------------------------*/
namespace mrpt
{
namespace utils
{
	/** This CStream derived class allow using a memory buffer as a CStream.
	 *  This class is useful for storing any required set of variables or objects,
	 *   and then read them to other objects, or storing them to a file, for example.
	 *
	 * \sa CStream
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CMemoryStream : public CStream
	{
	protected:
		 /** Method responsible for reading from the stream.
		 */
		size_t Read(void *Buffer, size_t Count);

		/** Method responsible for writing to the stream.
		 *  Write attempts to write up to Count bytes to Buffer, and returns the number of bytes actually written.
		 */
		size_t Write(const void *Buffer, size_t Count);

		/** Internal data
		 */
		void_ptr_noncopy	m_memory;
		uint64_t	m_size, m_position, m_bytesWritten;
		uint64_t	m_alloc_block_size;
		bool	m_read_only;   //!< If the memory block does not belong to the object.

		/** Resizes the internal buffer size.
		 */
		void resize(uint64_t newSize);
	public:
		/** Default constructor
		 */
		CMemoryStream();

		/** Constructor to initilize the data in the stream from a block of memory (which is copied), and sets the current stream position at the beginning of the data.
		 * \sa assignMemoryNotOwn
		 */
		CMemoryStream( const void *data, const uint64_t nBytesInData );

		/** Initilize the data in the stream from a block of memory which is NEITHER OWNED NOR COPIED by the object, so it must exist during the whole live of the object.
		  *  After assigning a block of data with this method, the object becomes "read-only", so further attempts to change the size of the buffer will raise an exception.
		  *  This method resets the write and read positions to the beginning.
		  */
		void assignMemoryNotOwn( const void *data, const uint64_t nBytesInData );

		/** Destructor
		 */
		virtual ~CMemoryStream();

		/** Clears the memory buffer.
		 */
		void  Clear();

		/** Change size. This would be rarely used. Use ">>" operators for writing to stream.
		 * \sa Stream
		 */
		void  changeSize( uint64_t newSize );

		/** Method for moving to a specified position in the streamed resource.
		 *  \sa CStream::Seek
		 */
		uint64_t Seek(long Offset, CStream::TSeekOrigin Origin = sFromBeginning);

		/** Returns the total size of the internal buffer.
		 */
		uint64_t getTotalBytesCount();

		/** Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one.
		 */
		uint64_t getPosition();

		/** Method for getting a pointer to the raw stored data.
		 * The lenght in bytes is given by getTotalBytesCount
		 */
		void*  getRawBufferData();


		/** Saves the entire buffer to a file
		  * \return true on success, false on error
		  */
		bool saveBufferToFile( const std::string &file_name );

		/** Loads the entire buffer from a file
		  * \return true on success, false on error
		  */
		bool loadBufferFromFile( const std::string &file_name );

		/** Change the size of the additional memory block that is reserved whenever the current block runs too short (default=0x10000 bytes) */
		void setAllocBlockSize( uint64_t  alloc_block_size )
		{
			ASSERT_(alloc_block_size>0)
			m_alloc_block_size = alloc_block_size;
		}

	}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
