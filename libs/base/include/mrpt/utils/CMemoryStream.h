/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CMEMORYSTREAM_H
#define  CMEMORYSTREAM_H

#include <mrpt/utils/CStream.h>
#include <mrpt/utils/safe_pointers.h>

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
		size_t Read(void *Buffer, size_t Count) MRPT_OVERRIDE;
		size_t Write(const void *Buffer, size_t Count) MRPT_OVERRIDE;

		/** Internal data */
		void_ptr_noncopy m_memory;
		uint64_t         m_size, m_position, m_bytesWritten;
		uint64_t         m_alloc_block_size;
		bool             m_read_only;   //!< If the memory block does not belong to the object.
		void resize(uint64_t newSize); //!< Resizes the internal buffer size.
	public:
		CMemoryStream(); //!< Default constructor

		/** Constructor to initilize the data in the stream from a block of memory (which is copied), and sets the current stream position at the beginning of the data.
		 * \sa assignMemoryNotOwn */
		CMemoryStream( const void *data, const uint64_t nBytesInData );

		/** Initilize the data in the stream from a block of memory which is NEITHER OWNED NOR COPIED by the object, so it must exist during the whole live of the object.
		  *  After assigning a block of data with this method, the object becomes "read-only", so further attempts to change the size of the buffer will raise an exception.
		  *  This method resets the write and read positions to the beginning. */
		void assignMemoryNotOwn( const void *data, const uint64_t nBytesInData );

		virtual ~CMemoryStream(); //!< Destructor

		void Clear(); //!< Clears the memory buffer.

		void changeSize( uint64_t newSize ); //!< Change size. This would be rarely used. Use ">>" operators for writing to stream \sa Stream

		// See docs in base class
		uint64_t Seek(uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning) MRPT_OVERRIDE;
		/** Returns the total size of the internal buffer  */
		uint64_t getTotalBytesCount() MRPT_OVERRIDE;
		/** Method for getting the current cursor position, where 0 is the first byte and TotalBytesCount-1 the last one */
		uint64_t getPosition() MRPT_OVERRIDE;

		/** Method for getting a pointer to the raw stored data. The lenght in bytes is given by getTotalBytesCount */
		void* getRawBufferData();

		/** Saves the entire buffer to a file \return true on success, false on error */
		bool saveBufferToFile( const std::string &file_name );

		/** Loads the entire buffer from a file * \return true on success, false on error */
		bool loadBufferFromFile( const std::string &file_name );

		/** Change the size of the additional memory block that is reserved whenever the current block runs too short (default=0x10000 bytes) */
		void setAllocBlockSize( uint64_t  alloc_block_size )
		{
			ASSERT_(alloc_block_size>0)
			m_alloc_block_size = alloc_block_size;
		}
	}; // End of class def.

	namespace internal {
		struct BASE_IMPEXP TFreeFnDataForZMQ
		{
			CMemoryStream *buf;
			bool do_free;
			TFreeFnDataForZMQ() : buf(NULL), do_free(true) { }
		};
		void BASE_IMPEXP free_fn_for_zmq(void *data, void *hint); //!< Used in mrpt_send_to_zmq(). `hint` points to a `TFreeFnDataForZMQ` struct, to be freed here.
	}
	} // End of namespace
} // end of namespace
#endif
