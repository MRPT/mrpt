/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/io/CStream.h>
#include <mrpt/core/safe_pointers.h>

namespace mrpt
{
namespace io
{
/** This CStream derived class allow using a memory buffer as a CStream.
 *  This class is useful for storing any required set of variables or objects,
 *   and then read them to other objects, or storing them to a file, for
 * example.
 *
 * \sa CStream
 * \ingroup mrpt_io_grp
 */
class CMemoryStream : public CStream
{
   public:
	size_t Read(void* Buffer, size_t Count) override;
	size_t Write(const void* Buffer, size_t Count) override;

   protected:
	/** Internal data */
	void_ptr_noncopy m_memory{nullptr};
	uint64_t m_size{0}, m_position{0}, m_bytesWritten{0};
	uint64_t m_alloc_block_size{0x1000};
	/** If the memory block does not belong to the object. */
	bool m_read_only{false};
	/** Resizes the internal buffer size. */
	void resize(uint64_t newSize);

   public:
	/** Default constructor */
	CMemoryStream() = default;
	/** Constructor to initilize the data in the stream from a block of memory
	 * (which is copied), and sets the current stream position at the beginning
	 * of the data.
	 * \sa assignMemoryNotOwn */
	CMemoryStream(const void* data, const uint64_t nBytesInData);

	/** Initilize the data in the stream from a block of memory which is NEITHER
	 * OWNED NOR COPIED by the object, so it must exist during the whole live of
	 * the object.
	 *  After assigning a block of data with this method, the object becomes
	 * "read-only", so further attempts to change the size of the buffer will
	 * raise an exception.
	 *  This method resets the write and read positions to the beginning. */
	void assignMemoryNotOwn(const void* data, const uint64_t nBytesInData);

	/** Destructor */
	~CMemoryStream() override;

	/** Clears the memory buffer. */
	void Clear();

	/** Change size. This would be rarely used. Use ">>" operators for writing
	 * to stream \sa Stream */
	void changeSize(uint64_t newSize);

	// See docs in base class
	uint64_t Seek(
		int64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning) override;
	/** Returns the total size of the internal buffer  */
	uint64_t getTotalBytesCount() const override;
	/** Method for getting the current cursor position, where 0 is the first
	 * byte and TotalBytesCount-1 the last one */
	uint64_t getPosition() const override;

	/** Method for getting a pointer to the raw stored data. The lenght in bytes
	 * is given by getTotalBytesCount */
	void* getRawBufferData();
	const void* getRawBufferData() const;

	/** Saves the entire buffer to a file \return true on success, false on
	 * error */
	bool saveBufferToFile(const std::string& file_name);

	/** Loads the entire buffer from a file * \return true on success, false on
	 * error */
	bool loadBufferFromFile(const std::string& file_name);

	/** Change the size of the additional memory block that is reserved whenever
	 * the current block runs too short (default=0x10000 bytes) */
	void setAllocBlockSize(uint64_t alloc_block_size)
	{
		ASSERT_(alloc_block_size > 0);
		m_alloc_block_size = alloc_block_size;
	}
};  // End of class def.

namespace internal
{
struct TFreeFnDataForZMQ
{
	CMemoryStream* buf{nullptr};
	bool do_free{true};
	TFreeFnDataForZMQ() = default;
};
/** Used in mrpt_send_to_zmq(). `hint` points to a `TFreeFnDataForZMQ` struct,
 * to be freed here. */
void free_fn_for_zmq(void* data, void* hint);
}  // namespace internal
}  // namespace io
}  // namespace mrpt
