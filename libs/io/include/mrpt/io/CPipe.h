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
#include <string>
#include <memory>  // for unique_ptr<>

namespace mrpt::io
{
class CPipeReadEndPoint;
class CPipeWriteEndPoint;

/** A pipe, portable across different OS.
 * Pipes can be used as intraprocess (inter-threads) or interprocess
 * communication mechanism.
 * Read more on pipes here:
 * http://www.gnu.org/software/libc/manual/html_node/Pipes-and-FIFOs.html
 *
 *  \code
 *    std::unique_ptr<CPipeReadEndPoint>  read_pipe;
 *    std::unique_ptr<CPipeWriteEndPoint> write_pipe;
 *
 *    CPipe::createPipe(read_pipe,write_pipe);
 *
 *  \endcode
 *
 * See also the example: MRPT/samples/threadsPipe/
 *
 * \ingroup mrpt_io_grp
 */
class CPipe
{
   public:
	/** Create via createPipe() instead */
	CPipe() = delete;
	~CPipe() = delete;

	/** Creates a new pipe and returns the read & write end-points as newly
	 * allocated objects.
	 * \exception std::exception On any error during the pipe creation
	 */
	/** Creates a new pipe and returns the read & write end-points as newly
	 * allocated objects. */
	template <typename ReadPtr, typename WritePtr>
	static void createPipe(ReadPtr& outReadPipe, WritePtr& outWritePipe);

	static void initializePipe(
		CPipeReadEndPoint& outReadPipe, CPipeWriteEndPoint& outWritePipe);
};  // end of CPipe

/** Common interface of read & write pipe end-points
 * \ingroup mrpt_io_grp
 */
class CPipeBaseEndPoint : public mrpt::io::CStream
{
	friend class CPipe;

   public:
	CPipeBaseEndPoint();

	CPipeBaseEndPoint(const CPipeBaseEndPoint&) = delete;
	CPipeBaseEndPoint& operator=(const CPipeBaseEndPoint&) = delete;

	~CPipeBaseEndPoint() override;

	/** De-serializes one end-point description, for example, from a parent
	 * process. */
	explicit CPipeBaseEndPoint(const std::string& serialized);

	/** Converts the end-point into a string suitable for reconstruction at a
	 * child process.
	 * This *invalidates* this object, since only one real end-point can exist
	 * at once.
	 */
	std::string serialize();

	/** (Default=0) Timeout for read operations: microseconds (us) to wait for
	 * the first byte. 0 means infinite timeout. */
	unsigned int timeout_read_start_us{0};
	/** (Default=0) Timeout between burst reads operations: microseconds (us) to
	 * wait between two partial reads inside one large read. 0 means infinite
	 * timeout. */
	unsigned int timeout_read_between_us{0};

	/** Returns false if the pipe was closed due to some error. */
	inline bool isOpen() const { return m_pipe_file != 0; }
	/** Closes the pipe (normally not needed to be called by users,
	 * automatically done at destructor) */
	void close();

   protected:
#ifdef _WIN32
	void* m_pipe_file;
#else
	int m_pipe_file{0};
#endif
   public:
	size_t Read(void* Buffer, size_t Count) override;
	size_t Write(const void* Buffer, size_t Count) override;

	/** Without effect in this class */
	uint64_t Seek(int64_t of, CStream::TSeekOrigin o = sFromBeginning) override;
	/** Without effect in this class */
	uint64_t getTotalBytesCount() const override;
	/** Without effect in this class */
	uint64_t getPosition() const override;
};  // end of CPipeBaseEndPoint
static_assert(
	!std::is_copy_constructible_v<CPipeBaseEndPoint> &&
		!std::is_copy_assignable_v<CPipeBaseEndPoint>,
	"Copy Check");

/** The read end-point in a pipe created with mrpt::synch::CPipe.
 * Use the method CStream::Read() of the base class CStream
 * for blocking reading.
 * \ingroup mrpt_io_grp
 */
class CPipeReadEndPoint : public CPipeBaseEndPoint
{
	friend class CPipe;

   public:
	/** De-serializes one end-point description, for example, from a parent
	 * process. */
	explicit CPipeReadEndPoint(const std::string& serialized);

	/** Read-only pipe, don't call this method */
	size_t Write(const void* Buffer, size_t Count) override
	{
		throw std::runtime_error("CPipeReadEndPoint::Write() cant be called.");
	}

   private:
	CPipeReadEndPoint();

};  // end of CPipeReadEndPoint

/** The write end-point in a pipe created with mrpt::synch::CPipe.
 * Use the method CStream::Write() of the base class CStream
 * for blocking writing. */
class CPipeWriteEndPoint : public CPipeBaseEndPoint
{
	friend class CPipe;

   public:
	/** De-serializes one end-point description, for example, from a parent
	 * process. */
	explicit CPipeWriteEndPoint(const std::string& serialized);

	/** Write-only pipe: read launches exception */
	size_t Read(void* Buffer, size_t Count) override
	{
		throw std::runtime_error("CPipeWriteEndPoint::Read() cant be called.");
	}

   private:
	CPipeWriteEndPoint();

};  // end of CPipeWriteEndPoint

template <typename ReadPtr, typename WritePtr>
void CPipe::createPipe(ReadPtr& outReadPipe, WritePtr& outWritePipe)
{
	outReadPipe = ReadPtr(new CPipeReadEndPoint);
	outWritePipe = WritePtr(new CPipeWriteEndPoint);
	CPipe::initializePipe(*outReadPipe, *outWritePipe);
}
}  // namespace mrpt::io
