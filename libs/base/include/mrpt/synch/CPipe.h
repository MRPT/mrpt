/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  mrpt_synch_pipe_H
#define  mrpt_synch_pipe_H

#include <mrpt/base/link_pragmas.h>
#include <mrpt/utils/CUncopiable.h>
#include <mrpt/utils/CStream.h>
#include <string>
#include <memory> // for auto_ptr<>, unique_ptr<>

namespace mrpt
{
	namespace synch
	{
		class CPipeReadEndPoint;
		class CPipeWriteEndPoint;

		/** A pipe, portable across different OS.
		  * Pipes can be used as intraprocess (inter-threads) or interprocess communication mechanism.
		  * Read more on pipes here: http://www.gnu.org/software/libc/manual/html_node/Pipes-and-FIFOs.html
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
		  * \ingroup synch_grp
		  */
		class BASE_IMPEXP CPipe
		{
		public:
			/** Creates a new pipe and returns the read & write end-points as newly allocated objects.
			  * \exception std::exception On any error during the pipe creation
			  */
			/** Creates a new pipe and returns the read & write end-points as newly allocated objects. */
			template <typename ReadPtr, typename WritePtr>
			static void createPipe(ReadPtr& outReadPipe,WritePtr& outWritePipe);

			static void initializePipe(CPipeReadEndPoint &outReadPipe, CPipeWriteEndPoint &outWritePipe);

		private:
			CPipe();  //!< No need to create any object of this class.
			~CPipe();
		}; // end of CPipe


		/** Common interface of read & write pipe end-points */
		class BASE_IMPEXP CPipeBaseEndPoint :
			public mrpt::utils::CUncopiable,
			public mrpt::utils::CStream
		{
			friend class CPipe;
		public:
			CPipeBaseEndPoint();
			virtual ~CPipeBaseEndPoint();

			/** De-serializes one end-point description, for example, from a parent process. */
			explicit CPipeBaseEndPoint(const std::string &serialized);

			/** Converts the end-point into a string suitable for reconstruction at a child process.
			  * This *invalidates* this object, since only one real end-point can exist at once.
			  */
			std::string serialize();

			unsigned int timeout_read_start_us;   //!< (Default=0) Timeout for read operations: microseconds (us) to wait for the first byte. 0 means infinite timeout.
			unsigned int timeout_read_between_us; //!< (Default=0) Timeout between burst reads operations: microseconds (us) to wait between two partial reads inside one large read. 0 means infinite timeout.

			/** Returns false if the pipe was closed due to some error. */
			inline bool isOpen() const { return m_pipe_file!=0; }

			/** Closes the pipe (normally not needed to be called by users, automatically done at destructor) */
			void close();
			
		protected:
#ifdef MRPT_OS_WINDOWS
			void * m_pipe_file;
#else
			int m_pipe_file;
#endif
			virtual size_t  Read(void *Buffer, size_t Count) MRPT_OVERRIDE;
			virtual size_t  Write(const void *Buffer, size_t Count) MRPT_OVERRIDE;

			virtual uint64_t Seek(uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning) MRPT_OVERRIDE; //!< Without effect in this class
			virtual uint64_t getTotalBytesCount() MRPT_OVERRIDE; //!< Without effect in this class
			virtual uint64_t getPosition() MRPT_OVERRIDE; //!< Without effect in this class
		}; // end of CPipeBaseEndPoint

		/** The read end-point in a pipe created with mrpt::synch::CPipe.
		  * Use the method mrpt::utils::CStream::ReadBuffer() of the base class CStream for blocking reading. */
		class BASE_IMPEXP CPipeReadEndPoint : public CPipeBaseEndPoint
		{
			friend class CPipe;
        public:
			/** De-serializes one end-point description, for example, from a parent process. */
			explicit CPipeReadEndPoint(const std::string &serialized);

		private:
			CPipeReadEndPoint();
			void  WriteBuffer (const void *Buffer, size_t Count);  //!< Hide the write method in this read-only pipe.

		}; // end of CPipeReadEndPoint

		/** The write end-point in a pipe created with mrpt::synch::CPipe.
		  * Use the method mrpt::utils::CStream::WriteBuffer() of the base class CStream for blocking writing. */
		class BASE_IMPEXP CPipeWriteEndPoint : public CPipeBaseEndPoint
		{
			friend class CPipe;
        public:
			/** De-serializes one end-point description, for example, from a parent process. */
			explicit CPipeWriteEndPoint(const std::string &serialized);

		private:
			CPipeWriteEndPoint();
			size_t ReadBuffer(void *Buffer, size_t Count);  //!< Hide the read method in this write-only pipe.

		}; // end of CPipeWriteEndPoint

		template <typename ReadPtr, typename WritePtr>
		void CPipe::createPipe(ReadPtr& outReadPipe,WritePtr& outWritePipe)
		{
			outReadPipe  = ReadPtr(new CPipeReadEndPoint);
			outWritePipe = WritePtr(new CPipeWriteEndPoint);
			CPipe::initializePipe(*outReadPipe, *outWritePipe);
		}



	} // End of namespace

} // End of namespace

#endif
