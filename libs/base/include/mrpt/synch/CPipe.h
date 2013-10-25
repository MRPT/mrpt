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
#ifndef  mrpt_synch_pipe_H
#define  mrpt_synch_pipe_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CUncopiable.h>
#include <mrpt/utils/CStream.h>

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
		  *    std::auto_ptr<CPipeReadEndPoint>  read_pipe; 
		  *    std::auto_ptr<CPipeWriteEndPoint> write_pipe;
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
			static void createPipe(std::auto_ptr<CPipeReadEndPoint>& outReadPipe,std::auto_ptr<CPipeWriteEndPoint>& outWritePipe);

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

		protected:
#ifdef MRPT_OS_WINDOWS
			void * m_pipe_file;
#else
			int m_pipe_file;
#endif
			virtual size_t  Read(void *Buffer, size_t Count);
			virtual size_t  Write(const void *Buffer, size_t Count);
			
			virtual uint64_t Seek(uint64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning); //!< Without effect in this class
			virtual uint64_t getTotalBytesCount(); //!< Without effect in this class
			virtual uint64_t getPosition(); //!< Without effect in this class

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


	} // End of namespace

} // End of namespace

#endif
