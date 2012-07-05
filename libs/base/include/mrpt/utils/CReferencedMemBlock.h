/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
#ifndef  CReferencedMemBlock_H
#define  CReferencedMemBlock_H

#include <mrpt/utils/utils_defs.h>
#include <utility>

namespace mrpt
{
	namespace utils
	{
		/** Represents a memory block (via "void*") that can be shared between several objects through copy operator (=).
		  *  It keeps the reference count and only when it comes to zero, the memory block is really freed.
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CReferencedMemBlock : public stlplus::smart_ptr< vector_byte >
		{
		public:
			/** Constructor with an optional size of the memory block */
			CReferencedMemBlock(size_t mem_block_size = 0 );

			/** Destructor, calls dereference_once. */
			virtual ~CReferencedMemBlock();

			/** Resize the shared memory block. */
			void resize(size_t mem_block_size );

			template <class T> T getAs()
			{
				if (!stlplus::smart_ptr< vector_byte >::present())
					THROW_EXCEPTION("Trying to access to an uninitialized memory block");

				if( stlplus::smart_ptr< vector_byte >::operator ->()->empty() )
					THROW_EXCEPTION("Trying to access to a memory block of size 0");

				return reinterpret_cast<T>( & stlplus::smart_ptr< vector_byte >::operator ->()->operator [](0) );
			}

			template <class T> T getAs() const
			{
				if (!stlplus::smart_ptr< vector_byte >::present())
					THROW_EXCEPTION("Trying to access to an uninitialized memory block");

				if( stlplus::smart_ptr< vector_byte >::operator ->()->empty() )
					THROW_EXCEPTION("Trying to access to a memory block of size 0");

				return reinterpret_cast<const T>( & stlplus::smart_ptr< vector_byte >::operator ->()->operator [](0) );
			}

		}; // End of class

	} // End of namespace
} // End of namespace

#endif
