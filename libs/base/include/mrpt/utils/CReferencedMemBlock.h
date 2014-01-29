/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
